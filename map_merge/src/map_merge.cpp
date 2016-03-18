/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <map_merge/map_merge.h>

#include <thread>

#include <ros/console.h>
#include <occupancy_grid_utils/combine_grids.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <tf/transform_datatypes.h>
#include <combine_grids/estimate_transform.h>

namespace map_merge
{
geometry_msgs::Pose& operator+=(geometry_msgs::Pose&,
                                const geometry_msgs::Pose&);

MapMerging::MapMerging()
{
  ros::NodeHandle private_nh("~");
  std::string frame_id;
  std::string merged_map_topic;

  private_nh.param("merging_rate", merging_rate_, 4.0);
  private_nh.param("discovery_rate", discovery_rate_, 0.05);
  private_nh.param("estimation_rate", estimation_rate_, 0.5);
  private_nh.param<std::string>("robot_map_topic", robot_map_topic_, "map");
  private_nh.param<std::string>("robot_map_updates_topic",
                                robot_map_updates_topic_, "map_updates");
  private_nh.param<std::string>("robot_namespace", robot_namespace_, "");
  private_nh.param<std::string>("merged_map_topic", merged_map_topic, "map");
  private_nh.param<std::string>("world_frame", frame_id, "world");

  merged_map_.header.frame_id = frame_id;

  /* publishing */
  merged_map_publisher_ =
      node_.advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 50, true);
}

/*
 * Subcribe to pose and map topics
 */
void MapMerging::topicSubscribing()
{
  ROS_DEBUG("Robot discovery started.");

  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);

  for (const auto& topic : topic_infos) {
    geometry_msgs::Pose init_pose;
    std::string robot_name;
    std::string map_topic;
    std::string map_updates_topic;

    // we check only map topic
    if (!isRobotMapTopic(topic)) {
      continue;
    }

    robot_name = robotNameFromTopic(topic.name);
    if (robots_.count(robot_name)) {
      // we already know this robot
      continue;
    }

    if (!getInitPose(robot_name, init_pose)) {
      ROS_WARN("couldn't get initial position for robot [%s]\n"
               "did you defined parameters map_merge/init_pose_[xyz]? in robot "
               "namespace?",
               robot_name.c_str());
      continue;
    }

    ROS_INFO("adding robot [%s] to system", robot_name.c_str());
    maps_.emplace_front();
    PosedMap& map = maps_.front();
    robots_.insert({robot_name, &map});
    map.initial_pose = init_pose;
    {
      // we need only shared lock here. Map updating callbacks are not using
      // grid_view_, but maps directly. Of course this must not run concurently
      // with map merging as all iterators may be invalidated for grid_view_
      boost::shared_lock<boost::shared_mutex> lock(merging_mutex_);
      grid_view_.emplace_back(map.map);
    }

    map_topic = ros::names::append(robot_name, robot_map_topic_);
    map_updates_topic =
        ros::names::append(robot_name, robot_map_updates_topic_);
    ROS_INFO("Subscribing to MAP topic: %s.", map_topic.c_str());
    map.map_sub = node_.subscribe<nav_msgs::OccupancyGrid>(
        map_topic, 50,
        [this, &map](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
          fullMapUpdate(msg, map);
        });
    ROS_INFO("Subscribing to MAP updates topic: %s.",
             map_updates_topic.c_str());
    map.map_updates_sub = node_.subscribe<map_msgs::OccupancyGridUpdate>(
        map_updates_topic, 50,
        [this, &map](const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
          partialMapUpdate(msg, map);
        });
  }
}

/*
 * mapMerging()
 */
void MapMerging::mapMerging()
{
  ROS_DEBUG("Map merging started.");

  {
    // exclusive locking. we don't want map sizes to change during muxing.
    std::lock_guard<boost::shared_mutex> lock(merging_mutex_);
    occupancy_grid_utils::combineGrids(grid_view_.cbegin(), grid_view_.cend(),
                                       merged_map_);
  }

  ROS_DEBUG("all maps merged, publishing");
  ros::Time now = ros::Time::now();
  merged_map_.info.map_load_time = now;
  merged_map_.header.stamp = now;
  merged_map_publisher_.publish(merged_map_);
}

void MapMerging::poseEstimation()
{
  ROS_DEBUG("Grid pose estimation started.");
}

void MapMerging::fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
                               PosedMap& map)
{
  ROS_DEBUG("received full map update");
  // we don't want to access the same map twice
  std::lock_guard<std::mutex> lock(map.mutex);
  // it's ok if we do multiple updates of *different* maps concurently.
  // However we can't do any merging while updating any map metadata (size!)
  boost::shared_lock<boost::shared_mutex> merging_lock(merging_mutex_);

  /* We dont need to lock both locks at once. Deadlock will not occur here.
   * Circular waiting will never occur. We don't lock individual mutexes for
   * maps while merging. So merging and map update share only second lock. Lock
   * on map itself is here to avoid multiple accesses to the same map, which can
   * occur when this callback runs concurently. Also map could be accessed by
   * other callbacks (partials updates). */

  map.map = *msg;

  ROS_DEBUG("Adjusting origin");
  // compute global position
  ROS_DEBUG("origin %f %f, (%f, %f, %f, %f)", map.map.info.origin.position.x,
            map.map.info.origin.position.y, map.map.info.origin.orientation.x,
            map.map.info.origin.orientation.y,
            map.map.info.origin.orientation.z,
            map.map.info.origin.orientation.w);
  map.map.info.origin += map.initial_pose;
  ROS_DEBUG("origin %f %f, (%f, %f, %f, %f)", map.map.info.origin.position.x,
            map.map.info.origin.position.y, map.map.info.origin.orientation.x,
            map.map.info.origin.orientation.y,
            map.map.info.origin.orientation.z,
            map.map.info.origin.orientation.w);
}

void MapMerging::partialMapUpdate(
    const map_msgs::OccupancyGridUpdate::ConstPtr& msg, PosedMap& map)
{
  ROS_DEBUG("received partial map update");

  if (msg->x < 0 || msg->y < 0) {
    ROS_ERROR("negative coordinates, invalid update. x: %d, y: %d", msg->x,
              msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  // we must lock map. Otherwise it could break horribly if fullMapUpdate would
  // run at the same time (reaalloc of vector)
  std::lock_guard<std::mutex> lock(map.mutex);

  /* map merging is not blocked by these small updates. It can cause minor
   * inconsitency, but thats not a big issue, it could only bring parts of new
   * data to merged map. No locking needed. */

  size_t grid_xn = map.map.info.width;
  size_t grid_yn = map.map.info.height;

  if (xn > grid_xn || x0 > grid_xn || yn > grid_yn || y0 > grid_yn) {
    ROS_WARN("received update doesn't fully fit into existing map, "
             "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
             "map is: [0, %lu], [0, %lu]",
             x0, xn, y0, yn, grid_xn, grid_yn);
  }

  // update map with data
  size_t i = 0;
  for (size_t y = y0; y < yn && y < grid_yn; ++y) {
    for (size_t x = x0; x < xn && x < grid_xn; ++x) {
      size_t idx = y * grid_xn + x;  // index to grid for this specified cell
      map.map.data[idx] = msg->data[i];
      ++i;
    }
  }
}

std::string MapMerging::robotNameFromTopic(const std::string& topic)
{
  return ros::names::parentNamespace(topic);
}

geometry_msgs::Pose& operator+=(geometry_msgs::Pose& p1,
                                const geometry_msgs::Pose& p2)
{
  tf::Pose tf_p1, tf_p2;
  tf::poseMsgToTF(p1, tf_p1);
  tf::poseMsgToTF(p2, tf_p2);

  tf_p1 *= tf_p2;
  tf::poseTFToMsg(tf_p1, p1);

  return p1;
}

/* identifies topic via suffix */
bool MapMerging::isRobotMapTopic(const ros::master::TopicInfo& topic)
{
  /* test whether topic is robot_map_topic_ */
  std::string topic_namespace = ros::names::parentNamespace(topic.name);
  bool is_map_topic =
      ros::names::append(topic_namespace, robot_map_topic_) == topic.name;

  /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.name.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  /* we support only occupancy grids as maps */
  bool is_occupancy_grid = topic.datatype == "nav_msgs/OccupancyGrid";

  /* we don't want to subcribe on published merged map */
  bool is_our_topic = merged_map_publisher_.getTopic() == topic.name;

  return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
         is_map_topic;
}

/*
 * Get robot's initial position
 */
bool MapMerging::getInitPose(const std::string& name, geometry_msgs::Pose& pose)
{
  std::string merging_namespace = ros::names::append(name, "map_merge");
  double yaw = 0.0;

  bool success =
      ros::param::get(ros::names::append(merging_namespace, "init_pose_x"),
                      pose.position.x) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_y"),
                      pose.position.y) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_z"),
                      pose.position.z) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_yaw"),
                      yaw);

  pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  return success;
}

/*
 * execute()
 */
void MapMerging::executemapMerging()
{
  ros::Rate r(merging_rate_);
  while (node_.ok()) {
    mapMerging();
    r.sleep();
  }
}

void MapMerging::executetopicSubscribing()
{
  ros::Rate r(discovery_rate_);
  while (node_.ok()) {
    topicSubscribing();
    r.sleep();
  }
}

void MapMerging::executeposeEstimation()
{
  ros::Rate r(estimation_rate_);
  while (node_.ok()) {
    poseEstimation();
    r.sleep();
  }
}

/*
 * spin()
 */
void MapMerging::spin()
{
  ros::spinOnce();
  std::thread merging_thr([this]() { executemapMerging(); });
  std::thread subscribing_thr([this]() { executetopicSubscribing(); });
  std::thread estimation_thr([this]() { executeposeEstimation(); });
  ros::spin();
  merging_thr.join();
  subscribing_thr.join();
  estimation_thr.join();
}

}  // namespace map_merge

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_merge");
  // this package is still in development -- start wil debugging enabled
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  map_merge::MapMerging map_merging;
  map_merging.spin();
  return 0;
}
