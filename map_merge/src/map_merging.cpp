/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015, Jiri Horner.
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
 *   * Neither the name of the Zhi Yan nor the names of its
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

#include "../include/map_merging.hpp"

#include <functional>

#include <ros/console.h>
#include <occupancy_grid_utils/combine_grids.h>
#include <tf/transform_datatypes.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

bool has_suffix(const std::string &str, const std::string &suffix);
geometry_msgs::Pose& operator+=(geometry_msgs::Pose&, const geometry_msgs::Pose&);

MapMerging::MapMerging() {
  ros::NodeHandle relative_nh;
  ros::NodeHandle private_nh("~");
  std::string frame_id;

  private_nh.param("merging_rate", merging_rate_, 4.0);
  private_nh.param<std::string>("pose_topic", pose_topic_, "pose");
  private_nh.param<std::string>("map_topic", map_topic_, "map");
  private_nh.param<std::string>("world_frame", frame_id, "world");

  merged_map_.header.frame_id = frame_id;

  /* publishing */
  merged_map_publisher_ = relative_nh.advertise<nav_msgs::OccupancyGrid>("map", 50, true);
}

/*
 * Subcribe to pose and map topics
 */
void MapMerging::topicSubscribing() {
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);

  for(const auto& topic : topic_infos) {
    // we check only pose topic and expect map topic to exist relatively
    // to pose topic for consistency
    if(isPoseTopic(topic)) {
      std::string robot_name = robotNameFromTopic(topic.name);

      // if we don't know this robot yet
      if (!robots_.count(robot_name)) {
        geometry_msgs::Pose init_pose;
        if(!getInitPose(robot_name, init_pose)) {
          ROS_WARN("couldn't get initial position for robot [%s]\n"
            "did you defined parameters map_merging/init_pose_[xyz]? in robot namespace?",
            robot_name.c_str());
          continue;
        }

        ROS_INFO("adding robot [%s] to system", robot_name.c_str());
        maps_.emplace_front();
        PosedMap *map = &maps_.front();
        robots_.insert({ robot_name, map });
        map->initial_pose = init_pose;

        std::string pose_topic = ros::names::append(robot_name, pose_topic_);
        ROS_INFO("Subscribing to POSE topic: %s.", pose_topic.c_str());
        map->pose_sub = node_.subscribe<geometry_msgs::PoseStamped>(
          pose_topic,
          50,
          std::bind(&MapMerging::poseCallback, this, std::placeholders::_1, map)
        );

        std::string map_topic = ros::names::append(robot_name, map_topic_);
        ROS_INFO("Subscribing to MAP topic: %s.", map_topic.c_str());
        map->map_sub = node_.subscribe<nav_msgs::OccupancyGrid>(
          map_topic,
          50,
          std::bind(&MapMerging::mapCallback, this, std::placeholders::_1, map)
        );
      }
    }
  }
}

/*
 * mapMerging()
 */
void MapMerging::mapMerging() {
  std::vector<std::reference_wrapper<nav_msgs::OccupancyGrid>> maps_merged;
  maps_merged.reserve(robots_.size());
  for (auto& p_map : maps_) {
    ROS_DEBUG("Merging map at [%p]", &p_map);
    std::lock_guard<std::mutex> lock(p_map.mutex);

    // map not yet received
    if (p_map.map.data.empty()) {
      continue;
    }

    maps_merged.emplace_back(p_map.map);

    // TODO: do not merge maps which has not been updated
    p_map.updated = false;
  }

  if (maps_merged.empty()) {
    return;
  }

  occupancy_grid_utils::combineGrids(maps_merged.cbegin(), maps_merged.cend(), merged_map_);

  ROS_DEBUG("all maps merged, publishing");
  ros::Time now = ros::Time::now();
  merged_map_.info.map_load_time = now;
  merged_map_.header.stamp = now;
  merged_map_publisher_.publish(merged_map_);
}

/*********************
 * callback functions *
 *********************/
void MapMerging::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, PosedMap *map) {
  ROS_DEBUG("poseCallback");
  std::lock_guard<std::mutex> lock(map->mutex);

  map->updated = true;
  map->pose = *msg;
}

void MapMerging::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg, PosedMap *map) {
  ROS_DEBUG("mapCallback for map at [%p]", map);
  std::lock_guard<std::mutex> lock(map->mutex);

  map->updated = true;
  map->map = *msg;

  ROS_DEBUG("Adjusting origin");
  // compute global position
  ROS_DEBUG("origin %f %f, (%f, %f, %f, %f)", map->map.info.origin.position.x, 
    map->map.info.origin.position.y, map->map.info.origin.orientation.x, map->map.info.origin.orientation.y,
    map->map.info.origin.orientation.z, map->map.info.origin.orientation.w);
  map->map.info.origin += map->initial_pose;
  ROS_DEBUG("origin %f %f, (%f, %f, %f, %f)", map->map.info.origin.position.x, 
    map->map.info.origin.position.y, map->map.info.origin.orientation.x, map->map.info.origin.orientation.y,
    map->map.info.origin.orientation.z, map->map.info.origin.orientation.w);
}

/*********************
 * private function *
 *********************/
std::string MapMerging::robotNameFromTopic(const std::string& topic) {
  return ros::names::parentNamespace(topic);
}

bool has_suffix(const std::string& str, const std::string& suffix) {
  return str.size() >= suffix.size() &&
    str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

geometry_msgs::Pose& operator+=(geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
  tf::Pose tf_p1, tf_p2;
  tf::poseMsgToTF(p1, tf_p1);
  tf::poseMsgToTF(p2, tf_p2);

  tf_p1 *= tf_p2;
  tf::poseTFToMsg(tf_p1, p1);
  
  return p1;
}

/* identifies topic via suffix */
bool MapMerging::isPoseTopic(const ros::master::TopicInfo& topic) {
  return has_suffix(topic.name, pose_topic_);
}

/*
 * Get robot's initial position
 */
bool MapMerging::getInitPose(const std::string& name, geometry_msgs::Pose& pose) {
  std::string merging_namespace = ros::names::append(name, "map_merging");
  double yaw;

  bool success = ros::param::get(ros::names::append(merging_namespace, "init_pose_x"), pose.position.x) &&
    ros::param::get(ros::names::append(merging_namespace, "init_pose_y"), pose.position.y) &&
    ros::param::get(ros::names::append(merging_namespace, "init_pose_z"), pose.position.z) &&
    ros::param::get(ros::names::append(merging_namespace, "init_pose_yaw"), yaw);

  pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  return success;
}

/*
 * execute()
 */
void MapMerging::execute() {
  ros::Rate r(merging_rate_);
  while(node_.ok()) {
    topicSubscribing();
    mapMerging();
    r.sleep();
  }
}

/*
 * spin()
 */
void MapMerging::spin() {
  ros::spinOnce();
  boost::thread t(boost::bind(&MapMerging::execute, this));
  ros::spin();
  t.join();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_merging");
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  MapMerging map_merging;
  map_merging.spin();
  return 0;
}
