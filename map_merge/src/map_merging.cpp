/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
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

#include "map_merging.hpp"

MapMerging::MapMerging() {
  ros::NodeHandle private_nh("~");
  private_nh.param("merging_rate", merging_rate_, 10.0);
  private_nh.param<int>("max_number_robots", max_number_robots_, 100);
  private_nh.param<double>("max_comm_distance", max_comm_distance_, 10.0);
  private_nh.param<std::string>("pose_topic", pose_topic_, "pose");
  private_nh.param<std::string>("map_topic", map_topic_, "map");

  /* publisher params */
  map_has_been_merged_ = new bool[max_number_robots_]();
  merged_map_publisher_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

  /* get robot's name */
  my_name_ = ros::this_node::getNamespace(); // Get the robot's name.
  my_name_.erase(0, 1); // [ROS_BUG #3671] Remove the first slash in front of the name.

  my_id_ = tm_id_ = UNKNOWN;
}

MapMerging::~MapMerging() {
  delete[] map_has_been_merged_;
}

/*
 * topicSubscribing()
 */
void MapMerging::topicSubscribing() {
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);

  for(ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin(); it_topic != topic_infos.end(); ++it_topic) {
    const ros::master::TopicInfo &published_topic = *it_topic;
    /* robot pose subscribing */
    if(published_topic.name.find(pose_topic_) != std::string::npos && !poseFound(published_topic.name, poses_)) {
      ROS_INFO("[%s]:Subscribe to POSE topic: %s.", (ros::this_node::getName()).c_str(), published_topic.name.c_str());
      poses_.push_back(Pose(published_topic.name, false));
      pose_subsribers_.push_back(node_.subscribe<geometry_msgs::PoseStamped>(published_topic.name, 1, boost::bind(&MapMerging::poseCallback, this, _1, poses_.size()-1)));
      /* get robot's ID */
      if(my_name_.size() > 0 && published_topic.name.compare(0, my_name_.size(), my_name_) == 0) {
        my_id_ = poses_.size()-1;
        ROS_INFO("[%s]:My name is %s, ID = %d.", (ros::this_node::getName()).c_str(), my_name_.c_str(), my_id_);
      }
    }
  }

  if(my_id_ == UNKNOWN) {
    ROS_WARN("[%s]:Can not get robot's pose.", (ros::this_node::getName()).c_str());
  }
}

/*
 * handShaking(): handshaking if robots are within the communication range.
 */
void MapMerging::handShaking() {
  if(my_id_ == UNKNOWN || tm_id_ != UNKNOWN)
    return;

  geometry_msgs::Pose my_pose, tm_pose;
  if(poses_[my_id_].received && getInitPose(my_name_, my_pose)) {
    my_pose.position.x += poses_[my_id_].data.pose.position.x;
    my_pose.position.y += poses_[my_id_].data.pose.position.y;
    my_pose.position.z += poses_[my_id_].data.pose.position.z;
    poses_[my_id_].received = false;
    //std::cout << "[" << ros::this_node::getName() << "] " << poses_[my_id_].data.header.frame_id << " x = " << my_pose.position.x << " y = " << my_pose.position.y << std::endl;
    
    for(int i = 0; i < (int)poses_.size(); i++) {
      if(i != my_id_ && !map_has_been_merged_[i]) {
        tm_name_ = robotNameParsing(poses_[i].name);
        if(poses_[i].received && getInitPose(tm_name_, tm_pose)) {
          tm_pose.position.x += poses_[i].data.pose.position.x;
          tm_pose.position.y += poses_[i].data.pose.position.y;
          tm_pose.position.z += poses_[i].data.pose.position.z;
          poses_[i].received = false;
          if(distBetween(tm_pose.position, my_pose.position) < max_comm_distance_) {
            ROS_DEBUG("[%s]:Handshake with %s.", (ros::this_node::getName()).c_str(), tm_name_.c_str());
            tm_id_ = i;
            break;
          }
        }
      }
    }
  }
  
  if(tm_id_ == UNKNOWN) {
    // do something here ...
  }
}

/*
 * mapMerging()
 */
void MapMerging::mapMerging() {
  if(my_id_ == UNKNOWN || tm_id_ == UNKNOWN)
    return;

  std::string map_topic_name;
  for(int i = 0; i < (int)poses_.size(); i++) {
    map_topic_name = robotNameParsing(poses_[i].name)+"/"+map_topic_;
    if(!mapFound(map_topic_name, maps_)) {
      maps_.push_back(Map(map_topic_name, false));
      map_subsribers_.push_back(node_.subscribe<nav_msgs::OccupancyGrid>(map_topic_name, 1, boost::bind(&MapMerging::mapCallback, this, _1, maps_.size()-1)));
      ROS_INFO("[%s]:Subscribe to MAP topic: %s.", (ros::this_node::getName()).c_str(), map_topic_name.c_str());
    }
  }

  bool map_merging_end = false;
  if(maps_[my_id_].received) {
    if(maps_[tm_id_].received) {
      ROS_DEBUG("[%s]:Exchange map with %s.", (ros::this_node::getName()).c_str(), tm_name_.c_str());
      geometry_msgs::Pose delta;
      if(getRelativePose(my_name_, tm_name_, delta, maps_[my_id_].data.info.resolution)) {
        //std::cout << "[" << ros::this_node::getName() << "] deltaX = " << delta.position.x << " deltaY = " << delta.position.y << std::endl;
        if(!map_has_been_merged_[my_id_]) {
          merged_map_ = maps_[my_id_].data;
          map_has_been_merged_[my_id_] = true;
        }
        greedyMerging(round(delta.position.x), round(delta.position.y), tm_id_);
        map_merging_end = true;
      }
      maps_[tm_id_].received = false;
    }
    maps_[my_id_].received = false;
  }

  if(map_merging_end) {
    ROS_DEBUG("[%s]:Map has been merged with %s.", (ros::this_node::getName()).c_str(), tm_name_.c_str());
    map_has_been_merged_[tm_id_] = true;
    tm_id_ = UNKNOWN;

    // It has coordinated with all that can be coordinated, so reset.
    bool reset_and_publish = true;
    for(int i = 0; i < (int)poses_.size(); i++) {
      if(!map_has_been_merged_[i]) {
        reset_and_publish = false;
        break;
      }
    }

    if(reset_and_publish) {
      for(int i = 0; i < (int)poses_.size(); i++)
        map_has_been_merged_[i] = false;
      merged_map_publisher_.publish(merged_map_);
    }
  }
}

/*********************
 * callback function *
 *********************/
void MapMerging::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id) {
  ROS_DEBUG("poseCallback");
  //boost::mutex::scoped_lock pose_lock (pose_mutex_);
  poses_[id].received = true;
  poses_[id].data = *msg;
}

void MapMerging::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg, int id) {
  ROS_DEBUG("mapCallback");
  //boost::mutex::scoped_lock map_lock (map_mutex_);
  maps_[id].received = true;
  maps_[id].data = *msg;
}

/*********************
 * private function *
 *********************/
bool MapMerging::poseFound(std::string name, std::vector<Pose> &poses) {
  for(std::vector<Pose>::iterator it_pose = poses.begin(); it_pose != poses.end(); ++it_pose) {
    if((*it_pose).name.compare(name) == 0)
      return true;
  }
  return false;
}

bool MapMerging::mapFound(std::string name, std::vector<Map> &maps) {
  for(std::vector<Map>::iterator it_map = maps.begin(); it_map != maps.end(); ++it_map) {
    if((*it_map).name.compare(name) == 0)
      return true;
  }
  return false;
}

std::string MapMerging::robotNameParsing(std::string s) {
  return s.erase(s.find('/', 1), s.size());
}

/*
 * Get robot's initial position
 * TODO: get orientation
 */
bool MapMerging::getInitPose(std::string name, geometry_msgs::Pose &pose) {
  if(ros::param::get(name+"/map_merging/init_pose_x", pose.position.x) &&
     ros::param::get(name+"/map_merging/init_pose_y", pose.position.y) &&
     ros::param::get(name+"/map_merging/init_pose_z", pose.position.z)) {
    return true;
  }
  return false;
}

/*
 * Get the relative position of two robots
 * TODO: get orientation
 */
bool MapMerging::getRelativePose(std::string n, std::string m, geometry_msgs::Pose &delta, double resolution) {
  geometry_msgs::Pose p, q;
  if(ros::param::get(n+"/map_merging/init_pose_x", p.position.x) &&
     ros::param::get(n+"/map_merging/init_pose_y", p.position.y) &&
     ros::param::get(n+"/map_merging/init_pose_z", p.position.z) &&
     ros::param::get(m+"/map_merging/init_pose_x", q.position.x) &&
     ros::param::get(m+"/map_merging/init_pose_y", q.position.y) &&
     ros::param::get(m+"/map_merging/init_pose_z", q.position.z)) {
    delta.position.x = round((p.position.x - q.position.x) / resolution);
    delta.position.y = round((p.position.y - q.position.y) / resolution);
    delta.position.z = round((p.position.z - q.position.z) / resolution);
    return true;
  }
  return false;
}

double MapMerging::distBetween(geometry_msgs::Point &p, geometry_msgs::Point &q) {
  // Euclidean distance
  return sqrt((p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z));
}

/*
 * Algorithm 1 - Greedy Merging
 * yz14simpar
 */
void MapMerging::greedyMerging(int delta_x, int delta_y, int map_id) {
  for(int i = 0; i < (int)merged_map_.info.width; i++) {
    for(int j = 0; j < (int)merged_map_.info.height; j++) {
      if(i+delta_x >= 0 && i+delta_x < (int)maps_[map_id].data.info.width &&
         j+delta_y >= 0 && j+delta_y < (int)maps_[map_id].data.info.height) {
        if((int)merged_map_.data[i+j*(int)merged_map_.info.width] == UNKNOWN)
          merged_map_.data[i+j*(int)merged_map_.info.width] = maps_[map_id].data.data[i+delta_x+(j+delta_y)*(int)maps_[map_id].data.info.width];
      }
    }
  }
}

/*
 * execute()
 */
void MapMerging::execute() {
  ros::Rate r(merging_rate_);
  while(node_.ok()) {
    topicSubscribing();
    handShaking();
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
  MapMerging map_merging;
  map_merging.spin();
  return 0;
}
