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

#ifndef MAP_MERGE_H_
#define MAP_MERGE_H_

#include <vector>
#include <unordered_map>
#include <mutex>
#include <forward_list>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

namespace map_merge
{
struct PosedMap {
  std::mutex mutex;

  geometry_msgs::Pose initial_pose;
  nav_msgs::OccupancyGrid map;

  ros::Subscriber map_sub;
  ros::Subscriber map_updates_sub;
};

class MapMerging
{
private:
  ros::NodeHandle node_;

  /* parameters */
  double merging_rate_;
  double discovery_rate_;
  double estimation_rate_;
  double confidence_treshold_;
  std::string robot_map_topic_;
  std::string robot_map_updates_topic_;
  std::string robot_namespace_;
  bool have_initial_poses_;

  /* publisher */
  nav_msgs::OccupancyGrid merged_map_;
  ros::Publisher merged_map_publisher_;

  // maps robots namespaces to maps. does not own
  std::unordered_map<std::string, PosedMap*> robots_;
  // owns maps -- iterator safe
  std::forward_list<PosedMap> maps_;
  size_t maps_size_;
  // does not own. view of only grids from PosedMaps for merging
  typedef std::vector<std::reference_wrapper<nav_msgs::OccupancyGrid>>
      grids_view_t;
  // always contains all grids
  grids_view_t all_grids_view_;
  // aplicable only when estimations is on, will contain only grids which poses
  // could be properly estimated
  grids_view_t estimated_grids_view_;
  // this must be locked exclusively when modifying grid_view_ or changing
  // metadata (esp. size!) of OccupancyGrids inside. This could otherwise break
  // horribly because merging algorithm needs to compute merged map size first.
  boost::shared_mutex merging_mutex_;

  std::string robotNameFromTopic(const std::string& topic);
  bool isRobotMapTopic(const ros::master::TopicInfo& topic);
  bool getInitPose(const std::string& name, geometry_msgs::Pose& pose);

  void fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
                     PosedMap& map);
  void partialMapUpdate(const map_msgs::OccupancyGridUpdate::ConstPtr& msg,
                        PosedMap& map);

public:
  MapMerging();

  void spin();
  void executetopicSubscribing();
  void executemapMerging();
  void executeposeEstimation();

  void topicSubscribing();
  void mapMerging();
  /**
   * @brief Estimates initial positions of grids
   * @details Relevant only if initial poses are not known
   */
  void poseEstimation();
};

}  // namespace map_merge

#endif /* MAP_MERGE_H_ */
