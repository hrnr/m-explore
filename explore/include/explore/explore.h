/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
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
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <explore/explore_frontier.h>
#include <explore/costmap_client.h>

/* I have been using modified version of navfn for a long time here.
Unfornutely my changes are affecting ABI of the component, so the can't be
accepted for ROS Jade or below. I expect them be accepted for ROS Karmic. Then
this should be moved back to using standard ROS navfn `<navfn/navfn_ros.h>`.
For the meanwhile, to allow seamless building, I will be using internal
version included from ROS. */

#include <explore/navfn_ros.h>

namespace explore {

/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the robot base to explore its environment.
 */
class Explore {
public:
  Explore();

  void execute();

  void spin();

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  /**
   * @brief  Publish a frontiers as markers
   */
  void publishFrontiers();

  void reachedGoal(
    const actionlib::SimpleClientGoalState& status,
    const move_base_msgs::MoveBaseResultConstPtr& result,
    const geometry_msgs::PoseStamped& frontier_goal);

  bool goalOnBlacklist(const geometry_msgs::PoseStamped& goal);

  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  ros::Publisher marker_array_publisher_;
  tf::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  navfn::NavfnROS planner_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
  ExploreFrontier explorer_;
  std::mutex planning_mutex_;

  tf::Stamped<tf::Pose> global_pose_;
  double planner_frequency_;
  std::vector<geometry_msgs::PoseStamped> frontier_blacklist_;
  geometry_msgs::PoseStamped prev_goal_;
  size_t prev_plan_size_;
  double time_since_progress_, progress_timeout_;
  double potential_scale_, orientation_scale_, gain_scale_;
  bool done_exploring_;
  bool visualize_;
};

}

#endif

