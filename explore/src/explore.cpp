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

#include <explore/explore.h>
#include <explore/explore_frontier.h>

#include <thread>

namespace explore {

Explore::Explore() :
  private_nh_("~"),
  tf_listener_(ros::Duration(10.0)),
  costmap_client_(private_nh_, relative_nh_, &tf_listener_),
  planner_("explore_planner", costmap_client_.getCostmap(),
    costmap_client_.getGlobalFrameID()),
  move_base_client_("move_base"),
  explorer_(&costmap_client_, &planner_),
  prev_plan_size_(0),
  done_exploring_(false)
{
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", progress_timeout_, 30.0);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0); // TODO: set this back to 0.318 once getOrientationChange is fixed
  private_nh_.param("gain_scale", gain_scale_, 1.0);

  if(visualize_) {
    marker_array_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }
}

void Explore::publishFrontiers() {
  visualization_msgs::MarkerArray markers;
  explorer_.getVisualizationMarkers(markers);
  ROS_DEBUG("publishing %lu markers", markers.markers.size());
  marker_array_publisher_.publish(markers);
}

void Explore::makePlan() {
  // this method may be called from callback
  std::lock_guard<std::mutex> lock(planning_mutex_);

  tf::Stamped<tf::Pose> robot_pose;
  costmap_client_.getRobotPose(robot_pose);

  std::vector<geometry_msgs::Pose> goals;

  explorer_.getExplorationGoals(robot_pose, goals, potential_scale_, orientation_scale_, gain_scale_);
  if (goals.size() == 0) {
    done_exploring_ = true;
    ROS_DEBUG("no explorations goals found");
  } else {
    ROS_DEBUG("found %lu explorations goals", goals.size());
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    publishFrontiers();
  }

  bool valid_plan = false;
  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped goal_pose, robot_pose_msg;
  tf::poseStampedTFToMsg(robot_pose, robot_pose_msg);

  goal_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal_pose.header.stamp = ros::Time::now();
  planner_.computePotential(robot_pose_msg.pose.position); // just to be safe, though this should already have been done in explorer_->getExplorationGoals
  size_t blacklist_count = 0;
  for (auto& goal : goals) {
    goal_pose.pose = goal;
    if (goalOnBlacklist(goal_pose)) {
      ++blacklist_count;
      continue;
    }

    valid_plan = planner_.getPlanFromPotential(goal_pose, plan) && !plan.empty();
    if (valid_plan) {
      ROS_DEBUG("got valid plan");
      break;
    } else {
      ROS_DEBUG("got invalid plan");
    }
  }

  if (valid_plan) {
    if (prev_plan_size_ != plan.size()) {
      time_since_progress_ = 0.0;
    } else {
      double dx = prev_goal_.pose.position.x - goal_pose.pose.position.x;
      double dy = prev_goal_.pose.position.y - goal_pose.pose.position.y;
      double dist = sqrt(dx*dx+dy*dy);
      if (dist < 0.01) {
        time_since_progress_ += 1.0 / planner_frequency_;
      }
    }

    // black list goals for which we've made no progress for a long time
    if (time_since_progress_ > progress_timeout_) {
      frontier_blacklist_.push_back(goal_pose);
      ROS_DEBUG("Adding current goal to black list");
    }

    prev_plan_size_ = plan.size();
    prev_goal_ = goal_pose;

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose;
    move_base_client_.sendGoal(goal, boost::bind(&Explore::reachedGoal, this, _1, _2, goal_pose));

  } else {
    ROS_WARN("Done exploring with %lu goals left that could not be reached."
      "There are %lu goals on our blacklist, and %lu of the frontier goals are too close to"
      "them to pursue. The rest had global planning fail to them. \n",
      goals.size(), frontier_blacklist_.size(), blacklist_count);
    ROS_INFO("Exploration finished. Hooray.");
    done_exploring_ = true;
  }
}

bool Explore::goalOnBlacklist(const geometry_msgs::PoseStamped& goal){
  costmap_2d::Costmap2D *costmap2d = costmap_client_.getCostmap();

  //check if a goal is on the blacklist for goals that we're pursuing
  for(auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.pose.position.x - frontier_goal.pose.position.x);
    double y_diff = fabs(goal.pose.position.y - frontier_goal.pose.position.y);

    if(x_diff < 2 * costmap2d->getResolution() && y_diff < 2 * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(
  const actionlib::SimpleClientGoalState& status,
  const move_base_msgs::MoveBaseResultConstPtr&,
  const geometry_msgs::PoseStamped& frontier_goal){

  ROS_DEBUG("Reached goal");
  if(status == actionlib::SimpleClientGoalState::ABORTED){
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  //create a plan from the frontiers left and send a new goal to move_base
  makePlan();
}

void Explore::execute() {
  while (!move_base_client_.waitForServer(ros::Duration(5,0)))
    ROS_WARN("Waiting to connect to move_base server");

  ROS_INFO("Connected to move_base server");

  // This call sends the first goal, and sets up for future callbacks.
  makePlan();

  ros::Rate r(planner_frequency_);
  while (relative_nh_.ok() && (!done_exploring_)) {
    makePlan();
    r.sleep();
  }

  move_base_client_.cancelAllGoals();
}

void Explore::spin() {
  ros::spinOnce();
  std::thread t(&Explore::execute, this);
  ros::spin();
  if (t.joinable())
    t.join();
}

} // namespace explore

int main(int argc, char** argv){
  ros::init(argc, argv, "explore");

  explore::Explore explore;
  explore.spin();

  return(0);
}

