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

inline static bool operator==(const geometry_msgs::PoseStamped& one,
                              const geometry_msgs::PoseStamped& two)
{
  double dx = one.pose.position.x - two.pose.position.x;
  double dy = one.pose.position.y - two.pose.position.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , planner_("explore_planner", costmap_client_.getCostmap(),
             costmap_client_.getGlobalFrameID())
  , move_base_client_("move_base")
  , explorer_(&costmap_client_, &planner_)
  , prev_plan_size_(0)
{
  double timeout;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  // TODO: set this back to 0.318 once getOrientationChange is fixed
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::publishFrontiers()
{
  visualization_msgs::MarkerArray markers;
  explorer_.getVisualizationMarkers(markers);

  // make frontiers on blacklist red
  for (auto& m : markers.markers) {
    if (goalOnBlacklist(m.pose)) {
      m.color.r = 255;
      m.color.g = 0;
      m.color.b = 0;
      m.color.a = 255;
    }
  }

  ROS_DEBUG("publishing %lu markers", markers.markers.size());
  marker_array_publisher_.publish(markers);
}

void Explore::makePlan()
{
  tf::Stamped<tf::Pose> robot_pose;
  costmap_client_.getRobotPose(robot_pose);
  std::vector<geometry_msgs::Pose> goals;
  explorer_.getExplorationGoals(robot_pose, goals, potential_scale_,
                                orientation_scale_, gain_scale_);

  if (goals.empty()) {
    stop();
    ROS_DEBUG("no explorations goals found");
    return;
  }
  ROS_DEBUG("found %lu explorations goals", goals.size());

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
  // just to be safe, though this should already have been done in
  // explorer_->getExplorationGoals
  planner_.computePotential(robot_pose_msg.pose.position);
  size_t blacklist_count = 0;
  for (auto& goal : goals) {
    goal_pose.pose = goal;
    if (goalOnBlacklist(goal_pose.pose)) {
      ++blacklist_count;
      continue;
    }

    valid_plan =
        planner_.getPlanFromPotential(goal_pose, plan) && !plan.empty();
    if (valid_plan) {
      ROS_DEBUG("got valid plan");
      break;
    } else {
      ROS_DEBUG("got invalid plan");
    }
  }

  if (valid_plan) {
    // if the plan is to pursue the current goal
    bool same_goal = prev_goal_ == goal_pose;
    if (!same_goal || prev_plan_size_ != plan.size()) {
      // we have different goal or we made some progress
      last_progress_ = ros::Time::now();
    }

    // black list goals for which we've made no progress for a long time
    if (ros::Time::now() - last_progress_ > progress_timeout_) {
      frontier_blacklist_.push_back(goal_pose);
      ROS_DEBUG("Adding current goal to black list");
    }

    prev_plan_size_ = plan.size();
    prev_goal_ = goal_pose;

    if (same_goal) {
      return;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose;
    move_base_client_.sendGoal(
        goal, boost::bind(&Explore::reachedGoal, this, _1, _2, goal_pose));

  } else {
    ROS_WARN("Done exploring with %lu goals left that could not be reached."
             "There are %lu goals on our blacklist, and %lu of the frontier "
             "goals are too close to"
             "them to pursue. The rest had global planning fail to them. \n",
             goals.size(), frontier_blacklist_.size(), blacklist_count);
    ROS_INFO("Exploration finished. Hooray.");
    stop();
  }
}

bool Explore::goalOnBlacklist(const geometry_msgs::Pose& goal)
{
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.position.x - frontier_goal.pose.position.x);
    double y_diff = fabs(goal.position.y - frontier_goal.pose.position.y);

    if (x_diff < 2 * costmap2d->getResolution() &&
        y_diff < 2 * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::PoseStamped& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");

  explore::Explore explore;
  ros::spin();

  return 0;
}
