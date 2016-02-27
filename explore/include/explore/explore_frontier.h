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

#ifndef EXPLORE_FRONTIER_H_
#define EXPLORE_FRONTIER_H_

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/LinearMath/Vector3.h>

#include <explore/costmap_client.h>

/* I have been using modified version of navfn for a long time here.
Unfornutely my changes are affecting ABI of the component, so the can't be
accepted for ROS Jade or below. I expect them be accepted for ROS Karmic. Then
this should be moved back to using standard ROS navfn `<navfn/navfn_ros.h>`.
For the meanwhile, to allow seamless building, I will be using internal
version included from ROS. */

#include <explore/navfn_ros.h>

namespace explore {

struct FrontierPoint {
  size_t idx;     //position
  tf::Vector3 d; //direction

  FrontierPoint() : idx(0) {}
  FrontierPoint(size_t idx_, const tf::Vector3& d_) : idx(idx_), d(d_) {}
};

struct Frontier {
  size_t size;
  geometry_msgs::Pose pose;

  Frontier() : size(0) {}
  Frontier(size_t size_, const geometry_msgs::Pose& pose_) : size(size_), pose(pose_) {}
};

struct WeightedFrontier {
  double cost;
  Frontier frontier;

  WeightedFrontier() : cost(1e9) {}
  WeightedFrontier(double cost_, const Frontier& frontier_) : cost(cost_), frontier(frontier_) {}
  bool operator<(const WeightedFrontier& o) const { return cost < o.cost; }
};

/**
 * @class ExploreFrontier
 * @brief A class that will identify frontiers in a partially explored map
 */
class ExploreFrontier {
private:
  // can't be const as we will do locking
  Costmap2DClient* const costmap_client_;
  costmap_2d::Costmap2D* const costmap_;
  navfn::NavfnROS* const planner_;

  nav_msgs::OccupancyGrid map_;
  std::vector<Frontier> frontiers_;

  size_t last_markers_count_;

  /**
   * @brief Finds frontiers and populates frontiers_
   */
  void findFrontiers();

  /**
   * @brief Calculates cost to explore frontier
   * @param frontier to evaluate
   */
  double getFrontierCost(const Frontier& frontier);

  /**
   * @brief Calculates how much the robot would have to turn to face this frontier
   * @param frontier to evaluate
   * @param robot_pose current pose
   */
  double getOrientationChange(const Frontier& frontier, const tf::Stamped<tf::Pose>& robot_pose) const;

  /**
   * @brief Calculates potential information gain of exploring frontier
   * @param frontier to evaluate
   */
  double getFrontierGain(const Frontier& frontier) const;

public:
  /**
   * @brief Constructs ExploreFrintier with costmap to work on
   * @param costmap The costmap to search for frontiers
   * @param planner A planner to evaluate the cost of going to any frontier
   * @throws std::invalid_argument if any argument is NULL
   */
  ExploreFrontier(Costmap2DClient* costmap, navfn::NavfnROS* planner);

  /**
   * @brief Returns all frontiers
   * @param frontiers Will be filled with current frontiers
   * @return True if at least one frontier was found
   */
  bool getFrontiers(std::vector<geometry_msgs::Pose>& frontiers);

  /**
   * @brief Returns a list of frontiers, sorted by the planners estimated cost to visit each frontier
   * @param start The current position of the robot
   * @param goals Will be filled with sorted list of current goals
   * @param cost_scale A scaling for the potential to a frontier goal point for the frontier's cost
   * @param orientation_scale A scaling for the change in orientation required to get to a goal point for the frontier's cost
   * @param gain_scale A scaling for the expected information gain to get to a goal point for the frontier's cost
   * @return True if at least one frontier was found
   *
   * The frontiers are weighted by a simple cost function, which prefers
   * frontiers which are large and close:
   *   frontier cost = travel cost / frontier size
   *
   * Several different positions are evaluated for each frontier. This
   * improves the robustness of goals which may lie near other obstacles
   * which would prevent planning.
   */
  bool getExplorationGoals(
    tf::Stamped<tf::Pose> start,
    std::vector<geometry_msgs::Pose>& goals,
    double cost_scale, double orientation_scale, double gain_scale
  );

  /**
   * @brief  Returns markers representing all frontiers
   * @param markers All markers will be added to this vector
   */
  void getVisualizationMarkers(visualization_msgs::MarkerArray& markers);
};

}

#endif /* EXPLORE_FRONTIER_H_ */
