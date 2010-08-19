/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
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

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <LinearMath/btVector3.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <tf/transform_listener.h>

namespace explore {

struct FrontierPoint{
  int idx;     //position
  btVector3 d; //direction

  FrontierPoint(int idx_, btVector3 d_) : idx(idx_), d(d_) {};
};

struct Frontier {
  geometry_msgs::Pose pose;
  int size;
  Frontier():pose(),size(0) {}
  Frontier(const Frontier& copy) { pose = copy.pose; size = copy.size; }
};

struct WeightedFrontier {
  Frontier frontier;
  float cost;
  bool operator<(const WeightedFrontier& o) const { return cost < o.cost; }
  WeightedFrontier():frontier(),cost(1e9) {}
  WeightedFrontier(const WeightedFrontier& copy) { frontier = copy.frontier; cost = copy.cost; }
};

/**
 * @class ExploreFrontier
 * @brief A class that will identify frontiers in a partially explored map
 */
class ExploreFrontier {
private:
  nav_msgs::OccupancyGrid map_;

  uint lastMarkerCount_;
  float costmapResolution_;

  navfn::NavfnROS* planner_;
protected:
  std::vector<Frontier> frontiers_;

  /**
   * @brief Finds frontiers and populates frontiers_
   * @param costmap The costmap to search for frontiers
   */
  virtual void findFrontiers(costmap_2d::Costmap2DROS& costmap_);

  /**
   * @brief Calculates cost to explore frontier
   * @param frontier to evaluate
   */
  virtual float getFrontierCost(const Frontier& frontier);

  /**
   * @brief Calculates how much the robot would have to turn to face this frontier
   * @param frontier to evaluate
   * @param robot_pose current pose
   */
  virtual double getOrientationChange(const Frontier& frontier, const tf::Stamped<tf::Pose>& robot_pose);

  /**
   * @brief Calculates potential information gain of exploring frontier
   * @param frontier to evaluate
   */
  virtual float getFrontierGain(const Frontier& frontier, double map_resolution);

public:
  ExploreFrontier();
  virtual ~ExploreFrontier();

  /**
   * @brief Returns all frontiers
   * @param costmap The costmap to search for frontiers
   * @param frontiers Will be filled with current frontiers
   * @return True if at least one frontier was found
   */
  virtual bool getFrontiers(costmap_2d::Costmap2DROS& costmap, std::vector<geometry_msgs::Pose>& frontiers);

  /**
   * @brief Returns a list of frontiers, sorted by the planners estimated cost to visit each frontier
   * @param costmap The costmap to search for frontiers
   * @param start The current position of the robot
   * @param goals Will be filled with sorted list of current goals
   * @param planner A planner to evaluate the cost of going to any frontier
   * @param potential_scale A scaling for the potential to a frontier goal point for the frontier's cost
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
  virtual bool getExplorationGoals(costmap_2d::Costmap2DROS& costmap, tf::Stamped<tf::Pose> robot_pose, navfn::NavfnROS* planner, std::vector<geometry_msgs::Pose>& goals, double cost_scale, double orientation_scale, double gain_scale);

  /**
   * @brief  Returns markers representing all frontiers
   * @param markers All markers will be added to this vector
   */
  virtual void getVisualizationMarkers(std::vector<visualization_msgs::Marker>& markers);
};

}

#endif /* EXPLORE_FRONTIER_H_ */
