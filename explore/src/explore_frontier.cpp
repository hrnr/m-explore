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

#include <explore/explore_frontier.h>

#include <exception>
#include <mutex>
#include <boost/thread.hpp>

namespace explore {

ExploreFrontier::ExploreFrontier(Costmap2DClient* costmap, navfn::NavfnROS* planner) :
  costmap_client_(costmap),
  costmap_(costmap_client_->getCostmap()),
  planner_(planner),
  last_markers_count_(0)
  {
    if (!costmap || !planner) {
      throw std::invalid_argument("costmap and planner passed to constructor may not be NULL");
    }
  }

bool ExploreFrontier::getFrontiers(std::vector<geometry_msgs::Pose>& frontiers)
{
  findFrontiers();
  if (frontiers_.size() == 0)
    return false;

  frontiers.clear();
  for (auto& frontier : frontiers_) {
    frontiers.push_back(frontier.pose);
  }

  return true;
}

double ExploreFrontier::getFrontierCost(const Frontier& frontier) {
  ROS_DEBUG("cost of frontier: %f, at position: (%.2f, %.2f, %.2f)",
    planner_->getPointPotential(frontier.pose.position),
    frontier.pose.position.x, frontier.pose.position.y, tf::getYaw(frontier.pose.orientation));

  return planner_->getPointPotential(frontier.pose.position); // / 20000.0;
}

// TODO: what is this doing exactly?
double ExploreFrontier::getOrientationChange(
  const Frontier& frontier,
  const tf::Stamped<tf::Pose>& robot_pose) const {
  double robot_yaw = tf::getYaw(robot_pose.getRotation());
  double robot_atan2 = atan2(robot_pose.getOrigin().y()
    + sin(robot_yaw), robot_pose.getOrigin().x() + cos(robot_yaw));
  double frontier_atan2 = atan2(frontier.pose.position.x, frontier.pose.position.y);
  double orientation_change = robot_atan2 - frontier_atan2;
//  ROS_DEBUG("Orientation change: %.3f degrees, (%.3f radians)", orientation_change * (180.0 / M_PI), orientation_change);
  return orientation_change;
}

double ExploreFrontier::getFrontierGain(const Frontier& frontier) const {
  return frontier.size * costmap_->getResolution();
}

bool ExploreFrontier::getExplorationGoals(
  tf::Stamped<tf::Pose> robot_pose,
  std::vector<geometry_msgs::Pose>& goals,
  double potential_scale, double orientation_scale, double gain_scale)
{
  findFrontiers();

  if (frontiers_.size() == 0) {
    ROS_DEBUG("no frontiers found");
    return false;
  }

  geometry_msgs::Point start;
  start.x = robot_pose.getOrigin().x();
  start.y = robot_pose.getOrigin().y();
  start.z = robot_pose.getOrigin().z();
  planner_->computePotential(start);

  //we'll make sure that we set goals for the frontier at least the circumscribed
  //radius away from unknown space
  // 2015: this may not be necessary
  // float step = -1.0 * costmap_resolution;
  // int c = ceil(costmap.getCircumscribedRadius() / costmap_resolution);
  // WeightedFrontier goal;
  std::vector<WeightedFrontier> weightedFrontiers;
  weightedFrontiers.reserve(frontiers_.size());
  for (auto& frontier: frontiers_) {
    WeightedFrontier weightedFrontier;
    weightedFrontier.frontier = frontier;

    tf::Point p(frontier.pose.position.x, frontier.pose.position.y, frontier.pose.position.z);
    tf::Quaternion bt;
    tf::quaternionMsgToTF(frontier.pose.orientation, bt);
    tf::Vector3 v(cos(bt.getAngle()), sin(bt.getAngle()), 0.0);

//     for (int j=0; j <= c; j++) {
//       tf::Vector3 check_point = p + (v * (step * j));
//       weightedFrontier.frontier.pose.position.x = check_point.x();
//       weightedFrontier.frontier.pose.position.y = check_point.y();
//       weightedFrontier.frontier.pose.position.z = check_point.z();

//       weightedFrontier.cost = potential_scale * getFrontierCost(weightedFrontier.frontier) + orientation_scale * getOrientationChange(weightedFrontier.frontier, robot_pose) - gain_scale * getFrontierGain(weightedFrontier.frontier, costmapResolution_);
// //      weightedFrontier.cost = getFrontierCost(weightedFrontier.frontier) - getFrontierGain(weightedFrontier.frontier, costmapResolution_);
// //      ROS_DEBUG("cost: %f (%f * %f + %f * %f - %f * %f)",
// //          weightedFrontier.cost,
// //          potential_scale,
// //          getFrontierCost(weightedFrontier.frontier),
// //          orientation_scale,
// //          getOrientationChange(weightedFrontier.frontier, robot_pose),
// //          gain_scale,
// //          getFrontierGain(weightedFrontier.frontier, costmapResolution_) );
//       weightedFrontiers.push_back(weightedFrontier);
//     }
    weightedFrontier.cost =
      potential_scale * getFrontierCost(weightedFrontier.frontier)
      + orientation_scale * getOrientationChange(weightedFrontier.frontier, robot_pose)
      - gain_scale * getFrontierGain(weightedFrontier.frontier);
    weightedFrontiers.push_back(std::move(weightedFrontier));
  }

  goals.clear();
  goals.reserve(weightedFrontiers.size());
  std::sort(weightedFrontiers.begin(), weightedFrontiers.end());
  for (uint i = 0; i < weightedFrontiers.size(); i++) {
    goals.push_back(weightedFrontiers[i].frontier.pose);
  }
  return (goals.size() > 0);
}

void ExploreFrontier::findFrontiers() {
  frontiers_.clear();

  const size_t w = costmap_->getSizeInCellsX();
  const size_t h = costmap_->getSizeInCellsY();
  const size_t size = w * h;

  map_.info.width = w;
  map_.info.height = h;
  map_.data.resize(size);
  map_.info.resolution = costmap_->getResolution();
  map_.info.origin.position.x = costmap_->getOriginX();
  map_.info.origin.position.y = costmap_->getOriginY();

  // lock as we are accessing raw underlying map
  auto *mutex = costmap_->getMutex();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  // Find all frontiers (open cells next to unknown cells).
  const unsigned char* map = costmap_->getCharMap();
  ROS_DEBUG_COND(!map, "no map available");
  for (size_t i = 0; i < size; ++i) {
//    //get the world point for the index
//    unsigned int mx, my;
//    costmap.indexToCells(i, mx, my);
//    geometry_msgs::Point p;
//    costmap.mapToWorld(mx, my, p.x, p.y);
//
    //check if the point has valid potential and is next to unknown space
//    bool valid_point = planner_->validPointPotential(p);
    bool valid_point = (map[i] < costmap_2d::LETHAL_OBSTACLE);
    // ROS_DEBUG_COND(!valid_point, "invalid point %u", map[i]);

    if ((valid_point && map) &&
      (((i+1 < size) && (map[i+1] == costmap_2d::NO_INFORMATION)) ||
       ((i-1 < size) && (map[i-1] == costmap_2d::NO_INFORMATION)) ||
       ((i+w < size) && (map[i+w] == costmap_2d::NO_INFORMATION)) ||
       ((i-w < size) && (map[i-w] == costmap_2d::NO_INFORMATION))))
    {
      ROS_DEBUG_THROTTLE(30, "found suitable point");
      map_.data[i] = -128;
    } else {
      map_.data[i] = -127;
    }
  }

  // Clean up frontiers detected on separate rows of the map
  for (size_t y = 0, i = h-1; y < w; ++y) {
    ROS_DEBUG_THROTTLE(30, "cleaning cell %lu", i);
    map_.data[i] = -127;
    i += h;
  }

  // Group adjoining map_ pixels
  signed char segment_id = 127;
  std::vector<std::vector<FrontierPoint>> segments;
  for (size_t i = 0; i < size; i++) {
    if (map_.data[i] == -128) {
      ROS_DEBUG_THROTTLE(30, "adjoining on %lu", i);
      std::vector<size_t> neighbors;
      std::vector<FrontierPoint> segment;
      neighbors.push_back(i);

      // claim all neighbors
      while (!neighbors.empty()) {
        ROS_DEBUG_THROTTLE(30, "got %lu neighbors", neighbors.size());
        size_t idx = neighbors.back();
        neighbors.pop_back();
        map_.data[idx] = segment_id;

        tf::Vector3 tot(0,0,0);
        size_t c = 0;
        if (idx+1 < size && map[idx+1] == costmap_2d::NO_INFORMATION) {
          tot += tf::Vector3(1,0,0);
          ++c;
        }
        if (idx-1 < size && map[idx-1] == costmap_2d::NO_INFORMATION) {
          tot += tf::Vector3(-1,0,0);
          ++c;
        }
        if (idx+w < size && map[idx+w] == costmap_2d::NO_INFORMATION) {
          tot += tf::Vector3(0,1,0);
          ++c;
        }
        if (idx-w < size && map[idx-w] == costmap_2d::NO_INFORMATION) {
          tot += tf::Vector3(0,-1,0);
          ++c;
        }

        if(!(c > 0)) {
          ROS_ERROR("assertion failed. corrupted costmap?");
          ROS_DEBUG("c is %lu", c);
          continue;
        }

        segment.push_back(FrontierPoint(idx, tot / c));

        // consider 8 neighborhood
        if (idx-1 < size && map_.data[idx-1] == -128)
          neighbors.push_back(idx-1);
        if (idx+1 < size && map_.data[idx+1] == -128)
          neighbors.push_back(idx+1);
        if (idx-w < size && map_.data[idx-w] == -128)
          neighbors.push_back(idx-w);
        if (idx-w+1 < size && map_.data[idx-w+1] == -128)
          neighbors.push_back(idx-w+1);
        if (idx-w-1 < size && map_.data[idx-w-1] == -128)
          neighbors.push_back(idx-w-1);
        if (idx+w < size && map_.data[idx+w] == -128)
          neighbors.push_back(idx+w);
        if (idx+w+1 < size && map_.data[idx+w+1] == -128)
          neighbors.push_back(idx+w+1);
        if (idx+w-1 < size && map_.data[idx+w-1] == -128)
          neighbors.push_back(idx+w-1);
      }

      segments.push_back(std::move(segment));
      segment_id--;
      if (segment_id < -127)
        break;
    }
  }

  // no longer accessing map
  lock.unlock();

  int num_segments = 127 - segment_id;
  if (num_segments <= 0) {
    ROS_DEBUG("#segments is <0, no frontiers found");
    return;
  }

  for (auto& segment : segments) {
    Frontier frontier;
    size_t segment_size = segment.size();
    float x = 0, y = 0;
    tf::Vector3 d(0,0,0);

    for (const auto& frontier_point : segment) {
      d += frontier_point.d;
      size_t idx = frontier_point.idx;
      x += (idx % w);
      y += (idx / w);
    }
    d = d / segment_size;
    frontier.pose.position.x = map_.info.origin.position.x + map_.info.resolution * (x / segment_size);
    frontier.pose.position.y = map_.info.origin.position.y + map_.info.resolution * (y / segment_size);
    frontier.pose.position.z = 0.0;

    frontier.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(d.y(), d.x()));
    frontier.size = segment_size;

    frontiers_.push_back(std::move(frontier));
  }

}

void ExploreFrontier::getVisualizationMarkers(visualization_msgs::MarkerArray& markers_msg)
{
  ROS_DEBUG("visualising %lu frontiers", frontiers_.size());
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_->getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.type = visualization_msgs::Marker::ARROW;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  m.action = visualization_msgs::Marker::ADD;
  size_t id=0;
  for (auto& frontier : frontiers_) {
    m.id = id;
    m.pose = frontier.pose;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = id;
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
}

}
