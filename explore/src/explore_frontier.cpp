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

/*
 *  Created on: Apr 6, 2009
 *      Author: duhadway
 */

#include <explore/explore_frontier.h>


using namespace visualization_msgs;
using namespace costmap_2d;

namespace explore {

ExploreFrontier::ExploreFrontier() :
  map_(),
  lastMarkerCount_(0),
  planner_(NULL),
  frontiers_()
{
}

ExploreFrontier::~ExploreFrontier()
{

}

bool ExploreFrontier::getFrontiers(Costmap2DROS& costmap, std::vector<geometry_msgs::Pose>& frontiers)
{
  findFrontiers(costmap);
  if (frontiers_.size() == 0)
    return false;

  frontiers.clear();
  for (uint i=0; i < frontiers_.size(); i++) {
    geometry_msgs::Pose frontier;
    frontiers.push_back(frontiers_[i].pose);
  }

  return (frontiers.size() > 0);
}

float ExploreFrontier::getFrontierCost(const Frontier& frontier) {
  ROS_DEBUG("cost of frontier: %f, at position: (%.2f, %.2f, %.2f)", planner_->getPointPotential(frontier.pose.position),
      frontier.pose.position.x, frontier.pose.position.y, tf::getYaw(frontier.pose.orientation));
  if (planner_ != NULL)
    return planner_->getPointPotential(frontier.pose.position); // / 20000.0;
  else
    return 1.0;
}

// TODO: what is this doing exactly?
double ExploreFrontier::getOrientationChange(const Frontier& frontier, const tf::Stamped<tf::Pose>& robot_pose){
  double robot_yaw = tf::getYaw(robot_pose.getRotation());
  double robot_atan2 = atan2(robot_pose.getOrigin().y() + sin(robot_yaw), robot_pose.getOrigin().x() + cos(robot_yaw));
  double frontier_atan2 = atan2(frontier.pose.position.x, frontier.pose.position.y);
  double orientation_change = robot_atan2 - frontier_atan2;
//  ROS_DEBUG("Orientation change: %.3f degrees, (%.3f radians)", orientation_change * (180.0 / M_PI), orientation_change);
  return orientation_change;
}

float ExploreFrontier::getFrontierGain(const Frontier& frontier, double map_resolution) {
  return frontier.size * map_resolution;
}

bool ExploreFrontier::getExplorationGoals(Costmap2DROS& costmap, tf::Stamped<tf::Pose> robot_pose, navfn::NavfnROS* planner, std::vector<geometry_msgs::Pose>& goals, double potential_scale, double orientation_scale, double gain_scale)
{
  findFrontiers(costmap);
  if (frontiers_.size() == 0)
    return false;

  geometry_msgs::Point start;
  start.x = robot_pose.getOrigin().x();
  start.y = robot_pose.getOrigin().y();
  start.z = robot_pose.getOrigin().z();

  planner->computePotential(start);

  planner_ = planner;
  costmapResolution_ = costmap.getResolution();

  //we'll make sure that we set goals for the frontier at least the circumscribed
  //radius away from unknown space
  float step = -1.0 * costmapResolution_;
  int c = ceil(costmap.getCircumscribedRadius() / costmapResolution_);
  WeightedFrontier goal;
  std::vector<WeightedFrontier> weightedFrontiers;
  weightedFrontiers.reserve(frontiers_.size() * c);
  for (uint i=0; i < frontiers_.size(); i++) {
    Frontier& frontier = frontiers_[i];
    WeightedFrontier weightedFrontier;
    weightedFrontier.frontier = frontier;

    tf::Point p(frontier.pose.position.x, frontier.pose.position.y, frontier.pose.position.z);
    tf::Quaternion bt;
    tf::quaternionMsgToTF(frontier.pose.orientation, bt);
    tf::Vector3 v(cos(bt.getAngle()), sin(bt.getAngle()), 0.0);

    for (int j=0; j <= c; j++) {
      tf::Vector3 check_point = p + (v * (step * j));
      weightedFrontier.frontier.pose.position.x = check_point.x();
      weightedFrontier.frontier.pose.position.y = check_point.y();
      weightedFrontier.frontier.pose.position.z = check_point.z();

      weightedFrontier.cost = potential_scale * getFrontierCost(weightedFrontier.frontier) + orientation_scale * getOrientationChange(weightedFrontier.frontier, robot_pose) - gain_scale * getFrontierGain(weightedFrontier.frontier, costmapResolution_);
//      weightedFrontier.cost = getFrontierCost(weightedFrontier.frontier) - getFrontierGain(weightedFrontier.frontier, costmapResolution_);
//      ROS_DEBUG("cost: %f (%f * %f + %f * %f - %f * %f)",
//          weightedFrontier.cost,
//          potential_scale,
//          getFrontierCost(weightedFrontier.frontier),
//          orientation_scale,
//          getOrientationChange(weightedFrontier.frontier, robot_pose),
//          gain_scale,
//          getFrontierGain(weightedFrontier.frontier, costmapResolution_) );
      weightedFrontiers.push_back(weightedFrontier);
    }
  }

  goals.clear();
  goals.reserve(weightedFrontiers.size());
  std::sort(weightedFrontiers.begin(), weightedFrontiers.end());
  for (uint i = 0; i < weightedFrontiers.size(); i++) {
    goals.push_back(weightedFrontiers[i].frontier.pose);
  }
  return (goals.size() > 0);
}

void ExploreFrontier::findFrontiers(Costmap2DROS& costmap_) {
  frontiers_.clear();

  Costmap2D costmap;
  costmap_.getCostmapCopy(costmap);

  int idx;
  int w = costmap.getSizeInCellsX();
  int h = costmap.getSizeInCellsY();
  int size = (w * h);

  map_.info.width = w;
  map_.info.height = h;
  map_.data.resize((size_t)size);
  map_.info.resolution = costmap.getResolution();
  map_.info.origin.position.x = costmap.getOriginX();
  map_.info.origin.position.y = costmap.getOriginY();

  // Find all frontiers (open cells next to unknown cells).
  const unsigned char* map = costmap.getCharMap();
  for (idx = 0; idx < size; idx++) {
//    //get the world point for the index
//    unsigned int mx, my;
//    costmap.indexToCells(idx, mx, my);
//    geometry_msgs::Point p;
//    costmap.mapToWorld(mx, my, p.x, p.y);
//
    //check if the point has valid potential and is next to unknown space
//    bool valid_point = planner_->validPointPotential(p);
    bool valid_point = (map[idx] < LETHAL_OBSTACLE);

    if ((valid_point && map) &&
        (((idx+1 < size) && (map[idx+1] == NO_INFORMATION)) ||
         ((idx-1 >= 0) && (map[idx-1] == NO_INFORMATION)) ||
         ((idx+w < size) && (map[idx+w] == NO_INFORMATION)) ||
         ((idx-w >= 0) && (map[idx-w] == NO_INFORMATION))))
    {
      map_.data[idx] = -128;
    } else {
      map_.data[idx] = -127;
    }
  }

  // Clean up frontiers detected on separate rows of the map
  idx = map_.info.height - 1;
  for (unsigned int y=0; y < map_.info.width; y++) {
    map_.data[idx] = -127;
    idx += map_.info.height;
  }

  // Group adjoining map_ pixels
  int segment_id = 127;
  std::vector< std::vector<FrontierPoint> > segments;
  for (int i = 0; i < size; i++) {
    if (map_.data[i] == -128) {
      std::vector<int> neighbors;
      std::vector<FrontierPoint> segment;
      neighbors.push_back(i);

      // claim all neighbors
      while (neighbors.size() > 0) {
        int idx = neighbors.back();
        neighbors.pop_back();
        map_.data[idx] = segment_id;

        btVector3 tot(0,0,0);
        int c = 0;
        if ((idx+1 < size) && (map[idx+1] == NO_INFORMATION)) {
          tot += btVector3(1,0,0);
          c++;
        }
        if ((idx-1 >= 0) && (map[idx-1] == NO_INFORMATION)) {
          tot += btVector3(-1,0,0);
          c++;
        }
        if ((idx+w < size) && (map[idx+w] == NO_INFORMATION)) {
          tot += btVector3(0,1,0);
          c++;
        }
        if ((idx-w >= 0) && (map[idx-w] == NO_INFORMATION)) {
          tot += btVector3(0,-1,0);
          c++;
        }
        assert(c > 0);
        segment.push_back(FrontierPoint(idx, tot / c));

        // consider 8 neighborhood
        if (((idx-1)>0) && (map_.data[idx-1] == -128))
          neighbors.push_back(idx-1);
        if (((idx+1)<size) && (map_.data[idx+1] == -128))
          neighbors.push_back(idx+1);
        if (((idx-map_.info.width)>0) && (map_.data[idx-map_.info.width] == -128))
          neighbors.push_back(idx-map_.info.width);
        if (((idx-map_.info.width+1)>0) && (map_.data[idx-map_.info.width+1] == -128))
          neighbors.push_back(idx-map_.info.width+1);
        if (((idx-map_.info.width-1)>0) && (map_.data[idx-map_.info.width-1] == -128))
          neighbors.push_back(idx-map_.info.width-1);
        if (((idx+(int)map_.info.width)<size) && (map_.data[idx+map_.info.width] == -128))
          neighbors.push_back(idx+map_.info.width);
        if (((idx+(int)map_.info.width+1)<size) && (map_.data[idx+map_.info.width+1] == -128))
          neighbors.push_back(idx+map_.info.width+1);
        if (((idx+(int)map_.info.width-1)<size) && (map_.data[idx+map_.info.width-1] == -128))
          neighbors.push_back(idx+map_.info.width-1);
      }

      segments.push_back(segment);
      segment_id--;
      if (segment_id < -127)
        break;
    }
  }

  int num_segments = 127 - segment_id;
  if (num_segments <= 0)
    return;

  for (unsigned int i=0; i < segments.size(); i++) {
    Frontier frontier;
    std::vector<FrontierPoint>& segment = segments[i];
    uint size = segment.size();

    //we want to make sure that the frontier is big enough for the robot to fit through
    if (size * costmap.getResolution() < costmap.getInscribedRadius())
      continue;

    float x = 0, y = 0;
    btVector3 d(0,0,0);

    for (uint j=0; j<size; j++) {
      d += segment[j].d;
      int idx = segment[j].idx;
      x += (idx % map_.info.width);
      y += (idx / map_.info.width);
    }
    d = d / size;
    frontier.pose.position.x = map_.info.origin.position.x + map_.info.resolution * (x / size);
    frontier.pose.position.y = map_.info.origin.position.y + map_.info.resolution * (y / size);
    frontier.pose.position.z = 0.0;

    frontier.pose.orientation = tf::createQuaternionMsgFromYaw(btAtan2(d.y(), d.x()));
    frontier.size = size;

    frontiers_.push_back(frontier);
  }

}

void ExploreFrontier::getVisualizationMarkers(std::vector<Marker>& markers)
{
  Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "frontiers";
  m.type = Marker::ARROW;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  m.lifetime = ros::Duration(0);

  m.action = Marker::ADD;
  uint id = 0;
  for (uint i=0; i<frontiers_.size(); i++) {
    Frontier frontier = frontiers_[i];
    m.id = id;
    m.pose = frontier.pose;
    markers.push_back(Marker(m));
    id++;
  }

  m.action = Marker::DELETE;
  for (; id < lastMarkerCount_; id++) {
    m.id = id;
    markers.push_back(Marker(m));
  }

  lastMarkerCount_ = markers.size();
}

}
