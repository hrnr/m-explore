/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2015-2016, Jiri Horner
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Jiri Horner nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <cmath>

#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/assign.hpp>

#include <tf/transform_datatypes.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <occupancy_grid_utils/combine_grids.h>
#include <occupancy_grid_utils/coordinate_conversions.h>

namespace gm=geometry_msgs;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;

using std::ostream;
using gu::Cell;
using std::abs;
using boost::bind;
using std::vector;
using std::set;
using boost::assign::operator+=;
using std::operator<<;

const double PI = 3.14159265;
const double TOL = 1e-6;


typedef vector<Cell> Path;
typedef set<Cell> Cells;
typedef boost::shared_ptr<nm::OccupancyGrid> GridPtr;
typedef boost::shared_ptr<nm::OccupancyGrid const> GridConstPtr;

/* helpers for tests */

bool approxEqual (const double x, const double y);
int val (const nm::OccupancyGrid& g, const gu::coord_t x, const gu::coord_t y);
gm::Point makePoint (const double x, const double y);
gm::Pose makePose (const double x, const double y, const double theta);
void setVal (nm::OccupancyGrid* g, const gu::coord_t x, const gu::coord_t y, const int v);

namespace geometry_msgs {
bool operator== (const Polygon& p1, const Polygon& p2);

bool operator== (const Polygon& p1, const Polygon& p2)
{
  if (p1.points.size() != p2.points.size())
    return false;
  for (unsigned i=0; i<p1.points.size(); i++) 
    if (!approxEqual(p1.points[i].x, p2.points[i].x) ||
        !approxEqual(p1.points[i].y, p2.points[i].y) || 
        !approxEqual(p1.points[i].z, p2.points[i].z))
      return false;
  return true;  
}

} // namespace geometry_msgs

bool approxEqual (const double x, const double y)
{
  return abs(x-y)<TOL;
}

int val (const nm::OccupancyGrid& g, const gu::coord_t x, const gu::coord_t y)
{
  const gu::Cell c(x, y);
  ROS_ASSERT (gu::withinBounds(g.info, c));
  return g.data[cellIndex(g.info, c)];
}

gm::Point makePoint (const double x, const double y)
{
  gm::Point p;
  p.x = x;
  p.y = y;
  return p;
}

gm::Pose makePose (const double x, const double y, const double theta)
{
  gm::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = tf::createQuaternionMsgFromYaw(theta);
  return p;
}

void setVal (nm::OccupancyGrid* g, const gu::coord_t x, const gu::coord_t y, const int v)
{
  const gu::Cell c(x, y);
  ROS_ASSERT (gu::withinBounds(g->info, c));
  g->data[cellIndex(g->info, c)]=v;
}

TEST(GridUtils, CoordinateConversions)
{
  // Set up info for a map at (2, -1) that is rotated 45 degrees, with resolution 0.1
  nm::MapMetaData info;
  info.resolution = 0.1;
  info.origin = makePose(2, -1, PI/4);
  info.height = 50;
  info.width = 80;

  // Check conversions
  EXPECT_EQ (804u, gu::cellIndex(info, Cell(4, 10)));
  EXPECT_EQ (Cell(8, 1), gu::pointCell(info, makePoint(2.5, -0.35)));
  EXPECT_EQ (88u, gu::pointIndex(info, makePoint(2.5, -0.35)));
  EXPECT_EQ (Cell(-8, 7), gu::pointCell(info, makePoint(1, -1)));
  EXPECT_THROW (gu::pointIndex(info, makePoint(1, -1)), std::out_of_range);
  EXPECT_THROW (gu::cellIndex(info, Cell(100, 100)), std::out_of_range);

  // Cell polygon 
  const double side=sqrt(2)/2;
  gm::Polygon expected;
  expected.points.resize(4);
  expected.points[0].x = 2 + .1*side;
  expected.points[0].y = -1 + .3*side;
  expected.points[1].x = 2;
  expected.points[1].y = -1 + .4*side;
  expected.points[2].x = 2 + .1*side;
  expected.points[2].y = -1 + .5*side;
  expected.points[3].x = 2 + .2*side;
  expected.points[3].y = -1 + .4*side;
  EXPECT_EQ(expected, cellPolygon(info, Cell(2, 1)));
}

TEST(GridUtils, CombineGrids)
{
  GridPtr g1(new nm::OccupancyGrid());
  GridPtr g2(new nm::OccupancyGrid());
  
  g1->info.origin = makePose(2, 1, 0);
  g2->info.origin = makePose(3, 0, PI/4);
  g1->info.resolution = 0.5;
  g2->info.resolution = 0.5;
  g1->info.height = 4;
  g1->info.width = 6;
  g2->info.height = 4;
  g2->info.width = 4;
  g1->data.resize(24);
  g2->data.resize(16);

  fill(g1->data.begin(), g1->data.end(), -1);
  fill(g2->data.begin(), g2->data.end(), -1);

  setVal(g2.get(), 1, 0, 0);
  setVal(g2.get(), 0, 3, 50);
  setVal(g2.get(), 2, 0, 42);
  setVal(g2.get(), 0, 2, 11);
  setVal(g2.get(), 3, 2, 0);
  setVal(g2.get(), 3, 0, 110);
  
  setVal(g1.get(), 5, 3, 100);
  setVal(g1.get(), 3, 0, 24);
  setVal(g1.get(), 0, 0, 66);
  setVal(g1.get(), 4, 0, 90);


  
  vector<GridConstPtr> grids;
  grids += g1, g2;
  GridPtr combined = gu::combineGrids(grids, g1->info.resolution/2.0);
  
  EXPECT_PRED2(approxEqual, 3-sqrt(2), combined->info.origin.position.x);
  EXPECT_PRED2(approxEqual, 0, combined->info.origin.position.y);
  EXPECT_EQ(0.25, combined->info.resolution);
  EXPECT_EQ(round((2+sqrt(2))/0.25), combined->info.width);
  EXPECT_EQ(12u, combined->info.height);

  // Note there are rounding issues that mean that some of the values below
  // could theoretically go either way
  EXPECT_EQ(-1, val(*combined, 11, 2));
  EXPECT_EQ(-1, val(*combined, 2, 0));
  EXPECT_EQ(-1, val(*combined, 5, 0));
  EXPECT_EQ(0, val(*combined, 7, 2));
  EXPECT_EQ(66, val(*combined, 1, 6));
  EXPECT_EQ(100, val(*combined, 11, 11));
  EXPECT_EQ(-1, val(*combined, 2, 11));
  EXPECT_EQ(11, val(*combined, 3, 2));
  EXPECT_EQ(42, val(*combined, 7, 4));
  EXPECT_EQ(66, val(*combined, 2, 4));
  EXPECT_EQ(0, val(*combined, 6, 8));
  EXPECT_EQ(90, val(*combined, 10, 5));
}

int main (int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
