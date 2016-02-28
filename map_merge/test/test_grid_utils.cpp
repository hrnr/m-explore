/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/exceptions.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/combine_grids.h>
#include <occupancy_grid_utils/geometry.h>
#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/assign.hpp>
#include <cmath>

namespace gm=geometry_msgs;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;
namespace sm=sensor_msgs;

using std::ostream;
using gu::Cell;
using std::abs;
using boost::bind;
using std::vector;
using std::set;
using boost::assign::operator+=;
using std::operator<<;

const double PI = 3.14159265;

const double TOL=1e-6;


typedef vector<Cell> Path;
typedef set<Cell> Cells;
typedef boost::shared_ptr<nm::OccupancyGrid> GridPtr;
typedef boost::shared_ptr<nm::OccupancyGrid const> GridConstPtr;

bool samePath (const Path& p1, const Path& p2)
{
  if (p1.size()!=p2.size())
    return false;
  for (unsigned i=0; i<p1.size(); i++)
    if (!(p1[i]==p2[i]))
      return false;
  return true;
}

void setOccupied (nm::OccupancyGrid* g, const unsigned x, const unsigned y)
{
  g->data[gu::cellIndex(g->info, Cell(x, y))] = gu::OCCUPIED;;
}

void setOccupied (GridPtr g, const unsigned x, const unsigned y)
{
  setOccupied(g.get(), x, y);
}

template <class T>
bool equalSets (const set<T>& s1, const set<T>& s2)
{
  BOOST_FOREACH (const T& x, s1) 
  {
    if (s2.find(x)==s2.end())
      return false;
  }
  BOOST_FOREACH (const T& x, s2)
  {
    if (s1.find(x)==s1.end())
      return false;
  }
  return true;
}

bool approxEqual (const double x, const double y)
{
  return abs(x-y)<TOL;
}


namespace geometry_msgs
{

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


template <class T>
ostream& operator<< (ostream& str, const vector<T>& s)
{
  str << "(";
  BOOST_FOREACH (const T& x, s) 
    str << x << " ";
  str << ")";
  return str;
}

template <class T>
ostream& operator<< (ostream& str, const set<T>& s)
{
  str << "{";
  std::ostream_iterator<T> iter(str, ", ");
  copy(s.begin(), s.end(), iter);
  str << "}";
  return str;
}


ostream& operator<< (ostream& str, const gm::Point& p)
{
  str << "(" << p.x << ", " << p.y << ")";
  return str;
}

gm::Point makePoint (const double x, const double y)
{
  gm::Point p;
  p.x = x;
  p.y = y;
  return p;
}

gm::Point32 makePoint32 (const double x, const double y)
{
  gm::Point32 p;
  p.x = x;
  p.y = y;
  return p;
}

double norm (const double x, const double y)
{
  return sqrt(x*x + y*y);
}

struct CloseTo
{
  CloseTo(double res) : res(res) {}
  bool operator() (const gm::Point& p, const gm::Point& p2) 
  {
    const double dx=p2.x-p.x;
    const double dy=p2.y-p.y;
    return norm(dx, dy) < res*sqrt(2);
  }
  const double res;
};

double angle (const double x1, const double y1, const double x2, const double y2)
{
  const double ip = x1*x2 + y1*y2;
  return acos(ip/(norm(x1, y1)*norm(x2, y2)));
}

gm::Pose makePose (const double x, const double y, const double theta)
{
  gm::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = tf::createQuaternionMsgFromYaw(theta);
  return p;
}

float dist (const gu::DistanceField& d, int x, int y)
{
  return d[gu::Cell(x,y)];
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
  EXPECT_THROW (gu::pointIndex(info, makePoint(1, -1)), gu::CellOutOfBoundsException);
  EXPECT_THROW (gu::cellIndex(info, Cell(100, 100)), gu::CellOutOfBoundsException);

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


  
TEST(GridUtils, DistanceField)
{
  nm::OccupancyGrid g;
  g.info.height=5;
  g.info.width=6;
  g.data.resize(30);
  g.info.resolution=0.5;
  setOccupied(&g, 3, 1);
  setOccupied(&g, 0, 4);
  setOccupied(&g, 4, 3);
  
  gu::DistanceField d = gu::distanceField(g);
  EXPECT_EQ(2, dist(d, 0, 0));
  EXPECT_EQ(1.5, dist(d, 1, 0));
  EXPECT_EQ(1, dist(d, 2, 0));
  EXPECT_EQ(.5, dist(d, 3, 0));
  EXPECT_EQ(1, dist(d, 4, 0));
  EXPECT_EQ(0, dist(d, 3, 1));
  EXPECT_EQ(0, dist(d, 0, 4));
  EXPECT_EQ(1.5, dist(d, 1, 2));
  EXPECT_EQ(1, dist(d, 1, 3));
  EXPECT_EQ(1, dist(d, 2, 2));
  EXPECT_EQ(1, dist(d, 2, 3));
  EXPECT_EQ(.5, dist(d, 4, 4));

}
gm::Point lastPoint (const nm::MapMetaData& info, const gu::RayTraceIterRange& r)
{
  boost::optional<gm::Point> last_point;
  BOOST_FOREACH (const Cell& c, r) {
    last_point = gu::cellCenter(info, c);
  }
  ROS_ASSERT(last_point);
  return *last_point;
}

TEST(GridUtils, RayTrace)
{
  const double res=0.2;
  nm::MapMetaData info;
  info.resolution = res;
  info.origin = makePose(3, 1, PI/2);
  info.height = 100;
  info.width = 50;
  
  gu::RayTraceIterRange r = gu::rayTrace(info, makePoint(1, 2), makePoint(2, 5));
  gm::Point last_point = lastPoint(info, r);

  EXPECT_PRED2 (CloseTo(res), makePoint(2, 5), last_point);
  EXPECT_PRED2 (CloseTo(res), makePoint(1, 2), gu::cellCenter(info, *(r.first)));
  EXPECT_EQ (16, std::distance(r.first, r.second));

  // Try again with x and y flipped
  nm::MapMetaData info2=info;
  info2.origin = makePose(1, 3, PI/-2);
  info.height = 50;
  info.width = 100;

  gu::RayTraceIterRange r2 = gu::rayTrace(info2, makePoint(2, 1), makePoint(5, 2));
  gm::Point lp2 = lastPoint(info2, r2);
  
  EXPECT_PRED2 (CloseTo(res), makePoint(5, 2), lp2);
  EXPECT_PRED2 (CloseTo(res), makePoint(2, 1), gu::cellCenter(info2, *(r2.first)));
  EXPECT_EQ (16, std::distance(r2.first, r2.second));


  // Out of bounds points
  EXPECT_THROW(gu::rayTrace(info, makePoint(1, 2), makePoint(4, 2)), gu::PointOutOfBoundsException);
  gu::RayTraceIterRange r3 = gu::rayTrace(info, makePoint(1, 2), makePoint(4, 2), true);
  gm::Point lp3 = lastPoint(info, r3);
  EXPECT_PRED2 (CloseTo(res), makePoint(3, 2), lp3);
  EXPECT_PRED2 (CloseTo(res), makePoint(1, 2), gu::cellCenter(info, *(r.first)));
  EXPECT_EQ (10, std::distance(r3.first, r3.second));
  EXPECT_THROW(gu::rayTrace(info, makePoint(4, 2), makePoint(1, 2), true), gu::PointOutOfBoundsException);
}


TEST(GridUtils, GridOverlay)
{
  // Grid at origin with resolution 0.1
  const double res=10.0;
  nm::MapMetaData info;
  info.resolution = res;
  info.origin = makePose(0, 0, 0);
  info.height = 4;
  info.width = 4;
  
  // Create clouds
  typedef vector<gm::Point32> PointVec;
  typedef boost::shared_ptr<gu::LocalizedCloud const> CloudConstPtr;
  typedef boost::shared_ptr<gu::LocalizedCloud> CloudPtr;

  CloudPtr c1(new gu::LocalizedCloud());
  c1->sensor_pose = makePose(.001, .001, 0.0);
  c1->header.frame_id = "foo";
  c1->cloud.points = PointVec(1);
  CloudPtr c2(new gu::LocalizedCloud());
  c2->sensor_pose = makePose(25.0, 5.0, PI/2);
  c2->header.frame_id = "foo";
  c2->cloud.points = PointVec(4);
  c1->cloud.points[0] = makePoint32(20, 20);
  c2->cloud.points[0] = makePoint32(20, 0);
  c2->cloud.points[1] = makePoint32(45, 35);
  c2->cloud.points[2] = makePoint32(0, 1000);
  c2->cloud.points[3] = makePoint32(-1, -1);

  // Overlay two clouds
  nm::OccupancyGrid fake_grid;
  fake_grid.info = info;
  gu::OverlayClouds o1 = gu::createCloudOverlay(fake_grid, "foo", .5, 200);
  gu::addCloud(&o1, c1);
  gu::addCloud(&o1, c2);
  GridConstPtr grid = gu::getGrid(o1);
  EXPECT_EQ (gu::UNKNOWN, grid->data[0]);
  EXPECT_EQ (gu::UNKNOWN, grid->data[1]);
  EXPECT_EQ (gu::UNOCCUPIED, grid->data[2]);
  EXPECT_EQ (gu::UNKNOWN, grid->data[3]);
  EXPECT_EQ (gu::UNKNOWN, grid->data[4]);
  EXPECT_EQ (gu::UNOCCUPIED, grid->data[5]);
  EXPECT_EQ (gu::UNKNOWN, grid->data[6]);
  EXPECT_EQ (gu::UNKNOWN, grid->data[9]);
  EXPECT_EQ (gu::OCCUPIED, grid->data[10]);

  // Vary parameters
  gu::OverlayClouds o2 = gu::createCloudOverlay(fake_grid, "foo", .1, 2000);
  gu::addCloud(&o2, c2);
  gu::addCloud(&o2, c1);
  GridConstPtr grid2 = gu::getGrid(o2);
  EXPECT_EQ (gu::UNOCCUPIED, grid2->data[0]);
  EXPECT_EQ (gu::UNKNOWN, grid2->data[1]);
  EXPECT_EQ (gu::OCCUPIED, grid2->data[2]);
  EXPECT_EQ (gu::UNKNOWN, grid2->data[3]);
  EXPECT_EQ (gu::UNKNOWN, grid2->data[4]);
  EXPECT_EQ (gu::UNOCCUPIED, grid2->data[5]);
  EXPECT_EQ (gu::UNKNOWN, grid2->data[6]);
  EXPECT_EQ (gu::UNKNOWN, grid2->data[9]);
  EXPECT_EQ (gu::OCCUPIED, grid2->data[10]);
  EXPECT_EQ (gu::UNKNOWN, grid2->data[15]);

  // Add another cloud
  gu::addCloud(&o1, c1);
  GridConstPtr grid3 = gu::getGrid(o1);
  EXPECT_EQ (gu::UNOCCUPIED, grid3->data[0]);
  EXPECT_EQ (gu::UNKNOWN, grid3->data[1]);
  EXPECT_EQ (gu::UNOCCUPIED, grid3->data[2]);

  // Subtract the cloud we just added and check that it's the same as grid 1
  gu::removeCloud(&o1, c1);
  GridConstPtr grid4 = gu::getGrid(o1);
  for (unsigned i=0; i<16; i++)
    EXPECT_EQ (grid->data[i], grid4->data[i]);

  // Check header
  EXPECT_EQ(grid4->header.frame_id, "foo");
}


TEST(GridUtils, SimulateScan)
{
  nm::MapMetaData info;
  info.resolution = 0.2;
  info.origin = makePose(1, 1, 0);
  info.height = 20;
  info.width = 20;

  GridPtr grid(new nm::OccupancyGrid());
  grid->info = info;
  grid->data.resize(400);
  setOccupied(grid, 1, 6);
  setOccupied(grid, 3, 4);
  setOccupied(grid, 3, 7);
  setOccupied(grid, 4, 4);
  setOccupied(grid, 4, 6);
  setOccupied(grid, 5, 4);
  setOccupied(grid, 7, 8);

  sm::LaserScan scan_info;
  scan_info.angle_min = -PI/2;
  scan_info.angle_max = PI/2;
  scan_info.angle_increment = PI/4;
  scan_info.range_max = 1.0;

  const gm::Pose sensor_pose = makePose(1.7, 1.9, PI/2);

  sm::LaserScan::ConstPtr scan =
    gu::simulateRangeScan(*grid, sensor_pose, scan_info);

  ASSERT_EQ(scan->ranges.size(), 5);
  EXPECT_FLOAT_EQ(scan->ranges[0], 0.2);
  EXPECT_TRUE(scan->ranges[1] > 1.0);
  EXPECT_FLOAT_EQ(scan->ranges[2], 0.6);
  EXPECT_FLOAT_EQ(scan->ranges[3], 0.4*sqrt(2));
  EXPECT_TRUE(scan->ranges[4] > 1.0);
}


TEST(GridUtils, ShortestPath)
{
  nm::MapMetaData info;
  info.resolution = 0.1;
  info.origin = makePose(0, 0, 0);
  info.height = 5;
  info.width = 5;

  GridPtr grid(new nm::OccupancyGrid());
  grid->info = info;
  grid->data.resize(25);
  setOccupied(grid, 1, 3);
  setOccupied(grid, 2, 2);
  setOccupied(grid, 2, 1);
  setOccupied(grid, 3, 2);
  setOccupied(grid, 3, 3);
  setOccupied(grid, 3, 4);
  setOccupied(grid, 4, 2);

  Cell c1(0, 1);
  Cell c2(1, 4);
  Cell c3(2, 3);
  Cell c4(3, 1);
  Cell c5(4, 2);
  Cell c6(4, 3);

  const gu::ResultPtr res=singleSourceShortestPaths(*grid, c1);
  
  EXPECT_PRED2(approxEqual, 0.1*(2+sqrt(2)), *distanceTo(res, c2));
  EXPECT_PRED2(approxEqual, 0.1*(2*sqrt(2)), *distanceTo(res, c3));
  EXPECT_PRED2(approxEqual, 0.1*(1+2*sqrt(2)), *distanceTo(res, c4));
  EXPECT_TRUE(!distanceTo(res, c5));
  EXPECT_TRUE(!distanceTo(res, c6));

  Path p12, p13, p14;
  p12 += Cell(0, 1), Cell(0, 2), Cell(0, 3), Cell(1, 4);
  p13 += Cell(0, 1), Cell(1, 2), Cell(2, 3);
  p14 += Cell(0, 1), Cell(1, 1), Cell(2, 0), Cell(3, 1);
  
  EXPECT_PRED2(samePath, p12, *extractPath(res, c2));
  EXPECT_PRED2(samePath, p13, *extractPath(res, c3));
  EXPECT_PRED2(samePath, p14, *extractPath(res, c4));
  EXPECT_TRUE(!extractPath(res, c5));
  EXPECT_TRUE(!extractPath(res, c6));

  // Conversion to and from ros messages
  const gu::NavigationFunction msg = gu::shortestPathResultToMessage(res);
  const gu::ResultPtr res2 = gu::shortestPathResultFromMessage(msg);
  
  EXPECT_PRED2(approxEqual, 0.1*(2+sqrt(2)), *distanceTo(res2, c2));
  EXPECT_PRED2(approxEqual, 0.1*(2*sqrt(2)), *distanceTo(res2, c3));
  EXPECT_PRED2(approxEqual, 0.1*(1+2*sqrt(2)), *distanceTo(res2, c4));
  EXPECT_TRUE(!distanceTo(res2, c5));
  EXPECT_TRUE(!distanceTo(res2, c6));

  EXPECT_PRED2(samePath, p12, *extractPath(res2, c2));
  EXPECT_PRED2(samePath, p13, *extractPath(res2, c3));
  EXPECT_PRED2(samePath, p14, *extractPath(res2, c4));
  EXPECT_TRUE(!extractPath(res2, c5));
  EXPECT_TRUE(!extractPath(res2, c6));


  // Termination conditions
  const gu::TerminationCondition t1(0.4, false);
  const gu::ResultPtr res3 = gu::singleSourceShortestPaths(*grid, c4, t1);
  EXPECT_PRED2(approxEqual, 0.1*(1+2*sqrt(2)), *distanceTo(res3, c1));
  EXPECT_TRUE(!distanceTo(res3, c2));
  EXPECT_TRUE(!distanceTo(res3, c3));

  gu::Cells goals;
  goals += c3, c6;
  const gu::TerminationCondition t2(goals);
  const gu::ResultPtr res4 = gu::singleSourceShortestPaths(*grid, c4, t2);
  EXPECT_PRED2(approxEqual, 0.1*(1+3*sqrt(2)), *distanceTo(res4, c3));
  EXPECT_PRED2(approxEqual, 0.1*(1+2*sqrt(2)), *distanceTo(res4, c1));
  EXPECT_TRUE(!distanceTo(res4, c6));

  const gu::TerminationCondition t3(goals, 0.4, false);
  const gu::ResultPtr res5 = gu::singleSourceShortestPaths(*grid, c4, t3);
  EXPECT_PRED2(approxEqual, 0.1*(1+2*sqrt(2)), *distanceTo(res5, c1));
  EXPECT_TRUE(!distanceTo(res5, c2));
  EXPECT_TRUE(!distanceTo(res5, c3));


  // A*
  boost::optional<gu::AStarResult> res6 = shortestPathAStar(*grid, c1, c4);
  EXPECT_FLOAT_EQ (res6->second, 0.5);
  EXPECT_EQ(6u, res6->first.size());

  boost::optional<gu::AStarResult> res7 = shortestPathAStar(*grid, c1, c1);
  EXPECT_FLOAT_EQ (res7->second, 0);
  EXPECT_EQ(1u, res7->first.size());

  boost::optional<gu::AStarResult> res8 = shortestPathAStar(*grid, c1, c5);
  EXPECT_TRUE (!res8);

  boost::optional<gu::AStarResult> res9 = shortestPathAStar(*grid, c1, c6);
  EXPECT_TRUE (!res9);
  
}

int val (const nm::OccupancyGrid& g, const gu::coord_t x, const gu::coord_t y)
{
  const gu::Cell c(x, y);
  ROS_ASSERT (gu::withinBounds(g.info, c));
  return g.data[cellIndex(g.info, c)];
}

void setVal (nm::OccupancyGrid* g, const gu::coord_t x, const gu::coord_t y, const int v)
{
  const gu::Cell c(x, y);
  ROS_ASSERT (gu::withinBounds(g->info, c));
  g->data[cellIndex(g->info, c)]=v;
}


TEST(GridUtils, InflateObstacles)
{
  GridPtr g(new nm::OccupancyGrid());
  g->info.origin = makePose(0, 0, 0);
  g->info.resolution = 0.5;
  g->info.width = 10;
  g->info.height = 11;
  g->data.resize(g->info.width*g->info.height);
  fill(g->data.begin(), g->data.end(), -1);
  
  setVal(g.get(), 2, 2, 0);
  for (unsigned x=7; x<10; x++) {
    for (unsigned y=7; y<11; y++) {
      setVal(g.get(), x, y, 0);
    }
  }
    
  setVal(g.get(), 3, 6, 50);
  setVal(g.get(), 3, 7, 100);
  setVal(g.get(), 3, 4, 0);

  GridPtr g2 = gu::inflateObstacles(*g, 0.7, false);

  EXPECT_EQ (100, val(*g2, 3, 6));
  EXPECT_EQ (50, val(*g2, 3, 4));
  EXPECT_EQ (100, val(*g2, 3, 5));
  EXPECT_EQ (-1, val(*g2, 1, 3));
  EXPECT_EQ (-1, val(*g2, 7, 7));
  EXPECT_EQ (0, val(*g2, 9, 9));
  

}

bool pred (const Cell& c)
{
  if (c.x==2 || c.y==2)
    return false;
  if (((c.x==1) || (c.x==3)) &&
      ((c.y==1) || (c.y==3)))
    return false;
  return true;
}

bool not_pred (const Cell& c)
{
  return !pred(c);
}

TEST(GridUtils, Geometry)
{
  nm::MapMetaData info;
  info.origin = makePose(.1,.1,0);
  info.resolution = .10;
  info.height = 5;
  info.width = 5;

  gm::Polygon poly;
  poly.points.push_back(makePoint32(.41, .61));
  poly.points.push_back(makePoint32(.43, .45));
  poly.points.push_back(makePoint32(.51, .41));
  poly.points.push_back(makePoint32(.8, .46));
  poly.points.push_back(makePoint32(.78, .63));
  
  const Cells cells = gu::cellsInConvexPolygon(info, poly);
  Cells expected;
  expected.insert(Cell(4,4));
  expected.insert(Cell(4,3));
  expected.insert(Cell(3,4));
  expected.insert(Cell(3,3));
  EXPECT_EQ(expected, cells);
  

  Cells cells2 = gu::tileCells(info, 0.2, pred);
  Cells expected2;
  expected2.insert(Cell(0,0));
  expected2.insert(Cell(0,3));
  expected2.insert(Cell(3,0));
  expected2.insert(Cell(3,4));
  EXPECT_EQ(expected2, cells2);
  
  Cells cells3 = gu::tileCells(info, 0.2, not_pred);
  Cells expected3;
  expected3.insert(Cell(0,2));
  expected3.insert(Cell(2,0));
  expected3.insert(Cell(2,3));
  expected3.insert(Cell(4,2));
  EXPECT_EQ(expected3, cells3);
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
