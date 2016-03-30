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
 *
 */


/**
 * \file
 *
 * Combining overlapping grids
 *
 * \author Bhaskara Marthi
 * \author Jiri Horner
 */

#ifndef OCCUPANCY_GRID_UTILS_COMBINE_GRIDS_H
#define OCCUPANCY_GRID_UTILS_COMBINE_GRIDS_H

#include <set>
#include <iterator>
#include <limits>
#include <cmath>

#include <ros/assert.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>

#include <boost/foreach.hpp>

#include <occupancy_grid_utils/coordinate_conversions.h>

/*
 * internal macro
 * @brief Computes specified boundary for polygon
 * @details [long description]
 *
 * @param polygon pologon to work on
 * @param minmax Must be 'min' or 'max' will compute minimum or maximum
 * respectively
 * @param xy Must be 'x' or 'y'. Coordinate.
 */
#define POLYGON_BOUND(polygon, minmax, xy) \
 static_cast<double>(std::minmax(polygon.points[0].xy, \
  std::minmax(polygon.points[1].xy, \
  std::minmax(polygon.points[2].xy, polygon.points[3].xy))))

namespace occupancy_grid_utils
{

/// \brief Combines a set of grids
///
/// The resulting grid's origin will be a translated version of grid 1's origin, with resolution \a resolution.
///
/// All empty grids will be skipped.
///
/// The policy for combination is that for each cell, we look at each grid cell that
/// intersects it and consider their values.  Map these to integers, where everything above
/// OCCUPIED (100) goes to -1.  Then take the max.  If there are no intersecting cells, value is -1.
///
/// Assumes all grids lie on the xy plane, and will fail in weird ways if that's not true
template<typename ForwardIt>
void combineGrids(ForwardIt first, ForwardIt last, double resolution, nav_msgs::OccupancyGrid& result);

/// Version of combineGrids that uses the resolution of the first grid.
template<typename ForwardIt>
void combineGrids (ForwardIt first, ForwardIt last, nav_msgs::OccupancyGrid& result);

/* for backward compatibility */

/// Version of combineGrids that uses shared_ptr
///
/// you should use combineGrids with raw pointers instead, which saves reference counting
/// \deprecated
nav_msgs::OccupancyGrid::Ptr combineGrids(const std::vector<nav_msgs::OccupancyGrid::ConstPtr>& grids, double resolution);

/// Version of combineGrids that uses shared_ptr
///
/// you should use combineGrids with raw pointers instead, which saves reference counting
/// \deprecated
nav_msgs::OccupancyGrid::Ptr combineGrids(const std::vector<nav_msgs::OccupancyGrid::ConstPtr>& grids);

/* util functions */
template<typename ForwardIt>
nav_msgs::MapMetaData getCombinedGridInfo (ForwardIt first, ForwardIt last, const double resolution);
std::set<Cell> intersectingCells (const nav_msgs::MapMetaData& info, const nav_msgs::MapMetaData& info2, const Cell& cell2);
geometry_msgs::Pose transformPose (const tf::Pose trans, const geometry_msgs::Pose p);

/* templates */

// Get the dimensions of a combined grid
template<typename ForwardIt>
nav_msgs::MapMetaData getCombinedGridInfo (ForwardIt first, ForwardIt last, const double resolution)
{
  nav_msgs::MapMetaData info;
  info.resolution = static_cast<float>(resolution);
  tf::Pose trans;
  double min_x, max_x, min_y, max_y;
  min_x = min_y = std::numeric_limits<double>::infinity();
  max_x = max_y = -std::numeric_limits<double>::infinity();

  if(first == last) {
    return info;
  }

  const nav_msgs::OccupancyGrid& first_ref = *first; // needed to support reference_wrapper
  tf::poseMsgToTF(first_ref.info.origin, trans);


  for (ForwardIt grid_it = first; grid_it != last; ++grid_it) {
    const nav_msgs::OccupancyGrid& grid = *grid_it; // needed to support reference_wrapper
    if (grid.data.empty()) {
      // skip empty grids as nothing will merged for them
      continue;
    }

    /* get bounding polygon for grid in world frame */
    nav_msgs::MapMetaData grid_info = grid.info;
    grid_info.origin = transformPose(trans.inverse(), grid.info.origin);
    geometry_msgs::Polygon polygon = gridPolygon(grid_info);

    min_x = std::min(min_x, POLYGON_BOUND(polygon, min, x));
    min_y = std::min(min_y, POLYGON_BOUND(polygon, min, y));
    max_x = std::max(max_x, POLYGON_BOUND(polygon, max, x));
    max_y = std::max(max_y, POLYGON_BOUND(polygon, max, y));
  }

  // all grids were empty
  if (!std::isfinite(min_x)) {
    return info;
  }

  const double dx = max_x - min_x;
  const double dy = max_y - min_y;

  // min and max might be egual if we have only 1 grid
  ROS_ASSERT((max_x >= min_x) && (max_y >= min_y));

  geometry_msgs::Pose pose_in_grid_frame;
  pose_in_grid_frame.position.x = min_x;
  pose_in_grid_frame.position.y = min_y;
  pose_in_grid_frame.orientation.w = 1.0;
  info.origin = transformPose(trans, pose_in_grid_frame);
  info.height = std::ceil(dy/info.resolution);
  info.width = std::ceil(dx/info.resolution);

  return info;
}


// Main function
template<typename ForwardIt>
void combineGrids (ForwardIt first, ForwardIt last, const double resolution, nav_msgs::OccupancyGrid& combined_grid)
{
  nav_msgs::MapMetaData grid_info;

  if(first == last)
    return;

  // compute size of merged grid
  grid_info = getCombinedGridInfo(first, last, resolution);
  // if all grid are empty (zero size) or size could not be computed
  if (grid_info.width == 0 || grid_info.height == 0) {
    return;
  }

  combined_grid.info = grid_info;
  combined_grid.data.resize(combined_grid.info.width*combined_grid.info.height);
  fill(combined_grid.data.begin(), combined_grid.data.end(), -1);
  ROS_DEBUG_NAMED ("combine_grids", "Combining %zu grids", std::distance(first, last));

  for (ForwardIt grid_it = first; grid_it != last; ++grid_it) {
    const nav_msgs::OccupancyGrid& grid = *grid_it; // needed to support reference_wrapper
    for (coord_t x=0; x<(int)grid.info.width; x++) {
      for (coord_t y=0; y<(int)grid.info.height; y++) {
        const Cell cell(x, y);
        const signed char value=grid.data[cellIndex(grid.info, cell)];

        // Only proceed if the value is not unknown
        if ((value>=0) && (value<=100)) {
          BOOST_FOREACH (const Cell& intersecting_cell,
                         intersectingCells(combined_grid.info, grid.info, cell)) {
            const index_t ind = cellIndex(combined_grid.info, intersecting_cell);
            combined_grid.data[ind] = std::max(combined_grid.data[ind], value);
          }
        }
      }
    }
  }

  ROS_DEBUG_NAMED ("combine_grids", "Done combining grids");
}

template<typename ForwardIt>
void combineGrids (ForwardIt first, ForwardIt last, nav_msgs::OccupancyGrid& result)
{
  if(first == last)
    return;
  const nav_msgs::OccupancyGrid& first_ref = *first; // needed to support reference_wrapper
  combineGrids(first, last, first_ref.info.resolution, result);
}

} // namespace occupancy_grid_utils

// internal macro
#undef POLYGON_BOUND

#endif // include guard
