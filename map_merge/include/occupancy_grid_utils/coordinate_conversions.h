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
 * General utilities for coordinate conversions
 *
 * \author Bhaskara Marthi
 * \author Jiri Horner
 */


#ifndef OCCUPANCY_GRID_UTILS_COORDINATE_CONVERSIONS_H
#define OCCUPANCY_GRID_UTILS_COORDINATE_CONVERSIONS_H

#include <stdexcept>
#include <cstdlib>
#include <cmath>

#include <ros/assert.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>

namespace occupancy_grid_utils
{

/************************************************************
 * Public interface
 ************************************************************/

typedef uint32_t index_t;
typedef int16_t coord_t;

struct Cell
{
  Cell(const coord_t x=0, const coord_t y=0) : x(x), y(y) {}
  coord_t x;
  coord_t y;

  bool operator== (const Cell& c) const;
  bool operator< (const Cell& c) const;
};


// These values have conventional meanings for occupancy grids
const int8_t UNOCCUPIED=0;
const int8_t OCCUPIED=100;
const int8_t UNKNOWN=-1;


/// \brief Returns the index of a cell.  
/// 
/// \throws std::out_of_range if cell isn't within grid bounds
index_t cellIndex (const nav_msgs::MapMetaData& info, const Cell& c);

/// \brief Returns cell corresponding to index
Cell indexCell (const nav_msgs::MapMetaData& info, index_t ind);

/// \brief Returns cell corresponding to a point.
///
/// The z coordinate of the point is ignored.
Cell pointCell (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p);

/// \brief Returns index of a point. 
/// \throws std::out_of_range if point isn't within grid bounds
///
/// Ignores z coordinate of point
index_t pointIndex (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p);

/// \brief Return center of a cell
geometry_msgs::Point cellCenter (const nav_msgs::MapMetaData& info, const Cell& c);

/// \brief Return polygon corresponding to a cell
geometry_msgs::Polygon cellPolygon (const nav_msgs::MapMetaData& info, const Cell& c);

/// \brief Check if a point is on the grid
bool withinBounds (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p);

/// \brief Check if a cell is on the grid
bool withinBounds (const nav_msgs::MapMetaData& info, const Cell& c);

/// \brief Return polygon corresponding to grid bounds
geometry_msgs::Polygon gridPolygon (const nav_msgs::MapMetaData& info);

/// \brief Verify that data vector has the right size
/// \throws std::logic_error otherwise
void verifyDataSize (const nav_msgs::OccupancyGrid& g);

/************************************************************
 * Implementations of inline functions
 ************************************************************/

inline
index_t cellIndex (const nav_msgs::MapMetaData& info, const Cell& c)
{
  if (!withinBounds(info, c))
    throw std::out_of_range("Cell is off grid according to grid info");
  return c.x + c.y*info.width;
}

inline
Cell indexCell (const nav_msgs::MapMetaData& info, const index_t ind)
{
  const div_t result = std::div((int) ind, (int) info.width);
  return Cell(result.rem, result.quot);
}


inline
tf::Transform mapToWorld (const nav_msgs::MapMetaData& info)
{
  tf::Transform world_to_map;
  tf::poseMsgToTF (info.origin, world_to_map);
  return world_to_map;
}

inline
tf::Transform worldToMap (const nav_msgs::MapMetaData& info)
{
  return mapToWorld(info).inverse();
}

inline
Cell pointCell (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
{
  tf::Point pt;
  tf::pointMsgToTF(p, pt);
  tf::Point p2 = worldToMap(info)*pt;
  return Cell(std::floor(p2.x()/info.resolution), std::floor(p2.y()/info.resolution));
}

inline 
index_t pointIndex (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
{
  return cellIndex(info, pointCell(info, p));
}

inline
geometry_msgs::Point cellCenter (const nav_msgs::MapMetaData& info, const Cell& c)
{
  tf::Point pt((c.x+0.5)*info.resolution, (c.y+0.5)*info.resolution, 0.0);
  geometry_msgs::Point p;
  tf::pointTFToMsg(mapToWorld(info)*pt, p);
  return p;
}


inline
bool Cell::operator== (const Cell& c) const
{
  return ((this->x == c.x) && (this->y == c.y));
}

inline
bool Cell::operator< (const Cell& c) const
{
  return ((this->x < c.x) || ((this->x == c.x) && (this->y < c.y)));
}

inline
std::ostream& operator<< (std::ostream& str, const Cell& c)
{
  str << "(" << c.x << ", " << c.y << ")";
  return str;
}

inline
bool withinBounds (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
{
  return withinBounds(info, pointCell(info, p));
}

inline
bool withinBounds (const nav_msgs::MapMetaData& info, const Cell& c)
{
  return (c.x >= 0) && (c.y >= 0) && (c.x < (coord_t) info.width) && (c.y < (coord_t) info.height);
}

} // namespace occupancy_grid_utils

#endif // include guard
