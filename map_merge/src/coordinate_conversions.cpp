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
 *
 */

/**
 * \file 
 * 
 * Implementation for coordinate_conversions.h
 *
 * \author Bhaskara Marthi
 */

#include <occupancy_grid_utils/coordinate_conversions.h>

namespace occupancy_grid_utils
{

namespace gm=geometry_msgs;
namespace nm=nav_msgs;

gm::Polygon cellPolygon (const nm::MapMetaData& info, const Cell& c)
{
  const float dx[4] = {0.0, 0.0, 1.0, 1.0};
  const float dy[4] = {0.0, 1.0, 1.0, 0.0};
  
  const tf::Transform trans = mapToWorld(info);
  gm::Polygon p;
  p.points.resize(4);
  for (unsigned i=0; i<4; i++) {
    tf::Point pt((c.x+dx[i])*info.resolution, (c.y+dy[i])*info.resolution, 0.0);
    tf::Point transformed = trans*pt;
    p.points[i].x = transformed.x();
    p.points[i].y = transformed.y();
    p.points[i].z = transformed.z();
  }
  return p;
}
  

gm::Polygon gridPolygon (const nm::MapMetaData& info)
{
  const float x[4] = {0.0, 0.0, 1.0, 1.0};
  const float y[4] = {0.0, 1.0, 1.0, 0.0};

  const tf::Transform trans = mapToWorld(info);
  gm::Polygon p;
  p.points.resize(4);

  for (unsigned i=0; i<4; i++) {
    tf::Point pt(x[i]*info.resolution*info.width, y[i]*info.resolution*info.height, 0.0);
    tf::Point transformed=trans*pt;
    p.points[i].x = transformed.x();
    p.points[i].y = transformed.y();
    p.points[i].z = transformed.z();
  }
  return p;
}

void verifyDataSize (const nm::OccupancyGrid& g)
{
  const size_t expected = g.info.height*g.info.width;
  if (expected!=g.data.size())
    throw DataSizeException(expected, g.data.size());
}


} // namespace
