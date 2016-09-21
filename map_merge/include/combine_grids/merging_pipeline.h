/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
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

#ifndef MERGING_PIPELINE_H_
#define MERGING_PIPELINE_H_

#include <vector>

#include <geometry_msgs/Transform.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/utility.hpp>

namespace combine_grids
{
/**
 * @brief Pipeline for merging overlapping occupancy grids
 * @details Pipeline works on internally stored grids.
 */
class MergingPipeline
{
public:
  template <typename InputIt>
  void feed(InputIt grids_begin, InputIt grids_end);
  bool estimateTransform(double confidence = 1.0);
  nav_msgs::OccupancyGrid::Ptr composeGrids();

  std::vector<geometry_msgs::Transform> getTransforms();

private:
  std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids_;
  std::vector<cv::Mat> images_;
  std::vector<cv::Mat> transforms_;
};

template <typename InputIt>
void MergingPipeline::feed(InputIt grids_begin, InputIt grids_end)
{
  static_assert(std::is_assignable<nav_msgs::OccupancyGrid::ConstPtr&,
                                   decltype(*grids_begin)>::value,
                "grids_begin must point to nav_msgs::OccupancyGrid::ConstPtr "
                "data");

  // we can't reserve anything, because we want to support just InputIt and
  // their guarantee validity for only single-pass algos
  images_.clear();
  grids_.clear();
  for (InputIt it = grids_begin; it != grids_end; ++it) {
    if (*it && !(*it)->data.empty()) {
      grids_.push_back(*it);
      /* convert to opencv images. it creates only a view for opencv and does
       * not copy or own actual data. */
      images_.emplace_back(it->info.height, it->info.width, CV_8UC1,
                           it->data.data());
    } else {
      grids_.emplace_back();
      images_.emplace_back();
    }
  }
}

}  // namespace combine_grids

#endif  // MERGING_PIPELINE_H_
