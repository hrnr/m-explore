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
 *   * Neither the name of the author nor the names of its
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

#ifndef ESTIMATE_TRANSFORM_H_
#define ESTIMATE_TRANSFORM_H_

#include <vector>
#include <type_traits>

#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/utility.hpp>

namespace combine_grids
{
template <typename ForwardIt>
bool estimateGridTransform(ForwardIt first, ForwardIt last);

namespace internal
{
/**
 * @brief Estimates tranformation using opencv stitching pipeline
 * @details For given images computes transformation, such that all images
 *transformed makes
 *
 * @param images images usable by opencv stitching pipeline
 * @return [nothing yet]
 */
bool opencvEstimateTransform(const std::vector<cv::Mat>& images);

}  // namespace internal
}  // namespace combine_grids

namespace combine_grids
{
template <typename ForwardIt>
bool estimateGridTransform(ForwardIt first, ForwardIt last)
{
  static_assert(
      std::is_assignable<nav_msgs::OccupancyGrid&, decltype(*first)>::value,
      "iterators must point to nav_msgs::OccupancyGrid data");

  std::vector<cv::Mat> images;

  ROS_DEBUG("estimating transformations between grids");

  /* convert to opencv images. it creates only a view for opencv and does not
   * copy actual data. */
  ROS_DEBUG("generating opencv stub images");
  images.reserve(std::distance(first, last));
  for (ForwardIt it = first; it != last; ++it) {
    nav_msgs::OccupancyGrid& it_ref = *it;  // support reference_wrapper
    // we need to skip empty grids, does not play well in opencv
    if (it_ref.data.empty()) {
      continue;
    }
    // Mat does no support constness in constructor
    images.emplace_back(it_ref.info.height, it_ref.info.width, CV_8UC1,
                        it_ref.data.data());
  }

  return internal::opencvEstimateTransform(images);
}

}  // namespace combine_grids

#endif  // ESTIMATE_TRANSFORM_H_
