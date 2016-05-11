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
#include <cmath>

#include <ros/console.h>
#include <ros/assert.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/utility.hpp>

namespace combine_grids
{
/**
 * @brief Estimates transformation between grids when initial positions of grids
 *in the world are unknown
 * @details Uses sophisticated algorithm based on image processing to estimate
 *transformation between grids. Transformations are returned through container
 *in form of `geometry_msgs::Pose`. Output poses can be interpreted as starting
 *point of grid in the world.
 *
 *Output range (transforms) must have at least the size of input range (grids).
 *Typically you want them to have the same size.
 *
 *Note: grids are not modified in any way. Esp. their origins are unchanged.
 *
 * @param grids_begin,grids_end the range of input grids
 * @param transforms_begin the beginning of the destination range for estimated
 *transforms
 * @param confidence minimal confidence according to probabilistic model for
 *pairwise transform to be considered for final estimation. Default value 1.0 is
 *suitable for most grids, increase this value for more confident estimations.
 *Number of estimated transformations may decrease with increasing confidence.
 *Good range for tuning is [1.0, 2.0].
 * @return number of sucessfuly estimated transforms. If transformation
 *could not be established for given grid, empty Pose will be set in trasforms.
 */
template <typename ForwardIt, typename OutputIt>
size_t estimateGridTransform(ForwardIt grids_begin, ForwardIt grids_end,
                             OutputIt transforms_begin,
                             double confidence = 1.0);

namespace internal
{
/**
 * @brief Estimates tranformation using opencv stitching pipeline
 * @details For given images computes transformations, such that all images
 *transformed makes one image.
 *
 *All transforms will form one component (there will always exist transforms
 *between each other). This will be the biggest possible component. Solo matches
 *and smaller components are left out
 *
 * @param images images usable by opencv stitching pipeline
 * @param transforms estimated transforms
 * @param confidence confidence treshold for probabilistic model based on number
 *of inliers
 * @return number of successfully estimated transforms with enough confidence.
 */
size_t opencvEstimateTransform(const std::vector<cv::Mat>& images,
                               std::vector<cv::Mat>& transforms,
                               double confidence);

}  // namespace internal
}  // namespace combine_grids

/* implementation */

namespace combine_grids
{
template <typename ForwardIt, typename OutputIt>
size_t estimateGridTransform(ForwardIt grids_begin, ForwardIt grids_end,
                             OutputIt transforms_begin, double confidence)
{
  static_assert(std::is_assignable<nav_msgs::OccupancyGrid&,
                                   decltype(*grids_begin)>::value,
                "grids_begin must point to nav_msgs::OccupancyGrid data");

  static_assert(std::is_assignable<geometry_msgs::Pose&,
                                   decltype(*transforms_begin)>::value,
                "transforms_begin must point to geometry_msgs::Pose data");

  std::vector<cv::Mat> images;
  std::vector<cv::Mat> transforms;

  ROS_DEBUG("estimating transformations between grids");

  /* convert to opencv images. it creates only a view for opencv and does not
   * copy actual data. */
  ROS_DEBUG("generating opencv stub images");
  images.reserve(std::distance(grids_begin, grids_end));
  for (ForwardIt it = grids_begin; it != grids_end; ++it) {
    nav_msgs::OccupancyGrid& it_ref = *it;  // support reference_wrapper
    // we need to skip empty grids, does not play well in opencv
    if (it_ref.data.empty()) {
      continue;
    }
    // Mat does no support constness in constructor
    images.emplace_back(it_ref.info.height, it_ref.info.width, CV_8UC1,
                        it_ref.data.data());
  }

  size_t num_estimates =
      internal::opencvEstimateTransform(images, transforms, confidence);

  auto transform_it = transforms_begin;
  auto grid_it = grids_begin;
  for (auto& transform : transforms) {
    nav_msgs::OccupancyGrid& grid = *grid_it;  // support reference_wrapper
    geometry_msgs::Pose& output_transform =
        *transform_it;  // support reference_wrapper
    if (transform.empty()) {
      // empty means transformation could not be found
      output_transform = geometry_msgs::Pose();
    } else {
      double translation_x = transform.at<double>(0, 2);
      double translation_y = transform.at<double>(1, 2);
      double rotation_rad =
          std::atan2(transform.at<double>(0, 1), transform.at<double>(1, 1));

      /* transformation was computed in pixels, we need to convert to meters
       * (map resolution). also we want to compesate this trasformation so we
       * need to use oposite values. */
      output_transform.position.x = -1. * translation_x * grid.info.resolution;
      output_transform.position.y = -1. * translation_y * grid.info.resolution;
      output_transform.position.z = 0;
      output_transform.orientation =
          tf::createQuaternionMsgFromYaw(-1. * rotation_rad);
    }
    ++grid_it;
    ++transform_it;
  }

  return num_estimates;
}

}  // namespace combine_grids

#endif  // ESTIMATE_TRANSFORM_H_
