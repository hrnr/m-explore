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

#include <vector>

#include <opencv2/core/utility.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/stitching/detail/autocalib.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/stitching/warpers.hpp>

#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>

namespace combine_grids
{
template <typename ForwardIt>
bool estimateGridTransform(ForwardIt first, ForwardIt last)
{
  std::size_t length = std::distance(first, last);
  std::vector<cv::Mat> images;
  std::vector<cv::detail::ImageFeatures> image_features;
  std::vector<cv::detail::MatchesInfo> pairwise_matches;
  std::vector<cv::detail::CameraParams> transforms;
  cv::Ptr<cv::detail::FeaturesFinder> finder;
  cv::Ptr<cv::detail::FeaturesMatcher> matcher;
  cv::Ptr<cv::detail::HomographyBasedEstimator> estimator;

  if (length < 2)
    return false;

  /* convert to opencv images. it creates only a view for opencv and does not
   * copy actual data. */
  images.reserve(length);
  for (ForwardIt it = first; it != last; ++it) {
    nav_msgs::OccupancyGrid& it_ref = *it;  // support reference_wrapper
    // Mat does no support constness in constructor
    images.emplace_back(it_ref.info.height, it_ref.info.width, CV_8SC1,
                        it_ref.data.data());
  }

  /* find features in images */
  finder = cv::makePtr<cv::detail::OrbFeaturesFinder>();
  image_features.reserve(images.size());
  for (cv::Mat& image : images) {
    image_features.emplace_back();
    (*finder)(image, image_features.back());
  }
  finder->collectGarbage();

  /* find corespondent features */
  // matches only some (5) images, scales better than full pairwise matcher
  matcher = cv::makePtr<cv::detail::BestOf2NearestRangeMatcher>();
  (*matcher)(image_features, pairwise_matches);
  matcher->collectGarbage();

  /* estimate transform */
  transforms.reserve(images.size());
  if (!(*estimator)(image_features, pairwise_matches, transforms)) {
    return false;
  }

  for (cv::detail::CameraParams& transform : transforms)
    ROS_DEBUG("TRANSFORM ppx: %f, ppy %f\n", transform.ppx, transform.ppy);

  return true;
}
}
