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

#include <combine_grids/estimate_transform.h>

#include <cmath>

#include <opencv2/core/utility.hpp>

#include <ros/console.h>
#include <ros/assert.h>

#include <combine_grids/features_matcher.h>
#include <combine_grids/transform_estimator.h>

namespace combine_grids
{
namespace internal
{
size_t opencvEstimateTransform(const std::vector<cv::Mat>& images,
                               std::vector<cv::Mat>& final_transforms,
                               double confidence)
{
  std::vector<cv::detail::ImageFeatures> image_features;
  std::vector<cv::detail::MatchesInfo> pairwise_matches;
  std::vector<cv::detail::CameraParams> transforms;
  std::vector<int> matched_indices;
  cv::Ptr<cv::detail::FeaturesFinder> finder =
      cv::makePtr<cv::detail::OrbFeaturesFinder>();
  cv::Ptr<cv::detail::FeaturesMatcher> matcher =
      cv::makePtr<internal::AffineBestOf2NearestMatcher>();
  cv::Ptr<cv::detail::Estimator> estimator =
      cv::makePtr<internal::AffineBasedEstimator>();

  if (images.size() < 2) {
    return 0;
  }

  /* find features in images */
  ROS_DEBUG("computing features");
  image_features.reserve(images.size());
  for (const cv::Mat& image : images) {
    image_features.emplace_back();
    (*finder)(image, image_features.back());
  }
  finder->collectGarbage();

  /* find corespondent features */
  // matches only some (5) images, scales better than full pairwise matcher
  ROS_DEBUG("pairwise matching features");
  (*matcher)(image_features, pairwise_matches);
  matcher->collectGarbage();

  /* use only matches that has enough confidence. leave out matches that are not
   * connected (small components) */
  matched_indices = cv::detail::leaveBiggestComponent(
      image_features, pairwise_matches, static_cast<float>(confidence));

  /* estimate transform */
  ROS_DEBUG("estimating final transform");
  // note: currently used estimator never fails
  if (!(*estimator)(image_features, pairwise_matches, transforms)) {
    return 0;
  }

  ROS_ASSERT(matched_indices.size() == transforms.size());

  final_transforms.clear();
  final_transforms.resize(images.size());
  size_t i = 0;
  for (auto& transform : transforms) {
    if (!transform.R.empty()) {
      ROS_DEBUG("TRANSFORM: trans x: %f, trans y %f, rot: %f\n",
                transform.R.at<double>(0, 2), transform.R.at<double>(1, 2),
                std::atan2(transform.R.at<double>(0, 1),
                           transform.R.at<double>(1, 1)));
    }

    final_transforms[i] = transform.R;
    ++i;
  }

  ROS_ASSERT(final_transforms.size() == images.size());

  return transforms.size();
}
}  // namespace internal
}  // namespace combine_grids
