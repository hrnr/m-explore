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

#include <combine_grids/grid_compositor.h>
#include <combine_grids/grid_warper.h>
#include <combine_grids/merging_pipeline.h>
#include <ros/assert.h>
#include <ros/console.h>

#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>

#include "estimation_internal.h"

namespace combine_grids
{
bool MergingPipeline::estimateTransforms(FeatureType feature_type,
                                         double confidence)
{
  std::vector<cv::detail::ImageFeatures> image_features;
  std::vector<cv::detail::MatchesInfo> pairwise_matches;
  std::vector<cv::detail::CameraParams> transforms;
  std::vector<int> good_indices;
  // TODO investigate value translation effect on features
  auto finder = internal::chooseFeatureFinder(feature_type);
  cv::Ptr<cv::detail::FeaturesMatcher> matcher =
      cv::makePtr<cv::detail::AffineBestOf2NearestMatcher>();
  cv::Ptr<cv::detail::Estimator> estimator =
      cv::makePtr<cv::detail::AffineBasedEstimator>();
  cv::Ptr<cv::detail::BundleAdjusterBase> adjuster =
      cv::makePtr<cv::detail::BundleAdjusterAffinePartial>();

  if (images_.empty()) {
    return true;
  }

  /* find features in images */
  ROS_DEBUG("computing features");
  image_features.reserve(images_.size());
  for (const cv::Mat& image : images_) {
    image_features.emplace_back();
    if (!image.empty()) {
#if CV_VERSION_MAJOR >= 4
      cv::detail::computeImageFeatures(finder, image, image_features.back());
#else
      (*finder)(image, image_features.back());
#endif
    }
  }
  finder = {};

  /* find corespondent features */
  ROS_DEBUG("pairwise matching features");
  (*matcher)(image_features, pairwise_matches);
  matcher = {};

#ifndef NDEBUG
  internal::writeDebugMatchingInfo(images_, image_features, pairwise_matches);
#endif

  /* use only matches that has enough confidence. leave out matches that are not
   * connected (small components) */
  good_indices = cv::detail::leaveBiggestComponent(
      image_features, pairwise_matches, static_cast<float>(confidence));

  // no match found. try set first non-empty grid as reference frame. we try to
  // avoid setting empty grid as reference frame, in case some maps never
  // arrive. If all is empty just set null transforms.
  if (good_indices.size() == 1) {
    transforms_.clear();
    transforms_.resize(images_.size());
    for (size_t i = 0; i < images_.size(); ++i) {
      if (!images_[i].empty()) {
        // set identity
        transforms_[i] = cv::Mat::eye(3, 3, CV_64F);
        break;
      }
    }
    return true;
  }

  /* estimate transform */
  ROS_DEBUG("calculating transforms in global reference frame");
  // note: currently used estimator never fails
  if (!(*estimator)(image_features, pairwise_matches, transforms)) {
    return false;
  }

  /* levmarq optimization */
  // openCV just accepts float transforms
  for (auto& transform : transforms) {
    transform.R.convertTo(transform.R, CV_32F);
  }
  ROS_DEBUG("optimizing global transforms");
  adjuster->setConfThresh(confidence);
  if (!(*adjuster)(image_features, pairwise_matches, transforms)) {
    ROS_WARN("Bundle adjusting failed. Could not estimate transforms.");
    return false;
  }

  transforms_.clear();
  transforms_.resize(images_.size());
  size_t i = 0;
  for (auto& j : good_indices) {
    // we want to work with transforms as doubles
    transforms[i].R.convertTo(transforms_[static_cast<size_t>(j)], CV_64F);
    ++i;
  }

  return true;
}

// checks whether given matrix is an identity, i.e. exactly appropriate Mat::eye
static inline bool isIdentity(const cv::Mat& matrix)
{
  if (matrix.empty()) {
    return false;
  }
  cv::MatExpr diff = matrix != cv::Mat::eye(matrix.size(), matrix.type());
  return cv::countNonZero(diff) == 0;
}

nav_msgs::OccupancyGrid::Ptr MergingPipeline::composeGrids()
{
  ROS_ASSERT(images_.size() == transforms_.size());
  ROS_ASSERT(images_.size() == grids_.size());

  if (images_.empty()) {
    return nullptr;
  }

  ROS_DEBUG("warping grids");
  internal::GridWarper warper;
  std::vector<cv::Mat> imgs_warped;
  imgs_warped.reserve(images_.size());
  std::vector<cv::Rect> rois;
  rois.reserve(images_.size());

  for (size_t i = 0; i < images_.size(); ++i) {
    if (!transforms_[i].empty() && !images_[i].empty()) {
      imgs_warped.emplace_back();
      rois.emplace_back(
          warper.warp(images_[i], transforms_[i], imgs_warped.back()));
    }
  }

  if (imgs_warped.empty()) {
    return nullptr;
  }

  ROS_DEBUG("compositing result grid");
  nav_msgs::OccupancyGrid::Ptr result;
  internal::GridCompositor compositor;
  result = compositor.compose(imgs_warped, rois);

  // set correct resolution to output grid. use resolution of identity (works
  // for estimated trasforms), or any resolution (works for know_init_positions)
  // - in that case all resolutions should be the same.
  float any_resolution = 0.0;
  for (size_t i = 0; i < transforms_.size(); ++i) {
    // check if this transform is the reference frame
    if (isIdentity(transforms_[i])) {
      result->info.resolution = grids_[i]->info.resolution;
      break;
    }
    if (grids_[i]) {
      any_resolution = grids_[i]->info.resolution;
    }
  }
  if (result->info.resolution <= 0.f) {
    result->info.resolution = any_resolution;
  }

  // set grid origin to its centre
  result->info.origin.position.x =
      -(result->info.width / 2.0) * double(result->info.resolution);
  result->info.origin.position.y =
      -(result->info.height / 2.0) * double(result->info.resolution);
  result->info.origin.orientation.w = 1.0;

  return result;
}

std::vector<geometry_msgs::Transform> MergingPipeline::getTransforms() const
{
  std::vector<geometry_msgs::Transform> result;
  result.reserve(transforms_.size());

  for (auto& transform : transforms_) {
    if (transform.empty()) {
      result.emplace_back();
      continue;
    }

    ROS_ASSERT(transform.type() == CV_64F);
    geometry_msgs::Transform ros_transform;
    ros_transform.translation.x = transform.at<double>(0, 2);
    ros_transform.translation.y = transform.at<double>(1, 2);
    ros_transform.translation.z = 0.;

    // our rotation is in fact only 2D, thus quaternion can be simplified
    double a = transform.at<double>(0, 0);
    double b = transform.at<double>(1, 0);
    ros_transform.rotation.w = std::sqrt(2. + 2. * a) * 0.5;
    ros_transform.rotation.x = 0.;
    ros_transform.rotation.y = 0.;
    ros_transform.rotation.z = std::copysign(std::sqrt(2. - 2. * a) * 0.5, b);

    result.push_back(ros_transform);
  }

  return result;
}

}  // namespace combine_grids
