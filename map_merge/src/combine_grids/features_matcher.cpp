/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2000-2008, Intel Corporation, all rights reserved.
 *  Copyright (c) 2009, Willow Garage Inc., all rights reserved.
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

#include <combine_grids/features_matcher.h>

#include <combine_grids/estimate_rigid_transform.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/console.h>
#include <ros/assert.h>

namespace combine_grids
{
namespace internal
{
/* modified implementation of match from BestOf2NearestMatcher */
void AffineBestOf2NearestMatcher::match(
    const cv::detail::ImageFeatures &features1,
    const cv::detail::ImageFeatures &features2,
    cv::detail::MatchesInfo &matches_info)
{
  (*impl_)(features1, features2, matches_info);

  ROS_DEBUG("AffineMatcher: have %lu matches", matches_info.matches.size());

  // Check if it makes sense to find homography
  if (matches_info.matches.size() < static_cast<size_t>(num_matches_thresh1_))
    return;

  // Construct point-point correspondences for homography estimation
  // Points are centered s.t. image center is (0,0). This is similar to other
  // matchers a shows better results in practice (numerical stability?).
  cv::Mat src_points(1, static_cast<int>(matches_info.matches.size()),
                     CV_32FC2);
  cv::Mat dst_points(1, static_cast<int>(matches_info.matches.size()),
                     CV_32FC2);
  for (size_t i = 0; i < matches_info.matches.size(); ++i) {
    const cv::DMatch &m = matches_info.matches[i];

    cv::Point2f p = features1.keypoints[static_cast<size_t>(m.queryIdx)].pt;
    p.x -= features1.img_size.width * 0.5f;
    p.y -= features1.img_size.height * 0.5f;
    src_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;

    p = features2.keypoints[static_cast<size_t>(m.trainIdx)].pt;
    p.x -= features2.img_size.width * 0.5f;
    p.y -= features2.img_size.height * 0.5f;
    dst_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
  }

  // Find pair-wise motion
  // this is my special modified version of estimateRigidTransform
  matches_info.H = estimateRigidTransform(src_points, dst_points,
                                          matches_info.inliers_mask, false);
  ROS_DEBUG_STREAM("estimate:\n" << matches_info.H);

  if (matches_info.H.empty()) {
    // could not find trasformation
    matches_info.confidence = 0;
    matches_info.num_inliers = 0;
    return;
  }

  // extend H to represent linear tranformation in homogeneous coordinates
  matches_info.H.push_back(cv::Mat::zeros(1, 3, CV_64F));
  matches_info.H.at<double>(2, 2) = 1;

  /* TODO: should we handle determinant ~ 0 (can it happen due to agressive
   * scaling?) */

  // Find number of inliers
  matches_info.num_inliers = 0;
  for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
    if (matches_info.inliers_mask[i])
      matches_info.num_inliers++;

  ROS_DEBUG("num_inliers %d", matches_info.num_inliers);

  // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic
  // Image Stitching using Invariant Features"
  matches_info.confidence =
      matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());

  /* we don't want any cuttof for merging maps. TODO: allow to set this in
   * constructor. */
  // Set zero confidence to remove matches between too close images, as they
  // don't provide additional information anyway. The threshold was set
  // experimentally.
  // matches_info.confidence =
  // matches_info.confidence > 3. ? 0. : matches_info.confidence;

  /* Unlike other matchers it makes no sense to rerun estimation on liers only.
   * estimateRigidTransform already did this for us. So we are done here. */
}

}  // namespace internal
}  // namespace combine_grids
