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

#ifndef ESTIMATION_INTERNAL_H_
#define ESTIMATION_INTERNAL_H_

#include <combine_grids/merging_pipeline.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching/detail/matchers.hpp>

namespace combine_grids
{
namespace internal
{
static inline cv::Ptr<cv::detail::FeaturesFinder>
chooseFeatureFinder(FeatureType type)
{
  switch (type) {
    case FeatureType::AKAZE:
      return cv::makePtr<cv::detail::AKAZEFeaturesFinder>();
    case FeatureType::ORB:
      return cv::makePtr<cv::detail::OrbFeaturesFinder>();
    case FeatureType::SURF:
      return cv::makePtr<cv::detail::SurfFeaturesFinder>();
  }
}

static inline void writeDebugMatchingInfo(
    const std::vector<cv::Mat>& images,
    const std::vector<cv::detail::ImageFeatures>& image_features,
    const std::vector<cv::detail::MatchesInfo>& pairwise_matches)
{
  for (auto& match_info : pairwise_matches) {
    if (match_info.H.empty() ||
        match_info.src_img_idx >= match_info.dst_img_idx) {
      continue;
    }
    std::cout << match_info.src_img_idx << " " << match_info.dst_img_idx
              << std::endl
              << "features: "
              << image_features[size_t(match_info.src_img_idx)].keypoints.size()
              << " "
              << image_features[size_t(match_info.dst_img_idx)].keypoints.size()
              << std::endl
              << "matches: " << match_info.matches.size() << std::endl
              << "inliers: " << match_info.num_inliers << std::endl
              << "inliers/matches ratio: "
              << match_info.num_inliers / double(match_info.matches.size())
              << std::endl
              << "confidence: " << match_info.confidence << std::endl
              << match_info.H << std::endl;
    cv::Mat img;
    // draw all matches
    cv::drawMatches(images[size_t(match_info.src_img_idx)],
                    image_features[size_t(match_info.src_img_idx)].keypoints,
                    images[size_t(match_info.dst_img_idx)],
                    image_features[size_t(match_info.dst_img_idx)].keypoints,
                    match_info.matches, img);
    cv::imwrite(std::to_string(match_info.src_img_idx) + "_" +
                    std::to_string(match_info.dst_img_idx) + "_matches.png",
                img);
    // draw inliers only
    cv::drawMatches(
        images[size_t(match_info.src_img_idx)],
        image_features[size_t(match_info.src_img_idx)].keypoints,
        images[size_t(match_info.dst_img_idx)],
        image_features[size_t(match_info.dst_img_idx)].keypoints,
        match_info.matches, img, cv::Scalar::all(-1), cv::Scalar::all(-1),
        *reinterpret_cast<const std::vector<char>*>(&match_info.inliers_mask));
    cv::imwrite(std::to_string(match_info.src_img_idx) + "_" +
                    std::to_string(match_info.dst_img_idx) +
                    "_matches_inliers.png",
                img);
  }
}

}  // namespace internal
}  // namespace combine_grids

#endif  // ESTIMATION_INTERNAL_H_
