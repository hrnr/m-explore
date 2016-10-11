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

#ifndef FEATURES_MATCHER_H_
#define FEATURES_MATCHER_H_

#include <opencv2/stitching/detail/matchers.hpp>

namespace cv_backport
{
/** @brief Features matcher similar to cv::detail::BestOf2NearestMatcher which
finds two best matches for each feature and leaves the best one only if the
ratio between descriptor distances is greater than the threshold match_conf.

Unlike cv::detail::BestOf2NearestMatcher this matcher uses affine
transformation (affine trasformation estimate will be placed in matches_info).

@sa cv::detail::FeaturesMatcher cv::detail::BestOf2NearestMatcher
 */
class CV_EXPORTS AffineBestOf2NearestMatcher
    : public cv::detail::BestOf2NearestMatcher
{
public:
  /** @brief Constructs a "best of 2 nearest" matcher that expects affine
  trasformation
  between images

  @param full_affine whether to use full affine transformation with 6 degress of
  freedom or reduced
  transformation with 4 degrees of freedom using only rotation, translation and
  uniform scaling
  @param try_use_gpu Should try to use GPU or not
  @param match_conf Match distances ration threshold
  @param num_matches_thresh1 Minimum number of matches required for the 2D
  affine transform
  estimation used in the inliers classification step

  @sa cv::estimateAffine2D cv::estimateAffinePartial2D
   */
  AffineBestOf2NearestMatcher(bool full_affine = false,
                              bool try_use_gpu = false, float match_conf = 0.3f,
                              int num_matches_thresh1 = 6)
    : BestOf2NearestMatcher(try_use_gpu, match_conf, num_matches_thresh1,
                            num_matches_thresh1)
    , full_affine_(full_affine)
  {
  }

protected:
  void match(const cv::detail::ImageFeatures &features1,
             const cv::detail::ImageFeatures &features2,
             cv::detail::MatchesInfo &matches_info);

  bool full_affine_;
};

/** @brief AKAZE features finder.

@sa detail::FeaturesFinder, AKAZE
*/
class CV_EXPORTS AKAZEFeaturesFinder : public cv::detail::FeaturesFinder
{
public:
    AKAZEFeaturesFinder(int descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB,
                        int descriptor_size = 0,
                        int descriptor_channels = 3,
                        float threshold = 0.001f,
                        int nOctaves = 4,
                        int nOctaveLayers = 4,
                        int diffusivity = cv::KAZE::DIFF_PM_G2);

private:
    void find(cv::InputArray image, cv::detail::ImageFeatures &features);

    cv::Ptr<cv::AKAZE> akaze;
};

}  // namespace cv_backport

#endif  // FEATURES_MATCHER_H_
