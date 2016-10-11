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

#include "matchers.hpp"

#include <opencv2/core/utility.hpp>
#include "../calib3d/calib3d.hpp"

using namespace cv;
using namespace cv::detail;

namespace cv_backport
{
void AffineBestOf2NearestMatcher::match(const ImageFeatures &features1, const ImageFeatures &features2,
                                        MatchesInfo &matches_info)
{
    (*impl_)(features1, features2, matches_info);

    // Check if it makes sense to find transform
    if (matches_info.matches.size() < static_cast<size_t>(num_matches_thresh1_))
        return;

    // Construct point-point correspondences for transform estimation
    Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
    Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
    for (size_t i = 0; i < matches_info.matches.size(); ++i)
    {
        const cv::DMatch &m = matches_info.matches[i];
        src_points.at<Point2f>(0, static_cast<int>(i)) = features1.keypoints[m.queryIdx].pt;
        dst_points.at<Point2f>(0, static_cast<int>(i)) = features2.keypoints[m.trainIdx].pt;
    }

    // Find pair-wise motion
    if (full_affine_)
        matches_info.H = estimateAffine2D(src_points, dst_points, matches_info.inliers_mask);
    else
        matches_info.H = estimateAffinePartial2D(src_points, dst_points, matches_info.inliers_mask);

    if (matches_info.H.empty()) {
        // could not find transformation
        matches_info.confidence = 0;
        matches_info.num_inliers = 0;
        return;
    }

    // Find number of inliers
    matches_info.num_inliers = 0;
    for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
        if (matches_info.inliers_mask[i])
            matches_info.num_inliers++;

    // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic
    // Image Stitching using Invariant Features"
    matches_info.confidence =
        matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());

    /* should we remove matches between too close images? */
    // matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;

    // extend H to represent linear tranformation in homogeneous coordinates
    matches_info.H.push_back(Mat::zeros(1, 3, CV_64F));
    matches_info.H.at<double>(2, 2) = 1;
}

AKAZEFeaturesFinder::AKAZEFeaturesFinder(int descriptor_type,
                                         int descriptor_size,
                                         int descriptor_channels,
                                         float threshold,
                                         int nOctaves,
                                         int nOctaveLayers,
                                         int diffusivity)
{
    akaze = AKAZE::create(descriptor_type, descriptor_size, descriptor_channels,
                          threshold, nOctaves, nOctaveLayers, diffusivity);
}

void AKAZEFeaturesFinder::find(InputArray image, detail::ImageFeatures &features)
{
    CV_Assert((image.type() == CV_8UC3) || (image.type() == CV_8UC1));
    Mat descriptors;
    UMat uimage = image.getUMat();
    akaze->detectAndCompute(uimage, UMat(), features.keypoints, descriptors);
    features.descriptors = descriptors.getUMat(ACCESS_READ);
}

}  // namespace cv_backport
