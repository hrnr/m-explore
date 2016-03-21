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

#include <combine_grids/transform_estimator.h>

#include <opencv2/core/utility.hpp>

#include <ros/console.h>
#include <ros/assert.h>

namespace combine_grids
{
namespace internal
{
/**
 * @brief Old-style functor calculating final tranformation related to reference
 *frame
 *
 */
class CalcAffineTransform
{
public:
  CalcAffineTransform(
      int _num_images,
      const std::vector<cv::detail::MatchesInfo> &_pairwise_matches,
      std::vector<cv::detail::CameraParams> &_cameras)
    : num_images(_num_images)
    , pairwise_matches(&_pairwise_matches[0])
    , cameras(&_cameras[0])
  {
  }

  void operator()(const cv::detail::GraphEdge &edge)
  {
    int pair_idx = edge.from * num_images + edge.to;

    ROS_DEBUG("computing transform, from %d, to %d, pair_idx %d", edge.from,
              edge.to, pair_idx);

    ROS_DEBUG_STREAM("cameras[edge.from]:\n" << cameras[edge.from].R);
    ROS_DEBUG_STREAM("H:\n" << pairwise_matches[pair_idx].H);

    cameras[edge.to].R = cameras[edge.from].R * pairwise_matches[pair_idx].H;
  }

private:
  int num_images;
  const cv::detail::MatchesInfo *pairwise_matches;
  cv::detail::CameraParams *cameras;
};

bool AffineBasedEstimator::estimate(
    const std::vector<cv::detail::ImageFeatures> &features,
    const std::vector<cv::detail::MatchesInfo> &pairwise_matches,
    std::vector<cv::detail::CameraParams> &cameras)
{
  ROS_DEBUG("AffineBasedEstimator: have %lu pairwise matches.",
            pairwise_matches.size());

  cameras.resize(features.size());
  const int num_images = static_cast<int>(features.size());

  // find if we have any affine estimates
  bool have_any_transform = false;
  for(auto& match : pairwise_matches) {
    have_any_transform |= !match.H.empty();
  }
  // we don't need to do anything
  if(!have_any_transform) {
    return true;
  }

  // find maximum spaning tree on pairwise matches
  cv::detail::Graph span_tree;
  std::vector<int> span_tree_centers;
  // function from motion estimators, uses number of inliers as weights
  findMaxSpanningTree(num_images, pairwise_matches, span_tree,
                      span_tree_centers);

  ROS_DEBUG("constructed spanning tree with %d vertices",
            span_tree.numVertices());

  // compute final transform by chaining H together
  span_tree.walkBreadthFirst(
      span_tree_centers[0],
      CalcAffineTransform(num_images, pairwise_matches, cameras));
  return true;
}
}  // namespace internal
}  // namespace combine_grids
