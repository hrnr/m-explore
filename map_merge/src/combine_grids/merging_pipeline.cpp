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
#define _USE_MATH_DEFINES
#include <math.h>

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
  cv::Ptr<cv::detail::FeaturesFinder> finder =
      internal::chooseFeatureFinder(feature_type);
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
      (*finder)(image, image_features.back());
    }
  }
  finder->collectGarbage();

  /* find corespondent features */
  ROS_DEBUG("pairwise matching features");
  (*matcher)(image_features, pairwise_matches);
  matcher->collectGarbage();

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
  //if (!(*adjuster)(image_features, pairwise_matches, transforms)) {
  //  ROS_WARN("Bundle adjusting failed. Could not estimate transforms.");
  //  return false;
  //}

  transforms_.clear();
  transforms_.resize(images_.size());
  size_t i = 0;
  ROS_DEBUG("Adding transforms into transforms_");
  for (auto& j : good_indices) {
    // we want to work with transforms as doubles
    transforms[i].R.convertTo(transforms_[static_cast<size_t>(j)], CV_64F);
    ++i;
  }
  ROS_DEBUG("Finished adding transforms into transforms_");

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
  std::vector<cv::Point> corners;
  corners.reserve(images_.size());
  std::vector<cv::Size> sizes;
  sizes.reserve(images_.size());
  result = compositor.compose(imgs_warped, rois);
  ROS_DEBUG("Just after getting results. IN COMPOSE GRIDS: Result map size: (%i, %i)", (result->info.width, result->info.height));
  roi_info_.clear();
  for (auto& roi : rois) {
	  roi_info_.push_back(roi);
	  corners.push_back(roi.tl());
	  sizes.push_back(roi.size());
  }
  complete_roi_ = cv::detail::resultRoi(corners, sizes);
  // TODO: CHECK WHY THE VALUE FROM result->info.height is WRONG
  ROS_DEBUG("IN COMPOSE GRIDS: Complete roi: (%i, %i)", (complete_roi_.width, complete_roi_.height));
  result->info.width = complete_roi_.width;
  result->info.height = complete_roi_.height;

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

  // set grid origin to its centre - Why?
  result->info.origin.position.x = 0.0; // (result->info.width / 2.0) * double(result->info.resolution); //complete_roi_.x * double(result->info.resolution);
  result->info.origin.position.y = 0.0; //(result->info.height / 2.0) * double(result->info.resolution); //complete_roi_.y * double(result->info.resolution);
  result->info.origin.orientation.w = 1.0;
  int aux_width = result->info.width;
  int aux_height = result->info.height;
  ROS_DEBUG("IN COMPOSE GRIDS: Result map size: (%i, %i)", (aux_width, aux_height));
  result_map_width = (float)aux_width * result->info.resolution; // in meters
  result_map_height = (float)aux_height * result->info.resolution; // in meters
  ROS_DEBUG("IN COMPOSE GRIDS: Result map size: (%f, %f)", (result_map_width, result_map_height));

  return result;
}

std::vector<geometry_msgs::Transform> MergingPipeline::getTransforms() const
{
  std::vector<geometry_msgs::Transform> result;
  result.reserve(transforms_.size());

  geometry_msgs::Transform identity_quaternion;
  identity_quaternion.rotation.w = 1;

  ROS_DEBUG("IN GET_TRANSFORMS");
  for (auto& transform : transforms_) {
    if (transform.empty()) {
      result.emplace_back(identity_quaternion);
      ROS_DEBUG("TRANSFORM EMPTY");
      continue;
    }

    int loop_counter = 0;
    float result_map_center_x = result_map_width / 2.0;
    float result_map_center_y = result_map_height / 2.0;
    bool is_ready = false;
    if (result_map_width > 0.0 && result_map_height > 0.0){
	    is_ready = true;
    }
    ROS_DEBUG("Result map size: (%f, %f)", ((float)result_map_width, (float)result_map_height));
    std::vector<cv::Point2f> corners(4);

    ROS_DEBUG("TRANSFORM NOT EMPTY. LETS COMPUTE IT");

    if (is_ready)
    {
    	ROS_DEBUG("INSIDE IF...");
		// Required variables - Maps data
		float resolution = grids_[loop_counter]->info.resolution;
		ROS_DEBUG("Resolution: %f", resolution);
		float map_height = grids_[loop_counter]->info.height * resolution;
		float map_width = grids_[loop_counter]->info.width * resolution;
		ROS_DEBUG("Height and width");
		ROS_DEBUG("Result map size: (%f, %f)", ((float)result_map_width, (float)result_map_height));
		float center_bias_x = (roi_info_[loop_counter].x + complete_roi_.x) * resolution;
		float center_bias_y = (roi_info_[loop_counter].y + complete_roi_.y) * resolution;
		ROS_DEBUG("Center bias");
	//	float y = grids_[loop_counter]->info.height * resolution;
	//	float x = grids_[loop_counter]->info.width * resolution;

		// our rotation is in fact only 2D, thus quaternion can be simplified
		double a = transform.at<double>(0, 0);
		double b = transform.at<double>(1, 0);
		if (std::abs(a) > 1){
			a = std::copysign(1, a);
		}
		double alpha = std::acos(a);
		ROS_DEBUG("Rotated: %f", alpha);

		// Add corners
		corners.clear();
		for (int i = 0; i < 4; i++){
			if (i == 0){
				corners[i].x = 0.0f;
				corners[i].y = 0.0f;
			}
			if (i == 1){
				corners[i].x = 0.0f;
				corners[i].y = map_height;
			}
			if (i == 2){
				corners[i].x = map_width;
				corners[i].y = 0.0f;
			}
			if (i == 3){
				corners[i].x = map_width;
				corners[i].y = map_height;
			}
			ROS_DEBUG("Corner init x: %f", corners[i].x);
			ROS_DEBUG("Corner init y: %f", corners[i].y);
		}

		// Rotate corners as image
		for (int i = 0; i < 4; i++){
			float aux_x = corners[i].x * std::cos(alpha) - corners[i].y * std::sin(alpha);
			float aux_y = corners[i].x * std::sin(alpha) + corners[i].y * std::cos(alpha);
			corners[i].x = aux_x;
			corners[i].y = aux_y;
		}

		ROS_ASSERT(transform.type() == CV_64F);
		geometry_msgs::Transform ros_transform;

		//Rotation distance - Only for the ones that rotate
		//from the position of the turned corned (width,height) until the Origin (0.0)
		//TODO: Instead of the turned corned (width, height). Do it with the corner that is more far from the origin!
		if (std::abs(alpha) > 0.001){
			ros_transform.translation.x = 0.0 - (map_width * std::cos(alpha) - map_height * std::sin(alpha));
			ros_transform.translation.y = 0.0 - (map_width * std::sin(alpha) + map_height * std::cos(alpha));
			ROS_DEBUG("Rotation distance x: %f", ros_transform.translation.x);
			ROS_DEBUG("Rotation distance y: %f", ros_transform.translation.y);
			// Corners displacement - This might be useless if the previous TODO is applied.
			for (int i = 0; i < 4; i++){
				corners[i].x = corners[i].x - center_bias_x + 0.0 - (map_width * std::cos(alpha) - map_height * std::sin(alpha));
				corners[i].y = corners[i].y - center_bias_y + 0.0 - (map_width * std::sin(alpha) + map_height * std::cos(alpha));
				// Check Corners out of result map
				if (corners[i].x < 0.0){
					ros_transform.translation.x = ros_transform.translation.x + std::abs(corners[i].x);
					ROS_DEBUG("Corner out of bounds. Moved: %f", std::abs(corners[i].x));
				}
				if (corners[i].y < 0.0){
					ros_transform.translation.y = ros_transform.translation.y + std::abs(corners[i].y);
					ROS_DEBUG("Corner out of bounds. Moved: %f", std::abs(corners[i].y));
				}
				if (corners[i].x > result_map_width){
					ros_transform.translation.x = ros_transform.translation.x - (corners[i].x - result_map_width);
					ROS_DEBUG("Corner out of bounds. Moved: %f", - (corners[i].x - result_map_width));
				}
				if (corners[i].y > result_map_height){
					ros_transform.translation.y = ros_transform.translation.y - (corners[i].y - result_map_height);
					ROS_DEBUG("Corner out of bounds. Moved: %f", - (corners[i].y - result_map_height));
				}
			}
		}

		// Adding center bias
		ROS_DEBUG("center_bias_x: %f", center_bias_x);
		ROS_DEBUG("center_bias_y: %f", center_bias_y);
		ros_transform.translation.x = ros_transform.translation.x - center_bias_x;
		ros_transform.translation.y = ros_transform.translation.y - center_bias_y;
		// Apply bias to corners
		for (int i = 0; i < 4; i++){
			corners[i].x = corners[i].x - center_bias_x + 0.0 - (map_width * std::cos(alpha) - map_height * std::sin(alpha));
			corners[i].y = corners[i].y - center_bias_y + 0.0 - (map_width * std::sin(alpha) + map_height * std::cos(alpha));
		}

		ros_transform.translation.z = 0.;
		ROS_DEBUG("ros_transform.translation.x: %f", ros_transform.translation.x);
		ROS_DEBUG("ros_transform.translation.y: %f", ros_transform.translation.y);
		// our rotation is in fact only 2D, thus quaternion can be simplified
		ros_transform.rotation.w = std::cos(alpha * 0.5); //std::sqrt(2. + 2. * a) * 0.5;
		ros_transform.rotation.x = 0.;
		ros_transform.rotation.y = 0.;
		ros_transform.rotation.z = std::sin(alpha * 0.5); //std::copysign(std::sqrt(2. - 2. * a) * 0.5, b);

		result.push_back(ros_transform);
		loop_counter = loop_counter + 1;
    }
	// Adding map size bias
	// TODO: Properly define this bias



//    if (resolution > 0 && result_map_height > 0.0){
//    	//float a = roi_info_[loop_counter].x;
//        //float b = complete_roi_.x;
//    	if (roi_info_.size() > loop_counter) {
////    		if (!rotated_pi)
////    		{
////    		y = grids_[loop_counter]->info.height * resolution;
////			x = grids_[loop_counter]->info.width * resolution;
////			center_bias_x = (roi_info_[loop_counter].x + complete_roi_.x) * resolution;
////			center_bias_y = (roi_info_[loop_counter].y + complete_roi_.y) * resolution;
////			center_bias_x = (roi_info_[loop_counter].x - complete_roi_.x) * resolution;
////			center_bias_y = (roi_info_[loop_counter].y - complete_roi_.y) * resolution;
//    		if (std::abs(alpha) > 0.01) {
//    			ROS_DEBUG("Rotation distance x: %f", map_width * std::cos(alpha) - map_height * std::sin(alpha));
//				ROS_DEBUG("Rotation distance y: %f", map_width * std::sin(alpha) + map_height * std::cos(alpha));
//		    	ROS_DEBUG("center_bias_x: %f", center_bias_x);
//				ROS_DEBUG("center_bias_y: %f", center_bias_y);
//    			ros_transform.translation.x = - center_bias_x + 0.0 - (map_width * std::cos(alpha) - map_height * std::sin(alpha));
//				ros_transform.translation.y = - center_bias_y + 0.0 - (map_width * std::sin(alpha) + map_height * std::cos(alpha));
//				for (int i = 0; i < 4; i++){
//					corners[i].x = corners[i].x - center_bias_x + 0.0 - (map_width * std::cos(alpha) - map_height * std::sin(alpha));
//					corners[i].y = corners[i].y - center_bias_y + 0.0 - (map_width * std::sin(alpha) + map_height * std::cos(alpha));
//				}
//				if (ros_transform.translation.y < 0.0){
//					ros_transform.translation.y = ros_transform.translation.y + map_height;
//					for (int i = 0; i < 4; i++){
////						p.x = p.x - center_bias_x + 0.0 - (x * std::cos(alpha) - y * std::sin(alpha));
//						corners[i].y = corners[i].y + map_height;
//					}
////					ros_transform.translation.x = ros_transform.translation.x + x + (result_map_width - grids_[loop_counter]->info.width * resolution) * 0.5;
//				} else if (ros_transform.translation.x < 0.0) {
//					ros_transform.translation.x = ros_transform.translation.x + map_width;
//					for (int i = 0; i < 4; i++){
//						corners[i].x = corners[i].x + map_width;
//					}
////					ros_transform.translation.y = ros_transform.translation.y + y + (result_map_height - grids_[loop_counter]->info.height * resolution) * 0.5;
////					ros_transform.translation.x = ros_transform.translation.x + x + (result_map_height - grids_[loop_counter]->info.height * resolution) * 0.5;
//				}
//				if (map_width/map_height > 1.0) {
//					ros_transform.translation.x = ros_transform.translation.x - (result_map_width - map_width) * 0.25;
//					ros_transform.translation.y = ros_transform.translation.y - (result_map_height - map_height) * 0.25;
//					ROS_DEBUG("Map center_bias_x: %f", (result_map_width - map_width) * 0.25);
//					ROS_DEBUG("Map center_bias_y: %f", (result_map_height - map_height) * 0.25);
//					ROS_DEBUG("Corners: ");
//					for (int i = 0; i < 4; i++){
//						corners[i].x = corners[i].x - (result_map_width - map_width) * 0.25;
//						corners[i].y = corners[i].y - (result_map_height - map_height) * 0.25;
//						ROS_DEBUG("Corner x: %f", corners[i].x);
//						ROS_DEBUG("Corner y: %f", corners[i].y);
//					}
//				}
//				if (map_height/map_width > 1.0){
//					// FIXME: Maybe it does not depend on relation y/x but el quadrant del que prové... (o de la relació entre formes)
//					ros_transform.translation.x = ros_transform.translation.x + (result_map_width - map_width) * 0.25;
//					ros_transform.translation.y = ros_transform.translation.y + (result_map_height - map_height) * 0.5;
//					ROS_DEBUG("Map center_bias_x: %f", (result_map_height - map_width) * 0.25);
//					ROS_DEBUG("Map center_bias_y: %f", (result_map_width - map_height) * 0.25);
//					for (int i = 0; i < 4; i++){
//						corners[i].x = corners[i].x - (result_map_height - map_width) * 0.25;
//						corners[i].y = corners[i].y - (result_map_width- map_height) * 0.25;
//						ROS_DEBUG("Corner x: %f", corners[i].x);
//						ROS_DEBUG("Corner y: %f", corners[i].y);
//					}
//				}
//				for (int i = 0; i < 4; i++){
//					if (corners[i].x < 0.0){
//						ros_transform.translation.x = ros_transform.translation.x + std::abs(corners[i].x);
//					}
//					if (corners[i].y < 0.0){
//						ros_transform.translation.y = ros_transform.translation.y + std::abs(corners[i].y);
//					}
//					if (corners[i].x > result_map_width){
//						ros_transform.translation.x = ros_transform.translation.x - (corners[i].x - result_map_width);
//					}
//					if (corners[i].y > result_map_height){
//						ros_transform.translation.y = ros_transform.translation.y - (corners[i].y - result_map_height);
//					}
//				}
//    		} else {
//    			ROS_DEBUG("Rotation distance x: %f", result_map_center_x - (result_map_center_x * std::cos(alpha) - result_map_center_y * std::sin(alpha)));
//				ROS_DEBUG("Rotation distance y: %f", result_map_center_y - (result_map_center_x * std::sin(alpha) + result_map_center_y * std::cos(alpha)));
//		    	ROS_DEBUG("center_bias_x: %f", center_bias_x);
//				ROS_DEBUG("center_bias_y: %f", center_bias_y);
//    			ros_transform.translation.x = - center_bias_x + result_map_center_x - (result_map_center_x * std::cos(alpha) - result_map_center_y * std::sin(alpha));
//    			ros_transform.translation.y = - center_bias_y + result_map_center_y - (result_map_center_x * std::sin(alpha) + result_map_center_y * std::cos(alpha));
//    			for (int i = 0; i < 4; i++){
//    				corners[i].x = corners[i].x - center_bias_x + result_map_center_x - (result_map_center_x * std::cos(alpha) - result_map_center_y * std::sin(alpha));
//    				corners[i].y = corners[i].y - center_bias_y + result_map_center_y - (result_map_center_x * std::sin(alpha) + result_map_center_y * std::cos(alpha));
//					ROS_DEBUG("Corner x: %f", corners[i].x);
//					ROS_DEBUG("Corner y: %f", corners[i].y);
//    			}
//    		}
//    	} else {
//    		center_bias_x = 0.0;
//    		center_bias_y = 0.0;
//    	}
////    	if (transform.at<double>(0, 2) == 0.0 && transform.at<double>(1, 2) == 0.0) {
////    		ros_transform.translation.x = center_bias_x;
////    		ros_transform.translation.y = center_bias_y;
////    	} else {
////    		ros_transform.translation.x = -0.5*result_map_width + center_bias_x;
////    		ros_transform.translation.y = -0.5*result_map_height + center_bias_y;
////    	}
//    	//center_bias_x = 0.5 * (result_map_width - grids_[loop_counter]->info.width * resolution);
//    	//center_bias_y = 0.5 * (result_map_height - grids_[loop_counter]->info.height * resolution);
////    	ros_transform.translation.x = transform.at<double>(0, 2) * resolution + center_bias_x;
////		ros_transform.translation.y = transform.at<double>(1, 2) * resolution + center_bias_y;
//
//
//
//		//ROS_DEBUG("ros_transform - result_map_center_x: %f", grids_[loop_counter]->info.origin.position.x);
//		//ROS_DEBUG("ros_transform - result_map_center_y: %f", grids_[loop_counter]->info.origin.position.y);
//    } else {
//    	ros_transform.translation.x = 0.0; //transform.at<double>(0, 2);
//		ros_transform.translation.y = 0.0; //transform.at<double>(1, 2);
//    }
//    ros_transform.translation.z = 0.;
//
//
//    ROS_DEBUG("ros_transform.translation.x: %f", ros_transform.translation.x);
//	ROS_DEBUG("ros_transform.translation.y: %f", ros_transform.translation.y);
//    // our rotation is in fact only 2D, thus quaternion can be simplified
//    ros_transform.rotation.w = std::cos(alpha * 0.5); //std::sqrt(2. + 2. * a) * 0.5;
//    ros_transform.rotation.x = 0.;
//    ros_transform.rotation.y = 0.;
//    ros_transform.rotation.z = std::sin(alpha * 0.5); //std::copysign(std::sqrt(2. - 2. * a) * 0.5, b);
//
//    result.push_back(ros_transform);
//    loop_counter = loop_counter + 1;
  }

  return result;
}

}  // namespace combine_grids
