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

#include <combine_grids/merging_pipeline.h>
#include <gtest/gtest.h>
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/utility.hpp>
#include "testing_helpers.h"

const std::array<const char*, 13> hector_maps = {
    "map00.pgm", "map05.pgm", "map07.pgm", "map09.pgm", "map11.pgm",
    "map16.pgm", "map19.pgm", "map21.pgm", "map22.pgm", "map25.pgm",
    "map27.pgm", "map28.pgm", "map31.pgm",
};

constexpr bool verbose_tests = true;

TEST(MergingPipeline, canStich0Grid)
{
  std::vector<nav_msgs::OccupancyGridConstPtr> maps;
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin(), maps.end());
  EXPECT_TRUE(merger.estimateTransform());
  EXPECT_EQ(merger.composeGrids(), nullptr);
  EXPECT_EQ(merger.getTransforms().size(), 0);
}

TEST(MergingPipeline, canStich1Grid)
{
  auto maps = loadMaps(hector_maps.begin(), hector_maps.end());
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin() + 1, maps.begin() + 2);
  merger.estimateTransform();
  auto merged_grid = merger.composeGrids();

  // sanity of merged grid
  ASSERT_TRUE(static_cast<bool>(merged_grid));
  EXPECT_FALSE(merged_grid->data.empty());
  EXPECT_EQ((merged_grid->info.width) * (merged_grid->info.height),
            merged_grid->data.size());
  // merged must be the same with original
  EXPECT_EQ(merged_grid->info.width, maps[1]->info.width);
  EXPECT_EQ(merged_grid->info.height, maps[1]->info.height);
  EXPECT_EQ(merged_grid->data.size(), maps[1]->data.size());
  for (size_t i = 0; i < merged_grid->data.size(); ++i) {
    EXPECT_EQ(merged_grid->data[i], maps[1]->data[i]);
  }
  // check estimated transforms
  auto transforms = merger.getTransforms();
  EXPECT_EQ(transforms.size(), 1);
  tf2::Transform t;
  tf2::fromMsg(transforms[0], t);
  EXPECT_EQ(tf2::Transform::getIdentity(), t);
}

TEST(MergingPipeline, canStich2Grids)
{
  auto maps = loadMaps(hector_maps.begin(), hector_maps.end());
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin(), maps.begin() + 2);
  merger.estimateTransform();
  auto merged_grid = merger.composeGrids();

  // sanity of merged grid
  ASSERT_TRUE(static_cast<bool>(merged_grid));
  EXPECT_FALSE(merged_grid->data.empty());
  EXPECT_EQ((merged_grid->info.width) * (merged_grid->info.height),
            merged_grid->data.size());
  // transforms
  auto transforms = merger.getTransforms();
  EXPECT_EQ(transforms.size(), 2);
  tf2::Transform t;
  tf2::fromMsg(transforms[0], t);
  EXPECT_EQ(tf2::Transform::getIdentity(), t);

  if (verbose_tests) {
    for (auto& transform : transforms) {
      std::cout << transform << std::endl;
    }
    saveMap("test_canStich2Grids.pgm", merged_grid);
  }
}

int main(int argc, char** argv)
{
  ros::Time::init();
  if (verbose_tests &&
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
