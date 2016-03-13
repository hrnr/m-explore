/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this
license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Copyright (C) 2016, Jiri Horner, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without
modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright
notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote
products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is"
and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are
disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any
direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <combine_grids/estimate_rigid_transform.h>

#include <opencv2/video/tracking.hpp>

namespace cv
{
/* copied from opencv source as this fuction is not exported */
static void getRTMatrix(const Point2f* a, const Point2f* b, int count, Mat& M,
                        bool fullAffine)
{
  CV_Assert(M.isContinuous());

  if (fullAffine) {
    double sa[6][6] = {{0.}}, sb[6] = {0.};
    Mat A(6, 6, CV_64F, &sa[0][0]), B(6, 1, CV_64F, sb);
    Mat MM = M.reshape(1, 6);

    for (int i = 0; i < count; i++) {
      sa[0][0] += a[i].x * a[i].x;
      sa[0][1] += a[i].y * a[i].x;
      sa[0][2] += a[i].x;

      sa[1][1] += a[i].y * a[i].y;
      sa[1][2] += a[i].y;

      sa[2][2] += 1;

      sb[0] += a[i].x * b[i].x;
      sb[1] += a[i].y * b[i].x;
      sb[2] += b[i].x;
      sb[3] += a[i].x * b[i].y;
      sb[4] += a[i].y * b[i].y;
      sb[5] += b[i].y;
    }

    sa[3][4] = sa[4][3] = sa[1][0] = sa[0][1];
    sa[3][5] = sa[5][3] = sa[2][0] = sa[0][2];
    sa[4][5] = sa[5][4] = sa[2][1] = sa[1][2];

    sa[3][3] = sa[0][0];
    sa[4][4] = sa[1][1];
    sa[5][5] = sa[2][2];

    solve(A, B, MM, DECOMP_EIG);
  } else {
    double sa[4][4] = {{0.}}, sb[4] = {0.}, m[4];
    Mat A(4, 4, CV_64F, sa), B(4, 1, CV_64F, sb);
    Mat MM(4, 1, CV_64F, m);

    for (int i = 0; i < count; i++) {
      sa[0][0] += a[i].x * a[i].x + a[i].y * a[i].y;
      sa[0][2] += a[i].x;
      sa[0][3] += a[i].y;

      sa[2][1] += -a[i].y;
      sa[2][2] += 1;

      sa[3][0] += a[i].y;
      sa[3][1] += a[i].x;
      sa[3][3] += 1;

      sb[0] += a[i].x * b[i].x + a[i].y * b[i].y;
      sb[1] += a[i].x * b[i].y - a[i].y * b[i].x;
      sb[2] += b[i].x;
      sb[3] += b[i].y;
    }

    sa[1][1] = sa[0][0];
    sa[2][1] = sa[1][2] = -sa[0][3];
    sa[3][1] = sa[1][3] = sa[2][0] = sa[0][2];
    sa[2][2] = sa[3][3] = count;
    sa[3][0] = sa[0][3];

    solve(A, B, MM, DECOMP_EIG);

    double* om = M.ptr<double>();
    om[0] = om[4] = m[0];
    om[1] = -m[1];
    om[3] = m[1];
    om[2] = m[2];
    om[5] = m[3];
  }
}

/* modified version of opencv function */
Mat estimateRigidTransform(InputArray src1, InputArray src2,
                           OutputArray _inliers_mask, bool fullAffine)
{
  Mat M(2, 3, CV_64F), A = src1.getMat(), B = src2.getMat();

  const int RANSAC_MAX_ITERS = 500;
  const int RANSAC_SIZE0 = 3;
  const double RANSAC_GOOD_RATIO = 0.5;

  std::vector<Point2f> pA, pB;
  std::vector<size_t> good_idx;
  Mat inliers_mask;

  double scale = 1.;
  size_t i, j, k, k1;

  RNG rng(static_cast<uint64>(-1));
  size_t good_count = 0;

  if (A.size() != B.size())
    CV_Error(Error::StsUnmatchedSizes, "Both input images must have the same "
                                       "size");

  if (A.type() != B.type())
    CV_Error(Error::StsUnmatchedFormats, "Both input images must have the same "
                                         "data type");

  // returns negative if not vector
  int vector_size = A.checkVector(2);

  if (vector_size > 0) {
    A.reshape(2, vector_size).convertTo(pA, CV_32F);
    B.reshape(2, vector_size).convertTo(pB, CV_32F);
  } else {
    CV_Error(Error::StsUnsupportedFormat, "This function does not support "
                                          "images as input.");
  }
  size_t count = static_cast<size_t>(vector_size);

  good_idx.resize(count);
  inliers_mask = Mat::zeros(static_cast<int>(count), 1, CV_8U);

  if (count < RANSAC_SIZE0)
    return Mat();

  Rect brect = boundingRect(pB);

  // RANSAC stuff:
  // 1. find the consensus
  for (k = 0; k < RANSAC_MAX_ITERS; k++) {
    size_t idx[RANSAC_SIZE0];
    Point2f a[RANSAC_SIZE0];
    Point2f b[RANSAC_SIZE0];

    // choose random 3 non-complanar points from A & B
    for (i = 0; i < RANSAC_SIZE0; i++) {
      for (k1 = 0; k1 < RANSAC_MAX_ITERS; k1++) {
        idx[i] = static_cast<size_t>(rng.uniform(0, static_cast<int>(count)));

        for (j = 0; j < i; j++) {
          if (idx[j] == idx[i])
            break;
          // check that the points are not very close one each other
          if (fabs(pA[idx[i]].x - pA[idx[j]].x) +
                  fabs(pA[idx[i]].y - pA[idx[j]].y) <
              FLT_EPSILON)
            break;
          if (fabs(pB[idx[i]].x - pB[idx[j]].x) +
                  fabs(pB[idx[i]].y - pB[idx[j]].y) <
              FLT_EPSILON)
            break;
        }

        if (j < i)
          continue;

        if (i + 1 == RANSAC_SIZE0) {
          // additional check for non-complanar vectors
          a[0] = pA[idx[0]];
          a[1] = pA[idx[1]];
          a[2] = pA[idx[2]];

          b[0] = pB[idx[0]];
          b[1] = pB[idx[1]];
          b[2] = pB[idx[2]];

          double dax1 = a[1].x - a[0].x, day1 = a[1].y - a[0].y;
          double dax2 = a[2].x - a[0].x, day2 = a[2].y - a[0].y;
          double dbx1 = b[1].x - b[0].x, dby1 = b[1].y - b[0].y;
          double dbx2 = b[2].x - b[0].x, dby2 = b[2].y - b[0].y;
          const double eps = 0.01;

          if (fabs(dax1 * day2 - day1 * dax2) <
                  eps * std::sqrt(dax1 * dax1 + day1 * day1) *
                      std::sqrt(dax2 * dax2 + day2 * day2) ||
              fabs(dbx1 * dby2 - dby1 * dbx2) <
                  eps * std::sqrt(dbx1 * dbx1 + dby1 * dby1) *
                      std::sqrt(dbx2 * dbx2 + dby2 * dby2))
            continue;
        }
        break;
      }

      if (k1 >= RANSAC_MAX_ITERS)
        break;
    }

    if (i < RANSAC_SIZE0)
      continue;

    // estimate the transformation using 3 points
    getRTMatrix(a, b, 3, M, fullAffine);

    const double* m = M.ptr<double>();
    for (i = 0, good_count = 0; i < count; i++) {
      if (std::abs(m[0] * pA[i].x + m[1] * pA[i].y + m[2] - pB[i].x) +
              std::abs(m[3] * pA[i].x + m[4] * pA[i].y + m[5] - pB[i].y) <
          std::max(brect.width, brect.height) * 0.05) {
        good_idx[good_count++] = i;
        inliers_mask.data[i] = true;
      }
    }

    if (good_count >= count * RANSAC_GOOD_RATIO)
      break;
  }

  if (k >= RANSAC_MAX_ITERS)
    return Mat();

  if (good_count < count) {
    for (i = 0; i < good_count; i++) {
      j = good_idx[i];
      pA[i] = pA[j];
      pB[i] = pB[j];
    }
  }

  getRTMatrix(&pA[0], &pB[0], static_cast<int>(good_count), M, fullAffine);
  M.at<double>(0, 2) /= scale;
  M.at<double>(1, 2) /= scale;

  // return inliers mask
  if (_inliers_mask.needed()) {
    inliers_mask.copyTo(_inliers_mask);
  }

  return M;
}

}  // namespace cv
