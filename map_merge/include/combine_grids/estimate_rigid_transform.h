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
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
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

#ifndef ESTIMATE_RIGID_TRANSFORM_H_
#define ESTIMATE_RIGID_TRANSFORM_H_

#include <opencv2/core/utility.hpp>

namespace cv
{
/** @brief Computes an optimal affine transformation between two 2D point sets.

@param src First input 2D point set stored in std::vector or Mat
@param dst Second input 2D point set of the same size and the same type as A
@param fullAffine If true, the function finds an optimal affine transformation
with no additional restrictions (6 degrees of freedom). Otherwise, the class of
transformations to choose from is limited to combinations of translation,
rotation, and uniform scaling (5 degrees of freedom).

This fuction is similar to cv::estimateRigidTransform. Key difference are:

*	does not support images as input
*	output inliners in similar manner to cv::findHomography

The function finds an optimal affine transform *[A|b]* (a 2 x 3 floating-point
matrix) that approximates best the affine transformation between:

*   Two point sets

In case of point sets, the problem is formulated as follows: you need to find a
2x2 matrix *A* and 2x1 vector *b* so that:

\f[[A^*|b^*] = arg  \min _{[A|b]}  \sum _i  \| \texttt{dst}[i] - A {
\texttt{src}[i]}^T - b  \| ^2\f]
where src[i] and dst[i] are the i-th points in src and dst, respectively
\f$[A|b]\f$ can be either arbitrary (when fullAffine=true ) or have a form of
\f[\begin{bmatrix} a_{11} & a_{12} & b_1  \\ -a_{12} & a_{11} & b_2
\end{bmatrix}\f]
when fullAffine=false.

@sa
getAffineTransform, getPerspectiveTransform, findHomography
 */
Mat estimateRigidTransform(InputArray src, InputArray dst,
                           OutputArray inliers_mask, bool fullAffine);

}  // namespace cv

#endif  // ESTIMATE_RIGID_TRANSFORM_H_
