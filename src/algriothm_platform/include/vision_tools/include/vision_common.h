#ifndef __VISION_COMMON_H__
#define __VISION_COMMON_H__

#include "utils_common.h"
#include <opencv2/opencv.hpp>

namespace vision_common {

using namespace std;

// Here are universal functions for processing images

// calibrate the histogram of the image
void CalibrateHist(cv::Mat src, int &b_max_position, int &g_max_position, int &r_max_position);

// caculate the cross product of two vectors, return z component, o is the
// origin point
double cross(const cv::Point2f &o, const cv::Point2f &a, const cv::Point2f &b);

bool isPointInsideRect(const cv::Point2f &p, const cv::Rect &rect);

bool isPointInsideRect(const cv::Point2f &p, const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &c,
                       const cv::Point2f &d);

} // namespace vision_common

#endif // VISION_COMMON_H