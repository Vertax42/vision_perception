#include "vision_common.h"

void vision_common::CalibrateHist(cv::Mat src, int &b_max_position, int &g_max_position, int &r_max_position)
{
    // 初始化参数
    b_max_position = 0;
    g_max_position = 0;
    r_max_position = 0;

    // 步骤一：分通道显示
    std::vector<cv::Mat> bgr_planes;
    cv::split(src, bgr_planes);

    // 步骤二：计算直方图
    int histsize = 256;
    float range[] = { 0, 256 };
    const float *histRanges = { range };
    cv::Mat b_hist, g_hist, r_hist;

    cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histsize, &histRanges, true, false);
    cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histsize, &histRanges, true, false);
    cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histsize, &histRanges, true, false);

    // 归一化直方图
    int hist_h = 400; // 直方图高度
    int hist_w = 512; // 直方图宽度
    int bin_w = hist_w / histsize;
    cv::Mat histImage(hist_w, hist_h, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::normalize(b_hist, b_hist, 0, hist_h, cv::NORM_MINMAX, -1,
                  cv::Mat()); // blue
    cv::normalize(g_hist, g_hist, 0, hist_h, cv::NORM_MINMAX, -1,
                  cv::Mat()); // green
    cv::normalize(r_hist, r_hist, 0, hist_h, cv::NORM_MINMAX, -1,
                  cv::Mat()); // red

    // 步骤三：绘制直方图并计算峰值位置
    float b_max_value = 0.0, g_max_value = 0.0, r_max_value = 0.0;

    for(int i = 1; i < histsize; i++)
    {
        // 绘制蓝色分量直方图
        cv::line(histImage, cv::Point((i - 1) * bin_w, hist_h - cvRound(b_hist.at<float>(i - 1))),
                 cv::Point(i * bin_w, hist_h - cvRound(b_hist.at<float>(i))), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        if(b_hist.at<float>(i) > b_max_value)
        {
            b_max_value = b_hist.at<float>(i);
            b_max_position = i;
        }

        // 绘制绿色分量直方图
        cv::line(histImage, cv::Point((i - 1) * bin_w, hist_h - cvRound(g_hist.at<float>(i - 1))),
                 cv::Point(i * bin_w, hist_h - cvRound(g_hist.at<float>(i))), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        if(g_hist.at<float>(i) > g_max_value)
        {
            g_max_value = g_hist.at<float>(i);
            g_max_position = i;
        }

        // 绘制红色分量直方图
        cv::line(histImage, cv::Point((i - 1) * bin_w, hist_h - cvRound(r_hist.at<float>(i - 1))),
                 cv::Point(i * bin_w, hist_h - cvRound(r_hist.at<float>(i))), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        if(r_hist.at<float>(i) > r_max_value)
        {
            r_max_value = r_hist.at<float>(i);
            r_max_position = i;
        }
    }

    // 绘制最大值位置线
    cv::line(histImage, cv::Point((b_max_position - 1) * bin_w, 0), cv::Point((b_max_position - 1) * bin_w, hist_h),
             cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    cv::line(histImage, cv::Point((g_max_position - 1) * bin_w, 0), cv::Point((g_max_position - 1) * bin_w, hist_h),
             cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(histImage, cv::Point((r_max_position - 1) * bin_w, 0), cv::Point((r_max_position - 1) * bin_w, hist_h),
             cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    // 调试模式显示或保存直方图

    cv::imshow("calcHist", histImage);
    std::string histPath = "hist.jpg";
    cv::imwrite(histPath, histImage);
    cv::waitKey(0);
}

double vision_common::cross(const cv::Point2f &o, const cv::Point2f &a, const cv::Point2f &b)
{
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

bool vision_common::isPointInsideRect(const cv::Point2f &p, const cv::Rect &rect)
{
    return rect.contains(cv::Point(p.x, p.y));
}

bool vision_common::isPointInsideRect(const cv::Point2f &p, const cv::Point2f &a, const cv::Point2f &b,
                                      const cv::Point2f &c, const cv::Point2f &d)
{
    double d1 = cross(p, a, b);
    double d2 = cross(p, b, c);
    double d3 = cross(p, c, d);
    double d4 = cross(p, d, a);

    // check if the cross product has the same sign
    return ((d1 > 0 && d2 > 0 && d3 > 0 && d4 > 0) || (d1 < 0 && d2 < 0 && d3 < 0 && d4 < 0));
}