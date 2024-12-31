#ifndef __SLIC_H__
#define __SLIC_H__

/* slic.h.
 *
 * Written by: Pascal Mettes.
 *
 * This file contains the class elements of the class Slic. This class is an
 * implementation of the SLIC Superpixel algorithm by Achanta et al. [PAMI'12,
 * vol. 34, num. 11, pp. 2274-2282].
 *
 * This implementation is created for the specific purpose of creating
 * over-segmentations in an OpenCV-based environment.
 */

#include <float.h>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
using namespace std;

/* 2d matrices are handled by 2d vectors. */
#define vec2dd vector<vector<double> >
#define vec2di vector<vector<int> >
#define vec2db vector<vector<bool> >
/* The number of iterations run by the clustering algorithm. */
#define NR_ITERATIONS 10

/*
 * class Slic.
 *
 * In this class, an over-segmentation is created of an image, provided by the
 * step-size (distance between initial cluster locations) and the colour
 * distance parameter.
 */
class Slic {
private:
    /* The cluster assignments and distance values for each pixel. */
    vec2di clusters;  // 存储每个像素点所属的聚类ID
    vec2dd distances; // 存储每个像素点与聚类中心的距离

    /* The LAB and xy values of the centers. */
    vec2dd centers; // 存储聚类中心的坐标 (Lab 空间和空间坐标)
    /* The number of occurences of each center. */
    vector<int> center_counts; // 每个聚类中心对应的像素数量

    /* The step size per cluster, and the colour (nc) and distance (ns)
     * parameters. */
    int step, nc, ns; // 步长，颜色距离参数，空间距离参数

    /* Compute the distance between a center and an individual pixel. */
    double compute_dist(int ci, cv::Point pixel, cv::Scalar colour);
    /* Find the pixel with the lowest gradient in a 3x3 surrounding. */
    cv::Point find_local_minimum(cv::Mat &image, cv::Point center);

    /* Remove and initialize the 2d vectors. */
    void clear_data();
    void init_data(cv::Mat &image);

public:
    /* Class constructors and deconstructors. */
    Slic();
    ~Slic();

    /* Generate an over-segmentation for an image. */
    void generate_superpixels(cv::Mat &image, int step, int nc); // 生成超像素
    /* Enforce connectivity for an image. */
    void create_connectivity(cv::Mat &image); // 保证超像素的连通性

    /* Draw functions. Resp. displayal of the centers and the contours. */
    void display_center_grid(cv::Mat &image, cv::Scalar colour); // 显示聚类中心
    void display_contours(cv::Mat &image, cv::Scalar colour);    // 显示轮廓
    void colour_with_cluster_means(cv::Mat &image);

    /* 返回聚类结果 */
    vector<vector<int> > GetClusters() { return clusters; }
};

#endif // SLIC_H
