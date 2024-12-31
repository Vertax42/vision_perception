#ifndef __CIRCLE_DETECTOR_H__
#define __CIRCLE_DETECTOR_H__
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <limits>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <vector>

#include "version.h"

class circle_detector {
private:
    /* data */
public:
    enum DetectionType {
        Circle,
        NonCircle,
    };

    Eigen::Matrix4f averagePoseMatrices(const std::deque<Eigen::Matrix4f> &matrices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                                        pcl::ModelCoefficients::Ptr coefficients);

    Eigen::Matrix4f estimatePoseMatrixUsingPCA(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    Eigen::Matrix4f adjustPoseMatrixZAxis(Eigen::Matrix4f poseMatrix);

    void enhanceGrayImage(cv::Mat &grayImage);

    cv::Point2f calculateCircleCenter(cv::Point2f a, cv::Point2f b, cv::Point2f c);

    bool imageCallback(const std::vector<sensor_msgs::ImageConstPtr> &color_msg,
                       const std::vector<sensor_msgs::PointCloud2ConstPtr> &cloud_msg, cv::Mat &processed_image,
                       Eigen::Matrix4f &poseMatrix, sensor_msgs::PointCloud2 &ros_cloud, DetectionType type);

    bool is_same_point(const pcl::PointXYZRGB &point1, const pcl::PointXYZRGB &point2);

    bool is_valid_point(const pcl::PointXYZRGB &point);

    bool cloud_is_vaild(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float threhold);

    bool isDarkScene(const cv::Mat &grayImage, double threshold);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr completePlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &plane);

    pcl::PointXYZRGB projectPointOntoPlane(const pcl::PointXYZRGB &point,
                                           const pcl::ModelCoefficients::Ptr coefficients);

    cv_bridge::CvImagePtr chooseBestImage(const std::vector<cv_bridge::CvImagePtr> &colormsgVector);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    recoverPointcloudsfrommultiframes(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pointcloudVector);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeZeroPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
};

#endif // __CIRCLE_DETECTOR_H__