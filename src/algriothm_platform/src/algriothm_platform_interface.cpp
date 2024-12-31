#include <iostream>
#include <memory>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// #include <pcl/pcl_config.h>
// #include <opencv2/core/core.hpp>
#include "image.h"
#include "json.h"
#include "log4z.h"
//#include "utils_common.h"
//#include "vision_common.h"
#include "circle_detector.h"
#include "utils_logger.hpp"

using namespace zsummer::log4z;

void imagePub(const std::vector<sensor_msgs::ImageConstPtr> &color_msg,
              const std::vector<sensor_msgs::PointCloud2ConstPtr> &cloud_msg,
              image_transport::Publisher &processed_image_pub)
{
    circle_detector cd;
    cv::Mat processed_image;
    Eigen::Matrix4f poseMatrix;
    sensor_msgs::PointCloud2 ros_cloud;

    cd.imageCallback(color_msg, cloud_msg, processed_image, poseMatrix, ros_cloud,
                     circle_detector::DetectionType::Circle);

    // cd.imageCallback(color_msg,cloud_msg,processed_image,poseMatrix);
    // ROS_WARN("cols:%d",processed_image.cols);
    std::cout << "Estimated Pose Matrix :\n" << poseMatrix << std::endl;

    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(color_msg[0]->header, "bgr8", processed_image).toImageMsg();
    processed_image_pub.publish(output_msg);
}

int main(int argc, char *argv[])
{
    //执行 ros 节点初始化, log4z初始化
    common_tools::clean_log_files(5); // clean log4z files over 5, not include itself, default path is "./log4z/"
    ILog4zManager::getRef().start();
    ILog4zManager::getRef().setLoggerLevel(LOG4Z_MAIN_LOGGER_ID, LOG_LEVEL_DEBUG);

    // logger maxsize 100MB
    ILog4zManager::getRef().setLoggerLimitsize(LOG4Z_MAIN_LOGGER_ID, 100);

    ros::init(argc, argv, "algriothm_platform_interface");
    printf_program("Algriothm_platform_interface: for 2D and 3D perception "
                   "algorithm testing.");
    common_tools::printf_software_version("Algriothm_Platfrom");
    common_tools::dump_program_info_log4z("Algriothm_Platfrom");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    //控制台输出 hello world
    ROS_INFO("Algriothm_Platform program is running.");
    LOGD("Algriothm_Platform program is running.");
    std::string image_path = "/home/observer/Downloads/test_image/Lenna_(test_image).png";
    std::shared_ptr<Image> img = std::make_shared<Image>(image_path);
    img->showImg();
    // cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR); // 读取彩色图片
    // if (image.empty()) { // 检查图片是否成功加载
    //     std::cerr << "Error: Could not load image at " << image_path <<
    //     std::endl; return -1;
    // }
    // // 调用函数
    // int b_max_position, g_max_position, r_max_position;
    // vision_common::CalibrateHist(image, b_max_position, g_max_position,
    // r_max_position); message_filters::Subscriber<sensor_msgs::Image>
    // color_sub(nh, "/global_cam/rgb/rgb_raw", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh,
    // "/global_cam/depth/berxel_cloudpoint", 1);

    // message_filters::Cache<sensor_msgs::Image> color_cache(color_sub, 5);
    // message_filters::Cache<sensor_msgs::PointCloud2> cloud_cache(cloud_sub,
    // 5);

    // color_sub.registerCallback([&color_cache](const
    // sensor_msgs::ImageConstPtr& msg) {
    //     color_cache.add(msg);  // 将新消息添加到缓存
    // });

    // cloud_sub.registerCallback([&cloud_cache](const
    // sensor_msgs::PointCloud2ConstPtr& msg) {
    //     cloud_cache.add(msg);  // 将新消息添加到缓存
    // });

    // image_transport::Publisher processed_image_pub =
    // it.advertise("/camera/processed_image", 1); ros::Timer timer =
    // nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&) {
    //     // 获取缓存中的最近 5 条消息
    //     std::vector<sensor_msgs::ImageConstPtr> color_msgs =
    //     color_cache.getInterval(
    //         ros::Time::now() - ros::Duration(1.0), ros::Time::now());
    //     std::vector<sensor_msgs::PointCloud2ConstPtr> cloud_msgs =
    //     cloud_cache.getInterval(
    //         ros::Time::now() - ros::Duration(1.0), ros::Time::now());

    //     // 确保有足够的消息
    //     if (color_msgs.size() < 5 || cloud_msgs.size() < 5) {
    //         ROS_WARN("Not enough messages in cache");
    //         return;
    //     }

    //     // 调用 imagePub
    //     imagePub(color_msgs, cloud_msgs, processed_image_pub);
    // });

    // Eigen::Matrix4f testMatrix;
    // testMatrix << 1, 2, 3, 4,
    //               5, 6, 7, 8,
    //               9, 10, 11, 12,
    //               13, 14, 15, 16;

    // LOGFMTW("The testMatrix is: \n%s",
    // common_tools::eigenMatrixToString(testMatrix).c_str());

    // LOGFMTD("format input *** %s *** %d ***", "LOGFMTD", 123456);  // debug
    // LOGFMTI("format input *** %s *** %d ***", "LOGFMTI", 123456);  // info
    // LOGFMTW("format input *** %s *** %d ***", "LOGFMTW", 123456);  // warning

    // LOGFMTE("format input *** %s *** %d ***", "LOGFMTE", 123456);  // error
    // LOGFMTA("format input *** %s *** %d ***", "LOGFMTA", 123456);  // alarm
    // LOGFMTF("format input *** %s *** %d ***", "LOGFMTF", 123456);  // fatal

    // ILog4zManager::getRef().setLoggerLevel(LOG4Z_MAIN_LOGGER_ID,
    // LOG_LEVEL_DEBUG); LOGFMTD("1323123123 input *** %s *** %d ***", "LOGFMTD",
    // 123456);  // debug LOGFMTI("1231312kladasd input *** %s *** %d ***",
    // "LOGFMTI", 123456);  // info

    LOGA("main quit ...");
    // Ilog4zManager::getInstance()->stop(); // stop log4z
    return 0;
}