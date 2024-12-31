#ifndef __IMAGE_H__
#define __IMAGE_H__

#include "log4z.h"
#include "utils_common.h"
#include "vision_common.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <utils_colorprint.hpp>

enum class ImageFormat { Unknown = 0, PNG = 1, JPG = 2, BMP = 3, TIFF = 4, WEBP = 5 };

enum class DataType {
    Unknown = 0,
    Grayscale = 1, // 单通道灰度图
    BGR = 2,       // 三通道RGB图 OpenCV默认的颜色空间
    BGRA = 3,      // 四通道RGBA图
    Other = 4      // 其他格式
};

class Image {
public:
    // constructor
    Image()
        : _img(cv::Mat()), _inpath("input.jpg"), _outpath("output.jpg"), _imgname("test_img"),
          _dir(boost::filesystem::current_path().string()), _format(ImageFormat::Unknown), _datatype(DataType::Unknown),
          _file_size(0), _isLoaded(false)
    {}

    Image(const std::string &in_path, const std::string &out_path);

    Image(const std::string &in_path);

    Image(const Image &img)
    {
        _img = img.GetMat();
        _inpath = img._inpath;
        _outpath = img._outpath;
    };

    void operator=(const Image &img)
    {
        _img = img.GetMat();
        _inpath = img._inpath;
        _outpath = img._outpath;
    };

    // deconstructor
    ~Image();

    // member functions
    void loadImg(const std::string &path);

    void saveImg(const std::string &path);

    void showImg();

    std::string get_inpath();

    void set_outpath(std::string path);

    void printImageInfo();

    inline cv::Mat GetMat(void) const { return _img.clone(); }

    void determineDataType();

private:
    ImageFormat determineFormat(const std::string &path); // 根据文件后缀名判断图片格式

    std::string formatToString(const ImageFormat &format) const; // 根据图片格式返回对应的字符串

    std::string dataTypeToString(const DataType &datatype) const; // 根据图片数据类型返回对应的字符串

    void calculateStatistics(); // 计算图片的统计信息

    // member variables
private:
    cv::Mat _img;
    std::string _inpath;
    std::string _outpath;
    std::string _imgname;
    std::string _dir;

    ImageFormat _format;
    DataType _datatype;

    long _file_size;
    bool _isLoaded;

    // statistics members
    cv::Scalar _mean;
    cv::Scalar _stddev;
    cv::Mat _histogram;
};
#endif // __IMAGE_H__