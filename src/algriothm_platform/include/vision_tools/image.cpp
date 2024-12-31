#include "image.h"
#include <iostream>

Image::Image(const std::string &in_path, const std::string &out_path)
{
    _inpath = in_path;
    _outpath = out_path;
    _imgname = common_tools::get_filename(in_path);
    _dir = common_tools::get_currentpath(in_path);
    _file_size = common_tools::getFileSize(in_path);
    _format = determineFormat(in_path);
    loadImg(in_path);
    determineDataType();
    calculateStatistics();
    printImageInfo();
}

Image::Image(const std::string &in_path)
{
    _inpath = in_path;
    _outpath = "output.jpg";
    _imgname = common_tools::get_filename(in_path);
    _dir = common_tools::get_currentpath(in_path);
    _file_size = common_tools::getFileSize(in_path);
    _format = determineFormat(in_path);
    loadImg(in_path);
    determineDataType();
    calculateStatistics();
    printImageInfo();
}

Image::~Image() {}

void Image::loadImg(const std::string &path)
{
    _img = cv::imread(path, cv::IMREAD_COLOR);
    if(_img.empty())
    {
        // std::cerr << "Error: Could not load image at " << path << std::endl;
        LOGFMTE("Error: Could not load image at %s", path.c_str());
        return;
    }
    _isLoaded = true;
}

void Image::saveImg(const std::string &path)
{
    if(!_isLoaded)
    {
        // std::cerr << "Error: Image not loaded" << std::endl;
        LOGW("Error: Image not loaded");
        return;
    } else if(_img.empty())
    {
        // std::cerr << "Error: Image is empty" << std::endl;
        LOGW("Error: Image is empty");
        return;
    }
    cv::imwrite(path, _img);
}

void Image::showImg()
{
    if(!_isLoaded)
    {
        // std::cerr << "Error: Image not loaded" << std::endl;
        LOGW("Error: Image not loaded");
        return;
    } else if(_img.empty())
    {
        // std::cerr << "Error: Image is empty" << std::endl;
        LOGW("Error: Image is empty");
        return;
    }
    cv::imshow(_imgname, _img);
    cv::waitKey(0);
}

std::string Image::get_inpath() { return _inpath; }

void Image::set_outpath(std::string path) { _outpath = path; }

void Image::printImageInfo()
{
    if(!_isLoaded)
    {
        LOGE("Error: Image not loaded");
        // std::cerr << "Error: Image not loaded" << std::endl;
        return;
    } else if(_img.empty())
    {
        LOGE("Error: Image is empty");
        // std::cerr << "Error: Image is empty" << std::endl;
        return;
    }

    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);
    LOGD("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    LOGD("Image Info:");
    LOGFMTD("Image Name: %s", _imgname.c_str());
    LOGFMTD("Directory: %s", _dir.c_str());
    LOGFMTD("Format: %s", formatToString(_format).c_str());
    LOGFMTD("File Size: %f Mb", static_cast<double>(_file_size / (1024.0 * 1024.0)));
    LOGFMTD("Image Size: [%d x %d]", _img.size().width, _img.size().height);
    LOGFMTD("Image DataType: %s", dataTypeToString(_datatype).c_str());
    if(_datatype == DataType::Grayscale)
    {
        LOGFMTD("Mean: %f", _mean[0]);
        LOGFMTD("Stddev: %f", _stddev[0]);
    } else if(_datatype == DataType::BGR)
    {
        LOGFMTD("Mean B: %f, G: %f, R: %f", _mean[0], _mean[1], _mean[2]);
        LOGFMTD("Stddev B: %f, G: %f, R: %f", _stddev[0], _stddev[1], _stddev[2]);
    } else if(_datatype == DataType::BGRA)
    {
        LOGFMTD("Mean B: %f, G: %f, R: %f, A: %f", _mean[0], _mean[1], _mean[2], _mean[3]);
        LOGFMTD("Stddev B: %f, G: %f, R: %f, A: %f", _stddev[0], _stddev[1], _stddev[2], _stddev[3]);
    } else
    {
        LOGFMTE("Unsupported image channels: %d", _img.channels());
    }
    LOGD("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

    std::cout << ANSI_COLOR_PURPLE_BOLD << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "  Image Info  :" << std::endl;
    std::cout << "  Image Name  :" << _imgname << std::endl;
    std::cout << "  Directory   :" << _dir << std::endl;
    std::cout << "  Format      :" << formatToString(_format) << std::endl;
    std::cout << "  File Size   :" << static_cast<double>(_file_size / (1024.0 * 1024.0)) << std::endl;
    std::cout << "  Image Size  :" << _img.size() << std::endl;
    std::cout << "  DataType    :" << dataTypeToString(_datatype) << std::endl;
    if(_datatype == DataType::Grayscale)
    {
        std::cout << "  Mean        :" << _mean[0] << std::endl;
        std::cout << "  Stddev      :" << _stddev[0] << std::endl;
    } else if(_datatype == DataType::BGR)
    {
        std::cout << "  Mean B      :" << _mean[0] << std::endl;
        std::cout << "  Mean G      :" << _mean[1] << std::endl;
        std::cout << "  Mean R      :" << _mean[2] << std::endl;
        std::cout << "  Stddev B    :" << _stddev[0] << std::endl;
        std::cout << "  Stddev G    :" << _stddev[1] << std::endl;
        std::cout << "  Stddev R    :" << _stddev[2] << std::endl;
    } else if(_datatype == DataType::BGRA)
    {
        std::cout << "  Mean B      :" << _mean[0] << std::endl;
        std::cout << "  Mean G      :" << _mean[1] << std::endl;
        std::cout << "  Mean R      :" << _mean[2] << std::endl;
        std::cout << "  Mean A      :" << _mean[3] << std::endl;
        std::cout << "  Stddev B    :" << _stddev[0] << std::endl;
        std::cout << "  Stddev G    :" << _stddev[1] << std::endl;
        std::cout << "  Stddev R    :" << _stddev[2] << std::endl;
        std::cout << "  Stddev A    :" << _stddev[3] << std::endl;
    } else
    {
        std::cout << "Unsupported image channels: " << _img.channels() << std::endl;
    }
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << ANSI_COLOR_RESET << std::endl;

    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);
}

void Image::determineDataType()
{
    if(!_isLoaded)
    {
        LOGE("Error: Image not loaded");
        _format = ImageFormat::Unknown;
        _datatype = DataType::Unknown;
        return;
    } else if(_img.empty())
    {
        _format = ImageFormat::Unknown;
        _datatype = DataType::Unknown;
        LOGE("Error: Image is empty");
        return;
    }
    if(_img.channels() == 1)
    {
        _datatype = DataType::Grayscale;
        LOGFMTI("%s's Image data type: Grayscale", _imgname.c_str());
    } else if(_img.channels() == 3)
    {
        _datatype = DataType::BGR;
        LOGFMTI("%s's Image data type: BGR", _imgname.c_str());
    } else if(_img.channels() == 4)
    {
        LOGFMTI("%s's Image data type: BGRA", _imgname.c_str());
    } else
    {
        LOGFMTE("%s's Image data type: Other", _imgname.c_str());
    }
}

ImageFormat Image::determineFormat(const std::string &path)
{
    boost::filesystem::path p(path);
    std::string ext = p.extension().string();
    if(ext == ".png")
    {
        return ImageFormat::PNG;
    } else if(ext == ".jpg")
    {
        return ImageFormat::JPG;
    } else if(ext == ".bmp")
    {
        return ImageFormat::BMP;
    } else if(ext == ".tiff")
    {
        return ImageFormat::TIFF;
    } else if(ext == ".webp")
    {
        return ImageFormat::WEBP;
    } else
    {
        return ImageFormat::Unknown;
    }
}

std::string Image::formatToString(const ImageFormat &format) const
{
    switch(format)
    {
    case ImageFormat::PNG:
        return "PNG";
    case ImageFormat::JPG:
        return "JPG";
    case ImageFormat::BMP:
        return "BMP";
    case ImageFormat::TIFF:
        return "TIFF";
    case ImageFormat::WEBP:
        return "WEBP";
    default:
        return "Unknown";
    }
}

std::string Image::dataTypeToString(const DataType &datatype) const
{
    switch(datatype)
    {
    case DataType::Grayscale:
        return "Grayscale";
    case DataType::BGR:
        return "BGR";
    case DataType::BGRA:
        return "BGRA";
    default:
        return "Unknown";
    }
}

void Image::calculateStatistics()
{
    if(!_isLoaded)
    {
        LOGE("Error: Image not loaded");
        return;
    } else if(_img.empty())
    {
        LOGE("Error: Image is empty");
        return;
    }
    // cv::Scalar _mean, _stddev;
    cv::meanStdDev(_img, _mean, _stddev);

    if(_img.channels() == 1 && _datatype == DataType::Grayscale)
    {
        // 对于单通道图像
        LOGFMTI("_mean: %f", _mean[0]);
        LOGFMTI("_stddev: %f", _stddev[0]);
    } else if(_img.channels() == 3 && _datatype == DataType::BGR)
    {
        // 对于三通道图像（BGR）
        LOGFMTI("_mean B: %f, G: %f, R: %f", _mean[0], _mean[1], _mean[2]);
        LOGFMTI("_stddev B: %f, G: %f, R: %f", _stddev[0], _stddev[1], _stddev[2]);
    } else if(_img.channels() == 4 && _datatype == DataType::BGRA)
    {
        // 对于四通道图像（BGRA）
        LOGFMTI("_mean B: %f, G: %f, R: %f, A: %f", _mean[0], _mean[1], _mean[2], _mean[3]);
        LOGFMTI("_stddev B: %f, G: %f, R: %f, A: %f", _stddev[0], _stddev[1], _stddev[2], _stddev[3]);
    } else
    {
        LOGFMTE("Unsupported image channels: %d", _img.channels());
    }
}