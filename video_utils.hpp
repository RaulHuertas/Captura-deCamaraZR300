#ifndef VIDEO_UTILS_HPP
#define VIDEO_UTILS_HPP

#include <mutex>
#include "opencv2/opencv.hpp"
#include <QString>
#include <deque>
#include <vector>
#include "rs/utils/librealsense_conversion_utils.h"
#include "rs_sdk.h"

typedef  std::shared_ptr<rs::core::image_interface> CuadroRef;
typedef std::shared_ptr<cv::Mat> CuadroCamaraWeb;
struct FrameData{
    CuadroRef depth_image;
    CuadroRef fisheye_image;
    CuadroRef color_image;
    CuadroCamaraWeb imagenExtra;
    FrameData() = default;
    FrameData(
        CuadroRef& new_depth_image,
        CuadroRef& new_fisheye_image,
        CuadroRef& new_color_image,
        CuadroCamaraWeb& new_imagenExtra
    ){
        depth_image = new_depth_image;
        fisheye_image = new_fisheye_image;
        color_image = new_color_image;
        imagenExtra = new_imagenExtra;
    }
    FrameData& operator =(FrameData&& other){
        depth_image = std::move(other.depth_image);
        fisheye_image = std::move(other.fisheye_image);
        color_image = std::move(other.color_image);
        imagenExtra = std::move(other.imagenExtra);
        return *this;
    }
    FrameData& operator =(FrameData& other){
        depth_image = (other.depth_image);
        fisheye_image = (other.fisheye_image);
        color_image = (other.color_image);
        imagenExtra = (other.imagenExtra);
        return *this;
    }
};

typedef std::deque<FrameData> QueueCuadros;
std::mutex* proc_mutex();
QueueCuadros* proc_queue();


void video_start(int f_w, int f_h, int f_fps, int c_w, int c_h, int c_fps);
void video_close();
void video_addFrame_color(cv::Mat& frame);
void video_addFrame_fisheye(cv::Mat& frame);
void video_addFrame_webcam(cv::Mat& frame);
void video_checkTime(int f_w, int f_h, int f_fps, int c_w, int c_h, int c_fps, qulonglong timestamp);
int detectUVCCamera();


QString getStorageFolder();


#endif // VIDEO_UTILS_HPP


