#ifndef VIDEO_UTILS_HPP
#define VIDEO_UTILS_HPP

#include "opencv2/opencv.hpp"
#include <QString>
void video_start(int f_w, int f_h, int f_fps, int c_w, int c_h, int c_fps);
void video_close();
void video_addFrame_color(cv::Mat& frame);
void video_addFrame_fisheye(cv::Mat& frame);
void video_checkTime(int f_w, int f_h, int f_fps, int c_w, int c_h, int c_fps, qulonglong timestamp);

QString getStorageFolder();


#endif // VIDEO_UTILS_HPP


