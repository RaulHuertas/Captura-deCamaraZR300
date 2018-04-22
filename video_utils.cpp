#include "video_utils.hpp"
#include <QDateTime>

using namespace cv;;

static std::unique_ptr<cv::VideoWriter> colorMat;
static std::unique_ptr<cv::VideoWriter> fisheyeMat;
static std::unique_ptr<cv::VideoCapture> v_cap;

qulonglong startTimeStamp = 0;

void restartStream(int f_w, int f_h, int f_fps, int c_w, int c_h, int c_fps, qulonglong timestamp){

    auto tm_str = QString::number(timestamp);
    auto strgFolder = getStorageFolder();
    QString color_name_str = strgFolder+"/"+QString("color_")+tm_str+QString(".avi");
    QString fisheye_name_str = strgFolder+"/"+QString("fisheye_")+tm_str+QString(".avi");
    fisheyeMat = std::make_unique<VideoWriter>();
    colorMat = std::make_unique<VideoWriter>();
    fisheyeMat->open(fisheye_name_str.toStdString(), VideoWriter::fourcc('X','2','6','4'), f_fps, cv::Size(f_w,f_h), false);
    colorMat->open(color_name_str.toStdString(), VideoWriter::fourcc('X','2','6','4'), c_fps, cv::Size(c_w,c_h), true);
}

void video_checkTime(int f_w, int f_h, int f_fps, int c_w, int c_h, int c_fps, qulonglong timestamp){
    //video de maximo dos minutos
    if((timestamp-startTimeStamp)>2*60*1000){
        restartStream(f_w, f_h, f_fps, c_w, c_h, c_fps, timestamp);
        startTimeStamp = timestamp;
    }
}

void video_start(
     int f_w, int f_h, int f_fps, int c_w, int c_h, int c_fps
){
    v_cap = std::make_unique<cv::VideoCapture>(-1);
    auto timestamp = QDateTime::currentMSecsSinceEpoch();
    video_checkTime(f_w, f_h, f_fps, c_w, c_h, c_fps, timestamp);
}

void video_close(){
    colorMat = nullptr;
    fisheyeMat = nullptr;
}

void video_addFrame_color(cv::Mat& frame){
    colorMat->write(frame);
}

void video_addFrame_fisheye(cv::Mat& frame){
    fisheyeMat->write(frame);
}

