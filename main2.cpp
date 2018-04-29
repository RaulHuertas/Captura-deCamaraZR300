#include <vector>

#include <network_utils.hpp>
#include <QCoreApplication>
#include <stdlib.h>
#include <cstdint>
#include <stdio.h>
#include <iostream>
#include <array>
#include <QDateTime>
#include <QFileInfo>
#include <thread>
#include <signal.h>
#include "opencv2/opencv.hpp"
#include "video_utils.hpp"
#include <fcntl.h>

using namespace std;
using namespace rs::core;
using namespace rs::utils;
using namespace cv;

#pragma pack(push, 1)
struct rgb_pixel
{
    uint8_t r,g,b;
};
#pragma pack(pop)


volatile sig_atomic_t terminar = 0;
void signal_handler(int signal)
{
    terminar = 1;
}

void errorPrograma(char *msg) {
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[])
{

    QCoreApplication a(argc, argv);

    int indiceWebcam = detectUVCCamera();

    terminar = 0;
    char* hostname;

    int fisheye_sockfd, fisheye_portno;
    struct sockaddr_in fisheye_serveraddr;
    struct hostent* fisheye_server;
    int fisheye_serverlen;

    int color_sockfd, color_portno;
    struct sockaddr_in color_serveraddr;
    struct hostent* color_server;
    int color_serverlen;

    int webcam_sockfd, webcam_portno;
    struct sockaddr_in webcam_serveraddr;
    struct hostent* webcam_server;
    int webcam_serverlen;

    auto execName = QFileInfo(QCoreApplication::applicationFilePath()).fileName();
    if(argc!=5){
        cout<<("Argumentos del programa invalidos\n");
        cout<<(QString("Invocarlo como: ")+execName+QString(" <direccionDestino>  <puertoDestinoFISHEYE> <puertoDestinoCOLOR>\n")).toStdString();
        exit(EXIT_FAILURE);
    }
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGKILL, signal_handler);

    hostname = argv[1];
    fisheye_portno = atoi(argv[2]);
    color_portno = atoi(argv[3]);
    webcam_portno = atoi(argv[4]);


    fisheye_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fisheye_sockfd < 0){
        errorPrograma("ERROR abriendo el socket fiesheye");
    }
    fisheye_server = gethostbyname(hostname);
    if(fisheye_server == NULL) {
        fprintf(stderr,"ERROR, no such host as %s\n", hostname);
        exit(0);
    }
    bzero((char *) &fisheye_serveraddr, sizeof(fisheye_serveraddr));
    fisheye_serveraddr.sin_family = AF_INET;
    bcopy((char *)fisheye_server->h_addr,
          (char *)&fisheye_serveraddr.sin_addr.s_addr, fisheye_server->h_length);
    fisheye_serveraddr.sin_port = htons(fisheye_portno);
    fisheye_serverlen = sizeof(fisheye_serveraddr);
    int flags = fcntl(fisheye_sockfd, F_GETFL, 0);
    fcntl(fisheye_sockfd, F_SETFL, flags | O_NONBLOCK);

    color_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (color_sockfd < 0){
        errorPrograma("ERROR abriendo el socket color");
    }
    color_server = gethostbyname(hostname);
    if(color_server == NULL) {
        fprintf(stderr,"ERROR, no such host as %s\n", hostname);
        exit(0);
    }
    bzero((char *) &color_serveraddr, sizeof(color_serveraddr));
    color_serveraddr.sin_family = AF_INET;
    bcopy((char *)color_server->h_addr,
          (char *)&color_serveraddr.sin_addr.s_addr, color_server->h_length);
    color_serveraddr.sin_port = htons(color_portno);
    color_serverlen = sizeof(color_serveraddr);
    flags = fcntl(color_sockfd, F_GETFL, 0);
    fcntl(color_sockfd, F_SETFL, flags | O_NONBLOCK);


    webcam_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (webcam_sockfd < 0){
        errorPrograma("ERROR abriendo el socket webcam");
    }
    webcam_server = gethostbyname(hostname);
    if(webcam_server == NULL) {
        fprintf(stderr,"ERROR, no such host as %s\n", hostname);
        exit(0);
    }
    bzero((char *) &webcam_serveraddr, sizeof(webcam_serveraddr));
    webcam_serveraddr.sin_family = AF_INET;
    bcopy((char *)webcam_server->h_addr,
          (char *)&webcam_serveraddr.sin_addr.s_addr, webcam_server->h_length);
    webcam_serveraddr.sin_port = htons(webcam_portno);
    webcam_serverlen = sizeof(webcam_serveraddr);
    flags = fcntl(webcam_sockfd, F_GETFL, 0);
    fcntl(webcam_sockfd, F_SETFL, flags | O_NONBLOCK);

    rs::context ctx;
    if(ctx.get_device_count() == 0){
        cout<<"Dispositivo no detectado"<<endl;
        exit(0);
    }


    rs::device & dev = *ctx.get_device(0);
    dev.enable_stream(rs::stream::depth, rs::preset::highest_framerate);
    dev.enable_stream(rs::stream::color, rs::preset::highest_framerate);
    dev.enable_stream(rs::stream::fisheye, rs::preset::highest_framerate);
    dev.start();

    qulonglong startTime = QDateTime::currentMSecsSinceEpoch();
    auto depthIntrinsics = ( dev.get_stream_intrinsics(rs::stream::depth) );
    auto depthIntrinsics2 = rs::utils::convert_intrinsics( dev.get_stream_intrinsics(rs::stream::depth) );
    auto colorIntrinsics2 = rs::utils::convert_intrinsics( dev.get_stream_intrinsics(rs::stream::color) );
    auto fisheyeIntrinsics2 = rs::utils::convert_intrinsics( dev.get_stream_intrinsics(rs::stream::fisheye) );

    float fov[2] = { depthIntrinsics.hfov(), depthIntrinsics.vfov() };
    auto depthScale = dev.get_depth_scale();
    std::uint16_t rangoMinmo = (0.55/depthScale);
    cout<<"depthScale:"<<depthScale<<endl;
    cout<<"hfov:"<<fov[0]<<endl;
    cout<<"vfov:"<<fov[1]<<endl;
    cout<<"depthIntrinsics.width:"<<depthIntrinsics.width<<endl;
    cout<<"depthIntrinsics.height:"<<depthIntrinsics.height<<endl;


    if (dev.supports_option(rs::option::r200_emitter_enabled)) // For R200 family
    {
        dev.set_option(rs::option::r200_emitter_enabled, 1.f); // Enable laser emitter
    }
    if (dev.supports_option(rs::option::f200_laser_power)) // For R200 family
    {
        double min, max, step; // Query min and max values:
        dev.get_option_range(rs::option::f200_laser_power, min, max, step);
        dev.set_option(rs::option::f200_laser_power, max);
    }

    if (dev.supports_option(rs::option::color_enable_auto_exposure))
    {
        dev.set_option(rs::option::color_enable_auto_exposure, 1.f);
    }
    if (dev.supports_option(rs::option::color_enable_auto_white_balance))
    {
        dev.set_option(rs::option::color_enable_auto_white_balance, 1.f);
    }
    dev.set_option(rs::option::fisheye_color_auto_exposure, 1.f);
    dev.set_option(rs::option::fisheye_color_auto_exposure_mode, 2);


    int depth_width = dev.get_stream_width(rs::stream::depth);
    int depth_height = dev.get_stream_height(rs::stream::depth);
    auto depth_format = dev.get_stream_format(rs::stream::depth);
    int depth_pixel_size = 2;
    int fisheye_width = dev.get_stream_width(rs::stream::fisheye);
    int fisheye_height = dev.get_stream_height(rs::stream::fisheye);
    auto fisheye_format = dev.get_stream_format(rs::stream::fisheye);
    auto fisheye_fps = dev.get_stream_framerate(rs::stream::fisheye);
    int fisheye_pixel_size = 1;
    int color_width = dev.get_stream_width(rs::stream::color);
    int color_height = dev.get_stream_height(rs::stream::color);
    auto color_format = dev.get_stream_format(rs::stream::color);
    auto color_fps = dev.get_stream_framerate(rs::stream::color);
    int color_pixel_size =  3;

    video_start(fisheye_width, fisheye_height, fisheye_fps, color_width, color_height, color_fps);

    vector<point3dF32> depth_coordinates(1);
    point3dF32& minDepthPixelCoords = depth_coordinates[0];
    vector<pointF32> mapped_color_coordinates(depth_coordinates.size());
    vector<pointF32> mapped_fisheye_coordinates(depth_coordinates.size());

    rs::core::extrinsics extrin = rs::utils::convert_extrinsics(dev.get_extrinsics(rs::stream::depth, rs::stream::color));
    auto projection_ = rs::utils::get_unique_ptr_with_releaser(projection_interface::create_instance(
                                                                   &colorIntrinsics2,
                                                                   &depthIntrinsics2,
                                                                   &extrin
                                                                   ));


    rs::core::extrinsics extrinDtoF = rs::utils::convert_extrinsics(dev.get_extrinsics(rs::stream::depth, rs::stream::fisheye));
    auto projectionDtoF_ = rs::utils::get_unique_ptr_with_releaser(projection_interface::create_instance(
                                                                       &fisheyeIntrinsics2,
                                                                       &depthIntrinsics2,
                                                                       &extrinDtoF
                                                                       ));


    cout<<"DEPTH get_stream_width:"<<dev.get_stream_width(rs::stream::depth)<<endl;
    cout<<"DEPTH get_stream_height:"<<dev.get_stream_height(rs::stream::depth)<<endl;
    cout<<"DEPTH get_stream_framerate:"<<dev.get_stream_framerate(rs::stream::depth)<<endl;
    cout<<"DEPTH get_stream_format:"<<dev.get_stream_format(rs::stream::depth)<<endl;
    cout<<"COLOR get_stream_width:"<<dev.get_stream_width(rs::stream::color)<<endl;
    cout<<"COLOR get_stream_height:"<<dev.get_stream_height(rs::stream::color)<<endl;
    cout<<"COLOR get_stream_framerate:"<<dev.get_stream_framerate(rs::stream::color)<<endl;
    cout<<"COLOR get_stream_format:"<<dev.get_stream_format(rs::stream::color)<<endl;
    cout<<"FISHEYE get_stream_width:"<<dev.get_stream_width(rs::stream::fisheye)<<endl;
    cout<<"FISHEYE get_stream_height:"<<dev.get_stream_height(rs::stream::fisheye)<<endl;
    cout<<"FISHEYE get_stream_framerate:"<<dev.get_stream_framerate(rs::stream::fisheye)<<endl;
    cout<<"FISHEYE get_stream_format:"<<dev.get_stream_format(rs::stream::fisheye)<<endl;



    image_info  depth_info;
    depth_info.width = depth_width;
    depth_info.height = depth_height;
    depth_info.format = rs::utils::convert_pixel_format(depth_format);
    depth_info.pitch = depth_pixel_size * depth_width;
    image_info  fisheye_info;
    fisheye_info.width = fisheye_width;
    fisheye_info.height = fisheye_height;
    fisheye_info.format = rs::utils::convert_pixel_format(fisheye_format);
    fisheye_info.pitch = fisheye_pixel_size * fisheye_width;
    image_info  color_info;
    color_info.width = color_width;
    color_info.height = color_height;
    color_info.format = rs::utils::convert_pixel_format(color_format);
    color_info.pitch = color_pixel_size * color_width;

    bool transmit_fisheye = fisheye_portno!=0;
    bool transmit_color = color_portno!=0;
    bool record_fisheye = true;
    bool record_color = true;

    int tx_fishEye_dly=0;
    int tx_fishEye_dly_ctr=0;
    qulonglong tx_fishEye_lastRex=0;

    int tx_color_dly=0;
    int tx_color_dly_ctr=0;
    qulonglong tx_color_lastRex=0;

    unsigned char rcvBuffer[4];

    qulonglong thisIterationTime = QDateTime::currentMSecsSinceEpoch();
    std::thread t([&]() {
        while (true)
        {
            thisIterationTime = QDateTime::currentMSecsSinceEpoch();

            proc_mutex()->lock();
            if(proc_queue()->size()<=0){
                proc_mutex()->unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(8));
                continue;
            }
            FrameData frameAProcesar;
            frameAProcesar =  proc_queue()->front();
            proc_queue()->pop_front();
            proc_mutex()->unlock();

            auto& depth_image = frameAProcesar.depth_image;
            auto& fisheye_image = frameAProcesar.fisheye_image;
            auto& color_image = frameAProcesar.color_image;

            if(
                    (depth_image==nullptr) ||
                    (fisheye_image==nullptr) ||
                    (color_image==nullptr)
                    //                || (webcam_image->empty())
                    ){
                continue;
            }

            auto newDepthMap = ( std::uint16_t*)depth_image->query_data();
            if(newDepthMap!=nullptr){
                std::uint16_t minDepth = 65535;
                for(int h = 0; h<depth_height; h++){
                    for(int w = 0; w<depth_width; w++){
                        const auto& thisDepth = newDepthMap[h*depth_width+w];
                        if(thisDepth==0){
                            continue;
                        }
                        if(thisDepth<rangoMinmo){
                            continue;
                        }
                        if(minDepth>thisDepth){
                            minDepth = thisDepth;
                            minDepthPixelCoords.x = w;
                            minDepthPixelCoords.y = h;
                            minDepthPixelCoords.z = thisDepth;
                        }
                    }
                }
                bool colorMapped = false;
                bool fisheyeMapped = false;
                if(minDepth!=65535){
                    float minRealDepth = minDepth*depthScale;
                    if(projection_->map_depth_to_color(static_cast<int32_t>(depth_coordinates.size()), depth_coordinates.data(), mapped_color_coordinates.data()) < status_no_error)
                    {
#ifdef QT_DEBUG
                        cerr<<"failed to map the depth coordinates to color" << endl;
#endif //QT_DEBUG
                        colorMapped = false;
                        mapped_color_coordinates[0].x = -1.0f;
                        mapped_color_coordinates[0].y = -1.0f;
                    }else{
                        colorMapped = true;
                    }
                    if(projectionDtoF_->map_depth_to_color(static_cast<int32_t>(depth_coordinates.size()), depth_coordinates.data(), mapped_fisheye_coordinates.data()) < status_no_error)
                    {
#ifdef QT_DEBUG
                        cerr<<"failed to map the depth coordinates to fisheye" << endl;
#endif //QT_DEBUG
                        fisheyeMapped = false;
                        mapped_fisheye_coordinates[0].x = -1.0f;
                        mapped_fisheye_coordinates[0].y = -1.0f;
                    }else{
                        fisheyeMapped = true;
                    }
#ifdef QT_DEBUG
                    //                std::cout<<"Minima profundidad: "<<minRealDepth<<"m."<<endl;
                    //                std::cout<<"COLOR, X: "<<mapped_color_coordinates[0].x<<endl;
                    //                std::cout<<"COLOR, Y: "<<mapped_color_coordinates[0].y<<endl;
                    //                std::cout<<"FISHEYE, X: "<<mapped_fisheye_coordinates[0].x<<endl;
                    //                std::cout<<"FISHEYE, Y: "<<mapped_fisheye_coordinates[0].y<<endl;
#endif //QT_DEBUG
                }
                std::unique_ptr<Mat> fisheyeCVMat = nullptr;
                if(transmit_fisheye && (tx_fishEye_dly_ctr==0) ){
                    //comprimir
                    fisheyeCVMat = std::make_unique<Mat>(Size(fisheye_width, fisheye_height), CV_8U, const_cast<void*>(fisheye_image->query_data()), Mat::AUTO_STEP);
                    Mat& fisheyMat = *fisheyeCVMat.get();
                    //                Mat fisheyMat(Size(fisheye_width, fisheye_height), CV_8U, const_cast<void*>(fisheye_image->query_data()), Mat::AUTO_STEP);
                    std::vector<uchar> compressedBuffer;
                    std::vector<int> compressParams(2);
                    compressParams[0] = cv::IMWRITE_JPEG_QUALITY;
                    compressParams[1] = 25;//default(95) 0-100
                    int fishEyeSendFormat  = (int)fisheye_format;
                    //auto resultCompress = imencode(".jpg", fisheyMat, compressedBuffer, compressParams);
                    auto resultCompress = imencode(".jpg", fisheyMat, compressedBuffer, compressParams);
                    if(!resultCompress){
                        transmitirPorUDP(
                                    fisheye_width, fisheye_height,
                                    fishEyeSendFormat,
                                    fisheye_image->query_data(),
                                    fisheye_width*fisheye_height*1,
                                    minDepth, mapped_fisheye_coordinates[0].x, mapped_fisheye_coordinates[0].y, depthScale,
                                fisheye_sockfd, fisheye_serveraddr, fisheye_serverlen
                                );
                    }else{
                        fishEyeSendFormat+=4;
                        transmitirPorUDP(
                                    fisheye_width, fisheye_height,
                                    fishEyeSendFormat,
                                    compressedBuffer.data(),
                                    compressedBuffer.size(),
                                    minDepth, mapped_fisheye_coordinates[0].x, mapped_fisheye_coordinates[0].y, depthScale,
                                fisheye_sockfd, fisheye_serveraddr, fisheye_serverlen
                                );
                    }

                }

                if(record_fisheye){
                    if(fisheyeCVMat==nullptr){
                        fisheyeCVMat = std::make_unique<Mat>(Size(fisheye_width, fisheye_height), CV_8U, const_cast<void*>(fisheye_image->query_data()), Mat::AUTO_STEP);

                    }
                    video_addFrame_fisheye(*fisheyeCVMat.get());
                }

                //Versi hay comandosderecepcion
                int bytesRecibidos = recv(fisheye_sockfd, &rcvBuffer[0], sizeof(rcvBuffer), 0);
                if(bytesRecibidos==sizeof(rcvBuffer)){
                    if(rcvBuffer[0] == 0xFF){
                        //transmit_fisheye = rcvBuffer[3];
                        tx_fishEye_lastRex = thisIterationTime;
                    }
                }
                if((thisIterationTime-tx_fishEye_lastRex)>500){
                    tx_fishEye_dly = 32;
                }else{
                    tx_fishEye_dly = 1;
                }
                tx_fishEye_dly_ctr++;
                tx_fishEye_dly_ctr%=tx_fishEye_dly;
#ifdef QT_DEBUG
                cout<<"tx_fishEye_dly: "<<tx_fishEye_dly<<endl;
#endif //QT_DEBUG

                //COLOR
                std::unique_ptr<Mat> colorCVMat = nullptr;
                if(transmit_color && (tx_color_dly_ctr==0)){
                    colorCVMat = std::make_unique<Mat>(Size(color_width, color_height), CV_8UC3, const_cast<void*>(color_image->query_data()), Mat::AUTO_STEP);
                    Mat& colorMat = *colorCVMat.get();
                    cv::cvtColor(*colorCVMat.get(), *colorCVMat.get(), cv::COLOR_BGR2RGB);
                    std::vector<uchar> compressedBuffer;
                    std::vector<int> compressParams(2);
                    compressParams[0] = cv::IMWRITE_JPEG_QUALITY;
                    compressParams[1] = 25;//default(95) 0-100
                    int colorSendFormat  = (int)color_format;
                    auto resultCompress = imencode(".jpg", colorMat, compressedBuffer, compressParams);
                    if(!resultCompress){
                        transmitirPorUDP(
                                    color_width, color_height,
                                    colorSendFormat,
                                    color_image->query_data(),
                                    color_width*color_height*3,
                                    minDepth, mapped_fisheye_coordinates[0].x, mapped_color_coordinates[0].y, depthScale,
                                color_sockfd, color_serveraddr, color_serverlen
                                );
                    }else{
                        colorSendFormat+=4;
                        transmitirPorUDP(
                                    color_width, color_height,
                                    colorSendFormat,
                                    compressedBuffer.data(),
                                    compressedBuffer.size(),
                                    minDepth, mapped_fisheye_coordinates[0].x, mapped_color_coordinates[0].y, depthScale,
                                color_sockfd, color_serveraddr, color_serverlen
                                );
                    }
                }
                if(record_color){
                    if(colorCVMat==nullptr){
                        colorCVMat = std::make_unique<Mat>(Size(color_width, color_height), CV_8UC3, const_cast<void*>(color_image->query_data()), Mat::AUTO_STEP);
                        cv::cvtColor(*colorCVMat.get(), *colorCVMat.get(), cv::COLOR_BGR2RGB);
                    }
                    video_addFrame_color(*colorCVMat.get());
                }
                bytesRecibidos = recv(color_sockfd, &rcvBuffer[0], sizeof(rcvBuffer), 0);
                if(bytesRecibidos==sizeof(rcvBuffer)){
                    if(rcvBuffer[0] == 0xFF){
                        //transmit_color = rcvBuffer[3];
                        tx_color_lastRex = thisIterationTime;
                    }
                }
                if((thisIterationTime-tx_color_lastRex)>500){
                    tx_color_dly = 32;
                }else{
                    tx_color_dly = 1;
                }
                tx_color_dly_ctr++;
                tx_color_dly_ctr%=tx_color_dly;
#ifdef QT_DEBUG
                cout<<"tx_color_dly: "<<tx_color_dly<<endl;
#endif //QT_DEBUG
            }

            video_checkTime(fisheye_width, fisheye_height, fisheye_fps, color_width, color_height, color_fps, QDateTime::currentMSecsSinceEpoch());

        }
    });


    while(terminar==0){

#ifdef QT_DEBUG
        auto loopStartTime = QDateTime::currentMSecsSinceEpoch();
#endif //QT_DEBUG
        dev.wait_for_frames();


        CuadroRef depth_image = get_shared_ptr_with_releaser(image_interface::create_instance_from_raw_data(
                                                                 &depth_info,
        {dev.get_frame_data(rs::stream::depth), nullptr},
                                                                 stream_type::depth,
                                                                 image_interface::flag::any,
                                                                 dev.get_frame_timestamp(rs::stream::depth),
                                                                 dev.get_frame_number(rs::stream::depth))
                                                             );
        CuadroRef fisheye_image = get_shared_ptr_with_releaser(image_interface::create_instance_from_raw_data(
                                                                   &fisheye_info,
        {dev.get_frame_data(rs::stream::fisheye), nullptr},
                                                                   stream_type::fisheye,
                                                                   image_interface::flag::any,
                                                                   dev.get_frame_timestamp(rs::stream::fisheye),
                                                                   dev.get_frame_number(rs::stream::fisheye))
                                                               );
        CuadroRef color_image = get_shared_ptr_with_releaser(        image_interface::create_instance_from_raw_data(
                                                                         &color_info,
        {dev.get_frame_data(rs::stream::color), nullptr},
                                                                         stream_type::color,
                                                                         image_interface::flag::any,
                                                                         dev.get_frame_timestamp(rs::stream::color),
                                                                         dev.get_frame_number(rs::stream::color))
                                                                     );


        CuadroCamaraWeb webcamNewFrame;
        //webcamStream >> *webcamNewFrame.get();

        std::lock_guard<std::mutex> lock(*proc_mutex());
        proc_queue()->emplace_back(depth_image, fisheye_image, color_image, webcamNewFrame);

#ifdef QT_DEBUG
        //auto loopStartTime = QDateTime::currentMSecsSinceEpoch();
        auto loopEndTime = QDateTime::currentMSecsSinceEpoch();
        //cout<<"Loop time: "<<(loopEndTime-loopStartTime)<<endl;
#endif //QT_DEBUG
    }

    video_close();
#ifdef QT_DEBUG
    std::cout<<"Fin de la transmisión"<<endl;
#endif //QT_DEBUG

    dev.stop();

    return 0;

}
