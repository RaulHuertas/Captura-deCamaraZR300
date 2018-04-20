#include <vector>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <QCoreApplication>
#include <stdlib.h>
#include <cstdint>
#include <stdio.h>
#include <iostream>
#include <array>
#include <QDateTime>


using namespace std;
#pragma pack(push, 1)
struct rgb_pixel
{
    uint8_t r,g,b;
};
#pragma pack(pop)

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

constexpr int ancho = 640;
constexpr int alto = 480;
constexpr int oneChannelFrameSize =  ancho*alto;
constexpr int rgbSize =  ancho*alto*3;

int mainOLD(int argc, char *argv[])
{
//    rs2::context ctx;
//    auto listOfDevs = ctx.query_devices();
//    if(listOfDevs.size() == 0) {
//        std::cout<<"Dispositivio no detectado"<<endl;
//        return 0;
//    }
//    auto dev = listOfDevs.front();


    rs2::colorizer c;
//    rs2::config cfg;
//    cfg.enable_stream(RS2_STREAM_DEPTH);
//    cfg.enable_stream(RS2_STREAM_FISHEYE);
//    cfg.enable_stream(RS2_STREAM_INFRARED, 1);
//    cfg.enable_stream(RS2_STREAM_INFRARED, 2);
    rs2::pipeline pipe;
//    rs2::pipeline_profile selection = pipe.start(cfg);
    rs2::pipeline_profile selection = pipe.start();
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        //depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        //depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }

    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>();
    auto resolution_depth = std::make_pair(depth_stream.width(), depth_stream.height());
    auto fisheye_stream = selection.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>();

    auto depthIntrinsics = depth_stream.get_intrinsics();
    auto fisheyeIntrinsics = fisheye_stream.get_intrinsics();
    auto convExtrinsics = depth_stream.get_extrinsics_to(fisheye_stream);


    std::cout<<"depthIntrinsics.width: "<<depthIntrinsics.width<<endl;
    std::cout<<"depthIntrinsics.height: "<<depthIntrinsics.height<<endl;



    std::array<std::uint8_t, oneChannelFrameSize> buffer_Depth;
    std::array<std::uint8_t, rgbSize> buffer_Color;
    std::array<std::uint8_t, oneChannelFrameSize> buffer_Fisheye;
    std::array<std::uint8_t, oneChannelFrameSize> buffer_Infrared;

    bool continuar = true;
    qulonglong startTime = QDateTime::currentMSecsSinceEpoch();
    float escala = depth_sensor.get_depth_scale();
    cout<<"Depth Scale: "<<escala<<endl;
    std::uint16_t rangoMinmo = (0.55/escala);
    float minPixCoords[2];
    float maxPixCoords[2];
    float minDepthCameraCoords[3];
    float minFisheyeCameraCoords[3];
    float minFisheyePixCoords[2];
    while( continuar ){
        qulonglong actualTime = QDateTime::currentMSecsSinceEpoch();
        if((actualTime-startTime)>1000){
            break;
        }
        rs2::frameset frames = pipe.wait_for_frames();
        //auto profundidad = (std::uint16_t*)dev.get_frame_data(rs::stream::depth);
        auto depth_NewFrame = frames.get_depth_frame();
        if(!depth_NewFrame){
            continue;
        }
        auto profundidad =  (std::uint16_t*)depth_NewFrame.get_data();
        if(profundidad!=nullptr){
            std::uint16_t minDepth = 65535;
            std::uint16_t maxDepth = 0;
            bool minFoundQ = false;
            float minRealDepth;
            for(int h = 0; h<alto; h++){
                for(int w = 0; w<ancho; w++){
                    const auto& thisDepth = profundidad[h*ancho+w];
                    if(thisDepth==0){
                        continue;
                    }
                    if(thisDepth<rangoMinmo){
                        continue;
                    }
                    if(minDepth>thisDepth){
                        minDepth = thisDepth;
                        minPixCoords[0] = w;
                        minPixCoords[1] = h;
                        minFoundQ = true;
                    }
                    if(maxDepth<thisDepth){
                        maxDepth = thisDepth;
                        maxPixCoords[0] = w;
                        maxPixCoords[1] = h;
                    }
                }
            }


            if(minFoundQ){
                minRealDepth = minDepth*escala;
                std::cout<<"Minima profundidad: "<<minRealDepth<<"m."<<endl;


//                rs_deproject_pixel_to_point(minDepthCameraCoords, &depthIntrinsics, minPixCoords, minRealDepth);
//                rs_transform_point_to_point(minFisheyeCameraCoords, &convExtrinsics, minDepthCameraCoords);
//                rs_project_point_to_pixel(minFisheyePixCoords, &fisheyeIntrinsics, minFisheyeCameraCoords);

                rs2_deproject_pixel_to_point(minDepthCameraCoords, &depthIntrinsics, minPixCoords, minRealDepth);
                rs2_transform_point_to_point(minFisheyeCameraCoords, &convExtrinsics, minDepthCameraCoords);
                rs2_project_point_to_pixel(minFisheyePixCoords, &fisheyeIntrinsics, minFisheyeCameraCoords);

                 std::cout<<"Minima profundidad, depthCoords: "<<minPixCoords[0]<<","<<minPixCoords[1]<<endl;
                 std::cout<<"Minima profundidad, fisheyeCoords: "<<minFisheyePixCoords[0]<<","<<minFisheyePixCoords[1]<<endl;

                std::cout<<"Maxima profundidad: "<<minRealDepth<<"m."<<endl;
            }


        }


    }

    pipe.stop();

    return 0;
}
