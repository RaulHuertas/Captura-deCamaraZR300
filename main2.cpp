#include <vector>

//#include <librealsense2/rs.hpp>
//#include <librealsense2/rsutil.h>
#include <librealsense/rs.hpp>
#include <QCoreApplication>
#include <stdlib.h>
#include <cstdint>
#include <stdio.h>
#include <iostream>
#include <array>
#include <QDateTime>
#include "rs/utils/librealsense_conversion_utils.h"
#include "rs_sdk.h"

using namespace std;
using namespace rs::core;
using namespace rs::utils;

#pragma pack(push, 1)
struct rgb_pixel
{
    uint8_t r,g,b;
};
#pragma pack(pop)

//rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

constexpr int ancho = 640;
constexpr int alto = 480;
constexpr int oneChannelFrameSize =  ancho*alto;
constexpr int rgbSize =  ancho*alto*3;

int main(int argc, char *argv[])
{

    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);
    //dev.enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::highest_framerate);
    dev.enable_stream(rs::stream::fisheye, rs::preset::highest_framerate);
    //try { dev.enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch(...) {}
    dev.start();

    qulonglong startTime = QDateTime::currentMSecsSinceEpoch();
    rs::extrinsics extrinsicsDepth_To_Fisheye = dev.get_extrinsics(rs::stream::depth, rs::stream::fisheye);
    rs::extrinsics extrinsicsDepth_To_Color = dev.get_extrinsics(rs::stream::depth, rs::stream::fisheye);
    auto depthIntrinsics = ( dev.get_stream_intrinsics(rs::stream::depth) );
    auto colorIntrinsics = ( dev.get_stream_intrinsics(rs::stream::color) );
    auto depthIntrinsics2 = rs::utils::convert_intrinsics( dev.get_stream_intrinsics(rs::stream::depth) );
    auto colorIntrinsics2 = rs::utils::convert_intrinsics( dev.get_stream_intrinsics(rs::stream::color) );
    rs::intrinsics fisheyeIntrinsics = dev.get_stream_intrinsics(rs::stream::fisheye);

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



    int depth_width = dev.get_stream_width(rs::stream::depth);
    int depth_height = dev.get_stream_height(rs::stream::depth);

    float minDepthPixelCoords[2];
    float minDepthCameraCoords[3];
    float minFisheyeCameraCoords[3];
    float minFisheyePixelCoords[2];
    float minColorCameraCoords[3];
    float minColorPixelCoords[2];

    rs::core::extrinsics extrin = rs::utils::convert_extrinsics(dev.get_extrinsics(rs::stream::depth, rs::stream::color));
    auto projection_ = rs::utils::get_unique_ptr_with_releaser(projection_interface::create_instance(
                                        &colorIntrinsics2,
                                        &depthIntrinsics2,
                                        &extrin
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

    while(true){
        qulonglong actualTime = QDateTime::currentMSecsSinceEpoch();
        if((actualTime-startTime)>20000){
            break;
        }
        dev.wait_for_frames();
        auto newDepthMap = ( std::uint16_t*)dev.get_frame_data(rs::stream::depth);
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
                        minDepthPixelCoords[0] = w;
                        minDepthPixelCoords[1] = h;
                    }
                }
            }
            if(minDepth){
                float minRealDepth = minDepth*depthScale;

//                rs_deproject_pixel_to_point(minDepthCameraCoords, &depthIntrinsics, minDepthPixelCoords, minRealDepth);
//                rs_transform_point_to_point(minFisheyeCameraCoords, &extrinsicsDepth_To_Fisheye, minDepthCameraCoords);
//                rs_transform_point_to_point(minColorCameraCoords, &extrinsicsDepth_To_Color, minDepthCameraCoords);
//                rs_project_point_to_pixel(minFisheyePixelCoords, &fisheyeIntrinsics, minFisheyeCameraCoords);
//                rs_project_point_to_pixel(minColorPixelCoords, &colorIntrinsics, minColorCameraCoords);

//                std::cout<<"Minima profundidad: "<<minRealDepth<<"m."<<endl;
//                std::cout<<"Minima profundidad, DEPTH X: "<<minDepthPixelCoords[0]<<endl;
//                std::cout<<"Minima profundidad, DEPTH Y: "<<minDepthPixelCoords[1]<<endl;
//                std::cout<<"Minima profundidad, FISHEYE X: "<<minFisheyePixelCoords[0]<<endl;
//                std::cout<<"Minima profundidad, FISHEYE Y: "<<minFisheyePixelCoords[1]<<endl;
//                std::cout<<"Minima profundidad, COLOR X: "<<minColorPixelCoords[0]<<endl;
//                std::cout<<"Minima profundidad, COLOR Y: "<<minColorPixelCoords[1]<<endl;

            }


        }




    }

    dev.stop();
    return 0;
}
