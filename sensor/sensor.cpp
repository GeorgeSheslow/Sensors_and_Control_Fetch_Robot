#include "sensor.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

Sensor::Sensor()
{

}

/*
/base_scan
/head_camera/depth_downsample/camera_info
/head_camera/depth_downsample/image_raw
/head_camera/depth_downsample/image_raw/compressed
/head_camera/depth_downsample/image_raw/compressed/parameter_descriptions
/head_camera/depth_downsample/image_raw/compressed/parameter_updates
/head_camera/depth_downsample/image_raw/compressedDepth
/head_camera/depth_downsample/image_raw/compressedDepth/parameter_descriptions
/head_camera/depth_downsample/image_raw/compressedDepth/parameter_updates
/head_camera/depth_downsample/image_raw/theora
/head_camera/depth_downsample/image_raw/theora/parameter_descriptions
/head_camera/depth_downsample/image_raw/theora/parameter_updates
/head_camera/depth_downsample/points
/head_camera/depth_registered/camera_info
/head_camera/depth_registered/image_raw
/head_camera/depth_registered/points
/head_camera/head_camera_nodelet_manager/bond
/head_camera/parameter_descriptions
/head_camera/parameter_updates
/head_camera/rgb/camera_info
/head_camera/rgb/image_raw
/head_camera/rgb/image_raw/compressed
/head_camera/rgb/image_raw/compressed/parameter_descriptions
/head_camera/rgb/image_raw/compressed/parameter_updates
/head_camera/rgb/image_raw/compressedDepth
/head_camera/rgb/image_raw/compressedDepth/parameter_descriptions
/head_camera/rgb/image_raw/compressedDepth/parameter_updates
/head_camera/rgb/image_raw/theora
/head_camera/rgb/image_raw/theora/parameter_descriptions
/head_camera/rgb/image_raw/theora/parameter_updates
*/

int main(int argc, char **argv){
    ros::init(argc, **argv, "listener");
    ros::nodehandle n;
    ros::Rate loop_rate(10);

    ros::Subcriber sub1 = n.subscribe("/head_camera/depth_downsample/image_raw", 1000); //sub raw down sample
    ros::Subcriber sub2 = n.subscribe("base_scan", 1000); //base scan?
    ros::Subcriber sub3 = n.subscribe("/head_camera/depth_downsample/points", 1000); //sub to depth 3D point cloud
    ros::Subcriber sub3 = n.subscribe("/head_camera/rgb/image_raw", 1000); // sub to raw RGB

    ros::Publisher pub = n.advertisee<std_msgs::String>("robotMsG", 1000); //to the robot

    ros::spin();
    return 0;
}


