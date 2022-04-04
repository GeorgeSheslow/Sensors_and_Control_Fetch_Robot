#include "sensor.h"
#include "ros/ros.h"
// #include "std_msgs/String.h"

// #include "std_msgs/Int32.h"
// #include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/Images.h"

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <math.h>

Sensor::Sensor(ros::NodeHandle n):
    n_(n)
{
    sub1_ = n.subscribe("/head_camera/depth_downsample/image_raw", 1, &Sensor::depthRaw,this); //sub raw down sample
    sub2_ = n.subscribe("base_scan", 1, &Sensor::baseScan,this); //base scan?
    sub3_ = n.subscribe("/head_camera/depth_downsample/points", 1 &Sensor::depthDownPoints,this); //sub to depth 3D point cloud
    sub3_ = n.subscribe("/head_camera/rgb/image_raw", 1, &Sensor::rgbRaw,this); // sub to raw RGB

    pub_ = n.advertise("/image_converter/output_video", 1); //to the robot
    //Global Variables for the color detection initiation defaults
    LowH_ = 40;
    HighH_ = 80;

    LowS_= 0;
    HighS_ = 90;

    LowV_ = 100;
    HighV_ = 255;

    //Global Variables for the default values of the previous cooridnate of the target object
    LastX_ = -1;
    LastY_ = -1;
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

void Sensor::depthRaw(const sensor_msgs::image& msg){

}

void Sensor::baseScan(const sensor_msgs::ImgConstPtr& msg);{

}

void Sensor::depthDownPoints(const sensor_msgs::ImgConstPtr& msg);{

}

void Sensor::rgbRaw(const sensor_msgs::ImgConstPtr& msg);{

}




//image processing
//rqt (Plugin)-> select the camrea -> 3 topics -> xyz/controller
                                        // string name
