#include "sensor.h"
#include "ros/ros.h"
// #include "std_msgs/String.h"

// #include "std_msgs/Int32.h"
// #include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video.hpp>

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
    //sub2_ = n.subscribe("base_scan", 1, &Sensor::baseScan,this); //base scan?
    //sub3_ = n.subscribe("/head_camera/depth_downsample/points", 1 &Sensor::depthDownPoints,this); //sub to depth 3D point cloud
    sub3_ = n.subscribe("/head_camera/rgb/image_raw", 1, &Sensor::rgbRaw,this); // sub to raw RGB

    pub1_ = n.advertise<geometry_msgs::Point>("rgbData", 1); //to the robot
    pub2_ = n.advertise<int>("depthData", 1);
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

void Sensor::depthRaw(const sensor_msgs::Image& msgImg){
    geometry_msgs::Point 
    
    
    pub1_.publish()
}

// void Sensor::baseScan(const sensor_msgs::Images& msgImg);{

// }

// //not too sure if this is correct
// void Sensor::depthDownPoints(const sensor_msgs::PointCloud& msgDP);{

// }

void Sensor::rgbRaw(const sensor_msgs::Images& msgImg);{
    rgbData.clear();

    geometry_msgs::Point rgbData;
    geometry_msgs::Point centerXY;
    rgbData.x = msgImg.width;
    rbgdata.y = msgImg.height;

    double rbg_x_center = rgbData.x/2
    double rbg_y_center = rgbData.y/2

    centerXY.x = rbg_x_center;
    centerXY.y = rbg_y_center;

    pub1_.publish(centterXY)
}




//image processing
//rqt (Plugin)-> select the camrea -> 3 topics -> xyz/controller
                                        // string name
