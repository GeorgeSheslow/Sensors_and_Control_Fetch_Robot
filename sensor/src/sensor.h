#ifndef SENSOR_H
#define SENSOR_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"


class Sensor{
public:
    Sensor();

    Sensor(ros::NodeHandle n);

    void depthRaw(const sensor_msgs::Images& msgImg);
    //void baseScan(const sensor_msgs::Images& msgImg);
    //void depthDownPoints(const sensor_msgs::Images& msgDP);
    void rgbRaw(const sensor_msgs::Images& msgImg);
protected:
    ros::Nodehandle n_;

private:
    //Global Variables for the color detection initiation defaults
    int LowH_;
    int HighH_;

    int LowS_;
    int HighS_;

    int LowV_;
    int HighV_;

    //Global Variables for the default values of the previous cooridnate of the target object
    int LastX_;
    int LastY_;

    ros::Subscriber sub1_;
    //ros::Subscriber sub2_;
    //ros::Subscriber sub3_;
    ros::Subscriber sub4_;
    ros::Publisher pub1_;
    ros::Publisher pub2_

};

#endif // SENSOR_H
