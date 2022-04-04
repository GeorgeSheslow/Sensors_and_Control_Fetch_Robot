#ifndef SENSOR_H
#define SENSOR_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Images.h"

class Sensor{
public:
    Sensor();

    Sensor(ros::NodeHandle n);

    void depthRaw(const sensor_msgs::);
    void baseScan();
    void depthDownPoints();
    void rgbRaw();
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
    ros::Subscriber sub2_;
    ros::Subscriber sub3_;
    ros::Subscriber sub4_;
    ros::Publisher pub_;

};

#endif // SENSOR_H
