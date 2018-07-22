#include <ros/ros.h>

#include "LineDetector.h"

#include "BlueLine.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "line_node");

    ros::NodeHandle node;
    //ros::Rate rate(30);
    if(ros::ok() && LineDetector::Instance()->init(&node)) {
        LineDetector::Instance()->registerLineType(new BlueLine());
        while (ros::ok()) {
//            if(LineDetector::Instance()->captureOn)
//                LineDetector::Instance()->detect();
            cv::waitKey(30);
            //rate.sleep();
            ros::spinOnce();
        }
    }
    LineDetector::Instance()->clean();
}