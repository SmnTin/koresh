#include <ros/ros.h>

#include "LineDetector.h"

#include "RedLine.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "line_node");

    ros::NodeHandle node;
//    ros::Rate rate(30);
    if(ros::ok() && LineDetector::Instance()->init(&node)) {
        LineDetector::Instance()->registerLineType(new RedLine());
        while (ros::ok()) {
            LineDetector::Instance()->detect();
            cv::waitKey(30);
//            rate.sleep();
            ros::spinOnce();
        }
    }
    LineDetector::Instance()->clean();
}