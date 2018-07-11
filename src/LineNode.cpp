#include <ros/ros.h>

#include "LineDetector.h"

#include "RedLine.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "line_node");

    ros::NodeHandle node;

    if(ros::ok() && LineDetector::Instance()->init()) {
        LineDetector::Instance()->registerLineType(new RedLine());
        while (ros::ok()) {
            LineDetector::Instance()->detect();
            cv::waitKey(30);
            ros::spinOnce();
        }
    }
    LineDetector::Instance()->clean();
}