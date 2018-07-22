#include <ros/ros.h>

#include "MovingObstaclesDetector.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "moving_obstacles_detector_node");
    ros::NodeHandle node;

    if(MovingObstaclesDetector::Instance()->init(&node))
        ros::spin();
    return 0;
}