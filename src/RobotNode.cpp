#include "ros/ros.h"

#include "Car.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_node");

	ros::NodeHandle node;

	ros::Rate rate(30);
	if(ros::ok() && Car::Instance()->init(&node)) {
		while (ros::ok() && Car::Instance()->isRunning()) {
			Car::Instance()->update();
			Car::Instance()->applyChanges();
			ros::spinOnce();
			rate.sleep();
		}
	}
}
