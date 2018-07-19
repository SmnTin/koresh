#include <ros/ros.h>
#include <koresh/NewTwist.h>
#include <sstream>

#include <termios.h>
#include <unistd.h>
#include <assert.h>
#include <iostream>
/*
*	1
*     2 3 4
*/
bool acceleration = false;
double speedDefault = 80; // Vel without acceleration
double accelerate = 2; // Coff for acceleration
double angularCoff = 100; // Coff for angular motion
double angularLimitCoff = 400; // Coff for angular limited motion

const double PI = 3.1415926;

int getch()
{
  	static struct termios oldt, newt;
  	tcgetattr( STDIN_FILENO, &oldt);           
  	newt = oldt;
 	 newt.c_lflag &= ~(ICANON);                       
  	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  
  	int c = getchar();  
  	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  
  	return c;
}
int main(int argc, char **argv){
	ros::init(argc, argv, "joy_node");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<koresh::NewTwist>("control/joystick", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
	while(ros::ok()) {
		koresh::NewTwist msg;
		int in = -1;
		in = getch();
		switch(in){
			case 119: // Вперёд (w)
				msg.linear_vel = speedDefault;
				msg.orient = PI/2;
				break;
			case 97:  // Лево (a)
//				msg.linear_vel = -speedDefault/angularCoff;
				msg.angle_vel = +speedDefault/angularLimitCoff;
				break;
			case 115: // Назад (s)
				msg.linear_vel = speedDefault;
				msg.orient = -PI/2;
				break;
			case 100: // Право (d)
//				msg.linear_vel = speedDefault/angularCoff;
				msg.angle_vel = -speedDefault/angularLimitCoff;
				break;
			case 113: // (q)
				msg.linear_vel = -speedDefault;
				break;
			case 101: // (e)
				msg.linear_vel = speedDefault;
				break;
			case 122: // Ускорение (z)
				acceleration = !acceleration;
				continue;
			case 32:  // Тормоз (пробел)
				msg.angle_vel = 0;
				msg.linear_vel  = 0;
				break;
			default:
				msg.angle_vel = 0;
				msg.linear_vel = 0;
				break;
		}
		if(acceleration){
			msg.linear_vel *= accelerate;
			msg.angle_vel *= accelerate;
		}
		//std::cout << in << std::endl;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}
	return 0;
}

