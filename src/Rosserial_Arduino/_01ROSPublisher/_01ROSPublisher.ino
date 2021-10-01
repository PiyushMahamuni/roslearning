#include <ros.h>
#include <geometry_msgs/Twist.h>

// CONSTANTS
const char* VEL_TOPIC {"turtle1/cmd_vel"};
const byte linePin {A0};
const byte anglePin {A1};
const unsigned int PUB_FREQ {20};
#define PUB_FREQ 20
const unsigned long int PERIOD { 1000000 / PUB_FREQ };


// GLOBALS
ros::NodeHandle  nh;
geometry_msgs::Twist vel_msg;
ros::Publisher vel_pub(VEL_TOPIC, &vel_msg);
unsigned long int t;

void setup()
{
  vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = 0;
  vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0;
  pinMode(linePin, INPUT);
  pinMode(anglePin, INPUT);
  nh.initNode();
  nh.advertise(vel_pub);
}

void loop()
{
  t = micros() + PERIOD;
  vel_msg.linear.x = map(analogRead(linePin), 0.0, 1024.0, -1.6, 1.6);
  vel_msg.angular.z = map(analogRead(anglePin), 0.0, 1024.0, -1.5, 1.5);
  vel_pub.publish(&vel_msg);
  nh.spinOnce();
  while( micros() < t ){}
}
