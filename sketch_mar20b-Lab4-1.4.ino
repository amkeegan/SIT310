#define USE_USBCON
#include <Wire.h>
#include <Zumo32U4.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;
void ros_handler( const geometry_msgs::Twist& cmd_msg) 
{
  float x = cmd_msg.linear.x;
  float z = cmd_msg.angular.z;
  
  if(x == -2.0) backward(100);
  if(x == 2.0) forward(100);
  
  if(z == 2.0) turn_left(100);
  if(z == -2.0) turn_right(100);
  
  stop();
}

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", ros_handler);
Zumo32U4Motors motors;

void setup()
{
  // Uncomment if necessary to correct motor directions:
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
  nh.initNode();
  nh.subscribe(sub);
}

void forward(int time)
{
  motors.setLeftSpeed(100);
  motors.setRightSpeed(100);
  delay(time);
}

void backward(int time)
{
  motors.setLeftSpeed(-100);
  motors.setRightSpeed(-100);
  delay(time);
}

void turn_left(int time)
{
  motors.setLeftSpeed(-100);
  motors.setRightSpeed(100);
  delay(time);
}

void turn_right(int time)
{
  motors.setLeftSpeed(100);
  motors.setRightSpeed(-100);
  delay(time);
}

void stop()
{
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
