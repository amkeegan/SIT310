#define USE_USBCON
#include <Wire.h>
#include <Zumo32U4.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>

LSM303 compass;

ros::NodeHandle nh;

Zumo32U4ProximitySensors proxSensors;
bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;

std_msgs::Int16 heading;
std_msgs::Int8 prox_msg;


ros::Publisher pub_heading("/zumo/heading", &heading);
ros::Publisher pub_left("/zumo/prox_left", &prox_msg);
ros::Publisher pub_frontleft("/zumo/prox_frontleft", &prox_msg);
ros::Publisher pub_frontright("/zumo/prox_frontright", &prox_msg);
ros::Publisher pub_right("/zumo/prox_right", &prox_msg);

void ros_handler( const geometry_msgs::Twist& cmd_msg) 
{
  float x = cmd_msg.linear.x;
  float y = cmd_msg.linear.y;
  
  //Type conversion - I don't need to worry about decimal values after multiplacation.
  // e.g x = 0.4999; x_speed = 49;
  
  int x_speed = static_cast<int>(x * 100);
  int y_speed = static_cast<int>(y * 100);
  int duration = 100;

  //Set Max and Min speeds at 200 & -200
  if (x_speed > 200) x_speed = 200;
  if (x_speed < -200) x_speed = 200;
  if (y_speed > 200) y_speed = 200;
  if (y_speed < -200) y_speed = 200;

  //Send absolute speed and duration to new movement functions.
  // Movement functions only use unsigned values.
  if(x > 0) forward(abs(x_speed), duration);
  if(x < 0) backward(abs(x_speed), duration);
  if(y > 0) left(abs(y_speed), duration);
  if(y < 0) right(abs(y_speed), duration);
  
  stop();
}

ros::Subscriber<geometry_msgs::Twist> sub("/zumo/cmd_vel", ros_handler);

Zumo32U4Motors motors;

void setup()
{
  //setup the compass.
  Wire.begin();
  
  if (!compass.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the compass."));
      delay(100);
    }
  }
  
  //replace with your calibration values
  compass.enableDefault();
  compass.m_min.x=-10282;
  compass.m_min.y=-1094;
  compass.m_min.z=1485;
  compass.m_max.x=-4134;
  compass.m_max.y=2831;
  compass.m_max.z=8402;
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_heading);
  nh.advertise(pub_left);
  nh.advertise(pub_frontleft);
  nh.advertise(pub_frontright);
  nh.advertise(pub_right);
  
  proxSensors.initThreeSensors();
  uint16_t defaultBrightnessLevels[] = {1,2,3,4,5,6,7,8,9,10};
  proxSensors.setBrightnessLevels(defaultBrightnessLevels, 10);
}

void publishSensorData()
{
  prox_msg.data = proxSensors.countsLeftWithLeftLeds();
  pub_left.publish( &prox_msg);
  prox_msg.data = proxSensors.countsFrontWithLeftLeds();
  pub_frontleft.publish( &prox_msg);
  prox_msg.data = proxSensors.countsFrontWithRightLeds();
  pub_frontright.publish( &prox_msg);
  prox_msg.data = proxSensors.countsRightWithRightLeds();
  pub_right.publish( &prox_msg);
}

void forward(int speed, int time)
{ 
  //Serial.print("Moving forward at: "); Serial.print(speed); Serial.print(" for: "); Serial.println(time);
  motors.setLeftSpeed(speed);
  motors.setRightSpeed(speed);
  delay(time);
}

void backward(int speed, int time)
{
  motors.setLeftSpeed(-speed);
  motors.setRightSpeed(-speed);
  delay(time);
}

void left(int speed, int time)
{
  motors.setLeftSpeed(-speed);
  motors.setRightSpeed(speed);
  delay(time);
}

void right(int speed, int time)
{
  motors.setLeftSpeed(speed);
  motors.setRightSpeed(-speed);
  delay(time);
}

void stop()
{
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
}

//control timing without pausing program.
static uint16_t lastSampleTime = 0; 

void loop()
{
  ledRed(1);
  ledGreen(0);
  if ((uint16_t)(millis() - lastSampleTime) >= 500)
  {
    lastSampleTime = millis();
    
    // Send IR pulses and read the proximity sensors.
    proxSensors.read();
    publishSensorData();
    
    compass.read();
    
    //int reading = compass.heading();
    heading.data = compass.heading();
    
    pub_heading.publish(&heading);
    
    ledRed(0);
    ledGreen(1);
  }
  
  nh.spinOnce();
  delay(1);
}
