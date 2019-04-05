#define USE_USBCON
#include <Wire.h>
#include <Zumo32U4.h>
#include <ros.h>

Zumo32U4ButtonA buttonA;

Zumo32U4ProximitySensors proxSensors;
bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;
Zumo32U4Motors motors;

float Prox_Left;
float Prox_FrontLeft;
float Prox_FrontRight;
float Prox_Right;

void setup()
{
  // Uncomment if necessary to correct motor directions:
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
  
  buttonA.waitForButton();
  delay(1000);
  
  proxSensors.initThreeSensors();
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

void about_turn_right(int time)
{
  motors.setLeftSpeed(200);
  motors.setRightSpeed(-200);
  delay(time);
}

void about_turn_left(int time)
{
  motors.setLeftSpeed(-200);
  motors.setRightSpeed(200);
  delay(time);
}

void stop()
{
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
}

void checkProx()
{
  Prox_Left = proxSensors.countsLeftWithLeftLeds();
  
  Prox_FrontLeft = proxSensors.countsFrontWithLeftLeds();
  
  Prox_FrontRight = proxSensors.countsFrontWithRightLeds();
  
  Prox_Right = proxSensors.countsRightWithRightLeds();

  delay(100);
}

void loop()
{
  static uint16_t lastSampleTime = 0;
  
  if ((uint16_t)(millis() - lastSampleTime) >= 100)
  {
    lastSampleTime = millis();
    
    // Send IR pulses and read the proximity sensors.
    proxSensors.read();
    
    // Just read the proximity sensors without sending pulses.
    //proxLeftActive = proxSensors.readBasicLeft();
    //proxFrontActive = proxSensors.readBasicFront();
    //proxRightActive = proxSensors.readBasicRight();
    
    checkProx();
    
    if (Prox_FrontLeft >= 5.0 && Prox_FrontRight >= 5.0)
    {
      if (Prox_Left > Prox_Right)
      {
        about_turn_right(20);
      }
      else
      {
        about_turn_left(20);
      }
    }
       
    if (Prox_Left >= 5.0)
    {
      turn_right(20);
    }
    
    if (Prox_Right >= 5.0)
    {
      turn_left(20);
    }
    
    if (Prox_FrontLeft <= 5.0 && Prox_FrontRight <= 5.0)
    {
      
      forward(20);
    }
    
  }

  delay(1);
}
