#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
class MotorController
{
private:
  byte _pinPwm;
  byte _pinDir1;
  byte _pinDir2;
  float _currentAngSpeed;
  bool _direction; // if 0 ccw, if 1 cw

public:
  MotorController(){}
  MotorController(uint8_t pwm, uint8_t dir1, uint8_t dir2);
  void write(int pwm);
  void write(int pwm, bool dir);
  void write(float angular_velocity); 
  void changeDir(bool direction);
  float GetAngSpeed() {return _currentAngSpeed;}
};


#endif