#include "motor.h"

MotorController::MotorController(uint8_t pwm, uint8_t dir1, uint8_t dir2): _currentAngSpeed(0), _direction(1), _pinPwm(pwm), _pinDir1(dir1), _pinDir2(dir2){
  pinMode(_pinPwm, OUTPUT);
  pinMode(_pinDir1, OUTPUT);
  pinMode(_pinDir2, OUTPUT);
  changeDir(1);
}

void MotorController::changeDir(bool dir){
  _direction = dir;
  if (dir){
    digitalWrite(_pinDir1, HIGH);
    digitalWrite(_pinDir2, LOW);
  }
  else{
    digitalWrite(_pinDir1, LOW);
    digitalWrite(_pinDir2, HIGH);
  }
}

void MotorController::write(int pwm)
{
  analogWrite(_pinPwm, pwm);
}
void MotorController::write(int pwm, bool dir)
{
  changeDir(dir);
  write(pwm);
}

void MotorController::write(float ang_vel){ // ang_vel in terms of rad/s. needs to be converted to pwm
  if (ang_vel < 0) 
    changeDir(0);
  else changeDir(1);
  // ang_vel should be converted into pwm.
  int pwm = ang_vel;
  write(pwm);
}