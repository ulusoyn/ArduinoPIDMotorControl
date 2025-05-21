#include "motor.h"

#define ENCODER_A 2
#define ENCODER_B 3

// const int L298N_enA = 21;          // PWM
// const int L298N_in1 = 20;          // Dir Motor A
// const int L298N_in2 = 19;          // Dir Motor A
#define L298N_enA 21
#define L298N_in1 20
#define L298N_in2 19
#define COUNTS_PER_REV 449    // Encoder count per motor rotation
#define DEGREES_PER_REV 360
#define MILLISEC_2_SEC 1000
#define SEC_2_MIN     60
#define OUTPUT_MAX    255
#define OUTPUT_MIN    -255


// @@@@@@ NOTES ABOUT PROJECT
// Encoder counts 449 per motor rotation
// Encoder counts 22 per encoder rotation
// @@@@@@@ END NOTES

// encoder variables
volatile long motorPosition = 0;
unsigned long lastTime = 0;
long lastPosition = motorPosition;

// motor control
bool is_right_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_cmd_complete = false;
char value[] = "000";
uint8_t value_idx = 0;
int MIN_PWM = 0;
int MAX_PWM = 255;
MotorController *RightMotor;

// physical
float wheel_radius_ = 0.034; // meters


// PID Control
const float epsilon = 0.01;               // radians per second, adjust based on your precision needs
float desiredSpeed = 0, actualSpeed = 0;
float integral = 0, previousError = 0;
int pwmSpeed;

// PID VALUES                             // MUST BE TUNED
float KP_SPEED = 0.6;
float KD_SPEED = 0.2;
float KI_SPEED = 0.4;

void writePID()
{
  Serial.print("kd: ");
  Serial.print(KD_SPEED);
  Serial.print(" kp: ");
  Serial.print(KP_SPEED);
  Serial.print(" ki: ");
  Serial.println(KI_SPEED);
}


// finds the minimum pwm value rotates the motor
int findMotorDeadBand(MotorController *motor) {
  int pwmVal = 1;
  long start = millis();
  while (motorPosition == 0) {
    motor->write(pwmVal);
    if (millis() - start > 40) {
      start = millis();
      pwmVal++;
      delay(100);
    }
  }
  Serial.print("The value for Deadband is: ");
  Serial.println(pwmVal);
  return pwmVal;
}

void updateMotorPosition(){
  if (digitalRead(ENCODER_B) != digitalRead(ENCODER_A))
  {
    motorPosition++;
  }
  else {
    motorPosition--;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Motor initialization
  RightMotor = new MotorController(L298N_enA, L298N_in1, L298N_in2);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateMotorPosition, CHANGE);

  MIN_PWM = findMotorDeadBand(RightMotor);
  Serial.write("Deadband is:");
  Serial.write(MIN_PWM);
  
  RightMotor->write(0);
  delay(1000);
  motorPosition = 0;
  lastTime = millis();
}

void loop() {
  // Blink the onboard LED for status
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  
  
  // The if statement below is used to get a serial command to give a desired speed for motors. It ovverrides desiredSpeed variable
  // and this variable will be used for PID control.
  if (Serial.available()) { 
    char chr = Serial.read();
    if (chr == 'k')
    {
      char type = Serial.read();
      char val[5];
      int i = 0;
      while (Serial.available() && i < 5)
        val[i++] = Serial.read();

      float v = atof(val);
      if (type == 'p') KP_SPEED = v;
      else if (type == 'd') KD_SPEED = v;
      else if (type == 'i') KI_SPEED = v;
      else{
        Serial.println("There is no such a type");
      }
      writePID();
    }
    // Right Wheel Motor Command
    if (chr == 'r') {
      is_right_wheel_cmd = true;
      value_idx = 0;
      memset(value, '\0', sizeof(value));  // Reset the value array
    }
    // Positive direction
    else if (chr == 'p' && is_right_wheel_cmd) {
      if (!is_right_wheel_forward) {
        // Change direction to forward
        RightMotor->changeDir(1);
        is_right_wheel_forward = true;
      }
    }
    // Negative direction
    else if (chr == 'n' && is_right_wheel_cmd) {
      if (is_right_wheel_forward) {
        // Change direction to backward
        RightMotor->changeDir(0);
        is_right_wheel_forward = false;
      }
    }
    // Command Value Separator
    else if (chr == ',' && is_right_wheel_cmd) {
      desiredSpeed = atof(value);                           // value is in rad/s
      desiredSpeed = desiredSpeed;             // converted in deg/s
      Serial.print("Desired speed is");
      Serial.println(desiredSpeed);
      // Reset command flags
      is_right_wheel_cmd = false;
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Command Value Input
    else if (value_idx < sizeof(value) - 1) {
      value[value_idx++] = chr;
    }
  }

  unsigned long now = millis();
  unsigned long dt = now - lastTime;

  if (dt >= 50) { // every 50 ms
    noInterrupts();
    long currentPosition = motorPosition;
    interrupts();
    
    long deltaPos = currentPosition - lastPosition;
    double revs = deltaPos / (float)COUNTS_PER_REV;
    float deltaTime = dt / 1000.0;

    actualSpeed = revs / deltaTime * 2 * M_PI; // current speed in rad/s

    lastPosition = currentPosition;
    lastTime = now;
  }

  float error = desiredSpeed - actualSpeed;
  Serial.print("Actual Speed is: ");
  Serial.print(actualSpeed);
  Serial.print(" desired speed: ");
  Serial.print(desiredSpeed);
  Serial.print(" error: ");
  Serial.print(error);
  float dt_seconds = dt / 1000.0;
  integral += error * dt_seconds;
  float derivative = (error - previousError) / dt_seconds;
  float output = KP_SPEED * error + KI_SPEED * integral + KD_SPEED * derivative;
  
  Serial.print(" output: ");
  Serial.print(output);
  previousError = error;

  if (fabs(error) < epsilon) {
    integral = 0;
  }

  int pwmValue = constrain(RadtoPwm(output + actualSpeed), 0, MAX_PWM);
  
  Serial.print(" new target pwm: ");
  Serial.print(RadtoPwm(output+actualSpeed));
  Serial.print(" new target rad/s: ");
  Serial.print((output+actualSpeed));
  
  Serial.print(" calculated pwm: ");
  Serial.println(pwmValue);

  RightMotor->write(pwmValue);

}

int RadtoPwm(float rad_per_sec)
{ 
  if (rad_per_sec == 0) return 0;
  if (rad_per_sec < 8) rad_per_sec = 8;
  if (rad_per_sec > 35) rad_per_sec = 35;

  int pwm = 0.011823*power(rad_per_sec, 3) - 0.56285*power(rad_per_sec, 2) + 10.6229*rad_per_sec - 4.66451;
  return pwm;
}

float power(float numb, uint8_t pow)
{
  float ans = numb;
  for (uint8_t i = 0; i < pow - 1; i++)
    ans *= numb;

  return ans;
}
