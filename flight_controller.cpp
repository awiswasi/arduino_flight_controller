#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>

// define pins for motor control
const int motor1Pin = 3;
const int motor2Pin = 5;
const int motor3Pin = 6;
const int motor4Pin = 9;

// define variables for sensor calibration
const int calibrateTime = 5000;
const float gravity = 9.81;

// define variables for PID control
const float kp = 1.0;
const float ki = 0.0;
const float kd = 0.0;

// define variables for sensor readings
MPU6050 mpu;
Adafruit_BMP280 bmp;
float roll, pitch, yaw, altitude;
float setpointRoll = 0.0, setpointPitch = 0.0, setpointYaw = 0.0, setpointAltitude = 0.0;
float previousErrorRoll = 0.0, previousErrorPitch = 0.0, previousErrorYaw = 0.0, previousErrorAltitude = 0.0;
float integralRoll = 0.0, integralPitch = 0.0, integralYaw = 0.0, integralAltitude = 0.0;

// define variables for RC control
Servo channel1, channel2, channel3, channel4;
int channel1Value, channel2Value, channel3Value, channel4Value;

// define variables for arming and disarming the drone
bool armed = false;
int disarmChannel = 6;  // channel for disarming the drone
int armChannel = 7;     // channel for arming the drone
int disarmValue = 1000; // PWM signal value for disarming the drone
int armValue = 2000;    // PWM signal value for arming the drone

void setup() {
  // set up motor control pins as outputs
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);

  // set up I2C communication
  Wire.begin();

  // set up MPU6050 sensor
  mpu.initialize();
  mpu.calibrateGyro();
  mpu.calibrateAccel();
  delay(calibrateTime);
  mpu.getGyroOffsets();

  // set up BMP280 sensor
  bmp.begin();

  // set up RC control
  channel1.attach(2);
  channel2.attach(3);
  channel3.attach(4);
  channel4.attach(5);
  
  // initialize the drone in a disarmed state
  armed = false;
  analogWrite(motor1Pin, 0);
  analogWrite(motor2Pin, 0);
  analogWrite(motor3Pin, 0);
  analogWrite(motor4Pin, 0);
}

void loop() {
  // read sensor values
  mpu.update();
  roll = mpu.getRoll();
  pitch = mpu.getPitch();
  yaw = mpu.getYaw();
  altitude = bmp.readAltitude();

  // read RC control signals
  channel1Value = channel1.readMicroseconds();
  channel2Value = channel2.readMicroseconds();
  channel3Value = channel3.readMicroseconds();
  channel4Value = channel4.readMicroseconds();

  // check if the drone should be armed or disarmed based on RC signals
  if (channel1Value >= armValue && channel2Value >= armValue && channel3Value >= armValue && channel4Value >= armValue) {
    armed = true;
  }
  else if (channel1Value <= disarmValue && channel2Value <= disarmValue && channel3Value <= disarmValue && channel4Value <= disarmValue) {
    armed = false;
    analogWrite(motor1Pin, 0);
    analogWrite(motor2Pin, 0);
    analogWrite(motor3Pin, 0);
    analogWrite(motor4Pin, 0);
  }

  if (armed) {
    // map RC control signals to control signals for the drone flight controller
    float rollSignal = map(channel1Value, 1000, 2000, -255, 255);
    float pitchSignal = map(channel2Value, 1000, 2000, -255, 255);
    float yawSignal = map(channel3Value, 1000, 2000, -255, 255);
    float altitudeSignal = map(channel4Value, 1000, 2000, 0, 255);

    // convert control signals to setpoints for the PID controller
    setpointRoll = rollSignal;
    setpointPitch = pitchSignal;
    setpointYaw = yawSignal;
    setpointAltitude = altitudeSignal;

    // calculate PID control signals for pitch, roll, yaw, and altitude
    float errorRoll = setpointRoll - roll;
    float errorPitch = setpointPitch - pitch;
    float errorYaw = setpointYaw - yaw;
    float errorAltitude = setpointAltitude - altitude;
    integralRoll += errorRoll;
    integralPitch += errorPitch;
    integralYaw += errorYaw;
    integralAltitude += errorAltitude;
    float derivativeRoll = errorRoll - previousErrorRoll;
    float derivativePitch = errorPitch - previousErrorPitch;
    float derivativeYaw = errorYaw - previousErrorYaw;
    float derivativeAltitude = errorAltitude - previousErrorAltitude;
    pitchSignal = kp * errorPitch + ki * integralPitch + kd * derivativePitch;
    rollSignal = kp * errorRoll + ki * integralRoll + kd * derivativeRoll;
    yawSignal = kp * errorYaw + ki * integralYaw + kd * derivativeYaw;
    altitudeSignal = kp * errorAltitude + ki * integralAltitude + kd * derivativeAltitude;

    // check for excessive PID control signals
    if (abs(pitchSignal) > 255.0 || abs(rollSignal) > 255.0 || abs(yawSignal) > 255.0 || abs(altitudeSignal) > 255.0) {
      emergencyShutdown();
    }

    // set motor speeds based on PID control signals
    float motor1Speed = altitudeSignal + pitchSignal - rollSignal - yawSignal;
    float motor2Speed = altitudeSignal + pitchSignal + rollSignal + yawSignal;
    float motor3Speed = altitudeSignal - pitchSignal + rollSignal - yawSignal;
    float motor4Speed = altitudeSignal - pitchSignal - rollSignal + yawSignal;

    // constrain motor speeds to 0-255 range
    motor1Speed = constrain(motor1Speed, 0, 255);
    motor2Speed = constrain(motor2Speed, 0, 255);
    motor3Speed = constrain(motor3Speed, 0, 255);
    motor4Speed = constrain(motor4Speed, 0, 255);

    // set motor speeds
    analogWrite(motor1Pin, motor1Speed);
    analogWrite(motor2Pin, motor2Speed);
    analogWrite(motor3Pin, motor3Speed);
    analogWrite(motor4Pin, motor4Speed);

    // store previous error values
    previousErrorRoll = errorRoll;
    previousErrorPitch = errorPitch;
    previousErrorYaw = errorYaw;
    previousErrorAltitude = errorAltitude;
  } else {
    // if the drone is not armed, set motor speeds to 0
    analogWrite(motor1Pin, 0);
    analogWrite(motor2Pin, 0);
    analogWrite(motor3Pin, 0);
    analogWrite(motor4Pin, 0);
  }
}

void emergencyShutdown() {
  // stop motors
  analogWrite(motor1Pin, 0);
  analogWrite(motor2Pin, 0);
  analogWrite(motor3Pin, 0);
  analogWrite(motor4Pin, 0);

  // pause for 1 second
  delay(1000);

  // restart program
  setup();
  loop();
}
