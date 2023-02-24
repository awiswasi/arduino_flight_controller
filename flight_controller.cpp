#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>

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
}

void loop() {
  // read sensor values
  mpu.update();
  roll = mpu.getRoll();
  pitch = mpu.getPitch();
  yaw = mpu.getYaw();
  altitude = bmp.readAltitude();

  // check for low battery voltage
  float batteryVoltage = analogRead(A0) * 0.00488; // convert analog reading to voltage
  if (batteryVoltage < 10.0) {
    emergencyShutdown();
  }

  // check for excessive pitch or roll angles
  if (abs(roll) > 45.0 || abs(pitch) > 45.0) {
    emergencyShutdown();
  }

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
  float pitchSignal = kp * errorPitch + ki * integralPitch + kd * derivativePitch;
  float rollSignal = kp * errorRoll + ki * integralRoll + kd * derivativeRoll;
  float yawSignal = kp * errorYaw + ki * integralYaw + kd * derivativeYaw;
  float altitudeSignal = kp * errorAltitude + ki * integralAltitude + kd * derivativeAltitude;
  
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
