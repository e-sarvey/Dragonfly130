#include <Arduino.h>
#include <ESP32Encoder.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Variables to adjust
float desired_angle_deg = 45.0;
float Kp = 1e-5;
float Kd = 1e-5;
float Ki = 0.0;

float pwm_offset = 40; // the default PWM to reach 45 degrees
float max_pwm = 255;
float corner_freq = 10;

// Global variables
ESP32Encoder encoder;
unsigned long myTime;
unsigned long myTime2;
int8_t dutyCycle = 0;
char myData[50];
long int position = 0;
long int reference = 0;
long int gain = 1;

// Motor pins
const int Bin1_pin = 22;  
const int Bin2_pin = 23;   
// Button pin for zeroing (BOOT button is GPIO 0)
const int zeroButtonPin = 0;

// System parameters
float counts_per_rot = 400;
float stallTorque = 0.004*(9/5);
float torque = 0;
float angle_rad = 0;
float Ts = 0; 
float angle_rad_prev = 0;
float tt_sec = 0;
float tt_sec_last = 0;
float theta_dot = 0;
float error = 0;
// PID variables for angular position
float integral = 0;
float previous_error = 0;
float desired_angle_rad = desired_angle_deg *(PI/180);

// Function to parse and update PID gains
void parseAndUpdateGains(const char* input) {
  float parsedKp, parsedKd, parsedKi;
  
  if (sscanf(input, "[%f,%f,%f]", &parsedKp, &parsedKd, &parsedKi) == 3) {
    Kp = parsedKp;
    Kd = parsedKd;
    Ki = parsedKi;
    
    Serial.println("Updated PID Gains:");
    Serial.print("Kp = "); Serial.println(Kp);
    Serial.print("Kd = "); Serial.println(Kd);
    Serial.print("Ki = "); Serial.println(Ki);
  } else {
    Serial.println("Invalid input format for PID gains. Please use [Kp,Kd,Ki]");
  }
}

// Function to handle serial communication, including PID gains input
long int SerialComm() {
  myTime2 = millis();

  // Serial Plotter Mode with new syntax
  Serial.print(">");
  Serial.print("position:"); Serial.print(position);
  Serial.print(",angle:"); Serial.print(angle_rad * (180 / 3.1416));  // Convert radians to degrees
  Serial.print(",velocity:"); Serial.print(theta_dot * (180 / 3.1416));  // Convert rad/s to deg/s
  Serial.print(",torque:"); Serial.print(torque);
  Serial.print(",pwm:"); Serial.print(dutyCycle);
  Serial.print(",Sample Time:"); Serial.print(Ts);
  Serial.print(",Error:"); Serial.print(error);
  Serial.println("\r");

  if (Serial.available() != 0) {
    byte m = Serial.readBytesUntil('\n', myData, 50);
    myData[m] = '\0';

    if (strchr(myData, '[') && strchr(myData, ']')) {
      parseAndUpdateGains(myData);
    } else {
      long int z = strtol(myData, NULL, 10);
      memset(myData, 0x00, 50);
      return z;
    }
  }

  return dutyCycle;
}

// Function to control motor based on dutyCycle
void controlMotor(long int dutyCycle) {
  if (dutyCycle >= 0) {
    analogWrite(Bin1_pin, dutyCycle);
    analogWrite(Bin2_pin, 0);
  } else {
    analogWrite(Bin1_pin, 0);
    analogWrite(Bin2_pin, abs(dutyCycle));
  }
}

// Function to handle encoder readings, calculate filtered angle, and compute angular velocity
void getAngleAndVelocityFilt() {
  position = (int32_t)encoder.getCount();
  float angle_rad_raw = 2.0 * 3.1416 * position / counts_per_rot;
  tt_sec = (micros()) / 1e6;
  Ts = tt_sec - tt_sec_last;

  if (Ts > 0) {
    angle_rad = (angle_rad_prev + Ts * corner_freq * angle_rad_raw) / (1 + Ts * corner_freq);
    theta_dot = (angle_rad - angle_rad_prev) / Ts;
    angle_rad_prev = angle_rad;
  } else if (Ts == 0 ){
    Serial.println("CHANGE IN TIME IS 0. BAD!!");
  }
  tt_sec_last = tt_sec;
}

// Function to estimate torque based on PWM and stall torque
void estimateTorque() {
  torque = stallTorque * (dutyCycle / 255.0);
}

// Function to zero the encoder when the button is pressed
void checkZeroButton() {
  if (digitalRead(zeroButtonPin) == LOW) {
    encoder.setCount(0);
    angle_rad = 0;
    Serial.println("Encoder zeroed!");
    delay(500);
  }
}

// PID control function to compute the desired torque based on angular error
float PIDControlForAngle(float desired_angle, float actual_angle) {
  
  error = desired_angle - actual_angle;

  float Pout = Kp * error;
  integral += error * Ts;
  float Iout = Ki * integral;
  float derivative = theta_dot;
  float Dout = Kd * derivative;

  float output_torque = Pout + Iout + Dout;
  previous_error = error;

  return output_torque;
}

// Function to compute PWM from torque
long int computePWMFromTorque(float torque) {
  long int pwm_output = ((torque / stallTorque) * 255.0) + pwm_offset;
  
  // return 40;
  return pwm_output;
}

void setup() {
  Serial.begin(115200, SERIAL_8N1); 

  pinMode(Bin1_pin, OUTPUT);
pinMode(Bin2_pin, OUTPUT);
  pinMode(zeroButtonPin, INPUT_PULLUP);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(19, 18);
  encoder.setCount(0);

  Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));


}

void loop() {
  checkZeroButton();
  getAngleAndVelocityFilt();
  
  float desired_torque = PIDControlForAngle(desired_angle_rad, angle_rad);
  
  dutyCycle = computePWMFromTorque(desired_torque);

  // Serial.print("a ang:"); Serial.print(actual_angle);
  // Serial.print("d torque (x1000)::"); Serial.print(desired_torque * 1e3);
  // Serial.print(" pwm:"); Serial.print(dutyCycle);
  // Serial.println("\r");

  // Serial.print("position:"); Serial.print(position);
  // Serial.print(",angle:"); Serial.print(angle_rad * (180 / 3.1416));  // Convert radians to degrees
  // Serial.print(",velocity:"); Serial.print(theta_dot * (180 / 3.1416));  // Convert rad/s to deg/s
  // Serial.print(",torque:"); Serial.print(torque);
  // Serial.print(",pwm:"); Serial.print(dutyCycle);
  // Serial.print("Sample Time:"); Serial.print(Ts);
  // Serial.println("\r");


  controlMotor(dutyCycle);
  SerialComm();
  delayMicroseconds(10000);
}
