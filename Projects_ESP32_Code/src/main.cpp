#include <Arduino.h>
#include <ESP32Encoder.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// physical parameters
const float m = 0.00217;
const float g = 9.81;
const float l = 0.0508;
const float J = 1.8797e-06;
const float b = 0.007;
const float counts_per_rot = 400;

// Setup PID constants
const float desired_angle_deg = 0.0;
const float Kp = 1.6e-3;
const float Kd = 0.001;
const float Ki = 0.0001;

// observer constants
const float L1 = 3700;
const float L2 = -13778607.77;
const float mgl_over_J = m * g * l / J;
const float b_over_J = b / J;

// Pin definitions
const int Bin1_pin = 22;  // motor pin 1
const int Bin2_pin = 23;  // motor pin 2
const int zeroButtonPin = 0; // Button pin for zeroing (BOOT button is GPIO 0)

// Encoder Variables
ESP32Encoder encoder;
long int position = 0;

// Observer variables
float Ts = 0.01;
float x1_hat = 0.0;
float x2_hat = 0.0;
float angle_rad = 0.0;
float theta = 0.0;
float theta_dot = 0.0;
float t_curr = 0.0;
float t_prev = 0.0;
float x1_hat_old = 0.0;
float x2_hat_old = 0.0;
float input_u = 0.0;

void SerialComm() {
  Serial.print(">");
  Serial.print("position:"); Serial.print(position);
  Serial.print(",raw_angle:"); Serial.print(position * 360 / counts_per_rot);
  Serial.print(",obsv_angle:"); Serial.print(theta * (180 / 3.1416)); // Convert radians to degrees
  Serial.print(",velocity:"); Serial.print(theta_dot * (180 / 3.1416)); // Convert rad/s to deg/s
  Serial.print(",Sample_Time:"); Serial.print(Ts);
  Serial.println("\r");
}

void zeroEncoderButton() {
  if (digitalRead(zeroButtonPin) == LOW) {
    encoder.setCount(0);
    theta = 0;
    Serial.println("Encoder zeroed!");
    delay(500);
  }
}

void setup() {
  Serial.begin(115200, SERIAL_8N1);

  pinMode(Bin1_pin, OUTPUT);
  pinMode(Bin2_pin, OUTPUT);
  pinMode(zeroButtonPin, INPUT_PULLUP);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(19, 18);
  encoder.setCount(0);
}

void loop() {
  zeroEncoderButton();

  position = (int32_t)encoder.getCount();
  angle_rad = 2.0 * 3.1416 * position / counts_per_rot;

  t_curr = millis();
  t_curr = t_curr / 1000.0;
  Ts = t_curr - t_prev;

  // solve for new x1_hat state using implicit euler
  x1_hat = (x1_hat_old + Ts * x2_hat + Ts * L1 * angle_rad) / (1 + Ts * L1);
  x2_hat = (x2_hat_old - Ts * (mgl_over_J + L2) * x1_hat + Ts * L2 * angle_rad + Ts * (1 / J) * U) / (1 + Ts * b_over_J);

  theta_dot = (x1_hat - x1_hat_old) / Ts;

  // Update observer variables for the next iteration
  x1_hat_old = x1_hat;
  x2_hat_old = x2_hat;
  t_prev = t_curr;

  // Assign the observer angle to theta for output
  theta = x1_hat;

  SerialComm();
  delayMicroseconds(50000);
}
