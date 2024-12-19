#include <Arduino.h>
#include <ESP32Encoder.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Physical parameters
const float m = 0.00217;
const float g = 9.81;
const float l = 0.0508;
const float J = 1.8797e-06;
const float b = 0.007;
const float counts_per_rot = 400;

// Feedback gains
const float K1 = 0.001;
const float K2 = 1;

// Motor parameters
const float voltage = 12.0; 
const float stall_adjust = 0.004;       // Nm at 5V 
const float stall_torque = stall_adjust*(voltage/5.0);   // Max voltage
const int pwm_max = 255;                // Max PWM
const float torque_to_pwm = pwm_max / stall_torque;

// Pin definitions
const int Bin1_pin = 22;     // Motor pin 1
const int Bin2_pin = 23;     // Motor pin 2
const int zeroButtonPin = 0; // Button pin for zeroing (BOOT button is GPIO 0)

// Encoder variables
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
float control_input_u = 0.0;

// Observer constants
const float L1 = 3700;
const float L2 = -13778607.77;
const float mgl_over_J = m * g * l / J;
const float b_over_J = b / J;

void SerialComm() {
  Serial.print(">");
  Serial.print("position:"); Serial.print(position);
  Serial.print(",raw_angle:"); Serial.print(position * 360 / counts_per_rot);
  Serial.print(",obsv_angle:"); Serial.print(theta * (180 / 3.1416)); // Radians to degrees
  Serial.print(",velocity:"); Serial.print(theta_dot * (180 / 3.1416)); // Rad/s to deg/s
  Serial.print(",control_input_u:"); Serial.print(control_input_u);
  Serial.print(",Timestep:"); Serial.print(Ts);
  Serial.println("\r");
}

void zeroEncoderButton() {
  if (digitalRead(zeroButtonPin) == LOW) {
    encoder.setCount(0);
    theta = 0;
    theta_dot = 0;
    x1_hat = 0.0;
    x2_hat = 0.0;
    Serial.println("Encoder zeroed!");
    delay(500);
  }
}

void applyControl(float u) {
  int pwm_value = (int)(fabs(u) * torque_to_pwm);
  if (pwm_value > pwm_max) pwm_value = pwm_max; // Saturate PWM
  
  if (u > 0) { // Forward torque
    analogWrite(Bin1_pin, pwm_value);
    analogWrite(Bin2_pin, 0);
  } else { // Reverse torque
    analogWrite(Bin1_pin, 0);
    analogWrite(Bin2_pin, pwm_value);
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

  // Timing for sampling time Ts
  t_curr = millis() / 1000.0;
  Ts = t_curr - t_prev;

  // Observer updates
  x1_hat = (x1_hat_old + Ts * x2_hat + Ts * L1 * angle_rad) / (1 + Ts * L1);
  x2_hat = (x2_hat_old - Ts * (mgl_over_J + L2) * x1_hat + Ts * L2 * angle_rad + Ts * (1 / J) * control_input_u) / (1 + Ts * b_over_J);

  theta_dot = (x1_hat - x1_hat_old) / Ts;

  // Update observer variables for the next iteration
  x1_hat_old = x1_hat;
  x2_hat_old = x2_hat;
  t_prev = t_curr;

  // Assign observer outputs
  theta = x1_hat;

  control_input_u = -K1 * theta - K2 * theta_dot;
  //control_input_u = 0;
  // Apply control input as PWM to motor
  applyControl(control_input_u);

  // Send data over serial
  SerialComm();

  delayMicroseconds(30000); // ~5ms for stability
}
