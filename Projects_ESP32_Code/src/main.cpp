#include <Arduino.h>
#include <ESP32Encoder.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* strtol */
#include <string.h>

// Global variables
ESP32Encoder encoder;
unsigned long myTime;
unsigned long myTime2;
int8_t dutyCycle = 0;
char myData[50];   // Array to hold received string
long int position = 0;
long int reference = 0;
long int gain = 1;

// Motor pins
const int Bin1_pin = 15;  
const int Bin2_pin = 2;   

// Button pin for zeroing (BOOT button is GPIO 0)
const int zeroButtonPin = 0;

// System parameters
float counts_per_rot = 400;
float stallTorque = 0.004;  // N-m, known stall torque for torque-PWM conversion
float pwm_offset = 100;     // Default PWM offset, this will be converted to a torque offset
//float pendulum_m = 0.0031;  // kg
//float pendulum_J = 0.000022;
//float pendulum_L = 0.064;   // length in meters
//float pendulum_B = 0.001;
//float pendulum_K = 0.001;

float torque = 0;
float angle_rad = 0;
float Ts = 0; 
float corner_freq = 50; // filter cutoff
float angle_rad_prev = 0;
float tt_sec = 0;
float tt_sec_last = 0;
float theta_dot = 0;  // Angular velocity

// PID variables for angular position
float Kp = 0.0, Ki = 0.0, Kd = 0.0;  // PID gains for angle control
float integral = 0;
float previous_error = 0;
float desired_angle = 45.0;  // Desired angle in degrees
float max_pwm = 255;         // Maximum PWM value

// Set to true for Serial Plotter mode, false for Serial Monitor mode
bool serialPlotterMode = true; // Change this before recompiling

// Function to parse and update PID gains
void parseAndUpdateGains(const char* input) {
  float parsedKp, parsedKd, parsedKi;
  
  // Use sscanf to extract three floating point values from the input string
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
  
  if (serialPlotterMode) {
    // Serial Plotter Mode (space-separated values)
    Serial.println(String(position) + " " + String(angle_rad*(180/3.1416)) + " " + String(theta_dot*(180/3.1416)) + " " + String(torque) + " " + String(dutyCycle));
  } else {
    // Serial Monitor Mode (with labels)
    Serial.println("Raw Encoder: " + String(position) + "\t Angle (deg): " + String(angle_rad*(180/3.1416)) + "\t Angular Velocity (deg/s): " + String(theta_dot*(180/3.1416)) + "\t Torque (Nm): " + String(torque) + "\t PWM: " + String(dutyCycle) + "\t Time (ms): " + String(myTime2));
  }

  if (Serial.available() != 0) {
    byte m = Serial.readBytesUntil('\n', myData, 50); // Receive characters until Newline is detected
    myData[m] = '\0';  // Insert null-character at end of character array

    // Check if input is for PID gains (contains '[' and ']')
    if (strchr(myData, '[') && strchr(myData, ']')) {
      parseAndUpdateGains(myData);  // Parse the string and update PID gains
    } else {
      long int z = strtol(myData, NULL, 10);  // Convert received string to integer (for PWM)
      memset(myData, 0x00, 50);               // Reset the array
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
  tt_sec = (micros()) / 1e6;  // Convert to seconds
  Ts = tt_sec - tt_sec_last;     // Calculate sample time

  if (Ts > 0) {
    // Low-pass filter for the angle
    angle_rad = (angle_rad_prev + Ts * corner_freq * angle_rad_raw) / (1 + Ts * corner_freq);
    // Calculate angular velocity (theta_dot)
    theta_dot = (angle_rad - angle_rad_prev) / Ts;
    // Update previous angle for the next iteration
    angle_rad_prev = angle_rad;
  } else if (Ts == 0 ){
    Serial.println("CHANGE IS TIME IS 0. BAD!!");
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
    delay(500);  // Debounce delay to avoid multiple resets
  }
}

// PID control function to compute the desired torque based on angular error
float PIDControlForAngle(float desired_angle, float actual_angle) {
  // Compute the error (in degrees)
  float error = desired_angle - actual_angle;

  // Calculate proportional term
  float Pout = Kp * error;

  // Calculate integral term
  integral += error * Ts;
  float Iout = Ki * integral;

  // Calculate derivative term
  float derivative = (error - previous_error) / Ts;
  float Dout = Kd * derivative;

  // Compute the total torque output
  float output_torque = Pout + Iout + Dout;

  // Save the error for the next cycle (for derivative calculation)
  previous_error = error;

  return output_torque;  // Return the torque required to correct the angle
}

// Function to compute PWM from torque
long int computePWMFromTorque(float torque) {
  // Convert torque to PWM, accounting for the torque offset from the PWM offset
  long int pwm_output = ((torque / stallTorque) * 255.0);
  // Ensure the PWM output is within bounds
  pwm_output = constrain(pwm_output, -max_pwm, max_pwm);

  return pwm_output;
}

void setup() {
  Serial.begin(115200, SERIAL_8N1); 

  // Set motor pins as outputs
  pinMode(Bin1_pin, OUTPUT);
  pinMode(Bin2_pin, OUTPUT);
  // Set the zeroing button pin (BOOT button) as input with pull-up
  pinMode(zeroButtonPin, INPUT_PULLUP);

  // Enable weak pull-up resistors for the encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // Attach encoder to pins
  encoder.attachFullQuad(19, 18);
  // Set starting count value
  encoder.setCount(0);

  // Output initial encoder state
  Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
}

void loop() {
  checkZeroButton();               // Check if the zero button is pressed
  getAngleAndVelocityFilt();        // Update encoder, angle, and angular velocity
  
  // Perform PID control for angular position
  float actual_angle = angle_rad * (180.0 / 3.1416);  // Convert angle from radians to degrees
  float desired_torque = PIDControlForAngle(desired_angle, actual_angle);  // Compute the required torque
  
  // Compute the PWM value from the desired torque
  dutyCycle = computePWMFromTorque(desired_torque);
  controlMotor(dutyCycle);          // Control motor based on the updated duty cycle
  SerialComm();                     // Handle serial communication and update the outputs
  delayMicroseconds(100000);             // Short delay to avoid overwhelming serial communication
}
