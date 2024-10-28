#include <Arduino.h>
#include <ESP32Encoder.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* strtol */

// Global variables
ESP32Encoder encoder;
unsigned long myTime;
unsigned long myTime2;
long int dutyCycle = 0;
char myData[20];   // Array to hold received string
long int position = 0;
long int reference = 0;
long int gain = 1;

// Motor pins
const int Bin1_pin = 15;  
const int Bin2_pin = 2;   

// Button pin for zeroing (BOOT button is GPIO 0)
const int zeroButtonPin = 0;

float stallTorque = 2.0;  // Example stall torque in Nm, adjust based on your motor specs
float torque = 0;
float angle = 0;

// Function to handle serial communication
long int handleSerialComm() {
  myTime2 = millis();
  Serial.println("Angle: " + String(position) + "\t Torque: " + String(torque) + "\t PWM: " + String(dutyCycle) + "\t Time_ms: " + String(myTime2));

  if (Serial.available() != 0) {
    byte m = Serial.readBytesUntil('\n', myData, 20); // Receive characters until Newline is detected
    myData[m] = '\0';  // Insert null-character at end of character array
    long int z = strtol(myData, NULL, 10);  // Convert received string to integer

    // Reset the array and return the new duty cycle
    memset(myData, 0x00, 20);     
    return z;
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

// Function to handle encoder readings and angle calculation
void updateEncoderAndAngle() {
  position = (int32_t)encoder.getCount();
  angle = (float)position / 200.0 * 360.0;
}

// Function to estimate torque based on PWM and stall torque
void estimateTorque() {
  torque = stallTorque * ((float)dutyCycle / 255.0);
}

// Function to zero the encoder when the button is pressed
void checkZeroButton() {
  if (digitalRead(zeroButtonPin) == LOW) {
    encoder.setCount(0);
    angle = 0;
    Serial.println("Encoder zeroed!");
    delay(500);  // Debounce delay to avoid multiple resets
  }
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
  encoder.attachHalfQuad(19, 18);

  // Set starting count value
  encoder.setCount(0);

  // Output initial encoder state
  Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
}

void loop() {
  checkZeroButton();           // Check if the zero button is pressed
  updateEncoderAndAngle();      // Update encoder and angle
  dutyCycle = handleSerialComm();  // Handle serial communication and update duty cycle
  controlMotor(dutyCycle);      // Control motor based on the updated duty cycle
  estimateTorque();             // Estimate motor torque
  delayMicroseconds(20);        // Short delay to avoid overwhelming serial communication
}
