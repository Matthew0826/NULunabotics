#include <Servo.h>

#define MOTOR_COUNT 4
// For communication with ROS on Raspberry Pi
#define PACKET_SIZE 4
// Using the SPARKmini Motor Controller
#define FORWARD_PULSE_WIDTH 2500.0 // in microseconds
#define REVERSE_PULSE_WIDTH 500.0 // in microseconds

byte motorPins[MOTOR_COUNT] = {5, 9, 6, 11};

Servo motors[MOTOR_COUNT];

// unsigned long lastUpdateTime = 0;

// Format:
// Header: 0xFFFF
// Motor to move (1 byte)
// Amount to move (1 byte): 0 is backwards, 255 is forwards, and 127 is neutral
byte serialInput[PACKET_SIZE]; // Buffer for serial input
int serialIndex = 0;

void setup() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors[i].attach(motorPins[i]);
    motors[i].writeMicroseconds(1500); // neutral position
  }
  Serial.begin(9600); // Used to read commands from ROS
}

void loop() {
  // Handle non-blocking serial input
  if (Serial.available() > 0) {
    // Create a moving window of bytes
    byte input = Serial.read();
    serialInput[serialIndex] = input;
    serialIndex++;
    if (serialIndex >= PACKET_SIZE) {
      // When the header is detected,
      if (serialInput[0] == 0xFF && serialInput[1] == 0xFF) { // header is 0xFFFF
        // Parse the packet
        int motor = (int)serialInput[2]; // the index into the motors array
        int speed = (int)serialInput[3]; // 0 means reverse, 127 means neutral, and 255 means forward
        if (motor >= 0 && motor < MOTOR_COUNT) {
          motors[motor].writeMicroseconds(map(speed, 0, 255, REVERSE_PULSE_WIDTH, FORWARD_PULSE_WIDTH));
        }
      }

      serialIndex = 0;
    }
  }
}
