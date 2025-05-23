#include <Servo.h>
#include <TheiaSerial.h>

#define ID 4

// Using the SPARKmini Motor Controller
#define FORWARD_PULSE_WIDTH 2500.0 // in microseconds
#define REVERSE_PULSE_WIDTH 500.0 // in microseconds

#define MOTOR_COUNT 4
// on the board, it goes 11, 6, 9, 5
// so from left to right:
// left back, right back, right front, left front
// at the time of writing this comment, it is just most important that the right wheels go in the center 2 JST connectors
byte motorPins[MOTOR_COUNT] = {5, 11, 9, 6};
Servo motors[MOTOR_COUNT];

typedef struct {
  float leftFrontSpeed;
  float rightFrontSpeed;
  float leftBackSpeed;
  float rightBackSpeed;
} MotorData;

void setup() {
  TheiaSerial::begin();
  TheiaSerial::addId<MotorData>(ID, onMotorData);
  TheiaSerial::broadcastIds();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors[i].attach(motorPins[i]);
    motors[i].writeMicroseconds(1500); // neutral position
  }
}

void onMotorData(const MotorData& data) {
  // process the incoming data
  float speeds[MOTOR_COUNT] = {data.leftFrontSpeed, data.leftFrontSpeed, data.rightFrontSpeed, -data.rightFrontSpeed};
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // Map the speed to the pulse width
    int speedByte = (int)(((speeds[i] + 1.0) * 127.5));
    int pulseWidth = map(speedByte, 0, 255, REVERSE_PULSE_WIDTH, FORWARD_PULSE_WIDTH);
    motors[i].writeMicroseconds(pulseWidth);
  }
}

void loop() {
  TheiaSerial::tick();
}
