#include <Servo.h>
#include <TheiaSerial.h>

#define EXCAVATOR_ID 5
#define EXCAVATOR_PERCENT_ID 6

// Using the SPARKmini Motor Controller
#define FORWARD_PULSE_WIDTH 2500.0 // in microseconds
#define REVERSE_PULSE_WIDTH 500.0 // in microseconds

#define POTENTIOMETER_PIN1 A3
#define POTENTIOMETER_PIN2 A4

// Actuators, conveyor, and hatch motors
#define MOTOR_COUNT 3
byte motorPins[MOTOR_COUNT] = {6, 9, 11};
Servo motors[MOTOR_COUNT];

typedef struct {
    float leftActuatorSpeed;
    float rightActuatorSpeed;
    float conveyor;
    bool hatch;
} ExcavatorState;

typedef struct {
    float leftPercent;
    float rightPercent;
} LinearActuatorPercent;

void nothingCallback(const LinearActuatorPercent& percent) {
    // nothing
}

void setup() {
  TheiaSerial::begin();
  TheiaSerial::addId<ExcavatorState>(EXCAVATOR_ID, onMotorData);
  TheiaSerial::addId<LinearActuatorPercent>(EXCAVATOR_PERCENT_ID, nothingCallback);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors[i].attach(motorPins[i]);
    motors[i].writeMicroseconds(1500); // neutral position
  }

  analogReference(DEFAULT);
}

void onMotorData(const MotorData& data) {
  // process the incoming data
  float speeds[MOTOR_COUNT] = {data.leftActuatorSpeed, data.rightActuatorSpeed, data.conveyor};
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // Map the speed to the pulse width
    byte speedByte = (byte)(speeds[i] * 255 + 127.5);
    int pulseWidth = map(speedByte, 0, 255, REVERSE_PULSE_WIDTH, FORWARD_PULSE_WIDTH);
    motors[i].writeMicroseconds(pulseWidth);
  }
}

void loop() {
  TheiaSerial::tick();

  // Read the potentiometer values
  float leftPot = analogRead(POTENTIOMETER_PIN1);
  float rightPot = analogRead(POTENTIOMETER_PIN2);
  float leftPercent = map(leftPot, 0, 1023, 0, 100) / 100.0;
  float rightPercent = map(rightPot, 0, 1023, 0, 100) / 100.0;

  // Create a LinearActuatorPercent struct to hold the percent values
  LinearActuatorPercent excavatorPercent;
  excavatorPercent.leftPercent = leftPercent;
  excavatorPercent.rightPercent = rightPercent;

  // Send the percent values to the Raspberry Pi
  TheiaSerial::sendFramedMessage(EXCAVATOR_PERCENT_ID, excavatorPercent);

  delay(50);
}
