#include <Servo.h>
#include <TheiaSerial.h>

#define EXCAVATOR_ID 5
#define EXCAVATOR_PERCENT_ID 6
#define EXCAVATOR_DISTANCE_ID 7

// Using the SPARKmini Motor Controller
#define FORWARD_PULSE_WIDTH 2500.0 // in microseconds
#define REVERSE_PULSE_WIDTH 500.0 // in microseconds

#define DISTANCE_SENSOR_PIN A4
#define ACTUATOR_POTENTIOMETER_PIN A3
#define EXCAVATOR_POTENTIOMETER_PIN A5

#define MAX_VOLTS 5.0
#define BOARD_MAX_ANALOG_VALUE 1023.0

#define MOTOR_COUNT 3
byte motorPins[MOTOR_COUNT] = {6, 5, 9};
Servo motors[MOTOR_COUNT];

typedef struct {
    float excavatorLifterSpeed;
    float conveyorSpeed;
    float actuatorSpeed;
} ExcavatorState;

typedef struct {
  float excavatorLifterPercent;
  float actuatorPercent;
} PotentiometerPercents;

typedef struct {
    float distanceSensor;
} DistanceSensor;

void nothingCallback(const PotentiometerPercents& percent) {
    // nothing
}

void nothingCallback2(const DistanceSensor& percent) {
    // nothing
}

void setup() {
  TheiaSerial::begin();
  TheiaSerial::addId<ExcavatorState>(EXCAVATOR_ID, onExcavatorState);
  TheiaSerial::addId<PotentiometerPercents>(EXCAVATOR_PERCENT_ID, nothingCallback);
  TheiaSerial::addId<DistanceSensor>(EXCAVATOR_DISTANCE_ID, nothingCallback2);
  TheiaSerial::broadcastIds();

  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors[i].attach(motorPins[i]);
    motors[i].writeMicroseconds(1500); // neutral position
  }

  analogReference(DEFAULT);
}

void onExcavatorState(const ExcavatorState& data) {
  // process the incoming data
  float speeds[MOTOR_COUNT] = {
    data.excavatorLifterSpeed,
    data.conveyorSpeed,
    data.actuatorSpeed
  };
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // Map the speed to the pulse width
    int speedByte = (int)(((speeds[i] + 1.0) * 127.5));
    int pulseWidth = map(speedByte, 0, 255, REVERSE_PULSE_WIDTH, FORWARD_PULSE_WIDTH);
    motors[i].writeMicroseconds(pulseWidth);
  }
}

void loop() {
  TheiaSerial::tick();

  // Read the potentiometer values
  float actuatorPotentiometer = analogRead(ACTUATOR_POTENTIOMETER_PIN);
  float excavatorPotentiometer = analogRead(EXCAVATOR_POTENTIOMETER_PIN);
  float actuatorPercent = (float)map(actuatorPotentiometer, 0, 1023, 0, 100) / 100.0;
  float excavatorPercent = (float)map(excavatorPotentiometer, 0, 1023, 0, 100) / 100.0;
  if (actuatorPercent == 0.0) actuatorPercent = 0.001;
  if (excavatorPercent == 0.0) excavatorPercent = 0.001;

  // Read the distance sensor value
  int distanceSensorValue = analogRead(DISTANCE_SENSOR_PIN);
  float distanceSensorVoltage = (distanceSensorValue / BOARD_MAX_ANALOG_VALUE) * MAX_VOLTS;
  // Calculate distance in cm using the formula from datasheet
  // https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf
  float distance = 12.08 * pow(distanceSensorVoltage, -1.058);
  // Send the distance sensor data to the Raspberry Pi
  DistanceSensor distanceSensorData;
  distanceSensorData.distanceSensor = distance;
  TheiaSerial::sendFramedMessage(EXCAVATOR_DISTANCE_ID, distanceSensorData);

  delay(50);

  // Create a LinearActuatorPercent struct to hold the percent values
  PotentiometerPercents excavatorPercents;
  excavatorPercents.excavatorLifterPercent = actuatorPercent;
  excavatorPercents.actuatorPercent = excavatorPercent;

  // Send the percent values to the Raspberry Pi
  TheiaSerial::sendFramedMessage(EXCAVATOR_PERCENT_ID, excavatorPercents);

  delay(50);
}
