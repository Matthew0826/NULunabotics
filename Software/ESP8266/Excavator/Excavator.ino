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
#define LIMIT_SWITCH_PIN 7

#define MAX_VOLTS 5.0
#define BOARD_MAX_ANALOG_VALUE 1023.0

#define ACTUATOR_LIMIT_TOLERANCE 0.02

#define MOTOR_COUNT 3
byte motorPins[MOTOR_COUNT] = {6, 5, 9};
Servo motors[MOTOR_COUNT];

int loopCount = 0;
float actuatorPercent = 0.0;
float excavatorPercent = 0.0;
bool isLimitSwitchTriggered = false;

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
  pinMode(LIMIT_SWITCH_PIN, INPUT);
}

bool validateSpeed(float speed, int id) {
  // default is 0.5, which would mean its ok to move it since its in the middle
  float percentValue = 0.5;
  switch (id) {
    case 0:
      percentValue = excavatorPercent;
      if (isLimitSwitchTriggered) {
        return false;
      }
      break;
    case 1:
      percentValue = actuatorPercent;
      break;
  }
  bool movingBelowLimit = speed < 0 && percentValue < ACTUATOR_LIMIT_TOLERANCE;
  bool movingAboveLimit = speed > 0 && percentValue > (1.0 - ACTUATOR_LIMIT_TOLERANCE);
  return !movingAboveLimit && !movingBelowLimit;
}

void onExcavatorState(const ExcavatorState& data) {
  // process the incoming data
  float speeds[MOTOR_COUNT] = {
    data.excavatorLifterSpeed,
    data.conveyorSpeed,
    data.actuatorSpeed
  };
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (validateSpeed(speeds[i], i)) {
      // Map the speed to the pulse width
      int speedByte = (int)(((speeds[i] + 1.0) * 127.5));
      int pulseWidth = map(speedByte, 0, 255, REVERSE_PULSE_WIDTH, FORWARD_PULSE_WIDTH);
      motors[i].writeMicroseconds(pulseWidth);
    }
  }
}

void loop() {
  TheiaSerial::tick();

  // used to stop the excavator from hitting the robot when lifting it
  isLimitSwitchTriggered = (digitalRead(LIMIT_SWITCH_PIN) == HIGH);

  if (loopCount % 8 == 0) {
    // Read the potentiometer values
    float actuatorPotentiometer = analogRead(ACTUATOR_POTENTIOMETER_PIN);
    float excavatorPotentiometer = analogRead(EXCAVATOR_POTENTIOMETER_PIN);
    actuatorPercent = (float)map(actuatorPotentiometer, 0, 1023, 0, 100) / 100.0;
    excavatorPercent = (float)map(excavatorPotentiometer, 0, 1023, 0, 100) / 100.0;
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

    // Create a LinearActuatorPercent struct to hold the percent values
    PotentiometerPercents excavatorPercents;
    excavatorPercents.excavatorLifterPercent = excavatorPercent;
    excavatorPercents.actuatorPercent = actuatorPercent;

    // Send the percent values to the Raspberry Pi
    TheiaSerial::sendFramedMessage(EXCAVATOR_PERCENT_ID, excavatorPercents);
  }

  delay(10);

  loopCount++;
}
