// Calcaultes the voltage and current the battery is using and transmits the data over serial in packets
// Packet protocol is defined below
#include <Arduino.h>
#include "TheiaSerial.h"

#define RX_PIN A4// receives analog inputs
#define TX_PIN 2 // transmit 0=current and 1=voltage

#define HALL_EFFECT_SENSOR_SENSITIVITY 133.0/1000.0
#define MAX_VOLTS 5.0
#define BOARD_MAX_ANALOG_VALUE 1023.0

// Id for publishing to power data type (publisher of power has id of 3)
#define POWER_ID 3

// "there was something going on where it wouldn't like being a local variable"
int voltVal;

// Struct holding our power information
typedef struct {
    float voltage;
    float current;
} Power;

void setup() {
  analogReference(DEFAULT);

  pinMode(TX_PIN, OUTPUT);
  TheiaSerial::begin();

  // id of 3, and no callback function
  TheiaSerial::addId<Power>(POWER_ID, mockCallback);
}

// According to the spec sheet of the hall effect sensor
float voltsToAmps(float volts) {
  return (volts - 2.5)/0.133;
}

float readVolts() {
  voltVal = analogRead(RX_PIN);
  return voltVal * (MAX_VOLTS / BOARD_MAX_ANALOG_VALUE);
}

float readCurrent() {
  float volts = readVolts();
  return voltsToAmps(volts);
}

// pointer that automatically dereferences itself
void mockCallback(const Acceleration& acceleration) {
  // nothing
}

byte previousSerialByte = 0x00;
void loop() {
  // Flip bit to 0 to represent a current measurement
  digitalWrite(TX_PIN, LOW);
  // Wait for the MUX to update
  delay(100);
  float current = readCurrent()/16.0;
  delay(100);
  
  // Flip bit to 1 to represent a voltage measurement
  digitalWrite(TX_PIN, HIGH);
  // Wait for the MUX to update
  delay(100);
  float voltage = readVolts();
  delay(100);
  
  // Write results to Serial connection
  Power power;
  power.voltage = voltage;
  power.current = current;
  
  // Use Theia Serial with id for PowerData
  TheiaSerial::sendFramedMessage(POWER_ID, power);

  delay(20);
}
