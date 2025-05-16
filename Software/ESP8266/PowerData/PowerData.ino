// Calcaultes the voltage and current the battery is using and transmits the data over serial in packets
// Packet protocol is defined below
#include <Arduino.h>
#include <TheiaSerial.h>
#include <Wire.h>

#define FIVE_VOLT_MEASUREMENT_PIN A0
#define MAX_VOLTS 3.3
#define BOARD_MAX_ANALOG_VALUE 1023.0
#define VOLTAGE_DIVIDER_FORMULA (33.0/(33.0+36.0))
// max current in entire system measured in amps
#define MAX_EXPECTED_CURRENT 12.0
// Data sheet:
// https://www.ti.com/lit/ds/symlink/ina226.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1747291379891&ref_url=https%253A%252F%252Fwww.ti.com%252Fgeneral%252Fdocs%252Fsuppproductinfo.tsp%253FdistId%253D10%2526gotoUrl%253Dhttps%253A%252F%252Fwww.ti.com%252Flit%252Fgpn%252Fina226
#define CALIBRATION (0.00512/((MAX_EXPECTED_CURRENT/32768.0)*0.002)

// Id for publishing to power data type (publisher of power has id of 3)
#define POWER_ID 3

// Struct holding our power information
typedef struct {
    float voltage12V;
    float voltage5V;
    float current;
} Power;


void mockCallback(const Power& power) {}

void setup() {
  TheiaSerial::begin();
  analogReference(DEFAULT);

  // id of 3, and no callback function
  TheiaSerial::addId<Power>(POWER_ID, mockCallback);

  Wire.begin();
  // TODO: connect to the current sensor through I2C on ID 64 (0x40 in hex)
}

void loop() {
  TheiaSerial::tick();
  // Read the voltage from the 5V pin
  int rawValue = analogRead(FIVE_VOLT_MEASUREMENT_PIN);
  // Convert the raw value to voltage
  float rawVoltage3V3 = (rawValue / BOARD_MAX_ANALOG_VALUE) * MAX_VOLTS;
  float voltage5V = rawVoltage3V3 / VOLTAGE_DIVIDER_FORMULA;


  // calibrate for 10A

  // read bus voltage to get 12v
  // read current register to get current

  // Read the voltage and current from I2C
  // TODO: reference datasheet to figure out how to 
  
  // Write results to Serial connection
  Power power;
  power.voltage12V = voltage12V;
  power.voltage5V = voltage5V;
  power.current = current;
  
  // Use Theia Serial with id for PowerData
  TheiaSerial::sendFramedMessage(POWER_ID, power);

  delay(80);
}
