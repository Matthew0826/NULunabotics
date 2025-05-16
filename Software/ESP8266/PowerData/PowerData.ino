// Calcaultes the voltage and current the battery is using and transmits the data over serial in packets
// Packet protocol is defined below
#include <Arduino.h>
#include <TheiaSerial.h>
#include <Wire.h>

#define FIVE_VOLT_MEASUREMENT_PIN A0
#define MAX_VOLTS 3.3
#define BOARD_MAX_ANALOG_VALUE 1023.0
#define VOLTAGE_DIVIDER_FORMULA (33.0/(33.0+36.0))
#define MAX_EXPECTED_CURRENT 12.0
#define CALIBRATION (0.00512/((MAX_EXPECTED_CURRENT/32768.0)*0.002)

// Id for publishing to power data type (publisher of power has id of 3)
#define POWER_ID 3

// Struct holding our power information
typedef struct {
    float voltage12V;
    float voltage5V;
    float current;
} Power;


void setup() {
  TheiaSerial::begin();
  analogReference(DEFAULT);

  // id of 3, and no callback function
  TheiaSerial::addId<Power>(POWER_ID, mockCallback);

  Wire.begin();

}

// pointer that automatically dereferences itself
void mockCallback(const Acceleration& acceleration) {
  // nothing
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
  
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } 
  
  // Write results to Serial connection
  Power power;
  power.voltage12V = voltage12V;
  power.voltage5V = voltage5V;
  power.current = current;
  
  // Use Theia Serial with id for PowerData
  TheiaSerial::sendFramedMessage(POWER_ID, power);

  delay(80);
}
