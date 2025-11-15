// Calcaultes the voltage and current the battery is using and transmits the data over serial in packets
// Packet protocol is defined below
#include <Arduino.h>
#include <TheiaSerial.h>
#include <Wire.h>
// https://github.com/RobTillaart/INA226
#include "INA226.h"

#define FIVE_VOLT_MEASUREMENT_PIN A0
#define MAX_VOLTS 3.3
#define BOARD_MAX_ANALOG_VALUE 1023.0
#define VOLTAGE_DIVIDER_FORMULA (33.0/(33.0+36.0))
// max current in entire system measured in amps
#define MAX_EXPECTED_CURRENT 12.0
// Data sheet:
// https://www.ti.com/lit/ds/symlink/ina226.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1747291379891&ref_url=https%253A%252F%252Fwww.ti.com%252Fgeneral%252Fdocs%252Fsuppproductinfo.tsp%253FdistId%253D10%2526gotoUrl%253Dhttps%253A%252F%252Fwww.ti.com%252Flit%252Fgpn%252Fina226
#define SHUNT_RESISTANCE 0.002 // Ohms

// Id for publishing to power data type (publisher of power has id of 3)
#define POWER_ID 3

// Struct holding our power information
typedef struct {
    float voltage12V;
    float voltage5V;
    float current;
} Power;


INA226 INA(0x40);

void mockCallback(const Power& power) {}

void setup() {
  TheiaSerial::begin();
  analogReference(DEFAULT);

  // id of 3, and no callback function
  TheiaSerial::addId<Power>(POWER_ID, mockCallback);

  Wire.begin();
  while (!INA.begin()) {
    delay(1000);
  }
  float shunt = SHUNT_RESISTANCE;           /* shunt (Shunt Resistance in Ohms). Lower shunt gives higher accuracy but lower current measurement range. Recommended value 0.020 Ohm. Min 0.001 Ohm */
  float current_LSB_mA = 0.05;              /* current_LSB_mA (Current Least Significant Bit in milli Amperes). Recommended values: 0.050, 0.100, 0.250, 0.500, 1, 2, 2.5 (in milli Ampere units) */
  float current_zero_offset_mA = 0;         /* current_zero_offset_mA (Current Zero Offset in milli Amperes, default = 0) */
  uint16_t bus_V_scaling_e4 = 10000;        /* bus_V_scaling_e4 (Bus Voltage Scaling Factor, default = 10000) */
  bool success = INA.configure(shunt, current_LSB_mA, current_zero_offset_mA, bus_V_scaling_e4);
  delay(20);
  INA.setMaxCurrentShunt(MAX_EXPECTED_CURRENT, SHUNT_RESISTANCE);
  delay(20);
}

void loop() {
  TheiaSerial::tick();
  // Read the voltage from the 5V pin
  int rawValue = analogRead(FIVE_VOLT_MEASUREMENT_PIN);
  // Convert the raw value to voltage
  float rawVoltage3V3 = (rawValue / BOARD_MAX_ANALOG_VALUE) * MAX_VOLTS;
  float voltage5V = rawVoltage3V3 / VOLTAGE_DIVIDER_FORMULA;



  // Read the voltage and current from I2C
  float voltage12V = INA.getBusVoltage();
  float current = INA.getCurrent_mA() / 1000.0;
  
  // Write results to Serial connection
  Power power;
  power.voltage12V = voltage12V;
  power.voltage5V = voltage5V;
  power.current = current;
  
  // Use Theia Serial with id for PowerData
  TheiaSerial::sendFramedMessage(POWER_ID, power);

  delay(80);
}
