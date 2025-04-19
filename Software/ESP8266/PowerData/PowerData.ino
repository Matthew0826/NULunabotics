// Calcaultes the voltage and current the battery is using and transmits the data over serial in packets
// Packet protocol is defined below

#define ESP_BOARD_ID 4
#define RX_PIN 17 // receives analog inputs
// TODO: check if reversed
#define TX_PIN 16 // transmit 0=current and 1=voltage

#define HALL_EFFECT_SENSOR_SENSITIVITY 133.0/1000.0
#define MAX_VOLTS 5.0
#define BOARD_MAX_ANALOG_VALUE 1023.0


// Protocol for communication with Raspberry Pi:
// Header: 0xFFFFFFFF
// Voltage value (4 byte float): The voltage of the battery in volts
// Current value (4 byte float): The current being used by the robot in amps

// Identifier protocol:
// Header: 0xFFFE
// Response:
//   Header: 0xFFFE
//   Board ID: 1 byte

void setup() {
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  Serial.begin(9600);
}

void writeVoltageCurrent(float volts, float amps) {
  // // Write header
  // Serial.write(0xFF);
  // Serial.write(0xFF);
  // Serial.write(0xFF);
  // Serial.write(0xFF);

  // // Convert float values to bytes
  // Serial.write((byte*)&volts, 4);
  // Serial.write((byte*)&amps, 4);
  Serial.print("Volts:");
  Serial.print(volts);
  Serial.print(",Amps:");
  Serial.println(amps);
}

// According to the spec sheet of the hall effect sensor
float voltsToAmps(float volts) {
  return (volts - MAX_VOLTS/2.0)/HALL_EFFECT_SENSOR_SENSITIVITY;
}

float readVolts() {
  int analogValue = analogRead(RX_PIN);
  return analogValue * (MAX_VOLTS / BOARD_MAX_ANALOG_VALUE);
}

float readCurrent() {
  float volts = readVolts();
  return voltsToAmps(volts);
}

byte previousSerialByte = 0x00;
void loop() {
  if (Serial.available()) {
    // Identification protocol
    byte nextSerialByte = Serial.read();
    if (nextSerialByte == 0xFF && previousSerialByte == 0xFE) {
      Serial.write(0xFF);
      Serial.write(0xFE);
      Serial.write(ESP_BOARD_ID);
    }
    previousSerialByte = nextSerialByte;
  }

  // Flip bit to 0 to represent a current measurement
  digitalWrite(TX_PIN, LOW);
  // Wait for the MUX to update
  delay(100);
  float current = readCurrent();
  
  // Flip bit to 1 to represent a voltage measurement
  digitalWrite(TX_PIN, HIGH);
  // Wait for the MUX to update
  delay(100);
  int voltage = readVolts();
  
  // Write results to Serial connection
  writeVoltageCurrent(voltage, current);
}
