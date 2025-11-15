// Define the analog pin connected to the sensor
const int sensorPin = A4; // You can change this if using a different analog pin

void setup() {
  Serial.begin(9600);
  Serial.println("GP2Y0A41SK0F Distance Sensor Test");
}

void loop() {
  int analogValue = analogRead(sensorPin);

  // Convert analog value to voltage (assuming 5V Arduino)
  float voltage = analogValue * (5.0 / 1023.0);

  // Convert voltage to distance in cm (empirical formula from datasheet)
  // Note: This is a rough approximation
  float distance = 12.08 * pow(voltage, -1.058);

  Serial.print("Analog: ");
  Serial.print(analogValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V | Distance: ");
  Serial.print(distance, 1);
  Serial.println(" cm");

  delay(500); // Read every 0.5 seconds
}
