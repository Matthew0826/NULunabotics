//
// THIS IS THE BEACON CODE THAT GOES ON THE BOARD ON THE ROBOT
//

#define ESP_BOARD_ID 2

// Protocol for communication with Raspberry Pi:
// Header: 0xFF
// If next byte is 0xFF, that means its sending a distance to a beacon
// Anchor index: 1 byte (0, 1, or 2)
// Distance: 2 bytes (in centimeters)
// 
// If the next bytes after the header was 0xFD, its sending an accelerometer value
// Right now it sends:
// Angle on Z axis: 4 bytes (float from 0.0 to 360.0)

// Identifier protocol:
// Header: 0xFFFE

// Response:
// Header: 0xFFFE
// Board ID: 1 byte

#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// UWB stands for Ultra-Wideband and it's how the beacons ("anchors") find the distance to this "tag" board that's sitting on the robot
// Here's a link for reference on thier "AT" protocol: https://reyax.com//upload/products_download/download_file/AT_Command_RYUW122.pdf
SoftwareSerial uwbSerial(D6, D5);  // (RX, TX) NOTE: THEY ARE FLIPPED FROM THE ONES IN THE CIRCUIT!!

// Tag beacon configuration
const String TAG_ADDRESS = "TAG00010";   // This tag's address (8 bytes)
const String NETWORK_ID = "NEULUNAB";
const String UWB_BAUD = "115200";
const String PASSWORD = "FABC0002EEDCAA90FABC0002EEDCAA90";
const String MODE = "0";  // TAG mode

// For accelerometer
Adafruit_MPU6050 mpu;
float angleZ = 0.0;

// Variables for dealing with accelerometer error (check calculateIMUerror function)
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
int c = 0;

// Sends an AT command to the module and prints it to Serial
void sendCommand(const String &cmd) {
  uwbSerial.print(cmd + "\r\n");
  // Serial.print("Sent command: ");
  Serial.println(cmd);
  String response = "";
  // Wait until we get a response
  while (response.length() == 0) {
    if (uwbSerial.available()) {
      response = uwbSerial.readStringUntil('\n');  // Read a line from uwbSerial
      Serial.println(response);  // Print it to the standard Serial
    }
    delay(10);
  }
  Serial.println("Command done!");
  delay(10);
}

void setup() {
  Serial.begin(9600);
  while (!Serial)
    delay(10);
  // uwbSerial MUST run on 115200 baud rate, or else it fails to send anything
  uwbSerial.begin(115200);
  while (!uwbSerial)
    delay(10);
  delay(1000);

  // Configure UWB module for TAG mode
  // If you're having issues, try sending "AT+RESET" first
  sendCommand("AT+IPR=" + UWB_BAUD);
  sendCommand("AT+MODE=" + MODE);
  sendCommand("AT+NETWORKID=" + NETWORK_ID);
  sendCommand("AT+ADDRESS=" + TAG_ADDRESS);
  sendCommand("AT+CPIN=" + PASSWORD);

  // Serial.println("Tag module configured.");

  
  // Find accelerometer. Did you forget to plug it in to the header pins?
  if (!mpu.begin()) {
    // Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Set up accelerometer
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  calculateIMUerror();
  delay(20);

}

// Parse the custom payload from an anchor.
// Expected format: "A1XXYYDDD"
// - First char: Anchor ID (e.g. "1")
// - Last 3 chars: Distance (in cm)
void parsePayload(const String &payload) {
  if (payload.length() < 4) {
    // Serial.println("Invalid payload: " + payload);
    return;
  }
  // Serial.println(payload);

  int anchorID = payload.substring(0, 1).toInt();
  int distance = payload.substring(1, 5).toInt();

  // Sanitize data a little bit
  if (distance <= 0 || distance > 65533 || anchorID < 0 || anchorID > 3) return;

  // Write a header for beacons
  Serial.write(0xFF);
  Serial.write(0xFF);
  
  // Write data
  Serial.write((byte)(anchorID & 0xFF)); // send only the first byte of anchorID
  uint16_t distanceShort = distance & 0xFFFF;
  Serial.write((byte*)&distanceShort, 2); // send first two bytes of distance
}

byte previousSerialByte = 0x00;

void loop() {
  // Listen for incoming data from the UWB module.
  if (uwbSerial.available()) {
    String line = uwbSerial.readStringUntil('\n');
    line.trim();
    // Serial.println("Raw received: " + line);
    // Expected incoming format: "+TAG_RCV=<payload_length>,<payload>"
    int commaIndex = line.indexOf(",");
    if (commaIndex != -1 && commaIndex < line.length()-1) {
      String payload = line.substring(commaIndex+1);
      parsePayload(payload);
    }
  }
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
  
  // Get accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate angle from accelerometer data
  angleZ += (g.gyro.z + GyroErrorZ) * 1.403;
  if (angleZ < 0.0)        angleZ = angleZ + 360.0;
  else if (angleZ > 360.0) angleZ = angleZ - 360.0;
  
  // if (c % 30 == 0) {
  //   // Write a header for rotation
  //   Serial.write(0xFF);
  //   Serial.write(0xFD);
    
  //   // Write data
  //   Serial.write((byte*)&angleZ, 4);
  // }
  // c++; // no way! that's the name of the programming language!
  delay(20);
}


// Adapted from: https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
void calculateIMUerror() {
  // We can call this function in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    AccX = a.acceleration.x;
    AccY = a.acceleration.y;
    AccZ = a.acceleration.z;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));

    GyroX = g.gyro.x;
    GyroY = g.gyro.y;
    GyroZ = g.gyro.z;
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  // Serial.print("AccErrorX: ");
  // Serial.println(AccErrorX);
  // Serial.print("AccErrorY: ");
  // Serial.println(AccErrorY);
  // Serial.print("GyroErrorX: ");
  // Serial.println(GyroErrorX);
  // Serial.print("GyroErrorY: ");
  // Serial.println(GyroErrorY);
  // Serial.print("GyroErrorZ: ");
  // Serial.println(GyroErrorZ);
} 
