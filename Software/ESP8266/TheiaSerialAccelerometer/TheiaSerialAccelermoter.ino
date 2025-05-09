#include "TheiaSerial.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// id for acceleration
#define ACCELERATION_ID 0

// id for acceleration correction
#define CORRECTION_ID 1

// id for transmitting data about beacons
#define BEACON_ID 2

// For accelerometer
Adafruit_MPU6050 mpu;

// To find delta time
unsigned long previousTime = 0;

// Variables for dealing with accelerometer error (check calculateIMUerror function)
float AccX = 0.0;
float AccY = 0.0;
float AccZ = 0.0;
float GyroX = 0.0;
float GyroY = 0.0;
float GyroZ = 0.0;
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY = 0.0;
float GyroErrorZ = 0.0;
float initialAngleZ = 0.0;
float angleZ = 0.0;

int c = 0;

// save most recent distance values
float beaconDistances[3] = {-1, -1, -1};

typedef struct {
    float orientation;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
} Acceleration;

typedef struct {
    float initialOrientation;
    bool shouldReset;
} Correction;

typedef struct {
    float distance_0;
    float distance_1;
    float distance_2;
} BeaconDistances;

void handleCorrection(const Correction& correction) {
    initialAngleZ = correction.initialOrientation;
    if (correction.shouldReset) {
        angleZ = 0.0;
    }
}

void mockCallback(const Acceleration& acceleration) {
    // nothing
}

void setup() {
    TheiaSerial::begin();

    // id of 0, and no callback function
    TheiaSerial::addId<Acceleration>(ACCELERATION_ID, mockCallback);
    TheiaSerial::addId<Correction>(CORRECTION_ID, handleCorrection);

    // add the beacons to be published to pi
    TheiaSerial::addId<BeaconDistances>(BEACON_ID, mockCallback)

    // Find accelerometer. Did you forget to plug it in to the header pins?
    if (!mpu.begin()) {
        // Failed to find MPU6050 chip
        while (true) delay(10);
    }

    // Set up accelerometer
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    //calculateIMUerror();
    delay(20);
}

void loop() {
    TheiaSerial::tick();
  
    // Get accelerometer data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Time delta in seconds
    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Remove gyro bias
    float gyroZ = g.gyro.z - GyroErrorZ;
    angleZ += gyroZ * dt;

    Acceleration acceleration;
    float angleZDegrees = fmod(((angleZ * 180.0) / 3.1415) + 360.0, 360.0);
    acceleration.orientation = angleZDegrees + initialAngleZ;
    acceleration.accelerationX = a.acceleration.x + AccErrorX;
    acceleration.accelerationY = a.acceleration.y + AccErrorY;
    acceleration.accelerationZ = a.acceleration.z;
    TheiaSerial::sendFramedMessage(0, acceleration);

    // handle beacon data (distance from center to each beacon)
    updateUWBTag();

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
  
    // Sanitize data a little bit (check good distance data and valid anchor ids)
    if (distance <= 0 || distance > 65533 || anchorID < 0 || anchorID > 3) return;

    // save new value
    beaconDistances[anchorID] = distance
}

void updateUWBTag() {
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

    // publish data about the beacons
    if (beaconDistances[0] != -1 && beaconDistances[1] != -1 && beaconDistances[2] != -1) {

        // prepare beacon distances to send via Serial
        BeaconDistances distances;
        distances.distance_0 = beaconDistances[0];
        distances.distance_1 = beaconDistances[1];
        distances.distance_2 = beaconDistances[2];

        // Use Theia Serial with id for PowerData
        TheiaSerial::sendFramedMessage(BEACON_ID, distances);
    }
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
