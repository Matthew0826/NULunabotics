#include "TheiaSerial.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

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

typedef struct {
    float orientation;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
} Acceleration;

typedef struct {
    float initialOrientation;
} Correction;

void handleCorrection(const Correction& correction) {
    initialAngleZ = correction.initialOrientation;
}

void setup() {
    TheiaSerial::begin();

    // id of 0, and no callback function
    TheiaSerial::addId<Acceleration>(0, NULL);
    TheiaSerial::addId<Correction>(1, handleCorrection);

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

    delay(80);
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
