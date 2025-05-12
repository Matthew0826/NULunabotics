#include <TheiaSerial.h>
// For accelerometer data
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// For UWB data
#include <SoftwareSerial.h>
// For accelerometer algorithms
#include <Fusion.h>

// rate at which the accelerometer is sampled
#define SAMPLE_RATE 15
#define GYRO_RANGE_DEGREES 500.0

// id for acceleration
#define ACCELERATION_ID 0

// id for acceleration correction
#define CORRECTION_ID 1

// id for transmitting data about beacons
#define BEACON_ID 2

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

// UWB stands for Ultra-Wideband and it's how the beacons ("anchors") find the distance to this "tag" board that's sitting on the robot
// Here's a link for reference on their "AT" protocol: https://reyax.com//upload/products_download/download_file/AT_Command_RYUW122.pdf
SoftwareSerial uwbSerial(D6, D5);  // (RX, TX) NOTE: THEY ARE FLIPPED FROM THE ONES IN THE CIRCUIT!!

// Tag beacon configuration
const String TAG_ADDRESS = "TAG00010";   // This tag's address (8 bytes)
const String NETWORK_ID = "NEULUNAB";
const String UWB_BAUD = "115200";
const String PASSWORD = "FABC0002EEDCAA90FABC0002EEDCAA90";
const String MODE = "0";  // TAG mode

// Sends an AT command to the module and prints it to Serial
void sendCommand(const String &cmd) {
  uwbSerial.print(cmd + "\r\n");
  // Serial.print("Sent command: ");
  Serial.println(cmd);
  String response = "";
  // Wait until we get a response
//   while (response.length() == 0) {
//     if (uwbSerial.available()) {
//       response = uwbSerial.readStringUntil('\n');  // Read a line from uwbSerial
//       Serial.print("Response: ");
//       Serial.println(response);  // Print it to the standard Serial
//     }
//     delay(10);
//   }
  Serial.println("Command done!");
  delay(100);
}

// For accelerometer
Adafruit_MPU6050 mpu;

// To find delta time
unsigned long previousTime = 0;

// Does algorithms for accelerometer to calculate orientation
FusionOffset offset;
FusionAhrs ahrs;  
// set AHRS algorithm settings
const FusionAhrsSettings settings = {
    .convention = FusionConventionNwu,
    .gain = 0.4,  // or tune between 0.3 and 0.5
    .gyroscopeRange = GYRO_RANGE_DEGREES,
    .accelerationRejection = 10.0,  // keep high to reject motion
    .magneticRejection = 0.0,  // unused since magnetometer is disabled
    .recoveryTriggerPeriod = 5 * SAMPLE_RATE,
};

// save most recent distance values
// -1 means no value yet
float beaconDistances[3] = {-1.0, -1.0, -1.0};

float initialAngle = 0.0;
void handleCorrection(const Correction& correction) {
    initialAngle = correction.initialOrientation;
    if (correction.shouldReset) {
        // reset the ahrs
        FusionAhrsInitialise(&ahrs);
        FusionOffsetInitialise(&offset, SAMPLE_RATE);
    }
}

void mockCallback(const Acceleration& acceleration) {
    // nothing
}

void mockCallback2(const BeaconDistances& beaconDistances) {
    // nothing
}

void setup() {
    TheiaSerial::begin();

    // id of 0, and no callback function
    TheiaSerial::addId<Acceleration>(ACCELERATION_ID, mockCallback);
    TheiaSerial::addId<Correction>(CORRECTION_ID, handleCorrection);

    // add the beacons to be published to pi
    TheiaSerial::addId<BeaconDistances>(BEACON_ID, mockCallback2);

    TheiaSerial::broadcastIds();

    // uwbSerial MUST run on 115200 baud rate, or else it fails to send anything
    uwbSerial.begin(115200);
    while (!uwbSerial)
        delay(10);

    // Configure UWB module for TAG mode
    // If you're having issues, try sending "AT+RESET" first
    // sendCommand("AT+RESET");
    sendCommand("AT");
    sendCommand("AT+MODE=" + MODE);
    sendCommand("AT+NETWORKID=" + NETWORK_ID);
    sendCommand("AT+ADDRESS=" + TAG_ADDRESS);
    sendCommand("AT+CPIN=" + PASSWORD);

    // Find accelerometer
    if (!mpu.begin()) {
        // Failed to find MPU6050 chip
        while (true) delay(10);
    }

    // Set up accelerometer
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    // Set up accelerometer algorithm
    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSetSettings(&ahrs, &settings);

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

    Acceleration acceleration = updateAccelerometer(a, g, dt);
    TheiaSerial::sendFramedMessage(ACCELERATION_ID, acceleration);

    // handle beacon data (distance from center to each beacon)
    updateUWBTag();

    delay(20);
}

Acceleration updateAccelerometer(sensors_event_t a, sensors_event_t g, float dt) {
    // Convert gyroscope data to degrees per second
    FusionVector gyroscope = {FusionRadiansToDegrees(g.gyro.x), FusionRadiansToDegrees(g.gyro.y), FusionRadiansToDegrees(g.gyro.z)};
    // Convert to g's (1g = 9.81 m/s^2)
    FusionVector accelerometer = {a.acceleration.x / 9.81, a.acceleration.y / 9.81, a.acceleration.z / 9.81};

    // update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    
    Acceleration acceleration;
    acceleration.orientation = fmod(euler.angle.yaw + initialAngle + 360.0, 360.0);
    acceleration.accelerationX = a.acceleration.x;
    acceleration.accelerationY = a.acceleration.y;
    acceleration.accelerationZ = a.acceleration.z;
    return acceleration;
}

// Parse the custom payload from an anchor.
// Expected format: "A1XXYYDDD"
// - First char: Anchor ID (e.g. "1")
// - Last 3 chars: Distance (in cm)
void parsePayload(const String &payload) {
    if (payload.length() < 4) {
      return;
    }
  
    int anchorID = payload.substring(0, 1).toInt();
    int distance = payload.substring(1, 5).toInt();
  
    // Sanitize data a little bit (check good distance data and valid anchor ids)
    if (distance <= 0 || distance > 65533 || anchorID < 0 || anchorID > 3) return;

    // save new value
    beaconDistances[anchorID] = distance;
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
    }
}
