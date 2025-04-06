// Lidar wiki: https://wiki.youyeetoo.com/en/Lidar/LD20

#include <Wire.h>
#include <SoftwareSerial.h>

// Identifier protocol:
// Header: 0xFFFE

// Response:
// Header: 0xFFFE
// Board ID: 1 byte
#define ESP_BOARD_ID 3

#define LIDAR_PWM_PIN D3

// To keep track of how many bytes are in each rotation of the LiDAR
#define POINT_PER_PACK 12
#define BYTES_PER_PACK 44
#define READINGS_FOR_ROT 56
#define POINTS_PER_ROTATION (READINGS_FOR_ROT * POINT_PER_PACK)
#define BYTES_PER_ROTATION (POINTS_PER_ROTATION * BYTES_PER_PACK)

// Serial to communicate with the Raspberry Pi
#define LIDAR_BAUD 230400
#define USB_BAUD 19200
#define RX_PIN D2
#define TX_PIN D1
SoftwareSerial raspberryPiSerial(RX_PIN, TX_PIN);

// Defines a point in polar coordinates, where 0 is towards the motor in the LiDAR
// Distance is the number of millimeters away from the center of the LiDAR
// Weight is the condifence in this point. Higher is better
typedef struct {
    byte weight;
    unsigned short distance; // Units: millimeters
    unsigned short angle;    // Units: hundreds of degrees (36000 means 360 degrees)
} point_t;

// Defines a point in cartesian coordinates, where (0, 0) represents the center of the LiDAR
typedef struct {
  unsigned short x;
  unsigned short y;
} vector2_t;


// Feature represents a section of points close together spacially
// typedef struct {
//   vector2_t *featurePoints;
// } feature_t;


// Keep track of currently processing packet
byte previousByte = 0x00;
byte previousPiByte = 0x00;
byte packets[BYTES_PER_ROTATION];
int packetByteIndex = 0;
bool isReadingPacket = false;

// Keep track of a full rotation's worth of points
int pointIndex = 0;
point_t points[POINTS_PER_ROTATION];

// Unpack the raw bytes
void processPacket(byte currentPacket[BYTES_PER_PACK]) {
  // Packet format:
  // Initial identifier (1 Byte) - Fixed value 0x54, marks the start of the packet.
  // VerLen             (1 Byte) - Frame type (1) and number of measurement points (12 points).
  // Radar speed        (2 Bytes) - Speed of the LiDAR in degrees per second.
  // Starting angle     (2 Bytes) - Starting angle in 0.01 degrees (relative to 360 degrees scan).
  // Measurement data   (12 points, each 3 Bytes) - 2 bytes for distance (mm), 1 byte for signal strength.
  // End angle          (2 Bytes) - End angle in 0.01 degrees (relative to 360 degree scan).
  // Timestamp          (2 Bytes) - Timestamp in milliseconds, indicates when the packet was created.
  // CRC check          (2 Bytes) - Cyclic redundancy check for data integrity.
  
  // Calculate step size
  int startAngle = int(currentPacket[5]) * 256 + int(currentPacket[4]);
  int endAngle = int(currentPacket[43]) * 256 + int(currentPacket[42]);
  float step = (endAngle - startAngle) / (POINT_PER_PACK-1);
  
  // Read each point in the packet
  for (int i = 0; i < POINT_PER_PACK; i += 3) {
    if (pointIndex >= POINTS_PER_ROTATION) return;
    unsigned short distance = (unsigned short)(int(currentPacket[i+7]) * 256 + int(currentPacket[i+6]));
    byte strength = currentPacket[i+8];
    point_t point;
    point.distance = distance;
    point.angle = (unsigned short)(startAngle + step*i);
    point.weight = strength;
    points[pointIndex] = point;
    pointIndex++;
  }
}

void sendUnsignedShort(unsigned short val) {
  // Send the high byte (most significant byte)
  raspberryPiSerial.write(val >> 8);

  // Send the low byte (least significant byte)
  raspberryPiSerial.write(val & 0xFF);
}


void proccessFullRotation() {
  pointIndex = 0;

  // Send the delimiter FFFFFFFF (4 bytes) using a loop
  for (int i = 0; i < 4; i++)
    raspberryPiSerial.write(0xFF);
  
  // Send all points
  for (int i = 0; i < POINTS_PER_ROTATION; i++) {
    sendUnsignedShort(points[i].angle);
    sendUnsignedShort(points[i].distance);
    sendUnsignedShort(points[i].weight);
  }
}


vector2_t pointToVector(point_t point) {
  float degrees = ((float)point.angle)/100.0;
  float radians = PI * degrees / 180.0;
  vector2_t vector;
  vector.x = point.distance * cos(radians);
  vector.y = point.distance * sin(radians);
  return vector;
}


void setup() {
  // Serial is for the LiDAR data
  Serial.begin(LIDAR_BAUD);
  // raspberryPiSerial is for the USB connection to the Raspberry Pi
  raspberryPiSerial.begin(USB_BAUD);
}


void loop() {
  // Check if data is available on the Serial
  if (Serial.available()) {
    // Get the next byte that we read from the LiDAR
    byte nextByte = (byte)Serial.read();

    // Check if we detected the start of a packet
    if (nextByte == 0x2C && previousByte == 0x54) {
      packetByteIndex = 1;
      packets[0] = previousByte;
      packets[1] = nextByte;
      // Wait for enough bytes to come in
      while (Serial.available() < (BYTES_PER_ROTATION-2)) {}
      // Proccess all packets
      Serial.readBytes(packets+2, BYTES_PER_ROTATION-2); // -2 due to the header bytes
      for (int i = 0; i < POINTS_PER_ROTATION; i++) {
        processPacket(packets + i*BYTES_PER_PACK + 1); // +1 due to the header (only 1 is unexpected) (this is because I don't want to change the indexes in the old code)
      }
      proccessFullRotation();
    }
    previousByte = nextByte;
  }
  if (raspberryPiSerial.available()) {
    // Identification protocol
    byte nextSerialByte = raspberryPiSerial.read();
    if (nextSerialByte == 0xFF && previousPiByte == 0xFE) {
      raspberryPiSerial.write(0xFF);
      raspberryPiSerial.write(0xFE);
      raspberryPiSerial.write(ESP_BOARD_ID);
    }
    previousPiByte = nextSerialByte;
  }
}
