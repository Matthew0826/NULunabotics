#include <SoftwareSerial.h>

// Create a SoftwareSerial interface on pins RX:13, TX:15
SoftwareSerial uwbSerial(13, 15); // (RX, TX)

//–– Configuration ––//
const String TAG_ADDRESS = "TAG00010";   // This tag's address (8 bytes)
const String NETWORK_ID = "NEULUNAB";

// Structure to hold recent measurements from an anchor.
struct AnchorData {
  String id;            // e.g. "A1"
  int anchorX;          // fixed X coordinate (for reference)
  int anchorY;          // fixed Y coordinate (for reference)
  int measurements[20]; // last 20 distance measurements in cm
  int index;            // next write position (circular buffer)
};

// For simplicity, assume three anchors (A1, A2, A3)
AnchorData anchors[3];

// Sends an AT command to the module and prints it to Serial
void sendCommand(const String &cmd) {
  uwbSerial.write(cmd);
  Serial.print("Sent command: ");
  Serial.println(cmd);
  delay(500);
}

void setup() {
  Serial.begin(9600);
  uwbSerial.begin(9600);
  delay(1000);

  // Configure UWB module for TAG mode
  sendCommand("AT+MODE=0"); // TAG mode
  sendCommand("AT+NETWORKID=" + NETWORK_ID);
  sendCommand("AT+ADDRESS=" + TAG_ADDRESS);
  sendCommand("AT+CPIN=FABC0002EEDCAA90FABC0002EEDCAA90\r\n");

  // Initialize anchor data for three anchors.
  anchors[0].id = "A1";
  anchors[0].anchorX = 10;   // must match the anchor's payload settings
  anchors[0].anchorY = 20;
  anchors[0].index = 0;

  anchors[1].id = "A2";
  anchors[1].anchorX = 50;
  anchors[1].anchorY = 20;
  anchors[1].index = 0;

  anchors[2].id = "A3";
  anchors[2].anchorX = 30;
  anchors[2].anchorY = 60;
  anchors[2].index = 0;

  Serial.println("Tag module configured.");
}

// Save a new measurement into the circular buffer for the given anchor.
void storeMeasurement(const String &anchorID, int distance) {
  for (int i = 0; i < 3; i++) {
    if (anchors[i].id == anchorID) {
      anchors[i].measurements[anchors[i].index] = distance;
      anchors[i].index = (anchors[i].index + 1) % 20;
      Serial.print("Stored measurement for ");
      Serial.print(anchorID);
      Serial.print(": ");
      Serial.println(distance);
      return;
    }
  }
  Serial.println("Unknown Anchor ID: " + anchorID);
}

// Parse the custom payload from an anchor.
// Expected format: "A1XXYYDDD"
// - First 2 chars: Anchor ID (e.g. "A1")
// - Next 2 chars: X coordinate (ignored here, as tag already knows it)
// - Next 2 chars: Y coordinate
// - Last 3 chars: Distance (in cm)
void parsePayload(const String &payload) {
  if (payload.length() < 9) {
    Serial.println("Invalid payload: " + payload);
    return;
  }
  String anchorID = payload.substring(0, 2);
  String xStr = payload.substring(2, 4);
  String yStr = payload.substring(4, 6);
  String dStr = payload.substring(6, 9);

  int distance = dStr.toInt();
  Serial.print("Received from anchor ");
  Serial.print(anchorID);
  Serial.print(" (Location: ");
  Serial.print(xStr);
  Serial.print(",");
  Serial.print(yStr);
  Serial.print(") Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  storeMeasurement(anchorID, distance);
}

void loop() {
  // Listen for incoming data from the UWB module.
  if (uwbSerial.available()) {
    String line = uwbSerial.readStringUntil('\n');
    line.trim();
    Serial.println("Raw received: " + line);
    // Expected incoming format: "+TAG_RCV=<payload_length>,<payload>"
    int commaIndex = line.indexOf(",");
    if (commaIndex != -1 && commaIndex < line.length()-1) {
      String payload = line.substring(commaIndex+1);
      parsePayload(payload);
    }
  }
}
