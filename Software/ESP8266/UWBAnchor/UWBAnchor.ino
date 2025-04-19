//
// THIS IS A STATIONARY BEACON
//

#include <SoftwareSerial.h>

#define ANCHOR_ID 0
const String ANCHOR_ID_STRING = "0";

// UWB stands for Ultra-Wideband and it's how the beacons ("anchors") find the distance to this "tag" board that's sitting on the robot
// Here's a link for reference on thier "AT" protocol: https://reyax.com//upload/products_download/download_file/AT_Command_RYUW122.pdf
SoftwareSerial uwbSerial(D6, D5);  // (RX, TX) NOTE: THEY ARE FLIPPED FROM THE ONES IN THE CIRCUIT!!

// Anchor beacon configuration
const String TAG_ADDRESS = "TAG00010";      // Tag's unique address (8 bytes)
const String ANCHOR_ADDRESS = "ANCH001" + ANCHOR_ID_STRING;   // This anchor’s address (8 bytes)
const String NETWORK_ID = "NEULUNAB";       // Must match on both sides
const String UWB_BAUD = "115200";
const String PASSWORD = "FABC0002EEDCAA90FABC0002EEDCAA90";
const String MODE = "1";   // set module to ANCHOR mode

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
  // uwbSerial MUST run on 115200 baud rate, or else it fails to send anything
  uwbSerial.begin(115200);
  delay(2000);

  Serial.println("Setting up...");
  // Configure the UWB module for ANCHOR mode
  sendCommand("AT+IPR=" + UWB_BAUD);
  sendCommand("AT+MODE=" + MODE);
  sendCommand("AT+NETWORKID=" + NETWORK_ID);
  sendCommand("AT+ADDRESS=" + ANCHOR_ADDRESS);
  sendCommand("AT+CPIN=" + PASSWORD);

  Serial.println("Anchor module configured.");
}

// Performs a ranging transaction then sends a custom message containing location and distance.
void performRanging() {
  // Wait for the response from the module (expected format similar to:
  // "+ANCHOR_RCV=TAG0010,4,PONG,040 cm")
  unsigned long startTime = millis();
  String resp = "";
  while (millis() - startTime < 500) {
    while (uwbSerial.available()) {
      resp += (char)uwbSerial.read();
    }
  }
  Serial.println("Ranging response: " + resp);

  // 2. Parse out the measured distance from the response
  //    (Assumes response contains ",<distance> cm" at the end)
  // TODO: Find where distance is in the string some other way
  int cmIndex = resp.indexOf("cm");
  int commaIndex = resp.lastIndexOf(",", cmIndex);
  String distanceStr = "";
  if (commaIndex != -1 && cmIndex != -1) {
    distanceStr = resp.substring(commaIndex + 1, cmIndex);
    distanceStr.trim();
  }
  if (distanceStr == "") {
    distanceStr = "0";
  }
  // Serial.println("Measured distance: " + distanceStr + " cm");

  // 3. Construct a custom payload to send to the tag.
  // Format (4 characters):
  // - First 1: Anchor ID (e.g. "1")
  // - Last 3: distance in cm (3-digit, zero-padded)
  // Example: "1" + "040" → "1040"
  char payload[5];  // 4 characters + null terminator
  int distance = distanceStr.toInt();
  Serial.println(distance);
  sprintf(payload, "%d%03d", ANCHOR_ID, distance);
  String payloadStr = String(payload);
  Serial.println("Custom payload: " + payloadStr);

  // 4. Send the custom message.
  // Using a payload length (here, 4) that is different from the ranging phase to avoid a new measurement.
  String sendCmd = "AT+ANCHOR_SEND=" + TAG_ADDRESS + ",4," + payloadStr;
  sendCommand(sendCmd);
}

void loop() {
  performRanging();
  delay(20);
  // // Check if there's data available on Serial
  // if (Serial.available()) {
  //   String command = Serial.readStringUntil('\n');  // Read a line from Serial
  //   sendCommand(command);  // Send it using your custom function
  // }

  // // Check if there's data available on uwbSerial
  // if (uwbSerial.available()) {
  //   String response = uwbSerial.readStringUntil('\n');  // Read a line from uwbSerial
  //   Serial.println(response);  // Print it to the standard Serial
  // }
}
