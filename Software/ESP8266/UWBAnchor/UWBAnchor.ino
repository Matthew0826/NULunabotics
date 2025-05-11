//
// THIS IS A STATIONARY BEACON
//

#include <SoftwareSerial.h>

#define ANCHOR_ID 1
const String ANCHOR_ID_STRING = "1";

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
  // Serial.println(cmd);
  String response = "";
  // Wait until we get a response
  while (response.length() == 0) {
    if (uwbSerial.available()) {
      response = uwbSerial.readStringUntil('\n');  // Read a line from uwbSerial
      // Serial.println(response);  // Print it to the standard Serial
    }
    delay(10);
  }
  // Serial.println("Command done!");
  delay(10);
}

void setup() {
  // Serial.begin(9600);
  // uwbSerial MUST run on 115200 baud rate, or else it fails to send anything
  uwbSerial.begin(115200);

  // Serial.println("Setting up...");
  // Configure the UWB module for ANCHOR mode
  // sendCommand("AT+RESET");
  sendCommand("AT");
  sendCommand("AT+MODE=" + MODE);
  sendCommand("AT+NETWORKID=" + NETWORK_ID);
  sendCommand("AT+ADDRESS=" + ANCHOR_ADDRESS);
  sendCommand("AT+CPIN=" + PASSWORD);

  // Serial.println("Anchor module configured.");
}

void performRanging() {
  const int maxLength = 50;
  char resp[maxLength + 1];  // +1 for null terminator
  int idx = 0;
  unsigned long startTime = millis();

  // Read exactly expectedLength characters or until timeout
  while ((millis() - startTime < 100) && (idx < maxLength) && !(idx > 0 && resp[idx-1] =='c' && resp[idx] == 'm')) {
    if (uwbSerial.available()) {
      resp[idx++] = uwbSerial.read();
    }
  }
  resp[idx] = '\0';  // Null-terminate

  // Serial.print("Ranging response: ");
  // Serial.println(resp);

  // Extract distance (assumed at end after last comma)
  int distance = 0;
  char* lastComma = strrchr(resp, ',');
  if (lastComma && isdigit(*(lastComma + 1))) {
    distance = atoi(lastComma + 1);
  }

  // Serial.print("Measured distance: ");
  // Serial.println(distance);

  // Build payload
  char payload[5];
  sprintf(payload, "%d%03d", ANCHOR_ID, distance);

  // Serial.print("Custom payload: ");
  // Serial.println(payload);

  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+ANCHOR_SEND=%s,4,%s", TAG_ADDRESS, payload);
  sendCommand(String(cmd));
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
