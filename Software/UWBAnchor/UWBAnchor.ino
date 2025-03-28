#include <SoftwareSerial.h>

// Create a SoftwareSerial interface on pins RX:13, TX:15 (D7 and D8)
SoftwareSerial uwbSerial(13, 15);  // (RX, TX)

//–– Configuration ––//
const String TAG_ADDRESS = "TAG00010";      // Tag's unique address (8 bytes)
const String ANCHOR_ADDRESS = "ANCH0010";   // This anchor’s address (8 bytes)
const String NETWORK_ID = "NEULUNAB";       // Must match on both sides

// Fixed anchor location (assumed to be two-digit values for compact payload)
const int anchorX = 10;  // e.g. coordinate X = 10
const int anchorY = 20;  // e.g. coordinate Y = 20

// Sends an AT command to the module and prints it to Serial
void sendCommand(const String &cmd) {
  uwbSerial.write(cmd);
  Serial.print("Sent command: ");
  Serial.println(cmd);
  delay(500);  // wait a bit for module to process
}

void setup() {
  Serial.begin(9600);
  uwbSerial.begin(9600);
  delay(1000);

  // Configure the UWB module for ANCHOR mode
  sendCommand("AT+MODE=1");  // set module to ANCHOR mode
  sendCommand("AT+NETWORKID=" + NETWORK_ID);
  sendCommand("AT+ADDRESS=" + ANCHOR_ADDRESS);
  sendCommand("AT+CPIN=FABC0002EEDCAA90FABC0002EEDCAA90\r\n");

  Serial.println("Anchor module configured.");
}

// Performs a ranging transaction then sends a custom message containing location and distance.
void performRanging() {
  // 1. Perform ranging using a matching payload ("PING")
  String rangingCmd = "AT+ANCHOR_SEND=" + TAG_ADDRESS + ",4,PING";
  uwbSerial.write(rangingCmd);
  Serial.println("Sent ranging command: " + rangingCmd);

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
  int cmIndex = resp.indexOf("cm");
  int commaIndex = resp.lastIndexOf(",", cmIndex);
  String distanceStr = "";
  if (commaIndex != -1 && cmIndex != -1) {
    distanceStr = resp.substring(commaIndex + 1, cmIndex);
    distanceStr.trim();
  }
  if (distanceStr == "") {
    distanceStr = "000";
  }
  Serial.println("Measured distance: " + distanceStr + " cm");

  // 3. Construct a custom payload to send to the tag.
  // Format (9 characters):
  // - First 2: Anchor ID (e.g. "A1")
  // - Next 2: X coordinate (2-digit, zero-padded)
  // - Next 2: Y coordinate (2-digit, zero-padded)
  // - Last 3: distance in cm (3-digit, zero-padded)
  // Example: "A1" + "10" + "20" + "040" → "A11020040"
  char payload[10];  // 9 characters + null terminator
  int d = distanceStr.toInt();
  sprintf(payload, "A1%02d%02d%03d", anchorX, anchorY, d);
  String payloadStr = String(payload);
  Serial.println("Custom payload: " + payloadStr);

  // 4. Send the custom message.
  // Using a payload length (here, 9) that is different from the ranging phase to avoid a new measurement.
  String sendCmd = "AT+ANCHOR_SEND=" + TAG_ADDRESS + ",9," + payloadStr;
  uwbSerial.write(sendCmd);
  Serial.println("Sent custom message: " + sendCmd);

  delay(200);
}

void loop() {
  performRanging();
  delay(1000);  // repeat every second
}
