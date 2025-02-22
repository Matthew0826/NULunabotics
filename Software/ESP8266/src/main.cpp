#include <Arduino.h>
#include <SoftwareSerial.h>

const byte rxPin = 5;
const byte txPin = 4;

// Set up a new SoftwareSerial object
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup() {
  // put your setup code here, to run once:

  //Define the pins
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  //Set the baud rate for the SoftwareSerial Object
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  mySerial.println("Hello World");
  delay(1000);
}