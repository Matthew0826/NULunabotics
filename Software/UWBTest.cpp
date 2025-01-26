
#include <Arduino.h>
#include <SoftwareSerial.h>
#define LED 2

SoftwareSerial mySerial(D7, D8); //Define hardware connections

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, World!");
  mySerial.begin(9600);
  delay(500);
  bool ok = false;
  while( !ok ){
    mySerial.write("AT\r\n");
    while( mySerial.available() ){
      Serial.print(char( mySerial.read()) );
      ok = true;
    }
    delay(500);
  }


  mySerial.write("AT+CPIN=FABC0002EEDCAA90FABC0002EEDCAA90\r\n");
  delay( 500 );
  Serial.println("Reading...");
  while( mySerial.available() ){
    Serial.print(char( mySerial.read()) );
  }

}

void loop() {
    mySerial.write("AT+ANCHOR_SEND=NULBTAGG,4,TEST\r\n");
  delay(300);
  Serial.println("Reading...");
  while( mySerial.available() ){
    Serial.print(char( mySerial.read()) );
  }

}

//REMINDER: You have to power cycle the chip to get this to work

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/


// #include <Arduino.h>
// #include <SoftwareSerial.h>
// #define LED 2

// SoftwareSerial mySerial(D7, D8); //Define hardware connections

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   Serial.println("Hello, World!");
//   mySerial.begin(9600);
//   delay(500);
//   bool ok = false;
//   while( !ok ){
//     mySerial.write("AT\r\n");
//     while( mySerial.available() ){
//       Serial.print(char( mySerial.read()) );
//       ok = true;
//     }
//     delay(500);
//   }


//   mySerial.write("AT+CPIN=FABC0002EEDCAA90FABC0002EEDCAA90\r\n");
//   delay( 500 );
//   Serial.println("Reading...");
//   while( mySerial.available() ){
//     Serial.print(char( mySerial.read()) );
//   }
 

// }

// void loop() {
//     delay(500);
//   Serial.println("Reading...");
//   while( mySerial.available() ){
//     Serial.print(char( mySerial.read()) );
//   }
// }
