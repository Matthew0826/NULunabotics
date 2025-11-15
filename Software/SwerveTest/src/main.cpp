#include <Arduino.h>
#include <Servo.h>

Servo motor1;

int target = 500;
const double kP = 0.05;

void setup() {
  pinMode(2, INPUT);
  motor1.attach(11);

  Serial.begin(115200);  // Start serial communication at 115200 baud
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read until newline character
    input.trim(); // Remove any trailing \r or spaces
    
    target = input.toInt();

    delay(5);  // Optional: slight delay after processing
  }
  
  int currValue = analogRead(A1);
  int error = target - currValue;

  double output = max(min(1, kP * error), -1);

  motor1.writeMicroseconds(1500 + int(output * 1000)); 

  Serial.println( currValue );
} 

  // int analogValue = analogRead(A1);  // Read analog input on pin A0
  // Serial.println(analogValue);       // Print the value to the Serial Monitor
  // motor1.writeMicroseconds(1750); 
  // delay(2000);                        // Wait half a second
  // analogValue = analogRead(A1);  // Read analog input on pin A0
  // Serial.println(analogValue);       // Print the value to the Serial Monitor
  // motor1.writeMicroseconds(1250); 
  // delay(2000);                        // Wait half a second


// #include <Arduino.h>
// #include <SoftwareSerial.h>
// #include <Servo.h>

// volatile int leftEncoderPos = 0;
// volatile int rightEncoderPos = 0;

// int time = 0;

// const int leftEncoderPinA = 2;   // Interrupt pin
// const int leftEncoderPinB = 4;
// const int rightEncoderPinA = 3;  // Interrupt pin
// const int rightEncoderPinB = 5;
// // // Define the onboard LED motors
// Servo motor1;
// Servo motor2;

// void updateLeftEncoder();
// void updateRightEncoder();

// void spinwheelRaw();
// void rotateWheelRaw();

// void setup() {
//   // pinMode(leftEncoderPinA, INPUT);
//   // pinMode(leftEncoderPinB, INPUT);
//   // pinMode(rightEncoderPinA, INPUT);
//   // pinMode(rightEncoderPinB, INPUT);
//   motor1.attach(11); // PWM signal pin to Spark Mini signal input
//   // motor2.attach(10);
//   motor1.writeMicroseconds(1750); // 50% forward
//   // motor2.writeMicroseconds(1750);
//   // spinwheelRaw();
//   // attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
//   // attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);

//   Serial.begin(9600);
// }

// void loop() {
//   // Print both encoder positions
//   // Serial.print("Left: ");
//   // Serial.print(leftEncoderPos);
//   // Serial.print(" | Right: ");
//   // Serial.println(rightEncoderPos);
//   // time++;
//   // if (time < 10 ){
//   //   spinwheelRaw();
//   // }
//   // else if (time == 10 ){
//   //   leftEncoderPos = 0;
//   //   rightEncoderPos = 0 ;
//   // }
//   // else if (time < 20){
//   //   rotateWheelRaw();
//   // }else{
//   //   time = 0;
//   //   leftEncoderPos = 0;
//   //   rightEncoderPos = 0 ;
//   // }
//   // delay(100);
// }

// void updateLeftEncoder() {
//   int a = digitalRead(leftEncoderPinA);
//   int b = digitalRead(leftEncoderPinB);
//   if (a == b) {
//     leftEncoderPos++;
//   } else {
//     leftEncoderPos--;
//   }
// }

// void updateRightEncoder() {
//   int a = digitalRead(rightEncoderPinA);
//   int b = digitalRead(rightEncoderPinB);
//   if (a == b) {
//     rightEncoderPos++;
//   } else {
//     rightEncoderPos--;
//   }
// }

// void spinwheelRaw(){
//   int err = rightEncoderPos - leftEncoderPos;
//   motor1.writeMicroseconds(1750 - err); // 50% forward
//   motor2.writeMicroseconds(1750 + err);

// }

// void rotateWheelRaw(){
//   int err = rightEncoderPos + leftEncoderPos;
//   motor1.writeMicroseconds(1750 + err); // 50% forward
//   motor2.writeMicroseconds(1250 - err);

// }