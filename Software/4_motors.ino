#define MOTOR_COUNT 4
// For communication with ROS on Raspberry Pi
#define PACKET_SIZE 4
#define DEVICE_ANALOG_RANGE 1023.0
// For motor PWM
#define PWM_FREQUENCY 50
#define FORWARD_PULSE_WIDTH 2500.0
#define REVERSE_PULSE_WIDTH 500.0
#define FORWARD_WRITE ((int)(DEVICE_ANALOG_RANGE*FORWARD_PULSE_WIDTH*((float)PWM_FREQUENCY)/1000000.0))
#define REVERSE_WRITE ((int)(DEVICE_ANALOG_RANGE*REVERSE_PULSE_WIDTH*((float)PWM_FREQUENCY)/1000000.0))

byte motorPins[MOTOR_COUNT] = {D4, D5, D6, D7}; // TODO: Use actual pins!

// unsigned long lastUpdateTime = 0;

// Format:
// Header: 0xFFFF
// Motor to move (1 byte)
// Amount to move (1 byte): 0 is backwards, 255 is forwards, and 127 is neutral
byte serialInput[PACKET_SIZE]; // Buffer for serial input
int serialIndex = 0;

void setup() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motorPins[i], OUTPUT); // Motor control pin
  }
  Serial.begin(9600); // Used to read commands from ROS
  analogWriteFreq(PWM_FREQUENCY);
}

void loop() {
  // Handle non-blocking serial input
  if (Serial.available() > 0) {
    // Create a moving window of bytes
    byte input = Serial.read();
    serialInput[serialIndex] = input;
    serialIndex++;
    if (serialIndex >= PACKET_SIZE) {
      // When the header is detected,
      if (serialInput[0] == 0xFF && serialInput[1] == 0xFF) {
        // Parse the packet
        int motor = (int)serialInput[2];
        if (motor >= 0 && motor < MOTOR_COUNT) {
          // 25 and 128 for reverse and forward write
          analogWrite(motorPins[motor], map((int)serialInput[3], 0, 255, REVERSE_WRITE, FORWARD_WRITE));
        }
      }

      serialIndex = 0;
    }
  }
  
  // unsigned long currentTime = micros();
  // if (currentTime - lastUpdateTime >= (1000000 / PWM_FREQUENCY)) {
  //   lastUpdateTime = currentTime;
  //   for (int i = 0; i < MOTOR_COUNT; i++) {
  //     analogWrite(motorPins[i], (int)motorPulseWidths[i]);
  //   }
  // }
}
