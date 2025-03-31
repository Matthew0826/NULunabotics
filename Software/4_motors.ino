#define MOTOR_COUNT 4
// For communication with ROS on Raspberry Pi
#define PACKET_SIZE 4
// For motor PWM
#define DEVICE_ANALOG_RANGE 1023.0
#define PWM_FREQUENCY 50
#define FORWARD_PULSE_WIDTH 2500.0
#define REVERSE_PULSE_WIDTH 500.0
#define FORWARD_WRITE ((int)(DEVICE_ANALOG_RANGE*FORWARD_PULSE_WIDTH*((float)PWM_FREQUENCY)/1000000.0))
#define REVERSE_WRITE ((int)(DEVICE_ANALOG_RANGE*REVERSE_PULSE_WIDTH*((float)PWM_FREQUENCY)/1000000.0))

typedef struct {
  byte pin;
  byte pulseWidth;
} motor_t;

byte pins[MOTOR_COUNT] = {D0, D2, D3, D5};
motor_t motors[MOTOR_COUNT];

unsigned long lastUpdateTime = 0;

// Function to sort motors by pulseWidth in ascending order
void sortMotors() {
  for (int i = 0; i < MOTOR_COUNT - 1; i++) {
    for (int j = i + 1; j < MOTOR_COUNT; j++) {
      if (motors[i].pulseWidth > motors[j].pulseWidth) {
        // Swap motor_t elements to sort by pulseWidth
        motor_t temp = motors[i];
        motors[i] = motors[j];
        motors[j] = temp;
      }
    }
  }
}

// Format:
// Header: 0xFFFF
// Motor to move (1 byte)
// Amount to move (1 byte): 0 is backwards, 255 is forwards, and 127 is neutral
byte serialInput[PACKET_SIZE]; // Buffer for serial input
int serialIndex = 0;

void setup() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motors[i].pin, OUTPUT); // Motor control pin
    motors[i] = {pins[i], 127};
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
          byte pin = pins[motor];
          for (int i = 0; i < MOTOR_COUNT; i++) {
            if (pin == motors[i].pin) {
              motors[i].pulseWidth = serialInput[3];
              break;
            }
          }
          sortMotors();
        }
      }
      serialIndex = 0;
    }
  }
  
  unsigned long currentTime = micros();
  if (currentTime - lastUpdateTime >= 20000) {
    lastUpdateTime = currentTime;
    for (int i = 0; i < MOTOR_COUNT; i++) {
      digitalWrite(motors[i].pin, HIGH);
    }
    int delaySoFar = 0;
    for (int i = 0; i < MOTOR_COUNT; i++) {
      delayMicroseconds((int)(1490.0 + 1000.0*(((float)((int)motors[i].pulseWidth - delaySoFar))/127.5 - 1.0)));
      delaySoFar += (int)motors[i].pulseWidth;
      digitalWrite(motors[i].pin, LOW);
    }
    
  }
}
