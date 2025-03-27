int pulseWidth = 1500; // Default pulse width (neutral position)
unsigned long lastUpdateTime = 0; // Tracks the last time a PWM pulse was sent
const unsigned long pwmPeriod = 20000; // 20ms period (50Hz)
String serialInput = ""; // Buffer for serial input

void setup() {
  pinMode(D4, OUTPUT); // Motor control pin
  Serial.begin(9600); // Initialize serial communication at 9600 baud
}

void loop() {
  // Handle non-blocking serial input
  if (Serial.available() > 0) {
    char incomingChar = Serial.read(); // Read one character at a time
    if (incomingChar == '\n') { // If a newline character is received
      // Process the complete input
      float inputValue = serialInput.toFloat(); // Convert input to float
      if (inputValue >= -1.0 && inputValue <= 1.0) {
        // Map 0-1 to 500-2500 and update pulseWidth
        pulseWidth = 1500 + (inputValue * 1000);
        if (pulseWidth > 2490) {
          pulseWidth = 2490;
        } else if (pulseWidth < 510) {
          pulseWidth = 510;
        }
      }
      serialInput = ""; // Clear the buffer
    } else {
      serialInput += incomingChar; // Append character to buffer
    }
  }

  // Handle PWM timing
  unsigned long currentTime = micros(); // Use micros() for precise timing
  if (currentTime - lastUpdateTime >= pwmPeriod) {
    // Start a new PWM cycle
    lastUpdateTime = currentTime;
    digitalWrite(D4, HIGH); // Turn on motor
    delayMicroseconds(pulseWidth); // Pulse width (e.g., 500 to 2500)
    digitalWrite(D4, LOW); // Turn off motor
  }
}