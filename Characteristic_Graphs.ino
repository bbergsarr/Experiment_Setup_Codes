#include <CircularBuffer.hpp>
#include <AFMotor.h>

// Hardware configuration
AF_DCMotor motor(4);                // Motor connected to port 4
const int ENCODER_PIN = A0;         // Encoder signal pin
const int LED_PIN = A4;             // LED pin for status
const int BUTTON_PIN = A5;          // Button pin to start test
const int PULSES_PER_REV = 20;      // Encoder pulses per revolution
const int BUFFER_SIZE = 20;         // Size of pulse period buffers

// Timing constants (in microseconds)
const unsigned long STALL_CHECK_INTERVAL = 200000;  // 0.2s between stall checks
const unsigned long FULL_STALL_TIMEOUT = 1000000;   // 1.0s for full stall detection
const float DECAY_RATE = 0.2f;                      // 20% RPM decay per partial stall step

// PWM and test configuration
const int PWM_STEP = 5;             // PWM increment
const int ARRAY_SIZE = 256 / PWM_STEP + 1;  // Number of PWM steps (0 to 255, inclusive)
float rpm_values[ARRAY_SIZE];       // Array to store average RPM for each PWM value
const unsigned long TEST_INTERVAL = 5000;  // 1 second per PWM step (in milliseconds)
const unsigned long RPM_SAMPLE_INTERVAL = 50;  // Sample RPM every 50ms

// Data buffers and state variables
CircularBuffer<long, BUFFER_SIZE> risingEdgeBuffer;  // Buffer for rising edge periods
CircularBuffer<long, BUFFER_SIZE> fallingEdgeBuffer; // Buffer for falling edge periods
unsigned long lastRisingEdgeTime = 0;                // Time of last rising edge
unsigned long lastFallingEdgeTime = 0;               // Time of last falling edge
unsigned long lastStallCheckTime = 0;                // Time of last stall check
float currentRPM = 0.0;                              // Current motor speed in RPM
float lastValidRPM = 0.0;                            // Last RPM before stall began
uint8_t motorDirection = FORWARD;                    // Current motor direction
bool motorStalled = false;                           // Stall state flag

void setup() {
  Serial.begin(115200);              // Match Python baud rate
  pinMode(ENCODER_PIN, INPUT);       // Encoder pin as input
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Button with internal pull-up
  pinMode(LED_PIN, OUTPUT);          // LED as output
  digitalWrite(LED_PIN, LOW);        // LED off initially
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {  // Button pressed (LOW with pull-up)
    digitalWrite(LED_PIN, HIGH);         // Turn on LED during test
    runMotorCharacterization();          // Run the test
    sendData();                          // Send results to Python
    digitalWrite(LED_PIN, LOW);          // Turn off LED when done
    while (digitalRead(BUTTON_PIN) == LOW) delay(10);  // Wait for button release
  }
  processEncoder();                      // Continuously monitor encoder
  manageStallCondition();                // Handle stall detection
  delay(1);                              // Small delay to prevent CPU overload
}

// Run motor characterization test
void runMotorCharacterization() {
  for (int i = 0; i < ARRAY_SIZE; i++) {
    int pwm = i * PWM_STEP;             // Calculate PWM value (0, 5, 10, ..., 255)
    motor.setSpeed(pwm);                // Set motor speed
    motor.run(FORWARD);                 // Run motor forward
    unsigned long startTime = millis(); // Start timing for this PWM step

    // Clear buffers and reset variables
    risingEdgeBuffer.clear();
    fallingEdgeBuffer.clear();
    lastRisingEdgeTime = lastFallingEdgeTime = 0;
    lastValidRPM = 0.0;
    motorStalled = false;

    // Variables for averaging RPM
    float rpmSum = 0.0;
    int sampleCount = 0;
    unsigned long lastSampleTime = millis();

    // Run for 1 second, collecting RPM samples
    while (millis() - startTime < TEST_INTERVAL) {
      processEncoder();                 // Update buffers with pulses
      manageStallCondition();           // Handle stalls and update RPM
      if (millis() - lastSampleTime >= RPM_SAMPLE_INTERVAL) {
        rpmSum += currentRPM;
        sampleCount++;
        lastSampleTime = millis();
      }
    }

    // Calculate and store the average RPM
    rpm_values[i] = (sampleCount > 0) ? (rpmSum / sampleCount) : 0.0;

    // Stop motor briefly between steps
    motor.setSpeed(0);
    delay(50);                          // 50ms pause to settle
  }
}

// Process a single encoder pulse (rising or falling)
void processPulse(bool isRisingEdge) {
  unsigned long currentTime = micros();
  unsigned long* lastEdgeTime = isRisingEdge ? &lastRisingEdgeTime : &lastFallingEdgeTime;

  if (*lastEdgeTime != 0) {
    long pulsePeriod = currentTime - *lastEdgeTime;
    if (motorDirection == BACKWARD) pulsePeriod = -pulsePeriod;
    (isRisingEdge ? risingEdgeBuffer : fallingEdgeBuffer).push(pulsePeriod);
    updateRPM();
    lastValidRPM = currentRPM;
  }

  *lastEdgeTime = currentTime;
  lastStallCheckTime = currentTime;
  motorStalled = false;
}

// Detect encoder edges and process pulses
void processEncoder() {
  static int previousState = LOW;
  int currentState = digitalRead(ENCODER_PIN);

  if (currentState != previousState) {
    if (currentState == HIGH) processPulse(true);
    else processPulse(false);
    previousState = currentState;
  }
}

// Calculate current RPM based on buffered periods
void updateRPM() {
  if (risingEdgeBuffer.isEmpty() || fallingEdgeBuffer.isEmpty()) {
    currentRPM = 0.0;
    return;
  }

  long risingSum = 0, fallingSum = 0;
  size_t risingCount = risingEdgeBuffer.size();
  size_t fallingCount = fallingEdgeBuffer.size();

  for (size_t i = 0; i < risingCount; i++) risingSum += abs(risingEdgeBuffer[i]);
  for (size_t i = 0; i < fallingCount; i++) fallingSum += abs(fallingEdgeBuffer[i]);

  float avgRisingPeriod = static_cast<float>(risingSum) / risingCount;
  float avgFallingPeriod = static_cast<float>(fallingSum) / fallingCount;
  float averagePeriod = (avgRisingPeriod + avgFallingPeriod) / 2.0;

  currentRPM = (averagePeriod > 0) ? (60000000.0 / (averagePeriod * PULSES_PER_REV)) : 0.0;
}

// Calculate period corresponding to a given RPM
long periodForRPM(float rpm) {
  if (rpm <= 0) return 1000000;
  return static_cast<long>(60000000.0 / (rpm * PULSES_PER_REV));
}

// Manage stall detection and smooth RPM decay
void manageStallCondition() {
  unsigned long currentTime = micros();
  if (currentTime - lastStallCheckTime < STALL_CHECK_INTERVAL) return;

  unsigned long timeSinceLastEdge = currentTime - max(lastRisingEdgeTime, lastFallingEdgeTime);

  if (timeSinceLastEdge >= FULL_STALL_TIMEOUT) {
    if (!motorStalled) {
      motorStalled = true;
      decayToZeroRPM();
    }
  } else if (timeSinceLastEdge >= STALL_CHECK_INTERVAL) {
    float decayedRPM = lastValidRPM * (1.0 - DECAY_RATE);
    long decayedPeriod = periodForRPM(decayedRPM);
    int periodsToInsert = ceil(max(risingEdgeBuffer.size(), fallingEdgeBuffer.size()) * DECAY_RATE);
    insertPeriodsIntoBuffers(decayedPeriod, periodsToInsert);
    lastValidRPM = decayedRPM;
    updateRPM();
  }

  lastStallCheckTime = currentTime;
}

// Insert periods corresponding to a decaying RPM
void insertPeriodsIntoBuffers(long period, int count) {
  for (int i = 0; i < count; i++) {
    if (!risingEdgeBuffer.isFull()) risingEdgeBuffer.push(period);
    if (!fallingEdgeBuffer.isFull()) fallingEdgeBuffer.push(period);
  }
}

// Fully decay RPM to 0 by filling buffers with large periods
void decayToZeroRPM() {
  long zeroPeriod = periodForRPM(0.0);
  while (!risingEdgeBuffer.isFull()) risingEdgeBuffer.push(zeroPeriod);
  while (!fallingEdgeBuffer.isFull()) fallingEdgeBuffer.push(zeroPeriod);
}

// Send RPM data to Python
void sendData() {
  Serial.write(0xAA);  // Send start byte for synchronization
  delay(10);
  Serial.write((uint8_t)ARRAY_SIZE);  // Send array size
  delay(10);

  for (int i = 0; i < ARRAY_SIZE; i++) {
    Serial.write((byte*)&rpm_values[i], sizeof(float));  // Send each float
    delay(5);  // Small delay for reliable transmission
  }
}