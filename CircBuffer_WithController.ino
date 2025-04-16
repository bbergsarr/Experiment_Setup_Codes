#include <CircularBuffer.hpp>
#include <AFMotor.h>

// Hardware configuration
AF_DCMotor motor(4);                // Motor connected to port 4
const int ENCODER_PIN = A0;         // Encoder signal pin
const int PULSES_PER_REV = 20;      // Encoder pulses per revolution
const int BUFFER_SIZE = 10;         // Size of pulse period buffers

// Timing constants (in microseconds)
const unsigned long STALL_CHECK_INTERVAL = 200000;  // 0.2s between stall checks
const unsigned long FULL_STALL_TIMEOUT = 1000000;   // 1.0s for full stall detection
const float DECAY_RATE = 0.2f;                      // 20% RPM decay per partial stall step

// PID configuration
const float KP = 1.0;               // Proportional gain
const float KI = 0.001;               // Integral gain
const float KD = 0.005;              // Derivative gain

float targetRPM = 50.0;             // Target RPM (changeable)
float integral = 0.0;               // Integral term accumulator
float lastError = 0.0;              // Last error for derivative term
unsigned long lastPIDTime = 0;      // Last PID update time
const unsigned long PID_INTERVAL = 500000;  // PID update every 100ms (in microseconds)
int currentPWM = 0;                 // Global variable to track current PWM

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
  Serial.begin(115200);              // Higher baud rate for faster serial output
  pinMode(ENCODER_PIN, INPUT);       // Configure encoder pin as input
  currentPWM = rpmToPWM(targetRPM);  // Set initial PWM based on target RPM
  motor.setSpeed(currentPWM);        // Apply initial PWM
  motor.run(FORWARD);                // Start motor in forward direction
  lastPIDTime = micros();            // Initialize PID timing
}

void loop() {
  processEncoder();                  // Check for encoder pulses
  manageStallCondition();            // Handle stall detection and decay
  updatePID();                       // Update PID controller
  delay(1);                          // Small delay to prevent CPU overload
}

// Detect encoder edges and process pulses
void processEncoder() {
  static int previousState = LOW;    // Previous encoder state
  int currentState = digitalRead(ENCODER_PIN);

  if (currentState != previousState) {
    if (currentState == HIGH) {
      processPulse(true);            // Rising edge detected
    } else {
      processPulse(false);           // Falling edge detected
    }
    previousState = currentState;
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
    lastValidRPM = currentRPM;      // Update last valid RPM before stall
  }

  *lastEdgeTime = currentTime;
  lastStallCheckTime = currentTime;
  motorStalled = false;             // Reset stall flag on new pulse
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
  if (rpm <= 0) return 1000000;  // Large period for 0 RPM
  return static_cast<long>(60000000.0 / (rpm * PULSES_PER_REV));
}

// Manage stall detection and smooth RPM decay
void manageStallCondition() {
  unsigned long currentTime = micros();
  if (currentTime - lastStallCheckTime < STALL_CHECK_INTERVAL) {
    if (motorStalled) {
      currentRPM = 0.0;  // Ensure RPM is 0 during stall
    }
    return;
  }

  unsigned long timeSinceLastEdge = currentTime - max(lastRisingEdgeTime, lastFallingEdgeTime);

  if (timeSinceLastEdge >= FULL_STALL_TIMEOUT) {
    if (!motorStalled) {
      motorStalled = true;
      decayToZeroRPM();
    }
    currentRPM = 0.0;  // Full stall, RPM = 0
  } else if (timeSinceLastEdge >= STALL_CHECK_INTERVAL) {
    float decayedRPM = lastValidRPM * (1.0 - DECAY_RATE);
    long decayedPeriod = periodForRPM(decayedRPM);
    int periodsToInsert = ceil(max(risingEdgeBuffer.size(), fallingEdgeBuffer.size()) * DECAY_RATE);

    insertPeriodsIntoBuffers(decayedPeriod, periodsToInsert);
    lastValidRPM = decayedRPM;
    updateRPM();  // Update RPM with decayed value
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

// Set motor direction and reset timing
void setMotorDirection(uint8_t direction) {
  motorDirection = direction;
  motor.run(direction);
  lastRisingEdgeTime = lastFallingEdgeTime = micros();
}

// Convert RPM to PWM (placeholder function, adjustable)
int rpmToPWM(float rpm) {
  // Linear approximation: PWM = 5 * RPM, capped at 255 //FONKSIYON DÖNÜŞÜM GRAFİĞİN BURADA YER ALACAK  RPM VS PWM OLAN !!!!!
  int pwm = static_cast<int>(5.0 * rpm);
  if (pwm > 255) pwm = 255;
  if (pwm < 0) pwm = 0;
  return pwm;
}

// PID controller to adjust PWM based on RPM error
void updatePID() {
  unsigned long currentTime = micros();
  if (currentTime - lastPIDTime < PID_INTERVAL) return;  // Update every 100ms

  // Calculate error
  float error = targetRPM - currentRPM;
  unsigned long dt = currentTime - lastPIDTime;  // Time difference in microseconds
  float dtSeconds = dt / 1000000.0;              // Convert to seconds

  // Proportional term
  float proportional = KP * error;

  // Integral term (accumulated error)
  integral += error * dtSeconds;
  float integralTerm = KI * integral;

  // Derivative term (rate of error change)
  float derivative = KD * (error - lastError) / dtSeconds;

  // Total PID output (PWM adjustment)
  float pidOutput = proportional + integralTerm + derivative;

  // Current PWM and new PWM calculation
  int newPWM = currentPWM + static_cast<int>(pidOutput);

  // Constrain PWM to valid range (0-255)
  if (newPWM > 255) newPWM = 255;
  if (newPWM < 0) newPWM = 0;

  // Apply new PWM to motor and update currentPWM
  motor.setSpeed(newPWM);
  currentPWM = newPWM;

  // Print debug info to Serial
  Serial.print("[RPM] Measured: ");
  Serial.print(currentRPM, 2);
  Serial.print(" | Target: ");
  Serial.print(targetRPM, 2);
  Serial.print(" | Error: ");
  Serial.println(error, 2);

  // Update last values
  lastError = error;
  lastPIDTime = currentTime;
}