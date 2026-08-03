#include <MPU6050_tockn.h>
#include <Wire.h>

// ESP32 Pin Definitions
#define LEFT_PWM      22
#define LEFT_DIR      23
#define LEFT_SC       34

#define RIGHT_PWM     27
#define RIGHT_DIR     26
#define RIGHT_SC      35

#define I2C_SDA       32
#define I2C_SCL       33

#define MAX_PWM        120
#define LEFT_FORWARD   HIGH
#define RIGHT_FORWARD  LOW
#define WHEEL_DIAMETER_M  0.165f
#define PULSES_PER_REV    45
#define METERS_PER_PULSE  (PI * WHEEL_DIAMETER_M / PULSES_PER_REV)

#define WHEELBASE_M  0.52f

// ─── FILTER SETTINGS ──────────────────────────────────────────────────────

#define YAW_FILTER_ALPHA  0.95f
#define RATE_FILTER_ALPHA  0.7f

MPU6050 mpu6050(Wire);

// ─── PID GAINS ──────────────────────────────────────────────────────────────

// Heading correction - MUCH more aggressive
#define KP_HEADING 8.0      // Was 3.0
#define KI_HEADING 0.5      // Was 0.1
#define KD_HEADING 1.0      // Was 0.2

// Wheel speed PIDs - ensure both wheels match exactly
#define KP_WHEEL 8.0        // Was 5.0
#define KI_WHEEL 0.5        // Was 0.2
#define KD_WHEEL 0.0  

// ─── MOTOR CHARACTERIZATION ──────────────────────────────────────────────────

#define SPEED_TO_PWM_GAIN 180.0f
#define MIN_PWM_START 25     // Increased

#define CONTROL_INTERVAL 20
#define IMU_UPDATE_INTERVAL 5

volatile unsigned long debounceUS = 1000UL;

// ─── ENCODER VARIABLES ──────────────────────────────────────────────────────

volatile long          leftPulses  = 0;
volatile unsigned long lastLeftUS  = 0;
volatile bool          leftFwd     = true;

volatile long          rightPulses  = 0;
volatile unsigned long lastRightUS  = 0;
volatile bool          rightFwd     = true;

int leftCurrentPWM = 0;
int rightCurrentPWM = 0;

float currentLeftSpeed = 0.0;
float currentRightSpeed = 0.0;

float leftIntegral = 0;
float rightIntegral = 0;
float leftPrevError = 0;
float rightPrevError = 0;

unsigned long lastControlTime = 0;

float targetLinearVelocity = 0.0;
float targetAngularVelocity = 0.0;

// ─── HEADING CONTROL ──────────────────────────────────────────────────────

float headingIntegral = 0;
float headingPrevError = 0;
float targetYaw = 0;
float currentYaw = 0;
float filteredYaw = 0;
bool lockHeading = false;
bool headingInitialized = false;

// ─── TURN RATE CONTROL ──────────────────────────────────────────────────────

float turnRateIntegral = 0;
float turnRatePrevError = 0;
float filteredAngularVelocity = 0;
float previousYaw = 0;
unsigned long lastYawTime = 0;
unsigned long lastIMUUpdate = 0;

// ─── ODOMETRY ──────────────────────────────────────────────────────────────

float odomX = 0.0;
float odomY = 0.0;

// ─── DEBUG ──────────────────────────────────────────────────────────────────

unsigned long lastDebugPrint = 0;
#define DEBUG_INTERVAL 300  // Print debug every 300ms

// ─── UTILITY FUNCTIONS ──────────────────────────────────────────────────────

float expMovingAverage(float newValue, float prevValue, float alpha) {
  return alpha * newValue + (1.0f - alpha) * prevValue;
}

// ─── MPU6050 Functions ──────────────────────────────────────────────────────

void initMPU6050() {
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  Serial.println("# MPU6050 Calibrated!");
  
  mpu6050.update();
  currentYaw = mpu6050.getAngleZ();
  filteredYaw = currentYaw;
  targetYaw = currentYaw;
  previousYaw = currentYaw;
  lastYawTime = micros();
  headingInitialized = true;
}

void updateIMU() {
  mpu6050.update();
  currentYaw = mpu6050.getAngleZ();
  
  unsigned long now = micros();
  float dt = (now - lastYawTime) / 1000000.0f;
  
  if (dt > 0.001 && dt < 0.1) {
    float yawDelta = currentYaw - previousYaw;
    if (yawDelta > 180) yawDelta -= 360;
    if (yawDelta < -180) yawDelta += 360;
    
    float rawRate = yawDelta / dt;
    filteredAngularVelocity = expMovingAverage(rawRate, filteredAngularVelocity, RATE_FILTER_ALPHA);
    filteredYaw = expMovingAverage(currentYaw, filteredYaw, YAW_FILTER_ALPHA);
  }
  
  previousYaw = currentYaw;
  lastYawTime = now;
}

// ─── Heading PID ────────────────────────────────────────────────────────────

float computeHeadingCorrection(float target, float current, float dt) {
  float error = target - current;
  
  // Normalize to [-180, 180]
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  
  headingIntegral += error * dt;
  headingIntegral = constrain(headingIntegral, -100.0, 100.0);
  
  float derivative = (error - headingPrevError) / dt;
  
  float output = KP_HEADING * error + KI_HEADING * headingIntegral + KD_HEADING * derivative;
  
  headingPrevError = error;
  
  // Return PWM differential (how much to add/subtract from wheels)
  return output;
}

// ─── Motor Functions ────────────────────────────────────────────────────────

void IRAM_ATTR leftISR() {
  unsigned long now = micros();
  if (now - lastLeftUS >= debounceUS) {
    lastLeftUS = now;
    leftPulses += leftFwd ? 1 : -1;
  }
}

void IRAM_ATTR rightISR() {
  unsigned long now = micros();
  if (now - lastRightUS >= debounceUS) {
    lastRightUS = now;
    rightPulses += rightFwd ? 1 : -1;
  }
}

void setLeftMotor(int pwm) {
  leftCurrentPWM = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (leftCurrentPWM >= 0) {
    leftFwd = true;
    digitalWrite(LEFT_DIR, LEFT_FORWARD);
  } else {
    leftFwd = false;
    digitalWrite(LEFT_DIR, !LEFT_FORWARD);
  }
  ledcWrite(0, abs(leftCurrentPWM));
}

void setRightMotor(int pwm) {
  rightCurrentPWM = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (rightCurrentPWM >= 0) {
    rightFwd = true;
    digitalWrite(RIGHT_DIR, RIGHT_FORWARD);
  } else {
    rightFwd = false;
    digitalWrite(RIGHT_DIR, !RIGHT_FORWARD);
  }
  ledcWrite(1, abs(rightCurrentPWM));
}

void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  leftCurrentPWM = 0;
  rightCurrentPWM = 0;
  
  leftIntegral = 0;
  rightIntegral = 0;
  leftPrevError = 0;
  rightPrevError = 0;
  headingIntegral = 0;
  headingPrevError = 0;
  turnRateIntegral = 0;
  turnRatePrevError = 0;
  
  targetLinearVelocity = 0;
  targetAngularVelocity = 0;
  lockHeading = false;
}

// ─── Main Motor Update ──────────────────────────────────────────────────────

void updateMotorSpeeds() {
  unsigned long now = micros();
  static unsigned long lastUpdateUS = 0;
  float dt = (now - lastUpdateUS) / 1000000.0f;
  lastUpdateUS = now;
  
  if (dt > 0.1) dt = 0.02;
  if (dt < 0.001) dt = 0.02;
  
  // Read encoders
  noInterrupts();
  long lp = leftPulses;
  long rp = rightPulses;
  interrupts();
  
  static long lastLeftSnap = 0;
  static long lastRightSnap = 0;
  
  float rawLeftSpeed = (lp - lastLeftSnap) * METERS_PER_PULSE / dt;
  float rawRightSpeed = (rp - lastRightSnap) * METERS_PER_PULSE / dt;
  
  lastLeftSnap = lp;
  lastRightSnap = rp;
  
  // Filter speeds
  static float filtLeftSpeed = 0;
  static float filtRightSpeed = 0;
  filtLeftSpeed = 0.3 * rawLeftSpeed + 0.7 * filtLeftSpeed;
  filtRightSpeed = 0.3 * rawRightSpeed + 0.7 * filtRightSpeed;
  
  currentLeftSpeed = filtLeftSpeed;
  currentRightSpeed = filtRightSpeed;
  
  // Update IMU
  if (millis() - lastIMUUpdate >= IMU_UPDATE_INTERVAL) {
    updateIMU();
    lastIMUUpdate = millis();
  }
  
  // ─── CONTROL STRATEGY ────────────────────────────────────────────────
  
  int leftPWM = 0;
  int rightPWM = 0;
  
  // STRAIGHT LINE MODE
  if (abs(targetLinearVelocity) > 0.02 && abs(targetAngularVelocity) < 0.01) {
    
    // Lock heading when first entering straight mode
    if (!lockHeading) {
      targetYaw = filteredYaw;
      lockHeading = true;
      headingIntegral = 0;
      headingPrevError = 0;
      leftIntegral = 0;
      rightIntegral = 0;
      Serial.println("# LOCKED");
    }
    
    // Get heading correction from IMU
    float headingCorrection = computeHeadingCorrection(targetYaw, filteredYaw, dt);
    
    // Calculate base PWM for desired speed
    int basePWM = speedToPWM(targetLinearVelocity);
    
    // Apply heading correction as differential
    // Positive correction = robot has turned left, need more right wheel power
    int differential = constrain((int)headingCorrection, -MAX_PWM/2, MAX_PWM/2);
    
    leftPWM = basePWM - differential/2;
    rightPWM = basePWM + differential/2;
    
    // ALSO apply wheel speed matching using encoders
    // If left wheel is faster than right, reduce left, increase right
    float speedDiff = currentLeftSpeed - currentRightSpeed;
    float speedAvg = (abs(currentLeftSpeed) + abs(currentRightSpeed)) / 2.0;
    
    // Only apply speed matching if wheels are actually moving
    if (speedAvg > 0.02) {
      // Speed matching correction
      float matchCorrection = speedDiff * 500.0;  // Strong correction for speed mismatch
      leftPWM -= (int)matchCorrection;
      rightPWM += (int)matchCorrection;
    }
    
    // Debug output
    if (millis() - lastDebugPrint > DEBUG_INTERVAL) {
      lastDebugPrint = millis();
      float yawError = targetYaw - filteredYaw;
      while (yawError > 180) yawError -= 360;
      while (yawError < -180) yawError += 360;
      
      Serial.print("# YAW_ERR=");
      Serial.print(yawError, 1);
      Serial.print(" HDG_CORR=");
      Serial.print(headingCorrection, 1);
      Serial.print(" SPD_DIFF=");
      Serial.print(speedDiff, 3);
      Serial.print(" L=");
      Serial.print(currentLeftSpeed, 2);
      Serial.print(" R=");
      Serial.print(currentRightSpeed, 2);
      Serial.print(" LPWM=");
      Serial.print(leftPWM);
      Serial.print(" RPWM=");
      Serial.println(rightPWM);
    }
  }
  // ROTATION MODE
  else if (abs(targetLinearVelocity) < 0.02 && abs(targetAngularVelocity) > 0.01) {
    lockHeading = false;
    
    // Simple open-loop rotation with rate feedback
    int basePWM = MIN_PWM_START + (int)(abs(targetAngularVelocity) * 30.0);
    basePWM = constrain(basePWM, MIN_PWM_START, MAX_PWM/2);
    
    float targetRateDeg = targetAngularVelocity * 180.0 / PI;
    float rateError = targetRateDeg - filteredAngularVelocity;
    int correction = (int)(rateError * 5.0);
    
    if (targetAngularVelocity > 0) {
      // CCW: left backward, right forward
      leftPWM = -(basePWM + correction);
      rightPWM = basePWM - correction;
    } else {
      // CW: left forward, right backward
      leftPWM = basePWM - correction;
      rightPWM = -(basePWM + correction);
    }
  }
  // ARC MODE
  else if (abs(targetLinearVelocity) > 0.02 && abs(targetAngularVelocity) > 0.01) {
    lockHeading = false;
    
    float vL = targetLinearVelocity - (targetAngularVelocity * WHEELBASE_M) / 2.0f;
    float vR = targetLinearVelocity + (targetAngularVelocity * WHEELBASE_M) / 2.0f;
    
    leftPWM = speedToPWM(vL);
    rightPWM = speedToPWM(vR);
    
    // Rate feedback
    float targetRateDeg = targetAngularVelocity * 180.0 / PI;
    float rateError = targetRateDeg - filteredAngularVelocity;
    int correction = (int)(rateError * 5.0);
    leftPWM -= correction;
    rightPWM += correction;
  }
  // STOPPED
  else {
    lockHeading = false;
    leftPWM = 0;
    rightPWM = 0;
  }
  
  // Final constrain
  leftPWM = constrain(leftPWM, -MAX_PWM, MAX_PWM);
  rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);
  
  // Slew rate limiting
  static int prevLeftPWM = 0;
  static int prevRightPWM = 0;
  int maxChange = 30;
  leftPWM = constrain(leftPWM, prevLeftPWM - maxChange, prevLeftPWM + maxChange);
  rightPWM = constrain(rightPWM, prevRightPWM - maxChange, prevRightPWM + maxChange);
  prevLeftPWM = leftPWM;
  prevRightPWM = rightPWM;
  
  // Apply
  setLeftMotor(leftPWM);
  setRightMotor(rightPWM);
}

// ─── Speed to PWM ──────────────────────────────────────────────────────────

int speedToPWM(float speed) {
  if (abs(speed) < 0.01) return 0;
  
  int pwm = (int)(abs(speed) * SPEED_TO_PWM_GAIN);
  
  if (pwm < MIN_PWM_START) pwm = MIN_PWM_START;
  
  if (speed < 0) pwm = -pwm;
  
  return constrain(pwm, -MAX_PWM, MAX_PWM);
}

// ─── Setup ──────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(100);
  
  ledcSetup(0, 20000, 10);
  ledcSetup(1, 20000, 10);
  ledcAttachPin(LEFT_PWM, 0);
  ledcAttachPin(RIGHT_PWM, 1);
  
  pinMode(LEFT_DIR,  OUTPUT);
  pinMode(LEFT_SC,   INPUT_PULLUP);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_SC,  INPUT_PULLUP);

  attachInterrupt(LEFT_SC, leftISR, RISING);
  attachInterrupt(RIGHT_SC, rightISR, RISING);

  initMPU6050();

  digitalWrite(LEFT_DIR, LEFT_FORWARD);
  digitalWrite(RIGHT_DIR, RIGHT_FORWARD);
  stopMotors();

  Serial.println("READY");
}

// ─── Main Loop ──────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();
  
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    lastControlTime = now;
    updateMotorSpeeds();
    
    // Telemetry
    float actualOmega = (currentRightSpeed - currentLeftSpeed) / WHEELBASE_M;
    float actualLinear = (currentLeftSpeed + currentRightSpeed) / 2.0f;
    
    Serial.print("CNT,");
    Serial.print(now);
    Serial.print(',');
    Serial.print(currentLeftSpeed, 3);
    Serial.print(',');
    Serial.print(currentRightSpeed, 3);
    Serial.print(',');
    Serial.print(actualLinear, 3);
    Serial.print(',');
    Serial.print(actualOmega, 3);
    Serial.print(',');
    Serial.print(filteredAngularVelocity, 2);
    Serial.print(',');
    Serial.print(filteredYaw, 2);
    Serial.print(',');
    Serial.print(leftCurrentPWM);
    Serial.print(',');
    Serial.println(rightCurrentPWM);
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "s" || cmd == "S") {
      stopMotors();
      Serial.println("# Stopped");
    }
    else if (cmd == "c" || cmd == "C") {
      stopMotors();
      delay(500);
      mpu6050.calcGyroOffsets(true);
      mpu6050.update();
      currentYaw = mpu6050.getAngleZ();
      filteredYaw = currentYaw;
      targetYaw = currentYaw;
      previousYaw = currentYaw;
      headingIntegral = 0;
      headingPrevError = 0;
      lockHeading = false;
      Serial.println("# Calibrated");
    }
    else if (cmd == "z" || cmd == "Z") {
      noInterrupts();
      leftPulses = 0;
      rightPulses = 0;
      interrupts();
      odomX = 0.0;
      odomY = 0.0;
      Serial.println("# Zeroed");
    }
    else if (cmd.startsWith("V") || cmd.startsWith("v")) {
      cmd = cmd.substring(1);
      int comma = cmd.indexOf(',');
      if (comma != -1) {
        float v = cmd.substring(0, comma).toFloat();
        float w = cmd.substring(comma + 1).toFloat();
        
        targetLinearVelocity = constrain(v, -1.2, 1.2);
        targetAngularVelocity = constrain(w, -3.0, 3.0);
        
        lockHeading = false;
        headingIntegral = 0;
        headingPrevError = 0;
        turnRateIntegral = 0;
        turnRatePrevError = 0;
        leftIntegral = 0;
        rightIntegral = 0;
        
        Serial.print("# CMD v=");
        Serial.print(targetLinearVelocity, 2);
        Serial.print(" w=");
        Serial.println(targetAngularVelocity, 2);
      }
    }
  }
}