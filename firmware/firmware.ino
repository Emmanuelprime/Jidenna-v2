#include <math.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// Define custom I2C pins for MPU6050
#define I2C_SDA 32
#define I2C_SCL 33

// Motor control pins
#define LEFT_PWM     22
#define LEFT_DIR     23
#define LEFT_SC      34
#define RIGHT_PWM    19
#define RIGHT_DIR    26
#define RIGHT_SC     35

#define LEFT_PWM_CH   2
#define RIGHT_PWM_CH  1
#define PWM_FREQ      1000
#define PWM_RES       8
#define MAX_PWM       80

#define DEBOUNCE_TIME_US       1000
#define ENCODER_PRINT_INTERVAL 100 

#define LEFT_FORWARD   HIGH
#define RIGHT_FORWARD  LOW

#define WHEEL_DIAMETER_M    0.165f
#define WHEEL_SEPARATION_M  0.521f
#define PULSES_PER_REV      45

#define FILTER_ALPHA 0.3f

// Forward calibration
#define LEFT_SLOPE_FWD     0.0411f
#define LEFT_INTERCEPT_FWD -0.2392f
#define RIGHT_SLOPE_FWD    0.0267f
#define RIGHT_INTERCEPT_FWD -0.0816f

// Reverse calibration
#define LEFT_SLOPE_REV     0.0267f
#define LEFT_INTERCEPT_REV -0.0816f
#define RIGHT_SLOPE_REV    0.0411f
#define RIGHT_INTERCEPT_REV -0.2392f

// PID gains (increased for better response)
#define KP_L_LOW  8.0f
#define KI_L_LOW  0.08f
#define KD_L_LOW  0.02f
#define KP_R_LOW  8.5f
#define KI_R_LOW  0.08f
#define KD_R_LOW  0.02f

#define KP_L_MED  3.0f
#define KI_L_MED  0.15f
#define KD_L_MED  0.05f
#define KP_R_MED  3.5f
#define KI_R_MED  0.15f
#define KD_R_MED  0.05f

#define KP_L_HIGH 2.0f
#define KI_L_HIGH 0.20f
#define KD_L_HIGH 0.08f
#define KP_R_HIGH 2.5f
#define KI_R_HIGH 0.20f
#define KD_R_HIGH 0.08f

#define MIN_START_PWM 10
#define MAX_ACCELERATION 0.5f  // Increased for faster response
#define LOW_SPEED_PWM_PER_MPS 30.0f
#define LOW_SPEED_THRESHOLD 0.3f

// MPU6050 compensation parameters (improved)
#define YAW_COMPENSATION_KP 1.2f   // Increased for stronger correction
#define YAW_COMPENSATION_KI 0.15f  // Increased integral
#define YAW_COMPENSATION_KD 0.05f  // Increased derivative
#define MAX_YAW_CORRECTION 0.3f    // Increased max correction
#define YAW_DEADZONE 0.02f         // Ignore small yaw errors

MPU6050 mpu6050(Wire);

// Global variables
float x = 0.0;
float y = 0.0;
float theta = 0.0;

unsigned long samplerate = 50;

float vl = 0.0;
float vr = 0.0;
float filteredVl = 0.0;
float filteredVr = 0.0;

float targetVl = 0.0;
float targetVr = 0.0;
float rampedVl = 0.0;
float rampedVr = 0.0;

float prevErrorVl = 0.0;
float integralVl = 0.0;
float outputVl = 0.0;
float prevErrorVr = 0.0;
float integralVr = 0.0;
float outputVr = 0.0;

float kp_l, ki_l, kd_l;
float kp_r, ki_r, kd_r;

// Yaw control variables
float targetYaw = 0.0f;
float prevYawError = 0.0f;
float integralYaw = 0.0f;
bool yawLocked = false;
float initialYaw = 0.0f;
float yawFiltered = 0.0f;  // Filtered yaw for stability

// Encoder variables
volatile long lastLeftPulses = 0;
volatile long lastRightPulses = 0;
volatile long leftPulses = 0;
volatile long rightPulses = 0;

volatile unsigned long lastLeftInterrupt = 0;
volatile unsigned long lastRightInterrupt = 0;

volatile bool leftEncoderDirection = true;
volatile bool rightEncoderDirection = true;

unsigned long lastEncoderPrint = 0;
unsigned long lastSpeedMeasure = 0;
unsigned long lastPIDTime = 0;
unsigned long lastRampTime = 0;
unsigned long lastMPUUpdate = 0;
unsigned long yawLockTime = 0;

// Function prototypes
void normalizeAngle();
void updateOdometry();
void IRAM_ATTR leftISR();
void IRAM_ATTR rightISR();
void MeasureWheelSpeeds(float dt);
void updateGains(float speed);
float pidControl(float target, float current, float dt, float kp, float ki, float kd, float &prevError, float &integral);
void resetPID();
void setMotors(int leftPwm, int rightPwm);
void setVelocity(float targetLeft, float targetRight);
void updateVelocityRamping();
int calculateFeedForward(float targetSpeed, bool isLeft);
void updatePID();
void applyYawCompensation();

void normalizeAngle() {
  while (theta > PI) theta -= 2 * PI;
  while (theta < -PI) theta += 2 * PI;
}

void updateOdometry() {
  noInterrupts();
  long del_left = leftPulses - lastLeftPulses;
  long del_right = rightPulses - lastRightPulses;
  lastLeftPulses = leftPulses;
  lastRightPulses = rightPulses;
  interrupts();

  float dl = (del_left * PI * WHEEL_DIAMETER_M) / PULSES_PER_REV;
  float dr = (del_right * PI * WHEEL_DIAMETER_M) / PULSES_PER_REV;

  float dc = (dl + dr) / 2.0f;
  float del_theta = (dr - dl) / WHEEL_SEPARATION_M;

  x += dc * cos(theta);
  y += dc * sin(theta);
  theta += del_theta;
  
  normalizeAngle();
}

void IRAM_ATTR leftISR() { 
  unsigned long now = micros();
  if (now - lastLeftInterrupt >= DEBOUNCE_TIME_US) {
    lastLeftInterrupt = now;
    leftPulses += leftEncoderDirection ? 1 : -1;
  }
}

void IRAM_ATTR rightISR() { 
  unsigned long now = micros();
  if (now - lastRightInterrupt >= DEBOUNCE_TIME_US) {
    lastRightInterrupt = now;
    rightPulses += rightEncoderDirection ? 1 : -1;
  }
}

void MeasureWheelSpeeds(float dt) {
  static volatile long prevLeftPulses = 0;
  static volatile long prevRightPulses = 0;
  static unsigned long prevTime = 0;

  noInterrupts();
  long currentLeftPulses = leftPulses;
  long currentRightPulses = rightPulses;
  interrupts();

  long deltaLeft = currentLeftPulses - prevLeftPulses;
  long deltaRight = currentRightPulses - prevRightPulses;
  
  float distLeft = (deltaLeft * PI * WHEEL_DIAMETER_M) / PULSES_PER_REV;
  float distRight = (deltaRight * PI * WHEEL_DIAMETER_M) / PULSES_PER_REV;

  float rawVl = distLeft / dt;
  float rawVr = distRight / dt;
  
  filteredVl = FILTER_ALPHA * rawVl + (1.0f - FILTER_ALPHA) * filteredVl;
  filteredVr = FILTER_ALPHA * rawVr + (1.0f - FILTER_ALPHA) * filteredVr;
  
  vl = rawVl;
  vr = rawVr;
  
  prevLeftPulses = currentLeftPulses;
  prevRightPulses = currentRightPulses;
  prevTime = millis();
}

void updateGains(float speed) {
  float absSpeed = fabs(speed);
  
  if (absSpeed < 0.15f) {
    kp_l = KP_L_LOW;
    ki_l = KI_L_LOW;
    kd_l = KD_L_LOW;
    kp_r = KP_R_LOW;
    ki_r = KI_R_LOW;
    kd_r = KD_R_LOW;
  } else if (absSpeed < 0.40f) {
    kp_l = KP_L_MED;
    ki_l = KI_L_MED;
    kd_l = KD_L_MED;
    kp_r = KP_R_MED;
    ki_r = KI_R_MED;
    kd_r = KD_R_MED;
  } else {
    kp_l = KP_L_HIGH;
    ki_l = KI_L_HIGH;
    kd_l = KD_L_HIGH;
    kp_r = KP_R_HIGH;
    ki_r = KI_R_HIGH;
    kd_r = KD_R_HIGH;
  }
}

float pidControl(float target, float current, float dt, float kp, float ki, float kd, float &prevError, float &integral) {
  float error = target - current;
  
  float P = kp * error;
  
  integral += error * dt;
  integral = constrain(integral, -MAX_PWM/4, MAX_PWM/4);
  float I = ki * integral;
  
  float D = kd * (error - prevError) / dt;
  
  float output = P + I + D;
  prevError = error;
  
  return output;
}

void resetPID() {
  prevErrorVl = 0;
  integralVl = 0;
  prevErrorVr = 0;
  integralVr = 0;
  outputVl = 0;
  outputVr = 0;
}

void setMotors(int leftPwm, int rightPwm) {
  leftPwm = constrain(leftPwm, -MAX_PWM, MAX_PWM);
  rightPwm = constrain(rightPwm, -MAX_PWM, MAX_PWM);

  if (leftPwm >= 0) {
    digitalWrite(LEFT_DIR, LEFT_FORWARD);
    leftEncoderDirection = true;
  } else {
    digitalWrite(LEFT_DIR, !LEFT_FORWARD);
    leftEncoderDirection = false;
  }
  ledcWrite(LEFT_PWM_CH, abs(leftPwm));

  if (rightPwm >= 0) {
    digitalWrite(RIGHT_DIR, RIGHT_FORWARD);
    rightEncoderDirection = true;
  } else {
    digitalWrite(RIGHT_DIR, !RIGHT_FORWARD);
    rightEncoderDirection = false;
  }
  ledcWrite(RIGHT_PWM_CH, abs(rightPwm));
}

void setVelocity(float targetLeft, float targetRight) {
  targetVl = targetLeft;
  targetVr = targetRight;
  resetPID();
  
  // Lock yaw when moving straight
  if (fabs(targetLeft - targetRight) < 0.01f && fabs(targetLeft) > 0.01f) {
    // Wait for stable yaw reading
    delay(50);
    mpu6050.update();
    targetYaw = mpu6050.getAngleZ();
    initialYaw = targetYaw;
    yawFiltered = targetYaw;
    prevYawError = 0;
    integralYaw = 0;
    yawLocked = true;
    yawLockTime = millis();
    Serial.print("Yaw locked at: ");
    Serial.println(targetYaw);
  } else {
    yawLocked = false;
  }
}

void updateVelocityRamping() {
  unsigned long now = millis();
  float dt = (now - lastRampTime) / 1000.0f;
  
  if (dt <= 0) return;
  
  if (fabs(rampedVl - targetVl) > 0.001f) {
    float step = MAX_ACCELERATION * dt;
    if (rampedVl < targetVl) {
      rampedVl = fmin(rampedVl + step, targetVl);
    } else {
      rampedVl = fmax(rampedVl - step, targetVl);
    }
  }
  
  if (fabs(rampedVr - targetVr) > 0.001f) {
    float step = MAX_ACCELERATION * dt;
    if (rampedVr < targetVr) {
      rampedVr = fmin(rampedVr + step, targetVr);
    } else {
      rampedVr = fmax(rampedVr - step, targetVr);
    }
  }
  
  lastRampTime = now;
}

int calculateFeedForward(float targetSpeed, bool isLeft) {
  if (fabs(targetSpeed) <= 0.01f) return 0;
  
  float absSpeed = fabs(targetSpeed);
  int sign = (targetSpeed > 0) ? 1 : -1;
  
  if (absSpeed < LOW_SPEED_THRESHOLD) {
    int pwm = (int)(absSpeed * LOW_SPEED_PWM_PER_MPS);
    if (pwm > 0 && pwm < MIN_START_PWM) {
      pwm = MIN_START_PWM;
    }
    return constrain(pwm * sign, -MAX_PWM, MAX_PWM);
  }
  
  float slope, intercept;
  
  if (isLeft) {
    if (sign > 0) {
      slope = LEFT_SLOPE_FWD;
      intercept = LEFT_INTERCEPT_FWD;
    } else {
      slope = LEFT_SLOPE_REV;
      intercept = LEFT_INTERCEPT_REV;
    }
  } else {
    if (sign > 0) {
      slope = RIGHT_SLOPE_FWD;
      intercept = RIGHT_INTERCEPT_FWD;
    } else {
      slope = RIGHT_SLOPE_REV;
      intercept = RIGHT_INTERCEPT_REV;
    }
  }
  
  int pwm = (int)((absSpeed - intercept) / slope);
  return constrain(pwm * sign, -MAX_PWM, MAX_PWM);
}

void applyYawCompensation() {
  if (!yawLocked) return;
  
  // Update MPU data
  mpu6050.update();
  float currentYaw = mpu6050.getAngleZ();
  
  // Apply low-pass filter to yaw
  yawFiltered = 0.7f * yawFiltered + 0.3f * currentYaw;
  
  // Calculate yaw error
  float yawError = targetYaw - yawFiltered;
  
  // Normalize yaw error to [-PI, PI]
  while (yawError > PI) yawError -= 2 * PI;
  while (yawError < -PI) yawError += 2 * PI;
  
  // Apply deadzone to prevent oscillation
  if (fabs(yawError) < YAW_DEADZONE) {
    yawError = 0;
  }
  
  // PID control for yaw
  static unsigned long lastYawTime = 0;
  float dt = (millis() - lastYawTime) / 1000.0f;
  if (dt <= 0) dt = 0.01f;
  if (dt > 0.1f) dt = 0.1f;  // Limit dt
  
  float P = YAW_COMPENSATION_KP * yawError;
  
  integralYaw += yawError * dt;
  integralYaw = constrain(integralYaw, -0.3f, 0.3f);
  float I = YAW_COMPENSATION_KI * integralYaw;
  
  float D = YAW_COMPENSATION_KD * (yawError - prevYawError) / dt;
  
  float yawCorrection = P + I + D;
  yawCorrection = constrain(yawCorrection, -MAX_YAW_CORRECTION, MAX_YAW_CORRECTION);
  
  prevYawError = yawError;
  lastYawTime = millis();
  
  // Apply correction to wheel velocities
  // If robot yaws right (positive yaw), speed up left wheel or slow down right
  float leftCorrection = -yawCorrection / 2.0f;
  float rightCorrection = yawCorrection / 2.0f;
  
  // Apply corrections to target velocities
  float adjustedVl = targetVl + leftCorrection;
  float adjustedVr = targetVr + rightCorrection;
  
  // Ensure we maintain direction
  if (targetVl > 0) {
    adjustedVl = max(adjustedVl, 0.05f);
  } else if (targetVl < 0) {
    adjustedVl = min(adjustedVl, -0.05f);
  }
  
  if (targetVr > 0) {
    adjustedVr = max(adjustedVr, 0.05f);
  } else if (targetVr < 0) {
    adjustedVr = min(adjustedVr, -0.05f);
  }
  
  // Update ramped velocities to use adjusted targets
  rampedVl = adjustedVl;
  rampedVr = adjustedVr;
}

void updatePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  
  if (dt <= 0) return;
  if (dt > 0.1f) dt = 0.1f;  // Limit dt
  
  // Apply yaw compensation
  applyYawCompensation();
  
  // Update ramp if not yaw locked (yaw locked sets ramped velocities directly)
  if (!yawLocked) {
    updateVelocityRamping();
  }
  
  updateGains(rampedVl);
  updateGains(rampedVr);
  
  float currentVl = filteredVl;
  float currentVr = filteredVr;
  
  float pidOutputL = pidControl(rampedVl, currentVl, dt, kp_l, ki_l, kd_l, prevErrorVl, integralVl);
  float pidOutputR = pidControl(rampedVr, currentVr, dt, kp_r, ki_r, kd_r, prevErrorVr, integralVr);
  
  int ffL = calculateFeedForward(rampedVl, true);
  int ffR = calculateFeedForward(rampedVr, false);
  
  int leftPwm = constrain((int)(pidOutputL + ffL), -MAX_PWM, MAX_PWM);
  int rightPwm = constrain((int)(pidOutputR + ffR), -MAX_PWM, MAX_PWM);
  
  setMotors(leftPwm, rightPwm);
  
  lastPIDTime = now;
}

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050 with better calibration
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu6050.begin();
  
  // Longer calibration with explicit messages
  Serial.println("Calibrating MPU6050 gyro... Keep robot still!");
  mpu6050.calcGyroOffsets(true);
  delay(1000);
  
  // Get initial yaw
  mpu6050.update();
  yawFiltered = mpu6050.getAngleZ();
  targetYaw = yawFiltered;
  Serial.println("MPU6050 initialized and calibrated");
  Serial.print("Initial Yaw: ");
  Serial.println(yawFiltered);

  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_SC, INPUT_PULLUP);
  pinMode(RIGHT_SC, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_SC), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SC), rightISR, RISING);

  ledcSetup(LEFT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_PWM, LEFT_PWM_CH);
  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_PWM, RIGHT_PWM_CH);

  setMotors(0, 0);
  resetPID();
  rampedVl = 0;
  rampedVr = 0;
  lastRampTime = millis();
  
  Serial.println("Robot ready with improved MPU6050 yaw compensation");
  Serial.println("Commands:");
  Serial.println("  V<velL>,<velR> - Set target velocities in m/s");
  Serial.println("  S - Stop motors");
  Serial.println("  M<pwmL>,<pwmR> - Manual PWM control");
  Serial.println("  Y - Show current yaw angle");
  Serial.println("  C - Re-calibrate MPU6050");
}

void loop() {
  unsigned long now = millis();

  // Update MPU data at 100Hz for better response
  if (now - lastMPUUpdate >= 10) {  // 100Hz
    mpu6050.update();
    lastMPUUpdate = now;
  }

  if (now - lastSpeedMeasure >= samplerate) {
    float dt = (now - lastSpeedMeasure) / 1000.0f;
    MeasureWheelSpeeds(dt);
    lastSpeedMeasure = now;
  }
  
  if (now - lastPIDTime >= samplerate) {
    updatePID();
  }

  if (now - lastEncoderPrint >= ENCODER_PRINT_INTERVAL) {
    updateOdometry();
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(theta);
    Serial.print(",");
    Serial.print(vl);
    Serial.print(",");
    Serial.print(vr);
    Serial.print(",");
    Serial.print(rampedVl);
    Serial.print(",");
    Serial.print(rampedVr);
    Serial.print(",");
    Serial.print(yawFiltered);
    Serial.print(",");
    Serial.print(mpu6050.getAngleZ());
    Serial.print(",");
    Serial.println(yawLocked ? 1 : 0);
    lastEncoderPrint = now;
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("M") || cmd.startsWith("m")) {
      int commaIndex = cmd.indexOf(',');
      if (commaIndex > 0) {
        int leftPwm = cmd.substring(1, commaIndex).toInt();
        int rightPwm = cmd.substring(commaIndex + 1).toInt();
        setMotors(leftPwm, rightPwm);
        resetPID();
        targetVl = 0;
        targetVr = 0;
        rampedVl = 0;
        rampedVr = 0;
        yawLocked = false;
      }
    }
    else if (cmd.startsWith("V") || cmd.startsWith("v")) {
      int commaIndex = cmd.indexOf(',');
      if (commaIndex > 0) {
        float leftVel = cmd.substring(1, commaIndex).toFloat();
        float rightVel = cmd.substring(commaIndex + 1).toFloat();
        setVelocity(leftVel, rightVel);
        Serial.print("Target velocity: L=");
        Serial.print(leftVel);
        Serial.print(" m/s, R=");
        Serial.print(rightVel);
        Serial.println(" m/s");
      }
    }
    else if (cmd.equalsIgnoreCase("S")) {
      setMotors(0, 0);
      resetPID();
      targetVl = 0;
      targetVr = 0;
      rampedVl = 0;
      rampedVr = 0;
      yawLocked = false;
      Serial.println("Stopped");
    }
    else if (cmd.equalsIgnoreCase("Y")) {
      Serial.print("Current Yaw: ");
      Serial.print(mpu6050.getAngleZ());
      Serial.print("  Filtered: ");
      Serial.print(yawFiltered);
      Serial.print("  Target: ");
      Serial.print(targetYaw);
      Serial.print("  Locked: ");
      Serial.println(yawLocked ? "Yes" : "No");
    }
    else if (cmd.equalsIgnoreCase("C")) {
      Serial.println("Re-calibrating MPU6050...");
      mpu6050.calcGyroOffsets(true);
      delay(1000);
      mpu6050.update();
      yawFiltered = mpu6050.getAngleZ();
      targetYaw = yawFiltered;
      Serial.print("New initial yaw: ");
      Serial.println(yawFiltered);
    }
  }
}