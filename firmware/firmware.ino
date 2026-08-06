#include <math.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

#define LEFT_PWM     22
#define LEFT_DIR     23
#define LEFT_SC      34
#define RIGHT_PWM    19
#define RIGHT_DIR    26
#define RIGHT_SC     35

#define I2C_SDA 32
#define I2C_SCL 33

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

// ─── HEADING CONTROL FROM WORKING CODE ────────────────────────────────────
#define HEADING_DEADZONE_DEG  0.3f
#define YAW_FILTER_ALPHA  0.92f

// ─── PID GAINS FROM WORKING CODE ──────────────────────────────────────────
#define KP_HEADING 1.2f
#define KI_HEADING 0.01f
#define KD_HEADING 0.08f

#define KP_TURN_RATE 1.8f
#define KI_TURN_RATE 0.03f
#define KD_TURN_RATE 0.08f

#define KP_LEFT  1.8f   
#define KI_LEFT  0.02f  
#define KD_LEFT  0.15f  

#define KP_RIGHT 2.5f   
#define KI_RIGHT 0.03f  
#define KD_RIGHT 0.20f  

#define COMPLEMENTARY_ALPHA 0.98f

MPU6050 mpu6050(Wire);

// Forward calibration - from your working code
#define LEFT_FWD_SLOPE     0.0209f
#define LEFT_FWD_INTERCEPT -0.0167f
#define RIGHT_FWD_SLOPE    0.0209f
#define RIGHT_FWD_INTERCEPT -0.0167f

#define LEFT_DEADZONE 10
#define RIGHT_DEADZONE 10

#define CONTROL_INTERVAL 50
#define SAMPLE_TIME 0.05f

// Gain scheduling
#define KP_L_LOW  5.0f
#define KI_L_LOW  0.05f
#define KD_L_LOW  0.01f
#define KP_R_LOW  5.5f
#define KI_R_LOW  0.05f
#define KD_R_LOW  0.01f

#define KP_L_MED  1.5f
#define KI_L_MED  0.10f
#define KD_L_MED  0.03f
#define KP_R_MED  2.0f
#define KI_R_MED  0.12f
#define KD_R_MED  0.03f

#define KP_L_HIGH 1.0f
#define KI_L_HIGH 0.15f
#define KD_L_HIGH 0.05f
#define KP_R_HIGH 1.5f
#define KI_R_HIGH 0.18f
#define KD_R_HIGH 0.05f

#define MIN_START_PWM 10
#define MAX_ACCELERATION 0.3f
#define LOW_SPEED_PWM_PER_MPS 30.0f
#define LOW_SPEED_THRESHOLD 0.3f

float x = 0.0;
float y = 0.0;
float theta = 0.0;

float V = 0.0;
float W = 0.0;
float max_speed = 0.8f;
float yaw = 0.0;
float yawFiltered = 0.0;
float gyro_z = 0.0;

// ─── HEADING CONTROL VARIABLES FROM WORKING CODE ──────────────────────────
float headingIntegral = 0;
float headingPrevError = 0;
float targetHeading = 0;
bool headingInitialized = false;
float currentYaw = 0;
float filteredYaw = 0;

// ─── TURN RATE CONTROL ──────────────────────────────────────────────────────
float turnRateIntegral = 0;
float turnRatePrevError = 0;
float actualAngularVelocity = 0;
float filteredAngularVelocity = 0;
float previousYaw = 0;
unsigned long lastYawTime = 0;
bool turnRateControlActive = false;

float rampedV = 0.0;
float rampedW = 0.0;

bool headingLockEnabled = true;
bool pidEnabled = true;

// Debug
float lastHeadingError = 0.0;
float lastHeadingCorrection = 0.0;

unsigned long samplerate = 50;

float vl = 0.0;
float vr = 0.0;
float filteredVl = 0.0;
float filteredVr = 0.0;

float targetVl = 0.0;
float targetVr = 0.0;

float prevErrorVl = 0.0;
float integralVl = 0.0;
float outputVl = 0.0;
float prevErrorVr = 0.0;
float integralVr = 0.0;
float outputVr = 0.0;

float kp_l, ki_l, kd_l;
float kp_r, ki_r, kd_r;

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
unsigned long lastMPURead = 0;
unsigned long lastDebugPrint = 0;

// ─── SOFT DEADZONE ──────────────────────────────────────────────────────────
float softDeadzone(float error, float threshold) {
  if (abs(error) < threshold) {
    return error * (abs(error) / threshold);
  }
  return error;
}

// ─── EXPONENTIAL MOVING AVERAGE ────────────────────────────────────────────
float expMovingAverage(float newValue, float prevValue, float alpha) {
  return alpha * newValue + (1.0f - alpha) * prevValue;
}

// ─── HEADING PID FROM WORKING CODE ─────────────────────────────────────────
float computeHeadingPID(float target, float current, float dt) {
  float error = target - current;
  
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  
  error = softDeadzone(error, HEADING_DEADZONE_DEG);
  
  headingIntegral += error * dt;
  headingIntegral = constrain(headingIntegral, -5.0, 5.0);
  
  float derivative = (error - headingPrevError) / dt;
  float output = KP_HEADING * error + KI_HEADING * headingIntegral + KD_HEADING * derivative;
  
  headingPrevError = error;
  
  lastHeadingError = error;
  lastHeadingCorrection = output;
  
  return constrain(output, -0.3f, 0.3f);
}

// ─── TURN RATE PID FROM WORKING CODE ──────────────────────────────────────
float computeTurnRatePID(float target, float current, float dt) {
  float error = target - current;
  
  error = softDeadzone(error, 0.015f);
  
  turnRateIntegral += error * dt;
  turnRateIntegral = constrain(turnRateIntegral, -0.5, 0.5);
  
  float derivative = (error - turnRatePrevError) / dt;
  float output = KP_TURN_RATE * error + KI_TURN_RATE * turnRateIntegral + KD_TURN_RATE * derivative;
  
  turnRatePrevError = error;
  
  return constrain(output, -1.0, 1.0);
}

void getMpuData() {
  mpu6050.update();
  
  currentYaw = mpu6050.getAngleZ();
  gyro_z = mpu6050.getGyroZ() * PI / 180.0;
  
  // Use the same filtering as working code
  filteredYaw = expMovingAverage(currentYaw, filteredYaw, YAW_FILTER_ALPHA);
  
  // Calculate angular velocity from yaw delta
  static float prevYaw = 0;
  static unsigned long prevYawTime = 0;
  unsigned long now = micros();
  float dt = (now - prevYawTime) / 1000000.0f;
  if (dt > 0.001 && dt < 0.1) {
    float yawDelta = currentYaw - prevYaw;
    while (yawDelta > 180) yawDelta -= 360;
    while (yawDelta < -180) yawDelta += 360;
    filteredAngularVelocity = expMovingAverage(yawDelta / dt * (PI / 180.0), filteredAngularVelocity, 0.5f);
  }
  prevYaw = currentYaw;
  prevYawTime = now;
  
  yaw = currentYaw * PI / 180.0;
  yawFiltered = filteredYaw * PI / 180.0;
}

void diffDriveController(float v, float w, float dt) {
  float L = WHEEL_SEPARATION_M;
  
  float headingCorrection = 0;
  float turnRateCorrection = 0;
  
  // ─── HEADING CORRECTION (same as working code) ──────────────────────────
  if (headingLockEnabled && headingInitialized && abs(v) > 0.02 && abs(w) < 0.01) {
    headingCorrection = computeHeadingPID(targetHeading, filteredYaw, dt);
  }
  
  // ─── TURN RATE CORRECTION (same as working code) ────────────────────────
  if (abs(w) > 0.01) {
    turnRateControlActive = true;
    turnRateCorrection = computeTurnRatePID(w, filteredAngularVelocity, dt);
  } else {
    turnRateControlActive = false;
  }
  
  // Combine corrections (same weighting as working code)
  float effectiveOmega = w + headingCorrection * 0.8f + turnRateCorrection * 0.8f;
  
  // Debug output
  if (millis() - lastDebugPrint > 500 && headingLockEnabled) {
    Serial.print("Heading error: ");
    Serial.print(lastHeadingError);
    Serial.print(" deg, Correction: ");
    Serial.print(headingCorrection);
    Serial.print(" rad/s, Effective W: ");
    Serial.println(effectiveOmega);
    lastDebugPrint = millis();
  }
  
  float vl_target = v - (effectiveOmega * L) / 2.0f;
  float vr_target = v + (effectiveOmega * L) / 2.0f;
  
  vl_target = constrain(vl_target, -max_speed, max_speed);
  vr_target = constrain(vr_target, -max_speed, max_speed);
  
  targetVl = vl_target;
  targetVr = vr_target;
}

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
  
  // Only integrate when error is small (anti-windup from working code)
  if (abs(error) < 0.5) {
    integral += error * dt;
  }
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
  headingIntegral = 0;
  headingPrevError = 0;
  turnRateIntegral = 0;
  turnRatePrevError = 0;
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
}

void updateVelocityRamping() {
  unsigned long now = millis();
  float dt = (now - lastRampTime) / 1000.0f;
  
  if (dt <= 0) return;
  
  if (fabs(rampedV - V) > 0.001f) {
    float step = MAX_ACCELERATION * dt;
    if (rampedV < V) rampedV = fmin(rampedV + step, V);
    else rampedV = fmax(rampedV - step, V);
  }
  
  if (fabs(rampedW - W) > 0.001f) {
    float step = 2.0f * dt;
    if (rampedW < W) rampedW = fmin(rampedW + step, W);
    else rampedW = fmax(rampedW - step, W);
  }
  
  lastRampTime = now;
}

int calculateFeedForward(float targetSpeed, bool isLeft) {
  if (fabs(targetSpeed) <= 0.01f) return 0;
  
  float absSpeed = fabs(targetSpeed);
  int sign = (targetSpeed > 0) ? 1 : -1;
  
  // Use the working code's calibration
  float pwm;
  if (isLeft) {
    pwm = (absSpeed - LEFT_FWD_INTERCEPT) / LEFT_FWD_SLOPE;
  } else {
    pwm = (absSpeed - RIGHT_FWD_INTERCEPT) / RIGHT_FWD_SLOPE;
  }
  
  // Apply deadzone
  if (absSpeed > 0.01) {
    pwm = max(pwm, (float)(isLeft ? LEFT_DEADZONE : RIGHT_DEADZONE));
  } else {
    pwm = 0;
  }
  
  return constrain(pwm * sign, -MAX_PWM, MAX_PWM);
}

void updatePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  
  if (dt <= 0) dt = 0.05f;
  
  updateGains(targetVl);
  updateGains(targetVr);
  
  float currentVl = filteredVl;
  float currentVr = filteredVr;
  
  float pidOutputL = pidControl(targetVl, currentVl, dt, kp_l, ki_l, kd_l, prevErrorVl, integralVl);
  float pidOutputR = pidControl(targetVr, currentVr, dt, kp_r, ki_r, kd_r, prevErrorVr, integralVr);
  
  int ffL = calculateFeedForward(targetVl, true);
  int ffR = calculateFeedForward(targetVr, false);
  
  int leftPwm = constrain((int)(pidOutputL + ffL), -MAX_PWM, MAX_PWM);
  int rightPwm = constrain((int)(pidOutputR + ffR), -MAX_PWM, MAX_PWM);
  
  setMotors(leftPwm, rightPwm);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

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
  rampedV = 0;
  rampedW = 0;
  lastRampTime = millis();
  
  // Initialize heading
  mpu6050.update();
  currentYaw = mpu6050.getAngleZ();
  filteredYaw = currentYaw;
  targetHeading = currentYaw;
  headingInitialized = true;
  
  Serial.println("Robot ready - Using proven heading control from working code");
  Serial.println("Commands:");
  Serial.println("  D<V>,<W> - Set linear (m/s) and angular (rad/s) velocities");
  Serial.println("  V<left>,<right> - Set individual wheel velocities");
  Serial.println("  M<pwmL>,<pwmR> - Manual PWM control");
  Serial.println("  H - Toggle heading lock");
  Serial.println("  S - Stop");
  Serial.println("  P - Print pose");
  Serial.println("  Y - Print MPU data");
}

void loop() {
  unsigned long now = millis();

  if (now - lastMPURead >= 10) {
    getMpuData();
    lastMPURead = now;
  }

  if (now - lastSpeedMeasure >= samplerate) {
    float dt = (now - lastSpeedMeasure) / 1000.0f;
    MeasureWheelSpeeds(dt);
    lastSpeedMeasure = now;
  }
  
  if (now - lastPIDTime >= samplerate) {
    float dt = (now - lastPIDTime) / 1000.0f;
    
    if (pidEnabled) {
      updateVelocityRamping();
      diffDriveController(rampedV, rampedW, dt);
      updatePID();
    }
    lastPIDTime = now;
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
    Serial.print(targetVl);
    Serial.print(",");
    Serial.println(targetVr);
    lastEncoderPrint = now;
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("D") || cmd.startsWith("d")) {
      int commaIndex = cmd.indexOf(',');
      if (commaIndex > 0) {
        float v = cmd.substring(1, commaIndex).toFloat();
        float w = cmd.substring(commaIndex + 1).toFloat();
        
        pidEnabled = true;
        resetPID();
        
        V = v;
        W = w;
        rampedV = v;
        rampedW = w;
        
        if (headingLockEnabled && headingInitialized && abs(v) > 0.02) {
          targetHeading = filteredYaw;
          headingIntegral = 0;
          headingPrevError = 0;
          Serial.print("Heading locked at: ");
          Serial.print(targetHeading);
          Serial.println(" deg");
        }
        
        Serial.print("Differential drive: V=");
        Serial.print(v);
        Serial.print(" m/s, W=");
        Serial.print(w);
        Serial.println(" rad/s");
      }
    }
    else if (cmd.startsWith("M") || cmd.startsWith("m")) {
      int commaIndex = cmd.indexOf(',');
      if (commaIndex > 0) {
        int leftPwm = cmd.substring(1, commaIndex).toInt();
        int rightPwm = cmd.substring(commaIndex + 1).toInt();
        
        pidEnabled = false;
        setMotors(leftPwm, rightPwm);
        
        resetPID();
        V = 0;
        W = 0;
        rampedV = 0;
        rampedW = 0;
        targetVl = 0;
        targetVr = 0;
        
        Serial.print("Manual PWM: L=");
        Serial.print(leftPwm);
        Serial.print(" R=");
        Serial.println(rightPwm);
      }
    }
    else if (cmd.startsWith("V") || cmd.startsWith("v")) {
      int commaIndex = cmd.indexOf(',');
      if (commaIndex > 0) {
        float leftVel = cmd.substring(1, commaIndex).toFloat();
        float rightVel = cmd.substring(commaIndex + 1).toFloat();
        
        pidEnabled = true;
        
        V = (rightVel + leftVel) / 2.0f;
        W = (rightVel - leftVel) / WHEEL_SEPARATION_M;
        rampedV = V;
        rampedW = W;
        
        resetPID();
        
        Serial.print("Target velocity converted to: V=");
        Serial.print(V);
        Serial.print(" m/s, W=");
        Serial.println(W);
      }
    }
    else if (cmd.equalsIgnoreCase("H")) {
      headingLockEnabled = !headingLockEnabled;
      if (headingLockEnabled) {
        targetHeading = filteredYaw;
        headingIntegral = 0;
        headingPrevError = 0;
        Serial.println("Heading lock ENABLED");
      } else {
        Serial.println("Heading lock DISABLED");
      }
    }
    else if (cmd.equalsIgnoreCase("Y")) {
      Serial.print("Yaw: ");
      Serial.print(yaw * 180.0 / PI);
      Serial.print(" deg, Filtered: ");
      Serial.print(filteredYaw);
      Serial.print(" deg, Gyro Z: ");
      Serial.print(gyro_z * 180.0 / PI);
      Serial.println(" deg/s");
      Serial.print("Heading lock: ");
      Serial.println(headingLockEnabled ? "ENABLED" : "DISABLED");
      Serial.print("Target heading: ");
      Serial.println(targetHeading);
    }
    else if (cmd.equalsIgnoreCase("P")) {
      Serial.print("Pose: x=");
      Serial.print(x);
      Serial.print(" y=");
      Serial.print(y);
      Serial.print(" theta=");
      Serial.print(theta * 180.0 / PI);
      Serial.println(" deg");
    }
    else if (cmd.equalsIgnoreCase("S")) {
      setMotors(0, 0);
      pidEnabled = true;
      resetPID();
      targetVl = 0;
      targetVr = 0;
      V = 0;
      W = 0;
      rampedV = 0;
      rampedW = 0;
      Serial.println("Stopped");
    }
  }
}