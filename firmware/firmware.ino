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

// MPU6050 heading correction - MUCH MORE AGGRESSIVE
#define HEADING_CORRECTION_KP 15.0f   // Increased from 8
#define HEADING_CORRECTION_KI 0.3f    // Increased from 0.15
#define HEADING_CORRECTION_KD 0.15f   // Increased from 0.08

#define COMPLEMENTARY_ALPHA 0.98f

MPU6050 mpu6050(Wire);

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

// Heading correction variables
float targetHeading = 0.0;
float headingIntegral = 0.0;
float headingPrevError = 0.0;
bool headingLockEnabled = true;

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

void getMpuData() {
  mpu6050.update();
  
  float rawYaw = mpu6050.getAngleZ() * PI / 180.0;
  gyro_z = mpu6050.getGyroZ() * PI / 180.0;
  
  if (fabs(gyro_z) < 2.0f) {
    yawFiltered = COMPLEMENTARY_ALPHA * (yawFiltered + gyro_z * 0.01f) + (1.0f - COMPLEMENTARY_ALPHA) * rawYaw;
  } else {
    yawFiltered = yawFiltered + gyro_z * 0.01f;
  }
  
  yaw = rawYaw;
}

float headingCorrectionPID(float targetHeading, float currentHeading, float gyroRate, float dt) {
  float error = targetHeading - currentHeading;
  
  while (error > PI) error -= 2 * PI;
  while (error < -PI) error += 2 * PI;
  
  // Much more aggressive P
  float P = HEADING_CORRECTION_KP * error;
  
  // I term
  headingIntegral += error * dt;
  headingIntegral = constrain(headingIntegral, -0.8f, 0.8f);
  float I = HEADING_CORRECTION_KI * headingIntegral;
  
  // D term from gyro
  float D = HEADING_CORRECTION_KD * (-gyroRate);
  
  float output = P + I + D;
  
  // Allow larger corrections
  output = constrain(output, -3.0f, 3.0f);
  
  headingPrevError = error;
  
  // Store for debug
  lastHeadingError = error;
  lastHeadingCorrection = output;
  
  return output;
}

void diffDriveController(float v, float w) {
  float L = WHEEL_SEPARATION_M;
  
  // Heading lock
  if (headingLockEnabled && fabs(v) > 0.005f) {
    float heading = yawFiltered;
    
    // Always correct unless we're intentionally turning hard
    if (fabs(w) < 0.5f) {
      float correction = headingCorrectionPID(targetHeading, heading, gyro_z, 0.05f);
      
      // Apply full correction
      w = w + correction;
      
      // Allow more correction range
      w = constrain(w, -2.0f, 2.0f);
      
      // Debug output
      if (millis() - lastDebugPrint > 500) {
        Serial.print("Heading error: ");
        Serial.print(lastHeadingError * 180.0 / PI);
        Serial.print(" deg, Correction: ");
        Serial.print(lastHeadingCorrection);
        Serial.print(" rad/s, W: ");
        Serial.println(w);
        lastDebugPrint = millis();
      }
    } else {
      headingIntegral = 0;
    }
  }
  
  float vl_target = v - (w * L) / 2.0f;
  float vr_target = v + (w * L) / 2.0f;
  
  vl_target = constrain(vl_target, -max_speed, max_speed);
  vr_target = constrain(vr_target, -max_speed, max_speed);
  
  setVelocity(vl_target, vr_target);
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
  headingIntegral = 0;
  headingPrevError = 0;
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

void updatePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  
  if (dt <= 0) return;
  
  updateVelocityRamping();
  
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
  rampedVl = 0;
  rampedVr = 0;
  lastRampTime = millis();
  
  targetHeading = yawFiltered;
  
  Serial.println("Robot ready - Heading lock ENABLED (AGGRESSIVE MODE)");
  Serial.println("Commands:");
  Serial.println("  D<V>,<W> - Set linear (m/s) and angular (rad/s) velocities");
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
    Serial.println(rampedVr);
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
        V = v;
        W = w;
        
        if (headingLockEnabled && fabs(v) > 0.005f) {
          targetHeading = yawFiltered;
          headingIntegral = 0;
          headingPrevError = 0;
          Serial.print("Heading locked at: ");
          Serial.print(targetHeading * 180.0 / PI);
          Serial.println(" deg");
        }
        
        diffDriveController(v, w);
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
        setMotors(leftPwm, rightPwm);
        resetPID();
        targetVl = 0;
        targetVr = 0;
        rampedVl = 0;
        rampedVr = 0;
        V = 0;
        W = 0;
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
    else if (cmd.equalsIgnoreCase("H")) {
      headingLockEnabled = !headingLockEnabled;
      if (headingLockEnabled) {
        targetHeading = yawFiltered;
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
      Serial.print(yawFiltered * 180.0 / PI);
      Serial.print(" deg, Gyro Z: ");
      Serial.print(gyro_z * 180.0 / PI);
      Serial.println(" deg/s");
      Serial.print("Heading lock: ");
      Serial.println(headingLockEnabled ? "ENABLED" : "DISABLED");
      Serial.print("Target heading: ");
      Serial.println(targetHeading * 180.0 / PI);
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
      resetPID();
      targetVl = 0;
      targetVr = 0;
      rampedVl = 0;
      rampedVr = 0;
      V = 0;
      W = 0;
      Serial.println("Stopped");
    }
  }
}