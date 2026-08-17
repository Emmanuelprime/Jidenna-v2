#include <math.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// Define custom I2C pins for MPU6050
#define I2C_SDA 32
#define I2C_SCL 33

// Motor pins
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
#define ENCODER_PRINT_INTERVAL 50  // Changed from 100 to 50 for 20Hz telemetry
#define ODOMETRY_INTERVAL       20

#define LEFT_FORWARD   HIGH
#define RIGHT_FORWARD  LOW

#define WHEEL_DIAMETER_M    0.165f
#define WHEEL_SEPARATION_M  0.521f
#define PULSES_PER_REV      45

#define FILTER_ALPHA 0.15f

// Forward calibration
#define LEFT_SLOPE_FWD     0.0411f
#define LEFT_INTERCEPT_FWD -0.2392f
#define RIGHT_SLOPE_FWD    0.0267f
#define RIGHT_INTERCEPT_FWD -0.0816f

// Reverse calibration (swapped characteristics)
#define LEFT_SLOPE_REV     0.0267f   
#define LEFT_INTERCEPT_REV -0.0816f
#define RIGHT_SLOPE_REV    0.0411f   
#define RIGHT_INTERCEPT_REV -0.2392f

#define MIN_START_PWM 10

#define LOW_SPEED_PWM_PER_MPS 30.0f
#define LOW_SPEED_THRESHOLD 0.15f

// Reduced maximum acceleration for smoother transitions
#define MAX_ACCELERATION 0.8f

// Minimum speed for PID control to prevent hunting
#define MIN_SPEED_FOR_PID 0.02f

// Stop sequence timing
#define STOP_HOLD_TIME 200  // Hold zero PWM for 200ms when stopping

// Command timeout
#define COMMAND_TIMEOUT_MS 2000  // Stop if no command received within 500ms

// Odometry variables
float x = 0.0;
float y = 0.0;
float theta = 0.0;

unsigned long samplerate = 100;

// Wheel speeds
float vl = 0.0;
float vr = 0.0;
float filteredVl = 0.0;
float filteredVr = 0.0;

// Command vs Actual Ramped Target Velocities
float rawTargetVl = 0.0;
float rawTargetVr = 0.0;
float targetVl = 0.0;
float targetVr = 0.0;

// PID variables
float prevErrorVl = 0.0;
float integralVl = 0.0;
float outputVl = 0.0;
float prevErrorVr = 0.0;
float integralVr = 0.0;
float outputVr = 0.0;

float prevMeasuredVl = 0.0;
float prevMeasuredVr = 0.0;

// PID Gains
float kp_l = 15.0f;     
float ki_l = 5.0f;      
float kd_l = 0.5f;      

float kp_r = 20.0f;     
float ki_r = 8.0f;      
float kd_r = 0.5f;      

// Encoder variables
volatile long lastLeftPulses = 0;
volatile long lastRightPulses = 0;
volatile long leftPulses = 0;
volatile long rightPulses = 0;

volatile unsigned long lastLeftInterrupt = 0;
volatile unsigned long lastRightInterrupt = 0;

volatile bool leftEncoderDirection = true;
volatile bool rightEncoderDirection = true;

// Timing variables
unsigned long lastEncoderPrint = 0;
unsigned long lastOdometryUpdate = 0;
unsigned long lastSpeedMeasure = 0;
unsigned long lastPIDTime = 0;
unsigned long lastRampTime = 0;
unsigned long lastCommandTime = 0;  // Track when last command was received

// Anti-windup and stopping sequence
bool isStopping = false;
unsigned long stopStartTime = 0;

// MPU6050 object
MPU6050 mpu6050(Wire);

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
  float dtheta = (dr - dl) / WHEEL_SEPARATION_M;
  
  theta += dtheta;
  normalizeAngle();
  
  x += dc * cos(theta);
  y += dc * sin(theta);
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
  static long prevLeftPulses = 0;
  static long prevRightPulses = 0;

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
}

float pidControl(float target, float current, float dt, float kp, float ki, float kd,
                  float &prevError, float &integral, float &prevMeasured) {
  float error = target - current;
  
  if (fabs(target) < MIN_SPEED_FOR_PID && fabs(current) < MIN_SPEED_FOR_PID) {
    integral = 0;
    prevError = 0;
    prevMeasured = current;
    return 0;
  }
  
  float P = kp * error;
  
  if (fabs(error) < 0.5f) {
    integral += error * dt;
    integral = constrain(integral, -50.0f, 50.0f);
  } else {
    integral *= 0.95f;
  }
  float I = ki * integral;
  
  float D = -kd * (current - prevMeasured) / dt;
  
  float output = P + I + D;
  prevError = error;
  prevMeasured = current;
  
  return output;
}

void resetPID() {
  prevErrorVl = 0;
  integralVl = 0;
  prevErrorVr = 0;
  integralVr = 0;
  outputVl = 0;
  outputVr = 0;
  prevMeasuredVl = 0;
  prevMeasuredVr = 0;
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

// Convert V,W commands to wheel velocities
void setVW(float linearVel, float angularVel) {
  // Differential drive kinematics:
  // vl = V - (W * L)/2
  // vr = V + (W * L)/2
  float leftVel = linearVel - (angularVel * WHEEL_SEPARATION_M) / 2.0f;
  float rightVel = linearVel + (angularVel * WHEEL_SEPARATION_M) / 2.0f;
  
  rawTargetVl = leftVel;
  rawTargetVr = rightVel;
  
  // Update command timestamp
  lastCommandTime = millis();
}

void updateVelocityRamping() {
  unsigned long now = millis();
  float dt = (now - lastRampTime) / 1000.0f;
  if (dt <= 0) return;
  lastRampTime = now;

  float maxStep = MAX_ACCELERATION * dt;

  if (isStopping) {
    maxStep *= 3.0f;
  }

  if (targetVl < rawTargetVl) {
    targetVl = fmin(targetVl + maxStep, rawTargetVl);
  } else if (targetVl > rawTargetVl) {
    targetVl = fmax(targetVl - maxStep, rawTargetVl);
  }

  if (targetVr < rawTargetVr) {
    targetVr = fmin(targetVr + maxStep, rawTargetVr);
  } else if (targetVr > rawTargetVr) {
    targetVr = fmax(targetVr - maxStep, rawTargetVr);
  }
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
  
  // Check for command timeout
  if (!isStopping && (now - lastCommandTime > COMMAND_TIMEOUT_MS)) {
    // No command received within timeout period, initiate stop sequence
    isStopping = true;
    stopStartTime = now;
    setMotors(0, 0);
    resetPID();
    filteredVl = 0; filteredVr = 0;
    vl = 0; vr = 0;
    rawTargetVl = 0; rawTargetVr = 0;
    targetVl = 0; targetVr = 0;
  }
  
  // Stopping sequence
  if (isStopping) {
    if (now - stopStartTime < STOP_HOLD_TIME) {
      setMotors(0, 0);
      lastPIDTime = now;
      return;
    } else {
      isStopping = false;
      filteredVl = 0;
      filteredVr = 0;
      vl = 0;
      vr = 0;
      resetPID();
      lastPIDTime = now;
      lastRampTime = now;
      return;
    }
  }
  
  float dt = (now - lastPIDTime) / 1000.0f;
  if (dt <= 0) return;

  updateVelocityRamping();
  
  float currentVl = filteredVl;
  float currentVr = filteredVr;

  float pidOutputL = pidControl(targetVl, currentVl, dt, kp_l, ki_l, kd_l,
                                 prevErrorVl, integralVl, prevMeasuredVl);
  float pidOutputR = pidControl(targetVr, currentVr, dt, kp_r, ki_r, kd_r,
                                 prevErrorVr, integralVr, prevMeasuredVr);
  
  int ffL = calculateFeedForward(targetVl, true);
  int ffR = calculateFeedForward(targetVr, false);
  
  int leftPwm = constrain((int)(pidOutputL + ffL), -MAX_PWM, MAX_PWM);
  int rightPwm = constrain((int)(pidOutputR + ffR), -MAX_PWM, MAX_PWM);
  
  setMotors(leftPwm, rightPwm);
  
  lastPIDTime = now;
}

void setup() {
  Serial.begin(115200);

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

  // Initialize I2C with custom pins for MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize MPU6050
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  setMotors(0, 0);
  resetPID();
  lastRampTime = millis();
  lastPIDTime = millis();
  lastOdometryUpdate = millis();
  lastCommandTime = millis();  // Initialize command time
}

void loop() {
  unsigned long now = millis();

  if (now - lastSpeedMeasure >= samplerate) {
    float dt = (now - lastSpeedMeasure) / 1000.0f;
    MeasureWheelSpeeds(dt);
    lastSpeedMeasure = now;
  }
  
  if (now - lastPIDTime >= samplerate) {
    updatePID();
  }

  if (now - lastOdometryUpdate >= ODOMETRY_INTERVAL) {
    updateOdometry();
    lastOdometryUpdate = now;
  }

  if (now - lastEncoderPrint >= ENCODER_PRINT_INTERVAL) {
    // Update MPU6050 data
    mpu6050.update();
    
    // Print all data with timestamp
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(theta);
    Serial.print(",");
    Serial.print(filteredVl); 
    Serial.print(",");
    Serial.print(filteredVr);
    Serial.print(",");
    Serial.print(mpu6050.getAngleZ());
    Serial.print(",");
    Serial.print(mpu6050.getGyroZ());
    Serial.print(",");
    Serial.println(now);  // Add timestamp in milliseconds
    lastEncoderPrint = now;
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Only accept V,W commands
    if (cmd.startsWith("V") || cmd.startsWith("v")) {
      // V,W command: V<linear_velocity>,<angular_velocity>
      int commaIndex = cmd.indexOf(',');
      if (commaIndex > 0) {
        float linearVel = cmd.substring(1, commaIndex).toFloat();
        float angularVel = cmd.substring(commaIndex + 1).toFloat();
        isStopping = false;
        setVW(linearVel, angularVel);
      }
    }
    // All other commands are ignored
  }
}