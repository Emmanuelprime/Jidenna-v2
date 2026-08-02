#include <MPU6050_tockn.h>
#include <Wire.h>
#include <esp_task_wdt.h>

// ─── CONFIGURATION ──────────────────────────────────────────────────────────

// ─── ESP32 PIN DEFINITIONS ──────────────────────────────────────────────────
#define LEFT_PWM      22
#define LEFT_DIR      23
#define LEFT_SC       34
#define LEFT_CURRENT  36

#define RIGHT_PWM     27
#define RIGHT_DIR     26
#define RIGHT_SC      35
#define RIGHT_CURRENT 39

#define I2C_SDA       32
#define I2C_SCL       33
#define E_STOP_PIN    38

// ─── PWM CONFIGURATION ──────────────────────────────────────────────────────
#define LEFT_PWM_CH   0
#define RIGHT_PWM_CH  1
#define PWM_FREQ      1000
#define PWM_RES       8
#define MAX_PWM       60
#define PWM_SLEW_RATE 5

// ─── MOTOR CONSTANTS ──────────────────────────────────────────────────────
#define WHEEL_DIAMETER_M  0.165f
#define PULSES_PER_REV    45
#define METERS_PER_PULSE  (PI * WHEEL_DIAMETER_M / PULSES_PER_REV)
#define WHEELBASE_M       0.52f

// ─── MOTOR CHARACTERIZATION ──────────────────────────────────────────────
#define LEFT_FWD_SLOPE     0.0209f
#define LEFT_FWD_INTERCEPT -0.0167f
#define LEFT_REV_SLOPE     0.0209f
#define LEFT_REV_INTERCEPT -0.0167f
#define LEFT_REV_COMP      1.30f

#define RIGHT_FWD_SLOPE    0.0209f
#define RIGHT_FWD_INTERCEPT -0.0167f
#define RIGHT_REV_SLOPE    0.0209f
#define RIGHT_REV_INTERCEPT -0.0167f
#define RIGHT_FWD_COMP     1.30f

#define LEFT_DEADZONE   10
#define RIGHT_DEADZONE  10

// ─── CONTROL TIMING ──────────────────────────────────────────────────────
#define CONTROL_INTERVAL_MS  50
#define CONTROL_DT           0.05f
#define WATCHDOG_TIMEOUT_MS  3000
#define IDLE_TIMEOUT_MS      5000

// ─── STATE MACHINE ──────────────────────────────────────────────────────
#define ACCELERATION_RATE  0.5f   
#define DECELERATION_RATE  0.8f   
#define EMERGENCY_STOP_DELAY_MS 100  

// ─── ENCODER ─────────────────────────────────────────────────────────────
#define DEBOUNCE_US 1000

// ─── IMU ─────────────────────────────────────────────────────────────────
#define HEADING_DEADZONE_DEG  0.3f
#define TURN_RATE_DEADZONE    0.015f
#define YAW_FILTER_ALPHA      0.92f
#define RATE_FILTER_ALPHA     0.5f
#define COMP_FILTER_ALPHA     0.98f

// ─── PID GAINS ──────────────────────────────────────────────────────────
#define KP_HEADING 1.2
#define KI_HEADING 0.01
#define KD_HEADING 0.08
#define HEADING_INTEGRAL_LIMIT 5.0
#define HEADING_OUTPUT_LIMIT 0.2

#define KP_TURN_RATE 1.8
#define KI_TURN_RATE 0.03
#define KD_TURN_RATE 0.08
#define TURN_RATE_INTEGRAL_LIMIT 0.5
#define TURN_RATE_OUTPUT_LIMIT 1.0

#define KP_LEFT  1.8   
#define KI_LEFT  0.02  
#define KD_LEFT  0.15  
#define LEFT_INTEGRAL_LIMIT  (MAX_PWM * 0.2f)

#define KP_RIGHT 2.5   
#define KI_RIGHT 0.03  
#define KD_RIGHT 0.20  
#define RIGHT_INTEGRAL_LIMIT (MAX_PWM * 0.2f)

// ─── CURRENT SENSING ──────────────────────────────────────────────────────
#define CURRENT_SCALE_FACTOR 0.0005f  // Calibrate based on your sensor
#define CURRENT_LIMIT_A 5.0f

// ─── GLOBALS ──────────────────────────────────────────────────────────────────

// ─── ENCODER VARIABLES ──────────────────────────────────────────────────────
volatile long leftPulses = 0;
volatile unsigned long lastLeftUS = 0;
volatile bool leftFwd = true;

volatile long rightPulses = 0;
volatile unsigned long lastRightUS = 0;
volatile bool rightFwd = true;

// ─── MOTOR STATE ──────────────────────────────────────────────────────────
float currentLeftSpeed = 0.0;
float currentRightSpeed = 0.0;
float filteredLeftSpeed = 0.0;
float filteredRightSpeed = 0.0;
int leftCurrentPWM = 0;
int rightCurrentPWM = 0;

// ─── COMMAND STATE ──────────────────────────────────────────────────────
float commandedLinearVelocity = 0.0;
float commandedAngularVelocity = 0.0;
float targetLinearVelocity = 0.0;
float targetAngularVelocity = 0.0;
float targetLeftSpeed = 0.0;
float targetRightSpeed = 0.0;
unsigned long lastCommandTime = 0;

// ─── PID STATE ──────────────────────────────────────────────────────────
float leftIntegral = 0;
float rightIntegral = 0;
float leftPrevError = 0;
float rightPrevError = 0;

float headingIntegral = 0;
float headingPrevError = 0;
float turnRateIntegral = 0;
float turnRatePrevError = 0;

// ─── IMU STATE ──────────────────────────────────────────────────────────
MPU6050 mpu6050(Wire);
float currentYaw = 0;
float filteredYaw = 0;
float targetYaw = 0;
float filteredAngularVelocity = 0;
float previousYaw = 0;
unsigned long lastYawTime = 0;
bool headingInitialized = false;
bool turnRateControlActive = false;

// ─── ODOMETRY ──────────────────────────────────────────────────────────
float odomX = 0.0;
float odomY = 0.0;
float odomYaw = 0.0;
float totalDistance = 0.0;
float stationaryTime = 0.0;

// ─── STATE MACHINE ──────────────────────────────────────────────────────
enum RobotState {
  STATE_IDLE,
  STATE_ACCELERATING,
  STATE_CRUISING,
  STATE_DECELERATING,
  STATE_EMERGENCY_STOP,
  STATE_CALIBRATING
};

RobotState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
bool emergencyStopFlag = false;

// ─── TIMING ──────────────────────────────────────────────────────────────
unsigned long lastControlTime = 0;
unsigned long lastIMUUpdate = 0;
unsigned long lastSerialTime = 0;

// ─── SERIAL BUFFER ──────────────────────────────────────────────────────
String serialBuffer = "";

// ─── FUNCTION PROTOTYPES ──────────────────────────────────────────────────
void initMPU6050();
void updateIMU();
void updateOdometry(float dt);
void updateStateMachine();
void updateMotorSpeeds();
void setMotorSpeed(float speed, bool isLeft);
void emergencyStop();
void processSerialCommands();
void parseCommand(String cmd);
void sendStatusReport();
void checkMotorCurrent();

// ─── ISR FUNCTIONS ──────────────────────────────────────────────────────────

void IRAM_ATTR leftISR() {
  unsigned long now = micros();
  if (now - lastLeftUS >= DEBOUNCE_US) {
    lastLeftUS = now;
    bool fwd = leftFwd;  // Atomic read
    leftPulses += fwd ? 1 : -1;
  }
}

void IRAM_ATTR rightISR() {
  unsigned long now = micros();
  if (now - lastRightUS >= DEBOUNCE_US) {
    lastRightUS = now;
    bool fwd = rightFwd;  // Atomic read
    rightPulses += fwd ? 1 : -1;
  }
}

void IRAM_ATTR emergencyStopISR() {
  emergencyStopFlag = true;
}

// ─── MOTOR CONTROL ──────────────────────────────────────────────────────────

void setMotorSpeed(float speed, bool isLeft) {
  bool fwd = speed >= 0;
  int pwm = constrain(abs(speed), 0, MAX_PWM);
  
  if (isLeft) {
    // Atomic update: set direction and PWM together
    noInterrupts();
    leftFwd = fwd;
    digitalWrite(LEFT_DIR, fwd ? HIGH : LOW);
    ledcWrite(LEFT_PWM_CH, pwm);
    leftCurrentPWM = pwm;
    interrupts();
  } else {
    noInterrupts();
    rightFwd = fwd;
    digitalWrite(RIGHT_DIR, fwd ? LOW : HIGH);  // RIGHT_FORWARD is LOW
    ledcWrite(RIGHT_PWM_CH, pwm);
    rightCurrentPWM = pwm;
    interrupts();
  }
}

void stopMotors() {
  setMotorSpeed(0, true);
  setMotorSpeed(0, rightFwd);
  
  // Reset all control states
  filteredLeftSpeed = 0;
  filteredRightSpeed = 0;
  targetLinearVelocity = 0;
  targetAngularVelocity = 0;
  targetLeftSpeed = 0;
  targetRightSpeed = 0;
  
  leftIntegral = 0;
  rightIntegral = 0;
  leftPrevError = 0;
  rightPrevError = 0;
  headingIntegral = 0;
  headingPrevError = 0;
  turnRateIntegral = 0;
  turnRatePrevError = 0;
  turnRateControlActive = false;
}

// ─── SPEED TO PWM CONVERSION ──────────────────────────────────────────────

float speedToPWM(float speed, bool isLeft) {
  float pwm;
  
  if (isLeft) {
    if (speed >= 0) {
      pwm = (speed - LEFT_FWD_INTERCEPT) / LEFT_FWD_SLOPE;
    } else {
      float absSpeed = -speed;
      pwm = (absSpeed - LEFT_REV_INTERCEPT) / LEFT_REV_SLOPE;
      pwm *= LEFT_REV_COMP;
      pwm = -pwm;
    }
  } else {
    if (speed >= 0) {
      pwm = (speed - RIGHT_FWD_INTERCEPT) / RIGHT_FWD_SLOPE;
      pwm *= RIGHT_FWD_COMP;
    } else {
      float absSpeed = -speed;
      pwm = (absSpeed - RIGHT_REV_INTERCEPT) / RIGHT_REV_SLOPE;
      pwm = -pwm;
    }
  }
  
  // Apply deadzone
  if (speed > 0.01) {
    pwm = max(pwm, (float)(isLeft ? LEFT_DEADZONE : RIGHT_DEADZONE));
  } else if (speed < -0.01) {
    pwm = min(pwm, -(float)(isLeft ? LEFT_DEADZONE : RIGHT_DEADZONE));
  } else {
    pwm = 0;
  }
  
  return constrain(pwm, -MAX_PWM, MAX_PWM);
}

// ─── PID CONTROLLER ──────────────────────────────────────────────────────────

struct PIDController {
  float Kp, Ki, Kd;
  float integralLimit;
  float outputLimit;
  float integral;
  float prevError;
  
  void init(float p, float i, float d, float iLimit, float oLimit) {
    Kp = p; Ki = i; Kd = d;
    integralLimit = iLimit;
    outputLimit = oLimit;
    integral = 0;
    prevError = 0;
  }
  
  float update(float target, float current, float dt) {
    float error = target - current;
    float output = Kp * error + integral + Kd * (error - prevError) / dt;
    
    // Anti-windup: only integrate if output isn't saturated
    if (abs(output) < outputLimit) {
      integral += Ki * error * dt;
      integral = constrain(integral, -integralLimit, integralLimit);
    }
    
    prevError = error;
    return constrain(output, -outputLimit, outputLimit);
  }
  
  void reset() {
    integral = 0;
    prevError = 0;
  }
};

PIDController leftPID, rightPID, headingPID, turnRatePID;

// ─── IMU FUNCTIONS ──────────────────────────────────────────────────────────

void initMPU6050() {
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  mpu6050.update();
  currentYaw = mpu6050.getAngleZ();
  filteredYaw = currentYaw;
  targetYaw = currentYaw;
  previousYaw = currentYaw;
  lastYawTime = micros();
  headingInitialized = true;
  
  odomYaw = filteredYaw * (PI / 180.0);
  
  // Initialize PID controllers
  leftPID.init(KP_LEFT, KI_LEFT, KD_LEFT, LEFT_INTEGRAL_LIMIT, MAX_PWM);
  rightPID.init(KP_RIGHT, KI_RIGHT, KD_RIGHT, RIGHT_INTEGRAL_LIMIT, MAX_PWM);
  headingPID.init(KP_HEADING, KI_HEADING, KD_HEADING, HEADING_INTEGRAL_LIMIT, HEADING_OUTPUT_LIMIT);
  turnRatePID.init(KP_TURN_RATE, KI_TURN_RATE, KD_TURN_RATE, TURN_RATE_INTEGRAL_LIMIT, TURN_RATE_OUTPUT_LIMIT);
}

void updateIMU() {
  mpu6050.update();
  currentYaw = mpu6050.getAngleZ();
  
  unsigned long now = micros();
  float dt = (now - lastYawTime) / 1000000.0f;
  if (dt > 0.001 && dt < 0.1) {
    float yawDelta = currentYaw - previousYaw;
    while (yawDelta > 180) yawDelta -= 360;
    while (yawDelta < -180) yawDelta += 360;
    float rawRate = yawDelta / dt * (PI / 180.0);
    
    // Exponential moving average filters
    filteredAngularVelocity = RATE_FILTER_ALPHA * rawRate + (1.0f - RATE_FILTER_ALPHA) * filteredAngularVelocity;
    filteredYaw = YAW_FILTER_ALPHA * currentYaw + (1.0f - YAW_FILTER_ALPHA) * filteredYaw;
  }
  previousYaw = currentYaw;
  lastYawTime = now;
}

// ─── ODOMETRY ──────────────────────────────────────────────────────────────

void updateOdometry(float dt) {
  float v = (currentLeftSpeed + currentRightSpeed) / 2.0f;
  float w = (currentRightSpeed - currentLeftSpeed) / WHEELBASE_M;
  
  // Correct heading with IMU when stationary to prevent drift
  if (abs(currentLeftSpeed) < 0.01 && abs(currentRightSpeed) < 0.01) {
    stationaryTime += dt;
    if (stationaryTime > 0.5 && headingInitialized) {
      odomYaw = filteredYaw * (PI / 180.0);
      stationaryTime = 0;
    }
  } else {
    stationaryTime = 0;
  }
  
  if (headingInitialized) {
    odomYaw = filteredYaw * (PI / 180.0);
  } else {
    static float wheelYaw = 0;
    wheelYaw += w * dt;
    odomYaw = wheelYaw;
  }
  
  float deltaX = v * cos(odomYaw) * dt;
  float deltaY = v * sin(odomYaw) * dt;
  
  odomX += deltaX;
  odomY += deltaY;
  totalDistance += abs(v) * dt;
}

// ─── STATE MACHINE ──────────────────────────────────────────────────────────

void updateStateMachine() {
  unsigned long now = millis();
  
  // Check emergency stop
  if (emergencyStopFlag) {
    emergencyStop();
    emergencyStopFlag = false;
    return;
  }
  
  switch (currentState) {
    case STATE_IDLE:
      if (commandedLinearVelocity != 0 || commandedAngularVelocity != 0) {
        currentState = STATE_ACCELERATING;
        stateStartTime = now;
        targetLinearVelocity = 0;
        targetAngularVelocity = 0;
      }
      break;
      
    case STATE_ACCELERATING: {
      float dt_accel = (now - stateStartTime) / 1000.0f;
      float target_v = commandedLinearVelocity;
      float target_w = commandedAngularVelocity;
      
      // Smooth S-curve acceleration
      float ramp_v = target_v * (dt_accel * ACCELERATION_RATE / (abs(target_v) + 0.001));
      float ramp_w = target_w * (dt_accel * ACCELERATION_RATE / (abs(target_w) + 0.001));
      
      if (abs(ramp_v) >= abs(target_v)) ramp_v = target_v;
      if (abs(ramp_w) >= abs(target_w)) ramp_w = target_w;
      
      targetLinearVelocity = ramp_v;
      targetAngularVelocity = ramp_w;
      
      if (abs(targetLinearVelocity - commandedLinearVelocity) < 0.01 && 
          abs(targetAngularVelocity - commandedAngularVelocity) < 0.01) {
        currentState = STATE_CRUISING;
        stateStartTime = now;
      }
      break;
    }
      
    case STATE_CRUISING:
      targetLinearVelocity = commandedLinearVelocity;
      targetAngularVelocity = commandedAngularVelocity;
      
      if (commandedLinearVelocity == 0 && commandedAngularVelocity == 0) {
        currentState = STATE_DECELERATING;
        stateStartTime = now;
      }
      break;
      
    case STATE_DECELERATING: {
      float dt_decel = (now - stateStartTime) / 1000.0f;
      float decel_v = commandedLinearVelocity * (1.0f - dt_decel * DECELERATION_RATE / (abs(commandedLinearVelocity) + 0.001));
      float decel_w = commandedAngularVelocity * (1.0f - dt_decel * DECELERATION_RATE / (abs(commandedAngularVelocity) + 0.001));
      
      if (abs(decel_v) > 0 && abs(decel_v) < 0.01) decel_v = 0;
      if (abs(decel_w) > 0 && abs(decel_w) < 0.01) decel_w = 0;
      
      targetLinearVelocity = decel_v;
      targetAngularVelocity = decel_w;
      
      if (abs(targetLinearVelocity) < 0.005 && abs(targetAngularVelocity) < 0.005) {
        targetLinearVelocity = 0;
        targetAngularVelocity = 0;
        currentState = STATE_IDLE;
        stateStartTime = now;
      }
      break;
    }
      
    case STATE_EMERGENCY_STOP:
      targetLinearVelocity = 0;
      targetAngularVelocity = 0;
      stopMotors();
      
      if (now - stateStartTime > EMERGENCY_STOP_DELAY_MS) {
        currentState = STATE_IDLE;
        stateStartTime = now;
      }
      break;
      
    case STATE_CALIBRATING:
      if (now - stateStartTime > 3000) {
        currentState = STATE_IDLE;
        stateStartTime = now;
      }
      break;
  }
  
  // Safety: Auto-stop if no command received for too long
  if (currentState != STATE_IDLE && currentState != STATE_EMERGENCY_STOP) {
    if (commandedLinearVelocity != 0 || commandedAngularVelocity != 0) {
      lastCommandTime = now;
    }
    if (now - lastCommandTime > IDLE_TIMEOUT_MS && lastCommandTime > 0) {
      Serial.println("WARN: Idle timeout - emergency stop");
      emergencyStop();
    }
  }
}

void emergencyStop() {
  Serial.println("EMERGENCY STOP");
  currentState = STATE_EMERGENCY_STOP;
  stateStartTime = millis();
  commandedLinearVelocity = 0;
  commandedAngularVelocity = 0;
  targetLinearVelocity = 0;
  targetAngularVelocity = 0;
  stopMotors();
}

void setCommand(float v, float w) {
  commandedLinearVelocity = constrain(v, -1.2, 1.2);
  commandedAngularVelocity = constrain(w, -2.0, 2.0);
  lastCommandTime = millis();
  
  if (currentState == STATE_IDLE && (commandedLinearVelocity != 0 || commandedAngularVelocity != 0)) {
    currentState = STATE_ACCELERATING;
    stateStartTime = millis();
  }
  
  // Reset integrals on new command
  leftPID.reset();
  rightPID.reset();
  
  if (abs(v) > 0.01 && abs(w) < 0.01) {
    targetYaw = filteredYaw;
    headingPID.reset();
  }
}

// ─── MAIN MOTOR UPDATE ──────────────────────────────────────────────────────

void updateMotorSpeeds() {
  unsigned long now = micros();
  static unsigned long lastUpdateUS = 0;
  float dt = (now - lastUpdateUS) / 1000000.0f;
  lastUpdateUS = now;
  
  if (dt > 0.1) dt = 0.1;
  if (dt < 0.001) dt = 0.001;
  
  // Read encoders atomically
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
  
  // Filter velocities
  float alpha = 0.3;
  filteredLeftSpeed = alpha * rawLeftSpeed + (1 - alpha) * filteredLeftSpeed;
  filteredRightSpeed = alpha * rawRightSpeed + (1 - alpha) * filteredRightSpeed;
  
  currentLeftSpeed = filteredLeftSpeed;
  currentRightSpeed = filteredRightSpeed;
  
  updateOdometry(dt);
  
  // Update IMU
  if (millis() - lastIMUUpdate > 10) {
    updateIMU();
    lastIMUUpdate = millis();
  }
  
  updateStateMachine();
  
  // Heading correction when moving forward
  float headingCorrection = 0;
  float turnRateCorrection = 0;
  
  if (headingInitialized && abs(targetLinearVelocity) > 0.02 && abs(targetAngularVelocity) < 0.01) {
    headingCorrection = headingPID.update(targetYaw, filteredYaw, dt);
    turnRateControlActive = false;
  }
  
  // Turn rate control when turning
  if (abs(targetAngularVelocity) > 0.01) {
    turnRateControlActive = true;
    turnRateCorrection = turnRatePID.update(targetAngularVelocity, filteredAngularVelocity, dt);
  } else {
    turnRateControlActive = false;
  }
  
  float effectiveOmega = targetAngularVelocity + headingCorrection * 0.8 + turnRateCorrection * 0.8;
  
  // Calculate wheel speeds
  float vL = targetLinearVelocity - (effectiveOmega * WHEELBASE_M) / 2.0f;
  float vR = targetLinearVelocity + (effectiveOmega * WHEELBASE_M) / 2.0f;
  
  targetLeftSpeed = vL;
  targetRightSpeed = vR;
  
  // Calculate PWM with feedforward + PID correction
  float leftFeedforward = speedToPWM(targetLeftSpeed, true);
  float rightFeedforward = speedToPWM(targetRightSpeed, false);
  
  float leftPidCorrection = leftPID.update(targetLeftSpeed, currentLeftSpeed, dt);
  float rightPidCorrection = rightPID.update(targetRightSpeed, currentRightSpeed, dt);
  
  float leftOutput = leftFeedforward + leftPidCorrection;
  float rightOutput = rightFeedforward + rightPidCorrection;
  
  // Apply slew rate limiting
  static int prevLeftPWM = 0;
  static int prevRightPWM = 0;
  
  if (targetLeftSpeed != 0 || targetRightSpeed != 0) {
    int leftPWM = constrain(abs(leftOutput), 0, MAX_PWM);
    int rightPWM = constrain(abs(rightOutput), 0, MAX_PWM);
    
    leftPWM = constrain(leftPWM, prevLeftPWM - PWM_SLEW_RATE, prevLeftPWM + PWM_SLEW_RATE);
    rightPWM = constrain(rightPWM, prevRightPWM - PWM_SLEW_RATE, prevRightPWM + PWM_SLEW_RATE);
    
    prevLeftPWM = leftPWM;
    prevRightPWM = rightPWM;
    
    setMotorSpeed(leftPWM * (vL >= 0 ? 1 : -1), true);
    setMotorSpeed(rightPWM * (vR >= 0 ? 1 : -1), false);
  } else {
    setMotorSpeed(0, true);
    setMotorSpeed(0, false);
    prevLeftPWM = 0;
    prevRightPWM = 0;
  }
  
  // Check motor current
  checkMotorCurrent();
}

// ─── CURRENT MONITORING ──────────────────────────────────────────────────────

void checkMotorCurrent() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 100) return;
  lastCheck = millis();
  
  float leftCurrent = analogRead(LEFT_CURRENT) * CURRENT_SCALE_FACTOR;
  float rightCurrent = analogRead(RIGHT_CURRENT) * CURRENT_SCALE_FACTOR;
  
  if (leftCurrent > CURRENT_LIMIT_A || rightCurrent > CURRENT_LIMIT_A) {
    Serial.printf("WARN: Overcurrent L=%.2fA R=%.2fA\n", leftCurrent, rightCurrent);
    emergencyStop();
  }
}

// ─── SERIAL COMMANDS ──────────────────────────────────────────────────────────

void processSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (serialBuffer.length() > 0) {
        parseCommand(serialBuffer);
        serialBuffer = "";
      }
    } else if (c == '\r') {
      // Ignore carriage return
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 64) serialBuffer = "";
    }
  }
}

void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  
  if (cmd.charAt(0) == 'V' || cmd.charAt(0) == 'v') {
    cmd = cmd.substring(1);
    int comma = cmd.indexOf(',');
    if (comma != -1) {
      float v = cmd.substring(0, comma).toFloat();
      float w = cmd.substring(comma + 1).toFloat();
      setCommand(v, w);
      
      // Send acknowledgment
      Serial.printf("ACK,V,%.3f,%.3f\n", v, w);
    }
  } else if (cmd == "STATUS" || cmd == "status") {
    sendStatusReport();
  } else if (cmd == "STOP" || cmd == "stop") {
    emergencyStop();
    Serial.println("ACK,STOP");
  } else if (cmd == "RESET" || cmd == "reset") {
    ESP.restart();
  } else if (cmd == "CALIBRATE" || cmd == "calibrate") {
    currentState = STATE_CALIBRATING;
    stateStartTime = millis();
    initMPU6050();
    Serial.println("ACK,CALIBRATE");
  }
}

void sendStatusReport() {
  Serial.printf("STATUS,state=%d,x=%.3f,y=%.3f,yaw=%.2f,v=%.3f,w=%.3f,left=%.3f,right=%.3f,pwmL=%d,pwmR=%d\n",
    currentState, odomX, odomY, filteredYaw, 
    (currentLeftSpeed + currentRightSpeed) / 2.0f,
    (currentRightSpeed - currentLeftSpeed) / WHEELBASE_M,
    currentLeftSpeed, currentRightSpeed,
    leftCurrentPWM, rightCurrentPWM);
}

void sendTelemetry() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend < CONTROL_INTERVAL_MS) return;
  lastSend = millis();
  
  float actualOmega = (currentRightSpeed - currentLeftSpeed) / WHEELBASE_M;
  float actualLinear = (currentLeftSpeed + currentRightSpeed) / 2.0f;
  
  Serial.print("CNT,");
  Serial.print(millis());
  Serial.print(',');
  Serial.print(currentLeftSpeed, 3);
  Serial.print(',');
  Serial.print(currentRightSpeed, 3);
  Serial.print(',');
  Serial.print(actualLinear, 3);
  Serial.print(',');
  Serial.print(actualOmega, 3);
  Serial.print(',');
  Serial.print(filteredAngularVelocity, 3);
  Serial.print(',');
  Serial.print(filteredYaw, 2);
  Serial.print(',');
  Serial.print(odomX, 3);
  Serial.print(',');
  Serial.print(odomY, 3);
  Serial.print(',');
  Serial.print(leftCurrentPWM);
  Serial.print(',');
  Serial.print(rightCurrentPWM);
  Serial.print(',');
  Serial.print(currentState);
  Serial.println();
}

// ─── SETUP ──────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // ─── Initialize Hardware ──────────────────────────────────────────────────
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_SC, INPUT_PULLUP);
  pinMode(LEFT_CURRENT, INPUT);
  
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_SC, INPUT_PULLUP);
  pinMode(RIGHT_CURRENT, INPUT);
  
  pinMode(E_STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), emergencyStopISR, FALLING);
  
  // ─── PWM Setup ────────────────────────────────────────────────────────────
  ledcSetup(LEFT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_PWM, LEFT_PWM_CH);
  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_PWM, RIGHT_PWM_CH);
  
  // ─── Encoder Interrupts ──────────────────────────────────────────────────
  attachInterrupt(digitalPinToInterrupt(LEFT_SC), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SC), rightISR, RISING);
  
  // ─── IMU Setup ────────────────────────────────────────────────────────────
  initMPU6050();
  
  // ─── Watchdog Timer ──────────────────────────────────────────────────────
  esp_task_wdt_init(WATCHDOG_TIMEOUT_MS / 1000, true);
  esp_task_wdt_add(NULL);
  
  // ─── Initial State ──────────────────────────────────────────────────────
  stopMotors();
  Serial.println("READY");
  Serial.println("# Send V<v>,<w> to control robot");
  Serial.println("# Example: V0.5,0.0 (forward 0.5 m/s)");
  Serial.println("# Example: V0.0,1.0 (spin left 1.0 rad/s)");
  Serial.println("# Commands: STATUS, STOP, RESET, CALIBRATE");
  Serial.println("# Output: CNT,time,vL,vR,linear,omega,actualOmega,yaw,x,y,leftPWM,rightPWM,state");
}

// ─── MAIN LOOP ──────────────────────────────────────────────────────────────

void loop() {
  // Reset watchdog
  esp_task_wdt_reset();
  
  // Process serial commands
  processSerialCommands();
  
  // Update motor control
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = now;
    updateMotorSpeeds();
    sendTelemetry();
  }
  
  delay(1);
}