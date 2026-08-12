#include <Arduino.h>

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
#define MAX_PWM       60

#define ENCODER_PRINT_INTERVAL 100
#define DEBOUNCE_TIME_US 100

#define LEFT_FORWARD   HIGH
#define RIGHT_FORWARD  LOW

#define PULSES_PER_REVOLUTION 45   
#define WHEEL_DIAMETER_M 0.165
#define WHEEL_CIRCUMFERENCE_M (WHEEL_DIAMETER_M * 3.14159)

// Filter settings - removed moving average
#define MAX_SPEED_CHANGE 0.05

volatile long leftPulses = 0;
volatile long rightPulses = 0;
volatile bool leftDirection = true;
volatile bool rightDirection = true;
volatile unsigned long lastLeftInterrupt = 0;
volatile unsigned long lastRightInterrupt = 0;
int leftPWM = 0;
int rightPWM = 0;
unsigned long lastEncoderPrint = 0;

unsigned long lastSpeedCalcTime = 0;
long lastLeftPulses = 0;
long lastRightPulses = 0;
float leftSpeed = 0.0;
float rightSpeed = 0.0;

// Removed moving average buffers and related variables
float prevLeftSpeed = 0.0;
float prevRightSpeed = 0.0;
bool firstReading = true;

void IRAM_ATTR leftISR() { 
  unsigned long now = micros();
  if (now - lastLeftInterrupt >= DEBOUNCE_TIME_US) {
    lastLeftInterrupt = now;
    if (leftDirection) leftPulses++; else leftPulses--;
  }
}

void IRAM_ATTR rightISR() { 
  unsigned long now = micros();
  if (now - lastRightInterrupt >= DEBOUNCE_TIME_US) {
    lastRightInterrupt = now;
    if (rightDirection) rightPulses++; else rightPulses--;
  }
}

void setLeftDirection(bool forward) {
  setLeftPWM(0);
  delay(50);
  digitalWrite(LEFT_DIR, forward ? LEFT_FORWARD : !LEFT_FORWARD);
  leftDirection = forward;
}

void setRightDirection(bool forward) {
  setRightPWM(0);
  delay(50);
  digitalWrite(RIGHT_DIR, forward ? RIGHT_FORWARD : !RIGHT_FORWARD);
  rightDirection = forward;
}

void setLeftPWM(int pwm) {
  pwm = constrain(abs(pwm), 0, MAX_PWM);
  leftPWM = pwm;
  ledcWrite(LEFT_PWM_CH, pwm);
}

void setRightPWM(int pwm) {
  pwm = constrain(abs(pwm), 0, MAX_PWM);
  rightPWM = pwm;
  ledcWrite(RIGHT_PWM_CH, pwm);
}

void stopMotors() {
  setLeftPWM(0);
  setRightPWM(0);
}

void moveBothMotors(int leftPwm, int rightPwm) {
  if (leftPwm >= 0) {
    if (!leftDirection) setLeftDirection(true);
  } else {
    if (leftDirection) setLeftDirection(false);
  }
  
  if (rightPwm >= 0) {
    if (!rightDirection) setRightDirection(true);
  } else {
    if (rightDirection) setRightDirection(false);
  }
  
  setLeftPWM(abs(leftPwm));
  setRightPWM(abs(rightPwm));
}

void calculateSpeed() {
  unsigned long now = millis();
  float dt = (now - lastSpeedCalcTime) / 1000.0;
  
  if (dt > 0.001) {
    noInterrupts();
    long leftDelta = leftPulses - lastLeftPulses;
    long rightDelta = rightPulses - lastRightPulses;
    lastLeftPulses = leftPulses;
    lastRightPulses = rightPulses;
    interrupts();

    float leftRPS = leftDelta / (PULSES_PER_REVOLUTION * dt);
    float rightRPS = rightDelta / (PULSES_PER_REVOLUTION * dt);

    float leftRawSpeed = leftRPS * WHEEL_CIRCUMFERENCE_M;
    float rightRawSpeed = rightRPS * WHEEL_CIRCUMFERENCE_M;
    
    // Keep the speed change limiter (optional - can also remove if you want)
    if (!firstReading) {
      if (fabs(leftRawSpeed - prevLeftSpeed) > MAX_SPEED_CHANGE && prevLeftSpeed != 0) {
        if (leftRawSpeed > prevLeftSpeed) {
          leftRawSpeed = prevLeftSpeed + MAX_SPEED_CHANGE;
        } else {
          leftRawSpeed = prevLeftSpeed - MAX_SPEED_CHANGE;
        }
      }
      if (fabs(rightRawSpeed - prevRightSpeed) > MAX_SPEED_CHANGE && prevRightSpeed != 0) {
        if (rightRawSpeed > prevRightSpeed) {
          rightRawSpeed = prevRightSpeed + MAX_SPEED_CHANGE;
        } else {
          rightRawSpeed = prevRightSpeed - MAX_SPEED_CHANGE;
        }
      }
    }
    
    prevLeftSpeed = leftRawSpeed;
    prevRightSpeed = rightRawSpeed;
    
    // Removed moving average filter - using raw speed directly
    leftSpeed = leftRawSpeed;
    rightSpeed = rightRawSpeed;
    
    if (firstReading) firstReading = false;
  }
  
  lastSpeedCalcTime = now;
}

void printData() {
  // Format: PWM_LEFT,PWM_RIGHT,SPEED_LEFT,SPEED_RIGHT
  Serial.print(leftPWM);
  Serial.print(",");
  Serial.print(rightPWM);
  Serial.print(",");
  Serial.print(leftSpeed, 4);
  Serial.print(",");
  Serial.println(rightSpeed, 4);
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
  
  setLeftDirection(true);
  setRightDirection(true);
  stopMotors();
  
  lastEncoderPrint = millis();
  lastSpeedCalcTime = millis();
  lastLeftPulses = 0;
  lastRightPulses = 0;
  lastLeftInterrupt = 0;
  lastRightInterrupt = 0;
  prevLeftSpeed = 0;
  prevRightSpeed = 0;
  firstReading = true;
  
  Serial.println("READY");
  Serial.println("Commands:");
  Serial.println("  M<pwmL>,<pwmR>  - Move both motors (M50,-30)");
  Serial.println("                    Positive = Forward, Negative = Reverse");
  Serial.println("  S                - Stop both motors");
  Serial.println("Format: PWM_LEFT,PWM_RIGHT,SPEED_LEFT,SPEED_RIGHT");
}

void loop() {
  unsigned long now = millis();
  
  if (now - lastSpeedCalcTime >= ENCODER_PRINT_INTERVAL) {
    calculateSpeed();
  }
  
  if (now - lastEncoderPrint >= ENCODER_PRINT_INTERVAL) {
    printData();
    lastEncoderPrint = now;
  }
  
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() > 0) {
      char command = cmd.charAt(0);
      
      if (command == 'M' || command == 'm') {
        if (cmd.length() > 2) {
          String params = cmd.substring(1);
          int commaIndex = params.indexOf(',');
          
          if (commaIndex > 0) {
            int leftPwm = params.substring(0, commaIndex).toInt();
            int rightPwm = params.substring(commaIndex + 1).toInt();
            
            leftPwm = constrain(leftPwm, -MAX_PWM, MAX_PWM);
            rightPwm = constrain(rightPwm, -MAX_PWM, MAX_PWM);
            
            moveBothMotors(leftPwm, rightPwm);
            
            Serial.print("Moving - Left: ");
            Serial.print(leftPwm);
            Serial.print("  Right: ");
            Serial.println(rightPwm);
          }
        }
      }
      else if (command == 'S' || command == 's') {
        stopMotors();
        Serial.println("Stopped");
      }
      else {
        Serial.println("Commands: M<pwmL>,<pwmR>, S");
        Serial.println("Example: M50,-30 (Left forward 50, Right reverse 30)");
      }
    }
  }
}