#include <Arduino.h>

// Pin definitions
#define LEFT_PWM     22
#define LEFT_DIR     23
#define LEFT_SC      34

#define RIGHT_PWM    27
#define RIGHT_DIR    26
#define RIGHT_SC     35

// PWM settings
#define LEFT_PWM_CH   2
#define RIGHT_PWM_CH  1
#define PWM_FREQ      1000
#define PWM_RES       8
#define MAX_PWM       60

// Encoder settings
#define DEBOUNCE_TIME_US   1000
#define ENCODER_PRINT_INTERVAL 100 // Print every 100ms

#define LEFT_FORWARD   HIGH
#define RIGHT_FORWARD  LOW

// Variables
volatile long leftPulses = 0;
volatile long rightPulses = 0;
volatile unsigned long lastLeftInterrupt = 0;
volatile unsigned long lastRightInterrupt = 0;
volatile bool leftDirection = true;
volatile bool rightDirection = true;
int leftPWM = 0;
int rightPWM = 0;
unsigned long lastEncoderPrint = 0;

// Left encoder interrupt
void IRAM_ATTR leftISR() { 
  unsigned long now = micros();
  if (now - lastLeftInterrupt >= DEBOUNCE_TIME_US) {
    lastLeftInterrupt = now;
    if (leftDirection) {
      leftPulses++;
    } else {
      leftPulses--;
    }
  }
}

// Right encoder interrupt
void IRAM_ATTR rightISR() { 
  unsigned long now = micros();
  if (now - lastRightInterrupt >= DEBOUNCE_TIME_US) {
    lastRightInterrupt = now;
    if (rightDirection) {
      rightPulses++;
    } else {
      rightPulses--;
    }
  }
}

// Motor control functions
void setLeftDirection(bool forward) {
  // Stop motor before changing direction
  setLeftPWM(0);
  delay(50); // Brief pause for motor to stop
  digitalWrite(LEFT_DIR, forward ? LEFT_FORWARD : !LEFT_FORWARD);
  leftDirection = forward;
}

void setRightDirection(bool forward) {
  // Stop motor before changing direction
  setRightPWM(0);
  delay(50); // Brief pause for motor to stop
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
  // Set directions based on PWM sign
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
  
  // Set PWM values
  setLeftPWM(abs(leftPwm));
  setRightPWM(abs(rightPwm));
}

void printEncoders() {
  noInterrupts();
  long left = leftPulses;
  long right = rightPulses;
  interrupts();
  
  Serial.print("E,");
  Serial.print(left);
  Serial.print(",");
  Serial.print(right);
  Serial.print(",");
  Serial.print(leftPWM);
  Serial.print(",");
  Serial.print(rightPWM);
  Serial.print(",");
  Serial.print(leftDirection ? "F" : "R");
  Serial.print(",");
  Serial.println(rightDirection ? "F" : "R");
}

void setup() {
  Serial.begin(115200);
  
  // Setup pins
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_SC, INPUT_PULLUP);
  pinMode(RIGHT_SC, INPUT_PULLUP);
  
  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_SC), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SC), rightISR, RISING);
  
  // Setup PWM
  ledcSetup(LEFT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_PWM, LEFT_PWM_CH);
  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_PWM, RIGHT_PWM_CH);
  
  // Initialize
  setLeftDirection(true);
  setRightDirection(true);
  stopMotors();
  
  lastEncoderPrint = millis();
  
  Serial.println("READY");
  Serial.println("Commands:");
  Serial.println("  M<pwmL>,<pwmR>  - Move both motors (M50,-30)");
  Serial.println("                    Positive = Forward, Negative = Reverse");
  Serial.println("  S                - Stop both motors");
  Serial.println("  P                - Print encoder counts once");
  Serial.println("  D                - Change direction (both motors)");
  Serial.println("  DL               - Change left direction");
  Serial.println("  DR               - Change right direction");
  Serial.println("  E                - Toggle encoder printing (on by default)");
  Serial.println("Format: E,leftPulses,rightPulses,leftPWM,rightPWM,leftDir,rightDir");
}

void loop() {
  // Print encoders continuously
  unsigned long now = millis();
  if (now - lastEncoderPrint >= ENCODER_PRINT_INTERVAL) {
    printEncoders();
    lastEncoderPrint = now;
  }
  
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() > 0) {
      char command = cmd.charAt(0);
      
      // Move both motors simultaneously
      if (command == 'M' || command == 'm') {
        if (cmd.length() > 2) {
          String params = cmd.substring(1);
          int commaIndex = params.indexOf(',');
          
          if (commaIndex > 0) {
            int leftPwm = params.substring(0, commaIndex).toInt();
            int rightPwm = params.substring(commaIndex + 1).toInt();
            
            // Clamp values
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
      // Stop both motors
      else if (command == 'S' || command == 's') {
        stopMotors();
        Serial.println("Stopped");
      }
      // Change both motor directions (with safety stop)
      else if (command == 'D') {
        if (cmd.length() == 1) {
          // Stop motors first
          stopMotors();
          delay(100);
          
          setLeftDirection(!leftDirection);
          setRightDirection(!rightDirection);
          Serial.print("Both motors direction: ");
          Serial.println(leftDirection ? "Forward" : "Reverse");
        }
        else if (cmd.length() > 1) {
          char subCmd = cmd.charAt(1);
          
          if (subCmd == 'L' || subCmd == 'l') {
            stopMotors();
            delay(100);
            setLeftDirection(!leftDirection);
            Serial.print("Left motor direction: ");
            Serial.println(leftDirection ? "Forward" : "Reverse");
          }
          else if (subCmd == 'R' || subCmd == 'r') {
            stopMotors();
            delay(100);
            setRightDirection(!rightDirection);
            Serial.print("Right motor direction: ");
            Serial.println(rightDirection ? "Forward" : "Reverse");
          }
        }
      }
      // Print encoder counts once
      else if (command == 'P' || command == 'p') {
        printEncoders();
      }
      // Toggle encoder printing
      else if (command == 'E' || command == 'e') {
        // Print is always on now, this just shows current state
        Serial.println("Encoder printing is always on");
        printEncoders();
      }
      else {
        Serial.println("Commands: M<pwmL>,<pwmR>, S, P, D, DL, DR");
        Serial.println("Example: M50,-30 (Left forward 50, Right reverse 30)");
      }
    }
  }
}