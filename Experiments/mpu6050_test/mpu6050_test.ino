#include <MPU6050_tockn.h>
#include <Wire.h>

// Define custom I2C pins
#define I2C_SDA 32
#define I2C_SCL 33

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C with custom pins
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize MPU6050
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();
  Serial.print("Angle X: ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("  Y: ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("  Z: ");
  Serial.println(mpu6050.getAngleZ());
  delay(100);
}