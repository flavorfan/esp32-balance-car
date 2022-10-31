
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

int sda_pin=13;
int scl_pin=15;
 
void setup() {
  Serial.begin(115200);
  Wire.begin(sda_pin,scl_pin);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();
  Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
  Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
  Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
//  Serial.print("\tangleY : ");
//  Serial.print(mpu6050.getAngleY());
//  Serial.print("\tangleZ : ");
//  Serial.println(mpu6050.getAngleZ());
}
