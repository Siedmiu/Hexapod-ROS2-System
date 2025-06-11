#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 imu;
unsigned long startTime;

#define IMU_SDA_PIN 4      
#define IMU_SCL_PIN 18      
#define IMU_INT_PIN 5      


void setup() {
  Serial.begin(115200);
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);


  if (!imu.begin(0x68)) {
    Serial.println("Nie znaleziono imu na adresie 0x68");
    while (1) delay(10);
  }

  imu.setAccelerometerRange(MPU6050_RANGE_8_G);

  imu.setGyroRange(MPU6050_RANGE_500_DEG);

  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("imu: Time [ms],Accel X [m/s^2],Accel Y [m/s^2],Accel Z [m/s^2],Angle X [deg],Angle Y [deg],Angle Z [deg]");

  startTime = millis();
}

void loop() {
  sensors_event_t a1, g1, temp1;
  sensors_event_t a2, g2, temp2;

  imu.getEvent(&a1, &g1, &temp1);
  unsigned long elapsed = millis() - startTime;

  Serial.print("imu: ");
  //Serial.print("1,");
  Serial.print(elapsed);
  Serial.print(",");
  Serial.print(a1.acceleration.x, 3);
  Serial.print(",");
  Serial.print(a1.acceleration.y, 3);
  Serial.print(",");
  Serial.print(a1.acceleration.z, 3);
  Serial.print(",");
  Serial.print(g1.gyro.x, 3);
  Serial.print(",");
  Serial.print(g1.gyro.y, 3);
  Serial.print(",");
  Serial.println(g1.gyro.z, 3);

  delay(100); // 10 Hz
}
