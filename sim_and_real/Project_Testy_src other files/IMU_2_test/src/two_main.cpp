/*#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

// Create IMU objects
Adafruit_MPU6050 imu1;
Adafruit_MPU6050 imu2;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize IMU 1 (default address 0x68)
  if (!imu1.begin(0x68)) {
    Serial.println("Failed to find IMU1 at 0x68");
    while (1) delay(10);
  }

  // Initialize IMU 2 (custom address 0x69)
  if (!imu2.begin(0x69)) {
    Serial.println("Failed to find IMU2 at 0x69");
    while (1) delay(10);
  }

  imu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu2.setAccelerometerRange(MPU6050_RANGE_8_G);

  imu1.setGyroRange(MPU6050_RANGE_500_DEG);
  imu2.setGyroRange(MPU6050_RANGE_500_DEG);

  imu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
  imu2.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("IMUs initialized.");
}

void loop() {
  sensors_event_t a1, g1, temp1;
  sensors_event_t a2, g2, temp2;

  imu1.getEvent(&a1, &g1, &temp1);
  imu2.getEvent(&a2, &g2, &temp2);

  StaticJsonDocument<512> doc;

  JsonObject imu1_json = doc["imu1"].to<JsonObject>();
  imu1_json["frame_id"] = "imu1";
  imu1_json["linear_acceleration"]["x"] = a1.acceleration.x;
  imu1_json["linear_acceleration"]["y"] = a1.acceleration.y;
  imu1_json["linear_acceleration"]["z"] = a1.acceleration.z;
  imu1_json["angular_velocity"]["x"] = g1.gyro.x;
  imu1_json["angular_velocity"]["y"] = g1.gyro.y;
  imu1_json["angular_velocity"]["z"] = g1.gyro.z;

  JsonObject imu2_json = doc["imu2"].to<JsonObject>();
  imu2_json["frame_id"] = "imu2";
  imu2_json["linear_acceleration"]["x"] = a2.acceleration.x;
  imu2_json["linear_acceleration"]["y"] = a2.acceleration.y;
  imu2_json["linear_acceleration"]["z"] = a2.acceleration.z;
  imu2_json["angular_velocity"]["x"] = g2.gyro.x;
  imu2_json["angular_velocity"]["y"] = g2.gyro.y;
  imu2_json["angular_velocity"]["z"] = g2.gyro.z;

  serializeJson(doc, Serial);
  Serial.println();

  delay(100); // 10 Hz
}
*/
/*
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 imu1;
Adafruit_MPU6050 imu2;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!imu1.begin(0x68)) {
    Serial.println("Failed to find IMU1 at 0x68");
    while (1) delay(10);
  }
  if (!imu2.begin(0x69)) {
    Serial.println("Failed to find IMU2 at 0x69");
    while (1) delay(10);
  }

  imu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu2.setAccelerometerRange(MPU6050_RANGE_8_G);

  imu1.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  imu2.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  imu1.setGyroRange(MPU6050_RANGE_500_DEG);
  imu2.setGyroRange(MPU6050_RANGE_500_DEG);

  imu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
  imu2.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Kalibracja sensorów
    Serial.println("Kalibracja imu1");
    imu1.CalibrateAccel(6);
    imu1.CalibrateGyro(6);
    Serial.println("Kalibracja imu1 zakończona");    
    

    Serial.println("Kalibracja imu2");
    imu2.CalibrateAccel(6);
    imu2.CalibrateGyro(6);
    Serial.println("Kalibracja imu2 zakończona");

  Serial.println("IMUnr,Time [ms],Accel X [m/s^2],Accel Y [m/s^2],Accel Z [m/s^2],Angle X [deg],Angle Y [deg],Angle Z [deg]");

  startTime = millis();
}

void loop() {
  sensors_event_t a1, g1, temp1;
  sensors_event_t a2, g2, temp2;

  imu1.getEvent(&a1, &g1, &temp1);
  imu2.getEvent(&a2, &g2, &temp2);

  unsigned long elapsed = millis() - startTime;

  // IMU1 data line
  Serial.print("1,");
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

  // IMU2 data line
  Serial.print("2,");
  Serial.print(elapsed);
  Serial.print(",");
  Serial.print(a2.acceleration.x, 3);
  Serial.print(",");
  Serial.print(a2.acceleration.y, 3);
  Serial.print(",");
  Serial.print(a2.acceleration.z, 3);
  Serial.print(",");
  Serial.print(g2.gyro.x, 3);
  Serial.print(",");
  Serial.print(g2.gyro.y, 3);
  Serial.print(",");
  Serial.println(g2.gyro.z, 3);

  delay(100); // 10 Hz
}*/
/*
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h> 

MPU6050 imu1(0x68); // IMU 1 at address 0x68
MPU6050 imu2(0x69); // IMU 2 at address 0x69

unsigned long startTime;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Initialize both IMUs
    imu1.initialize();
    imu2.initialize();

    // Set both to ±2g
    imu1.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    imu2.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    // Calibrate both IMUs
    Serial.println("Kalibracja IMU 1...");
    imu1.CalibrateAccel(6);
    imu1.CalibrateGyro(6);

    Serial.println("Kalibracja IMU 2...");
    imu2.CalibrateAccel(6);
    imu2.CalibrateGyro(6);

    Serial.println("Kalibracja zakończona");

    startTime = millis();
}

void loop() {
    int16_t ax1, ay1, az1, gx1, gy1, gz1;
    int16_t ax2, ay2, az2, gx2, gy2, gz2;

    imu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    imu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

    unsigned long elapsed = millis() - startTime;

    // Convert raw accel to m/s²
    float ax1_ms2 = ax1 / 16384.0 * 9.81;
    float ay1_ms2 = ay1 / 16384.0 * 9.81;
    float az1_ms2 = az1 / 16384.0 * 9.81;

    float ax2_ms2 = ax2 / 16384.0 * 9.81;
    float ay2_ms2 = ay2 / 16384.0 * 9.81;
    float az2_ms2 = az2 / 16384.0 * 9.81;

    // Convert raw gyro to deg/s
    float gx1_dps = gx1 / 131.0;
    float gy1_dps = gy1 / 131.0;
    float gz1_dps = gz1 / 131.0;

    float gx2_dps = gx2 / 131.0;
    float gy2_dps = gy2 / 131.0;
    float gz2_dps = gz2 / 131.0;

    // Print IMU1
    Serial.print("1,");
    Serial.print(elapsed);
    Serial.print(",");
    Serial.print(ax1_ms2, 3);
    Serial.print(",");
    Serial.print(ay1_ms2, 3);
    Serial.print(",");
    Serial.print(az1_ms2, 3);
    Serial.print(",");
    Serial.print(gx1_dps, 3);
    Serial.print(",");
    Serial.print(gy1_dps, 3);
    Serial.print(",");
    Serial.println(gz1_dps, 3);

    // Print IMU2
    Serial.print("2,");
    Serial.print(elapsed);
    Serial.print(",");
    Serial.print(ax2_ms2, 3);
    Serial.print(",");
    Serial.print(ay2_ms2, 3);
    Serial.print(",");
    Serial.print(az2_ms2, 3);
    Serial.print(",");
    Serial.print(gx2_dps, 3);
    Serial.print(",");
    Serial.print(gy2_dps, 3);
    Serial.print(",");
    Serial.println(gz2_dps, 3);

    delay(100); // 10 Hz
}
*/


/*
//BEST CODE FROM ALL THOSE

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 imu1;
//Adafruit_MPU6050 imu2;
unsigned long startTime;

#define IMU_SDA_PIN 4       // I2C SDA pin - GPIO 4 (dobry pin)
#define IMU_SCL_PIN 18      // I2C SCL pin - GPIO 18 (dobry pin)  
#define IMU_INT_PIN 5       // Interrupt pin - GPIO 5 (akceptowalny)

// Funkcja dodatkowej kalibracji osi Z dla imu1
float calibrateAccelZ(float rawAccelZ) {
  // Stała wartość offsetu, którą trzeba odjąć (np. 2 m/s²) aby wartość spoczynkowa była bliższa 9.8 m/s²
  const float offset = 1.67;
  return rawAccelZ - offset;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);


  if (!imu1.begin(0x68)) {
    Serial.println("Nie znaleziono IMU1 na adresie 0x68");
    while (1) delay(10);
  }
  if (!imu2.begin(0x69)) {
    Serial.println("Nie znaleziono IMU2 na adresie 0x69");
    while (1) delay(10);
  } 

  // Ustawienie zakresu akcelerometru i żyroskopu
  imu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  //imu2.setAccelerometerRange(MPU6050_RANGE_8_G);

  imu1.setGyroRange(MPU6050_RANGE_500_DEG);
  //imu2.setGyroRange(MPU6050_RANGE_500_DEG);

  imu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //imu2.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("IMUnr,Time [ms],Accel X [m/s^2],Accel Y [m/s^2],Accel Z [m/s^2],Angle X [deg],Angle Y [deg],Angle Z [deg]");

  startTime = millis();
}

void loop() {
  sensors_event_t a1, g1, temp1;
  sensors_event_t a2, g2, temp2;

  imu1.getEvent(&a1, &g1, &temp1);
  //imu2.getEvent(&a2, &g2, &temp2);

  unsigned long elapsed = millis() - startTime;

  // Dane z IMU1 z dodatkową kalibracją osi Z
  Serial.print("1,");
  Serial.print(elapsed);
  Serial.print(",");
  Serial.print(a1.acceleration.x, 3);
  Serial.print(",");
  Serial.print(a1.acceleration.y, 3);
  Serial.print(",");
  // Zastosowanie funkcji kalibrującej dla osi Z (odjęcie offsetu)
  Serial.print(a1.acceleration.z, 3);//calibrateAccelZ(a1.acceleration.z), 3);
  Serial.print(",");
  Serial.print(g1.gyro.x, 3);
  Serial.print(",");
  Serial.print(g1.gyro.y, 3);
  Serial.print(",");
  Serial.println(g1.gyro.z, 3);

  // Dane z IMU2 bez dodatkowej korekty
  Serial.print("2,");
  Serial.print(elapsed);
  Serial.print(",");
  Serial.print(a2.acceleration.x, 3);
  Serial.print(",");
  Serial.print(a2.acceleration.y, 3);
  Serial.print(",");
  Serial.print(a2.acceleration.z, 3);
  Serial.print(",");
  Serial.print(g2.gyro.x, 3);
  Serial.print(",");
  Serial.print(g2.gyro.y, 3);
  Serial.print(",");
  Serial.println(g2.gyro.z, 3);

  delay(100); // 10 Hz
}

*/


/*
//FOR ONE
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

  Serial.println("IMUnr,Time [ms],Accel X [m/s^2],Accel Y [m/s^2],Accel Z [m/s^2],Angle X [deg],Angle Y [deg],Angle Z [deg]");

  startTime = millis();
}

void loop() {
  sensors_event_t a1, g1, temp1;
  sensors_event_t a2, g2, temp2;

  imu.getEvent(&a1, &g1, &temp1);
  unsigned long elapsed = millis() - startTime;

  Serial.print("1,");
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
*/