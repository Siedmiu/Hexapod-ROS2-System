#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Sterowniki PCA9685
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
BluetoothSerial SerialBT;

// IMU sensor
Adafruit_MPU6050 imu;
unsigned long imuStartTime;

const int SERVO_MIN = 104;
const int SERVO_MAX = 490;
const int PWM_FREQUENCY = 50;
const int STEP_DELAY = 300;

// Timing for data transmission
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // 100ms = 10Hz for both pin status and IMU

// IMU interrupt pin (optional)
#define IMU_INT_PIN 5      

int pin_status[6] = {0};
int offsets[18] = {0}; // Offsety dla 18 serw

// Monitorowane piny
const int monitoredPins[] = {26, 14, 27, 32, 33, 25};
int lastPinStates[6] = {0};

void saveOffsets() {
  File file = SPIFFS.open("/offsets.txt", "w");
  if (!file) {
    //Serial.println("Błąd zapisu offsetów");
    return;
  }
  for (int i = 0; i < 18; i++) {
    file.printf("%d:%d\n", i, offsets[i]);
  }
  file.close();
  //Serial.println("Offsety zapisane.");
}

bool loadOffsets() {
  if (!SPIFFS.begin(true)) {
    //Serial.println("SPIFFS nie działa.");
    return false;
  }
  File file = SPIFFS.open("/offsets.txt", "r");
  if (!file) {
    //Serial.println("Brak pliku offsetów. Użycie domyślnych.");
    return false;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    int colon = line.indexOf(':');
    if (colon > 0) {
      int index = line.substring(0, colon).toInt();
      int value = line.substring(colon + 1).toInt();
      if (index >= 0 && index < 18) {
        offsets[index] = value;
      }
    }
  }
  file.close();
  //Serial.println("Offsety załadowane.");
  return true;
}

void setServoAngle(uint8_t servoIndex, float angle) {
  angle = constrain(angle + offsets[servoIndex], 0, 180);
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  if (servoIndex < 9)
    pwm1.setPWM(servoIndex, 0, pulse);
  else if (servoIndex < 18)
    pwm2.setPWM(servoIndex - 9, 0, pulse);
  // else
    //Serial.println("Zły numer serwa.");
}

bool isLeftCoxa(int idx) {
  return (idx == 0 || idx == 3 || idx == 6);
}

void moveLegToStand(int baseIndex) {
  float angleCoxa = isLeftCoxa(baseIndex) ? 90 : 90;
  setServoAngle(baseIndex, angleCoxa);
  setServoAngle(baseIndex + 1, 90);
  setServoAngle(baseIndex + 2, 60);
}

void stand() {
  for (int i = 0; i < 6; i++) {
    moveLegToStand(i * 3);
  }
  //Serial.println("Tryb STAND.");
}

void moveLeg(int baseIndex, float j1, float j2, float j3) {
  float angleCoxa = isLeftCoxa(baseIndex) ? 180 - j1 : j1;
  setServoAngle(baseIndex, angleCoxa);
  setServoAngle(baseIndex + 1, j2);
  setServoAngle(baseIndex + 2, j3);
}

void walkStepTripod(int groupA[], int groupB[], int stepAngle) {
  for (int i = 0; i < 3; i++)
    moveLeg(groupA[i], 90, 60, 45);
  delay(STEP_DELAY);

  for (int i = 0; i < 3; i++)
    moveLeg(groupA[i], 90 + stepAngle, 90, 60);
  delay(STEP_DELAY);

  for (int i = 0; i < 3; i++)
    moveLeg(groupA[i], 90 - stepAngle, 90, 60);

  for (int i = 0; i < 3; i++)
    moveLeg(groupB[i], 90, 60, 45);
  delay(STEP_DELAY);

  for (int i = 0; i < 3; i++)
    moveLeg(groupB[i], 90 + stepAngle, 90, 60);
  delay(STEP_DELAY);

  for (int i = 0; i < 3; i++)
    moveLeg(groupB[i], 90 - stepAngle, 90, 60);
}

void walk(int steps = 4, int stepAngle = 20) {
  int groupA[] = {0, 9, 12};
  int groupB[] = {3, 6, 15};
  for (int i = 0; i < steps; i++)
    walkStepTripod(groupA, groupB, stepAngle);
}

int getMaxOffsetForServo(int servoNum) {
  int jointPos = servoNum % 3;  // 0,1,2 - joint1,2,3
  if (jointPos == 0) return 6;  // joint 1 (coxa) ±6°
  else return 4;                // joint 2 i 3 ±4°
}

void parseAndHandleCommand(String msg) {
  msg.trim();
  if (msg.startsWith("servo")) {
    int digitPos = 5;
    while (digitPos < msg.length() && !isDigit(msg[digitPos])) digitPos++;
    int spacePos = msg.indexOf(' ', digitPos);
    if (spacePos != -1) {
      int servoNum = msg.substring(digitPos, spacePos).toInt();
      int angle = msg.substring(spacePos + 1).toInt();
      if (servoNum >= 0 && servoNum < 18 && angle >= 0 && angle <= 180) {
        setServoAngle(servoNum, angle);
        // Serial.printf("Servo %d -> %d°\n", servoNum, angle);
      } 
      // else {
        //Serial.printf("Błąd serwa lub kąta: %d / %d\n", servoNum, angle);
      // }
    }
  }
  else if (msg.startsWith("offset")) {
    int parts = msg.indexOf(' ', 7);
    if (parts != -1) {
      int servoNum = msg.substring(7, parts).toInt();
      int offsetVal = msg.substring(parts + 1).toInt();
      if (servoNum >= 0 && servoNum < 18) {
        int maxOffset = getMaxOffsetForServo(servoNum);
        if (offsetVal >= -maxOffset && offsetVal <= maxOffset) {
          offsets[servoNum] = offsetVal;
          saveOffsets();
          //Serial.printf("Offset %d ustawiony na %d\n", servoNum, offsetVal);
        } 
        // else {
          //Serial.printf("Błąd: Offset %d przekracza limit ±%d dla serwa %d\n", offsetVal, maxOffset, servoNum);
        // }
      }
    }
  }
  else if (msg.equalsIgnoreCase("stand")) {
    //Serial.println("Ustawianie w pozycji stand...");
    stand();
  }
  else if (msg.startsWith("walk")) {
    int space = msg.indexOf(' ');
    int steps = 4;
    if (space != -1) {
      steps = msg.substring(space + 1).toInt();
      if (steps <= 0) steps = 4;
    }
    //Serial.printf("Kroków: %d\n", steps);
    walk(steps, 20);
  }
  else {
    //Serial.printf("Nieznana komenda: %s\n", msg.c_str());
  }
}

void sendIMUData() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);
  unsigned long elapsed = millis() - imuStartTime;

  Serial.print("imu: ");
  Serial.print(elapsed);
  Serial.print(",");
  Serial.print(a.acceleration.x, 3);
  Serial.print(",");
  Serial.print(a.acceleration.y, 3);
  Serial.print(",");
  Serial.print(a.acceleration.z, 3);
  Serial.print(",");
  Serial.print(g.gyro.x, 3);
  Serial.print(",");
  Serial.print(g.gyro.y, 3);
  Serial.print(",");
  Serial.println(g.gyro.z, 3);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C with standard pins (21=SDA, 22=SCL)
  Wire.begin();
  
  // Initialize SPIFFS
  SPIFFS.begin(true);

  // Initialize PCA9685 drivers
  pwm1.begin();
  pwm1.setPWMFreq(PWM_FREQUENCY);
  pwm2.begin();
  pwm2.setPWMFreq(PWM_FREQUENCY);

  Serial.println("PCA9685 gotowe.");
  
  // Initialize IMU
  if (!imu.begin(0x68)) {
    Serial.println("Nie znaleziono IMU na adresie 0x68");
    // Continue without IMU if not found
  } else {
    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("imu: Time [ms],Accel X [m/s^2],Accel Y [m/s^2],Accel Z [m/s^2],Angle X [deg],Angle Y [deg],Angle Z [deg]");
    imuStartTime = millis();
  }
  
  // Load servo offsets
  loadOffsets();

  // Configure monitored pins as inputs
  for (int i = 0; i < 6; i++) {
    pinMode(monitoredPins[i], INPUT_PULLUP);
    lastPinStates[i] = digitalRead(monitoredPins[i]);
  }
}

void loop() {
  // Handle serial commands
  static String inputString = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        parseAndHandleCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += c;
    }
  }

  // Send data at regular intervals (100ms = 10Hz)
  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= sendInterval) {
    lastSendTime = currentMillis;
    
    // Monitor pin states and send if changed
    bool pinChange = false;
    for (int i = 0; i < 6; i++) {
      int currentState = digitalRead(monitoredPins[i]);
      if (currentState != lastPinStates[i]) {
        pin_status[i] = currentState;
        lastPinStates[i] = currentState;
        pinChange = true;
      }
    }
    
    if (pinChange) {
      String statusMessage = "Pin status: ";
      for (int i = 0; i < 6; i++) {
        statusMessage += String(pin_status[i]) + " ";
      }
      Serial.println(statusMessage);
    }
    
    // Send IMU data
    sendIMUData();
  }
}