#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <FS.h>
#include <SPIFFS.h>
#include <esp_now.h>
#include <WiFi.h>

// Sterowniki PCA9685
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

const int SERVO_MIN = 104;
const int SERVO_MAX = 490;
const int PWM_FREQUENCY = 50;
const int STEP_DELAY = 1000;

int offsets[18] = {0}; // Offsety dla 18 serw

// Piny do odczytu stanów
const int inputPins[6] = {39, 34, 35, 32, 33, 25};
uint8_t pinStates[6] = {0}; // ZMIANA: uint8_t zamiast int
uint8_t prevPinStates[6] = {255, 255, 255, 255, 255, 255}; // ZMIANA: 255 zamiast -1

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100;

uint8_t broadcastAddress[] = {0xCC, 0xDB, 0xA7, 0x9E, 0x60, 0x2C};

// --- Funkcje do offsetów ---
void saveOffsets() {
  File file = SPIFFS.open("/offsets.txt", "w");
  if (!file) {
    Serial.println("Błąd zapisu offsetów");
    return;
  }
  for (int i = 0; i < 18; i++) {
    file.printf("%d:%d\n", i, offsets[i]);
  }
  file.close();
  Serial.println("Offsety zapisane.");
}

bool loadOffsets() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS nie działa.");
    return false;
  }
  File file = SPIFFS.open("/offsets.txt", "r");
  if (!file) {
    Serial.println("Brak pliku offsetów. Użycie domyślnych.");
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
  Serial.println("Offsety załadowane.");
  return true;
}

// --- Sterowanie serwami ---
void setServoAngle(uint8_t servoIndex, float angle) {
  angle = constrain(angle + offsets[servoIndex], 0, 180);
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  if (servoIndex < 9)
    pwm1.setPWM(servoIndex, 0, pulse);
  else if (servoIndex < 18)
    pwm2.setPWM(servoIndex - 9, 0, pulse);
  else
    Serial.println("Zły numer serwa.");
}

bool isLeftCoxa(int idx) {
  return (idx == 0 || idx == 3 || idx == 6);
}

void moveLegToStand(int baseIndex) {
  float angleCoxa = isLeftCoxa(baseIndex) ? 90 : 90;
  setServoAngle(baseIndex, angleCoxa);
  setServoAngle(baseIndex + 1, 90);
  setServoAngle(baseIndex + 2, 20);
}

void stand() {
  for (int i = 0; i < 6; i++) {
    moveLegToStand(i * 3);
  }
  Serial.println("Tryb STAND.");
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
    moveLeg(groupA[i], 90 + stepAngle, 90, 20);
  delay(STEP_DELAY);

  for (int i = 0; i < 3; i++)
    moveLeg(groupA[i], 90 - stepAngle, 90, 20);

  for (int i = 0; i < 3; i++)
    moveLeg(groupB[i], 90, 60, 45);
  delay(STEP_DELAY);

  for (int i = 0; i < 3; i++)
    moveLeg(groupB[i], 90 + stepAngle, 90, 20);
  delay(STEP_DELAY);

  for (int i = 0; i < 3; i++)
    moveLeg(groupB[i], 90 - stepAngle, 90, 20);
}

void walk(int steps = 4, int stepAngle = 20) {
  int groupA[] = {0, 6, 12};
  int groupB[] = {3, 9, 15};
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
        Serial.printf("Servo %d -> %d°\n", servoNum, angle);
      } else {
        Serial.printf("Błąd serwa lub kąta: %d / %d\n", servoNum, angle);
      }
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
          Serial.printf("Offset %d ustawiony na %d\n", servoNum, offsetVal);
        } else {
          Serial.printf("Błąd: Offset %d przekracza limit ±%d dla serwa %d\n", offsetVal, maxOffset, servoNum);
        }
      }
    }
  }
  else if (msg.equalsIgnoreCase("stand")) {
    Serial.println("Ustawianie w pozycji stand...");
    stand();
  }
  else if (msg.startsWith("walk")) {
    int space = msg.indexOf(' ');
    int steps = 4;
    if (space != -1) {
      steps = msg.substring(space + 1).toInt();
      if (steps <= 0) steps = 4;
    }
    Serial.printf("Kroków: %d\n", steps);
    walk(steps, 20);
  }
  else {
    Serial.printf("Nieznana komenda: %s\n", msg.c_str());
  }
}

// --- ESP-NOW callback ---
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char msg[len + 1];
  memcpy(msg, incomingData, len);
  msg[len] = '\0';
  String message = String(msg);
  Serial.printf("Odebrano ESP-NOW: %s\n", msg);
  parseAndHandleCommand(message);
}

// --- Odczyt stanów pinów ---
void readPins() {
  for (int i = 0; i < 6; i++) {
    pinStates[i] = digitalRead(inputPins[i]) ? 1 : 0; // ZMIANA: jawne przypisanie 0/1
  }
}

// --- Wysyłanie stanów pinów ---
void sendPinStates() {
  // Sprawdź czy peer istnieje
  if (!esp_now_is_peer_exist(broadcastAddress)) {
    Serial.println("Odbiornik nie istnieje!");
    return;
  }
  
  esp_err_t result = esp_now_send(broadcastAddress, pinStates, sizeof(pinStates));
  
  if (result == ESP_OK) {
    Serial.print("Wysyłam stany: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(pinStates[i]);
      if (i < 5) Serial.print(",");
    }
    Serial.println();
  } else {
    Serial.printf("Błąd wysyłania: %d\n", result);
    
    // Dodatkowe info o błędach
    switch(result) {
      case ESP_ERR_ESPNOW_NOT_INIT:
        Serial.println("ESP-NOW nie zainicjalizowany");
        break;
      case ESP_ERR_ESPNOW_ARG:
        Serial.println("Nieprawidłowy argument");
        break;
      case ESP_ERR_ESPNOW_INTERNAL:
        Serial.println("Błąd wewnętrzny");
        break;
      case ESP_ERR_ESPNOW_NO_MEM:
        Serial.println("Brak pamięci");
        break;
      case ESP_ERR_ESPNOW_NOT_FOUND:
        Serial.println("Peer nie znaleziony");
        break;
      default:
        Serial.printf("Nieznany błąd: %d\n", result);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  SPIFFS.begin(true);

  pwm1.begin();
  pwm1.setPWMFreq(PWM_FREQUENCY);
  pwm2.begin();
  pwm2.setPWMFreq(PWM_FREQUENCY);

  Serial.println("PCA9685 gotowe.");
  loadOffsets();

  // Konfiguracja pinów wejściowych
  for (int i = 0; i < 6; i++) {
    pinMode(inputPins[i], INPUT);
  }

  // WiFi i ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(true) { delay(1000); }
  }
  
  esp_now_register_recv_cb(onDataRecv);
  
  // Dodanie odbiorcy (peer)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;  // WAŻNE: ustawienie interfejsu
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Błąd dodawania odbiorcy");
  } else {
    Serial.println("Odbiornik dodany pomyślnie");
  }
  
  // Callback dla potwierdzenia wysyłania
  esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
      Serial.println("✓ Wysłano pomyślnie");
    } else {
      Serial.println("✗ Błąd wysyłania");
    }
  });

  Serial.print("MAC nadajnika: ");
  Serial.println(WiFi.macAddress());
  
  stand();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    readPins();
    
    bool changed = false;
    for (int i = 0; i < 6; i++) {
      if (pinStates[i] != prevPinStates[i]) {
        changed = true;
        break;
      }
    }

    if (changed) {
      memcpy(prevPinStates, pinStates, sizeof(pinStates));
      sendPinStates();
    }
  }
}