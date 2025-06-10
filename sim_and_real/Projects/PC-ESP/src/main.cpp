#include <esp_now.h>
#include <WiFi.h>

#define MAX_MSG_LEN 250

// Adres MAC odbiorcy — ten ESP odbiera i kontroluje serwa,
// tutaj możesz zostawić dowolny lub NULL dla broadcast
uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}; // przykładowy MAC, zmień jeśli chcesz

String inputString = "";

// Callback po wysłaniu wiadomości
void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Wysłano ESP-NOW do %02X:%02X:%02X:%02X:%02X:%02X - status: %s\n",
    mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
    (status == ESP_NOW_SEND_SUCCESS) ? "OK" : "FAIL");
}

// Callback po odebraniu wiadomości ESP-NOW
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.printf("Odebrano od %02X:%02X:%02X:%02X:%02X:%02X, długość: %d bajtów\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len);
  
  // Zakładam, że otrzymujesz tablicę stanów pinów, np. 6 bajtów
  Serial.print("Dane: ");
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", incomingData[i]);
  }
  Serial.println();

  // Możesz zrobić tu konwersję do tablicy int lub bool i użyć według potrzeb
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) { delay(1000); }
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onDataRecv);  // Rejestrujemy odbiór

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Nie udało się dodać odbiorcy ESP-NOW");
    while(true) { delay(1000); }
  }

  Serial.println("Most UART->ESP-NOW gotowy.");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        if (inputString.length() > MAX_MSG_LEN) {
          Serial.println("Wiadomość zbyt długa.");
          inputString = "";
          return;
        }
        // Wyślij przez ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)inputString.c_str(), inputString.length());
        if (result == ESP_OK) {
          Serial.printf("Wysłano: %s\n", inputString.c_str());
        } else {
          Serial.printf("Błąd wysyłania ESP-NOW: %d\n", result);
        }
        inputString = "";
      }
    } else {
      inputString += c;
    }
  }
}
