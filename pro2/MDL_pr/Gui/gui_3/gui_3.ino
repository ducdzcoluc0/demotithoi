#include "Arduino.h"
#include "LoRa.h"
#include "DHT.h"

// Dinh nghia chan 
#define NSS 5
#define RST 14
#define DIO0 26
#define MISO 19
#define SCK 18
#define MOSI 23

// Cau hinh chan cam bien DHT11
#define DHTtype DHT11
#define DHTpins 13
DHT myDHT(DHTpins, DHTtype);

float NhietDo, DoAm;

// Timing variables
unsigned long lastSend = 0;
const unsigned long sendInterval = 2000;  // Gui du lieu moi 2 giay

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("Bat dau gui...");
  myDHT.begin();

  // Khoi dong lora
  LoRa.setPins(NSS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Loi lora!");
    while (1);
  }
  
  Serial.println("Thanh cong!");

  // Cau hinh Lora
  LoRa.setTxPower(14);              // Cong suat phat (2-20)
  LoRa.setSpreadingFactor(7);       // He so trai pho
  LoRa.setSignalBandwidth(125E3);   // bang thong tin hieu 125k Hz
  LoRa.setCodingRate4(5);           // thiet lap ty le ma hoa
  LoRa.setPreambleLength(8);        // thiet lap do dai tien to
  LoRa.setSyncWord(0x12);           // ma dinh danh 
  LoRa.enableCrc();                 // bat kiem tra loi CRC cho goi tin
}

bool readSensor() {
  NhietDo = myDHT.readTemperature();
  DoAm = myDHT.readHumidity();
  
  if (isnan(NhietDo) || isnan(DoAm)) {
    Serial.println("Loi cam bien!");
    return false;
  }
  
  // Validate humidity range
  if (DoAm < 0) DoAm = 0;
  if (DoAm > 100) DoAm = 100;
  
  return true;
}

void sendData() {
  if (readSensor()) {
    // Simple format: "T25.5H60.2"
    String dataPacket = "T" + String(NhietDo, 1) + "H" + String(DoAm, 1);
    
    Serial.print("Sending: ");
    Serial.println(dataPacket);
    
    LoRa.beginPacket();
    LoRa.print(dataPacket);
    LoRa.endPacket();
  } else {
    // Send error message
    Serial.println("Loi tin hieu");
    LoRa.beginPacket();
    LoRa.print("ERROR");
    LoRa.endPacket();
  }
}

void loop() {
  unsigned long now = millis();
  
  // Send data at regular intervals
  if (now - lastSend >= sendInterval) {
    sendData();
    lastSend = now;
  }
  
  delay(100);  
}
// Tom tat hoat dong
// 1. Khoi dong LoRa
// 2. Cu 2s doc nhiet do, do am, tao chuoi dang T25.5H60.2, gui qua Lora
// 3. Neu loi cam bien -> GUI THONG BAO LOI
