// Simplified Transmitter Code - Fast and Stable
#include "Arduino.h"
#include "LoRa.h"
#include "DHT.h"

// Pin definitions
#define NSS 5
#define RST 14
#define DIO0 26
#define MISO 19
#define SCK 18
#define MOSI 23

#define LedAlarm 2

// DHT sensor configuration
#define DHTtype DHT11
#define DHTpins 13
DHT myDHT(DHTpins, DHTtype);

// Variables for sensor data
float NhietDo, DoAm;

// Timing variables
unsigned long lastSend = 0;
const unsigned long sendInterval = 2000;  // Send every 2 seconds

void setup() {
  Serial.begin(9600);
  delay(1000);

  pinMode(LedAlarm, OUTPUT);

  Serial.println("Bat dau gui...");
  myDHT.begin();

  // Initialize LoRa
  LoRa.setPins(NSS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed!");
    while (1);
  }
  
  Serial.println("LoRa initialized successfully!");

  // cau hinh lora
  LoRa.setTxPower(14);              // cong suat vua phai
  LoRa.setSpreadingFactor(7);       // he so trai pho nho -> toc do cao
  LoRa.setSignalBandwidth(125E3);   // bang thong chuan 125KHz
  LoRa.setCodingRate4(5);           // ty le ma hoa
  LoRa.setPreambleLength(8);        // do dai tien to
  LoRa.setSyncWord(0x12);           // ma dong bo de phan biet mang
  LoRa.enableCrc();                 // bat kiem tra loi CRC
}

bool readSensor() {
  NhietDo = myDHT.readTemperature(); // doc du lieu nhiet do tu cam bien
  DoAm = myDHT.readHumidity(); // doc du lieu do am tu cam bien
  
  if (isnan(NhietDo) || isnan(DoAm)) {
    Serial.println("Loi cam bien");
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
    String dataPacket = "T" + String(NhietDo, 1) + "H" + String(100, 1);
    
    Serial.print("Dang gui: ");
    Serial.println(dataPacket);
    
    LoRa.beginPacket();
    LoRa.print(dataPacket);
    LoRa.endPacket();
  } else {
    // Send error message
    Serial.println("Dang gui thong bao loi");
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
  if (NhietDo > 33) {
    digitalWrite(LedAlarm, HIGH);
    delay(30);
    digitalWrite(LedAlarm, LOW);
    delay(500);
  }
  delay(100);  // Small delay to prevent CPU overload
}