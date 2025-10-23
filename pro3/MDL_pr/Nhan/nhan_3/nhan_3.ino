#include "Arduino.h"
#include "LoRa.h"
#include <LiquidCrystal_I2C.h>
#include "Wire.h"

// Pin definitions
#define NSS 5
#define RST 14
#define DIO0 26
#define MISO 19
#define SCK 18
#define MOSI 23

// LED and buzzer pins
#define buz 15

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Connection monitoring
unsigned long lastReceived = 0;
const unsigned long connectionTimeout = 10000;  // 10 seconds timeout
bool isConnected = false;

void setup() {
  Serial.begin(9600);
  delay(1000);


  pinMode(buz, OUTPUT);


  digitalWrite(buz, LOW);

  // Initialize LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Nhom 13-Xin Chao!");
  delay(2000);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Khoi dong lora...");

  // Initialize LoRa
  LoRa.setPins(NSS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Khoi tao lora loi!");
    lcd.setCursor(0, 1);
    lcd.print("Loi Lora!");
    while (1);
  }

  // cau hinh lora
  LoRa.setTxPower(14);              // cong suat phat
  LoRa.setSpreadingFactor(7);       // he so trai pho
  LoRa.setSignalBandwidth(125E3);   // bang thong
  LoRa.setCodingRate4(5);           // 4/5 coding rate - good balance
  LoRa.setPreambleLength(8);        // Standard preamble
  LoRa.setSyncWord(0x12);           // ma dinh danh 
  LoRa.enableCrc();                 // bat kiem tra loi bit

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cho du lieu....");
  Serial.println("San sang nhan!");
}

void displayData(float temperature, float humidity) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Nhiet do: ");
  lcd.print(temperature, 1);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Do Am: ");
  lcd.print(humidity, 1);
  lcd.print("%");
}

// canh bao 
void handleAlerts(float temperature, float humidity) {
  // Turn off all first
  digitalWrite(buz, LOW);
  
  if (temperature > 32) {
    digitalWrite(buz, HIGH);
    delay(2000);
    digitalWrite(buz, LOW);
  }
}

// tach du lieu 
bool parseData(String data, float &temperature, float &humidity) {
  if (!data.startsWith("T")) return false;
  
  int hIndex = data.indexOf('H');
  if (hIndex == -1) return false;

  temperature = data.substring(1, hIndex).toFloat();
  humidity = data.substring(hIndex + 1).toFloat();
  
  return !isnan(temperature) && !isnan(humidity);
}

void showConnectionLost() {
  isConnected = false;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Mat ket noi!");
  lcd.setCursor(0, 1);
  lcd.print("Cho ket noi...");
  
}

void loop() {
  // nhan goi tin 
  int packetSize = LoRa.parsePacket();
  // neu co goi tin truyen den thi doc tung byte vao bien receivedData
  if (packetSize) {
    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }

    Serial.print("Received: ");
    Serial.println(receivedData);
    lastReceived = millis();
    isConnected = true;

    if (receivedData == "ERROR") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Loi cam bien!");
      Serial.println("Loi cam bien ben gui!");
    } else {
      float temperature, humidity;
      if (parseData(receivedData, temperature, humidity)) {
        displayData(temperature, humidity);
        handleAlerts(temperature, humidity);
        
        Serial.print("Nhiet do: ");
        Serial.print(temperature);
        Serial.print("°C, Do am: ");
        Serial.print(humidity);
        Serial.println("%");
      } else {
        Serial.println("Data parsing error");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Loi du lieu!");
      }
    }
  }

  // Check for connection timeout
  if (isConnected && (millis() - lastReceived > connectionTimeout)) {
    Serial.println("Connection timeout");
    showConnectionLost();
  }

  delay(100);
}