#include "Arduino.h"
#include "LoRa.h"
#include <LiquidCrystal_I2C.h>
#include "Wire.h"

#define NSS 5
#define RST 14
#define DIO0 26
#define MISO 19
#define SCK 18
#define MOSI 23

#define buz 15
#define redled 25
#define greenled 33
#define blueled 32

LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long LanNhanCuoi = 0;
const unsigned long TIMEOUT_KETNOI = 40000;
bool TrangThaiKetNoi = false;

void setup() {
  Serial.begin(9600);
  delay(1000);

  pinMode(buz, OUTPUT);
  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(blueled, OUTPUT);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Nhom 5 - Hello!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Khoi dong LoRa");

  LoRa.setPins(NSS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Loi LoRa");
    lcd.setCursor(0, 1);
    lcd.print("Loi ket noi!");
    while (1);
  }

  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(12);        // Balance between range and speed
  LoRa.setSignalBandwidth(62.5E3);    // Standard bandwidth
  LoRa.setCodingRate4(8);            // 4/5 coding rate - good error correction
  LoRa.setPreambleLength(8);        // Increased for better detection
  LoRa.setSyncWord(0x12);            // Network identifier
  LoRa.enableCrc(); 

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dang cho ket noi");
}

void GuiPhanHoi(const String &msg) {
  delay(random(100, 300));  // Avoid collision
  LoRa.beginPacket();
  // Add a header byte for reliable packet identification
  LoRa.write(0xAC);  // ACK header byte
  LoRa.print(msg);
  LoRa.endPacket();
  Serial.print("Gui phan hoi: ");
  Serial.println(msg);
}

void HienThiDuLieu(float t, float h) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Nhiet do: ");
  lcd.print(t, 1);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Do am: ");
  lcd.print(h, 1);
  lcd.print("%");
}

void CanhBao(float t, float h) 
{
  if (t < 20) {
    digitalWrite(redled, 1);
    digitalWrite(greenled, 1);
    digitalWrite(blueled, 0);
  }
  else if (t > 20 && t < 45) {
    digitalWrite(redled, 1);
    digitalWrite(greenled, 0);
    digitalWrite(blueled, 1); 
  }
  else if (t > 45) {
    digitalWrite(buz, 1);
    digitalWrite(redled, 0);
    digitalWrite(greenled, 1);
    digitalWrite(blueled, 1);
  }
}

bool TachDuLieu(String data, float &t, float &h, int &id) {
  if (!data.startsWith("T")) return false;
  int hIndex = data.indexOf('H');
  int idIndex = data.indexOf(':');
  if (hIndex == -1 || idIndex == -1 || hIndex > idIndex) return false;

  t = data.substring(1, hIndex).toFloat();
  h = data.substring(hIndex + 1, idIndex).toFloat();
  id = data.substring(idIndex + 1).toInt();
  return !isnan(t) && !isnan(h);
}

void MatKetNoi() {
  TrangThaiKetNoi = false;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Mat ket noi!");
  lcd.setCursor(0, 1);
  lcd.print("Dang cho ket noi");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    byte header = LoRa.read();
    String data = "";
    while (LoRa.available()) {
      data += (char)LoRa.read();
    }

    Serial.print("Nhan [");
    Serial.print(header, HEX);
    Serial.print("]: ");
    Serial.println(data);
    LanNhanCuoi = millis();

    if (header == 0xC0 && data == "CONN") {
      TrangThaiKetNoi = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Da ket noi!");
      GuiPhanHoi("OK");  // 
    } 
    else if (header == 0xDA) {
      float nhietdo, doam;
      int id;
      if (TachDuLieu(data, nhietdo, doam, id)) {
        TrangThaiKetNoi = true;
        HienThiDuLieu(nhietdo, doam);
        GuiPhanHoi("OK");  // Changed from "OOK" to "OK"
        CanhBao(nhietdo, doam);
      } else {
        Serial.println("Loi du lieu, yeu cau gui lai");
        GuiPhanHoi("RETRY");
      }
    } 
    else if (header == 0xE0) {
      TrangThaiKetNoi = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Cam bien loi");
      GuiPhanHoi("OK");  // Added acknowledgment for error reports
    }
  }

  if (TrangThaiKetNoi && (millis() - LanNhanCuoi > TIMEOUT_KETNOI)) {
    Serial.println("Mat ket noi do timeout");
    MatKetNoi();
    //GuiPhanHoi("RETRY");
  }

  delay(50);
}