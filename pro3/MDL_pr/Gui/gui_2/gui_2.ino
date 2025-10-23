// Transmitter Code - Enhanced with improved reliability
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

// DHT sensor configuration
#define DHTtype DHT11
#define DHTpins 13
DHT myDHT(DHTpins, DHTtype);

// Variables for sensor data
float NhietDo, DoAm;

// Timing variables
unsigned long ThoiGianGuiCuoi = 0;
unsigned long ThoiGianXacNhanCuoi = 0;
const unsigned long ThoiGianNghi = 5000;        // Send data every 5 seconds
const unsigned long ThoiGianChoKetNoi = 6000;   // Wait for connection response
const unsigned long KiemTraKetNoi = 25000;      // Time to consider connection lost
const unsigned long GuiLaiData = 3000;          // Retry sending data if no ACK

// Connection state
bool TrangThaiKetNoi = false;
bool DangChoXacNhan = false;
unsigned long ThoiGianGuiYeuCauKetNoi = 0;
int SoLanThuGuiKetNoi = 0;
const int MAX_RETRIES = 5;
String GuiDuLieuHienTai = "";
byte packetID = 0;  // To identify packets

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  //pinMode(DHTpins, INPUT_PULLUP);
  // Initialize random seed for backoff
  randomSeed(analogRead(0));

  Serial.println("Bat dau khoi dong");
  myDHT.begin();

  Serial.println("Bat dau khoi dong lora");
  LoRa.setPins(NSS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Khoi dong loi");
    while (1);
  }
  Serial.println("Khoi dong thanh cong!");

  // Configure LoRa with more reliable settings
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(12);        // Balance between range and speed
  LoRa.setSignalBandwidth(62.5E3);    // Standard bandwidth
  LoRa.setCodingRate4(8);            // 4/5 coding rate - good error correction
  LoRa.setPreambleLength(8);        // Increased for better detection
  LoRa.setSyncWord(0x12);            // Network identifier
  LoRa.enableCrc();                   // Add CRC check for reliability

  // Initial connection request
  YeuCauKetNoi();
}

void YeuCauKetNoi() {
  if (SoLanThuGuiKetNoi >= MAX_RETRIES) {
    // Implement exponential backoff after multiple failed attempts
    delay(random(1000, 5000 * (SoLanThuGuiKetNoi - MAX_RETRIES + 1)));
  } else {
    // Add small random delay to avoid collisions
    delay(random(100, 500));
  }
  
  LoRa.beginPacket();
  LoRa.write(0xC0);                // Connection header byte
  LoRa.print("CONN");              // Connection request
  LoRa.endPacket();
  
  Serial.print("Gui yeu cau ket noi (lan thu ");
  Serial.print(SoLanThuGuiKetNoi + 1);
  Serial.println(")");
  
  DangChoXacNhan = true;
  ThoiGianGuiYeuCauKetNoi = millis();
  SoLanThuGuiKetNoi++;
}

bool DocCamBien() {
  // Read sensor with retry mechanism
  for (int i = 0; i < 3; i++) {  // Try up to 3 times
    NhietDo = myDHT.readTemperature();
    delay(50);  // Short delay between readings
    DoAm = myDHT.readHumidity();
    
    if (!isnan(NhietDo) && !isnan(DoAm)) {
      // Validate humidity range
      if (DoAm < 0) DoAm = 0;
      if (DoAm > 100) DoAm = 100;
      return true;
    }
    delay(500);  // Wait before retry
  }
  return false;  // Failed after all retries
}

void GuiDuLieu() {
  if (DocCamBien()) {
    // Format: "Txx.xxHyy.yy:id" - Added packet ID for tracking
    GuiDuLieuHienTai = "T" + String(NhietDo, 2) + "H" + String(DoAm, 2) + ":" + String(packetID);
    
    Serial.print("Dang gui du lieu: ");
    Serial.println(GuiDuLieuHienTai);
    
    LoRa.beginPacket();
    LoRa.write(0xDA);            // Data header byte
    LoRa.print(GuiDuLieuHienTai);
    LoRa.endPacket();
    
    packetID = (packetID + 1) % 256;  // Increment packet ID with rollover
  } else {
    // Sensor error
    LoRa.beginPacket();
    LoRa.write(0xE0);           // Error header byte
    LoRa.print("ERROR:DHT");
    LoRa.endPacket();
    Serial.println("Cam bien loi");
  }
  
  ThoiGianGuiCuoi = millis();
  DangChoXacNhan = true;
}

void loop() {
  unsigned long now = millis();
  
  // Check for incoming packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    byte header = LoRa.read();  // Read header byte
    String receivedData = "";
    
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    
    Serial.print("Da nhan duoc [");
    Serial.print(header, HEX);
    Serial.print("]: ");
    Serial.println(receivedData);

    // Check for ACK packets with proper header and content
    if (header == 0xAC && receivedData == "OK") {
      TrangThaiKetNoi = true;
      DangChoXacNhan = false;
      ThoiGianXacNhanCuoi = now;
      SoLanThuGuiKetNoi = 0;  // Reset retry counter on success
      
      if (!TrangThaiKetNoi) {
        Serial.println("Da ket noi thanh cong!");
        // Send first data with delay to avoid collision
        delay(random(500, 1000));
        GuiDuLieu();
      } else {
        Serial.println("Xac nhan du lieu thanh cong");
      }
    }
    else if (header == 0xAC && receivedData == "RETRY") {
      // Receiver requests a retransmission
      Serial.println("Nhan duoc yeu cau gui lai");
      delay(random(200, 500));  // Small random delay before resending
      // Resend last packet
      LoRa.beginPacket();
      LoRa.write(0xDA);         // Data header
      LoRa.print(GuiDuLieuHienTai);
      LoRa.endPacket();
      
      ThoiGianGuiCuoi = now;
    }
  }
  
  // Connection state machine
  if (TrangThaiKetNoi) {
    // Connected state
    
    // Regular data transmission
    if ((now - ThoiGianGuiCuoi >= ThoiGianNghi) && !DangChoXacNhan) {
      GuiDuLieu();
    }
    
    // If waiting for ACK and timeout occurred, resend data
    if (DangChoXacNhan && (now - ThoiGianGuiCuoi >= GuiLaiData)) {
      Serial.println("Khong nhan duoc xac nhan, gui lai...");
      
      // Resend last packet
      LoRa.beginPacket();
      LoRa.write(0xDA);  // Data header
      LoRa.print(GuiDuLieuHienTai);
      LoRa.endPacket();
      
      ThoiGianGuiCuoi = now;
    }
    
    // Check if connection is lost (no ACK for too long)
    if (now - ThoiGianXacNhanCuoi >= KiemTraKetNoi) {
      Serial.println("Mat ket noi, thu ket noi lai...");
      TrangThaiKetNoi = false;
      DangChoXacNhan = false;
      SoLanThuGuiKetNoi = 0;
      YeuCauKetNoi();
    }
  } 
  else {
    // Not connected state
    
    // If waiting for connection response and timed out
    if (DangChoXacNhan && (now - ThoiGianGuiYeuCauKetNoi >= ThoiGianChoKetNoi)) {
      DangChoXacNhan = false;
      Serial.println("Khong nhan duoc phan hoi ket noi");
      
      // Try reconnecting with increasing delays to avoid network congestion
      if (SoLanThuGuiKetNoi < MAX_RETRIES * 2) {
        YeuCauKetNoi();
      } else {
        Serial.println("Doi thoi gian dai hon truoc khi thu lai...");
        delay(random(5000, 15000));  // Longer wait time after multiple failures
        SoLanThuGuiKetNoi = 0;  // Reset retry counter
        YeuCauKetNoi();
      }
    }
  }
  
  // Short delay to prevent CPU hogging
  delay(50);
}