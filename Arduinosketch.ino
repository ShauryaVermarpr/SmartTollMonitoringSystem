#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>

// ==================== WI-FI CONFIGURATION ====================
const char* ssid = "shar";         
const char* password = "ramramji";  

WebServer server(80); 

// ==================== HARDWARE CONFIG ====================
// LCD (Address 0x27 or 0x3F)
#define LCD_SDA 15
#define LCD_SCL 14
LiquidCrystal_I2C lcd(0x27, 20, 4); 

// --- LANE 1 PINS ---
const int trigPin1 = 25; const int echoPin1 = 26;
const int START_IR_PIN1 = 16; const int END_IR_PIN1 = 17;
#define SS_PIN1 5
#define RST_PIN1 4
const int SERVO_PIN1 = 13;
Servo gateServo1;

// --- LANE 2 PINS ---
const int trigPin2 = 32; const int echoPin2 = 33;
const int START_IR_PIN2 = 34; const int END_IR_PIN2 = 35;
#define SS_PIN2 21
#define RST_PIN2 22
const int SERVO_PIN2 = 12; // WARNING: Unplug during upload if it fails
Servo gateServo2;

const int BUZZER_PIN = 2; 

// ==================== CALIBRATION (UPDATED) ====================
// Sensor Height: 21cm
const float ROOF_HEIGHT_CM = 21.0;      

// Vehicle Logic:
// CAR:   3cm to 8cm
// TRUCK: 9cm to 20cm

const float MIN_VEHICLE_HEIGHT_CM = 3.0;  // Below 3cm is noise
const float CAR_MAX_HEIGHT_CM = 8.5;      // Cutoff: <= 8.5 is Car, > 8.5 is Truck

const int BALANCE_BLOCK = 1; 
MFRC522::MIFARE_Key key;

MFRC522 mfrc522_1(SS_PIN1, RST_PIN1);
MFRC522 mfrc522_2(SS_PIN2, RST_PIN2);

long TOLL_FARE_CENTS = 500; 
long AUTO_TOP_UP = 1000;    
const unsigned long TAILGATE_WINDOW_MS = 3000; 
const unsigned long GATE_OPEN_DURATION = 5000; 

// ==================== VARIABLES ====================
// EV Flags
bool evMode1 = false;
bool evMode2 = false;

// Lane 1 Variables
int vehiclesInLane1 = 0;
bool startSensorPrevState1 = HIGH;
bool endSensorPrevState1 = HIGH;
unsigned long vehicleExitTime1 = 0;
unsigned long lastVehicleClassification1 = 0;
unsigned long lastRFIDCheck1 = 0;
bool vehicleDetected1 = false;
String currentVehicleType1 = "None";
float currentVehicleHeight1 = 0;
bool gateOpen1 = false;
unsigned long gateOpenTime1 = 0;
float currentFare1 = 0.0; 
String lane1Message = "System Ready";

// Lane 2 Variables
int vehiclesInLane2 = 0;
bool startSensorPrevState2 = HIGH;
bool endSensorPrevState2 = HIGH;
unsigned long vehicleExitTime2 = 0;
unsigned long lastVehicleClassification2 = 0;
unsigned long lastRFIDCheck2 = 0;
bool vehicleDetected2 = false;
String currentVehicleType2 = "None";
float currentVehicleHeight2 = 0;
bool gateOpen2 = false;
unsigned long gateOpenTime2 = 0;
float currentFare2 = 0.0; 
String lane2Message = "System Ready";

bool lcdNeedsUpdate = true; 
bool wifiConnected = false;

// ==================== IMPROVED WIFI SETUP ====================
void connectToWiFi() {
  Serial.println();
  Serial.println("Connecting to WiFi: " + String(ssid));
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CONNECTING TO:");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
    // Show connection progress on LCD
    static int dots = 0;
    lcd.setCursor(0, 2);
    lcd.print("Connecting");
    for (int i = 0; i < dots; i++) {
      lcd.print(".");
    }
    dots = (dots + 1) % 4;
    
    // Timeout after 20 seconds
    if (millis() - startAttemptTime > 20000) {
      Serial.println("\nFailed to connect to WiFi!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WIFI FAILED!");
      lcd.setCursor(0, 1);
      lcd.print("Check Credentials");
      lcd.setCursor(0, 2);
      lcd.print("SSID: " + String(ssid));
      wifiConnected = false;
      return;
    }
  }
  
  wifiConnected = true;
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WIFI CONNECTED!");
  lcd.setCursor(0, 1);
  lcd.print("IP:");
  lcd.setCursor(0, 2);
  lcd.print(WiFi.localIP());
  delay(3000);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("System Starting...");

  // --- I2C & LCD ---
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init(); 
  lcd.backlight(); 
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print("SYSTEM STARTING...");
  delay(1000);

  // --- SERVOS ---
  gateServo1.setPeriodHertz(50); 
  gateServo1.attach(SERVO_PIN1, 500, 2400); 
  gateServo1.write(0); // Force Closed

  gateServo2.setPeriodHertz(50); 
  gateServo2.attach(SERVO_PIN2, 500, 2400); 
  gateServo2.write(0); // Force Closed

  // --- IMPROVED WIFI CONNECTION ---
  connectToWiFi();

  // --- WEB SERVER (Only start if WiFi connected) ---
  if (wifiConnected) {
    server.on("/", HTTP_GET, []() { 
      server.send(200, "text/plain", "Toll Plaza API Online"); 
    });
    server.on("/data", HTTP_GET, handleData); 
    server.on("/set_ev", HTTP_GET, handleEVRequest); 
    server.begin();
    Serial.println("Web Server Started!");
  } else {
    Serial.println("Web Server NOT started - No WiFi connection");
  }

  // --- PINS ---
  pinMode(trigPin1, OUTPUT); pinMode(echoPin1, INPUT);
  pinMode(START_IR_PIN1, INPUT_PULLUP); pinMode(END_IR_PIN1, INPUT_PULLUP);
  
  pinMode(trigPin2, OUTPUT); pinMode(echoPin2, INPUT);
  pinMode(START_IR_PIN2, INPUT_PULLUP); pinMode(END_IR_PIN2, INPUT_PULLUP);
  
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  // --- RFID ---
  SPI.begin();
  mfrc522_1.PCD_Init(); 
  mfrc522_1.PCD_SetAntennaGain(mfrc522_1.RxGain_max);
  mfrc522_2.PCD_Init(); 
  mfrc522_2.PCD_SetAntennaGain(mfrc522_2.RxGain_max);

  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  // Init Sensor States
  startSensorPrevState1 = digitalRead(START_IR_PIN1);
  endSensorPrevState1 = digitalRead(END_IR_PIN1);
  startSensorPrevState2 = digitalRead(START_IR_PIN2);
  endSensorPrevState2 = digitalRead(END_IR_PIN2);

  updateLaneSuggestion();
}

// ==================== GATE LOGIC ====================
void openGate1() { gateServo1.write(90); gateOpen1 = true; gateOpenTime1 = millis(); Serial.println("L1: OPEN"); }
void closeGate1() { gateServo1.write(0); gateOpen1 = false; Serial.println("L1: CLOSED"); }
void openGate2() { gateServo2.write(90); gateOpen2 = true; gateOpenTime2 = millis(); Serial.println("L2: OPEN"); }
void closeGate2() { gateServo2.write(0); gateOpen2 = false; Serial.println("L2: CLOSED"); }

void manageGates() {
  if (!evMode1) {
    if (gateOpen1 && (millis() - gateOpenTime1 > GATE_OPEN_DURATION)) closeGate1();
  }
  if (!evMode2) {
    if (gateOpen2 && (millis() - gateOpenTime2 > GATE_OPEN_DURATION)) closeGate2();
  }
}

// ==================== WEB HANDLERS ====================
void handleEVRequest() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  if (server.hasArg("lane")) {
    String lane = server.arg("lane");
    if (lane == "1") {
       evMode1 = true;
       lane1Message = "!!! EV INCOMING !!!";
       closeGate1(); delay(200); openGate1(); 
    } 
    else if (lane == "2") {
       evMode2 = true;
       lane2Message = "!!! EV INCOMING !!!";
       closeGate2(); delay(200); openGate2(); 
    }
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("!! EV INCOMING !!");
    lcd.setCursor(0, 1); lcd.print(lane == "1" ? " USE LANE 1 " : " USE LANE 2 ");
    lcdNeedsUpdate = false;
  }
  server.send(200, "text/plain", "EV Mode Activated");
}

void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String json = "{";
  json += "\"wifi_connected\":" + String(wifiConnected ? "true" : "false") + ",";
  json += "\"ip_address\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"ev_mode_l1\":" + String(evMode1 ? "true" : "false") + ",";
  json += "\"ev_mode_l2\":" + String(evMode2 ? "true" : "false") + ",";
  
  json += "\"l1_count\":" + String(vehiclesInLane1) + ",";
  json += "\"l1_type\":\"" + currentVehicleType1 + "\",";
  json += "\"l1_fare\":" + String(currentFare1) + ",";
  json += "\"l1_gate\":" + String(gateOpen1 ? "true" : "false") + ",";
  json += "\"l1_msg\":\"" + lane1Message + "\",";

  json += "\"l2_count\":" + String(vehiclesInLane2) + ",";
  json += "\"l2_type\":\"" + currentVehicleType2 + "\",";
  json += "\"l2_fare\":" + String(currentFare2) + ",";
  json += "\"l2_gate\":" + String(gateOpen2 ? "true" : "false") + ",";
  json += "\"l2_msg\":\"" + lane2Message + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// ==================== LCD & LOGIC ====================
void updateLaneSuggestion() {
  if (evMode1 || evMode2) return; 

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("L1:"); lcd.print(vehiclesInLane1);
  lcd.setCursor(11, 0); lcd.print("L2:"); lcd.print(vehiclesInLane2);
  
  // Show WiFi status on LCD
  if (wifiConnected) {
    lcd.setCursor(0, 3); 
    lcd.print("WIFI:OK");
  } else {
    lcd.setCursor(0, 3); 
    lcd.print("WIFI:OFF");
  }

  lcd.setCursor(0, 1);
  if (vehiclesInLane1 == 0 && vehiclesInLane2 == 0) lcd.print(" BOTH LANES EMPTY ");
  else if (vehiclesInLane1 == vehiclesInLane2) lcd.print(" BOTH LANES EQUAL ");
  else if (vehiclesInLane1 < vehiclesInLane2) lcd.print(" >> GO TO LANE 1 << ");
  else lcd.print(" >> GO TO LANE 2 << ");
  
  lcdNeedsUpdate = false;
}

void updateFareDisplay1(String type) {
  if (type == "CAR") currentFare1 = 5.00; 
  else if (type == "TRUCK") currentFare1 = 10.00; 
  else currentFare1 = 0.00;
}
void updateFareDisplay2(String type) {
  if (type == "CAR") currentFare2 = 5.00; 
  else if (type == "TRUCK") currentFare2 = 10.00; 
  else currentFare2 = 0.00;
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.0343 / 2;
}

// UPDATED CLASSIFICATION (NO VAN)
String classifyVehicle(float height) {
  if (height < MIN_VEHICLE_HEIGHT_CM) return "No Vehicle";
  else if (height <= CAR_MAX_HEIGHT_CM) return "CAR"; // <= 8.5cm
  else return "TRUCK"; // > 8.5cm
}

void performVehicleClassification1() {
  if (millis() - lastVehicleClassification1 < 1500) return;
  float d = getDistance(trigPin1, echoPin1); if (d < 0) return;
  float h = ROOF_HEIGHT_CM - d; String t = classifyVehicle(h);
  if (t != "No Vehicle") {
    currentVehicleHeight1 = h; currentVehicleType1 = t; vehicleDetected1 = true;
    updateFareDisplay1(t); 
    lane1Message = "Detected: " + t;
    Serial.println("L1: " + lane1Message); 
    if (t == "CAR") TOLL_FARE_CENTS = 500; else TOLL_FARE_CENTS = 1000;
  }
  lastVehicleClassification1 = millis();
}

void performVehicleClassification2() {
  if (millis() - lastVehicleClassification2 < 1500) return;
  float d = getDistance(trigPin2, echoPin2); if (d < 0) return;
  float h = ROOF_HEIGHT_CM - d; String t = classifyVehicle(h);
  if (t != "No Vehicle") {
    currentVehicleHeight2 = h; currentVehicleType2 = t; vehicleDetected2 = true;
    updateFareDisplay2(t); 
    lane2Message = "Detected: " + t;
    Serial.println("L2: " + lane2Message); 
    if (t == "CAR") TOLL_FARE_CENTS = 500; else TOLL_FARE_CENTS = 1000;
  }
  lastVehicleClassification2 = millis();
}

// ==================== LANE MONITORING ====================
void monitorLane1() {
  bool startState = digitalRead(START_IR_PIN1);
  bool endState = digitalRead(END_IR_PIN1);

  if (startSensorPrevState1 == HIGH && startState == LOW) {
    vehiclesInLane1++; lcdNeedsUpdate = true; vehicleDetected1 = false; 
    lane1Message = "Entered Lane 1";
  }
  if (endSensorPrevState1 == HIGH && endState == LOW) {
    if (evMode1) {
       evMode1 = false; closeGate1(); lane1Message = "EV Passed. Reset."; lcdNeedsUpdate = true;
    } 
    else if ((millis() - vehicleExitTime1 < TAILGATE_WINDOW_MS) && (vehicleExitTime1 != 0)) { 
      soundAlarm(); lane1Message = "!!! TAILGATING !!!";
    } else {
      lane1Message = "Exited";
    }
    if (vehiclesInLane1 > 0) { vehiclesInLane1--; vehicleExitTime1 = millis(); lcdNeedsUpdate = true; }
    vehicleDetected1 = false; currentVehicleType1 = "None"; currentFare1 = 0.00;
  }
  startSensorPrevState1 = startState; endSensorPrevState1 = endState;
}

void monitorLane2() {
  bool startState = digitalRead(START_IR_PIN2);
  bool endState = digitalRead(END_IR_PIN2);

  if (startSensorPrevState2 == HIGH && startState == LOW) {
    vehiclesInLane2++; lcdNeedsUpdate = true; vehicleDetected2 = false; 
    lane2Message = "Entered Lane 2";
  }
  if (endSensorPrevState2 == HIGH && endState == LOW) {
    if (evMode2) {
       evMode2 = false; closeGate2(); lane2Message = "EV Passed. Reset."; lcdNeedsUpdate = true;
    }
    else if ((millis() - vehicleExitTime2 < TAILGATE_WINDOW_MS) && (vehicleExitTime2 != 0)) { 
      soundAlarm(); lane2Message = "!!! TAILGATING !!!";
    } else {
      lane2Message = "Exited";
    }
    if (vehiclesInLane2 > 0) { vehiclesInLane2--; vehicleExitTime2 = millis(); lcdNeedsUpdate = true; }
    vehicleDetected2 = false; currentVehicleType2 = "None"; currentFare2 = 0.00;
  }
  startSensorPrevState2 = startState; endSensorPrevState2 = endState;
}

// ==================== RFID LOGIC ====================
void soundAlarm() { for(int i=0;i<3;i++){digitalWrite(BUZZER_PIN,HIGH);delay(100);digitalWrite(BUZZER_PIN,LOW);delay(100);}}
void soundSuccessBeep() { for(int i=0;i<2;i++){digitalWrite(BUZZER_PIN,HIGH);delay(200);digitalWrite(BUZZER_PIN,LOW);delay(100);}}
long bytesToLong(byte *data) { return (long)data[0] | (long)data[1] << 8 | (long)data[2] << 16 | (long)data[3] << 24; }
void longToBytes(long value, byte *data) { data[0] = (byte)(value & 0xFF); data[1] = (byte)((value >> 8) & 0xFF); data[2] = (byte)((value >> 16) & 0xFF); data[3] = (byte)((value >> 24) & 0xFF); }

bool processRFIDTransaction(MFRC522 &mfrc522, int laneNumber) {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) return false;

  if ((laneNumber == 1 && evMode1) || (laneNumber == 2 && evMode2)) {
    Serial.println("Lane " + String(laneNumber) + ": Payment Skipped (EV Mode)");
    return false;
  }

  MFRC522::StatusCode status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, BALANCE_BLOCK, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) { mfrc522.PICC_HaltA(); return false; }

  byte buffer[18]; byte size = sizeof(buffer);
  status = mfrc522.MIFARE_Read(BALANCE_BLOCK, buffer, &size);
  if (status != MFRC522::STATUS_OK) { mfrc522.PICC_HaltA(); return false; }

  long currentBalance = bytesToLong(buffer);
  if (currentBalance > 10000000 || currentBalance < 0) currentBalance = 0; 
  
  long fareToDeduct = 0;
  // UPDATED FARE LOGIC (NO VAN)
  if(laneNumber == 1) { 
    if(currentVehicleType1 == "CAR") fareToDeduct = 500; else fareToDeduct = 1000; 
  } else { 
    if(currentVehicleType2 == "CAR") fareToDeduct = 500; else fareToDeduct = 1000; 
  }

  if (currentBalance < fareToDeduct) {
    long newBalance = AUTO_TOP_UP;
    byte writeData[16]; longToBytes(newBalance, writeData); for(int i=4;i<16;i++) writeData[i]=0;
    mfrc522.MIFARE_Write(BALANCE_BLOCK, writeData, 16);
    currentBalance = newBalance;
  }

  long newBalance = currentBalance - fareToDeduct;
  byte writeData[16]; longToBytes(newBalance, writeData); for(int i=4;i<16;i++) writeData[i]=0;
  status = mfrc522.MIFARE_Write(BALANCE_BLOCK, writeData, 16);
  
  if (status == MFRC522::STATUS_OK) {
    String msg = "Paid. Bal: " + String(newBalance/100.0);
    if (laneNumber == 1) { openGate1(); lane1Message = msg; } 
    else { openGate2(); lane2Message = msg; }
    soundSuccessBeep(); mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1(); return true;
  }
  mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1(); return false;
}

void loop() {
  // Handle web server only if WiFi is connected
  if (wifiConnected) {
    server.handleClient(); 
  }
  
  performVehicleClassification1(); 
  performVehicleClassification2();
  monitorLane1(); 
  monitorLane2();

  if (millis() - lastRFIDCheck1 > 50) { 
    processRFIDTransaction(mfrc522_1, 1); 
    lastRFIDCheck1 = millis(); 
  }
  if (millis() - lastRFIDCheck2 > 50) { 
    processRFIDTransaction(mfrc522_2, 2); 
    lastRFIDCheck2 = millis(); 
  }
  
  manageGates();
  if (lcdNeedsUpdate) updateLaneSuggestion();
  delay(10); 
}
