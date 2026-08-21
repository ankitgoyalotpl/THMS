// ============================================================================
// ESP32 SCHNEIDER 66-REG 4G MQTT GATEWAY + RTC SD CARD LOGGER + DI (41/42)
// ============================================================================

#include <Arduino.h>
#include <PPP.h>
#include <WiFi.h>
#include <NetworkClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Wire.h>
#include <esp_task_wdt.h>   // ESP32 Hardware Watchdog Library

// --------------------------------------------------------------------------
// 📌 DIGITAL INPUT PINS CONFIG (GPIO 41 & GPIO 42)
// --------------------------------------------------------------------------
#define DI_PIN_1  41   // Digital Input 1 (GPIO 41)
#define DI_PIN_2  42   // Digital Input 2 (GPIO 42)

// --------------------------------------------------------------------------
// SD CARD & DS3231 RTC CONFIG
// --------------------------------------------------------------------------
#define SD_SCK  12
#define SD_MISO 13
#define SD_MOSI 11
#define SD_CS   21     // 📌 Set your Board's SD CS Pin (e.g. 21 or 10)

#define I2C_SDA 8
#define I2C_SCL 9
#define RTC_I2C_ADDR 0x68
#define WDT_TIMEOUT  60     // 60 Seconds Timeout

const char* logFileName = "/telemetry_log.txt";
bool sd_card_mounted = false;
File currentLogFile;

// --------------------------------------------------------------------------
// MODEM / PPP CONFIG
// --------------------------------------------------------------------------
#define PPP_MODEM_APN        "airtelgprs.com"
#define PPP_MODEM_PIN        NULL              

#define PPP_MODEM_TX_PIN     4
#define PPP_MODEM_RX_PIN     5
#define PPP_MODEM_RST_PIN    3
#define PPP_MODEM_RST_LOW    false   
#define PPP_MODEM_RST_DELAY  200

#define PPP_MODEM_MODEL        PPP_MODEM_BG96

// --------------------------------------------------------------------------
// 📌 4-20mA ANALOG SENSOR PIN
// --------------------------------------------------------------------------
#define ANALOG_4_20MA_PIN   1

int   analog_raw_adc    = 0;
float analog_4_20mA_val = 0.0;
int   last_valid_rssi   = 16;

// --------------------------------------------------------------------------
// MQTT CONFIG
// --------------------------------------------------------------------------
const char* mqtt_broker = "otplai.com";
const int   mqtt_port   = 8883;
const char* mqtt_user   = "oxmo";
const char* mqtt_pass   = "123456789";
const char* device_id   = "OXMO_GW_01";
char        mqtt_topic[64];

// --------------------------------------------------------------------------
// RS485 CONFIG
// --------------------------------------------------------------------------
#define RS485_RX_PIN  18
#define RS485_TX_PIN  17
#define METER_ID      1
#define MODBUS_BAUD   9600
HardwareSerial RS485Serial(1);

NetworkClientSecure secureClient;
PubSubClient        mqttClient(secureClient);

volatile bool ppp_got_ip = false;
bool is_meter_data_available = false;

// DATA STRUCTS
struct EnergyData {
  float import_kWh, export_kWh, totalActive_kWh, netActive_kWh;
  float reactiveDeliv_kVARh, reactiveRecv_kVARh, totalReactive_kVARh, netReactive_kVARh;
  float apparentDeliv_kVAh, apparentRecv_kVAh, totalApparent_kVAh, netApparent_kVAh;
};

struct InstantaneousData {
  float currentA, currentB, currentC, neutralCurrent, groundCurrent, avgCurrent;
  float currentUnbalanceA, currentUnbalanceB, currentUnbalanceC, worstCurrentUnbalance;
  float voltageAB, voltageBC, voltageCA, avgLineVoltage;
  float voltageAN, voltageBN, voltageCN, voltageNG, avgPhaseVoltage;
  float voltageUnbalanceAB, voltageUnbalanceBC, voltageUnbalanceCA, worstVoltageUnbalanceLL;
  float voltageUnbalanceAN, voltageUnbalanceBN, voltageUnbalanceCN, worstVoltageUnbalanceLN;
  float activePowerA, activePowerB, activePowerC, totalActivePower;
  float reactivePowerA, reactivePowerB, reactivePowerC, totalReactivePower;
  float apparentPowerA, apparentPowerB, apparentPowerC, totalApparentPower;
  float powerFactorA, powerFactorB, powerFactorC, totalPowerFactor;
  float frequency;
};

struct THDData {
  float thdCurrentA, thdCurrentB, thdCurrentC;
  float thdVoltageAB, thdVoltageBC, thdVoltageCA;
  float thdVoltageAN, thdVoltageBN, thdVoltageCN;
};

EnergyData        sec1;
InstantaneousData sec2;
THDData           sec3;

// 🆔 Read True Hardware Factory MAC Address from ESP32 eFuse
String getESP32HardwareMAC() {
  uint64_t chipid = ESP.getEfuseMac();
  char macStr[13];
  snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X",
           (uint8_t)(chipid >> 40),
           (uint8_t)(chipid >> 32),
           (uint8_t)(chipid >> 24),
           (uint8_t)(chipid >> 16),
           (uint8_t)(chipid >> 8),
           (uint8_t)chipid);
  return String(macStr);
}

// 🕒 RTC HELPER
String getFormattedTime() {
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return "[RTC ERR]";

    Wire.requestFrom(RTC_I2C_ADDR, 7);
    if (Wire.available() < 7) return "[RTC TIMEOUT]";

    int second = (Wire.read() & 0x7F); second = (second / 16 * 10) + (second % 16);
    int minute = Wire.read(); minute = (minute / 16 * 10) + (minute % 16);
    int hour   = (Wire.read() & 0x3F); hour = (hour / 16 * 10) + (hour % 16);
    Wire.read();
    int day    = Wire.read(); day = (day / 16 * 10) + (day % 16);
    int month  = (Wire.read() & 0x1F); month = (month / 16 * 10) + (month % 16);
    int year   = Wire.read(); year = (year / 16 * 10) + (year % 16) + 2000;

    char buffer[30];
    sprintf(buffer, "[%04d-%02d-%02d %02d:%02d:%02d]", year, month, day, hour, minute, second);
    return String(buffer);
}

byte decToBcd(byte val) {
    return ((val / 10 * 16) + (val % 10));
}

// 💾 DUAL LOGGING HELPER (Serial + SD Card simultaneously)
void logPrint(String text) {
  Serial.print(text);
  if (sd_card_mounted && currentLogFile) {
    currentLogFile.print(text);
  }
}

void logPrintln(String text) {
  Serial.println(text);
  if (sd_card_mounted && currentLogFile) {
    currentLogFile.println(text);
  }
}

void setDS3231Time(int year, int month, int day, int hour, int minute, int second) {
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(0x00);
    Wire.write(decToBcd(second));
    Wire.write(decToBcd(minute));
    Wire.write(decToBcd(hour));
    Wire.write(decToBcd(1));
    Wire.write(decToBcd(day));
    Wire.write(decToBcd(month));
    Wire.write(decToBcd(year - 2000));
    Wire.endTransmission();
    Serial.printf("\n🕒 [RTC] Time successfully set to: %04d-%02d-%02d %02d:%02d:%02d\n\n", year, month, day, hour, minute, second);
}

// 💾 SD LOG HELPER
void logDataToSD(String logMsg) {
  if (!sd_card_mounted) return;
  
  File logFile = SD.open(logFileName, FILE_APPEND);
  if (logFile) {
    String formattedLine = getFormattedTime() + " " + logMsg;
    logFile.println(formattedLine);
    logFile.close();
  }
}

void read4to20mASensor() {
  analog_raw_adc = analogRead(ANALOG_4_20MA_PIN);
  float voltage     = (analog_raw_adc / 4095.0) * 3.3;
  analog_4_20mA_val = (voltage / 150.0) * 1000.0;
}

uint16_t modbusCRC(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}
void clearRX() { while (RS485Serial.available()) RS485Serial.read(); }

bool readHoldingRegistersSchneider(uint8_t slave, uint16_t startRegister, uint16_t quantity, uint16_t *outRegs) {
  if (quantity == 0 || quantity > 120) return false;
  uint16_t address = startRegister - 1;
  uint8_t request[8];
  request[0] = slave; request[1] = 0x03;
  request[2] = highByte(address); request[3] = lowByte(address);
  request[4] = highByte(quantity); request[5] = lowByte(quantity);
  uint16_t crc = modbusCRC(request, 6);
  request[6] = lowByte(crc); request[7] = highByte(crc);

  clearRX();
  RS485Serial.write(request, sizeof(request));
  RS485Serial.flush();
  delay(5);

  const uint16_t expectedLength = 5 + quantity * 2;
  uint8_t response[255];
  uint16_t received = 0;
  uint32_t t0 = millis();
  while ((millis() - t0) < 1000 && received < expectedLength) {
    while (RS485Serial.available() && received < sizeof(response)) {
      response[received++] = RS485Serial.read();
    }
  }
  if (received < 5 || response[0] != slave || (response[1] & 0x80) || response[1] != 0x03) return false;
  uint16_t byteCount = response[2];
  if (byteCount != quantity * 2) return false;
  uint16_t frameLength = 3 + byteCount + 2;
  if (received < frameLength) return false;
  uint16_t rxCRC = (uint16_t)response[frameLength - 2] | ((uint16_t)response[frameLength - 1] << 8);
  if (rxCRC != modbusCRC(response, frameLength - 2)) return false;
  for (uint16_t i = 0; i < quantity; i++) {
    outRegs[i] = ((uint16_t)response[3 + i * 2] << 8) | response[4 + i * 2];
  }
  return true;
}

float makeFloat32FromRegs(const uint16_t *r) {
  uint32_t u = ((uint32_t)r[0] << 16) | r[1];
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

bool readSchneiderMeterData() {
  uint16_t buffer[120];

  bool s1_ok = readHoldingRegistersSchneider(METER_ID, 2700, 24, buffer);
  if (s1_ok) {
    sec1.import_kWh          = makeFloat32FromRegs(&buffer[2700 - 2700]);
    sec1.export_kWh          = makeFloat32FromRegs(&buffer[2702 - 2700]);
    sec1.totalActive_kWh     = makeFloat32FromRegs(&buffer[2704 - 2700]);
    sec1.netActive_kWh       = makeFloat32FromRegs(&buffer[2706 - 2700]);
    sec1.reactiveDeliv_kVARh = makeFloat32FromRegs(&buffer[2708 - 2700]);
    sec1.reactiveRecv_kVARh  = makeFloat32FromRegs(&buffer[2710 - 2700]);
    sec1.totalReactive_kVARh = makeFloat32FromRegs(&buffer[2712 - 2700]);
    sec1.netReactive_kVARh   = makeFloat32FromRegs(&buffer[2714 - 2700]);
    sec1.apparentDeliv_kVAh  = makeFloat32FromRegs(&buffer[2716 - 2700]);
    sec1.apparentRecv_kVAh   = makeFloat32FromRegs(&buffer[2718 - 2700]);
    sec1.totalApparent_kVAh  = makeFloat32FromRegs(&buffer[2720 - 2700]);
    sec1.netApparent_kVAh    = makeFloat32FromRegs(&buffer[2722 - 2700]);
  }
  delay(50);

  bool s2_ok = readHoldingRegistersSchneider(METER_ID, 3000, 112, buffer);
  if (s2_ok) {
    sec2.currentA              = makeFloat32FromRegs(&buffer[3000 - 3000]);
    sec2.currentB              = makeFloat32FromRegs(&buffer[3002 - 3000]);
    sec2.currentC              = makeFloat32FromRegs(&buffer[3004 - 3000]);
    sec2.neutralCurrent        = makeFloat32FromRegs(&buffer[3006 - 3000]);
    sec2.groundCurrent         = makeFloat32FromRegs(&buffer[3008 - 3000]);
    sec2.avgCurrent            = makeFloat32FromRegs(&buffer[3010 - 3000]);
    sec2.currentUnbalanceA     = makeFloat32FromRegs(&buffer[3012 - 3000]);
    sec2.currentUnbalanceB     = makeFloat32FromRegs(&buffer[3014 - 3000]);
    sec2.currentUnbalanceC     = makeFloat32FromRegs(&buffer[3016 - 3000]);
    sec2.worstCurrentUnbalance = makeFloat32FromRegs(&buffer[3018 - 3000]);

    sec2.voltageAB             = makeFloat32FromRegs(&buffer[3020 - 3000]);
    sec2.voltageBC             = makeFloat32FromRegs(&buffer[3022 - 3000]);
    sec2.voltageCA             = makeFloat32FromRegs(&buffer[3024 - 3000]);
    sec2.avgLineVoltage        = makeFloat32FromRegs(&buffer[3026 - 3000]);

    sec2.voltageAN             = makeFloat32FromRegs(&buffer[3028 - 3000]);
    sec2.voltageBN             = makeFloat32FromRegs(&buffer[3030 - 3000]);
    sec2.voltageCN             = makeFloat32FromRegs(&buffer[3032 - 3000]);
    sec2.voltageNG             = makeFloat32FromRegs(&buffer[3034 - 3000]);
    sec2.avgPhaseVoltage       = makeFloat32FromRegs(&buffer[3036 - 3000]);

    sec2.voltageUnbalanceAB     = makeFloat32FromRegs(&buffer[3038 - 3000]);
    sec2.voltageUnbalanceBC     = makeFloat32FromRegs(&buffer[3040 - 3000]);
    sec2.voltageUnbalanceCA     = makeFloat32FromRegs(&buffer[3042 - 3000]);
    sec2.worstVoltageUnbalanceLL= makeFloat32FromRegs(&buffer[3044 - 3000]);
    sec2.voltageUnbalanceAN     = makeFloat32FromRegs(&buffer[3046 - 3000]);
    sec2.voltageUnbalanceBN     = makeFloat32FromRegs(&buffer[3048 - 3000]);
    sec2.voltageUnbalanceCN     = makeFloat32FromRegs(&buffer[3050 - 3000]);
    sec2.worstVoltageUnbalanceLN= makeFloat32FromRegs(&buffer[3052 - 3000]);

    sec2.activePowerA          = makeFloat32FromRegs(&buffer[3054 - 3000]);
    sec2.activePowerB          = makeFloat32FromRegs(&buffer[3056 - 3000]);
    sec2.activePowerC          = makeFloat32FromRegs(&buffer[3058 - 3000]);
    sec2.totalActivePower       = makeFloat32FromRegs(&buffer[3060 - 3000]);

    sec2.reactivePowerA        = makeFloat32FromRegs(&buffer[3062 - 3000]);
    sec2.reactivePowerB        = makeFloat32FromRegs(&buffer[3064 - 3000]);
    sec2.reactivePowerC        = makeFloat32FromRegs(&buffer[3066 - 3000]);
    sec2.totalReactivePower     = makeFloat32FromRegs(&buffer[3068 - 3000]);

    sec2.apparentPowerA        = makeFloat32FromRegs(&buffer[3070 - 3000]);
    sec2.apparentPowerB        = makeFloat32FromRegs(&buffer[3072 - 3000]);
    sec2.apparentPowerC        = makeFloat32FromRegs(&buffer[3074 - 3000]);
    sec2.totalApparentPower     = makeFloat32FromRegs(&buffer[3076 - 3000]);

    sec2.powerFactorA          = makeFloat32FromRegs(&buffer[3078 - 3000]);
    sec2.powerFactorB          = makeFloat32FromRegs(&buffer[3080 - 3000]);
    sec2.powerFactorC          = makeFloat32FromRegs(&buffer[3082 - 3000]);
    sec2.totalPowerFactor       = makeFloat32FromRegs(&buffer[3084 - 3000]);

    sec2.frequency              = makeFloat32FromRegs(&buffer[3110 - 3000]);
  }
  delay(50);

  bool s3_ok = readHoldingRegistersSchneider(METER_ID, 21300, 36, buffer);
  if (s3_ok) {
    sec3.thdCurrentA  = makeFloat32FromRegs(&buffer[21300 - 21300]);
    sec3.thdCurrentB  = makeFloat32FromRegs(&buffer[21302 - 21300]);
    sec3.thdCurrentC  = makeFloat32FromRegs(&buffer[21304 - 21300]);
    sec3.thdVoltageAB = makeFloat32FromRegs(&buffer[21322 - 21300]);
    sec3.thdVoltageBC = makeFloat32FromRegs(&buffer[21324 - 21300]);
    sec3.thdVoltageCA = makeFloat32FromRegs(&buffer[21326 - 21300]);
    sec3.thdVoltageAN = makeFloat32FromRegs(&buffer[21330 - 21300]);
    sec3.thdVoltageBN = makeFloat32FromRegs(&buffer[21332 - 21300]);
    sec3.thdVoltageCN = makeFloat32FromRegs(&buffer[21334 - 21300]);
  }

  is_meter_data_available = (s1_ok && s2_ok && s3_ok);
  return is_meter_data_available;
}

void onPPPEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_PPP_START:
      Serial.println("[PPP] Started");
      break;
    case ARDUINO_EVENT_PPP_CONNECTED:
      Serial.println("[PPP] Connected to modem");
      break;
    case ARDUINO_EVENT_PPP_GOT_IP:
      Serial.println("[PPP] GOT IP! ✅");
      Serial.println(PPP.localIP());
      ppp_got_ip = true;
      break;
    case ARDUINO_EVENT_PPP_LOST_IP:
      Serial.println("[PPP] Lost IP");
      ppp_got_ip = false;
      break;
    case ARDUINO_EVENT_PPP_DISCONNECTED:
      Serial.println("[PPP] Disconnected");
      ppp_got_ip = false;
      break;
    default:
      break;
  }
}

void hardwareResetModem() {
  Serial.println("\n🔄 [PPP] Hard Resetting 4G Modem via GPIO 3...");
  
  PPP.end();
  delay(1000);

  pinMode(PPP_MODEM_RST_PIN, OUTPUT);
  digitalWrite(PPP_MODEM_RST_PIN, LOW);   
  delay(500);                             
  digitalWrite(PPP_MODEM_RST_PIN, HIGH);  
  pinMode(PPP_MODEM_RST_PIN, INPUT_PULLUP);
  
  Serial.println("⏳ [4G] Waiting 20 seconds for Modem Bootup & Cellular Registration...");
  uint32_t t0 = millis();
  while (millis() - t0 < 20000) {
    esp_task_wdt_reset();
    delay(500);
  }
  
  ppp_got_ip = false;
}

bool startPPP() {
  Serial.println("[PPP] Configuring modem...");
  PPP.setApn(PPP_MODEM_APN);
  if (PPP_MODEM_PIN) PPP.setPin(PPP_MODEM_PIN);
  PPP.setResetPin(PPP_MODEM_RST_PIN, PPP_MODEM_RST_LOW, PPP_MODEM_RST_DELAY);
  PPP.setPins(PPP_MODEM_TX_PIN, PPP_MODEM_RX_PIN, -1, -1, ESP_MODEM_FLOW_CONTROL_NONE);

  Serial.println("[PPP] Starting modem -- this can take 10-30s...");
  if (!PPP.begin(PPP_MODEM_MODEL, /*uart_num=*/2, /*baud_rate=*/115200)) {
    Serial.println("[PPP] begin() FAILED! Check wiring/power/model.");
    return false;
  }

  Serial.print("[PPP] Manufacturer: "); Serial.println(PPP.moduleName());
  Serial.print("[PPP] IMEI: "); Serial.println(PPP.IMEI());

  Serial.println("[PPP] Waiting for network registration...");
  uint32_t regStart = millis();
  bool attached = false;
  while (millis() - regStart < 60000) {
    esp_task_wdt_reset();
    attached = PPP.attached();
    int rssi = PPP.RSSI();
    if (rssi != -1 && rssi != 0) {
      last_valid_rssi = rssi;
    }
    Serial.print("[PPP] attached="); Serial.print(attached);
    Serial.print(" | RSSI="); Serial.print(rssi);
    Serial.print(" | operator="); Serial.println(PPP.operatorName());
    if (attached) break;
    delay(2000);
  }

  if (!attached) {
    Serial.println("[PPP] Network NEVER attached!");
    return false;
  }
  Serial.println("[PPP] Network ATTACHED. Switching to data mode...");

  if (!PPP.mode(ESP_MODEM_MODE_DATA)) {
    Serial.println("[PPP] mode(DATA) switch FAILED!");
    return false;
  }

  Serial.print("[PPP] Waiting for IP...");
  uint32_t t0 = millis();
  while (!ppp_got_ip && millis() - t0 < 60000) {
    esp_task_wdt_reset();
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  return ppp_got_ip;
}

void reconnectMQTT() {
  secureClient.setInsecure(); 
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setBufferSize(2048);

  Serial.print("[MQTT] Connecting to "); Serial.print(mqtt_broker); Serial.println(" ...");
  if (mqttClient.connect(device_id, mqtt_user, mqtt_pass)) {
    Serial.println("[MQTT] CONNECTED! ✅");
  } else {
    Serial.print("[MQTT] FAILED, state="); Serial.println(mqttClient.state());
  }
}

void printAllDataToSerial() {
  int cur_val  = last_valid_rssi;
  int gsm_csq  = (cur_val < 0) ? ((cur_val + 113) / 2) : cur_val;
  int gsm_rssi = (cur_val < 0) ? cur_val : (-113 + (2 * gsm_csq));

  // SD Card file open for full scan log
  if (sd_card_mounted) {
    currentLogFile = SD.open(logFileName, FILE_APPEND);
  }

  logPrintln("\n" + getFormattedTime() + " =================================================================");
  logPrint("   SCHNEIDER EM6436H FULL TELEMETRY SCAN (SLAVE ID: 1 | STATUS: ");
  logPrintln(is_meter_data_available ? "ONLINE ✅)" : "OFFLINE ❌)");
  logPrintln("=================================================================");

  logPrint("📶 4G CSQ: "); logPrint(String(gsm_csq));
  logPrint(" (RSSI: "); logPrint(String(gsm_rssi)); logPrint(" dBm)");
  logPrint(" | PPP LINK: "); logPrint(ppp_got_ip ? "UP" : "DOWN");
  logPrint(" | MQTT: "); logPrintln(mqttClient.connected() ? "ONLINE" : "OFFLINE");

  logPrintln("\n🟢 [SECTION 1] ALL 12 ENERGY PARAMETERS (2700 - 2723):");
  logPrint("   1. Import Active Energy (Reg 2700):      "); logPrint(String(sec1.import_kWh, 2)); logPrintln(" kWh");
  logPrint("   2. Export Active Energy (Reg 2702):      "); logPrint(String(sec1.export_kWh, 2)); logPrintln(" kWh");
  logPrint("   3. Total Active Energy (Reg 2704):       "); logPrint(String(sec1.totalActive_kWh, 2)); logPrintln(" kWh");
  logPrint("   4. Net Active Energy (Reg 2706):         "); logPrint(String(sec1.netActive_kWh, 2)); logPrintln(" kWh");
  logPrint("   5. Reactive Energy Deliv (Reg 2708):     "); logPrint(String(sec1.reactiveDeliv_kVARh, 2)); logPrintln(" kVARh");
  logPrint("   6. Reactive Energy Recv (Reg 2710):      "); logPrint(String(sec1.reactiveRecv_kVARh, 2)); logPrintln(" kVARh");
  logPrint("   7. Total Reactive Energy (Reg 2712):     "); logPrint(String(sec1.totalReactive_kVARh, 2)); logPrintln(" kVARh");
  logPrint("   8. Net Reactive Energy (Reg 2714):       "); logPrint(String(sec1.netReactive_kVARh, 2)); logPrintln(" kVARh");
  logPrint("   9. Apparent Energy Deliv (Reg 2716):     "); logPrint(String(sec1.apparentDeliv_kVAh, 2)); logPrintln(" kVAh");
  logPrint("  10. Apparent Energy Recv (Reg 2718):      "); logPrint(String(sec1.apparentRecv_kVAh, 2)); logPrintln(" kVAh");
  logPrint("  11. Total Apparent Energy (Reg 2720):     "); logPrint(String(sec1.totalApparent_kVAh, 2)); logPrintln(" kVAh");
  logPrint("  12. Net Apparent Energy (Reg 2722):       "); logPrint(String(sec1.netApparent_kVAh, 2)); logPrintln(" kVAh");

  logPrintln("\n🟢 [SECTION 2] ALL 45 INSTANTANEOUS PARAMETERS (3000 - 3111):");
  logPrintln("  --- CURRENTS ---");
  logPrint("  13. Current A (Reg 3000):                 "); logPrint(String(sec2.currentA, 2)); logPrintln(" A");
  logPrint("  14. Current B (Reg 3002):                 "); logPrint(String(sec2.currentB, 2)); logPrintln(" A");
  logPrint("  15. Current C (Reg 3004):                 "); logPrint(String(sec2.currentC, 2)); logPrintln(" A");
  logPrint("  16. Neutral Current (Reg 3006):           "); logPrint(String(sec2.neutralCurrent, 2)); logPrintln(" A");
  logPrint("  17. Ground Current (Reg 3008):            "); logPrint(String(sec2.groundCurrent, 2)); logPrintln(" A");
  logPrint("  18. Average Current (Reg 3010):           "); logPrint(String(sec2.avgCurrent, 2)); logPrintln(" A");
  logPrint("  19. Current Unbalance A (Reg 3012):       "); logPrint(String(sec2.currentUnbalanceA, 1)); logPrintln(" %");
  logPrint("  20. Current Unbalance B (Reg 3014):       "); logPrint(String(sec2.currentUnbalanceB, 1)); logPrintln(" %");
  logPrint("  21. Current Unbalance C (Reg 3016):       "); logPrint(String(sec2.currentUnbalanceC, 1)); logPrintln(" %");
  logPrint("  22. Worst Current Unbalance (Reg 3018):   "); logPrint(String(sec2.worstCurrentUnbalance, 1)); logPrintln(" %");

  logPrintln("  --- LINE VOLTAGES ---");
  logPrint("  23. Voltage AB (Reg 3020):                "); logPrint(String(sec2.voltageAB, 1)); logPrintln(" V");
  logPrint("  24. Voltage BC (Reg 3022):                "); logPrint(String(sec2.voltageBC, 1)); logPrintln(" V");
  logPrint("  25. Voltage CA (Reg 3024):                "); logPrint(String(sec2.voltageCA, 1)); logPrintln(" V");
  logPrint("  26. Average Line Voltage (Reg 3026):      "); logPrint(String(sec2.avgLineVoltage, 1)); logPrintln(" V");

  logPrintln("  --- PHASE VOLTAGES ---");
  logPrint("  27. Voltage AN (Reg 3028):                "); logPrint(String(sec2.voltageAN, 1)); logPrintln(" V");
  logPrint("  28. Voltage BN (Reg 3030):                "); logPrint(String(sec2.voltageBN, 1)); logPrintln(" V");
  logPrint("  29. Voltage CN (Reg 3032):                "); logPrint(String(sec2.voltageCN, 1)); logPrintln(" V");
  logPrint("  30. Voltage NG (Reg 3034):                "); logPrint(String(sec2.voltageNG, 1)); logPrintln(" V");
  logPrint("  31. Average Phase Voltage (Reg 3036):     "); logPrint(String(sec2.avgPhaseVoltage, 1)); logPrintln(" V");

  logPrintln("  --- VOLTAGE UNBALANCES ---");
  logPrint("  32. Voltage Unbalance AB (Reg 3038):      "); logPrint(String(sec2.voltageUnbalanceAB, 1)); logPrintln(" %");
  logPrint("  33. Voltage Unbalance BC (Reg 3040):      "); logPrint(String(sec2.voltageUnbalanceBC, 1)); logPrintln(" %");
  logPrint("  34. Voltage Unbalance CA (Reg 3042):      "); logPrint(String(sec2.voltageUnbalanceCA, 1)); logPrintln(" %");
  logPrint("  35. Worst Voltage Unbalance LL(Reg 3044): "); logPrint(String(sec2.worstVoltageUnbalanceLL, 1)); logPrintln(" %");
  logPrint("  36. Voltage Unbalance AN (Reg 3046):      "); logPrint(String(sec2.voltageUnbalanceAN, 1)); logPrintln(" %");
  logPrint("  37. Voltage Unbalance BN (Reg 3048):      "); logPrint(String(sec2.voltageUnbalanceBN, 1)); logPrintln(" %");
  logPrint("  38. Voltage Unbalance CN (Reg 3050):      "); logPrint(String(sec2.voltageUnbalanceCN, 1)); logPrintln(" %");
  logPrint("  39. Worst Voltage Unbalance LN(Reg 3052): "); logPrint(String(sec2.worstVoltageUnbalanceLN, 1)); logPrintln(" %");

  logPrintln("  --- POWERS ---");
  logPrint("  40. Active Power A (Reg 3054):            "); logPrint(String(sec2.activePowerA, 2)); logPrintln(" kW");
  logPrint("  41. Active Power B (Reg 3056):            "); logPrint(String(sec2.activePowerB, 2)); logPrintln(" kW");
  logPrint("  42. Active Power C (Reg 3058):            "); logPrint(String(sec2.activePowerC, 2)); logPrintln(" kW");
  logPrint("  43. Total Active Power (Reg 3060):        "); logPrint(String(sec2.totalActivePower, 2)); logPrintln(" kW");

  logPrint("  44. Reactive Power A (Reg 3062):          "); logPrint(String(sec2.reactivePowerA, 2)); logPrintln(" kVAR");
  logPrint("  45. Reactive Power B (Reg 3064):          "); logPrint(String(sec2.reactivePowerB, 2)); logPrintln(" kVAR");
  logPrint("  46. Reactive Power C (Reg 3066):          "); logPrint(String(sec2.reactivePowerC, 2)); logPrintln(" kVAR");
  logPrint("  47. Total Reactive Power (Reg 3068):      "); logPrint(String(sec2.totalReactivePower, 2)); logPrintln(" kVAR");

  logPrint("  48. Apparent Power A (Reg 3070):          "); logPrint(String(sec2.apparentPowerA, 2)); logPrintln(" kVA");
  logPrint("  49. Apparent Power B (Reg 3072):          "); logPrint(String(sec2.apparentPowerB, 2)); logPrintln(" kVA");
  logPrint("  50. Apparent Power C (Reg 3074):          "); logPrint(String(sec2.apparentPowerC, 2)); logPrintln(" kVA");
  logPrint("  51. Total Apparent Power (Reg 3076):      "); logPrint(String(sec2.totalApparentPower, 2)); logPrintln(" kVA");

  logPrintln("  --- POWER FACTOR & FREQUENCY ---");
  logPrint("  52. Power Factor A (Reg 3078):            "); logPrintln(String(sec2.powerFactorA, 2));
  logPrint("  53. Power Factor B (Reg 3080):            "); logPrintln(String(sec2.powerFactorB, 2));
  logPrint("  54. Power Factor C (Reg 3082):            "); logPrintln(String(sec2.powerFactorC, 2));
  logPrint("  55. Total Power Factor (Reg 3084):        "); logPrintln(String(sec2.totalPowerFactor, 2));
  logPrint("  56. Frequency (Reg 3110):                 "); logPrint(String(sec2.frequency, 2)); logPrintln(" Hz");

  logPrintln("\n🟢 [SECTION 3] ALL 9 THD HARMONICS PARAMETERS (21300 - 21335):");
  logPrint("  57. THD Current A (Reg 21300):            "); logPrint(String(sec3.thdCurrentA, 2)); logPrintln(" %");
  logPrint("  58. THD Current B (Reg 21302):            "); logPrint(String(sec3.thdCurrentB, 2)); logPrintln(" %");
  logPrint("  59. THD Current C (Reg 21304):            "); logPrint(String(sec3.thdCurrentC, 2)); logPrintln(" %");

  logPrint("  60. THD Voltage AB (Reg 21322):           "); logPrint(String(sec3.thdVoltageAB, 2)); logPrintln(" %");
  logPrint("  61. THD Voltage BC (Reg 21324):           "); logPrint(String(sec3.thdVoltageBC, 2)); logPrintln(" %");
  logPrint("  62. THD Voltage CA (Reg 21326):           "); logPrint(String(sec3.thdVoltageCA, 2)); logPrintln(" %");

  logPrint("  63. THD Voltage AN (Reg 21330):           "); logPrint(String(sec3.thdVoltageAN, 2)); logPrintln(" %");
  logPrint("  64. THD Voltage BN (Reg 21332):           "); logPrint(String(sec3.thdVoltageBN, 2)); logPrintln(" %");
  logPrint("  65. THD Voltage CN (Reg 21334):           "); logPrint(String(sec3.thdVoltageCN, 2)); logPrintln(" %");

  logPrintln("\n🟢 [SECTION 4] 4-20mA ANALOG SENSOR & DIGITAL INPUTS:");
  logPrint("  66. Raw Board ADC Value:                  "); logPrintln(String(analog_raw_adc));
  logPrint("      4-20mA Calculated Value:             "); logPrint(String(analog_4_20mA_val, 2)); logPrintln(" mA");
  logPrint("  67. DI 1 Status (GPIO 41):                "); logPrintln(digitalRead(DI_PIN_1) ? "HIGH (OPEN)" : "LOW (CLOSED)");
  logPrint("  68. DI 2 Status (GPIO 42):                "); logPrintln(digitalRead(DI_PIN_2) ? "HIGH (OPEN)" : "LOW (CLOSED)");
  logPrintln("=================================================================\n");

  if (sd_card_mounted && currentLogFile) {
    currentLogFile.close();
  }
}

void publishMQTTTelemetry() {
  DynamicJsonDocument doc(2048);

  int cur_val  = last_valid_rssi;
  int gsm_csq  = (cur_val < 0) ? ((cur_val + 113) / 2) : cur_val;
  int gsm_rssi = (cur_val < 0) ? cur_val : (-113 + (2 * gsm_csq));

  JsonObject sysObj = doc.createNestedObject("sys");
  sysObj["ppp_ip"]         = ppp_got_ip;
  sysObj["mqtt_connected"] = mqttClient.connected();
  sysObj["data_available"] = true;               
  sysObj["rssi"]           = gsm_rssi;           
  sysObj["csq"]            = gsm_csq;            
  sysObj["free_heap"]      = ESP.getFreeHeap();

  // 📌 DIGITAL INPUT STATES (GPIO 41 & GPIO 42)
  JsonObject digitalObj = doc.createNestedObject("digital_input");
  digitalObj["di1_gpio41"] = digitalRead(DI_PIN_1);
  digitalObj["di2_gpio42"] = digitalRead(DI_PIN_2);

  JsonObject analogObj = doc.createNestedObject("analog");
  analogObj["raw_adc"] = analog_raw_adc;
  analogObj["mA_val"]  = analog_4_20mA_val;

  JsonObject energyObj = doc.createNestedObject("energy");
  energyObj["imp_kwh"]      = sec1.import_kWh;
  energyObj["exp_kwh"]      = sec1.export_kWh;
  energyObj["tot_kwh"]      = sec1.totalActive_kWh;
  energyObj["net_kwh"]      = sec1.netActive_kWh;
  energyObj["rec_del_kvarh"] = sec1.reactiveDeliv_kVARh;
  energyObj["rec_rec_kvarh"] = sec1.reactiveRecv_kVARh;
  energyObj["tot_kvarh"]    = sec1.totalReactive_kVARh;
  energyObj["net_kvarh"]    = sec1.netReactive_kVARh;
  energyObj["app_del_kvah"]  = sec1.apparentDeliv_kVAh;
  energyObj["app_rec_kvah"]  = sec1.apparentRecv_kVAh;
  energyObj["tot_kvah"]     = sec1.totalApparent_kVAh;
  energyObj["net_kvah"]     = sec1.netApparent_kVAh;

  JsonObject elecObj = doc.createNestedObject("electrical");
  elecObj["iA"]       = sec2.currentA;
  elecObj["iB"]       = sec2.currentB;
  elecObj["iC"]       = sec2.currentC;
  elecObj["iN"]       = sec2.neutralCurrent;
  elecObj["iG"]       = sec2.groundCurrent;
  elecObj["iAvg"]     = sec2.avgCurrent;
  elecObj["iUnbA"]    = sec2.currentUnbalanceA;
  elecObj["iUnbB"]    = sec2.currentUnbalanceB;
  elecObj["iUnbC"]    = sec2.currentUnbalanceC;
  elecObj["iUnbMax"]  = sec2.worstCurrentUnbalance;

  elecObj["vAB"]      = sec2.voltageAB;
  elecObj["vBC"]      = sec2.voltageBC;
  elecObj["vCA"]      = sec2.voltageCA;
  elecObj["vLineAvg"] = sec2.avgLineVoltage;

  elecObj["vAN"]      = sec2.voltageAN;
  elecObj["vBN"]      = sec2.voltageBN;
  elecObj["vCN"]      = sec2.voltageCN;
  elecObj["vNG"]      = sec2.voltageNG;
  elecObj["vPhsAvg"]  = sec2.avgPhaseVoltage;

  elecObj["vUnbAB"]   = sec2.voltageUnbalanceAB;
  elecObj["vUnbBC"]   = sec2.voltageUnbalanceBC;
  elecObj["vUnbCA"]   = sec2.voltageUnbalanceCA;
  elecObj["vUnbLLMax"]= sec2.worstVoltageUnbalanceLL;
  elecObj["vUnbAN"]   = sec2.voltageUnbalanceAN;
  elecObj["vUnbBN"]   = sec2.voltageUnbalanceBN;
  elecObj["vUnbCN"]   = sec2.voltageUnbalanceCN;
  elecObj["vUnbLNMax"]= sec2.worstVoltageUnbalanceLN;

  elecObj["kwA"]      = sec2.activePowerA;
  elecObj["kwB"]      = sec2.activePowerB;
  elecObj["kwC"]      = sec2.activePowerC;
  elecObj["kwTot"]    = sec2.totalActivePower;

  elecObj["kvarA"]    = sec2.reactivePowerA;
  elecObj["kvarB"]    = sec2.reactivePowerB;
  elecObj["kvarC"]    = sec2.reactivePowerC;
  elecObj["kvarTot"]  = sec2.totalReactivePower;

  elecObj["kvaA"]     = sec2.apparentPowerA;
  elecObj["kvaB"]     = sec2.apparentPowerB;
  elecObj["kvaC"]     = sec2.apparentPowerC;
  elecObj["kvaTot"]   = sec2.totalApparentPower;

  elecObj["pfA"]      = sec2.powerFactorA;
  elecObj["pfB"]      = sec2.powerFactorB;
  elecObj["pfC"]      = sec2.powerFactorC;
  elecObj["pfTot"]    = sec2.totalPowerFactor;

  elecObj["freq"]     = sec2.frequency;

  JsonObject thdObj = doc.createNestedObject("thd");
  thdObj["thd_iA"]  = sec3.thdCurrentA;
  thdObj["thd_iB"]  = sec3.thdCurrentB;
  thdObj["thd_iC"]  = sec3.thdCurrentC;
  thdObj["thd_vAB"] = sec3.thdVoltageAB;
  thdObj["thd_vBC"] = sec3.thdVoltageBC;
  thdObj["thd_vCA"] = sec3.thdVoltageCA;
  thdObj["thd_vAN"] = sec3.thdVoltageAN;
  thdObj["thd_vBN"] = sec3.thdVoltageBN;
  thdObj["thd_vCN"] = sec3.thdVoltageCN;

  char jsonBuffer[2048];
  size_t n = serializeJson(doc, jsonBuffer);

  Serial.print("[MQTT] Publishing "); Serial.print(n); Serial.print(" bytes to "); Serial.println(mqtt_topic);
  if (mqttClient.publish(mqtt_topic, jsonBuffer)) {
    Serial.println("✅ Publish OK");
  } else {
    Serial.println("❌ Publish FAILED");
  }

  // 💾 SAVE EXACT MQTT JSON PAYLOAD TO SD CARD LOGS
  if (sd_card_mounted) {
    File logFile = SD.open(logFileName, FILE_APPEND);
    if (logFile) {
      logFile.print(getFormattedTime() + " [MQTT JSON PAYLOAD]: ");
      logFile.println(jsonBuffer);
      logFile.close();
    }
  }
}

// --------------------------------------------------------------------------
// SETUP / LOOP
// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // 📌 1. CONFIGURE DIGITAL INPUT PINS (GPIO 41 & GPIO 42)
  pinMode(DI_PIN_1, INPUT_PULLUP);
  pinMode(DI_PIN_2, INPUT_PULLUP);

  // 📌 RECONFIGURE WDT TO 60 SECONDS SAFELY
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = (1 << configNUM_CORES) - 1,
      .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&twdt_config);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_add(NULL);
#endif
  Serial.println("🐕 [WDT] Hardware Watchdog Reconfigured to 60s Safely!");

  analogReadResolution(12);

  // 📌 2. DYNAMIC MQTT TOPIC: transformer/<MACADDRESS>/rx
  String mac = getESP32HardwareMAC();
  mac.replace(":", "");
  snprintf(mqtt_topic, sizeof(mqtt_topic), "transformer/%s/rx", mac.c_str());

  Serial.println("\n========================================================");
  Serial.print("   TARGET MQTT TOPIC: "); Serial.println(mqtt_topic);
  Serial.println("   ESP32 SCHNEIDER GATEWAY -- TRUE PPP + RTC SD + DI(41/42) ");
  Serial.println("========================================================");

  // 🕒 Start I2C & SPI for RTC + SD Card
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  Serial.println("[SD] Mounting SD Card...");
  if (SD.begin(SD_CS)) {
    sd_card_mounted = true;
    Serial.println("[SD] SD Card Mounted Successfully! ✅");
    logDataToSD("=== OFFLINE GATEWAY TELEMETRY LOG SESSION STARTED ===");
  } else {
    Serial.println("[SD] SD Card Mount Failed! ❌");
  }

  RS485Serial.begin(MODBUS_BAUD, SERIAL_8E1, RS485_RX_PIN, RS485_TX_PIN);

  Network.onEvent(onPPPEvent);

  if (!startPPP()) {
    Serial.println("[PPP] Initial connect FAILED -- will retry in loop().");
  } else {
    reconnectMQTT();
  }
}

static int network_fail_count = 0;

void loop() {
  esp_task_wdt_reset(); // Feed Watchdog

  // 🕒 SERIAL COMMAND TO SET RTC TIME (e.g. set=2026,08,11,15,40,00)
  if (Serial.available() > 0) {
      String inputStr = Serial.readStringUntil('\n');
      inputStr.trim();
      if (inputStr.startsWith("set=")) {
          inputStr = inputStr.substring(4);
          int values[6];
          int parsed = sscanf(inputStr.c_str(), "%d,%d,%d,%d,%d,%d", 
                              &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]);
          if (parsed == 6) {
              setDS3231Time(values[0], values[1], values[2], values[3], values[4], values[5]);
          } else {
              Serial.println("❌ [RTC ERROR] Invalid format! Use: set=YYYY,MM,DD,hh,mm,ss");
          }
      }
  }

  mqttClient.loop();

  if (!ppp_got_ip && !PPP.attached()) {
    network_fail_count++;
    Serial.print("⚠️ [PPP] Link Down! Retry Attempt: "); 
    Serial.println(network_fail_count);

    if (network_fail_count >= 3) {
      hardwareResetModem();
      network_fail_count = 0;
    }

    startPPP();
  } else if (ppp_got_ip) {
    network_fail_count = 0;
  }

  if (ppp_got_ip && !mqttClient.connected()) {
    reconnectMQTT();
  }

  readSchneiderMeterData();
  read4to20mASensor();
  printAllDataToSerial();

  // 💾 SAVE TELEMETRY LOG SUMMARY TO SD CARD WITH RTC TIMESTAMP
  if (sd_card_mounted) {
    String logLine = "kWh:" + String(sec1.totalActive_kWh, 2) + 
                     " | V_AB:" + String(sec2.voltageAB, 1) + 
                     " | I_A:" + String(sec2.currentA, 2) + 
                     " | 4-20mA:" + String(analog_4_20mA_val, 2) + "mA" +
                     " | DI1:" + String(digitalRead(DI_PIN_1)) +
                     " | DI2:" + String(digitalRead(DI_PIN_2));
    logDataToSD(logLine);
  }

  if (mqttClient.connected()) {
    publishMQTTTelemetry();
  }

  Serial.print("[SYS] Free heap: "); Serial.println(ESP.getFreeHeap());

  // 📌 WDT-SAFE 5 SECOND DELAY LOOP
  uint32_t delayStart = millis();
  while (millis() - delayStart < 5000) {
    esp_task_wdt_reset();
    delay(200);
  }
}
