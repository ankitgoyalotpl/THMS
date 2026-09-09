// ============================================================================
// ESP32 SCHNEIDER 66-REG + TPR-702 TEMP 4G MQTT GATEWAY + RTC SD CARD LOGGER
// ============================================================================

#include <Arduino.h>
#include <PPP.h>
#include <WiFi.h>
#include <WebServer.h>     // 📌 Local Web Server Library
#include <NetworkClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <ETH.h>
#include <Wire.h>
#include <Preferences.h>   // 📌 Flash Storage for Tap Counter
#include <esp_task_wdt.h>   // ESP32 Hardware Watchdog Library



// --------------------------------------------------------------------------
// 📌 ETHERNET (W5500) PINS CONFIG
// --------------------------------------------------------------------------
#define ETH_CS_PIN    6
#define ETH_SCK_PIN   7
#define ETH_MOSI_PIN  15
#define ETH_MISO_PIN  16
#define ETH_INT_PIN   35
SPIClass SPI_ETH(HSPI); // ESP32-S3 ke liye second SPI bus instance
volatile bool eth_got_ip = false; // Ethernet IP flag

// --------------------------------------------------------------------------
// 📌 DIGITAL INPUT PINS CONFIG (GPIO 41 & GPIO 42)
// --------------------------------------------------------------------------
#define DI_PIN_1  41   // Digital Input 1 (GPIO 41)
#define DI_PIN_2  42   // Digital Input 2 (GPIO 42)
#define DI_PIN_3  38   // 👈 Naya: Digital Input 3 (Apni PCB ke hisaab se pin daalein)
#define DI_PIN_4  39   // 👈 Naya: Digital Input 4
#define DI_PIN_5  40   // 👈 Naya: Digital Input 5
#define DI_PIN_6  14    // 👈 Naya: Digital Input 6

// --------------------------------------------------------------------------
// 🚨 TIMERS & ALARM LOGIC
// --------------------------------------------------------------------------
uint32_t last_meter_read_time = 0;
bool     force_mqtt_publish   = false;
String   current_alarm_cause  = "";
bool     last_di_states[6]    = {false, false, false, false, false, false};


// --------------------------------------------------------------------------
// SD CARD & DS3231 RTC CONFIG
// --------------------------------------------------------------------------
#define SD_SCK  12
#define SD_MISO 13
#define SD_MOSI 11
#define SD_CS   21     // SD CS Pin (Board GPIO 21)

#define I2C_SDA 8
#define I2C_SCL 9
#define RTC_I2C_ADDR 0x68
#define WDT_TIMEOUT  60     // 60 Seconds Watchdog Timeout

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

#define PPP_MODEM_MODEL      PPP_MODEM_BG96

// --------------------------------------------------------------------------
// 📌 4-20mA ANALOG SENSOR PIN
// --------------------------------------------------------------------------
#define ANALOG_4_20MA_PIN   1
#define OLTC_TAP_PIN        10   // 👈 NAYA: Dedicated OLTC Tap & Counter (GPIO 10)

int   analog_raw_adc    = 0;
float analog_4_20mA_val = 0.0;
float oltc_live_voltage = 0.0; // 👈 Pin 10 ka Live Measured Voltage (Volts)
int   last_valid_rssi   = 16;

// 📌 OLTC Structure & Flash Storage
struct OLTCData {
  int      currentTap;       // 1 to 17
  uint32_t tapCounter;       // Total Operations (Permanent Flash)
  bool     sensor_ok;
};

OLTCData    oltcData;
Preferences nvsStorage;
int         lastConfirmedTap  = -1;
int         candidateNewTap   = -1;
uint32_t    tapDebounceTimer  = 0;
char        mqtt_sub_topic[64]; // transformer/<MAC>/tx (Dashboard Commands)


// --------------------------------------------------------------------------
// MQTT CONFIG
// --------------------------------------------------------------------------
String mqtt_broker = "otplai.com";
int    mqtt_port   = 8883;
String mqtt_user   = "oxmo";
String mqtt_pass   = "123456789";
String wifi_ssid = "";
String wifi_pass = "";
const char* device_id   = "OXMO_GW_01";
char        mqtt_topic[64];

uint32_t upload_interval_sec = 60; // 👈 Dashboard se change hoga (Default 60s)
int      alarm_oil_temp      = 80; // 👈 Default threshold
int      alarm_hv_temp       = 90;
int      alarm_lv_temp       = 90;
// --------------------------------------------------------------------------
// RS485 & MODBUS SLAVE CONFIG
// --------------------------------------------------------------------------
#define RS485_RX_PIN    18
#define RS485_TX_PIN    17
// ✅ Ab aisa kar dijiye:
// #define SCHNEIDER_1_ID  1    // Meter 1 (HV / Incomer)
// #define SCHNEIDER_2_ID  3    // 👈 Naya: Meter 2 (LV / Outgoing)
// #define TPR702_ID       2    // TPR-702 Temp Controller
// #define MODBUS_BAUD     9600
uint8_t schneider_1_id = 1;
uint8_t schneider_2_id = 3;
uint8_t tpr702_id      = 2;
uint32_t modbus_baud   = 9600;
String modbus_parity   = "8E1"; // Default 8E1


HardwareSerial RS485Serial(1);

NetworkClientSecure secureClient;
PubSubClient        mqttClient(secureClient);

volatile bool ppp_got_ip = false;
bool is_meter_data_available = false;

// --------------------------------------------------------------------------
// DATA STRUCTS FOR SCHNEIDER METER (66 REGISTERS)
// --------------------------------------------------------------------------
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

// --------------------------------------------------------------------------
// 📌 DATA STRUCT FOR TPR-702 TRANSFORMER TEMPERATURE CONTROLLER
// --------------------------------------------------------------------------
struct TPR702Data {
  uint16_t oilTemp;  // Reg 40001 (Offset 0x0000)
  uint16_t hvTemp;   // Reg 40002 (Offset 0x0001)
  uint16_t lvTemp;   // Reg 40003 (Offset 0x0002)
  bool is_valid;
};

// 📌 METER 1 DATA STRUCTS (Slave ID: 1)
EnergyData        m1_sec1;
InstantaneousData m1_sec2;
THDData           m1_sec3;
bool              m1_online = false;

// 📌 METER 2 DATA STRUCTS (Slave ID: 3 - Naya Meter)
EnergyData        m2_sec1;
InstantaneousData m2_sec2;
THDData           m2_sec3;
bool              m2_online = false;

// TPR-702
TPR702Data        tprData;

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
  // Serial.print(text);
  if (sd_card_mounted && currentLogFile) {
    currentLogFile.print(text);
  }
}

void logPrintln(String text) {
  // Serial.println(text);
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

// --------------------------------------------------------------------------
// 📌 1. READ GENERAL 4-20mA SENSOR (GPIO 1)
// --------------------------------------------------------------------------
void read4to20mASensor() {
  analog_raw_adc = analogRead(ANALOG_4_20MA_PIN);
  float voltage = (analog_raw_adc / 4095.0) * 3.3;
  
  // Standard 4-20mA Calculation (0.44V = 4mA, 3.13V = 20mA)
  if (voltage < 0.25) {
    analog_4_20mA_val = 0.0; // Wire cut / Disconnected
  } else {
    analog_4_20mA_val = 4.0 + ((voltage - 0.44) / 2.69) * 16.0;
    if (analog_4_20mA_val < 0.0)  analog_4_20mA_val = 0.0;
    if (analog_4_20mA_val > 24.0) analog_4_20mA_val = 24.0;
  }
}

// --------------------------------------------------------------------------
// 📌 2. READ DEDICATED OLTC TAP & COUNTER (GPIO 10) - NEAREST MATCH TABLE
// --------------------------------------------------------------------------
static int      stableTapCandidate = -1;
static uint32_t tapCandidateTimer  = 0;

// 📌 17 Taps ke Voltages (Web Portal se modify ho sakte hain aur Flash me save honge):
float TAP_VOLTAGES[17] = {
  0.000, 0.190, 0.380, 0.575, 0.770, 0.960, 1.160, 1.350, 1.540,
  1.740, 1.930, 2.120, 2.320, 2.510, 2.710, 2.910, 3.100
};
// Factory Defaults (Agar Reset karna ho):
const float DEFAULT_TAP_VOLTAGES[17] = {
  0.000, 0.190, 0.380, 0.575, 0.770, 0.960, 1.160, 1.350, 1.540,
  1.740, 1.930, 2.120, 2.320, 2.510, 2.710, 2.910, 3.100
};

// 📌 Function: Jo Tap sabse kareeb hoga, wahi choose karega
int getNearestTap(float v) {
  int bestTap = 1;
  float minDiff = 999.0;
  for (int i = 0; i < 17; i++) {
    float diff = abs(v - TAP_VOLTAGES[i]);
    if (diff < minDiff) {
      minDiff = diff;
      bestTap = i + 1;
    }
  }
  return bestTap;
}

void readOLTCSensor() {
  delay(10);

  // 📌 30 Samples ka Average (ESP32 Hardware Factory Millivolts)
  long sumMv = 0;
  for (int i = 0; i < 30; i++) {
    sumMv += analogReadMilliVolts(OLTC_TAP_PIN); // 👈 Multimeter se 100% matched
    delayMicroseconds(200);
  }
   float voltage     = (sumMv / 30.0) / 1000.0;   // 👈 Exact Volts
  oltc_live_voltage = voltage;                   // 👈 MQTT aur Web ke liye save

  // Debug Serial: Multimeter aur Code ka voltage live dekhein
  // Serial.printf("🔍 [TAP LIVE] Voltage: %.3f V | Nearest: Tap %d | Active: Tap %d | Ops: %u\n", 
  //               voltage, getNearestTap(voltage), oltcData.currentTap, oltcData.tapCounter);

  // Wire Cut / Disconnect check (<0.30V ya >3.35V)
  if (voltage > 3.35) {
    oltcData.sensor_ok = false;
    stableTapCandidate = -1;
    return;
  }
  oltcData.sensor_ok = true;

  // 🎯 Nearest Tap Nikaalein:
  int calculatedTap = getNearestTap(voltage);

  // First Bootup
  if (lastConfirmedTap == -1) {
    lastConfirmedTap    = calculatedTap;
    oltcData.currentTap = calculatedTap;
    return;
  }

  // 🛡️ SYNCED TAP & COUNTER LOGIC (1.5s Stable Hold):
  if (calculatedTap != lastConfirmedTap) {
    if (calculatedTap != stableTapCandidate) {
      stableTapCandidate = calculatedTap;
      tapCandidateTimer  = millis();
    } else if (millis() - tapCandidateTimer >= 1500) { // 1.5 second hold
      int stepsMoved = abs(stableTapCandidate - lastConfirmedTap);
      
      lastConfirmedTap    = stableTapCandidate;
      oltcData.currentTap = stableTapCandidate;

      oltcData.tapCounter += stepsMoved;
      nvsStorage.putUInt("tap_count", oltcData.tapCounter);

      // Serial.printf("\n⚡ [OLTC SYNCED EVENT] New Tap: %d | Steps: +%d | Total Ops: %u\n\n", 
                    // oltcData.currentTap, stepsMoved, oltcData.tapCounter);

      stableTapCandidate = -1;
    }
  } else {
    stableTapCandidate = -1;
  }
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

// --------------------------------------------------------------------------
// READ SCHNEIDER MODBUS HOLDING REGISTERS (Function Code 0x03)
// --------------------------------------------------------------------------
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

// --------------------------------------------------------------------------
// 📌 READ TPR-702 TEMPERATURE CONTROLLER REGISTERS (40001, 40002, 40003)
// --------------------------------------------------------------------------
bool readTPR702(uint8_t slaveId, uint16_t &oilTemp, uint16_t &hvTemp, uint16_t &lvTemp) {
  uint8_t request[8];
  request[0] = slaveId;
  request[1] = 0x03;          // Function Code 03
  request[2] = 0x00;          // Start Address High Byte (0x0000 = Reg 40001)
  request[3] = 0x00;          // Start Address Low Byte
  request[4] = 0x00;          // Quantity High Byte
  request[5] = 0x03;          // Quantity Low Byte (Read 3 Registers)

  uint16_t crc = modbusCRC(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  clearRX();
  RS485Serial.write(request, sizeof(request));
  RS485Serial.flush();

  uint8_t response[11];
  uint8_t receivedBytes = 0;
  uint32_t startTime = millis();

  while ((millis() - startTime) < 1000) {
    while (RS485Serial.available()) {
      if (receivedBytes < sizeof(response)) {
        response[receivedBytes++] = RS485Serial.read();
      } else {
        RS485Serial.read();
      }
    }
    if (receivedBytes >= 11) break;
  }

  if (receivedBytes != 11 || response[0] != slaveId || (response[1] & 0x80) || response[1] != 0x03 || response[2] != 6) {
    return false;
  }

  uint16_t rxCRC = response[9] | ((uint16_t)response[10] << 8);
  if (rxCRC != modbusCRC(response, 9)) return false;

  oilTemp = ((uint16_t)response[3] << 8) | response[4];
  hvTemp  = ((uint16_t)response[5] << 8) | response[6];
  lvTemp  = ((uint16_t)response[7] << 8) | response[8];

  return true;
}

// --------------------------------------------------------------------------
// UNIVERSAL FUNCTION: READ ANY SCHNEIDER METER (66 REGISTERS)
// --------------------------------------------------------------------------
bool readSchneiderMeter(uint8_t slaveId, EnergyData &e, InstantaneousData &inst, THDData &thd) {
  uint16_t buffer[120];

  // 1. SECTION 1: Energy (2700 - 2723)
  bool s1_ok = readHoldingRegistersSchneider(slaveId, 2700, 24, buffer);
  if (s1_ok) {
    e.import_kWh          = makeFloat32FromRegs(&buffer[0]);
    e.export_kWh          = makeFloat32FromRegs(&buffer[2]);
    e.totalActive_kWh     = makeFloat32FromRegs(&buffer[4]);
    e.netActive_kWh       = makeFloat32FromRegs(&buffer[6]);
    e.reactiveDeliv_kVARh = makeFloat32FromRegs(&buffer[8]);
    e.reactiveRecv_kVARh  = makeFloat32FromRegs(&buffer[10]);
    e.totalReactive_kVARh = makeFloat32FromRegs(&buffer[12]);
    e.netReactive_kVARh   = makeFloat32FromRegs(&buffer[14]);
    e.apparentDeliv_kVAh  = makeFloat32FromRegs(&buffer[16]);
    e.apparentRecv_kVAh   = makeFloat32FromRegs(&buffer[18]);
    e.totalApparent_kVAh  = makeFloat32FromRegs(&buffer[20]);
    e.netApparent_kVAh    = makeFloat32FromRegs(&buffer[22]);
  }
  delay(40);

  // 2. SECTION 2: Instantaneous Electrical (3000 - 3111)
  bool s2_ok = readHoldingRegistersSchneider(slaveId, 3000, 112, buffer);
  if (s2_ok) {
    inst.currentA              = makeFloat32FromRegs(&buffer[3000 - 3000]);
    inst.currentB              = makeFloat32FromRegs(&buffer[3002 - 3000]);
    inst.currentC              = makeFloat32FromRegs(&buffer[3004 - 3000]);
    inst.neutralCurrent        = makeFloat32FromRegs(&buffer[3006 - 3000]);
    inst.groundCurrent         = makeFloat32FromRegs(&buffer[3008 - 3000]);
    inst.avgCurrent            = makeFloat32FromRegs(&buffer[3010 - 3000]);
    inst.currentUnbalanceA     = makeFloat32FromRegs(&buffer[3012 - 3000]);
    inst.currentUnbalanceB     = makeFloat32FromRegs(&buffer[3014 - 3000]);
    inst.currentUnbalanceC     = makeFloat32FromRegs(&buffer[3016 - 3000]);
    inst.worstCurrentUnbalance = makeFloat32FromRegs(&buffer[3018 - 3000]);

    inst.voltageAB             = makeFloat32FromRegs(&buffer[3020 - 3000]);
    inst.voltageBC             = makeFloat32FromRegs(&buffer[3022 - 3000]);
    inst.voltageCA             = makeFloat32FromRegs(&buffer[3024 - 3000]);
    inst.avgLineVoltage        = makeFloat32FromRegs(&buffer[3026 - 3000]);

    inst.voltageAN             = makeFloat32FromRegs(&buffer[3028 - 3000]);
    inst.voltageBN             = makeFloat32FromRegs(&buffer[3030 - 3000]);
    inst.voltageCN             = makeFloat32FromRegs(&buffer[3032 - 3000]);
    inst.voltageNG             = makeFloat32FromRegs(&buffer[3034 - 3000]);
    inst.avgPhaseVoltage       = makeFloat32FromRegs(&buffer[3036 - 3000]);

    inst.voltageUnbalanceAB     = makeFloat32FromRegs(&buffer[3038 - 3000]);
    inst.voltageUnbalanceBC     = makeFloat32FromRegs(&buffer[3040 - 3000]);
    inst.voltageUnbalanceCA     = makeFloat32FromRegs(&buffer[3042 - 3000]);
    inst.worstVoltageUnbalanceLL= makeFloat32FromRegs(&buffer[3044 - 3000]);
    inst.voltageUnbalanceAN     = makeFloat32FromRegs(&buffer[3046 - 3000]);
    inst.voltageUnbalanceBN     = makeFloat32FromRegs(&buffer[3048 - 3000]);
    inst.voltageUnbalanceCN     = makeFloat32FromRegs(&buffer[3050 - 3000]);
    inst.worstVoltageUnbalanceLN= makeFloat32FromRegs(&buffer[3052 - 3000]);

    inst.activePowerA          = makeFloat32FromRegs(&buffer[3054 - 3000]);
    inst.activePowerB          = makeFloat32FromRegs(&buffer[3056 - 3000]);
    inst.activePowerC          = makeFloat32FromRegs(&buffer[3058 - 3000]);
    inst.totalActivePower       = makeFloat32FromRegs(&buffer[3060 - 3000]);

    inst.reactivePowerA        = makeFloat32FromRegs(&buffer[3062 - 3000]);
    inst.reactivePowerB        = makeFloat32FromRegs(&buffer[3064 - 3000]);
    inst.reactivePowerC        = makeFloat32FromRegs(&buffer[3066 - 3000]);
    inst.totalReactivePower     = makeFloat32FromRegs(&buffer[3068 - 3000]);

    inst.apparentPowerA        = makeFloat32FromRegs(&buffer[3070 - 3000]);
    inst.apparentPowerB        = makeFloat32FromRegs(&buffer[3072 - 3000]);
    inst.apparentPowerC        = makeFloat32FromRegs(&buffer[3074 - 3000]);
    inst.totalApparentPower     = makeFloat32FromRegs(&buffer[3076 - 3000]);

    inst.powerFactorA          = makeFloat32FromRegs(&buffer[3078 - 3000]);
    inst.powerFactorB          = makeFloat32FromRegs(&buffer[3080 - 3000]);
    inst.powerFactorC          = makeFloat32FromRegs(&buffer[3082 - 3000]);
    inst.totalPowerFactor       = makeFloat32FromRegs(&buffer[3084 - 3000]);

    inst.frequency              = makeFloat32FromRegs(&buffer[3110 - 3000]);
  }
  delay(40);

  // 3. SECTION 3: THD Harmonics (21300 - 21335)
  bool s3_ok = readHoldingRegistersSchneider(slaveId, 21300, 36, buffer);
  if (s3_ok) {
    thd.thdCurrentA  = makeFloat32FromRegs(&buffer[0]);
    thd.thdCurrentB  = makeFloat32FromRegs(&buffer[2]);
    thd.thdCurrentC  = makeFloat32FromRegs(&buffer[4]);
    thd.thdVoltageAB = makeFloat32FromRegs(&buffer[22]);
    thd.thdVoltageBC = makeFloat32FromRegs(&buffer[24]);
    thd.thdVoltageCA = makeFloat32FromRegs(&buffer[26]);
    thd.thdVoltageAN = makeFloat32FromRegs(&buffer[30]);
    thd.thdVoltageBN = makeFloat32FromRegs(&buffer[32]);
    thd.thdVoltageCN = makeFloat32FromRegs(&buffer[34]);
  }

  return (s1_ok && s2_ok && s3_ok);
}


void onNetworkEvent(arduino_event_id_t event, arduino_event_info_t info) {
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
    // 👇 NAYE ETHERNET CASES YAHAN DALO:
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] Started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] Connected to LAN Cable");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[ETH] GOT IP! ✅ IP: ");
      Serial.println(ETH.localIP());
      eth_got_ip = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] Disconnected / Cable Removed");
      eth_got_ip = false;
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
  delay(1000);
  digitalWrite(PPP_MODEM_RST_PIN, LOW);
  
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

// 📌 Dashboard se Counter Set karne ka Command Receiver
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[256];
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);
  msg[length] = '\0';

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (!err) {
    if (doc.containsKey("set_tap_count") || doc["cmd"] == "set_tap_count") {
      uint32_t val = doc.containsKey("set_tap_count") ? doc["set_tap_count"] : doc["val"];
      oltcData.tapCounter = val;
      nvsStorage.putUInt("tap_count", val);
      Serial.printf("✅ [WEB DASHBOARD CMD] Tap Counter Calibrated To: %u\n", val);
    }
  }
}

void reconnectMQTT() {
  secureClient.setInsecure(); 
  mqttClient.setServer(mqtt_broker.c_str(), mqtt_port); // 👈 Updated
  mqttClient.setCallback(mqttCallback);  
  mqttClient.setBufferSize(6144);

  Serial.print("[MQTT] Connecting to "); Serial.print(mqtt_broker); Serial.println(" ...");
  String client_id_unique = "OXMO_" + getESP32HardwareMAC();
  if (mqttClient.connect(client_id_unique.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) { // 👈 Updated
    Serial.println("[MQTT] CONNECTED! ✅");
    mqttClient.subscribe(mqtt_sub_topic); 
  } else {
    Serial.print("[MQTT] FAILED, state="); Serial.println(mqttClient.state());
  }
}


// --------------------------------------------------------------------------
// 🖨️ HELPER: PRINT ALL 66 REGISTERS FOR A METER
// --------------------------------------------------------------------------
void printFullMeter66Registers(const char* meterTitle, uint8_t slaveId, bool isOnline, 
                               EnergyData &sec1, InstantaneousData &sec2, THDData &sec3) {
  logPrintln("\n=================================================================");
  logPrint("   📊 "); logPrint(meterTitle); logPrint(" (SLAVE ID: "); logPrint(String(slaveId));
  logPrintln(isOnline ? " | STATUS: ONLINE ✅)" : " | STATUS: OFFLINE ❌)");
  logPrintln("=================================================================");

  if (!isOnline) {
    logPrintln("  ❌ Modbus Communication Failed! Check RS485 wiring & Slave ID.");
    return;
  }

  // 🟢 SECTION 1: ALL 12 ENERGY PARAMETERS (2700 - 2723)
  logPrintln("\n  🟢 [SECTION 1] ALL 12 ENERGY PARAMETERS (2700 - 2723):");
  logPrint("     1. Import Active Energy (2700):        "); logPrint(String(sec1.import_kWh, 2)); logPrintln(" kWh");
  logPrint("     2. Export Active Energy (2702):        "); logPrint(String(sec1.export_kWh, 2)); logPrintln(" kWh");
  logPrint("     3. Total Active Energy (2704):         "); logPrint(String(sec1.totalActive_kWh, 2)); logPrintln(" kWh");
  logPrint("     4. Net Active Energy (2706):           "); logPrint(String(sec1.netActive_kWh, 2)); logPrintln(" kWh");
  logPrint("     5. Reactive Energy Deliv (2708):       "); logPrint(String(sec1.reactiveDeliv_kVARh, 2)); logPrintln(" kVARh");
  logPrint("     6. Reactive Energy Recv (2710):        "); logPrint(String(sec1.reactiveRecv_kVARh, 2)); logPrintln(" kVARh");
  logPrint("     7. Total Reactive Energy (2712):       "); logPrint(String(sec1.totalReactive_kVARh, 2)); logPrintln(" kVARh");
  logPrint("     8. Net Reactive Energy (2714):         "); logPrint(String(sec1.netReactive_kVARh, 2)); logPrintln(" kVARh");
  logPrint("     9. Apparent Energy Deliv (2716):       "); logPrint(String(sec1.apparentDeliv_kVAh, 2)); logPrintln(" kVAh");
  logPrint("    10. Apparent Energy Recv (2718):        "); logPrint(String(sec1.apparentRecv_kVAh, 2)); logPrintln(" kVAh");
  logPrint("    11. Total Apparent Energy (2720):       "); logPrint(String(sec1.totalApparent_kVAh, 2)); logPrintln(" kVAh");
  logPrint("    12. Net Apparent Energy (2722):         "); logPrint(String(sec1.netApparent_kVAh, 2)); logPrintln(" kVAh");

  // 🟢 SECTION 2: ALL 45 INSTANTANEOUS PARAMETERS (3000 - 3111)
  logPrintln("\n  🟢 [SECTION 2] ALL 45 INSTANTANEOUS PARAMETERS (3000 - 3111):");
  logPrintln("    --- CURRENTS ---");
  logPrint("    13. Current A (3000):                   "); logPrint(String(sec2.currentA, 2)); logPrintln(" A");
  logPrint("    14. Current B (3002):                   "); logPrint(String(sec2.currentB, 2)); logPrintln(" A");
  logPrint("    15. Current C (3004):                   "); logPrint(String(sec2.currentC, 2)); logPrintln(" A");
  logPrint("    16. Neutral Current (3006):             "); logPrint(String(sec2.neutralCurrent, 2)); logPrintln(" A");
  logPrint("    17. Ground Current (3008):              "); logPrint(String(sec2.groundCurrent, 2)); logPrintln(" A");
  logPrint("    18. Average Current (3010):             "); logPrint(String(sec2.avgCurrent, 2)); logPrintln(" A");
  logPrint("    19. Current Unbalance A (3012):         "); logPrint(String(sec2.currentUnbalanceA, 1)); logPrintln(" %");
  logPrint("    20. Current Unbalance B (3014):         "); logPrint(String(sec2.currentUnbalanceB, 1)); logPrintln(" %");
  logPrint("    21. Current Unbalance C (3016):         "); logPrint(String(sec2.currentUnbalanceC, 1)); logPrintln(" %");
  logPrint("    22. Worst Current Unbalance (3018):     "); logPrint(String(sec2.worstCurrentUnbalance, 1)); logPrintln(" %");

  logPrintln("    --- LINE VOLTAGES ---");
  logPrint("    23. Voltage AB (3020):                  "); logPrint(String(sec2.voltageAB, 1)); logPrintln(" V");
  logPrint("    24. Voltage BC (3022):                  "); logPrint(String(sec2.voltageBC, 1)); logPrintln(" V");
  logPrint("    25. Voltage CA (3024):                  "); logPrint(String(sec2.voltageCA, 1)); logPrintln(" V");
  logPrint("    26. Average Line Voltage (3026):        "); logPrint(String(sec2.avgLineVoltage, 1)); logPrintln(" V");

  logPrintln("    --- PHASE VOLTAGES ---");
  logPrint("    27. Voltage AN (3028):                  "); logPrint(String(sec2.voltageAN, 1)); logPrintln(" V");
  logPrint("    28. Voltage BN (3030):                  "); logPrint(String(sec2.voltageBN, 1)); logPrintln(" V");
  logPrint("    29. Voltage CN (3032):                  "); logPrint(String(sec2.voltageCN, 1)); logPrintln(" V");
  logPrint("    30. Voltage NG (3034):                  "); logPrint(String(sec2.voltageNG, 1)); logPrintln(" V");
  logPrint("    31. Average Phase Voltage (3036):       "); logPrint(String(sec2.avgPhaseVoltage, 1)); logPrintln(" V");

  logPrintln("    --- VOLTAGE UNBALANCES ---");
  logPrint("    32. Voltage Unbalance AB (3038):        "); logPrint(String(sec2.voltageUnbalanceAB, 1)); logPrintln(" %");
  logPrint("    33. Voltage Unbalance BC (3040):        "); logPrint(String(sec2.voltageUnbalanceBC, 1)); logPrintln(" %");
  logPrint("    34. Voltage Unbalance CA (3042):        "); logPrint(String(sec2.voltageUnbalanceCA, 1)); logPrintln(" %");
  logPrint("    35. Worst Voltage Unbalance LL (3044):  "); logPrint(String(sec2.worstVoltageUnbalanceLL, 1)); logPrintln(" %");
  logPrint("    36. Voltage Unbalance AN (3046):        "); logPrint(String(sec2.voltageUnbalanceAN, 1)); logPrintln(" %");
  logPrint("    37. Voltage Unbalance BN (3048):        "); logPrint(String(sec2.voltageUnbalanceBN, 1)); logPrintln(" %");
  logPrint("    38. Voltage Unbalance CN (3050):        "); logPrint(String(sec2.voltageUnbalanceCN, 1)); logPrintln(" %");
  logPrint("    39. Worst Voltage Unbalance LN (3052):  "); logPrint(String(sec2.worstVoltageUnbalanceLN, 1)); logPrintln(" %");

  logPrintln("    --- POWERS ---");
  logPrint("    40. Active Power A (3054):              "); logPrint(String(sec2.activePowerA, 2)); logPrintln(" kW");
  logPrint("    41. Active Power B (3056):              "); logPrint(String(sec2.activePowerB, 2)); logPrintln(" kW");
  logPrint("    42. Active Power C (3058):              "); logPrint(String(sec2.activePowerC, 2)); logPrintln(" kW");
  logPrint("    43. Total Active Power (3060):          "); logPrint(String(sec2.totalActivePower, 2)); logPrintln(" kW");

  logPrint("    44. Reactive Power A (3062):            "); logPrint(String(sec2.reactivePowerA, 2)); logPrintln(" kVAR");
  logPrint("    45. Reactive Power B (3064):            "); logPrint(String(sec2.reactivePowerB, 2)); logPrintln(" kVAR");
  logPrint("    46. Reactive Power C (3066):            "); logPrint(String(sec2.reactivePowerC, 2)); logPrintln(" kVAR");
  logPrint("    47. Total Reactive Power (3068):        "); logPrint(String(sec2.totalReactivePower, 2)); logPrintln(" kVAR");

  logPrint("    48. Apparent Power A (3070):            "); logPrint(String(sec2.apparentPowerA, 2)); logPrintln(" kVA");
  logPrint("    49. Apparent Power B (3072):            "); logPrint(String(sec2.apparentPowerB, 2)); logPrintln(" kVA");
  logPrint("    50. Apparent Power C (3074):            "); logPrint(String(sec2.apparentPowerC, 2)); logPrintln(" kVA");
  logPrint("    51. Total Apparent Power (3076):        "); logPrint(String(sec2.totalApparentPower, 2)); logPrintln(" kVA");

  logPrintln("    --- POWER FACTOR & FREQUENCY ---");
  logPrint("    52. Power Factor A (3078):              "); logPrintln(String(sec2.powerFactorA, 2));
  logPrint("    53. Power Factor B (3080):              "); logPrintln(String(sec2.powerFactorB, 2));
  logPrint("    54. Power Factor C (3082):              "); logPrintln(String(sec2.powerFactorC, 2));
  logPrint("    55. Total Power Factor (3084):          "); logPrintln(String(sec2.totalPowerFactor, 2));
  logPrint("    56. Frequency (3110):                   "); logPrint(String(sec2.frequency, 2)); logPrintln(" Hz");

  // 🟢 SECTION 3: ALL 9 THD HARMONICS PARAMETERS (21300 - 21335)
  logPrintln("\n  🟢 [SECTION 3] ALL 9 THD HARMONICS PARAMETERS (21300 - 21335):");
  logPrint("    57. THD Current A (21300):              "); logPrint(String(sec3.thdCurrentA, 2)); logPrintln(" %");
  logPrint("    58. THD Current B (21302):              "); logPrint(String(sec3.thdCurrentB, 2)); logPrintln(" %");
  logPrint("    59. THD Current C (21304):              "); logPrint(String(sec3.thdCurrentC, 2)); logPrintln(" %");
  logPrint("    60. THD Voltage AB (21322):             "); logPrint(String(sec3.thdVoltageAB, 2)); logPrintln(" %");
  logPrint("    61. THD Voltage BC (21324):             "); logPrint(String(sec3.thdVoltageBC, 2)); logPrintln(" %");
  logPrint("    62. THD Voltage CA (21326):             "); logPrint(String(sec3.thdVoltageCA, 2)); logPrintln(" %");
  logPrint("    63. THD Voltage AN (21330):             "); logPrint(String(sec3.thdVoltageAN, 2)); logPrintln(" %");
  logPrint("    64. THD Voltage BN (21332):             "); logPrint(String(sec3.thdVoltageBN, 2)); logPrintln(" %");
  logPrint("    65. THD Voltage CN (21334):             "); logPrint(String(sec3.thdVoltageCN, 2)); logPrintln(" %");
}

void printAllDataToSerial() {
  int cur_val  = last_valid_rssi;
  int gsm_csq  = (cur_val < 0) ? ((cur_val + 113) / 2) : cur_val;
  int gsm_rssi = (cur_val < 0) ? cur_val : (-113 + (2 * gsm_csq));

  if (sd_card_mounted) {
    currentLogFile = SD.open(logFileName, FILE_APPEND);
  }

  logPrintln("\n" + getFormattedTime() + " =================================================================");
  logPrintln("          🚀 COMPLETE SUBSTATION TELEMETRY SCAN");
  logPrintln("=================================================================");
  logPrint("📶 4G CSQ: "); logPrint(String(gsm_csq));
  logPrint(" (RSSI: "); logPrint(String(gsm_rssi)); logPrint(" dBm)");
  logPrint(" | ETH: "); logPrint(eth_got_ip ? "CONNECTED" : "DOWN"); // 👈 Ye add karo
  logPrint(" | PPP: "); logPrint(ppp_got_ip ? "CONNECTED" : "DOWN");
  logPrint(" | MQTT: "); logPrintln(mqttClient.connected() ? "ONLINE" : "OFFLINE");

  // 📌 1. PRINT ALL 66 REGISTERS OF METER 1
  printFullMeter66Registers("SCHNEIDER METER 1 (INCOMER)", schneider_1_id, m1_online, m1_sec1, m1_sec2, m1_sec3);

  // 📌 2. PRINT ALL 66 REGISTERS OF METER 2
  printFullMeter66Registers("SCHNEIDER METER 2 (OUTGOING)", schneider_2_id, m2_online, m2_sec1, m2_sec2, m2_sec3);

  // 📌 3. TPR-702 TEMPERATURES
  logPrintln("\n=================================================================");
  logPrint("   🌡️ TPR-702 TRANSFORMER TEMPERATURE (SLAVE ID: "); logPrint(String(tpr702_id));
  logPrintln(tprData.is_valid ? " | STATUS: ONLINE ✅)" : " | STATUS: OFFLINE ❌)");
  logPrintln("=================================================================");
  if (tprData.is_valid) {
    logPrint("    Oil Temperature:  "); logPrint(String(tprData.oilTemp)); logPrintln(" °C");
    logPrint("    HV Winding Temp:  "); logPrint(String(tprData.hvTemp)); logPrintln(" °C");
    logPrint("    LV Winding Temp:  "); logPrint(String(tprData.lvTemp)); logPrintln(" °C");
  }

   // 📌 4. OLTC & 4-20mA SENSORS
  logPrintln("\n=================================================================");
  logPrintln("   🎛️ OLTC MONITORING & ANALOG SENSORS");
  logPrintln("=================================================================");
  logPrint("    Analog 4-20mA (GPIO 1):  "); logPrint(String(analog_4_20mA_val, 2)); logPrintln(" mA");
  logPrint("    Live Tap (GPIO 10):      Tap "); logPrint(String(oltcData.currentTap)); logPrintln(" / 17");
  logPrint("    Tap Counter:             "); logPrint(String(oltcData.tapCounter)); logPrintln(" Operations");

  // 📌 5. DIGITAL INPUTS (DI 1 to DI 6)
  logPrintln("\n=================================================================");
  logPrintln("   🚪 DIGITAL INPUT STATUS (DI 1 to DI 6)");
  logPrintln("=================================================================");
  logPrint("    DI 1 (41): "); logPrint(digitalRead(DI_PIN_1) ? "HIGH" : "LOW");
  logPrint(" | DI 2 (42): "); logPrint(digitalRead(DI_PIN_2) ? "HIGH" : "LOW");
  logPrint(" | DI 3 (38): "); logPrint(digitalRead(DI_PIN_3) ? "HIGH" : "LOW");
  logPrint(" | DI 4 (39): "); logPrint(digitalRead(DI_PIN_4) ? "HIGH" : "LOW");
  logPrint(" | DI 5 (40): "); logPrint(digitalRead(DI_PIN_5) ? "HIGH" : "LOW");
  logPrint(" | DI 6 (14):  "); logPrintln(digitalRead(DI_PIN_6) ? "HIGH" : "LOW");
  logPrintln("=================================================================\n");

  if (sd_card_mounted && currentLogFile) {
    currentLogFile.close();
  }
}

// --------------------------------------------------------------------------
// 📦 HELPER: PACK ALL 66 REGISTERS INTO JSON FOR A SCHNEIDER METER
// --------------------------------------------------------------------------
void addMeterToJSON(JsonObject &meterObj, bool isOnline, EnergyData &sec1, InstantaneousData &sec2, THDData &sec3) {
  meterObj["online"] = isOnline;

  // 🟢 SECTION 1: ALL 12 ENERGY REGISTERS (2700 - 2723)
  JsonObject energyObj = meterObj.createNestedObject("energy");
  energyObj["imp_kwh"]       = sec1.import_kWh;
  energyObj["exp_kwh"]       = sec1.export_kWh;
  energyObj["tot_kwh"]       = sec1.totalActive_kWh;
  energyObj["net_kwh"]       = sec1.netActive_kWh;
  energyObj["rec_del_kvarh"] = sec1.reactiveDeliv_kVARh;
  energyObj["rec_rec_kvarh"] = sec1.reactiveRecv_kVARh;
  energyObj["tot_kvarh"]     = sec1.totalReactive_kVARh;
  energyObj["net_kvarh"]     = sec1.netReactive_kVARh;
  energyObj["app_del_kvah"]  = sec1.apparentDeliv_kVAh;
  energyObj["app_rec_kvah"]  = sec1.apparentRecv_kVAh;
  energyObj["tot_kvah"]      = sec1.totalApparent_kVAh;
  energyObj["net_kvah"]      = sec1.netApparent_kVAh;

  // 🟢 SECTION 2: ALL 45 INSTANTANEOUS REGISTERS (3000 - 3111)
  JsonObject elecObj = meterObj.createNestedObject("electrical");
  elecObj["iA"]        = sec2.currentA;
  elecObj["iB"]        = sec2.currentB;
  elecObj["iC"]        = sec2.currentC;
  elecObj["iN"]        = sec2.neutralCurrent;
  elecObj["iG"]        = sec2.groundCurrent;
  elecObj["iAvg"]      = sec2.avgCurrent;
  elecObj["iUnbA"]     = sec2.currentUnbalanceA;
  elecObj["iUnbB"]     = sec2.currentUnbalanceB;
  elecObj["iUnbC"]     = sec2.currentUnbalanceC;
  elecObj["iUnbMax"]   = sec2.worstCurrentUnbalance;

  elecObj["vAB"]       = sec2.voltageAB;
  elecObj["vBC"]       = sec2.voltageBC;
  elecObj["vCA"]       = sec2.voltageCA;
  elecObj["vLineAvg"]  = sec2.avgLineVoltage;

  elecObj["vAN"]       = sec2.voltageAN;
  elecObj["vBN"]       = sec2.voltageBN;
  elecObj["vCN"]       = sec2.voltageCN;
  elecObj["vNG"]       = sec2.voltageNG;
  elecObj["vPhsAvg"]   = sec2.avgPhaseVoltage;

  elecObj["vUnbAB"]    = sec2.voltageUnbalanceAB;
  elecObj["vUnbBC"]    = sec2.voltageUnbalanceBC;
  elecObj["vUnbCA"]    = sec2.voltageUnbalanceCA;
  elecObj["vUnbLLMax"] = sec2.worstVoltageUnbalanceLL;
  elecObj["vUnbAN"]    = sec2.voltageUnbalanceAN;
  elecObj["vUnbBN"]    = sec2.voltageUnbalanceBN;
  elecObj["vUnbCN"]    = sec2.voltageUnbalanceCN;
  elecObj["vUnbLNMax"] = sec2.worstVoltageUnbalanceLN;

  elecObj["kwA"]       = sec2.activePowerA;
  elecObj["kwB"]       = sec2.activePowerB;
  elecObj["kwC"]       = sec2.activePowerC;
  elecObj["kwTot"]     = sec2.totalActivePower;

  elecObj["kvarA"]     = sec2.reactivePowerA;
  elecObj["kvarB"]     = sec2.reactivePowerB;
  elecObj["kvarC"]     = sec2.reactivePowerC;
  elecObj["kvarTot"]   = sec2.totalReactivePower;

  elecObj["kvaA"]      = sec2.apparentPowerA;
  elecObj["kvaB"]      = sec2.apparentPowerB;
  elecObj["kvaC"]      = sec2.apparentPowerC;
  elecObj["kvaTot"]    = sec2.totalApparentPower;

  elecObj["pfA"]       = sec2.powerFactorA;
  elecObj["pfB"]       = sec2.powerFactorB;
  elecObj["pfC"]       = sec2.powerFactorC;
  elecObj["pfTot"]     = sec2.totalPowerFactor;

  elecObj["freq"]      = sec2.frequency;

  // 🟢 SECTION 3: ALL 9 THD HARMONICS REGISTERS (21300 - 21335)
  JsonObject thdObj = meterObj.createNestedObject("thd");
  thdObj["thd_iA"]     = sec3.thdCurrentA;
  thdObj["thd_iB"]     = sec3.thdCurrentB;
  thdObj["thd_iC"]     = sec3.thdCurrentC;
  thdObj["thd_vAB"]    = sec3.thdVoltageAB;
  thdObj["thd_vBC"]    = sec3.thdVoltageBC;
  thdObj["thd_vCA"]    = sec3.thdVoltageCA;
  thdObj["thd_vAN"]    = sec3.thdVoltageAN;
  thdObj["thd_vBN"]    = sec3.thdVoltageBN;
  thdObj["thd_vCN"]    = sec3.thdVoltageCN;
}

void publishMQTTTelemetry(String trigger_cause) {
  DynamicJsonDocument doc(6144); 

  int cur_val  = last_valid_rssi;
  int gsm_csq  = (cur_val < 0) ? ((cur_val + 113) / 2) : cur_val;
  int gsm_rssi = (cur_val < 0) ? cur_val : (-113 + (2 * gsm_csq));

  // 1. SYSTEM TELEMETRY
  JsonObject sysObj = doc.createNestedObject("sys");
  sysObj["trigger"]        = trigger_cause; // 👈 NAYA: "DI_1_ALARM" ya "1_MIN_HEARTBEAT"
  sysObj["eth_ip"]         = eth_got_ip;  // 👈 Ye line add karo
  sysObj["ppp_ip"]         = ppp_got_ip;
  sysObj["mqtt_connected"] = mqttClient.connected();
  sysObj["data_available"] = true;               
  sysObj["rssi"]           = gsm_rssi;           
  sysObj["csq"]            = gsm_csq;            
  sysObj["free_heap"]      = ESP.getFreeHeap();
  sysObj["timestamp"]      = getFormattedTime();

  // 2. DIGITAL INPUTS (DI 1 to DI 6)
  JsonObject digitalObj = doc.createNestedObject("digital_input");
  digitalObj["di1"] = digitalRead(DI_PIN_1);
  digitalObj["di2"] = digitalRead(DI_PIN_2);
  digitalObj["di3"] = digitalRead(DI_PIN_3);
  digitalObj["di4"] = digitalRead(DI_PIN_4);
  digitalObj["di5"] = digitalRead(DI_PIN_5);
  digitalObj["di6"] = digitalRead(DI_PIN_6);

  // 3. ANALOG & OLTC
  JsonObject analogObj = doc.createNestedObject("analog");
  analogObj["raw_adc"] = analog_raw_adc;
  analogObj["mA_val"]  = analog_4_20mA_val;

  JsonObject oltcObj = doc.createNestedObject("oltc");
  oltcObj["current_tap"] = oltcData.currentTap;
  oltcObj["tap_voltage"] = oltc_live_voltage; // 👈 Ye 1 line add karein (Live Volts)
  oltcObj["tap_counter"] = oltcData.tapCounter;
  oltcObj["sensor_ok"]   = oltcData.sensor_ok;

  // 4. TPR-702 TEMPERATURES
  JsonObject tprObj = doc.createNestedObject("tpr702");
  tprObj["oil_temp"] = tprData.oilTemp;
  tprObj["hv_temp"]  = tprData.hvTemp;
  tprObj["lv_temp"]  = tprData.lvTemp;
  tprObj["valid"]    = tprData.is_valid;

  // 5. 📌 SCHNEIDER METER 1 (ALL 66 REGISTERS)
  JsonObject m1Obj = doc.createNestedObject("meter1");
  addMeterToJSON(m1Obj, m1_online, m1_sec1, m1_sec2, m1_sec3);

  // 6. 📌 SCHNEIDER METER 2 (ALL 66 REGISTERS)
  JsonObject m2Obj = doc.createNestedObject("meter2");
  addMeterToJSON(m2Obj, m2_online, m2_sec1, m2_sec2, m2_sec3);

  // 👇 YAHAN BAS 'static' LAGA DIJIYE (Stack Overflow 100% Solve):
  static char jsonBuffer[6144]; 
  size_t n = serializeJson(doc, jsonBuffer);
    Serial.println(jsonBuffer);


  // Serial.print("[MQTT] Publishing Complete Telemetry ("); 
  // Serial.print(n); Serial.print(" bytes) to "); Serial.println(mqtt_topic);

  if (mqttClient.publish(mqtt_topic, jsonBuffer)) {
    // Serial.println("✅ Publish OK (Both Meters 132 Regs Included!)");
  } else {
    // Serial.println("❌ Publish FAILED -- Check buffer size!");
  }

  // 💾 SAVE EXACT FULL JSON TO SD CARD
  if (sd_card_mounted) {
    File logFile = SD.open(logFileName, FILE_APPEND);
    if (logFile) {
      logFile.print(getFormattedTime() + " [FULL MQTT JSON]: ");
      logFile.println(jsonBuffer);
      logFile.close();
    }
  }
}



// --------------------------------------------------------------------------
// 📱 LOCAL WIFI AP & WEB CONFIG PORTAL (192.168.4.1)
// --------------------------------------------------------------------------
WebServer localServer(80);

void handleRoot() {
  String html = "";
  html.reserve(7000); 
  
  html += "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>OXMO Gateway Config</title>";
  html += "<style>";
  html += ":root { --bg: #0f172a; --card: #1e293b; --border: #334155; --text: #f8fafc; --text-muted: #94a3b8; --accent: #0ea5e9; --success: #10b981; }";
  html += "* { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', Roboto, sans-serif; }";
  html += "body { background: var(--bg); color: var(--text); padding: 15px; display: flex; justify-content: center; }";
  html += ".container { max-width: 600px; width: 100%; }";
  
  html += ".header { text-align: center; padding: 10px 0 20px; font-size: 26px; font-weight: bold; }";
  html += ".header span { color: var(--accent); }";
  
  /* Top Stats */
  html += ".stats-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; margin-bottom: 20px; }";
  html += ".stat-card { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 15px; text-align: center; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }";
  html += ".stat-card .label { font-size: 11px; color: var(--text-muted); text-transform: uppercase; margin-bottom: 6px; letter-spacing: 1px; }";
  html += ".stat-card .val { font-size: 22px; font-weight: bold; color: var(--text); }";
  
  /* Tabs System */
  html += ".tabs { display: flex; gap: 5px; margin-bottom: 15px; background: var(--card); padding: 6px; border-radius: 10px; border: 1px solid var(--border); }";
  html += ".tab-btn { flex: 1; padding: 10px; text-align: center; font-size: 14px; font-weight: 600; color: var(--text-muted); cursor: pointer; border-radius: 8px; transition: 0.2s; }";
  html += ".tab-btn.active { background: var(--accent); color: #fff; }";
  
  html += ".tab-content { display: none; background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 20px; animation: fadeIn 0.3s; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }";
  html += ".tab-content.active { display: block; }";
  html += "@keyframes fadeIn { from { opacity: 0; transform: translateY(5px); } to { opacity: 1; transform: translateY(0); } }";
  
  /* Inputs */
  html += ".form-group { margin-bottom: 15px; }";
  html += "label { display: block; font-size: 13px; color: var(--text-muted); margin-bottom: 5px; font-weight: 500;}";
  html += "input { width: 100%; background: #0b1120; border: 1px solid var(--border); color: var(--text); padding: 10px; border-radius: 8px; font-size: 14px; outline: none; }";
  html += "input:focus { border-color: var(--accent); }";
  html += ".input-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 12px; }";
  html += ".tap-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; max-height: 250px; overflow-y: auto; padding-right: 5px; }";
  
  /* Buttons */
  html += "button { width: 100%; padding: 12px; font-size: 14px; font-weight: bold; color: #fff; background: var(--accent); border: none; border-radius: 8px; cursor: pointer; margin-top: 5px; }";
  
  html += "::-webkit-scrollbar { width: 6px; }";
  html += "::-webkit-scrollbar-track { background: transparent; }";
  html += "::-webkit-scrollbar-thumb { background: var(--border); border-radius: 10px; }";
  html += "</style>";
  
  html += "<script>";
  html += "function openTab(event, id) {";
  html += "  document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));";
  html += "  document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));";
  html += "  document.getElementById(id).classList.add('active');";
  html += "  event.currentTarget.classList.add('active');";
  html += "}";
  
  // AJAX BACKGROUND REFRESH (No page reload needed!)
  html += "setInterval(() => {";
  html += "  fetch('/api/data').then(r => r.json()).then(data => {";
  html += "    document.getElementById('val_ma').innerText = data.mA.toFixed(2) + ' mA';";
  html += "    document.getElementById('val_v').innerText = data.volt.toFixed(3) + ' V';";
  html += "    document.getElementById('val_tap').innerText = 'Tap ' + data.tap;";
  html += "    document.getElementById('val_ops').innerText = data.ops;";
  html += "  }).catch(e => console.log('Wait'));";
  html += "}, 2000);"; // Har 2 sec me background update
  html += "</script>";
  html += "</head><body>";
  
  html += "<div class='container'>";
  html += "<div class='header'>⚡ OXMO <span>Gateway</span></div>";
  
  // 📊 Live Background Updating Stats
  html += "<div class='stats-grid'>";
  html += "<div class='stat-card'><div class='label'>4-20mA Sensor</div><div class='val' id='val_ma'>" + String(analog_4_20mA_val, 2) + " mA</div></div>";
  html += "<div class='stat-card'><div class='label'>OLTC Voltage</div><div class='val' style='color:var(--success);' id='val_v'>" + String(oltc_live_voltage, 3) + " V</div></div>";
  html += "<div class='stat-card'><div class='label'>Live Tap Pos</div><div class='val' id='val_tap'>Tap " + String(oltcData.currentTap) + "</div></div>";
  html += "<div class='stat-card'><div class='label'>Total Operations</div><div class='val' id='val_ops'>" + String(oltcData.tapCounter) + "</div></div>";
  html += "</div>";

  // 📑 Tabs Buttons
  html += "<div class='tabs'>";
  html += "<div class='tab-btn active' onclick=\"openTab(event, 'tab1')\">🌐 Network</div>";
  html += "<div class='tab-btn' onclick=\"openTab(event, 'tab2')\">🚨 Alarms</div>";
  html += "<div class='tab-btn' onclick=\"openTab(event, 'tab3')\">🎛️ Tap Calib</div>";
  html += "<div class='tab-btn' onclick=\"openTab(event, 'tab4')\">📶 Wi-Fi</div>"; // 👈 YEH NAYI LINE
  html += "<div class='tab-btn' onclick=\"openTab(event, 'tab5')\">🔌 Modbus</div>";
  html += "</div>";

  // 🌐 TAB 1: Network Settings
  html += "<div id='tab1' class='tab-content active'>";
  html += "<form action='/set_config' method='POST'>";
  // Hidden inputs for Temp Alarms (Taki network save karte time alarms delete na ho jaye)
  html += "<input type='hidden' name='alrm_oil' value='" + String(alarm_oil_temp) + "'>";
  html += "<input type='hidden' name='alrm_hv' value='" + String(alarm_hv_temp) + "'>";
  html += "<input type='hidden' name='alrm_lv' value='" + String(alarm_lv_temp) + "'>";
  
  html += "<div class='form-group'><label>MQTT Broker URL / IP</label><input type='text' name='mq_broker' value='" + mqtt_broker + "' required></div>";
  html += "<div class='input-grid'>";
  html += "<div class='form-group'><label>Port</label><input type='number' name='mq_port' value='" + String(mqtt_port) + "' required></div>";
  html += "<div class='form-group'><label>Heartbeat (Secs)</label><input type='number' name='up_int' value='" + String(upload_interval_sec) + "' required></div>";
  html += "<div class='form-group'><label>Username</label><input type='text' name='mq_user' value='" + mqtt_user + "' required></div>";
  html += "<div class='form-group'><label>Password</label><input type='text' name='mq_pass' value='" + mqtt_pass + "' required></div>";
  html += "</div>";
  html += "<button type='submit'>Save Network Settings 💾</button>";
  html += "</form></div>";

  // 🚨 TAB 2: Alarm Settings
  html += "<div id='tab2' class='tab-content'>";
  html += "<form action='/set_config' method='POST'>";
  // Hidden inputs for Network (Taki alarm save karte time network delete na ho)
  html += "<input type='hidden' name='mq_broker' value='" + mqtt_broker + "'>";
  html += "<input type='hidden' name='mq_port' value='" + String(mqtt_port) + "'>";
  html += "<input type='hidden' name='up_int' value='" + String(upload_interval_sec) + "'>";
  html += "<input type='hidden' name='mq_user' value='" + mqtt_user + "'>";
  html += "<input type='hidden' name='mq_pass' value='" + mqtt_pass + "'>";
  
  html += "<h4 style='color:var(--text); margin-bottom:15px; font-weight:normal;'>TPR-702 Temperature Thresholds (°C)</h4>";
  html += "<div class='input-grid' style='grid-template-columns: repeat(3, 1fr);'>";
  html += "<div class='form-group'><label>Oil Limit</label><input type='number' name='alrm_oil' value='" + String(alarm_oil_temp) + "' required></div>";
  html += "<div class='form-group'><label>HV Limit</label><input type='number' name='alrm_hv' value='" + String(alarm_hv_temp) + "' required></div>";
  html += "<div class='form-group'><label>LV Limit</label><input type='number' name='alrm_lv' value='" + String(alarm_lv_temp) + "' required></div>";
  html += "</div>";
  html += "<button type='submit'>Save Alarm Settings 🚨</button>";
  html += "</form></div>";

  // 🎛️ TAB 3: Tap Calibration
  html += "<div id='tab3' class='tab-content'>";
  html += "<form action='/set_counter' method='POST'>";
  html += "<div class='input-grid' style='align-items:end;'>";
  html += "<div class='form-group' style='margin:0;'><label>Override Base Counter</label><input type='number' name='count_val' required></div>";
  html += "<div><button type='submit' style='margin:0;'>Update</button></div>";
  html += "</div></form>";
  
  html += "<hr style='border:none; border-top:1px dashed var(--border); margin:20px 0;'>";
  html += "<label>17-Tap Voltage Mapping (Volts)</label>";
  html += "<form action='/set_voltages' method='POST'>";
  html += "<div class='tap-grid'>";
  for (int i = 0; i < 17; i++) {
    html += "<div class='form-group' style='margin-bottom:0;'><label style='font-size:11px;'>Tap " + String(i + 1) + "</label>";
    html += "<input type='number' step='0.001' name='tap_" + String(i + 1) + "' value='" + String(TAP_VOLTAGES[i], 3) + "' required></div>";
  }
  html += "</div>";
  html += "<button type='submit' style='margin-top:15px;'>Save Voltages to Flash</button>";
  html += "</form>";
  html += "</div>";

  // 📶 TAB 4: Wi-Fi Setup
  html += "<div id='tab4' class='tab-content'>";
  html += "<form action='/set_wifi' method='POST'>";
  html += "<div class='form-group'><label>Wi-Fi SSID</label><input type='text' name='w_ssid' value='" + wifi_ssid + "' placeholder='Enter Wi-Fi Name'></div>";
  html += "<div class='form-group'><label>Wi-Fi Password</label><input type='text' name='w_pass' value='" + wifi_pass + "' placeholder='Enter Password'></div>";
  html += "<button type='submit'>Save Wi-Fi Credentials 💾</button>";
  html += "</form></div>";

  // 🔌 TAB 5: Modbus RS485 Setup
  html += "<div id='tab5' class='tab-content'>";
  html += "<form action='/set_modbus' method='POST'>";
  
  html += "<div class='input-grid'>";
  html += "<div class='form-group'><label>Baud Rate</label><select name='mb_baud' style='width:100%; padding:10px; border-radius:8px; background:#0b1120; color:white; border:1px solid var(--border);'>";
  String bauds[] = {"2400", "4800", "9600", "19200", "38400", "57600", "115200"};
  for (String b : bauds) {
      html += "<option value='" + b + "'" + (String(modbus_baud) == b ? " selected" : "") + ">" + b + "</option>";
  }
  html += "</select></div>";
  
  html += "<div class='form-group'><label>Parity / Flow</label><select name='mb_parity' style='width:100%; padding:10px; border-radius:8px; background:#0b1120; color:white; border:1px solid var(--border);'>";
  String parities[] = {"8N1", "8E1", "8O1", "8N2", "8E2", "8O2"};
  for (String p : parities) {
      html += "<option value='" + p + "'" + (modbus_parity == p ? " selected" : "") + ">" + p + "</option>";
  }
  html += "</select></div>";
  html += "</div>";

  html += "<div class='input-grid'>";
  html += "<div class='form-group'><label>Meter 1 ID</label><input type='number' name='m1_id' value='" + String(schneider_1_id) + "' required></div>";
  html += "<div class='form-group'><label>Meter 2 ID</label><input type='number' name='m2_id' value='" + String(schneider_2_id) + "' required></div>";
  html += "<div class='form-group'><label>TPR-702 ID</label><input type='number' name='tpr_id' value='" + String(tpr702_id) + "' required></div>";
  html += "</div>"; 

  html += "<button type='submit'>Save & Restart Modbus 💾</button>";
  html += "</form></div>";

  html += "</div></body></html>";
  localServer.send(200, "text/html; charset=utf-8", html);
}

void handleSetCounter() {
  if (localServer.hasArg("count_val")) {
    uint32_t newCount = localServer.arg("count_val").toInt();
    oltcData.tapCounter = newCount;
    nvsStorage.putUInt("tap_count", newCount);
    Serial.printf("\n📱 [PORTAL] Tap Counter Calibrated To: %u\n", newCount);
    localServer.sendHeader("Location", "/");
    localServer.send(303);
  }
}

// 📌 17 Taps Ke Custom Voltages Flash me Save Karne Ka Handler
void handleSetVoltages() {
  for (int i = 0; i < 17; i++) {
    String fieldName = "tap_" + String(i + 1);
    if (localServer.hasArg(fieldName)) {
      TAP_VOLTAGES[i] = localServer.arg(fieldName).toFloat();
    }
  }
  nvsStorage.putBytes("tap_volts", TAP_VOLTAGES, sizeof(TAP_VOLTAGES));
  Serial.println("\n✅ [PORTAL] All 17 Tap Voltages Successfully Saved to Flash NVS!\n");

  localServer.sendHeader("Location", "/");
  localServer.send(303);
}

// 📌 Factory Defaults Restore Karne Ka Handler
void handleResetVoltages() {
  memcpy(TAP_VOLTAGES, DEFAULT_TAP_VOLTAGES, sizeof(TAP_VOLTAGES));
  nvsStorage.putBytes("tap_volts", TAP_VOLTAGES, sizeof(TAP_VOLTAGES));
  Serial.println("\n🔄 [PORTAL] Tap Voltages Restored to Factory Defaults!\n");

  localServer.sendHeader("Location", "/");
  localServer.send(303);
}

// 👇👇👇 YAHAN PAR NAYA FUNCTION PASTE KAREIN 👇👇👇
void handleSetConfig() {
  if (localServer.hasArg("mq_broker")) {
    mqtt_broker = localServer.arg("mq_broker"); nvsStorage.putString("mq_broker", mqtt_broker);
    mqtt_port   = localServer.arg("mq_port").toInt(); nvsStorage.putUInt("mq_port", mqtt_port);
    mqtt_user   = localServer.arg("mq_user"); nvsStorage.putString("mq_user", mqtt_user);
    mqtt_pass   = localServer.arg("mq_pass"); nvsStorage.putString("mq_pass", mqtt_pass);
    
    upload_interval_sec = localServer.arg("up_int").toInt(); nvsStorage.putUInt("up_int", upload_interval_sec);
    alarm_oil_temp      = localServer.arg("alrm_oil").toInt(); nvsStorage.putUInt("alrm_oil", alarm_oil_temp);
    alarm_hv_temp       = localServer.arg("alrm_hv").toInt(); nvsStorage.putUInt("alrm_hv", alarm_hv_temp);
    alarm_lv_temp       = localServer.arg("alrm_lv").toInt(); nvsStorage.putUInt("alrm_lv", alarm_lv_temp);
    Serial.println("\n✅ [PORTAL] Config Updated! Disconnecting MQTT to apply live... ");
    if (mqttClient.connected()) {
      mqttClient.disconnect(); // 👈 Bina ESP32 Restart kiye Agli hi second naye server par connect kar lega!
    }
  }
  localServer.sendHeader("Location", "/");
  localServer.send(303);
}

// 📶 Wi-Fi Save Karne Ka Handler
void handleSetWifi() {
  if (localServer.hasArg("w_ssid")) {
    wifi_ssid = localServer.arg("w_ssid"); 
    nvsStorage.putString("wifi_ssid", wifi_ssid);
    
    wifi_pass = localServer.arg("w_pass"); 
    nvsStorage.putString("wifi_pass", wifi_pass);
    
    Serial.println("\n✅ [PORTAL] Wi-Fi Credentials Saved to Flash NVS!");
  }
  localServer.sendHeader("Location", "/");
  localServer.send(303);
}

void handleSetModbus() {
  if (localServer.hasArg("mb_baud")) {
    modbus_baud = localServer.arg("mb_baud").toInt();
    nvsStorage.putUInt("mb_baud", modbus_baud);
    
    modbus_parity = localServer.arg("mb_parity");
    nvsStorage.putString("mb_parity", modbus_parity);

    schneider_1_id = localServer.arg("m1_id").toInt();
    nvsStorage.putUInt("m1_id", schneider_1_id);

    schneider_2_id = localServer.arg("m2_id").toInt();
    nvsStorage.putUInt("m2_id", schneider_2_id);

    tpr702_id = localServer.arg("tpr_id").toInt();
    nvsStorage.putUInt("tpr_id", tpr702_id);
    
    Serial.println("\n✅ [PORTAL] Modbus Config Saved! Applying changes live (No Restart)...");
    
    // 👇 BINA RESTART KIYE LIVE SERIAL PORT UPDATE 👇
    uint32_t config = SERIAL_8E1;
    if (modbus_parity == "8N1") config = SERIAL_8N1;
    else if (modbus_parity == "8E1") config = SERIAL_8E1;
    else if (modbus_parity == "8O1") config = SERIAL_8O1;
    else if (modbus_parity == "8N2") config = SERIAL_8N2;
    else if (modbus_parity == "8E2") config = SERIAL_8E2;
    else if (modbus_parity == "8O2") config = SERIAL_8O2;

    RS485Serial.end(); // Purana Port Band karein
    delay(20);
    RS485Serial.begin(modbus_baud, config, RS485_RX_PIN, RS485_TX_PIN); // Nayi settings ke sath Start karein
    Serial.println("🔌 RS485 Port re-initialized with new Baud & Parity!");
    // 👆 ========================================= 👆

  }
  
  localServer.sendHeader("Location", "/");
  localServer.send(303);
}


// 👇 YEH NAYA FUNCTION ADD KAREIN (Background me live stats bhejne ke liye)
void handleLiveStats() {
  String json = "{";
  json += "\"mA\":" + String(analog_4_20mA_val, 2) + ",";
  json += "\"volt\":" + String(oltc_live_voltage, 3) + ",";
  json += "\"tap\":" + String(oltcData.currentTap) + ",";
  json += "\"ops\":" + String(oltcData.tapCounter);
  json += "}";
  localServer.send(200, "application/json", json);
}

void setupLocalWebPortal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("OXMO_GATEWAY_CONFIG", "12345678");
  Serial.println("📱 [WIFI AP] Hotspot: OXMO_GATEWAY_CONFIG (Password: 12345678)");
  Serial.println("🌐 [WEB PORTAL] Open: http://192.168.4.1");

  localServer.on("/", handleRoot);
  localServer.on("/set_counter", HTTP_POST, handleSetCounter);
  localServer.on("/set_voltages", HTTP_POST, handleSetVoltages); // 👈 Naya Handler
  localServer.on("/reset_voltages", HTTP_POST, handleResetVoltages); // 👈 Reset Handler
  localServer.on("/set_config", HTTP_POST, handleSetConfig); // 👈 Yeh line add karni hai
  localServer.on("/api/data", HTTP_GET, handleLiveStats); // 👈 BAS YEH 1 LINE NAYI ADD KAREIN
  localServer.on("/set_wifi", HTTP_POST, handleSetWifi); // 👈 YEH LINE ADD KAREIN
  localServer.on("/set_modbus", HTTP_POST, handleSetModbus); // 👈 Yeh nayi line add karni hai
  localServer.begin();
}

bool isModemAlive() {
  Serial.println("[PPP] Checking if 4G Modem is physically connected...");
  
  // Modem ko halke se wake-up pulse dena
  pinMode(PPP_MODEM_RST_PIN, OUTPUT);
  digitalWrite(PPP_MODEM_RST_PIN, LOW);   
  delay(200);                             
  digitalWrite(PPP_MODEM_RST_PIN, HIGH);  
  delay(1000); 
  digitalWrite(PPP_MODEM_RST_PIN, LOW);   
  delay(1000);    

  // Temporarily UART khol kar AT bhej kar check karna
  HardwareSerial probeSerial(2);
  probeSerial.begin(115200, SERIAL_8N1, PPP_MODEM_RX_PIN, PPP_MODEM_TX_PIN);
  
  probeSerial.println("AT");
  delay(100);
  probeSerial.println("AT"); 
  
  uint32_t t0 = millis();
  bool alive = false;
  while(millis() - t0 < 3000) { // 3 seconds tak reply ka wait
    if(probeSerial.available()) {
      String res = probeSerial.readString();
      if(res.indexOf("OK") != -1 || res.indexOf("AT") != -1 || res.indexOf("RDY") != -1) {
        alive = true;
        break;
      }
    }
    delay(10);
  }
  
  probeSerial.end(); // UART ko wapas PPP library ke liye chhod do
  return alive;
}


// --------------------------------------------------------------------------
// SETUP / LOOP
// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

   // 📌 Local Wi-Fi Config Hotspot Shuru Karein
  setupLocalWebPortal();   // 👈 Ye line add karein

  // 📌 1. CONFIGURE DIGITAL INPUT PINS (GPIO 41 & GPIO 42)
  pinMode(DI_PIN_1, INPUT_PULLUP);
  pinMode(DI_PIN_2, INPUT_PULLUP);
  pinMode(DI_PIN_3, INPUT_PULLUP);  // 👈 Naya
  pinMode(DI_PIN_4, INPUT_PULLUP);  // 👈 Naya
  pinMode(DI_PIN_5, INPUT_PULLUP);  // 👈 Naya
  pinMode(DI_PIN_6, INPUT_PULLUP);  // 👈 Naya

  // 👇 YEH NAYA ADD KAREIN (Bootup par current status save karne ke liye):
  last_di_states[0] = digitalRead(DI_PIN_1);
  last_di_states[1] = digitalRead(DI_PIN_2);
  last_di_states[2] = digitalRead(DI_PIN_3);
  last_di_states[3] = digitalRead(DI_PIN_4);
  last_di_states[4] = digitalRead(DI_PIN_5);
  last_di_states[5] = digitalRead(DI_PIN_6);

  // 👇 BAS YEH 1 LINE ADD KAR DIJIYE:
  pinMode(OLTC_TAP_PIN, INPUT_PULLDOWN); // 👈 Hawa me Pin 10 ko 0V par lock rakhega!

    // 👇 YAHAN PASTE KAREIN (Flash Memory se Tap Counter Load):
  nvsStorage.begin("oltc_nvs", false);

  // 👇 YEH 9 LINES NAYI ADD KAREIN:
  schneider_1_id = nvsStorage.getUInt("m1_id", 1);
  schneider_2_id = nvsStorage.getUInt("m2_id", 3);
  tpr702_id      = nvsStorage.getUInt("tpr_id", 2);
  modbus_baud    = nvsStorage.getUInt("mb_baud", 9600);
  modbus_parity  = nvsStorage.getString("mb_parity", "8E1");

  mqtt_broker = nvsStorage.getString("mq_broker", "otplai.com");
  mqtt_port   = nvsStorage.getUInt("mq_port", 8883);
  mqtt_user   = nvsStorage.getString("mq_user", "oxmo");
  mqtt_pass   = nvsStorage.getString("mq_pass", "123456789");
  wifi_ssid = nvsStorage.getString("wifi_ssid", "");
  wifi_pass = nvsStorage.getString("wifi_pass", "");

  
  upload_interval_sec = nvsStorage.getUInt("up_int", 60);
  alarm_oil_temp      = nvsStorage.getUInt("alrm_oil", 80);
  alarm_hv_temp       = nvsStorage.getUInt("alrm_hv", 90);
  alarm_lv_temp       = nvsStorage.getUInt("alrm_lv", 90);

  oltcData.tapCounter = nvsStorage.getUInt("tap_count", 0);
  oltcData.currentTap = 1;
  oltcData.sensor_ok  = false;
  // 👇 YEH NAYA CODE ADD KAREIN (Saved Tap Voltages Load):
  size_t vLen = nvsStorage.getBytes("tap_volts", TAP_VOLTAGES, sizeof(TAP_VOLTAGES));
  if (vLen == sizeof(TAP_VOLTAGES)) {
    Serial.println("💾 [FLASH NVS] Loaded Custom Tap Voltages from Flash! ✅");
  } else {
    Serial.println("💾 [FLASH NVS] Using Default Tap Voltages.");
  }
  Serial.printf("\n💾 [FLASH NVS] Loaded Tap Counter: %u\n", oltcData.tapCounter);

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

  // 📌 DYNAMIC MQTT TOPIC: transformer/<MACADDRESS>/rx
  String mac = getESP32HardwareMAC();
  mac.replace(":", "");
  snprintf(mqtt_topic, sizeof(mqtt_topic), "transformer/%s/rx", mac.c_str());
  // 👇 BAS YE 1 LINE ADD KAR LO (Command Rx Topic):
  snprintf(mqtt_sub_topic, sizeof(mqtt_sub_topic), "transformer/%s/tx", mac.c_str());

  Serial.println("\n========================================================");
  Serial.print("   TARGET MQTT TOPIC: "); Serial.println(mqtt_topic);
  Serial.println("   ESP32 GATEWAY -- SCHNEIDER + TPR-702 + PPP + RTC SD ");
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

  // RS485Serial.begin(MODBUS_BAUD, SERIAL_8E1, RS485_RX_PIN, RS485_TX_PIN);
    // 📌 DYNAMIC RS485 PORT CONFIGURATION
  uint32_t config = SERIAL_8E1;
  if (modbus_parity == "8N1") config = SERIAL_8N1;
  else if (modbus_parity == "8E1") config = SERIAL_8E1;
  else if (modbus_parity == "8O1") config = SERIAL_8O1;
  else if (modbus_parity == "8N2") config = SERIAL_8N2;
  else if (modbus_parity == "8E2") config = SERIAL_8E2;
  else if (modbus_parity == "8O2") config = SERIAL_8O2;

  RS485Serial.begin(modbus_baud, config, RS485_RX_PIN, RS485_TX_PIN);


  Network.onEvent(onNetworkEvent);

   // =============================================================
  // 🚀 SMART STARTUP SEQUENCE 
  // =============================================================

  // STEP 1: ETHERNET PEHLE START KAR DO
  SPI_ETH.begin(ETH_SCK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
  Serial.println("[ETH] Starting Ethernet...");
  ETH.begin(ETH_PHY_W5500, 1, ETH_CS_PIN, ETH_INT_PIN, -1, SPI_ETH);

  // STEP 2: SMART MODEM CHECK (Agar laga hai toh hi start hoga)
  if (isModemAlive()) {
    Serial.println("[PPP] Modem Detected! Starting PPP...");
    if (!startPPP()) {
      Serial.println("[PPP] Connection FAILED -- will retry in loop().");
    }
  } else {
    Serial.println("[PPP] ❌ Modem NOT DETECTED! Bypassing 4G to prevent Hang...");
  }

  // STEP 3: START LOCAL WEB PORTAL / WIFI AP
  setupLocalWebPortal();

  // STEP 4: CONNECT MQTT
  reconnectMQTT();
}


static int network_fail_count = 0;

void loop() {
  esp_task_wdt_reset(); // Feed Watchdog
  localServer.handleClient(); // 👈 Ye line add karein (Web Page Requests)

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
      // 👇 YAHAN PASTE KAREIN (Tap Counter Command):
      else if (inputStr.startsWith("set_tap=")) {
          uint32_t val = inputStr.substring(8).toInt();
          oltcData.tapCounter = val;
          nvsStorage.putUInt("tap_count", val);
          Serial.printf("✅ [SERIAL] Tap Counter Successfully Calibrated To: %u\n", val);
      }
  }

  mqttClient.loop();

  if (!eth_got_ip && !ppp_got_ip && !PPP.attached()) {
    network_fail_count++;
    Serial.print("⚠️ [PPP] Link Down! Retry Attempt: "); 
    Serial.println(network_fail_count);

    if (network_fail_count >= 10) {
      hardwareResetModem();
      network_fail_count = 0; 
    }

    // Yahan bhi check karega ki hardware juda hai ya nahi
    if (isModemAlive()) {
      startPPP();
    } else {
      Serial.println("⚠️ [PPP] Modem Missing! Cannot reconnect 4G. Check LAN...");
      delay(2000); // 2 second ruko taaki screen par spam na ho
    }
  } else if (eth_got_ip || ppp_got_ip) {
    network_fail_count = 0;
  }

  if ((eth_got_ip || ppp_got_ip) && !mqttClient.connected()) {
    reconnectMQTT();
  }

    // ==========================================================
  // ⚡ FAST LOOP: HAR 200ms ME ALARM & TAP CHANGE MONITOR KAREGA
  // ==========================================================
  bool current_di[6] = {
    digitalRead(DI_PIN_1), digitalRead(DI_PIN_2), digitalRead(DI_PIN_3),
    digitalRead(DI_PIN_4), digitalRead(DI_PIN_5), digitalRead(DI_PIN_6)
  };
  
  for(int i = 0; i < 6; i++) {
    if(current_di[i] != last_di_states[i]) {
      last_di_states[i] = current_di[i]; 
      force_mqtt_publish = true;         
      current_alarm_cause = "DI_" + String(i+1) + "_ALARM";
      // Serial.println("\n🚨 [ALARM TRIGGERED] " + current_alarm_cause);
    }
  }
  int oldTap = oltcData.currentTap;
  readOLTCSensor(); 
  if (oltcData.currentTap != oldTap) {
    force_mqtt_publish = true;
    current_alarm_cause = "OLTC_TAP_CHANGED";
    // Serial.println("\n🎛️ [TAP CHANGED] Publishing Immediately!");
  }
  // ==========================================================
  // ⏳ SLOW LOOP: HAR 1 MINUTE (60000ms) ME MODBUS READ KAREGA
  // ==========================================================
  if (millis() - last_meter_read_time >= (upload_interval_sec * 1000) || last_meter_read_time == 0) {
    last_meter_read_time = millis();
    
    // Serial.println("\n⏱️ [HEARTBEAT] 1-Minute Modbus Meter Reading Started...");
    
    m1_online = readSchneiderMeter(schneider_1_id, m1_sec1, m1_sec2, m1_sec3);
    delay(50); 
    m2_online = readSchneiderMeter(schneider_2_id, m2_sec1, m2_sec2, m2_sec3);
    delay(50); 
    tprData.is_valid = readTPR702(tpr702_id, tprData.oilTemp, tprData.hvTemp, tprData.lvTemp);

     // 👇 YEH ALARM CHECK ADD KAREIN:
    if (tprData.is_valid) {
      if (tprData.oilTemp >= alarm_oil_temp)      current_alarm_cause = "HIGH_OIL_TEMP_ALARM";
      else if (tprData.hvTemp >= alarm_hv_temp)   current_alarm_cause = "HIGH_HV_TEMP_ALARM";
      else if (tprData.lvTemp >= alarm_lv_temp)   current_alarm_cause = "HIGH_LV_TEMP_ALARM";
    }
    
    read4to20mASensor();
    
    printAllDataToSerial(); // Log Full Data to Serial and SD Card
    
    // Heartbeat par bhi force publish karein
    force_mqtt_publish = true; 
    if (current_alarm_cause == "") {
      current_alarm_cause = String(upload_interval_sec) + "_SEC_HEARTBEAT";
    }
  }
    // ==========================================================
  // 📤 PUBLISH TO MQTT & SD CARD (Jab Alarm aayega ya 1-Min poora hoga)
  // ==========================================================
  if (force_mqtt_publish) {
    
    // 1. 📡 Publish to MQTT (Aur SD Card me FULL JSON Save karein)
    // Yeh function automatically Internet na hone par bhi JSON ko SD me save kar dega!
    publishMQTTTelemetry(current_alarm_cause);

    // 2. 💾 SAVE TELEMETRY LOG SUMMARY TO SD CARD WITH RTC TIMESTAMP
    if (sd_card_mounted) {
      String logLine = "M1_kWh:" + String(m1_sec1.totalActive_kWh, 2) + 
                       ", M1_V:" + String(m1_sec2.avgLineVoltage, 1) + 
                       ", M1_A:" + String(m1_sec2.avgCurrent, 2) + 
                       ", M1_kW:" + String(m1_sec2.totalActivePower, 2) +
                       " | M2_kWh:" + String(m2_sec1.totalActive_kWh, 2) + 
                       ", M2_V:" + String(m2_sec2.avgLineVoltage, 1) + 
                       ", M2_A:" + String(m2_sec2.avgCurrent, 2) + 
                       ", M2_kW:" + String(m2_sec2.totalActivePower, 2) +
                       " | OilT:" + String(tprData.oilTemp) + "C" +
                       ", HVT:" + String(tprData.hvTemp) + "C" +
                       ", LVT:" + String(tprData.lvTemp) + "C" +
                       " | Tap:" + String(oltcData.currentTap) + 
                       ", TapOps:" + String(oltcData.tapCounter) +
                       ", mA:" + String(analog_4_20mA_val, 2) +
                       " | DI:[" + String(digitalRead(DI_PIN_1)) + String(digitalRead(DI_PIN_2)) + 
                                   String(digitalRead(DI_PIN_3)) + String(digitalRead(DI_PIN_4)) + 
                                   String(digitalRead(DI_PIN_5)) + String(digitalRead(DI_PIN_6)) + "]";
      logDataToSD(logLine);
    }

    // 3. 🔄 Sab save hone ke baad flags reset karein!
    force_mqtt_publish = false;
    current_alarm_cause = "";
  }

  // Serial.print("[SYS] Free heap: "); Serial.println(ESP.getFreeHeap());

  // 📌 WDT-SAFE SHORT DELAY (Ab yeh 5 second ki jagah sirf 200ms rukega)
  // Taaki DI Alarm aur OLTC Tap ki scanning lagataar super-fast speed me hoti rahe!
  uint32_t delayStart = millis();
  while (millis() - delayStart < 200) {
    esp_task_wdt_reset();
    localServer.handleClient(); 
    delay(20);
  }
}

