/*
  ESP32 DevKit V1 / ESP-WROOM-32 inverter simulator
  Wi-Fi AP+STA, UTF-8 web UI, seven active-HIGH outputs, onboard blue LED status,
  TWAI/CAN (TX GPIO21, RX GPIO22) and UART2 selectable between Modbus RTU slave and
  raw text (RX GPIO16, TX GPIO17, RS-485 driver enable GPIO4, slave ID 1).
  The RGB matrix is disabled to free GPIO2 for LED status.

  IMPORTANT HARDWARE LIMITATIONS:
  - The RGB matrix is disabled in this build to use GPIO2 for the onboard blue LED.
  - GPIO0 and GPIO2 are ESP32 strapping pins. External circuitry connected to
    these pins can affect boot; keep GPIO0 from being pulled LOW at reset.
  - GPIO0 is intentionally unused by the control outputs to avoid boot-mode conflicts.
  - Do not connect ESP32 GPIOs directly to a CANH/CANL bus: use a 3.3-V-compatible
    CAN transceiver. The CAN controller pins are GPIO21 (TX) and GPIO22 (RX); UART2 uses GPIO17 (TX) and GPIO16 (RX).
  - The AP uses WPA2 password protection by default (12345678); change it for bench use.
    The web UI has no user login. This is not a safety controller.
  - RGB matrix support is intentionally disabled in this build. GPIO2 is used for the onboard blue LED.

  Arduino IDE libraries:
    * ESP32 board package (classic ESP32 target)

  Tested compile target should be a classic ESP32 Dev Module with Arduino-ESP32
  core 2.x / ESP-IDF 4.x (legacy driver/twai.h API). Core 3.x compatibility may
  require adapting the TWAI API to the installed ESP-IDF version.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include "driver/twai.h"

// ---------------- Wi-Fi ----------------
static const char *DEFAULT_AP_SSID = "Repair Center";
static const char *DEFAULT_AP_PASSWORD = "12345678";
String apSSID;
String apPassword;
static const IPAddress AP_IP(10, 10, 10, 10);
static const IPAddress AP_GATEWAY(10, 10, 10, 10);
static const IPAddress AP_SUBNET(255, 255, 255, 0);
static const char *PREF_NAMESPACE = "wifi-config";

// ---------------- GPIO assignment ----------------
// RGB matrix pins/object are intentionally removed in this build.
// GPIO2 is used for the onboard blue status LED (common DevKit V1 wiring).
// If your board LED is active-LOW, swap LED_ON_LEVEL and LED_OFF_LEVEL.
static constexpr int PIN_STATUS_LED = 2;
static constexpr uint8_t LED_ON_LEVEL = HIGH;
static constexpr uint8_t LED_OFF_LEVEL = LOW;

static constexpr int PIN_BRAKE  = 13;
static constexpr int PIN_ENABLE = 14;
static constexpr int PIN_UP     = 18;
static constexpr int PIN_DOWN   = 19;
static constexpr int PIN_SPEED0 = 25;
static constexpr int PIN_SPEED1 = 26;
static constexpr int PIN_SPEED2 = 27;
// Additional relay features (classic ESP32 DevKit V1; verify board wiring).
static constexpr int PIN_READY_BUTTON = 32; // button to GND, active LOW
static constexpr int PIN_READY_RELAY = 23;  // active-HIGH relay module: LOW=OFF, HIGH=ON
static constexpr int PIN_BUZZER_RELAY = 33; // active-HIGH relay driver
static constexpr uint32_t BUZZER_PULSE_MS = 250UL;

static constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_21;
static constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_22;
static constexpr int PIN_UART_TX = 17;
static constexpr int PIN_UART_RX = 16;
static constexpr int PIN_MODBUS_DE = 4; // RS-485 DE and /RE tied together; LOW=receive, HIGH=transmit
static constexpr uint8_t MODBUS_SLAVE_ID = 1;
static constexpr uint32_t DEFAULT_UART_BAUD = 115200;
uint32_t uartBaud = DEFAULT_UART_BAUD;
uint32_t canBitrate = 500000;

WebServer server(80);
DNSServer dnsServer;
static constexpr uint16_t DNS_PORT = 53;
Preferences prefs;
HardwareSerial ExternalSerial(2);
SemaphoreHandle_t uartMutex;
volatile bool modbusModeEnabled = true;
volatile bool passiveMonitorEnabled = false;
volatile uint32_t uartModeGeneration = 0;
uint16_t monitorReadStart = 0;
uint8_t monitorReadCount = 0;
uint8_t monitorReadUnit = 0;
uint8_t monitorReadFunction = 0;

struct OutputDef { const char *name; uint8_t pin; };
static const OutputDef OUTPUTS[] = {
  {"Brake",  PIN_BRAKE},
  {"Enable", PIN_ENABLE},
  {"Up",     PIN_UP},
  {"Down",   PIN_DOWN},
  {"Speed 0", PIN_SPEED0},
  {"Speed 1", PIN_SPEED1},
  {"Speed 2", PIN_SPEED2}
};
static constexpr size_t OUTPUT_COUNT = sizeof(OUTPUTS) / sizeof(OUTPUTS[0]);

bool outputState[OUTPUT_COUNT] = {false, false, false, false, false, false, false};
SemaphoreHandle_t stateMutex;
SemaphoreHandle_t controlStateMutex;
SemaphoreHandle_t timerMutex;
portMUX_TYPE sequenceRequestMux = portMUX_INITIALIZER_UNLOCKED;
// Sequence requests are consumed by the control task; 0=none, 1=Auto UP, 2=Auto DOWN, 3=Stop.
volatile uint8_t sequenceRequest = 0;
volatile bool sequenceRunning = false;
volatile bool runModeActive = false;
volatile bool buzzerAfterRunMode = false;
volatile bool readyDelayPending = false;
volatile bool readyRelayOn = false;
volatile uint32_t readyDelayStartMs = 0;
volatile bool buzzerRelayOn = false;
uint32_t buzzerNextPulseMs = 0;
uint32_t buzzerPulseEndMs = 0;
volatile uint32_t runModeEndMs = 0;
String sequenceStatus = "Idle";
volatile uint32_t sequenceStepStartedMs = 0;
volatile uint32_t sequenceStepDurationMs = 0;
volatile uint32_t sequenceStepRemainingMs = 0;
volatile uint32_t lastCanTransmitMs = 0;
volatile uint32_t controlHeartbeatMs = 0;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t communicationTaskHandle = nullptr;
TaskHandle_t statusLedTaskHandle = nullptr;

// Editable sequence durations, in milliseconds. Stored in NVS namespace "seq-timers".
enum TimerIndex : uint8_t { T_BRAKE, T_ENABLE, T_DIRECTION, T_SPEED0_A, T_SPEED1_A,
                            T_SPEED2, T_SPEED1_B, T_SPEED0_B, T_DIRECTION_OFF, T_ENABLE_OFF,
                            T_BUZZER_INTERVAL, T_READY_DELAY, T_RUN_MODE, TIMER_COUNT };
static const char *TIMER_KEYS[TIMER_COUNT] = {"brake", "enable", "direction", "speed0a", "speed1a",
                                               "speed2", "speed1b", "speed0b", "dirOff", "enOff", "buzzerInterval", "readyDelay", "runModeHours"};
static const char *TIMER_LABELS[TIMER_COUNT] = {"Brake hold", "Enable delay", "Direction delay", "Speed 0 (first)",
                                                "Speed 1 (first)", "Speed 2", "Speed 1 (return)", "Speed 0 (return)",
                                                "Direction off delay", "Enable off delay", "Buzzer pulse interval", "Ready Signal delay", "Run Mode duration"};
uint32_t sequenceTimersMs[TIMER_COUNT] = {2000, 2000, 2000, 3000, 3000, 5000, 3000, 3000, 2000, 2000, 2000, 30000, 3600000};

uint32_t getSequenceTimerMs(size_t index) {
  if (index >= TIMER_COUNT) return 0;
  if (!timerMutex || xSemaphoreTake(timerMutex, portMAX_DELAY) != pdTRUE) return 0;
  const uint32_t value = sequenceTimersMs[index];
  xSemaphoreGive(timerMutex);
  return value;
}

void setSequenceTimerMs(size_t index, uint32_t value) {
  if (index >= TIMER_COUNT) return;
  if (!timerMutex || xSemaphoreTake(timerMutex, portMAX_DELAY) != pdTRUE) return;
  sequenceTimersMs[index] = value;
  xSemaphoreGive(timerMutex);
}

void getSequenceSnapshot(String &status, uint32_t &stepStartedMs, uint32_t &stepDurationMs) {
  status = "Unavailable";
  stepStartedMs = millis();
  stepDurationMs = 0;
  if (controlStateMutex && xSemaphoreTake(controlStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    status = sequenceStatus;
    stepStartedMs = sequenceStepStartedMs;
    stepDurationMs = sequenceStepDurationMs;
    xSemaphoreGive(controlStateMutex);
  }
}

void setSequenceStatus(const String &status) {
  if (controlStateMutex && xSemaphoreTake(controlStateMutex, portMAX_DELAY) == pdTRUE) {
    sequenceStatus = status;
    xSemaphoreGive(controlStateMutex);
  }
}

bool queueSequenceRequest(uint8_t request, bool allowReplaceStop = false) {
  bool accepted = false;
  portENTER_CRITICAL(&sequenceRequestMux);
  if (request == 3) {
    sequenceRequest = 3;
    accepted = true;
  } else if ((!sequenceRunning && sequenceRequest == 0) || (allowReplaceStop && sequenceRequest == 3)) {
    sequenceRequest = request;
    accepted = true;
  }
  portEXIT_CRITICAL(&sequenceRequestMux);
  return accepted;
}

uint8_t takeSequenceRequest() {
  portENTER_CRITICAL(&sequenceRequestMux);
  const uint8_t request = sequenceRequest;
  sequenceRequest = 0;
  if (request == 1 || request == 2 || request == 4) sequenceRunning = true;
  portEXIT_CRITICAL(&sequenceRequestMux);
  return request;
}

void setSequenceRunning(bool running) {
  portENTER_CRITICAL(&sequenceRequestMux);
  sequenceRunning = running;
  portEXIT_CRITICAL(&sequenceRequestMux);
}

void updateSequenceStep(uint32_t started, uint32_t duration, const String &status) {
  if (controlStateMutex && xSemaphoreTake(controlStateMutex, portMAX_DELAY) == pdTRUE) {
    sequenceStepStartedMs = started;
    sequenceStepDurationMs = duration;
    sequenceStepRemainingMs = duration;
    sequenceStatus = status;
    xSemaphoreGive(controlStateMutex);
  }
}

void clearSequenceStep() {
  if (controlStateMutex && xSemaphoreTake(controlStateMutex, portMAX_DELAY) == pdTRUE) {
    sequenceStepDurationMs = 0;
    sequenceStepRemainingMs = 0;
    xSemaphoreGive(controlStateMutex);
  }
}
String staSSID;
String staPassword;
volatile uint32_t canRxCount = 0;
volatile uint32_t uartRxCount = 0;
String lastUartLine;
volatile uint32_t modbusRxCount = 0;
volatile uint32_t modbusErrorCount = 0;
volatile uint16_t modbusLastCommand = 0;

// Modbus RTU holding registers: 0=last command (0 stop, 1 up, 2 down, 3 run),
// 1=status (0 idle, 1 sequence, 2 run mode), 2=output bitfield, 3=step seconds remaining.
// 0x0010..0x001C are timer settings in seconds, except runModeHours (hours).
static constexpr uint16_t MODBUS_TIMER_BASE = 0x0010;
static constexpr size_t MODBUS_MAX_ADU = 260;

// Bounded analyzer log: keeps the latest 80 CAN/UART events without unbounded RAM use.
static constexpr size_t ANALYZER_LOG_CAPACITY = 80;
struct AnalyzerEvent {
  uint32_t id;
  uint32_t ms;
  char source[7];
  char text[96];
};
AnalyzerEvent analyzerLog[ANALYZER_LOG_CAPACITY];
size_t analyzerWriteIndex = 0;
size_t analyzerCount = 0;
uint32_t analyzerNextId = 1;
SemaphoreHandle_t analyzerMutex;
volatile uint32_t canErrorCount = 0;
volatile uint32_t uartDroppedBytes = 0;
volatile uint32_t analyzerDroppedLogs = 0;
volatile uint32_t canLastRxMs = 0;
volatile uint32_t uartLastRxMs = 0;
volatile uint8_t canBusState = 0; // 0=unknown/stopped, 1=running, 2=bus-off, 3=recovering
volatile bool canDriverReady = false;

void addAnalyzerEvent(const char *source, const char *text) {
  if (!analyzerMutex) { ++analyzerDroppedLogs; return; }
  if (xSemaphoreTake(analyzerMutex, pdMS_TO_TICKS(10)) != pdTRUE) { ++analyzerDroppedLogs; return; }
  AnalyzerEvent &e = analyzerLog[analyzerWriteIndex];
  e.id = analyzerNextId++;
  e.ms = millis();
  snprintf(e.source, sizeof(e.source), "%s", source);
  snprintf(e.text, sizeof(e.text), "%s", text);
  analyzerWriteIndex = (analyzerWriteIndex + 1) % ANALYZER_LOG_CAPACITY;
  if (analyzerCount < ANALYZER_LOG_CAPACITY) ++analyzerCount;
  else ++analyzerDroppedLogs; // oldest entry overwritten by bounded ring buffer
  xSemaphoreGive(analyzerMutex);
}

// ---------------- Helpers ----------------
int outputIndexForName(const String &name) {
  for (size_t i = 0; i < OUTPUT_COUNT; ++i) {
    if (name.equals(OUTPUTS[i].name)) return (int)i;
  }
  return -1;
}

bool applyOutputChanges(uint32_t mask, uint32_t values) {
  if (!stateMutex || xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  for (size_t i = 0; i < OUTPUT_COUNT; ++i) {
    if (mask & (1UL << i)) outputState[i] = (values & (1UL << i)) != 0;
  }
  xSemaphoreGive(stateMutex);
  return true;
}

void setOutputState(size_t index, bool state) {
  if (index >= OUTPUT_COUNT) return;
  const uint32_t bit = 1UL << index;
  applyOutputChanges(bit, state ? bit : 0);
}

void copyOutputStates(bool *dst) {
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (size_t i = 0; i < OUTPUT_COUNT; ++i) dst[i] = outputState[i];
    xSemaphoreGive(stateMutex);
  }
}

String jsonEscape(const String &in) {
  String out;
  out.reserve(in.length() + 16);
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  for (size_t i = 0; i < in.length(); ++i) {
    const uint8_t c = (uint8_t)in[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if (c == '\b') out += "\\b";
    else if (c == '\f') out += "\\f";
    else if (c < 0x20) { out += "\\u00"; out += HEX_DIGITS[c >> 4]; out += HEX_DIGITS[c & 0x0F]; }
    else out += (char)c;
  }
  return out;
}

String makeStatusJson(bool includeDiagnostics = true) {
  bool states[OUTPUT_COUNT];
  copyOutputStates(states);
  String sequenceStatusCopy;
  uint32_t sequenceStartedCopy, sequenceDurationCopy;
  getSequenceSnapshot(sequenceStatusCopy, sequenceStartedCopy, sequenceDurationCopy);
  String json = "{\"apSsid\":\"" + jsonEscape(apSSID) + "\",\"apIp\":\"" + WiFi.softAPIP().toString();
  json += "\",\"staConnected\":";
  json += (WiFi.status() == WL_CONNECTED) ? "true" : "false";
  json += ",\"staIp\":\"" + WiFi.localIP().toString() + "\",\"apClients\":" + String(WiFi.softAPgetStationNum());
  json += ",\"canBitrate\":" + String(canBitrate) + ",\"uartBaud\":" + String(uartBaud);
  if (includeDiagnostics) {
  json += ",\"canRx\":" + String(canRxCount);
  json += ",\"uartRx\":" + String(uartRxCount);
  json += ",\"canErrors\":" + String(canErrorCount);
  json += ",\"uartDropped\":" + String(uartDroppedBytes);
  json += ",\"analyzerDropped\":" + String(analyzerDroppedLogs);
  json += ",\"canLastRxMs\":" + String(canLastRxMs);
  json += ",\"uartLastRxMs\":" + String(uartLastRxMs);
  }
  json += ",\"canBusState\":" + String(canBusState);
  json += ",\"modbusFrames\":" + String(modbusRxCount) + ",\"modbusErrors\":" + String(modbusErrorCount);
  json += ",\"uartMode\":\"" + String(modbusModeEnabled ? "modbus" : (passiveMonitorEnabled ? "monitor" : "raw")) + "\"";
  json += ",\"canDriverReady\":" + String(canDriverReady ? "true" : "false");
  json += ",\"freeHeap\":" + String(ESP.getFreeHeap());
  json += ",\"minFreeHeap\":" + String(ESP.getMinFreeHeap());
  json += ",\"sequenceRunning\":" + String(sequenceRunning ? "true" : "false");
  json += ",\"runModeActive\":" + String(runModeActive ? "true" : "false");
  json += ",\"readyDelayPending\":" + String(readyDelayPending ? "true" : "false");
  json += ",\"readyRelayOn\":" + String(readyRelayOn ? "true" : "false");
  json += ",\"buzzerAfterRunMode\":" + String(buzzerAfterRunMode ? "true" : "false");
  json += ",\"buzzerRelayOn\":" + String(buzzerRelayOn ? "true" : "false");
  const uint32_t elapsed = millis() - sequenceStartedCopy;
  const uint32_t remaining = sequenceDurationCopy > elapsed ? sequenceDurationCopy - elapsed : 0;
  json += ",\"sequenceStepDurationMs\":" + String(sequenceDurationCopy);
  json += ",\"sequenceStepElapsedMs\":" + String(elapsed);
  json += ",\"sequenceStepRemainingMs\":" + String(sequenceRunning ? remaining : 0);
  const char *safetyState = (!canDriverReady || canBusState == 2) ? "FAULT" : (sequenceRunning ? "RUNNING" : "READY");
  json += ",\"safetyState\":\"" + String(safetyState) + "\"";
    json += ",\"sequenceStatus\":\"" + jsonEscape(sequenceStatusCopy) + "\"";
  json += ",\"apActive\":" + String(WiFi.softAPIP()[0] != 0 ? "true" : "false");
  if (includeDiagnostics) {
  json += ",\"controlHeartbeatAgeMs\":" + String(millis() - controlHeartbeatMs);
  json += ",\"controlTaskHealthy\":" + String((millis() - controlHeartbeatMs) < 500 ? "true" : "false");
  json += ",\"controlStackHighWaterWords\":" + String(controlTaskHandle ? uxTaskGetStackHighWaterMark(controlTaskHandle) : 0);
  size_t diagCount = 0;
  if (includeDiagnostics && analyzerMutex && xSemaphoreTake(analyzerMutex, pdMS_TO_TICKS(2)) == pdTRUE) { diagCount = analyzerCount; xSemaphoreGive(analyzerMutex); }
  json += ",\"analyzerCount\":" + String(diagCount) + ",\"analyzerCapacity\":" + String(ANALYZER_LOG_CAPACITY);
  }
  json += ",\"outputs\":{";
  for (size_t i = 0; i < OUTPUT_COUNT; ++i) {
    if (i) json += ',';
    json += '"'; json += OUTPUTS[i].name; json += "\":";
    json += (digitalRead(OUTPUTS[i].pin) == HIGH) ? "true" : "false";
  }
  json += "}}";
  return json;
}

uint16_t modbusCrc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

void modbusSend(const uint8_t *payload, size_t length) {
  uint8_t response[MODBUS_MAX_ADU];
  if (length + 2 > sizeof(response)) return;
  memcpy(response, payload, length);
  const uint16_t crc = modbusCrc16(payload, length);
  response[length++] = (uint8_t)(crc & 0xFF);
  response[length++] = (uint8_t)(crc >> 8);
  digitalWrite(PIN_MODBUS_DE, HIGH);
  delayMicroseconds(50);
  ExternalSerial.write(response, length);
  ExternalSerial.flush();
  delayMicroseconds(50);
  digitalWrite(PIN_MODBUS_DE, LOW);
}

void modbusException(uint8_t function, uint8_t code) {
  const uint8_t reply[] = {MODBUS_SLAVE_ID, (uint8_t)(function | 0x80), code};
  modbusSend(reply, sizeof(reply));
}

bool modbusReadRegister(uint16_t address, uint16_t &value) {
  if (address == 0) {
    value = modbusLastCommand;
    return true;
  }
  if (address == 1) {
    value = runModeActive ? 2 : (sequenceRunning ? 1 : 0);
    return true;
  }
  if (address == 2) {
    bool states[OUTPUT_COUNT] = {};
    copyOutputStates(states);
    value = 0;
    for (size_t i = 0; i < OUTPUT_COUNT; ++i) if (states[i]) value |= (uint16_t)(1U << i);
    return true;
  }
  if (address == 3) {
    String sequenceStatusCopy;
    uint32_t sequenceStartedCopy, sequenceDurationCopy;
    getSequenceSnapshot(sequenceStatusCopy, sequenceStartedCopy, sequenceDurationCopy);
    const uint32_t elapsed = millis() - sequenceStartedCopy;
    const uint32_t remaining = sequenceDurationCopy > elapsed ? sequenceDurationCopy - elapsed : 0;
    value = (uint16_t)((remaining + 999UL) / 1000UL);
    return true;
  }
  if (address >= MODBUS_TIMER_BASE && address < MODBUS_TIMER_BASE + TIMER_COUNT) {
    const size_t index = address - MODBUS_TIMER_BASE;
    const uint32_t timer = getSequenceTimerMs(index);
    value = (uint16_t)(index == T_RUN_MODE ? timer / 3600000UL : timer / 1000UL);
    return true;
  }
  return false;
}

bool modbusSetCommand(uint16_t command) {
  if (command == 0) {
    queueSequenceRequest(3);
    modbusLastCommand = command;
    return true;
  }
  if (command > 3 || !queueSequenceRequest(command == 1 ? 1 : (command == 2 ? 2 : 4))) return false;
  modbusLastCommand = command;
  return true;
}

bool modbusSetTimer(uint16_t address, uint16_t value) {
  if (address < MODBUS_TIMER_BASE || address >= MODBUS_TIMER_BASE + TIMER_COUNT) return false;
  const size_t index = address - MODBUS_TIMER_BASE;
  const uint16_t maxValue = index == T_READY_DELAY ? 600 : (index == T_RUN_MODE ? 24 : 30);
  if (value < 1 || value > maxValue) return false;
  const uint32_t timerMs = index == T_RUN_MODE ? (uint32_t)value * 3600000UL : (uint32_t)value * 1000UL;
  setSequenceTimerMs(index, timerMs);
  prefs.begin("seq-timers", false);
  prefs.putUInt(TIMER_KEYS[index], value);
  prefs.end();
  return true;
}

void modbusProcessFrame(const uint8_t *frame, size_t length) {
  if (length < 4) return;
  const uint16_t expectedCrc = (uint16_t)frame[length - 2] | ((uint16_t)frame[length - 1] << 8);
  if (modbusCrc16(frame, length - 2) != expectedCrc) { ++modbusErrorCount; return; }
  if (frame[0] != MODBUS_SLAVE_ID) return;
  ++modbusRxCount;
  const uint8_t function = frame[1];
  char eventText[48];
  snprintf(eventText, sizeof(eventText), "Modbus RTU RX ID=%u FC=0x%02X", frame[0], function);
  addAnalyzerEvent("serial", eventText);
  if (function == 0x03) {
    const uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    const uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];
    if (count == 0 || count > 125) { modbusException(function, 3); return; }
    uint8_t reply[3 + 2 * 125];
    reply[0] = MODBUS_SLAVE_ID; reply[1] = function; reply[2] = (uint8_t)(count * 2);
    for (uint16_t i = 0; i < count; ++i) {
      uint16_t value;
      if (!modbusReadRegister((uint16_t)(start + i), value)) { modbusException(function, 2); return; }
      reply[3 + i * 2] = (uint8_t)(value >> 8);
      reply[4 + i * 2] = (uint8_t)value;
    }
    modbusSend(reply, 3 + count * 2);
    return;
  }
  if (function == 0x06) {
    const uint16_t address = ((uint16_t)frame[2] << 8) | frame[3];
    const uint16_t value = ((uint16_t)frame[4] << 8) | frame[5];
    bool ok = address == 0 ? modbusSetCommand(value) : modbusSetTimer(address, value);
    if (!ok) { modbusException(function, address == 0 ? (value > 3 ? 3 : 6) : 3); return; }
    modbusSend(frame, 6);
    return;
  }
  if (function == 0x10) {
    const uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    const uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];
    if (count == 0 || count > 123 || frame[6] != count * 2) { modbusException(function, 3); return; }
    // FC16 is reserved for timer blocks; issue commands through FC06 register 0.
    if (start < MODBUS_TIMER_BASE || start + count > MODBUS_TIMER_BASE + TIMER_COUNT) { modbusException(function, 2); return; }
    uint16_t values[123];
    for (uint16_t i = 0; i < count; ++i) values[i] = ((uint16_t)frame[7 + i * 2] << 8) | frame[8 + i * 2];
    for (uint16_t i = 0; i < count; ++i) {
      const uint16_t address = start + i;
      const size_t index = address - MODBUS_TIMER_BASE;
      const uint16_t maxValue = index == T_READY_DELAY ? 600 : (index == T_RUN_MODE ? 24 : 30);
      if (values[i] < 1 || values[i] > maxValue) { modbusException(function, 3); return; }
    }
    for (uint16_t i = 0; i < count; ++i) modbusSetTimer(start + i, values[i]);
    uint8_t reply[] = {MODBUS_SLAVE_ID, function, frame[2], frame[3], frame[4], frame[5]};
    modbusSend(reply, sizeof(reply));
    return;
  }
  modbusException(function, 1);
}

void modbusProcessByte(uint8_t value, uint8_t *buffer, size_t &length, size_t &expectedLength) {
  if (length >= MODBUS_MAX_ADU) { length = expectedLength = 0; ++modbusErrorCount; }
  buffer[length++] = value;
  if (length == 2) {
    const uint8_t function = buffer[1];
    expectedLength = (function == 0x10) ? 0 : 8;
  }
  if (length >= 7 && buffer[1] == 0x10 && expectedLength == 0) expectedLength = 9 + buffer[6];
  if (expectedLength && length >= expectedLength) {
    modbusProcessFrame(buffer, expectedLength);
    length = expectedLength = 0;
  }
}

void passiveModbusProcessFrame(const uint8_t *frame, size_t length) {
  if (length < 4) return;
  const uint16_t expectedCrc = (uint16_t)frame[length - 2] | ((uint16_t)frame[length - 1] << 8);
  if (modbusCrc16(frame, length - 2) != expectedCrc) { ++modbusErrorCount; return; }
  ++modbusRxCount;
  const uint8_t unit = frame[0], function = frame[1];
  char eventText[96];
  if ((function == 0x03 || function == 0x04) && length == 8) {
    const uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    const uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];
    monitorReadUnit = unit; monitorReadFunction = function;
    monitorReadStart = start; monitorReadCount = count > 125 ? 0 : (uint8_t)count;
    snprintf(eventText, sizeof(eventText), "RTU request ID=%u FC=0x%02X start=0x%04X count=%u", unit, function, start, count);
    addAnalyzerEvent("serial", eventText);
    return;
  }
  if ((function == 0x03 || function == 0x04) && length >= 7 && frame[2] == length - 5) {
    const uint8_t byteCount = frame[2];
    int used = snprintf(eventText, sizeof(eventText), "RTU response ID=%u FC=0x%02X", unit, function);
    const bool matched = unit == monitorReadUnit && function == monitorReadFunction && monitorReadCount == byteCount / 2;
    for (uint8_t i = 0; i + 1 < byteCount && used > 0 && (size_t)used < sizeof(eventText); i += 2) {
      const uint16_t value = ((uint16_t)frame[3 + i] << 8) | frame[4 + i];
      const uint16_t address = matched ? monitorReadStart + i / 2 : i / 2;
      used += snprintf(eventText + used, sizeof(eventText) - (size_t)used, " %s%u=%u(0x%04X)", matched ? "R" : "D", address, value, value);
    }
    addAnalyzerEvent("serial", eventText);
    monitorReadCount = 0;
    return;
  }
  snprintf(eventText, sizeof(eventText), "RTU frame ID=%u FC=0x%02X len=%u", unit, function, (unsigned)length);
  addAnalyzerEvent("serial", eventText);
}

void passiveModbusProcessByte(uint8_t value, uint8_t *buffer, size_t &length, size_t &expectedLength) {
  if (length >= MODBUS_MAX_ADU) { length = expectedLength = 0; ++modbusErrorCount; }
  buffer[length++] = value;
  if (length == 2) expectedLength = 8;
  if (length >= 3 && (buffer[1] == 0x03 || buffer[1] == 0x04)) {
    const size_t responseLength = 5U + buffer[2];
    if (responseLength > 8) expectedLength = responseLength;
    if (length == responseLength && responseLength >= 7 && responseLength != 8) {
      const uint16_t crc = (uint16_t)buffer[length - 2] | ((uint16_t)buffer[length - 1] << 8);
      if (modbusCrc16(buffer, length - 2) == crc) {
        passiveModbusProcessFrame(buffer, responseLength);
        length = expectedLength = 0;
        return;
      }
    }
  }
  if (length == 8 && modbusCrc16(buffer, 6) == ((uint16_t)buffer[6] | ((uint16_t)buffer[7] << 8))) {
    passiveModbusProcessFrame(buffer, 8);
    length = expectedLength = 0;
    return;
  }
  if (length >= 7 && buffer[1] == 0x10) expectedLength = 9U + buffer[6];
  if (length == 8 && buffer[1] != 0x10 && buffer[1] != 0x03 && buffer[1] != 0x04) {
    ++modbusErrorCount;
    length = expectedLength = 0;
    return;
  }
  if (length == 8 && (buffer[1] == 0x03 || buffer[1] == 0x04) && 5U + buffer[2] <= 8) {
    ++modbusErrorCount;
    length = expectedLength = 0;
    return;
  }
  if (expectedLength > 8 && length == expectedLength) {
    passiveModbusProcessFrame(buffer, expectedLength);
    length = expectedLength = 0;
  } else if (length >= MODBUS_MAX_ADU) {
    length = expectedLength = 0;
    ++modbusErrorCount;
  }
}

const char INDEX_HTML[] PROGMEM =
  "\n"
  "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">\n"
  "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
  "<title>I.R.A — Interactive Relay Automation</title>\n"
  "<style>\n"
  ":root{color-scheme:dark;font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,\"Segoe UI\",sans-serif;--ink:#eafff1;--muted:#a7cbb2;--panel:rgba(8,34,25,.91);--line:#285b43;--page:#041b13;--card-top:rgba(13,54,37,.96);--card-bottom:rgba(5,28,20,.96);--input:#071d15;--label:#d8f4df;--trace:rgba(76,210,130,.13);--grid:rgba(73,196,119,.045)}\n"
  ":root[data-theme=\"light\"]{color-scheme:light;--ink:#173727;--muted:#507560;--panel:rgba(238,251,241,.94);--line:#b6d9c1;--page:#dcefe1;--card-top:rgba(248,255,249,.96);--card-bottom:rgba(226,245,231,.96);--input:#f7fff9;--label:#234b34;--trace:rgba(24,133,73,.15);--grid:rgba(20,110,61,.06)}\n"
  "*{box-sizing:border-box}body{margin:0;color:var(--ink);background-color:var(--page);background-image:radial-gradient(ellipse at 12% 0%,rgba(31,135,78,.23),transparent 43%),radial-gradient(ellipse at 95% 25%,rgba(27,112,65,.15),transparent 35%),linear-gradient(135deg,transparent 0 24%,var(--grid) 24.2% 24.6%,transparent 24.8% 100%),linear-gradient(45deg,transparent 0 65%,var(--grid) 65.2% 65.6%,transparent 65.8% 100%);min-height:100vh;position:relative;isolation:isolate;transition:background-color .25s,color .25s}\n"
  "body:before{content:\"\";position:fixed;inset:0;z-index:-1;pointer-events:none;opacity:1;background-image:linear-gradient(90deg,transparent 0 7%,var(--trace) 7% 7.2%,transparent 7.2% 100%),linear-gradient(0deg,transparent 0 18%,var(--trace) 18% 18.2%,transparent 18.2% 100%),radial-gradient(circle at 7% 18%,var(--trace) 0 3px,transparent 4px),radial-gradient(circle at 38% 18%,var(--trace) 0 3px,transparent 4px),radial-gradient(circle at 38% 44%,var(--trace) 0 3px,transparent 4px),radial-gradient(circle at 91% 44%,var(--trace) 0 3px,transparent 4px),linear-gradient(90deg,transparent 0 38%,var(--trace) 38% 38.2%,transparent 38.2% 100%),linear-gradient(0deg,transparent 0 44%,var(--trace) 44% 44.2%,transparent 44.2% 100%);background-size:100% 100%;mask-image:linear-gradient(to bottom,black,transparent 95%)}\n"
  "main{max-width:900px;margin:auto;padding:clamp(14px,3vw,28px)}.topbar{display:flex;align-items:center;justify-content:space-between;gap:14px;margin-bottom:5px;flex-wrap:wrap}.topbar-main{display:flex;align-items:center;justify-content:space-between;gap:14px;flex-wrap:wrap;width:100%}.dashboard{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:9px;width:100%;margin:12px 0 5px}.dash-pill{min-width:0;display:flex;align-items:center;gap:9px;padding:10px 13px;border:1px solid var(--line);border-radius:999px;background:linear-gradient(180deg,var(--card-top),var(--card-bottom));box-shadow:0 4px 0 rgba(0,0,0,.10),inset 0 1px 0 rgba(255,255,255,.1)}.dash-icon{width:30px;height:30px;flex:0 0 30px;display:grid;place-items:center;border-radius:50%;background:rgba(55,190,112,.16);font-size:15px}.dash-copy{min-width:0;display:flex;flex-direction:column;gap:2px}.dash-label{font-size:.68rem;letter-spacing:.07em;text-transform:uppercase;color:var(--muted);font-weight:800}.dash-value{font-size:.88rem;font-weight:800;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;font-variant-numeric:tabular-nums}.dash-pill.good .dash-icon{background:rgba(35,190,102,.2);color:#37d878}.dash-pill.bad .dash-icon{background:rgba(240,68,82,.16);color:#f04452}.dash-pill.warn .dash-icon{background:rgba(255,196,43,.2);color:#e8b322}.brand{display:flex;align-items:center;gap:10px}h1{font-size:clamp(1.6rem,4vw,2.2rem);letter-spacing:-.035em;margin:0}.brandmark{width:38px;height:38px;display:grid;place-items:center;border:1px solid #48b878;border-radius:12px;background:linear-gradient(145deg,#17683e,#0a3322);color:#d6ffe3;box-shadow:0 0 20px rgba(49,190,105,.15)}.muted{color:var(--muted)}\n"
  ".theme-toggle{display:inline-flex;align-items:center;gap:8px;min-height:44px;padding:9px 14px;border-radius:999px;background:linear-gradient(180deg,var(--card-top),var(--card-bottom));border:1px solid var(--line);color:var(--ink);box-shadow:0 4px 0 rgba(0,0,0,.14),inset 0 1px 0 rgba(255,255,255,.12);font-size:16px;font-weight:700;cursor:pointer}.theme-toggle:active{transform:translateY(2px);box-shadow:0 1px 0 rgba(0,0,0,.14),inset 0 2px 4px rgba(0,0,0,.12)}.theme-icon{font-size:1.15rem;line-height:1}\n"
  ".card{background:linear-gradient(145deg,var(--card-top),var(--card-bottom));border:1px solid var(--line);border-radius:22px;padding:clamp(15px,3vw,22px);margin:16px 0;box-shadow:0 16px 38px rgba(0,0,0,.13),inset 0 1px 0 rgba(255,255,255,.06);backdrop-filter:blur(12px)}\n"
  "details.card{padding:0;overflow:hidden}details.card>summary{list-style:none;cursor:pointer;display:flex;align-items:center;justify-content:space-between;gap:12px;padding:clamp(15px,3vw,22px);font-size:1.18rem;font-weight:800}details.card>summary::-webkit-details-marker{display:none}details.card>summary:after{content:\"＋\";font-size:1.15rem;color:var(--muted);flex:0 0 auto}details.card[open]>summary:after{content:\"−\"}details.card[open]>summary{border-bottom:1px solid var(--line)}.collapse-body{padding:clamp(15px,3vw,22px)}.collapse-body h2{margin-top:0}\n"
  "h2{font-size:1.18rem;letter-spacing:-.015em;margin-top:0}h3{font-size:1rem;margin-top:22px}.controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,160px),1fr));gap:10px}\n"
  "button{appearance:none;-webkit-appearance:none;border:1px solid rgba(255,255,255,.14);border-radius:999px;padding:clamp(10px,1.5vw,12px) clamp(12px,1.8vw,16px);font-weight:750;letter-spacing:.01em;font-size:clamp(.9rem,1.6vw,.96rem);background:linear-gradient(180deg,#40516b 0%,#27364d 48%,#1c293e 100%);color:#f8fbff;cursor:pointer;min-height:46px;box-shadow:0 5px 0 #0b1423,0 9px 18px rgba(0,0,0,.18),inset 0 1px 0 rgba(255,255,255,.22);text-shadow:0 1px 1px rgba(0,0,0,.25);transition:transform .12s ease,box-shadow .12s ease,filter .12s ease,background .12s ease}\n"
  "button:hover{filter:brightness(1.12)}button.manual-tactile{transition:transform .09s ease,box-shadow .09s ease,filter .09s ease}button.manual-tactile:active,button.manual-tactile.pressed{transform:translateY(4px) scale(.975);box-shadow:0 1px 0 #0b1423,0 3px 8px rgba(0,0,0,.18),inset 0 2px 5px rgba(0,0,0,.2);filter:brightness(1.18)}\n"
  "button.analyzer-tactile{transition:transform .09s ease,box-shadow .09s ease,filter .09s ease}button.analyzer-tactile:active,button.analyzer-tactile.pressed{transform:translateY(3px) scale(.98);box-shadow:0 1px 0 #0b1423,0 3px 8px rgba(0,0,0,.18),inset 0 2px 5px rgba(0,0,0,.2);filter:brightness(1.18)}button.sequence-tactile{transition:transform .09s ease,box-shadow .09s ease,filter .09s ease}button.sequence-tactile:active,button.sequence-tactile.pressed{transform:translateY(4px) scale(.975);box-shadow:0 1px 0 rgba(0,0,0,.45),0 3px 8px rgba(0,0,0,.18),inset 0 2px 5px rgba(0,0,0,.2);filter:brightness(1.16)}button.sequence-tactile:disabled{transform:none}button:focus-visible,input:focus-visible,select:focus-visible{outline:3px solid #8dd8ff;outline-offset:3px}\n"
  "button.on{background:linear-gradient(180deg,#ff777c 0%,#f04452 45%,#bd1f35 100%);border-color:#ff9ca1;box-shadow:0 5px 0 #761326,0 0 22px rgba(239,54,75,.25),inset 0 1px 0 rgba(255,255,255,.36)}button.seq{background:linear-gradient(180deg,#68c5ff 0%,#2588f4 48%,#1553c7 100%);border-color:#9bdcff;box-shadow:0 5px 0 #103778,0 8px 22px rgba(37,136,244,.2),inset 0 1px 0 rgba(255,255,255,.38)}button.run{background:linear-gradient(180deg,#fff1a1 0%,#ffd43b 48%,#e7a900 100%);color:#342400;border-color:#fff0a0;text-shadow:0 1px 0 rgba(255,255,255,.25);box-shadow:0 5px 0 #8e5e00,0 8px 22px rgba(255,190,0,.18),inset 0 1px 0 rgba(255,255,255,.6)}button.stop{background:linear-gradient(180deg,#ff858b 0%,#f04452 48%,#b91c32 100%);border-color:#ffb0b3;box-shadow:0 5px 0 #741326,0 8px 20px rgba(240,68,82,.18),inset 0 1px 0 rgba(255,255,255,.35)}button:disabled{opacity:.45;cursor:not-allowed;filter:saturate(.55);transform:none}\n"
  "label{display:block;margin:10px 0 6px;color:var(--label);font-weight:600}select,input{box-sizing:border-box;width:100%;padding:12px 14px;border-radius:14px;border:1px solid var(--line);background:linear-gradient(180deg,var(--input),var(--card-bottom));color:var(--ink);font-size:16px;box-shadow:inset 0 2px 5px rgba(0,0,0,.1)}.actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}.actions button{min-height:42px}.status{white-space:pre-wrap;line-height:1.8;background:var(--panel);border:1px solid var(--line);border-radius:15px;padding:13px 15px}.warn{color:#d78b28}small{color:var(--muted)}\n"
  ":root[data-theme=\"light\"] .warn{color:#9a5810}:root[data-theme=\"light\"] .brandmark{background:linear-gradient(145deg,#d6f8df,#a9e4bb);color:#15512e}\n"
  ".diagnostics-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.diag-tile{min-width:0;padding:12px;border:1px solid var(--line);border-radius:13px;background:var(--panel);display:flex;flex-direction:column;gap:5px}.diag-tile span{font-size:.82rem;color:var(--muted)}.diag-tile strong{font-size:1.05rem;overflow-wrap:anywhere}.diag-tile small{font-size:.75rem;color:var(--muted)}@media(min-width:700px){.diagnostics-grid{grid-template-columns:repeat(4,minmax(0,1fr))}}\n"
  ".seq-progress{margin-top:12px;padding:12px 14px;border:1px solid var(--line);border-radius:14px;background:var(--panel)}.seq-progress-head,.seq-progress-foot{display:flex;justify-content:space-between;gap:10px;align-items:center;flex-wrap:wrap}.seq-progress-head{font-size:.95rem}.seq-progress-foot{margin-top:7px;color:var(--muted);font-size:.82rem}.seq-track{height:8px;background:var(--line);border-radius:99px;overflow:hidden;margin-top:10px}.seq-fill{height:100%;width:0;background:linear-gradient(90deg,#16834d,#4ee58a);border-radius:99px;transition:width .2s linear}.can-tx-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;width:100%}.can-tx-grid>div{min-width:0}.can-tx-grid input,.can-tx-grid select{width:100%}.recording-indicator{font-weight:700;color:#f59e0b}@media(max-width:620px){.can-tx-grid{grid-template-columns:1fr 1fr}}.analyzer-head{display:flex;align-items:flex-start;justify-content:space-between;gap:12px;flex-wrap:wrap}.analyzer-stats{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin:12px 0}.analyzer-tabs{display:flex;gap:8px;margin:14px 0}.analyzer-tabs .tab{flex:1;border-radius:999px}.analyzer-tabs .tab.active{background:linear-gradient(180deg,#238b58,#12683e);color:#fff}.analyzer-toolbar{display:flex;gap:7px;align-items:center;flex-wrap:wrap;margin-bottom:9px}.analyzer-toolbar button{min-height:40px;padding:9px 12px;font-size:.88rem}.analyzer-tabs{gap:7px}.analyzer-tabs .tab{min-height:42px;padding:10px 12px}.checklabel{display:flex;align-items:center;gap:6px;font-size:.9rem;color:var(--muted)}.checklabel input{width:18px;height:18px;accent-color:#238b58}.console{height:min(52vh,480px);min-height:260px;overflow:auto;white-space:pre-wrap;overflow-wrap:anywhere;background:#020a07;color:#b7f7cd;border:1px solid #285b43;border-radius:15px;padding:13px;font:12px/1.65 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}.console .can{color:#7dc8ff}.console .serial{color:#b7f7cd}.console .time{color:#769985}.analyzer-stats .dash-pill{border-radius:18px}@media(max-width:620px){.analyzer-stats{grid-template-columns:1fr 1fr}.analyzer-stats .dash-pill:last-child{grid-column:1/-1}.analyzer-head button{width:100%}.analyzer-toolbar button{flex:1}}\n"
  "@media(max-width:620px){.dashboard{grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.dash-pill{padding:9px 11px}.dash-value{font-size:.82rem}}@media(max-width:360px){.dashboard{grid-template-columns:1fr}}\n"
  ".dashboard{grid-template-columns:1fr;gap:10px;margin:14px 0 8px}.dashboard-metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px}.dashboard .dash-pill{border-radius:16px;background:var(--panel);box-shadow:inset 0 1px 0 rgba(255,255,255,.05);padding:11px 13px}.status-hero{min-height:72px;padding:14px 16px!important;border-radius:18px!important;background:linear-gradient(120deg,var(--card-top),var(--panel))!important}.status-hero .dash-icon{width:40px;height:40px;flex-basis:40px;font-size:18px}.status-hero .dash-value{font-size:1.08rem}@media(max-width:760px){.dashboard-metrics{grid-template-columns:repeat(2,minmax(0,1fr))}}@media(max-width:420px){.dashboard .dash-pill{padding:10px}.dashboard-metrics{gap:7px}}@media(max-width:340px){.dashboard-metrics{grid-template-columns:1fr}}.top-actions{display:flex;align-items:center;gap:clamp(6px,1vw,10px);flex:0 1 auto}.status-hero{height:44px;min-height:44px!important;padding:5px 11px!important;border-radius:999px!important;background:linear-gradient(180deg,var(--card-top),var(--card-bottom))!important;box-shadow:0 4px 0 rgba(0,0,0,.12),inset 0 1px 0 rgba(255,255,255,.1);gap:7px;flex:0 1 205px}.status-hero .dash-icon{width:27px;height:27px;flex:0 0 27px;font-size:14px}.status-hero .dash-copy{gap:0}.status-hero .dash-label{font-size:.55rem;letter-spacing:.045em}.status-hero .dash-value{font-size:.78rem;line-height:1.05}@media(max-width:620px){.top-actions{width:100%;justify-content:center;flex-wrap:wrap}.status-hero{flex:0 1 205px}}@media(max-width:420px){.top-actions{gap:7px}.status-hero{flex:1 1 160px;max-width:205px}}@media(max-width:340px){.top-actions{gap:5px}.status-hero{max-width:175px}}.top-actions{align-items:center}.status-hero{width:max-content;flex:0 0 auto;max-width:none}.output-controls{grid-template-columns:repeat(2,minmax(0,1fr))}.connection-line{display:flex;align-items:center;gap:7px;min-height:18px;margin:6px 2px 12px;color:var(--muted);font-size:.78rem}.connection-dot{width:8px;height:8px;flex:0 0 8px;border-radius:50%;background:#e8b322}.connection-line.live .connection-dot{background:#37d878;box-shadow:0 0 10px rgba(55,216,120,.45)}.connection-line.offline{color:#f04452}.connection-line.offline .connection-dot{background:#f04452}.manual-lock{margin:8px 0;padding:9px 12px;border-radius:12px;border:1px solid rgba(232,179,34,.28);background:rgba(232,179,34,.08);color:var(--muted);font-size:.84rem}@media(max-width:620px){main{padding-bottom:82px}#stopSeq{position:fixed;z-index:999;left:12px;right:12px;bottom:calc(10px + env(safe-area-inset-bottom));width:auto;min-height:48px;border-radius:16px;box-shadow:0 8px 24px rgba(0,0,0,.38),0 4px 0 #741326}}.top-actions{flex-wrap:nowrap;white-space:nowrap}@media(max-width:620px){.top-actions{width:100%;justify-content:space-between;flex-wrap:nowrap}}@media(max-width:420px){.top-actions{gap:5px}.top-actions .theme-toggle{padding:7px 9px;font-size:.82rem;gap:5px}.status-hero{padding:5px 8px!important;gap:4px}.status-hero .dash-icon{width:22px;height:22px;flex-basis:22px}.status-hero .dash-copy{flex-direction:row;align-items:baseline;gap:4px}.status-hero .dash-label{font-size:.48rem;letter-spacing:.02em}.status-hero .dash-value{font-size:.73rem}}@media(max-width:360px){#themeLabel{display:none}.top-actions .theme-toggle{padding:7px 8px}}\n"
  "@media(max-width:420px){main{padding:12px}.controls{grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.controls #stopSeq{grid-column:1/-1}.controls button{min-height:43px;padding:10px 8px;font-size:.88rem}.actions button{flex:1 1 145px}.card{border-radius:18px}.topbar{align-items:flex-start}.theme-toggle{min-height:40px;padding:8px 11px;font-size:.9rem}}\n"
  ".toast{position:fixed;z-index:10000;top:max(18px,env(safe-area-inset-top));right:max(16px,env(safe-area-inset-right));width:min(310px,calc(100vw - 32px));padding:13px 16px;border:1px solid rgba(127,235,177,.42);border-radius:14px;background:linear-gradient(145deg,rgba(17,54,39,.97),rgba(5,27,19,.97));color:#eafff1;box-shadow:0 14px 34px rgba(0,0,0,.32),inset 0 1px 0 rgba(255,255,255,.12);font-size:.92rem;font-weight:700;line-height:1.4;display:flex;align-items:center;gap:10px;opacity:0;transform:translateX(115%);pointer-events:none;transition:transform .34s cubic-bezier(.2,.8,.2,1),opacity .3s ease}.toast.show{opacity:1;transform:translateX(0)}.toast.hide{opacity:0;transform:translateX(115%)}.toast:before{content:\"✓\";width:23px;height:23px;flex:0 0 23px;border-radius:50%;display:grid;place-items:center;background:rgba(61,214,128,.2);color:#64ed9a}.toast.warn:before{content:\"!\";background:rgba(255,196,43,.2);color:#ffd34f}:root[data-theme=\"light\"] .toast{background:linear-gradient(145deg,rgba(247,255,249,.98),rgba(225,245,231,.98));color:#173727;border-color:#a5d9b4}\n"
  "@media(prefers-reduced-motion:reduce){button,.theme-toggle,body,.toast{transition:none}}\n"
  "</style></head><body><div id=\"toast\" class=\"toast\" role=\"status\" aria-live=\"polite\"></div><main>\n"
  "<div id=\"fullBrowserPrompt\" style=\"display:none;position:sticky;top:0;z-index:9999;margin:0 0 12px;padding:12px;border:1px solid #2fa36b;border-radius:12px;background:#10251d;color:#eafff3;box-shadow:0 4px 16px #0005\">\n"
  "  <div style=\"font-weight:700;margin-bottom:5px\">Open I.R.A. in your full browser</div>\n"
  "  <div style=\"font-size:13px;margin-bottom:10px\">Android may keep this page inside the Wi-Fi sign-in window. Tap below to try opening Chrome.</div>\n"
  "  <a id=\"openFullBrowser\" href=\"intent://10.10.10.10/#Intent;scheme=http;package=com.android.chrome;end\" style=\"display:inline-block;padding:10px 14px;border-radius:8px;background:#22a06b;color:#fff;font-weight:700;text-decoration:none\">Open in Chrome</a>\n"
  "  <a href=\"http://10.10.10.10/\" style=\"display:inline-block;margin-left:8px;color:#d5f7e5\">Open dashboard here</a>\n"
  "  "
  "</div>\n"
  "<script>\n"
  "(function(){\n"
  "  // Android captive-portal WebViews cannot be forced to launch Chrome. Offer an\n"
  "  // explicit Android intent link, with a normal local URL fallback.\n"
  "  var ua=navigator.userAgent||'';\n"
  "  var isAndroid=/Android/i.test(ua);\n"
  "  var isStandaloneChrome=/Chrome\\//i.test(ua)&&!/wv\\)|; wv\\)/i.test(ua);\n"
  "  if(isAndroid&&!isStandaloneChrome){var p=document.getElementById('fullBrowserPrompt');if(p)p.style.display='block';}\n"
  "})();\n"
  "</script>\n"
  "<div class=\"topbar\"><div class=\"topbar-main\"><div class=\"brand\"><div class=\"brandmark\" aria-hidden=\"true\">⌁</div><div><h1>Repair Center</h1><div class=\"muted\">I.R.A - Interactive Relay Automation</div></div></div><div class=\"top-actions\"><button type=\"button\" class=\"theme-toggle\" id=\"openAnalyzer\">▤ Analyzer</button><div class=\"dash-pill status-hero warn\" id=\"safetyPill\"><span class=\"dash-icon\" aria-hidden=\"true\">!</span><span class=\"dash-copy\"><span class=\"dash-label\">Controller state</span><span class=\"dash-value\" id=\"dashSafety\">Checking…</span></span></div><button type=\"button\" class=\"theme-toggle\" id=\"themeToggle\" aria-label=\"Switch color theme\"><span class=\"theme-icon\" id=\"themeIcon\" aria-hidden=\"true\">☀️</span><span id=\"themeLabel\">Light mode</span></button></div></div><div class=\"dashboard\" aria-label=\"Live system dashboard\"><div class=\"dashboard-metrics\"><div class=\"dash-pill\" id=\"wifiPill\"><span class=\"dash-icon\" aria-hidden=\"true\">⌁</span><span class=\"dash-copy\"><span class=\"dash-label\">Wi-Fi STA</span><span class=\"dash-value\" id=\"dashWifi\">Checking…</span></span></div><div class=\"dash-pill\" id=\"ramPill\"><span class=\"dash-icon\" aria-hidden=\"true\">▦</span><span class=\"dash-copy\"><span class=\"dash-label\">Free RAM / minimum</span><span class=\"dash-value\" id=\"dashRam\">—</span></span></div><div class=\"dash-pill good\" id=\"ipPill\"><span class=\"dash-icon\" aria-hidden=\"true\">⌖</span><span class=\"dash-copy\"><span class=\"dash-label\">Access point IP</span><span class=\"dash-value\" id=\"dashIp\">10.10.10.10</span></span></div><div class=\"dash-pill\" id=\"clientPill\"><span class=\"dash-icon\" aria-hidden=\"true\">◎</span><span class=\"dash-copy\"><span class=\"dash-label\">Clients</span><span class=\"dash-value\" id=\"dashClients\">0 connected</span></span></div></div></div></div>\n"
  "<div class=\"connection-line\" id=\"connectionLine\"><span class=\"connection-dot\" aria-hidden=\"true\"></span><span id=\"connectionState\">Connecting…</span></div>\n"
  "<div class=\"card\"><h2>Controls</h2><p class=\"muted\">Tap a control to toggle its output. Run Mode alternates Auto UP and Auto DOWN for the configured duration.</p>\n"
  "<div class=\"controls output-controls\" id=\"controls\">\n"
  "<button data-name=\"Brake\">Brake</button><button data-name=\"Enable\">Enable</button>\n"
  "<button data-name=\"Up\">Up</button><button data-name=\"Down\">Down</button>\n"
  "<button data-name=\"Speed 0\">Speed 0</button><button data-name=\"Speed 1\">Speed 1</button>\n"
  "<button data-name=\"Speed 2\">Speed 2</button><button class=\"run\" id=\"runMode\">Run Mode (1 hour)</button></div>\n"
  "<p id=\"manualLockNotice\" class=\"manual-lock\" hidden>Direction and speed controls are locked during an automatic sequence.</p>\n"
  "<h3>Automatic sequences</h3><div class=\"controls\">\n"
  "<button class=\"seq\" id=\"autoUp\">Auto UP</button><button class=\"seq\" id=\"autoDown\">Auto DOWN</button>\n"
  "<button class=\"stop\" id=\"stopSeq\">Stop sequence / Run Mode</button>\n"
  "</div><p class=\"muted\">Ready Signal: Switch to Timer; configured delay the relay turns ON. After Run Mode completes, the Buzzer relay pulses until Stop is pressed.</p><p id=\"seqStatus\" class=\"muted\">Sequence: Idle</p><div class=\"seq-progress\" aria-live=\"polite\"><div class=\"seq-progress-head\"><strong id=\"seqStepName\">Waiting for sequence</strong><span id=\"seqStepTime\">—</span></div><div class=\"seq-track\"><div id=\"seqProgressBar\" class=\"seq-fill\"></div></div><div class=\"seq-progress-foot\"><span id=\"seqDirection\">Direction: —</span><span id=\"seqStepDetail\">Step progress appears while running</span></div></div></div>\n"
  "<details class=\"card collapsible\"><summary>Sequence timer settings</summary><div class=\"collapse-body\"><p class=\"muted\">Choose each sequence-step duration, Buzzer pulse interval, Ready Signal delay (1–600 seconds), and Run Mode duration (1–24 hours). Settings are saved to ESP32 nonvolatile memory (NVS). The buzzer pulses briefly every selected interval after Run Mode completes; Stop silences it.</p>\n"
  "<form id=\"timerForm\"><div id=\"timerFields\" class=\"controls\"></div><div class=\"actions\"><button type=\"submit\">Save sequence timers</button><button type=\"button\" id=\"reloadTimers\">Reload saved timers</button></div></form><p id=\"timerMessage\" class=\"muted\"></p></div></details>\n"
  "<details class=\"card collapsible\"><summary>Wi-Fi configuration</summary><div class=\"collapse-body\">\n"
  "<p class=\"muted\">Configure the ESP32 access point and optional router connection. Settings are saved in nonvolatile memory (NVS).</p>\n"
  "<form method=\"post\" action=\"/apwifi\">\n"
  "<h3>ESP32 Access Point (AP)</h3>\n"
  "<label for=\"apSsid\">Access point name (SSID)</label><input id=\"apSsid\" name=\"apSsid\" maxlength=\"32\" required autocomplete=\"off\" placeholder=\"Repair Center\">\n"
  "<label for=\"apPassword\">Access point password</label><input id=\"apPassword\" name=\"apPassword\" type=\"password\" minlength=\"8\" maxlength=\"63\" required autocomplete=\"new-password\" placeholder=\"At least 8 characters\">\n"
  "<div class=\"actions\"><button type=\"submit\">Save &amp; restart access point</button></div>\n"
  "</form>\n"
  "<hr style=\"border:0;border-top:1px solid var(--line);margin:20px 0\">\n"
  "<h3>Router connection (STA)</h3><p class=\"muted\">Optional: connect the ESP32 to a router.</p>\n"
  "<form method=\"post\" action=\"/wifi\">\n"
  "<label for=\"ssid\">Router SSID</label><input id=\"ssid\" name=\"ssid\" maxlength=\"32\" autocomplete=\"off\" placeholder=\"Wi-Fi network name\">\n"
  "<label for=\"password\">Router password</label><input id=\"password\" name=\"password\" type=\"password\" maxlength=\"64\" autocomplete=\"new-password\" placeholder=\"Leave blank for an open router\">\n"
  "<div class=\"actions\"><button type=\"submit\">Save &amp; reconnect router</button></div></form>\n"
  "<p class=\"warn\">Changing AP settings disconnects connected clients. Reconnect using the new network name and password. The default password is 12345678; change it for your bench.</p></div></details>\n"
  "<details class=\"card collapsible\"><summary>Status</summary><div class=\"collapse-body\"><div id=\"status\" class=\"status\">Loading…</div></div></details>\n"
  "<details class=\"card collapsible\"><summary>Modbus RTU slave register map</summary><div class=\"collapse-body\"><p class=\"muted\">UART2 offers Modbus RTU slave, Passive Monitor, and Raw UART text modes. Only one is active at a time; choose in Protocol Analyzer. Passive Monitor never transmits. Use an RS-485 transceiver; connect its RO to GPIO16, DI to GPIO17, and tied DE+/RE to GPIO4. Configure your master to match the UART baud shown in Protocol Analyzer.</p><div class=\"table-wrap\"><table><thead><tr><th>Holding register</th><th>Meaning</th><th>Access / values</th></tr></thead><tbody><tr><td>0</td><td>Sequence command / last command</td><td>Read/write: 0 stop, 1 up, 2 down, 3 run</td></tr><tr><td>1</td><td>Sequence status</td><td>Read: 0 idle, 1 sequence, 2 run mode</td></tr><tr><td>2</td><td>Output bitfield</td><td>Read: bits 0–6 = Brake, Enable, Up, Down, Speed 0, Speed 1, Speed 2</td></tr><tr><td>3</td><td>Current step time remaining</td><td>Read, seconds</td></tr><tr><td>16–28 (0x0010–0x001C)</td><td>Sequence timer settings</td><td>Read/write: seconds, except Run Mode duration in hours</td></tr></tbody></table></div><p class=\"muted\">Supported functions: 03 read holding registers, 06 write single register, and 16 (0x10) write multiple timer registers. Timer writes are saved to NVS. Function 16 is for timers; use function 06 for commands.</p></div></details>\n"
  "<details class=\"card collapsible\" id=\"diagnosticsPanel\"><summary>Diagnostics &amp; controller health</summary><div class=\"collapse-body\"><p class=\"muted\">Read-only health snapshot, refreshed with the dashboard. These checks report software and interface status; they do not verify external relay contacts or inverter safety.</p><div class=\"diagnostics-grid\"><div class=\"diag-tile\"><span>Control task</span><strong id=\"diagControl\">Checking…</strong><small id=\"diagControlDetail\">Heartbeat</small></div><div class=\"diag-tile\"><span>Minimum free heap</span><strong id=\"diagHeap\">—</strong><small id=\"diagHeapDetail\">Since boot</small></div><div class=\"diag-tile\"><span>Wi-Fi access point</span><strong id=\"diagWifi\">—</strong><small id=\"diagWifiDetail\">Clients</small></div><div class=\"diag-tile\"><span>Router Wi-Fi (STA)</span><strong id=\"diagSta\">—</strong><small id=\"diagStaDetail\">IP status</small></div><div class=\"diag-tile\"><span>CAN / TWAI</span><strong id=\"diagCan\">—</strong><small id=\"diagCanDetail\">Driver and errors</small></div><div class=\"diag-tile\"><span>Analyzer buffer</span><strong id=\"diagBuffer\">—</strong><small id=\"diagBufferDetail\">Bounded ring buffer</small></div><div class=\"diag-tile\"><span>Control stack headroom</span><strong id=\"diagStack\">—</strong><small>FreeRTOS stack high-water mark</small></div><div class=\"diag-tile\"><span>Log losses</span><strong id=\"diagLoss\">—</strong><small id=\"diagLossDetail\">Overwrites / dropped UART bytes</small></div></div><p class=\"muted\" id=\"diagUpdated\">Waiting for first status update…</p></div></details>\n"
  "<section class=\"card\" id=\"analyzerView\" hidden>\n"
  "<div class=\"analyzer-head\"><div><h2>Protocol Analyzer</h2><p class=\"muted\">Buffered live receive monitor · timestamps are milliseconds since ESP32 boot</p></div><button type=\"button\" id=\"backDashboard\">← Back to Dashboard</button></div>\n"
  "<div class=\"analyzer-stats\"><div class=\"dash-pill\"><span class=\"dash-copy\"><span class=\"dash-label\">CAN RX frames</span><span class=\"dash-value\" id=\"anCanCount\">0</span></span></div><div class=\"dash-pill\"><span class=\"dash-copy\"><span class=\"dash-label\">Modbus RX frames</span><span class=\"dash-value\" id=\"anUartCount\">0</span></span></div><div class=\"dash-pill\"><span class=\"dash-copy\"><span class=\"dash-label\">CAN bus state</span><span class=\"dash-value\" id=\"anCanState\">Unknown</span></span></div><div class=\"dash-pill\"><span class=\"dash-copy\"><span class=\"dash-label\">CAN errors</span><span class=\"dash-value\" id=\"anCanErrors\">0</span></span></div><div class=\"dash-pill\"><span class=\"dash-copy\"><span class=\"dash-label\">Dropped log records</span><span class=\"dash-value\" id=\"anDropped\">0</span></span></div><div class=\"dash-pill\"><span class=\"dash-copy\"><span class=\"dash-label\">Last RX age</span><span class=\"dash-value\" id=\"anLastRx\">Never</span></span></div></div>\n"
  "<div class=\"analyzer-tabs\"><button type=\"button\" class=\"tab active\" data-an-tab=\"can\">CAN Console</button><button type=\"button\" class=\"tab\" data-an-tab=\"serial\">UART / Modbus</button></div>\n"
  "<div class=\"analyzer-toolbar\"><label class=\"checklabel\"><input type=\"checkbox\" id=\"anPause\"> Pause display</label><label class=\"checklabel\"><input type=\"checkbox\" id=\"anHex\" checked> Hex / raw view</label><label for=\"anFilter\">Filter text / CAN ID</label><input id=\"anFilter\" type=\"search\" maxlength=\"64\" placeholder=\"e.g. 0x123 or RX HEX\" style=\"max-width:240px\"><button type=\"button\" id=\"anRecord\">Start recording</button><span id=\"anRecordState\" class=\"recording-indicator\">Not recording</span><button type=\"button\" id=\"anClear\">Clear display</button><button type=\"button\" id=\"anExportTxt\">Export TXT</button><button type=\"button\" id=\"anExportCsv\">Export CSV</button></div>\n"
  "<div class=\"analyzer-toolbar\" id=\"canRateRow\"><label for=\"canRate\">CAN bitrate</label><select id=\"canRate\"><option value=\"125000\">125 kbit/s</option><option value=\"250000\">250 kbit/s</option><option value=\"500000\" selected>500 kbit/s</option><option value=\"1000000\">1 Mbit/s</option></select><button type=\"button\" id=\"applyCanRate\">Apply CAN bitrate</button></div>\n"
  "<div class=\"analyzer-toolbar\" id=\"serialRateRow\" hidden><label for=\"serialRate\">UART baud rate</label><select id=\"serialRate\"><option>1200</option><option>2400</option><option>4800</option><option>9600</option><option>19200</option><option>38400</option><option>57600</option><option selected>115200</option><option>230400</option><option>460800</option><option>921600</option></select><button type=\"button\" id=\"applySerialRate\">Apply UART baud</button></div>\n"
  "<div class=\"analyzer-toolbar\" id=\"uartModeRow\" hidden><label for=\"uartMode\">UART2 mode</label><select id=\"uartMode\"><option value=\"modbus\">Modbus RTU slave</option><option value=\"monitor\">Passive Modbus Monitor</option><option value=\"raw\">Raw UART text</option></select><button type=\"button\" id=\"applyUartMode\">Apply UART mode</button></div><div class=\"analyzer-toolbar\" id=\"canTxRow\"><label class=\"checklabel\"><input type=\"checkbox\" id=\"canTxArm\"> Enable CAN transmission (bench only)</label><div class=\"can-tx-grid\"><div><label for=\"canTxId\">CAN ID (hex)</label><input id=\"canTxId\" maxlength=\"8\" value=\"123\" placeholder=\"123\"></div><div><label for=\"canTxType\">Frame type</label><select id=\"canTxType\"><option value=\"std\">Standard (11-bit)</option><option value=\"ext\">Extended (29-bit)</option></select></div><div><label for=\"canTxData\">Payload hex (0–8 bytes)</label><input id=\"canTxData\" maxlength=\"23\" value=\"11 22 33 44\" placeholder=\"11 22 33 44\"></div><div style=\"display:flex;align-items:end\"><button type=\"button\" id=\"sendCanTx\" style=\"width:100%\">Transmit CAN</button></div></div></div><div class=\"analyzer-toolbar\" id=\"serialTxRow\" hidden><label for=\"serialTx\">Transmit text</label><input id=\"serialTx\" type=\"text\" maxlength=\"240\" placeholder=\"Enter UART command\" style=\"flex:1;min-width:180px\"><label class=\"checklabel\"><input type=\"checkbox\" id=\"serialTxNewline\" checked> Append CR/LF</label><button type=\"button\" id=\"sendSerialTx\">Transmit</button></div>\n"
  "<div class=\"console\" id=\"anConsole\" role=\"log\" aria-live=\"off\">Waiting for analyzer events…</div><p class=\"muted\" id=\"anInfo\">CAN transmission is enabled for controlled bench testing only. Verify the CAN ID, payload, bitrate, transceiver, and bus before transmitting. Modbus RTU uses the selected UART baud, 8N1, and slave ID 1.</p>\n"
  "</section>\n"
  "<script>\n"
  "// iOS captive-network windows can restore input focus and raise the keyboard.\n"
  "// Remove restored/programmatic focus, but permit an intentional user tap on a field.\n"
  "let iraUserRequestedInputFocusUntil = 0;\n"
  "function iraIsFormControl(el) {\n"
  "  return !!(el && el.matches && el.matches('input,textarea,select,[contenteditable=\"true\"]'));\n"
  "}\n"
  "function iraClearRestoredFocus() {\n"
  "  const active = document.activeElement;\n"
  "  if (iraIsFormControl(active) && Date.now() > iraUserRequestedInputFocusUntil) active.blur();\n"
  "}\n"
  "function iraNoteFormControlGesture(ev) {\n"
  "  if (iraIsFormControl(ev.target)) iraUserRequestedInputFocusUntil = Date.now() + 1500;\n"
  "}\n"
  "document.addEventListener('touchstart', iraNoteFormControlGesture, true);\n"
  "document.addEventListener('pointerdown', iraNoteFormControlGesture, true);\n"
  "document.addEventListener('mousedown', iraNoteFormControlGesture, true);\n"
  "document.addEventListener('focusin', ev => {\n"
  "  if (iraIsFormControl(ev.target) && Date.now() > iraUserRequestedInputFocusUntil) ev.target.blur();\n"
  "}, true);\n"
  "window.addEventListener('load', () => {\n"
  "  iraClearRestoredFocus();\n"
  "  setTimeout(iraClearRestoredFocus, 100);\n"
  "  setTimeout(iraClearRestoredFocus, 500);\n"
  "  setTimeout(iraClearRestoredFocus, 1200);\n"
  "});\n"
  "window.addEventListener('pageshow', () => {\n"
  "  iraClearRestoredFocus();\n"
  "  setTimeout(iraClearRestoredFocus, 250);\n"
  "});\n"
  "document.addEventListener('visibilitychange', () => {\n"
  "  if (document.visibilityState === 'visible') setTimeout(iraClearRestoredFocus, 100);\n"
  "});\n"
  "const root=document.documentElement;const themeToggle=document.getElementById('themeToggle');const themeIcon=document.getElementById('themeIcon');const themeLabel=document.getElementById('themeLabel');\n"
  "function setTheme(theme){root.dataset.theme=theme;const light=theme==='light';themeIcon.textContent=light?'🌙':'☀️';themeLabel.textContent=light?'Dark mode':'Light mode';themeToggle.setAttribute('aria-label',light?'Switch to dark theme':'Switch to light theme');try{localStorage.setItem('repairCenterTheme',theme);}catch(e){}}\n"  "let toastTimer=null,toastQueue=[],toastBusy=false;function showToast(message,kind=''){toastQueue.push([message,kind]);if(toastBusy)return;toastBusy=true;const toast=document.getElementById('toast');const next=()=>{if(!toastQueue.length){toastBusy=false;return;}const item=toastQueue.shift();toast.textContent=item[0];toast.classList.toggle('warn',item[1]==='warn');toast.classList.remove('hide');toast.classList.add('show');toastTimer=setTimeout(()=>{toast.classList.remove('show');toast.classList.add('hide');setTimeout(next,350);},1700);};next();}\n"  ""
  ""
  "let savedTheme='dark';try{savedTheme=localStorage.getItem('repairCenterTheme')||'dark';}catch(e){}setTheme(savedTheme==='light'?'light':'dark');themeToggle.addEventListener('click',()=>setTheme(root.dataset.theme==='dark'?'light':'dark'));\n"
  "const buttons=[...document.querySelectorAll('[data-name]')];let states={};let refreshTimer=null,lastSequenceActive=false,lastStaConnected=null,lastReadyRelayOn=null,statusErrorCount=0,serialRateInitialized=false,uartModeInitialized=false,activeUartMode='modbus';\n"
  "function setConnectionState(online){const el=document.getElementById('connectionLine');el.classList.toggle('live',online);el.classList.toggle('offline',!online);el.classList.remove('connecting');document.getElementById('connectionState').textContent=online?'Live · updated '+new Date().toLocaleTimeString():'Connection lost · retrying';}\n"
  "function applyManualLocks(){const lockedNames=['Up','Down','Speed 0','Speed 1','Speed 2'];buttons.forEach(b=>{const locked=lastSequenceActive&&lockedNames.includes(b.dataset.name);b.disabled=locked;b.setAttribute('aria-disabled',String(locked));});document.getElementById('manualLockNotice').hidden=!lastSequenceActive;}\n"  "function updateDiagnostics(s){const set=(id,value)=>{const el=document.getElementById(id);if(el)el.textContent=value;};const good=(id,ok)=>{const el=document.getElementById(id);if(el)el.style.color=ok?'var(--good, #36c878)':'var(--bad, #f87171)';};set('diagControl',s.controlTaskHealthy?'Running':'Heartbeat stale');good('diagControl',!!s.controlTaskHealthy);set('diagControlDetail','Last heartbeat '+(s.controlHeartbeatAgeMs??'—')+' ms ago');set('diagHeap',(Number(s.minFreeHeap||0)/1024).toFixed(1)+' KB');set('diagHeapDetail','Current '+(Number(s.freeHeap||0)/1024).toFixed(1)+' KB free');set('diagWifi',s.apActive?'Active':'Not active');good('diagWifi',!!s.apActive);set('diagWifiDetail',(s.apClients||0)+' connected client(s) · '+(s.apIp||'no IP'));set('diagSta',s.staConnected?'Connected':'Disconnected');good('diagSta',!!s.staConnected);set('diagStaDetail',s.staConnected?(s.staIp||'Connected'):'Optional router connection');const cs=s.canBusState===1?'Running':s.canBusState===2?'BUS-OFF':s.canBusState===3?'Recovering':(s.canDriverReady?'Unknown':'Stopped / unavailable');set('diagCan',cs);good('diagCan',s.canDriverReady&&s.canBusState===1);set('diagCanDetail',(s.canErrors||0)+' error(s) · '+(s.canRx||0)+' RX frames');set('diagBuffer',(s.analyzerCount||0)+' / '+(s.analyzerCapacity||80));good('diagBuffer',(s.analyzerCount||0)<(s.analyzerCapacity||80));set('diagBufferDetail','Ring buffer occupancy');set('diagStack',s.controlStackHighWaterWords==null?'Unavailable':s.controlStackHighWaterWords+' words');set('diagLoss',(s.analyzerDropped||0)+' analyzer');set('diagLossDetail',(s.uartDropped||0)+' UART byte(s) dropped');set('diagUpdated','Updated '+new Date().toLocaleTimeString());}\n"
  "async function refresh(){if(document.visibilityState!=='visible'){clearTimeout(refreshTimer);refreshTimer=setTimeout(refresh,10000);return;}let pollDelay=5000;try{const diagnosticsOpen=document.getElementById('diagnosticsPanel').open;const r=await fetch('/api/status'+(diagnosticsOpen?'':'?compact=1'),{cache:'no-store'});const s=await r.json();setConnectionState(true);if(!serialRateInitialized){document.getElementById('serialRate').value=String(s.uartBaud);serialRateInitialized=true;}if(!uartModeInitialized){activeUartMode=s.uartMode||'modbus';document.getElementById('uartMode').value=activeUartMode;updateUartModeControls();uartModeInitialized=true;}statusErrorCount=0;if(diagnosticsOpen)updateDiagnostics(s);states=s.outputs||{};lastSequenceActive=!!s.sequenceRunning;applyManualLocks();if(lastStaConnected!==null&&lastStaConnected!==!!s.staConnected)showToast(s.staConnected?'Router Wi-Fi connected':'Router Wi-Fi disconnected',s.staConnected?'':'warn');lastStaConnected=!!s.staConnected;if(lastReadyRelayOn!==null&&lastReadyRelayOn!==!!s.readyRelayOn&&s.readyRelayOn)showToast('Ready signal is ON');lastReadyRelayOn=!!s.readyRelayOn;\n"
  "buttons.forEach(b=>{const on=!!states[b.dataset.name];b.classList.toggle('on',on);b.setAttribute('aria-pressed',String(on));});\n"
  " const canRateField=document.getElementById('canRate');if(canRateField&&document.activeElement!==canRateField)canRateField.value=String(s.canBitrate||500000);const serialRateField=document.getElementById('serialRate');if(serialRateField&&document.activeElement!==serialRateField)serialRateField.value=String(s.uartBaud||115200);const apSsidField=document.getElementById('apSsid');if(apSsidField&&document.activeElement!==apSsidField)apSsidField.value=s.apSsid||'Repair Center';\n"
  " const wifiPill=document.getElementById('wifiPill');wifiPill.classList.toggle('good',!!s.staConnected);wifiPill.classList.toggle('bad',!s.staConnected);document.getElementById('dashWifi').textContent=s.staConnected?'Connected':'Not connected';document.getElementById('dashRam').textContent=(Number(s.freeHeap||0)/1024).toFixed(1)+' / '+(Number(s.minFreeHeap||0)/1024).toFixed(1)+' KB';const safetyPill=document.getElementById('safetyPill');document.getElementById('dashSafety').textContent=s.safetyState||'UNKNOWN';safetyPill.classList.toggle('good',s.safetyState==='READY');safetyPill.classList.toggle('warn',s.safetyState==='RUNNING');safetyPill.classList.toggle('bad',s.safetyState==='FAULT');document.getElementById('dashIp').textContent=s.apIp||'Unavailable';document.getElementById('dashClients').textContent=`${s.apClients} connected`;document.getElementById('clientPill').classList.toggle('good',Number(s.apClients)>0);document.getElementById('status').textContent=`AP: ${s.apSsid} (${s.apIp})\\nSTA: ${s.staConnected?'connected':'not connected'}${s.staConnected?' — '+s.staIp:''}\\nAP clients: ${s.apClients}\\nFree heap current/minimum: ${(Number(s.freeHeap||0)/1024).toFixed(1)} / ${(Number(s.minFreeHeap||0)/1024).toFixed(1)} KB\\nController state: ${s.safetyState||'UNKNOWN'}\\nCAN bus: ${['Unknown/stopped','Running','BUS-OFF','Recovering'][Number(s.canBusState)||0]}\\nAnalyzer dropped records: ${s.analyzerDropped??'Open diagnostics'}\\nBlue LED: ${s.apClients>0?'ON':'blinking'}\\nCAN RX frames: ${s.canRx??'—'}\\nModbus RX frames: ${s.modbusFrames??'—'}`;\n"
  " document.getElementById('seqStatus').textContent='Sequence: '+(s.sequenceStatus||'Idle')+(s.runModeActive?' — Run Mode active (up/down alternating; configured duration)':'');\n"
  " const rem=Math.max(0,Number(s.sequenceStepRemainingMs||0)),dur=Math.max(0,Number(s.sequenceStepDurationMs||0)),elapsed=Math.max(0,Number(s.sequenceStepElapsedMs||0));document.getElementById('seqStepName').textContent=s.sequenceRunning?(s.sequenceStatus||'Sequence running'):'Waiting for sequence';document.getElementById('seqStepTime').textContent=s.sequenceRunning?(Math.ceil(rem/1000)+' s remaining'):'—';document.getElementById('seqProgressBar').style.width=s.sequenceRunning&&dur?Math.min(100,elapsed/dur*100)+'%':'0%';document.getElementById('seqDirection').textContent='Direction: '+(String(s.sequenceStatus||'').toLowerCase().includes('down')?'DOWN':(String(s.sequenceStatus||'').toLowerCase().includes('up')?'UP':'—'));document.getElementById('seqStepDetail').textContent=s.sequenceRunning?('Elapsed '+Math.floor(elapsed/1000)+' s of '+Math.ceil(dur/1000)+' s'):'Progress appears while running';\n"
  " document.getElementById('autoUp').disabled=!!s.sequenceRunning;document.getElementById('autoDown').disabled=!!s.sequenceRunning;document.getElementById('runMode').disabled=!!s.sequenceRunning;\n"
  "}catch(e){setConnectionState(false);statusErrorCount++;pollDelay=Math.min(20000,5000*(statusErrorCount+1));document.getElementById('status').textContent='Controller not reachable. Reconnect to the Repair Center Wi-Fi, then open http://10.10.10.10/.';}finally{clearTimeout(refreshTimer);refreshTimer=setTimeout(refresh,document.visibilityState!=='visible'?10000:(lastSequenceActive?500:pollDelay));}}\n"
  "buttons.forEach(b=>{b.classList.add('manual-tactile');b.addEventListener('pointerdown',()=>{b.classList.add('pressed');if(navigator.vibrate)navigator.vibrate(18);});['pointerup','pointercancel','pointerleave'].forEach(ev=>b.addEventListener(ev,()=>b.classList.remove('pressed')));b.addEventListener('click',async()=>{b.disabled=true;try{const r=await fetch('/api/output?name='+encodeURIComponent(b.dataset.name)+'&state='+(states[b.dataset.name]?'0':'1'),{method:'POST'});if(!r.ok){const e=await r.json();alert(e.error||'Command rejected');}await refresh();setTimeout(refresh,100);}finally{applyManualLocks();}});});\n"
  "async function sequence(cmd){const r=await fetch('/api/sequence?cmd='+encodeURIComponent(cmd),{method:'POST'});if(!r.ok){const e=await r.json();alert(e.error||'Sequence command rejected');}else{showToast(cmd==='up'?'Auto UP accepted':cmd==='down'?'Auto DOWN accepted':cmd==='run'?'Run Mode accepted':'Stop command sent',cmd==='stop'?'warn':'');}await refresh();}\n"
  "document.getElementById('autoUp').addEventListener('click',()=>sequence('up'));\n"
  "document.getElementById('autoDown').addEventListener('click',()=>sequence('down'));\n"
  "document.getElementById('stopSeq').addEventListener('click',()=>sequence('stop'));\n"
  "document.getElementById('runMode').addEventListener('click',()=>sequence('run'));\n"
  "// Restore tactile press feedback for sequence controls (Auto UP, Auto DOWN, Run Mode, Stop).\n"
  "const sequenceActionButtons=['autoUp','autoDown','runMode','stopSeq'].map(id=>document.getElementById(id));\n"
  "sequenceActionButtons.forEach(b=>{\n"
  "  b.classList.add('sequence-tactile');\n"
  "  b.addEventListener('pointerdown',()=>{\n"
  "    if(b.disabled)return;\n"
  "    b.classList.add('pressed');\n"
  "    if(navigator.vibrate)navigator.vibrate(18);\n"
  "  });\n"
  "  ['pointerup','pointercancel','pointerleave','blur'].forEach(ev=>b.addEventListener(ev,()=>b.classList.remove('pressed')));\n"
  "  b.addEventListener('keydown',e=>{if(!b.disabled&&(e.key===' '||e.key==='Enter'))b.classList.add('pressed');});\n"
  "  b.addEventListener('keyup',()=>b.classList.remove('pressed'));\n"
  "});\n"
  "const timerNames=[['brake','Brake hold'],['enable','Enable delay'],['direction','Direction delay'],['speed0a','Speed 0 (first)'],['speed1a','Speed 1 (first)'],['speed2','Speed 2'],['speed1b','Speed 1 (return)'],['speed0b','Speed 0 (return)'],['dirOff','Direction off delay'],['enOff','Enable off delay'],['buzzerInterval','Buzzer pulse interval'],['readyDelay','Ready Signal delay (seconds)'],['runModeHours','Run Mode duration (hours)']];\n"
  "function timerOptions(selected,key){let h='';const ready=key==='readyDelay',hours=key==='runModeHours',max=ready?600:(hours?24:30),unit=hours?' hour':(ready?' second':' second');for(let n=1;n<=max;n++)h+=`<option value=\"${n}\" ${Number(selected)===n?'selected':''}>${n}${unit}${n===1?'':'s'}</option>`;return h;}\n"
  "function updateRunModeLabel(hours){const n=Math.max(1,Number(hours)||1);document.getElementById('runMode').textContent=`Run Mode (${n} hour${n===1?'':'s'})`;}\n"
  "async function loadTimers(){try{const r=await fetch('/api/timers',{cache:'no-store'});const t=await r.json();updateRunModeLabel(t.runModeHours);document.getElementById('timerFields').innerHTML=timerNames.map(([key,label])=>`<div><label for=\"t_${key}\">${label}</label><select id=\"t_${key}\" name=\"${key}\">${timerOptions(t[key]||(key==='readyDelay'?30:(key==='runModeHours'?1:2)),key)}</select></div>`).join('');document.getElementById('timerMessage').textContent='Loaded saved timer settings.';}catch(e){document.getElementById('timerMessage').textContent='Could not load timer settings.';}}\n"
  "document.getElementById('timerForm').addEventListener('submit',async e=>{e.preventDefault();const data=new URLSearchParams();timerNames.forEach(([key])=>data.set(key,document.getElementById('t_'+key).value));try{const r=await fetch('/api/timers',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data});const j=await r.json();if(r.ok)updateRunModeLabel(document.getElementById('t_runModeHours').value);document.getElementById('timerMessage').textContent=r.ok?'Timers saved to ESP32 NVS.':(j.error||'Save failed.');}catch(err){document.getElementById('timerMessage').textContent='Could not save timers.';}});\n"
  "document.getElementById('reloadTimers').addEventListener('click',loadTimers);\n"
  "// Analyzer view stays in the same browser tab; main controls and sequences keep running.\n"
  "const analyzerView=document.getElementById('analyzerView');\n"
  "const mainCards=[...document.querySelectorAll('main > .card')].filter(el=>el.id!=='analyzerView');\n"
  "const topbar=document.querySelector('.topbar');\n"
  "let analyzerSource='can', analyzerEvents=[], analyzerLastId=0, recording=false, recordedEvents=[], recordedFirstId=0;\n"
  "function showAnalyzer(show){if(show) iraClearRestoredFocus();mainCards.forEach(el=>el.hidden=show);topbar.hidden=show;analyzerView.hidden=!show;if(show){iraClearRestoredFocus();pollAnalyzer();}}\n"
  "document.getElementById('openAnalyzer').addEventListener('click',()=>showAnalyzer(true));\n"
  "// Give every Analyzer action button consistent tactile press/release feedback.\n"
  "const analyzerActionButtons=[...document.querySelectorAll('#analyzerView button, #openAnalyzer')];\n"
  "analyzerActionButtons.forEach(b=>{b.classList.add('analyzer-tactile');b.addEventListener('pointerdown',()=>{if(b.disabled)return;b.classList.add('pressed');if(navigator.vibrate)navigator.vibrate(18);});['pointerup','pointercancel','pointerleave','blur'].forEach(ev=>b.addEventListener(ev,()=>b.classList.remove('pressed')));b.addEventListener('keydown',e=>{if(e.key===' '||e.key==='Enter')b.classList.add('pressed');});b.addEventListener('keyup',()=>b.classList.remove('pressed'));});\n"
  "document.getElementById('backDashboard').addEventListener('click',()=>showAnalyzer(false));\n"
  "document.querySelectorAll('[data-an-tab]').forEach(b=>b.addEventListener('click',()=>{analyzerSource=b.dataset.anTab;document.querySelectorAll('[data-an-tab]').forEach(x=>x.classList.toggle('active',x===b));document.getElementById('canRateRow').hidden=analyzerSource!=='can';document.getElementById('serialRateRow').hidden=analyzerSource!=='serial';document.getElementById('uartModeRow').hidden=analyzerSource!=='serial';document.getElementById('serialTxRow').hidden=analyzerSource!=='serial';document.getElementById('canTxRow').hidden=analyzerSource!=='can';renderAnalyzer();pollAnalyzer(true);}));\n"
  "document.getElementById('anRecord').addEventListener('click',()=>{if(!recording)recordedEvents=[];recording=!recording;document.getElementById('anRecord').textContent=recording?'Stop recording':'Start recording';document.getElementById('anRecordState').textContent=recording?'Recording locally (max 2000 events)':'Not recording';});\n"
  "document.getElementById('anClear').addEventListener('click',()=>{analyzerEvents=[];renderAnalyzer();});\n"
  "document.getElementById('anHex').addEventListener('change',()=>renderAnalyzer());document.getElementById('anFilter').addEventListener('input',()=>renderAnalyzer());\n"
  "document.getElementById('anPause').addEventListener('change',()=>{if(!document.getElementById('anPause').checked)pollAnalyzer(true);});\n"
  "function visibleAnalyzerEvents(){return analyzerEvents.filter(e=>e.source===analyzerSource);}\n"
  "function downloadAnalyzer(content,type,filename){const url=URL.createObjectURL(new Blob([content],{type}));const a=document.createElement('a');a.href=url;a.download=filename;document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);}\n"
  "function csvCell(v){return '\"'+String(v??'').replace(/\"/g,'\"\"')+'\"';}\n"
  "document.getElementById('anExportTxt').addEventListener('click',()=>{const rows=(recordedEvents.length?recordedEvents:analyzerEvents).filter(e=>e.source===analyzerSource);const text=rows.map(e=>`${e.ms} ms [${e.source}] ${e.text}`).join('\\n');downloadAnalyzer(text,'text/plain;charset=utf-8',`repair-center-${analyzerSource}-log.txt`);});\n"
  "document.getElementById('anExportCsv').addEventListener('click',()=>{const rows=(recordedEvents.length?recordedEvents:analyzerEvents).filter(e=>e.source===analyzerSource);const csv=['timestamp_ms,source,message',...rows.map(e=>[e.ms,e.source,e.text].map(csvCell).join(','))].join('\\r\\n');downloadAnalyzer('\\uFEFF'+csv,'text/csv;charset=utf-8',`repair-center-${analyzerSource}-log.csv`);});\n"
  "async function applyAnalyzerRate(kind,value){const body=new URLSearchParams({kind,value:String(value)});const r=await fetch('/api/analyzer/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();document.getElementById('anInfo').textContent=r.ok?j.message:(j.error||'Could not apply setting.');if(r.ok&&kind==='can')document.getElementById('canRate').value=String(value);if(r.ok&&kind==='serial')document.getElementById('serialRate').value=String(value);}\n"
  "document.getElementById('applyCanRate').addEventListener('click',()=>applyAnalyzerRate('can',document.getElementById('canRate').value).catch(()=>document.getElementById('anInfo').textContent='CAN bitrate update failed.'));\n"
  "document.getElementById('applySerialRate').addEventListener('click',()=>applyAnalyzerRate('serial',document.getElementById('serialRate').value).catch(()=>document.getElementById('anInfo').textContent='UART baud update failed.'));\n"
  "function updateUartModeControls(){const raw=activeUartMode==='raw',monitor=activeUartMode==='monitor';document.getElementById('uartMode').value=activeUartMode;document.getElementById('serialTx').disabled=!raw;document.getElementById('sendSerialTx').disabled=!raw;document.getElementById('anInfo').textContent=raw?'Raw UART mode active. Modbus requests are not being processed.':(monitor?'Passive Modbus Monitor active. Frames are captured without replies or transmissions.':'Modbus RTU slave active (ID 1). Switch to Raw UART mode before transmitting text.');}\n"
  "document.getElementById('applyUartMode').addEventListener('click',async()=>{const mode=document.getElementById('uartMode').value;const body=new URLSearchParams({mode});try{const r=await fetch('/api/analyzer/mode',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();if(!r.ok)throw new Error(j.error||'Mode change failed');activeUartMode=mode;updateUartModeControls();}catch(e){document.getElementById('anInfo').textContent=e.message||'UART mode change failed.';}});\n"
  "async function transmitSerial(){const field=document.getElementById('serialTx');let data=field.value;if(document.getElementById('serialTxNewline').checked)data+='\\r\\n';if(!data.trim()){document.getElementById('anInfo').textContent='Enter UART text before transmitting.';return;}const btn=document.getElementById('sendSerialTx');btn.disabled=true;try{const body=new URLSearchParams({data});const r=await fetch('/api/analyzer/tx',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();document.getElementById('anInfo').textContent=r.ok?j.message:(j.error||'UART transmit failed.');if(r.ok){field.value='';await pollAnalyzer();}}catch(e){document.getElementById('anInfo').textContent='UART transmit failed; controller not reachable.';}finally{btn.disabled=activeUartMode!=='raw';}}\n"
  "document.getElementById('sendSerialTx').addEventListener('click',transmitSerial);document.getElementById('serialTx').addEventListener('keydown',e=>{if(e.key==='Enter'){e.preventDefault();transmitSerial();}});\n"
  "async function transmitCan(){if(!document.getElementById('canTxArm').checked){document.getElementById('anInfo').textContent='Enable CAN transmission first.';return;}const idText=document.getElementById('canTxId').value.trim().replace(/^0x/i,'');const type=document.getElementById('canTxType').value;const dataText=document.getElementById('canTxData').value.trim();if(!/^[0-9a-fA-F]{1,8}$/.test(idText)){document.getElementById('anInfo').textContent='Enter a valid hexadecimal CAN ID.';return;}if(dataText&&!/^(?:[0-9a-fA-F]{2})(?:[\\s,]+[0-9a-fA-F]{2}){0,7}$/.test(dataText)){document.getElementById('anInfo').textContent='Payload must contain 0–8 two-digit hex bytes, e.g. 11 22 33.';return;}const id=parseInt(idText,16);if((type==='std'&&id>0x7ff)||(type==='ext'&&id>0x1fffffff)){document.getElementById('anInfo').textContent='CAN ID exceeds the selected frame type range.';return;}const btn=document.getElementById('sendCanTx');btn.disabled=true;try{const body=new URLSearchParams({id:idText,type,data:dataText,armed:'1'});const r=await fetch('/api/analyzer/can-tx',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();document.getElementById('anInfo').textContent=r.ok?j.message:(j.error||'CAN transmit failed.');if(r.ok)await pollAnalyzer();}catch(e){document.getElementById('anInfo').textContent='CAN transmit failed; check controller and bus.';}finally{btn.disabled=false;}}\n"
  "document.getElementById('canTxArm').addEventListener('change',e=>{document.getElementById('sendCanTx').disabled=!e.target.checked;document.getElementById('anInfo').textContent=e.target.checked?'CAN transmit armed. Use only on a controlled bench bus.':'CAN transmit disarmed.';});document.getElementById('sendCanTx').disabled=true;document.getElementById('sendCanTx').addEventListener('click',transmitCan);\n"
  "async function pollAnalyzer(reset=false){if(analyzerView.hidden)return;if(document.getElementById('anPause').checked&&!reset)return;try{const r=await fetch('/api/analyzer?source='+analyzerSource+'&since='+(reset?0:analyzerLastId),{cache:'no-store'});const j=await r.json();document.getElementById('anCanCount').textContent=j.canRx;document.getElementById('anUartCount').textContent=j.modbusFrames;document.getElementById('anCanErrors').textContent=j.canErrors;document.getElementById('anCanState').textContent=j.canStateText||'Unknown';document.getElementById('anDropped').textContent=j.droppedLogs||0;document.getElementById('anLastRx').textContent=analyzerSource==='can'?(j.canLastRxMs?`${Math.max(0,Math.floor((j.uptimeMs-j.canLastRxMs)/1000))} s ago`:'Never'):(j.uartLastRxMs?`${Math.max(0,Math.floor((j.uptimeMs-j.uartLastRxMs)/1000))} s ago`:'Never');let changed=reset;if(reset){analyzerEvents=[];analyzerLastId=0;}for(const e of j.events||[]){if(!analyzerEvents.some(x=>x.id===e.id)){analyzerEvents.push(e);changed=true;if(recording){recordedEvents.push(e);if(recordedEvents.length>2000)recordedEvents.shift();}}analyzerLastId=Math.max(analyzerLastId,e.id);}if(analyzerEvents.length>300)analyzerEvents=analyzerEvents.slice(-300);if(changed)renderAnalyzer();}catch(e){document.getElementById('anInfo').textContent='Analyzer endpoint unavailable; check controller connection.';}}\n"
  "function renderAnalyzer(){const consoleEl=document.getElementById('anConsole');const term=(document.getElementById('anFilter')?.value||'').trim().toLowerCase();const filtered=analyzerEvents.filter(e=>e.source===analyzerSource&&(!term||(e.text+' '+e.source).toLowerCase().includes(term)));if(!filtered.length){consoleEl.textContent=analyzerSource==='can'?'Waiting for CAN frames…':'Waiting for UART bytes…';return;}const hexMode=document.getElementById('anHex').checked;const stickToBottom=consoleEl.scrollHeight-consoleEl.scrollTop-consoleEl.clientHeight<48;consoleEl.innerHTML=filtered.map(e=>{let text=e.text;if(e.source==='serial'&&!hexMode&&text.startsWith('RX HEX: ')){const bytes=text.slice(8).trim().split(/\\s+/).filter(Boolean);text='RX ASCII: '+bytes.map(h=>{const n=parseInt(h,16);return n>=32&&n<=126?String.fromCharCode(n):'.';}).join('');}return `<div class=\"${e.source}\"><span class=\"time\">+${e.ms} ms</span> ${e.source==='can'?'CAN':'UART'} ${escapeHtml(text)}</div>`;}).join('');if(stickToBottom)consoleEl.scrollTop=consoleEl.scrollHeight;}\n"
  "function escapeHtml(s){return String(s).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]));}\n"
  "document.getElementById('diagnosticsPanel').addEventListener('toggle',()=>{if(document.getElementById('diagnosticsPanel').open)refresh();});document.addEventListener('visibilitychange',()=>{if(document.visibilityState==='visible')refresh();});loadTimers();refresh();setInterval(()=>{if(!analyzerView.hidden)pollAnalyzer();},750);\n"
  "</script></main></body></html>\n";

// ---------------- HTTP handlers ----------------
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  const bool includeDiagnostics = server.arg("compact") != "1";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", makeStatusJson(includeDiagnostics));
}

void handleOutput() {
  if (!server.hasArg("name") || !server.hasArg("state")) {
    server.send(400, "application/json", "{\"error\":\"name and state required\"}");
    return;
  }
  const int index = outputIndexForName(server.arg("name"));
  const String stateArg = server.arg("state");
  if (index < 0 || (stateArg != "0" && stateArg != "1")) {
    server.send(400, "application/json", "{\"error\":\"invalid output or state\"}");
    return;
  }
  const bool requested = stateArg == "1";
  // During a sequence, manual direction/speed changes are locked; Brake and Enable remain independent.
  // Check this before changing any output so rejected commands have no side effects.
  if (sequenceRunning && index >= 2) {
    server.send(409, "application/json; charset=utf-8", "{\"error\":\"Direction and speed controls are locked during an automatic sequence\"}");
    return;
  }
  // Update selector groups under one lock so observers never see conflicting states.
  uint32_t mask = 1UL << index;
  uint32_t values = requested ? mask : 0;
  if (requested && (index == 2 || index == 3)) {
    mask |= (1UL << 2) | (1UL << 3);
    values = 1UL << index;
  } else if (requested && index >= 4 && index <= 6) {
    mask |= (1UL << 4) | (1UL << 5) | (1UL << 6);
    values = 1UL << index;
  }
  if (!applyOutputChanges(mask, values)) {
    server.send(503, "application/json; charset=utf-8", "{\"error\":\"Output state is busy; try again\"}");
    return;
  }
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void handleSequence() {
  if (!server.hasArg("cmd")) {
    server.send(400, "application/json; charset=utf-8", "{\"error\":\"cmd required: up, down, run, stop\"}");
    return;
  }
  String cmd = server.arg("cmd");
  cmd.toLowerCase();
  if (cmd == "stop") {
    queueSequenceRequest(3);
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"Sequence stopping\"}");
    return;
  }
  if (cmd != "up" && cmd != "down" && cmd != "run") {
    server.send(400, "application/json; charset=utf-8", "{\"error\":\"unknown sequence\"}");
    return;
  }
  if (!queueSequenceRequest((cmd == "up") ? 1 : (cmd == "down" ? 2 : 4))) {
    server.send(409, "application/json; charset=utf-8", "{\"error\":\"A sequence is already running\"}");
    return;
  }
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"Sequence accepted\"}");
}

void sendTimerJson() {
  uint32_t timerSnapshot[TIMER_COUNT];
  if (!timerMutex || xSemaphoreTake(timerMutex, pdMS_TO_TICKS(50)) != pdTRUE) { server.send(503, "application/json", "{\"error\":\"Timer settings busy\"}"); return; }
  memcpy(timerSnapshot, sequenceTimersMs, sizeof(timerSnapshot));
  xSemaphoreGive(timerMutex);
  String json = "{";
  for (size_t i = 0; i < TIMER_COUNT; ++i) {
    if (i) json += ',';
    json += "\"" + String(TIMER_KEYS[i]) + "\":" + String(i == T_RUN_MODE ? timerSnapshot[i] / 3600000UL : timerSnapshot[i] / 1000UL);
  }
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleTimersGet() { sendTimerJson(); }

void handleTimersSave() {
  uint32_t parsed[TIMER_COUNT];
  for (size_t i = 0; i < TIMER_COUNT; ++i) {
    if (!server.hasArg(TIMER_KEYS[i])) {
      server.send(400, "application/json; charset=utf-8", "{\"error\":\"All timer fields are required\"}");
      return;
    }
    const long value = server.arg(TIMER_KEYS[i]).toInt();
    const long maxValue = (i == T_READY_DELAY) ? 600 : ((i == T_RUN_MODE) ? 24 : 30);
    if (value < 1 || value > maxValue) {
      server.send(400, "application/json; charset=utf-8", "{\"error\":\"Timer value out of allowed range\"}");
      return;
    }
    parsed[i] = (i == T_RUN_MODE) ? ((uint32_t)value * 3600000UL) : ((uint32_t)value * 1000UL);
  }
  if (!timerMutex || xSemaphoreTake(timerMutex, pdMS_TO_TICKS(50)) != pdTRUE) { server.send(503, "application/json; charset=utf-8", "{\"error\":\"Timer settings busy\"}"); return; }
  for (size_t i = 0; i < TIMER_COUNT; ++i) sequenceTimersMs[i] = parsed[i];
  xSemaphoreGive(timerMutex);
  prefs.begin("seq-timers", false);
  for (size_t i = 0; i < TIMER_COUNT; ++i) {
    prefs.putUInt(TIMER_KEYS[i], i == T_RUN_MODE ? parsed[i] / 3600000UL : parsed[i] / 1000UL);
  }
  prefs.end();
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

// Forward declaration: analyzer configuration can restart the CAN driver before its definition below.
bool startCan();

void handleAnalyzerConfig() {
  if (!server.hasArg("kind") || !server.hasArg("value")) {
    server.send(400, "application/json; charset=utf-8", "{\"error\":\"kind and value required\"}"); return;
  }
  const String kind = server.arg("kind");
  const uint32_t value = (uint32_t)server.arg("value").toInt();
  if (kind == "serial") {
    const uint32_t valid[] = {1200,2400,4800,9600,19200,38400,57600,115200,230400,460800,921600};
    bool ok = false; for (uint32_t v : valid) if (v == value) ok = true;
    if (!ok) { server.send(400, "application/json", "{\"error\":\"Unsupported UART baud rate\"}"); return; }
    if (!uartMutex || xSemaphoreTake(uartMutex, pdMS_TO_TICKS(50)) != pdTRUE) { server.send(503, "application/json", "{\"error\":\"UART is busy; try again\"}"); return; }
    uartBaud = value;
    ExternalSerial.updateBaudRate(uartBaud);
    xSemaphoreGive(uartMutex);
    prefs.begin("analyzer", false); prefs.putUInt("uartBaud", uartBaud); prefs.end();
    addAnalyzerEvent("serial", "UART baud changed from web analyzer");
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"UART baud updated. The inverter port must use the same baud rate.\"}"); return;
  }
  if (kind == "can") {
    if (value != 125000 && value != 250000 && value != 500000 && value != 1000000) {
      server.send(400, "application/json", "{\"error\":\"Unsupported CAN bitrate\"}"); return;
    }
    if (sequenceRunning) { server.send(409, "application/json", "{\"error\":\"Stop the automatic sequence before changing CAN bitrate\"}"); return; }
    twai_stop(); twai_driver_uninstall();
    canBitrate = value;
    const bool ok = startCan();
    canDriverReady = ok;
    if (!ok) { server.send(500, "application/json", "{\"error\":\"CAN driver restart failed\"}"); return; }
    prefs.begin("analyzer", false); prefs.putUInt("canRate", canBitrate); prefs.end();
    addAnalyzerEvent("can", "CAN bitrate changed from web analyzer");
    String msg = "CAN bitrate updated to " + String(canBitrate / 1000) + " kbit/s. Configure all bus nodes to the same rate.";
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"" + msg + "\"}"); return;
  }
  server.send(400, "application/json", "{\"error\":\"kind must be can or serial\"}");
}

void handleAnalyzerMode() {
  if (!server.hasArg("mode")) { server.send(400, "application/json; charset=utf-8", "{\"error\":\"mode must be modbus, monitor or raw\"}"); return; }
  const String mode = server.arg("mode");
  if (mode != "modbus" && mode != "monitor" && mode != "raw") { server.send(400, "application/json; charset=utf-8", "{\"error\":\"mode must be modbus, monitor or raw\"}"); return; }
  if (!uartMutex || xSemaphoreTake(uartMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    server.send(503, "application/json; charset=utf-8", "{\"error\":\"UART is busy; try again\"}"); return;
  }
  modbusModeEnabled = mode == "modbus";
  passiveMonitorEnabled = mode == "monitor";
  ++uartModeGeneration;
  while (ExternalSerial.available()) ExternalSerial.read();
  xSemaphoreGive(uartMutex);
  const String message = modbusModeEnabled ? "Modbus RTU slave enabled (ID 1)." : (passiveMonitorEnabled ? "Passive Modbus monitor enabled. No Modbus replies or UART transmissions will be sent." : "Raw UART mode enabled. Modbus requests will not be processed.");
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"" + message + "\"}");
}

void handleAnalyzerTx() {
  if (!server.hasArg("data")) { server.send(400, "application/json; charset=utf-8", "{\"error\":\"data field required\"}"); return; }
  String data = server.arg("data");
  if (data.isEmpty() || data.length() > 242) { server.send(400, "application/json; charset=utf-8", "{\"error\":\"Transmit text must be 1–242 bytes including line ending\"}"); return; }
  if (!uartMutex || xSemaphoreTake(uartMutex, pdMS_TO_TICKS(50)) != pdTRUE) { server.send(503, "application/json; charset=utf-8", "{\"error\":\"UART is busy; try again\"}"); return; }
  if (modbusModeEnabled || passiveMonitorEnabled) {
    xSemaphoreGive(uartMutex);
    server.send(409, "application/json; charset=utf-8", "{\"error\":\"Switch UART2 to Raw UART mode before sending text\"}"); return;
  }
  digitalWrite(PIN_MODBUS_DE, HIGH);
  delayMicroseconds(50);
  const size_t written = ExternalSerial.write((const uint8_t *)data.c_str(), data.length());
  ExternalSerial.flush();
  delayMicroseconds(50);
  digitalWrite(PIN_MODBUS_DE, LOW);
  xSemaphoreGive(uartMutex);
  if (written != data.length()) { server.send(500, "application/json; charset=utf-8", "{\"error\":\"UART transmit was incomplete\"}"); return; }
  String logged = "TX ASCII: ";
  for (size_t i = 0; i < data.length(); ++i) {
    const uint8_t c = (uint8_t)data[i];
    if (c == '\r') logged += "\\r";
    else if (c == '\n') logged += "\\n";
    else if (c >= 32 && c <= 126) logged += (char)c;
    else logged += '.';
  }
  addAnalyzerEvent("serial", logged.c_str());
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"UART text transmitted.\"}");
}

void handleAnalyzerCanTx() {
  // Require explicit browser-side arm state as well as conservative rate limiting.
  if (!server.hasArg("armed") || server.arg("armed") != "1") { server.send(403, "application/json; charset=utf-8", "{\"error\":\"Enable CAN transmission in the analyzer before sending\"}"); return; }
  // This is a bench tool, not a high-rate CAN generator.
  const uint32_t now = millis();
  if (now - lastCanTransmitMs < 100) {
    server.send(429, "application/json; charset=utf-8", "{\"error\":\"CAN transmit rate limited; wait at least 100 ms between frames\"}"); return;
  }
  if (!canDriverReady || canBusState != 1) {
    server.send(409, "application/json; charset=utf-8", "{\"error\":\"CAN driver is not in RUNNING state; check transceiver, bus and bitrate\"}"); return;
  }
  if (!server.hasArg("id") || !server.hasArg("type") || !server.hasArg("data")) {
    server.send(400, "application/json; charset=utf-8", "{\"error\":\"id, type and data are required\"}"); return;
  }
  String idText = server.arg("id"); idText.trim(); if (idText.startsWith("0x") || idText.startsWith("0X")) idText.remove(0, 2);
  if (idText.isEmpty() || idText.length() > 8) { server.send(400, "application/json; charset=utf-8", "{\"error\":\"CAN ID must be 1–8 hexadecimal digits\"}"); return; }
  uint32_t id = 0;
  for (size_t i=0; i<idText.length(); ++i) { char c=idText[i]; uint8_t v=(c>='0'&&c<='9')?c-'0':((c>='a'&&c<='f')?c-'a'+10:((c>='A'&&c<='F')?c-'A'+10:255)); if(v>15){server.send(400,"application/json; charset=utf-8","{\"error\":\"CAN ID contains non-hex characters\"}");return;} id=(id<<4)|v; }
  const String type = server.arg("type"); const bool ext = type == "ext";
  if ((!ext && type != "std") || (!ext && id > 0x7FF) || (ext && id > 0x1FFFFFFF)) { server.send(400, "application/json; charset=utf-8", "{\"error\":\"CAN ID is outside the selected frame type range\"}"); return; }
  String payload = server.arg("data"); payload.trim();
  twai_message_t msg = {};
  msg.identifier = id; msg.extd = ext ? 1 : 0; msg.rtr = 0; msg.data_length_code = 0;
  int pos = 0;
  while (pos < payload.length()) {
    while (pos < payload.length() && (payload[pos]==' ' || payload[pos]==',' || payload[pos]=='\t')) ++pos;
    if (pos >= payload.length()) break;
    if (msg.data_length_code >= 8 || pos + 1 >= payload.length()) { server.send(400, "application/json; charset=utf-8", "{\"error\":\"Payload must contain 0–8 complete hex bytes\"}"); return; }
    auto hexNibble = [](char c)->int { if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;return -1; };
    int hi=hexNibble(payload[pos]), lo=hexNibble(payload[pos+1]); if(hi<0||lo<0){server.send(400,"application/json; charset=utf-8","{\"error\":\"Payload bytes must be two-digit hexadecimal values\"}");return;}
    msg.data[msg.data_length_code++] = (uint8_t)((hi<<4)|lo); pos += 2;
    if (pos < payload.length() && payload[pos] != ' ' && payload[pos] != ',' && payload[pos] != '\t') { server.send(400,"application/json; charset=utf-8","{\"error\":\"Separate payload bytes with spaces or commas\"}");return; }
  }
  esp_err_t err = twai_transmit(&msg, 0);
  if (err != ESP_OK) { server.send(503, "application/json; charset=utf-8", "{\"error\":\"TWAI could not queue the frame; check bus state and TX load\"}"); return; }
  lastCanTransmitMs = now;
  char logText[96]; int used=snprintf(logText,sizeof(logText),"TX %s ID=0x%lX DLC=%u DATA=",ext?"EXT":"STD",(unsigned long)id,msg.data_length_code);
  for(uint8_t i=0;i<msg.data_length_code&&used>0&&(size_t)used<sizeof(logText);++i) used+=snprintf(logText+used,sizeof(logText)-(size_t)used,"%s%02X",i?" ":"",msg.data[i]);
  addAnalyzerEvent("can", logText);
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"CAN frame queued to TWAI driver. Check CAN bus state and analyzer for observed traffic; queueing does not guarantee ACK or delivery.\"}");
}

void handleAnalyzer() {
  String source = server.hasArg("source") ? server.arg("source") : "can";
  if (source != "can" && source != "serial") {
    server.send(400, "application/json; charset=utf-8", "{\"error\":\"source must be can or serial\"}");
    return;
  }
  const uint32_t since = server.hasArg("since") ? (uint32_t)server.arg("since").toInt() : 0;
  const char *canStateText = !canDriverReady ? "STOPPED" : (canBusState == 2 ? "BUS-OFF" : (canBusState == 3 ? "RECOVERING" : (canBusState == 1 ? "RUNNING" : "UNKNOWN")));
  String json = "{\"canRx\":" + String(canRxCount) + ",\"uartRx\":" + String(uartRxCount) + ",\"modbusFrames\":" + String(modbusRxCount) + ",\"modbusErrors\":" + String(modbusErrorCount) + ",\"canErrors\":" + String(canErrorCount) + ",\"droppedLogs\":" + String(analyzerDroppedLogs) + ",\"canLastRxMs\":" + String(canLastRxMs) + ",\"uartLastRxMs\":" + String(uartLastRxMs) + ",\"uptimeMs\":" + String(millis()) + ",\"canStateText\":\"" + String(canStateText) + "\",\"events\":[";
  bool first = true;
  if (analyzerMutex && xSemaphoreTake(analyzerMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    const size_t start = (analyzerWriteIndex + ANALYZER_LOG_CAPACITY - analyzerCount) % ANALYZER_LOG_CAPACITY;
    for (size_t n = 0; n < analyzerCount; ++n) {
      const AnalyzerEvent &e = analyzerLog[(start + n) % ANALYZER_LOG_CAPACITY];
      if (e.id <= since || source != e.source) continue;
      if (!first) json += ',';
      first = false;
      json += "{\"id\":" + String(e.id) + ",\"ms\":" + String(e.ms) + ",\"source\":\"" + String(e.source) + "\",\"text\":\"" + jsonEscape(String(e.text)) + "\"}";
    }
    xSemaphoreGive(analyzerMutex);
  }
  json += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", json);
}

void handleApWifiSave() {
  if (!server.hasArg("apSsid") || !server.hasArg("apPassword")) {
    server.send(400, "text/plain; charset=utf-8", "Access point name and password are required.");
    return;
  }
  String newSSID = server.arg("apSsid");
  String newPassword = server.arg("apPassword");
  newSSID.trim();
  if (newSSID.isEmpty() || newSSID.length() > 32) {
    server.send(400, "text/plain; charset=utf-8", "Access point name must be 1–32 characters.");
    return;
  }
  if (newPassword.length() < 8 || newPassword.length() > 63) {
    server.send(400, "text/plain; charset=utf-8", "Access point password must be 8–63 characters.");
    return;
  }

  prefs.begin(PREF_NAMESPACE, false);
  prefs.putString("apSsid", newSSID);
  prefs.putString("apPass", newPassword);
  prefs.end();
  apSSID = newSSID;
  apPassword = newPassword;

  server.send(200, "text/html; charset=utf-8",
    "<!doctype html><html lang='en'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<body style='font-family:system-ui;background:#101827;color:white;padding:2rem'><h2>Access point settings saved</h2>"
    "<p>The access point is restarting. Reconnect your phone to the Wi-Fi network name and password you just saved. If Android asks, choose to stay connected without internet. Then open <code>http://10.10.10.10/</code> in your browser.</p>"
    "<p><a style='color:#8bd5ff' href='http://10.10.10.10/'>Return to controls</a></p></body></html>");
  delay(250);
  WiFi.softAPdisconnect(true);
  delay(150);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(apSSID.c_str(), apPassword.c_str());
}

void handleWifiSave() {
  if (!server.hasArg("ssid") || !server.hasArg("password")) {
    server.send(400, "text/plain; charset=utf-8", "SSID and password fields are required.");
    return;
  }
  staSSID = server.arg("ssid");
  staPassword = server.arg("password");
  staSSID.trim();
  if (staSSID.length() > 32 || staPassword.length() > 64) {
    server.send(400, "text/plain; charset=utf-8", "SSID or password is too long.");
    return;
  }
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putString("ssid", staSSID);
  prefs.putString("password", staPassword);
  prefs.end();

  // Respond before reconnecting, so the browser receives confirmation.
  server.send(200, "text/html; charset=utf-8",
    "<!doctype html><html lang='en'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<body style='font-family:system-ui;background:#101827;color:white;padding:2rem'><h2>Wi-Fi settings saved</h2>"
    "<p>Your router connection is being restarted. The Repair Center access point should remain available. If this page closes, reconnect to Repair Center Wi-Fi and open <code>http://10.10.10.10/</code>.</p>"
    "<p><a style='color:#8bd5ff' href='/'>Return to controls</a></p></body></html>");
  delay(150);
  WiFi.disconnect(false, false);
  if (staSSID.length()) WiFi.begin(staSSID.c_str(), staPassword.c_str());
}

// ---------------- FreeRTOS tasks ----------------
void controlTask(void *parameter) {
  enum Stage : uint8_t { IDLE, BRAKE_WAIT, ENABLE_WAIT, DIRECTION_WAIT, SPEED0_A, SPEED1_A,
                         SPEED2_HOLD, SPEED1_B, SPEED0_B, DIRECTION_OFF_WAIT, ENABLE_OFF_WAIT,
                         RUNMODE_BRAKE_OFF_WAIT };
  Stage stage = IDLE;
  uint32_t deadline = 0;
  int directionIndex = 2;
  bool buttonLastReading = HIGH;
  bool buttonStableState = HIGH;
  uint32_t buttonLastChangeMs = 0;
  auto allSpeedsOff = []() { applyOutputChanges((1UL << 4) | (1UL << 5) | (1UL << 6), 0); };
  auto setOnlySpeed = [](int index) { applyOutputChanges((1UL << 4) | (1UL << 5) | (1UL << 6), 1UL << index); };
  auto beginStage = [&](Stage next, uint32_t durationMs, const char *label) {
    stage = next; const uint32_t started = millis(); deadline = started + durationMs; updateSequenceStep(started, durationMs, label);
  };
  auto beginCycle = [&](int direction) {
    directionIndex = direction;
    applyOutputChanges(0x7D, 1UL << 0); // clear directions/speeds and engage brake atomically
    beginStage(BRAKE_WAIT, getSequenceTimerMs(T_BRAKE), directionIndex == 2 ? "Auto UP: Brake hold" : "Auto DOWN: Brake hold");
    setSequenceRunning(true);
  };
  for (;;) {
    controlHeartbeatMs = millis();
    const uint32_t now = millis();

    // READY SIGNAL STATE MACHINE:
    // 1) Startup state is OFF (active-HIGH relay: LOW = OFF).
    // 2) A debounced press starts the configured delay; relay remains OFF.
    // 3) If the button is still held when the delay expires, relay turns ON.
    // 4) A raw release immediately turns the relay OFF and cancels the delay.
    // 5) Release synchronizes button state so the next press starts a fresh delay.
    const bool buttonReading = digitalRead(PIN_READY_BUTTON); // INPUT_PULLUP: LOW = pressed
    if (buttonReading != buttonLastReading) {
      buttonLastReading = buttonReading;
      buttonLastChangeMs = now;
    }

    // Do not debounce the release safety path: release must switch OFF now.
    if (buttonReading == HIGH) {
      // Release is authoritative: cancel any pending delay and force relay OFF.
      // Synchronize the debounced state as well, so a quick subsequent press
      // can start a fresh delay cleanly.
      readyDelayPending = false;
      buttonStableState = HIGH;
      readyRelayOn = false;
      digitalWrite(PIN_READY_RELAY, LOW); // active-HIGH relay OFF immediately on release
    }

    // Debounce only the press edge before starting the timer.
    if ((uint32_t)(now - buttonLastChangeMs) >= 35UL && buttonStableState != buttonReading) {
      buttonStableState = buttonReading;
      if (buttonStableState == LOW && !readyRelayOn && !readyDelayPending) {
        readyDelayPending = true;
        readyDelayStartMs = now;
      }
    }

    // Relay activates only after the full delay while the physical button is held.
    if (readyDelayPending && buttonReading == LOW && buttonStableState == LOW &&
        (uint32_t)(now - readyDelayStartMs) >= getSequenceTimerMs(T_READY_DELAY)) {
      readyDelayPending = false;
      readyRelayOn = true;
      digitalWrite(PIN_READY_RELAY, HIGH); // active-HIGH relay ON
    }

    // Brief buzzer relay pulse at a configurable interval after Run Mode completes.
    if (buzzerAfterRunMode) {
      if (!buzzerRelayOn && (int32_t)(now - buzzerNextPulseMs) >= 0) {
        buzzerRelayOn = true;
        buzzerPulseEndMs = now + BUZZER_PULSE_MS;
        digitalWrite(PIN_BUZZER_RELAY, HIGH);
      } else if (buzzerRelayOn && (int32_t)(now - buzzerPulseEndMs) >= 0) {
        buzzerRelayOn = false;
        digitalWrite(PIN_BUZZER_RELAY, LOW);
        buzzerNextPulseMs = now + getSequenceTimerMs(T_BUZZER_INTERVAL);
      }
    } else if (buzzerRelayOn) {
      buzzerRelayOn = false;
      digitalWrite(PIN_BUZZER_RELAY, LOW);
    }

    const uint8_t request = takeSequenceRequest();
    if (request == 3) {
      stage = IDLE; setSequenceRunning(false); runModeActive = false; clearSequenceStep(); buzzerAfterRunMode = false; buzzerRelayOn = false; digitalWrite(PIN_BUZZER_RELAY, LOW);
      // Stop all motion outputs and speed selections; de-energize outputs to a safe software state.
      applyOutputChanges((1UL << OUTPUT_COUNT) - 1UL, 0);
      setSequenceStatus("Stopped — outputs OFF");
    } else if ((request == 1 || request == 2 || request == 4) && stage == IDLE) {
      runModeActive = (request == 4);
      if (runModeActive) { buzzerAfterRunMode = false; buzzerRelayOn = false; digitalWrite(PIN_BUZZER_RELAY, LOW); }
      runModeEndMs = runModeActive ? now + getSequenceTimerMs(T_RUN_MODE) : 0;
      beginCycle(request == 2 ? 3 : 2);
      if (runModeActive) setSequenceStatus(String("Run Mode: Auto UP (") + String(getSequenceTimerMs(T_RUN_MODE) / 3600000UL) + " hour(s))");
    }
    if (runModeActive && (int32_t)(now - runModeEndMs) >= 0) {
      stage = IDLE; setSequenceRunning(false); runModeActive = false; clearSequenceStep();
      applyOutputChanges((1UL << OUTPUT_COUNT) - 1UL, 0);
      setSequenceStatus("Run Mode complete — outputs OFF");
      buzzerAfterRunMode = true; buzzerNextPulseMs = now;
    }
    if (stage != IDLE && (int32_t)(now - deadline) >= 0) {
      switch (stage) {
        case BRAKE_WAIT:
          setOutputState(1, true); beginStage(ENABLE_WAIT, getSequenceTimerMs(T_ENABLE), "Enable ON"); break;
        case ENABLE_WAIT:
          setOutputState((size_t)directionIndex, true); beginStage(DIRECTION_WAIT, getSequenceTimerMs(T_DIRECTION), directionIndex == 2 ? "Up ON" : "Down ON"); break;
        case DIRECTION_WAIT:
          setOnlySpeed(4); beginStage(SPEED0_A, getSequenceTimerMs(T_SPEED0_A), "Speed 0 (first)"); break;
        case SPEED0_A:
          setOnlySpeed(5); beginStage(SPEED1_A, getSequenceTimerMs(T_SPEED1_A), "Speed 1 (first)"); break;
        case SPEED1_A:
          setOnlySpeed(6); beginStage(SPEED2_HOLD, getSequenceTimerMs(T_SPEED2), "Speed 2"); break;
        case SPEED2_HOLD:
          setOnlySpeed(5); beginStage(SPEED1_B, getSequenceTimerMs(T_SPEED1_B), "Speed 1 (return)"); break;
        case SPEED1_B:
          setOnlySpeed(4); beginStage(SPEED0_B, getSequenceTimerMs(T_SPEED0_B), "Speed 0 (return)"); break;
        case SPEED0_B:
          allSpeedsOff(); setOutputState((size_t)directionIndex, false);
          beginStage(DIRECTION_OFF_WAIT, getSequenceTimerMs(T_DIRECTION_OFF), "Direction OFF delay"); break;
        case DIRECTION_OFF_WAIT:
          setOutputState(1, false); beginStage(ENABLE_OFF_WAIT, getSequenceTimerMs(T_ENABLE_OFF), "Enable OFF delay"); break;
        case ENABLE_OFF_WAIT:
          setOutputState(0, false);
          if (runModeActive && (int32_t)(millis() - runModeEndMs) < 0) {
            // Keep the brake de-energized for 2 seconds between Run Mode directions.
            setOutputState(0, false);
            beginStage(RUNMODE_BRAKE_OFF_WAIT, 2000, "Run Mode: Brake OFF pause (2 s)");
          } else {
            stage = IDLE; setSequenceRunning(false);
            if (runModeActive) { runModeActive = false; setSequenceStatus("Run Mode complete — outputs OFF"); buzzerAfterRunMode = true; buzzerNextPulseMs = millis(); }
            else setSequenceStatus("Completed");
          }
          break;
        case RUNMODE_BRAKE_OFF_WAIT:
          if (runModeActive && (int32_t)(millis() - runModeEndMs) < 0) {
            const int nextDirection = directionIndex == 2 ? 3 : 2;
            beginCycle(nextDirection);
            setSequenceStatus(nextDirection == 2 ? "Run Mode: Auto UP" : "Run Mode: Auto DOWN");
          } else {
            stage = IDLE; setSequenceRunning(false); runModeActive = false; clearSequenceStep();
            applyOutputChanges((1UL << OUTPUT_COUNT) - 1UL, 0);
            setSequenceStatus("Run Mode complete — outputs OFF");
            buzzerAfterRunMode = true; buzzerNextPulseMs = millis();
          }
          break;
        case IDLE: break;
      }
    }
    bool local[OUTPUT_COUNT];
    copyOutputStates(local);
    for (size_t i = 0; i < OUTPUT_COUNT; ++i) digitalWrite(OUTPUTS[i].pin, local[i] ? HIGH : LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void communicationTask(void *parameter) {
  uint8_t modbusBuffer[MODBUS_MAX_ADU];
  size_t modbusLength = 0;
  size_t modbusExpectedLength = 0;
  uint32_t seenUartModeGeneration = uartModeGeneration;
  bool canRecoveryPending = false;
  uint32_t lastCanRecoveryAttemptMs = 0;
  twai_message_t message;
  for (;;) {
    // Batch UART bytes into bounded log records; one event per chunk avoids
    // flooding the ring buffer at high baud rates.
    char uartBatch[80];
    size_t uartBatchLen = 0;
    // UART2 is dedicated to Modbus RTU; bytes are framed and checked before requests are handled.
    if (uartMutex && xSemaphoreTake(uartMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (seenUartModeGeneration != uartModeGeneration) {
        modbusLength = modbusExpectedLength = 0;
        seenUartModeGeneration = uartModeGeneration;
      }
      while (ExternalSerial.available() && uartBatchLen < 48) {
        const int value = ExternalSerial.read();
        if (value < 0) break;
        ++uartRxCount;
        uartLastRxMs = millis();
        if (modbusModeEnabled) modbusProcessByte((uint8_t)value, modbusBuffer, modbusLength, modbusExpectedLength);
        else if (passiveMonitorEnabled) passiveModbusProcessByte((uint8_t)value, modbusBuffer, modbusLength, modbusExpectedLength);
        const size_t room = sizeof(uartBatch) - uartBatchLen;
        if (room > 5) {
          const int wrote = snprintf(uartBatch + uartBatchLen, room, "%s%02X", uartBatchLen ? " " : "", (unsigned)(uint8_t)value);
          if (wrote > 0) uartBatchLen += (size_t)wrote;
        }
      }
      xSemaphoreGive(uartMutex);
    }
    // Keep raw byte chunks only in raw UART mode; Modbus modes log validated frames instead.
    if (uartBatchLen && !modbusModeEnabled && !passiveMonitorEnabled) {
      char uartEvent[96];
      snprintf(uartEvent, sizeof(uartEvent), "RX HEX: %s", uartBatch);
      addAnalyzerEvent("serial", uartEvent);
    }

    // CAN receive endpoint. Application-specific CAN IDs/payloads were not
    // specified, so received frames are counted but do not change outputs.
    if (twai_receive(&message, 0) == ESP_OK) {
      ++canRxCount;
      canLastRxMs = millis();
      char frame[96];
      int used = snprintf(frame, sizeof(frame), "%s ID=0x%08lX DLC=%u DATA=", message.extd ? "EXT" : "STD", (unsigned long)message.identifier, message.data_length_code);
      for (uint8_t i = 0; i < message.data_length_code && i < 8 && used > 0 && (size_t)used < sizeof(frame); ++i) {
        used += snprintf(frame + used, sizeof(frame) - (size_t)used, "%s%02X", i ? " " : "", message.data[i]);
      }
      addAnalyzerEvent("can", frame);
    }
    twai_status_info_t twaiStatus;
    if (twai_get_status_info(&twaiStatus) == ESP_OK) {
      canErrorCount = twaiStatus.bus_error_count + twaiStatus.tx_error_counter + twaiStatus.rx_error_counter;
      canBusState = (twaiStatus.state == TWAI_STATE_RUNNING) ? 1 : ((twaiStatus.state == TWAI_STATE_BUS_OFF) ? 2 : ((twaiStatus.state == TWAI_STATE_RECOVERING) ? 3 : 0));
      const uint32_t statusNow = millis();
      if (twaiStatus.state == TWAI_STATE_BUS_OFF && !canRecoveryPending &&
          (uint32_t)(statusNow - lastCanRecoveryAttemptMs) >= 1000UL) {
        lastCanRecoveryAttemptMs = statusNow;
        const esp_err_t recovery = twai_initiate_recovery();
        if (recovery == ESP_OK) {
          canRecoveryPending = true;
          addAnalyzerEvent("can", "Bus-off detected; TWAI recovery started");
        } else {
          addAnalyzerEvent("can", "TWAI recovery request failed; will retry");
        }
      } else if (twaiStatus.state == TWAI_STATE_STOPPED && canRecoveryPending &&
                 (uint32_t)(statusNow - lastCanRecoveryAttemptMs) >= 1000UL) {
        lastCanRecoveryAttemptMs = statusNow;
        const esp_err_t restart = twai_start();
        if (restart == ESP_OK) {
          canRecoveryPending = false;
          canDriverReady = true;
          addAnalyzerEvent("can", "TWAI recovered and restarted");
        } else {
          addAnalyzerEvent("can", "TWAI restart after recovery failed; will retry");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void statusLedTask(void *parameter) {
  // Blink while no clients are connected to the ESP32 access point;
  // remain steadily ON as soon as one or more AP clients are connected.
  bool ledOn = false;
  for (;;) {
    const uint8_t clients = WiFi.softAPgetStationNum();
    if (clients > 0) {
      digitalWrite(PIN_STATUS_LED, LED_ON_LEVEL);
      ledOn = true;
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      ledOn = !ledOn;
      digitalWrite(PIN_STATUS_LED, ledOn ? LED_ON_LEVEL : LED_OFF_LEVEL);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

bool startCan() {
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t timing;
  if (canBitrate == 125000) timing = TWAI_TIMING_CONFIG_125KBITS();
  else if (canBitrate == 250000) timing = TWAI_TIMING_CONFIG_250KBITS();
  else if (canBitrate == 1000000) timing = TWAI_TIMING_CONFIG_1MBITS();
  else timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t err = twai_driver_install(&general, &timing, &filter);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;
  err = twai_start();
  return err == ESP_OK || err == ESP_ERR_INVALID_STATE;
}

// Captive-portal detection: redirect HTTP probes to the I.R.A. dashboard.
void redirectToPortal() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Location", String("http://") + AP_IP.toString() + "/", true);
  server.send(302, "text/plain; charset=utf-8", "Opening I.R.A. dashboard...");
}

void failSafeHalt(const char *reason) {
  for (size_t i = 0; i < OUTPUT_COUNT; ++i) digitalWrite(OUTPUTS[i].pin, LOW);
  digitalWrite(PIN_READY_RELAY, LOW);
  digitalWrite(PIN_BUZZER_RELAY, LOW);
  digitalWrite(PIN_MODBUS_DE, LOW);
  Serial.begin(115200);
  Serial.print("FATAL INIT: ");
  Serial.println(reason);
  for (;;) delay(1000);
}

void setup() {
  // Safe software state from first initialization. For hazardous equipment,
  // external pull-downs / hardware interlocks are still required during reset.
  pinMode(PIN_READY_BUTTON, INPUT_PULLUP);
  digitalWrite(PIN_MODBUS_DE, LOW);
  pinMode(PIN_MODBUS_DE, OUTPUT);
  // Ready relay is active-HIGH: preload OFF level before enabling output to avoid an ON pulse at boot.
  digitalWrite(PIN_READY_RELAY, LOW); pinMode(PIN_READY_RELAY, OUTPUT);
  digitalWrite(PIN_BUZZER_RELAY, LOW); pinMode(PIN_BUZZER_RELAY, OUTPUT);
  for (size_t i = 0; i < OUTPUT_COUNT; ++i) {
    digitalWrite(OUTPUTS[i].pin, LOW);
    pinMode(OUTPUTS[i].pin, OUTPUT);
  }

  stateMutex = xSemaphoreCreateMutex();
  controlStateMutex = xSemaphoreCreateMutex();
  timerMutex = xSemaphoreCreateMutex();
  analyzerMutex = xSemaphoreCreateMutex();
  uartMutex = xSemaphoreCreateMutex();
  if (!stateMutex || !controlStateMutex || !timerMutex || !analyzerMutex || !uartMutex) {
    failSafeHalt("FreeRTOS mutex allocation failed");
  }

  prefs.begin("seq-timers", true);
  for (size_t i = 0; i < TIMER_COUNT; ++i) {
    const uint32_t defaultValue = (i == T_RUN_MODE) ? (sequenceTimersMs[i] / 3600000UL) : (sequenceTimersMs[i] / 1000UL);
    uint32_t value = prefs.getUInt(TIMER_KEYS[i], defaultValue);
    const uint32_t maxValue = (i == T_READY_DELAY) ? 600UL : ((i == T_RUN_MODE) ? 24UL : 30UL);
    if (value < 1 || value > maxValue) value = defaultValue;
    sequenceTimersMs[i] = (i == T_RUN_MODE) ? value * 3600000UL : value * 1000UL;
  }
  prefs.end();

  prefs.begin(PREF_NAMESPACE, true);
  staSSID = prefs.getString("ssid", "");
  staPassword = prefs.getString("password", "");
  apSSID = prefs.getString("apSsid", DEFAULT_AP_SSID);
  apPassword = prefs.getString("apPass", DEFAULT_AP_PASSWORD);
  prefs.end();
  if (apSSID.isEmpty() || apSSID.length() > 32) apSSID = DEFAULT_AP_SSID;
  if (apPassword.length() < 8 || apPassword.length() > 63) apPassword = DEFAULT_AP_PASSWORD;

  // Matrix is disabled; GPIO2 is reserved for the onboard status LED.
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LED_OFF_LEVEL);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  // Configure the AP address before starting the access point.
  if (!WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET)) failSafeHalt("Access point IP configuration failed");
  if (!WiFi.softAP(apSSID.c_str(), apPassword.c_str())) failSafeHalt("Access point start failed");
  // Resolve all hostnames to the ESP32 so captive-portal checks reach this server.
  if (!dnsServer.start(DNS_PORT, "*", AP_IP)) failSafeHalt("Captive portal DNS start failed");
  if (staSSID.length()) WiFi.begin(staSSID.c_str(), staPassword.c_str());

  server.on("/", HTTP_GET, handleRoot);
  // Android, Apple and Windows captive-network probes. A redirect indicates
  // that this Wi-Fi network requires a captive-portal sign-in/action.
  server.on("/generate_204", HTTP_ANY, redirectToPortal);
  server.on("/gen_204", HTTP_ANY, redirectToPortal);
  server.on("/hotspot-detect.html", HTTP_ANY, redirectToPortal);
  server.on("/library/test/success.html", HTTP_ANY, redirectToPortal);
  server.on("/connecttest.txt", HTTP_ANY, redirectToPortal);
  server.on("/ncsi.txt", HTTP_ANY, redirectToPortal);
  server.on("/redirect", HTTP_ANY, redirectToPortal);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/analyzer", HTTP_GET, handleAnalyzer);
  server.on("/api/analyzer/config", HTTP_POST, handleAnalyzerConfig);
  server.on("/api/analyzer/mode", HTTP_POST, handleAnalyzerMode);
  server.on("/api/analyzer/tx", HTTP_POST, handleAnalyzerTx);
  server.on("/api/analyzer/can-tx", HTTP_POST, handleAnalyzerCanTx);
  server.on("/api/output", HTTP_POST, handleOutput);
  server.on("/api/sequence", HTTP_POST, handleSequence);
  server.on("/api/timers", HTTP_GET, handleTimersGet);
  server.on("/api/timers", HTTP_POST, handleTimersSave);
  server.on("/wifi", HTTP_POST, handleWifiSave);
  server.on("/apwifi", HTTP_POST, handleApWifiSave);
  // Unknown HTTP paths are redirected to the dashboard, which also helps
  // browsers opened by Android captive-portal notifications.
  server.onNotFound(redirectToPortal);
  server.begin();

  prefs.begin("analyzer", true);
  uartBaud = prefs.getUInt("uartBaud", DEFAULT_UART_BAUD);
  canBitrate = prefs.getUInt("canRate", 500000);
  prefs.end();
  if (uartBaud < 1200 || uartBaud > 921600) uartBaud = DEFAULT_UART_BAUD;
  if (canBitrate != 125000 && canBitrate != 250000 && canBitrate != 500000 && canBitrate != 1000000) canBitrate = 500000;
  ExternalSerial.begin(uartBaud, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  const bool canStarted = startCan();
  canDriverReady = canStarted;
  if (!canStarted) canBusState = 0;

  // Arduino loop and Wi-Fi system work use the framework's normal scheduling.
  // The explicit control, communications and status-LED tasks use FreeRTOS.
  if (xTaskCreatePinnedToCore(controlTask, "control", 3072, nullptr, 3, &controlTaskHandle, 1) != pdPASS) {
    failSafeHalt("Control task creation failed");
  }
  if (xTaskCreatePinnedToCore(communicationTask, "comms", 4096, nullptr, 2, &communicationTaskHandle, 1) != pdPASS) {
    vTaskDelete(controlTaskHandle); controlTaskHandle = nullptr;
    failSafeHalt("Communications task creation failed");
  }
  if (xTaskCreatePinnedToCore(statusLedTask, "status-led", 2048, nullptr, 1, &statusLedTaskHandle, 1) != pdPASS) {
    vTaskDelete(communicationTaskHandle); communicationTaskHandle = nullptr;
    vTaskDelete(controlTaskHandle); controlTaskHandle = nullptr;
    failSafeHalt("Status LED task creation failed");
  }
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  vTaskDelay(pdMS_TO_TICKS(2));
}
