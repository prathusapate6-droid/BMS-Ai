/* ================================================================
 * Smart EV BMS — STM32F103C8 prathamesh
 * ---------------------------------------------------------------
 * ================================================================*/

#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
#include <math.h>

// -------------------- PINS --------------------
#define PIN_RELAY_FAN PA5
#define PIN_RELAY_SOLAR PA6  // LOW = Solar ON
#define PIN_RELAY_CUT PA7    // LOW = CUT (disconnect)
// Fire spray servo now on ESP8266 (D4)
#define PIN_VPACK PA0

#define PIN_VSOLAR PA4
#define PIN_DS18B20 PA8
#define ACS_PIN PA2  // << ACS on PA2

// ---- NEW: page control buttons on STM32 (active-LOW) ----
#define BTN_UP PB13
#define BTN_DOWN PB14

// ---- 3 Cell Voltage Monitoring ----
#define CELL1_PIN PA1  // 0-4.2V
#define CELL2_PIN PA3  // 0-8.4V
#define CELL3_PIN PB1  // 0-12.6V

// ---- NEW: LCD page state (0..4) ----
volatile int lcd_page = 0;
const int LCD_PAGES = 5;

uint32_t last_btn_ms = 0;
bool prev_up = true, prev_down = true;

// -------------------- SERIALS -----------------
// HardwareSerial& DBG = Serial2;  // Debug (PA9/PA10)
HardwareSerial &ESP = Serial1;  // ESP8266 (PA9/PA10)
HardwareSerial &MP3 = Serial3;  // DFPlayer (PB10/PB11)
DFRobotDFPlayerMini dfp;

// -------------------- CONSTANTS --------------------
static const uint8_t WARN_REPEAT_COUNT = 3;
static const uint32_t WARN_REPEAT_GAP_MS = 1500;

// Dividers & ADC
static const float R1 = 33000.0f, R2 = 6800.0f;    // Vbat divider
static const float R1s = 33000.0f, R2s = 6800.0f;  // Vsolar divider
static const float VREF = 3.20f;
static const float ADC_MAX = 4095.0f;
static const float ADC_TO_V = (VREF / ADC_MAX);

// Thresholds
static const float V_OV_DANGER = 14.5f;
static const float V_UV_DANGER = 7.0f;
static const float V_UV_WARN = 9.0f;
static const float T_DANGER = 40.0f;  // °C over-temperature cutoff
static const float T_FIRE = 60.0f;    // Fire Trigger
static const float I_DANGER = 3.0f;   // A  over-current cutoff
static const float VSOLAR_MIN_FOR_SOLAR = 10.0f;

// ACS712 (5A default)
static float ACS_MV_PER_AMP = 100.0f;  // 5A=185, 20A=100, 30A=66
static int acs_offset_counts = 2048;   // learned at boot
static const unsigned US_GAP = 150;    // sampling gap (µs)

// Display smoothing & noise clamp
static const float I_NOISE_FLOOR = 0.05f;    // ~50 mA — shows small loads
static const float CURRENT_STEP_A = 0.008f;  // ~8 mA display steps

// -------------------- Command / override --------------------
enum ForceMode { FM_AUTO = 0,
                 FM_AC = 1,
                 FM_SOLAR = 2 };
ForceMode forceMode = FM_AUTO;
int8_t manualFan = -1;  // -1:Auto, 0:Off, 1:On
uint32_t forceHoldUntil = 0;
const uint32_t FORCE_HOLD_MS = 60000;  // 60 s hold for manual/AI override
String rxCmd;
String voice_msg = "NORMAL";  // Current voice state for dashboard

// -------------------- FILTER/STATE --------------------
static inline float ema(float prev, float x, float a) {
  return (prev == 0.0f) ? x : (prev * (1.0f - a) + a * x);
}
static inline float quantize_amp(float x, float step) {
  return roundf(x / step) * step;
}

float ema_v = 0, ema_s = 0, ema_i = 0, ema_t = 0;
String mode = "AC", pred = "SAFE";
int soc = 0, soh = 95, sop = 80;
uint8_t flags_fan = 0, flags_ac = 1, flags_solar = 0, flags_cut = 0;

String prev_mode = "", prev_pred = "";
bool prev_cOV = false, prev_cUV = false, prev_cOT = false, prev_cOC = false;
uint32_t last_tick_ms = 0;

// WARN repeat
uint8_t warn_left = 2;
uint32_t warn_next_ms = 0;

// ---- Fault alarm repeat (OT / OC / OV / UV) ----
enum AlarmType { ALARM_NONE,
                 ALARM_OT,
                 ALARM_OC,
                 ALARM_OV,
                 ALARM_UV };
AlarmType active_alarm = ALARM_NONE;

uint8_t alarm_repeat_left = 0;                // how many times to repeat "over-XXXX detected"
uint32_t alarm_next_ms = 0;                   // next time we are allowed to speak
static const uint8_t FAULT_REPEAT_COUNT = 2;  // change to 4 if you want 4 times

// -------------------- VOLTAGE READ --------------------
// Global variables for individual cells
float v_cell1 = 0.0f;
float v_cell2 = 0.0f;
float v_cell3 = 0.0f;

float readVbat() {
  // Cell 1
  int adc_val1 = analogRead(CELL1_PIN);
  float v1 = (adc_val1 * 3.40f) / 4095.0f;
  float c1_in = v1 / (6800.0f / (33000.0f + 6800.0f));

  // Cell 1+2
  int adc_val2 = analogRead(CELL2_PIN);
  float v2 = (adc_val2 * 3.40f) / 4095.0f;
  float c2_in = v2 / (6800.0f / (33000.0f + 6800.0f));

  // Cell 1+2+3
  int adc_val3 = analogRead(CELL3_PIN);
  float v3 = (adc_val3 * 3.40f) / 4095.0f;
  float c3_in = v3 / (6800.0f / (33000.0f + 6800.0f));

  // Total Pack Voltage (uses 5.2V VREF — separate voltage divider reference)
  int adc_val_pack = analogRead(PIN_VPACK);
  float v_pack_adc = (adc_val_pack * 3.40f) / 4095.0f;
  float pack_in = v_pack_adc / (6800.0f / (33000.0f + 6800.0f));

  // Individual cell voltages — DIFFERENTIAL (cumulative minus previous stage)
  v_cell1 = c1_in;          // Cell 1 voltage
  v_cell2 = c2_in - c1_in;  // Cell 2 = cumulative(C1+C2) - cumulative(C1)
  v_cell3 = c3_in - c2_in;  // Cell 3 = cumulative(C1+C2+C3) - cumulative(C1+C2)

  if (v_cell1 < 0)
    v_cell1 = 0;
  if (v_cell2 < 0)
    v_cell2 = 0;
  if (v_cell3 < 0)
    v_cell3 = 0;

  return pack_in;
}

float readVoltageRaw(uint8_t pin) {
  return analogRead(pin) * ADC_TO_V;
}
float readVsol() {
  return readVoltageRaw(PIN_VSOLAR) / (R2s / (R1s + R2s));
}

// -------------------- DS18B20 (bit-banged) --------------------
bool DS18B20_Init() {
  pinMode(PIN_DS18B20, OUTPUT);
  digitalWrite(PIN_DS18B20, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_DS18B20, LOW);
  delayMicroseconds(750);
  digitalWrite(PIN_DS18B20, HIGH);
  pinMode(PIN_DS18B20, INPUT);
  int t = 0;
  while (digitalRead(PIN_DS18B20)) {
    t++;
    if (t > 60)
      return false;
    delayMicroseconds(1);
  }
  t = 480 - t;
  pinMode(PIN_DS18B20, OUTPUT);
  delayMicroseconds(t);
  digitalWrite(PIN_DS18B20, HIGH);
  return true;
}
void DS18B20_Write(byte data) {
  pinMode(PIN_DS18B20, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_DS18B20, LOW);
    delayMicroseconds(10);
    (data & 1) ? digitalWrite(PIN_DS18B20, HIGH)
               : digitalWrite(PIN_DS18B20, LOW);
    data >>= 1;
    delayMicroseconds(50);
    digitalWrite(PIN_DS18B20, HIGH);
  }
}
byte DS18B20_Read() {
  pinMode(PIN_DS18B20, OUTPUT);
  digitalWrite(PIN_DS18B20, HIGH);
  delayMicroseconds(2);
  byte data = 0;
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_DS18B20, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_DS18B20, HIGH);
    pinMode(PIN_DS18B20, INPUT);
    delayMicroseconds(5);
    data >>= 1;
    if (digitalRead(PIN_DS18B20))
      data |= 0x80;
    delayMicroseconds(55);
    pinMode(PIN_DS18B20, OUTPUT);
    digitalWrite(PIN_DS18B20, HIGH);
  }
  return data;
}
float readTempC() {
  if (!DS18B20_Init())
    return NAN;
  DS18B20_Write(0xCC);
  DS18B20_Write(0x44);
  if (!DS18B20_Init())
    return NAN;
  DS18B20_Write(0xCC);
  DS18B20_Write(0xBE);
  int t = DS18B20_Read();
  t |= (DS18B20_Read() << 8);
  return t * 0.0625f;
}

// -------------------- SOC (3S OCV curve) --------------------
int socFromV(float vpack) {
  const float vpc = vpack / 3.0f;
  struct {
    float v;
    int s;
  } T[] = { { 3.60, 100 }, { 3.50, 95 }, { 3.45, 90 }, { 3.40, 85 }, { 3.35, 75 }, { 3.30, 60 }, { 3.25, 45 }, { 3.20, 30 }, { 3.10, 15 }, { 3.00, 5 }, { 2.80, 0 } };
  if (vpc >= T[0].v)
    return 100;
  if (vpc <= T[10].v)
    return 0;
  for (int i = 0; i < 10; i++) {
    if (vpc <= T[i].v && vpc >= T[i + 1].v) {
      float t = (vpc - T[i + 1].v) / (T[i].v - T[i + 1].v);
      return (int)round(T[i + 1].s + t * (T[i].s - T[i + 1].s));
    }
  }
  return 0;
}

// -------------------- DFPlayer helpers --------------------
uint32_t last_voice_ms = 0;
const uint32_t MIN_VOICE_GAP_MS = 1200;

bool canSpeak() {
  return (millis() - last_voice_ms) > MIN_VOICE_GAP_MS;
}
bool voicePlay(uint8_t folder, uint8_t file) {
  if (!canSpeak())
    return false;
  dfp.playFolder(folder, file);
  last_voice_ms = millis();
  return true;
}
bool voiceCause_OC() {
  voice_msg = "OVER CURRENT";
  return voicePlay(3, 2);
}
bool voiceCause_OV() {
  voice_msg = "OVER VOLTAGE";
  return voicePlay(3, 3);
}
bool voiceCause_UV() {
  voice_msg = "UNDER VOLTAGE";
  return voicePlay(3, 4);
}
bool voiceCause_OT() {
  voice_msg = "OVER TEMPERATURE";
  return voicePlay(3, 1);
}
bool voiceDanger() {
  voice_msg = "DANGER";
  return voicePlay(1, 3);
}
bool voiceWarning() {
  voice_msg = "WARNING";
  return voicePlay(1, 2);
}
bool voiceNormal() {
  voice_msg = "NORMAL";
  return voicePlay(1, 1);
}
bool voiceSolar() {
  voice_msg = "SOLAR MODE";
  return voicePlay(2, 1);
}
bool voiceAC() {
  voice_msg = "AC MODE";
  return voicePlay(2, 2);
}
bool voiceCutOff() {
  voice_msg = "CUT OFF";
  return voicePlay(2, 3);
}
bool voiceChargingDone() {
  voice_msg = "CHARGING COMPLETE";
  return voicePlay(4, 3);
}

void printDetail(uint8_t type, int value) {
  // Currently not used for logic; kept so DFPlayer callbacks still compile.
  // You can add debug here if you want.
}

// -------------------- ACS712: offset + VPP→VRMS window --------------------
void acsCalibrate(unsigned samples = 1200) {
  long sum = 0;
  for (unsigned i = 0; i < samples; i++) {
    sum += analogRead(ACS_PIN);
    delayMicroseconds(US_GAP);
  }
  acs_offset_counts = (int)(sum / (long)samples);
}
void acsMeasure(float &i_dc, float &i_rms) {
  // DC via average vs offset
  const unsigned N_DC = 600;
  long sum = 0;
  for (unsigned i = 0; i < N_DC; i++) {
    sum += analogRead(ACS_PIN);
    delayMicroseconds(US_GAP);
  }
  int avgC = (int)(sum / (long)N_DC);
  float delta_counts = (float)avgC - (float)acs_offset_counts;
  float delta_mV = (delta_counts * ADC_TO_V) * 1000.0f;
  i_dc = delta_mV / ACS_MV_PER_AMP;

  // RMS via VPP window (~25 ms)
  int minC = 4095, maxC = 0;
  const uint32_t win_us = 25000;
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < win_us) {
    int a = analogRead(ACS_PIN);
    if (a < minC)
      minC = a;
    if (a > maxC)
      maxC = a;
    delayMicroseconds(US_GAP);
  }
  float vpp = (maxC - minC) * ADC_TO_V;
  float vrms = (vpp * 0.5f) * 0.70710678f;  // sine assumption
  i_rms = (vrms * 1000.0f) / ACS_MV_PER_AMP;

  if (fabs(i_dc) < I_NOISE_FLOOR)
    i_dc = 0.0f;
  if (fabs(i_rms) < I_NOISE_FLOOR)
    i_rms = 0.0f;
}

// -------------------- Relays + command handling --------------------
inline void setFan(bool on) {
  digitalWrite(PIN_RELAY_FAN, on ? HIGH : LOW);
  flags_fan = on ? 1 : 0;
}
inline void setCut(bool on) {
  digitalWrite(PIN_RELAY_CUT, on ? LOW : HIGH);
  flags_cut = on ? 1 : 0;
}
inline void setAC() {
  digitalWrite(PIN_RELAY_SOLAR, HIGH);
  flags_ac = 1;
  flags_solar = 0;
}
inline void setSOLAR() {
  digitalWrite(PIN_RELAY_SOLAR, LOW);
  flags_ac = 0;
  flags_solar = 1;
}  // Note: Fire spray servo control moved to ESP8266 (D4)

void sendAck(const char *what) {
  ESP.print("{\"ack\":\"");
  ESP.print(what);
  ESP.println("\"}");
}

void handleCmd(const String &cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();
  String up = cmd;
  up.toUpperCase();
  // Hold manual/AI override for 60s
  forceHoldUntil = millis() + FORCE_HOLD_MS;

  if (up == "FAN_ON") {
    manualFan = 1;
    setFan(true);
    voicePlay(4, 1);
    sendAck("FAN_ON");
    return;
  }
  if (up == "FAN_OFF") {
    manualFan = 0;
    setFan(false);
    voicePlay(4, 2);
    sendAck("FAN_OFF");
    return;
  }
  if (up == "CUT_ON") {
    setCut(true);
    voiceCutOff();
    sendAck("CUT_ON");
    return;
  }
  if (up == "CUT_OFF") {
    setCut(false);
    voiceNormal();
    sendAck("CUT_OFF");
    return;
  }

  if (up == "AC_ON" || up == "MODE_AC") {
    setAC();
    forceMode = FM_AC;
    voiceAC();
    sendAck("AC");
    return;
  }
  if (up == "SOLAR_ON" || up == "MODE_SOLAR") {
    setSOLAR();
    forceMode = FM_SOLAR;
    voiceSolar();
    sendAck("SOLAR");
    return;
  }
  // Note: SPRAY commands now handled by ESP8266
  if (up == "MODE_AUTO") {
    forceMode = FM_AUTO;
    manualFan = -1;
    voiceNormal();
    sendAck("AUTO");
    return;
  }

  if (up == "CAL_ACS") {
    acsCalibrate();
    voiceNormal();
    sendAck("CAL_ACS");
    return;
  }
}

void pollCommands() {
  while (ESP.available()) {
    char c = (char)ESP.read();
    if (c == '\n') {
      rxCmd.trim();
      if (rxCmd.startsWith("CMD:"))
        handleCmd(rxCmd.substring(4));
      rxCmd = "";
    } else if (c != '\r') {
      rxCmd += c;
      if (rxCmd.length() > 120)
        rxCmd = "";
    }
  }
}

void pollButtonsLCD() {
  bool up = digitalRead(BTN_UP);  // pull-up: LOW = pressed
  bool dn = digitalRead(BTN_DOWN);
  uint32_t now = millis();

  const uint32_t DEBOUNCE_MS = 180;

  if (!up && prev_up && (now - last_btn_ms > DEBOUNCE_MS)) {
    lcd_page = (lcd_page + 1) % LCD_PAGES;
    last_btn_ms = now;
  }
  if (!dn && prev_down && (now - last_btn_ms > DEBOUNCE_MS)) {
    lcd_page = (lcd_page - 1 + LCD_PAGES) % LCD_PAGES;
    last_btn_ms = now;
  }
  prev_up = up;
  prev_down = dn;
}

// -------------------- SETUP --------------------
void setup() {
  // DBG.begin(115200);
  delay(50);
  // DBG.println("\n[Boot] BMS MiniAI (UART map OK, override OK)");

  MP3.begin(9600);
  delay(1200);
  if (dfp.begin(MP3)) {
    dfp.volume(100);  // Removed: Setting volume here can freeze some DFPlayer
                      // clones on boot.
    dfp.EQ(DFPLAYER_EQ_NORMAL);
  }

  ESP.begin(9600);

  pinMode(PIN_RELAY_FAN, OUTPUT);
  pinMode(PIN_RELAY_SOLAR, OUTPUT);
  pinMode(PIN_RELAY_CUT, OUTPUT);
  // Fire spray servo now on ESP8266 (D4)

  pinMode(CELL1_PIN, INPUT_ANALOG);
  pinMode(CELL2_PIN, INPUT_ANALOG);
  pinMode(CELL3_PIN, INPUT_ANALOG);
  pinMode(PIN_VPACK, INPUT_ANALOG);
  pinMode(PIN_VSOLAR, INPUT_ANALOG);
  pinMode(ACS_PIN, INPUT_ANALOG);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  // defaults
  setFan(false);
  setAC();        // AC by default
  setCut(false);  // enable load/charger
  // Fire spray servo now on ESP8266

  acsCalibrate();  // calibrate at idle
  delay(3000);
  acsCalibrate();
  voiceNormal();
}

// -------------------- LOOP --------------------
void loop() {
  pollCommands();
  if (dfp.available()) {
    printDetail(dfp.readType(), dfp.read());
  }

  // Read sensors
  float vbat = readVbat();
  float vsol = readVsol();
  pollButtonsLCD();
  float i_dc = 0.0f, i_rms = 0.0f;
  acsMeasure(i_dc, i_rms);

  // Hybrid display: RMS if ripple dominates, else DC
  float i_show = (fabs(i_rms) > fabs(i_dc) * 1.2f) ? i_rms : i_dc;

  float tempC = readTempC();
  ema_v = ema(ema_v, vbat, 0.15f);
  ema_s = ema(ema_s, vsol, 0.15f);
  ema_i = ema(ema_i, i_show, 0.20f);
  if (!isnan(tempC))
    ema_t = ema(ema_t, tempC, 0.10f);

  float i_out = quantize_amp(ema_i, CURRENT_STEP_A);
  soc = socFromV(ema_v);

  // Faults (OC on DC path only)
  bool cOV = (ema_v >= V_OV_DANGER);
  bool cUV =
    (ema_v > 2.0f && ema_v <= V_UV_DANGER);  // Ignore pure 0V (relay disconnect artifact)

  bool cOT = (ema_t >= T_DANGER);
  bool cFire = (ema_t >= T_FIRE);

  // Fire Protection: ESP8266 handles servo when temp >= 60°C
  // (Data sent via JSON, ESP8266 triggers servo on D4)

  // Robust Over-Current Detection (Ignores inrush spikes)
  static uint32_t oc_latch_ms = 0;
  static uint8_t oc_spike_count = 0;

  if (fabs(i_dc) >= I_DANGER) {
    oc_spike_count++;
    // Require ~5 consecutive readings (approx 150ms) of high current to trigger fault
    if (oc_spike_count >= 5) {
      oc_latch_ms = millis() + 5000;  // Latch Over-Current for 5 seconds
      oc_spike_count = 5;             // prevent overflow
    }
  } else {
    oc_spike_count = 0;  // reset on clean reading
  }
  bool cOC = (millis() < oc_latch_ms);

  bool wUV = (!cUV && ema_v <= V_UV_WARN);

  // Charging Complete Detection (high voltage + low current)
  static bool was_charging = false;
  static bool charging_done_pending = false;
  bool charging_now = (ema_v >= 13.5f && fabs(i_dc) < 0.3f);
  if (charging_now && !was_charging) {
    charging_done_pending = true;
  }
  was_charging = charging_now;

  if (charging_done_pending && active_alarm == ALARM_NONE) {
    if (voiceChargingDone()) {
      charging_done_pending = false;
    }
  }

  // AUTO decision
  String autoMode = (ema_s >= VSOLAR_MIN_FOR_SOLAR ? "SOLAR" : "AC");

  // Charging Status Detection
  // 0 = Not Charging, 1 = AC Charging, 2 = Solar Charging
  int charging_status = 0;
  if (i_dc < -0.2f) {  // Negative current = charging (adjust threshold as needed)
    if (flags_solar == 1) {
      charging_status = 2;  // Solar Charging
    } else if (flags_ac == 1) {
      charging_status = 1;  // AC Charging
    }
  }

  // Apply manual/AI override if still within hold window
  bool manualActive = (millis() < forceHoldUntil) && (forceMode != FM_AUTO);
  String forced = (forceMode == FM_SOLAR ? "SOLAR" : "AC");

  // Final desired mode with safety override
  String desiredMode = (cOV || cUV || cOT || cOC) ? "CUT"
                       : manualActive             ? forced
                                                  : autoMode;

  // Drive outputs with RECOVERY CHARGING MODE
  // When UV detected: Disconnect LOAD but Allow CHARGER (recovery)
  if (desiredMode == "CUT") {
    // Check if UV is the only fault (recovery charging possible)
    bool onlyUV = cUV && !cOV && !cOT && !cOC;

    if (onlyUV) {
      // RECOVERY MODE: Allow charging but disconnect load
      setCut(true);   // Disconnect load (safety)
      setFan(false);  // No fan needed during recovery

      // Enable charger based on available source
      if (ema_s >= VSOLAR_MIN_FOR_SOLAR) {
        setSOLAR();  // Use Solar for recovery
      } else {
        setAC();  // Use AC for recovery
      }
    } else {
      // Other faults (OV/OT/OC): Full shutdown + cooling
      setCut(true);
      setFan(true);
    }
  } else {
    setCut(false);
    // Fan Manual Override
    if (millis() < forceHoldUntil && manualFan != -1) {
      setFan(manualFan == 1);
    } else {
      setFan(false);
    }
    if (desiredMode == "SOLAR")
      setSOLAR();
    else
      setAC();
  }

  String newMode = desiredMode;
  String newPred =
    (cOV || cUV || cOT || cOC) ? "DANGER" : (wUV ? "WARN" : "SAFE");

  // Cause -> multi-repeat alarm (edge start OR periodic reminder)
  bool anyFault = (cOT || cOC || cOV || cUV);
  static uint32_t last_fault_reminder_ms = 0;

  if (anyFault) {
    bool trigger = false;
    if (cOT && !prev_cOT) {
      active_alarm = ALARM_OT;
      trigger = true;
    } else if (cOC && !prev_cOC) {
      active_alarm = ALARM_OC;
      trigger = true;
    } else if (cOV && !prev_cOV) {
      active_alarm = ALARM_OV;
      trigger = true;
    } else if (cUV && !prev_cUV) {
      active_alarm = ALARM_UV;
      trigger = true;
    }

    // Periodic reminder if still in danger and no active sequence running
    if (!trigger && active_alarm == ALARM_NONE && (millis() - last_fault_reminder_ms > 12000)) {
      if (cOT)
        active_alarm = ALARM_OT;
      else if (cOC)
        active_alarm = ALARM_OC;
      else if (cOV)
        active_alarm = ALARM_OV;
      else if (cUV)
        active_alarm = ALARM_UV;
      trigger = true;
    }

    if (trigger) {
      alarm_repeat_left = FAULT_REPEAT_COUNT;
      alarm_next_ms = millis();
      last_fault_reminder_ms = millis();
    }
  }

  prev_cOT = cOT;
  prev_cOC = cOC;
  prev_cOV = cOV;
  prev_cUV = cUV;

  // State voices (SAFE / WARN / DANGER) - Fault alarms take priority over
  // "DANGER" voice
  if (newPred != prev_pred && active_alarm == ALARM_NONE) {
    if (newPred == "SAFE") {
      voiceNormal();
      warn_left = 0;
    } else if (newPred == "WARN") {
      voiceWarning();
      warn_left = (WARN_REPEAT_COUNT ? WARN_REPEAT_COUNT - 1 : 0);
      warn_next_ms = millis() + WARN_REPEAT_GAP_MS;
    } else {
      warn_left = 0;
    }
  }

  if (newPred == "WARN" && warn_left && (long)(millis() - warn_next_ms) >= 0 && active_alarm == ALARM_NONE) {
    if (voiceWarning()) {
      warn_left--;
      warn_next_ms = millis() + WARN_REPEAT_GAP_MS;
    }
  }

  // Mode voices (SOLAR / AC / CUT) - Only if no critical alarm is trying to
  // speak
  if (newMode != prev_mode && active_alarm == ALARM_NONE) {
    if (newMode == "SOLAR")
      voiceSolar();
    else if (newMode == "AC")
      voiceAC();
    else
      voiceCutOff();
  }

  mode = newMode;
  pred = newPred;
  prev_mode = mode;
  prev_pred = pred;

  // --------------------------------------------------
  // Fault alarm voice manager:
  //   - Repeat "over-XXX" FAULT_REPEAT_COUNT times
  //   - 1 second gap
  //   - Then say generic DANGER once
  // --------------------------------------------------
  if (active_alarm != ALARM_NONE) {
    uint32_t now = millis();

    if (now >= alarm_next_ms) {
      bool success = false;
      if (alarm_repeat_left > 0) {
        // speak specific fault phrase
        if (active_alarm == ALARM_OT)
          success = voiceCause_OT();
        else if (active_alarm == ALARM_OC)
          success = voiceCause_OC();
        else if (active_alarm == ALARM_OV)
          success = voiceCause_OV();
        else if (active_alarm == ALARM_UV)
          success = voiceCause_UV();

        if (success) {
          alarm_repeat_left--;
          alarm_next_ms =
            now + 1500;  // 1.5 second delay between repeats for clarity
        }
      } else {
        // after repeats, say generic DANGER once
        if (voiceDanger()) {
          active_alarm = ALARM_NONE;  // stop sequence
        }
      }
    }
  }

  // 1 Hz telemetry (Debug + JSON to ESP)
  if (millis() - last_tick_ms >= 1000) {
    last_tick_ms = millis();

    const char *cause = cOT ? "OT" : cOC ? "OC"
                                   : cOV ? "OV"
                                   : cUV ? "UV"
                                         : "";

    float watt = ema_v * i_out;

    // JSON to ESP (adds override info)
    ESP.print("{\"ts\":");
    ESP.print(millis() / 1000.0f);
    ESP.print(",\"v_pack\":");
    ESP.print(ema_v, 2);

    // NEW: Individual cell voltages
    ESP.print(",\"c1\":");
    ESP.print(v_cell1, 2);
    ESP.print(",\"c2\":");
    ESP.print(v_cell2, 2);
    ESP.print(",\"c3\":");
    ESP.print(v_cell3, 2);

    ESP.print(",\"v_solar\":");
    ESP.print(ema_s, 2);

    // CURRENT ALWAYS POSITIVE IN JSON
    ESP.print(",\"i_pack\":");
    ESP.print(fabs(i_out), 2);

    ESP.print(",\"t_amb\":");
    ESP.print(ema_t, 1);
    ESP.print(",\"soc\":");
    ESP.print(soc);

    ESP.print(",\"pred\":\"");
    ESP.print(pred);
    ESP.print("\"");
    ESP.print(",\"mode\":\"");
    ESP.print(mode);
    ESP.print("\"");

    ESP.print(",\"force\":\"");
    ESP.print((millis() < forceHoldUntil)
                ? (forceMode == FM_SOLAR
                     ? "SOLAR"
                     : (forceMode == FM_AC ? "AC" : "AUTO"))
                : "AUTO");
    ESP.print("\"");

    ESP.print(",\"hold_s\":");
    ESP.print((millis() < forceHoldUntil)
                ? (int)((forceHoldUntil - millis()) / 1000)
                : 0);

    // flags
    ESP.print(",\"flags\":{");
    ESP.print("\"fan\":");
    ESP.print(flags_fan);
    ESP.print(",\"ac\":");
    ESP.print(flags_ac);
    ESP.print(",\"solar\":");
    ESP.print(flags_solar);
    ESP.print(",\"cut\":");
    ESP.print(flags_cut);
    ESP.print(",\"charging\":");
    ESP.print(charging_status);  // 0=None, 1=AC, 2=Solar
    ESP.print("}");
    ESP.print(",\"alarms\":{");
    ESP.print("\"ov\":");
    ESP.print(cOV ? 1 : 0);
    ESP.print(",\"uv\":");
    ESP.print(cUV ? 1 : 0);
    ESP.print(",\"ot\":");
    ESP.print(cOT ? 1 : 0);
    ESP.print(",\"oc\":");
    ESP.print(cOC ? 1 : 0);

    // NEW: Calculate cell fault flag right here where it's needed
    bool f_cell = (v_cell1 < 3.0f || v_cell1 > 4.25f || v_cell2 < 3.0f || v_cell2 > 4.25f || v_cell3 < 3.0f || v_cell3 > 4.25f);
    ESP.print(",\"cell\":");
    ESP.print(f_cell ? 1 : 0);
    ESP.print("}");

    // cause and current LCD page
    ESP.print(",\"cause\":\"");
    ESP.print(cause);
    ESP.print("\"");
    ESP.print(",\"lcd_page\":");
    ESP.print(lcd_page);

    ESP.print(",\"voice_msg\":\"");
    ESP.print(voice_msg);
    ESP.print("\"");
    ESP.println("}");
  }
}
