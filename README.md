# 🔋 Smart EV BMS - Quick Reference

## 🚀 Quick Start

### Option 1: Using Script (Easiest)
```bash
cd /Users/prathmeshsapate/BMS
./start_bms.sh
```

### Option 2: Direct Command

**Simulator Mode:**
```bash
cd /Users/prathmeshsapate/BMS
python3 bms_dashboard_ai.py --simulate 1 --model models/bms_model.joblib
```

**Live Mode:**
```bash
cd /Users/prathmeshsapate/BMS
python3 bms_dashboard_ai.py --model models/bms_model.joblib
```

Then open: **http://localhost:5050**

---

## 📊 Dashboard URL

**Local:** http://localhost:5050  
**Network:** http://192.168.31.113:5050

---

## 🎮 Controls

### Manual Commands
```
FAN_ON          Turn cooling fan on
FAN_OFF         Turn cooling fan off
AC_ON           Switch to AC charger
SOLAR_ON        Switch to solar input
CUT_ON          Disconnect load/charger
CUT_OFF         Reconnect load/charger
MODE_AUTO       Let AI decide source
CAL_ACS         Calibrate current sensor
```

---

## 📈 Live Metrics

| Metric | Normal Range | Warning | Danger |
|--------|--------------|---------|--------|
| Pack Voltage | 11.1 - 12.6 V | < 9.0 V | < 7.0 V or > 13.7 V |
| Current | 0 - 3 A | 3 - 5 A | > 5 A |
| Temperature | 20 - 35 °C | 35 - 40 °C | > 40 °C |
| Risk Score | 0 - 69 | 70 - 84 | 85 - 100 |

---

## 🔐 Safety Thresholds

**Hardware Protection (STM32):**
- Over-voltage: 13.7 V → CUT
- Under-voltage: 7.0 V → CUT
- Over-temperature: 40 °C → CUT
- Over-current: 5 A → CUT

**AI Protection:**
- Risk ≥ 85 → CUT
- Risk ≥ 70 → FAN_ON + Monitor
- Anomaly > 75 → Alert

---

## 🔧 Hardware Pins

### STM32F103C8
```
Sensors:
  PA0  - Pack Voltage (divider 33k/6.8k)
  PA4  - Solar Voltage (divider 33k/6.8k)
  PA2  - Current ACS712
  PA8  - Temperature DS18B20

Relays:
  PA5  - Fan (HIGH = ON)
  PA6  - Solar/AC (LOW = Solar, HIGH = AC)
  PA7  - Cut (LOW = CUT, HIGH = Connected)

Buttons:
  PB13 - LCD Page UP
  PB14 - LCD Page DOWN

Serial:
  PA9/PA10  - UART1 to ESP8266 (9600 baud)
  PB10/PB11 - UART3 to DFPlayer Mini (9600 baud)
```

### ESP8266 NodeMCU
```
UART:
  D6 (GPIO12) - RX from STM32
  D5 (GPIO14) - TX to STM32

I2C LCD:
  D2 (GPIO4)  - SDA
  D1 (GPIO5)  - SCL
  Address: 0x27 (or 0x3F)
```

---

## 📡 Network Configuration

### ESP8266 WiFi
```cpp
SSID: "Prathmesh"
Password: "Prathmesh"
MQTT Broker: 192.168.31.96:1883
```

### MQTT Topics
```
Publish:
  bms/base      - Telemetry data (1 Hz)
  bms/last      - Last known state (retained)
  bms/online    - Connection status (LWT)
  bms/bridge/ip - ESP IP address

Subscribe:
  bms/cmd       - Command channel
```

---

## 🧪 Testing Checklist

### Software ✅
- [x] Dashboard loads
- [x] All KPIs display
- [x] Charts render
- [x] Controls work
- [x] Events log
- [x] CSV export

### Hardware ⏳
- [ ] ESP8266 WiFi connects
- [ ] LCD displays data
- [ ] MQTT publishes
- [ ] Commands execute
- [ ] Voice alerts play
- [ ] Relays switch

---

## 🐛 Troubleshooting

### Dashboard Won't Start
```bash
# Check Python version (need 3.10+)
python3 --version

# Reinstall dependencies
pip3 install --upgrade tornado flask flask-socketio paho-mqtt numpy scikit-learn joblib
```

### Port Already in Use
```bash
# Find process on port 5050
lsof -i :5050

# Kill process
kill -9 <PID>
```

### MQTT Not Connecting
```bash
# Check broker is running
brew services list | grep mosquitto

# Start broker
brew services start mosquitto

# Test connection
mosquitto_sub -h 192.168.31.96 -t bms/base -v
```

---

## 📚 File Locations

```
Project: /Users/prathmeshsapate/BMS/

Important Files:
  bms_dashboard_ai.py          Main dashboard
  ai_service.py                AI processing
  models/bms_model.joblib      Trained model
  all_logs.csv                 Training data
  start_bms.sh                 Quick start script
  
Firmware:
  BMS_RAC_AI_NEW_ESP8266/      ESP8266 code
  BSM_RAC_AI_NEW_STM32/        STM32 code

Artifacts:
  ~/.gemini/antigravity/brain/a15681ee-20a1-43c5-a4e7-2875c4d0fcb4/
    ├── deployment_walkthrough.md
    ├── system_health_report.md
    ├── task.md
    ├── bms_dashboard_live_*.png
    └── bms_dashboard_demo_*.webp
```

---

## 💡 Tips

1. **Simulator Mode** - Test without hardware
2. **CSV Export** - Download data at `/export.csv`
3. **Model Retraining** - Use `--train` flag
4. **Environment Vars** - Override config with `BMS_MQTT` etc.
5. **Multiple Dashboards** - Run on different ports with `--port`

---

## 📞 Support

**Dashboard:** http://localhost:5050  
**Project:** /Users/prathmeshsapate/BMS  
**Documentation:** See deployment_walkthrough.md

---

**Status:** ✅ OPERATIONAL  
**Mode:** Simulator Running  
**Last Updated:** 2026-01-26 19:16 IST
