#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smart EV BMS — LIVE Dashboard + AI Policy (macOS)
UI unchanged (dark dashboard at http://0.0.0.0:5050)

Adds:
- Robust CSV training (--train/--out) with auto-label fallback if CSV has only SAFE
- Model loading via --model or BMS_MODEL env; live inference with fallback to heuristic
- Same controls, events, simulator, and CSV export as your working build

Deps (runtime): flask flask-socketio paho-mqtt numpy
Deps (training): scikit-learn joblib pandas
"""

import argparse, json, time, math, random, threading, csv, io, os
from collections import deque
from typing import Optional, List, Tuple

from flask import Flask, Response, render_template_string
from flask_socketio import SocketIO, emit

# Optional runtime deps
try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None

try:
    import numpy as np
except Exception:
    np = None

# Optional ML deps (for train + inference). Script works without them in LIVE mode.
SKLEARN_AVAILABLE = True
try:
    # ML Classification model (SAFE/WARN/DANGER prediction)
    from sklearn.ensemble import GradientBoostingClassifier, IsolationForest
    # ↑ IsolationForest: Anomaly detection ke liye (unusual patterns dhundne ke liye)
    
    from sklearn.preprocessing import StandardScaler  # Features ko normalize karne ke liye
    from sklearn.pipeline import Pipeline  # Model pipeline banane ke liye
    from sklearn.model_selection import train_test_split  # Data ko train/test mein divide karne ke liye
    from sklearn.metrics import classification_report, confusion_matrix  # Accuracy check karne ke liye
    import joblib  # Model ko save/load karne ke liye (.joblib files)
except Exception:
    SKLEARN_AVAILABLE = False

# =========================
# Configuration (edit if needed)
# =========================
MQTT_BROKER    = os.environ.get("BMS_MQTT",      "e5c6d611df63436992755767b6967071.s1.eu.hivemq.cloud")
MQTT_PORT      = int(os.environ.get("BMS_MQTT_PORT", "8883"))
MQTT_USERNAME  = os.environ.get("BMS_MQTT_USER", "smartwater")
MQTT_PASSWORD  = os.environ.get("BMS_MQTT_PASS", "SmartWater2026!")
TOPIC_TICK     = os.environ.get("BMS_TOPIC_IN",  "bms/base")
TOPIC_CMD      = os.environ.get("BMS_TOPIC_CMD", "bms/cmd")

HIST_SECONDS = 600  # ~10 min @2Hz
MAX_POINTS   = int(HIST_SECONDS * 2)

# MCU-aligned thresholds
V_OV_DANGER = 14.5
V_UV_DANGER = 7.0
V_UV_WARN   = 9.0
T_DANGER    = 40.0
I_DANGER    = 3.0
SOLAR_MARGIN_V = 1.0

# Risk gates
RISK_WARN = 70
RISK_CUT  = 85

# Cell balancing (placeholder analytics)
CELL_COUNT         = 3
BALANCE_THRESH_mV  = 50

# Command debounce
CMD_MIN_INTERVAL_S = 5.0

# =========================
# App / state
# =========================
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

class Ring:
    def __init__(self, n): self.b = deque(maxlen=n)
    def push(self, x): self.b.append(x)
    def list(self): return list(self.b)
    def clear(self): self.b.clear()
    def last(self, k=1):
        if not self.b: return []
        k = min(k, len(self.b))
        return list(self.b)[-k:]

class Welford:
    def __init__(self):
        self.n=0; self.mean=0.0; self.M2=0.0
    def update(self, x: float):
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        self.M2 += d * (x - self.mean)
    def std(self) -> float:
        if self.n < 2: return 0.0
        return math.sqrt(max(self.M2 / (self.n - 1), 0.0))

# Buffers for plots/features
buf_ts     = Ring(MAX_POINTS)
buf_vpack  = Ring(MAX_POINTS)
buf_vsolar = Ring(MAX_POINTS)
buf_ipack  = Ring(MAX_POINTS)
buf_temp   = Ring(MAX_POINTS)
buf_power  = Ring(MAX_POINTS)
buf_risk   = Ring(MAX_POINTS)
buf_c1     = Ring(MAX_POINTS)
buf_c2     = Ring(MAX_POINTS)
buf_c3     = Ring(MAX_POINTS)
buf_pred   = Ring(MAX_POINTS)   # 0/1/2
buf_mode   = Ring(MAX_POINTS)   # 0/1/2
buf_soc    = Ring(MAX_POINTS)
buf_soh    = Ring(MAX_POINTS)
buf_sop    = Ring(MAX_POINTS)
buf_anom   = Ring(MAX_POINTS)
buf_frisk  = Ring(MAX_POINTS)
buf_vdev   = Ring(MAX_POINTS)
buf_ispike = Ring(MAX_POINTS)
buf_tpat   = Ring(MAX_POINTS)
buf_rint   = Ring(MAX_POINTS)

# Stats
STAT_I = Welford()
STAT_T = Welford()
STAT_V = Welford()

# EIS & events
EIS_SWEEPS = deque(maxlen=8)
EVENTS     = deque(maxlen=500)
CONN       = {"mqtt":"disconnected","rate_hz":0.0,"last_tick":0.0}

LABEL_TO_IDX = {"SAFE":0,"WARN":1,"DANGER":2}
MODE_TO_IDX  = {"SOLAR":0,"AC":1,"CUT":2}
IDX_TO_LABEL = {0:"SAFE",1:"WARN",2:"DANGER"}
IDX_TO_MODE  = {0:"SOLAR",1:"AC",2:"CUT"}

AUTO_ACTIONS = True
LAST_CMD     = {"name":None, "t":0.0}

# SOH via Rint baseline
RINT_BASELINE_OHM: Optional[float] = None
RINT_BASELINE_LOCK = threading.Lock()

class RintEstimator:
    def __init__(self, n=40):
        self.v = deque(maxlen=n)
        self.i = deque(maxlen=n)
        self.last = 0.12
    def update(self, v: float, i: float) -> float:
        self.v.append(float(v)); self.i.append(float(i))
        if len(self.v) < 6: return float(self.last)
        vv = list(self.v); ii = list(self.i)
        pairs = []
        for k in range(1, len(vv)):
            di = ii[k]-ii[k-1]
            if abs(di) < 1e-3: continue
            dv = vv[k]-vv[k-1]
            r = abs(dv/di)
            pairs.append(r)
        if not pairs: return float(self.last)
        pairs.sort()
        med = pairs[len(pairs)//2]
        self.last = float(clamp(med, 0.02, 0.6))
        return self.last

RINT = RintEstimator()

# =========================
# Helpers
# =========================
def log_event(kind: str, detail: str):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {kind} — {detail}")
    EVENTS.appendleft({"ts": ts, "kind": kind, "detail": detail})
    socketio.emit("event", {"ts": ts, "kind": kind, "detail": detail})

def clamp(x, a, b):
    return a if x < a else (b if x > b else x)

def est_soc_from_voltage(v):
    lo, hi = 10.5, 12.6
    return int(round(100 * clamp((v - lo) / (hi - lo), 0, 1)))

def estimate_cells_from_pack(v_pack: float, last_cells: Optional[List[float]]) -> List[float]:
    if last_cells and len(last_cells) == CELL_COUNT:
        s = sum(last_cells)
        if s > 0:
            scale = v_pack / s
            return [max(0.0, c * scale) for c in last_cells]
    per = max(0.0, v_pack / CELL_COUNT)
    eps = 0.002
    return [per * (1.0 - eps), per, per * (1.0 + eps)]

def cell_metrics(cell_v: List[float]) -> Tuple[int, float]:
    if not cell_v or len(cell_v) != CELL_COUNT:
        return 0, 0.0
    hi = max(cell_v); lo = min(cell_v)
    imb_mv = int(round((hi - lo) * 1000.0))
    score = clamp((imb_mv - 10) / (120 - 10), 0.0, 1.0) * 100.0
    return imb_mv, score

def compute_risk(v: float, i: float, t: float) -> int:
    risk = 0.0
    if t > 35.0:
        risk += (t - 35.0) * 2.0
    risk += min(40.0, abs(i) * 3.0)
    if v < 12.0:
        risk += (12.0 - v) * 6.0
    return int(clamp(risk, 0, 100))

def maybe_set_rint_baseline(r_now: float, i_now: float, t_now: float, risk_now: int):
    global RINT_BASELINE_OHM
    if RINT_BASELINE_OHM is not None: return
    if risk_now > 20: return
    if abs(i_now) > 0.5: return
    if t_now < 15 or t_now > 38: return
    with RINT_BASELINE_LOCK:
        if RINT_BASELINE_OHM is None:
            RINT_BASELINE_OHM = float(r_now)
            log_event("SOH", f"Baseline Rint={RINT_BASELINE_OHM:.4f} Ω set")

def estimate_soh_from_rint(r_now: float) -> int:
    base = RINT_BASELINE_OHM if RINT_BASELINE_OHM else 0.15
    ratio = base / max(r_now, 1e-4)
    # User requested 80-90%
    return int(clamp(100.0 * ratio, 85.0, 100.0))

def estimate_sop_pct(v: float, r_now: float) -> int:
    pmax = (v * v) / max(4.0 * r_now, 1e-3)
    return int(clamp(100.0 * (pmax / 200.0), 0.0, 100.0))

# ===== Features (training + live inference) =====
FEAT_NAMES = [
    "v_pack","i_pack","t_amb","v_solar",
    "c1","c2","c3",
    "power","abs_i","v_margin",
    "dv_dt","di_dt","dt_dt",
    "rint",
    "i_std10","t_std10","v_std10",
    "soc"
]

def rolling_stats(buf: Ring, k=10):
    arr = buf.last(k)
    if not arr or len(arr) < 2:
        return 0.0, (arr[-1] if arr else 0.0)
    if np is not None:
        a = np.array(arr, dtype=float)
        return float(a.std()), float(a.mean())
    mean = sum(arr)/len(arr)
    std = (sum((x-mean)*(x-mean) for x in arr)/len(arr))**0.5
    return float(std), float(mean)

def build_feature_vector(ts, v, i, t, vs, soc=0.0, c1=0.0, c2=0.0, c3=0.0):
    # derivatives vs previous sample
    if buf_ts.last():
        ts_prev = buf_ts.last(1)[-1]
        dt = max(1e-3, ts - ts_prev)
        v_prev = buf_vpack.last(1)[-1] if buf_vpack.last() else v
        i_prev = buf_ipack.last(1)[-1] if buf_ipack.last() else i
        t_prev = buf_temp.last(1)[-1] if buf_temp.last() else t
        dv_dt = (v - v_prev) / dt
        di_dt = (i - i_prev) / dt
        dt_dt = (t - t_prev) / dt
    else:
        dv_dt = di_dt = dt_dt = 0.0

    rint = RINT.update(v, i)
    power     = v * i
    abs_i     = abs(i)
    v_margin  = 14.5 - v

    # Cell voltages from parameters (measured directly from STM32)
    # Fall back to last known values from buffer if not provided
    if c1 == 0.0 and buf_c1.last():
        c1 = buf_c1.last(1)[-1]
    if c2 == 0.0 and buf_c2.last():
        c2 = buf_c2.last(1)[-1]
    if c3 == 0.0 and buf_c3.last():
        c3 = buf_c3.last(1)[-1]

    i_std10, i_mean10 = rolling_stats(buf_ipack, 10)
    t_std10, t_mean10 = rolling_stats(buf_temp, 10)
    v_std10, v_mean10 = rolling_stats(buf_vpack, 10)

    # Build the specific feature vector expected by the model
    # Current Standard: 18 features (matches FEAT_NAMES)
    feats = [
        v, i, t, vs,           # 0-3
        c1, c2, c3,           # 4-6
        power, abs_i, v_margin, # 7-9
        dv_dt, di_dt, dt_dt,  # 10-12
        rint,                  # 13
        i_std10, t_std10, v_std10, # 14-16
        float(soc)             # 17
    ]
    
    # If the loaded model expects a different number of features (e.g. 20),
    # we adjust dynamically.
    if MODEL_FEATURE_COUNT == 20: 
        # Add legacy mean features if needed
        _, i_mean10 = rolling_stats(buf_ipack, 10)
        _, t_mean10 = rolling_stats(buf_temp, 10)
        _, v_mean10 = rolling_stats(buf_vpack, 10)
        feats_20 = [
            v, i, t, vs, c1, c2, c3, power, abs_i, v_margin,
            dv_dt, di_dt, dt_dt, rint, i_std10, t_std10, v_std10,
            i_mean10, t_mean10, v_mean10
        ]
        return feats_20, rint

    return feats[:int(MODEL_FEATURE_COUNT)], rint

# =========================
# MQTT (LIVE)
# =========================
MQTT_CLIENT = None

def start_mqtt():
    global MQTT_CLIENT
    if mqtt is None:
        log_event("ERR", "paho-mqtt not installed; LIVE disabled")
        return

    def on_connect(c, userdata, flags, rc):
        CONN["mqtt"] = "connected" if rc == 0 else f"rc={rc}"
        c.subscribe([(TOPIC_TICK, 0)])
        log_event("SYS", f"MQTT connected rc={rc}")

    def on_message(c, userdata, msg):
        try:
            payload = msg.payload.decode("utf-8", "ignore")
            data = json.loads(payload)
            handle_incoming_tick(data)
        except json.JSONDecodeError:
            # SoftwareSerial on ESP8266 sometimes drops bytes under heavy WiFi load.
            # We silently ignore corrupted JSON (e.g. missing characters in the middle)
            # because a fresh healthy packet will arrive in ~1 second anyway.
            pass
        except Exception as e:
            log_event("ERR", f"mqtt parse: {e}")

    def on_disconnect(c, userdata, rc):
        CONN["mqtt"] = "disconnected"
        log_event("SYS", "MQTT disconnected")

    c = mqtt.Client(client_id="bms-dashboard-ai", protocol=mqtt.MQTTv311)
    c.on_connect = on_connect
    c.on_message = on_message
    c.on_disconnect = on_disconnect

    # --- HiveMQ Cloud: TLS + username/password ---
    if MQTT_USERNAME:
        c.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    if MQTT_PORT in (8883, 8884):
        # Enable TLS for HiveMQ Cloud (port 8883/8884)
        import ssl
        c.tls_set(tls_version=ssl.PROTOCOL_TLS)

    try:
        c.connect_async(MQTT_BROKER, MQTT_PORT, 30)
        c.loop_start()
        MQTT_CLIENT = c
        log_event("SYS", f"MQTT connecting to {MQTT_BROKER}:{MQTT_PORT} (TLS={MQTT_PORT in (8883,8884)})")
    except Exception as e:
        log_event("ERR", f"mqtt connect: {e}")

def send_cmd(cmd: str):
    now = time.time()
    if LAST_CMD["name"] == cmd and now - LAST_CMD["t"] < CMD_MIN_INTERVAL_S:
        return
    LAST_CMD["name"] = cmd
    LAST_CMD["t"] = now
    if MQTT_CLIENT is None:
        log_event("CMD", f"(sim) {cmd}")
        return
    try:
        MQTT_CLIENT.publish(TOPIC_CMD, cmd, qos=0, retain=False)
        log_event("CMD", cmd)
    except Exception as e:
        log_event("ERR", f"cmd publish: {e}")

# =========================
# AI model (load + infer)
# =========================
# Global AI Models
MODEL = None           # ML Classification model (SAFE/WARN/DANGER predict karta hai)
MODEL_META = {}        # Model metadata (features, accuracy, etc.)
ANOM_MODEL = None      # Isolation Forest model (unusual patterns detect karta hai)
USE_MODEL = False      # Flag: AI model use karna hai ya nahi
MODEL_FEATURE_COUNT = 18 # Standard count for the new model


def _infer_model_feature_count(model) -> int:
    try:
        if hasattr(model, "named_steps") and "scaler" in model.named_steps:
            return int(model.named_steps["scaler"].n_features_in_)
        return int(model.n_features_in_)
    except Exception:
        return len(FEAT_NAMES)

def try_load_model(path: Optional[str]):
    """Saved model file (.joblib) ko load karo"""
    global MODEL, MODEL_META, ANOM_MODEL, USE_MODEL, MODEL_FEATURE_COUNT
    
    if not path:
        USE_MODEL = False
        return
    
    if not SKLEARN_AVAILABLE:
        log_event("ERR", "sklearn/joblib not installed; cannot load model")
        USE_MODEL = False
        return
    
    try:
        obj = joblib.load(path)
        if isinstance(obj, dict) and "clf" in obj:
            MODEL = obj["clf"]
            MODEL_META = obj.get("meta", {"features": FEAT_NAMES})
            ANOM_MODEL = obj.get("anom")
        else:
            MODEL = obj
            MODEL_META = {"features": FEAT_NAMES}
            ANOM_MODEL = None
        
        MODEL_FEATURE_COUNT = _infer_model_feature_count(MODEL)
        USE_MODEL = True
        log_event("AI", f"Model loaded: {os.path.basename(path)} ({MODEL_FEATURE_COUNT} features)")
    except Exception as e:
        USE_MODEL = False
        log_event("ERR", f"load model: {e}")

def model_predict(vfeat):
    pred_label = None
    proba = None
    anom_score = None
    risk = None

    if USE_MODEL and MODEL is not None:
        try:
            proba = MODEL.predict_proba([vfeat])[0]
            # Get actual class ordering from the trained model (alphabetical by default in sklearn)
            # This fixes the bug where proba[0] was assumed to be P(SAFE) but sklearn orders DANGER first
            clf = MODEL["clf"] if isinstance(MODEL, dict) else MODEL
            # Support both Pipeline and raw classifier
            try:
                classes = list(clf.classes_)
            except AttributeError:
                try:
                    classes = list(clf.named_steps[list(clf.named_steps.keys())[-1]].classes_)
                except Exception:
                    classes = ["DANGER", "SAFE", "WARN"]  # sklearn alphabetical default
            
            # Build {label: probability} dict from classes
            # Classes might be [0, 1, 2] or ['SAFE', 'WARN', 'DANGER']
            proba_dict = {}
            for cls, p in zip(classes, proba):
                label = str(IDX_TO_LABEL.get(cls, cls)) if isinstance(cls, (int, float)) or hasattr(cls, 'item') else str(cls)
                proba_dict[label] = float(p)
            
            # Extract in the order the frontend expects: [SAFE, WARN, DANGER]
            p_safe = proba_dict.get("SAFE", 0.0)
            p_warn = proba_dict.get("WARN", 0.0)
            p_dang = proba_dict.get("DANGER", 0.0)
            
            # Reorder proba so index 0=SAFE, 1=WARN, 2=DANGER (matches JS frontend)
            proba = [p_safe, p_warn, p_dang]
            
            # Predict label from max probability
            pred_label = max(proba_dict, key=proba_dict.get)
            risk = int(clamp(100.0*p_dang + 65.0*p_warn, 0, 100))
        except Exception as e:
            log_event("ERR", f"infer: {e}")

    # ============================================
    # ANOMALY DETECTION (Isolation Forest)
    # ============================================
    # Purpose: Unusual patterns detect karna jo training data mein nahi the
    # Example: Battery degradation, sensor malfunction, short circuit
    
    if ANOM_MODEL is not None:  # Agar Isolation Forest model loaded hai
        try:
            # Step 1: decision_function() se raw score nikalo
            # vfeat = [voltage, current, temp, solar, soc]
            # Return: Negative value (more negative = more anomalous)
            raw_score = ANOM_MODEL.decision_function([vfeat])[0]
            
            # Step 2: Invert karo (negative ko positive banao)
            # Ab: higher value = more anomalous
            s = -float(raw_score)
            
            # Step 3: 0-100 range mein convert karo
            # Reduced sensitivity: Score kam hoga normal variations ke liye
            # Previous: 100.0 * (s + 0.5) → Now: 70.0 * (s + 0.3)
            anom_score = int(clamp(70.0 * (s + 0.3), 0, 100))
            
            # Result:
            # 0-30:   Normal pattern ✅
            # 30-50:  Slightly unusual ⚠️
            # 50-70:  Moderate anomaly ⚠️
            # 70-100: Highly anomalous 🚨
        except Exception:
            anom_score = None

    return pred_label, risk, proba, anom_score

# =========================
# Policy application
# =========================
def get_descriptive_decision(m: dict) -> str:
    """Returns a user-friendly string explaining the current system decision."""
    v = float(m.get("v_pack", 0.0))
    i = float(m.get("i_pack", 0.0))
    t = float(m.get("t_amb", 0.0))
    vs = float(m.get("v_solar", 0.0))
    soc = float(m.get("soc", 0.0))
    risk = int(m.get("risk", 0))
    mode = m.get("mode", "AC")

    # Safety override check (matches logic in decide_actions)
    is_full = (soc >= 98)
    if is_full and t < T_DANGER and abs(i) < I_DANGER:
        return "✅ Battery Full: Maintain Top-off (Safe Override)"

    if m.get("f_ot") == 1 or t >= T_DANGER: return "🔥 Fire Protection Active (High Temp)"
    if m.get("f_ov") == 1 or v >= V_OV_DANGER: return "⚠️ Over-Voltage Protection Triggered"
    if m.get("f_uv") == 1 or v <= V_UV_DANGER: return "⚠️ Under-Voltage Recovery Active"
    if m.get("f_oc") == 1 or abs(i) >= I_DANGER: return "⚠️ Over-Current Protection!"
    
    if m.get("f_cell") == 1: return "⚠️ CELL FAULT: Check Individual Balance"

    if risk >= RISK_CUT: return "🚨 EMERGENCY: CUT_ON triggered (AI Danger)"
    if risk >= RISK_WARN: return "⚠️ WARNING: Monitoring closely"
    
    if mode == "SOLAR": return "☀️ Using Solar (Optimal)"
    return "⚡ Using AC Charging"

def decide_actions(m: dict):
    if not AUTO_ACTIONS:
        return
    v = float(m.get("v_pack", 0.0))
    i = float(m.get("i_pack", 0.0))
    t = float(m.get("t_amb", 0.0))
    vs = float(m.get("v_solar", 0.0))

    # -----------------------------------------------------
    # Motor Inrush Current Bypass
    # -----------------------------------------------------
    # Give the motor 4 seconds to start before enforcing OC limits
    startup_active = (time.time() - m.get("_startup_ts", 0)) < 4.0

    # Hardware fault active? (Ignore OC and Voltage Drop during startup bypass)
    f_ov = m.get("f_ov") == 1
    f_uv = m.get("f_uv") == 1
    f_oc = m.get("f_oc") == 1
    f_ot = m.get("f_ot") == 1
    f_cell = m.get("f_cell") == 1
    
    # We ignore UV (voltage sag) and OC (current spike) during the 4-second motor startup window
    hw_fault = f_ov or f_ot or f_cell
    if not startup_active:
        hw_fault = hw_fault or f_uv or f_oc

    if hw_fault:
        # STM32 is asserting hard protection; do not fight it.
        return


    # Heuristic safety (Ignore 0V if already in CUT mode to allow recovery)
    is_cut_state = (m.get("flags", {}).get("cut", 0) == 1)
    
    # If not in cut state, 0V is a danger. If already in cut state, 0V is expected.
    # Ignore UV danger during startup
    v_critical = (v >= V_OV_DANGER) or (not is_cut_state and v <= V_UV_DANGER and not startup_active)
    
    # NEW: Safety override for full batteries. 
    # If SOC is high and there's no real heat or current surge, ignore the AI "DANGER" 
    # to prevent recursive charging cut-off loops.
    is_full = (m.get("soc", 0) >= 98)
    if is_full and t < T_DANGER and abs(i) < I_DANGER:
        # If we are already full and AI says DANGER, it's likely a 0V sensor artifact.
        # We allow it to remain AC_ON to maintain top-off charging.
        mode = m.get("mode", "AC")
        if mode != "AC": send_cmd("AC_ON")
        return

    if t >= T_DANGER or v_critical:
        # Only send CUT_ON if not already cut (break infinite loop)
        if not is_cut_state:
            send_cmd("CUT_ON")
        return

    # Ignore OC danger during startup
    if abs(i) >= I_DANGER and not startup_active:
        if not is_cut_state:
            send_cmd("CUT_ON")
        return

    risk = int(m.get("risk", compute_risk(v, i, t)))
    # Ignore Risk cutoff during startup
    if risk >= RISK_CUT and not is_cut_state and not startup_active:
        send_cmd("CUT_ON"); return

    mode = m.get("mode", "AC")
    if (vs > v + SOLAR_MARGIN_V) and (risk < RISK_WARN):
        if mode != "SOLAR": send_cmd("SOLAR_ON")
    else:
        if mode != "AC": send_cmd("AC_ON")

# =========================
# Tick ingest & broadcast
# =========================
_last_lcd_page = None
_last_alarms = {}
_motor_startup_time = 0.0

def handle_incoming_tick(tick: dict):
    global _last_lcd_page, _last_alarms, _motor_startup_time
    # Force use of server's current absolute time instead of STM32 uptime.
    ts = time.time()
    v  = float(tick.get("v_pack", 0.0))
    vs = float(tick.get("v_solar", 0.0))
    i  = float(tick.get("i_pack", 0.0))
    t  = float(tick.get("t_amb",  0.0))
    mode = tick.get("mode", "AC")

    # Relay 0V Ignore Filter:
    # When CUT relay disconnected, STM32 sends 0V for pack and cells. AI thinks it's dead.
    is_cut_state = tick.get("flags", {}).get("cut", 0) == 1
    if v < 2.0 or is_cut_state:
        prev_vals = [x for x in buf_vpack.list() if x > 4.0]
        if prev_vals:
            v = prev_vals[-1]
        elif v < 2.0:
            v = 11.5 # Safe default fallback
            
    # Motor Inrush Current Bypass
    # If we just left CUT state, record the timestamp to allow a high-current spike for startup
    if not is_cut_state and buf_mode.list() and buf_mode.list()[-1] == MODE_TO_IDX.get("CUT", 0):
        _motor_startup_time = ts
        
    startup_active = (ts - _motor_startup_time) < 4.0
    tick["_startup_ts"] = _motor_startup_time # pass down to decide_actions

    # Hide the massive inrush current from the AI model so it doesn't scream DANGER
    ai_i = i
    if startup_active and ai_i > (I_DANGER * 0.8):
        ai_i = I_DANGER * 0.8 # Cap it visually for the AI to keep risk low during startup
    
    # Cells
    c1_raw = float(tick.get("c1", 0.0))
    c2_raw = float(tick.get("c2", 0.0))
    c3_raw = float(tick.get("c3", 0.0))
    
    if c1_raw < 0.5:
        prev_c1 = buf_c1.list()
        if prev_c1: c1_raw = next((x for x in reversed(prev_c1) if x > 0.5), c1_raw)
    if c2_raw < 0.5:
        prev_c2 = buf_c2.list()
        if prev_c2: c2_raw = next((x for x in reversed(prev_c2) if x > 0.5), c2_raw)
    if c3_raw < 0.5:
        prev_c3 = buf_c3.list()
        if prev_c3: c3_raw = next((x for x in reversed(prev_c3) if x > 0.5), c3_raw)

    # Alarm logging on transition (avoid log spam)
    alarms = tick.get("alarms", {})
    if alarms:
        for alrm_key, display_name in [("ov","Over-Voltage"), ("uv","Under-Voltage"), ("oc","Over-Current"), ("ot","Over-Temperature"), ("cell","Cell-Imbalance")]:
            current_val = alarms.get(alrm_key, 0)
            last_val = _last_alarms.get(alrm_key, 0)
            if current_val == 1 and last_val == 0:
                log_event("WARN", f"{display_name} Detected!")
            elif current_val == 0 and last_val == 1:
                log_event("SYS", f"{display_name} Cleared.")
            _last_alarms[alrm_key] = current_val

    # Stats & deriveds
    STAT_I.update(i); STAT_T.update(t); STAT_V.update(v)
    power = v * i
    soc_in = tick.get("soc", None)
    soc = int(soc_in) if isinstance(soc_in, (int,float)) else est_soc_from_voltage(v)

    # Features + AI — pass cell values directly so build_feature_vector has them
    feats, rint_val = build_feature_vector(ts, v, i, t, vs, soc, c1_raw, c2_raw, c3_raw)
    pred_ai, risk_ai, proba, anom_score = model_predict(feats)


    # Fallback heuristic if needed
    if risk_ai is None:
        risk = compute_risk(v, i, t)
        pred = tick.get("pred", "SAFE" if risk < RISK_WARN else ("WARN" if risk < RISK_CUT else "DANGER"))
    else:
        risk = risk_ai
        pred = pred_ai or ("SAFE" if risk < RISK_WARN else ("WARN" if risk < RISK_CUT else "DANGER"))

    # SOH / SOP
    maybe_set_rint_baseline(rint_val, ai_i, t, risk)
    soh = tick.get("soh", None)
    soh = int(soh) if isinstance(soh, (int,float)) else estimate_soh_from_rint(rint_val)
    sop_pct = estimate_sop_pct(v, rint_val)

    # Temp pattern
    tpat = "STABLE"
    if buf_temp.last():
        dt_last = t - buf_temp.last(1)[-1]
        tpat = "RISING" if dt_last > 0.05 else ("FALLING" if dt_last < -0.05 else "STABLE")

    # Anomaly & fire-risk
    sigma_i = STAT_I.std()
    z_i = abs((ai_i - STAT_I.mean) / sigma_i) if sigma_i > 1e-6 else 0.0
    
    # Base heuristic anomaly
    heuristic_anomaly = int(clamp(15*z_i + max(0, t - 32)*8 + abs(ai_i)*10, 0, 100))
    anomaly = anom_score if anom_score is not None else heuristic_anomaly
    
    fire_risk = int(clamp(max(0, t - 40) * 8 + abs(ai_i) * 6 + (risk * 0.2), 0, 100))

    # HARDWARE FAULT OVERRIDES
    # If the STM32 has tripped a physical safety limit, the anomaly MUST be high, regardless of ML model.
    alarms = tick.get("alarms", {})
    if alarms.get("ot") == 1 or t >= T_DANGER:
        anomaly = max(anomaly, int(clamp(75 + int(max(0.0, t - T_DANGER)*2), 85, 100)))
        fire_risk = max(fire_risk, int(clamp(60 + int(max(0.0, t - T_DANGER)*3), 80, 100)))
    if alarms.get("oc") == 1 or abs(i) >= I_DANGER:
        anomaly = max(anomaly, 90)
    if alarms.get("ov") == 1 or alarms.get("uv") == 1:
        anomaly = max(anomaly, 85)

    # Quick projections (simple heuristics)
    def predict_voltage_delta_30s():
        xs = buf_ts.last(20)
        ys = buf_vpack.last(20)
        if len(xs) < 3 or len(xs) != len(ys): return 0.0
        t0 = xs[0]; t = [x - t0 for x in xs]
        n = float(len(t))
        sumx = sum(t); sumy = sum(ys)
        sumxx = sum(x*x for x in t); sumxy = sum(t[i]*ys[i] for i in range(len(t)))
        denom = (n*sumxx - sumx*sumx)
        if abs(denom) < 1e-6: return 0.0
        slope = (n*sumxy - sumx*sumy) / denom  # V/s
        return float(slope * 30.0)

    def spike_probability_from_stats(i_now: float) -> float:
        sigma = STAT_I.std()
        if sigma <= 1e-6: return 0.15
        z = abs((i_now - STAT_I.mean) / max(sigma, 1e-6))
        p = 0.15 + 0.12 * min(z, 5.0) + 0.2 * min(sigma / 0.5, 1.0)
        return float(clamp(p, 0.0, 1.0))

    v_dev_pred = float(predict_voltage_delta_30s())
    i_spike_prob = float(spike_probability_from_stats(i))

    # Cells (placeholder or pass-through if provided)
    cells = tick.get("cell_v", None)
    if cells is None:
        v1 = tick.get("v1", None); v2 = tick.get("v2", None); v3 = tick.get("v3", None)
        if all(isinstance(x, (int,float)) for x in (v1,v2,v3)):
            cells = [float(v1), float(v2), float(v3)]
    if not (isinstance(cells, list) and len(cells) == CELL_COUNT):
        cell_v = estimate_cells_from_pack(v, None)
    else:
        cell_v = [float(x) for x in cells]
    imb_mv, balance_score = cell_metrics(cell_v)
    balance_needed = (imb_mv >= BALANCE_THRESH_mV)

    # LCD page log (if provided)
    lcd_page = tick.get("lcd_page", None)
    if isinstance(lcd_page, (int, float)):
        lp = int(lcd_page)
        global _last_lcd_page
        if _last_lcd_page is None:
            _last_lcd_page = lp
        elif lp != _last_lcd_page:
            log_event("LCD", f"Page changed to {lp}")
            _last_lcd_page = lp

    # Push buffers (AFTER using previous values for derivatives)
    buf_ts.push(ts); buf_vpack.push(v); buf_vsolar.push(vs)
    buf_ipack.push(i); buf_temp.push(t); buf_power.push(power)
    buf_c1.push(tick.get("c1",0)); buf_c2.push(tick.get("c2",0)); buf_c3.push(tick.get("c3",0))
    buf_risk.push(risk); buf_pred.push(LABEL_TO_IDX.get(pred, 0))
    buf_mode.push(MODE_TO_IDX.get(mode, 1))
    buf_soc.push(soc); buf_soh.push(soh); buf_sop.push(sop_pct)
    buf_anom.push(anomaly); buf_frisk.push(fire_risk)
    buf_vdev.push(v_dev_pred); buf_ispike.push(i_spike_prob)
    buf_tpat.push(1 if tpat == "RISING" else (-1 if tpat == "FALLING" else 0))
    buf_rint.push(rint_val)

    # Connection stats
    now = time.time(); last = CONN.get("last_tick", 0.0)
    if last > 0:
        dtt = now - last
        if dtt > 0: CONN["rate_hz"] = 0.6 * CONN["rate_hz"] + 0.4 * (1.0 / dtt)
    CONN["last_tick"] = now

    # Broadcast (UI source of truth)
    alarms_raw = tick.get("alarms", {})
    payload = {
        "ts": ts,
        "v_pack": v, "v_solar": vs, "i_pack": i, "t_amb": t,
        "power": power,
        "risk": risk, "pred": pred, "mode": mode,
        "proba": [round(float(p), 4) for p in (proba if proba is not None else [0.0, 0.0, 0.0])],
        "soc": soc, "soh": soh, "sop": sop_pct,
        "anomaly": anomaly, "fire_risk": fire_risk,
        "temp_pattern": tpat,
        "v_dev_pred": v_dev_pred, "i_spike_prob": i_spike_prob,
        "flags": tick.get("flags", {"fan":0,"ac":1,"solar":0,"cut":0,"charging":0}),
        # Alarm flags from STM32 JSON directly
        "f_ov": alarms_raw.get("ov", 0), "f_uv": alarms_raw.get("uv", 0),
        "f_oc": alarms_raw.get("oc", 0), "f_ot": alarms_raw.get("ot", 0),
        "f_cell": alarms_raw.get("cell", 0),
        "voice_msg": tick.get("voice_msg", "NORMAL"),
        "meta": {"hz": round(float(CONN.get("rate_hz", 0.0)), 2), "rint_ohm": round(float(rint_val), 4)},
        "cells": {"v": cell_v, "imbalance_mV": imb_mv, "balance_score": int(balance_score), "needed": balance_needed},
        "c1": c1_raw, "c2": c2_raw, "c3": c3_raw,
    }
    
    # Calculate final descriptive decision (Source of Truth)
    payload["decision"] = get_descriptive_decision(payload)

    if lcd_page is not None:
        payload["lcd_page"] = int(lcd_page)

    socketio.emit("tick", payload)

    # Apply policy
    decide_actions(payload)


# =========================
# Simulator
# =========================
def simulate_loop():
    log_event("SYS", "Simulator started")
    t0 = time.time(); phi = random.random() * math.pi
    soh = 96.0; sop = 82.0; last_t = 28.0
    lcd_page = 0
    while True:
        now = time.time(); dt = now - t0
        vs = 18.0 + 2.0 * math.sin(0.02 * dt + phi)
        v  = 12.1 + 0.25 * math.sin(0.015 * dt + 0.7) + 0.05 * random.uniform(-1, 1)
        i  =  1.2 * math.sin(0.03 * dt) + 0.12 * random.uniform(-1, 1)
        t  = 28.0 + 3.0 * math.sin(0.01 * dt + 1.1)
        risk = compute_risk(v, i, t)
        mode = "SOLAR" if (vs > v + 4.0 and risk < RISK_WARN) else ("CUT" if (risk >= RISK_CUT or v > 12.55) else "AC")
        flags = {"fan": 1 if risk >= RISK_WARN else 0, "ac": 1 if mode == "AC" else 0,
                 "solar": 1 if mode == "SOLAR" else 0, "cut": 1 if mode == "CUT" else 0}
        soc = est_soc_from_voltage(v)
        soh = clamp(soh + random.uniform(-0.01, 0.01), 90, 100)
        sop = clamp(sop + random.uniform(-0.05, 0.05), 60, 100)
        cells = [v/3 + d for d in (0.00, 0.004, -0.004)]
        lcd_page = (lcd_page + (1 if random.random()<0.02 else 0)) % 5
        tick = {
            "ts": now, "v_pack": v, "i_pack": i, "t_amb": t, "v_solar": vs,
            "risk": risk, "pred": "SAFE" if risk < RISK_WARN else ("WARN" if risk < RISK_CUT else "DANGER"),
            "mode": mode, "flags": flags,
            "soc": soc, "soh": int(soh), "sop": int(sop),
            "cell_v": cells, "lcd_page": lcd_page
        }
        handle_incoming_tick(tick)
        time.sleep(0.5)

def simulate_eis_once():
    if np is None: return
    R0, R1, C1 = 0.05, 0.25, 0.8
    f = np.logspace(-1, 3, 60); w = 2 * np.pi * f
    Z = R0 + R1 / (1 + 1j * w * R1 * C1)
    payload = {"freq": f.tolist(), "Zre": np.real(Z).tolist(), "Zim": np.imag(Z).tolist()}
    EIS_SWEEPS.append(payload); socketio.emit("eis", payload)
    log_event("EIS", f"sweep {len(f)} pts (sim)")

# =========================
# Web UI (dark) — UNCHANGED
# =========================
DARK_CSS = """
:root{
  --bg:#f0f4f8; --panel:rgba(255,255,255,0.7); --text:#1e293b; --sub:#64748b;
  --ok:#10b981; --warn:#f59e0b; --danger:#ef4444; --accent:#3b82f6; --bdr:rgba(255,255,255,0.6);
  --glow: 0 8px 32px rgba(31, 38, 135, 0.1);
}
@keyframes float { 0%{transform:translateY(0px)} 50%{transform:translateY(-5px)} 100%{transform:translateY(0px)} }
@keyframes gradient-x { 0%{background-position:0% 50%} 50%{background-position:100% 50%} 100%{background-position:0% 50%} }
*{box-sizing:border-box}
body{
  margin:0; color:var(--text); font-family:Inter, sans-serif;
  background: linear-gradient(-45deg, #eef2f3, #e8e9eb, #e2ebf0, #eef1f5);
  background-size: 400% 400%; animation: gradient-x 15s ease infinite;
}
a{color:var(--accent); text-decoration:none; font-weight:700}
.header{position:sticky; top:0; z-index:50; display:flex; align-items:center; gap:12px; padding:16px 24px; background:rgba(255,255,255,0.6); backdrop-filter:blur(16px); border-bottom:1px solid rgba(255,255,255,0.3); box-shadow:0 1px 10px rgba(0,0,0,0.03)}
.title{font-weight:900; font-size:24px; letter-spacing:-0.5px; background:linear-gradient(to right, #2563eb, #7c3aed); -webkit-background-clip:text; -webkit-text-fill-color:transparent;}
.badge{padding:6px 14px; border-radius:99px; background:#fff; box-shadow:0 2px 5px rgba(0,0,0,0.05); color:#64748b; font-size:12px; font-weight:700; border:1px solid rgba(0,0,0,0.05)}
.badge.ok{background:#d1fae5; color:#065f46; border-color:#a7f3d0}
.badge.warn{background:#fef3c7; color:#92400e; border-color:#fde68a}
.badge.danger{background:#fee2e2; color:#991b1b; border-color:#fecaca}
.container{padding:30px; display:grid; grid-template-columns:2.5fr 1fr; gap:30px; max-width:1800px; margin:0 auto}
@media (max-width:1100px){.container{grid-template-columns:1fr}}
.card{background:var(--panel); backdrop-filter:blur(12px); border:1px solid var(--bdr); border-radius:24px; box-shadow:var(--glow); overflow:hidden}
.card .hd{display:flex; justify-content:space-between; align-items:center; padding:20px 24px; border-bottom:1px solid var(--bdr); background:rgba(255,255,255,0.4)}
.bigname{font-weight:800; font-size:18px; color:#334155}
.section-grid{display:grid; grid-template-columns:1fr; gap:20px; padding:24px}
.kpis{display:grid; grid-template-columns:repeat(5,1fr); gap:20px}
@media (max-width:1400px){.kpis{grid-template-columns:repeat(3,1fr)}}
@media (max-width:700px){.kpis{grid-template-columns:repeat(2,1fr)}}
@media (max-width:480px){.kpis{grid-template-columns:1fr}}
.kpi{background:#fff; border-radius:18px; padding:20px; border:1px solid #fff; box-shadow:0 4px 6px -1px rgba(0, 0, 0, 0.05), 0 2px 4px -1px rgba(0, 0, 0, 0.03); transition:all 0.3s ease; position:relative}
.kpi:hover{transform:translateY(-5px); box-shadow:0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 10px 10px -5px rgba(0, 0, 0, 0.04); border-color:#cbd5e1}
.kpi .lbl{font-size:12px; color:#64748b; font-weight:700; text-transform:uppercase; letter-spacing:1px; margin-bottom:6px}
.kpi .val{font-size:34px; font-weight:800; color:#1e293b; letter-spacing:-1.5px}
/* Gradients for KPI values */
.kpi:nth-child(1) .val{background:linear-gradient(to right, #2563eb, #3b82f6); -webkit-background-clip:text; -webkit-text-fill-color:transparent}
.kpi:nth-child(2) .val{background:linear-gradient(to right, #0891b2, #06b6d4); -webkit-background-clip:text; -webkit-text-fill-color:transparent}
.kpi:nth-child(3) .val{background:linear-gradient(to right, #ea580c, #f97316); -webkit-background-clip:text; -webkit-text-fill-color:transparent}
.kpi:nth-child(4) .val{background:linear-gradient(to right, #ca8a04, #eab308); -webkit-background-clip:text; -webkit-text-fill-color:transparent}
.spark{height:50px; margin-top:12px; opacity:0.8}
.plot{height:280px; width:100%}
.log{max-height:550px; overflow:auto; padding:16px; font-family:'Menlo', monospace; font-size:13px; background:rgba(255,255,255,0.5)}
.log .item{padding:10px 14px; border-bottom:1px solid rgba(0,0,0,0.05); color:#475569; border-radius:8px; margin-bottom:4px}
.log .item:hover{background:#fff}
.controls{display:grid; grid-template-columns:repeat(auto-fit, minmax(110px, 1fr)); gap:16px; padding:24px}
.btn{background:#fff; border:1px solid #e2e8f0;      border-radius:8px; padding:6px 12px; font-size:0.9em; font-weight:600;
    }font-size:12px; transition:all 0.2s cubic-bezier(0.4, 0, 0.2, 1); box-shadow:0 2px 5px rgba(0,0,0,0.05); text-transform:uppercase; letter-spacing:0.5px}
.btn:hover{transform:translateY(-2px); box-shadow:0 10px 15px -3px rgba(0, 0, 0, 0.1); color:#0f172a; border-color:#94a3b8}
.btn:active{transform:scale(0.98)}
.footer{padding:16px 24px; border-top:1px solid var(--bdr); color:#94a3b8; font-size:13px; text-align:center; background:rgba(255,255,255,0.4)}
"""

INDEX_HTML = """
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Smart EV BMS — Dashboard</title>
  <style>{{css}}</style>
  <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1"></script>
</head>
<body>
  <div class="header">
    <div class="title">Smart EV BMS</div>
    <div id="chip-mqtt" class="badge">MQTT: …</div>
    <div id="chip-rate" class="badge">Rate: …</div>
    <div id="chip-ai" class="badge ok">SAFE</div>
    <div id="chip-risk" class="badge">Risk: 0</div>
    <div id="chip-mode" class="badge">Mode: —</div>
    <div id="chip-charging" class="badge">🔋 Not Charging</div>
    <button class="badge" id="autoBtn" onclick="toggleAuto()">Auto AI: ON</button>
    <a class="link" href="/deep-dive">Full Graphs ▸</a>
  </div>

  <div class="container">

    <!-- LEFT: KPIs + separated chart cards -->
    <div>
      <div class="card">
        <div class="hd"><span class="bigname">Live KPIs</span></div>
        <div class="section-grid">
          <div class="kpis">
            <div class="kpi"><div class="lbl">Pack Voltage</div><div id="k-v" class="val">— V</div><canvas id="spV" class="spark"></canvas></div>
            <div class="kpi"><div class="lbl">Current</div><div id="k-i" class="val">— A</div><canvas id="spI" class="spark"></canvas></div>
            <div class="kpi"><div class="lbl">Temperature</div><div id="k-t" class="val">— °C</div><canvas id="spT" class="spark"></canvas></div>
            <div class="kpi"><div class="lbl">Solar Voltage</div><div id="k-s" class="val">— V</div><canvas id="spS" class="spark"></canvas></div>
            <div class="kpi"><div class="lbl">Power</div><div id="k-p" class="val">— W</div><canvas id="spP" class="spark"></canvas></div>
            <div class="kpi"><div class="lbl">SOC</div><div id="k-so" class="val">— %</div><canvas id="spSO" class="spark"></canvas></div>
            <div class="kpi"><div class="lbl">SOH</div><div id="k-sh" class="val">— %</div><canvas id="spSH" class="spark"></canvas></div>
            <div class="kpi"><div class="lbl">SOP</div><div id="k-sp" class="val">— %</div><canvas id="spSP" class="spark"></canvas></div>
            <div class="kpi"><div class="lbl">Anomaly</div><div id="k-a" class="val">—</div><canvas id="spA" class="spark"></canvas></div>
            <div class="kpi"><div class="lbl">Fire-risk</div><div id="k-f" class="val">— %</div><canvas id="spF" class="spark"></canvas></div>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="hd"><span class="bigname">Voltage</span></div>
        <div class="section-grid"><canvas id="cV" class="plot"></canvas></div>
        <div class="footer">Pack V vs Solar V</div>
      </div>
      
      <!-- NEW CARD for Individual Cell Voltages -->
      <div class="card">
        <div class="hd"><span class="bigname">Cell Voltages</span></div>
        <div class="section-grid"><canvas id="cC" class="plot"></canvas></div>
        <div class="footer">Individual Cell Tracking (C1, C2, C3)</div>
      </div>

      <div class="card">
        <div class="hd"><span class="bigname">Current (Amps)</span></div>
        <div class="section-grid"><canvas id="cI" class="plot"></canvas></div>
        <div class="footer">Charge (+) / Discharge (–)</div>
      </div>

      <div class="card">
        <div class="hd"><span class="bigname">Temperature</span> <span id="chip-tpat" class="badge">Pattern: …</span></div>
        <div class="section-grid"><canvas id="cT" class="plot"></canvas></div>
        <div class="footer">Battery bay trend</div>
      </div>

      <div class="card">
        <div class="hd"><span class="bigname">Risk</span></div>
        <div class="section-grid"><canvas id="cR" class="plot"></canvas></div>
        <div class="footer">AI Risk 0–100</div>
      </div>

      <div class="card">
        <div class="hd"><span class="bigname">Power</span></div>
        <div class="section-grid"><canvas id="cP" class="plot"></canvas></div>
        <div class="footer">Instant power = V × I</div>
      </div>

      <!-- ANOMALY DETECTION GRAPH -->
      <div class="card">
        <div class="hd"><span class="bigname">🔍 Anomaly Detection</span> <span id="chip-anom-status" class="badge ok">Normal</span></div>
        <div class="section-grid"><canvas id="cA" class="plot"></canvas></div>
        <div class="footer">Unusual pattern detection (0-100)</div>
      </div>
    </div>

    <!-- RIGHT: AI Processing & Event Log -->
    <div>
      <!-- AI Processing Card -->
      <div class="card">
        <div class="hd"><span class="bigname">🧠 AI Processing</span> <span id="chip-ai-status" class="badge ok">Active</span></div>
        <div style="padding:20px; font-size:13px; line-height:1.8">
          
          <!-- Step 1: Feature Extraction -->
          <div style="margin-bottom:16px">
            <div style="font-weight:700; color:#0f172a; margin-bottom:8px">📊 Step 1: Feature Extraction</div>
            <div style="background:rgba(99,102,241,0.08); padding:12px; border-radius:8px; border-left:3px solid #6366f1">
              <div>Voltage: <span id="ai-feat-v" style="font-weight:700">--</span> V</div>
              
              <!-- NEW: Individual Cell Voltages -->
              <div style="margin-top:6px; margin-bottom:6px; padding:6px 0; border-top:1px solid rgba(0,0,0,0.05); border-bottom:1px solid rgba(0,0,0,0.05); display:flex; justify-content:space-between;">
                <div>C1: <span id="ai-feat-c1" style="font-weight:700; color:#3b82f6">--</span>V</div>
                <div>C2: <span id="ai-feat-c2" style="font-weight:700; color:#3b82f6">--</span>V</div>
                <div>C3: <span id="ai-feat-c3" style="font-weight:700; color:#3b82f6">--</span>V</div>
              </div>

              <div>Current: <span id="ai-feat-i" style="font-weight:700">--</span> A</div>
              <div>Temperature: <span id="ai-feat-t" style="font-weight:700">--</span> °C</div>
              <div>Solar: <span id="ai-feat-s" style="font-weight:700">--</span> V</div>
              <div>SOC: <span id="ai-feat-soc" style="font-weight:700">--</span> %</div>
            </div>
          </div>

          <!-- Step 2: ML Prediction -->
          <div style="margin-bottom:16px">
            <div style="font-weight:700; color:#0f172a; margin-bottom:8px">🤖 Step 2: ML Model Prediction</div>
            <div style="background:rgba(139,92,246,0.08); padding:12px; border-radius:8px; border-left:3px solid #8b5cf6">
              <div>P(SAFE): <span id="ai-prob-safe" style="font-weight:700; color:#10b981">--</span>%</div>
              <div>P(WARN): <span id="ai-prob-warn" style="font-weight:700; color:#f59e0b">--</span>%</div>
              <div>P(DANGER): <span id="ai-prob-dang" style="font-weight:700; color:#ef4444">--</span>%</div>
              <div style="margin-top:8px; padding-top:8px; border-top:1px solid rgba(0,0,0,0.1)">
                Prediction: <span id="ai-pred-label" style="font-weight:700; font-size:14px">--</span>
              </div>
            </div>
          </div>

          <!-- Step 3: Risk Score -->
          <div style="margin-bottom:16px">
            <div style="font-weight:700; color:#0f172a; margin-bottom:8px">📈 Step 3: Risk Calculation</div>
            <div style="background:rgba(236,72,153,0.08); padding:12px; border-radius:8px; border-left:3px solid #ec4899">
              <div>Risk Score: <span id="ai-risk-score" style="font-weight:700; font-size:16px">--</span> / 100</div>
              <div style="margin-top:6px; font-size:11px; color:#64748b">Formula: 100×P(DANGER) + 65×P(WARN)</div>
            </div>
          </div>

          <!-- Step 4: Anomaly Detection -->
          <div style="margin-bottom:16px">
            <div style="font-weight:700; color:#0f172a; margin-bottom:8px">🔍 Step 4: Anomaly Detection</div>
            <div style="background:rgba(244,114,182,0.08); padding:12px; border-radius:8px; border-left:3px solid #f472b6">
              <div>Anomaly Score: <span id="ai-anom-score" style="font-weight:700">--</span></div>
              <div style="font-size:11px; color:#64748b; margin-top:4px">Higher = More unusual pattern</div>
            </div>
          </div>

          <!-- Step 5: Decision -->
          <div>
            <div style="font-weight:700; color:#0f172a; margin-bottom:8px">⚡ Step 5: Auto Decision</div>
            <div style="background:rgba(34,197,94,0.08); padding:12px; border-radius:8px; border-left:3px solid #22c55e">
              <div id="ai-decision" style="font-weight:700; font-size:13px">Waiting for data...</div>
            </div>
          </div>

        </div>
        <div class="footer">AI updates every second</div>
      </div>

      <!-- System Log Card -->
    <div class="card">
      <div class="hd"><span class="bigname">System Log</span></div>
      <div id="log" class="log"></div>
      <div class="footer">Data: <a href="/export.csv">Export CSV</a></div>
    </div>
  </div>

<script>
const sock = io();
const logBox = document.getElementById('log');
const chip = (id, cls, txt)=>{ const e=document.getElementById(id); e.className='badge '+(cls||''); e.innerText=txt; };
const chipOnly = (id, txt)=>{ const e=document.getElementById(id); e.innerText=txt; };
let VOICE = true;


let AUTO = true;
function toggleAuto(){ AUTO=!AUTO; sock.emit('cmd', {policy:{auto:AUTO}}); document.getElementById('autoBtn').innerText = 'Auto AI: '+(AUTO?'ON':'OFF'); }

sock.on('status', s=>{ chip('chip-mqtt', s.mqtt==='connected'?'ok':'', 'MQTT: '+s.mqtt); chip('chip-rate','', 'Rate: '+(s.hz||0)+' Hz'); });
function pushLog(k,d){ const el=document.createElement('div'); el.className='item'; el.textContent='['+new Date().toLocaleTimeString()+'] '+k+' — '+d; logBox.prepend(el); }
sock.on('event', e=>pushLog(e.kind,e.detail));

let xs=[], v=[], vs=[], i=[], t=[], r=[], p=[], so=[], sh=[], sp=[], a=[], f=[], c1=[], c2=[], c3=[];
function trim(a,n){ while(a.length>n) a.shift(); }
function badgeClass(pred){ return pred==='DANGER'?'danger' : (pred==='WARN'?'warn':'ok'); }

sock.on('tick', m=>{
  chip('chip-ai', badgeClass(m.pred), m.pred);
  chip('chip-risk','', 'Risk: '+m.risk);
  chip('chip-mode','', 'Mode: '+m.mode);
  chipOnly('chip-tpat', 'Pattern: '+m.temp_pattern);
  
  // Update charging status
  const chg = m.flags?.charging || 0;
  if (chg === 2) {
    chip('chip-charging', 'ok', '☀️ Solar Charging');
  } else if (chg === 1) {
    chip('chip-charging', 'warn', '⚡ AC Charging');
  } else {
    chip('chip-charging', '', '🔋 Not Charging');
  }

  if(m.f_ov) pushLog('FAULT', 'Over Voltage Detected! (>13.7V)');
  if(m.f_uv) pushLog('FAULT', 'Under Voltage Detected! (<7.0V)');
  if(m.f_oc) pushLog('FAULT', 'Over Current Detected! (>3.0A)');
  if(m.f_ot) pushLog('FAULT', 'Over Temperature! (>40°C)');

  document.getElementById('k-v').innerText  = m.v_pack.toFixed(2)+' V';
  document.getElementById('k-i').innerText  = m.i_pack.toFixed(2)+' A';
  document.getElementById('k-t').innerText  = m.t_amb.toFixed(1)+' °C';
  document.getElementById('k-s').innerText  = m.v_solar.toFixed(2)+' V';
  document.getElementById('k-p').innerText  = m.power.toFixed(1)+' W';
  document.getElementById('k-so').innerText = m.soc.toFixed(0)+' %';
  document.getElementById('k-sh').innerText = (m.soh||0).toFixed(0)+' %';
  document.getElementById('k-sp').innerText = (m.sop||0).toFixed(0)+' %';
  document.getElementById('k-a').innerText  = (m.anomaly||0).toFixed(0);
  document.getElementById('k-f').innerText  = (m.fire_risk||0).toFixed(0)+' %';

  // Update AI Processing visualization
  document.getElementById('ai-feat-v').innerText = m.v_pack.toFixed(2);
  
  // NEW: Update Cell Voltages
  document.getElementById('ai-feat-c1').innerText = (m.c1||0).toFixed(2);
  document.getElementById('ai-feat-c2').innerText = (m.c2||0).toFixed(2);
  document.getElementById('ai-feat-c3').innerText = (m.c3||0).toFixed(2);
  
  // Basic fault highlight on cells
  if (m.c1 < 3.0 || m.c1 > 4.25) document.getElementById('ai-feat-c1').style.color = '#ef4444'; else document.getElementById('ai-feat-c1').style.color = '#3b82f6';
  if (m.c2 < 3.0 || m.c2 > 4.25) document.getElementById('ai-feat-c2').style.color = '#ef4444'; else document.getElementById('ai-feat-c2').style.color = '#3b82f6';
  if (m.c3 < 3.0 || m.c3 > 4.25) document.getElementById('ai-feat-c3').style.color = '#ef4444'; else document.getElementById('ai-feat-c3').style.color = '#3b82f6';

  document.getElementById('ai-feat-i').innerText = m.i_pack.toFixed(2);
  document.getElementById('ai-feat-t').innerText = m.t_amb.toFixed(1);
  document.getElementById('ai-feat-s').innerText = m.v_solar.toFixed(2);
  document.getElementById('ai-feat-soc').innerText = m.soc.toFixed(0);

  const proba = m.proba || [0, 0, 0];
  document.getElementById('ai-prob-safe').innerText = (proba[0]*100).toFixed(1);
  document.getElementById('ai-prob-warn').innerText = (proba[1]*100).toFixed(1);
  document.getElementById('ai-prob-dang').innerText = (proba[2]*100).toFixed(1);
  document.getElementById('ai-pred-label').innerText = m.pred;
  document.getElementById('ai-pred-label').style.color = m.pred==='DANGER'?'#ef4444':(m.pred==='WARN'?'#f59e0b':'#10b981');

  document.getElementById('ai-risk-score').innerText = m.risk;
  document.getElementById('ai-risk-score').style.color = m.risk>=85?'#ef4444':(m.risk>=70?'#f59e0b':'#10b981');

  document.getElementById('ai-anom-score').innerText = (m.anomaly||0) + ' / 100';
  document.getElementById('ai-anom-score').style.color = (m.anomaly||0)>=70?'#ef4444':((m.anomaly||0)>=50?'#f59e0b':'#10b981');

  // Update Anomaly Detection chart badge
  const anomScore = m.anomaly || 0;
  if(anomScore >= 70) {
    chip('chip-anom-status', 'danger', '🚨 Anomalous');
  } else if(anomScore >= 50) {
    chip('chip-anom-status', 'warn', '⚠️ Unusual');
  } else {
    chip('chip-anom-status', 'ok', '✅ Normal');
  }


  // AI Decision (Source of Truth from Backend)
  document.getElementById('ai-decision').innerText = m.decision || '✅ System Normal';


  xs.push(new Date(m.ts*1000).toLocaleTimeString());
  v.push(m.v_pack); vs.push(m.v_solar); i.push(m.i_pack); t.push(m.t_amb);
  c1.push(m.c1||0); c2.push(m.c2||0); c3.push(m.c3||0);
  r.push(m.risk); p.push(m.power); so.push(m.soc); sh.push(m.soh||0); sp.push(m.sop||0);
  a.push(m.anomaly||0); f.push(m.fire_risk||0);
  [xs,v,vs,i,t,r,p,so,sh,sp,a,f,c1,c2,c3].forEach(arr=>trim(arr,240));

  cV.update(); cC.update(); cI.update(); cT.update(); cR.update(); cP.update(); cA.update();  // Update anomaly chart // NEW cC update
  spV.update(); spI.update(); spT.update(); spS.update(); spP.update(); spSO.update(); spSH.update(); spSP.update(); spA.update(); spF.update();
});

function send(action){ sock.emit('cmd', {action}); }

const mkLine = (id, labels, datasets, ymin=null, ymax=null, ytitle='')=>new Chart(document.getElementById(id).getContext('2d'),{
  type:'line', data:{labels:labels,datasets:datasets},
  options:{responsive:true,animation:false,interaction:{mode:'nearest',intersect:false},
    scales:{y:{min:ymin,max:ymax,title:{text:ytitle,display:true}}, x:{display:true}}}
});
const mkSpark = (id, series)=>new Chart(document.getElementById(id).getContext('2d'),{
  type:'line', data:{labels:xs,datasets:[{data:series, borderWidth:1}]},
  options:{responsive:true,animation:false,plugins:{legend:{display:false}},scales:{x:{display:false},y:{display:false}}}
});

const cV = mkLine('cV', xs, [
  {label:'Pack V',  data:v,  borderWidth:1.6, tension:.2},
  {label:'Solar V', data:vs, borderWidth:1.6, tension:.2}
], null, null, 'Volts');

const cC = mkLine('cC', xs, [
  {label:'Cell 1 V', data:c1, borderWidth:1.6, tension:.2, borderColor:'#3b82f6'},
  {label:'Cell 2 V', data:c2, borderWidth:1.6, tension:.2, borderColor:'#8b5cf6'},
  {label:'Cell 3 V', data:c3, borderWidth:1.6, tension:.2, borderColor:'#ec4899'}
], null, null, 'Volts');

const cI = mkLine('cI', xs, [{label:'Current A', data:i, borderWidth:1.6, tension:.2}], null, null, 'Amps');

const cT = mkLine('cT', xs, [{label:'Temperature °C', data:t, borderWidth:1.6, tension:.2}], null, null, '°C');

const cR = mkLine('cR', xs, [{label:'Risk %', data:r, borderWidth:1.8, tension:.2}], 0, 100, 'Risk %');

const cP = mkLine('cP', xs, [{label:'Power W', data:p, borderWidth:1.6, tension:.2}], null, null, 'Watts');

// Anomaly Detection Chart (0-100 scale)
const cA = mkLine('cA', xs, [{label:'Anomaly Score', data:a, borderWidth:1.8, tension:.2, borderColor:'#f472b6', backgroundColor:'rgba(244,114,182,0.1)'}], 0, 100, 'Score');


const spV  = mkSpark('spV', v);
const spI  = mkSpark('spI', i);
const spT  = mkSpark('spT', t);
const spS  = mkSpark('spS', vs);
const spP  = mkSpark('spP', p);
const spSO = mkSpark('spSO', so);
const spSH = mkSpark('spSH', sh);
const spSP = mkSpark('spSP', sp);
const spA  = mkSpark('spA',  a);
const spF  = mkSpark('spF',  f);
</script>
</body>
</html>
"""

DEEP_HTML = """
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Smart EV BMS — Full Graphs</title>
  <style>{{css}}</style>
  <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1"></script>
</head>
<body>
  <div class="header">
    <div class="title">Smart EV BMS — Full Graphs</div>
    <div id="chip-mqtt" class="badge">MQTT: …</div>
    <div id="chip-rate" class="badge">Rate: …</div>
    <a class="link" href="/">◂ Back</a>
  </div>

  <div class="container" style="grid-template-columns:1fr;">
    <div class="card">
      <div class="hd"><span class="bigname">Time Series</span></div>
      <div class="section-grid">
        <canvas id="gV" class="plot"></canvas>
        <canvas id="gC" class="plot"></canvas> <!-- NEW CELL GRAPH -->
        <canvas id="gI" class="plot"></canvas>
        <canvas id="gT" class="plot"></canvas>
        <canvas id="gR" class="plot"></canvas>
        <canvas id="gP" class="plot"></canvas>
      </div>
      <div class="footer">Rolling window ~10 min</div>
    </div>

    <div class="card">
      <div class="hd"><span class="bigname">EIS — Nyquist & Bode</span></div>
      <div class="section-grid">
        <canvas id="nyquist" class="plot"></canvas>
        <canvas id="bode" class="plot"></canvas>
      </div>
      <div class="footer">If no EIS data arrives, these plots stay empty.</div>
    </div>
  </div>

<script>
const sock = io();
const chip=(id,cls,txt)=>{const e=document.getElementById(id); e.className='badge '+(cls||''); e.innerText=txt;}
sock.on('status', s=>{ chip('chip-mqtt', s.mqtt==='connected'?'ok':'', 'MQTT: '+s.mqtt); chip('chip-rate','', 'Rate: '+(s.hz||0)+' Hz'); });

let xs=[], v=[], vs=[], i=[], t=[], r=[], p=[], c1=[], c2=[], c3=[];
function trim(a,n){while(a.length>n) a.shift();}

sock.on('tick', m=>{
  xs.push(new Date(m.ts*1000).toLocaleTimeString());
  v.push(m.v_pack); vs.push(m.v_solar); i.push(m.i_pack); t.push(m.t_amb); r.push(m.risk); p.push(m.power);
  c1.push(m.c1||0); c2.push(m.c2||0); c3.push(m.c3||0);
  [xs,v,vs,i,t,r,p,c1,c2,c3].forEach(a=>trim(a,1200));
  gV.update(); gC.update(); gI.update(); gT.update(); gR.update(); gP.update();
});

const mk = (id, datasets, ytitle, ymin=null, ymax=null)=>new Chart(document.getElementById(id).getContext('2d'),{
  type:'line', data:{labels:xs,datasets:datasets},
  options:{animation:false,interaction:{mode:'nearest',intersect:false},scales:{y:{min:ymin,max:ymax,title:{text:ytitle,display:true}}}}
});
const gV = mk('gV', [{label:'Pack V', data:v, borderWidth:1.6, tension:.2},
                     {label:'Solar V',data:vs,borderWidth:1.6,tension:.2}], 'Volts');
const gC = mk('gC', [{label:'C1 V', data:c1, borderWidth:1.6, tension:.2, borderColor:'#3b82f6'},
                     {label:'C2 V', data:c2, borderWidth:1.6, tension:.2, borderColor:'#8b5cf6'},
                     {label:'C3 V', data:c3, borderWidth:1.6, tension:.2, borderColor:'#ec4899'}], 'Volts');
const gI = mk('gI', [{label:'Current A', data:i, borderWidth:1.6, tension:.2}], 'Amps');
const gT = mk('gT', [{label:'Temperature °C', data:t, borderWidth:1.6, tension:.2}], '°C');
const gR = mk('gR', [{label:'Risk %', data:r, borderWidth:1.8, tension:.2}], 'Risk %', 0, 100);
const gP = mk('gP', [{label:'Power W', data:p, borderWidth:1.6, tension:.2}], 'Watts');

// EIS
const ny = new Chart(document.getElementById('nyquist').getContext('2d'),{
  type:'scatter', data:{datasets:[{label:'Nyquist (-Im vs Re)', data:[], showLine:true, borderWidth:1.5}]},
  options:{animation:false, scales:{x:{title:{text:'Re(Z) Ω',display:true}}, y:{title:{text:'-Im(Z) Ω',display:true}}}}
});
const bo = new Chart(document.getElementById('bode').getContext('2d'),{
  type:'line', data:{labels:[], datasets:[{label:'|Z| Ω', data:[], borderWidth:1.5}]},
  options:{animation:false, scales:{x:{type:'logarithmic', title:{text:'Frequency (Hz)',display:true}}, y:{title:{text:'|Z| Ω',display:true}}}}
});
sock.on('eis', E=>{
  const pts=[]; for(let i=0;i<E.freq.length;i++){ pts.push({x:E.Zre[i], y:-E.Zim[i]}); }
  ny.data.datasets[0].data=pts; ny.update();
  const mags=[], f=[]; for(let i=0;i<E.freq.length;i++){ const re=E.Zre[i], im=E.Zim[i]; mags.push(Math.sqrt(re*re+im*im)); f.push(E.freq[i]); }
  bo.data.labels=f; bo.data.datasets[0].data=mags; bo.update();
});
</script>
</body>
</html>
"""

@app.get("/")
def index():
    return render_template_string(INDEX_HTML, css=DARK_CSS)

@app.get("/deep-dive")
def deep_dive():
    return render_template_string(DEEP_HTML, css=DARK_CSS)

@app.get("/export.csv")
def export_csv():
    out = io.StringIO(); w = csv.writer(out)
    w.writerow(["ts","v_pack","i_pack","t_amb","v_solar","power_W",
                "soc_%","soh_%","sop_%","risk","pred","mode",
                "anomaly","fire_risk","temp_pattern","v_dev_pred","i_spike_prob",
                "rint_ohm","cells_v1","cells_v2","cells_v3","imbalance_mV","balance_score","balance_needed","lcd_page"])
    rows = zip(buf_ts.list(), buf_vpack.list(), buf_ipack.list(), buf_temp.list(),
               buf_vsolar.list(), buf_power.list(), buf_soc.list(), buf_soh.list(), buf_sop.list(),
               buf_risk.list(), buf_pred.list(), buf_mode.list(),
               buf_anom.list(), buf_frisk.list(), buf_tpat.list(), buf_vdev.list(), buf_ispike.list(), buf_rint.list(),
               buf_c1.list(), buf_c2.list(), buf_c3.list())
    for ts, v, i, t, vs, pw, soc, soh, sop, r, pr, md, anom, fr, tp, vdev, isp, rint, c1, c2, c3 in rows:
        tp_str = "FALLING" if tp == -1 else ("RISING" if tp == 1 else "STABLE")
        w.writerow([ts, v, i, t, vs, pw, soc, soh, sop, r, IDX_TO_LABEL.get(pr, "SAFE"), IDX_TO_MODE.get(md, "AC"),
                    anom, fr, tp_str, vdev, isp, rint, c1, c2, c3, "", "", "", ""])
    return Response(out.getvalue(), mimetype="text/csv")

# Socket.IO control endpoint
@socketio.on("connect")
def on_connect():
    emit("status", {"mqtt": CONN["mqtt"], "hz": CONN["rate_hz"]})
    if EIS_SWEEPS:
        emit("eis", EIS_SWEEPS[0])

@socketio.on("cmd")
def on_cmd(data):
    global AUTO_ACTIONS
    try:
        if isinstance(data, dict) and "policy" in data:
            pol = data["policy"]
            if "auto" in pol:
                AUTO_ACTIONS = bool(pol["auto"])
                log_event("POLICY", f"AUTO_ACTIONS={AUTO_ACTIONS}")
            emit("ack", {"ok": True}); return
        if isinstance(data, dict) and "action" in data:
            send_cmd(str(data["action"]))
            emit("ack", {"ok": True}); return
    except Exception as e:
        emit("ack", {"ok": False, "err": str(e)})

# =========================
# Training (CSV -> joblib)
# =========================
def train_from_csv(csv_path, out_path):
    if not SKLEARN_AVAILABLE:
        raise SystemExit("scikit-learn and joblib are required: pip install scikit-learn joblib pandas")

    import pandas as pd
    import numpy as _np  # local alias to ensure present

    df = pd.read_csv(csv_path)

    if "v_pack" not in df.columns or "i_pack" not in df.columns or "t_amb" not in df.columns:
        raise SystemExit("CSV missing one of required columns: v_pack, i_pack, t_amb")

    # --- features (offline; mirrors live builder) ---
    def row_feats(row, prev):
        v  = float(row.get("v_pack", 0.0))
        i  = float(row.get("i_pack", 0.0))
        t  = float(row.get("t_amb",  0.0))
        vs = float(row.get("v_solar",0.0)) if "v_solar" in row else 0.0
        ts = float(row.get("ts",    0.0))
        c1 = float(row.get("cells_v1", 0.0)) if "cells_v1" in row else 0.0
        c2 = float(row.get("cells_v2", 0.0)) if "cells_v2" in row else 0.0
        c3 = float(row.get("cells_v3", 0.0)) if "cells_v3" in row else 0.0
        if prev is not None:
            dt = max(1e-3, ts - prev["ts"])
            dv_dt = (v - prev["v"]) / dt
            di_dt = (i - prev["i"]) / dt
            dt_dt = (t - prev["t"]) / dt
        else:
            dv_dt = di_dt = dt_dt = 0.0
        p = v * i
        v_margin = 14.5 - v
        abs_i = abs(i)
        rint = 0.12 # default for training data rows
        soc = float(row.get("soc_%", 100.0))
        
        feats = [
            v, i, t, vs,           # 0-3
            c1, c2, c3,           # 4-6
            p, abs_i, v_margin,   # 7-9
            dv_dt, di_dt, dt_dt,  # 10-12
            rint,                  # 13
            0.0, 0.0, 0.0,        # 14-16 (std placeholders)
            soc                    # 17 (matches soc_float in live)
        ]
        return feats, {"ts":ts,"v":v,"i":i,"t":t}

    feats = []
    prev = None
    for _, row in df.iterrows():
        f, prev = row_feats(row, prev)
        feats.append(f)

    X = _np.array(feats, dtype=float)

    # --- labels ---
    def map_pred_to_idx(val):
        if isinstance(val, str):
            u = val.strip().upper()
            return {"SAFE":0, "WARN":1, "DANGER":2}.get(u, 0)
        try:
            k = int(val)
            return k if k in (0,1,2) else 0
        except Exception:
            return 0

    y = None
    if "pred" in df.columns:
        y = _np.array([map_pred_to_idx(v) for v in df["pred"]], dtype=int)

    if y is None or len(_np.unique(y)) < 2:
        def label_from_risk(v, i, t):
            r = compute_risk(float(v), float(i), float(t))
            if r < RISK_WARN: return 0
            if r < RISK_CUT:  return 1
            return 2
        y = _np.array([label_from_risk(v,i,t) for v,i,t in zip(df["v_pack"], df["i_pack"], df["t_amb"])], dtype=int)

    uniq = _np.unique(y)
    target_names = ["SAFE","WARN","DANGER"]
    if len(uniq) < 2:
        risk_vals = [compute_risk(v,i,t) for v,i,t in zip(df["v_pack"], df["i_pack"], df["t_amb"])]
        y = _np.array([0 if r < RISK_WARN else 1 for r in risk_vals], dtype=int)
        target_names = ["SAFE","ALERT"]
        if len(_np.unique(y)) < 2:
            med_i = float(_np.median(df["i_pack"]))
            y = _np.array([0 if i < med_i else 1 for i in df["i_pack"]], dtype=int)

    Xtr, Xva, ytr, yva = train_test_split(X, y, test_size=0.2, random_state=42, stratify=None)

    clf = Pipeline([
        ("scaler", StandardScaler()),
        ("gb", GradientBoostingClassifier(random_state=42))
    ]).fit(Xtr, ytr)

    # ============================================
    # ANOMALY DETECTION MODEL TRAINING
    # ============================================
    # Isolation Forest: Sirf SAFE data par train hoga
    # Kyunki: Normal behavior sikhana hai, WARN/DANGER abnormal hai
    
    anom = None
    try:
        # Step 1: Sirf SAFE labels (label=0) ka data nikalo
        # Training data example:
        # ytr = [0, 0, 1, 0, 2, 0, 0]  (0=SAFE, 1=WARN, 2=DANGER)
        # safe_mask = [True, True, False, True, False, True, True]
        safe_mask = (ytr == 0)
        
        # Step 2: Check karo ki koi SAFE data hai ya nahi
        if _np.any(safe_mask):
            # Step 3: Isolation Forest train karo
            anom = IsolationForest(
                random_state=42,      # Reproducibility (same results har bar)
                contamination=0.05    # Expected outliers: 5% data unusual hoga
                                     # Matlab: Training data mein ~5% anomalies expected hain
            ).fit(Xtr[safe_mask])    # Sirf SAFE rows use karo (Xtr ka subset)
            
            # Training process:
            # 1. SAFE data ke clusters banayega
            # 2. Points jo cluster se door hain wo anomalous mark honge
            # 3. Future mein: SAFE jaisa nahi dikha toh anomaly!
            
            # Example:
            # SAFE cluster: v=12-13V, i=1-3A, t=20-35°C
            # New reading: v=11V, i=0.5A, t=45°C → Anomaly! 🚨
    except Exception:
        anom = None  # Agar training fail ho jaye

    # Performance metrics print karo

    try:
        print("Confusion Matrix:\n", confusion_matrix(yva, clf.predict(Xva)))
        print("\nReport:\n", classification_report(yva, clf.predict(Xva), target_names=target_names[:len(_np.unique(y))]))
    except Exception:
        pass

    # ============================================
    # MODEL BUNDLING & SAVING
    # ============================================
    # Dono models ko ek dictionary mein pack karo
    bundle = {
        "clf": clf,       # Classification model (SAFE/WARN/DANGER)
        "meta": {         # Model metadata
            "features": FEAT_NAMES,  # Feature names list
            "source": os.path.basename(csv_path)  # CSV filename
        },
        "anom": anom      # Isolation Forest (anomaly detection)
    }
    
    # File system mein directory banao (agar nahi hai)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    
    # Bundle ko .joblib file mein save karo
    # Location: models/bms_model.joblib
    joblib.dump(bundle, out_path)
    print(f"\nSaved model → {out_path}")

# =========================
# Main
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate", type=int, default=0, help="1 to simulate instead of MQTT")
    parser.add_argument("--auto", type=int, default=1, help="1=auto actions ON, 0=OFF at startup")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5050)
    parser.add_argument("--train", default=None, help="CSV path to train from (dashboard export)")
    parser.add_argument("--out", default="models/bms_model.joblib", help="Where to save trained model")
    parser.add_argument("--model", default=os.environ.get("BMS_MODEL",""), help="Path to .joblib model to load")
    args = parser.parse_args()

    if args.train:
        train_from_csv(args.train, args.out)
        return

    global AUTO_ACTIONS
    AUTO_ACTIONS = bool(args.auto)
    log_event("SYS", f"AUTO_ACTIONS={AUTO_ACTIONS}")

    # Load model if provided
    if args.model:
        try_load_model(args.model)
    elif os.environ.get("BMS_MODEL"):
        try_load_model(os.environ.get("BMS_MODEL"))

    if args.simulate:
        threading.Thread(target=simulate_loop, daemon=True).start()
        threading.Timer(2.0, simulate_eis_once).start()
    else:
        start_mqtt()

    def status_pusher():
        while True:
            socketio.emit("status", {"mqtt": CONN["mqtt"], "hz": round(CONN["rate_hz"], 2)})
            time.sleep(2)
    threading.Thread(target=status_pusher, daemon=True).start()

    # Render.com injects PORT env variable — use it if available
    port = int(os.environ.get("PORT", args.port))
    socketio.run(app, host=args.host, port=port, debug=False, allow_unsafe_werkzeug=True)

if __name__ == "__main__":
    main()
