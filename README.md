# Full-Body-MoCap
## ⚠️ Project Status

**This is Version 1 — a communication testbed, not a finished mocap suit.**

The goal of v1 is to validate the full data pipeline with the minimum viable hardware: **2 nodes** (1 hub + 1 peripheral), breadboarded on DevKit boards. Once the wireless stack and packet format are proven, the project will expand to a proper PCB design and a full 9-node body suit.

| Version | Status | Hardware | Nodes |
|---------|--------|----------|-------|
| **v1 (this)** | ✅ In progress | ESP32 DevKit + BNO085 breakout | 2 |
| v2 | 🔲 Planned | Custom PCB + integrated IMU + mounting | 9 |

---

## What This Does

Two ESP32 + BNO085 combos communicate wirelessly:

- **Hub (Node 0 — Torso):** Reads its own BNO085 at 100 Hz, receives quaternion data from the peripheral over **ESP-NOW**, computes relative joint orientations, and forwards a compact binary UDP packet to a PC at ~100 Hz.
- **Peripheral (Node 1):** Reads its own BNO085 at 100 Hz and streams raw quaternions to the hub via **ESP-NOW** — no router involved between nodes.
- **`mocap_receiver.py`:** Runs on the PC to validate the incoming UDP stream, decode quaternions, and display Euler angles in the terminal. Intended for Unreal Engine 5 integration.

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                   Body (v1 — 2 nodes)               │
│                                                     │
│   [Node 1 — any segment]                            │
│    BNO085 + ESP32 DevKit                            │
│         │                                           │
│         │  ESP-NOW (2.4 GHz, peer-to-peer)          │
│         │                                           │
│   [Node 0 — TORSO / HUB]                            │
│    BNO085 + ESP32 DevKit                            │
│    Wi-Fi STA                                        │
└──────────────────────┬──────────────────────────────┘
                       │  UDP  (Wi-Fi LAN)
           ┌───────────▼────────────┐
           │  PC                    │
           │  mocap_receiver.py     │
           │  (→ Unreal Engine 5)   │
           └────────────────────────┘
```

The hub runs **Wi-Fi STA** (for UDP to PC) and **ESP-NOW** simultaneously on the same radio — no second chip needed.

---

## Hardware (v1)

### Per Node

| Component | Part |
|-----------|------|
| Microcontroller | ESP32 DevKit (38-pin, any variant) |
| IMU | BNO085 breakout board (SparkFun SEN-22857 or equivalent) |
| Power | USB (bench testing) |

### Wiring — identical for hub and peripheral

```
BNO085 VDD  →  3.3 V
BNO085 GND  →  GND
BNO085 SDA  →  GPIO 21   (+ 4.7 kΩ pull-up to 3.3 V)
BNO085 SCL  →  GPIO 22   (+ 4.7 kΩ pull-up to 3.3 V)
BNO085 RST  →  GPIO 4    (set IMU_RST -1 in firmware if not wired)
BNO085 PS0  →  GND       (I²C mode)
BNO085 PS1  →  GND       (I²C mode)
BNO085 ADR  →  GND  →  address 0x4A  (peripheral default)
            →  3V3  →  address 0x4B  (hub default)
```

---

## Node ID / Body Segment Map

The firmware supports up to 9 nodes. v1 uses only 0 and 1.

| ID | Segment | Anchor |
|----|---------|--------|
| **0** | **Torso (hub)** | World (absolute) |
| 1 | L Upper Arm | Torso |
| 2 | L Forearm | L Upper Arm |
| 3 | R Upper Arm | Torso |
| 4 | R Forearm | R Upper Arm |
| 5 | L Thigh | Torso |
| 6 | L Calf | L Thigh |
| 7 | R Thigh | Torso |
| 8 | R Calf | R Thigh |

---

## UDP Packet Format

Sent by the hub to the PC on port `12345`. All values little-endian.

```
Offset   Size   Type     Field
──────   ────   ───────  ──────────────────────────────
0        5      ASCII    Magic: "MOCAP"
5        1      uint8    N — number of active bones
6 + i×17 1      uint8    boneId
       + 4      float    qi
       + 4      float    qj
       + 4      float    qk
       + 4      float    qr  (real / w)
```

Total size: `6 + N × 17` bytes — **40 bytes** for v1's 2-sensor setup.

Quaternions for non-root bones are **relative to their anchor** (parent segment). Node 0 sends its absolute world-frame orientation.

---

## Repository Structure

```
Full-Body-MoCap/
├── mocap_hub.ino          # Hub firmware (Node 0 — Torso)
├── mocap_peripheral.ino   # Peripheral firmware (Nodes 1–8)
├── mocap_receiver.py      # Python UDP receiver / debug tool
├── docs/
│   └── DEVLOG.md          # Development log
└── README.md
```

---

## Dependencies

### Arduino / ESP32

| Library | Notes |
|---------|-------|
| SparkFun BNO08x (patched) | Install via Arduino Library Manager |
| ESP32 Arduino core `2.0.18-arduino5` | Via Boards Manager |
| `WiFi`, `WiFiUdp`, `esp_now` | Built into ESP32 core |

### Python

Standard library only — no `pip install` needed.
```
Python 3.8+
socket, struct, math, time, argparse
```

---

## Quick Start

### 1. Flash the Hub

1. Open `mocap_hub.ino` in Arduino IDE.
2. Edit the config section:
   ```cpp
   static const char *WIFI_SSID = "your_network";
   static const char *WIFI_PASS = "your_password";
   static const char *UE5_IP   = "192.168.x.x";    // your PC's IP
   static const uint8_t PERIPHERAL_IDS[] = { 1 };  // one peripheral for v1
   ```
3. Flash to the hub ESP32.
4. Open Serial Monitor at **115200 baud** and copy the printed **Hub MAC address**.

### 2. Flash the Peripheral

1. Open `mocap_peripheral.ino`.
2. Edit:
   ```cpp
   #define NODE_ID  1
   static const uint8_t TORSO_MAC[6] = { 0xXX, 0xXX, ... };  // paste hub MAC
   ```
3. Flash to the second ESP32.

### 3. Verify on PC

```bash
# Verbose — print every packet with Euler angles
python mocap_receiver.py --port 12345

# Rate-only — check Hz without flooding terminal
python mocap_receiver.py --port 12345 --hz
```

Expected verbose output:
```
── Packet #1  (2 bones)  from 192.168.1.x ───
  TORSO       q=(+0.001,+0.002,-0.003,+1.000)  R+0.2° P+0.1° Y-0.3°
  L_UPPER_ARM q=(+0.012,-0.043,+0.021,+0.998)  R+1.4° P-5.0° Y+2.4°
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `ERROR: Hub BNO085 not found` | Wrong I²C address or wiring | Check `IMU_ADDR` (0x4A vs 0x4B), verify SDA/SCL pins |
| `Waiting for nodes: 1` (never clears) | Peripheral not reaching hub | Verify `TORSO_MAC` matches hub Serial output exactly |
| `[WARN] Bad packet` in Python | Packet size mismatch | Ensure `PERIPHERAL_IDS` count in hub matches active nodes |
| No UDP on PC | Firewall blocking | Allow port 12345 UDP inbound on the PC |
| Quaternion yaw drifts over time | No magnetometer reference | Expected — BNO085 Game Rotation Vector is gravity-only |

---

## Roadmap

### v1 — Communication PoC *(current)*
- [x] ESP-NOW link: peripheral → hub
- [x] UDP stream: hub → PC
- [x] Binary packet format (extensible to 9 nodes)
- [x] Python debug receiver with Euler angle display
- [ ] Validate latency and packet loss at ~100 Hz
- [ ] Test with UE5 UDP consumer

### v2 — Full Body Suit *(planned)*
- [ ] Custom PCB: ESP32 + BNO085 integrated, compact form factor
- [ ] Body mounting system (straps / enclosures) for all 9 segments
- [ ] 9 nodes: torso, both arms + forearms, both thighs + calves
- [ ] LiPo battery + charging on each PCB
- [ ] UE5 LiveLink plugin or Blueprint UDP consumer
- [ ] Web-based status dashboard (hub as soft-AP)
- [ ] T-pose calibration routine

---

## License

MIT — see [LICENSE](LICENSE).
