---

## v1 — Communication Proof of Concept

**Goal:** Validate the full data path — IMU → ESP-NOW → hub → UDP → PC — with the minimum hardware (2 nodes, DevKit boards, no custom PCB) before committing to a 9-node body suit build.

---

### [v1] Why ESP-NOW instead of direct Wi-Fi per node

The obvious first instinct was to give every node its own Wi-Fi connection and have each one send UDP directly to the PC. The problem is that full Wi-Fi association on battery-powered wearable nodes is expensive in both power and latency. Every node would need its own TCP/IP stack running, its own DHCP lease, and the PC would need to manage 9 separate UDP sockets.

ESP-NOW operates at the MAC layer — no router, no DHCP, no association handshake. A peripheral wakes up, sends a 21-byte frame to the hub's MAC address, and goes back to processing the next IMU sample. Measured round-trip at the MAC layer is 1–3 ms.

The hub is the only node that needs a full Wi-Fi connection (for UDP to the PC), and the ESP32 handles Wi-Fi STA and ESP-NOW simultaneously on the same 2.4 GHz radio. This works because ESP-NOW runs at the MAC layer below the Wi-Fi association state — the radio services both without any explicit coordination in user code.

**One channel dependency to be aware of:** ESP-NOW peers must operate on the same Wi-Fi channel as the hub. Since the hub connects to the router first and inherits the router's channel, all peripherals just need `peer.channel = 0` (auto-follow). No manual channel management needed for a single-router environment.

---

### [v1] Why BNO085 instead of a raw 6-axis IMU

A raw gyro + accelerometer (MPU-6050, ICM-42688) outputs angular rate and linear acceleration. To get orientation, you have to run an AHRS filter on the ESP32 — Madgwick or Mahony. This is doable, but it adds:

- A beta/gain parameter that needs tuning per sensor and mounting position.
- Heading drift that accumulates differently depending on filter quality and sensor noise.
- CPU cycles on a chip that's also managing Wi-Fi + ESP-NOW.

The BNO085 runs its own internal Kalman filter and outputs quaternions directly over I²C via the SHTP protocol. The ESP32 just reads the result. The **Game Rotation Vector** report — which is what v1 uses — is gravity-referenced but magnetometer-free, which means it's stable indoors (no compass interference from electronics or steel) but has no absolute yaw reference. Yaw is relative to wherever the sensor was pointing at boot.

For v1 this is fine — the goal is to test the communication stack, not produce production-quality animation. For v2, switching to the **Rotation Vector** report (9-DOF with magnetometer) would give absolute heading, at the cost of needing magnetometer calibration.

**Library note:** The SparkFun BNO08x Arduino library requires `Wire.setTimeOut(200)` before `imu.begin()` on ESP32. Without it, the SHTP handshake occasionally stalls I²C on cold boot, causing the hub to halt at startup. This took a while to track down since it only happens ~1 in 5 boots without the timeout set.

---

### [v1] Relative quaternions — why and how

The BNO085 outputs world-frame (absolute) quaternions for every node. If you send absolute quaternions directly to a skeleton rig, rotating the torso does not automatically rotate the arms — the arm bones would appear to stay fixed in world space while the torso moves underneath them. Skeletal animation engines expect each bone's rotation to be expressed relative to its parent.

For each non-root bone, the hub computes:

```
q_relative = conjugate(q_anchor) * q_child
```

This gives the rotation of the child bone in the coordinate frame of its parent. The torso (Node 0) is the skeleton root and sends its absolute world-frame quaternion unchanged.

The anchor relationships are hardcoded in `ANCHOR_OF[]` in the hub firmware. For v2 with a head node or hand nodes, new entries just need to be added to that array.

---

### [v1] Packet format decisions

**Binary over JSON:** At 100 Hz with 9 nodes, JSON would produce roughly 15–20 KB/s of text. The binary format produces 159 bytes per packet = ~16 KB/s, but is trivially parseable in both Python (`struct.unpack`) and C++ (`#pragma pack`). More importantly, JSON parsing in a UE5 Blueprint or C++ plugin is painful. The binary format maps directly to a C struct with no parsing overhead.

**Magic header "MOCAP":** Five ASCII bytes at the start make packets easy to identify in Wireshark and give the receiver a sanity check before attempting to parse bone data. A bad packet (wrong length, corrupt header) is discarded with a warning rather than silently producing garbage quaternions.

**Dynamic bone count byte:** Byte 5 of every packet is the bone count `N`. This means the receiver never needs to be reconfigured as nodes are added — it reads `N`, parses exactly `N × 17` bytes, and is done. The Python receiver and any future UE5 plugin handle 1-node and 9-node packets identically.

**One null terminator bug found:** `static const char HEADER[] = "MOCAP"` in Arduino includes a null terminator, making `sizeof(HEADER)` equal to 6. The hub uses `udp.write(HEADER, 5)` explicitly to write exactly 5 bytes. An earlier version used `sizeof(HEADER)` by mistake, sending 6 bytes and breaking the parser. The Python receiver in v1 handles both cases for robustness, but the firmware is now explicit.

---

### [v1] Latency budget

| Stage | Estimated |
|-------|-----------|
| BNO085 report interval | 10 ms (100 Hz) |
| ESP-NOW peripheral → hub | 1–3 ms |
| Hub quaternion math + pack | < 1 ms |
| UDP hub → PC (LAN) | 1–5 ms |
| **Total end-to-end** | **~12–19 ms** |

At 60 FPS this is 0.7–1.1 frames of lag — imperceptible for real-time animation preview. For biomechanics or precise timing research a hardware timestamp from the BNO085's internal clock would be needed; v1 does not include one.

---

### [v1] Things that don't work yet / known issues

- **No packet sequence numbers.** The receiver has no way to detect dropped packets or measure loss rate. A `uint16_t seq` field should be added to the packet header in v2.
- **Hub is a single point of failure.** If the hub crashes or its BNO085 locks up, all data stops. A watchdog timer would recover from this automatically.
- **`nodeReady[]` gate can stall forever.** If a peripheral never connects (dead battery, wrong MAC), the hub prints a warning every second but never sends any UDP packets at all — even the hub's own torso data is suppressed. In v2, a timeout should allow the hub to stream whatever nodes are available.
- **Fixed channel assumption.** If the router changes its Wi-Fi channel (rare, but possible), ESP-NOW stops working until all boards are rebooted. A channel scan at startup would handle this.

---

## v2 — Full Body Suit (Planned)

Not started. Design goals:

- **9 nodes:** Torso (hub), L/R Upper Arm, L/R Forearm, L/R Thigh, L/R Calf. Head and hands as optional additions.
- **Custom PCB:** ESP32 module + BNO085 on a single small board with a LiPo connector and charging IC. Target footprint: < 50 × 35 mm.
- **Mounting:** Elastic strap enclosures at each segment. Rigid enough to prevent sensor twist under fast movement, easy to remove for charging.
- **Calibration:** T-pose routine at startup — each node records its offset quaternion so the neutral skeleton pose maps to a standard anatomical reference.
- **UE5 integration:** Either a custom LiveLink source or a C++ UDP consumer that drives a retargeted skeleton directly from incoming bone quaternions.
