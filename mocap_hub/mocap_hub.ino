/*
 * ============================================================================
 *  FLEXIBLE MOCAP — HUB / ANCHOR NODE
 *  Scales from 1 sensor (hub-only, no peripherals) up to 9 sensors.
 *
 *  ── QUICK START FOR 2-SENSOR SETUP ────────────────────────────────────────
 *  You have:
 *    • This ESP32 (HUB) + BNO085  →  NODE 0  (TORSO — world anchor)
 *    • 1 × peripheral ESP32       →  NODE 1  (pick any segment you want)
 *
 *  1. Flash THIS sketch to the TORSO ESP32.
 *  2. Set WIFI_SSID / WIFI_PASS / UE5_IP below.
 *  3. Set ACTIVE_NODES to list only the node IDs you have (see below).
 *  4. Open Serial at 115200, copy the printed MAC → paste into peripheral.
 *  5. Flash mocap_peripheral.ino to the second ESP32 with NODE_ID = 1.
 *
 *  ── ADDING MORE SENSORS LATER ──────────────────────────────────────────────
 *  Just add the new node ID to ACTIVE_NODES and flash another peripheral.
 *  The UDP packet grows automatically; update mocap_receiver.py too.
 *
 *  ── NODE ID / BODY SEGMENT TABLE ──────────────────────────────────────────
 *   ID │ Segment       │ Anchor (delta relative to)
 *   ───┼───────────────┼───────────────────────────
 *    0 │ TORSO (hub)   │ World (absolute)
 *    1 │ L Upper Arm   │ Torso
 *    2 │ L Forearm     │ L Upper Arm
 *    3 │ R Upper Arm   │ Torso
 *    4 │ R Forearm     │ R Upper Arm
 *    5 │ L Thigh       │ Torso
 *    6 │ L Calf        │ L Thigh
 *    7 │ R Thigh       │ Torso
 *    8 │ R Calf        │ R Thigh
 *
 *  ── UDP PACKET FORMAT (to UE5) ────────────────────────────────────────────
 *  Header: "MOCAP" (5 bytes)
 *  Byte 5: uint8 — number of active bones N
 *  Then N × 17-byte MocapBone structs (only active nodes):
 *     uint8_t boneId
 *     float   qi, qj, qk, qr   (relative quaternion)
 *  Total = 5 + 1 + N×17 bytes  (e.g. 40 bytes for 2 sensors)
 *
 *  ── DEPENDENCIES ──────────────────────────────────────────────────────────
 *  • SparkFun BNO08x patched library
 *  • ESP32 board package 2.0.18-arduino5
 *  • WiFi, WiFiUdp, esp_now (built into ESP32 core)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

// ── !! EDIT — Wi-Fi credentials !! ──────────────────────────────────────────
static const char *WIFI_SSID = "YOUR_SSID";
static const char *WIFI_PASS = "YOUR_PASSWORD";

// ── !! EDIT — Unreal Engine 5 host !! ───────────────────────────────────────
static const char *UE5_IP    = "192.168.1.100";   // PC running UE5
static const uint16_t UE5_PORT = 12345;

// ── !! EDIT — Active nodes !! ────────────────────────────────────────────────
//  List EVERY node ID you have hardware for.
//  Node 0 (torso/hub) is ALWAYS included automatically — don't add it here.
//
//  Examples:
//   2 sensors:  { 1 }              → Torso + L Upper Arm
//   3 sensors:  { 1, 2 }           → Torso + L Upper Arm + L Forearm
//   Full body:  { 1, 2, 3, 4, 5, 6, 7, 8 }
//
static const uint8_t PERIPHERAL_IDS[] = { 1 };   // ← edit this line
// ────────────────────────────────────────────────────────────────────────────

// ── Hardware (hub IMU) ───────────────────────────────────────────────────────
#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_FREQ    400000UL
#define IMU_ADDR    0x4B      // match ADR wiring: GND=0x4A, 3V3=0x4B
#define IMU_RST     -1        // set to GPIO pin if RST wired, else -1
#define REPORT_MS   10        // 100 Hz

// ── Derived constants (do not edit) ─────────────────────────────────────────
static const uint8_t NUM_PERIPHERALS =
    sizeof(PERIPHERAL_IDS) / sizeof(PERIPHERAL_IDS[0]);

static const uint8_t TOTAL_NODES = 1 + NUM_PERIPHERALS; // hub + peripherals

// ── Anchor table ─────────────────────────────────────────────────────────────
//  For each node ID 0-8, which node is its anchor?
//  -1 = world (torso only — sends absolute quat)
static const int8_t ANCHOR_OF[9] = {
    -1,  // 0 TORSO    → world
     0,  // 1 L U-Arm  → Torso
     1,  // 2 L Forearm→ L U-Arm
     0,  // 3 R U-Arm  → Torso
     3,  // 4 R Forearm→ R U-Arm
     0,  // 5 L Thigh  → Torso
     5,  // 6 L Calf   → L Thigh
     0,  // 7 R Thigh  → Torso
     7,  // 8 R Calf   → R Thigh
};

// ── Quaternion ───────────────────────────────────────────────────────────────
struct Quat {
    float i, j, k, r;
    static Quat identity() { return {0, 0, 0, 1}; }
    Quat conjugate() const { return {-i, -j, -k, r}; }
    Quat operator*(const Quat &b) const {
        return {
             r*b.i + i*b.r + j*b.k - k*b.j,
             r*b.j - i*b.k + j*b.r + k*b.i,
             r*b.k + i*b.j - j*b.i + k*b.r,
             r*b.r - i*b.i - j*b.j - k*b.k
        };
    }
    Quat normalised() const {
        float n = sqrtf(r*r + i*i + j*j + k*k);
        if (n < 1e-6f) return identity();
        return {i/n, j/n, k/n, r/n};
    }
};

static Quat relativeQuat(const Quat &anchor, const Quat &child) {
    return (anchor.conjugate() * child).normalised();
}

// ── Packets ──────────────────────────────────────────────────────────────────
#pragma pack(push, 1)
struct ImuPacket {          // received from peripherals via ESP-NOW
    uint8_t nodeId;
    float   qi, qj, qk, qr;
};
struct MocapBone {          // sent to UE5 via UDP
    uint8_t boneId;
    float   qi, qj, qk, qr;
};
#pragma pack(pop)

// ── State ─────────────────────────────────────────────────────────────────────
BNO08x imu;
Quat   absQuat[9];          // world-frame quaternion for each node ID 0-8
bool   nodeReady[9] = {};   // true once first data received

WiFiUdp udp;
bool    wifiConnected = false;

// ── ESP-NOW receive callback ──────────────────────────────────────────────────
static void onReceive(const uint8_t *, const uint8_t *data, int len) {
    if (len < (int)sizeof(ImuPacket)) return;
    const ImuPacket *p = reinterpret_cast<const ImuPacket *>(data);
    if (p->nodeId >= 9) return;
    absQuat[p->nodeId] = {p->qi, p->qj, p->qk, p->qr};
    nodeReady[p->nodeId] = true;
}

// ── Build and send UDP packet ─────────────────────────────────────────────────
//  Only active nodes are included. Packet layout:
//    "MOCAP"  (5 bytes)
//    N        (1 byte  — number of bones)
//    N × MocapBone (17 bytes each)
static void sendToUE5() {
    static const char HEADER[] = "MOCAP";

    // Build the active-node bone list (hub first, then peripherals)
    MocapBone bones[9];
    uint8_t n = 0;

    // Helper lambda: compute relative quat for a node
    auto boneQuat = [&](uint8_t id) -> Quat {
        int8_t anchorId = ANCHOR_OF[id];
        if (anchorId < 0) {
            // Torso: send absolute (world-frame)
            return absQuat[id].normalised();
        }
        return relativeQuat(absQuat[anchorId], absQuat[id]);
    };

    // Torso (hub) always first
    Quat q = boneQuat(0);
    bones[n++] = { 0, q.i, q.j, q.k, q.r };

    // Active peripherals
    for (uint8_t pi = 0; pi < NUM_PERIPHERALS; pi++) {
        uint8_t id = PERIPHERAL_IDS[pi];
        q = boneQuat(id);
        bones[n++] = { id, q.i, q.j, q.k, q.r };
    }

    udp.beginPacket(UE5_IP, UE5_PORT);
    udp.write(reinterpret_cast<const uint8_t *>(HEADER), 5);
    udp.write(&n, 1);                                            // bone count
    udp.write(reinterpret_cast<const uint8_t *>(bones), n * sizeof(MocapBone));
    udp.endPacket();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== FLEXIBLE MOCAP HUB ===\n");

    // Print active configuration
    Serial.printf("Active sensors: %d  (hub=TORSO", TOTAL_NODES);
    for (uint8_t i = 0; i < NUM_PERIPHERALS; i++) {
        Serial.printf(" + node%d", PERIPHERAL_IDS[i]);
    }
    Serial.println(")\n");

    // Init absQuat to identity
    for (int i = 0; i < 9; i++) absQuat[i] = Quat::identity();

    // ── I²C + IMU ────────────────────────────────────────────────────────────
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    Wire.setTimeOut(200);
    if (IMU_RST >= 0) { pinMode(IMU_RST, OUTPUT); digitalWrite(IMU_RST, HIGH); }

    Serial.println("Waiting 2 s for BNO085 boot…");
    delay(2000);

    if (!imu.begin(IMU_ADDR, Wire, -1, IMU_RST)) {
        Serial.println("ERROR: Hub BNO085 not found! Halting.");
        while (true) delay(100);
    }
    Serial.println("Hub BNO085 online.");
    imu.enableGameRotationVector(REPORT_MS);

    // ── Wi-Fi ────────────────────────────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    Serial.print("Hub MAC: "); Serial.println(WiFi.macAddress());
    Serial.println("↑ Copy this into TORSO_MAC[] in every peripheral sketch!\n");

    Serial.printf("Connecting to \"%s\"…", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
        delay(500); Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.printf("\nConnected! Hub IP: %s\n", WiFi.localIP().toString().c_str());
        udp.begin(4444);
    } else {
        Serial.println("\nWi-Fi FAILED — UDP disabled.");
    }

    // ── ESP-NOW ──────────────────────────────────────────────────────────────
    if (NUM_PERIPHERALS > 0) {
        if (esp_now_init() != ESP_OK) {
            Serial.println("ERROR: ESP-NOW init failed! Halting.");
            while (true) delay(100);
        }
        esp_now_register_recv_cb(onReceive);
        Serial.println("ESP-NOW listening for peripherals.\n");
    } else {
        Serial.println("No peripherals configured — hub-only mode.\n");
    }

    // Hub node is always ready (it reads its own IMU)
    // Peripherals start as not-ready; loop waits for each one.
    if (NUM_PERIPHERALS == 0) {
        nodeReady[0] = true;   // lone hub: start immediately
    }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // 1. Poll hub IMU
    if (imu.getSensorEvent()) {
        if (imu.wasReset()) {
            Serial.println("[Hub IMU reset — re-enabling]");
            imu.enableGameRotationVector(REPORT_MS);
        } else if (imu.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
            absQuat[0] = {
                imu.getGameQuatI(), imu.getGameQuatJ(),
                imu.getGameQuatK(), imu.getGameQuatReal()
            };
            nodeReady[0] = true;
        }
    }

    if (!wifiConnected) { delay(1); return; }

    // 2. Wait until all active nodes have reported at least once
    bool allReady = nodeReady[0];
    for (uint8_t i = 0; i < NUM_PERIPHERALS && allReady; i++) {
        allReady = nodeReady[PERIPHERAL_IDS[i]];
    }

    // Print which nodes are still missing (once per second, while waiting)
    if (!allReady) {
        static uint32_t lastWarn = 0;
        if (millis() - lastWarn > 1000) {
            lastWarn = millis();
            Serial.print("Waiting for nodes:");
            if (!nodeReady[0]) Serial.print(" 0(hub)");
            for (uint8_t i = 0; i < NUM_PERIPHERALS; i++) {
                if (!nodeReady[PERIPHERAL_IDS[i]])
                    Serial.printf(" %d", PERIPHERAL_IDS[i]);
            }
            Serial.println();
        }
        delay(1);
        return;
    }

    // 3. Send
    sendToUE5();

    // 4. Debug print at ~1 Hz
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        Quat &t = absQuat[0];
        Serial.printf("Hub(TORSO) abs  i:%.3f j:%.3f k:%.3f r:%.3f\n",
                      t.i, t.j, t.k, t.r);
        for (uint8_t i = 0; i < NUM_PERIPHERALS; i++) {
            uint8_t id = PERIPHERAL_IDS[i];
            int8_t  ancId = ANCHOR_OF[id];
            Quat rel = (ancId < 0) ? absQuat[id].normalised()
                                   : relativeQuat(absQuat[ancId], absQuat[id]);
            Serial.printf("Node %d rel  i:%.3f j:%.3f k:%.3f r:%.3f\n",
                          id, rel.i, rel.j, rel.k, rel.r);
        }
    }

    delay(1);
}
