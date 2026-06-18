/*
 * ============================================================================
 *  FLEXIBLE MOCAP — PERIPHERAL NODE
 *  Runs on every ESP32 that is NOT the hub (torso).
 *
 *  ── CONFIGURATION — edit the two lines marked !! EDIT !! ──────────────────
 *   NODE_ID     which body segment this board represents (1–8)
 *   TORSO_MAC   MAC address of the hub ESP32
 *               (shown in Serial when hub first boots — copy & paste)
 *
 *  ── NODE ID / BODY SEGMENT TABLE ──────────────────────────────────────────
 *   ID │ Segment        (pick whichever two make sense for your use case)
 *   ───┼───────────────
 *    1 │ L Upper Arm
 *    2 │ L Forearm
 *    3 │ R Upper Arm
 *    4 │ R Forearm
 *    5 │ L Thigh
 *    6 │ L Calf
 *    7 │ R Thigh
 *    8 │ R Calf
 *
 *  ── WIRING (identical for all nodes) ─────────────────────────────────────
 *   BNO085 VDD  → 3.3 V
 *   BNO085 GND  → GND
 *   BNO085 SDA  → GPIO21   (+4.7 kΩ pull-up)
 *   BNO085 SCL  → GPIO22   (+4.7 kΩ pull-up)
 *   BNO085 RST  → GPIO4    (or set IMU_RST -1 to skip)
 *   BNO085 ADR  → GND  →  I²C address 0x4A
 *               → 3V3  →  I²C address 0x4B   (match IMU_ADDR below)
 *   BNO085 PS0  → GND  (I²C mode)
 *   BNO085 PS1  → GND  (I²C mode)
 *
 *  ── DEPENDENCIES ──────────────────────────────────────────────────────────
 *  • SparkFun BNO08x patched library
 *  • ESP32 board package 2.0.18-arduino5
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

// ── !! EDIT THESE TWO LINES !! ───────────────────────────────────────────────
#define NODE_ID  1    // 1–8 — which body segment is this board?

// MAC of the hub ESP32 (printed to Serial on hub boot).
static const uint8_t TORSO_MAC[6] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
// ─────────────────────────────────────────────────────────────────────────────

// ── Hardware ─────────────────────────────────────────────────────────────────
#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_FREQ    400000UL
#define IMU_ADDR    0x4A      // ADR→GND = 0x4A, ADR→3V3 = 0x4B
#define IMU_RST     4         // set to -1 if RST not wired
#define REPORT_MS   10        // 100 Hz

// ── ESP-NOW packet ────────────────────────────────────────────────────────────
#pragma pack(push, 1)
struct ImuPacket {
    uint8_t nodeId;
    float   qi, qj, qk, qr;   // raw game-rotation-vector quaternion
};
#pragma pack(pop)

// ── Globals ───────────────────────────────────────────────────────────────────
BNO08x    imu;
ImuPacket pkt;
bool      peerAdded = false;

static void onSent(const uint8_t *, esp_now_send_status_t) {}  // optional hook

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.printf("\n=== MOCAP PERIPHERAL — NODE %d ===\n", NODE_ID);

    // ── I²C ──────────────────────────────────────────────────────────────────
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    Wire.setTimeOut(200);

    // ── IMU ──────────────────────────────────────────────────────────────────
    if (IMU_RST >= 0) { pinMode(IMU_RST, OUTPUT); digitalWrite(IMU_RST, HIGH); }
    Serial.println("Waiting 2 s for BNO085 boot…");
    delay(2000);

    if (!imu.begin(IMU_ADDR, Wire, -1, IMU_RST)) {
        Serial.println("ERROR: BNO085 not found! Halting.");
        while (true) delay(100);
    }
    Serial.println("BNO085 online.");
    imu.enableGameRotationVector(REPORT_MS);

    // ── Wi-Fi / ESP-NOW ───────────────────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.print("Node MAC: "); Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: ESP-NOW init failed! Halting.");
        while (true) delay(100);
    }
    esp_now_register_send_cb(onSent);

    // Register hub as peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, TORSO_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("ERROR: failed to register hub peer! Halting.");
        while (true) delay(100);
    }

    peerAdded = true;
    pkt.nodeId = NODE_ID;
    Serial.printf("Streaming as node %d → hub. Ready.\n\n", NODE_ID);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (!imu.getSensorEvent()) {
        delayMicroseconds(500);
        return;
    }
    if (imu.wasReset()) {
        Serial.println("[IMU reset — re-enabling]");
        imu.enableGameRotationVector(REPORT_MS);
        return;
    }
    if (imu.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
        pkt.qi = imu.getGameQuatI();
        pkt.qj = imu.getGameQuatJ();
        pkt.qk = imu.getGameQuatK();
        pkt.qr = imu.getGameQuatReal();

        if (peerAdded) {
            esp_now_send(TORSO_MAC,
                         reinterpret_cast<const uint8_t *>(&pkt),
                         sizeof(pkt));
        }
    }
}
