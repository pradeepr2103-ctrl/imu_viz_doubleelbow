#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

const char* ssid     = "TP-Link_DF6C_Cave";
const char* password = "Caveiot@123";

// One multicast address — all 3 laptops receive automatically
const char* multicast_ip = "239.0.0.1";
const unsigned int port  = 5005;

// ← CHANGE THIS per sensor: L_FA, R_FA, L_UA, R_UA, L_TH, L_SH, R_TH, R_SH, HIPS, CHEST
const char* SENSOR_LABEL = "R_TH";

WiFiUDP udp;
MPU6050 mpu;
uint8_t fifoBuffer[64];

void setup() {
    Serial.begin(115200);
    Wire.begin(8, 9);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi Connected");

    mpu.initialize();
    if (mpu.dmpInitialize() == 0) {
        mpu.setDMPEnabled(true);
        Serial.println("DMP Ready");
    } else { while(1); }
}

void loop() {
    static unsigned long lastSend = 0;

    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;
    if (millis() - lastSend < 15) return;  // cap at ~67Hz
    lastSend = millis();

    Quaternion q;
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    Serial.print(SENSOR_LABEL);
    Serial.print(" : ");
    Serial.print(q.w, 4);
    Serial.print(" ");
    Serial.print(q.x, 4);
    Serial.print(" ");
    Serial.print(q.y, 4);
    Serial.print(" ");
    Serial.println(q.z, 4);

    char payload[64];
    snprintf(payload, sizeof(payload), "%s,%.4f,%.4f,%.4f,%.4f",
             SENSOR_LABEL, q.w, q.x, q.y, q.z);

    // Single send — router replicates to all 3 laptops
    udp.beginPacket(multicast_ip, port);
    udp.print(payload);
    udp.endPacket();
}