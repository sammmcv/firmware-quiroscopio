// frame_building.c - Construcción de supertramas binarias
// Módulo para empaquetado eficiente de datos de sensores

#include "../include/shared_types.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"

// ===== Prototipo de función externa (de ble_connection.c) =====
extern void ble_notify_att_payload(const uint8_t *data, size_t len);
extern bool ble_has_space_for(uint16_t bytes);
extern void ble_send_bytes(const uint8_t *data, uint16_t len);
extern volatile uint32_t ble_drops;

// ===== Raw Packing Helpers (sin FP) =====
// Escribir int16 como little-endian
static inline void put_le16(uint8_t* p, int16_t v){
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

// Empaquetar sensor_sample_raw_t: 14 bytes (3x acc + 4x quat)
size_t pack_sensor_raw(uint8_t* out, const sensor_sample_raw_t* s){
    uint8_t* p = out;
    put_le16(p+0,  s->ax_mg);
    put_le16(p+2,  s->ay_mg);
    put_le16(p+4,  s->az_mg);
    put_le16(p+6,  s->qw_q14);
    put_le16(p+8,  s->qi_q14);
    put_le16(p+10, s->qj_q14);
    put_le16(p+12, s->qk_q14);
    return 14; // bytes por sensor
}

// ===== Supratrama State Variables =====
static uint16_t g_sf_counter10 = 0;   // wraps at 1024 (10-bit counter)
uint8_t  g_sf_presence  = 0;   // bits 0..4 per sensor
static int16_t  g_sf_accquat[5][7];   // staging for sensors present: [ax,ay,az,qx,qy,qz,qw]
static uint8_t  g_sf_buf[244];        // ATT payload max (MTU 247 -> 247-3=244)

// ===== Supratrama Protocol Functions =====
// Control byte packing:
//   Byte0: [7..3]=presence(5b), [2..1]=cnt[9..8], [0]=RSV(0)
//   Byte1: cnt[7..0]
static inline void pack_ctrl(uint8_t presence5, uint16_t cnt10, uint8_t *b0, uint8_t *b1) {
    presence5 &= 0x1F;            // 5 bits
    cnt10     &= 0x03FF;          // 10 bits
    *b0 = (uint8_t)((presence5 << 3) | ((cnt10 >> 8) & 0x03)); // RSV=0
    *b1 = (uint8_t)(cnt10 & 0xFF);
}

static inline void unpack_ctrl(uint8_t b0, uint8_t b1, uint8_t *presence5, uint16_t *cnt10) {
    if (presence5) *presence5 = (uint8_t)((b0 >> 3) & 0x1F);
    if (cnt10)     *cnt10     = (uint16_t)((((uint16_t)(b0 & 0x06)) << 7) | b1);
}

// Pack supratrama: control (2B) + payload (14B per sensor present)
// acc_quat[i][0..6] = {ax,ay,az,qx,qy,qz,qw}, i=0..4 (skip if bit i = 0)
static size_t pack_superframe(uint8_t *out, uint8_t presence5, uint16_t cnt10,
                              const int16_t acc_quat[5][7]) {
    uint8_t b0, b1; 
    pack_ctrl(presence5, cnt10, &b0, &b1);
    size_t off = 0; 
    out[off++] = b0; 
    out[off++] = b1;
    
    for (int i = 0; i < 5; ++i) {
        if (!(presence5 & (1u << i))) continue;
        for (int k = 0; k < 7; ++k) {
            int16_t v = acc_quat[i][k];
            out[off++] = (uint8_t)(v & 0xFF);
            out[off++] = (uint8_t)((v >> 8) & 0xFF);
        }
    }
    return off; // bytes written
}

static bool unpack_superframe(const uint8_t *in, size_t len,
                              uint8_t *presence5, uint16_t *cnt10,
                              int16_t acc_quat[5][7]) {
    if (len < 2) return false;
    uint8_t p; 
    uint16_t c; 
    unpack_ctrl(in[0], in[1], &p, &c);
    size_t need = 2 + 14 * __builtin_popcount((unsigned)p);
    if (len < need) return false;
    if (presence5) *presence5 = p;
    if (cnt10)     *cnt10     = c;
    size_t off = 2;
    for (int i = 0; i < 5; ++i) {
        if (!(p & (1u << i))) continue;
        for (int k = 0; k < 7; ++k) {
            int16_t v = (int16_t)(in[off] | (in[off+1] << 8));
            acc_quat[i][k] = v; 
            off += 2;
        }
    }
    return true;
}

static inline uint8_t make_flags(uint8_t src, uint8_t core) {
    return (src & 0x03) | ((core & 0x01) << 2);
}

// CRC16-CCITT: polinomio 0x1021, init 0xFFFF
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc = (crc << 1);
        }
    }
    return crc;
}

// ===== API Pública: Supratrama =====

// Flush supratrama if any sensors present
void superframe_flush_and_send_if_ready(void) {
    if (g_sf_presence == 0) return; // no sensors this cycle
    size_t len = pack_superframe(g_sf_buf, g_sf_presence, g_sf_counter10, g_sf_accquat);
    // Ensure does not exceed ATT payload (MTU-3). With up to 5 sensors, len <= 72.
    ble_notify_att_payload(g_sf_buf, len);
    g_sf_presence = 0;
    g_sf_counter10 = (uint16_t)((g_sf_counter10 + 1) & 0x03FF);
}

// Stage a sensor sample into the supratrama buffer
// sensor_idx: 0=UART, 1=I2C0/0x28, 2=I2C0/0x29, 3=I2C1/0x28, 4=I2C1/0x29
void superframe_stage_sensor(int sensor_idx,
                             int16_t ax, int16_t ay, int16_t az,
                             int16_t qx, int16_t qy, int16_t qz, int16_t qw) {
    if (sensor_idx < 0 || sensor_idx > 4) return;
    g_sf_accquat[sensor_idx][0] = ax;
    g_sf_accquat[sensor_idx][1] = ay;
    g_sf_accquat[sensor_idx][2] = az;
    g_sf_accquat[sensor_idx][3] = qx;
    g_sf_accquat[sensor_idx][4] = qy;
    g_sf_accquat[sensor_idx][5] = qz;
    g_sf_accquat[sensor_idx][6] = qw;
    g_sf_presence |= (uint8_t)(1u << sensor_idx);
}
