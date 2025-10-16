#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/sync.h"  // Para spin_lock_t

// Constantes
#define NUM_SENSORS 2

// Tamaño del buffer TX BLE (fuente única de verdad)
#ifndef BLE_TX_BUF_SIZE
#define BLE_TX_BUF_SIZE 4096
#endif

// Tipo legacy para I2C (todavía usa floats - TODO: migrar a raw)
typedef struct {
    bool valid;
    float x, y, z;          // TODO: eliminar cuando I2C migre a raw
    float qx, qy, qz, qw;   // TODO: eliminar cuando I2C migre a raw
} sensor_sample_t;

// Datos crudos: sin floats (usado por UART)
// Nota de escala:
// - Aceleración: ax_mg/ay_mg/az_mg en milig (mg) o en la LSB real usada por el sensor
// - Cuaternión: Q14 (1 LSB = 1/16384)
typedef struct {
    int16_t ax_mg, ay_mg, az_mg;
    int16_t qw_q14, qi_q14, qj_q14, qk_q14;
} sensor_sample_raw_t;

// Calibración en fijo (opcional)
// bias en mg y ganancia en Q15 (1.0 = 32768)
typedef struct {
    int16_t bx_mg, by_mg, bz_mg;
    int16_t gx_q15, gy_q15, gz_q15;
} acc_calib_q15_t;

typedef struct {
    uint8_t zero_run;
    uint8_t fail;
    uint8_t moved;
    uint32_t last_reinit;
} heal_state_t;

// Prototipos de funciones públicas (sin static)
// De ble_connection.c
void ble_notify_att_payload(const uint8_t *data, size_t len);
bool ble_has_space_for(uint16_t bytes);
void ble_send_bytes(const uint8_t *data, uint16_t len);
void ble_try_flush(void);
void ble_stack_register_handlers(void);
void ble_init_lock(spin_lock_t *lock);
uint16_t ble_tx_free_space(void);
uint16_t ble_current_att_mtu(void);  // NUEVO: Obtener MTU actual

// De calibration.c
void backup_all_offsets(void);
void restore_all_offsets(void);
void acc_apply_calib_q15(sensor_sample_raw_t* s, const acc_calib_q15_t* c);

// De get_sensors.c
void init_bno_uart(void);
void init_bno_i2c(i2c_inst_t *port, uint8_t addr);
void init_i2c_port(i2c_inst_t *port, uint sda_pin, uint scl_pin, uint baud);
bool bno_read_linear_acc(int16_t *x, int16_t *y, int16_t *z);
void bno_uart_recover(void);
bool bno_read_acc_raw(sensor_sample_raw_t* s);
bool bno_read_quat_raw(sensor_sample_raw_t* s);
// I2C lectores (raw)
bool bno_i2c_read_acc_raw(i2c_inst_t *i2c, uint8_t dev_addr, sensor_sample_raw_t *s);
bool bno_i2c_read_quat_raw(i2c_inst_t *i2c, uint8_t dev_addr, sensor_sample_raw_t *s);
bool read_bno_i2c_sample_raw(int idx, sensor_sample_raw_t *out);

// De frame_building.c
void superframe_stage_sensor(int sensor_idx, int16_t ax, int16_t ay, int16_t az,
                             int16_t qx, int16_t qy, int16_t qz, int16_t qw);
void superframe_flush_and_send_if_ready(void);
size_t pack_sensor_raw(uint8_t* out, const sensor_sample_raw_t* s);

// Variables globales compartidas (extern)
extern sensor_sample_t g_uart;
extern sensor_sample_t g_i2c0[];
extern sensor_sample_t g_i2c1[];
extern volatile bool g_show_calib;
extern volatile bool g_freeze_i2c0, g_freeze_i2c1;
extern uint8_t g_sf_presence;
extern const uint8_t SENSOR_ADDR[];

// Variable BLE compartida
extern uint16_t ble_con_handle;
extern volatile bool ble_advertising;
extern volatile uint32_t last_adv_restart;
extern volatile uint32_t ble_skipped_samples;

// Variables para sincronización de cores
extern spin_lock_t *g_i2c1_lock;
extern heal_state_t i2c0_heal[];
extern uint32_t last_reinit_uart;
extern uint8_t fail_uart, zero_run_uart, moved_uart;
