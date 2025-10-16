// calibration.c - Gestión de calibración y offsets en Flash
// Módulo para backup/restore de offsets de calibración BNO055

#include "../include/shared_types.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

// ===== Q15 helpers (sin FP) =====
// Ganancia en Q15 (1.0 = 32768), bias en mg
#ifndef Q15_ONE
#define Q15_ONE 32768
#endif

static inline int16_t q15_mul_s16(int16_t x, int16_t g_q15){
    int32_t t = (int32_t)x * (int32_t)g_q15; // 16x16 -> 32
    t += (1 << 14);                          // redondeo
    t >>= 15;
    if (t > 32767) t = 32767;
    if (t < -32768) t = -32768;
    return (int16_t)t;
}

void acc_apply_calib_q15(sensor_sample_raw_t* s, const acc_calib_q15_t* c){
    int16_t x = (int16_t)(s->ax_mg - c->bx_mg);
    int16_t y = (int16_t)(s->ay_mg - c->by_mg);
    int16_t z = (int16_t)(s->az_mg - c->bz_mg);
    s->ax_mg = q15_mul_s16(x, c->gx_q15);
    s->ay_mg = q15_mul_s16(y, c->gy_q15);
    s->az_mg = q15_mul_s16(z, c->gz_q15);
}

// ===== Flash Defines =====
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define FLASH_MAGIC_WORD    0xB0055CA1
#define MAX_BACKUPS 8

// ===== BNO055 Register Defines (necesarios para calibración) =====
#define BNO055_OPR_MODE_ADDR       0x3D
#define BNO055_CALIB_STAT_ADDR     0x35
#define BNO055_SYS_TRIGGER_ADDR    0x3F
#define BNO055_OPR_MODE_NDOF       0x0C

// ===== Hardware Defines (copiados de get_sensors.c para evitar dependencias circulares) =====
#define UART_ID          uart0
#define I2C_PORT         i2c0
#define I2C_PORT_1       i2c1

// (SENSOR_ADDR y NUM_SENSORS vienen de shared_types.h)

// ===== Estructura para guardar offsets =====
typedef struct {
    uint32_t magic;        // 4
    uint8_t  sensor_id;    // 1
    uint8_t  calib_status; // 1
    uint8_t  offsets[22];  // 22
    uint32_t crc;          // 4  => total 32 bytes
} __attribute__((packed)) offset_backup_t;

// ===== Prototipos de funciones externas (de get_sensors.c) =====
// Estas funciones son implementadas en get_sensors.c
extern bool i2c_read_regs_port(i2c_inst_t *port, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);
extern bool bno_read(uart_inst_t *u, uint8_t reg, uint8_t len, uint8_t *out);
extern bool bno_write(uart_inst_t *u, uint8_t reg, const uint8_t *data, uint8_t len);

// ===== CRC32 para validación =====
static uint32_t crc32_simple(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

// ===== Lectura de estado de calibración =====
static bool read_calib_i2c(i2c_inst_t *port, uint8_t addr, uint8_t *out) {
    return i2c_read_regs_port(port, addr, BNO055_CALIB_STAT_ADDR, out, 1);
}

static bool read_calib_uart(uint8_t *out) {
    return bno_read(UART_ID, BNO055_CALIB_STAT_ADDR, 1, out);
}

// ===== Leer offsets de sensor I2C =====
static bool read_offsets_i2c_fixed(i2c_inst_t *port, uint8_t addr, uint8_t offsets[22]) {
    uint8_t cur_mode, calib_stat;
    
    // Verificar que está calibrado antes de leer offsets
    if (!i2c_read_regs_port(port, addr, BNO055_CALIB_STAT_ADDR, &calib_stat, 1)) {
        return false;
    }
    
    uint8_t sys = (calib_stat >> 6) & 0x03;
    if (sys < 2) {
        return false;
    }
    
    // Guardar modo actual
    if (!i2c_read_regs_port(port, addr, BNO055_OPR_MODE_ADDR, &cur_mode, 1)) {
        return false;
    }
    
    // Cambiar a modo CONFIG
    uint8_t config_mode = 0x00;
    if (i2c_write_timeout_us(port, addr, (uint8_t[]){BNO055_OPR_MODE_ADDR, config_mode}, 2, false, 10000) != 2) {
        return false;
    }
    sleep_ms(25);  // Tiempo crítico para cambio de modo

    // Leer offsets con doble verificación
    uint8_t offsets1[22], offsets2[22];
    bool read1 = i2c_read_regs_port(port, addr, 0x55, offsets1, 22);
    sleep_ms(5);
    bool read2 = i2c_read_regs_port(port, addr, 0x55, offsets2, 22);
    
    bool success = false;
    if (read1 && read2 && memcmp(offsets1, offsets2, 22) == 0) {
        // Verificar que no son datos ASCII obviamente inválidos
        int ascii_count = 0;
        for (int i = 0; i < 22; i++) {
            if (offsets1[i] >= 0x20 && offsets1[i] <= 0x7E) ascii_count++;
        }
        
        if (ascii_count < 10) {  // Menos de 10 caracteres ASCII = probablemente válido
            memcpy(offsets, offsets1, 22);
            success = true;
        }
    }
    
    // Restaurar modo original
    (void)i2c_write_timeout_us(port, addr, (uint8_t[]){BNO055_OPR_MODE_ADDR, cur_mode}, 2, false, 10000);
    sleep_ms(25);
    
    return success;
}

// ===== Leer offsets de sensor UART =====
static bool read_offsets_uart_fixed(uint8_t offsets[22]) {
    uint8_t calib_stat;
    
    // Verificar calibración
    if (!bno_read(UART_ID, BNO055_CALIB_STAT_ADDR, 1, &calib_stat)) {
        return false;
    }
    
    uint8_t sys = (calib_stat >> 6) & 0x03;
    if (sys < 2) {
        return false;
    }
    
    // Cambiar a CONFIG
    uint8_t config_mode = 0x00;
    if (!bno_write(UART_ID, BNO055_OPR_MODE_ADDR, &config_mode, 1)) {
        return false;
    }
    sleep_ms(25);
    
    // Leer offsets con verificación
    uint8_t offsets1[22], offsets2[22];
    bool read1 = bno_read(UART_ID, 0x55, 22, offsets1);
    bool read2 = bno_read(UART_ID, 0x55, 22, offsets2);
    
    bool success = false;
    if (read1 && read2 && memcmp(offsets1, offsets2, 22) == 0) {
        memcpy(offsets, offsets1, 22);
        success = true;
    }
    
    // Restaurar NDOF
    uint8_t ndof_mode = BNO055_OPR_MODE_NDOF;
    (void)bno_write(UART_ID, BNO055_OPR_MODE_ADDR, &ndof_mode, 1);
    sleep_ms(25);
    
    return success;
}

// ===== Guardar offsets en flash =====
static bool save_offsets_to_flash(uint8_t sensor_id, const uint8_t offsets[22], uint8_t calib_status) {
    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    offset_backup_t backups[MAX_BACKUPS];
    
    memcpy(backups, flash_data, sizeof(backups));
    
    int slot = -1;
    for (int i = 0; i < MAX_BACKUPS; i++) {
        if (backups[i].magic == FLASH_MAGIC_WORD && backups[i].sensor_id == sensor_id) {
            slot = i;
            break;
        }
        if (backups[i].magic != FLASH_MAGIC_WORD && slot == -1) {
            slot = i;
        }
    }
    
    if (slot == -1) {
        return false;
    }
    
    offset_backup_t *backup = &backups[slot];
    backup->magic = FLASH_MAGIC_WORD;
    backup->sensor_id = sensor_id;
    backup->calib_status = calib_status;
    memcpy(backup->offsets, offsets, 22);
    backup->crc = crc32_simple((uint8_t*)backup, sizeof(offset_backup_t) - sizeof(uint32_t));
    
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, (uint8_t*)backups, 256);
    restore_interrupts(ints);
    
    return true;
}

// ===== Restaurar offsets desde flash =====
static bool restore_offsets_from_flash(uint8_t sensor_id, uint8_t offsets[22]) {
    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    const offset_backup_t *backups = (const offset_backup_t*)flash_data;
    
    for (int i = 0; i < MAX_BACKUPS; i++) {
        const offset_backup_t *backup = &backups[i];
        
        if (backup->magic != FLASH_MAGIC_WORD || backup->sensor_id != sensor_id) {
            continue;
        }
        
        uint32_t expected_crc = crc32_simple((uint8_t*)backup, sizeof(offset_backup_t) - sizeof(uint32_t));
        if (backup->crc != expected_crc) {
            continue;
        }
        
        memcpy(offsets, backup->offsets, 22);
        return true;
    }
    
    return false;
}

// ===== Escribir offsets a sensor I2C =====
static bool write_offsets_i2c(i2c_inst_t *port, uint8_t addr, const uint8_t offsets[22]) {
    uint8_t cur_mode;
    
    if (!i2c_read_regs_port(port, addr, BNO055_OPR_MODE_ADDR, &cur_mode, 1)) {
        return false;
    }
    
    uint8_t config_mode = 0x00;
    if (i2c_write_timeout_us(port, addr, (uint8_t[]){BNO055_OPR_MODE_ADDR, config_mode}, 2, false, 10000) != 2) {
        return false;
    }
    sleep_ms(25);
    
    bool success = true;
    for (int i = 0; i < 22; i++) {
        uint8_t reg_data[2] = {0x55 + i, offsets[i]};
        if (i2c_write_timeout_us(port, addr, reg_data, 2, false, 10000) != 2) {
            success = false;
            break;
        }
        sleep_us(200);
    }
    
    (void)i2c_write_timeout_us(port, addr, (uint8_t[]){BNO055_OPR_MODE_ADDR, cur_mode}, 2, false, 10000);
    sleep_ms(25);
    
    return success;
}

// ===== Escribir offsets a sensor UART =====
static bool write_offsets_uart(const uint8_t offsets[22]) {
    uint8_t config_mode = 0x00;
    if (!bno_write(UART_ID, BNO055_OPR_MODE_ADDR, &config_mode, 1)) {
        return false;
    }
    sleep_ms(25);
    
    bool success = true;
    for (int i = 0; i < 22; i++) {
        if (!bno_write(UART_ID, 0x55 + i, &offsets[i], 1)) {
            success = false;
            break;
        }
        sleep_ms(1);
    }
    
    uint8_t ndof_mode = BNO055_OPR_MODE_NDOF;
    (void)bno_write(UART_ID, BNO055_OPR_MODE_ADDR, &ndof_mode, 1);
    sleep_ms(25);
    
    uint8_t xtal_on = 0x80;
    (void)bno_write(UART_ID, BNO055_SYS_TRIGGER_ADDR, &xtal_on, 1);
    sleep_ms(650);
    
    return success;
}

// ===== API Pública: Backup de todos los sensores calibrados =====
void backup_all_offsets(void) {
    // Backup sensor UART
    uint8_t uart_offsets[22];
    if (read_offsets_uart_fixed(uart_offsets)) {
        uint8_t uart_calib;
        if (read_calib_uart(&uart_calib)) {
            save_offsets_to_flash(0, uart_offsets, uart_calib);
        }
    }
    
    // Backup sensores I2C
    uint8_t sensor_id = 1;
    i2c_inst_t *ports[] = {I2C_PORT, I2C_PORT_1};
    
    for (int p = 0; p < 2; p++) {
        for (size_t i = 0; i < NUM_SENSORS; i++) {
            uint8_t offsets[22];
            if (read_offsets_i2c_fixed(ports[p], SENSOR_ADDR[i], offsets)) {
                uint8_t calib_stat;
                if (read_calib_i2c(ports[p], SENSOR_ADDR[i], &calib_stat)) {
                    save_offsets_to_flash(sensor_id, offsets, calib_stat);
                }
            }
            sensor_id++;
        }
    }
}

// ===== API Pública: Restaurar todos los offsets =====
void restore_all_offsets(void) {
    // Restaurar sensor UART
    uint8_t uart_offsets[22];
    if (restore_offsets_from_flash(0, uart_offsets)) {
        write_offsets_uart(uart_offsets);
    }
    
    // Restaurar sensores I2C
    uint8_t sensor_id = 1;
    i2c_inst_t *ports[] = {I2C_PORT, I2C_PORT_1};
    
    for (int p = 0; p < 2; p++) {
        for (size_t i = 0; i < NUM_SENSORS; i++) {
            uint8_t offsets[22];
            if (restore_offsets_from_flash(sensor_id, offsets)) {
                write_offsets_i2c(ports[p], SENSOR_ADDR[i], offsets);
            }
            sensor_id++;
        }
    }
}
