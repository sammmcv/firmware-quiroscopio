// get_sensors.c - Lectura de sensores (I2C + UART)
// Módulo independiente para gestión de sensores BNO055

#include "../include/shared_types.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/sync.h"
#include "hardware/irq.h"

// forward (local): se usa antes de definirse
static void try_reinit_bno_i2c(i2c_inst_t *port, uint8_t addr);

// ===== I2C helpers (no bloqueantes, sin float) =====
static inline int16_t le16(const uint8_t *p){
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// Lee N bytes con deadline total (no por byte)
static bool i2c_read_n_with_deadline(i2c_inst_t *i2c, uint8_t dev_addr,
                                     uint8_t start_reg, uint8_t *buf, size_t n,
                                     uint32_t deadline_us)
{
    uint64_t limit = time_us_64() + deadline_us;

    // Seleccionar registro inicial con repeated start
    while (i2c_write_blocking(i2c, dev_addr, &start_reg, 1, true) == PICO_ERROR_GENERIC){
        if (time_us_64() > limit) return false;
        tight_loop_contents();
    }

    size_t got = 0;
    while (got < n){
        int r = i2c_read_blocking(i2c, dev_addr, buf + got, (int)(n - got), false);
        if (r < 0) {
            if (time_us_64() > limit) return false;
            tight_loop_contents();
            continue;
        }
        got += (size_t)r;
    }
    return true;
}

// ===== Hardware Defines =====
#define I2C_PROBE_TIMEOUT_US 5000
#define I2C_RW_TIMEOUT_US    5000

// I2C0 (core0)
#define I2C_SDA_PIN   4
#define I2C_SCL_PIN   5
#define I2C_PORT      i2c0
#define I2C_BAUDRATE  400000

// I2C1 (core1)
#define I2C_SDA_PIN_1 6
#define I2C_SCL_PIN_1 7
#define I2C_PORT_1    i2c1
#define I2C_BAUDRATE_1 400000

// Pines BNO055 para modo UART (PS0 a GND, PS1 a VDDIO)
#define BNO_PS0_PIN      2
#define BNO_PS1_PIN      3

// UART por GPIO
#define UART_ID          uart0
#define UART_BAUDRATE    115200
#define UART_TX_PIN      0
#define UART_RX_PIN      1

// ===== BNO055 Protocol Defines =====
#define BNO_START_BYTE   0xAA
#define BNO_WRITE_CMD    0x00
#define BNO_READ_CMD     0x01
#define BNO_ACK_HEADER   0xEE
#define BNO_READ_RSP     0xBB

// ===== BNO055 Register Defines =====
#define BNO055_CHIP_ID_ADDR                    0x00
#define BNO055_OPR_MODE_ADDR                   0x3D
#define BNO055_PWR_MODE_ADDR                   0x3E
#define BNO055_SYS_TRIGGER_ADDR                0x3F
#define BNO055_UNIT_SEL_ADDR                   0x41
#define BNO055_DATA_SELECT_ADDR                0x42
#define BNO055_AXIS_MAP_SIGN_ADDR              0x3B
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR    0x28
#define BNO055_CALIB_STAT_ADDR                 0x35
#define BNO055_QUATERNION_DATA_W_LSB_ADDR      0x20

// ===== BNO055 Modes =====
#define BNO055_OPR_MODE_NDOF                   0x0C
#define BNO055_PWR_MODE_NORMAL                 0x00
#define BNO055_UNIT_SEL_DEFAULT                0x24

// ===== Aliases y Escalas =====
#define BNO_REG_LIA     BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR
#define BNO_ACC_SCALE   (1.0f / 100.f)      // 1 m/s² = 100 LSB
#define BNO_QUAT_SCALE  (1.0f / 16384.0f)   // cuaternión escala

// ===== Sensor Config =====
const uint8_t SENSOR_ADDR[] = { 0x28, 0x29 };
// NUM_SENSORS definido en shared_types.h

// ===== Auto-reinit Defines =====
#define REINIT_COOLDOWN_MS 800
#define ZERO_STUCK_N       3
#define FAIL_STUCK_N       2

// (Tipos vienen de ../include/shared_types.h)

// ===== USE_EXT_XTAL Define =====
#ifndef USE_EXT_XTAL
#define USE_EXT_XTAL 0
#endif

// ===== RAW register windows (ajustadas a defines BNO*) =====
#ifndef REG_ACC_START
#define REG_ACC_START   BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR
#endif
#ifndef REG_QUAT_START
#define REG_QUAT_START  BNO055_QUATERNION_DATA_W_LSB_ADDR
#endif

// ===== RAW I2C reads (ACC 6B, QUAT 8B) =====
bool bno_i2c_read_acc_raw(i2c_inst_t *i2c, uint8_t dev_addr, sensor_sample_raw_t *s){
    uint8_t buf[6];
    if (!i2c_read_n_with_deadline(i2c, dev_addr, REG_ACC_START, buf, sizeof buf, 3000))
        return false;
    s->ax_mg = le16(&buf[0]);
    s->ay_mg = le16(&buf[2]);
    s->az_mg = le16(&buf[4]);
    return true;
}

bool bno_i2c_read_quat_raw(i2c_inst_t *i2c, uint8_t dev_addr, sensor_sample_raw_t *s){
    uint8_t buf[8];
    if (!i2c_read_n_with_deadline(i2c, dev_addr, REG_QUAT_START, buf, sizeof buf, 3000))
        return false;
    s->qw_q14 = le16(&buf[0]);
    s->qi_q14 = le16(&buf[2]);
    s->qj_q14 = le16(&buf[4]);
    s->qk_q14 = le16(&buf[6]);
    return true;
}

// ===== idx mapping helpers =====
static inline i2c_inst_t* i2c_for_idx(int idx){
    // idx: 1=I2C0/0x28, 2=I2C0/0x29, 3=I2C1/0x28, 4=I2C1/0x29
    if (idx <= 0) return NULL;
    return (idx <= 2) ? I2C_PORT : I2C_PORT_1;
}

static inline uint8_t i2c_addr_for_idx(int idx){
    int local = ((idx - 1) % 2);
    return SENSOR_ADDR[local];
}

// Devuelve muestra cruda para el sensor i2c 'idx' (1–4)
bool read_bno_i2c_sample_raw(int idx, sensor_sample_raw_t *out){
    i2c_inst_t *i2c = i2c_for_idx(idx);
    if (!i2c || !out) return false;
    uint8_t addr = i2c_addr_for_idx(idx);

    sensor_sample_raw_t s = {0};
    if (!bno_i2c_read_acc_raw(i2c, addr, &s)) return false;
    if (!bno_i2c_read_quat_raw(i2c, addr, &s)) return false;

    // Aplicar calibración si existiera (opcional)
    // acc_apply_calib_q15(&s, &g_acc_calib[idx]); // si/cuando tengas tabla cargada

    *out = s;
    return true;
}
// ===== Variables Globales (exportadas) =====
sensor_sample_t g_uart = {0};
sensor_sample_t g_i2c0[NUM_SENSORS] = {0};
sensor_sample_t g_i2c1[NUM_SENSORS] = {0};
spin_lock_t *g_i2c1_lock;

// ===== Variables Internas =====
heal_state_t i2c0_heal[NUM_SENSORS] = {0};  // Expuesta para main.c
static heal_state_t i2c1_heal[NUM_SENSORS] = {0};  // Solo usada internamente en core1
uint8_t zero_run_uart = 0, fail_uart = 0, moved_uart = 0;  // Expuestas para main.c
uint32_t last_reinit_uart = 0;  // Expuesta para main.c

// ===== Helper Functions =====
static inline bool cooldown_elapsed(uint32_t *stamp, uint32_t ms) {
    uint32_t t = to_ms_since_boot(get_absolute_time());
    if (t - *stamp < ms) return false;
    *stamp = t;
    return true;
}

// ===== I2C Functions =====
void init_i2c_port(i2c_inst_t *port, uint sda_pin, uint scl_pin, uint baud) {
    i2c_init(port, baud);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

static void config_sensor_i2c(i2c_inst_t *port, uint8_t addr) {
    sleep_ms(50);
    const uint8_t rst[2] = { BNO055_SYS_TRIGGER_ADDR, 0x40 };
    i2c_write_blocking(port, addr, rst, 2, false);
    sleep_ms(650); // espera larga tras reset

    const uint8_t cfg0[][2] = {
        { BNO055_OPR_MODE_ADDR,    0x00 },                    // CONFIG
        { BNO055_PWR_MODE_ADDR,    BNO055_PWR_MODE_NORMAL },  // NORMAL
    };
    for (int i = 0; i < 2; ++i) { 
        i2c_write_blocking(port, addr, cfg0[i], 2, false); 
        sleep_ms(20); 
    }

    // Activar cristal externo solo si USE_EXT_XTAL y no es I2C0/0x28
    if (USE_EXT_XTAL && !(port == I2C_PORT && addr == 0x28)) {
        const uint8_t xtal_on[2] = { BNO055_SYS_TRIGGER_ADDR, 0x80 };
        i2c_write_blocking(port, addr, xtal_on, 2, false);
        sleep_ms(650);
    }

    const uint8_t cfg1[][2] = {
        { BNO055_UNIT_SEL_ADDR,    BNO055_UNIT_SEL_DEFAULT }, // unidades
        { BNO055_DATA_SELECT_ADDR, 0x00 },                    // data select
        { BNO055_AXIS_MAP_SIGN_ADDR, 0x08 },                  // signo ejes
        { BNO055_OPR_MODE_ADDR,    BNO055_OPR_MODE_NDOF }     // NDOF
    };
    for (int i = 0; i < 4; ++i) { 
        i2c_write_blocking(port, addr, cfg1[i], 2, false); 
        sleep_ms(20); 
    }
    sleep_ms(100);
}

void init_bno_i2c(i2c_inst_t *port, uint8_t addr) {
    uint8_t reg = BNO055_CHIP_ID_ADDR, chipID = 0;
    int w = i2c_write_timeout_us(port, addr, &reg, 1, true, I2C_PROBE_TIMEOUT_US);
    int r = (w == 1) ? i2c_read_timeout_us(port, addr, &chipID, 1, false, I2C_PROBE_TIMEOUT_US) : -1;
    if (r != 1) {
        return;
    }
    config_sensor_i2c(port, addr);
}

static void i2c_bus_recover(i2c_inst_t *port) {
    uint sda_pin = (port == I2C_PORT) ? I2C_SDA_PIN : I2C_SDA_PIN_1;
    uint scl_pin = (port == I2C_PORT) ? I2C_SCL_PIN : I2C_SCL_PIN_1;
    uint baud    = (port == I2C_PORT) ? I2C_BAUDRATE : I2C_BAUDRATE_1;
    
    i2c_deinit(port);
    gpio_set_function(sda_pin, GPIO_FUNC_SIO);
    gpio_set_function(scl_pin, GPIO_FUNC_SIO);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    gpio_set_dir(sda_pin, GPIO_IN);
    gpio_set_dir(scl_pin, GPIO_OUT);
    
    for (int i = 0; i < 9 && !gpio_get(sda_pin); ++i) {
        gpio_put(scl_pin, 0); sleep_us(5);
        gpio_put(scl_pin, 1); sleep_us(5);
    }
    
    // STOP manual
    gpio_set_dir(sda_pin, GPIO_OUT);
    gpio_put(sda_pin, 0); sleep_us(5);
    gpio_put(scl_pin, 1); sleep_us(5);
    gpio_put(sda_pin, 1); sleep_us(5);
    
    // Volver a modo I2C
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    i2c_init(port, baud);
}

bool i2c_read_regs_port(i2c_inst_t *port, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    const uint32_t TO_US = I2C_RW_TIMEOUT_US;

    int w = i2c_write_timeout_us(port, addr, &reg, 1, true, TO_US);
    if (w != 1) {
        i2c_bus_recover(port);
        w = i2c_write_timeout_us(port, addr, &reg, 1, true, TO_US);
        if (w != 1) return false;
    }

    int r = i2c_read_timeout_us(port, addr, buf, (uint)len, false, TO_US);
    if (r != (int)len) {
        i2c_bus_recover(port);
        w = i2c_write_timeout_us(port, addr, &reg, 1, true, TO_US);
        if (w != 1) return false;
        r = i2c_read_timeout_us(port, addr, buf, (uint)len, false, TO_US);
        if (r != (int)len) return false;
    }
    return true;
}

// Re-inicializar el sensor I2C si responde
static void try_reinit_bno_i2c(i2c_inst_t *port, uint8_t addr) {
    // Si el dispositivo responde, re-afirma NDOF y listo
    uint8_t reg = BNO055_CHIP_ID_ADDR, cid = 0;
    int w = i2c_write_timeout_us(port, addr, &reg, 1, true, I2C_PROBE_TIMEOUT_US);
    if (w != 1) return;
    int r = i2c_read_timeout_us(port, addr, &cid, 1, false, I2C_PROBE_TIMEOUT_US);
    if (r != 1 || cid != 0xA0) return;

    const uint8_t ndof[2] = { BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_NDOF };
    (void)i2c_write_blocking(port, addr, ndof, 2, false);
    sleep_ms(25);
}

// ===== UART Functions =====
// ---- UART RX Ring Buffer + IRQ ----
#define UART_RX_BUF 1024
static volatile uint8_t uart_rb[UART_RX_BUF];
static volatile uint16_t uart_rb_head = 0;
static volatile uint16_t uart_rb_tail = 0;

static inline uint16_t uart_rb_next(uint16_t idx){ return (uint16_t)((idx + 1) % UART_RX_BUF); }
static inline bool uart_rb_empty(void){ return uart_rb_head == uart_rb_tail; }

static inline bool uart_rb_pop(uint8_t *out){
    if (uart_rb_empty()) return false;
    uint16_t t = uart_rb_tail;
    *out = uart_rb[t];
    uart_rb_tail = uart_rb_next(t);
    return true;
}

static inline void uart_rb_flush_all(uart_inst_t *u){
    // Vaciar ring buffer y FIFO HW
    uart_rb_tail = uart_rb_head;
    while (uart_is_readable(u)) (void)uart_getc(u);
}

static void on_uart0_rx(void){
    while (uart_is_readable(UART_ID)) {
        uint8_t b = uart_getc(UART_ID);
        uint16_t n = uart_rb_next(uart_rb_head);
        if (n != uart_rb_tail) {
            uart_rb[uart_rb_head] = b;
            uart_rb_head = n;
        } else {
            // overflow -> drop byte
        }
    }
}

static inline void uart_rx_flush(uart_inst_t *u) {
    // Redefinido para limpiar ring + FIFO
    uart_rb_flush_all(u);
}

static bool uart_read_byte_timeout(uart_inst_t *u, uint8_t *b, uint32_t timeout_ms) {
    // Reimplementado usando ring buffer + deadline
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    (void)u; // no se usa, lectura desde ring
    while (!time_reached(deadline)) {
        if (uart_rb_pop(b)) return true;
        tight_loop_contents();
    }
    return false;
}

static bool bno_wait_header(uart_inst_t *u, uint8_t *hdr, uint32_t timeout_ms) {
    // Esperar 0xBB (READ_RSP) o 0xEE (ACK) desde ring buffer
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    (void)u;
    while (!time_reached(deadline)) {
        uint8_t c;
        if (uart_rb_pop(&c)) {
            if (c == BNO_READ_RSP || c == BNO_ACK_HEADER) {
                *hdr = c;
                return true;
            }
            // descartar otros bytes
        }
        tight_loop_contents();
    }
    return false;
}

// Leer N bytes desde ring buffer con deadline en microsegundos
static bool uart_rb_read_n_with_deadline(uint8_t *out, size_t n, uint32_t deadline_us){
    uint64_t limit = time_us_64() + deadline_us;
    size_t i = 0;
    while (i < n) {
        uint8_t b;
        if (uart_rb_pop(&b)) {
            out[i++] = b;
        } else {
            if (time_us_64() > limit) return false;
            tight_loop_contents();
        }
    }
    return true;
}

bool bno_read(uart_inst_t *u, uint8_t reg, uint8_t len, uint8_t *out) {
    uart_rx_flush(u);
    uint8_t frame[4] = {BNO_START_BYTE, BNO_READ_CMD, reg, len};
    uart_write_blocking(u, frame, 4);

    uint8_t hdr;
    if (!bno_wait_header(u, &hdr, 200) || hdr != BNO_READ_RSP) return false;

    uint8_t rlen;
    if (!uart_read_byte_timeout(u, &rlen, 3)) return false; // ~3ms max
    if (rlen != len) {
        // Drenar carga reportada y salir
        uint8_t dump_buf[32];
        size_t to_drain = rlen;
        while (to_drain) {
            size_t chunk = to_drain > sizeof(dump_buf) ? sizeof(dump_buf) : to_drain;
            if (!uart_rb_read_n_with_deadline(dump_buf, chunk, 3000)) break;
            to_drain -= chunk;
        }
        return false;
    }

    if (!uart_rb_read_n_with_deadline(out, len, 3000)) return false; // ~3ms total
    return true;
}

bool bno_write(uart_inst_t *u, uint8_t reg, const uint8_t *data, uint8_t len) {
    uart_rx_flush(u);
    uint8_t hdr_frame[4] = {BNO_START_BYTE, BNO_WRITE_CMD, reg, len};
    uart_write_blocking(u, hdr_frame, 4);
    if (len > 0) {
        uart_write_blocking(u, data, len);
    }

    uint8_t ack_hdr, status_code;
    if (!bno_wait_header(u, &ack_hdr, 200) || ack_hdr != BNO_ACK_HEADER) return false;
    if (!uart_read_byte_timeout(u, &status_code, 3)) return false; // ~3ms máximo

    return status_code == 0x01; // 0x01 es WRITE_SUCCESS
}

void init_bno_uart(void) {
    gpio_init(BNO_PS0_PIN);
    gpio_set_dir(BNO_PS0_PIN, GPIO_OUT);
    gpio_put(BNO_PS0_PIN, 0);

    gpio_init(BNO_PS1_PIN);
    gpio_set_dir(BNO_PS1_PIN, GPIO_OUT);
    gpio_put(BNO_PS1_PIN, 1);
    
    // Configurar UART
    uart_init(UART_ID, UART_BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    // Inicializar ring buffer
    uart_rb_head = uart_rb_tail = 0;
    uart_rx_flush(UART_ID);

    // Configurar IRQ RX
    irq_set_exclusive_handler(UART0_IRQ, on_uart0_rx);
    // Opcional: prioridad más alta para minimizar pérdida
    // irq_set_priority(UART0_IRQ, 0x40);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false); // RX on, TX off
}

bool bno_read_linear_acc(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    if (!bno_read(UART_ID, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, 6, buf)) {
        *x = *y = *z = 0;
        return false;
    }
    *x = (int16_t)((buf[1] << 8) | buf[0]);
    *y = (int16_t)((buf[3] << 8) | buf[2]);
    *z = (int16_t)((buf[5] << 8) | buf[4]);
    return true;
}

void bno_uart_recover(void) {
    uint8_t m = BNO055_OPR_MODE_NDOF;
    (void)bno_write(UART_ID, BNO055_OPR_MODE_ADDR, &m, 1);
    sleep_ms(25);
}

// ===== Raw Sensor Reading (sin FP) =====
// Lee aceleración lineal en crudo (int16, unidad LSB según BNO)
bool bno_read_acc_raw(sensor_sample_raw_t* s) {
    uint8_t buf[6];
    if (!bno_read(UART_ID, BNO_REG_LIA, 6, buf)) {
        return false;
    }
    // BNO055 devuelve little-endian, escala 1 LSB = 0.01 m/s² (100 LSB = 1 m/s²)
    // Guardamos LSB tal cual para procesamiento posterior
    int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
    s->ax_mg = ax;
    s->ay_mg = ay;
    s->az_mg = az;
    return true;
}

// Lee cuaternión en crudo (int16, Q14 según BNO)
bool bno_read_quat_raw(sensor_sample_raw_t* s) {
    uint8_t buf[8];
    if (!bno_read(UART_ID, BNO055_QUATERNION_DATA_W_LSB_ADDR, 8, buf)) {
        return false;
    }
    // BNO055: little-endian, escala Q14 (1 LSB = 1/16384)
    int16_t qw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t qx = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t qy = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t qz = (int16_t)((buf[7] << 8) | buf[6]);
    s->qw_q14 = qw;
    s->qi_q14 = qx;
    s->qj_q14 = qy;
    s->qk_q14 = qz;
    return true;
}
