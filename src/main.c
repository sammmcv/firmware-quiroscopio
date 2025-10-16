// ===========================
// main.c
// Sistema principal de coordianación y bucle de adquisición
// Inicializa BLE, sensores (UART + I2C0), lanza Core1,
// orquesta polling y envío de supertramas
// ===========================

#include "../include/shared_types.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/sync.h"
#include "hardware/flash.h"

#include "btstack.h"

#include "../include/shared_types.h"

// ========== Timing y Control ==========
#define BOOT_WAIT_MS    3000
#define LOOP_PERIOD_MS  10

// ========== Variables de control global ==========
volatile bool g_show_calib = false;
volatile bool g_freeze_i2c0 = false;
volatile bool g_freeze_i2c1 = false;
uint64_t g_line_number = 0;

// ========== Variables de métricas de bucle ==========
static uint32_t loop_start_us = 0;
static uint32_t loop_count = 0;
static uint32_t min_loop_us = UINT32_MAX;
static uint32_t max_loop_us = 0;
static uint32_t total_loop_us = 0;

// ========== Declaraciones externas ==========
// De get_sensors.c
extern sensor_sample_t g_uart;
extern sensor_sample_t g_i2c0[];
extern sensor_sample_t g_i2c1[];
extern spin_lock_t *g_i2c1_lock;

extern void init_i2c_port(i2c_inst_t *port, uint sda_pin, uint scl_pin, uint baud);
extern void init_bno_i2c(i2c_inst_t *port, uint8_t addr);
extern void init_bno_uart(void);
extern bool bno_read(uart_inst_t *uart, uint8_t reg, size_t len, uint8_t *dst);
extern bool bno_write(uart_inst_t *uart, uint8_t reg, const uint8_t *data, size_t len);
// poll_i2c_bus ya está declarada en shared_types.h, no redeclarar aquí
extern void bno_uart_recover(void);

// De calibration.c
extern void backup_all_offsets(void);
extern void restore_all_offsets(void);

// De frame_building.c - Ya declaradas en shared_types.h, no redeclarar aquí
// superframe_stage_sensor, superframe_flush_and_send_if_ready


// De ble_connection.c
extern void ble_try_flush(void);
extern uint16_t ble_tx_free_space(void);
extern volatile uint32_t ble_skipped_samples;

// Variable generada por gatt_db.h (desde include/gatt_db.h generado)
extern const uint8_t profile_data[];

// ========== Defines de BNO055 ==========
#define BNO055_CHIP_ID_ADDR     0x00
#define BNO055_UNIT_SEL_ADDR    0x3B
#define BNO055_OPR_MODE_ADDR    0x3D
#define BNO055_OPR_MODE_NDOF    0x0C
#define BNO055_UNIT_SEL_DEFAULT 0x00
#define BNO055_ACC_SCALE        0.01f

// ========== Defines de sensores ==========
#define UART_ID        uart0
#define I2C_PORT       i2c0
#define I2C_SDA_PIN    4
#define I2C_SCL_PIN    5
#define I2C_BAUDRATE   400000

// Direcciones I2C
extern const uint8_t SENSOR_ADDR[];

// ========== Defines de auto-sanación ==========
#define ZERO_STUCK_N           10
#define FAIL_STUCK_N           25
#define REINIT_COOLDOWN_MS     2000

// ========== Helpers de tiempo ==========
static inline bool cooldown_elapsed(volatile uint32_t *last_ms, uint32_t cooldown_ms) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - *last_ms >= cooldown_ms) {
        *last_ms = now;
        return true;
    }
    return false;
}

static inline bool near_zero3(float x, float y, float z) {
    const float thresh = 0.02f;
    return (fabsf(x) < thresh && fabsf(y) < thresh && fabsf(z) < thresh);
}

// ========== Helpers LED/USB ==========
static inline void led_on(bool on) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

static inline bool usb_conectado(void) {
    return stdio_usb_connected();
}

// ========== Core1 Entry Point ==========
// Ejecuta polling continuo de I2C1 (segundo bus I2C)
void core1_entry(void) {
    // Inicializar I2C1
    const uint I2C_PORT_1 = 1;
    init_i2c_port(i2c1, 6, 7, 400000);
    
    // Inicializar BNO055 en I2C1
    for (size_t i = 0; i < NUM_SENSORS; ++i) {
        init_bno_i2c(i2c1, SENSOR_ADDR[i]);
    }
    
    // Core1 sin polling de I2C1 (ruta I2C RAW la lleva Core0 ahora)
    while (true) {
        sleep_ms(10);
    }
}

// ========================
int main() {
    stdio_init_all();
    
    // ===== Inicialización BLE (BTstack + CYW43) =====
    if (cyw43_arch_init()) {
        return -1;
    }
    
    // Desactivar Wifi, solo BLE
    cyw43_arch_disable_sta_mode();
    
    // Inicializar spin lock para BLE buffer PRIMERO
    uint ble_lock_num = spin_lock_claim_unused(true);
    spin_lock_t *ble_lock = spin_lock_init(ble_lock_num);
    ble_init_lock(ble_lock);  // Pasar lock a ble_connection.c
    
    l2cap_init();
    sm_init();
    
    // Solicitar MTU de 247 bytes (máximo BLE)
    l2cap_set_max_le_mtu(247);
    
    att_server_init(profile_data, NULL, NULL);
    
    // Registrar handler ATT/HCI (delegado a ble_connection.c)
    ble_stack_register_handlers();

    bd_addr_t null_addr = {0,0,0,0,0,0};
    gap_advertisements_set_params(0x0030, 0x0030, 0, 0, null_addr, 0x07, 0);
    const uint8_t adv_data[] = {
        0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
        0x08, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'q','u','i','r','o','s','c','o','p','e'
    };
    gap_advertisements_set_data(sizeof(adv_data), (uint8_t*)adv_data);
    hci_power_control(HCI_POWER_ON);
    gap_advertisements_enable(1);
    // MARCAR que arrancamos publicitando
    ble_advertising = true;

    sleep_ms(BOOT_WAIT_MS);
    
    // Inicializar spin lock para datos de Core1
    uint lock_num = spin_lock_claim_unused(true);
    g_i2c1_lock = spin_lock_init(lock_num);

    // --- Inicialización Core0 ---
    init_i2c_port(I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, I2C_BAUDRATE);
    for (size_t i = 0; i < NUM_SENSORS; ++i) {
        init_bno_i2c(I2C_PORT, SENSOR_ADDR[i]);
    }
    init_bno_uart();
    // Fijar unidades también en el BNO por UART (una vez)
    uint8_t units = BNO055_UNIT_SEL_DEFAULT; (void)bno_write(UART_ID, BNO055_UNIT_SEL_ADDR, &units, 1); sleep_ms(10);
    uint8_t opmode = BNO055_OPR_MODE_NDOF; (void)bno_write(UART_ID, BNO055_OPR_MODE_ADDR, &opmode, 1);

    // Detectar BNO055 por UART y reportar
    uint8_t chip_id = 0;
    (void)bno_read(UART_ID, BNO055_CHIP_ID_ADDR, 1, &chip_id);

    // --- Lanzar Core1 (siempre) ---
    multicore_launch_core1(core1_entry);
    // Desactivar polling de I2C1 en Core1: leeremos I2C1 también desde Core0 en modo RAW
    g_freeze_i2c1 = true;
    // --- Bucle Principal (Core0) ---
    int16_t accelX, accelY, accelZ;
    float fX, fY, fZ; // usar aceleración lineal en I2C
    // --- Variables temporales para cuaterniones UART ---
    float fQX, fQY, fQZ, fQW;
    // --- Variable para lectura cruda UART (sin FP) ---
    sensor_sample_raw_t uart_raw;
    // Precalcular etiquetas para evitar snprintf en cada iteración
    char tag_i2c0[NUM_SENSORS][32], tag_i2c1[NUM_SENSORS][32];
    for (size_t i = 0; i < NUM_SENSORS; ++i) {
        snprintf(tag_i2c0[i], sizeof(tag_i2c0[i]), "I2C CORE0 0x%02X", SENSOR_ADDR[i]);
        snprintf(tag_i2c1[i], sizeof(tag_i2c1[i]), "I2C CORE1 0x%02X", SENSOR_ADDR[i]);
    }

    // --- Variables para medición de tiempo de bucle ---
    uint32_t last_report_ms = 0;

    while (true) {
        // --- Inicio de medición de bucle ---
        loop_start_us = time_us_32();

        // --- Comandos de control ---
        {
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                switch ((uint8_t)ch) {
                    case 0x00: // Modo calibración (toggle)
                        g_show_calib = !g_show_calib;
                        break;
                    case 0x01: // Backup offsets
                        g_freeze_i2c0 = g_freeze_i2c1 = true;
                        sleep_ms(LOOP_PERIOD_MS + 10);
                        backup_all_offsets();
                        g_freeze_i2c0 = g_freeze_i2c1 = false;
                        break;
                    case 0x02: // Restore offsets
                        g_freeze_i2c0 = g_freeze_i2c1 = true;
                        sleep_ms(LOOP_PERIOD_MS + 10);
                        restore_all_offsets();
                        g_freeze_i2c0 = g_freeze_i2c1 = false;
                        break;
                    case 0x03: // Estadísticas BLE
                        break;
                    default:
                        break;
                }
            }
        }

        // Asegurar re-activación de advertising cada 2s si no hay conexión ni advertising activo
        if (ble_con_handle == HCI_CON_HANDLE_INVALID && !ble_advertising) {
            if (cooldown_elapsed(&last_adv_restart, 2000)) {
                gap_advertisements_enable(1);
                ble_advertising = true;
                // ELIMINADO: printf
            }
        }

        // Estado LED según USB
        {
            static uint8_t usb_cnt = 0;
            static bool usb_estable = false;
            static uint32_t t_led = 0;

            bool v = usb_conectado();
            if (v == usb_estable) {
                usb_cnt = 0;
            } else if (++usb_cnt >= 4) {
                usb_estable = v;
                usb_cnt = 0;
            }

            if (usb_estable) {
                led_on(true);
            } else {
                uint32_t now = to_ms_since_boot(get_absolute_time());
                if (now - t_led >= 1000) {
                    t_led = now;
                    static bool s = false;
                    s = !s;
                    led_on(s);
                }
            }
        }

        // === BACKPRESSURE: Dar tiempo a BLE para vaciar buffer ===
        // Si el buffer está > 75% lleno, dar prioridad a transmitir
        uint16_t free_space = ble_tx_free_space();
        if (free_space < (BLE_TX_BUF_SIZE / 4)) {
            // Buffer casi lleno - saltar esta muestra y dar tiempo a BLE
            ble_skipped_samples++;
            ble_try_flush();  // Intentar vaciar inmediatamente
            sleep_ms(LOOP_PERIOD_MS);
            continue;
        }

        // Lectura de sensores I2C en Core0 (RAW, idx 1-4)
        sensor_sample_raw_t s1 = {0}, s2 = {0}, s3 = {0}, s4 = {0};
        bool i2c1_ok = !g_freeze_i2c0 ? read_bno_i2c_sample_raw(1, &s1) : false;
        bool i2c2_ok = !g_freeze_i2c0 ? read_bno_i2c_sample_raw(2, &s2) : false;
        bool i2c3_ok = read_bno_i2c_sample_raw(3, &s3);
        bool i2c4_ok = read_bno_i2c_sample_raw(4, &s4);
        
        // Lectura de sensor UART en Core0 (RAW - sin FP)
        bool uart_ok = bno_read_acc_raw(&uart_raw) && bno_read_quat_raw(&uart_raw);
        if (uart_ok) {
            // Convertir a float solo para validación/healing (no para encoding)
            fX = uart_raw.ax_mg * BNO055_ACC_SCALE;
            fY = uart_raw.ay_mg * BNO055_ACC_SCALE;
            fZ = uart_raw.az_mg * BNO055_ACC_SCALE;
            
            g_uart.valid = true;
            
            const float arm = 0.05f;
            bool mv = (fabsf(fX) > arm) || (fabsf(fY) > arm) || (fabsf(fZ) > arm);
            if (mv) moved_uart = 6; 
            else if (moved_uart) --moved_uart;

            if (near_zero3(fX, fY, fZ) && moved_uart) {
                if (++zero_run_uart >= ZERO_STUCK_N && 
                    cooldown_elapsed(&last_reinit_uart, REINIT_COOLDOWN_MS)) {
                    bno_uart_recover();
                    zero_run_uart = 0;
                }
            } else { 
                zero_run_uart = 0; 
                fail_uart = 0; 
            }
        } else {
            g_uart.valid = false;
            if (++fail_uart >= FAIL_STUCK_N && 
                cooldown_elapsed(&last_reinit_uart, REINIT_COOLDOWN_MS)) {
                bno_uart_recover();
                fail_uart = 0;
            }
        }

        // --- Modo calibración ---
        if (g_show_calib) {
            sleep_ms(LOOP_PERIOD_MS);
            continue;
        }

        // === EMISIÓN BINARIA CON SUPRATRAMA ===
        // Reset staging for this cycle
        extern uint8_t g_sf_presence;
        g_sf_presence = 0;
        
        // Stage UART sensor (idx=0) - USAR RAW DIRECTO
        if (g_uart.valid) {
            superframe_stage_sensor(0,
                uart_raw.ax_mg, uart_raw.ay_mg, uart_raw.az_mg,
                uart_raw.qw_q14, uart_raw.qi_q14, uart_raw.qj_q14, uart_raw.qk_q14);
        }
        
        // Stage I2C sensors RAW: 1..4
        if (i2c1_ok) {
            superframe_stage_sensor(1,
                s1.ax_mg, s1.ay_mg, s1.az_mg,
                s1.qw_q14, s1.qi_q14, s1.qj_q14, s1.qk_q14);
        }
        if (i2c2_ok) {
            superframe_stage_sensor(2,
                s2.ax_mg, s2.ay_mg, s2.az_mg,
                s2.qw_q14, s2.qi_q14, s2.qj_q14, s2.qk_q14);
        }
        if (i2c3_ok) {
            superframe_stage_sensor(3,
                s3.ax_mg, s3.ay_mg, s3.az_mg,
                s3.qw_q14, s3.qi_q14, s3.qj_q14, s3.qk_q14);
        }
        if (i2c4_ok) {
            superframe_stage_sensor(4,
                s4.ax_mg, s4.ay_mg, s4.az_mg,
                s4.qw_q14, s4.qi_q14, s4.qj_q14, s4.qk_q14);
        }
        
        // Flush supratrama if any sensors present
        superframe_flush_and_send_if_ready();
        
        // Intentar vaciar buffer BLE
        ble_try_flush();
        
        g_line_number++;
        
        // Fin de medición y cálculo de estadísticas ---
        uint32_t loop_end_us = time_us_32();
        uint32_t loop_time_us = loop_end_us - loop_start_us;
        
        // Actualizar estadísticas
        if (loop_time_us < min_loop_us) min_loop_us = loop_time_us;
        if (loop_time_us > max_loop_us) max_loop_us = loop_time_us;
        total_loop_us += loop_time_us;
        loop_count++;
        
        // Reportar estadísticas cada 2 segundos por SERIAL ÚNICAMENTE
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_report_ms >= 1000) {
            float avg_loop_us = (float)total_loop_us / loop_count;
            // Usar printf directo para evitar envío por BLE
            min_loop_us = UINT32_MAX;
            max_loop_us = 0;
            total_loop_us = 0;
            loop_count = 0;
            last_report_ms = now_ms;
        }

        sleep_ms(LOOP_PERIOD_MS);
    }
    return 0;
}
