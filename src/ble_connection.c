// ble_connection.c - Comunicación BLE y gestión de cola
// Módulo independiente que gestiona toda la comunicación Bluetooth Low Energy

#include "../include/shared_types.h"
#include "../include/btstack_config.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "gatt_db.h"
#include "hardware/sync.h"

// ===== Defines BLE =====
#define BLE_TX_BUF_SIZE 4096  // Buffer circular para cola de transmisión
#define TXQ_MAX 256           // Cola de supertramas para CAN_SEND_NOW (aumentado para minimizar drops)

// ===== Variables BLE Globales =====
uint16_t ble_con_handle = HCI_CON_HANDLE_INVALID;  // Expuesta para main.c
static btstack_packet_callback_registration_t ble_hci_cb;

// --- BLE TX queue (buffer circular con protección thread-safe) ---
static uint8_t  ble_tx_buf[BLE_TX_BUF_SIZE];
static volatile uint16_t ble_tx_write_pos = 0;
static volatile uint16_t ble_tx_read_pos = 0;
static spin_lock_t *ble_tx_lock;

// --- Estado de advertising + chunk dinámico ---
volatile bool ble_advertising = false;  // Expuesta para main.c
volatile uint32_t last_adv_restart = 0;  // Expuesta para main.c
static volatile uint16_t ble_chunk_max = 0;  // MTU negociado

// --- Nueva cola CAN_SEND_NOW (patrón BTstack oficial) ---
static struct {
    uint16_t len;
    uint8_t data[256];
} txq[TXQ_MAX];
static volatile uint16_t txq_head = 0;
static volatile uint16_t txq_tail = 0;
static uint16_t att_handle_tx = 0;  // Handle de TX characteristic (se inicializa después)

// --- MTU tracking ---
static uint16_t g_att_mtu = 23;  // Valor por defecto BLE

// --- Métricas BLE para debugging ---
volatile uint32_t ble_packets_sent = 0;
volatile uint32_t ble_bytes_sent = 0;
volatile uint32_t ble_drops = 0;
volatile uint32_t ble_skipped_samples = 0;

// ===== LED y USB Helpers =====
static inline void led_on(bool on) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

static inline bool usb_conectado(void) {
    // No usar VBUS - usar conexión stdio USB
    return stdio_usb_connected();
}

// ===== Funciones para cola CAN_SEND_NOW =====

// Obtener MTU actual
uint16_t ble_current_att_mtu(void) {
    return g_att_mtu;
}

// Actualizar MTU cuando cambie
static void on_mtu_change(uint16_t mtu) {
    g_att_mtu = mtu;
    // Mensaje eliminado para minimizar latencia
}

// Push a la cola CAN_SEND_NOW
static void txq_push(const uint8_t *d, uint16_t n) {
    if (n == 0 || n > 256) return;
    
    uint16_t next_head = (txq_head + 1) % TXQ_MAX;
    if (next_head == txq_tail) {
        // Cola llena - drop
        ble_drops++;
        return;
    }
    
    uint16_t i = txq_head;
    txq[i].len = n;
    memcpy(txq[i].data, d, n);
    
    // Barrier para asegurar escritura completa antes de avanzar head
    __dmb();
    txq_head = next_head;
    
    // Solicitar envío si hay conexión
    if (ble_con_handle != HCI_CON_HANDLE_INVALID) {
        att_server_request_can_send_now_event(ble_con_handle);
    }
}

// Pop de la cola CAN_SEND_NOW
static int txq_pop(uint8_t **d) {
    if (txq_tail == txq_head) return 0;  // Cola vacía
    
    uint16_t i = txq_tail;
    *d = txq[i].data;
    return txq[i].len;
}

// Consumir elemento de la cola
static void txq_consume(void) {
    if (txq_tail != txq_head) {
        txq_tail = (txq_tail + 1) % TXQ_MAX;
    }
}

// Verificar si hay datos en la cola CAN_SEND_NOW
static inline bool txq_has_data(void) {
    return txq_tail != txq_head;
}

// Vaciar cola CAN_SEND_NOW (llamar en desconexión)
static void txq_reset(void) {
    txq_head = 0;
    txq_tail = 0;
}

// ===== Buffer Circular - Helpers (legacy, mantener por compatibilidad) =====
uint16_t ble_tx_free_space(void) {
    uint16_t wp = ble_tx_write_pos;
    uint16_t rp = ble_tx_read_pos;
    if (wp >= rp) {
        return BLE_TX_BUF_SIZE - (wp - rp) - 1;
    } else {
        return (rp - wp) - 1;
    }
}

static inline uint16_t ble_tx_available(void) {
    uint16_t wp = ble_tx_write_pos;
    uint16_t rp = ble_tx_read_pos;
    if (wp >= rp) {
        return wp - rp;
    } else {
        return BLE_TX_BUF_SIZE - (rp - wp);
    }
}

static inline bool ble_tx_has_data(void) {
    return ble_tx_read_pos != ble_tx_write_pos;
}

static inline void ble_tx_reset(void) {
    uint32_t irq = spin_lock_blocking(ble_tx_lock);
    ble_tx_write_pos = 0;
    ble_tx_read_pos = 0;
    spin_unlock(ble_tx_lock, irq);
}

// ===== API Pública: Verificar espacio =====
bool ble_has_space_for(uint16_t bytes) {
    return ble_tx_free_space() >= (bytes + 32);  // Margen reducido (con MTU wait ya no necesitamos 100B)
}

// ===== Encolar datos con protección thread-safe =====
static void ble_tx_queue(const uint8_t *data, uint16_t len) {
    if (len == 0) return;
    
    uint32_t irq = spin_lock_blocking(ble_tx_lock);
    
    uint16_t free = ble_tx_free_space();
    if (len > free) {
        // Marcar drop pero intentar meter lo que quepa
        ble_drops++;
        len = free;
    }
    
    if (len > 0) {
        uint16_t wp = ble_tx_write_pos;
        
        // Copiar en dos partes si cruza el final del buffer
        uint16_t first_chunk = BLE_TX_BUF_SIZE - wp;
        if (len <= first_chunk) {
            memcpy(&ble_tx_buf[wp], data, len);
            ble_tx_write_pos = (wp + len) % BLE_TX_BUF_SIZE;
        } else {
            memcpy(&ble_tx_buf[wp], data, first_chunk);
            memcpy(&ble_tx_buf[0], data + first_chunk, len - first_chunk);
            ble_tx_write_pos = len - first_chunk;
        }
    }
    
    spin_unlock(ble_tx_lock, irq);
}

// ===== Vaciar cola respetando ATT CAN_SEND_NOW =====
void ble_try_flush(void) {
    // NUEVO: Usar la cola CAN_SEND_NOW si está habilitada
    if (txq_has_data() && ble_con_handle != HCI_CON_HANDLE_INVALID) {
        att_server_request_can_send_now_event(ble_con_handle);
        return;
    }
    
    // LEGACY: Mantener funcionalidad del buffer circular por compatibilidad
    if (ble_con_handle == HCI_CON_HANDLE_INVALID) {
        ble_tx_reset();
        return;
    }
    
    if (!ble_tx_has_data()) return;
    
    if (!att_server_can_send_packet_now(ble_con_handle)) {
        att_server_request_can_send_now_event(ble_con_handle);
        return;
    }
    
    // Enviar un paquete del buffer circular
    uint32_t irq = spin_lock_blocking(ble_tx_lock);
    
    uint16_t avail = ble_tx_available();
    if (avail == 0) {
        spin_unlock(ble_tx_lock, irq);
        return;
    }
    
    // Determinar tamaño del próximo paquete
    // Superframes tienen 2 bytes de control + 14*N bytes de datos (N = popcount de presence)
    // Mínimo: 2 bytes (sin sensores), Máximo: 72 bytes (5 sensores)
    uint16_t rp = ble_tx_read_pos;
    
    // Leer los 2 bytes de control para calcular tamaño
    uint8_t b0, b1;
    if (avail < 2) {
        spin_unlock(ble_tx_lock, irq);
        return;
    }
    
    // Peek en los primeros 2 bytes
    b0 = ble_tx_buf[rp];
    b1 = ble_tx_buf[(rp + 1) % BLE_TX_BUF_SIZE];
    
    uint8_t presence = (b0 >> 3) & 0x1F;
    uint16_t packet_size = 2 + 14 * __builtin_popcount((unsigned)presence);
    
    if (avail < packet_size) {
        // Paquete incompleto, esperar más datos
        spin_unlock(ble_tx_lock, irq);
        return;
    }
    
    uint8_t packet[72];  // Máximo tamaño de supratrama
    
    // Copiar paquete del buffer circular
    uint16_t first_chunk = BLE_TX_BUF_SIZE - rp;
    if (packet_size <= first_chunk) {
        memcpy(packet, &ble_tx_buf[rp], packet_size);
    } else {
        memcpy(packet, &ble_tx_buf[rp], first_chunk);
        memcpy(packet + first_chunk, &ble_tx_buf[0], packet_size - first_chunk);
    }
    
    ble_tx_read_pos = (rp + packet_size) % BLE_TX_BUF_SIZE;
    
    spin_unlock(ble_tx_lock, irq);
    
    // Enviar paquete
    att_server_notify(ble_con_handle,
        ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE,
        packet, packet_size);
    
    ble_packets_sent++;
    ble_bytes_sent += packet_size;
    
    // Solicitar siguiente envío si hay más datos
    if (ble_tx_has_data()) {
        att_server_request_can_send_now_event(ble_con_handle);
    }
}

// ===== BLE Packet Handler =====
static void ble_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    const uint8_t evt = hci_event_packet_get_type(packet);
    switch (evt) {
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                ble_con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                ble_advertising = false;
                ble_chunk_max = 0;  // NO enviar hasta recibir MTU
                
                // Inicializar handle de TX characteristic
                att_handle_tx = ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE;
                
                // Solicitar parámetros de conexión agresivos (15ms interval para baja latencia)
                // conn_interval_min/max: 12 unidades = 12 × 1.25ms = 15ms
                // slave_latency: 0 (sin saltar eventos)
                // supervision_timeout: 200 unidades = 200 × 10ms = 2000ms
                gap_request_connection_parameter_update(ble_con_handle, 12, 12, 0, 200);
                
                // NO solicitar envío aún - esperar MTU exchange
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            ble_con_handle = HCI_CON_HANDLE_INVALID;
            ble_tx_reset();
            txq_reset();  // Limpiar cola CAN_SEND_NOW
            ble_chunk_max = 0;
            gap_advertisements_enable(1);
            ble_advertising = true;
            last_adv_restart = to_ms_since_boot(get_absolute_time());
            break;
        case ATT_EVENT_MTU_EXCHANGE_COMPLETE: {
            // Capturar MTU negociado
            uint16_t mtu = att_event_mtu_exchange_complete_get_MTU(packet);
            on_mtu_change(mtu);
            ble_chunk_max = mtu - 3;
            
            // AHORA SÍ iniciar flujo de envío (MTU listo)
            if (txq_has_data()) {
                att_server_request_can_send_now_event(ble_con_handle);
            }
            break;
        }
        case ATT_EVENT_CAN_SEND_NOW: {
            // Handler principal para CAN_SEND_NOW
            uint8_t *ptr;
            int n = txq_pop(&ptr);
            if (n > 0) {
                att_server_notify(ble_con_handle, att_handle_tx, ptr, n);
                txq_consume();
                ble_packets_sent++;
                ble_bytes_sent += n;
                
                // Si quedan datos, solicitar otro evento inmediatamente
                if (txq_has_data()) {
                    att_server_request_can_send_now_event(ble_con_handle);
                }
            }
            break;
        }
        default:
            break;
    }
}

// ===== API Pública: Enviar bytes =====
void ble_send_bytes(const uint8_t *data, uint16_t len) {
    if (ble_con_handle == HCI_CON_HANDLE_INVALID || len == 0) return;
    
    // Optimización: supertramas nunca exceden 72 bytes, evitar segmentación innecesaria
    uint16_t max_payload = g_att_mtu - 3;
    if (max_payload < 20) max_payload = 20;
    
    if (len <= max_payload) {
        // Caso común: paquete cabe en un solo chunk (fast path)
        txq_push(data, len);
    } else {
        // Caso raro: segmentar (solo si MTU es muy pequeño o datos grandes)
        while (len > 0) {
            uint16_t chunk_len = (len > max_payload) ? max_payload : len;
            txq_push(data, chunk_len);
            data += chunk_len;
            len -= chunk_len;
        }
    }
}

// ===== API Pública: Notificación ATT =====
void ble_notify_att_payload(const uint8_t *data, size_t len) {
    if (ble_con_handle == HCI_CON_HANDLE_INVALID || len == 0) return;
    
    // ESPERAR MTU: No enviar hasta que MTU esté negociado
    if (ble_chunk_max == 0) {
        ble_skipped_samples++;
        return;
    }
    
    // Enviar directamente - la cola txq[] maneja drops automáticamente
    ble_send_bytes(data, (uint16_t)len);
}

// ===== API Pública: Registrar handlers BLE =====
void ble_stack_register_handlers(void) {
    att_server_register_packet_handler(&ble_packet_handler);
    ble_hci_cb.callback = &ble_packet_handler;
    hci_add_event_handler(&ble_hci_cb);
}

// ===== API Pública: Inicializar spin lock =====
void ble_init_lock(spin_lock_t *lock) {
    ble_tx_lock = lock;
}
