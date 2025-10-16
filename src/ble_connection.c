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
// Usar BLE_TX_BUF_SIZE desde shared_types.h
// Única cola basada en CAN_SEND_NOW (BTstack)
#define TXQ_MAX 256           // Cola de supertramas para CAN_SEND_NOW (aumentado para minimizar drops)

// ===== Variables BLE Globales =====
uint16_t ble_con_handle = HCI_CON_HANDLE_INVALID;  // Expuesta para main.c
static btstack_packet_callback_registration_t ble_hci_cb;

// --- Spinlock para sincronización (puede usarse en el futuro si es necesario) ---
static spin_lock_t *ble_tx_lock;

// --- Estado de advertising + chunk dinámico ---
volatile bool ble_advertising = false;  // Expuesta para main.c
volatile uint32_t last_adv_restart = 0;  // Expuesta para main.c
static volatile uint16_t ble_chunk_max = 0;  // MTU negociado

// --- Cola CAN_SEND_NOW (patrón BTstack oficial) ---
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

// ===== Funciones para cola CAN_SEND_NOW =====

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

// ===== API Pública: Verificar espacio (métricas proporcionales) =====
// Simula "bytes libres" basándose en el número de slots libres de txq[]
// para conservar la semántica esperada por main.c sin el buffer circular legacy.
uint16_t ble_tx_free_space(void) {
    uint16_t head = txq_head;
    uint16_t tail = txq_tail;
    uint16_t used = (head >= tail) ? (head - tail) : (TXQ_MAX - (tail - head));
    uint16_t free_slots = (TXQ_MAX - 1) - used;
    // Mapear linealmente a un "capacidad virtual" de BLE_TX_BUF_SIZE
    // Devuelve 0..(BLE_TX_BUF_SIZE - step), con step ~ BLE_TX_BUF_SIZE/(TXQ_MAX-1)
    if (free_slots == 0) return 0;
    uint32_t virtual_bytes = ((uint32_t)BLE_TX_BUF_SIZE * free_slots) / (TXQ_MAX - 1);
    if (virtual_bytes > 0xFFFF) virtual_bytes = 0xFFFF;
    return (uint16_t)virtual_bytes;
}

// ===== Vaciar cola respetando ATT CAN_SEND_NOW =====
void ble_try_flush(void) {
    // Único camino: solicitar evento CAN_SEND_NOW si hay datos y hay conexión
    if (ble_con_handle == HCI_CON_HANDLE_INVALID) {
        return;
    }
    if (txq_has_data()) {
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
