// btstack_config.h
#pragma once

// --- BLE roles ---
#ifndef ENABLE_BLE
#define ENABLE_BLE
#endif
#ifndef ENABLE_LE_PERIPHERAL
#define ENABLE_LE_PERIPHERAL
#endif

// --- Conexiones / buffers ---
#define MAX_NR_LE_PERIPHERAL_CONNECTIONS   1
#define MAX_NR_LE_CENTRAL_CONNECTIONS      0
#define MAX_NR_HCI_CONNECTIONS            (MAX_NR_LE_PERIPHERAL_CONNECTIONS + MAX_NR_LE_CENTRAL_CONNECTIONS)

#define HCI_OUTGOING_PRE_BUFFER_SIZE       4

// --- Packet sizes / MTU ---
#define HCI_ACL_PAYLOAD_SIZE               251   // LE Data Length Extension (PDU)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT       4     // Requerido por CYW43

#define ATT_MAX_MTU                        185   // negotiated ATT MTU (usable: 244 bytes)

// --- ATT / GATT resources ---
#define ATT_MAX_ATT_DB_SIZE                1024  // Tamaño de base de datos de atributos
#define MAX_NR_GATT_CLIENTS                1
#define MAX_NR_L2CAP_SERVICES              2
#define MAX_NR_L2CAP_CHANNELS              2

// --- Notificaciones ---
#define MAX_NR_GATT_SUBSCRIPTIONS          4

// --- Security Manager / bonding ---
#define MAX_NR_LE_DEVICE_DB_ENTRIES        4
#define NVM_NUM_DEVICE_DB_ENTRIES          MAX_NR_LE_DEVICE_DB_ENTRIES

// --- Malloc support ---
#define HAVE_MALLOC                        1

// --- Embedded time support ---
#ifndef HAVE_EMBEDDED_TIME_MS
#define HAVE_EMBEDDED_TIME_MS              1
#endif

// --- HCI dump (stdout) ---
#ifndef ENABLE_PRINTF_HEXDUMP
#define ENABLE_PRINTF_HEXDUMP
#endif

// --- Logging opcional (descomentar para debug) ---
// #define ENABLE_LOG_INFO
// #define ENABLE_LOG_ERROR
