/**
 * @file protocol.h
 * @brief Binary communication protocol definitions for the Traffic FSM.
 * 
 * All multi-byte integers are expected to be transmitted in Little-Endian format.
 * Structures are strictly packed to prevent compiler-specific memory padding.
 * 
 * 11.02.26, Paweł Bolek
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include "traffic_queue.h"

/**
 * @brief Supported command opcodes sent from the Host to the MCU/Core.
 */
typedef enum {
    CMD_CONFIG = 0,
    CMD_ADD_VEHICLE = 1,
    CMD_STEP = 2,
    CMD_STOP = 99
} CommandType;

/**
 * @brief Universal 1-byte header preceding every incoming payload.
 */
typedef struct __attribute__((packed)) {
    uint8_t cmd_type;
} CmdHeader;

/**
 * @brief Payload for CMD_CONFIG (28 bytes).
 * Defines the operational bounds
 */
typedef struct __attribute__((packed)) {
    uint32_t green_st;
    uint32_t green_lt;
    uint32_t yellow;
    uint32_t all_red;
    uint32_t ext_threshold;
    uint32_t max_ext;
    uint32_t skip_limit;
} PayloadConfig;

/**
 * @brief Payload for CMD_ADD_VEHICLE (38 bytes).
 */
typedef struct __attribute__((packed)) {
    char vehicle_id[VEHICLE_ID_LEN];
    uint8_t start_road; // Origin direction (0=N, 1=E, 2=S, 3=W)
    uint8_t end_road; // Destination direction (0=N, 1=E, 2=S, 3=W)
    uint32_t arrival_time; // Timestamp of vehicle appearance
} PayloadAddVehicle;

/**
 * @brief Response sent from Core to Host after CMD_STEP (11 bytes).
 * * Note: If vehicles_out > 0, this struct is immediately followed by 
 * an array of (vehicles_out * VEHICLE_ID_LEN) bytes containing the IDs 
 * of the departing vehicles.
 */
typedef struct __attribute__((packed)) {
    uint32_t current_step;
    uint8_t current_state;
    
    uint8_t light_ns_st;
    uint8_t light_ns_lt;
    uint8_t light_ew_st;
    uint8_t light_ew_lt;
    
    uint16_t vehicles_out;      
} ResponseStep;

#endif