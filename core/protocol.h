#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include "traffic_queue.h"

typedef enum {
    CMD_ADD_VEHICLE = 1,
    CMD_STEP = 2,
    CMD_STOP = 99
} CommandType;

typedef struct __attribute__((packed)) {
    uint8_t cmd_type;
} CmdHeader;

typedef struct __attribute__((packed)) {
    char vehicle_id[VEHICLE_ID_LEN];
    uint8_t start_road; // Direction (0-3)
    uint8_t end_road; // Direction (0-3)
    uint32_t arrival_time;
} PayloadAddVehicle;

typedef struct __attribute__((packed)) {
    uint32_t current_step;
    uint16_t vehicles_left_count;
} ResponseStep;

#endif