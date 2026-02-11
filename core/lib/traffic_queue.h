#ifndef TRAFFIC_QUEUE_H
#define TRAFFIC_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MAX_VEHICLES_PER_ROAD 50
#define VEHICLE_ID_LEN 32

typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
} Direction;

typedef struct {
    char id[VEHICLE_ID_LEN];
    uint8_t start_road;
    uint8_t end_road;
    uint32_t arrival_step;
} Vehicle;

typedef struct {
    Vehicle vehicles[MAX_VEHICLES_PER_ROAD];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    uint32_t max_wait_time;
} VehicleQueue;

void queue_init(VehicleQueue* q);
bool queue_enqueue(VehicleQueue* q, const char* id, Direction start, Direction end, uint32_t arrival_step);
bool queue_dequeue(VehicleQueue* q, char* out_id, uint32_t current_step, uint32_t* wait_time);
bool queue_peek(const VehicleQueue* q, Vehicle* out);
bool queue_is_empty(const VehicleQueue* q);
bool queue_is_full(const VehicleQueue* q);
uint16_t queue_count(const VehicleQueue* q) ;

#endif