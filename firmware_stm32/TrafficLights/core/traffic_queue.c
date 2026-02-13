/**
 * @file traffic_queue.c
 * @brief Circular queue implementation for vehicle management at intersection
 * 
 * 11.02.26, Paweł Bolek
 */

#include "traffic_queue.h"

void queue_init(VehicleQueue* q) {
    if (!q) return;
    memset(q, 0, sizeof(VehicleQueue));
}

bool queue_enqueue(VehicleQueue* q, const char* id, Direction start, Direction end, uint32_t arrival_step) {
    if (!q || queue_is_full(q)) { return false; }

    Vehicle* vehicle = &q->vehicles[q->tail];

    strncpy(vehicle->id, id, VEHICLE_ID_LEN - 1);
    vehicle->id[VEHICLE_ID_LEN - 1] = '\0'; 
    vehicle->start_road = start;
    vehicle->end_road = end;
    vehicle->arrival_step = arrival_step;

    q->tail = (q->tail + 1) % MAX_VEHICLES_PER_ROAD;
    q->count++;

    return true;
}

bool queue_dequeue(VehicleQueue* q, char* out_id, uint32_t current_step, uint32_t* wait_time) {
    if (!q || queue_is_empty(q)) { return false; }

    Vehicle* v = &q->vehicles[q->head];

    if (out_id) {
        strncpy(out_id, v->id, VEHICLE_ID_LEN);
    }

    if (current_step >= v->arrival_step) {
        uint32_t calculated_wait = current_step - v->arrival_step;
        
        if (calculated_wait > q->max_wait_time) {
            q->max_wait_time = calculated_wait;
        }

        if (wait_time) {
            *wait_time = calculated_wait;
        }
    }
    
    q->head = (q->head + 1) % MAX_VEHICLES_PER_ROAD;
    q->count--;

    return true;
}

bool queue_peek(const VehicleQueue* q, Vehicle* out) {
    if (!q || queue_is_empty(q) || !out) { return false; }

    if (out) {
        *out = q->vehicles[q->head];
    }
    
    return true;
}

bool queue_is_empty(const VehicleQueue* q) {
    return q->count == 0;
}

bool queue_is_full(const VehicleQueue* q) {
    return q->count >= MAX_VEHICLES_PER_ROAD;
}

uint16_t queue_count(const VehicleQueue* q) {
    return q->count;
}