/**
 * @file traffic_queue.h
 * @brief Circular queue implementation for vehicle management at intersection
 * 
 * 11.02.26, Paweł Bolek
 */

#ifndef TRAFFIC_QUEUE_H
#define TRAFFIC_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * @def MAX_VEHICLES_PER_ROAD
 * @brief Maximum number of vehicles that can wait in a single lane
 */
#define MAX_VEHICLES_PER_ROAD 50

/**
 * @def VEHICLE_ID_LEN
 * @brief Maximum length of vehicle identifier strings (including null terminator)
 */
#define VEHICLE_ID_LEN 32

/**
 * @enum Direction
 * @brief Cardinal directions representing roads approaching the intersection
 * 
 * Order matters for turn calculations: (end - start) % 4 determines turn direction
 * - 1 = left turn
 * - 2 = straight
 * - 3 = right turn
 */
typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
} Direction;

/**
 * @struct Vehicle
 * @brief Represents a single vehicle in the queue
 */
typedef struct {
    char id[VEHICLE_ID_LEN]; /* Unique vehicle identifier (null-terminated) */
    uint8_t start_road; /* Entry road (Direction as uint8) */
    uint8_t end_road; /* Exit road (Direction as uint8) */
    uint32_t arrival_step; /* Simulation step when vehicle arrived */
} Vehicle;

/**
 * @struct VehicleQueue
 * @brief Circular queue implementation for vehicles in a single lane
 * 
 * @note The queue is full when count == MAX_VEHICLES_PER_ROAD
 * @note The queue is empty when count == 0
 */
typedef struct {
    Vehicle vehicles[MAX_VEHICLES_PER_ROAD];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    uint32_t max_wait_time;
} VehicleQueue;

/**
 * @brief Initialize a vehicle queue to empty state
 * 
 * Sets all fields to zero and prepares queue for use.
 * Must be called before any other queue operations.
 * 
 * @param q Pointer to VehicleQueue structure (must not be NULL)
 */
void queue_init(VehicleQueue* q);

/**
 * @brief Add a vehicle to the end of the queue
 * 
 * Copies vehicle data into the queue. If the queue is full,
 * the operation fails and returns false.
 * 
 * @param q Pointer to initialized VehicleQueue
 * @param id Vehicle identifier string (will be truncated if too long)
 * @param start Entry road (direction)
 * @param end Exit road (destination)
 * @param arrival_step Simulation step when vehicle arrived
 * 
 * @return true if vehicle was added successfully, false if queue is full
 */
bool queue_enqueue(VehicleQueue* q, const char* id, 
                   Direction start, Direction end, 
                   uint32_t arrival_step);

/**
 * @brief Remove and retrieve the front vehicle from the queue
 * 
 * @param q Pointer to VehicleQueue
 * @param out_id Buffer to store vehicle ID (can be NULL if not needed)
 * @param current_step Current simulation step (for wait time calculation)
 * @param wait_time Pointer to store calculated wait time (can be NULL)
 * 
 * @return true if vehicle was removed successfully, false if queue is empty
 */
bool queue_dequeue(VehicleQueue* q, char* out_id, 
                   uint32_t current_step, uint32_t* wait_time);

/**
 * @brief View the front vehicle without removing it
 * 
 * Copies the front vehicle data to the output structure.
 * Useful for checking which vehicle is next without modifying the queue.
 * 
 * @param q Pointer to VehicleQueue
 * @param out Pointer to Vehicle structure to receive the data
 * 
 * @return true if vehicle was peeked successfully, false if queue is empty
 */
bool queue_peek(const VehicleQueue* q, Vehicle* out);

/**
 * @brief Check if the queue has no vehicles
 * 
 * @param q Pointer to VehicleQueue
 * 
 * @return true if queue is empty (count == 0), false otherwise
 */
bool queue_is_empty(const VehicleQueue* q);

/**
 * @brief Check if the queue has reached maximum capacity
 * 
 * @param q Pointer to VehicleQueue
 * 
 * @return true if queue is full (count == MAX_VEHICLES_PER_ROAD), false otherwise
 */
bool queue_is_full(const VehicleQueue* q);

/**
 * @brief Get current number of vehicles in the queue
 * 
 * @param q Pointer to VehicleQueue
 * @return Number of vehicles waiting (0 to MAX_VEHICLES_PER_ROAD)
 */
uint16_t queue_count(const VehicleQueue* q);

/**
 * @brief Get the maximum observed wait time for this queue
 * 
 * @param q Pointer to VehicleQueue
 * @return Maximum wait time in simulation steps observed so far
 */
static inline uint32_t queue_get_max_wait(const VehicleQueue* q) {
    return q ? q->max_wait_time : 0;
}

#endif // TRAFFIC_QUEUE_H