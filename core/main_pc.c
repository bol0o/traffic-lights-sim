/**
 * @file main.c
 * 
 * @brief Hardware Abstraction Layer for traffic simulation.
 * 
 * * This module acts as the bridge between Python host
 * and the internal FSM. It listens on standard input for 
 * binary command frames, deserializes them, triggers the FSM logic, 
 * and serializes the responses back to standard output.
 * 
 * 11.02.26, Paweł Bolek
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "protocol.h"
#include "traffic_fsm.h"

TrafficSystem sys;

/**
 * @brief Handles CMD_CONFIG: Deserializes timing constraints and resets FSM.
 */
void handle_config() {
    PayloadConfig payload;
    size_t read_count = fread(&payload, sizeof(PayloadConfig), 1, stdin);
    
    if (read_count != 1) {
        fprintf(stderr, "[C-ERR] Failed to read Config payload\n");
        return;
    }

    TimingConfig config = {
        .green_st = payload.green_st,
        .green_lt = payload.green_lt,
        .yellow = payload.yellow,
        .all_red = payload.all_red,
        .red_yellow = 1,
        .ext_threshold = payload.ext_threshold,
        .max_ext = payload.max_ext,
        .skip_limit = payload.skip_limit
    };
    
    traffic_init(&sys, config);
    fprintf(stderr, "[C-OK] Config loaded: ST=%d, LT=%d, Y=%d, AR=%d TH=%d MAX=%d LIM=%d\n",
            config.green_st, config.green_lt, config.yellow, config.all_red, config.ext_threshold, 
            config.max_ext, config.skip_limit);
}

/**
 * @brief Handles CMD_ADD_VEHICLE: Pushes a new vehicle into the proper approach queue.
 */
void handle_add_vehicle() {
    PayloadAddVehicle payload;
    size_t read_count = fread(&payload, sizeof(PayloadAddVehicle), 1, stdin);
    
    if (read_count != 1) {
        fprintf(stderr, "[C-ERR] Failed to read AddVehicle payload\n");
        return;
    }

    bool success = traffic_add_vehicle(&sys, payload.vehicle_id, payload.start_road, payload.end_road, payload.arrival_time);
    if (!success) {
        fprintf(stderr, "[C-WARN] Failed to add vehicle %s (queue full or invalid direction)\n", 
                payload.vehicle_id);
    }
}

/**
 * @brief Handles CMD_STEP: Advances FSM by one tick and transmits hardware state.
 * * First sends the fixed 11-byte ResponseStep header. If any vehicles 
 * passed through the intersection during this step, their 32-byte string IDs 
 * are appended consecutively to the output stream.
 */
void handle_step() {
    char discharged_ids[ROAD_COUNT * LANES_PER_ROAD][VEHICLE_ID_LEN];
    memset(discharged_ids, 0, sizeof(discharged_ids));

    int count = traffic_fsm_step(&sys, discharged_ids);

    ResponseStep resp;
    resp.current_step = sys.current_step;
    resp.current_state = (uint8_t)sys.current_state;
    resp.light_ns_st = (uint8_t)sys.lights[NORTH][LANE_STRAIGHT_RIGHT];
    resp.light_ns_lt = (uint8_t)sys.lights[NORTH][LANE_LEFT];
    resp.light_ew_st = (uint8_t)sys.lights[EAST][LANE_STRAIGHT_RIGHT];
    resp.light_ew_lt = (uint8_t)sys.lights[EAST][LANE_LEFT];
    resp.vehicles_out = (uint16_t)count;

    fwrite(&resp, sizeof(ResponseStep), 1, stdout);

    if (count > 0) {
        fwrite(discharged_ids, VEHICLE_ID_LEN, count, stdout);
    }
    
    fflush(stdout);
}

/**
 * @brief Main execution loop.
 * Disables stream buffering to ensure smooth communication 
 * and prevent pipeline deadlocks with the Python wrapper. Operates in 
 * a blocking event loop reading from stdin.
 */
int main() {
    // Disable buffering on stdin/stdout to prevent deadlocks over OS pipes.
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);

    TimingConfig default_config = DEFAULT_TIMING;
    traffic_init(&sys, default_config);

    CmdHeader header;
    // Blocking read - waits for host command
    while (fread(&header, sizeof(CmdHeader), 1, stdin) == 1) {
        switch (header.cmd_type) {
            case CMD_CONFIG:
                handle_config();
                break;
                
            case CMD_ADD_VEHICLE:
                handle_add_vehicle();
                break;
            
            case CMD_STEP:
                handle_step();
                break;

            case CMD_STOP:
                return 0;

            default:
                fprintf(stderr, "[C-ERR] Unknown command: %d\n", header.cmd_type);
                break;
        }
    }

    return 0;
}