#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "protocol.h"
#include "traffic_fsm.h"

TrafficSystem sys;

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
        .all_red = payload.all_red
    };
    
    traffic_init(&sys, config);
    fprintf(stderr, "[C-OK] Config loaded: ST=%d, LT=%d, Y=%d, AR=%d\n",
            config.green_st, config.green_lt, config.yellow, config.all_red);
}

void handle_add_vehicle() {
    PayloadAddVehicle payload;
    size_t read_count = fread(&payload, sizeof(PayloadAddVehicle), 1, stdin);
    
    if (read_count != 1) {
        fprintf(stderr, "[C-ERR] Failed to read AddVehicle payload\n");
        return;
    }

    traffic_add_vehicle(&sys, payload.vehicle_id, payload.start_road, payload.end_road, payload.arrival_time);
}

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

int main() {
    // No buffer on input and output
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);

    TimingConfig default_config = DEFAULT_TIMING;
    traffic_init(&sys, default_config);

    CmdHeader header;
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