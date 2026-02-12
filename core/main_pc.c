#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "protocol.h"
#include "traffic_fsm.h"

TrafficSystem sys;

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
    // TODO: traffic_fsm_step(); and rest of response 
    sys.current_step++; 

    ResponseStep resp;
    resp.current_step = sys.current_step;
    resp.vehicles_left_count = 0;

    fwrite(&resp, sizeof(ResponseStep), 1, stdout);
}

int main() {
    // No buffer on input and output
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);

    traffic_init(&sys);
    
    CmdHeader header;
    while (fread(&header, sizeof(CmdHeader), 1, stdin) == 1) {
        switch (header.cmd_type) {
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