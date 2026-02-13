#include "TrafficLights_Main.h"
#include "traffic_fsm.h"
#include "protocol.h"
#include <string.h>

extern UART_HandleTypeDef huart2; 
#define COMM_UART &huart2

const RoadLeds_t North = {{LED_N_RED_GPIO_Port, LED_N_RED_Pin}, {LED_N_YELLOW_GPIO_Port, LED_N_YELLOW_Pin}, {LED_N_GREEN_GPIO_Port, LED_N_GREEN_Pin}, {LED_N_FIRST_GPIO_Port, LED_N_FIRST_Pin}, {LED_N_SECOND_GPIO_Port, LED_N_SECOND_Pin}, 0};
const RoadLeds_t South = {{LED_S_RED_GPIO_Port, LED_S_RED_Pin}, {LED_S_YELLOW_GPIO_Port, LED_S_YELLOW_Pin}, {LED_S_GREEN_GPIO_Port, LED_S_GREEN_Pin}, {LED_S_FIRST_GPIO_Port, LED_S_FIRST_Pin}, {LED_S_SECOND_GPIO_Port, LED_S_SECOND_Pin}, 0};
const RoadLeds_t East  = {{LED_E_RED_GPIO_Port, LED_E_RED_Pin}, {LED_E_YELLOW_GPIO_Port, LED_E_YELLOW_Pin}, {LED_E_GREEN_GPIO_Port, LED_E_GREEN_Pin}, {LED_E_FIRST_GPIO_Port, LED_E_FIRST_Pin}, {LED_E_SECOND_GPIO_Port, LED_E_SECOND_Pin}, 0};
const RoadLeds_t West  = {{LED_W_RED_GPIO_Port, LED_W_RED_Pin}, {LED_W_YELLOW_GPIO_Port, LED_W_YELLOW_Pin}, {LED_W_GREEN_GPIO_Port, LED_W_GREEN_Pin}, {LED_W_FIRST_GPIO_Port, LED_W_FIRST_Pin}, {LED_W_SECOND_GPIO_Port, LED_W_SECOND_Pin}, 0};

TrafficSystem sys;

void Led_Set(Led_t led, GPIO_PinState state) {
    HAL_GPIO_WritePin(led.port, led.pin, state);
}

void Road_Off(const RoadLeds_t* road) {
    Led_Set(road->red, GPIO_PIN_RESET);
    Led_Set(road->yellow, GPIO_PIN_RESET);
    Led_Set(road->green, GPIO_PIN_RESET);
    Led_Set(road->q_first, GPIO_PIN_RESET);
    Led_Set(road->q_second, GPIO_PIN_RESET);
}

static void Apply_LightColor_To_Physical(const RoadLeds_t* road, LightColor color) {
    Led_Set(road->red,    (color == LIGHT_RED || color == LIGHT_RED_YELLOW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Led_Set(road->yellow, (color == LIGHT_YELLOW || color == LIGHT_RED_YELLOW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    Led_Set(road->green,  (color == LIGHT_GREEN || color == LIGHT_RIGHT_ARROW_GREEN) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Update_Hardware_From_FSM(void) {
    Apply_LightColor_To_Physical(&North, sys.lights[NORTH][LANE_STRAIGHT_RIGHT]);
    Apply_LightColor_To_Physical(&East,  sys.lights[EAST][LANE_STRAIGHT_RIGHT]);
    Apply_LightColor_To_Physical(&South, sys.lights[SOUTH][LANE_STRAIGHT_RIGHT]);
    Apply_LightColor_To_Physical(&West,  sys.lights[WEST][LANE_STRAIGHT_RIGHT]);

    uint16_t qN = traffic_get_queue_size(&sys, NORTH, LANE_STRAIGHT_RIGHT) + traffic_get_queue_size(&sys, NORTH, LANE_LEFT);
    Led_Set(North.q_first,  (qN >= 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Led_Set(North.q_second, (qN >= 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    uint16_t qE = traffic_get_queue_size(&sys, EAST, LANE_STRAIGHT_RIGHT) + traffic_get_queue_size(&sys, EAST, LANE_LEFT);
    Led_Set(East.q_first,   (qE >= 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Led_Set(East.q_second,  (qE >= 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    uint16_t qS = traffic_get_queue_size(&sys, SOUTH, LANE_STRAIGHT_RIGHT) + traffic_get_queue_size(&sys, SOUTH, LANE_LEFT);
    Led_Set(South.q_first,  (qS >= 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Led_Set(South.q_second, (qS >= 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    uint16_t qW = traffic_get_queue_size(&sys, WEST, LANE_STRAIGHT_RIGHT) + traffic_get_queue_size(&sys, WEST, LANE_LEFT);
    Led_Set(West.q_first,   (qW >= 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Led_Set(West.q_second,  (qW >= 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Traffic_Lights_Init(void) {
    Road_Off(&North); Road_Off(&South); Road_Off(&East); Road_Off(&West);
    
    TimingConfig default_config = DEFAULT_TIMING;
    traffic_init(&sys, default_config);
    Update_Hardware_From_FSM();
}

void TrafficLights_Main(void) {
    CmdHeader header;
    char discharged_ids[ROAD_COUNT * LANES_PER_ROAD][VEHICLE_ID_LEN];

    if (HAL_UART_Receive(COMM_UART, (uint8_t*)&header, sizeof(CmdHeader), HAL_MAX_DELAY) == HAL_OK) {
        if (header.cmd_type == CMD_CONFIG) {
            PayloadConfig payload;
            if (HAL_UART_Receive(COMM_UART, (uint8_t*)&payload, sizeof(PayloadConfig), 1000) == HAL_OK) {
                TimingConfig config = {
                    .green_st = payload.green_st, .green_lt = payload.green_lt,
                    .yellow = payload.yellow, .all_red = payload.all_red,
                    .ext_threshold = payload.ext_threshold, .max_ext = payload.max_ext,
                    .skip_limit = payload.skip_limit
                };
                traffic_init(&sys, config);
                Update_Hardware_From_FSM();
            }
        } 
        
        else if (header.cmd_type == CMD_ADD_VEHICLE) {
            PayloadAddVehicle payload;
            if (HAL_UART_Receive(COMM_UART, (uint8_t*)&payload, sizeof(PayloadAddVehicle), 1000) == HAL_OK) {
                payload.vehicle_id[VEHICLE_ID_LEN - 1] = '\0';
                traffic_add_vehicle(&sys, payload.vehicle_id, payload.start_road, payload.end_road, payload.arrival_time);
                Update_Hardware_From_FSM();
            }
        } 

        else if (header.cmd_type == CMD_STEP) {
            memset(discharged_ids, 0, sizeof(discharged_ids));
            int count = traffic_fsm_step(&sys, discharged_ids);
            Update_Hardware_From_FSM();

            ResponseStep resp;
            resp.current_step = sys.current_step;
            resp.current_state = (uint8_t)sys.current_state;
            resp.light_ns_st = (uint8_t)sys.lights[NORTH][LANE_STRAIGHT_RIGHT];
            resp.light_ns_lt = (uint8_t)sys.lights[NORTH][LANE_LEFT];
            resp.light_ew_st = (uint8_t)sys.lights[EAST][LANE_STRAIGHT_RIGHT];
            resp.light_ew_lt = (uint8_t)sys.lights[EAST][LANE_LEFT];
            resp.vehicles_out = (uint16_t)count;

            HAL_UART_Transmit(COMM_UART, (uint8_t*)&resp, sizeof(ResponseStep), 1000);

            if (count > 0) {
                HAL_UART_Transmit(COMM_UART, (uint8_t*)discharged_ids, VEHICLE_ID_LEN * count, 1000);
            }
        }
    }
}