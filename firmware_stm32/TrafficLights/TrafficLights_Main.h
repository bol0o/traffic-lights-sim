#ifndef TRAFFICLIGHTS_MAIN_H
#define TRAFFICLIGHTS_MAIN_H

#include "main.h"
#include "traffic_fsm.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} Led_t;

typedef struct {
    Led_t red;
    Led_t yellow;
    Led_t green;
    Led_t q_first;
    Led_t q_second;
} RoadLeds_t;

void Traffic_Lights_Init(void);
void TrafficLights_Main(void);

void Led_Set(Led_t led, GPIO_PinState state);
void Road_Off(const RoadLeds_t* road);

#endif // TRAFFICLIGHTS_MAIN_H