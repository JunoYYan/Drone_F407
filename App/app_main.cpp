#include "app_main.hpp"
#include "FlightController.hpp"
#include "Actuators/Motors/Motors.hpp"

extern "C" {
#include "i2c.h"
#include "tim.h"
#include "usart.h"
}

static FlightController controller(
    &hi2c1,
    &htim2,
    &huart1
);

void App_Init(void)
{
    controller.init();
}

void App_Loop(void)
{
    controller.update();
}