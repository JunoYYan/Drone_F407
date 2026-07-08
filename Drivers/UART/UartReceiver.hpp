#pragma once

#include "stm32f4xx_hal.h"


class UartReceiver
{

public:

    UartReceiver(
        UART_HandleTypeDef* huart
    );

    void update();

    bool consumeArmCommand();
    bool consumeDisarmCommand();
    uint16_t getThrottle();


private:

    UART_HandleTypeDef* huart_;

    char buffer_[64];
    uint8_t index_;
    uint16_t throttle_;

    bool armCommand_;
    bool disarmCommand_;

};