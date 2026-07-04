#pragma once

#include "stm32f4xx_hal.h"
#include "MPU6050.hpp"
#include "GPS.hpp"
#include "Actuators/Motors/Motors.hpp"

class FlightController {
public:
    enum class State {
        Init,
        Calibrating,
        Idle,
        Armed,
        Error,
        SafeStop
    };

    FlightController(
        I2C_HandleTypeDef *hi2c,
        TIM_HandleTypeDef* htim_motor
    );

    void init();
    void update();

private:
    MPU6050 imu_;
    GPS gps_;
    Motors motors_;


    State state_ = State::Init;

    void handleInit();
    void handleCalibrating();
    void handleIdle();
    void handleArmed();
    void handleSafeStop();
};