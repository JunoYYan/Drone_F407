#pragma once

#include "stm32f4xx_hal.h"
#include "MPU6050.hpp"
#include "GPS.hpp"
#include "Actuators/Motors/Motors.hpp"
#include "Algorithm/PID/PID.hpp"
#include "Algorithm/Filter/ComplementaryFilter.hpp"
#include "Algorithm/Mixer/Mixer.hpp"
#include "UART/UartReceiver.hpp"

class FlightController {
public:
    enum class State {
        Init,
        Calibrating,
        // Idle,
        Disarmed,
        Armed,
        Error,
        SafeStop
    };

    FlightController(
        I2C_HandleTypeDef *hi2c,
        TIM_HandleTypeDef* htim_motor,
        UART_HandleTypeDef* huart
    );

    void init();
    void update();

private:
    MPU6050 imu_;
    GPS gps_;
    Motors motors_;

    ComplementaryFilter attitude_;

    PID pitchAnglePID_;
    PID rollAnglePID_;

    PID pitchRatePID_;
    PID rollRatePID_;

    Mixer mixer_;

    UartReceiver receiver_;
    uint16_t throttle_;

    State state_ = State::Init;

    void handleInit();
    void handleCalibrating();
    // void handleIdle();
    void handleDisarmed();
    void handleArmed();
    void handleSafeStop();


};