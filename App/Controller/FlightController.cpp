#include "FlightController.hpp"
#include <cstdio>

FlightController::FlightController(
    I2C_HandleTypeDef* hi2c,
    TIM_HandleTypeDef* htim_motor
)
    : imu_(hi2c),
      motors_(htim_motor)
{
}

void FlightController::init()
{
    motors_.init();
    motors_.stop();

    HAL_Delay(5000);

    imu_.init();
    gps_.init();

    state_ = State::Calibrating;
}

void FlightController::update()
{
    switch (state_) {

    case State::Init:
        handleInit();
        break;

    case State::Calibrating:
        handleCalibrating();
        break;

    case State::Idle:
        handleIdle();
        break;

    case State::Armed:
        handleArmed();
        break;

    case State::Error:
        break;
    
    case State::SafeStop:
        handleSafeStop();
        break;
    }
}

void FlightController::handleInit()
{
    state_ = State::Calibrating;
}

void FlightController::handleCalibrating()
{   
    motors_.stop();

    printf("IMU calibrating... Keep drone still and level.\r\n");

    bool ok = imu_.calibrate(1000, 2);

    if (!ok)
    {
        printf("IMU calibration failed!\r\n");
        state_ = State::Error;
        return;
    }

    imu_.update();

    printf("IMU calibration done.\r\n");
    printf("ax: %.2f ay: %.2f az: %.2f gx: %.2f gy: %.2f gz: %.2f\r\n",
        imu_.ax,
        imu_.ay,
        imu_.az,
        imu_.gx,
        imu_.gy,
        imu_.gz);

    state_ = State::Idle;
}

void FlightController::handleIdle()
{
    motors_.stop();
    imu_.update();
    printf("ax: %.2f ay: %.2f az: %.2f gx: %.2f gy: %.2f gz: %.2f\r\n",
        imu_.ax,
        imu_.ay,
        imu_.az,
        imu_.gx,
        imu_.gy,
        imu_.gz);
    
    HAL_Delay(1000);

    // state_ = State::Armed;

    // 这里未来加：
    // arm switch / RC input

    // 示例：
    // if (rc.arm == true)
    //     state_ = State::Armed;
}

void FlightController::handleArmed()
{
    imu_.update();

    for (uint16_t pulse = 1200; pulse <= 1300; pulse += 100)
    {
        motors_.setAllPulse(pulse);
        HAL_Delay(3000);
    }

    

    motors_.stop();

    state_ = State::SafeStop;

    // 飞控核心（以后加）
    // - attitude estimation
    // - PID control
    // - motor output
}

void FlightController::handleSafeStop()
{
    motors_.stop();
    HAL_Delay(1000);
}

