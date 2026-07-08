#include "FlightController.hpp"
#include <cstdio>
#include <cmath>


FlightController::FlightController(
    I2C_HandleTypeDef* hi2c,
    TIM_HandleTypeDef* htim_motor,
    UART_HandleTypeDef* huart
) : 
    imu_(hi2c),
    motors_(htim_motor),
    receiver_(huart),
    mixer_(),
    attitude_(0.98f),

    pitchAnglePID_(
        8.0f,
        0,
        0,
        200
    ),

    rollAnglePID_(
        8.0f,
        0,
        0,
        200
    ),


    pitchRatePID_(
        0.5f,
        0,
        0,
        500
    ),

    rollRatePID_(
        0.5f,
        0,
        0,
        500
    ),

    throttle_(1000)

{}


void FlightController::init()
{
    // gps_.init();

    motors_.init();
    motors_.stop();
    HAL_Delay(5000);

    if(!imu_.init())
    {
        printf("MPU6050 init failed!\r\n");
        state_ = State::Error;
        return;
    }

    printf("MPU6050 init success!\r\n");

}


void FlightController::update()
{
    receiver_.update();

    if(receiver_.consumeDisarmCommand())
    {
        printf("DISARM received!\r\n");

        throttle_ = 1000;
        motors_.stop();
        state_ = State::Disarmed;
    }

    if(receiver_.consumeArmCommand())
    {
        printf("ARM received!\r\n");
        state_ = State::Armed;
    }


    switch (state_) {
        case State::Init:
            handleInit();
            break;

        case State::Calibrating:
            handleCalibrating();
            break;

        case State::Disarmed:
            handleDisarmed();
            break;

        // case State::Idle:
        //     handleIdle();
        //     break;

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

    bool ok = imu_.calibrate(1000, 2); // 每隔2ms采样1000次

    if (!ok)
    {
        printf("IMU calibration failed!\r\n");
        state_ = State::Error;
        return;
    }

    imu_.update();

    float pitch0 = atan2(imu_.ax, imu_.az) * 180.0f / 3.1415926f;
    float roll0 = atan2(imu_.ay, imu_.az) * 180.0f / 3.1415926f;

    attitude_.reset(
        pitch0,
        roll0
    );

    printf("IMU calibration done.\r\n");
    // printf("ax: %.2f ay: %.2f az: %.2f gx: %.2f gy: %.2f gz: %.2f\r\n",
    //     imu_.ax,
    //     imu_.ay,
    //     imu_.az,
    //     imu_.gx,
    //     imu_.gy,
    //     imu_.gz);

    // state_ = State::Idle;
    state_ = State::Disarmed;
}


void FlightController::handleDisarmed()
{
    motors_.stop();

}


// void FlightController::handleIdle()
// {
//     motors_.stop();
//     imu_.update();
    
//     float dt = 0.01f;
//     attitude_.update(
//         imu_.ax,
//         imu_.ay,
//         imu_.az,
//         imu_.gx,
//         imu_.gy,
//         dt
//     ); // 不断更新最近的pitch和roll

//     printf(
//         "pitch %.2f roll %.2f\r\n",
//         attitude_.getPitch(),
//         attitude_.getRoll()
//     );
    
//     HAL_Delay(10);

//     state_=State::Armed;
// }


void FlightController::handleArmed()
{
    if(receiver_.consumeDisarmCommand())
    {
        printf("DISARM received in Armed state!\r\n");
        throttle_ = 1000;
        motors_.stop();
        state_ = State::Disarmed;
        return;
    }


    throttle_ = receiver_.getThrottle();

    imu_.update();

    float dt = 0.01f;

    attitude_.update(
        imu_.ax,
        imu_.ay,
        imu_.az,
        imu_.gx,
        imu_.gy,
        dt
    ); // 不断更新最近的pitch和roll

    float pitch = attitude_.getPitch();
    float roll = attitude_.getRoll();


    // Angle PID
    float targetPitchRate =
        pitchAnglePID_.update(
            0,
            pitch,
            dt
        );


    float targetRollRate =
        rollAnglePID_.update(
            0,
            roll,
            dt
        );


    // Rate PID
    float pitchCorrection =
        pitchRatePID_.updateRate(
            targetPitchRate,
            imu_.gy,
            dt
        );


    float rollCorrection =
        rollRatePID_.updateRate(
            targetRollRate,
            imu_.gx,
            dt
        );


    // Mixer
    MotorOutput motors =
        mixer_.mix(
            throttle_, // throttle
            pitchCorrection,
            rollCorrection,
            0
        );

    motors_.setMotorOutputs(motors);
    

    static uint32_t lastPrintTime = 0;

    if(HAL_GetTick() - lastPrintTime >= 500)
    {
        lastPrintTime = HAL_GetTick();

        printf(
            "M1:%d M2:%d M3:%d M4:%d throttle:%d\r\n",
            motors.m1,
            motors.m2,
            motors.m3,
            motors.m4,
            throttle_
        );
    }

}


void FlightController::handleSafeStop()
{
    motors_.stop();
    HAL_Delay(1000);
}

