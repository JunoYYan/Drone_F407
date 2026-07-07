#include "PID.hpp"



PID::PID(
    float kp,
    float ki,
    float kd,
    float output_limit
) :
    kp_(kp),
    ki_(ki),
    kd_(kd),
    integral_(0.0f),
    previousError_(0.0f),
    output_limit_(output_limit),
    integral_limit_(output_limit)
{}


float PID::update(
    float target,
    float current,
    float dt
)
{

    float error = target - current;


    // P term
    float P = kp_ * error;

    // I term
    integral_ += error * dt;

    if(integral_ > integral_limit_)
        integral_ = integral_limit_;

    if(integral_ < -integral_limit_)
        integral_ = -integral_limit_;

    float I = ki_ * integral_;

    // D term
    // 这里先保留，后面 rate PID 会换成 gyro feedback
    float derivative = (error - previousError_) / dt;
    float D = kd_ * derivative;


    previousError_ = error;

    float output = P + I + D;

    if(output > output_limit_)
        output = output_limit_;

    if(output < -output_limit_)
        output = -output_limit_;


    return output;

}


float PID::updateRate(
    float targetRate,
    float gyroRate,
    float dt
)
{
    float error = targetRate - gyroRate;


    // P
    float P = kp_ * error;

    // I
    integral_ += error * dt;

    if(integral_ > integral_limit_)
        integral_ = integral_limit_;

    if(integral_ < -integral_limit_)
        integral_ = -integral_limit_;

    float I = ki_ * integral_;


    // D， 四旋翼rate PID直接使用gyro阻尼
    float D = -kd_ * gyroRate;

    // 输出
    float output = P + I + D;

    if(output > output_limit_)
        output = output_limit_;

    if(output < -output_limit_)
        output = -output_limit_;

    return output;
}


void PID::reset()
{
    integral_ = 0.0f;

    previousError_ = 0.0f;
}