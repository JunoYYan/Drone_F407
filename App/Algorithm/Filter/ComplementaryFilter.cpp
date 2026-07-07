#include "ComplementaryFilter.hpp"
#include <cmath>

// filter的本质就是acc和gyro的互补滤波

#define PI 3.14159265358979323846f


ComplementaryFilter::ComplementaryFilter(float alpha)
    :
    alpha_(alpha),
    pitch_(0),
    roll_(0)
{}


void ComplementaryFilter::update(
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float dt
)
{

    // gyro integration
    pitch_ += gy * dt;
    roll_ += gx * dt;


    // accelerometer angle
    float pitch_acc = atan2(ax, az) * 180.0f / PI;
    float roll_acc = atan2(ay, az) * 180.0f / PI;


    // complementary filter
    pitch_ = alpha_ * pitch_ + (1-alpha_) * pitch_acc;
    roll_ = alpha_ * roll_ + (1-alpha_) * roll_acc;

}


float ComplementaryFilter::getPitch() const
{
    return pitch_;
}


float ComplementaryFilter::getRoll() const
{
    return roll_;
}


void ComplementaryFilter::reset(float pitch, float roll)
{
    pitch_ = pitch;
    roll_ = roll;
}