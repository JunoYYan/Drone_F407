#include "Mixer.hpp"


MotorOutput Mixer::mix(
    uint16_t throttle,
    float pitch,
    float roll,
    float yaw
)
{
    MotorOutput output;

    output.m1 =
        throttle
        + pitch
        - roll
        + yaw;


    output.m2 =
        throttle
        + pitch
        + roll
        - yaw;


    output.m3 =
        throttle
        - pitch
        + roll
        + yaw;


    output.m4 =
        throttle
        - pitch
        - roll
        - yaw;


    return output;
}