#pragma once

#include <stdint.h>


struct MotorOutput
{
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
};


class Mixer
{

public:

    MotorOutput mix(
        uint16_t throttle,
        float pitch,
        float roll,
        float yaw
    );

};