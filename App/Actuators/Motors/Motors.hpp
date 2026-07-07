#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "Algorithm/Mixer/Mixer.hpp"

enum class MotorId : uint8_t
{
    M1 = 0,
    M2 = 1,
    M3 = 2,
    M4 = 3
};

class Motors
{
public:
    static constexpr uint16_t MIN_PULSE_US = 1000;
    static constexpr uint16_t MAX_PULSE_US = 2000;
    static constexpr uint16_t ARM_PULSE_US = 1000;

    explicit Motors(TIM_HandleTypeDef* htim);

    void init();

    void setPulse(MotorId id, uint16_t pulse_us);
    void setAllPulse(uint16_t pulse_us);

    void stop();

    void setMotorOutputs(
        const MotorOutput& output
    );

private:
    TIM_HandleTypeDef* htim_;

    static constexpr uint32_t channels_[4] = {
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        TIM_CHANNEL_3,
        TIM_CHANNEL_4
    };

    uint16_t clampPulse(uint16_t pulse_us);
};