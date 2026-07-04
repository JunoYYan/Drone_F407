#include "Motors.hpp"

Motors::Motors(TIM_HandleTypeDef* htim)
    : htim_(htim)
{
}

void Motors::init()
{
    HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_4);

    stop();
}

void Motors::setPulse(MotorId id, uint16_t pulse_us)
{
    uint8_t index = static_cast<uint8_t>(id);

    if (index >= 4)
    {
        return;
    }

    pulse_us = clampPulse(pulse_us);

    __HAL_TIM_SET_COMPARE(htim_, channels_[index], pulse_us);
}

void Motors::setAllPulse(uint16_t pulse_us)
{
    pulse_us = clampPulse(pulse_us);

    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, pulse_us);
    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, pulse_us);
    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, pulse_us);
    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_4, pulse_us);
}

void Motors::stop()
{
    setAllPulse(MIN_PULSE_US);
}

uint16_t Motors::clampPulse(uint16_t pulse_us)
{
    if (pulse_us < MIN_PULSE_US)
    {
        return MIN_PULSE_US;
    }

    if (pulse_us > MAX_PULSE_US)
    {
        return MAX_PULSE_US;
    }

    return pulse_us;
}