#pragma once
#include "stm32f4xx_hal.h"

class MPU6050 {
public:
    MPU6050(I2C_HandleTypeDef *hi2c);

    bool init();
    bool update();
    bool calibrate(uint16_t samples = 1000, uint32_t delay_ms = 2);

    float ax, ay, az;
    float gx, gy, gz;

private:
    I2C_HandleTypeDef *hi2c;

    uint8_t addr = 0x68 << 1;

    float ax_bias = 0, ay_bias = 0, az_bias = 0;
    float gx_bias = 0, gy_bias = 0, gz_bias = 0;

    bool writeReg(uint8_t reg, uint8_t data);
    bool readRegs(uint8_t reg, uint8_t* data, uint16_t len);
};