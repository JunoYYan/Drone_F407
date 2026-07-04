#include "MPU6050.hpp"

MPU6050::MPU6050(I2C_HandleTypeDef *hi2c)
{
    this->hi2c = hi2c;
}


bool MPU6050::writeReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(hi2c, addr, reg,
        I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK;
}


bool MPU6050::readRegs(uint8_t reg, uint8_t* data, uint16_t len)
{
    return HAL_I2C_Mem_Read(
        hi2c,
        addr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        100
    ) == HAL_OK;
}


bool MPU6050::init()
{

    // 1. 检查设备是否存在
    if (HAL_I2C_IsDeviceReady(hi2c, addr, 1, 100) != HAL_OK)
        return false;

    // 2. 唤醒 MPU6050，寄存器PWR_MGMT_1，默认上电bit6是1也就是sleep，需要写入0
    writeReg(0x6B, 0x00);

    // 3. gyro ±250 dps
    writeReg(0x1B, 0x00);

    // 4. accel ±2g
    writeReg(0x1C, 0x00);

    return true;
}


bool MPU6050::calibrate(uint16_t samples, uint32_t delay_ms)
{
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    // 防止重复校准时旧 bias 影响新结果
    ax_bias = 0;
    ay_bias = 0;
    az_bias = 0;
    gx_bias = 0;
    gy_bias = 0;
    gz_bias = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        uint8_t data[14];

        if (!readRegs(0x3B, data, 14))
            return false;

        int16_t ax_raw = (int16_t)((data[0] << 8) | data[1]);
        int16_t ay_raw = (int16_t)((data[2] << 8) | data[3]);
        int16_t az_raw = (int16_t)((data[4] << 8) | data[5]);

        int16_t gx_raw = (int16_t)((data[8] << 8) | data[9]);
        int16_t gy_raw = (int16_t)((data[10] << 8) | data[11]);
        int16_t gz_raw = (int16_t)((data[12] << 8) | data[13]);

        sum_ax += ax_raw / 16384.0f;
        sum_ay += ay_raw / 16384.0f;
        sum_az += az_raw / 16384.0f;

        sum_gx += gx_raw / 131.0f;
        sum_gy += gy_raw / 131.0f;
        sum_gz += gz_raw / 131.0f;

        HAL_Delay(delay_ms);
    }

    float avg_ax = sum_ax / samples;
    float avg_ay = sum_ay / samples;
    float avg_az = sum_az / samples;

    float avg_gx = sum_gx / samples;
    float avg_gy = sum_gy / samples;
    float avg_gz = sum_gz / samples;

    // 水平静止时，理论上 ax=0, ay=0, az=1g
    ax_bias = avg_ax;
    ay_bias = avg_ay;
    az_bias = avg_az - 1.0f;

    // 静止时，理论上 gyro 全部为 0 deg/s
    gx_bias = avg_gx;
    gy_bias = avg_gy;
    gz_bias = avg_gz;

    return true;
}


bool MPU6050::update()
{
    uint8_t data[14];

    if (!readRegs(0x3B, data, 14))
        return false;

    int16_t ax_raw = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay_raw = (int16_t)((data[2] << 8) | data[3]);
    int16_t az_raw = (int16_t)((data[4] << 8) | data[5]);

    int16_t gx_raw = (int16_t)((data[8] << 8) | data[9]);
    int16_t gy_raw = (int16_t)((data[10] << 8) | data[11]);
    int16_t gz_raw = (int16_t)((data[12] << 8) | data[13]);

    ax = ax_raw / 16384.0f - ax_bias;
    ay = ay_raw / 16384.0f - ay_bias;
    az = az_raw / 16384.0f - az_bias;

    gx = gx_raw / 131.0f - gx_bias;
    gy = gy_raw / 131.0f - gy_bias;
    gz = gz_raw / 131.0f - gz_bias;

    return true;
}