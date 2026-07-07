#pragma once


class PID
{

public:

    PID(
        float kp,
        float ki,
        float kd,
        float output_limit
    );


    float update(
        float target,
        float current,
        float dt
    );

    float updateRate(
        float targetRate,
        float gyroRate,
        float dt
    );

    void reset();


private:

    float kp_;
    float ki_;
    float kd_;


    float integral_;

    float previousError_;


    float output_limit_;
    float integral_limit_;

};