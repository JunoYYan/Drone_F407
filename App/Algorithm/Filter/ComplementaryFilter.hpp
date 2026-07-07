#pragma once


class ComplementaryFilter
{

public:

    ComplementaryFilter(float alpha = 0.98f);


    void update(
        float ax,
        float ay,
        float az,
        float gx,
        float gy,
        float dt
    );

    float getPitch() const;
    float getRoll() const;
    void reset(float pitch, float roll);

private:

    float alpha_;


    float pitch_;
    float roll_;

};