#include "UartReceiver.hpp"

#include <cstring>
#include <cstdlib>


UartReceiver::UartReceiver(
    UART_HandleTypeDef* huart
)
:
huart_(huart),
index_(0),
throttle_(1000),
armCommand_(false),
disarmCommand_(false)
{

}


void UartReceiver::update()
{
    uint8_t c;

    while (
        HAL_UART_Receive(
            huart_,
            &c,
            1,
            0
        ) == HAL_OK
    )
    {
        // 一键急停：只要收到 X/x，立刻 disarm，不需要回车
        if(c == 'X' || c == 'x')
        {
            disarmCommand_ = true;
            index_ = 0;
            return;
        }

        // 处理回车：PuTTY 可能发 \r，也可能发 \n
        if(c == '\r' || c == '\n')
        {
            if(index_ == 0)
            {
                continue;
            }

            buffer_[index_] = '\0';

            if(strcmp(buffer_, "ARM") == 0)
            {
                armCommand_ = true;
            }
            else if(strcmp(buffer_, "DISARM") == 0)
            {
                disarmCommand_ = true;
            }
            else if(strncmp(buffer_, "THROTTLE", 8) == 0)
            {
                char* p = buffer_ + 8;

                while(*p == ' ' || *p == ':' || *p == '=')
                {
                    p++;
                }

                int value = atoi(p);

                if(value >= 1000 && value <= 2000)
                {
                    throttle_ = value;
                }
            }

            index_ = 0;
        }
        else
        {
            if(index_ < sizeof(buffer_) - 1)
            {
                buffer_[index_++] = static_cast<char>(c);
            }
            else
            {
                index_ = 0;
            }
        }
    }
}


bool UartReceiver::consumeArmCommand()
{
    if(armCommand_)
    {
        armCommand_ = false;
        return true;
    }

    return false;
}


bool UartReceiver::consumeDisarmCommand()
{

    if(disarmCommand_)
    {
        disarmCommand_ = false;
        return true;
    }

    return false;

}


uint16_t UartReceiver::getThrottle()
{
    return throttle_;
}