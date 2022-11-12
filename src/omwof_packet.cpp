#include "omwof_packet.hpp"
#include "string.h"

data_pack::data_pack()
{
    memset(&(this->payload), 0, sizeof(data_pack));
}
data_pack::~data_pack()
{
}

bool data_pack::set_high_reg(uint8_t a)
{
    this->payload.reg_high = a;
    return true;
}
bool data_pack::set_low_reg(uint8_t a)
{
    this->payload.reg_low = a;
    return true;
}

bool data_pack::set_string(uint8_t *str_out, uint8_t len)
{
    if (len < MESSAGE_LENGTH)
    {
        memcpy(&this->payload.message[0], (void *)str_out, (size_t)len);
        return true;
    }
    else
    {
        return false
    }
}