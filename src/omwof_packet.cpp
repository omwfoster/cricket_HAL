#include "omwof_packet.hpp"
#include "string.h"

data_pack::data_pack()
{
    memset(&(this->payload),0,sizeof(data_pack));
}
data_pack::~data_pack()
{
}

bool data_pack::set_high_reg(uint8_t a)
{
    this->payload.reg_high=a;
    return true;
}
bool data_pack::set_low_reg(uint8_t a)
{
    this->payload.reg_low=a;
    return true;
}

bool data_pack::set_string(char * str_out, uint8_t len)
{
    this->set_string(str_out,len);
    return true;
}