#include "omwof_packet.hpp"
#include "string.h"

packet_class::packet_class()
{
    memset(&(this->payload),0,sizeof(packet_class));
}
packet_class::~packet_class()
{
}

bool packet_class::set_high_reg(uint8_t a)
{
    this->payload.reg_high=a;
    return true;
}
bool packet_class::set_low_reg(uint8_t a)
{
    this->payload.reg_low=a;
    return true;
}