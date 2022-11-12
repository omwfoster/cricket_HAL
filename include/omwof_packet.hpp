#ifndef AD9ABB9E_1FFA_4735_BDC3_E764FA1EE0EC
#define AD9ABB9E_1FFA_4735_BDC3_E764FA1EE0EC

#include "stdint.h"
#include "stdbool.h"
#include "omwof_ada_interface.hpp"

#define MESSAGE_LENGTH 30

typedef struct __attribute__((packed, aligned(1)))  om_packet{
    uint8_t reg_high;
    uint8_t reg_low;
    uint8_t message[MESSAGE_LENGTH];
    uint8_t cursor;
    bool processed;
}om_packet;


class data_pack
{
    public:
    data_pack();
    ~data_pack();
    
    bool set_high_reg(uint8_t);
    bool set_low_reg(uint8_t);
    bool set_string(uint8_t *, uint8_t len);
    om_packet * output_string();

    private:
    om_packet payload;

};

#endif /* AD9ABB9E_1FFA_4735_BDC3_E764FA1EE0EC */
