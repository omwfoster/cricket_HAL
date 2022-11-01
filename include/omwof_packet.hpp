#ifndef AD9ABB9E_1FFA_4735_BDC3_E764FA1EE0EC
#define AD9ABB9E_1FFA_4735_BDC3_E764FA1EE0EC

#include "stdint.h"
#include "stdbool.h"


typedef struct __attribute__((packed, aligned(1)))  om_packet{
    uint8_t reg_high;
    uint8_t reg_low;
    uint8_t message[30];
    uint8_t cursor;
}om_packet;

class packet_class
{
    public:
    packet_class();
    ~packet_class();
    
    bool set_high_reg(uint8_t);
    bool set_low_reg(uint8_t);


    private:

    om_packet payload;








};

#endif /* AD9ABB9E_1FFA_4735_BDC3_E764FA1EE0EC */
