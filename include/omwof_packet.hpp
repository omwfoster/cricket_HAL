#ifndef AD9ABB9E_1FFA_4735_BDC3_E764FA1EE0EC
#define AD9ABB9E_1FFA_4735_BDC3_E764FA1EE0EC

#include "stdint.h"
#include "stdbool.h"
#include "omwof_ada_interface.hpp"




class data_pack
{
    public:
    data_pack();
    ~data_pack();
    
    bool set_high_reg(uint8_t);
    bool set_low_reg(uint8_t);
    bool set_string(char *, uint8_t len);


    private:

    om_packet payload;








};

#endif /* AD9ABB9E_1FFA_4735_BDC3_E764FA1EE0EC */
