#ifndef C9AB2501_CD8A_4D28_A0E8_2030A60FFD39
#define C9AB2501_CD8A_4D28_A0E8_2030A60FFD39

#include "stdint.h"


typedef struct __attribute__((packed, aligned(1)))  om_packet{
    uint8_t reg_high;
    uint8_t reg_low;
    uint8_t message[30];
    uint8_t cursor;
    bool processed;
}om_packet;



#ifdef __cplusplus
extern "C"
{
#endif
    void cricket_main_c();  // C function to call into Cpp event loop from main
#ifdef __cplusplus
}
#endif

#endif /* C9AB2501_CD8A_4D28_A0E8_2030A60FFD39 */
