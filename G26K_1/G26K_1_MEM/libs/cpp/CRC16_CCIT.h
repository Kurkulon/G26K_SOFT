#ifndef CRC16_CCIT_H__05_12_2019__15_18
#define CRC16_CCIT_H__05_12_2019__15_18

#include "types.h"

extern const u16 tableCRC_CCIT[256];

extern word GetCRC16_CCIT(const void *data, u32 len, word init);


#endif // CRC16_CCIT_H__05_12_2019__15_18
