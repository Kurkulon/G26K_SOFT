#ifndef CRC16_0805_H__24_11_2020__20_06
#define CRC16_0805_H__24_11_2020__20_06

#include "types.h"

extern const u16 tableCRC_8005[256];

extern word GetCRC16_8005(const void *data, u32 len, word init);
extern word GetCRC16_8005_refl(const void *data, u32 len, word init);


#endif // CRC16_0805_H__24_11_2020__20_06
