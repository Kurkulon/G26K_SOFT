#ifndef CRC32_H__07_09_2010__15_55
#define CRC32_H__07_09_2010__15_55

#include "types.h"

extern const u32 tableCRC32[256];

extern u32 GetCRC32(const void *data, u16 len);
extern u32 GetCRC32(const void *data, u16 len, u32 init, u32 xorOut);

inline u32 CheckCRC32(const void *data, u16 len) { return GetCRC32(data, len) ^ 0x2144DF1C; }

#endif // CRC32_H__07_09_2010__15_55
