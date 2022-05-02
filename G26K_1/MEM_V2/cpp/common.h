#ifndef MEMORY_AT91SAM7X256_COMMON_H
#define MEMORY_AT91SAM7X256_COMMON_H

//#include "AT91SAM7X256.h"
//#define __inline extern inline
//#include "lib_AT91SAM7X256.h"
//#include "gt/inttypes.h"
//#include "gt/system.h"
//#include "stddef.h"
//#include "string.h"

//#define SWAP16(x)	((((x) & 0xff) << 8) | ((x) >> 8))
//#define SWAP32(x)	((((x) & 0xff000000) >>24) | (((x)&0x00ff0000) >> 8) | (((x)&0x0000ff00) << 8) | (((x)&0x000000ff) <<24))

extern void inline COPY(const char *src, char *dst, unsigned int size)	// всё это из-за того, что нельзя копировать 4 байтные данные по указателям
{
	while(size--) *dst++ = *src++;
}

#endif
