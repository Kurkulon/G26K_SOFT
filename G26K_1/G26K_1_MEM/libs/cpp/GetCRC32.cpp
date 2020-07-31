#include "types.h"
#include "CRC32.h"

#pragma O3
#pragma Otime

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 GetCRC32(const void *data, u16 len)
{
	union
	{
		u32 r;
		byte b[4];
	};

	byte *s = (byte*)data;

	r = 0xFFFFFFFF;

	for ( ; len > 0; len--)
	{
		byte t = b[0];
		r >>= 8;
		r ^= tableCRC32[t ^ *(s++)];
	};

	return r^0xFFFFFFFF;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 GetCRC32(const void *data, u16 len, u32 init, u32 xorOut)
{
	union {	u32 r;	byte b[4];	};

	byte *s = (byte*)data;

	r = init;

	for ( ; len > 0; len--)
	{
		byte t = b[0];
		r >>= 8;
		r ^= tableCRC32[t ^ *(s++)];
	};

	return r ^ xorOut;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
