#include "types.h"
#include "CRC16_8005.h"

#pragma O3
#pragma Otime

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

word GetCRC16_8005(const void *data, u32 len, word init)
{
	DataCRC CRC = { init };

	const byte *s = (const byte*)data;

	for ( ; len > 0; len--)
	{
		CRC.w = tableCRC_8005[CRC.b[0] ^ *(s++)] ^ CRC.b[1];
	};

	return CRC.w;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

word GetCRC16_8005_refl(const void *data, u32 len, word init)
{
	DataCRC CRC = { init };

	const byte *s = (const byte*)data;

	for ( ; len > 0; len--)
	{
		CRC.w = tableCRC_8005[CRC.b[1] ^ *(s++)] ^ (CRC.w<<8);
	};

	return CRC.w;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
