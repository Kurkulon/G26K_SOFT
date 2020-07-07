#include "types.h"
#include "CRC16.h"

#pragma O3
#pragma Otime

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

word GetCRC16(const void *data, u32 len)
{
	DataCRC CRC = { 0xFFFF };

	const byte *s = (const byte*)data;

	for ( ; len > 0; len--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ *(s++)] ^ CRC.b[1];
	};

	return CRC.w;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

word GetCRC16_asm(const void *data, u32 len)
{
	register u32 crc = 0xFFFF;

//	register u32 sum = 0;
	register u32 t;

	register const u16 *tb = tableCRC;


	if (len > 0)
	{

loop:
		__asm
		{
			LDRB	t, [data], #1
			EOR		t, crc
			UXTB	t, t
			LDRH	t, [tb, t]
			EOR		crc, t, crc, LSR #8
			SUBS	len, len, #1
			BNE		loop

		};
	};

	return crc;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
