#include "types.h"
#include "CRC16.h"

word GetCRC16(const void *data, u32 len, word init, word xorOut)
{
	DataCRC CRC = { init };

	const byte *s = (const byte*)data;

	for ( ; len > 0; len--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ *(s++)] ^ CRC.b[1];
	};

	return CRC.w ^ xorOut;
}

