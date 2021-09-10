#include "PointerCRC.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PointerCRC::ReadArrayW(DataPointer p, word count)
{
	for (; count > 0 ; count--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
	};
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
