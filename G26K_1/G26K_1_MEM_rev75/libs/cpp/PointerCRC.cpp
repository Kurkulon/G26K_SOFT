#include "PointerCRC.h"

#pragma O3
#pragma Otime

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PointerCRC::ReadArrayB(DataPointer p, word count)
{
	for (; count > 0 ; count--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PointerCRC::ReadArrayD(DataPointer p, word count)
{
	for (; count > 0 ; count--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PointerCRC::ReadArrayF(DataPointer p, word count)
{
	for (; count > 0 ; count--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(p.b++) = *(b++))] ^ CRC.b[1];
	};
}

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

void PointerCRC::WriteArrayB(ConstDataPointer p, word count)
{
	for (; count > 0 ; count--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PointerCRC::WriteArrayD(ConstDataPointer p, word count)
{
	for (; count > 0 ; count--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PointerCRC::WriteArrayF(ConstDataPointer p, word count)
{
	for (; count > 0 ; count--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PointerCRC::WriteArrayW(ConstDataPointer p, word count)
{
	for (; count > 0 ; count--)
	{
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
		CRC.w = tableCRC[CRC.b[0] ^ (*(b++) = *(p.b++))] ^ CRC.b[1];
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
