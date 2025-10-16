#include "pack.h"

#pragma optimize_for_speed


//#define FDCT_BAND_DIV 2

//#include "WAVEPACK\fdct_imp.h"
#include "WAVEPACK\wavepack_imp.h"

//#include "PackWave.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void Pack_uLaw_12Bit(i16* src, byte* dst, u16 len)
//{
//	WavePack_uLaw_12Bit((u16*)src, dst, len);
//}
//
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//void Pack_uLaw_16Bit(i16* src, byte* dst, u16 len)
//{
//	WavePack_uLaw_16Bit((u16*)src, dst, len);
//}
//
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//void Pack_ADPCMIMA(i16* src, byte* dst, u16 len)
//{
//	WavePack_ADPCMIMA((u16*)src, dst, len);
//}
//
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Pack_uLaw12_FDCT(FDCT_DATA* src, byte* dst, u16 len, u16 scale)
{
	WavePack_uLaw12_FDCT(src, dst, len, scale);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 Pack_FDCT_Quant12(FDCT_DATA* src, u16 packLen, u16 shift, u16* const scale)
{
	packLen -= 1;

	packLen = MIN(packLen, FDCT_N-1);

	FDCT_DATA max = 0;
	FDCT_DATA avrmax = 0;
	FDCT_DATA sum = 0;

	for (u32 n = 0, i = 1; i <= packLen; i++,n++)
	{
		FDCT_DATA t = ABS(src[i]);

		sum += t; 

		max = Max32(max,t); 
		if ((n&7) == 0) avrmax = Max32(avrmax, sum), sum = 0;
	};

	avrmax = Max32(avrmax, sum);

	//FDCT_DATA *p = fdct_w + packLen - 1;
	FDCT_DATA lim = avrmax/8;

	lim = (i32)lim >> shift;

	*scale = 0;

	max = Max32(max, ABS(src[0]));

	while (max > 2047) { max /= 2; *scale += 1; };

	u32 xx = 1 << *scale;

	if (lim < xx) lim = xx;

	for (u32 i = packLen; i > 1; i--)
	{
		FDCT_DATA t = ABS(src[i]);

		if (t > lim) { packLen = i; break; };
	};

	return (packLen + 2) & ~1;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//void Pack_Init()
//{
//	FDCT_Init();
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
