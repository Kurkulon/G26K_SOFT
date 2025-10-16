#ifndef PACK_H__17_02_2024__00_00
#define PACK_H__17_02_2024__00_00

#define FDCT_LOG2N		6
#define FDCT_N			(1UL<<FDCT_LOG2N)
#define FDCT_TRIG		u16
#define FDCT_TRIGBITS	(17-FDCT_LOG2N)

#include "WAVEPACK\fdct.h"

extern u16 WavePack_uLaw_12Bit(i16* src, byte* dst, u16 len);
extern u16 WavePack_ADPCMIMA(i16* src, byte* dst, u16 len);

extern void Pack_uLaw12_FDCT(FDCT_DATA* src, byte* dst, u16 len, u16 scale);
extern u16 Pack_FDCT_Quant12(FDCT_DATA* src, u16 packLen, u16 shift, u16* const scale);

//#define WAVEPACK_V2

//#include "WAVEPACK\wavepack.h"

//enum PackType { PACK_NO = 0, PACK_ULAW12, PACK_ULAW16, PACK_ADPCMIMA, PACK_DCT0, PACK_DCT1, PACK_DCT2, PACK_DCT3 };

//extern void Pack_uLaw_12Bit(i16* src, byte* dst, u16 len);
//extern void Pack_uLaw_16Bit(i16* src, byte* dst, u16 len);
//extern void Pack_ADPCMIMA(i16* src, byte* dst, u16 len);
//extern	u16	Pack_FDCT(i16* src, byte* dst, u16 len, u16 shift, u16 OVRLAP, u16* packedLen);
//extern void Pack_Init();

#endif //PACK_H__17_02_2024__00_00
