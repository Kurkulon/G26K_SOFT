#ifndef HARDWARE_H__15_05_2009__14_35
#define HARDWARE_H__15_05_2009__14_35
  
#include "types.h"
#include "core.h"

#ifdef WIN32
#include <windows.h>
#endif


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct  ReqDsp05 { u16 rw; u16 crc; };												// запрос контрольной суммы и длины программы во флэш-памяти
struct  ReqDsp06 { u16 rw; u16 stAdr; u16 len; byte data[256]; u16 crc; };			// запись страницы во флэш
struct  ReqDsp07 { u16 rw; word crc; };												// перезагрузить блэкфин
struct  RspDsp05 { u16 rw; u16 flashLen; u32 startAdr; u16 flashCRC; u16 crc; };	// запрос контрольной суммы и длины программы во флэш-памяти
struct  RspDsp06 { u16 rw; u16 res; u16 crc; };										// запись страницы во флэш

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct Req
{
	Req *next;

	ReqDsp06	req;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();
extern void InitIVG(u32 IVG, u32 PID, void (*EVT)());

extern void WriteTWI(void *src, u16 len);
extern void ReadTWI(void *dst, u16 len);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u64 GetCycles64()
{
	u64 res;

	__asm volatile ("CLI r0;       \n"  
                    "r2 = CYCLES;  \n"  
                    "r1 = CYCLES2; \n"  
                    "STI r0;       \n"  
                    "[%0]   = r2;  \n"  
                    "[%0+4] = r1;  \n"  
                    : : "p" (&res) 
                    : "r0", "r1", "r2" ); 

	return res;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma always_inline
inline u32 GetCycles32()
{
	//u32 res;

	//__asm volatile ("%0 = CYCLES;  \n"	: "=d" (res)	:	:	); 

	return sysreg_read(reg_CYCLES); 
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MS2RT(x) MS2CCLK(x)
#define US2RT(x) US2CCLK(x)

//inline u32 GetRTT() { extern u32 mmsec; return mmsec; }

struct RTM32
{
	u32 pt;

	RTM32() : pt(0) {}
	bool Check(u32 v) { u32 t = GetCycles32(); if ((t - pt) >= v) { pt = t; return true; } else { return false; }; }
	void Reset() { pt = GetCycles32(); }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#endif // HARDWARE_H__15_05_2009__14_35
