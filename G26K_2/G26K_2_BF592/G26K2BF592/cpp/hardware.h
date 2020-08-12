#ifndef HARDWARE_H__15_05_2009__14_35
#define HARDWARE_H__15_05_2009__14_35
  
#include "types.h"
#include "core.h"

#ifdef WIN32
#include <windows.h>
#endif


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PPI_BUF_LEN (1024+32)

struct DSCPPI
{
	DSCPPI	*next;
	u32		mmsec;
	u32		shaftTime;
	u32		shaftPrev;
	u32		fireIndex;
	u16		motoCount;
	u16		shaftCount;
	u16		sensType;
	u16		gain;
	u16		len;
	u16		offset;
	u16		ppiclkdiv;
	u16		ppidelay;
	u16		sampleTime;
	u16		sampleDelay;
	u16		busy;
	u16		data[PPI_BUF_LEN];
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct SENS
{
	u16 	gain; 
	u16 	st;	 
	u16 	sl; 
	u16 	sd; 
	u16		thr;
	u16		descr;
	u16		freq;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma pack(1)

struct ReqDsp01	// ������ �������
{
	u16 	rw;
	u16 	mode; 
	u32 	mmsecTime; 
	u32		hallTime; 
	u16		motoCount; 
	u16		headCount;
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16		sensType; 
	u16		angle;

	SENS	mainSens;
	SENS	refSens;

	u16		vavesPerRoundCM;
	u16		vavesPerRoundIM;

	u16		filtrType;
	u16		packType;

	u16 	crc;  
};

//#pragma pack()

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma pack(1)

struct RspDsp01	// ������ �������
{
	u16 rw; 
	u32 time; 
	u32 hallTime; 

	union
	{
		struct { u16 motoCount; u16 headCount; u16 ax; u16 ay; u16 az; u16 at; u16 sensType; u16 angle; u16 gain; u16 st; u16 sl; u16 sd; u16 pakType; u16 pakLen; u16 data[2048]; } CM;
		struct { u16 ax; u16 ay; u16 az; u16 at; u16 gain; u16 dataLen; u16 data[2048];} IM;
	};
};

//pragma pack()

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct  ReqDsp05 { u16 rw; u16 crc; };										// ������ ����������� ����� � ����� ��������� �� ����-������
struct  ReqDsp06 { u16 rw; u16 stAdr; u16 len; byte data[256]; u16 crc; }; // ������ �������� �� ����
struct  ReqDsp07 { u16 rw; word crc; };										// ������������� �������
struct  RspDsp05 { u16 rw; u16 flashLen; u16 flashCRC; u16 crc; };					// ������ ����������� ����� � ����� ��������� �� ����-������
struct  RspDsp06 { u16 rw; u16 res; u16 crc; };									// ������ �������� �� ����

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();
extern void InitIVG(u32 IVG, u32 PID, void (*EVT)());
extern void SetDspVars(const ReqDsp01 *v);


//extern bool defPPI_Ready;

//extern void SyncReadSPORT(void *dst1, void *dst2, u16 len1, u16 len2, u16 clkdiv, bool *ready0, bool *ready1);
//extern void ReadPPI(void *dst);
extern DSCPPI* GetDscPPI();
extern void FreeDscPPI(DSCPPI* dsc);
extern DSCPPI* AllocDscPPI();

extern void WriteTWI(void *src, u16 len);
extern void ReadTWI(void *dst, u16 len);

extern void SetGain(byte v);

#define MS2RT(x) ((x)*10)
#define US2RT(x) ((x)/100)

inline u32 GetRTT() { extern u32 mmsec; return mmsec; }

struct RTM32
{
	u32 pt;

	RTM32() : pt(0) {}
	bool Check(u32 v) { if ((GetRTT() - pt) >= v) { pt = GetRTT(); return true; } else { return false; }; }
	void Reset() { pt = GetRTT(); }
};

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

#pragma always_inline
inline u32 GetCycles32()
{
	//u32 res;

	//__asm volatile ("%0 = CYCLES;  \n"	: "=d" (res)	:	:	); 

	return sysreg_read(reg_CYCLES); 
}


#endif // HARDWARE_H__15_05_2009__14_35
