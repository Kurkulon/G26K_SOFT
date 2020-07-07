#ifndef TIME_H__04_08_2009__17_35
#define TIME_H__04_08_2009__17_35

#ifdef WIN32
#include <windows.h>
#endif

#include "types.h"
#include "core.h"

//typedef dword time_utc;

struct time_bdc 
{ 
	byte hsecond; byte second; byte minute; byte hour; byte day; byte month; word year; 

	//time_bdc(const time_bdc& t) 
	//{
	//	*(dword*)this = *(dword*)&t;
	//	*((dword*)this+1) = *((dword*)&t+1);
	//	*((word*)this+4) = *((word*)&t+4);
	//}

	//void operator=(const time_bdc& t) 
	//{
	//	*(dword*)this = *(dword*)&t;
	//	*((dword*)this+1) = *((dword*)&t+1);
	//	*((dword*)this+2) = *((dword*)&t+2);
	//	*((word*)this+6) = *((word*)&t+6);
	//}
};

extern time_bdc timeBDC;

//extern time_utc mktime_utc(const time_bdc &t);
//extern time_utc mktime_utc(time_bdc *t);
//extern bool mktime_bdc(time_utc t, time_bdc *tbdc);
extern void Init_time();
extern void RTT_Init();

//extern bool SetTime(time_utc t);
extern bool SetTime(const time_bdc &t);
//extern time_utc GetTime();
extern void GetTime(time_bdc *t);
extern bool CheckTime(const time_bdc &t);
extern int CompareTime(const time_bdc &t1, const time_bdc &t2);
extern void NextDay(time_bdc *t);
extern void NextHour(time_bdc *t);
extern void PrevDay(time_bdc *t);
extern void PrevHour(time_bdc *t);
extern dword msec;

inline dword GetMilliseconds()
{
#ifndef WIN32
	extern dword msec;
	return msec;
#else
	return GetTickCount();
#endif
}

inline word GetMillisecondsLow()
{
#ifndef WIN32
	extern dword msec;
	return (word)msec;
#else
	return (word)(GetTickCount());
#endif
}

#define US2RT(x) ((x*32768+500000)/1000000)
#define MS2RT(x) ((x*32768+500)/1000)

//inline u32 GetRTT() { return HW::TC0->C0.CV; }

inline u32 GetRTT()
{
	u32 t1 = HW::RTT->VR;
	u32 t2;

	while ((t2 = HW::RTT->VR) != t1) { t1 = t2; };

	return t2;
} 

//inline u32 GetRTT() { return HW::RTT->VR|HW::RTT->VR; }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct TM32
{
	u32 pt;

//	static u32 ipt;

//	TM32() : pt(ipt++) {}
	bool Check(u32 v) { if ((GetMilliseconds() - pt) >= v) { pt = GetMilliseconds(); return true; } else { return false; }; }
	void Reset() { pt = GetMilliseconds(); }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct RTM32
{
	u32 pt;

//	RTM32() : pt(0) {}
	bool Check(u32 v) { if ((GetRTT() - pt) >= v) { pt = GetRTT(); return true; } else { return false; }; }
	void Reset() { pt = GetRTT(); }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // TIME_H__04_08_2009__17_35
