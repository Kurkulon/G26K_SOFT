#include "time.h"
#include "core.h"

#include <stdlib.h>

#ifdef WIN32

#include <windows.h>
#include <time.h>

#endif

#pragma O3
#pragma Otime

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

dword msec = 0;

//static U32u __hsec(0);

//u32 TM32::ipt = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//time_bdc timeBDC;

//static bool sync_Loc_RTC = false; // Синхронизация локальных часов с часами реального времени
//static bool sync_RTC_Loc = false; // Синхронизация часов реального времени с локальными часами 

//static U32u timeUpdate(0);
//static U32u dateUpdate(0);
//static u32 timeBuf = 0;
//static u32 dateBuf = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static const byte daysInMonth[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static const word __diyr[] = {									/* days in normal year array */
//    0,                                                          /* Jan */
//    31,                                                         /* Feb */
//    31 + 28,                                                    /* Mar */
//    31 + 28 + 31,                                               /* Apr */
//    31 + 28 + 31 + 30,                                          /* May */
//    31 + 28 + 31 + 30 + 31,                                     /* Jun */
//    31 + 28 + 31 + 30 + 31 + 30,                                /* Jul */
//    31 + 28 + 31 + 30 + 31 + 30 + 31,                           /* Aug */
//    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31,                      /* Sep */
//    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30,                 /* Oct */
//    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31,            /* Nov */
//    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30,       /* Dec */
//    31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30 + 31   /* Jan, next year */
//};
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//static const word __dilyr[] = {									/* days in leap year array */
//    0,                                                          /* Jan */
//    31,                                                         /* Feb */
//    31 + 29,                                                    /* Mar */
//    31 + 29 + 31,                                               /* Apr */
//    31 + 29 + 31 + 30,                                          /* May */
//    31 + 29 + 31 + 30 + 31,                                     /* Jun */
//    31 + 29 + 31 + 30 + 31 + 30,                                /* Jul */
//    31 + 29 + 31 + 30 + 31 + 30 + 31,                           /* Aug */
//    31 + 29 + 31 + 30 + 31 + 30 + 31 + 31,                      /* Sep */
//    31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30,                 /* Oct */
//    31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31,            /* Nov */
//    31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30,       /* Dec */
//    31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30 + 31   /* Jan, next year */
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//inline bool __leapyear(word year)
//{
//    return (year&3) == 0;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void Timer_Handler (void)
{
	msec++;
//	__hsec.d += 6521;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitTimer()
{
	enum { freq = 1000 };

	CM4::SysTick->LOAD = (MCK+freq/2)/freq;
	VectorTableInt[15] = Timer_Handler;
	CM4::SysTick->CTRL = 7;
	__enable_irq();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RTT_Init()
{
	using namespace HW;

	//PMC->PCER0 = PID::TC3_M;
	//TC1->C0.CMR = TIMER_CLOCK5; // SCLK
	//TC1->C0.CCR = CLKEN|SWTRG;

	RTT->MR = 0x40001;
//	RTT->MR = 0x100000;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// #define DAYS_IN_4_YRS   ( 365 + 365 + 365 + 366 )
// #define DAYS_IN_400_YRS ( ( 100 * DAYS_IN_4_YRS ) - 3 )

//  #define SECONDS_PER_DAY ( 24 * 60 * 60 )
//  extern  short   __diyr[], __dilyr[];

/*
 The number of leap years from year 1 to year 1900 is 460.
 The number of leap years from year 1 to current year is
 expressed by "years/4 - years/100 + years/400". To determine
 the number of leap years from current year to 1900, we subtract
 460 from the formula result. We do this since "days" is the
 number of days since 1900.
*/

//static dword __DaysToJan1(word year)
//{
//    unsigned    years = 1900 + year - 1;
//    unsigned    leap_days = years / 4 - years / 100 + years / 400 - 460;
//
//    return( year * 365UL + leap_days );
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define SECONDS_FROM_1900_TO_1970       2208988800UL
//#define SECONDS_FROM_1972_TO_2000       883612800UL
//#define SECONDS_FROM_1970_TO_1972       63072000UL
//#define SECONDS_FROM_1970_TO_2000		(SECONDS_FROM_1970_TO_1972+SECONDS_FROM_1972_TO_2000)      
//
//#define SECONDS_PER_DAY                 (24 * 60 * 60)
//#define DAYS_FROM_1900_TO_1970          ( ( long ) ( SECONDS_FROM_1900_TO_1970 / SECONDS_PER_DAY ) ) 
//
//#define MONTH_YR        ( 12 )
//#define DAY_YR          ( 365 )
//#define HOUR_YR         ( DAY_YR * 24 )
//#define MINUTE_YR       ( HOUR_YR * 60 )
//#define SECOND_YR       ( MINUTE_YR * 60 )
//#define __MONTHS        ( INT_MIN / MONTH_YR )
//#define __DAYS          ( INT_MIN / DAY_YR )

//// these ones can underflow in 16bit environments,
//// so check the relative values first
//#if ( HOUR_YR ) < ( INT_MAX / 60 )
// #define __MINUTES      ( INT_MIN / MINUTE_YR )
// #if ( MINUTE_YR ) < ( INT_MAX / 60 )
//  #define __SECONDS     ( INT_MIN / SECOND_YR )
// #else
//  #define __SECONDS     ( 0 )
// #endif
//#else
// #define __MINUTES      ( 0 )
// #define __SECONDS      ( 0 )
//#endif

//#define SMALLEST_YEAR_VALUE ( __MONTHS + __DAYS + __MINUTES + __SECONDS )

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//time_utc mktime_utc(const time_bdc &t)
//{
//    byte        month;
//    int         year;
//    long        days;
//    long        seconds;
//    const word*	month_start;
//
//	year = t.year - 1972;
//
//	month = t.month - 1;
//
//	if (year < 0 || month > 11)
//	{
//        return (-1);
//	};
//
//	month_start = __leapyear(year) ? __dilyr : __diyr;
//
//    days = year * 365L					/* # of days in the years */
//        + ((year + 3) >> 2)				/* add # of leap years before year */
//        + month_start[month]			/* # of days to 1st of month*/
//		+ t.day - 1;					/* day of the month */
//
//		seconds = (t.hour * 60L + t.minute) * 60L + t.second;
//                                        
//    return (seconds + days * SECONDS_PER_DAY + SECONDS_FROM_1970_TO_1972);
//} 

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//time_utc mktime_utc(time_bdc *t)
//{
//	return t->t = mktime_utc(*t);
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32
struct lldiv_t { __int64 quot, rem; };

lldiv_t lldiv(__int64 n, __int64 d)
{
	lldiv_t l;

	l.quot = n / d;
	l.rem = n % d;

	return l;
}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//bool mktime_bdc(time_utc t, time_bdc *tbdc)
//{
//    const word*	month_start;
//
//	div_t s;
//
//	lldiv_t s64;
//
//	dword y4;
//
//	t -= SECONDS_FROM_1970_TO_1972;
//
//	s64 = lldiv(t, 60);
//	tbdc->second = s64.rem;
//
//	s = div((dword)s64.quot, 60);
//	tbdc->minute = s.rem;
//
//	s = div(s.quot, 24);
//	tbdc->hour = s.rem;
//
////	tbdc->dayofweek = (s.quot+6) % 7;
//
//	s = div(s.quot, 1461);
//
//	y4 = s.quot;
//
//	if (s.rem > 365)
//	{
//		s.rem -= 366;
//		s = div(s.rem, 365);
//		s.quot += 1;
//
//		month_start = __diyr;
//	}
//	else
//	{
//		s.quot = 0;
//
//		month_start = __dilyr;
//	};
//
//	s.rem += 1;
//
//	byte i = (s.rem >> 5) + 1;
//
//	if (s.rem > month_start[i]) { i += 1; };
//
//	tbdc->month = i;
//
//	s.rem -= month_start[i-1];
//
//	tbdc->day = s.rem;
//
//	tbdc->year = (y4 << 2) + s.quot + 1972;
//
//	tbdc->t = t;
//
//	return true;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static __irq void RTC_IntHandler()
//{
//	using namespace HW;
//
//	if ((RTC->SR & 1) != 0)
//	{
//		RTC->TIMR = timeUpdate;
//		RTC->CALR = dateUpdate;
//		
//		RTC->CR &= ~3;
//
//		RTC->SCCR = 1;
//	}
//	else if ((RTC->SR & 4) != 0)
//	{
//		timeBuf = RTC->TIMR;
//		dateBuf = RTC->CALR;
//
//		__hsec = 0;
//
//		RTC->SCCR = 4;
//	}
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void GetTime(time_bdc *t)
//{
//
//	if (t == 0) return;
//
//#ifndef WIN32
//
//	U32u d1(0), t1(0);
//	byte hs;
//
////	CM3::NVIC->ICER[0] = HW::PID::RTC_M;
//
//	__disable_irq();
//
//	d1.d = dateBuf;
//	t1.d = timeBuf;
//	hs = __hsec.b[2];
//
//	__enable_irq();
//
////	CM3::NVIC->ISER[0] = HW::PID::RTC_M;
//
//	if (hs > 99) { hs = 99; };
//
//	t->hsecond = hs;
//	t->second =		(t1.b[0]&15) + (t1.b[0] >> 4) * 10;
//	t->minute =		(t1.b[1]&15) + (t1.b[1] >> 4) * 10;
//	t->hour =		(t1.b[2]&15) + ((t1.b[2] >> 4) & 3) * 10;
//	t->day =		(d1.b[3]&15) + ((d1.b[3] >> 4) & 3) * 10;
//	t->month =		(d1.b[2]&15) + ((d1.b[2] >> 4) & 1) * 10;
//	t->year =		(d1.b[1]&15) + (d1.b[1]>>4) * 10 + (d1.b[0] & 15) * 100 + ((d1.b[0] >> 4) & 7) * 1000;
////	t->dayofweek =	(d1.b[2]>>5) - 1;
//
//#else
//
//	SYSTEMTIME lt;
//
//	GetLocalTime(&lt);
//
//	t->hsecond = lt.wMilliseconds/10;
//	t->second = lt.wSecond;
//	t->minute = lt.wMinute;
//	t->hour = lt.wHour;
//	t->day = lt.wDay;
//	t->month = lt.wMonth;
//	t->year = lt.wYear;
//
//#endif
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//bool CheckTime(const time_bdc &t)
//{
//	if (t.second > 59 || t.minute > 59 || t.hour > 23 || t.day < 1 || (t.month-1) > 11 || (t.year-2000) > 99) { return false; };
//
//	byte d = daysInMonth[t.month] + ((t.month == 2 && (t.year&3) == 0) ? 1 : 0);
//
//	if (t.day > d) { return false; };
//
//	return true;
//}
//
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//bool SetTime(const time_bdc &t)
//{
//#ifndef WIN32
//
//	if (!CheckTime(t)) { return false; };
//
////	time_utc tutc = mktime_utc(t);
//
//	div_t s;
//
//	s = div(t.second, 10);
//	timeUpdate.b[0] = (byte)((s.quot<<4)|(s.rem));
//
//	s = div(t.minute, 10);
//	timeUpdate.b[1] = (byte)((s.quot<<4)|(s.rem));
//
//	s = div(t.hour, 10);
//	timeUpdate.b[2] = (byte)((s.quot<<4)|(s.rem));
//
//	s = div(t.day, 10);
//	dateUpdate.b[3] = (byte)((s.quot<<4)|(s.rem));
//
//	s = div(t.month, 10);
//	dateUpdate.b[2] = (byte)((s.quot<<4)|(s.rem)|0x20);
//
//	s = div(t.year - 2000, 10);
//	dateUpdate.b[1] = (byte)((s.quot<<4)|(s.rem));
//
//	dateUpdate.b[0] = 0x20;
//
//	HW::RTC->CR = 3;
//
//	return true;
//
//#else
//
//	SYSTEMTIME lt;
//
//	lt.wMilliseconds	=	t.hsecond*10;
//	lt.wSecond			=	t.second	;
//	lt.wMinute			=	t.minute	;
//	lt.wHour			=	t.hour		;
//	lt.wDay				=	t.day		;
//	lt.wMonth			=	t.month	;
//	lt.wYear			=	t.year		;
//
//	return SetLocalTime(&lt);
//
//#endif
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Init_time()
{
	using namespace HW;

	InitTimer();
	RTT_Init();

	//PMC->PCER0 = PID::RTC_M;

	//RTC->CR = 0;
	//RTC->MR = 0;

	//timeBuf = RTC->TIMR;
	//dateBuf = RTC->CALR;

	//VectorTableExt[PID::RTC_I] = RTC_IntHandler;

	//CM3::NVIC->ICPR[0] = PID::RTC_M;
	//CM3::NVIC->ISER[0] = PID::RTC_M;

	//RTC->IER = 5; //SECEN ACKEN

	__enable_irq();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//int CompareTime(const time_bdc &t1, const time_bdc &t2)
//{
//	if (t1.year > t2.year)
//	{
//		return 1;
//	}
//	else if (t1.year < t2.year)
//	{
//		return -1;
//	}
//	else if (t1.month > t2.month)
//	{
//		return 1;
//	}
//	else if (t1.month < t2.month)
//	{
//		return -1;
//	}
//	else if (t1.day > t2.day)
//	{
//		return 1;
//	}
//	else if (t1.day < t2.day)
//	{
//		return -1;
//	}
//	else if (t1.hour > t2.hour)
//	{
//		return 1;
//	}
//	else if (t1.hour < t2.hour)
//	{
//		return -1;
//	}
//	else if (t1.minute > t2.minute)
//	{
//		return 1;
//	}
//	else if (t1.minute < t2.minute)
//	{
//		return -1;
//	}
//	else if (t1.second > t2.second)
//	{
//		return 1;
//	}
//	else if (t1.second < t2.second)
//	{
//		return -1;
//	}
//	else
//	{
//		return 0;
//	};
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void NextDay(time_bdc *t)
//{
//	unsigned char maxDay = ((t->year & 3) == 0 && t->month == 2) ? 29 : daysInMonth[t->month];
//
//	if ((t->day += 1) > maxDay)
//	{
//		t->day = 1;
//
//		if ((t->month += 1) > 12)
//		{
//			t->month = 1;
//			t->year += 1;
//		};
//	};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void NextHour(time_bdc *t)
//{
//	t->hour += 1;
//
//	if (t->hour > 23)
//	{
//		t->hour = 0;
//		
//		if ((t->day += 1) > (daysInMonth[t->month] + (((t->year & 3) == 0) ? 1 : 0)))
//		{
//			t->day = 1;
//			if ((t->month += 1) > 12)
//			{
//				t->month = 1;
//				t->year += 1;
//			};
//		};
//	};
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void PrevDay(time_bdc *t)
//{
//	unsigned char maxDay;
//
//	if ((t->day -= 1) == 0)
//	{
//		if ((t->month -= 1) == 0)
//		{
//			t->month = 12;
//			t->year -= 1;
//		};
//	
//		maxDay = ((t->year & 3) == 0 && t->month == 2) ? 29 : daysInMonth[t->month];
//		t->day = maxDay;
//	};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void PrevHour(time_bdc *t)
//{
//	if (t->hour == 0)
//	{
//		t->hour = 23;
//		
//		if (t->day == 1)
//		{
//			if (t->month == 1)
//			{
//				t->month = 12;
//				t->year -= 1;
//			}
//			else
//			{
//				t->month -= 1;
//			};
//
//			t->day = daysInMonth[t->month] + ((t->month == 2 && (t->year & 3) == 0) ? 1 : 0);
//		}
//		else
//		{
//			t->day -= 1;
//		};
//	}
//	else
//	{
//		t->hour -= 1;
//	};
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


