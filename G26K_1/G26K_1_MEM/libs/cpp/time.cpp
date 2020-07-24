#include "time.h"
#include "core.h"

#ifdef WIN32

#include <windows.h>
#include <time.h>

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 msec = 0;

RTC timeBDC;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const byte daysInMonth[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define SECONDS_FROM_1900_TO_1970       2208988800UL
#define SECONDS_FROM_1972_TO_2000       883612800UL
#define SECONDS_FROM_1970_TO_1972       63072000UL
#define SECONDS_FROM_1970_TO_2000		(SECONDS_FROM_1970_TO_1972+SECONDS_FROM_1972_TO_2000)      

#define SECONDS_PER_DAY                 (24 * 60 * 60)
#define DAYS_FROM_1900_TO_1970          ( ( long ) ( SECONDS_FROM_1900_TO_1970 / SECONDS_PER_DAY ) ) 

#define MONTH_YR        ( 12 )
#define DAY_YR          ( 365 )
#define HOUR_YR         ( DAY_YR * 24 )
#define MINUTE_YR       ( HOUR_YR * 60 )
#define SECOND_YR       ( MINUTE_YR * 60 )
#define __MONTHS        ( INT_MIN / MONTH_YR )
#define __DAYS          ( INT_MIN / DAY_YR )

#define SMALLEST_YEAR_VALUE ( __MONTHS + __DAYS + __MINUTES + __SECONDS )

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void GetTime(RTC *t)
{
	if (t == 0) return;

	__disable_irq();

	t->date = timeBDC.date;
	t->time = timeBDC.time;

	__enable_irq();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool CheckTime(const RTC &t)
{
	if (t.sec > 59 || t.min > 59 || t.hour > 23 || t.day < 1 || (t.mon-1) > 11 || (t.year-2000) > 99) { return false; };

	byte d = daysInMonth[t.mon] + ((t.mon == 2 && (t.year&3) == 0) ? 1 : 0);

	if (t.day > d) { return false; };

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool SetTime(const RTC &t)
{
	if (!CheckTime(t)) { return false; };

	__disable_irq();

	timeBDC = t;

	__enable_irq();

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void Timer_Handler (void)
{
	msec++;

	if (timeBDC.msec < 999)
	{
		timeBDC.msec += 1;
	}
	else
	{
		timeBDC.msec = 0;

		if (timeBDC.sec < 59)
		{
			timeBDC.sec += 1;
		}
		else
		{
			timeBDC.sec = 0;

			if (timeBDC.min < 59)
			{
				timeBDC.min += 1;
			}
			else
			{
				timeBDC.min = 0;

				if (timeBDC.hour < 23)
				{
					timeBDC.hour += 1;
				}
				else
				{
					timeBDC.hour = 0;

					byte day = daysInMonth[timeBDC.mon] + ((timeBDC.mon == 2 && (timeBDC.year&3) == 0) ? 1 : 0);

//					if ((timeBDC.dayofweek += 1) > 6) timeBDC.dayofweek = 0;

					if (timeBDC.day < day)
					{
						timeBDC.day += 1;
					}
					else
					{
						timeBDC.day = 1;

						if (timeBDC.mon < 12)
						{
							timeBDC.mon += 1;
						}
						else
						{
							timeBDC.mon = 1;

							timeBDC.year += 1;
						};
					};
				};
			};
		};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitTimer()
{
	enum { freq = 1000 };

	timeBDC.day = 1;
	timeBDC.mon = 1;
	timeBDC.year = 2000;
	timeBDC.time = 0;

	CM4::SysTick->LOAD = (MCK+freq/2)/freq;
	VectorTableInt[15] = Timer_Handler;
	CM4::SysTick->VAL = 0;
	CM4::SysTick->CTRL = 7;
	__enable_irq();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RTT_Init()
{
	using namespace HW;

	#ifdef CPU_SAME53	

		HW::OSC32KCTRL->RTCCTRL = OSC32_RTCSEL_ULP32K;

		HW::MCLK->APBAMASK |= APBA_RTC;

		HW::RTC->CTRLA = 0;

		HW::RTC->CTRLA = RTC_SWRST;

		while(HW::RTC->CTRLA & RTC_SWRST);

		HW::RTC->CTRLA = RTC_ENABLE|RTC_MODE_COUNT32|RTC_COUNTSYNC;

		while(HW::RTC->SYNCBUSY != 0);

	#elif defined(CPU_XMC48)

		T_HW::CCU4_GLOBAL_Type * const module = HW::CCU43;
		T_HW::CCU4_CC4_Type * const slice = HW::CCU43_CC43;

		CCU_Enable(PID_CCU43);

		module->GCTRL = 0;

		module->GIDLC = CCU4_GIDLC_CS3I_Msk|CCU4_GIDLC_SPRB_Msk;

		slice->PRS = 0xFFFF;
		slice->PSC = 11; //20.48us

		module->GCSS = CCU4_GCSS_S3SE_Msk;  

		slice->TCSET = 1;

	#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Init_time()
{
	using namespace HW;

	InitTimer();
	RTT_Init();
}

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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

