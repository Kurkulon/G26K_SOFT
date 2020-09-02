#include "types.h"
#include "core.h"
#include "time.h"

#include "hardware.h"

#include "ComPort.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void IntDummyHandler()
{
	__breakpoint(0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void HardFaultHandler()
{
	__breakpoint(0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void ExtDummyHandler()
{
	__breakpoint(0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitVectorTable()
{
	for (u32 i = 0; i < ArraySize(VectorTableInt); i++)
	{
		VectorTableInt[i] = IntDummyHandler;
	};

	for (u32 i = 0; i < ArraySize(VectorTableExt); i++)
	{
		VectorTableExt[i] = ExtDummyHandler;
	};

	VectorTableInt[3] = HardFaultHandler;

	CM0::SCB->VTOR = (u32)VectorTableInt;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*----------------------------------------------------------------------------
  Initialize the system
 *----------------------------------------------------------------------------*/
//extern "C" void SystemInit()
//{
//	u32 i;
//	using namespace CM0;
//	using namespace HW;
//
//	SYSCON->SYSAHBCLKCTRL |= CLK::SWM_M | CLK::IOCON_M | CLK::GPIO_M | HW::CLK::MRT_M | HW::CLK::UART0_M | HW::CLK::CRC_M | HW::CLK::DMA_M;
//
//	GPIO->DIRSET0 = (1<<27)|(1<<14)|(1<<17)|(1<<18)|(1<<19)|(1<<20)|(1<<21)|(1<<22)|(1<<12)|(1<<4);
//	GPIO->CLR0 = (1<<27)|(1<<14)|(1<<20)|(1<<21)|(1<<22)|(1<<4);
//	GPIO->SET0 = (1<<17)|(1<<18)|(1<<19);
//
//	IOCON->PIO0_1.B.MODE = 0;
//
//	HW::GPIO->NOT0 = 1<<12;
//
//	SWM->PINENABLE0.B.CLKIN = 0;
//
//	for (i = 0; i < 200; i++) __nop();
//
//	SYSCON->SYSPLLCLKSEL  = 3;					/* Select PLL Input         */
//	SYSCON->SYSPLLCLKUEN  = 0;					/* Update Clock Source      */
//	SYSCON->SYSPLLCLKUEN  = 1;					/* Update Clock Source      */
//	while (!(SYSCON->SYSPLLCLKUEN & 1));		/* Wait Until Updated       */
//
//	HW::GPIO->NOT0 = 1<<12;
//
//	SYSCON->MAINCLKSEL    = 1;					/* Select PLL Clock Output  */
//	SYSCON->MAINCLKUEN    = 0;					/* Update MCLK Clock Source */
//	SYSCON->MAINCLKUEN    = 1;					/* Update MCLK Clock Source */
//	while (!(SYSCON->MAINCLKUEN & 1));			/* Wait Until Updated       */
//
//	HW::GPIO->NOT0 = 1<<12;
//
////	SYSCON->SYSAHBCLKDIV  = SYSAHBCLKDIV_Val;
//
//	SYSCON->UARTCLKDIV = 1;
//	SWM->U0_RXD = 26;
//	SWM->U0_TXD = 16;
//
//	DMA->SRAMBASE = DmaTable;
//	DMA->CTRL = 1;
//
//	HW::GPIO->NOT0 = 1<<12;
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
	using namespace HW;

	InitVectorTable();
	Init_time();

	SYSCON->SYSAHBCLKCTRL |= HW::CLK::WWDT_M;
	SYSCON->PDRUNCFG &= ~(1<<6); // WDTOSC_PD = 0
	SYSCON->WDTOSCCTRL = (1<<5)|59; // 600kHz/60 = 10kHz = 0.1ms

#ifndef _DEBUG

	WDT->TC = 2500; // * 0.4ms
	WDT->MOD = 0x3;
	ResetWDT();

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
	static byte i = 0;

	static Deb db(false, 20);

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL(	);
		CALL(	);
		CALL(	);
		CALL(	);
		CALL( 	);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetCRC(const void *data, u32 len)
{
	union { u32 *pd; u16 *pw; u8 *pb; };

	pb = (byte*)data;

//	byte * const p = (byte*)HW::CRC->B;

	HW::CRC->MODE = 0x15;
	HW::CRC->SEED = 0xFFFF;

	u32 dl = len>>2;
	u32 wl = (len&3)>>1;
	u32 bl = (len&1);

	for ( ; dl > 0; dl--) 
	{
		HW::CRC->D = *(pd++);
	};

	for ( ; wl > 0; wl--) 
	{
		HW::CRC->W = *(pw++);
	};

	for ( ; bl > 0; bl--) 
	{
		HW::CRC->B = *(pb++);
	};

	//for ( ; len > 0; len--) 
	//{
	//	HW::CRC->B = *(pb++);
	//};

	return HW::CRC->SUM;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
