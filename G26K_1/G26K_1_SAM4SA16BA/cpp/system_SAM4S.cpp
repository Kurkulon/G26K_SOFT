#include "core.h"
//#include "hardware.h"
//#include "time.h"

#pragma O3
#pragma Otime

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

static __irq void MemFaultHandler()
{
	__breakpoint(0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void BusFaultHandler()
{
	__breakpoint(0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void UsageFaultHandler()
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
	VectorTableInt[4] = MemFaultHandler;
	VectorTableInt[5] = BusFaultHandler;
	VectorTableInt[6] = UsageFaultHandler;

	CM4::SCB->VTOR = (u32)VectorTableInt;

	__enable_irq();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern "C" void SystemInit (void)
{
	using namespace HW;

	//MATRIX->SCFG[0] |= 0x00060000;
	//MATRIX->SCFG[1] |= 0x00060000;

	InitVectorTable();

	EFC0->FMR = 0x0500;
	EFC1->FMR = 0x0500;

	PMC->PCER0 = PID::PIOA_M|PID::PIOB_M|PID::UART0_M|PID::UART1_M|PID::USART0_M|PID::USART1_M|PID::TWI1_M;
	//PMC->PCER1 = PID::ADC_M|PID::PWM_M|PID::DMAC_M;

								// 1  8 7  4 3  0 9  6 5  2 1  8 7  4 3  0								
	PIOA->PDR =		0x01607660;	// 0000 0001 0110 0000 0111 0110 0110 0000
	PIOA->ABCDSR1 =	0x00000000;	// 0000 0000 0000 0000 0000 0000 0000 0000		
	PIOA->ABCDSR2 =	0x00000000;	// 0000 0000 0000 0000 0000 0000 0000 0000		
	PIOA->PER = 	0xFF9F899F;	// 1111 1111 1001 1111 1000 1001 1001 1111		
	PIOA->OER = 	0xFF9F891F;	// 1111 1111 1001 1111 1000 1001 0001 1111		
	PIOA->ODR = 	0x00000000;	// 0000 0000 0000 0000 0000 0000 0000 0000		
	//PIOA->MDDR =	0x00000000;	// 0000 0000 0000 0000 0000 0000 0000 0000		
	PIOA->OWER =	0x0000000F;	// 0000 0000 0000 0000 0000 0000 0000 1111		

								// 1  8 7  4 3  0 9  6 5  2 1  8 7  4 3  0								
	PIOB->PDR =		0x0000003C;	// 0000 0000 0000 0000 0000 0000 0011 1100
	PIOB->ABCDSR1 =	0x00000000;	// 0000 0000 0000 0000 0000 0000 0000 0000		
	PIOB->ABCDSR2 =	0x00000000;	// 0000 0000 0000 0000 0000 0000 0000 0000		
	PIOB->PER = 	0x00006D03;	// 0000 0000 0000 0000 0110 1101 0000 0011		
	PIOB->OER = 	0x00006D00;	// 0000 0000 0000 0000 0110 1101 0000 0000		
	PIOB->ODR = 	0x00000003;	// 0000 0000 0000 0000 0000 0000 0000 0011		
	//PIOB->MDDR =	0x00000000;	// 0000 0000 0000 0000 0000 0000 0000 0000		
	//PIOB->OWER =	0x00000000;	// 0000 0000 0000 0000 0000 0000 0000 0000		


	MATRIX->SYSIO = (1<<4)|(1<<5)|(1<<10)|(1<<11); // PB4 PB5 PB10 PB11

//	WDT->MR = (((1000 * 32768 / 128 + 500) / 1000) & 0xFFF) | 0x0FFF6000;
	HW::WDT->MR = 0x8000;	// Disable Watchdog

	PMC->MOR	= 0x01370102; while ((PMC->SR & 1) == 0) {};

	PMC->PLLAR	= 0x20030101; while ((PMC->SR & 2) == 0) {}; 

//	PMC->MCKR = (PMC->MCKR & 3) | 0x10; while ((PMC->SR & 8) == 0) {}; 

	PMC->MCKR = 0x02; while ((PMC->SR & 8) == 0) {};

	PMC->PCK[1] = 0x64;
	PMC->SCER = 0x200;



// Init ADC

	PMC->PCER0 = PID::ADC_M;

	ADC->MR = 0x2F3FFF80;
	ADC->ACR = 0x00;
	ADC->CHER = 0x0008;
	ADC->CR = 10;


	PIOA->PUDR = 1<<20;
	PIOA->PPDDR = 1<<20;

	__asm { DSB };
	__asm { ISB };

	CMCC->CTRL = 1;		// cache enable
	CMCC->MAINT0 = 1;	// invalidate all cache entries

	__asm { DSB };
	__asm { ISB };

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
