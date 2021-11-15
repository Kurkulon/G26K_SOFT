#include "hardware.h"

#include <bfrom.h>
#include <sys\exception.h>
//#include <cdefBF592-A.h>
//#include <ccblkfn.h>


// 5 - PF5 - 
// 6 - PF6 - 
// 7 - PF7 - 
// 8 - PF8 - SpiFlashSelect - Main Loop
// 36 - PG4 - FIRE_PPI_ISR
// 37 - PG5
// 38 - PG6
// 39 - PG7

// Вектора прерываний
// IVG7		- 
// IVG8 	- DMA0 (PPI)
// IVG9 	- PORTF PF4 SYNC
// IVG10 	- GPTIMER0 FIRE
// IVG11 	- GPTIMER2 RTT
// IVG12 	- TWI


// CoreTimer - PPI delay

// TIMER0 	- Fire
// TIMER1 	- PPI CLK
// TIMER2 	- RTT

// UART0	- 
// SPI0		- Boot flash
// SPI1 	- 
// TWI		- 

#define IVG_EMULATION		0
#define IVG_RESET			1
#define IVG_NMI				2
#define IVG_EXEPTIONS		3
#define IVG_HW_ERROR		5
#define IVG_CORETIMER		6
#define IVG_PORTF_SYNC		7
#define IVG_PORTF_SHAFT		8
#define IVG_GPTIMER2_RTT	9
#define IVG_PPI_DMA0		10
#define IVG_PORTG_ROT		11
//#define IVG_TWI				12
//#define IVG_GPTIMER0_FIRE	10

#define PPI_BUF_NUM 6

#define PIN_SHAFT		6
#define PIN_SYNC		4
#define PIN_ROT			5
#define BM_SHAFT		(1 << PIN_SHAFT)	
#define BM_SYNC			(1 << PIN_SYNC)
#define BM_ROT			(1 << PIN_ROT)

#define PIN_GAIN_EN		1
#define PIN_GAIN_0		0
#define PIN_GAIN_1		2
#define PIN_GAIN_2		3
#define PIN_A0			4

#define GAIN_EN		(1 << PIN_GAIN_EN)	
#define GAIN_0		(1 << PIN_GAIN_0)
#define GAIN_1		(1 << PIN_GAIN_1)
#define GAIN_2		(1 << PIN_GAIN_2)
#define A0			(1 << PIN_A0)

#define GAIN_M0		(0)
#define GAIN_M1		(GAIN_EN)
#define GAIN_M2		(GAIN_EN|GAIN_0)	
#define GAIN_M3		(GAIN_EN|GAIN_1)	
#define GAIN_M4		(GAIN_EN|GAIN_1|GAIN_0)	
#define GAIN_M5		(GAIN_EN|GAIN_2)	
#define GAIN_M6		(GAIN_EN|GAIN_2|GAIN_0)	
#define GAIN_M7		(GAIN_EN|GAIN_2|GAIN_1)	
#define GAIN_M8		(GAIN_EN|GAIN_2|GAIN_1|GAIN_0)

#define StartPPI()	{ *pTIMER_ENABLE = TIMEN1; }
#define StopPPI()	{ *pTIMER_DISABLE = TIMDIS1; }

#define StartFire()	{ *pTIMER_ENABLE = TIMEN0; }
#define StopFire()	{ *pTIMER_DISABLE = TIMDIS0; }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitIVG(u32 IVG, u32 PID, void (*EVT)())
{
	if (IVG <= 15)
	{
		*(pEVT0 + IVG) = (void*)EVT;
		*pIMASK |= 1<<IVG; 

		if (IVG > 6)
		{
			IVG -= 7;

			byte n = PID/8;
			byte i = (PID&7)*4;

			pSIC_IAR0[n] = (pSIC_IAR0[n] & ~(0xF<<i)) | (IVG<<i);

			*pSIC_IMASK |= 1<<PID;
		};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void LowLevelInit();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_PLL()
{
	u32 SIC_IWR1_reg;                /* backup SIC_IWR1 register */

	/* use Blackfin ROM SysControl() to change the PLL */
    ADI_SYSCTRL_VALUES sysctrl = {	VRCTL_VALUE,
									PLLCTL_VALUE,		/* (25MHz CLKIN x (MSEL=16))::CCLK = 400MHz */
									PLLDIV_VALUE,		/* (400MHz/(SSEL=4))::SCLK = 100MHz */
									PLLLOCKCNT_VALUE,
									PLLSTAT_VALUE };

	/* use the ROM function */
	bfrom_SysControl( SYSCTRL_WRITE | SYSCTRL_PLLCTL | SYSCTRL_PLLDIV, &sysctrl, 0);

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LowLevelInit()
{
	Init_PLL();

								//  5  2 1  8 7  4 3  0								
	*pPORTF_MUX = 0x000F;		//  0000 0000 0000 1111
	*pPORTG_MUX = 0xFF00;		//  1111 1111 0000 0000

	*pPORTF_FER = 0x1E0F;		//  0001 1110 0000 1111
	*pPORTG_FER = 0xFF00;		//  1111 1111 0000 0000

	*pPORTFIO_DIR = 0x01A0;		//  0000 0001 1010 0000
	*pPORTGIO_DIR = 0x00FF;		//  0000 0000 1111 1111

	*pPORTFIO_INEN = 0x0000;	//  0000 0000 0000 0000
	*pPORTGIO_INEN = 0x0000;	//  0000 0000 0000 0000

	*pPORTGIO = 0;
	*pPORTFIO = 0;

	*pPORTFIO_POLAR = 0;
	*pPORTFIO_EDGE = 0;
	*pPORTFIO_BOTH = 0;
	*pPORTFIO_MASKA = 0;
	*pPORTFIO_MASKB = 0;

	*pPORTGIO_POLAR = 0;
	*pPORTGIO_EDGE = 0;
	*pPORTGIO_BOTH = 0;
	*pPORTGIO_MASKA = 0;
	*pPORTGIO_MASKB = 0;

#ifndef _DEBUG
	//*pWDOG_CNT = MS2CLK(10);
	//*pWDOG_CTL = WDEV_RESET|WDEN;
#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
	LowLevelInit();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{

//	spi.Update();

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
