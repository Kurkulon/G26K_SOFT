#include "hardware.h"

#include <bfrom.h>
#include <sys\exception.h>
//#include <cdefBF592-A.h>
//#include <ccblkfn.h>

#include "list.h"

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

#define IVG_CORETIMER		6
#define IVG_PPI_DMA0		8
#define IVG_PORTF_SYNC		9
#define IVG_GPTIMER0_FIRE	10
#define IVG_GPTIMER2_RTT	11
#define IVG_TWI				12
#define IVG_PORTF_SHAFT		13

#define PPI_BUF_NUM 4

#define PIN_GAIN_EN		1
#define PIN_GAIN_0		3
#define PIN_GAIN_1		2
#define PIN_GAIN_2		0
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

byte bitGain[16] = {GAIN_M0, GAIN_M1, GAIN_M2, GAIN_M3, GAIN_M4, GAIN_M5, GAIN_M6, GAIN_M7, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8, GAIN_M8 };

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static DSCPPI *curDscPPI = 0;
static DSCPPI *lastDscPPI = 0;

//static u16 ppi_buf[PPI_BUF_LEN][PPI_BUF_NUM];

static DSCPPI ppidsc[PPI_BUF_NUM];
//static u16 startIndPPI = 0;
//static u16 endIndPPI = 0;g118

u16 ppiClkDiv = NS2CLK(400);
u16 ppiLen = 16;
u16 ppiOffset = 19;

u32 ppiDelay = US2CCLK(10);

u32 mmsec = 0; // 0.1 ms

#pragma instantiate List<DSCPPI>
static List<DSCPPI> freePPI;
static List<DSCPPI> readyPPI;

static ReqDsp01 dspVars;

u32 shaftCount = 0;
u32 shaftMMSEC = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetDspVars(const ReqDsp01 *v)
{
	//u32 i = cli();

	dspVars = *v;

	//sti(i);

	ppiClkDiv = dspVars.st * (NS2CLK(50));

	if (ppiClkDiv == 0) ppiClkDiv = 1;

	ppiLen = dspVars.sl;

	if (ppiLen < 16) ppiLen = 16;

	ppiDelay = dspVars.sd * (NS2CLK(50));
	
	if (ppiDelay > US2CLK(500)) ppiDelay = US2CLK(500);

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(RTT_ISR)
{
	if (*pTIMER_STATUS & TIMIL2)
	{
		*pTIMER_STATUS = TIMIL2; 

		*pPORTGIO_TOGGLE = 1<<6;

		mmsec++;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitRTT()
{
	*pTIMER2_CONFIG = PERIOD_CNT|PWM_OUT|OUT_DIS|IRQ_ENA;
	*pTIMER2_PERIOD = US2CLK(100);

	InitIVG(IVG_GPTIMER2_RTT, PID_GP_Timer_2, RTT_ISR);

	*pTIMER_ENABLE = TIMEN2;
}

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

DSCPPI* GetDscPPI()
{
	return readyPPI.Get();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeDscPPI(DSCPPI* dsc)
{
	freePPI.Add(dsc);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void ReadPPI()
{
	curDscPPI = freePPI.Get();

	*pTIMER_DISABLE = TIMDIS1;

	if (curDscPPI != 0)
	{
		curDscPPI->busy = false;

		*pTIMER1_CONFIG = PERIOD_CNT|PWM_OUT;
		*pTIMER1_PERIOD = curDscPPI->clkdiv = ppiClkDiv;
		*pTIMER1_WIDTH = curDscPPI->clkdiv>>1;

		*pDMA0_START_ADDR = curDscPPI->data+(curDscPPI->offset = ppiOffset);
		*pDMA0_X_COUNT = curDscPPI->len = ppiLen;
		*pDMA0_X_MODIFY = 2;

		*pDMA0_CONFIG = FLOW_STOP|DI_EN|WDSIZE_16|SYNC|WNR|DMAEN;
		*pPPI_CONTROL = FLD_SEL|PORT_CFG|POLC|DLEN_12|XFR_TYPE|PORT_EN;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(PPI_ISR)
{
	if (*pDMA0_IRQ_STATUS & (DMA_DONE|DMA_ERR))
	{
		*pDMA0_IRQ_STATUS = DMA_DONE|DMA_ERR;
		*pPPI_CONTROL = 0;
		*pDMA0_CONFIG = 0;

		*pTIMER_DISABLE = TIMDIS1;
		*pPORTFIO_CLEAR = 1<<9; // SYNC 

		curDscPPI->busy = false;
		readyPPI.Add(curDscPPI);

		ReadPPI();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(FIRE_PPI_ISR)
{
	if (*pTIMER_STATUS & TIMIL0)
	{
		*pTIMER_STATUS = TIMIL0; 
		*pPORTGIO_TOGGLE = 1<<6;

		//if (curDscPPI != 0 && !curDscPPI->ready) { *pTIMER_ENABLE = TIMEN1; };
	};
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(TIMER_PPI_ISR)
{
	*pTIMER_ENABLE = TIMEN1;
	*pTCNTL = 0;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SYNC_ISR)
{
	*pPORTGIO_TOGGLE = 1<<7;
	*pPORTFIO_CLEAR = 1<<4;
	*pPORTGIO_TOGGLE = 1<<7;

	*pTIMER_ENABLE = TIMEN0; // Start Fire Pulse

	if (curDscPPI != 0)
	{
		if (!curDscPPI->busy)
		{
			curDscPPI->busy = true;

			u32 t = mmsec; //dspVars.mmsecTime;

			curDscPPI->data[1] = t;
			curDscPPI->data[2] = t>>16;

			t = shaftMMSEC; //dspVars.hallTime;

			curDscPPI->data[3] = t;
			curDscPPI->data[4] = t>>16;

			curDscPPI->data[5] = dspVars.motoCount;
			curDscPPI->data[6] = shaftCount;

			if (curDscPPI->delay == 0)
			{ 
				*pTCNTL = 0;
				*pTIMER_ENABLE = TIMEN1;
			}
			else
			{
				*pTSCALE = 0;
				*pTCOUNT = curDscPPI->delay;
				*pTCNTL = TINT|TMPWR|TMREN;
			};
		};
	}
	else
	{
		ReadPPI();
	};

}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void InitPPI()
//{
//	*pTIMER_DISABLE = TIMDIS1;
//	*pTIMER1_CONFIG = PERIOD_CNT|PWM_OUT;
//	*pTIMER1_PERIOD = 5;
//	*pTIMER1_WIDTH = 2;
//
//	*pPPI_CONTROL = 0;
//	*pDMA0_CONFIG = 0;
//
//	*pEVT8 = (void*)PPI_ISR;
//	*pIMASK |= EVT_IVG8; 
//	*pSIC_IMASK |= 1<<PID_DMA0_PPI;
//
//	//InitIVG(0, 0, PPI_ISR);
//	//*pEVT6 = (void*)TIMER_PPI_ISR;
//	//*pIMASK |= EVT_IVTMR; 
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitFire()
{
	for (u16 i = 0; i < ArraySize(ppidsc); i++)
	{
		DSCPPI &dsc = ppidsc[i];

		dsc.busy = false;

		freePPI.Add(&dsc);
	};

	*pPORTF_FER |= PF9;
	*pPORTF_MUX &= ~PF9;

	*pTIMER_DISABLE = TIMDIS1;
	*pTIMER1_CONFIG = PERIOD_CNT|PWM_OUT;
	*pTIMER1_PERIOD = 5;
	*pTIMER1_WIDTH = 2;

	*pPPI_CONTROL = 0;
	*pDMA0_CONFIG = 0;

	InitIVG(IVG_PPI_DMA0, PID_DMA0_PPI, PPI_ISR);

	InitIVG(IVG_PORTF_SYNC, PID_Port_F_Interrupt_A, SYNC_ISR);

	*pPORTFIO_INEN |= 1<<4;
	*pPORTFIO_EDGE |= 1<<4;
	*pPORTFIO_BOTH &= ~(1<<4);
	*pPORTFIO_CLEAR = 1<<4;
	*pPORTFIO_MASKA = 1<<4;

	ReadPPI();

	InitIVG(IVG_GPTIMER0_FIRE, PID_GP_Timer_0, FIRE_PPI_ISR);

	*pTIMER0_CONFIG = /*PERIOD_CNT|*/PWM_OUT|PULSE_HI/*|IRQ_ENA*/;
	*pTIMER0_PERIOD = MS2CLK(1000) / 500 / 4;
	*pTIMER0_WIDTH = US2CLK(1);
	*pTIMER_ENABLE = TIMEN0; 
	
	InitIVG(IVG_CORETIMER, 0, TIMER_PPI_ISR);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SHAFT_ISR)
{
	*pPORTFIO_CLEAR = 1<<6;

	shaftCount++;

	shaftMMSEC = mmsec;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitShaft()
{
	InitIVG(IVG_PORTF_SHAFT, PID_Port_F_Interrupt_B, SHAFT_ISR);

	*pPORTFIO_INEN |= 1<<6;
	*pPORTFIO_EDGE |= 1<<6;
	*pPORTFIO_BOTH &= ~(1<<6);
	*pPORTFIO_CLEAR = 1<<6;
	*pPORTFIO_MASKB = 1<<6;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 twiWriteCount = 0;
static u16 twiReadCount = 0;
static u16 *twiWriteData = 0;
static u16 *twiReadData = 0;

EX_INTERRUPT_HANDLER(TWI_ISR)
{
	if (*pTWI_INT_STAT & RCVSERV)
	{
		*twiReadData++ = *pTWI_RCV_DATA16;
		twiReadCount++;


		*pTWI_INT_STAT = RCVSERV;
	};
	
	if (*pTWI_INT_STAT & XMTSERV)
	{
		*pTWI_XMT_DATA16 = *twiWriteData++;
		twiWriteCount--;


		*pTWI_INT_STAT = XMTSERV;
	};
	
	if (*pTWI_INT_STAT & MERR)
	{
		

		*pTWI_INT_STAT = MERR;
	};

	if (*pTWI_INT_STAT & MCOMP)
	{
		*pTWI_MASTER_CTL = 0;
		*pTWI_MASTER_STAT = 0x3E;
		*pTWI_FIFO_CTL = XMTFLUSH|RCVFLUSH;

		*pTWI_INT_MASK = 0;
		*pTWI_INT_STAT = MCOMP;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitTWI()
{
	*pTWI_CONTROL = TWI_ENA | 10;
	*pTWI_CLKDIV = (8<<8)|12;
	*pTWI_INT_MASK = 0;
	*pTWI_MASTER_ADDR = 0;

//	*pSIC_IAR3 = (*pSIC_IAR3 & ~0xF)|8;

	InitIVG(IVG_TWI, PID_TWI, TWI_ISR);
	//*pEVT12 = (void*)TWI_ISR;
	//*pIMASK |= EVT_IVG12; 
	//*pSIC_IMASK |= 1<<24;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void WriteTWI(void *src, u16 len)
{
	*pTWI_MASTER_CTL = 0;
	*pTWI_MASTER_STAT = 0x3E;
	*pTWI_FIFO_CTL = XMTINTLEN;

	twiWriteData = (u16*)src;
	twiWriteCount = len>>1;
	*pTWI_MASTER_ADDR = 11;
	*pTWI_XMT_DATA16 = *twiWriteData++;	twiWriteCount--;
	*pTWI_INT_MASK = XMTSERV|MERR|MCOMP;
	*pTWI_MASTER_CTL = (len<<6)|FAST|MEN;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ReadTWI(void *dst, u16 len)
{
	*pTWI_MASTER_CTL = 0;
	*pTWI_MASTER_STAT = 0x3E;

	twiReadData = (u16*)dst;
	twiReadCount = 0;
	*pTWI_MASTER_ADDR = 11;
	*pTWI_FIFO_CTL = RCVINTLEN;
	*pTWI_INT_MASK = RCVSERV|MERR|MCOMP;
	*pTWI_MASTER_CTL = (len<<6)|MDIR|FAST|MEN;
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

	*pPORTFIO_EDGE = 0;
	*pPORTFIO_BOTH = 0;
	*pPORTFIO_MASKA = 0;
	*pPORTFIO_MASKB = 0;

	*pPORTGIO_EDGE = 0;
	*pPORTGIO_BOTH = 0;
	*pPORTGIO_MASKA = 0;
	*pPORTGIO_MASKB = 0;

#ifndef _DEBUG
	*pWDOG_CNT = MS2CLK(10);
	*pWDOG_CTL = WDEV_RESET|WDEN;
#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetGain(byte v) 
{
	*pPORTGIO = (*pPORTGIO & ~0xF) | bitGain[v&0xF];
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

	InitRTT();

//	InitPPI();

//	InitTWI();

	InitFire();

	InitShaft();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{

//	spi.Update();

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
