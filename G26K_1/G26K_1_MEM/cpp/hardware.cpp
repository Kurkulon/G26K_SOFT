#include "types.h"
#include "core.h"
#include "time.h"
#include "COM_DEF.h"
#include "CRC16_8005.h"

#include "hardware.h"

#ifdef WIN32

#include <windows.h>
#include <Share.h>
#include <conio.h>
#include <stdarg.h>
#include <stdio.h>
#include <intrin.h>
#include "CRC16_CCIT.h"

static HANDLE handleNandFile;
static const char nameNandFile[] = "NAND_FLASH_STORE.BIN";

static byte nandChipSelected = 0;

static u64 curNandFilePos = 0;
static u64 curNandFileBlockPos = 0;
static u32 curBlock = 0;
static u16 curPage = 0;
static u16 curCol = 0;

static OVERLAPPED	_overlapped;
static u32			_ovlReadedBytes = 0;
static u32			_ovlWritenBytes = 0;

static void* nandEraseFillArray;
static u32 nandEraseFillArraySize = 0;
static byte nandReadStatus = 0x41;
static u32 lastError = 0;


static byte fram_I2c_Mem[0x10000];
static byte fram_SPI_Mem[0x40000];

static bool fram_spi_WREN = false;

static u16 crc_ccit_result = 0;

#else

#pragma O3
#pragma Otime

#endif 

#define GEAR_RATIO 12.25

const u16 pulsesPerHeadRoundFix4 = GEAR_RATIO * 6 * 16;

const u16 testNandChipMask = 0xFFFF;

static volatile u32 shaftCounter = 0;
static volatile u32 shaftPrevTime = 0;
static volatile u32 shaftCount = 0;
static volatile u32 shaftTime = 0;
u16 shaftRPS = 0;
volatile u16 curShaftCounter = 0;

static void I2C_Init();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53	

	// Test Pins
	// 37	- PB15	- SPI_Handler
	// 42	- PC12
	// 43	- PC13
	// 52	- PA16
	// 57	- PC17
	// 58	- PC18
	// 59	- PC19
	// 66	- PB18	- ManRcvIRQ sync true
	// 74	- PA24	- ManRcvIRQ
	// 75	- PA25	- main loop



	#define GEN_MCK		0
	#define GEN_32K		1
	#define GEN_25M		2
	#define GEN_1M		3
	//#define GEN_500K	4

	#define	NAND_DMACH		0
	#define	COM1_DMACH		1
	#define	COM2_DMACH		2
	#define	COM3_DMACH		3
	#define	SPI_DMACH_TX	4
	#define	SPI_DMACH_RX	5
	#define	DSP_DMACH		30
	#define	CRC_DMACH		31

	#define I2C			HW::I2C3
	#define PIO_I2C		HW::PIOA 
	#define PIN_SDA		22 
	#define PIN_SCL		23 
	#define SDA			(1<<PIN_SDA) 
	#define SCL			(1<<PIN_SCL) 

	__align(16) T_HW::DMADESC DmaTable[32];
	__align(16) T_HW::DMADESC DmaWRB[32];

	#define SPI				HW::SPI0
	#define PIO_SPCK		HW::PIOA
	#define PIO_MOSI		HW::PIOA
	#define PIO_MISO		HW::PIOA
	#define PIO_CS			HW::PIOB

	#define PIN_SPCK		9
	#define PIN_MOSI		8 
	#define PIN_MISO		10 
	#define PIN_CS0			10 
	#define PIN_CS1			11

	#define SPCK			(1<<PIN_SPCK) 
	#define MOSI			(1<<PIN_MOSI) 
	#define MISO			(1<<PIN_MISO) 
	#define CS0				(1<<PIN_CS0) 
	#define CS1				(1<<PIN_CS1) 

	#define SPI_IRQ			SERCOM0_0_IRQ
	//#define SPI_PID			PID_USIC1

	#define Pin_SPI_IRQ_Set() HW::PIOB->BSET(15)		
	#define Pin_SPI_IRQ_Clr() HW::PIOB->BCLR(15)		


	#define EVENT_NAND_1	0
	//#define EVENT_NAND_2	1
	//#define EVENT_NAND_3	2
	#define EVENT_MANR_1	3
	#define EVENT_MANR_2	4

	#define ManTT			HW::TC0
	#define ManIT			HW::TC1
	//#define 				HW::TC2
	//#define 				HW::TC3
	//#define 				HW::TC4
	//#define 				HW::TC5
	//#define 				HW::TC6
	//#define 				HW::TC7

	#define nandTCC			HW::TCC0
	//#define				HW::TCC1
	#define ManRT			HW::TCC2
	#define SyncTmr			HW::TCC3
	#define RotTmr			HW::TCC4
	//#define MltTmr			HW::TCC4

	#define MT(v)			(v)
	#define BAUD2CLK(x)		((u32)(1000000/x+0.5))

	#define MANT_IRQ		TC0_IRQ
	#define MANR_IRQ		TCC2_1_IRQ
	//#define MANR_EXTINT		11
	#define MANR_EXTINT		7

	
	#define PIO_MANCH		HW::PIOC
	#define PIN_L1			25 
	#define PIN_H1			24 
	#define PIN_L2			27 
	#define PIN_H2			26

	#define L1				(1UL<<PIN_L1)
	#define H1				(1UL<<PIN_H1)
	#define L2				(1UL<<PIN_L2)
	#define H2				(1UL<<PIN_H2)

	#define PIO_MANCHRX		HW::PIOA
	#define PIO_RXD			HW::PIOB
	#define PIN_MANCHRX		11
	#define PIN_RXD			23
	#define MANCHRX			(1UL<<PIN_MANCHRX)
	#define RXD				(1UL<<PIN_RXD)



	//#define ManRxd()		((PIO_MANCH->IN >> PIN_MANCHRX) & 1)

	#define Pin_ManRcvIRQ_Set()	HW::PIOA->BSET(24)
	#define Pin_ManRcvIRQ_Clr()	HW::PIOA->BCLR(24)

	#define Pin_ManTrmIRQ_Set()	HW::PIOB->BSET(21)		
	#define Pin_ManTrmIRQ_Clr()	HW::PIOB->BCLR(21)		

	#define Pin_ManRcvSync_Set()	HW::PIOB->BSET(18)		
	#define Pin_ManRcvSync_Clr()	HW::PIOB->BCLR(18)		

	#define PIO_WP			HW::PIOB 
	#define PIO_FLREADY		HW::PIOB
	#define PIO_FCS			HW::PIOB

	#define PIN_WP			24 
	#define PIN_FLREADY		25 
	#define PIN_FCS0		2 
	#define PIN_FCS1		9 
	#define PIN_FCS2		8 
	#define PIN_FCS3		7 
	#define PIN_FCS4		6 
	#define PIN_FCS5		5 
	#define PIN_FCS6		4 
	#define PIN_FCS7		3 

	#define WP				(1<<PIN_WP) 
	#define FLREADY			(1UL<<PIN_FLREADY) 
	#define FCS0			(1<<PIN_FCS0) 
	#define FCS1			(1<<PIN_FCS1) 
	#define FCS2			(1<<PIN_FCS2) 
	#define FCS3			(1<<PIN_FCS3) 
	#define FCS4			(1<<PIN_FCS4) 
	#define FCS5			(1<<PIN_FCS5) 
	#define FCS6			(1<<PIN_FCS6) 
	#define FCS7			(1<<PIN_FCS7) 

	#define PIO_ENVCORE		HW::PIOC
	#define PIN_ENVCORE		14 
	#define ENVCORE			(1<<PIN_ENVCORE) 
	
	#define PIN_RESET		10
	#define PIO_RESET		HW::PIOC
	#define RESET			(1<<PIN_RESET)

	#define PIN_SYNC		12
	#define PIN_ROT			14

	#define SYNC			(1<<PIN_SYNC)
	#define ROT				(1<<PIN_ROT)
	#define PIO_SYNC		HW::PIOB
	#define PIO_ROT			HW::PIOB
	#define US2SRT(v)		(v)


	#define PIN_SHAFT		13
	#define SHAFT			(1<<PIN_SHAFT)
	#define PIO_SHAFT		HW::PIOB
	#define SHAFT_EXTINT	13
	#define IRQ_SHAFT		(EIC_0_IRQ+SHAFT_EXTINT)

	#define Pin_ShaftIRQ_Set()		//HW::P6->BSET(6);
	#define Pin_ShaftIRQ_Clr()		//HW::P6->BCLR(6);

	#define PIO_NAND_DATA	HW::PIOA
//	#define PIO_WP			HW::PIOB 
	#define PIO_ALE			HW::PIOB 
	#define PIO_CLE			HW::PIOB 
	#define PIO_WE_RE		HW::PIOB 
	//#define PIO_RB			HW::PIOC
	//#define PIO_FCS			HW::PIOC

	#define PIN_WE			30 
	#define PIN_RE			31 
	#define PIN_ALE			0 
	#define PIN_CLE			1 

	#define WE				(1UL<<PIN_WE) 
	#define RE				(1UL<<PIN_RE) 
	#define ALE				(1UL<<PIN_ALE) 
	#define CLE				(1UL<<PIN_CLE) 
//	#define WP				(1<<6) 
//	#define FCS1			(1<<5) 
//	#define FCS2			(1<<6) 
//	#define RB				(1<<7) 

	#define PIO_USART0		HW::PIOB 
	#define PIO_USART1		HW::PIOB 
	#define PIO_USART2		HW::PIOC 

	#define PIN_UTXD0		21 
	#define PIN_URXD0		20 
	#define PIN_UTXD1		16
	#define PIN_URXD1		17
	#define PIN_UTXD2		7 
	#define PIN_URXD2		6 

	#define UTXD0			(1<<PIN_UTXD0) 
	#define URXD0			(1<<PIN_URXD0) 
	#define UTXD1			(1<<PIN_UTXD1) 
	#define URXD1			(1<<PIN_URXD1) 
	#define UTXD2			(1<<PIN_UTXD2) 
	#define URXD2			(1<<PIN_URXD2) 


#elif defined(CPU_XMC48) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Test Pins
	// 63	- P2.13	- main loop
	// 64	- P2.12 - SPI_Handler
	// 76	- P2.6	- ManTrmIRQ
	// 77	- P5.7	- UpdateMan	
	// 78	- P5.6	- 	
	// 79	- P5.5	- 	
	// 80	- P5.4
	// 81	- P5.3
	// 83	- P5.1	- ManRcvIRQ 
	// 95	- P6.6	- ShaftIRQ
	// 96	- P6.5	- CRC_CCITT_DMA
	// 99	- P6.2	- I2C_Handler 
	// 100	- P6.1
	// 101	- P6.0
	// 104	- P1.12
	// 109	- P1.3
	// 111	- P1.2
	// 111	- P1.1
	// 113	- P1.9
	// 115	- P1.7
	// 116	- P1.6
	// 119	- P4.5
	// 120	- P4.4
	// 121	- P4.3
	// 122	- P4.2
	// 123	- P4.1
	// 124	- P1.0
	// 131	- P3.4
	// 132	- P3.3
	// 137	- P0.13
	// 138	- P0.12


	#define	NAND_DMA				HW::GPDMA0
	#define	NAND_DMACH				HW::GPDMA0_CH7
	#define	NAND_DMA_CHEN			(0x101<<7)
	#define	NAND_DMA_CHST			(1<<7)

	#define	DSP_DMA					HW::GPDMA0
	#define	DSP_DMACH				HW::GPDMA0_CH6
	#define	DSP_DMA_CHEN			(0x101<<6)
	#define	DSP_DMA_CHST			(1<<6)

	#define	SPI_DMA					HW::GPDMA0
	#define	SPI_DMACH				HW::GPDMA0_CH5
	#define	SPI_DMA_CHEN			(0x101<<5)
	#define	SPI_DMA_CHDIS			(0x100<<5)
	#define	SPI_DMA_CHST			(1<<5)
	#define	SPI_DLR					(1)
	#define	SPI_DLR_LNEN			(1<<SPI_DLR)

	#define	NAND_MEMCOPY_DMA		HW::GPDMA0
	#define	NAND_MEMCOPY_DMACH		HW::GPDMA0_CH4
	#define	NAND_MEMCOPY_DMA_CHEN	(0x101<<4)
	#define	NAND_MEMCOPY_DMA_CHST	(1<<4)

	#define	CRC_DMA					HW::GPDMA1
	#define	CRC_DMACH				HW::GPDMA1_CH2
	#define	CRC_DMA_CHEN			(0x101<<2)
	#define	CRC_FCE					HW::FCE_KE3

	#define I2C						HW::USIC2_CH0
	#define PIO_I2C					HW::P5
	#define PIN_SDA					0 
	#define PIN_SCL					2 
	#define SDA						(1<<PIN_SDA) 
	#define SCL						(1<<PIN_SCL) 
	#define I2C_IRQ					USIC2_0_IRQn
	#define I2C_PID					PID_USIC2

	#define ManRT					HW::CCU41_CC43		//HW::CCU41_CC42
	#define ManTT					HW::CCU41_CC40
	#define ManCCU					HW::CCU41
	#define ManCCU_PID				PID_CCU41
	#define ManTmr					HW::CCU41_CC41
	#define ManRT_PSC				3
	#define MT(v)					((u16)((MCK_MHz*(v)+(1<<ManRT_PSC)/2)/(1<<ManRT_PSC)))
	#define BAUD2CLK(x)				((u32)((MCK/8.0)/x+0.5))

	#define MANT_IRQ				CCU41_0_IRQn
	#define MANR_IRQ				CCU41_2_IRQn

	#define ManCCU_GIDLC			(CCU4_CS1I | CCU4_CS3I | CCU4_SPRB)	// (CCU4_CS1I | CCU4_CS2I | CCU4_SPRB)
	#define ManCCU_GCSS				(CCU4_S1SE | CCU4_S3SE)				// (CCU4_S1SE | CCU4_S2SE)
	#define ManRT_INS				(CC4_EV0IS(2) | CC4_EV0EM_BOTH_EDGES | CC4_LPF0M_7CLK)
	#define ManRT_SRS				CC4_POSR(3)

	#define ManT1					HW::CCU80_CC80
	#define ManT2					HW::CCU80_CC81
	#define ManT3					HW::CCU80_CC82
	
	//#define ManT_L1					HW::CCU81_CC80
	//#define ManT_H1					HW::CCU81_CC81
	//#define ManT_L2					HW::CCU81_CC82
	//#define ManT_H2					HW::CCU81_CC83

	#define ManT_CCUCON				SCU_GENERAL_CCUCON_GSC80_Msk
	#define ManT_CCU8				HW::CCU80
	#define ManT_CCU8_PID			PID_CCU80
	#define MANT_CCU8_IRQ			CCU80_0_IRQn
	#define ManT_CCU8_GIDLC			(CCU8_CS0I | CCU8_CS1I | CCU8_CS2I | CCU8_SPRB)	// (CCU4_CS1I | CCU4_CS2I | CCU4_SPRB)
	#define ManT_CCU8_GIDLS			(CCU8_SS0I | CCU8_SS1I | CCU8_SS2I | CCU8_CPRB)	// (CCU4_CS1I | CCU4_CS2I | CCU4_SPRB)
	#define ManT_CCU8_GCSS			(CCU8_S0SE | CCU8_S1SE | CCU8_S2SE)				// (CCU4_S1SE | CCU4_S2SE)
	#define ManT_PSC				3					// 0.04us
	#define US2MT(v)				((u16)((MCK_MHz*(v)+((1<<(ManT_PSC))/2))/(1<<(ManT_PSC))))
	#define ManT_SET_PR(v)			{ ManT1->PRS = (v); ManT2->PRS = (v); ManT3->PRS = (v); }
	#define ManT_SET_CR(v)			{ ManT1->CR2S = (v); ManT2->CR1S = (v); ManT2->CR2S = (v); ManT3->CR1S = (v);}
	#define ManT1_PSL				(0) 
	#define ManT1_CHC				(CC8_OCS2 | CC8_OCS3)			
	#define ManT2_CHC				(CC8_OCS2 | CC8_OCS3)			
	#define ManT3_CHC				(CC8_OCS2)			
	#define ManT_OUT_GCSS			(CCU8_S1ST1C | CCU8_S1ST2S | CCU8_S1ST2S)
	#define ManT_OUT_GCSC			(CCU8_S0ST2C)


	#define PIO_MANCH				HW::P0
	#define PIN_L1					0 
	#define PIN_H1					1 
	#define PIN_L2					9 
	#define PIN_H2					10

	#define L1						(1UL<<PIN_L1)
	#define H1						(1UL<<PIN_H1)
	#define L2						(1UL<<PIN_L2)
	#define H2						(1UL<<PIN_H2)

	//#define ManRxd()				((PIO_MANCH->IN >> PIN_MANCHRX) & 1)

	#define Pin_ManRcvIRQ_Set()		HW::P5->BSET(1);
	#define Pin_ManRcvIRQ_Clr()		HW::P5->BCLR(1);

	#define Pin_ManTrmIRQ_Set()		HW::P2->BSET(6);
	#define Pin_ManTrmIRQ_Clr()		HW::P2->BCLR(6);

	#define Pin_ManRcvSync_Set()			
	#define Pin_ManRcvSync_Clr()			

	#define PIO_WP					HW::P5 
	#define PIO_FLREADY				HW::P15
	#define PIO_FCS					HW::P3

	#define PIN_WP					10 
	#define PIN_FLREADY				7 
	#define PIN_FCS0				13 
	#define PIN_FCS1				2 
	#define PIN_FCS2				12 
	#define PIN_FCS3				11 
	#define PIN_FCS4				10 
	#define PIN_FCS5				9 
	#define PIN_FCS6				8 
	#define PIN_FCS7				7 

	#define WP						(1<<PIN_WP) 
	#define FLREADY					(1<<PIN_FLREADY) 
	#define FCS0					(1<<PIN_FCS0) 
	#define FCS1					(1<<PIN_FCS1) 
	#define FCS2					(1<<PIN_FCS2) 
	#define FCS3					(1<<PIN_FCS3) 
	#define FCS4					(1<<PIN_FCS4) 
	#define FCS5					(1<<PIN_FCS5) 
	#define FCS6					(1<<PIN_FCS6) 
	#define FCS7					(1<<PIN_FCS7) 

	#define PIO_ENVCORE				HW::P2
	#define PIN_ENVCORE				11 
	#define ENVCORE					(1<<PIN_ENVCORE) 
	
	#define PIN_RESET				11
	#define PIO_RESET				HW::P0
	#define RESET					(1<<PIN_RESET)

	#define PIN_SYNC				14
	#define PIN_ROT					15

	#define SYNC					(1<<PIN_SYNC)
	#define ROT						(1<<PIN_ROT)
	#define PIO_SYNC				HW::P0
	#define PIO_ROT					HW::P0

	#define SyncTmr					HW::CCU40_CC41
	#define RotTmr					HW::CCU40_CC40
	#define SyncRotCCU				HW::CCU40
	#define SyncRotCCU_PID			PID_CCU40
	#define Sync_GCSS				CCU4_S1SE	
	#define Rot_GCSS				CCU4_S0SE
	#define SyncRot_GIDLC			(CCU4_S0I|CCU4_S1I|CCU4_PRB)
	#define SyncRot_PSC				8					//1.28us
	#define SyncRot_DIV				(1<<SyncRot_PSC)	

	#define US2SRT(v)				(((MCK_MHz*(v)+SyncRot_DIV/2)/SyncRot_DIV))

	#define PIN_SHAFT				6
	#define SHAFT					(1<<PIN_SHAFT)
	#define PIO_SHAFT				HW::P0
	#define IRQ_SHAFT				ERU0_0_IRQn

	#define Pin_ShaftIRQ_Set()		HW::P6->BSET(6);
	#define Pin_ShaftIRQ_Clr()		HW::P6->BCLR(6);

	#define SPI						HW::USIC1_CH0
	#define	SPI_INPR				(0)
	#define PIO_SPCK				HW::P5
	#define PIO_MOSI				HW::P2
	#define PIO_MISO				HW::P2
	#define PIO_CS					HW::P5

	#define Pin_SPI_IRQ_Set()		HW::P2->BSET(12);
	#define Pin_SPI_IRQ_Clr()		HW::P2->BCLR(12);

	#define PIN_SPCK				8 
	#define PIN_MOSI				14 
	#define PIN_MISO				15 
	#define PIN_CS0					9 
	#define PIN_CS1					11

	#define SPCK					(1<<PIN_SPCK) 
	#define MOSI					(1<<PIN_MOSI) 
	#define MISO					(1<<PIN_MISO) 
	#define CS0						(1<<PIN_CS0) 
	#define CS1						(1<<PIN_CS1) 

	#define SPI_IRQ					USIC1_5_IRQn
	#define SPI_PID					PID_USIC1


	/*******************************************************************************
	 * MACROS
	 *******************************************************************************/
	#define	OFI_FREQUENCY        (24000000UL)  /**< 24MHz Backup Clock (fOFI) frequency. */
	#define OSI_FREQUENCY        (32768UL)    /**< 32KHz Internal Slow Clock source (fOSI) frequency. */  

	#define XMC4800_F144x2048

	#define CHIPID_LOC ((uint8_t *)0x20000000UL)

	#define PMU_FLASH_WS          (NS2CLK(30))	//(0x3U)

	#define OSCHP_FREQUENCY			(25000000U)
	#define FOSCREF					(2500000U)
	#define VCO_NOM					(400000000UL)
	#define VCO_IN_MAX				(5000000UL)

	#define DELAY_CNT_50US_50MHZ  (2500UL)
	#define DELAY_CNT_150US_50MHZ (7500UL)
	#define DELAY_CNT_50US_48MHZ  (2400UL)
	#define DELAY_CNT_50US_72MHZ  (3600UL)
	#define DELAY_CNT_50US_96MHZ  (4800UL)
	#define DELAY_CNT_50US_120MHZ (6000UL)
	#define DELAY_CNT_50US_144MHZ (7200UL)

	#define SCU_PLL_PLLSTAT_OSC_USABLE  (SCU_PLL_PLLSTAT_PLLHV_Msk | \
										 SCU_PLL_PLLSTAT_PLLLV_Msk | \
										 SCU_PLL_PLLSTAT_PLLSP_Msk)


	#define USB_PDIV (4U)
	#define USB_NDIV (79U)


	/*
	//    <o> Backup clock calibration mode
	//       <0=> Factory calibration
	//       <1=> Automatic calibration
	//    <i> Default: Automatic calibration
	*/
	#define FOFI_CALIBRATION_MODE 1
	#define FOFI_CALIBRATION_MODE_FACTORY 0
	#define FOFI_CALIBRATION_MODE_AUTOMATIC 1

	/*
	//    <o> Standby clock (fSTDBY) source selection
	//       <0=> Internal slow oscillator (32768Hz)
	//       <1=> External crystal (32768Hz)
	//    <i> Default: Internal slow oscillator (32768Hz)
	*/
	#define STDBY_CLOCK_SRC 0
	#define STDBY_CLOCK_SRC_OSI 0
	#define STDBY_CLOCK_SRC_OSCULP 1

	/*
	//    <o> PLL clock source selection
	//       <0=> External crystal
	//       <1=> Internal fast oscillator
	//    <i> Default: External crystal
	*/
	#define PLL_CLOCK_SRC 0
	#define PLL_CLOCK_SRC_EXT_XTAL 0
	#define PLL_CLOCK_SRC_OFI 1

	#define PLL_CON1(ndiv, k2div, pdiv) (((ndiv) << SCU_PLL_PLLCON1_NDIV_Pos) | ((k2div) << SCU_PLL_PLLCON1_K2DIV_Pos) | ((pdiv) << SCU_PLL_PLLCON1_PDIV_Pos))

	/* PLL settings, fPLL = 288MHz */
	#if PLL_CLOCK_SRC == PLL_CLOCK_SRC_EXT_XTAL

		#define PLL_K2DIV	((VCO_NOM/MCK)-1)
		#define PLL_PDIV	(((OSCHP_FREQUENCY-VCO_IN_MAX)*2/VCO_IN_MAX+1)/2)
		#define PLL_NDIV	((MCK*(PLL_K2DIV+1)*2/(OSCHP_FREQUENCY/(PLL_PDIV+1))+1)/2-1) // (7U) 

		#define VCO ((OSCHP_FREQUENCY / (PLL_PDIV + 1UL)) * (PLL_NDIV + 1UL))

	#else /* PLL_CLOCK_SRC == PLL_CLOCK_SRC_EXT_XTAL */

		#define PLL_PDIV (1U)
		#define PLL_NDIV (23U)
		#define PLL_K2DIV (0U)

		#define VCO ((OFI_FREQUENCY / (PLL_PDIV + 1UL)) * (PLL_NDIV + 1UL))

	#endif /* PLL_CLOCK_SRC == PLL_CLOCK_SRC_OFI */

	#define PLL_K2DIV_24MHZ  ((VCO / OFI_FREQUENCY) - 1UL)
	#define PLL_K2DIV_48MHZ  ((VCO / 48000000U) - 1UL)
	#define PLL_K2DIV_72MHZ  ((VCO / 72000000U) - 1UL)
	#define PLL_K2DIV_96MHZ  ((VCO / 96000000U) - 1UL)
	#define PLL_K2DIV_120MHZ ((VCO / 120000000U) - 1UL)

	//#define SCU_CLK_CLKCLR_ENABLE_USBCLK SCU_CLK_CLKCLR_USBCDI_Msk
	//#define SCU_CLK_CLKCLR_ENABLE_MMCCLK SCU_CLK_CLKCLR_MMCCDI_Msk
	//#define SCU_CLK_CLKCLR_ENABLE_ETHCLK SCU_CLK_CLKCLR_ETH0CDI_Msk
	//#define SCU_CLK_CLKCLR_ENABLE_EBUCLK SCU_CLK_CLKCLR_EBUCDI_Msk
	//#define SCU_CLK_CLKCLR_ENABLE_CCUCLK SCU_CLK_CLKCLR_CCUCDI_Msk

	//#define SCU_CLK_SYSCLKCR_SYSSEL_OFI      (0U << SCU_CLK_SYSCLKCR_SYSSEL_Pos)
	//#define SCU_CLK_SYSCLKCR_SYSSEL_PLL      (1U << SCU_CLK_SYSCLKCR_SYSSEL_Pos)

	//#define SCU_CLK_USBCLKCR_USBSEL_USBPLL   (0U << SCU_CLK_USBCLKCR_USBSEL_Pos)
	//#define SCU_CLK_USBCLKCR_USBSEL_PLL      (1U << SCU_CLK_USBCLKCR_USBSEL_Pos)

	//#define SCU_CLK_ECATCLKCR_ECATSEL_USBPLL (0U << SCU_CLK_ECATCLKCR_ECATSEL_Pos)
	//#define SCU_CLK_ECATCLKCR_ECATSEL_PLL    (1U << SCU_CLK_ECATCLKCR_ECATSEL_Pos)

	#define SCU_CLK_WDTCLKCR_WDTSEL_OFI      (0U << SCU_CLK_WDTCLKCR_WDTSEL_Pos)
	//#define SCU_CLK_WDTCLKCR_WDTSEL_STANDBY  (1U << SCU_CLK_WDTCLKCR_WDTSEL_Pos)
	//#define SCU_CLK_WDTCLKCR_WDTSEL_PLL      (2U << SCU_CLK_WDTCLKCR_WDTSEL_Pos)

	//#define SCU_CLK_EXTCLKCR_ECKSEL_SYS      (0U << SCU_CLK_EXTCLKCR_ECKSEL_Pos)
	//#define SCU_CLK_EXTCLKCR_ECKSEL_USBPLL   (2U << SCU_CLK_EXTCLKCR_ECKSEL_Pos)
	//#define SCU_CLK_EXTCLKCR_ECKSEL_PLL      (3U << SCU_CLK_EXTCLKCR_ECKSEL_Pos)

	//#define EXTCLK_PIN_P0_8  (1)
	//#define EXTCLK_PIN_P1_15 (2)

	#define __CLKSET    (0x00000000UL)
	#define __SYSCLKCR  (0x00010000UL)
	#define __CPUCLKCR  (0x00000000UL)
	#define __PBCLKCR   (0x00000000UL)
	#define __CCUCLKCR  (0x00000000UL)
	#define __WDTCLKCR  (0x00000000UL)
	#define __EBUCLKCR  (0x00000003UL)
	#define __USBCLKCR  (0x00010005UL)
	#define __ECATCLKCR (0x00000001UL)

	#define __EXTCLKCR (0x01200003UL)
	//#define __EXTCLKPIN (0U)

	//#define ENABLE_PLL \
	//	(((__SYSCLKCR & SCU_CLK_SYSCLKCR_SYSSEL_Msk) == SCU_CLK_SYSCLKCR_SYSSEL_PLL) || \
	//	 ((__ECATCLKCR & SCU_CLK_ECATCLKCR_ECATSEL_Msk) == SCU_CLK_ECATCLKCR_ECATSEL_PLL) || \
	//	 ((__CLKSET & SCU_CLK_CLKSET_EBUCEN_Msk) != 0) || \
	//	 (((__CLKSET & SCU_CLK_CLKSET_USBCEN_Msk) != 0) && ((__USBCLKCR & SCU_CLK_USBCLKCR_USBSEL_Msk) == SCU_CLK_USBCLKCR_USBSEL_PLL)) || \
	//	 (((__CLKSET & SCU_CLK_CLKSET_WDTCEN_Msk) != 0) && ((__WDTCLKCR & SCU_CLK_WDTCLKCR_WDTSEL_Msk) == SCU_CLK_WDTCLKCR_WDTSEL_PLL)))

	//#define ENABLE_USBPLL \
	//	(((__ECATCLKCR & SCU_CLK_ECATCLKCR_ECATSEL_Msk) == SCU_CLK_ECATCLKCR_ECATSEL_USBPLL) || \
	//	 (((__CLKSET & SCU_CLK_CLKSET_USBCEN_Msk) != 0) && ((__USBCLKCR & SCU_CLK_USBCLKCR_USBSEL_Msk) == SCU_CLK_USBCLKCR_USBSEL_USBPLL)) || \
	//	 (((__CLKSET & SCU_CLK_CLKSET_MMCCEN_Msk) != 0) && ((__USBCLKCR & SCU_CLK_USBCLKCR_USBSEL_Msk) == SCU_CLK_USBCLKCR_USBSEL_USBPLL)))

	#define SLAD(v)		((v)&0xffff)                /*!< USIC_CH PCR_IICMode: SLAD (Bitfield-Mask: 0xffff)           */
	#define ACK00     	(1<<16UL)                    /*!< USIC_CH PCR_IICMode: ACK00 (Bit 16)                         */
	#define STIM      	(1<<17UL)                    /*!< USIC_CH PCR_IICMode: STIM (Bit 17)                          */
	#define SCRIEN    	(1<<18UL)                    /*!< USIC_CH PCR_IICMode: SCRIEN (Bit 18)                        */
	#define RSCRIEN   	(1<<19UL)                    /*!< USIC_CH PCR_IICMode: RSCRIEN (Bit 19)                       */
	#define PCRIEN    	(1<<20UL)                    /*!< USIC_CH PCR_IICMode: PCRIEN (Bit 20)                        */
	#define NACKIEN   	(1<<21UL)                    /*!< USIC_CH PCR_IICMode: NACKIEN (Bit 21)                       */
	#define ARLIEN    	(1<<22UL)                    /*!< USIC_CH PCR_IICMode: ARLIEN (Bit 22)                        */
	#define SRRIEN    	(1<<23UL)                    /*!< USIC_CH PCR_IICMode: SRRIEN (Bit 23)                        */
	#define ERRIEN    	(1<<24UL)                    /*!< USIC_CH PCR_IICMode: ERRIEN (Bit 24)                        */
	#define SACKDIS   	(1<<25UL)                    /*!< USIC_CH PCR_IICMode: SACKDIS (Bit 25)                       */
	#define HDEL(v)		(((v)&0xF)<<26UL)                    /*!< USIC_CH PCR_IICMode: HDEL (Bit 26)                          */
	#define ACKIEN    	(1<<30UL)                    /*!< USIC_CH PCR_IICMode: ACKIEN (Bit 30)                        */
	//#define MCLK      	(1<<31UL)                    /*!< USIC_CH PCR_IICMode: MCLK (Bit 31)                          */

	#define SLSEL         (0x1UL)                   /*!< USIC_CH PSR_IICMode: SLSEL (Bitfield-Mask: 0x01)            */
	#define WTDF          (0x2UL)                   /*!< USIC_CH PSR_IICMode: WTDF (Bitfield-Mask: 0x01)             */
	#define SCR           (0x4UL)                   /*!< USIC_CH PSR_IICMode: SCR (Bitfield-Mask: 0x01)              */
	#define RSCR          (0x8UL)                   /*!< USIC_CH PSR_IICMode: RSCR (Bitfield-Mask: 0x01)             */
	#define PCR           (0x10UL)                  /*!< USIC_CH PSR_IICMode: PCR (Bitfield-Mask: 0x01)              */
	#define NACK          (0x20UL)                  /*!< USIC_CH PSR_IICMode: NACK (Bitfield-Mask: 0x01)             */
	#define ARL           (0x40UL)                  /*!< USIC_CH PSR_IICMode: ARL (Bitfield-Mask: 0x01)              */
	#define SRR           (0x80UL)                  /*!< USIC_CH PSR_IICMode: SRR (Bitfield-Mask: 0x01)              */
	#define ERR           (0x100UL)                 /*!< USIC_CH PSR_IICMode: ERR (Bitfield-Mask: 0x01)              */
	#define ACK           (0x200UL)                 /*!< USIC_CH PSR_IICMode: ACK (Bitfield-Mask: 0x01)              */
	#define RSIF          (0x400UL)                 /*!< USIC_CH PSR_IICMode: RSIF (Bitfield-Mask: 0x01)             */
	#define DLIF          (0x800UL)                 /*!< USIC_CH PSR_IICMode: DLIF (Bitfield-Mask: 0x01)             */
	#define TSIF          (0x1000UL)                /*!< USIC_CH PSR_IICMode: TSIF (Bitfield-Mask: 0x01)             */
	#define TBIF          (0x2000UL)                /*!< USIC_CH PSR_IICMode: TBIF (Bitfield-Mask: 0x01)             */
	#define RIF           (0x4000UL)                /*!< USIC_CH PSR_IICMode: RIF (Bitfield-Mask: 0x01)              */
	#define AIF           (0x8000UL)                /*!< USIC_CH PSR_IICMode: AIF (Bitfield-Mask: 0x01)              */
	#define BRGIF         (0x10000UL)               /*!< USIC_CH PSR_IICMode: BRGIF (Bitfield-Mask: 0x01)            */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define TDF_MASTER_SEND				(0U << 8U)
	#define TDF_SLAVE_SEND				(1U << 8U)
	#define TDF_MASTER_RECEIVE_ACK   	(2U << 8U)
	#define TDF_MASTER_RECEIVE_NACK  	(3U << 8U)
	#define TDF_MASTER_START         	(4U << 8U)
	#define TDF_MASTER_RESTART      	(5U << 8U)
	#define TDF_MASTER_STOP         	(6U << 8U)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define I2C__SCTR (SDIR(1) | TRM(3) | FLE(0x3F) | WLE(7))

	#define I2C__CCR (MODE(4))

	#define I2C__BRG (DCTQ(24)|SCLKCFG(0))

	#define I2C__DX0CR (DSEL(1) | INSW(0) | DFEN(1) | DSEN(1) | DPOL(0) | SFSEL(1) | CM(0) | DXS(0))
	#define I2C__DX1CR (DSEL(0) | INSW(0) | DFEN(1) | DSEN(1) | DPOL(0) | SFSEL(1) | CM(0) | DXS(0))
	#define I2C__DX2CR (DSEL(0) | INSW(0) | DFEN(0) | DSEN(0) | DPOL(0) | SFSEL(0) | CM(0) | DXS(0))
	#define I2C__DX3CR (DSEL(0) | INSW(0) | DFEN(0) | DSEN(0) | DPOL(0) | SFSEL(0) | CM(0) | DXS(0))

	#define I2C__PCR (STIM)

	#define I2C__FDR ((1024 - (((MCK + 400000/2) / 400000 + 8) / 16)) | DM(1))

	#define I2C__TCSR (TDEN(1)|TDSSM(1))


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define MSLSEN    	(0x1UL)         	/*!< USIC_CH PCR_SSCMode: MSLSEN (Bitfield-Mask: 0x01)           */
	#define SELCTR    	(0x2UL)         	/*!< USIC_CH PCR_SSCMode: SELCTR (Bitfield-Mask: 0x01)           */
	#define SELINV    	(0x4UL)         	/*!< USIC_CH PCR_SSCMode: SELINV (Bitfield-Mask: 0x01)           */
	#define FEM       	(0x8UL)         	/*!< USIC_CH PCR_SSCMode: FEM (Bitfield-Mask: 0x01)              */
	#define CTQSEL1(v)	(((v)&3)<<4)		/*!< USIC_CH PCR_SSCMode: CTQSEL1 (Bitfield-Mask: 0x03)          */
	#define PCTQ1(v)	(((v)&3)<<6)    	/*!< USIC_CH PCR_SSCMode: PCTQ1 (Bitfield-Mask: 0x03)            */
	#define DCTQ1(v)	(((v)&0x1F)<<8)		/*!< USIC_CH PCR_SSCMode: DCTQ1 (Bitfield-Mask: 0x1f)            */
	#define PARIEN    	(0x2000UL)      	/*!< USIC_CH PCR_SSCMode: PARIEN (Bitfield-Mask: 0x01)           */
	#define MSLSIEN   	(0x4000UL)      	/*!< USIC_CH PCR_SSCMode: MSLSIEN (Bitfield-Mask: 0x01)          */
	#define DX2TIEN   	(0x8000UL)      	/*!< USIC_CH PCR_SSCMode: DX2TIEN (Bitfield-Mask: 0x01)          */
	#define SELO(v)		(((v)&0xFF)<<16)	/*!< USIC_CH PCR_SSCMode: SELO (Bitfield-Mask: 0xff)             */
	#define TIWEN     	(0x1000000UL)   	/*!< USIC_CH PCR_SSCMode: TIWEN (Bitfield-Mask: 0x01)            */
	#define SLPHSEL   	(0x2000000UL)   	/*!< USIC_CH PCR_SSCMode: SLPHSEL (Bitfield-Mask: 0x01)          */
	#define MCLK      	(0x80000000UL)  	/*!< USIC_CH PCR_SSCMode: MCLK (Bitfield-Mask: 0x01)             */

	#define MSLS      	(0x1UL)           	/*!< USIC_CH PSR_SSCMode: MSLS (Bitfield-Mask: 0x01)             */
	#define DX2S      	(0x2UL)           	/*!< USIC_CH PSR_SSCMode: DX2S (Bitfield-Mask: 0x01)             */
	#define MSLSEV    	(0x4UL)           	/*!< USIC_CH PSR_SSCMode: MSLSEV (Bitfield-Mask: 0x01)           */
	#define DX2TEV    	(0x8UL)           	/*!< USIC_CH PSR_SSCMode: DX2TEV (Bitfield-Mask: 0x01)           */
	#define PARERR    	(0x10UL)          	/*!< USIC_CH PSR_SSCMode: PARERR (Bitfield-Mask: 0x01)           */
	#define RSIF      	(0x400UL)         	/*!< USIC_CH PSR_SSCMode: RSIF (Bitfield-Mask: 0x01)             */
	#define DLIF      	(0x800UL)         	/*!< USIC_CH PSR_SSCMode: DLIF (Bitfield-Mask: 0x01)             */
	#define TSIF      	(0x1000UL)        	/*!< USIC_CH PSR_SSCMode: TSIF (Bitfield-Mask: 0x01)             */
	#define TBIF      	(0x2000UL)        	/*!< USIC_CH PSR_SSCMode: TBIF (Bitfield-Mask: 0x01)             */
	#define RIF       	(0x4000UL)        	/*!< USIC_CH PSR_SSCMode: RIF (Bitfield-Mask: 0x01)              */
	#define AIF       	(0x8000UL)        	/*!< USIC_CH PSR_SSCMode: AIF (Bitfield-Mask: 0x01)              */
	#define BRGIF     	(0x10000UL)       	/*!< USIC_CH PSR_SSCMode: BRGIF (Bitfield-Mask: 0x01)            */

	#define SPI__SCTR (SDIR(1) | TRM(1) | FLE(0x3F) | WLE(7))

	#define SPI__CCR (MODE(1))

	#define SPI__BRG (SCLKCFG(2)|CTQSEL(0)|DCTQ(1)|PCTQ(3)|CLKSEL(0))

	#define SPI__DX0CR (DSEL(2) | INSW(1) | DFEN(0) | DSEN(0) | DPOL(0) | SFSEL(1) | CM(0) | DXS(0))
	#define SPI__DX1CR (DSEL(0) | INSW(0) | DFEN(1) | DSEN(1) | DPOL(0) | SFSEL(1) | CM(0) | DXS(0))
	#define SPI__DX2CR (DSEL(0) | INSW(0) | DFEN(0) | DSEN(0) | DPOL(0) | SFSEL(0) | CM(0) | DXS(0))
	#define SPI__DX3CR (DSEL(0) | INSW(0) | DFEN(0) | DSEN(0) | DPOL(0) | SFSEL(0) | CM(0) | DXS(0))

	#define SPI__PCR (MSLSEN | SELINV |  TIWEN | MCLK | CTQSEL1(0) | PCTQ1(0) | DCTQ1(0))

	#define SPI__BAUD (4000000)

	#define SPI__FDR ((1024 - ((MCK + SPI__BAUD/2) / SPI__BAUD + 1) / 2) | DM(1))

	#define SPI__BAUD2FDR(v) ((1024 - ((MCK + (v)/2) / (v) + 1) / 2) | DM(1))

	#define SPI__TCSR (TDEN(1)|HPCMD(0))

	static void delay(u32 cycles) { for(volatile u32 i = 0UL; i < cycles ;++i) { __nop(); }}

#elif defined(WIN32) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define BAUD2CLK(x)				(x)
	#define MT(v)					(v)

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//static void InitVectorTable();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

__forceinline 	void EnableVCORE()	{ PIO_ENVCORE->CLR(ENVCORE); 	}
__forceinline 	void DisableVCORE()	{ PIO_ENVCORE->SET(ENVCORE); 	}
				void EnableDSP()	{ PIO_RESET->CLR(RESET); 		}
				void DisableDSP()	{ PIO_RESET->SET(RESET); 		}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*----------------------------------------------------------------------------
  Initialize the system
 *----------------------------------------------------------------------------*/
#ifndef WIN32

extern "C" void SystemInit()
{
	//u32 i;
	using namespace CM4;
	using namespace HW;

//	__breakpoint(0);

	#ifdef CPU_SAME53	

		HW::PIOA->DIRSET = (1<<27)|(1<<25)|(1<<24)|(1<<16)|0xFF;
		HW::PIOA->CLR((1<<27)|(1<<25)|(1<<24)|(1<<16));

		PIO_USART0->SetWRCONFIG(UTXD0|URXD0, PORT_PMUX(3)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_PULLEN);
		PIO_USART1->SetWRCONFIG(UTXD1|URXD1, PORT_PMUX(2)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_PULLEN);
		PIO_USART2->SetWRCONFIG(UTXD2|URXD2, PORT_PMUX(2)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_PULLEN);

		HW::PIOB->DIRSET = (1<<15)|(1<<18)|(1<<24)|(1<<21);
		//HW::PIOB->WRCONFIG = ((1<<17)>>16) |PORT_HWSEL_HI|PORT_PMUX(11)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX;

		HW::PIOC->DIRSET = (1<<5)|(1<<10)|(1<<11)|(1<<12)|(1<<13)|(1<<14)|(1<<15)|(1<<17)|(1<<18)|(1<<19)|(1<<21);
		HW::PIOC->SET((1<<15));

		HW::PIOA->BCLR(25);
		HW::PIOA->BSET(25);
		HW::PIOA->BCLR(25);

		OSCCTRL->XOSC[1] = XOSC_ENABLE|XOSC_ONDEMAND; // RUNSTDBY|ENABLE

		HW::PIOA->BSET(25);

		OSCCTRL->DPLL[0].CTRLA = 0; while ((OSCCTRL->DPLL[0].SYNCBUSY & DPLLSYNCBUSY_ENABLE) != 0);

		OSCCTRL->DPLL[0].CTRLB = DPLL_REFCLK_XOSC1|DPLL_DIV(24);	// 0x70010; // XOSC clock source division factor 50 = 2*(DIV+1), XOSC clock reference
		OSCCTRL->DPLL[0].RATIO = DPLL_LDR((MCK*2+500000)/1000000-1)|DPLL_LDRFRAC(0);	// 47; // Loop Divider Ratio = 200, Loop Divider Ratio Fractional Part = 0

		OSCCTRL->DPLL[0].CTRLA = DPLL_ONDEMAND|DPLL_ENABLE; 

		HW::PIOA->BCLR(25);

		HW::GCLK->GENCTRL[GEN_MCK] = GCLK_DIV(0)|GCLK_SRC_DPLL0|GCLK_GENEN;

		HW::PIOA->BSET(25);

		HW::MCLK->AHBMASK |= AHB_DMAC;
		HW::DMAC->CTRL = 0;
		HW::DMAC->CTRL = DMAC_SWRST;
		HW::DMAC->DBGCTRL = DMAC_DBGRUN;
		HW::DMAC->BASEADDR	= DmaTable;
		HW::DMAC->WRBADDR	= DmaWRB;
		HW::DMAC->CTRL = DMAC_DMAENABLE|DMAC_LVLEN0|DMAC_LVLEN1|DMAC_LVLEN2|DMAC_LVLEN3;

		HW::PIOA->BCLR(25);

		if ((CMCC->SR & CMCC_CSTS) == 0)
		{
			CMCC->CTRL = CMCC_CEN;
		};

		HW::PIOA->BSET(25);
		HW::PIOA->BCLR(25);
		HW::PIOA->BSET(25);
		HW::PIOA->BCLR(25);

	#elif defined(CPU_XMC48)

		__disable_irq();

//		__DSB();
		__enable_irq();

		HW::FLASH0->FCON = FLASH_FCON_IDLE_Msk | PMU_FLASH_WS;

		/* enable PLL */
		SCU_PLL->PLLCON0 &= ~(SCU_PLL_PLLCON0_VCOPWD_Msk | SCU_PLL_PLLCON0_PLLPWD_Msk);

		SCU_OSC->OSCHPCTRL = OSC_MODE(2) | OSC_OSCVAL(OSCHP_FREQUENCY / FOSCREF - 1UL);

			/* select OSC_HP clock as PLL input */
			SCU_PLL->PLLCON2 = 0;

			/* restart OSC Watchdog */
			SCU_PLL->PLLCON0 &= ~SCU_PLL_PLLCON0_OSCRES_Msk;

			while ((SCU_PLL->PLLSTAT & SCU_PLL_PLLSTAT_OSC_USABLE) != SCU_PLL_PLLSTAT_OSC_USABLE);
		//};

		/* Go to bypass the Main PLL */
		SCU_PLL->PLLCON0 |= SCU_PLL_PLLCON0_VCOBYP_Msk;

		/* disconnect Oscillator from PLL */
		SCU_PLL->PLLCON0 |= SCU_PLL_PLLCON0_FINDIS_Msk;

		/* Setup divider settings for main PLL */
		SCU_PLL->PLLCON1 =  PLL_CON1(PLL_NDIV, PLL_K2DIV_24MHZ, PLL_PDIV);

		/* Set OSCDISCDIS */
		SCU_PLL->PLLCON0 |= SCU_PLL_PLLCON0_OSCDISCDIS_Msk;

		/* connect Oscillator to PLL */
		SCU_PLL->PLLCON0 &= ~SCU_PLL_PLLCON0_FINDIS_Msk;

		/* restart PLL Lock detection */
		SCU_PLL->PLLCON0 |= SCU_PLL_PLLCON0_RESLD_Msk;	while ((SCU_PLL->PLLSTAT & SCU_PLL_PLLSTAT_VCOLOCK_Msk) == 0U);

		/* Disable bypass- put PLL clock back */
		SCU_PLL->PLLCON0 &= ~SCU_PLL_PLLCON0_VCOBYP_Msk;	while ((SCU_PLL->PLLSTAT & SCU_PLL_PLLSTAT_VCOBYST_Msk) != 0U);

		/* Before scaling to final frequency we need to setup the clock dividers */
		SCU_CLK->SYSCLKCR = __SYSCLKCR;
		SCU_CLK->PBCLKCR = __PBCLKCR;
		SCU_CLK->CPUCLKCR = __CPUCLKCR;
		SCU_CLK->CCUCLKCR = __CCUCLKCR;
		SCU_CLK->WDTCLKCR = __WDTCLKCR;
		SCU_CLK->EBUCLKCR = __EBUCLKCR;
		SCU_CLK->USBCLKCR = __USBCLKCR;
		SCU_CLK->ECATCLKCR = __ECATCLKCR;
		SCU_CLK->EXTCLKCR = __EXTCLKCR;

		/* PLL frequency stepping...*/
		/* Reset OSCDISCDIS */
		SCU_PLL->PLLCON0 &= ~SCU_PLL_PLLCON0_OSCDISCDIS_Msk;

		SCU_PLL->PLLCON1 = PLL_CON1(PLL_NDIV, PLL_K2DIV_48MHZ, PLL_PDIV);	delay(DELAY_CNT_50US_48MHZ);

		SCU_PLL->PLLCON1 = PLL_CON1(PLL_NDIV, PLL_K2DIV_72MHZ, PLL_PDIV);	delay(DELAY_CNT_50US_72MHZ);

		SCU_PLL->PLLCON1 = PLL_CON1(PLL_NDIV, PLL_K2DIV_96MHZ, PLL_PDIV);	delay(DELAY_CNT_50US_96MHZ);

		SCU_PLL->PLLCON1 = PLL_CON1(PLL_NDIV, PLL_K2DIV_120MHZ, PLL_PDIV);	delay(DELAY_CNT_50US_120MHZ);

		SCU_PLL->PLLCON1 = PLL_CON1(PLL_NDIV, PLL_K2DIV, PLL_PDIV);			delay(DELAY_CNT_50US_144MHZ);

		/* Enable selected clocks */
		SCU_CLK->CLKSET = __CLKSET;

		//P2->ModePin10(	G_PP	);
		//P2->BSET(10);

		P0->ModePin0(	G_PP	);
		P0->ModePin1(	G_PP	);
		P0->ModePin2(	HWIO1	);
		P0->ModePin3(	HWIO1	);
		P0->ModePin4(	HWIO1	);
		P0->ModePin5(	HWIO1	);
		P0->ModePin6(	I1DPD	);
		P0->ModePin7(	HWIO1	);
		P0->ModePin8(	HWIO1	);
		P0->ModePin9(	G_PP	);
		P0->ModePin10(	G_PP	);
		P0->ModePin11(	G_PP	);
		P0->ModePin12(	G_PP	);
		P0->ModePin13(	G_PP	);
		P0->ModePin14(	G_PP	);
		P0->ModePin15(	G_PP	);

		P0->PPS = 0;

		P1->ModePin0(	G_PP	);
		P1->ModePin1(	G_PP	);
		P1->ModePin2(	G_PP	);
		P1->ModePin3(	G_PP	);
		P1->ModePin4(	I2DPU	);
		P1->ModePin5(	A2PP	);
		P1->ModePin6(	G_PP	);
		P1->ModePin7(	G_PP	);
		P1->ModePin8(	G_PP	);
		P1->ModePin9(	G_PP	);
		P1->ModePin10(	I2DPU	);
		P1->ModePin11(	I2DPU	);
		P1->ModePin12(	G_PP	);
		P1->ModePin13(	G_PP	);
		P1->ModePin14(	HWIO1	);
		P1->ModePin15(	HWIO1	);

		P1->PPS = 0;

		P2->ModePin0(	HWIO0	);
		P2->ModePin1(	I1DPD	);
		P2->ModePin2(	I2DPU	);
		P2->ModePin3(	I1DPD	);
		P2->ModePin4(	I1DPD	);
		P2->ModePin5(	A1PP	);
		P2->ModePin6(	G_PP	);
		P2->ModePin7(	A1PP	);
		P2->ModePin8(	A1PP	);
		P2->ModePin9(	A1PP	);
		P2->ModePin10(	G_PP	);
		P2->ModePin11(	G_PP	);
		P2->ModePin12(	G_PP	);
		P2->ModePin13(	G_PP	);
		//P2->ModePin14(	A2PP	);
		//P2->ModePin15(	I2DPU	);

		P2->PPS = 0;

		P3->ModePin0(	HWIO1	);
		P3->ModePin1(	HWIO1	);
		P3->ModePin2(	G_PP	);
		P3->ModePin3(	G_PP	);
		P3->ModePin4(	G_PP	);
		P3->ModePin5(	HWIO1	);
		P3->ModePin6(	HWIO1	);
		P3->ModePin7(	G_PP	);
		P3->ModePin8(	G_PP	);
		P3->ModePin9(	G_PP	);
		P3->ModePin10(	G_PP	);
		P3->ModePin11(	G_PP	);
		P3->ModePin12(	G_PP	);
		P3->ModePin13(	G_PP	);
		P3->ModePin14(	I2DPU	);
		P3->ModePin15(	A2PP	);

		P3->PPS = 0;

		P4->ModePin0(	G_PP	);
		P4->ModePin1(	G_PP	);
		P4->ModePin2(	G_PP	);
		P4->ModePin3(	G_PP	);
		P4->ModePin4(	G_PP	);
		P4->ModePin5(	G_PP	);
		P4->ModePin6(	I2DPU	);
		P4->ModePin7(	A1PP	);

		P4->PPS = 0;

		P5->ModePin0(	HWIO0	);
		P5->ModePin1(	G_PP	);
		P5->ModePin2(	A1OD	);
		P5->ModePin3(	G_PP	);
		P5->ModePin4(	G_PP	);
		P5->ModePin5(	G_PP	);
		P5->ModePin6(	G_PP	);
		P5->ModePin7(	G_PP	);
		//P5->ModePin8(	A2PP	);
		//P5->ModePin9(	G_PP	);
		P5->ModePin10(	G_PP	);
		//P5->ModePin11(	G_PP	);

		P5->PPS = 0;


		P6->ModePin0(	G_PP	);
		P6->ModePin1(	G_PP	);
		P6->ModePin2(	G_PP	);
		P6->ModePin3(	I2DPU	);
		P6->ModePin4(	A2PP	);
		P6->ModePin5(	G_PP	);
		P6->ModePin6(	G_PP	);

		P6->PPS = 0;

		P14->ModePin0(	I2DPU	);
		P14->ModePin1(	I2DPU	);
		P14->ModePin2(	I2DPU	);
		P14->ModePin3(	I2DPU	);
		P14->ModePin4(	I2DPU	);
		P14->ModePin5(	I2DPU	);
		P14->ModePin6(	I2DPU	);
		P14->ModePin7(	I2DPU	);
		P14->ModePin8(	I2DPU	);
		P14->ModePin9(	I2DPU	);
		P14->ModePin12(	I0DNP	);
		P14->ModePin13(	I2DPU	);
		P14->ModePin14(	I2DPU	);
		P14->ModePin15(	I2DPU	);

		P14->PPS = 0;
		P14->PDISC = (1<<0);

		P15->ModePin2(	I2DPU	);
		P15->ModePin3(	I2DPU	);
		P15->ModePin4(	I2DPU	);
		P15->ModePin5(	I2DPU	);
		P15->ModePin6(	I2DPU	);
		P15->ModePin7(	I2DPU	);
		P15->ModePin8(	I2DPU	);
		P15->ModePin9(	I1DPD	);
		P15->ModePin12(	I2DPU	);
		P15->ModePin13(	I2DPU	);
		P15->ModePin14(	I2DPU	);
		P15->ModePin15(	I2DPU	);

		P15->PPS = 0;
		P15->PDISC = 0;

		HW::Peripheral_Enable(PID_DMA0);
		HW::Peripheral_Enable(PID_DMA1);

		//HW::DLR->SRSEL0 = SRSEL0(10,11,0,0,0,0,0,0);
		//HW::DLR->SRSEL1 = SRSEL1(0,0,0,0);

		HW::DLR->DRL0 = DRL0_USIC0_SR0;
		HW::DLR->DRL1 = DRL1_USIC1_SR0;
		HW::DLR->DRL2 = DRL2_USIC0_SR1;
		HW::DLR->DRL9 = DRL9_USIC2_SR1;

		HW::DLR->LNEN |= (1<<0)|(1<<2)|(1<<9);

	#endif

	#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
		CM4::SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
							(3UL << 11*2)  );               /* set CP11 Full Access */
	#else
		CM4::SCB->CPACR = 0;
	#endif

  /* Enable unaligned memory access - SCB_CCR.UNALIGN_TRP = 0 */
	CM4::SCB->CCR &= ~(SCB_CCR_UNALIGN_TRP_Msk);

}

#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define NAND_BASE_PIO_DATA 		AT91C_BASE_PIOA
//#define NAND_PIO_DATA_OFFSET	10
//#define NAND_PIO_DATA_MASK		0xFF
//#define NAND_READ_PACK_BYTES	512
//#define NAND_WRITE_PACK_BYTES	256

#define NAND_CMD_RESET				0xFF
#define NAND_CMD_READ_ID			0x90
#define NAND_CMD_READ_1				0x00
#define NAND_CMD_READ_2				0x30
#define NAND_CMD_RANDREAD_1			0x05
#define NAND_CMD_RANDREAD_2			0xE0
#define NAND_CMD_PAGE_PROGRAM_1		0x80
#define NAND_CMD_PAGE_PROGRAM_2		0x10
#define NAND_CMD_READ_STATUS		0x70
#define NAND_CMD_BLOCK_ERASE_1		0x60
#define NAND_CMD_BLOCK_ERASE_2		0xD0
#define NAND_CMD_READ_PARAM			0xEC
#define NAND_CMD_CHANGE_WRCOL		0x85
#define NAND_CMD_COPYBACK_PROGRAM	0x85
#define NAND_CMD_SET_FEATURES		0xEF
#define NAND_CMD_GET_FEATURES		0xEE



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

NandMemSize nandSize;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct NandParamPage
{
 	char	signature[4];
 	u16		revisionNumber;
 	u16		featuresSupported;
	u16		optionalCommandsSupported;
	byte	_rezerved1[22];
	char	deviceManufacturer[12];
	char	deviceModel[20];
	byte	JEDEC_manufacturer_ID;
	u16		dateCode;
	byte	_rezerved2[13];
	u32		numberDataBytesPerPage;
	u16		numberSpareBytesPerPage;
	byte	_rezerved3[6];
	u32		numberPagesPerBlock;
	u32		numberBlocksPerLUN;
	byte	numberLUNsPerChip;
	byte	numberAddressCycles;
	byte	numberBitsPerCell;
	u16		badBlocksMaximumPerLUN;
	u16		blockEndurans;
	byte	guaranteedValidBlocks;
	u16		blockEnduranceForGuaranteedValidBlocks;
	byte	numberProgramsPerPage;
	byte	_rezerved4[143];
	u16		integrityCRC;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

NandParamPage nandParamPage[NAND_MAX_CHIP];

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

byte	FLADR::chipValidNext[NAND_MAX_CHIP]; // Если чип битый, то по индексу находится следующий хороший чип
byte	FLADR::chipValidPrev[NAND_MAX_CHIP]; // Если чип битый, то по индексу находится предыдущий хороший чип
u32		FLADR::chipOffsetNext[NAND_MAX_CHIP]; // Если чип битый, то по индексу находится смещение адреса на следующий хороший чип
u32		FLADR::chipOffsetPrev[NAND_MAX_CHIP]; // Если чип битый, то по индексу находится смещение адреса на предыдущий хороший чип
byte 	FLADR::COL_BITS;
byte 	FLADR::PAGE_BITS;		
byte 	FLADR::BLOCK_BITS;		
//byte 	FLADR::COL_OFF;		// = 0
byte 	FLADR::PAGE_OFF;	// = COL_BITS;
byte 	FLADR::CHIP_OFF;	// = PAGE_OFF + PAGE_BITS
byte 	FLADR::BLOCK_OFF;	// = CHIP_OFF + NAND_CHIP_BITS 		
u32 	FLADR::COL_MASK;
u32 	FLADR::PAGE_MASK;		
u32 	FLADR::CHIP_MASK;		
u32 	FLADR::BLOCK_MASK;		
u32 	FLADR::RAWPAGE_MASK;	
u32 	FLADR::RAWBLOCK_MASK;	
u64 	FLADR::RAWADR_MASK;
u32 	FLADR::pg; //enum { pg = (1<<NAND_COL_BITS) };
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void	FLADR::InitVaildTables(u16 mask)
{
	u32 blocksize = 1UL << CHIP_OFF;

	for(byte chip = 0; chip < NAND_MAX_CHIP; chip++)
	{
		chipValidNext[chip] = 0;
		chipValidPrev[chip] = 0;

		u32 offset = 0;

		for (byte i = 0; i < NAND_MAX_CHIP; i++)
		{
			byte cn = chip+i; if (cn >= NAND_MAX_CHIP) cn = 0;

			if (mask & (1<<cn))
			{
				chipValidNext[chip] = cn;
				chipOffsetNext[chip] = offset;
				break;
			};

			offset += blocksize;
		};

		offset = 0;

		for (byte i = 0; i < NAND_MAX_CHIP; i++)
		{
			byte cp = chip-i; if (cp >= NAND_MAX_CHIP) cp = NAND_MAX_CHIP - 1;

			if (mask & (1<<cp))
			{
				chipValidPrev[chip] = cp;
				chipOffsetPrev[chip] = offset;
				break;
			};
			
			offset += blocksize;
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte clog2(u32 v) {byte r = 0; while (v>>=1) {r++;}; return r;}

#ifndef WIN32

static u32 chipSelect[NAND_MAX_CHIP] = { FCS0, FCS1, FCS2, FCS3, FCS4, FCS5, FCS6, FCS7 };

#define maskChipSelect (FCS0|FCS1|FCS2|FCS3|FCS4|FCS5|FCS6|FCS7)

#endif

#ifdef CPU_SAME53	

	#define NAND_DIR_IN() { PIO_NAND_DATA->DIRCLR = 0xFF; }
	#define NAND_DIR_OUT() { PIO_NAND_DATA->DIRSET = 0xFF; }

#elif defined(CPU_XMC48)

	volatile byte * const FLC = (byte*)0x60000008;	
	volatile byte * const FLA = (byte*)0x60000010;	
	volatile byte * const FLD = (byte*)0x60000000;	

	#define NAND_DIR_IN() {}
	#define NAND_DIR_OUT() {}

#elif defined(WIN32)

	#define NAND_DIR_IN() {}
	#define NAND_DIR_OUT() {}

#endif



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte NAND_READ()
{
	#ifdef CPU_SAME53	
		PIO_WE_RE->CLR(RE); 
		__nop(); __nop();
		byte v = PIO_NAND_DATA->IN; 
		PIO_WE_RE->SET(RE); 
		return v; 
	#elif defined(CPU_XMC48)
		return *FLD;
	#elif defined(WIN32)
		return 0;
	#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma push
//#pragma O0

void NAND_WRITE(byte data)
{ 
	#ifdef CPU_SAME53	

		PIO_WE_RE->CLR(WE); 
		PIO_NAND_DATA->OUT8(data); 
		__nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
		__nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
		PIO_WE_RE->SET(WE); 

	#elif defined(CPU_XMC48)

		*FLD = data;

	#endif
}       

//#pragma pop

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void NAND_CMD_LATCH(byte cmd)
{ 
	#ifdef CPU_SAME53	

		PIO_CLE->SET(CLE); 
		PIO_ALE->CLR(ALE); 
		NAND_WRITE(cmd); 
		PIO_CLE->CLR(CLE|ALE); 

	#elif defined(CPU_XMC48)

		*FLC = cmd;

	#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void NAND_ADR_LATCH(byte cmd)
{
	#ifdef CPU_SAME53	

		PIO_ALE->SET(ALE); 
		PIO_CLE->CLR(CLE); 
		NAND_WRITE(cmd); 
		PIO_CLE->CLR(CLE|ALE); 

	#elif defined(CPU_XMC48)

		*FLA = cmd;

	#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte NAND_ADR_READ()
{
#ifdef CPU_SAME53	
	PIO_ALE->SET(ALE);
	PIO_CLE->CLR(CLE);
	__nop(); __nop();
	PIO_WE_RE->CLR(RE);
	__nop(); __nop();
	byte v = PIO_NAND_DATA->IN;
	PIO_WE_RE->SET(RE);
	PIO_CLE->CLR(CLE | ALE);
	return v;
#elif defined(CPU_XMC48)
	return *FLA;
#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void NAND_ADR_LATCH_COL(u16 col) 
{ 
	#ifdef CPU_SAME53	

		PIO_CLE->CLR(CLE); 
		PIO_ALE->SET(ALE); 
		NAND_WRITE(col); 
		NAND_WRITE(col>>8); 
		PIO_CLE->CLR(CLE|ALE); 

	#elif defined(CPU_XMC48)

		*FLA = col; *FLA = col >> 8;

	#endif	
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void NAND_ADR_LATCH_ROW(u32 row) 
{ 
	#ifdef CPU_SAME53	

		PIO_CLE->CLR(CLE); 
		PIO_ALE->SET(ALE); 
		NAND_WRITE(row); 
		NAND_WRITE(row>>8); 
		NAND_WRITE(row>>16); 
		PIO_CLE->CLR(CLE|ALE); 

	#elif defined(CPU_XMC48)

		*FLA = row; *FLA = row >> 8; *FLA = row >> 16;

	#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void NAND_ADR_LATCH_COL_ROW(u16 col, u32 row)
{ 
	#ifdef CPU_SAME53	

		PIO_CLE->CLR(CLE); 
		PIO_ALE->SET(ALE); 
		NAND_WRITE(col); 
		NAND_WRITE(col>>8); 
		NAND_WRITE(row); 
		NAND_WRITE(row>>8); 
		NAND_WRITE(row>>16); 
		PIO_CLE->CLR(CLE|ALE);

	#elif defined(CPU_XMC48)

		*FLA = col; *FLA = col >> 8;
		*FLA = row; *FLA = row >> 8; *FLA = row >> 16;

	#endif
} 

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//inline bool FlashReady()
//{
//	return (HW::PIOC->PDSR & (1UL<<31)) != 0;
//}
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//inline bool FlashBusy()
//{
//	return (HW::PIOC->PDSR & (1UL<<31)) == 0;
//}
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool NAND_BUSY() 
{
	#ifdef CPU_SAME53	
		return PIO_FLREADY->TBCLR(PIN_FLREADY); 
	#elif defined(CPU_XMC48)
		return PIO_FLREADY->TBCLR(PIN_FLREADY);
	#elif defined(WIN32)
		return !HasOverlappedIoCompleted(&_overlapped);
	#endif
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline void EnableWriteProtect()
{
#ifndef WIN32
	PIO_WP->CLR(WP);
#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline void DisableWriteProtect()
{
#ifndef WIN32
	PIO_WP->SET(WP);
#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

byte NAND_CmdReadStatus()
{
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_READ_STATUS);
	NAND_DIR_IN();
	return NAND_READ();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool NAND_CmdBusy()
{
	return NAND_BUSY() || ((NAND_CmdReadStatus() & NAND_SR_RDY) == 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_Chip_Select(byte chip) 
{    
#ifndef WIN32
	if(chip < NAND_MAX_CHIP)                   
	{ 				
	#ifdef CPU_SAME53	
		PIO_WE_RE->SET(RE|WE); 
	#endif
		PIO_FCS->SET(maskChipSelect ^ chipSelect[chip]);
		PIO_FCS->CLR(chipSelect[chip]);
	};
#else
	nandChipSelected = chip;
#endif
}                                                                              

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_Chip_Disable() 
{  
#ifndef WIN32
	PIO_FCS->SET(maskChipSelect);

	#ifdef CPU_SAME53	
		NAND_DIR_IN();
		PIO_WE_RE->SET(RE|WE); 
	#endif

#else
	nandChipSelected = ~0;
#endif
}                                                                              

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ResetNand()
{
	while(NAND_BUSY());
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_RESET);
	NAND_CmdReadStatus();
	u32 count = 1000; while (!NAND_BUSY() && (count-- > 0));
	while(NAND_BUSY());
	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool NAND_CheckDataComplete_old()
{
	#ifdef CPU_SAME53	
		return (HW::DMAC->CH[NAND_DMACH].CTRLA & DMCH_ENABLE) == 0;
	#elif defined(CPU_XMC48)
		return (HW::GPDMA1->CHENREG & (1<<3)) == 0;
	#elif defined(WIN32)
		return HasOverlappedIoCompleted(&_overlapped);
	#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_XMC48	

static byte *nandWriteDataSrc = 0;
static byte *nandReadDataDst = 0;
static u16	nandWriteDataLen = 0;
static u16	nandReadDataLen = 0;

#endif

bool NAND_CheckDataComplete()
{
	#ifdef CPU_SAME53

	if (((HW::DMAC->CH[NAND_DMACH].CTRLA & DMCH_ENABLE) == 0) || (HW::DMAC->CH[NAND_DMACH].INTFLAG & DMCH_TCMPL))
		{
			PIO_WE_RE->SET(WE|RE);
			PIO_WE_RE->DIRSET = WE|RE;
			PIO_WE_RE->PINCFG[PIN_RE] = PINGFG_DRVSTR;
			PIO_WE_RE->PINCFG[PIN_WE] = PINGFG_DRVSTR;

			return true;
		}
		else
		{
			return false;
		};
	
	#elif defined(CPU_XMC48)

		if ((NAND_DMA->CHENREG & NAND_DMA_CHST) == 0)
		{
			if (nandWriteDataLen > 0) 
			{
				NAND_WriteDataDMA(nandWriteDataSrc, nandWriteDataLen);
			}
			else if (nandReadDataLen > 0) 
			{
				NAND_ReadDataDMA(nandReadDataDst, nandReadDataLen);
			}
			else
			{
				return true;
			};
		};

		return false;

	#elif defined(WIN32)

		return HasOverlappedIoCompleted(&_overlapped);

	#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void NAND_Set_Features(byte adr, byte p1, byte p2, byte p3, byte p4)
{
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_SET_FEATURES);
	NAND_ADR_LATCH(adr);
	NAND_WRITE(p1); 
	NAND_WRITE(p2); 
	NAND_WRITE(p3); 
	NAND_WRITE(p4); 
	while(!NAND_BUSY());
	while(NAND_BUSY());

	NAND_DIR_IN();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void NAND_Get_Features(byte adr, byte* p)
{
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_GET_FEATURES);
	NAND_ADR_LATCH(adr);

	while(!NAND_BUSY());

	NAND_DIR_IN();

	while(NAND_BUSY());

	p[0] = NAND_READ(); 
	p[1] = NAND_READ(); 
	p[2] = NAND_READ(); 
	p[3] = NAND_READ(); 
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool NAND_Read_ID(NandID *id)
{
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_READ_ID);
	NAND_ADR_LATCH(0);
	NAND_DIR_IN();

	NAND_ReadDataDMA(id, sizeof(NandID));

	while (!NAND_CheckDataComplete());

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void NAND_Read_PARAM(NandParamPage *pp)
{
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_READ_PARAM);
	NAND_ADR_LATCH(0);
	NAND_DIR_IN();

	while(!NAND_BUSY());
	while(NAND_BUSY());
	
	NAND_ReadDataDMA(pp, sizeof(NandParamPage)); while (!NAND_CheckDataComplete());

	//PIO_WE_RE->SET(RE); 
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u32 NAND_ROW(u32 block, u16 page)
{
	return (block << FLADR::PAGE_BITS) + page;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

static void SetCurAdrNandFile(u16 col, u32 bl, u16 pg)
{
	curBlock = bl;
	curPage = pg;
	curCol = col;

	//bl = ROW(bl, pg);

	u64 adr = bl;
	
	adr = (adr << NAND_CHIP_BITS) + nandChipSelected;

	adr *= (NAND_PAGE_SIZE + NAND_SPARE_SIZE) << NAND_PAGE_BITS;

	adr += (u32)(NAND_PAGE_SIZE + NAND_SPARE_SIZE) * pg;

	adr += col & ((1 << (NAND_COL_BITS+1))-1);

	curNandFilePos = adr;

	//SetFilePointer(handleNandFile, _overlapped.Offset, (i32*)&_overlapped.OffsetHigh, FILE_BEGIN);
}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_CmdEraseBlock(u32 bl)
{
#ifndef WIN32

	bl = NAND_ROW(bl, 0);
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_BLOCK_ERASE_1);
	NAND_ADR_LATCH_ROW(bl);
	NAND_CMD_LATCH(NAND_CMD_BLOCK_ERASE_2);

#else

	SetCurAdrNandFile(0, bl, 0);

	*((u32*)nandEraseFillArray) = (bl << NAND_CHIP_BITS) + nandChipSelected;

	_overlapped.Offset = (u32)curNandFilePos;
	_overlapped.OffsetHigh = (u32)(curNandFilePos>>32);
	_overlapped.hEvent = 0;
	_overlapped.Internal = 0;
	_overlapped.InternalHigh = 0;

	WriteFile(handleNandFile, nandEraseFillArray, (NAND_PAGE_SIZE+NAND_SPARE_SIZE) << NAND_PAGE_BITS, 0, &_overlapped);

	nandReadStatus = 0;

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_CmdRandomRead(u16 col)
{
#ifndef WIN32

	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_RANDREAD_1);
	NAND_ADR_LATCH_COL(col);
	NAND_CMD_LATCH(NAND_CMD_RANDREAD_2);

#else

	SetCurAdrNandFile(col, curBlock, curPage);

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_CmdReadPage(u16 col, u32 bl, u16 pg)
{
#ifndef WIN32

	bl = NAND_ROW(bl, pg);
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_READ_1);
	NAND_ADR_LATCH_COL_ROW(col, bl);
	NAND_CMD_LATCH(NAND_CMD_READ_2);

#else

	SetCurAdrNandFile(col, bl, pg);

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_CmdWritePage(u16 col, u32 bl, u16 pg)
{
#ifndef WIN32

	bl = NAND_ROW(bl, pg);
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_PAGE_PROGRAM_1);
	NAND_ADR_LATCH_COL_ROW(col, bl);

#else

	SetCurAdrNandFile(col, bl, pg);

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_CmdWritePage2()
{
#ifndef WIN32
	NAND_DIR_OUT();
	NAND_CMD_LATCH(NAND_CMD_PAGE_PROGRAM_2);
#else

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

struct FLADR_old
{
	union
	{
		struct
		{
			u64		col		: NAND_COL_BITS;
			u64 	page	: NAND_PAGE_BITS;
			u64		chip	: NAND_CHIP_BITS;
			u64 	block	: NAND_BLOCK_BITS;

			u64		overflow : (64-(NAND_COL_BITS+NAND_PAGE_BITS+NAND_CHIP_BITS+NAND_BLOCK_BITS));
		};

		u64	raw;
	};

	enum { pg = (1<<NAND_COL_BITS) };
//	u32		rawpage;

//	const NandMemSize& sz;

	inline void operator=(const FLADR &a) { raw = a.raw; }

	FLADR_old() : raw(0) {}
	FLADR_old(u32 bl, u16 pg, u16 cl, byte ch) : block(bl), page(pg), col(cl), chip(ch) {}
	FLADR_old(u32 pg) : col(0) { SetRawPage(pg); }

	u32		GetRawPage() { return (raw & NAND_RAWADR_MASK) >> NAND_COL_BITS; }

	void	SetRawPage(u32 p) { raw = (u64)(p & NAND_RAWPAGE_MASK) << NAND_COL_BITS; };

	u32		GetRawBlock() { return (raw & NAND_RAWADR_MASK) >> (NAND_COL_BITS+NAND_PAGE_BITS); }

	void	SetRawBlock(u32 b) { raw = (u64)(b & NAND_RAWBLOCK_MASK) << (NAND_COL_BITS+NAND_PAGE_BITS); };

	u64		GetRawAdr()	{ return raw & NAND_RAWADR_MASK; };
	void	SetRawAdr(u64 a) { raw  = a & NAND_RAWADR_MASK; };

	void	NextPage()	{ col = 0; raw += 1 << NAND_COL_BITS; raw += FLADR::chipOffsetNext[chip]; }
	void	NextBlock()	{ col = 0;page = 0;raw += 1 << (NAND_COL_BITS + NAND_PAGE_BITS); raw += FLADR::chipOffsetNext[chip];}
	void	PrevPage()	{ raw -= 1 << NAND_COL_BITS; col = 0; raw -= FLADR::chipOffsetPrev[chip]; }
	void	PrevBlock()	{ raw -= 1 << (NAND_COL_BITS + NAND_PAGE_BITS);col = 0;page = 0; raw -= FLADR::chipOffsetPrev[chip];}

	void	AddRaw(u32 v) { raw += v; raw += FLADR::chipOffsetNext[chip]; }
	void	SubRaw(u32 v) { raw -= v; raw -= FLADR::chipOffsetPrev[chip]; }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_Test_FLADR()
{
	FLADR_old	x;
	FLADR		y;

	u32 col = 0;
	u32 page = 0;
	u32 chip = 0;
	u32 block = 0;
	u32 rawpage = 0;
	u32 rawblock = 0;
	u64 rawadr = 0;

	for (u32 n = 1; n < (1<<NAND_MAX_CHIP); n++)
	{
		FLADR::InitVaildTables(n);

		for (u32 i = 0; i < 10000; i++)
		{
			x.col = col;				y.SetCol(col);				if (x.raw != y.raw)	__debugbreak();

			x.page = page;				y.SetPage(page);			if (x.raw != y.raw)	__debugbreak();

			x.chip = chip;				y.SetChip(chip);			if (x.raw != y.raw)	__debugbreak();

			x.block = block;			y.SetBlock(block);			if (x.raw != y.raw)	__debugbreak();

			x.SetRawPage(rawpage);		y.SetRawPage(rawpage);		if (x.raw != y.raw)	__debugbreak();

			x.SetRawBlock(rawblock);	y.SetRawBlock(rawblock);	if (x.raw != y.raw)	__debugbreak();

			x.SetRawAdr(rawadr);		y.SetRawAdr(rawadr);		if (x.raw != y.raw)	__debugbreak();

			x.NextPage();				y.NextPage();				if (x.raw != y.raw)	__debugbreak();
			x.NextBlock();				y.NextBlock();				if (x.raw != y.raw)	__debugbreak();
			x.PrevPage();				y.PrevPage();				if (x.raw != y.raw)	__debugbreak();
			x.PrevBlock();				y.PrevBlock();				if (x.raw != y.raw)	__debugbreak();


			if (x.GetRawPage()	!= y.GetRawPage())		__debugbreak();
			if (x.GetRawBlock() != y.GetRawBlock())		__debugbreak();
			if (x.GetRawAdr()	!= y.GetRawAdr())		__debugbreak();

			col		+= 9859;
			page	+= 1019;
			chip	+= 647;
			block		= (block+1019)*9871;
			rawpage		= (rawpage+1087)*98887;
			rawblock	= (rawblock+1153)*99901;
			rawadr		= (rawadr+1229)*99991;
		};
	};

}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void NAND_Init()
{
#ifndef WIN32

	using namespace HW;

	byte p[4];

#ifdef CPU_SAME53

	HW::GCLK->PCHCTRL[GCLK_TCC0_TCC1] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;
	
	HW::MCLK->APBBMASK |= APBB_TCC0;

	HW::GCLK->PCHCTRL[EVENT_NAND_1+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;
	//HW::GCLK->PCHCTRL[EVENT_NAND_2+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;
	//HW::GCLK->PCHCTRL[EVENT_NAND_3+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;

	EVSYS->CH[EVENT_NAND_1].CHANNEL = EVGEN_DMAC_CH_0|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE;
//	EVSYS->USER[EVSYS_USER_PORT_EV_1] = EVENT_NAND_1+1;
	EVSYS->USER[EVSYS_USER_TCC0_EV_1] = EVENT_NAND_1+1;

//	EVSYS->CH[EVENT_NAND_2].CHANNEL = EVGEN_TC0_MC_0|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE;
//	EVSYS->USER[EVSYS_USER_PORT_EV_0] = EVENT_NAND_2+1;

//	EVSYS->CH[EVENT_NAND_3].CHANNEL = EVGEN_TC0_OVF|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE;
//	EVSYS->USER[EVSYS_USER_PORT_EV_2] = EVENT_NAND_3+1;

	PIO_NAND_DATA->DIRSET = 0xFF;
	PIO_NAND_DATA->SetWRCONFIG(0xFF, PORT_INEN|PORT_DRVSTR|PORT_WRPINCFG);
	PIO_NAND_DATA->CTRL |= 0xFF;
	PIO_FCS->DIRSET = FCS0|FCS1|FCS2|FCS3|FCS4|FCS5|FCS6|FCS7; PIO_FCS->SET(FCS0|FCS1|FCS2|FCS3|FCS4|FCS5|FCS6|FCS7);
	PIO_CLE->DIRSET = CLE; PIO_CLE->CLR(CLE);
	PIO_ALE->DIRSET = ALE; PIO_ALE->CLR(ALE);

	PIO_WE_RE->DIRSET = WE; PIO_WE_RE->SET(WE); PIO_WE_RE->SetWRCONFIG(WE, PORT_DRVSTR|PORT_WRPINCFG|PORT_PMUX(6)|PORT_WRPMUX/*|PORT_PMUXEN*/);
	PIO_WE_RE->DIRSET = RE; PIO_WE_RE->SET(RE); PIO_WE_RE->SetWRCONFIG(RE, PORT_DRVSTR|PORT_WRPINCFG|PORT_PMUX(6)|PORT_WRPMUX/*|PORT_PMUXEN*/);

	PIO_FLREADY->DIRCLR = FLREADY; PIO_FLREADY->PINCFG[PIN_FLREADY] = PINGFG_INEN|PINGFG_PULLEN; PIO_FLREADY->CTRL |= FLREADY; PIO_FLREADY->SET(FLREADY);
	PIO_WP->DIRSET = WP; PIO_WP->SET(WP);

	nandTCC->CTRLA = 0;
	nandTCC->CTRLA = TCC_SWRST;
	while(nandTCC->SYNCBUSY & TCC_SWRST);

	nandTCC->CTRLA = 0;
	nandTCC->WAVE = TCC_WAVEGEN_NPWM|TCC_POL0;
	nandTCC->DRVCTRL = TCC_NRE0|TCC_NRE1|TCC_NRV0|TCC_NRV1;
	nandTCC->PER = 250;
	nandTCC->CC[0] = 2; 
	nandTCC->CC[1] = 2; 

	nandTCC->EVCTRL = TCC_OVFEO|TCC_MCEO0|TCC_TCEI0|TCC_EVACT0_RETRIGGER;

	nandTCC->CTRLBSET = TCC_ONESHOT;
	nandTCC->CTRLA = TCC_ENABLE;

	//PIO_WE_RE->EVCTRL.EV[0] = PIN_WE|PORT_PORTEI|PORT_EVACT_SET;
	//PIO_WE_RE->EVCTRL.EV[1] = PIN_WE|PORT_PORTEI|PORT_EVACT_CLR;

	NAND_DIR_OUT();

#elif defined(CPU_XMC48)

	HW::EBU_Enable(0);

	HW::Peripheral_Enable(PID_DMA0);

	NAND_DMA->DMACFGREG = 1;

	EBU->CLC = 0;
	EBU->MODCON = /*EBU_ARBSYNC|*/EBU_ARBMODE(3);
	EBU->USERCON = 0x3FF<<16;

	EBU->ADDRSEL0 = EBU_REGENAB/*|EBU_ALTENAB*/;

	EBU->BUSRCON0 = EBU_AGEN(4)|EBU_WAIT(0)|EBU_PORTW(1);
	EBU->BUSRAP0 = EBU_ADDRC(0)|EBU_CMDDELAY(0)|EBU_WAITRDC(NS2CLK(60))|EBU_DATAC(0)|EBU_RDRECOVC(NS2CLK(0))|EBU_RDDTACS(0);

	EBU->BUSWCON0 = EBU_LOCKCS|EBU_AGEN(4)|EBU_WAIT(0)|EBU_PORTW(1);

//				 = |			|				 |		tWP		 |			   |			   |				;
	EBU->BUSWAP0 = EBU_ADDRC(0)|EBU_CMDDELAY(0)|EBU_WAITWRC(NS2CLK(45))|EBU_DATAC(0)|EBU_WRRECOVC(NS2CLK(0))|EBU_WRDTACS(0);

#endif

	byte checkBuf[16];
	byte readBuf[16];

	for (byte i = 0; i < 8; i++)
	{
		checkBuf[i] = 1 << i;
		checkBuf[i+8] = ~(1 << i);
	};

	for(byte chip = 0; chip < NAND_MAX_CHIP; chip ++)
	{
		NAND_Chip_Select(chip);
		ResetNand();

		for (byte i = 0; i < ArraySize(checkBuf); i++)
		{
			NAND_ADR_LATCH(checkBuf[i]);
			readBuf[i] = NAND_ADR_READ();
		};

		byte bitMask = 0;

		for (byte i = 0; i < 8; i++)
		{
			if (checkBuf[i] != readBuf[i] || checkBuf[i + 8] != readBuf[i + 8])
			{
				bitMask |= 1 << i;
			};
		};

		nandSize.chipDataBusMask[chip] = ~bitMask;
	
		NandID &id = nandSize.id[chip];

		NAND_Read_ID(&id);

		if((id.maker == 0xEC) && (id.device == 0xD3))
		{
			if (nandSize.ch == 0)
			{
				FLADR::PAGE_OFF			= FLADR::COL_BITS = id.pageSize + 10;
				FLADR::CHIP_OFF			= id.blockSize + 16;
				FLADR::PAGE_BITS		= FLADR::CHIP_OFF - FLADR::PAGE_OFF;
				FLADR::BLOCK_BITS		= (id.planeSize + 23 + id.planeNumber) - FLADR::CHIP_OFF;

				nandSize.ch = 1ULL << (FLADR::COL_BITS+FLADR::PAGE_BITS+FLADR::BLOCK_BITS);
			};
			
			nandSize.fl += nandSize.ch;
		
			if (bitMask == 0) { nandSize.mask |= (1 << chip); };

			nandSize.integrityCRC[chip] = 0;
		}
		else if((id.maker == 0x2C) && (id.device == 0x68))
		{
			NAND_Set_Features(1, 5, 0, 0, 0);
			
			ResetNand();

			NAND_Get_Features(1, p);

			NandParamPage &np = nandParamPage[chip];

			NAND_Read_PARAM(&np);

			u16 crc = GetCRC16_8005_refl(&np, sizeof(np)-2, 0x4F4E);

			nandSize.integrityCRC[chip] = crc;

			if (np.integrityCRC == crc/* || np.integrityCRC == 0xA61F*/)
			{
				if (nandSize.mask == 0)
				{
					FLADR::PAGE_OFF			= FLADR::COL_BITS = clog2(np.numberDataBytesPerPage);
					FLADR::PAGE_BITS		= clog2(np.numberPagesPerBlock);
					FLADR::CHIP_OFF			= FLADR::PAGE_OFF + FLADR::PAGE_BITS;
					FLADR::BLOCK_BITS		= clog2(np.numberBlocksPerLUN * np.numberLUNsPerChip);

					nandSize.ch = 1ULL << (FLADR::COL_BITS+FLADR::PAGE_BITS+FLADR::BLOCK_BITS);
				};
				
				nandSize.fl += nandSize.ch;
				
				if (bitMask == 0) { nandSize.mask |= (1 << chip); };
			};
		};
	};

	if (nandSize.ch != 0)
	{
		FLADR::BLOCK_OFF		= FLADR::CHIP_OFF + NAND_CHIP_BITS;
		FLADR::COL_MASK			= (1UL<<FLADR::COL_BITS)-1;
		FLADR::PAGE_MASK		= (1UL<<FLADR::PAGE_BITS)-1;
		FLADR::CHIP_MASK		= NAND_CHIP_MASK;
		FLADR::BLOCK_MASK		= (1UL<<FLADR::BLOCK_BITS)-1;
		FLADR::RAWPAGE_MASK		= (1UL<<(FLADR::PAGE_BITS+NAND_CHIP_BITS+FLADR::BLOCK_BITS))-1;
		FLADR::RAWBLOCK_MASK	= (1UL<<(NAND_CHIP_BITS+FLADR::BLOCK_BITS))-1;
		FLADR::RAWADR_MASK		= (1ULL<<(FLADR::COL_BITS+FLADR::PAGE_BITS+NAND_CHIP_BITS+FLADR::BLOCK_BITS))-1;
		FLADR::pg				= 1UL << FLADR::COL_BITS;
	};

	nandSize.mask &= testNandChipMask;

	FLADR::InitVaildTables(nandSize.mask);

	NAND_Chip_Disable();

	DisableWriteProtect();

#else

	printf("Open file %s ... ", nameNandFile);

	handleNandFile = CreateFile(nameNandFile, GENERIC_READ|GENERIC_WRITE, FILE_SHARE_READ, 0, OPEN_ALWAYS, FILE_FLAG_OVERLAPPED , 0);
	//handleNandFile = CreateFile(nameNandFile, GENERIC_READ|GENERIC_WRITE, FILE_SHARE_READ, 0, OPEN_ALWAYS, 0 , 0);

	cputs((handleNandFile == INVALID_HANDLE_VALUE) ? "!!! ERROR !!!\n" : "OK\n");

	FLADR::PAGE_OFF			= FLADR::COL_BITS = NAND_COL_BITS;
	FLADR::PAGE_BITS		= NAND_PAGE_BITS;
	FLADR::CHIP_OFF			= FLADR::PAGE_OFF + FLADR::PAGE_BITS;
	FLADR::BLOCK_BITS		= NAND_BLOCK_BITS;
	FLADR::BLOCK_OFF		= FLADR::CHIP_OFF + NAND_CHIP_BITS;
	FLADR::COL_MASK			= (1UL<<FLADR::COL_BITS)-1;
	FLADR::PAGE_MASK		= (1UL<<FLADR::PAGE_BITS)-1;
	FLADR::CHIP_MASK		= NAND_CHIP_MASK;
	FLADR::BLOCK_MASK		= (1UL<<FLADR::BLOCK_BITS)-1;
	FLADR::RAWPAGE_MASK		= (1UL<<(FLADR::PAGE_BITS+NAND_CHIP_BITS+FLADR::BLOCK_BITS))-1;
	FLADR::RAWBLOCK_MASK	= (1UL<<(NAND_CHIP_BITS+FLADR::BLOCK_BITS))-1;
	FLADR::RAWADR_MASK		= (1ULL<<(FLADR::COL_BITS+FLADR::PAGE_BITS+NAND_CHIP_BITS+FLADR::BLOCK_BITS))-1;
	FLADR::pg				= 1UL << FLADR::COL_BITS;

	//nandSize.pg = 1 << (nandSize.bitCol = NAND_COL_BITS);
	//nandSize.bl = 1 << (nandSize.shBl = NAND_COL_BITS+NAND_PAGE_BITS);
	nandSize.ch = 1ULL << (NAND_COL_BITS+NAND_PAGE_BITS+NAND_BLOCK_BITS);
	
	//nandSize.pagesInBlock = 1 << (nandSize.bitPage = nandSize.shBl - nandSize.shPg);

	//nandSize.maskPage = nandSize.pagesInBlock - 1;
	//nandSize.maskBlock = (1 << (nandSize.bitBlock = nandSize.shCh - nandSize.shBl)) - 1;
			
	nandSize.fl = nandSize.ch * NAND_MAX_CHIP;
			
	nandSize.mask = (1 << NAND_MAX_CHIP) - 1;

	cputs("Alloc nandEraseFillArray ... ");

	nandEraseFillArray = VirtualAlloc(0, nandEraseFillArraySize = 1UL << (NAND_COL_BITS+NAND_PAGE_BITS+1), MEM_COMMIT, PAGE_READWRITE);

	cputs((nandEraseFillArray == NULL) ? "!!! ERROR !!!\n" : "OK\n");

	if (nandEraseFillArray != 0) for (u32 i = nandEraseFillArraySize/4, *p = (u32*)nandEraseFillArray; i != 0; i--) *(p++) = ~0;

	NAND_Test_FLADR();

	FLADR::InitVaildTables(nandSize.mask);

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53	

//void NAND_WriteDataDMA_old(volatile void *src, u16 len)
//{
//	using namespace HW;
//
//	nandTC->CTRLA = 0;
//	nandTC->CTRLA = TC_SWRST;
//
//	DmaTable[0].SRCADDR = (byte*)src+len;
//	DmaTable[0].DSTADDR = &HW::PIOA->OUT;
//	DmaTable[0].DESCADDR = 0;
//	DmaTable[0].BTCNT = len;
//	DmaTable[0].BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_SRCINC|DMDSC_EVOSEL_BEAT;
//
//	DMAC->CH[0].EVCTRL = DMCH_EVOE;
//	DMAC->CH[0].PRILVL = DMCH_PRILVL_LVL3;
//	DMAC->CH[0].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_TC0_MC0;
//
//	PIO_WE_RE->EVCTRL.EV[0] = PIN_WE|PORT_PORTEI|PORT_EVACT_SET;
//	PIO_WE_RE->EVCTRL.EV[1] = PIN_WE|PORT_PORTEI|PORT_EVACT_CLR;
//	PIO_WE_RE->EVCTRL.EV[2] = PIN_WE|PORT_PORTEI|PORT_EVACT_SET;
//
//	while(nandTC->SYNCBUSY & TC_SWRST);
//
//	nandTC->CTRLA = TC_MODE_COUNT8;
//	nandTC->WAVE = TC_WAVEGEN_NPWM;
//	nandTC->PER8 = 250;
//	nandTC->CC8[0] = 1; 
//
//	NAND_DIR_OUT();
//
//	nandTC->EVCTRL = TC_OVFEO|TC_MCEO0|TC_TCEI|TC_EVACT_RETRIGGER;
//
//	nandTC->CTRLBSET = TC_ONESHOT;
//	nandTC->CTRLA = TC_MODE_COUNT8|TC_ENABLE;
//
//	DMAC->SWTRIGCTRL = 1;
//}

#elif defined(CPU_XMC48)
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_WriteDataDMA(volatile void *src, u16 len)
{
#ifndef WIN32

	using namespace HW;

	#ifdef CPU_SAME53	

		nandTCC->CTRLA = 0;
		nandTCC->CTRLA = TC_SWRST;

		DmaTable[NAND_DMACH].SRCADDR = (byte*)src+len;
		DmaTable[NAND_DMACH].DSTADDR = &HW::PIOA->OUT;
		DmaTable[NAND_DMACH].DESCADDR = 0;
		DmaTable[NAND_DMACH].BTCNT = len;
		DmaTable[NAND_DMACH].BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_SRCINC|DMDSC_EVOSEL_BEAT;

		DMAC->CH[NAND_DMACH].INTENCLR = ~0;
		DMAC->CH[NAND_DMACH].INTFLAG = ~0;
		DMAC->CH[NAND_DMACH].EVCTRL = DMCH_EVOE;
		DMAC->CH[NAND_DMACH].PRILVL = DMCH_PRILVL_LVL3;
		DMAC->CH[NAND_DMACH].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_TCC0_MC1;

		nandTCC->WAVE = TCC_WAVEGEN_NPWM|TCC_POL0;
		nandTCC->DRVCTRL = TCC_NRE0|TCC_NRE1|TCC_NRV0|TCC_NRV1;
		nandTCC->PER = NS2CLK(40)-1;
		nandTCC->CC[0] = NS2CLK(20); 
		nandTCC->CC[1] = NS2CLK(20); 

		nandTCC->EVCTRL = TCC_OVFEO|TCC_MCEO1|TCC_TCEI1|TCC_EVACT1_RETRIGGER;

		NAND_DIR_OUT();
		PIO_WE_RE->SET(WE|RE); 
		PIO_WE_RE->DIRSET = WE|RE;
		PIO_WE_RE->PINCFG[PIN_WE] = PINGFG_PMUXEN|PINGFG_DRVSTR;

		nandTCC->CTRLBSET = TC_ONESHOT;
		nandTCC->CTRLA = TC_ENABLE;

		DMAC->SWTRIGCTRL = 1UL<<NAND_DMACH;

	#elif defined(CPU_XMC48)

		if (len > BLOCK_TS(~0))
		{
			nandWriteDataSrc = ((byte*)src) + BLOCK_TS(~0);
			nandWriteDataLen = len - BLOCK_TS(~0);
			len = BLOCK_TS(~0);
		}
		else
		{
			nandWriteDataLen = 0;
		};

		nandReadDataLen = 0;

		NAND_DMA->DMACFGREG = 1;

		NAND_DMACH->CTLL = DST_NOCHANGE|SRC_INC|TT_FC_M2M_GPDMA|DEST_MSIZE_8|SRC_MSIZE_8;
		NAND_DMACH->CTLH = BLOCK_TS(len);

		NAND_DMACH->SAR = (u32)src;
		NAND_DMACH->DAR = (u32)FLD;	//0x1FFE8000;//;
		NAND_DMACH->CFGL = 0;
		NAND_DMACH->CFGH = PROTCTL(1);

		NAND_DMA->CHENREG = NAND_DMA_CHEN;

//		NAND_WriteDataPIO(src, len);

	#endif

#else

	_overlapped.Offset = (u32)curNandFilePos;
	_overlapped.OffsetHigh = (u32)(curNandFilePos>>32);
	_overlapped.hEvent = 0;
	_overlapped.Internal = 0;
	_overlapped.InternalHigh = 0;

	WriteFile(handleNandFile, (void*)src, len, 0, &_overlapped);

	curNandFilePos += len;

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_WriteDataPIO(volatile void *src, u16 len)
{
	//using namespace HW;

	byte* p = (byte*)src;

	#ifdef CPU_SAME53	

		PIO_WE_RE->PINCFG[PIN_WE] &= ~PINGFG_PMUXEN;
		PIO_WE_RE->PINCFG[PIN_RE] &= ~PINGFG_PMUXEN;

		NAND_DIR_OUT();

		while(len != 0) { NAND_WRITE(*(p++)); len--; };

	#elif defined(CPU_XMC48)

		while(len != 0) { *FLD = *(p++); len--; };

	#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef CPU_SAME53	

//void NAND_ReadDataDMA_old(volatile void *dst, u16 len)
//{
//	using namespace HW;
//
//	nandTC->CTRLA = 0;
//	nandTC->CTRLA = TC_SWRST;
//
//	DmaTable[0].SRCADDR = &PIO_NAND_DATA->IN;
//	DmaTable[0].DSTADDR = (byte*)dst+len;
//	DmaTable[0].DESCADDR = 0;
//	DmaTable[0].BTCNT = len;
//	DmaTable[0].BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_DSTINC|DMDSC_EVOSEL_BEAT;
//
//	DMAC->CH[0].EVCTRL = DMCH_EVOE;
//	DMAC->CH[0].PRILVL = DMCH_PRILVL_LVL3;
//	DMAC->CH[0].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_TC0_MC0;
//
//	PIO_WE_RE->EVCTRL.EV[0] = PIN_RE|PORT_PORTEI|PORT_EVACT_CLR;
//	PIO_WE_RE->EVCTRL.EV[1] = PIN_RE|PORT_PORTEI|PORT_EVACT_SET;
//	PIO_WE_RE->EVCTRL.EV[2] = PIN_RE|PORT_PORTEI|PORT_EVACT_SET;
//
//	while(nandTC->SYNCBUSY & TC_SWRST);
//
//	nandTC->CTRLA = TC_MODE_COUNT8;
//	nandTC->WAVE = TC_WAVEGEN_NPWM;
//	nandTC->PER8 = 250;
//	nandTC->CC8[0] = 1; 
//
//	NAND_DIR_IN();
//	PIO_WE_RE->CLR(RE); 
//
//	nandTC->EVCTRL = TC_OVFEO|TC_MCEO0|TC_TCEI|TC_EVACT_RETRIGGER;
//
//	nandTC->CTRLBSET = TC_ONESHOT;
//	nandTC->CTRLA = TC_MODE_COUNT8|TC_ENABLE;
//
//	DMAC->SWTRIGCTRL = 1;
//}

#elif defined(CPU_XMC48)
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef CPU_XMC48	


#endif

void NAND_ReadDataDMA(volatile void *dst, u16 len)
{
#ifndef WIN32

	using namespace HW;

	#ifdef CPU_SAME53	

		nandTCC->CTRLA = 0;
		nandTCC->CTRLA = TC_SWRST;

		DmaTable[NAND_DMACH].SRCADDR = &PIO_NAND_DATA->IN;
		DmaTable[NAND_DMACH].DSTADDR = (byte*)dst+len;
		DmaTable[NAND_DMACH].DESCADDR = 0;
		DmaTable[NAND_DMACH].BTCNT = len;
		DmaTable[NAND_DMACH].BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_DSTINC|DMDSC_EVOSEL_BLOCK;

		DMAC->CH[NAND_DMACH].INTENCLR = ~0;
		DMAC->CH[NAND_DMACH].INTFLAG = ~0;
		DMAC->CH[NAND_DMACH].EVCTRL = DMCH_EVOE;
		DMAC->CH[NAND_DMACH].PRILVL = DMCH_PRILVL_LVL3;
		DMAC->CH[NAND_DMACH].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_TCC0_MC0;

		//PIO_WE_RE->EVCTRL.EV[0] = PIN_RE|PORT_PORTEI|PORT_EVACT_CLR;
		//PIO_WE_RE->EVCTRL.EV[1] = PIN_RE|PORT_PORTEI|PORT_EVACT_SET;
		//PIO_WE_RE->EVCTRL.EV[2] = PIN_RE|PORT_PORTEI|PORT_EVACT_SET;

		nandTCC->WAVE = TCC_WAVEGEN_NPWM|TCC_POL0;
		nandTCC->DRVCTRL = TCC_NRE0|TCC_NRE1|TCC_NRV0|TCC_NRV1;
		nandTCC->PER = NS2CLK(70)-1;
		nandTCC->CC[0] = NS2CLK(40); 
		nandTCC->CC[1] = NS2CLK(35); 

		nandTCC->EVCTRL = TCC_OVFEO|TCC_MCEO0|TCC_TCEI1|TCC_EVACT1_STOP;


		NAND_DIR_IN();
		PIO_WE_RE->SET(WE|RE); 
		PIO_WE_RE->DIRSET = WE|RE;
		PIO_WE_RE->PINCFG[PIN_RE] = PINGFG_PMUXEN|PINGFG_DRVSTR;

		nandTCC->CTRLA = TCC_ENABLE;
		nandTCC->CTRLBSET = /*TCC_ONESHOT|*/TCC_CMD_RETRIGGER;

	//	DMAC->SWTRIGCTRL = 1;

	#elif defined(CPU_XMC48)

		if (len > BLOCK_TS(~0))
		{
			nandReadDataDst = ((byte*)dst) + BLOCK_TS(~0);
			nandReadDataLen = len - BLOCK_TS(~0);
			len = BLOCK_TS(~0);
		}
		else
		{
			nandReadDataLen = 0;
		};

		nandWriteDataLen = 0;

		NAND_DMA->DMACFGREG = 1;

		NAND_DMACH->CTLL = DST_INC|SRC_NOCHANGE|TT_FC(0);
		NAND_DMACH->CTLH = BLOCK_TS(len);

		NAND_DMACH->SAR = (u32)FLD;
		NAND_DMACH->DAR = (u32)dst;
		NAND_DMACH->CFGL = 0;
		NAND_DMACH->CFGH = PROTCTL(1);

		NAND_DMA->CHENREG = NAND_DMA_CHEN;

	#endif

#else

	_overlapped.Offset = (u32)curNandFilePos;
	_overlapped.OffsetHigh = (u32)(curNandFilePos>>32);
	_overlapped.hEvent = 0;
	_overlapped.Internal = 0;
	_overlapped.InternalHigh = 0;

	ReadFile(handleNandFile, (void*)dst, len, 0, &_overlapped);

	curNandFilePos += len;

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_ReadDataPIO(volatile void *dst, u16 len)
{
	//using namespace HW;

	byte* p = (byte*)dst;

	#ifdef CPU_SAME53

		PIO_WE_RE->PINCFG[PIN_WE] &= ~PINGFG_PMUXEN;
		PIO_WE_RE->PINCFG[PIN_RE] &= ~PINGFG_PMUXEN;

		NAND_DIR_IN();


	#elif defined(CPU_XMC48)
		
		while(len != 0) { *(p++) = *FLD; len--; };

	#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef CPU_XMC48	

static byte *nandCopyDataSrc = 0;
static byte *nandCopyDataDst = 0;
static u16	nandCopyDataLen = 0;

#endif

void NAND_CopyDataDMA(volatile void *src, volatile void *dst, u16 len)
{
#ifndef WIN32

	using namespace HW;

	#ifdef CPU_SAME53	

		DmaTable[NAND_DMACH].SRCADDR = (byte*)src+len;
		DmaTable[NAND_DMACH].DSTADDR = (byte*)dst+len;
		DmaTable[NAND_DMACH].DESCADDR = 0;
		DmaTable[NAND_DMACH].BTCNT = len;
		DmaTable[NAND_DMACH].BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_DSTINC|DMDSC_SRCINC;

		DMAC->CH[NAND_DMACH].INTENCLR = ~0;
		DMAC->CH[NAND_DMACH].INTFLAG = ~0;
		DMAC->CH[NAND_DMACH].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_TRANSACTION;
		DMAC->SWTRIGCTRL = 1UL<<NAND_DMACH;

	#elif defined(CPU_XMC48)

//		register u32 t __asm("r0");

		if (len > BLOCK_TS(~0))
		{
			nandCopyDataSrc = ((byte*)src) + BLOCK_TS(~0);
			nandCopyDataDst = ((byte*)dst) + BLOCK_TS(~0);
			nandCopyDataLen = len - BLOCK_TS(~0);
			len = BLOCK_TS(~0);
		}
		else
		{
			nandCopyDataLen = 0;
		};

		NAND_MEMCOPY_DMA->DMACFGREG = 1;

		NAND_MEMCOPY_DMACH->CTLL = DST_INC|SRC_INC|TT_FC(0)|DEST_MSIZE(0)|SRC_MSIZE(0);
		NAND_MEMCOPY_DMACH->CTLH = BLOCK_TS(len);

		NAND_MEMCOPY_DMACH->SAR = (u32)src;
		NAND_MEMCOPY_DMACH->DAR = (u32)dst;
		NAND_MEMCOPY_DMACH->CFGL = 0;
		NAND_MEMCOPY_DMACH->CFGH = PROTCTL(1);

		NAND_MEMCOPY_DMA->CHENREG = NAND_MEMCOPY_DMA_CHEN;

	#endif

#else

	DataPointer s((void*)src);
	DataPointer d((void*)dst);

	while (len > 3) *d.d++ = *s.d++, len -= 4;
	while (len > 0) *d.b++ = *s.b++, len -= 1;

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool NAND_CheckCopyComplete()
{
	#ifdef CPU_SAME53

		return (HW::DMAC->CH[NAND_MEMCOPY_DMACH].CTRLA & DMCH_ENABLE) == 0 || (HW::DMAC->CH[NAND_MEMCOPY_DMACH].INTFLAG & DMCH_TCMPL);
	
	#elif defined(CPU_XMC48)

		if ((NAND_MEMCOPY_DMA->CHENREG & NAND_MEMCOPY_DMA_CHST) == 0)
		{
			if (nandCopyDataLen > 0) 
			{
				NAND_CopyDataDMA(nandCopyDataSrc, nandCopyDataDst, nandCopyDataLen);
			}
			else
			{
				return true;;
			};
		};

		return false;

	#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

inline void ManDisable()	{ PIO_MANCH->CLR(L1|L2);	PIO_MANCH->SET(H1|H2);							} 
inline void ManZero()		{ PIO_MANCH->CLR(L2);		PIO_MANCH->SET(L1|H1);		PIO_MANCH->CLR(H2);	} 
inline void ManOne()		{ PIO_MANCH->CLR(L1);		PIO_MANCH->SET(L2|H2);		PIO_MANCH->CLR(H1);	} 
inline void ManDischarge()	{ PIO_MANCH->CLR(L1|L2);	PIO_MANCH->CLR(H1|H2);							} 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const u16 manbaud[5] = { BAUD2CLK(20833), BAUD2CLK(41666), BAUD2CLK(62500), BAUD2CLK(83333), BAUD2CLK(104166) };//0:20833Hz, 1:41666Hz,2:62500Hz,3:83333Hz

static u16 trmHalfPeriod = (manbaud[0]+1)/2;
static u16 trmHalfPeriod2 = manbaud[0];
static u16 trmHalfPeriod3 = (manbaud[0]*3+1)/2;
static u16 trmHalfPeriod4 = manbaud[0] * 2;
static u16 trmHalfPeriod6 = manbaud[0] * 3;
static u16 trmHalfPeriod7 = (manbaud[0] * 7 + 1) / 2;

#endif 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte stateManTrans = 0;
static MTB *manTB = 0;
static bool trmBusy = false;
static bool trmTurbo = false;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

static u16 GetTrmBaudRate(byte i)
{
	if (i >= ArraySize(manbaud)) { i = ArraySize(manbaud) - 1; };

	return (manbaud[i]+1)/2;
}
#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static u16 rcvCount = 0;
static bool rcvBusy = false;
byte stateManRcvr = 0;

const u16 rcvPeriod = BAUD2CLK(20833);

static u16* rcvManPtr = 0;
static u16 rcvManCount = 0;

static u16 rcvManLen = 0;

static MRB *manRB = 0;

//static u16 rcvManLen72 = 0;
//static u16 rcvManLen96 = 0;
//static u16 rcvManLen24 = 0;
//static u16 rcvManLen48 = 0;
//
//static u32 rcvManSum72 = 0;
//static u32 rcvManSum96 = 0;
//static u32 rcvManSum24 = 0;
//static u32 rcvManSum48 = 0;
//
//static u16 rcvManCount72 = 0;
//static u16 rcvManCount96 = 0;
//static u16 rcvManCount24 = 0;
//static u16 rcvManCount48 = 0;

static u16 rcvManLen12 = 0;
static u32 rcvManSum12 = 0;
static u16 rcvManCount12 = 0;
u16 rcvManQuality = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

inline u16 CheckParity(u16 x)
{
	u16 y = x ^ (x >> 1);

	y = y ^ (y >> 2);
	y = y ^ (y >> 4);
	
	return (y ^ (y >> 8))^1;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void ManTrmIRQ()
{
	static u32 tw = 0;
	static u16 count = 0;
	static byte i = 0;
	static const u16 *data = 0;
	static u16 len = 0;
	static bool cmd = false;

	Pin_ManTrmIRQ_Set();

	switch (stateManTrans)
	{
		case 0:	// Idle; 

			ManDisable();
		
			data = manTB->data1;
			len = manTB->len1;
			stateManTrans = 1;

			break;

		case 1: // Start data

			i = 3;
			tw = ((u32)(*data) << 1) | (CheckParity(*data) & 1);

			data++;
			len--;

			ManOne();

			stateManTrans++;

			break;

		case 2:	// Wait data 1-st sync imp

			i--;

			if (i == 0)
			{
				stateManTrans++;
				ManZero();
				i = 3;
			};

			break;

		case 3: // Wait 2-nd sync imp

			i--; 

			if (i == 0)
			{
				stateManTrans++;
				count = 17;

				if (tw & 0x10000) { ManZero(); } else { ManOne(); };
			};

			break;

		case 4: // 1-st half bit wait

//			HW::PIOE->SODR = 1;

			if (tw & 0x10000) { ManOne(); } else { ManZero(); };

			count--;

			if (count == 0)
			{
				if (len > 0)
				{
					stateManTrans = 1;
				}
				else if (manTB->len2 != 0 && manTB->data2 != 0)
				{
					len = manTB->len2;
					data = manTB->data2;

					manTB->len2 = 0;
					manTB->data2 = 0;

					stateManTrans = 1;
				}
				else
				{
					stateManTrans = 6;
				};
			}
			else
			{
				stateManTrans++;
			};

			break;

		case 5: // 2-nd half bit wait

			tw <<= 1;
			stateManTrans = 4;
			if (tw & 0x10000) { ManZero(); } else { ManOne(); };

			break;

		case 6:

			if (tw & 0x10000) { ManZero(); } else { ManOne(); };

			stateManTrans++;

			break;

		case 7:

			ManDisable();
			stateManTrans = 0;

			#ifdef CPU_SAME53	
				ManTT->CTRLA = 0;
				ManTT->INTENCLR = ~0;
			#elif defined(CPU_XMC48)
				ManTT->TCCLR = CC4_TRBC;
			#endif

			manTB->ready = true;
			trmBusy = false;

			break;


	}; // 	switch (stateManTrans)


	#ifdef CPU_SAME53	
		ManTT->INTFLAG = ~0;//TCC_OVF;
	#endif

	Pin_ManTrmIRQ_Clr();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool SendManData(MTB *mtb)
{
#ifndef WIN32

	if (trmBusy || rcvBusy || mtb == 0 || mtb->data1 == 0 || mtb->len1 == 0)
	{
		return false;
	};

	mtb->ready = false;

	manTB = mtb;

	stateManTrans = 0;

	#ifdef CPU_SAME53	


		ManTT->CTRLA = TC_MODE_COUNT8;
		ManTT->WAVE = TC_WAVEGEN_NPWM;
		ManTT->PER8 = GetTrmBaudRate(mtb->baud) - 1; //trmHalfPeriod-1;

		ManTT->INTENCLR = ~TC_OVF;
		ManTT->INTENSET = TC_OVF;

		ManTT->INTFLAG = ~0;

		ManTT->CTRLA = TC_MODE_COUNT8|TC_ENABLE;
		ManTT->CTRLBSET = TC_CMD_RETRIGGER;

		//ManTT->CTRLA = 0;

		//ManTT->PER = trmHalfPeriod-1;

		//ManTT->INTENCLR = ~TCC_OVF;
		//ManTT->INTENSET = TCC_OVF;

		//ManTT->INTFLAG = ~0;

		//ManTT->CTRLA = TCC_ENABLE;

	#elif defined(CPU_XMC48)

		ManTT->PRS = GetTrmBaudRate(mtb->baud)-1; //trmHalfPeriod - 1;
		ManTT->PSC = 3; //0.08us

		ManCCU->GCSS = CCU4_S0SE;  

		ManTT->SWR = ~0;
		ManTT->INTE = CC4_PME;

		ManTT->TCSET = CC4_TRBS;

	#endif

	return trmBusy = true;

#else

	mtb->ready = true;

	return true;

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

static void InitManTransmit()
{
	using namespace HW;

	VectorTableExt[MANT_IRQ] = ManTrmIRQ;
	CM4::NVIC->CLR_PR(MANT_IRQ);
	CM4::NVIC->SET_ER(MANT_IRQ);

#ifdef CPU_SAME53	

	HW::GCLK->PCHCTRL[GCLK_TC0_TC1] = GCLK_GEN(GEN_1M)|GCLK_CHEN;

	HW::MCLK->APBAMASK |= APBA_TC0;

	PIO_MANCH->DIRSET = L1|H1|L2|H2;

	ManTT->CTRLA = TCC_SWRST;

	while(ManTT->SYNCBUSY);

	//SetTrmBoudRate(0);

	ManTT->CTRLA = TC_MODE_COUNT8;
	ManTT->WAVE = TC_WAVEGEN_NPWM;
	ManTT->PER8 = GetTrmBaudRate(0) - 1;

	ManTT->INTENCLR = ~TC_OVF;
	ManTT->INTENSET = TC_OVF;

	ManTT->INTFLAG = ~0;

#elif defined(CPU_XMC48)

	HW::CCU_Enable(ManCCU_PID);

	ManCCU->GCTRL = 0;

	ManCCU->GIDLC = CCU4_S0I|CCU4_PRB;

	ManTT->PRS = GetTrmBaudRate(0)-1;
	ManTT->PSC = 3; //0.08us

	ManCCU->GCSS = CCU4_S0SE;  

	ManTT->SRS = 0;

	ManTT->SWR = ~0;
	ManTT->INTE = CC4_PME;

#endif

	ManDisable();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void ManTrmIRQ2()
{
	static u32 tw = 0;
	static u16 count = 0;
	static byte i = 0;
	static const u16* data = 0;
	static u16 len = 0;
	static bool cmd = false;

	Pin_ManTrmIRQ_Set();

	switch (stateManTrans)
	{
		case 0:	// 1-st sync imp 

			data = manTB->data1;
			len = manTB->len1;
			cmd = false;
			stateManTrans++;

		case 1:

			if (len == 0)
			{
				data = manTB->data2;
				len = manTB->len2;
				manTB->len2 = 0;
			};

			if (len != 0)
			{
				tw = ((u32)(*data) << 1) | (CheckParity(*data) & 1);

				data++;
				len--;

				count = 17;

				u32 tadd = (cmd) ? trmHalfPeriod : 0;

				ManT_SET_CR(trmHalfPeriod3+tadd);

				if (tw & 0x10000)
				{
					ManT_SET_PR(trmHalfPeriod7+tadd); //US2MT(96);
					stateManTrans += 2;
				}
				else
				{
					ManT_SET_PR(trmHalfPeriod6+tadd); //US2MT(72);
					stateManTrans++;
				};

				ManT_CCU8->GCSS = ManT_CCU8_GCSS;

				tw <<= 1;
				count--;
			}
			else
			{
				stateManTrans = 4;
			};

			break;

		case 2:	

			ManT_SET_CR(trmHalfPeriod);

			if (count == 0)
			{
				ManT_SET_PR(trmHalfPeriod2);
				cmd = false;
				stateManTrans = 1;
			}
			else
			{
				if (tw & 0x10000)
				{
					ManT_SET_PR(trmHalfPeriod3);

					if (count == 1)
					{
						cmd = true;
						stateManTrans = 1;
					}
					else
					{
						stateManTrans++;
					};
				}
				else
				{
					ManT_SET_PR(trmHalfPeriod2);
				};

				tw <<= 1;
				count--;
			};

			ManT_CCU8->GCSS = ManT_CCU8_GCSS;

			break;

		case 3: 

			if (tw & 0x10000)
			{
				ManT_SET_CR(trmHalfPeriod);
				ManT_SET_PR(trmHalfPeriod2);

				tw <<= 1;
				count--;

				if (count == 0)
				{
					cmd = true;
					stateManTrans = 1;
				};
			}
			else
			{
				tw <<= 1;
				count--;

				ManT_SET_CR(trmHalfPeriod2);

				if (tw & 0x10000)
				{
					ManT_SET_PR(trmHalfPeriod4);
					
					if (count == 1)
					{
						cmd = true;
						stateManTrans = 1;
					};
				}
				else
				{
					ManT_SET_PR(trmHalfPeriod3);

					if (count == 0)
					{
						cmd = false;
						stateManTrans = 1;
					}
					else
					{
						stateManTrans--;
					}
				};

//				if (count != 0)
				{
					tw <<= 1;
					count--;
				};
			};

			ManT_CCU8->GCSS = ManT_CCU8_GCSS;

			break;

		case 4:

			stateManTrans++;
			break;

		case 5:

			stateManTrans = 0;

			HW::SCU_GENERAL->CCUCON &= ~ManT_CCUCON;


			ManT1->TCCLR = CC8_TRBC;
			ManT1->INTE = 0;
			ManT2->TCCLR = CC8_TRBC;
			ManT3->TCCLR = CC8_TRBC;

			ManT_CCU8->GCSS = ManT_OUT_GCSS;
			ManT_CCU8->GCSC = ManT_OUT_GCSC;
			//ManT_CCU8->GIDLS = ManT_CCU8_GIDLS;

			manTB->ready = true;
			trmBusy = false;

			break;


	}; // 	switch (stateManTrans)


	Pin_ManTrmIRQ_Clr();
}

#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool SendManData2(MTB* mtb)
{
#ifndef WIN32
	if (trmBusy || rcvBusy || mtb == 0 || mtb->data1 == 0 || mtb->len1 == 0)
	{
		return false;
	};

	mtb->ready = false;

	manTB = mtb;

	trmHalfPeriod = GetTrmBaudRate(mtb->baud);
	trmHalfPeriod2 = trmHalfPeriod * 2 - 1;
	trmHalfPeriod3 = trmHalfPeriod * 3 - 1;
	trmHalfPeriod4 = trmHalfPeriod * 4 - 1;
	trmHalfPeriod6 = trmHalfPeriod * 6 - 1;
	trmHalfPeriod7 = trmHalfPeriod * 7 - 1;

	stateManTrans = 0;

#ifdef CPU_SAME53	


	ManTT->CTRLA = TC_MODE_COUNT8;
	ManTT->WAVE = TC_WAVEGEN_NPWM;
	ManTT->PER8 = GetTrmBaudRate(mtb->baud) - 1; //trmHalfPeriod-1;

	ManTT->INTENCLR = ~TC_OVF;
	ManTT->INTENSET = TC_OVF;

	ManTT->INTFLAG = ~0;

	ManTT->CTRLA = TC_MODE_COUNT8 | TC_ENABLE;
	ManTT->CTRLBSET = TC_CMD_RETRIGGER;

	//ManTT->CTRLA = 0;

	//ManTT->PER = trmHalfPeriod-1;

	//ManTT->INTENCLR = ~TCC_OVF;
	//ManTT->INTENSET = TCC_OVF;

	//ManTT->INTFLAG = ~0;

	//ManTT->CTRLA = TCC_ENABLE;

#elif defined(CPU_XMC48)

	ManT_SET_PR(US2MT(50)-1); //trmHalfPeriod - 1;
	ManT1->CR2S = (~0); ManT2->CR1S = (0); ManT2->CR2S = (0); ManT3->CR1S = (~0);

	ManT1->PSC = ManT_PSC; //0.08us
	ManT2->PSC = ManT_PSC; //0.08us
	ManT3->PSC = ManT_PSC; //0.08us

	//ManT1->PSL = ManT1_PSL;
	//ManT2->PSL = ManT2_PSL;
	//ManT3->PSL = ManT3_PSL;

	ManT1->INS = CC8_EV0IS(7) | CC4_EV0EM_RISING_EDGE;
	ManT2->INS = CC8_EV0IS(7) | CC4_EV0EM_RISING_EDGE;
	ManT3->INS = CC8_EV0IS(7) | CC4_EV0EM_RISING_EDGE;

	ManT1->CMC = CC4_STRTS_EVENT0;
	ManT2->CMC = CC4_STRTS_EVENT0;
	ManT3->CMC = CC4_STRTS_EVENT0;
	
	ManT1->TC = CC8_STRM;
	ManT2->TC = CC8_STRM;
	ManT3->TC = CC8_STRM;

	ManT1->SWR = ~0;
	ManT2->SWR = ~0;
	ManT3->SWR = ~0;

	ManT1->CHC = ManT1_CHC;
	ManT2->CHC = ManT2_CHC;
	ManT3->CHC = ManT3_CHC;

	ManT1->INTE = CC8_PME;
	ManT2->INTE = 0;
	ManT3->INTE = 0;

	ManT_CCU8->GCSS = ManT_CCU8_GCSS;
	ManT_CCU8->GIDLC = ManT_CCU8_GIDLC;

	HW::SCU_GENERAL->CCUCON |= ManT_CCUCON;

#endif

	return trmBusy = true;
#else

	mtb->ready = true;

	return true;

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

static void InitManTransmit2()
{

	using namespace HW;

	VectorTableExt[MANT_CCU8_IRQ] = ManTrmIRQ2;
	CM4::NVIC->CLR_PR(MANT_CCU8_IRQ);
	CM4::NVIC->SET_ER(MANT_CCU8_IRQ);

	HW::CCU_Enable(ManT_CCU8_PID);

	ManT_CCU8->GCTRL = 0;
	ManT_CCU8->GIDLC = ManT_CCU8_GIDLC;
	ManT_CCU8->GCSS = ManT_OUT_GCSS;
	ManT_CCU8->GCSC = ManT_OUT_GCSC;

	PIO_MANCH->ModePin(PIN_L1, A3PP);
	PIO_MANCH->ModePin(PIN_H1, A3PP);
	PIO_MANCH->ModePin(PIN_L2, A3PP);
	PIO_MANCH->ModePin(PIN_H2, A3PP);

	//ManT_CCU8->GIDLS = ManT_CCU8_GIDLS;

	//ManT1->PRS = US2MT(100) - 1;
	//ManT1->CR1S = US2MT(50) - 1;
	//ManT1->CR2S = US2MT(50) - 1;

	//ManT1->PSC = ManT_PSC; 
	//ManT2->PSC = ManT_PSC;
	//ManT3->PSC = ManT_PSC; 

	////ManT->CHC = CC8_;

	////ManT->TCSET = CC8_TRBS;

	////ManT1->PSL = ManT1_PSL;
	////ManT2->PSL = ManT2_PSL;
	////ManT3->PSL = ManT3_PSL;

	//ManT1->INS = CC8_EV0IS(7) | CC4_EV0EM_RISING_EDGE;
	//ManT2->INS = CC8_EV0IS(7) | CC4_EV0EM_RISING_EDGE;

	//ManT1->CMC = CC4_STRTS_EVENT0;
	//ManT2->CMC = CC4_STRTS_EVENT0;
	//ManT3->CMC = CC4_STRTS_EVENT0;

	ManT1->CHC = ManT1_CHC;
	ManT2->CHC = ManT2_CHC;
	ManT3->CHC = ManT3_CHC;

	//ManT_CCU8->GCSS = ManT_CCU8_GCSS;

	//ManT1->SRS = 0;

	//ManT1->SWR = ~0;
	//ManT_CCU8->GIDLC = ManT_CCU8_GIDLC;
	//ManT->INTE = CC8_PME;
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void ManRcvEnd(bool ok)
{
#ifdef CPU_SAME53	
	ManRT->INTENCLR = ~0;
#elif defined(CPU_XMC48)
	ManTmr->INTE = 0;
#endif

	manRB->OK = ok;
	manRB->ready = true;
	manRB->len = rcvManLen;

	rcvManLen12 = (rcvManCount12 != 0) ? (rcvManSum12 / rcvManCount12) : 0;

	rcvManQuality = (rcvManLen12 > MT(12)) ? 0 : (((MT(12) - rcvManLen12) * 100 + MT(6))/MT(12));

	rcvBusy = false;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static RTM manRcvTime;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

static __irq void ManRcvIRQ2()
{
	using namespace HW;

	static u32 _number = 0;
	static u32 _length = 0;
	static u32 _data = 0;
	static bool _command = false;
	static bool _parity_temp = false;
	const bool _parity = true;
	static bool _sync = false;
	static bool _state = false;

	Pin_ManRcvIRQ_Set();

	#ifdef CPU_SAME53	

		u32 len = ManRT->CC[0];

		ManRT->CTRLBSET = TCC_CMD_RETRIGGER;

	#elif defined(CPU_XMC48)

		u16 len = ManTmr->CV[1];

		//ManTmr->TCCLR = CC4_TCC;
		//ManTmr->TCSET = CC4_TRB;

	#endif

	_state = !_state;

	if (len <= MT(60))
	{
		i16 dl;

		if (len <= MT(36))
		{
			_length += 1; dl = len - MT(24); 
		}
		else
		{
			_length += 2; dl = len - MT(48);
		};

		if (dl < 0) dl = -dl; rcvManSum12 += dl; rcvManCount12++;

		if(_length >= 3)
		{
			_sync = false;
		};
	}
	else
	{
		if(len > MT(108))
		{
			_sync = false;
		}
		else
		{
			manRcvTime.Reset();

			_sync = true;
			_data = 0;
			_parity_temp = _parity;
			_number = 0;
			_command = !_state; 

			i16 dl;

			if (len <= MT(84))
			{
				_length = 1; dl = len - MT(72); 
			}
			else
			{
				_length = 2; dl = len - MT(96); 
			};

			if (dl < 0) dl = -dl; rcvManSum12 += dl; rcvManCount12++;
		};
	};

	if(_sync && _length == 2)
	{
		manRcvTime.Reset();

		if(_number < 16)
		{
			_data <<= 1;
			_data |= _state;
			_parity_temp ^= _state;
			_number++;
			_length = 0;
		}
	 	else
		{
			Pin_ManRcvSync_Set();

			_sync = false;

			if(_state != _parity_temp)
			{
				_state = !_state;
				_data = (~_data);
				_command = !_command;
			};

			if (rcvManLen == 0)
			{
				if(_command)
				{
					*rcvManPtr++ = _data;
					rcvManLen = 1;
				};
			}
			else 
			{
				if(rcvManLen < rcvManCount)
				{
					*rcvManPtr++ = _data;
				};

				rcvManLen += 1;	
			};

			Pin_ManRcvSync_Clr();
		};
	};

	#ifdef CPU_SAME53	
		ManRT->INTFLAG = ~0;
	#elif defined(CPU_XMC48)
	#endif

	Pin_ManRcvIRQ_Clr();
}
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ManRcvUpdate()
{
	if (rcvBusy)
	{
		u32 irq = Push_IRQ();

		if (rcvManLen > 0 && manRcvTime.Timeout(US2RT(200)))
		{
			ManRcvEnd(true);
		}
		else
		{
			manRB->len = rcvManLen;
		};

		Pop_IRQ(irq);
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

static void InitManRecieve()
{
	using namespace HW;

	VectorTableExt[MANR_IRQ] = ManRcvIRQ2;
	CM4::NVIC->CLR_PR(MANR_IRQ);
	CM4::NVIC->SET_ER(MANR_IRQ);	

#ifdef CPU_SAME53	

	HW::GCLK->PCHCTRL[GCLK_TCC2_TCC3]	= GCLK_GEN(GEN_1M)|GCLK_CHEN;
	HW::GCLK->PCHCTRL[GCLK_TC0_TC1]		= GCLK_GEN(GEN_1M)|GCLK_CHEN;

	HW::MCLK->APBCMASK |= APBC_TCC2;
	HW::MCLK->APBAMASK |= APBA_TC1;

	HW::GCLK->PCHCTRL[EVENT_MANR_1+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;
	HW::GCLK->PCHCTRL[EVENT_MANR_2+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;

	EIC->CTRLA = 0;
	while(EIC->SYNCBUSY);

	EIC->EVCTRL |= EIC_EXTINT0<<MANR_EXTINT;
	EIC->SetConfig(MANR_EXTINT, 1, EIC_SENSE_BOTH);
	EIC->INTENCLR = EIC_EXTINT0<<MANR_EXTINT;
	EIC->CTRLA = EIC_ENABLE;

	EVSYS->CH[EVENT_MANR_1].CHANNEL = (EVGEN_EIC_EXTINT_0+MANR_EXTINT)|EVSYS_PATH_ASYNCHRONOUS;
	EVSYS->USER[EVSYS_USER_TC1_EVU] = EVENT_MANR_1+1;

	EVSYS->CH[EVENT_MANR_2].CHANNEL = EVGEN_TC1_OVF|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE;;
	EVSYS->USER[EVSYS_USER_TCC2_MC_0] = EVENT_MANR_2+1;

	PIO_MANCHRX->DIRCLR = MANCHRX;
	PIO_MANCHRX->CTRL |= MANCHRX;

	PIO_RXD->DIRCLR = RXD;
	PIO_RXD->CTRL |= RXD;

	PIO_MANCHRX->SetWRCONFIG(	MANCHRX,	PORT_PMUX(0)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_INEN);
	PIO_RXD->SetWRCONFIG(		RXD,		PORT_PMUX(0)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_INEN);

	//PIO_MANCHRX->PINCFG[PIN_MANCHRX] = PINGFG_INEN|PINGFG_PMUXEN;
	//PIO_RXD->PINCFG[PIN_RXD] = PINGFG_INEN|PINGFG_PMUXEN;

	ManRT->CTRLA = TCC_SWRST;
	while(ManRT->SYNCBUSY);

	ManRT->CTRLA = TCC_CPTEN0;
	ManRT->EVCTRL = TCC_MCEI0;

	ManRT->PER = ~0;
	ManRT->CC[1] = 250;

	ManRT->INTENCLR = ~0;
	//ManRT->INTENSET = TCC_MC0;

	ManRT->CTRLA = TCC_CPTEN0|TCC_ENABLE;


	ManIT->CTRLA = TCC_SWRST;

	while(ManIT->SYNCBUSY);

	ManIT->CTRLA = TC_MODE_COUNT8;
	ManIT->WAVE = TC_WAVEGEN_NPWM;

	ManIT->EVCTRL = TC_TCEI|TC_EVACT_RETRIGGER|TC_OVFEO;

	ManIT->PER8 = 11;
	ManIT->CC8[0] = ~0;
	ManIT->CC8[1] = ~0;

	ManIT->INTENCLR = ~0;

	ManIT->INTFLAG = ~0;

	ManIT->CTRLA = TC_MODE_COUNT8|TC_ENABLE;

	ManIT->CTRLBSET = TC_ONESHOT;

#elif defined(CPU_XMC48)

	HW::CCU_Enable(ManCCU_PID);

	P1->ModePin10(I2DPU);
	P1->ModePin11(I2DPU);

	ManCCU->GCTRL = 0;

	ManCCU->GIDLC = ManCCU_GIDLC;//CCU4_CS1I|CCU4_CS2I|CCU4_SPRB;

	ManRT->PRS = MT(12) - 1;
	ManRT->CRS = MT(12) - 1;
	ManRT->PSC = ManRT_PSC; //1.28us

	ManTmr->PRS = MT(250);
	ManTmr->PSC = ManRT_PSC; //1.28us

	ManCCU->GCSS = ManCCU_GCSS;//CCU4_S1SE|CCU4_S2SE;  

	ManRT->INS = ManRT_INS;//CC4_EV0IS(2)|CC4_EV0EM_BOTH_EDGES|CC4_LPF0M_7CLK;
	ManRT->CMC = CC4_STRTS_EVENT0;
	ManRT->TC = CC4_STRM|CC4_TSSM;

	ManRT->INTE = 0;//CC4_PME;
	ManRT->SRS = 0;//ManRT_SRS;//CC4_POSR(2);

	ManTmr->INS = CC4_EV0IS(15) | CC4_EV0EM_RISING_EDGE;
	ManTmr->CMC = CC4_CAP0S_EVENT0|CC4_STRTS_EVENT0;
	ManTmr->SRS = CC4_E0SR(2);
	ManTmr->TC = CC4_TSSM|CC4_CAPC_ALWAYS;
	ManTmr->TCSET = CC4_TRB;

	ManTmr->INTE = 0;//CC4_PME;

#endif

}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool RcvManData(MRB *mrb)
{
#ifndef WIN32

	if (rcvBusy /*|| trmBusy*/ || mrb == 0 || mrb->data == 0 || mrb->maxLen == 0)
	{
		return false;
	};

	//ManDisable();

	mrb->ready = mrb->OK = false;
	mrb->len = 0;

	manRB = mrb;
	
	rcvManLen = 0;

	rcvManPtr = manRB->data;
	rcvManCount = manRB->maxLen;

	rcvManSum12 = 0; rcvManCount12 = 0;

	#ifdef CPU_SAME53	

		ManRT->INTFLAG = ~0;
		ManRT->INTENSET = TCC_MC0;

	#elif defined(CPU_XMC48)

		ManTmr->SWR = CC4_RE0A;
		ManTmr->INTE = CC4_E0AE;

	#endif

	return rcvBusy = true;

#else

	mrb->ready = mrb->OK = false;
	mrb->len = 0;

	return true;

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte *twi_wrPtr = 0;
static byte *twi_rdPtr = 0;
static u16 twi_wrCount = 0;
static u16 twi_rdCount = 0;
static byte *twi_wrPtr2 = 0;
static u16 twi_wrCount2 = 0;
static byte twi_adr = 0;
static DSCI2C* twi_dsc = 0;
static DSCI2C* twi_lastDsc = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53	

static __irq void I2C_Handler()
{
	using namespace HW;

	HW::PIOB->BSET(20);

	byte state = I2C->INTFLAG;
	bool nextdsc = false;

	if(state & I2C_ERROR) // Received data is available
	{
		I2C->INTFLAG = I2C_ERROR;
		I2C->STATUS = ~0;
		nextdsc = true;
	}
	else if(state & I2C_SB) // Received data is available
	{
		*twi_rdPtr++ = I2C->DATA; // receive data

		twi_rdCount--;

		if (twi_rdCount > 0)
		{
			I2C->CTRLB = I2C_CMD_2;
		}
		else
		{
			I2C->CTRLB = I2C_ACKACT;

			nextdsc = true; 
		};
	}
	else if(state & I2C_MB) // Data can be transmitted 
	{
		if (twi_wrCount > 0)
		{
			I2C->DATA = *twi_wrPtr++;

			twi_wrCount--;

			twi_dsc->ack = true;

			if(twi_wrCount == 0 && twi_wrCount2 != 0)
			{
				twi_wrPtr = twi_wrPtr2;
				twi_wrCount = twi_wrCount2;
				twi_wrCount2 = 0;
			};
		}
		else if (twi_rdCount > 0)
		{
			I2C->ADDR = (twi_adr << 1) | 1;
		}
		else
		{
			nextdsc = true; //I2C->CTRLB |= I2C_CMD_STOP;
		};
	}
	else
	{
		twi_rdCount = 0;
		twi_wrCount = 0;

		nextdsc = true; //I2C->CTRLB |= I2C_CMD_STOP;
	};

	if (nextdsc)
	{
		twi_dsc->ready = true;
		twi_dsc->readedLen = twi_dsc->rlen - twi_rdCount;

		DSCI2C *ndsc = twi_dsc->next;

		if (ndsc != 0)
		{
			twi_dsc->next = 0;
			twi_dsc = ndsc;

			twi_dsc->ready = false;
			twi_dsc->ack = false;
			twi_dsc->readedLen = 0;

			twi_wrPtr = (byte*)twi_dsc->wdata;	
			twi_rdPtr = (byte*)twi_dsc->rdata;	
			twi_wrPtr2 = (byte*)twi_dsc->wdata2;	
			twi_wrCount = twi_dsc->wlen;
			twi_wrCount2 = twi_dsc->wlen2;
			twi_rdCount = twi_dsc->rlen;
			twi_adr = twi_dsc->adr;

			if (twi_wrPtr2 == 0) twi_wrCount2 = 0;

			//I2C->STATUS.BUSSTATE = BUSSTATE_IDLE;

			I2C->INTFLAG = ~0;
			I2C->INTENSET = I2C_MB|I2C_SB;

			I2C->ADDR = (twi_dsc->adr << 1) | ((twi_wrCount == 0) ? 1 : 0);
		}
		else
		{
			I2C->CTRLB = I2C_CMD_STOP|I2C_ACKACT;

			twi_lastDsc = twi_dsc = 0;
		};
	};

	HW::PIOB->BCLR(20);
}

#elif defined(CPU_XMC48)

static __irq void I2C_Handler()
{
	using namespace HW;

//	HW::P6->BSET(2);

	u32 a = I2C->PSR_IICMode;

	if(a & ACK)
	{
		if (twi_wrCount > 0)
		{
			I2C->TBUF[0] = TDF_MASTER_SEND | *twi_wrPtr++;

			twi_wrCount--;

			twi_dsc->ack = true;

			if(twi_wrCount == 0 && twi_wrCount2 != 0)
			{
				twi_wrPtr = twi_wrPtr2;
				twi_wrCount = twi_wrCount2;
				twi_wrCount2 = 0;
			};
		}
		else if (twi_rdCount > 0)
		{
			if(a & (SCR|RSCR))
			{
				I2C->TBUF[0] = TDF_MASTER_RECEIVE_ACK; 
			}
			else
			{
				I2C->TBUF[0] = TDF_MASTER_RESTART | (twi_adr << 1) | 1;
			};
		}
		else
		{
			I2C->TBUF[0] = TDF_MASTER_STOP;
		};
	}
	else if (a & (RIF|AIF))
	{
		byte t = I2C->RBUF;

		if (twi_rdCount > 0)
		{
			*twi_rdPtr++ = t; // receive data
			twi_rdCount--;
		};
			
		I2C->TBUF[0] = (twi_rdCount > 0) ? TDF_MASTER_RECEIVE_ACK : TDF_MASTER_RECEIVE_NACK; 
	}
	else if ((a & PCR) == 0)
	{
		I2C->TBUF[0] = TDF_MASTER_STOP; 
	}
	else
	{
		twi_dsc->ready = true;
		twi_dsc->readedLen = twi_dsc->rlen - twi_rdCount;

//		state = 0;
		
		DSCI2C *ndsc = twi_dsc->next;

		if (ndsc != 0)
		{
			twi_dsc->next = 0;
			twi_dsc = ndsc;

			twi_dsc->ready = false;
			twi_dsc->ack = false;
			twi_dsc->readedLen = 0;

			twi_wrPtr = (byte*)twi_dsc->wdata;	
			twi_rdPtr = (byte*)twi_dsc->rdata;	
			twi_wrPtr2 = (byte*)twi_dsc->wdata2;	
			twi_wrCount = twi_dsc->wlen;
			twi_wrCount2 = twi_dsc->wlen2;
			twi_rdCount = twi_dsc->rlen;
			twi_adr = twi_dsc->adr;

			if (twi_wrPtr2 == 0) twi_wrCount2 = 0;

			//I2C->CCR |= RIEN|AIEN;
			//I2C->PCR_IICMode |= PCRIEN|NACKIEN|ARLIEN|SRRIEN|ERRIEN|ACKIEN;

			I2C->PSCR = ~0;

			I2C->TBUF[0] = TDF_MASTER_START | (twi_dsc->adr << 1) | ((twi_wrCount == 0) ? 1 : 0);
		}
		else
		{
			I2C->CCR = I2C__CCR;
			I2C->PCR_IICMode = I2C__PCR;

			twi_lastDsc = twi_dsc = 0;
		};

//		I2C->PSCR = PCR|NACK;
	};

	I2C->PSCR = a;

//	HW::P6->BCLR(2);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_Write(DSCI2C *d)
{
#ifndef WIN32

	using namespace HW;

	if (twi_dsc != 0 || d == 0) { return false; };
	if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

	twi_dsc = d;

	twi_dsc->ready = false;
	twi_dsc->ack = false;
	twi_dsc->readedLen = 0;

	twi_wrPtr = (byte*)twi_dsc->wdata;	
	twi_rdPtr = (byte*)twi_dsc->rdata;	
	twi_wrPtr2 = (byte*)twi_dsc->wdata2;	
	twi_wrCount = twi_dsc->wlen;
	twi_wrCount2 = twi_dsc->wlen2;
	twi_rdCount = twi_dsc->rlen;
	twi_adr = twi_dsc->adr;

	if (twi_wrPtr2 == 0) twi_wrCount2 = 0;

	__disable_irq();

	#ifdef CPU_SAME53

		I2C->STATUS.BUSSTATE = BUSSTATE_IDLE;

		I2C->INTFLAG = ~0;
		I2C->INTENSET = I2C_MB|I2C_SB;

		I2C->ADDR = (twi_dsc->adr << 1) | ((twi_wrCount == 0) ? 1 : 0);

	#elif defined(CPU_XMC48)

		I2C->PSCR = ~0;//RIF|AIF|TBIF|ACK|NACK|PCR;

//		state = (wrCount == 0) ? 1 : 0;

		I2C->TBUF[0] = TDF_MASTER_START | (twi_dsc->adr << 1) | ((twi_wrCount == 0) ? 1 : 0);

		I2C->CCR |= RIEN|AIEN;
		I2C->PCR_IICMode |= PCRIEN|NACKIEN|ARLIEN|SRRIEN|ERRIEN|ACKIEN;

	#endif
		
	__enable_irq();

#endif

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_AddRequest(DSCI2C *d)
{
	if (d == 0) { return false; };
	if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

#ifndef WIN32

	d->next = 0;
	d->ready = false;

	if (d->wdata2 == 0) d->wlen2 = 0;

	__disable_irq();

	if (twi_lastDsc == 0)
	{
		twi_lastDsc = d;

		__enable_irq();

		return I2C_Write(d);
	}
	else
	{
		twi_lastDsc->next = d;
		twi_lastDsc = d;

		__enable_irq();
	};

#else

	u16 adr;

	switch (d->adr)
	{
		case 0x49: //Temp

			if (d->rlen >= 2)
			{
				byte *p = (byte*)d->rdata;

				p[0] = 0;
				p[1] = 0;
			};
				
			d->readedLen = d->rlen;
			d->ack = true;
			d->ready = true;

			break;

		case 0x50: // FRAM

			d->readedLen = 0;

			if (d->wdata != 0 && d->wlen == 2)
			{
				adr = ReverseWord(*((u16*)d->wdata));

				adr %= sizeof(fram_I2c_Mem);

				if (d->wdata2 != 0 && d->wlen2 != 0)
				{
					u16 count = d->wlen2;
					byte *s = (byte*)d->wdata2;
					byte *d = fram_I2c_Mem + adr;

					while (count-- != 0) { *(d++) = *(s++); adr++; if (adr >= sizeof(fram_I2c_Mem)) { adr = 0; d = fram_I2c_Mem; }; };
				}
				else if (d->rdata != 0 && d->rlen != 0)
				{
					d->readedLen = d->rlen;
					u16 count = d->rlen;

					byte *p = (byte*)(d->rdata);
					byte *s = fram_I2c_Mem + adr;

					while (count-- != 0) { *(p++) = *(s++); adr++; if (adr >= sizeof(fram_I2c_Mem)) { adr = 0; s = fram_I2c_Mem; }; };
				};
			};

			d->ack = true;
			d->ready = true;

			break;
	};

#endif

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_Update()
{
	bool result = false;

#ifdef CPU_SAME53

#elif defined(CPU_XMC48)

	using namespace HW;

	static TM32 tm;

	__disable_irq();

	if (twi_dsc != 0)
	{
		if (I2C->PSR_IICMode & (PCR|NACK|ACK|RIF|AIF))
		{
			tm.Reset();
		}
		else if (tm.Check(10))
		{
			result = true;

			HW::Peripheral_Disable(I2C_PID);

			I2C_Init();

			twi_dsc->ready = true;
			twi_dsc->readedLen = twi_dsc->rlen - twi_rdCount;

			DSCI2C *ndsc = twi_dsc->next;

			if (ndsc != 0)
			{
				twi_dsc->next = 0;
				twi_dsc = ndsc;

				twi_dsc->ready = false;
				twi_dsc->ack = false;
				twi_dsc->readedLen = 0;

				twi_wrPtr = (byte*)twi_dsc->wdata;	
				twi_rdPtr = (byte*)twi_dsc->rdata;	
				twi_wrPtr2 = (byte*)twi_dsc->wdata2;	
				twi_wrCount = twi_dsc->wlen;
				twi_wrCount2 = twi_dsc->wlen2;
				twi_rdCount = twi_dsc->rlen;
				twi_adr = twi_dsc->adr;

				if (twi_wrPtr2 == 0) twi_wrCount2 = 0;

				I2C->PSCR = ~0;//RIF|AIF|TBIF|ACK|NACK|PCR;

				I2C->CCR |= RIEN|AIEN;
				I2C->PCR_IICMode |= PCRIEN|NACKIEN|ARLIEN|SRRIEN|ERRIEN|ACKIEN;

				I2C->TBUF[0] = TDF_MASTER_START | (twi_dsc->adr << 1) | ((twi_wrCount == 0) ? 1 : 0);
			}
			else
			{
				I2C->CCR = I2C__CCR;
				I2C->PCR_IICMode = I2C__PCR;

				twi_lastDsc = twi_dsc = 0;
			};
		};
	}
	else
	{
		tm.Reset();
	};
	
	__enable_irq();

#endif

	return result;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void I2C_Init()
{
#ifndef WIN32

	using namespace HW;

	#ifdef CPU_SAME53	

		HW::GCLK->PCHCTRL[GCLK_SERCOM3_CORE]	= GCLK_GEN(GEN_25M)|GCLK_CHEN;	// 25 MHz

		MCLK->APBBMASK |= APBB_SERCOM3;

		PIO_I2C->SetWRCONFIG(SDA|SCL, PORT_PMUX(2)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX);

		I2C->CTRLA = I2C_SWRST;

		while(I2C->SYNCBUSY);

		I2C->CTRLA = SERCOM_MODE_I2C_MASTER;

		I2C->CTRLA = SERCOM_MODE_I2C_MASTER|I2C_INACTOUT_205US|I2C_SPEED_SM;
		I2C->CTRLB = 0;
		I2C->BAUD = 0x0018;

		I2C->CTRLA |= I2C_ENABLE;

		while(I2C->SYNCBUSY);

		I2C->STATUS = 0;
		I2C->STATUS.BUSSTATE = BUSSTATE_IDLE;

		VectorTableExt[SERCOM3_0_IRQ] = I2C_Handler;
		VectorTableExt[SERCOM3_1_IRQ] = I2C_Handler;
		VectorTableExt[SERCOM3_3_IRQ] = I2C_Handler;
		CM4::NVIC->CLR_PR(SERCOM3_0_IRQ);
		CM4::NVIC->CLR_PR(SERCOM3_1_IRQ);
		CM4::NVIC->CLR_PR(SERCOM3_3_IRQ);
		CM4::NVIC->SET_ER(SERCOM3_0_IRQ);
		CM4::NVIC->SET_ER(SERCOM3_1_IRQ);
		CM4::NVIC->SET_ER(SERCOM3_3_IRQ);

	#elif defined(CPU_XMC48)

		HW::Peripheral_Enable(I2C_PID);

 		P5->ModePin0(A1OD);
		P5->ModePin2(A1PP);

		I2C->KSCFG = MODEN|BPMODEN|BPNOM|NOMCFG(0);

		I2C->SCTR = I2C__SCTR;

		I2C->FDR = I2C__FDR;
		I2C->BRG = I2C__BRG;
	    
		I2C->TCSR = I2C__TCSR;

		I2C->PSCR = ~0;

		I2C->CCR = 0;

		I2C->DX0CR = I2C__DX0CR;
		I2C->DX1CR = I2C__DX1CR;

		I2C->CCR = I2C__CCR;


		I2C->PCR_IICMode = I2C__PCR;

		VectorTableExt[I2C_IRQ] = I2C_Handler;
		CM4::NVIC->CLR_PR(I2C_IRQ);
		CM4::NVIC->SET_ER(I2C_IRQ);

	#endif

#else

	HANDLE h;

	h = CreateFile("FRAM_I2C_STORE.BIN", GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	ReadFile(h, fram_I2c_Mem, sizeof(fram_I2c_Mem), &bytes, 0);
	CloseHandle(h);

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

void I2C_Destroy()
{
	HANDLE h;

	h = CreateFile("FRAM_I2C_STORE.BIN", GENERIC_WRITE, 0, 0, OPEN_ALWAYS, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	if (!WriteFile(h, fram_I2c_Mem, sizeof(fram_I2c_Mem), &bytes, 0))
	{
		dword le = GetLastError();
	};

	CloseHandle(h);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetClock(const RTC &t)
{
	static DSCI2C dsc;

	static byte reg = 0;
	static u16 rbuf = 0;
	static byte buf[10];

	buf[0] = 0;
	buf[1] = ((t.sec/10) << 4)|(t.sec%10);
	buf[2] = ((t.min/10) << 4)|(t.min%10);
	buf[3] = ((t.hour/10) << 4)|(t.hour%10);
	buf[4] = 1;
	buf[5] = ((t.day/10) << 4)|(t.day%10);
	buf[6] = ((t.mon/10) << 4)|(t.mon%10);

	byte y = t.year % 100;

	buf[7] = ((y/10) << 4)|(y%10);

	dsc.adr = 0x68;
	dsc.wdata = buf;
	dsc.wlen = 8;
	dsc.rdata = 0;
	dsc.rlen = 0;
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;

	if (SetTime(t))
	{
		I2C_AddRequest(&dsc);
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitClock()
{
	DSCI2C dsc;

	byte reg = 0;
	byte buf[10];
	
	RTC t;

	dsc.adr = 0x68;
	dsc.wdata = &reg;
	dsc.wlen = 1;
	dsc.rdata = buf;
	dsc.rlen = 7;
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;

	I2C_AddRequest(&dsc);

	while (!dsc.ready) { I2C_Update(); };

	t.sec	= (buf[0]&0xF) + ((buf[0]>>4)*10);
	t.min	= (buf[1]&0xF) + ((buf[1]>>4)*10);
	t.hour	= (buf[2]&0xF) + ((buf[2]>>4)*10);
	t.day	= (buf[4]&0xF) + ((buf[4]>>4)*10);
	t.mon	= (buf[5]&0xF) + ((buf[5]>>4)*10);
	t.year	= (buf[6]&0xF) + ((buf[6]>>4)*10) + 2000;

	SetTime(t);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_XMC48

u16 CRC_CCITT_PIO(const void *data, u32 len, u16 init)
{
	CRC_FCE->CRC = init;	//	DataCRC CRC = { init };

	__packed const byte *s = (__packed const byte*)data;

	for ( ; len > 0; len--)
	{
		CRC_FCE->IR = *(s++);
	};

	//if (len > 0)
	//{
	//	CRC_FCE->IR = *(s++)&0xFF;
	//}

	return CRC_FCE->RES;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 CRC_CCITT_DMA(const void *data, u32 len, u16 init)
{
	HW::P6->BSET(5);

	byte *s = (byte*)data;

	CRC_FCE->CRC = init;	//	DataCRC CRC = { init };

//	if ((u32)s & 1) { CRC_FCE->IR = *s++; len--; };

	if (len > 0)
	{
		CRC_DMACH->CTLH = BLOCK_TS(len);

		CRC_DMACH->SAR = (u32)s;

		CRC_DMA->CHENREG = CRC_DMA_CHEN;

		while(CRC_DMA->CHENREG & (1<<2));
	};

	//if (len & 1) { CRC_FCE->IR = s[len-1]; };

	HW::P6->BCLR(5);

	return (byte)CRC_FCE->RES;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CRC_CCITT_DMA_Async(const void* data, u32 len, u16 init)
{
	HW::P6->BSET(5);

	byte* s = (byte*)data;

	CRC_FCE->CRC = init;	//	DataCRC CRC = { init };

//	if ((u32)s & 1) { CRC_FCE->IR = *s++; len--; };

	if (len > 0)
	{
		CRC_DMACH->CTLH = BLOCK_TS(len);

		CRC_DMACH->SAR = (u32)s;

		CRC_DMA->CHENREG = CRC_DMA_CHEN;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool CRC_CCITT_DMA_CheckComplete(u16* crc)
{
	if ((CRC_DMA->CHENREG & (1 << 2)) == 0)
	{
		*crc = (byte)CRC_FCE->RES;

		HW::P6->BCLR(5);

		return true;
	}
	else
	{
		return false;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_CRC_CCITT_DMA()
{
	HW::Peripheral_Enable(PID_FCE);

	HW::FCE->CLC = 0;
	CRC_FCE->CFG = 0;

	CRC_DMA->DMACFGREG = 1;

	CRC_DMACH->CTLL = DST_NOCHANGE|SRC_INC|DST_TR_WIDTH_8|SRC_TR_WIDTH_8|TT_FC_M2M_GPDMA|DEST_MSIZE_1|SRC_MSIZE_1;
	CRC_DMACH->DAR = (u32)&CRC_FCE->IR;
	CRC_DMACH->CFGL = 0;
	CRC_DMACH->CFGH = PROTCTL(1);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif defined(CPU_SAME53)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 CRC_CCITT_DMA(const void *data, u32 len, u16 init)
{
	//HW::PIOC->BSET(26);

	T_HW::DMADESC &dmadsc = DmaTable[CRC_DMACH];
	T_HW::S_DMAC::S_DMAC_CH	&dmach = HW::DMAC->CH[CRC_DMACH];

	dmadsc.DESCADDR = 0;
	dmadsc.DSTADDR = (void*)init;
	dmadsc.SRCADDR = (byte*)data+len;
	dmadsc.BTCNT = len;
	dmadsc.BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_SRCINC;

	HW::DMAC->CRCCTRL = DMAC_CRCBEATSIZE_BYTE|DMAC_CRCPOLY_CRC16|DMAC_CRCMODE_CRCGEN|DMAC_CRCSRC(0x20+CRC_DMACH);

	dmach.INTENCLR = ~0;
	dmach.INTFLAG = ~0;
	dmach.CTRLA = DMCH_ENABLE/*|DMCH_TRIGACT_TRANSACTION*/;

	HW::DMAC->SWTRIGCTRL = 1UL << CRC_DMACH;

	while (((dmach.CTRLA & DMCH_ENABLE) != 0) && (dmach.INTFLAG & DMCH_TCMPL) == 0);

	//HW::PIOC->BCLR(26);

	return ReverseWord(HW::DMAC->CRCCHKSUM);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CRC_CCITT_DMA_Async(const void* data, u32 len, u16 init)
{
//	HW::PIOC->BSET(26);

	T_HW::DMADESC& dmadsc = DmaTable[CRC_DMACH];
	T_HW::S_DMAC::S_DMAC_CH& dmach = HW::DMAC->CH[CRC_DMACH];

	dmadsc.DESCADDR = 0;
	dmadsc.DSTADDR = (void*)init;
	dmadsc.SRCADDR = (byte*)data + len;
	dmadsc.BTCNT = len;
	dmadsc.BTCTRL = DMDSC_VALID | DMDSC_BEATSIZE_BYTE | DMDSC_SRCINC;

	HW::DMAC->CRCCTRL = DMAC_CRCBEATSIZE_BYTE | DMAC_CRCPOLY_CRC16 | DMAC_CRCMODE_CRCGEN | DMAC_CRCSRC(0x20 + CRC_DMACH);

	dmach.INTENCLR = ~0;
	dmach.INTFLAG = ~0;
	dmach.CTRLA = DMCH_ENABLE/*|DMCH_TRIGACT_TRANSACTION*/;

	HW::DMAC->SWTRIGCTRL = 1UL << CRC_DMACH;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool CRC_CCITT_DMA_CheckComplete(u16* crc)
{
	//T_HW::DMADESC &dmadsc = DmaTable[CRC_DMACH];
	T_HW::S_DMAC::S_DMAC_CH& dmach = HW::DMAC->CH[CRC_DMACH];

	if ((dmach.CTRLA & DMCH_ENABLE) == 0 || (dmach.INTFLAG & DMCH_TCMPL))
	{
		*crc = ReverseWord(HW::DMAC->CRCCHKSUM);

		//HW::PIOC->BCLR(26);

		return true;
	}
	else
	{
		return false;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_CRC_CCITT_DMA()
{
	//T_HW::DMADESC &dmadsc = DmaTable[CRC_DMACH];
	//T_HW::S_DMAC::S_DMAC_CH	&dmach = HW::DMAC->CH[CRC_DMACH];

	//HW::DMAC->CRCCTRL = DMAC_CRCBEATSIZE_BYTE|DMAC_CRCPOLY_CRC16|DMAC_CRCMODE_CRCGEN|DMAC_CRCSRC(0x3F);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif defined(WIN32)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 CRC_CCITT_DMA(const void *data, u32 len, u16 init)
{
	return GetCRC16_CCIT(data, len, init);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CRC_CCITT_DMA_Async(const void* data, u32 len, u16 init)
{
	crc_ccit_result = GetCRC16_CCIT(data, len, init);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool CRC_CCITT_DMA_CheckComplete(u16* crc)
{
	if (crc != 0)
	{
		*crc = crc_ccit_result;

		return true;
	}
	else
	{
		return false;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_CRC_CCITT_DMA()
{
	//T_HW::DMADESC &dmadsc = DmaTable[CRC_DMACH];
	//T_HW::S_DMAC::S_DMAC_CH	&dmach = HW::DMAC->CH[CRC_DMACH];

	//HW::DMAC->CRCCTRL = DMAC_CRCBEATSIZE_BYTE|DMAC_CRCPOLY_CRC16|DMAC_CRCMODE_CRCGEN|DMAC_CRCSRC(0x3F);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void WDT_Init()
{
	#ifdef CPU_SAME53	

		//HW::MCLK->APBAMASK |= APBA_WDT;

		//HW::WDT->CONFIG = WDT_WINDOW_CYC512|WDT_PER_CYC1024;
	
		#ifndef _DEBUG
//		HW::WDT->CTRLA = WDT_ENABLE|WDT_WEN|WDT_ALWAYSON;
		#else
//		HW::WDT->CTRLA = WDT_ENABLE|WDT_WEN|WDT_ALWAYSON;
		#endif

		//while(HW::WDT->SYNCBUSY);

	#elif defined(CPU_XMC48)

		#ifndef _DEBUG
	
		//HW::WDT_Enable();

		//HW::WDT->WLB = OFI_FREQUENCY/2;
		//HW::WDT->WUB = (3 * OFI_FREQUENCY)/2;
		//HW::SCU_CLK->WDTCLKCR = 0|SCU_CLK_WDTCLKCR_WDTSEL_OFI;

		//HW::WDT->CTR = WDT_CTR_ENB_Msk|WDT_CTR_DSP_Msk;

		#else

		HW::WDT_Disable();

		#endif

	#endif
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 rotCount = 0;

static __irq void RotTrmIRQ()
{
#ifdef CPU_SAME53

	//PIO_SYNCROT->WBIT(ROT, !(PIO_SYNCROT->ODSR & ROT));
	rotCount++;

	HW::PIOA->BCLR(15);

	//if (rotCount >= pulsesPerHeadRound)
	//{
	//	SyncTmr.CCR = SWTRG;
	//	rotCount = 0;
	//	
	//	HW::PIOA->BSET(15);
	//};

//	u32 tmp = RotTmr.SR;

#elif defined(CPU_XMC48)
#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Set_Sync_Rot(u16 RPS, u16 samplePerRound)
{
#ifndef WIN32

	u32 t = RPS;

	if (t == 0) t = 100;

	if (samplePerRound < 8) { samplePerRound = 8; };

	t *= samplePerRound;
	
	t = (100000000 + t/2) / t;
	
	t = US2SRT(t);

	if (t > 0xFFFF) t = 0xFFFF;

	//u16 r = (samplePerRound + 36) / 72;
	
	u32 r = ((u32)RPS * pulsesPerHeadRoundFix4) >> 4;

	if (r != 0)
	{
		r = US2SRT((100000000 + r/2) / r);
	};

	if (r > 0xFFFF) r = 0xFFFF;

	#ifdef CPU_SAME53	

		SyncTmr->PER = t;
		SyncTmr->CC[0] = US2SRT(10); 

		SyncTmr->CTRLA = (t != 0) ? TCC_ENABLE : 0;

		RotTmr->CC[0] = r;

		RotTmr->CTRLA = (r != 0) ? TCC_ENABLE : 0;

		SyncTmr->CTRLBSET = TCC_CMD_RETRIGGER;
		RotTmr->CTRLBSET = TCC_CMD_RETRIGGER;

	#elif defined(CPU_XMC48)

		PIO_SYNC->ModePin(PIN_SYNC, A3PP);
		PIO_ROT->ModePin(PIN_ROT, A3PP);

		HW::CCU_Enable(SyncRotCCU_PID);

		SyncRotCCU->GCTRL = 0;

		SyncRotCCU->GIDLC = SyncRot_GIDLC;

		SyncTmr->PRS = t-1;
		SyncTmr->CRS = US2SRT(10)-1;
		SyncTmr->PSC = SyncRot_PSC; 
		SyncTmr->PSL = 1; 

		if (t != 0) { SyncTmr->TCSET = CC4_TRBS; } else { SyncTmr->TCCLR = CC4_TRBC; };

		RotTmr->PRS = r-1;
		RotTmr->CRS = r/2;
		RotTmr->PSC = SyncRot_PSC; 
		RotTmr->TC = CC4_TCM;

		if (r != 0) { RotTmr->TCSET = CC4_TRBS; } else { RotTmr->TCCLR = CC4_TRBC; };

		SyncRotCCU->GCSS = Sync_GCSS|Rot_GCSS;  

	#endif

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

static void Init_Sync_Rot()
{
	using namespace HW;

#ifdef CPU_SAME53	


	PIO_SYNC->SetWRCONFIG(SYNC, PORT_PMUX(5)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_INEN);
	PIO_ROT->SetWRCONFIG(ROT,	PORT_PMUX(5)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_INEN);	

	HW::GCLK->PCHCTRL[GCLK_TCC2_TCC3]	= GCLK_GEN(GEN_1M)|GCLK_CHEN;
	HW::GCLK->PCHCTRL[GCLK_TCC4]		= GCLK_GEN(GEN_1M)|GCLK_CHEN;

	HW::MCLK->APBCMASK |= APBC_TCC3;
	HW::MCLK->APBDMASK |= APBD_TCC4;

	//PIO_MANCH->DIRSET = L1|H1|L2|H2;

	SyncTmr->CTRLA = TCC_SWRST;

	while(SyncTmr->SYNCBUSY);

	SyncTmr->CTRLA = 0;
	SyncTmr->WAVE = TCC_WAVEGEN_NPWM;//|TCC_POL0;
	SyncTmr->DRVCTRL = 0;//TCC_NRE0|TCC_NRE1|TCC_NRV0|TCC_NRV1;
	SyncTmr->PER = 250;
	SyncTmr->CC[0] = 2; 
	//SyncTmr->CC[1] = 2; 

	SyncTmr->EVCTRL = 0;

	SyncTmr->CTRLA = TCC_ENABLE;

	RotTmr->CTRLA = TCC_SWRST;

	while(RotTmr->SYNCBUSY);

	RotTmr->CTRLA = 0;
	RotTmr->WAVE = TCC_WAVEGEN_MFRQ;//|TCC_POL0;
	RotTmr->DRVCTRL = 0;//TCC_NRE0|TCC_NRE1|TCC_NRV0|TCC_NRV1;
	RotTmr->CC[0] = 250;
	//RotTmr->CC[0] = 2; 
	//RotTmr->CC[1] = 2; 

	RotTmr->EVCTRL = 0;

	RotTmr->CTRLA = 0;//TCC_ENABLE;
	
	SyncTmr->CTRLBSET = TCC_CMD_RETRIGGER;
	RotTmr->CTRLBSET = TCC_CMD_RETRIGGER;


#elif defined(CPU_XMC48)

	PIO_SYNC->ModePin(PIN_SYNC, A3PP);
	PIO_ROT->ModePin(PIN_ROT, A3PP);

	HW::CCU_Enable(SyncRotCCU_PID);

	SyncRotCCU->GCTRL = 0;

	SyncRotCCU->GIDLC = SyncRot_GIDLC;

	SyncTmr->PRS = US2SRT(60)-1;
	SyncTmr->CRS = US2SRT(10)-1;
	SyncTmr->PSC = SyncRot_PSC; 
	SyncTmr->PSL = 1;
	SyncTmr->TCSET = CC4_TRBS;

	RotTmr->PRS = ~0;
	RotTmr->CRS = 0x7FFF;
	RotTmr->PSC = SyncRot_PSC; 
	RotTmr->TC = CC4_TCM;
	RotTmr->TCSET = CC4_TRBS;

	SyncRotCCU->GCSS = Sync_GCSS|Rot_GCSS;  

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void ShaftIRQ()
{
	Pin_ShaftIRQ_Set();

#ifdef CPU_SAME53	

	HW::EIC->INTFLAG = 1<<SHAFT_EXTINT;

	SyncTmr->CTRLBSET = TCC_CMD_RETRIGGER;

#elif defined(CPU_XMC48)
	
	//SyncTmr->TCCLR = CC4_TCC;

	SyncTmr->TCCLR = CC4_TRBC;
	SyncTmr->TIMER = SyncTmr->PR >> 1;
	SyncTmr->TCSET = CC4_TRBS;

#endif

	shaftCounter++;
	curShaftCounter++;

	u32 tm = GetMilliseconds();
	u32 dt = tm - shaftPrevTime;

	if (dt >= 1000)
	{
		shaftPrevTime = tm;
		shaftCount = shaftCounter;
		shaftTime = dt;
		shaftCounter = 0;
	};

	rotCount = 0;

	Pin_ShaftIRQ_Clr();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateShaft()
{
	if (shaftCount != 0)
	{
		shaftRPS = shaftCount * 100000 / shaftTime;
		
		shaftCount = 0;
	}
	else if ((GetMilliseconds() - shaftPrevTime) > 1500)
	{
		shaftRPS = 0;
	};
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetShaftState()
{
#ifndef WIN32
	return PIO_SHAFT->TBCLR(PIN_SHAFT);
#else
	return 0;
#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

static void InitShaft()
{
	using namespace HW;

	VectorTableExt[IRQ_SHAFT] = ShaftIRQ;
	CM4::NVIC->CLR_PR(IRQ_SHAFT);
	CM4::NVIC->SET_ER(IRQ_SHAFT);	

#ifdef CPU_SAME53	

	EIC->CTRLA = 0;
	while(EIC->SYNCBUSY);

	EIC->EVCTRL |= EIC_EXTINT0<<SHAFT_EXTINT;
	EIC->SetConfig(SHAFT_EXTINT, 1, EIC_SENSE_RISE);
	EIC->INTENSET = EIC_EXTINT0<<SHAFT_EXTINT;
	EIC->CTRLA = EIC_ENABLE;

	PIO_SHAFT->SetWRCONFIG(SHAFT, PORT_PMUX(0)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_INEN);

#elif defined(CPU_XMC48)

	PIO_SHAFT->ModePin(PIN_SHAFT, I1DPD);

	// Event Request Select (ERS)
	
	ERU0->EXISEL = 2<<ERU_EXISEL_EXS3B_Pos;
	
	// Event Trigger Logic (ETL)

	ERU0->EXICON[3] = ERU_PE|ERU_RE|ERU_OCS(0)|ERU_SS_B;

	// Cross Connect Matrix

	// Output Gating Unit (OGU)

	ERU0->EXOCON[0] = ERU_GP(1);

#endif

}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void DSP_CopyDataDMA(volatile void *src, volatile void *dst, u16 len)
{
#ifndef WIN32

	using namespace HW;

	#ifdef CPU_SAME53	

		DmaTable[DSP_DMACH].SRCADDR = (byte*)src+len;
		DmaTable[DSP_DMACH].DSTADDR = (byte*)dst+len;
		DmaTable[DSP_DMACH].DESCADDR = 0;
		DmaTable[DSP_DMACH].BTCNT = len;
		DmaTable[DSP_DMACH].BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_DSTINC|DMDSC_SRCINC;

		DMAC->CH[DSP_DMACH].INTENCLR = ~0;
		DMAC->CH[DSP_DMACH].INTFLAG = ~0;
		DMAC->CH[DSP_DMACH].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_TRANSACTION;
		DMAC->SWTRIGCTRL = 1UL<<DSP_DMACH;

	#elif defined(CPU_XMC48)

//		register u32 t __asm("r0");

		if (len > BLOCK_TS(~0)) { len = BLOCK_TS(~0); };

		DSP_DMA->DMACFGREG = 1;

		DSP_DMACH->CTLL = DST_INC|SRC_INC|TT_FC(0)|DEST_MSIZE(0)|SRC_MSIZE(0);
		DSP_DMACH->CTLH = BLOCK_TS(len);

		DSP_DMACH->SAR = (u32)src;
		DSP_DMACH->DAR = (u32)dst;
		DSP_DMACH->CFGL = 0;
		DSP_DMACH->CFGH = PROTCTL(1);

		DSP_DMA->CHENREG = DSP_DMA_CHEN;

	#endif
#else

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool DSP_CheckDataComplete()
{
	#ifdef CPU_SAME53

	return (HW::DMAC->CH[DSP_DMACH].CTRLA & DMCH_ENABLE) == 0 || (HW::DMAC->CH[DSP_DMACH].INTFLAG & DMCH_TCMPL);
	
	#elif defined(CPU_XMC48)

		return (DSP_DMA->CHENREG & DSP_DMA_CHST) == 0;

	#elif defined(WIN32)

		return true;
		
	#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte *spi_wrPtr = 0;
static byte *spi_rdPtr = 0;
static u16 spi_wrCount = 0;
static u16 spi_count = 0;
static u16 spi_rdCount = 0;
static byte *spi_wrPtr2 = 0;
static u16 spi_wrCount2 = 0;
static u32 spi_adr = 0;
static DSCSPI* spi_dsc = 0;
static DSCSPI* spi_lastDsc = 0;

#ifndef WIN32
static u32 SPI_CS_MASK[2] = { CS0, CS1 };
#endif

static u32 spi_timestamp = 0;

//static bool SPI_Write(DSCSPI *d);
//static bool SPI_Read(DSCSPI *d);
static bool SPI_WriteRead(DSCSPI *d);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53	

static __irq void SPI_Handler()
{
	using namespace HW;

	byte state = SPI->INTFLAG & SPI->INTENSET;

	if (state & SPI_DRE)
	{
		Pin_SPI_IRQ_Set();

		if (spi_wrCount == 0)
		{
			if (spi_wrCount2 != 0)
			{
				HW::DMAC->CH[SPI_DMACH_TX].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_TX;
				spi_wrCount2 = 0;
				spi_rdCount = 0;
			};

			SPI->INTENCLR = ~0;
			SPI->INTENSET = SPI_TXC;
		}
		else
		{
			SPI->DATA = *(spi_wrPtr++); 
			spi_wrCount--;
		};
	}
	else if (state & SPI_TXC)
	{
		Pin_SPI_IRQ_Set();
		Pin_SPI_IRQ_Clr();
		Pin_SPI_IRQ_Set();

		if (spi_rdCount != 0)
		{
			spi_rdCount = 0;

			//SPI->DATA = 0; 
			SPI->INTFLAG = SPI_TXC;

			HW::DMAC->CH[SPI_DMACH_TX].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_TX;
			HW::DMAC->CH[SPI_DMACH_RX].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_RX;

			SPI->CTRLB |= SPI_RXEN;
		}
		else
		{
			HW::DMAC->CH[SPI_DMACH_TX].CTRLA = 0;
			HW::DMAC->CH[SPI_DMACH_RX].CTRLA = 0;

			SPI->INTENCLR = ~0;
			SPI->INTFLAG = ~0;

			DSCSPI *ndsc = spi_dsc->next;
				
			spi_dsc->next = 0;

			spi_dsc->ready = true;

			SPI->CTRLB &= ~SPI_RXEN;

			PIO_CS->SET(CS0|CS1);
			
			spi_dsc = 0;

			if (ndsc != 0)
			{
				SPI_WriteRead(ndsc);
			}
			else
			{
				spi_lastDsc = 0;
			};
		};
	};

	Pin_SPI_IRQ_Clr();
}

#elif defined(CPU_XMC48)

static __irq void SPI_Handler_Write()
{
	using namespace HW;

	Pin_SPI_IRQ_Set();

	volatile u32 a = SPI->PSR_SSCMode;
	
	a &= (SPI->CCR & (RIF|AIF))|MSLSEV;

	if(a & (RIF|AIF))
	{
		SPI->PSCR = RIF|DLIF|TSIF|MSLSEV;

		SPI->PCR_SSCMode |= MSLSIEN;
		SPI->CCR = SPI__CCR|TBIEN;
	}
	else if(a & MSLSEV)
	{
		SPI->PSCR = RIF|DLIF|TSIF|MSLSEV;

		SPI_DMA->CHENREG = SPI_DMA_CHDIS;

		SPI->TRBSCR = TRBSCR_FLUSHTB;

		SPI->PCR_SSCMode = SPI__PCR;

		SPI->TCSR = SPI__TCSR|TDSSM(1);

		SPI->CCR = SPI__CCR;

		SPI->PSCR = DLIF;

		DLR->LNEN &= ~SPI_DLR_LNEN;

		DSCSPI *ndsc = spi_dsc->next;
			
		spi_dsc->next = 0;

		spi_dsc->ready = true;

		PIO_CS->SET(CS0|CS1);
		
		spi_dsc = 0;

		if (ndsc != 0)
		{
			SPI_WriteRead(ndsc);
		}
		else
		{
			spi_lastDsc = 0;
		};
	};


	Pin_SPI_IRQ_Clr();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void SPI_Handler_Read()
{
	using namespace HW;

	Pin_SPI_IRQ_Set();

	volatile u32 a = SPI->PSR_SSCMode & SPI->CCR;

	if(a & TBIF)
	{
		SPI->PSCR = TBIF|TSIF;

		if (spi_count == 0)
		{
			a = SPI->RBUF;
			a = SPI->RBUF;

			SPI->PSCR = RIF|DLIF;
			SPI->CCR = SPI__CCR|RIEN|DLIEN;
			SPI->TBUF[0] = 0;
			SPI->TCSR = SPI__TCSR|TDSSM(0);
		}
		else
		{
			a = SPI->RBUF;
			a = SPI->RBUF;

			if (spi_wrCount > 0)
			{ 
				SPI->TBUF[0] = *(spi_wrPtr++); 
				spi_wrCount--;
			}
			else
			{
				SPI->TBUF[0] = 0;
			};

			spi_count--;
		};
	}
	else if(a & DLIF)
	{
		SPI_DMA->CHENREG = SPI_DMA_CHDIS;

		SPI->PCR_SSCMode = SPI__PCR;

		SPI->TCSR = SPI__TCSR|TDSSM(1);

		SPI->CCR = SPI__CCR;

		SPI->PSCR = ~0;

		DLR->LNEN &= ~SPI_DLR_LNEN;

		DSCSPI *ndsc = spi_dsc->next;
			
		spi_dsc->next = 0;

		spi_dsc->ready = true;

		PIO_CS->SET(CS0|CS1);
		
		spi_dsc = 0;

		if (ndsc != 0)
		{
			SPI_WriteRead(ndsc);
		}
		else
		{
			spi_lastDsc = 0;
		};
	};

	Pin_SPI_IRQ_Clr();
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool SPI_WriteRead(DSCSPI *d)
{
#ifndef WIN32

	using namespace HW;

	if (spi_dsc != 0 || d == 0) { return false; };
	//if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

	spi_dsc = d;

	spi_dsc->ready = false;

	u32 alen = (spi_dsc->alen > 4) ? 4 : spi_dsc->alen; 

	spi_wrPtr = (byte*)&spi_dsc->adr;	
	spi_wrCount = spi_count = alen;

	spi_wrPtr2 = (byte*)spi_dsc->wdata;	
	spi_wrCount2 = spi_dsc->wlen;

	spi_rdPtr = (byte*)spi_dsc->rdata;	
	spi_rdCount = spi_dsc->rlen;

	spi_timestamp = GetMilliseconds();

	u32 adr = spi_dsc->adr;

	__disable_irq();

	PIO_CS->CLR(SPI_CS_MASK[spi_dsc->csnum]);

	#ifdef CPU_SAME53

		SPI->INTFLAG = ~0;
		SPI->INTENSET = SPI_DRE;
		SPI->CTRLB &= ~SPI_RXEN;

		if (spi_wrCount2 != 0)
		{
			spi_rdCount = 0;

			DmaTable[SPI_DMACH_TX].SRCADDR = spi_wrPtr2 + spi_wrCount2;
			DmaTable[SPI_DMACH_TX].DSTADDR = &SPI->DATA;
			DmaTable[SPI_DMACH_TX].DESCADDR = 0;
			DmaTable[SPI_DMACH_TX].BTCNT = spi_wrCount2;
			DmaTable[SPI_DMACH_TX].BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_SRCINC;
		}
		else if (spi_rdCount != 0)
		{
			DmaTable[SPI_DMACH_TX].SRCADDR = spi_wrPtr;
			DmaTable[SPI_DMACH_TX].DSTADDR = &SPI->DATA;
			DmaTable[SPI_DMACH_TX].DESCADDR = 0;
			DmaTable[SPI_DMACH_TX].BTCNT = spi_rdCount+1;
			DmaTable[SPI_DMACH_TX].BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE;

			DmaTable[SPI_DMACH_RX].SRCADDR = &SPI->DATA;
			DmaTable[SPI_DMACH_RX].DSTADDR = spi_rdPtr + spi_rdCount;
			DmaTable[SPI_DMACH_RX].DESCADDR = 0;
			DmaTable[SPI_DMACH_RX].BTCNT = spi_rdCount;
			DmaTable[SPI_DMACH_RX].BTCTRL = DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_DSTINC;
		};

	#elif defined(CPU_XMC48)

		//SPI->CCR = 0;

		HW::DLR->LNEN &= ~SPI_DLR_LNEN;

		SPI_DMA->CHENREG = SPI_DMA_CHDIS;
		SPI_DMA->DMACFGREG = 1;

		if (spi_wrCount2 != 0)
		{
			SPI_DMACH->CTLL = DINC(2)|SINC(0)|TT_FC(1)|DEST_MSIZE(0)|SRC_MSIZE(0);
			SPI_DMACH->CTLH = BLOCK_TS(spi_dsc->wlen);

			SPI_DMACH->SAR = (u32)spi_dsc->wdata;
			SPI_DMACH->DAR = (u32)&SPI->IN[4];
			SPI_DMACH->CFGL = HS_SEL_SRC;
			SPI_DMACH->CFGH = PROTCTL(1)|DEST_PER(SPI_DLR&7);

			SPI->TRBSCR = TRBSCR_FLUSHTB;
			SPI->TBCTR = TBCTR_SIZE8|TBCTR_LIMIT(0);

			SPI->TCSR = SPI__TCSR|TDSSM(1);

			//SPI->FDR = SPI__BAUD2FDR(spi_dsc->baud);
			SPI->CCR = SPI__CCR;
			SPI->PCR_SSCMode = SPI__PCR|SELO(1<<spi_dsc->csnum);

			VectorTableExt[SPI_IRQ] = SPI_Handler_Write;
			//CM4::NVIC->CLR_PR(SPI_IRQ);
			//CM4::NVIC->SET_ER(SPI_IRQ);
			
			SPI->PSCR = ~0;

			while(SPI->PSR_SSCMode & TBIF)
			{
				SPI->PSCR = ~0;
			};

			while(alen > 0)
			{
				SPI->IN[4] = (byte)adr;
				adr >>= 8;
				alen--;
			};

			HW::DLR->LNEN |= SPI_DLR_LNEN;
			SPI_DMA->CHENREG = SPI_DMA_CHEN;

			SPI->PSCR = ~0;
			SPI->CCR = SPI__CCR|TBIEN|RIEN;
			SPI->INPR = TBINP(SPI_INPR)|RINP(5)|PINP(5);
		}
		else if (spi_rdCount != 0)
		{
			volatile u32 t;

			SPI_DMACH->CTLL = DINC(0)|SINC(2)|TT_FC(2)|DEST_MSIZE(0)|SRC_MSIZE(0);
			SPI_DMACH->CTLH = BLOCK_TS(spi_dsc->rlen);

			SPI_DMACH->SAR = (u32)&SPI->RBUF;
			SPI_DMACH->DAR = (u32)spi_dsc->rdata;
			SPI_DMACH->CFGL = HS_SEL_DST;
			SPI_DMACH->CFGH = PROTCTL(1)|SRC_PER(SPI_DLR&7);

			SPI->RBCTR = 0;
			SPI->TBCTR = 0;

			SPI->TCSR = SPI__TCSR|TDSSM(1);

			SPI->CCR = SPI__CCR;
			SPI->PCR_SSCMode = SPI__PCR|SELO(1<<spi_dsc->csnum);

			SPI_DMA->CHENREG = SPI_DMA_CHEN;

			t = SPI->RBUF;
			t = SPI->RBUF;

	//		SPI->PSCR = ~0;

			VectorTableExt[SPI_IRQ] = SPI_Handler_Read;
			//CM4::NVIC->CLR_PR(SPI_IRQ);
			//CM4::NVIC->SET_ER(SPI_IRQ);

			HW::DLR->LNEN |= SPI_DLR_LNEN;

			SPI->INPR = RINP(0)|PINP(5)|TBINP(5);

			SPI->PSCR = ~0;
			
			while(SPI->PSR_SSCMode & TBIF)
			{
				SPI->PSCR = ~0;
			};

			SPI->CCR = SPI__CCR | TBIEN;
		
			SPI->TBUF[0] = *(spi_wrPtr++);
			spi_wrCount--;
		}
		else
		{
			SPI->TRBSCR = TRBSCR_FLUSHTB;
			SPI->TBCTR = TBCTR_SIZE8|TBCTR_LIMIT(0);

			SPI->TCSR = SPI__TCSR|TDSSM(1);

			SPI->CCR = SPI__CCR;
			SPI->PCR_SSCMode = SPI__PCR|SELO(1<<spi_dsc->csnum);

			VectorTableExt[SPI_IRQ] = SPI_Handler_Write;
			//CM4::NVIC->CLR_PR(SPI_IRQ);
			//CM4::NVIC->SET_ER(SPI_IRQ);
			
			SPI->PSCR = ~0;

			while(SPI->PSR_SSCMode & TBIF)
			{
				SPI->PSCR = ~0;
			};

			SPI->PSCR = ~0;
			SPI->CCR = SPI__CCR|RIEN|AIEN;
			SPI->INPR = TBINP(SPI_INPR)|AINP(5)|RINP(5)|PINP(5);

			while(alen > 0)
			{
				SPI->IN[4] = (byte)adr;
				adr >>= 8;
				alen--;
			};
		};

	#endif
		
	__enable_irq();

#else

#endif

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool SPI_AddRequest(DSCSPI *d)
{
	if (d == 0) { return false; };
	//if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

#ifndef WIN32

	d->next = 0;
	d->ready = false;

	u32 t = __disable_irq();

	if (spi_lastDsc == 0)
	{
		spi_lastDsc = d;

		__enable_irq();

		return SPI_WriteRead(d);
	}
	else
	{
		spi_lastDsc->next = d;
		spi_lastDsc = d;

		__enable_irq();
	};

#else

	u32 adr;

	switch (d->csnum)
	{
		case 0x0: //Accel

			if (d->rlen >= 2)
			{
				byte *p = (byte*)d->rdata;

				p[0] = 0;
				p[1] = 0;
			};
				
			d->ready = true;

			break;

		case 0x1: // FRAM

			switch(d->adr&0xFF)
			{
				case 0x2: // WRITE

					if (d->alen == 4 && d->wdata != 0 && d->wlen != 0 && fram_spi_WREN)
					{
						adr = ReverseDword(d->adr & ~0xFF);
						adr %= sizeof(fram_SPI_Mem);

						u16 count = d->wlen;
						byte *s = (byte*)d->wdata;
						byte *d = fram_SPI_Mem + adr;

						while (count-- != 0) { *(d++) = *(s++); adr++; if (adr >= sizeof(fram_SPI_Mem)) { adr = 0; d = fram_SPI_Mem; }; };
					};

					fram_spi_WREN = false;

					break;

				case 0x3: // READ

					if (d->alen == 4 && d->rdata != 0 && d->rlen != 0)
					{
						adr = ReverseDword(d->adr & ~0xFF);
						adr %= sizeof(fram_SPI_Mem);

						u16 count = d->rlen;

						byte *p = (byte*)(d->rdata);
						byte *s = fram_SPI_Mem + adr;

						while (count-- != 0) { *(p++) = *(s++); adr++; if (adr >= sizeof(fram_SPI_Mem)) { adr = 0; s = fram_SPI_Mem; }; };
					};

					fram_spi_WREN = false;

					break;

				case 0x6: // WREN

					fram_spi_WREN = (d->alen == 1);

					break;

				default:	fram_spi_WREN = false; break;
			};

			d->ready = true;

			break;
	};

#endif

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SPI_Init();

bool SPI_Update()
{
	bool result = false;

#ifdef CPU_SAME53

#elif defined(CPU_XMC48)

	using namespace HW;

	static TM32 tm;

	__disable_irq();

	if (spi_dsc != 0)
	{
		if (!spi_dsc->ready && (GetMilliseconds() - spi_timestamp) > 100)
		{
			result = true;

			HW::Peripheral_Disable(SPI_PID);

			DSCSPI *dsc = spi_dsc;

			spi_dsc = 0;

			SPI_Init();

			SPI_WriteRead(dsc);
		};
	};
	
	__enable_irq();

#endif

	return result;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SPI_Init()
{
#ifndef WIN32

	using namespace HW;

	#ifdef CPU_SAME53	

		HW::GCLK->PCHCTRL[GCLK_SERCOM0_CORE] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;	// 25 MHz

		MCLK->APBAMASK |= APBA_SERCOM0;

		PIO_SPCK->SetWRCONFIG(SPCK, PORT_PMUX(2)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX);
		PIO_MOSI->SetWRCONFIG(MOSI, PORT_PMUX(2)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX);
		PIO_MISO->SetWRCONFIG(MISO, PORT_PMUX(2)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX);
		PIO_CS->DIRSET = CS0|CS1; 
		PIO_CS->SetWRCONFIG(CS0|CS1, PORT_WRPINCFG|PORT_WRPMUX);
		PIO_CS->SET(CS0|CS1); 

		SPI->CTRLA = SPI_SWRST;

		while(SPI->SYNCBUSY);

		SPI->CTRLA = SERCOM_MODE_SPI_MASTER;

		SPI->CTRLA = SERCOM_MODE_SPI_MASTER|SPI_CPHA|SPI_DIPO(2)|SPI_DOPO(0);
		SPI->CTRLB = 0;
		SPI->CTRLC = 1;
		SPI->BAUD = 12;

		SPI->DBGCTRL = 1;

		SPI->CTRLA |= SPI_ENABLE;

		while(SPI->SYNCBUSY);

		SPI->STATUS = ~0;

		VectorTableExt[SERCOM0_0_IRQ] = SPI_Handler;
		VectorTableExt[SERCOM0_1_IRQ] = SPI_Handler;
		VectorTableExt[SERCOM0_3_IRQ] = SPI_Handler;
		CM4::NVIC->CLR_PR(SERCOM0_0_IRQ);
		CM4::NVIC->CLR_PR(SERCOM0_1_IRQ);
		CM4::NVIC->CLR_PR(SERCOM0_3_IRQ);
		CM4::NVIC->SET_ER(SERCOM0_0_IRQ);
		CM4::NVIC->SET_ER(SERCOM0_1_IRQ);
		CM4::NVIC->SET_ER(SERCOM0_3_IRQ);

	#elif defined(CPU_XMC48)

		HW::Peripheral_Enable(SPI_PID);

		SPI->KSCFG = MODEN|BPMODEN|BPNOM|NOMCFG(0);

		SPI->CCR = 0;

		SPI->FDR = SPI__FDR;
		SPI->BRG = SPI__BRG;
	    
		SPI->SCTR = SPI__SCTR;
		SPI->TCSR = SPI__TCSR;

		SPI->PCR_SSCMode = SPI__PCR;

		SPI->PSCR = ~0;

		SPI->CCR = 0;

		SPI->DX0CR = SPI__DX0CR;
		SPI->DX1CR = SPI__DX1CR;

		SPI->TBCTR = 0;// TBCTR_SIZE8|TBCTR_LIMIT(0);
		SPI->RBCTR = 0;//RBCTR_SIZE8|RBCTR_LIMIT(0);

		SPI->CCR = SPI__CCR;

		PIO_SPCK->ModePin(PIN_SPCK, A2PP);
		PIO_MOSI->ModePin(PIN_MOSI, A2PP);
 		PIO_MISO->ModePin(PIN_MISO, I0DNP);
		PIO_CS->ModePin(PIN_CS0, G_PP);
		PIO_CS->ModePin(PIN_CS1, G_PP);
		PIO_CS->SET(CS0|CS1);

		VectorTableExt[SPI_IRQ] = SPI_Handler_Read;
		CM4::NVIC->CLR_PR(SPI_IRQ);
		CM4::NVIC->SET_ER(SPI_IRQ);


		//SPI->PCR_SSCMode = SPI__PCR|SELO(1);
		
	//	SPI->PSCR |= TBIF;

	//	SPI->CCR = SPI__CCR|TBIEN;
	//	SPI->INPR = 0;

		//SPI->IN[0] = 0x55;
		//SPI->IN[0] = 0x55;

		//while ((SPI->PSR & TSIF) == 0);

	//	SPI->CCR = SPI__CCR;

	#endif
#else

	HANDLE h;

	h = CreateFile("FRAM_SPI_STORE.BIN", GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	ReadFile(h, fram_SPI_Mem, sizeof(fram_SPI_Mem), &bytes, 0);
	CloseHandle(h);

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

void SPI_Destroy()
{
	HANDLE h;

	h = CreateFile("FRAM_SPI_STORE.BIN", GENERIC_WRITE, 0, 0, OPEN_ALWAYS, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	if (!WriteFile(h, fram_SPI_Mem, sizeof(fram_SPI_Mem), &bytes, 0))
	{
		dword le = GetLastError();
	};

	CloseHandle(h);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

//extern dword DI;
//extern dword DO;
char pressedKey;
//extern  dword maskLED[16];

//byte _array1024[0x60000]; 

HWND  hMainWnd;

HCURSOR arrowCursor;
HCURSOR handCursor;

HBRUSH	redBrush;
HBRUSH	yelBrush;
HBRUSH	grnBrush;
HBRUSH	gryBrush;

RECT rectLed[10] = { {20, 41, 33, 54}, {20, 66, 33, 79}, {20, 91, 33, 104}, {21, 117, 22, 118}, {20, 141, 33, 154}, {218, 145, 219, 146}, {217, 116, 230, 129}, {217, 91, 230, 104}, {217, 66, 230, 79}, {217, 41, 230, 54}  }; 
HBRUSH* brushLed[10] = { &yelBrush, &yelBrush, &yelBrush, &gryBrush, &grnBrush, &gryBrush, &redBrush, &redBrush, &redBrush, &redBrush };

//int x,y,Depth;

HFONT font1;
HFONT font2;

HDC memdc;
HBITMAP membm;

//HANDLE facebitmap;

static const u32 secBufferWidth = 80;
static const u32 secBufferHeight = 12;
static const u32 fontWidth = 12;
static const u32 fontHeight = 16;


static char secBuffer[secBufferWidth*secBufferHeight*2];

static u32 pallete[16] = {	0x000000,	0x800000,	0x008000,	0x808000,	0x000080,	0x800080,	0x008080,	0xC0C0C0,
							0x808080,	0xFF0000,	0x00FF00,	0xFFFF00,	0x0000FF,	0xFF00FF,	0x00FFFF,	0xFFFFFF };

const char lpAPPNAME[] = "АС76Ц_УПР";

int screenWidth = 0, screenHeight = 0;

LRESULT CALLBACK WindowProc (HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

static __int64		tickCounter = 0;

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

static void InitDisplay()
{
	WNDCLASS		    wcl;

	wcl.hInstance		= NULL;
	wcl.lpszClassName	= lpAPPNAME;
	wcl.lpfnWndProc		= WindowProc;
	wcl.style	    	= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;

	wcl.hIcon	    	= NULL;
	wcl.hCursor	    	= NULL;
	wcl.lpszMenuName	= NULL;

	wcl.cbClsExtra		= 0;
	wcl.cbWndExtra		= 0;
	wcl.hbrBackground	= NULL;

	RegisterClass (&wcl);

	int sx = screenWidth = GetSystemMetrics (SM_CXSCREEN);
	int sy = screenHeight = GetSystemMetrics (SM_CYSCREEN);

	hMainWnd = CreateWindowEx (0, lpAPPNAME, lpAPPNAME,	WS_DLGFRAME|WS_POPUP, 0, 0,	640, 480, NULL,	NULL, NULL, NULL);

	if(!hMainWnd) 
	{
		cputs("Error creating window\r\n");
		exit(0);
	};

	RECT rect;

	if (GetClientRect(hMainWnd, &rect))
	{
		MoveWindow(hMainWnd, 0, 0, 641 + 768 - rect.right - 2, 481 - rect.bottom - 1 + 287, true);
	};

	ShowWindow (hMainWnd, SW_SHOWNORMAL);

	font1 = CreateFont(30, 14, 0, 0, 100, false, false, false, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, PROOF_QUALITY, FIXED_PITCH|FF_DONTCARE, "Lucida Console");
	font2 = CreateFont(fontHeight, fontWidth-2, 0, 0, 100, false, false, false, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, PROOF_QUALITY, FIXED_PITCH|FF_DONTCARE, "Lucida Console");

	if (font1 == 0 || font2 == 0)
	{
		cputs("Error creating font\r\n");
		exit(0);
	};

	GetClientRect(hMainWnd, &rect);
	HDC hdc = GetDC(hMainWnd);
    memdc = CreateCompatibleDC(hdc);
	membm = CreateCompatibleBitmap(hdc, rect.right - rect.left + 1, rect.bottom - rect.top + 1 + secBufferHeight*fontHeight);
    SelectObject(memdc, membm);
	ReleaseDC(hMainWnd, hdc);


	arrowCursor = LoadCursor(0, IDC_ARROW);
	handCursor = LoadCursor(0, IDC_HAND);

	if (arrowCursor == 0 || handCursor == 0)
	{
		cputs("Error loading cursors\r\n");
		exit(0);
	};

	LOGBRUSH lb;

	lb.lbStyle = BS_SOLID;
	lb.lbColor = RGB(0xFF, 0, 0);

	redBrush = CreateBrushIndirect(&lb);

	lb.lbColor = RGB(0xFF, 0xFF, 0);

	yelBrush = CreateBrushIndirect(&lb);

	lb.lbColor = RGB(0x7F, 0x7F, 0x7F);

	gryBrush = CreateBrushIndirect(&lb);

	lb.lbColor = RGB(0, 0xFF, 0);

	grnBrush = CreateBrushIndirect(&lb);

	for (u32 i = 0; i < sizeof(secBuffer); i+=2)
	{
		secBuffer[i] = 0x20;
		secBuffer[i+1] = 0xF0;
	};
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

void UpdateDisplay()
{
	static byte curChar = 0;
	//static byte c = 0, i = 0;
	//static byte flashMask = 0;
	static const byte a[4] = { 0x80, 0x80+0x40, 0x80+20, 0x80+0x40+20 };

	MSG msg;

	static dword pt = 0;
	static TM32 tm;

	u32 t = GetTickCount();

	if ((t-pt) >= 2)
	{
		pt = t;

		if(PeekMessage (&msg, NULL, 0, 0, PM_NOREMOVE))
		{
			GetMessage (&msg, NULL, 0, 0);

			TranslateMessage (&msg);

			DispatchMessage (&msg);
		};
	};

	static char buf[80];
	static char buf1[sizeof(secBuffer)];

	if (tm.Check(20))
	{
		bool rd = true;

		if (rd)
		{
			//SelectObject(memdc, font1);
			//SetBkColor(memdc, 0x074C00);
			//SetTextColor(memdc, 0x00FF00);
			//TextOut(memdc, 443, 53, buf, 20);
			//TextOut(memdc, 443, 30*1+53, buf+20, 20);
			//TextOut(memdc, 443, 30*2+53, buf+40, 20);
			//TextOut(memdc, 443, 30*3+53, buf+60, 20);

			SelectObject(memdc, font2);

			for (u32 j = 0; j < secBufferHeight; j++)
			{
				for (u32 i = 0; i < secBufferWidth; i++)
				{
					u32 n = (j*secBufferWidth+i)*2;

					u8 t = secBuffer[n+1];

					SetBkColor(memdc, pallete[(t>>4)&0xF]);
					SetTextColor(memdc, pallete[t&0xF]);
					TextOut(memdc, i*fontWidth, j*fontHeight+287, secBuffer+n, 1);
				};
			};

			RedrawWindow(hMainWnd, 0, 0, RDW_INVALIDATE);

		}; // if (rd)

	}; // if ((t-pt) > 10)

}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

LRESULT CALLBACK WindowProc (HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int i;
	HDC hdc;
	PAINTSTRUCT ps;
	bool c;
	static int x, y;
	int x0, y0, r0;
	static char key = 0;
	static char pkey = 0;

	RECT rect;

	static bool move = false;
	static int movex, movey = 0;

//	char *buf = (char*)screenBuffer;

    switch (message)
	{
        case WM_CREATE:

            break;

	    case WM_DESTROY:

		    break;

        case WM_PAINT:

			if (GetUpdateRect(hWnd, &rect, false)) 
			{
				hdc = BeginPaint(hWnd, &ps);

				//printf("Update RECT: %li, %li, %li, %li\r\n", ps.rcPaint.left, ps.rcPaint.top, ps.rcPaint.right, ps.rcPaint.bottom);

				c = BitBlt(hdc, ps.rcPaint.left, ps.rcPaint.top, ps.rcPaint.right - ps.rcPaint.left + 1, ps.rcPaint.bottom - ps.rcPaint.top + 1, memdc, ps.rcPaint.left, ps.rcPaint.top, SRCCOPY);
	 
				EndPaint(hWnd, &ps);
			};

            break;

        case WM_CHAR:

			pressedKey = wParam;

			if (pressedKey == '`')
			{
				GetWindowRect(hWnd, &rect);

				if ((rect.bottom-rect.top) > 340)
				{
					MoveWindow(hWnd, rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top-fontHeight*secBufferHeight, true); 
				}
				else
				{
					MoveWindow(hWnd, rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top+fontHeight*secBufferHeight, true); 
				};
			};

            break;

		case WM_MOUSEMOVE:

			x = LOWORD(lParam); y = HIWORD(lParam);

			if (move)
			{
				GetWindowRect(hWnd, &rect);
				SetWindowPos(hWnd, HWND_TOP, rect.left+x-movex, rect.top+y-movey, 0, 0, SWP_NOSIZE); 
			};

			return 0;

			break;

		case WM_MOVING:

			return TRUE;

			break;

		case WM_SYSCOMMAND:

			return 0;

			break;

		case WM_LBUTTONDOWN:

			move = true;
			movex = x; movey = y;

			SetCapture(hWnd);

			return 0;

			break;

		case WM_LBUTTONUP:

			move = false;

			ReleaseCapture();

			return 0;

			break;

		case WM_MBUTTONDOWN:

			ShowWindow(hWnd, SW_MINIMIZE);

			return 0;

			break;

		case WM_ACTIVATE:

			if (HIWORD(wParam) != 0 && LOWORD(wParam) != 0)
			{
				ShowWindow(hWnd, SW_NORMAL);
			};

			break;

		case WM_CLOSE:

			//run = false;

			break;
	};
    
	return DefWindowProc(hWnd, message, wParam, lParam);
}

#endif		
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

int PutString(u32 x, u32 y, byte c, const char *str)
{
	char *dst = secBuffer+(y*secBufferWidth+x)*2;
	dword i = secBufferWidth-x;

	while (*str != 0 && i > 0)
	{
		*(dst++) = *(str++);
		*(dst++) = c;
		i -= 1;
	};

	return secBufferWidth-x-i;
}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef	WIN32

int Printf(u32 xx, u32 yy, byte c, const char *format, ... )
{
	char buf[1024];

	va_list arglist;

    va_start(arglist, format);
    vsprintf(buf, format, arglist);
    va_end(arglist);

	return PutString(xx, yy, c, buf);
}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
#ifdef CPU_SAME53	
	
	using namespace HW;

//	HW::PIOA->BSET(13);

	HW::GCLK->GENCTRL[GEN_32K]	= GCLK_DIV(1)	|GCLK_SRC_OSCULP32K	|GCLK_GENEN;

	HW::GCLK->GENCTRL[GEN_1M]	= GCLK_DIV(25)	|GCLK_SRC_XOSC1		|GCLK_GENEN		|GCLK_OE;

	HW::GCLK->GENCTRL[GEN_25M]	= GCLK_DIV(1)	|GCLK_SRC_XOSC1		|GCLK_GENEN;

//	HW::GCLK->GENCTRL[GEN_500K] = GCLK_DIV(50)	|GCLK_SRC_XOSC1		|GCLK_GENEN;


	HW::MCLK->APBAMASK |= APBA_EIC;
	HW::GCLK->PCHCTRL[GCLK_EIC] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;

	HW::MCLK->APBBMASK |= APBB_EVSYS;

	HW::GCLK->PCHCTRL[GCLK_SERCOM_SLOW]		= GCLK_GEN(GEN_32K)|GCLK_CHEN;	// 32 kHz
	HW::GCLK->PCHCTRL[GCLK_SERCOM5_CORE]	= GCLK_GEN(GEN_MCK)|GCLK_CHEN;	
	HW::GCLK->PCHCTRL[GCLK_SERCOM6_CORE]	= GCLK_GEN(GEN_MCK)|GCLK_CHEN;	
	HW::GCLK->PCHCTRL[GCLK_SERCOM7_CORE]	= GCLK_GEN(GEN_MCK)|GCLK_CHEN;	

	HW::MCLK->APBDMASK |= APBD_SERCOM5|APBD_SERCOM6|APBD_SERCOM7;


#endif

	Init_time();
	NAND_Init();
	I2C_Init();
	SPI_Init();

#ifndef WIN32

	InitClock();

	//InitManTransmit();
	InitManRecieve();
	Init_CRC_CCITT_DMA();

	InitManTransmit2();

	Init_Sync_Rot();

	InitShaft();

	EnableVCORE();

	WDT_Init();

#else

	InitDisplay();

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
#ifndef WIN32

	static byte i = 0;

	static Deb db(false, 20);

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateShaft();	);
		CALL( SPI_Update();		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
