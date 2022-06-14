#ifndef HW_CONF_H__20_04_2022__16_00
#define HW_CONF_H__20_04_2022__16_00

#include "types.h"
#include "core.h"

#define MCK_MHz 200
#define MCK (MCK_MHz*1000000)
#define NS2CLK(x) (((x)*MCK_MHz+500)/1000)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define FRAM_SPI_MAINVARS_ADR 0
#define FRAM_SPI_SESSIONS_ADR 0x200

#define FRAM_I2C_MAINVARS_ADR 0
#define FRAM_I2C_SESSIONS_ADR 0x200

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_XMC48

#define CPUCLK_MHz MCK_MHz

#if (CPUCLK_MHz > 100)
#define SYSCLK_MHz		((CPUCLK_MHz+1)/2)
#define __SYSCLK_DIV	1
#define __EBU_DIV		((CPUCLK_MHz/100)-1)
#else
#define SYSCLK_MHz		CPUCLK_MHz
#define __SYSCLK_DIV	0
#define __EBU_DIV		0
#endif

#define EBUCLK_MHz (CPUCLK_MHz/(__EBU_DIV+1))

#define CPUCLK (CPUCLK_MHz*1000000)
#define SYSCLK (SYSCLK_MHz*1000000)
#define EBUCLK (EBUCLK_MHz*1000000)

#define NS2CCLK(x)		(((x)*CPUCLK_MHz+500)/1000)
#define NS2SCLK(x)		(((x)*SYSCLK_MHz+500)/1000)
#define NS2EBUCLK(x)	(((x)*EBUCLK_MHz+500)/1000)

#define __PBCLKCR   (__SYSCLK_DIV)
#define __CCUCLKCR  (__SYSCLK_DIV)
#define __EBUCLKCR  (__EBU_DIV)

#endif

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
	#define GEN_EXT32K	5

	#define	NAND_DMA			DMA_CH0
	#define	COM1_DMA			DMA_CH1
	#define	COM2_DMA			DMA_CH2
	#define	COM3_DMA			DMA_CH3
	#define	SPI_DMA_TX			DMA_CH4
	#define	SPI_DMA_RX			DMA_CH5
	#define	NAND_MEMCOPY_DMA	DMA_CH6
	#define	I2C_DMA				DMA_CH7
	#define	DSP_DMA				DMA_CH30
	#define	CRC_DMA				DMA_CH31

	#define I2C					HW::I2C3
	#define PIO_I2C				HW::PIOA 
	#define PIN_SDA				22 
	#define PIN_SCL				23 
	#define SDA					(1<<PIN_SDA) 
	#define SCL					(1<<PIN_SCL) 
	#define I2C_TRIGSRC_RX		DMCH_TRIGSRC_SERCOM3_RX
	#define I2C_TRIGSRC_TX		DMCH_TRIGSRC_SERCOM3_TX

	//__align(16) T_HW::DMADESC DmaTable[32];
	//__align(16) T_HW::DMADESC DmaWRB[32];

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

	#define US2MT(v)		(v)
	#define BAUD2CLK(x)		((u32)(1000000/x+0.5))

	#define MANT_IRQ		TC0_IRQ
	#define MANR_IRQ		TCC2_1_IRQ
	//#define MANR_EXTINT		11
	#define MANR_EXTINT		7
	#define ManT_SET_PR(v)				{ ManRT->PERBUF = (v); }
	#define ManT_SET_CR(v)				{ ManRT->CCBUF[0] = (v); ManRT->CCBUF[1] = (v); }
	#define ManT_SHADOW_SYNC()			

	
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

	#define CLOCK_IRQ		0//SCU_0_IRQn
	#define PIO_32kHz		HW::PIOB 
	#define PIN_32kHz		19 
	

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


	// GPDMA0 DLR_SRSEL0
	#define DRL_RS0					DRL0_USIC0_SR0	// UART0
	#define DRL_RS1					DRL1_USIC1_SR0	// SPI
	#define DRL_RS2					DRL2_USIC0_SR1	// UART1			
	#define DRL_RS3					15					
	#define DRL_RS4					15					
	#define DRL_RS5					15					
	#define DRL_RS6					15					
	#define DRL_RS7					15

	// GPDMA1 DLR_SRSEL1
	#define DRL_RS8					DRL8_USIC2_SR0	// I2C					
	#define DRL_RS9					DRL9_USIC2_SR1	// UART2				
	#define DRL_RS10				15				
	#define DRL_RS11				15					

	#define UART0					HW::USIC0_CH0
	#define UART1					HW::USIC0_CH1
	#define SPI						HW::USIC1_CH0
	//#define 						HW::USIC1_CH1
	#define I2C						HW::USIC2_CH0
	#define UART2					HW::USIC2_CH1

	#define	UART0_DMA				DMA_CH0
	#define	UART1_DMA				DMA_CH1
	//#define						DMA_CH2
	//#define						DMA_CH3
	#define	NAND_MEMCOPY_DMA		DMA_CH4
	#define	DSP_DMA					DMA_CH6
	#define	NAND_DMA				DMA_CH7

	#define	I2C_DMA					DMA_CH8
	#define	UART2_DMA				DMA_CH9
	#define	CRC_DMA					DMA_CH10
	//#define						DMA_CH11


	#define	SPI_DMA					HW::GPDMA0
	#define	SPI_DMACH				HW::GPDMA0_CH5
	#define	SPI_DMA_CHEN			(0x101<<5)
	#define	SPI_DMA_CHDIS			(0x100<<5)
	#define	SPI_DMA_CHST			(1<<5)
	#define	SPI_DLR					(1)
	#define	SPI_DLR_LNEN			(1<<SPI_DLR)

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
	#define US2MT(v)				((u16)((SYSCLK_MHz*(v)+(1<<ManRT_PSC)/2)>>ManRT_PSC))
	#define BAUD2CLK(x)				((u32)((SYSCLK*1.0/(1<<ManRT_PSC))/x+0.5))

	#define MANT_IRQ				CCU41_0_IRQn
	#define MANR_IRQ				CCU41_2_IRQn

	#define ManCCU_GIDLC			(CCU4_CS1I | CCU4_CS3I | CCU4_SPRB)	// (CCU4_CS1I | CCU4_CS2I | CCU4_SPRB)
	#define ManCCU_GCSS				(CCU4_S1SE | CCU4_S3SE)				// (CCU4_S1SE | CCU4_S2SE)
	#define ManRT_INS				(CC4_EV0IS(2) | CC4_EV0EM_BOTH_EDGES | CC4_LPF0M_7CLK)
	#define ManTmr_INS				(CC4_EV0IS(12+3) | CC4_EV0EM_RISING_EDGE);
//	#define ManRT_SRS				CC4_POSR(3)

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
//	#define ManT_PSC				3					// 0.04us
	#define ManT_SET_PR(v)			{ ManT1->PRS = (v); ManT2->PRS = (v); ManT3->PRS = (v); }
	#define ManT_SET_CR(v)			{ ManT1->CR2S = (v); ManT2->CR1S = (v); ManT2->CR2S = (v); ManT3->CR1S = (v);}
	#define ManT_SHADOW_SYNC()		{ ManT_CCU8->GCSS = ManT_CCU8_GCSS; }	
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

	#define US2SRT(v)				(((SYSCLK_MHz*(v)+SyncRot_DIV/2)/SyncRot_DIV))

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

	#define CLOCK_IRQ				SCU_0_IRQn

	#define PIO_URXD0				HW::P1
	#define PIO_UTXD0				HW::P1
	#define PIO_RTS0				HW::P1

	#define PIO_URXD1				HW::P4
	#define PIO_UTXD1				HW::P4
	#define PIO_RTS1				HW::P1

	#define PIO_URXD2				HW::P6
	#define PIO_UTXD2				HW::P6
	#define PIO_RTS2				HW::P1

	#define PIN_URXD0				4 
	#define PIN_UTXD0				5 
	#define PIN_RTS0				0 
	#define PIN_URXD1				6 
	#define PIN_UTXD1				7
	#define PIN_RTS1				8 
	#define PIN_URXD2				3 
	#define PIN_UTXD2				4
	#define PIN_RTS2				13 

	#define URXD0					(1<<PIN_URXD0) 
	#define UTXD0					(1<<PIN_UTXD0) 
	#define RTS0					(1<<PIN_RTS0) 
	#define URXD1					(1<<PIN_URXD1) 
	#define UTXD1					(1<<PIN_UTXD1) 
	#define RTS1					(1<<PIN_RTS1) 
	#define URXD2					(1<<PIN_URXD2) 
	#define UTXD2					(1<<PIN_UTXD2) 
	#define RTS2					(1<<PIN_RTS2) 


	/*******************************************************************************
	 * MACROS
	 *******************************************************************************/
	#define	OFI_FREQUENCY        (24000000UL)  /**< 24MHz Backup Clock (fOFI) frequency. */
	#define OSI_FREQUENCY        (32768UL)    /**< 32KHz Internal Slow Clock source (fOSI) frequency. */  

	#define XMC4800_F144x2048

	#define CHIPID_LOC ((uint8_t *)0x20000000UL)

	#define PMU_FLASH_WS          (NS2CCLK(30))	//(0x3U)

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

		#define PLL_K2DIV	((VCO_NOM/CPUCLK)-1)
		#define PLL_PDIV	(((OSCHP_FREQUENCY-VCO_IN_MAX)*2/VCO_IN_MAX+1)/2)
		#define PLL_NDIV	((CPUCLK*(PLL_K2DIV+1)*2/(OSCHP_FREQUENCY/(PLL_PDIV+1))+1)/2-1) // (7U) 

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
//	#define __PBCLKCR   (0x00000000UL)
//	#define __CCUCLKCR  (0x00000000UL)
	#define __WDTCLKCR  (0x00000000UL)
//	#define __EBUCLKCR  (0x00000003UL)
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

	#define I2C__FDR ((1024 - (((SYSCLK + 400000/2) / 400000 + 8) / 16)) | DM(1))

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

	#define SPI__FDR ((1024 - ((SYSCLK + SPI__BAUD/2) / SPI__BAUD + 1) / 2) | DM(1))

	#define SPI__BAUD2FDR(v) ((1024 - ((SYSCLK + (v)/2) / (v) + 1) / 2) | DM(1))

	#define SPI__TCSR (TDEN(1)|HPCMD(0))

	static void delay(u32 cycles) { for(volatile u32 i = 0UL; i < cycles ;++i) { __nop(); }}

#elif defined(WIN32) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define BAUD2CLK(x)				(x)
	#define MT(v)					(v)

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




#endif // HW_CONF_H__20_04_2022__16_00
