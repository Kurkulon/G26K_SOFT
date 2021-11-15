#ifndef BF592_H__03_03_2014__11_41
#define BF592_H__03_03_2014__11_41

#ifndef CORETYPE_BF592
#error  Must #include "core.h"
#endif 

#include <cdefBF592-A.h>
#include <sysreg.h>

#include "types.h"



#ifndef WIN32
#define MK_PTR(n,p)  T_HW::S_##n * const n = ((T_HW::S_##n*)(p))
#else
extern byte core_sys_array[0x100000]; 
#define MK_PTR(n,p)  T_HW::S_##n * const n = ((T_HW::S_##n*)(core_sys_array-0x40000000+p))
#endif


#define MKPID(n,i) n##_M=(1UL<<(i&31)), n##_I=i

namespace T_HW
{
	typedef volatile u16 BF_R16;// Hardware register definition
	typedef volatile u32 BF_REG;// Hardware register definition
	typedef volatile void * BF_PTR;// Hardware register definition

	typedef void(*BF_IHP)();	// Interrupt handler pointer

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_SMC
	{
		BF_REG	NFC_CFG;	
		BF_REG	NFC_CTRL;	
		BF_REG	NFC_SR;	
		BF_REG	NFC_IER;	
		BF_REG	NFC_IDR;	
		BF_REG	NFC_IMR;	
		BF_REG	NFC_ADDR;	
		BF_REG	NFC_BANK;	
		BF_REG	ECC_CTRL;	
		BF_REG	ECC_MD;	
		BF_REG	ECC_SR1;	
		BF_REG	ECC_PR0;	
		BF_REG	ECC_PR1;	
		BF_REG	ECC_SR2;	
		BF_REG	ECC_PR2;	
		BF_REG	ECC_PR3;	
		BF_REG	ECC_PR4;	
		BF_REG	ECC_PR5;	
		BF_REG	ECC_PR6;	
		BF_REG	ECC_PR7;	
		BF_REG	ECC_PR8;	
		BF_REG	ECC_PR9;	
		BF_REG	ECC_PR10;	
		BF_REG	ECC_PR11;	
		BF_REG	ECC_PR12;	
		BF_REG	ECC_PR13;	
		BF_REG	ECC_PR14;	
		BF_REG	ECC_PR15;	

		struct S_CSR { BF_REG	SETUP, PULSE, CYCLE, TIMINGS, MODE;	} CSR[8];

		BF_REG	OCMS;	
		BF_REG	KEY1;	
		BF_REG	KEY2;	
		BF_REG	WPCR;	
		BF_REG	WPSR;	
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_PMC
	{
		BF_REG	SCER;
		BF_REG	SCDR;
		BF_REG	SCSR;
		BF_REG	z__reserved1;
		BF_REG	PCER0;
		BF_REG	PCDR0;
		BF_REG	PCSR0;
		BF_REG	UTMICR;
		BF_REG	MOR;
		BF_REG	MCFR;
		BF_REG	PLLAR;
		BF_REG	z__reserved3;
		BF_REG	MCKR;
		BF_REG	z__reserved4;
		BF_REG	USBCR;
		BF_REG	z__reserved5;
		BF_REG	PCK[3];
		BF_REG	z__reserved6[5];
		BF_REG	IER;
		BF_REG	IDR;
		BF_REG	SR;
		BF_REG	IMR;
		BF_REG	FSMR;
		BF_REG	FSPR;
		BF_REG	FOCR;
		BF_REG	z__reserved7[26];
		BF_REG	WPMR;
		BF_REG	WPSR;
		BF_REG	z__reserved8[5];
		BF_REG	PCER1;
		BF_REG	PCDR1;
		BF_REG	PCSR1;
		BF_REG	PCR;
	};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//struct S_EBI
	//{
	//	BF_REG	CSA;
	//	BF_REG	z__reserved1[3];
	//	S_SMC		SMC;
	//};

	//S_EBI * const EBI = MK_PTR(S_EBI,0xFFFFFF80);

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_PIO
	{
		BF_REG PER;
		BF_REG PDR;
		BF_REG PSR;
		BF_REG z__reserved1;
		BF_REG OER;
		BF_REG ODR;
		BF_REG OSR;
		BF_REG z__reserved2;
		BF_REG IFER;
		BF_REG IFDR;
		BF_REG IFSR;
		BF_REG z__reserved3;
		BF_REG SODR;
		BF_REG CODR;
		BF_REG ODSR;
		BF_REG PDSR;
		BF_REG IER;
		BF_REG IDR;
		BF_REG IMR;
		BF_REG ISR;
		BF_REG MDER;
		BF_REG MDDR;
		BF_REG MDSR;
		BF_REG z__reserved4;
		BF_REG PUDR;
		BF_REG PUER;
		BF_REG PUSR;
		BF_REG z__reserved5;
		BF_REG ABSR;
		BF_REG z__reserved6[3];
		BF_REG SCIFSR;
		BF_REG DIFSR;
		BF_REG IFDGSR;
		BF_REG SCDR;
		BF_REG z__reserved7[4];
		BF_REG OWER;
		BF_REG OWDR;
		BF_REG OWSR;
		BF_REG z__reserved8;
		BF_REG AIMER;
		BF_REG AIMDR;
		BF_REG AIMMR;
		BF_REG z__reserved9;
		BF_REG ESR;
		BF_REG LSR;
		BF_REG ELSR;
		BF_REG z__reserved10;
		BF_REG FELLSR;
		BF_REG REHLSR;
		BF_REG FRLHSTR;
		BF_REG z__reserved11;
		BF_REG LOCKSR;
		BF_REG WPMR;
		BF_REG WPSR;
	};
	
	typedef S_PIO S_PIOA, S_PIOB, S_PIOC, S_PIOD, S_PIOE;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_MC
	{
		BF_REG RCR;	
		BF_REG ASR;	
		BF_REG AASR;	
		BF_REG _RESERVED;	
		BF_REG PUIA0;	
		BF_REG PUIA1;	
		BF_REG PUIA2;	
		BF_REG PUIA3;	
		BF_REG PUIA4;	
		BF_REG PUIA5;	
		BF_REG PUIA6;	
		BF_REG PUIA7;	
		BF_REG PUIA8;	
		BF_REG PUIA9;	
		BF_REG PUIA10;	
		BF_REG PUIA11;	
		BF_REG PUIA12;	
		BF_REG PUIA13;	
		BF_REG PUIA14;	
		BF_REG PUIA15;	
		BF_REG PUP;	
		BF_REG PUER;	
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_TC
	{
		struct 
		{
			BF_REG CCR;
			BF_REG CMR;
			BF_REG SMMR;
			BF_REG z__reserved1;
			BF_REG CV;
			BF_REG RA;
			BF_REG RB;
			BF_REG RC;
			BF_REG SR;
			BF_REG IER;
			BF_REG IDR;
			BF_REG IMR;
			BF_REG z__reserved2[4];
		} C0, C1, C2;

		BF_REG BCR;
		BF_REG BMR;

		BF_REG QIER;
		BF_REG QIDR;
		BF_REG QIMR;
		BF_REG QISR;
		BF_REG FMR;
		BF_REG WPMR;
	};

	typedef S_TC S_TC0, S_TC1, S_TC2;


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_PDC
	{
		BF_PTR RPR;	// Receive Pointer Register	
		BF_REG RCR;	// Receive Counter Register
		BF_PTR TPR;	// Transmit Pointer Register
		BF_REG TCR;	// Transmit Counter Register
		BF_PTR RNPR;	// Receive Next Pointer Register	
		BF_REG RNCR;	// Receive Next Pointer Register
		BF_PTR TNPR;	// Transmit Next Pointer Register
		BF_REG TNCR;	// Transmit Next Counter Register
		BF_REG PTCR;	// PDC Transfer Control Register	
		BF_REG PTSR;	// PDC Transfer Status Register
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_UDP	// 0xFFFB0000
	{
		BF_REG FRM_NUM;
		BF_REG GLB_STAT;
		BF_REG FADDR;
		BF_REG z__reserved1;
		BF_REG IER;
		BF_REG IDR;
		BF_REG IMR;
		BF_REG ISR;
		BF_REG ICR;
		BF_REG z__reserved2;
		BF_REG RST_EP;
		BF_REG z__reserved3;
		BF_REG CSR[8];
		BF_REG FDR[8];
		BF_REG z__reserved4;
		BF_REG TXVC;
	};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_USART
	{
		BF_REG CR;				// Control Register 	
		BF_REG MR;				// Mode Register
		BF_REG IER;				// Interrupt Enable Register
		BF_REG IDR;				// Interrupt Disable Register
		BF_REG IMR;				// Interrupt Mask Register
		BF_REG CSR;				// Channel Status Register
		BF_REG RHR;				// Receiver Holding Register
		BF_REG THR;				// Transmitter Holding Register	
		BF_REG BRGR;				// Baud Rate Generator Register
		BF_REG RTOR;				// Receiver Time-out Register
		BF_REG TTGR;				// Transmitter Timeguard Register
		BF_REG z__reserved1[5];	
		BF_REG FIDI;				// FI DI Ratio Register
		BF_REG NER;				// Number of Errors Register
		BF_REG z__reserved2;	
		BF_REG IF;				// IrDA Filter Register
		BF_REG MAN;				// Manchester Encode Decode Register
		BF_REG z__reserved3[43];	

		S_PDC PDC;
	};

	typedef S_USART S_USART0, S_USART1, S_USART2, S_USART3;


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//namespace UART
	//{
	//	BF_R16& THR 	= *	pUART0_THR;
	//	BF_R16& RBR 	= * pUART0_RBR; 
	//	BF_R16& DLL 	= * pUART0_DLL; 
	//	BF_R16& IER 	= * pUART0_IER; 
	//	BF_R16& DLH 	= * pUART0_DLH; 
	//	BF_R16& IIR 	= * pUART0_IIR; 
	//	BF_R16& LCR 	= * pUART0_LCR; 
	//	BF_R16& MCR 	= * pUART0_MCR; 
	//	BF_R16& LSR 	= * pUART0_LSR; 
	//	BF_R16& SCR 	= * pUART0_SCR; 
	//	BF_R16& GCTL	= *	pUART0_GCTL;















		//BF_REG CR;	pUART0_RBR
		//BF_REG MR;	
		//BF_REG IER;	
		//BF_REG IDR;	
		//BF_REG IMR;	
		//BF_REG SR;	
		//BF_REG RHR;	
		//BF_REG THR;	
		//BF_REG BRGR;	
		//BF_REG z__reserved[55];	

		//S_PDC PDC;
	//};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_PIT
	{
		BF_REG		MR;
		BF_REG		SR;
		BF_REG		PIVR;
		BF_REG		PIIR;
	};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_RTT
	{
		BF_REG		MR;
		BF_REG		AR;
		BF_REG		VR;
		BF_REG		SR;
	};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_SPI
	{
		BF_REG		CR;
		BF_REG		MR;
		BF_REG		RDR;
		BF_REG		TDR;
		BF_REG		SR;
		BF_REG		IER;
		BF_REG		IDR;
		BF_REG		IMR;
		BF_REG		z__rsrvd1[4];
		BF_REG		CSR[4];
		BF_REG		z__rsrvd2[38];
		BF_REG		WPMR;
		BF_REG		WPSR;
	};
	 
	typedef S_SPI S_SPI0, S_SPI1;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_PWM
	{
		BF_REG		CLK;
		BF_REG		ENA;
		BF_REG		DIS;
		BF_REG		SR;
		BF_REG		IER1;
		BF_REG		IDR1;
		BF_REG		IMR1;
		BF_REG		ISR1;
		BF_REG		SCM;

		BF_REG		zrsrv1;

		BF_REG		SCUC;
		BF_REG		SCUP;
		BF_REG		SCUPUPD;
		BF_REG		IER2;
		BF_REG		IDR2;
		BF_REG		IMR2;
		BF_REG		ISR2;
		BF_REG		OOV;
		BF_REG		OS;
		BF_REG		OSS;
		BF_REG		OSC;
		BF_REG		OSSUPD;
		BF_REG		OSCUPD;
		BF_REG		FMR;
		BF_REG		FSR;
		BF_REG		FCR;
		BF_REG		FPV;
		BF_REG		FPE1;
		BF_REG		FPE2;

		BF_REG		zrsrv2[2];

		BF_REG		ELMR0;
		BF_REG		ELMR1;

		BF_REG		zrsrv3[11];

		BF_REG		SMMR;

		BF_REG		zrsrv4[12];

		BF_REG		WPCR;
		BF_REG		WPSR;

		BF_REG		zrsrv5[17];

		struct { BF_REG VR, VUR, MR, MUR; } CMP[8];

		BF_REG		zrsrv6[20];

		struct { BF_REG	MR, DTY, DTYUPD, PRD, PRDUPD, CNT, DT, DTUPD; } CH[4];
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_WDT
	{
		BF_REG		CR;
		BF_REG		MR;
		BF_REG		SR;
	};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_RSTC
	{
		BF_REG		CR;
		BF_REG		SR;
		BF_REG		MR;
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_ADC
	{
		BF_REG		CR;
		BF_REG		MR;

		BF_REG		SEQR1;
		BF_REG		SEQR2;

		BF_REG		CHER;
		BF_REG		CHDR;
		BF_REG		CHSR;

		BF_REG		zrsrv;

		BF_REG		LCDR;

		BF_REG		IER;
		BF_REG		IDR;
		BF_REG		IMR;
		BF_REG		ISR;

		BF_REG		zrsrv2[2];

		BF_REG		OVER;
		BF_REG		EMR;
		BF_REG		CWR;
		BF_REG		CGR;
		BF_REG		COR;
		BF_REG		CDR[16];

		BF_REG		zrsrv3;

		BF_REG		ACR;

		BF_REG		zrsrv4[19];

		BF_REG		WPMR;
		BF_REG		WPSR;

		BF_REG		zrsrv5[5];

		S_PDC			PDC;
	};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_RTC
	{
		BF_REG		CR;
		BF_REG		MR;
		BF_REG		TIMR;
		BF_REG		CALR;
		BF_REG		TIMALR;
		BF_REG		CALALR;
		BF_REG		SR;
		BF_REG		SCCR;
		BF_REG		IER;
		BF_REG		IDR;
		BF_REG		IMR;
		BF_REG		VER;
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_MATRIX
	{
		BF_REG		MCFG[16];
		BF_REG		SCFG[16];
		struct { BF_REG A; BF_REG B; } PRS[16];
		BF_REG		MRCR;
		BF_REG		zreserve[4];
		BF_REG		SYSIO;
		BF_REG		zreserve1[52];
		BF_REG		WPMR;
		BF_REG		WPSR;
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_EFC
	{
		BF_REG		FMR;
		BF_REG		FCR;
		BF_REG		FSR;
		BF_REG		FRR;
	};

	typedef S_EFC S_EFC0, S_EFC1;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_SUPC
	{
		BF_REG		CR;
		BF_REG		SMMR;
		BF_REG		MR;
		BF_REG		WUMR;
		BF_REG		WUIR;
		BF_REG		SR;
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_DMAC
	{
		BF_REG		GCFG;
		BF_REG		EN;
		BF_REG		SREQ;
		BF_REG		CREQ;
		BF_REG		LAST;
		BF_REG		z__rsrvd1;
		BF_REG		EBCIER;
		BF_REG		EBCIDR;
		BF_REG		EBCIMR;
		BF_REG		EBCISR;
		BF_REG		CHER;
		BF_REG		CHDR;
		BF_REG		CHSR;
		BF_REG		z__rsrvd2[2];

		struct { BF_PTR SADDR, DADDR, DSCR; BF_REG CTRLA, CTRLB, CFG, z__rsrvd[4]; } CH[6];

		BF_REG		z__rsrvd3[46];

		BF_REG		WPMR;
		BF_REG		WPSR;
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};


namespace HW
{
	//namespace PID
	//{
	//	enum {	MKPID(SUPC, 0),		MKPID(RSTC, 1),		MKPID(RTC, 2),		MKPID(RTT, 3),		MKPID(WDT, 4),		MKPID(PMC, 5),		MKPID(EEFC0, 6),	MKPID(EEFC1, 7),
	//			MKPID(UART, 8),		MKPID(SMC, 9),		MKPID(SDRAM, 10),	MKPID(PIOA, 11),	MKPID(PIOB, 12),	MKPID(PIOC, 13),	MKPID(PIOD, 14),	MKPID(PIOE, 15),
	//			MKPID(PIOF, 16),	MKPID(USART0, 17),	MKPID(USART1, 18),	MKPID(USART2, 19),	MKPID(USART3, 20),	MKPID(HSMCI, 21),	MKPID(TWI0, 22),	MKPID(TWI1, 23), 
	//			MKPID(SPI0, 24),	MKPID(SPI1, 25), 	MKPID(SSC, 26),		MKPID(TC0, 27),		MKPID(TC1, 28),		MKPID(TC2, 29),		MKPID(TC3, 30),		MKPID(TC4, 31), 
	//			MKPID(TC5, 32),		MKPID(TC6, 33),		MKPID(TC7, 34),		MKPID(TC8, 35), 	MKPID(PWM, 36),		MKPID(ADC, 37),		MKPID(DACC, 38),	MKPID(DMAC, 39), 
	//			MKPID(UOTGHS, 40),	MKPID(TRNG, 41),	MKPID(EMAC, 42),	MKPID(CAN0, 43),	MKPID(CAN1, 44) };
	//};

	MK_PTR(SMC, 	0x400E0000);

	MK_PTR(MATRIX,	0x400E0400);
	MK_PTR(PMC,		0x400E0600);
//	MK_PTR(UART,	0x400E0800);

	MK_PTR(EFC0,	0x400E0A00);
	MK_PTR(EFC1,	0x400E0C00);
	MK_PTR(PIOA,	0x400E0E00);
	MK_PTR(PIOB,	0x400E1000);
	MK_PTR(PIOC,	0x400E1200);
	MK_PTR(PIOD,	0x400E1400);

	MK_PTR(RSTC,	0X400E1A00);
	MK_PTR(SUPC,	0X400E1A10);
	MK_PTR(RTT,		0X400E1A30);
	MK_PTR(WDT,		0X400E1A50);
	MK_PTR(RTC,		0X400E1A60);

	MK_PTR(SPI,		0X40008000);
	MK_PTR(SPI0,	0X40008000);
	MK_PTR(SPI1,	0X4000C000);

	MK_PTR(TC0,		0X40080000);
	MK_PTR(TC1,		0X40084000);
	MK_PTR(TC2,		0X40088000);

	MK_PTR(PWM,		0X40094000);
	MK_PTR(USART0,	0x40098000);
	MK_PTR(USART1,	0x4009C000);
	MK_PTR(USART2,	0x400A0000);
	MK_PTR(USART3,	0x400A4000);

	MK_PTR(ADC,		0X400C0000);

	MK_PTR(DMAC,	0X400C4000);


//	MK_PTR(UDP,		0x400A4000);



	inline void ResetWDT() { WDT->CR = 0xA5000001; }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


}; // namespace HW

//extern T_HW::BF_IHP VectorTableInt[16];
//extern T_HW::BF_IHP VectorTableExt[45];

#define PID_Watchdog_Timer				(31)
#define PID_DMA_14_15_Mem_DMA_Stream_1	(30)
#define PID_DMA_12_13_Mem_DMA_Stream_0	(29)
#define PID_TWI							(24)
#define PID_Port_G_Interrupt_B			(23)
#define PID_Port_G_Interrupt_A			(22)
#define PID_GP_Timer_2					(21)
#define PID_GP_Timer_1					(20)
#define PID_GP_Timer_0					(19)
#define PID_Port_F_Interrupt_B			(18)
#define PID_Port_F_Interrupt_A			(17)
#define PID_DMA8_UART0_TX				(16)
#define PID_DMA7_UART0_RX				(15)
#define PID_DMA6_SPI1_RX_TX				(14)
#define PID_DMA5_SPI0_RX_TX				(13)
#define PID_DMA4_SPORT1_TX				(12)
#define PID_DMA3_SPORT1_RX				(11)
#define PID_DMA2_SPORT0_TX				(10)
#define PID_DMA1_SPORT0_RX				(9)
#define PID_DMA0_PPI					(8)
#define PID_UART0_Status				(7)
#define PID_SPI1_Status					(6) 
#define PID_SPI0_Status					(5) 
#define PID_SPORT1_Status				(4) 
#define PID_SPORT0_Status				(3) 
#define PID_PPI_Status					(2) 
#define PID_DMA_Error_generic			(1) 
#define PID_PLL_Wakeup					(0) 


#undef MK_PTR
#undef MKPID

#endif // BF592_H__03_03_2014__11_41
