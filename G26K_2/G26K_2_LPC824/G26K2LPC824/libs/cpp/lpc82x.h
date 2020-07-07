#ifndef LPC82X_H__21_01_2015__11_56
#define LPC82X_H__21_01_2015__11_56

#ifndef CORETYPE_LPC82X
#error  Must #include "core.h"
#endif 

#include "types.h"
#include "cm0.h"

#define MCK 25000000

#pragma anon_unions

#ifndef WIN32
#define MK_PTR(n,p)  T_HW::S_##n * const n = ((T_HW::S_##n*)(p))
#else
extern byte core_sys_array[0x100000]; 
#define MK_PTR(n,p)  T_HW::S_##n * const n = ((T_HW::S_##n*)(core_sys_array-0x40000000+p))
#endif

#define RA32(c,s,e) z__reserved##c[(e-s)/4+1]


#define MKPID(n,i) n##_M=(1UL<<(i&31)), n##_I=i


#define	SPI0_IRQ                         0               //   0  SPI0                                                             
#define	SPI1_IRQ                         1               //   1  SPI1                                                             
#define	UART0_IRQ                        3               //   3  UART0                                                            
#define	UART1_IRQ                        4               //   4  UART1                                                            
#define	UART2_IRQ                        5               //   5  UART2                                                            
#define	I2C1_IRQ                         7               //   7  I2C1                                                             
#define	I2C0_IRQ                         8               //   8  I2C0                                                             
#define	SCT_IRQ                          9               //   9  SCT                                                              
#define	MRT_IRQ                         10               //  10  MRT                                                              
#define	CMP_IRQ                         11               //  11  CMP                                                              
#define	WDT_IRQ                         12               //  12  WDT                                                              
#define	BOD_IRQ                         13               //  13  BOD                                                              
#define	FLASH_IRQ                       14               //  14  FLASH                                                            
#define	WKT_IRQ                         15               //  15  WKT                                                              
#define	ADC_SEQA_IRQ                    16               //  16  ADC_SEQA                                                         
#define	ADC_SEQB_IRQ                    17               //  17  ADC_SEQB                                                         
#define	ADC_THCMP_IRQ                   18               //  18  ADC_THCMP                                                        
#define	ADC_OVR_IRQ                     19               //  19  ADC_OVR                                                          
#define	DMA_IRQ                         20               //  20  DMA                                                              
#define	I2C2_IRQ                        21               //  21  I2C2                                                             
#define	I2C3_IRQ                        22               //  22  I2C3                                                             
#define	PIN_INT0_IRQ                    24               //  24  PIN_INT0                                                         
#define	PIN_INT1_IRQ                    25               //  25  PIN_INT1                                                         
#define	PIN_INT2_IRQ                    26               //  26  PIN_INT2                                                         
#define	PIN_INT3_IRQ                    27               //  27  PIN_INT3                                                         
#define	PIN_INT4_IRQ                    28               //  28  PIN_INT4                                                         
#define	PIN_INT5_IRQ                    29               //  29  PIN_INT5                                                         
#define	PIN_INT6_IRQ                    30               //  30  PIN_INT6                                                         
#define	PIN_INT7_IRQ                    31               //  31  PIN_INT7

// DMA_CFG

#define PERIPHREQEN     (1 << 0)	/*!< Enables Peripheral DMA requests */
#define HWTRIGEN        (1 << 1)	/*!< Use hardware triggering via imput mux */
#define TRIGPOL_LOW     (0 << 4)	/*!< Hardware trigger is active low or falling edge */
#define TRIGPOL_HIGH    (1 << 4)	/*!< Hardware trigger is active high or rising edge */
#define TRIGTYPE_EDGE   (0 << 5)	/*!< Hardware trigger is edge triggered */
#define TRIGTYPE_LEVEL  (1 << 5)	/*!< Hardware trigger is level triggered */
#define TRIGBURST_SNGL  (0 << 6)	/*!< Single transfer. Hardware trigger causes a single transfer */
#define TRIGBURST_BURST (1 << 6)	/*!< Burst transfer (see UM) */
#define BURSTPOWER_1    (0 << 8)	/*!< Set DMA burst size to 1 transfer */
#define BURSTPOWER_2    (1 << 8)	/*!< Set DMA burst size to 2 transfers */
#define BURSTPOWER_4    (2 << 8)	/*!< Set DMA burst size to 4 transfers */
#define BURSTPOWER_8    (3 << 8)	/*!< Set DMA burst size to 8 transfers */
#define BURSTPOWER_16   (4 << 8)	/*!< Set DMA burst size to 16 transfers */
#define BURSTPOWER_32   (5 << 8)	/*!< Set DMA burst size to 32 transfers */
#define BURSTPOWER_64   (6 << 8)	/*!< Set DMA burst size to 64 transfers */
#define BURSTPOWER_128  (7 << 8)	/*!< Set DMA burst size to 128 transfers */
#define BURSTPOWER_256  (8 << 8)	/*!< Set DMA burst size to 256 transfers */
#define BURSTPOWER_512  (9 << 8)	/*!< Set DMA burst size to 512 transfers */
#define BURSTPOWER_1024 (10 << 8)	/*!< Set DMA burst size to 1024 transfers */
#define BURSTPOWER(n)   ((n) << 8)	/*!< Set DMA burst size to 2^n transfers, max n=10 */
#define SRCBURSTWRAP    (1 << 14)	/*!< Source burst wrapping is enabled for this DMA channel */
#define DSTBURSTWRAP    (1 << 15)	/*!< Destination burst wrapping is enabled for this DMA channel */
#define CHPRIORITY(p)   ((p) << 16)	/*!< Sets DMA channel priority, min 0 (highest), max 3 (lowest) */


// DMA_XFERCFG

#define CFGVALID        (1 << 0)	/*!< Configuration Valid flag */
#define RELOAD          (1 << 1)	/*!< Indicates whether the channels control structure will be reloaded when the current descriptor is exhausted */
#define SWTRIG          (1 << 2)	/*!< Software Trigger */
#define CLRTRIG         (1 << 3)	/*!< Clear Trigger */
#define SETINTA         (1 << 4)	/*!< Set Interrupt flag A for this channel to fire when descriptor is complete */
#define SETINTB         (1 << 5)	/*!< Set Interrupt flag B for this channel to fire when descriptor is complete */
#define WIDTH_8         (0 << 8)	/*!< 8-bit transfers are performed */
#define WIDTH_16        (1 << 8)	/*!< 16-bit transfers are performed */
#define WIDTH_32        (2 << 8)	/*!< 32-bit transfers are performed */
#define SRCINC_0        (0 << 12)	/*!< DMA source address is not incremented after a transfer */
#define SRCINC_1        (1 << 12)	/*!< DMA source address is incremented by 1 (width) after a transfer */
#define SRCINC_2        (2 << 12)	/*!< DMA source address is incremented by 2 (width) after a transfer */
#define SRCINC_4        (3 << 12)	/*!< DMA source address is incremented by 4 (width) after a transfer */
#define DSTINC_0        (0 << 14)	/*!< DMA destination address is not incremented after a transfer */
#define DSTINC_1        (1 << 14)	/*!< DMA destination address is incremented by 1 (width) after a transfer */
#define DSTINC_2        (2 << 14)	/*!< DMA destination address is incremented by 2 (width) after a transfer */
#define DSTINC_4        (3 << 14)	/*!< DMA destination address is incremented by 4 (width) after a transfer */
#define XFERCOUNT(n)    ((n - 1) << 16)	/*!< DMA transfer count in 'transfers', between (0)1 and (1023)1024 */


namespace T_HW
{
	typedef volatile u32	LPC_REG;// Hardware register definition
	typedef volatile u8		LPC_R8;// Hardware register definition
	typedef volatile u16	LPC_R16;// Hardware register definition
	typedef volatile void * LPC_PTR;// Hardware register definition

	typedef void(*LPC_IHP)() __irq;	// Interrupt handler pointer

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_SYSCON
	{
		LPC_REG SYSMEMREMAP;            /*!< Offset: 0x000 System memory remap (R/W) */
		LPC_REG PRESETCTRL;             /*!< Offset: 0x004 Peripheral reset control (R/W) */
		LPC_REG SYSPLLCTRL;             /*!< Offset: 0x008 System PLL control (R/W) */
		LPC_REG SYSPLLSTAT;             /*!< Offset: 0x00C System PLL status (R/W ) */
		LPC_REG z_RESERVED0[4];

		LPC_REG SYSOSCCTRL;             /*!< Offset: 0x020 System oscillator control (R/W) */
		LPC_REG WDTOSCCTRL;             /*!< Offset: 0x024 Watchdog oscillator control (R/W) */
		LPC_REG IRCCTRL;
		LPC_REG z_RESERVED1;
		LPC_REG SYSRSTSTAT;             /*!< Offset: 0x030 System reset status Register (R/W ) */
		LPC_REG z_RESERVED2[3];
		LPC_REG SYSPLLCLKSEL;           /*!< Offset: 0x040 System PLL clock source select (R/W) */
		LPC_REG SYSPLLCLKUEN;           /*!< Offset: 0x044 System PLL clock source update enable (R/W) */
		LPC_REG z_RESERVED3[10];

		LPC_REG MAINCLKSEL;             /*!< Offset: 0x070 Main clock source select (R/W) */
		LPC_REG MAINCLKUEN;             /*!< Offset: 0x074 Main clock source update enable (R/W) */
		LPC_REG SYSAHBCLKDIV;           /*!< Offset: 0x078 System AHB clock divider (R/W) */
		LPC_REG z_RESERVED4[1];

		LPC_REG SYSAHBCLKCTRL;          /*!< Offset: 0x080 System AHB clock control (R/W) */
		LPC_REG z_RESERVED5[4];
		LPC_REG UARTCLKDIV;             /*!< Offset: 0x094 UART clock divider (R/W) */
		LPC_REG z_RESERVED6[18];

		LPC_REG CLKOUTSEL;              /*!< Offset: 0x0E0 CLKOUT clock source select (R/W) */
		LPC_REG CLKOUTUEN;              /*!< Offset: 0x0E4 CLKOUT clock source update enable (R/W) */
		LPC_REG CLKOUTDIV;              /*!< Offset: 0x0E8 CLKOUT clock divider (R/W) */
		LPC_REG z_RESERVED7;
		LPC_REG UARTFRGDIV;             /*!< Offset: 0x0F0 UART fractional divider SUB(R/W) */
		LPC_REG UARTFRGMULT;             /*!< Offset: 0x0F4 UART fractional divider ADD(R/W) */
		LPC_REG z_RESERVED8[1];
		LPC_REG EXTTRACECMD;            /*!< (@ 0x400480FC) External trace buffer command register  */
		LPC_REG PIOPORCAP0;             /*!< Offset: 0x100 POR captured PIO status 0 (R/ ) */
		LPC_REG z_RESERVED9[12];
		LPC_REG IOCONCLKDIV[7];			/*!< (@0x40048134-14C) Peripheral clock x to the IOCON block for programmable glitch filter */
		LPC_REG BODCTRL;                /*!< Offset: 0x150 BOD control (R/W) */
		LPC_REG SYSTCKCAL;              /*!< Offset: 0x154 System tick counter calibration (R/W) */
		LPC_REG z_RESERVED10[6];
		LPC_REG IRQLATENCY;             /*!< (@ 0x40048170) IRQ delay */
		LPC_REG NMISRC;                 /*!< (@ 0x40048174) NMI Source Control     */
		LPC_REG PINTSEL[8];             /*!< (@ 0x40048178) GPIO Pin Interrupt Select register 0 */
		LPC_REG z_RESERVED11[27];
		LPC_REG STARTERP0;              /*!< Offset: 0x204 Start logic signal enable Register 0 (R/W) */
		LPC_REG z_RESERVED12[3];
		LPC_REG STARTERP1;              /*!< Offset: 0x214 Start logic signal enable Register 0 (R/W) */
		LPC_REG z_RESERVED13[6];
		LPC_REG PDSLEEPCFG;             /*!< Offset: 0x230 Power-down states in Deep-sleep mode (R/W) */
		LPC_REG PDAWAKECFG;             /*!< Offset: 0x234 Power-down states after wake-up (R/W) */
		LPC_REG PDRUNCFG;               /*!< Offset: 0x238 Power-down configuration Register (R/W) */
		LPC_REG z_RESERVED14[110];
		LPC_REG DEVICE_ID;              /*!< Offset: 0x3F4 Device ID (R/ ) */

	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_IOCON							/*!< (@ 0x40044000) IOCONFIG Structure     */
	{					
		union S_PIO
		{
			LPC_REG		D;

			struct
			{
				LPC_REG		res02 : 3;
				LPC_REG		MODE : 2;
				LPC_REG		HYS : 1;
				LPC_REG		INV : 1;
				LPC_REG		res7 : 1;
				LPC_REG		I2CMODE : 2;
				LPC_REG		OD : 1;	
				LPC_REG		S_MODE : 2;
				LPC_REG		CLK_DIV : 3;
			} B;
		};

		S_PIO 	PIO0_17;                    /*!< (@ 0x40044000) I/O configuration for pin PIO0_17 */
		S_PIO 	PIO0_13;                    /*!< (@ 0x40044004) I/O configuration for pin PIO0_13 */
		S_PIO 	PIO0_12;                    /*!< (@ 0x40044008) I/O configuration for pin PIO0_12 */
		S_PIO 	PIO0_5;                     /*!< (@ 0x4004400C) I/O configuration for pin PIO0_5 */
		S_PIO 	PIO0_4;                     /*!< (@ 0x40044010) I/O configuration for pin PIO0_4 */
		S_PIO 	PIO0_3;                     /*!< (@ 0x40044014) I/O configuration for pin PIO0_3 */
		S_PIO 	PIO0_2;                     /*!< (@ 0x40044018) I/O configuration for pin PIO0_2 */
		S_PIO 	PIO0_11;                    /*!< (@ 0x4004401C) I/O configuration for pin PIO0_11 */
		S_PIO 	PIO0_10;                    /*!< (@ 0x40044020) I/O configuration for pin PIO0_10 */
		S_PIO 	PIO0_16;                    /*!< (@ 0x40044024) I/O configuration for pin PIO0_16 */
		S_PIO 	PIO0_15;                    /*!< (@ 0x40044028) I/O configuration for pin PIO0_15 */
		S_PIO 	PIO0_1;                     /*!< (@ 0x4004402C) I/O configuration for pin PIO0_1 */
		LPC_REG z_Reserved;                 /*!< (@ 0x40044030) I/O configuration for pin (Reserved) */
		S_PIO 	PIO0_9;                     /*!< (@ 0x40044034) I/O configuration for pin PIO0_9 */
		S_PIO 	PIO0_8;                     /*!< (@ 0x40044038) I/O configuration for pin PIO0_8 */
		S_PIO 	PIO0_7;                     /*!< (@ 0x4004403C) I/O configuration for pin PIO0_7 */
		S_PIO 	PIO0_6;                     /*!< (@ 0x40044040) I/O configuration for pin PIO0_6 */
		S_PIO 	PIO0_0;                     /*!< (@ 0x40044044) I/O configuration for pin PIO0_0 */
		S_PIO 	PIO0_14;                    /*!< (@ 0x40044048) I/O configuration for pin PIO0_14 */
		LPC_REG z_Reserved1;
		S_PIO 	PIO0_28;                   /*!< (@ 0x40044050) I/O configuration for pin PIO0_28                      */
		S_PIO 	PIO0_27;                   /*!< (@ 0x40044054) I/O configuration for pin PIO0_27                      */
		S_PIO 	PIO0_26;                   /*!< (@ 0x40044058) I/O configuration for pin PIO0_26                      */
		S_PIO 	PIO0_25;                   /*!< (@ 0x4004405C) I/O configuration for pin PIO0_25                      */
		S_PIO 	PIO0_24;                   /*!< (@ 0x40044060) I/O configuration for pin PIO0_24                      */
		S_PIO 	PIO0_23;                   /*!< (@ 0x40044064) I/O configuration for pin PIO0_23/ADC_3                */
		S_PIO	PIO0_22;                   /*!< (@ 0x40044068) I/O configuration for pin PIO0_22/ADC_4                */
		S_PIO	PIO0_21;                   /*!< (@ 0x4004406C) I/O configuration for pin PIO0_21/ACMP_I4/ADC_5        */
		S_PIO	PIO0_20;                   /*!< (@ 0x40044070) I/O configuration for pin PIO0_20/ADC_6                */
		S_PIO	PIO0_19;                   /*!< (@ 0x40044074) I/O configuration for pin PIO0_19/ADC_7                */
		S_PIO	PIO0_18;                   /*!< (@ 0x40044078) I/O configuration for pin PIO0_18/ADC_8                */
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_FLASHCTRL
	{					                            /*!< (@ 0x40040000) FLASHCTRL Structure    */
		LPC_REG  z_RESERVED0[4];
		LPC_REG  FLASHCFG;                          /*!< (@ 0x40040010) Flash configuration register                           */
		LPC_REG  z_RESERVED1[3];
		LPC_REG  FMSSTART;                          /*!< (@ 0x40040020) Signature start address register                       */
		LPC_REG  FMSSTOP;                           /*!< (@ 0x40040024) Signature stop-address register                        */
		LPC_REG  z_RESERVED2;
		LPC_REG  FMSW0;
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_PMU
	{
		LPC_REG PCON;                   /*!< Offset: 0x000 Power control Register (R/W) */
		LPC_REG GPREG0;                 /*!< Offset: 0x004 General purpose Register 0 (R/W) */
		LPC_REG GPREG1;                 /*!< Offset: 0x008 General purpose Register 1 (R/W) */
		LPC_REG GPREG2;                 /*!< Offset: 0x00C General purpose Register 2 (R/W) */
		LPC_REG GPREG3;                 /*!< Offset: 0x010 General purpose Register 3 (R/W) */
		LPC_REG DPDCTRL;                /*!< Offset: 0x014 Deep power-down control register (R/W) */
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	struct S_SWM
	{
		struct
		{
			LPC_REG		U0_TXD : 8;
			LPC_REG		U0_RXD : 8; 
			LPC_REG		U0_RTS : 8; 
			LPC_REG		U0_CTS : 8;
		};

		struct
		{
			LPC_REG		U0_SCLK : 8;
			LPC_REG		U1_TXD : 8;
			LPC_REG		U1_RXD : 8;
			LPC_REG		U1_RTS : 8;
		};

		struct
		{
			LPC_REG		U1_CTS : 8;
			LPC_REG		U1_SCLK : 8;
			LPC_REG		U2_TXD : 8;
			LPC_REG		U2_RXD : 8;
		};

		struct
		{
			LPC_REG		U2_RTS : 8;
			LPC_REG		U2_CTS : 8;
			LPC_REG		U2_SCLK : 8;
			LPC_REG		SPI0_SCK : 8;
		};

		struct
		{
			LPC_REG		SPI0_MOSI : 8; 
			LPC_REG		SPI0_MISO : 8;
			LPC_REG		SPI0_SSEL0 : 8;
			LPC_REG		SPI0_SSEL1 : 8;
		};

		struct
		{
			LPC_REG		SPI0_SSEL2 : 8;
			LPC_REG		SPI0_SSEL3 : 8;
			LPC_REG		SPI1_SCK : 8;
			LPC_REG		SPI1_MOSI : 8;
		};

		struct
		{
			LPC_REG		SPI1_MISO : 8;
			LPC_REG		SPI1_SSEL0 : 8;
			LPC_REG		SPI1_SSEL1 : 8;
			LPC_REG		CTIN_0 : 8;
		};

		struct
		{
			LPC_REG		CTIN_1 : 8;
			LPC_REG		CTIN_2 : 8;
			LPC_REG		CTIN_3 : 8;
			LPC_REG		CTOUT_0 : 8;
		};

		struct
		{
			LPC_REG		CTOUT_1 : 8;
			LPC_REG		CTOUT_2 : 8;
			LPC_REG		CTOUT_3 : 8;
			LPC_REG		CTOUT_4 : 8;
		};

		struct
		{
			LPC_REG		CTOUT_5 : 8;
			LPC_REG		I2C1_SDA : 8;
			LPC_REG		I2C1_SCL : 8;
			LPC_REG		I2C2_SDA : 8;
		};

		struct
		{
			LPC_REG		I2C2_SCL : 8;
			LPC_REG		I2C3_SDA : 8;
			LPC_REG		I2C3_SCL : 8;
			LPC_REG		ADC_PINTRIG0 : 8;
		};

		struct
		{
			LPC_REG		ADC_PINTRIG1 : 8;
			LPC_REG		ACMP_O : 8;
			LPC_REG		CLKOUT : 8;
			LPC_REG		GPIO_INT_BMAT : 8;
		};

		LPC_REG		z_RESERVED0[100];
		
		union
		{
			LPC_REG	D;

			struct
			{
				LPC_REG		ACMP_I1 : 1;
				LPC_REG		ACMP_I2 : 1;
				LPC_REG		ACMP_I3 : 1;
				LPC_REG		ACMP_I4 : 1;
				LPC_REG		SWCLK : 1;
				LPC_REG		SWDIO : 1;
				LPC_REG		XTALIN : 1;
				LPC_REG		XTALOUT : 1;
				LPC_REG		RESETN : 1;
				LPC_REG		CLKIN : 1;
				LPC_REG		VDDCMP : 1;
				LPC_REG		I2C0_SDA : 1;
				LPC_REG		I2C0_SCL : 1;
				LPC_REG		ADC_0 : 1;
				LPC_REG		ADC_1 : 1;
				LPC_REG		ADC_2 : 1;
				LPC_REG		ADC_3 : 1;
				LPC_REG		ADC_4 : 1;
				LPC_REG		ADC_5 : 1;
				LPC_REG		ADC_6 : 1;
				LPC_REG		ADC_7 : 1;
				LPC_REG		ADC_8 : 1;
				LPC_REG		ADC_9 : 1;
				LPC_REG		ADC_10 : 1;
				LPC_REG		ADC_11 : 1;
			} B;
		} PINENABLE0;
	};
	
	
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_ADC
	{                                    /*!< (@ 0x4001C000) ADC Structure                                          */
		LPC_REG  CTRL;                              /*!< (@ 0x4001C000) A/D Control Register. Contains the clock divide value, enable bits for each sequence and the A/D power-down bit.                                                                 */
		LPC_REG  RESERVED0;
		LPC_REG  SEQA_CTRL;                         /*!< (@ 0x4001C008) A/D Conversion Sequence-A control Register: Controls triggering and channel selection for conversion sequence-A. Also specifies interrupt mode for sequence-A.                        */
		LPC_REG  SEQB_CTRL;                         /*!< (@ 0x4001C00C) A/D Conversion Sequence-B Control Register: Controls triggering and channel selection for conversion sequence-B. Also specifies interrupt mode for sequence-B.                        */
		LPC_REG  SEQA_GDAT;                         /*!< (@ 0x4001C010) A/D Sequence-A Global Data Register. This register contains the result of the most recent A/D conversion performed under sequence-A                                                     */
		LPC_REG  SEQB_GDAT;                         /*!< (@ 0x4001C014) A/D Sequence-B Global Data Register. This register contains the result of the most recent A/D conversion performed under sequence-B                                                     */
		LPC_REG  RESERVED1[2];
		LPC_REG  DAT0;                              /*!< (@ 0x4001C020) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT1;                              /*!< (@ 0x4001C024) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT2;                              /*!< (@ 0x4001C028) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT3;                              /*!< (@ 0x4001C02C) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT4;                              /*!< (@ 0x4001C030) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT5;                              /*!< (@ 0x4001C034) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT6;                              /*!< (@ 0x4001C038) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT7;                              /*!< (@ 0x4001C03C) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT8;                              /*!< (@ 0x4001C040) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT9;                              /*!< (@ 0x4001C044) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT10;                             /*!< (@ 0x4001C048) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  DAT11;                             /*!< (@ 0x4001C04C) A/D Channel 0 Data Register. This register contains the result of the most recent conversion completed on channel 0.                                                                   */
		LPC_REG  THR0_LOW;                          /*!< (@ 0x4001C050) A/D Low Compare Threshold Register 0 : Contains the lower threshold level for automatic threshold comparison for any channels linked to threshold pair 0.                         */
		LPC_REG  THR1_LOW;                          /*!< (@ 0x4001C054) A/D Low Compare Threshold Register 1: Contains the lower threshold level for automatic threshold comparison for any channels linked to threshold pair 1.                         */
		LPC_REG  THR0_HIGH;                         /*!< (@ 0x4001C058) A/D High Compare Threshold Register 0: Contains the upper threshold level for automatic threshold comparison for any channels linked to threshold pair 0.                         */
		LPC_REG  THR1_HIGH;                         /*!< (@ 0x4001C05C) A/D High Compare Threshold Register 1: Contains the upper threshold level for automatic threshold comparison for any channels linked to threshold pair 1.                         */
		LPC_REG  CHAN_THRSEL;                       /*!< (@ 0x4001C060) A/D Channel-Threshold Select Register. Specifies which set of threshold compare registers are to be used for each channel                                                         */
		LPC_REG  INTEN;                             /*!< (@ 0x4001C064) A/D Interrupt Enable Register. This register contains enable bits that enable the sequence-A, sequence-B, threshold compare and data overrun interrupts to be generated.       */
		LPC_REG  FLAGS;                             /*!< (@ 0x4001C068) A/D Flags Register. Contains the four interrupt request flags and the individual component overrun and threshold-compare flags. (The overrun bits replicate information stored in the result registers).                                                   */
		LPC_REG  TRM;                               /*!< (@ 0x4001C06C) ADC trim register.                                     */
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_GPIO
	{
		byte	B0[29];						/*!< (@ 0xA0000000) Byte pin registers port 0 */
		byte	z_RESERVED0[4067];
		LPC_REG W0[29];						/*!< (@ 0xA0001000) Word pin registers port 0 */
		LPC_REG z_RESERVED1[995];
		LPC_REG DIR0;						/* 0x2000 */
		LPC_REG z_RESERVED2[31];
		LPC_REG MASK0;                      /* 0x2080 */
		LPC_REG z_RESERVED3[31];
		LPC_REG PIN0;                       /* 0x2100 */
		LPC_REG z_RESERVED4[31];
		LPC_REG MPIN0;                      /* 0x2180 */
		LPC_REG z_RESERVED5[31];
		LPC_REG SET0;                       /* 0x2200 */
		LPC_REG z_RESERVED6[31];
		LPC_REG CLR0;                       /* 0x2280 */
		LPC_REG z_RESERVED7[31];
		LPC_REG NOT0;                       /* 0x2300 */
		LPC_REG z_RESERVED8[31];
		LPC_REG DIRSET0;                    /* 0x2380 */
		LPC_REG z_RESERVED9[31];
		LPC_REG DIRCLR0;                    /* 0x2400 */
		LPC_REG z_RESERVED10[31];
		LPC_REG DIRNOT0;                    /* 0x2480 */

		void SET(u32 m) { SET0 = m; }
		void CLR(u32 m) { CLR0 = m; }
		void NOT(u32 m) { NOT0 = m; }
		void WBIT(u32 m, bool c) { if (c) SET(m); else CLR(m); }
		void BSET(byte b) { SET0 = 1 << b; }
		void BCLR(byte b) { CLR0 = 1 << b; }
		void BTGL(byte b) { NOT0 = 1 << b; }
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_PIN_INT
	{                           /*!< (@ 0xA0004000) PIN_INT Structure */
		LPC_REG ISEL;           /*!< (@ 0xA0004000) Pin Interrupt Mode register */
		LPC_REG IENR;           /*!< (@ 0xA0004004) Pin Interrupt Enable (Rising) register */
		LPC_REG SIENR;          /*!< (@ 0xA0004008) Set Pin Interrupt Enable (Rising) register */
		LPC_REG CIENR;          /*!< (@ 0xA000400C) Clear Pin Interrupt Enable (Rising) register */
		LPC_REG IENF;           /*!< (@ 0xA0004010) Pin Interrupt Enable Falling Edge / Active Level register */
		LPC_REG SIENF;          /*!< (@ 0xA0004014) Set Pin Interrupt Enable Falling Edge / Active Level register */
		LPC_REG CIENF;          /*!< (@ 0xA0004018) Clear Pin Interrupt Enable Falling Edge / Active Level address */
		LPC_REG RISE;           /*!< (@ 0xA000401C) Pin Interrupt Rising Edge register */
		LPC_REG FALL;           /*!< (@ 0xA0004020) Pin Interrupt Falling Edge register */
		LPC_REG IST;            /*!< (@ 0xA0004024) Pin Interrupt Status register */
		LPC_REG PMCTRL;         /*!< (@ 0xA0004028) GPIO pattern match interrupt control register          */
		LPC_REG PMSRC;          /*!< (@ 0xA000402C) GPIO pattern match interrupt bit-slice source register */
		LPC_REG PMCFG;          /*!< (@ 0xA0004030) GPIO pattern match interrupt bit slice configuration register */
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_CRC
	{
		LPC_REG MODE;
		LPC_REG SEED;
		union
		{
			LPC_REG SUM;
			LPC_REG D;
			LPC_R16	W;
			LPC_R8	B;
		};
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_MRT
	{
		struct
		{
			LPC_REG INTVAL;
			LPC_REG TIMER;
			LPC_REG CTRL;
			LPC_REG STAT;
		} Channel[4];
	   
		LPC_REG Reserved0[45];
		LPC_REG IDLE_CH;
		LPC_REG IRQ_FLAG;
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	typedef struct S_USART
	{
		LPC_REG  CFG;						/* 0x00 */
		LPC_REG  CTRL;
		LPC_REG  STAT;
		LPC_REG  INTENSET;
		LPC_REG  INTENCLR;					/* 0x10 */
		LPC_REG  RXDATA;
		LPC_REG  RXDATA_STAT;
		LPC_REG  TXDATA;
		LPC_REG  BRG;						/* 0x20 */
		LPC_REG  INTSTAT;
		LPC_REG  OSR;
		LPC_REG  ADR;
	} S_USART0, S_USART1, S_USART2;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	typedef struct S_SPI
	{
		LPC_REG  CFG;			/* 0x00 */
		LPC_REG  DLY;
		LPC_REG  STAT;
		LPC_REG  INTENSET;
		LPC_REG  INTENCLR;		/* 0x10 */
		LPC_REG  RXDAT;
		LPC_REG  TXDATCTL;
		LPC_REG  TXDAT;
		LPC_REG  TXCTRL;		/* 0x20 */
		LPC_REG  DIV;
		LPC_REG  INTSTAT;
	} S_SPI0, S_SPI1;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_WDT
	{
		LPC_REG MOD;                    /*!< Offset: 0x000 Watchdog mode register (R/W) */
		LPC_REG TC;                     /*!< Offset: 0x004 Watchdog timer constant register (R/W) */
		LPC_REG FEED;                   /*!< Offset: 0x008 Watchdog feed sequence register (W) */
		LPC_REG TV;                     /*!< Offset: 0x00C Watchdog timer value register (R) */
		LPC_REG z_RESERVED;             /*!< Offset: 0x010 RESERVED                          */
		LPC_REG WARNINT;                /*!< Offset: 0x014 Watchdog timer warning int. register (R/W) */
		LPC_REG WINDOW;                 /*!< Offset: 0x018 Watchdog timer window value register (R/W) */
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define CONFIG_SCT_nEV   (6)             /* Number of events */
#define CONFIG_SCT_nRG   (5)             /* Number of match/compare registers */
#define CONFIG_SCT_nOU   (4)             /* Number of outputs */

	struct S_SCT
	{
		LPC_REG CONFIG;              /* 0x000 Configuration Register */
		
		union
		{
			LPC_REG CTRL_U;		           /* 0x004 Control Register */
			struct
			{
				LPC_R16 CTRL_L;       /* 0x004 low control register */
				LPC_R16 CTRL_H;       /* 0x006 high control register */
			};
		};

		LPC_R16 LIMIT_L;              /* 0x008 limit register for counter L */
		LPC_R16 LIMIT_H;              /* 0x00A limit register for counter H */
		LPC_R16 HALT_L;               /* 0x00C halt register for counter L */
		LPC_R16 HALT_H;               /* 0x00E halt register for counter H */
		LPC_R16 STOP_L;               /* 0x010 stop register for counter L */
		LPC_R16 STOP_H;               /* 0x012 stop register for counter H */
		LPC_R16 START_L;              /* 0x014 start register for counter L */
		LPC_R16 START_H;              /* 0x016 start register for counter H */
		LPC_REG RESERVED1[10];        /* 0x018-0x03C reserved */
		
		union
		{
			LPC_REG COUNT_U;          /* 0x040 counter register */
			
			struct
			{
				LPC_R16 COUNT_L;      /* 0x040 counter register for counter L */
				LPC_R16 COUNT_H;      /* 0x042 counter register for counter H */
			};
		};

		LPC_R16 STATE_L;              /* 0x044 state register for counter L */
		LPC_R16 STATE_H;              /* 0x046 state register for counter H */
		LPC_REG INPUT;                /* 0x048 input register */
		LPC_R16 REGMODE_L;            /* 0x04C match - capture registers mode register L */
		LPC_R16 REGMODE_H;            /* 0x04E match - capture registers mode register H */
		LPC_REG OUTPUT;               /* 0x050 output register */
		LPC_REG OUTPUTDIRCTRL;        /* 0x054 Output counter direction Control Register */
		LPC_REG RES;                  /* 0x058 conflict resolution register */
		LPC_REG RESERVED2[37];        /* 0x05C-0x0EC reserved */
		LPC_REG EVEN;                 /* 0x0F0 event enable register */
		LPC_REG EVFLAG;               /* 0x0F4 event flag register */
		LPC_REG CONEN;                /* 0x0F8 conflict enable register */
		LPC_REG CONFLAG;              /* 0x0FC conflict flag register */

		union
		{
			union
			{                    /* 0x100-... Match / Capture value */
				LPC_REG U;                 /*       SCTMATCH[i].U  Unified 32-bit register */
				
				struct
				{
					LPC_R16 L;             /*       SCTMATCH[i].L  Access to L value */
					LPC_R16 H;             /*       SCTMATCH[i].H  Access to H value */
				};

			} MATCH[CONFIG_SCT_nRG];

			union
			{
				LPC_REG U;                 /*       SCTCAP[i].U  Unified 32-bit register */
				
				struct
				{
					LPC_R16 L;             /*       SCTCAP[i].L  Access to H value */
					LPC_R16 H;             /*       SCTCAP[i].H  Access to H value */
				};

			} CAP[CONFIG_SCT_nRG];
		};


		LPC_REG RESERVED3[32-CONFIG_SCT_nRG];      /* ...-0x17C reserved */

		union
		{
			LPC_R16 MATCH_L[CONFIG_SCT_nRG];       /* 0x180-... Match Value L counter */
			LPC_R16 CAP_L[CONFIG_SCT_nRG];         /* 0x180-... Capture Value L counter */
		};

		LPC_R16 RESERVED4[32-CONFIG_SCT_nRG];      /* ...-0x1BE reserved */
		
		union
		{
			LPC_R16 MATCH_H[CONFIG_SCT_nRG];       /* 0x1C0-... Match Value H counter */
			LPC_R16 CAP_H[CONFIG_SCT_nRG];         /* 0x1C0-... Capture Value H counter */
		};

		LPC_R16 RESERVED5[32-CONFIG_SCT_nRG];      /* ...-0x1FE reserved */

		union
		{
			union
			{                    /* 0x200-... Match Reload / Capture Control value */
				LPC_REG U;                 /*       SCTMATCHREL[i].U  Unified 32-bit register */
				
				struct
				{
					LPC_R16 L;             /*       SCTMATCHREL[i].L  Access to L value */
					LPC_R16 H;             /*       SCTMATCHREL[i].H  Access to H value */
				};

			} MATCHREL[CONFIG_SCT_nRG];
			
			union
			{
				LPC_REG U;                 /*       SCTCAPCTRL[i].U  Unified 32-bit register */
				
				struct
				{
					LPC_R16 L;             /*       SCTCAPCTRL[i].L  Access to H value */
					LPC_R16 H;             /*       SCTCAPCTRL[i].H  Access to H value */
				};

			} CAPCTRL[CONFIG_SCT_nRG];
		};

		LPC_REG RESERVED6[32-CONFIG_SCT_nRG];      /* ...-0x27C reserved */

		union
		{
			LPC_R16 MATCHREL_L[CONFIG_SCT_nRG];    /* 0x280-... Match Reload value L counter */
			LPC_R16 CAPCTRL_L[CONFIG_SCT_nRG];     /* 0x280-... Capture Control value L counter */
		};
		
		LPC_R16 RESERVED7[32-CONFIG_SCT_nRG];      /* ...-0x2BE reserved */
		
		union
		{
			LPC_R16 MATCHREL_H[CONFIG_SCT_nRG];    /* 0x2C0-... Match Reload value H counter */
			LPC_R16 CAPCTRL_H[CONFIG_SCT_nRG];     /* 0x2C0-... Capture Control value H counter */
		};
		
		LPC_R16 RESERVED8[32-CONFIG_SCT_nRG];      /* ...-0x2FE reserved */

		struct
		{                       /* 0x300-0x3FC  SCTEVENT[i].STATE / SCTEVENT[i].CTRL*/
			LPC_REG STATE;                 /* Event State Register */
			LPC_REG CTRL;                  /* Event Control Register */
		} EVENT[CONFIG_SCT_nEV];

		LPC_REG RESERVED9[128-2*CONFIG_SCT_nEV];   /* ...-0x4FC reserved */

		struct
		{                       /* 0x500-0x57C  SCTOUT[i].SET / SCTOUT[i].CLR */
			LPC_REG SET;                   /* Output n Set Register */
			LPC_REG CLR;                   /* Output n Clear Register */
		} OUT[CONFIG_SCT_nOU];

		LPC_REG RESERVED10[191-2*CONFIG_SCT_nOU];  /* ...-0x7F8 reserved */

		LPC_REG MODULECONTENT;        /* 0x7FC Module Content */

	};
	
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	struct DMADESC
	{
		LPC_REG	CFG;
		LPC_PTR SEA;
		LPC_PTR DEA;
		LPC_PTR NEXT;
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_DMA
	{
		LPC_REG  CTRL;                              /*!< (@ 0x50008000) DMA control.                                           */
		LPC_REG  INTSTAT;                           /*!< (@ 0x50008004) Interrupt status.                                      */
		LPC_PTR  SRAMBASE;                          /*!< (@ 0x50008008) SRAM address of the channel configuration table.       */
		LPC_REG  z_RESERVED0[5];
		LPC_REG  ENABLESET0;                        /*!< (@ 0x50008020) Channel Enable read and Set for all DMA channels.      */
		LPC_REG  z_RESERVED1;
		LPC_REG  ENABLECLR0;                        /*!< (@ 0x50008028) Channel Enable Clear for all DMA channels.             */
		LPC_REG  z_RESERVED2;
		LPC_REG  ACTIVE0;                           /*!< (@ 0x50008030) Channel Active status for all DMA channels.            */
		LPC_REG  z_RESERVED3;
		LPC_REG  BUSY0;                             /*!< (@ 0x50008038) Channel Busy status for all DMA channels.              */
		LPC_REG  z_RESERVED4;
		LPC_REG  ERRINT0;                           /*!< (@ 0x50008040) Error Interrupt status for all DMA channels.           */
		LPC_REG  z_RESERVED5;
		LPC_REG  INTENSET0;                         /*!< (@ 0x50008048) Interrupt Enable read and Set for all DMA channels.    */
		LPC_REG  z_RESERVED6;
		LPC_REG  INTENCLR0;                         /*!< (@ 0x50008050) Interrupt Enable Clear for all DMA channels.           */
		LPC_REG  z_RESERVED7;
		LPC_REG  INTA0;                             /*!< (@ 0x50008058) Interrupt A status for all DMA channels.               */
		LPC_REG  z_RESERVED8;
		LPC_REG  INTB0;                             /*!< (@ 0x50008060) Interrupt B status for all DMA channels.               */
		LPC_REG  z_RESERVED9;
		LPC_REG  SETVALID0;                         /*!< (@ 0x50008068) Set ValidPending control bits for all DMA channels.    */
		LPC_REG  z_RESERVED10;
		LPC_REG  SETTRIG0;                          /*!< (@ 0x50008070) Set Trigger control bits for all DMA channels.         */
		LPC_REG  z_RESERVED11;
		LPC_REG  ABORT0;                            /*!< (@ 0x50008078) Channel Abort control for all DMA channels.            */
		LPC_REG  z_RESERVED12[225];


		struct	S_CHNL
		{
			LPC_REG  CFG;      
			LPC_REG  CTLSTAT;  
			LPC_REG  XFERCFG;  
			LPC_REG  z_res;
		};					
		
		S_CHNL	CH[18];
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_DMATRIGMUX
	{
		LPC_REG	INMUX[18];	// Input mux register for trigger inputs 0 to 23 connected to DMA channel 0. Selects from ADC, SCT, ACMP, pin interrupts, and DMA requests.
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_INPUTMUX
	{
		LPC_REG	DMA_INMUX[2];
		
		LPC_REG	_z_RESERVED0[6];
		
		LPC_REG	SCT0_INMUX[4];
	};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	typedef struct S_TWI							// (@ 0x40050000) I2C0 Structure                                        
	{
		LPC_REG  CFG;                               // (@ 0x40050000) Configuration for shared functions.               
		LPC_REG  STAT;                              // (@ 0x40050004) Status register for Master, Slave, and Monitor functions.
		LPC_REG  INTENSET;                          // (@ 0x40050008) Interrupt Enable Set and read register.                
		LPC_REG  INTENCLR;                          // (@ 0x4005000C) Interrupt Enable Clear register.                       
		LPC_REG  TIMEOUT;                           // (@ 0x40050010) Time-out value register.                           
		LPC_REG  CLKDIV;                            // (@ 0x40050014) Clock pre-divider for the entire I2C block. This determines what time increments are used for the MSTTIME and SLVTIME registers.                                                
		LPC_REG  INTSTAT;                           // (@ 0x40050018) Interrupt Status register for Master, Slave, and Monitor functions.                                             
		LPC_REG  RESERVED0;
		LPC_REG  MSTCTL;                            // (@ 0x40050020) Master control register.                           
		LPC_REG  MSTTIME;                           // (@ 0x40050024) Master timing configuration.                       
		LPC_REG  MSTDAT;                            // (@ 0x40050028) Combined Master receiver and transmitter data register.                                                          
		LPC_REG  RESERVED1[5];
		LPC_REG  SLVCTL;                            // (@ 0x40050040) Slave control register.                            
		LPC_REG  SLVDAT;                            // (@ 0x40050044) Combined Slave receiver and transmitter data register.                                                          
		LPC_REG  SLVADR0;                           // (@ 0x40050048) Slave address 0.                                   
		LPC_REG  SLVADR1;                           // (@ 0x4005004C) Slave address 0.                                   
		LPC_REG  SLVADR2;                           // (@ 0x40050050) Slave address 0.                                   
		LPC_REG  SLVADR3;                           // (@ 0x40050054) Slave address 0.                                   
		LPC_REG  SLVQUAL0;                          // (@ 0x40050058) Slave Qualification for address 0.                 
		LPC_REG  RESERVED2[9];
		LPC_REG  MONRXDAT;                          // (@ 0x40050080) Monitor receiver data register.                    

	} S_I2C0, S_I2C1, S_I2C2, S_I2C3;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

namespace HW
{
	namespace CLK
	{
		enum {	MKPID(SYS, 0),		MKPID(ROM, 1),		MKPID(RAM, 2),		MKPID(FLASHREG, 3),		MKPID(FLASH, 4),	MKPID(I2C0, 5),		MKPID(GPIO, 6),		MKPID(SWM, 7),
				MKPID(SCT, 8),		MKPID(WKT, 9),		MKPID(MRT, 10),		MKPID(SPI0, 11),		MKPID(SPI1, 12),	MKPID(CRC, 13),		MKPID(UART0, 14),	MKPID(UART1, 15),
				MKPID(UART2, 16),	MKPID(WWDT, 17),	MKPID(IOCON, 18),	MKPID(ACMP, 19),		MKPID(I2C1, 21),	MKPID(I2C2, 22),	MKPID(I2C3, 23),	MKPID(ADC, 24),
				MKPID(MTB, 26),		MKPID(DMA, 29)		};
	};

	MK_PTR(WDT,				0x40000000);
	MK_PTR(MRT, 			0x40004000);
	MK_PTR(SWM,				0x4000C000);
	MK_PTR(ADC,				0x4001C000);
	MK_PTR(PMU,				0x40020000);
	MK_PTR(DMATRIGMUX,		0x40028000);
	MK_PTR(INPUTMUX,		0x4002C000);
	MK_PTR(FLASHCTRL,		0x40040000);
	MK_PTR(IOCON,			0x40044000);
	MK_PTR(SYSCON,			0x40048000);
	MK_PTR(SPI0,			0x40058000);
	MK_PTR(SPI1,			0x4005C000);
	MK_PTR(USART0,			0x40064000);
	MK_PTR(USART1,			0x40068000);
	MK_PTR(USART2,			0x4006C000);

	MK_PTR(I2C0,			0x40050000);
	MK_PTR(I2C1,			0x40054000);
	MK_PTR(I2C2,			0x40070000);
	MK_PTR(I2C3,			0x40074000);

	MK_PTR(CRC,				0x50000000);
	MK_PTR(SCT,				0x50004000);
	MK_PTR(DMA,				0x50008000);
	MK_PTR(GPIO,			0xA0000000);
	MK_PTR(PIN_INT,			0xA0004000);

	inline void ResetWDT() { WDT->FEED = 0xAA; WDT->FEED = 0x55;}


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


}; // namespace HW

extern T_HW::LPC_IHP VectorTableInt[16];
extern T_HW::LPC_IHP VectorTableExt[32];

extern T_HW::DMADESC DmaTable[18];

#undef MK_PTR
#undef MKPID

#endif // LPC82X_H__21_01_2015__11_56
