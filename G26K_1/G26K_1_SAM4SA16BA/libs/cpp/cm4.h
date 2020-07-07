#ifndef CM3_H__14_11_2012__12_19
#define CM3_H__14_11_2012__12_19

#include "types.h"

namespace CM4_TYPES
{
	typedef volatile const u32	I32;	/*!< Defines 'read only' permissions                 */
	typedef volatile const u8	I8;		/*!< Defines 'read only' permissions                 */
    typedef volatile u32		O32;	/*!< Defines 'write only' permissions                */
    typedef volatile u32		IO32;	/*!< Defines 'read / write' permissions              */
    typedef volatile u8			IO8;	/*!< Defines 'read / write' permissions              */

//brief  Union type to access the Application Program Status Register (APSR).
	typedef union
	{
		struct
		{
			u32 _reserved0:16;      /*!< bit:	0..15	Reserved                           */
			u32 GE:4;               /*!< bit:	16..19  Greater than or Equal flags        */
			u32 _reserved1:7;       /*!< bit:	20..26  Reserved                           */
			u32 Q:1;                /*!< bit: 	27  	Saturation condition flag          */
			u32 V:1;                /*!< bit: 	28  	Overflow condition code flag       */
			u32 C:1;                /*!< bit: 	29  	Carry condition code flag          */
			u32 Z:1;                /*!< bit: 	30  	Zero condition code flag           */
			u32 N:1;                /*!< bit: 	31  	Negative condition code flag       */
		} b;                        /*!< Structure used for bit  access                  */
		u32 w;						/*!< Type      used for word access                  */
	} APSR_T;


//brief  Union type to access the Interrupt Program Status Register (IPSR).

	typedef union
	{
		struct
		{
			u32 ISR:9;                      /*!< bit:  0.. 8  Exception number                   */
			u32 _reserved0:23;              /*!< bit:  9..31  Reserved                           */
		} b;                                   /*!< Structure used for bit  access                  */
		u32 w;                            /*!< Type      used for word access                  */
	} IPSR_T;


//brief  Union type to access the Special-Purpose Program Status Registers (xPSR).

	typedef union
	{
		struct
		{
			u32 ISR:9;                      /*!< bit:  0.. 8  Exception number                   */
			u32 _reserved0:7;               /*!< bit:  9..15  Reserved                           */
			u32 GE:4;                       /*!< bit: 16..19  Greater than or Equal flags        */
			u32 _reserved1:4;               /*!< bit: 20..23  Reserved                           */
			u32 T:1;                        /*!< bit:     24  Thumb bit        (read 0)          */
			u32 IT:2;                       /*!< bit: 25..26  saved IT state   (read 0)          */
			u32 Q:1;                        /*!< bit:     27  Saturation condition flag          */
			u32 V:1;                        /*!< bit:     28  Overflow condition code flag       */
			u32 C:1;                        /*!< bit:     29  Carry condition code flag          */
			u32 Z:1;                        /*!< bit:     30  Zero condition code flag           */
			u32 N:1;                        /*!< bit:     31  Negative condition code flag       */
		} b;                                   /*!< Structure used for bit  access                  */
		u32 w;                            /*!< Type      used for word access                  */
	} xPSR_T;


//brief  Union type to access the Control Registers (CONTROL).
 
	typedef union
	{
		struct
		{
			u32 nPRIV:1;                    /*!< bit:      0  Execution privilege in Thread mode */
			u32 SPSEL:1;                    /*!< bit:      1  Stack to be used                   */
			u32 FPCA:1;                     /*!< bit:      2  FP extension active flag           */
			u32 _reserved0:29;              /*!< bit:  3..31  Reserved                           */
		} b;                                   /*!< Structure used for bit  access                  */
		u32 w;                            /*!< Type      used for word access                  */
	} CONTROL_T;

//brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 
	typedef struct
	{
		IO32 ISER[8];                 /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register           */
		u32 RESERVED0[24];
		IO32 ICER[8];                 /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register         */
		u32 RSERVED1[24];
		IO32 ISPR[8];                 /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register          */
		u32 RESERVED2[24];
		IO32 ICPR[8];                 /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register        */
		u32 RESERVED3[24];
		IO32 IABR[8];                 /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register           */
		u32 RESERVED4[56];
		IO8  IP[240];                 /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
		u32 RESERVED5[644];
		O32 STIR;                    /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register     */
	}  NVIC_T;

//brief  Structure type to access the System Control Block (SCB).
 
	typedef struct
	{
		I32 CPUID;				/*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
		IO32 ICSR;            	/*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
		IO32 VTOR;            	/*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
		IO32 AIRCR;           	/*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
		IO32 SCR;             	/*!< Offset: 0x010 (R/W)  System Control Register                               */
		IO32 CCR;             	/*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
		IO8  SHP[12];         	/*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
		IO32 SHCSR;           	/*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
		IO32 CFSR;            	/*!< Offset: 0x028 (R/W)  Configurable Fault Status Register                    */
		IO32 HFSR;            	/*!< Offset: 0x02C (R/W)  HardFault Status Register                             */
		IO32 DFSR;            	/*!< Offset: 0x030 (R/W)  Debug Fault Status Register                           */
		IO32 MMFAR;           	/*!< Offset: 0x034 (R/W)  MemManage Fault Address Register                      */
		IO32 BFAR;            	/*!< Offset: 0x038 (R/W)  BusFault Address Register                             */
		IO32 AFSR;            	/*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       */
		I32 PFR[2];          	/*!< Offset: 0x040 (R/ )  Processor Feature Register                            */
		I32 DFR;             	/*!< Offset: 0x048 (R/ )  Debug Feature Register                                */
		I32 ADR;             	/*!< Offset: 0x04C (R/ )  Auxiliary Feature Register                            */
		I32 MMFR[4];         	/*!< Offset: 0x050 (R/ )  Memory Model Feature Register                         */
		I32 ISAR[5];         	/*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register                   */
		u32 RESERVED0[5];
		IO32 CPACR;				/*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register                   */
	} SCB_T;

//brief  Structure type to access the System Control and ID Register not in the SCB.

	typedef struct
	{
		u32 RESERVED0[1];
		I32 ICTR;               /*!< Offset: 0x004 (R/ )  Interrupt Controller Type Register      */
		IO32 ACTLR;             /*!< Offset: 0x008 (R/W)  Auxiliary Control Register      */
	} SCnSCB_T;

//brief  Structure type to access the System Timer (SysTick).

	typedef struct
	{
		IO32 	CTRL;                    /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
		IO32 	LOAD;                    /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register       */
		IO32 	VAL;                     /*!< Offset: 0x008 (R/W)  SysTick Current Value Register      */
		I32		CALIB;                   /*!< Offset: 0x00C (R/ )  SysTick Calibration Register        */
	} SysTick_T;

//brief  Structure type to access the Memory Protection Unit (MPU).

	typedef struct
	{
		I32		TYPE;                    /*!< Offset: 0x000 (R/ )  MPU Type Register                              */
		IO32 	CTRL;                    /*!< Offset: 0x004 (R/W)  MPU Control Register                           */
		IO32 	RNR;                     /*!< Offset: 0x008 (R/W)  MPU Region RNRber Register                     */
		IO32 	RBAR;                    /*!< Offset: 0x00C (R/W)  MPU Region Base Address Register               */
		IO32 	RASR;                    /*!< Offset: 0x010 (R/W)  MPU Region Attribute and Size Register         */
		IO32 	RBAR_A1;                 /*!< Offset: 0x014 (R/W)  MPU Alias 1 Region Base Address Register       */
		IO32 	RASR_A1;                 /*!< Offset: 0x018 (R/W)  MPU Alias 1 Region Attribute and Size Register */
		IO32 	RBAR_A2;                 /*!< Offset: 0x01C (R/W)  MPU Alias 2 Region Base Address Register       */
		IO32 	RASR_A2;                 /*!< Offset: 0x020 (R/W)  MPU Alias 2 Region Attribute and Size Register */
		IO32 	RBAR_A3;                 /*!< Offset: 0x024 (R/W)  MPU Alias 3 Region Base Address Register       */
		IO32 	RASR_A3;                 /*!< Offset: 0x028 (R/W)  MPU Alias 3 Region Attribute and Size Register */
	} MPU_T;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef MK_PTR
#undef MK_PTR
#endif

#ifndef WIN32
#define MK_PTR(n,p)  CM4_TYPES::n##_T * const n = ((CM4_TYPES::n##_T*)(p))
#else
extern byte CM4_sys_array[0x1000]; 
#define MK_PTR(n,p)  CM4_TYPES::n##_T * const n = ((CM4_TYPES::n##_T*)(CM4_sys_array-0xE000E000+p))
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

namespace CM4
{
//	using namespace CM3_TYPES;


// Memory mapping of Cortex-M4 Hardware 

//#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address  */
//#define ITM_BASE            (0xE0000000UL)                            /*!< ITM Base Address                   */
//#define DWT_BASE            (0xE0001000UL)                            /*!< DWT Base Address                   */
//#define TPI_BASE            (0xE0040000UL)                            /*!< TPI Base Address                   */
//#define CoreDebug_BASE      (0xE000EDF0UL)                            /*!< Core Debug Base Address            */
//#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address               */
//#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address                  */
//#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address  */

//#define SCnSCB              ((SCnSCB_Type    *)     SCS_BASE      )   /*!< System control Register not in SCB */
//#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct           */
//#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct       */
//#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct          */
//#define ITM                 ((ITM_Type       *)     ITM_BASE      )   /*!< ITM configuration struct           */
//#define DWT                 ((DWT_Type       *)     DWT_BASE      )   /*!< DWT configuration struct           */
//#define TPI                 ((TPI_Type       *)     TPI_BASE      )   /*!< TPI configuration struct           */
//#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct    */

	MK_PTR(SCnSCB,	0xE000E000);
	MK_PTR(SCB,		0xE000ED00);
	MK_PTR(SysTick, 0xE000E010);
	MK_PTR(NVIC,	0xE000E100);
	MK_PTR(MPU,		0xE000ED90);

//CM3_TYPES::SCnSCB_T		* const SCnSCB	= ((CM3_TYPES::SCnSCB_T*)	0xE000E000UL);	/*!< System control Register not in SCB */
//CM3_TYPES::SCB_T		* const SCB		= ((CM3_TYPES::SCB_T*)		0xE000ED00UL);	/*!< SCB configuration struct           */
//CM3_TYPES::SysTick_T	* const SysTick = ((CM3_TYPES::SysTick_T*)	0xE000E010UL);	/*!< SysTick configuration struct       */
//CM3_TYPES::NVIC_T		* const NVIC	= ((CM3_TYPES::NVIC_T*)		0xE000E100UL);	/*!< NVIC configuration struct          */
//CM3_TYPES::MPU_T		* const MPU		= ((CM3_TYPES::MPU_T*)		0xE000ED90UL);	/*!< Memory Protection Unit             */

};

#undef MK_PTR

#endif // CM3_H__14_11_2012__12_19
