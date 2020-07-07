#ifndef CM0_H__10_11_2014__11_54
#define CM0_H__10_11_2014__11_54

#include "types.h"

namespace CM0_TYPES
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
			u32 _reserved0:27;      /*!< bit:  0..26  Reserved                           */
			u32 Q:1;                /*!< bit:     27  Saturation condition flag          */
			u32 V:1;                /*!< bit:     28  Overflow condition code flag       */
			u32 C:1;                /*!< bit:     29  Carry condition code flag          */
			u32 Z:1;                /*!< bit:     30  Zero condition code flag           */
			u32 N:1;                /*!< bit:     31  Negative condition code flag       */
		} b;                            /*!< Structure used for bit  access                  */
		u32 w;                     /*!< Type      used for word access                  */
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
			u32 _reserved0:15;              /*!< bit:  9..23  Reserved                           */
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
		IO32 ISER[1];       // Interrupt Set Enable Register           
		u32 RESERVED0[31];
		IO32 ICER[1];       // Interrupt Clear Enable Register         
		u32 RSERVED1[31];
		IO32 ISPR[1];       // Interrupt Set Pending Register          
		u32 RESERVED2[31];
		IO32 ICPR[1];       // Interrupt Clear Pending Register        
		u32 RESERVED3[31];
		u32 RESERVED4[64];
		IO8  IP[8];		      // Interrupt Priority Register (8Bit wide) 
	}  NVIC_T;

//brief  Structure type to access the System Control Block (SCB).
 
	typedef struct
	{
		I32 CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
		IO32 ICSR;                  /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
		IO32 VTOR;					/*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */          
		IO32 AIRCR;               	/*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
		IO32 SCR;                 	/*!< Offset: 0x010 (R/W)  System Control Register                               */
		IO32 CCR;                 	/*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
		IO32 RESERVED1;
		IO8  SHP[2];			    /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
		IO32 SHCSR;                 /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
	} SCB_T;


//brief  Structure type to access the System Timer (SysTick).

	typedef struct
	{
		IO32 CTRL;                    /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
		IO32 LOAD;                    /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register       */
		IO32 VAL;                     /*!< Offset: 0x008 (R/W)  SysTick Current Value Register      */
		I32 CALIB;                   /*!< Offset: 0x00C (R/ )  SysTick Calibration Register        */
	} SysTick_T;

};

//brief  Structure type to access the Memory Protection Unit (MPU).

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef MK_PTR
#undef MK_PTR
#endif

#ifndef WIN32
#define MK_PTR(n,p)  CM0_TYPES::n##_T * const n = ((CM0_TYPES::n##_T*)(p))
#else
extern byte CM3_sys_array[0x1000]; 
#define MK_PTR(n,p)  CM3_TYPES::n##_T * const n = ((CM3_TYPES::n##_T*)(CM3_sys_array-0xE000E000+p))
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

namespace CM0
{

///* Memory mapping of Cortex-M0 Hardware */
//#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
//#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address              */
//#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address                 */
//#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

	MK_PTR(SCB,		0xE000ED00);
	MK_PTR(SysTick, 0xE000E010);
	MK_PTR(NVIC,	0xE000E100);
};

#undef MK_PTR

#endif // CM3_H__14_11_2012__12_19
