#ifndef SAMD21_H__09_04_2019__08_14
#define SAMD21_H__09_04_2019__08_14

#ifndef CORETYPE_SAME53
#error  Must #include "core.h"
#endif 

#include "types.h"

//#define __CM4_REV                 0x0200            /*!< Cortex-M4 Core Revision                                               */
//#define __MPU_PRESENT                  1            /*!< MPU present or not                                                    */
//#define __NVIC_PRIO_BITS               6            /*!< Number of Bits used for Priority Levels                               */
//#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
#define __FPU_PRESENT                  1            /*!< FPU present or not                                                    */

#include "cm4.h"

#define MCK_MHz 200
#define MCK (MCK_MHz*1000000)
#define NS2CLK(x) (((x)*MCK_MHz+500)/1000)


#ifndef WIN32
#define MK_PTR(n,p)  T_HW::S_##n * const n = ((T_HW::S_##n*)(p))
#else
extern byte core_sys_array[0x100000]; 
#define MK_PTR(n,p)  T_HW::S_##n * const n = ((T_HW::S_##n*)(core_sys_array-0x40000000+p))
#endif

#define MKPID(n,i) n##_M=(1UL<<(i&31)), n##_I=i

  /******  SAME53N19A-specific Interrupt Numbers *********************/

	#define PM_IRQ                    0		/**<  0 SAME53N19A Power Manager (PM) */
	#define MCLK_IRQ                  1		/**<  1 SAME53N19A Main Clock (MCLK) */
	#define OSCCTRL_0_IRQ             2		/**<  2 SAME53N19A Oscillators Control (OSCCTRL): OSCCTRL_XOSCFAIL_0, OSCCTRL_XOSCRDY_0 */
	#define OSCCTRL_1_IRQ             3		/**<  3 SAME53N19A Oscillators Control (OSCCTRL): OSCCTRL_XOSCFAIL_1, OSCCTRL_XOSCRDY_1 */
	#define OSCCTRL_2_IRQ             4		/**<  4 SAME53N19A Oscillators Control (OSCCTRL): OSCCTRL_DFLLLOCKC, OSCCTRL_DFLLLOCKF, OSCCTRL_DFLLOOB, OSCCTRL_DFLLRCS, OSCCTRL_DFLLRDY */
	#define OSCCTRL_3_IRQ             5		/**<  5 SAME53N19A Oscillators Control (OSCCTRL): OSCCTRL_DPLLLCKF_0, OSCCTRL_DPLLLCKR_0, OSCCTRL_DPLLLDRTO_0, OSCCTRL_DPLLLTO_0 */
	#define OSCCTRL_4_IRQ             6		/**<  6 SAME53N19A Oscillators Control (OSCCTRL): OSCCTRL_DPLLLCKF_1, OSCCTRL_DPLLLCKR_1, OSCCTRL_DPLLLDRTO_1, OSCCTRL_DPLLLTO_1 */
	#define OSC32KCTRL_IRQ            7		/**<  7 SAME53N19A 32kHz Oscillators Control (OSC32KCTRL) */
	#define SUPC_0_IRQ                8		/**<  8 SAME53N19A Supply Controller (SUPC): SUPC_B12SRDY, SUPC_B33SRDY, SUPC_BOD12RDY, SUPC_BOD33RDY, SUPC_VCORERDY, SUPC_VREGRDY */
	#define SUPC_1_IRQ                9		/**<  9 SAME53N19A Supply Controller (SUPC): SUPC_BOD12DET, SUPC_BOD33DET */
	#define WDT_IRQ                  10		/**< 10 SAME53N19A Watchdog Timer (WDT) */
	#define RTC_IRQ                  11		/**< 11 SAME53N19A Real-Time Counter (RTC) */
	#define EIC_0_IRQ                12		/**< 12 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_0 */
	#define EIC_1_IRQ                13		/**< 13 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_1 */
	#define EIC_2_IRQ                14		/**< 14 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_2 */
	#define EIC_3_IRQ                15		/**< 15 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_3 */
	#define EIC_4_IRQ                16		/**< 16 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_4 */
	#define EIC_5_IRQ                17		/**< 17 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_5 */
	#define EIC_6_IRQ                18		/**< 18 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_6 */
	#define EIC_7_IRQ                19		/**< 19 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_7 */
	#define EIC_8_IRQ                20		/**< 20 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_8 */
	#define EIC_9_IRQ                21		/**< 21 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_9 */
	#define EIC_10_IRQ               22		/**< 22 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_10 */
	#define EIC_11_IRQ               23		/**< 23 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_11 */
	#define EIC_12_IRQ               24		/**< 24 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_12 */
	#define EIC_13_IRQ               25		/**< 25 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_13 */
	#define EIC_14_IRQ               26		/**< 26 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_14 */
	#define EIC_15_IRQ               27		/**< 27 SAME53N19A External Interrupt Controller (EIC): EIC_EXTINT_15 */
	#define FREQM_IRQ                28		/**< 28 SAME53N19A Frequency Meter (FREQM) */
	#define NVMCTRL_0_IRQ            29		/**< 29 SAME53N19A Non-Volatile Memory Controller (NVMCTRL): NVMCTRL_0, NVMCTRL_1, NVMCTRL_2, NVMCTRL_3, NVMCTRL_4, NVMCTRL_5, NVMCTRL_6, NVMCTRL_7 */
	#define NVMCTRL_1_IRQ            30		/**< 30 SAME53N19A Non-Volatile Memory Controller (NVMCTRL): NVMCTRL_10, NVMCTRL_8, NVMCTRL_9 */
	#define DMAC_0_IRQ               31		/**< 31 SAME53N19A Direct Memory Access Controller (DMAC): DMAC_SUSP_0, DMAC_TCMPL_0, DMAC_TERR_0 */
	#define DMAC_1_IRQ               32		/**< 32 SAME53N19A Direct Memory Access Controller (DMAC): DMAC_SUSP_1, DMAC_TCMPL_1, DMAC_TERR_1 */
	#define DMAC_2_IRQ               33		/**< 33 SAME53N19A Direct Memory Access Controller (DMAC): DMAC_SUSP_2, DMAC_TCMPL_2, DMAC_TERR_2 */
	#define DMAC_3_IRQ               34		/**< 34 SAME53N19A Direct Memory Access Controller (DMAC): DMAC_SUSP_3, DMAC_TCMPL_3, DMAC_TERR_3 */
	#define DMAC_4_IRQ               35		/**< 35 SAME53N19A Direct Memory Access Controller (DMAC): DMAC_SUSP_10, DMAC_SUSP_11, DMAC_SUSP_12, DMAC_SUSP_13, DMAC_SUSP_14, DMAC_SUSP_15, DMAC_SUSP_16, DMAC_SUSP_17, DMAC_SUSP_18, DMAC_SUSP_19, DMAC_SUSP_20, DMAC_SUSP_21, DMAC_SUSP_22, DMAC_SUSP_23, DMAC_SUSP_24, DMAC_SUSP_25, DMAC_SUSP_26, DMAC_SUSP_27, DMAC_SUSP_28, DMAC_SUSP_29, DMAC_SUSP_30, DMAC_SUSP_31, DMAC_SUSP_4, DMAC_SUSP_5, DMAC_SUSP_6, DMAC_SUSP_7, DMAC_SUSP_8, DMAC_SUSP_9, DMAC_TCMPL_10, DMAC_TCMPL_11, DMAC_TCMPL_12, DMAC_TCMPL_13, DMAC_TCMPL_14, DMAC_TCMPL_15, DMAC_TCMPL_16, DMAC_TCMPL_17, DMAC_TCMPL_18, DMAC_TCMPL_19, DMAC_TCMPL_20, DMAC_TCMPL_21, DMAC_TCMPL_22, DMAC_TCMPL_23, DMAC_TCMPL_24, DMAC_TCMPL_25, DMAC_TCMPL_26, DMAC_TCMPL_27, DMAC_TCMPL_28, DMAC_TCMPL_29, DMAC_TCMPL_30, DMAC_TCMPL_31, DMAC_TCMPL_4, DMAC_TCMPL_5, DMAC_TCMPL_6, DMAC_TCMPL_7, DMAC_TCMPL_8, DMAC_TCMPL_9, DMAC_TERR_10, DMAC_TERR_11, DMAC_TERR_12, DMAC_TERR_13, DMAC_TERR_14, DMAC_TERR_15, DMAC_TERR_16, DMAC_TERR_17, DMAC_TERR_18, DMAC_TERR_19, DMAC_TERR_20, DMAC_TERR_21, DMAC_TERR_22, DMAC_TERR_23, DMAC_TERR_24, DMAC_TERR_25, DMAC_TERR_26, DMAC_TERR_27, DMAC_TERR_28, DMAC_TERR_29, DMAC_TERR_30, DMAC_TERR_31, DMAC_TERR_4, DMAC_TERR_5, DMAC_TERR_6, DMAC_TERR_7, DMAC_TERR_8, DMAC_TERR_9 */
	#define EVSYS_0_IRQ              36		/**< 36 SAME53N19A Event System Interface (EVSYS): EVSYS_EVD_0, EVSYS_OVR_0 */
	#define EVSYS_1_IRQ              37		/**< 37 SAME53N19A Event System Interface (EVSYS): EVSYS_EVD_1, EVSYS_OVR_1 */
	#define EVSYS_2_IRQ              38		/**< 38 SAME53N19A Event System Interface (EVSYS): EVSYS_EVD_2, EVSYS_OVR_2 */
	#define EVSYS_3_IRQ              39		/**< 39 SAME53N19A Event System Interface (EVSYS): EVSYS_EVD_3, EVSYS_OVR_3 */
	#define EVSYS_4_IRQ              40		/**< 40 SAME53N19A Event System Interface (EVSYS): EVSYS_EVD_10, EVSYS_EVD_11, EVSYS_EVD_4, EVSYS_EVD_5, EVSYS_EVD_6, EVSYS_EVD_7, EVSYS_EVD_8, EVSYS_EVD_9, EVSYS_OVR_10, EVSYS_OVR_11, EVSYS_OVR_4, EVSYS_OVR_5, EVSYS_OVR_6, EVSYS_OVR_7, EVSYS_OVR_8, EVSYS_OVR_9 */
	#define PAC_IRQ                  41		/**< 41 SAME53N19A Peripheral Access Controller (PAC) */
	#define RAMECC_IRQ               45		/**< 45 SAME53N19A RAM ECC (RAMECC) */
	#define SERCOM0_0_IRQ            46		/**< 46 SAME53N19A Serial Communication Interface 0 (SERCOM0): SERCOM0_0 */
	#define SERCOM0_1_IRQ            47		/**< 47 SAME53N19A Serial Communication Interface 0 (SERCOM0): SERCOM0_1 */
	#define SERCOM0_2_IRQ            48		/**< 48 SAME53N19A Serial Communication Interface 0 (SERCOM0): SERCOM0_2 */
	#define SERCOM0_3_IRQ            49		/**< 49 SAME53N19A Serial Communication Interface 0 (SERCOM0): SERCOM0_3, SERCOM0_4, SERCOM0_5, SERCOM0_6 */
	#define SERCOM1_0_IRQ            50		/**< 50 SAME53N19A Serial Communication Interface 1 (SERCOM1): SERCOM1_0 */
	#define SERCOM1_1_IRQ            51		/**< 51 SAME53N19A Serial Communication Interface 1 (SERCOM1): SERCOM1_1 */
	#define SERCOM1_2_IRQ            52		/**< 52 SAME53N19A Serial Communication Interface 1 (SERCOM1): SERCOM1_2 */
	#define SERCOM1_3_IRQ            53		/**< 53 SAME53N19A Serial Communication Interface 1 (SERCOM1): SERCOM1_3, SERCOM1_4, SERCOM1_5, SERCOM1_6 */
	#define SERCOM2_0_IRQ            54		/**< 54 SAME53N19A Serial Communication Interface 2 (SERCOM2): SERCOM2_0 */
	#define SERCOM2_1_IRQ            55		/**< 55 SAME53N19A Serial Communication Interface 2 (SERCOM2): SERCOM2_1 */
	#define SERCOM2_2_IRQ            56		/**< 56 SAME53N19A Serial Communication Interface 2 (SERCOM2): SERCOM2_2 */
	#define SERCOM2_3_IRQ            57		/**< 57 SAME53N19A Serial Communication Interface 2 (SERCOM2): SERCOM2_3, SERCOM2_4, SERCOM2_5, SERCOM2_6 */
	#define SERCOM3_0_IRQ            58		/**< 58 SAME53N19A Serial Communication Interface 3 (SERCOM3): SERCOM3_0 */
	#define SERCOM3_1_IRQ            59		/**< 59 SAME53N19A Serial Communication Interface 3 (SERCOM3): SERCOM3_1 */
	#define SERCOM3_2_IRQ            60		/**< 60 SAME53N19A Serial Communication Interface 3 (SERCOM3): SERCOM3_2 */
	#define SERCOM3_3_IRQ            61		/**< 61 SAME53N19A Serial Communication Interface 3 (SERCOM3): SERCOM3_3, SERCOM3_4, SERCOM3_5, SERCOM3_6 */
	#define SERCOM4_0_IRQ            62		/**< 62 SAME53N19A Serial Communication Interface 4 (SERCOM4): SERCOM4_0 */
	#define SERCOM4_1_IRQ            63		/**< 63 SAME53N19A Serial Communication Interface 4 (SERCOM4): SERCOM4_1 */
	#define SERCOM4_2_IRQ            64		/**< 64 SAME53N19A Serial Communication Interface 4 (SERCOM4): SERCOM4_2 */
	#define SERCOM4_3_IRQ            65		/**< 65 SAME53N19A Serial Communication Interface 4 (SERCOM4): SERCOM4_3, SERCOM4_4, SERCOM4_5, SERCOM4_6 */
	#define SERCOM5_0_IRQ            66		/**< 66 SAME53N19A Serial Communication Interface 5 (SERCOM5): SERCOM5_0 */
	#define SERCOM5_1_IRQ            67		/**< 67 SAME53N19A Serial Communication Interface 5 (SERCOM5): SERCOM5_1 */
	#define SERCOM5_2_IRQ            68		/**< 68 SAME53N19A Serial Communication Interface 5 (SERCOM5): SERCOM5_2 */
	#define SERCOM5_3_IRQ            69		/**< 69 SAME53N19A Serial Communication Interface 5 (SERCOM5): SERCOM5_3, SERCOM5_4, SERCOM5_5, SERCOM5_6 */
	#define SERCOM6_0_IRQ            70		/**< 70 SAME53N19A Serial Communication Interface 6 (SERCOM6): SERCOM6_0 */
	#define SERCOM6_1_IRQ            71		/**< 71 SAME53N19A Serial Communication Interface 6 (SERCOM6): SERCOM6_1 */
	#define SERCOM6_2_IRQ            72		/**< 72 SAME53N19A Serial Communication Interface 6 (SERCOM6): SERCOM6_2 */
	#define SERCOM6_3_IRQ            73		/**< 73 SAME53N19A Serial Communication Interface 6 (SERCOM6): SERCOM6_3, SERCOM6_4, SERCOM6_5, SERCOM6_6 */
	#define SERCOM7_0_IRQ            74		/**< 74 SAME53N19A Serial Communication Interface 7 (SERCOM7): SERCOM7_0 */
	#define SERCOM7_1_IRQ            75		/**< 75 SAME53N19A Serial Communication Interface 7 (SERCOM7): SERCOM7_1 */
	#define SERCOM7_2_IRQ            76		/**< 76 SAME53N19A Serial Communication Interface 7 (SERCOM7): SERCOM7_2 */
	#define SERCOM7_3_IRQ            77		/**< 77 SAME53N19A Serial Communication Interface 7 (SERCOM7): SERCOM7_3, SERCOM7_4, SERCOM7_5, SERCOM7_6 */
	#define USB_0_IRQ                80		/**< 80 SAME53N19A Universal Serial Bus (USB): USB_EORSM_DNRSM, USB_EORST_RST, USB_LPMSUSP_DDISC, USB_LPM_DCONN, USB_MSOF, USB_RAMACER, USB_RXSTP_TXSTP_0, USB_RXSTP_TXSTP_1, USB_RXSTP_TXSTP_2, USB_RXSTP_TXSTP_3, USB_RXSTP_TXSTP_4, USB_RXSTP_TXSTP_5, USB_RXSTP_TXSTP_6, USB_RXSTP_TXSTP_7, USB_STALL0_STALL_0, USB_STALL0_STALL_1, USB_STALL0_STALL_2, USB_STALL0_STALL_3, USB_STALL0_STALL_4, USB_STALL0_STALL_5, USB_STALL0_STALL_6, USB_STALL0_STALL_7, USB_STALL1_0, USB_STALL1_1, USB_STALL1_2, USB_STALL1_3, USB_STALL1_4, USB_STALL1_5, USB_STALL1_6, USB_STALL1_7, USB_SUSPEND, USB_TRFAIL0_TRFAIL_0, USB_TRFAIL0_TRFAIL_1, USB_TRFAIL0_TRFAIL_2, USB_TRFAIL0_TRFAIL_3, USB_TRFAIL0_TRFAIL_4, USB_TRFAIL0_TRFAIL_5, USB_TRFAIL0_TRFAIL_6, USB_TRFAIL0_TRFAIL_7, USB_TRFAIL1_PERR_0, USB_TRFAIL1_PERR_1, USB_TRFAIL1_PERR_2, USB_TRFAIL1_PERR_3, USB_TRFAIL1_PERR_4, USB_TRFAIL1_PERR_5, USB_TRFAIL1_PERR_6, USB_TRFAIL1_PERR_7, USB_UPRSM, USB_WAKEUP */
	#define USB_1_IRQ                81		/**< 81 SAME53N19A Universal Serial Bus (USB): USB_SOF_HSOF */
	#define USB_2_IRQ                82		/**< 82 SAME53N19A Universal Serial Bus (USB): USB_TRCPT0_0, USB_TRCPT0_1, USB_TRCPT0_2, USB_TRCPT0_3, USB_TRCPT0_4, USB_TRCPT0_5, USB_TRCPT0_6, USB_TRCPT0_7 */
	#define USB_3_IRQ                83		/**< 83 SAME53N19A Universal Serial Bus (USB): USB_TRCPT1_0, USB_TRCPT1_1, USB_TRCPT1_2, USB_TRCPT1_3, USB_TRCPT1_4, USB_TRCPT1_5, USB_TRCPT1_6, USB_TRCPT1_7 */
	#define GMAC_IRQ                 84		/**< 84 SAME53N19A Ethernet MAC (GMAC) */
	#define TCC0_0_IRQ               85		/**< 85 SAME53N19A Timer Counter Control 0 (TCC0): TCC0_CNT_A, TCC0_DFS_A, TCC0_ERR_A, TCC0_FAULT0_A, TCC0_FAULT1_A, TCC0_FAULTA_A, TCC0_FAULTB_A, TCC0_OVF, TCC0_TRG, TCC0_UFS_A */
	#define TCC0_1_IRQ               86		/**< 86 SAME53N19A Timer Counter Control 0 (TCC0): TCC0_MC_0 */
	#define TCC0_2_IRQ               87		/**< 87 SAME53N19A Timer Counter Control 0 (TCC0): TCC0_MC_1 */
	#define TCC0_3_IRQ               88		/**< 88 SAME53N19A Timer Counter Control 0 (TCC0): TCC0_MC_2 */
	#define TCC0_4_IRQ               89		/**< 89 SAME53N19A Timer Counter Control 0 (TCC0): TCC0_MC_3 */
	#define TCC0_5_IRQ               90		/**< 90 SAME53N19A Timer Counter Control 0 (TCC0): TCC0_MC_4 */
	#define TCC0_6_IRQ               91		/**< 91 SAME53N19A Timer Counter Control 0 (TCC0): TCC0_MC_5 */
	#define TCC1_0_IRQ               92		/**< 92 SAME53N19A Timer Counter Control 1 (TCC1): TCC1_CNT_A, TCC1_DFS_A, TCC1_ERR_A, TCC1_FAULT0_A, TCC1_FAULT1_A, TCC1_FAULTA_A, TCC1_FAULTB_A, TCC1_OVF, TCC1_TRG, TCC1_UFS_A */
	#define TCC1_1_IRQ               93		/**< 93 SAME53N19A Timer Counter Control 1 (TCC1): TCC1_MC_0 */
	#define TCC1_2_IRQ               94		/**< 94 SAME53N19A Timer Counter Control 1 (TCC1): TCC1_MC_1 */
	#define TCC1_3_IRQ               95		/**< 95 SAME53N19A Timer Counter Control 1 (TCC1): TCC1_MC_2 */
	#define TCC1_4_IRQ               96		/**< 96 SAME53N19A Timer Counter Control 1 (TCC1): TCC1_MC_3 */
	#define TCC2_0_IRQ               97		/**< 97 SAME53N19A Timer Counter Control 2 (TCC2): TCC2_CNT_A, TCC2_DFS_A, TCC2_ERR_A, TCC2_FAULT0_A, TCC2_FAULT1_A, TCC2_FAULTA_A, TCC2_FAULTB_A, TCC2_OVF, TCC2_TRG, TCC2_UFS_A */
	#define TCC2_1_IRQ               98		/**< 98 SAME53N19A Timer Counter Control 2 (TCC2): TCC2_MC_0 */
	#define TCC2_2_IRQ               99		/**< 99 SAME53N19A Timer Counter Control 2 (TCC2): TCC2_MC_1 */
	#define TCC2_3_IRQ               100 	/**< 100 SAME53N19A Timer Counter Control 2 (TCC2): TCC2_MC_2 */
	#define TCC3_0_IRQ               101 	/**< 101 SAME53N19A Timer Counter Control 3 (TCC3): TCC3_CNT_A, TCC3_DFS_A, TCC3_ERR_A, TCC3_FAULT0_A, TCC3_FAULT1_A, TCC3_FAULTA_A, TCC3_FAULTB_A, TCC3_OVF, TCC3_TRG, TCC3_UFS_A */
	#define TCC3_1_IRQ               102 	/**< 102 SAME53N19A Timer Counter Control 3 (TCC3): TCC3_MC_0 */
	#define TCC3_2_IRQ               103 	/**< 103 SAME53N19A Timer Counter Control 3 (TCC3): TCC3_MC_1 */
	#define TCC4_0_IRQ               104 	/**< 104 SAME53N19A Timer Counter Control 4 (TCC4): TCC4_CNT_A, TCC4_DFS_A, TCC4_ERR_A, TCC4_FAULT0_A, TCC4_FAULT1_A, TCC4_FAULTA_A, TCC4_FAULTB_A, TCC4_OVF, TCC4_TRG, TCC4_UFS_A */
	#define TCC4_1_IRQ               105 	/**< 105 SAME53N19A Timer Counter Control 4 (TCC4): TCC4_MC_0 */
	#define TCC4_2_IRQ               106 	/**< 106 SAME53N19A Timer Counter Control 4 (TCC4): TCC4_MC_1 */
	#define TC0_IRQ                  107 	/**< 107 SAME53N19A Basic Timer Counter 0 (TC0) */
	#define TC1_IRQ                  108 	/**< 108 SAME53N19A Basic Timer Counter 1 (TC1) */
	#define TC2_IRQ                  109 	/**< 109 SAME53N19A Basic Timer Counter 2 (TC2) */
	#define TC3_IRQ                  110 	/**< 110 SAME53N19A Basic Timer Counter 3 (TC3) */
	#define TC4_IRQ                  111 	/**< 111 SAME53N19A Basic Timer Counter 4 (TC4) */
	#define TC5_IRQ                  112 	/**< 112 SAME53N19A Basic Timer Counter 5 (TC5) */
	#define TC6_IRQ                  113 	/**< 113 SAME53N19A Basic Timer Counter 6 (TC6) */
	#define TC7_IRQ                  114 	/**< 114 SAME53N19A Basic Timer Counter 7 (TC7) */
	#define PDEC_0_IRQ               115 	/**< 115 SAME53N19A Quadrature Decodeur (PDEC): PDEC_DIR_A, PDEC_ERR_A, PDEC_OVF, PDEC_VLC_A */
	#define PDEC_1_IRQ               116 	/**< 116 SAME53N19A Quadrature Decodeur (PDEC): PDEC_MC_0 */
	#define PDEC_2_IRQ               117 	/**< 117 SAME53N19A Quadrature Decodeur (PDEC): PDEC_MC_1 */
	#define ADC0_0_IRQ               118 	/**< 118 SAME53N19A Analog Digital Converter 0 (ADC0): ADC0_OVERRUN, ADC0_WINMON */
	#define ADC0_1_IRQ               119 	/**< 119 SAME53N19A Analog Digital Converter 0 (ADC0): ADC0_RESRDY */
	#define ADC1_0_IRQ               120 	/**< 120 SAME53N19A Analog Digital Converter 1 (ADC1): ADC1_OVERRUN, ADC1_WINMON */
	#define ADC1_1_IRQ               121 	/**< 121 SAME53N19A Analog Digital Converter 1 (ADC1): ADC1_RESRDY */
	#define AC_IRQ                   122 	/**< 122 SAME53N19A Analog Comparators (AC) */
	#define DAC_0_IRQ                123 	/**< 123 SAME53N19A Digital-to-Analog Converter (DAC): DAC_OVERRUN_A_0, DAC_OVERRUN_A_1, DAC_UNDERRUN_A_0, DAC_UNDERRUN_A_1 */
	#define DAC_1_IRQ                124 	/**< 124 SAME53N19A Digital-to-Analog Converter (DAC): DAC_EMPTY_0 */
	#define DAC_2_IRQ                125 	/**< 125 SAME53N19A Digital-to-Analog Converter (DAC): DAC_EMPTY_1 */
	#define DAC_3_IRQ                126 	/**< 126 SAME53N19A Digital-to-Analog Converter (DAC): DAC_RESRDY_0 */
	#define DAC_4_IRQ                127 	/**< 127 SAME53N19A Digital-to-Analog Converter (DAC): DAC_RESRDY_1 */
	#define I2S_IRQ                  128 	/**< 128 SAME53N19A Inter-IC Sound Interface (I2S) */
	#define PCC_IRQ                  129 	/**< 129 SAME53N19A Parallel Capture Controller (PCC) */
	#define AES_IRQ                  130 	/**< 130 SAME53N19A Advanced Encryption Standard (AES) */
	#define TRNG_IRQ                 131 	/**< 131 SAME53N19A True Random Generator (TRNG) */
	#define ICM_IRQ                  132 	/**< 132 SAME53N19A Integrity Check Monitor (ICM) */
	#define PUKCC_IRQ                133 	/**< 133 SAME53N19A PUblic-Key Cryptography Controller (PUKCC) */
	#define QSPI_IRQ                 134 	/**< 134 SAME53N19A Quad SPI interface (QSPI) */
	#define SDHC0_IRQ                135 	/**< 135 SAME53N19A SD/MMC Host Controller 0 (SDHC0) */
	#define SDHC1_IRQ                136 	/**< 136 SAME53N19A SD/MMC Host Controller 1 (SDHC1) */



#pragma anon_unions

namespace T_HW
{
	typedef volatile unsigned int AT91_REG;// Hardware register definition
	typedef volatile void * AT91_PTR;// Hardware register definition

	typedef volatile void * RW_PTR;	
	typedef volatile const void * RO_PTR;	

	typedef void(*AT91_IHP)() __irq;	// Interrupt handler pointer

	typedef volatile const byte RO8, RoReg8;
	typedef volatile const u16 RO16, RoReg16;
	typedef volatile const u32 RO32, RoReg;

	typedef volatile byte RW8, RwReg8, WoReg8;
	typedef volatile u16 RW16, RwReg16;
	typedef volatile u32 RW32, RwReg;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_GCLK
	{
		RW8     CTRLA;				/**< \brief Offset: 0x00 (R/W  8) Control */
		RO8     z_Reserved1[0x3];
		RO32    SYNCBUSY;			/**< \brief Offset: 0x04 (R/  32) Synchronization Busy */
		RO8     z_Reserved2[0x18];
		RW32	GENCTRL[12];		/**< \brief Offset: 0x20 (R/W 32) Generic Clock Generator Control */
		RO8     z_Reserved3[0x30];
		RW32	PCHCTRL[48];		/**< \brief Offset: 0x80 (R/W 32) Peripheral Clock Control */

		//void	GenDivCtrl(byte genid, u16 div, u32 ctrl) { GENCTRL[genid] = GENDIV = genid|((u32)div<<8); GENCTRL = genid|ctrl; }
	};

	/* ========== Instance parameters for GCLK peripheral ========== */

	#define GCLK_SWRST					(1)           /**< \brief (GCLK_CTRLA) Software Reset */

	#define GCLK_GENCTRL0  				(1UL<<2)    /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 0 Synchronization Busy bits */
	#define GCLK_GENCTRL1  				(1UL<<3)    /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 1 Synchronization Busy bits */
	#define GCLK_GENCTRL2  				(1UL<<4)    /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 2 Synchronization Busy bits */
	#define GCLK_GENCTRL3  				(1UL<<5)    /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 3 Synchronization Busy bits */
	#define GCLK_GENCTRL4  				(1UL<<6)    /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 4 Synchronization Busy bits */
	#define GCLK_GENCTRL5  				(1UL<<7)    /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 5 Synchronization Busy bits */
	#define GCLK_GENCTRL6  				(1UL<<8)    /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 6 Synchronization Busy bits */
	#define GCLK_GENCTRL7  				(1UL<<9)    /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 7 Synchronization Busy bits */
	#define GCLK_GENCTRL8  				(1UL<<10)   /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 8 Synchronization Busy bits */
	#define GCLK_GENCTRL9  				(1UL<<11)   /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 9 Synchronization Busy bits */
	#define GCLK_GENCTRL10 				(1UL<<12)   /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 10 Synchronization Busy bits */
	#define GCLK_GENCTRL11 				(1UL<<13)   /**< \brief (GCLK_SYNCBUSY) Generic Clock Generator Control 11 Synchronization Busy bits */

	#define	GCLK_SRC_XOSC0      		(0x0)   	/**< \brief (GCLK_GENCTRL) XOSC0 oscillator output */
	#define	GCLK_SRC_XOSC1      		(0x1)   	/**< \brief (GCLK_GENCTRL) XOSC1 oscillator output */
	#define	GCLK_SRC_GCLKIN     		(0x2)   	/**< \brief (GCLK_GENCTRL) Generator input pad */
	#define	GCLK_SRC_GCLKGEN1   		(0x3)   	/**< \brief (GCLK_GENCTRL) Generic clock generator 1 output */
	#define	GCLK_SRC_OSCULP32K  		(0x4)   	/**< \brief (GCLK_GENCTRL) OSCULP32K oscillator output */
	#define	GCLK_SRC_XOSC32K    		(0x5)   	/**< \brief (GCLK_GENCTRL) XOSC32K oscillator output */
	#define	GCLK_SRC_DFLL       		(0x6)   	/**< \brief (GCLK_GENCTRL) DFLL output */
	#define	GCLK_SRC_DPLL0      		(0x7)   	/**< \brief (GCLK_GENCTRL) DPLL0 output */
	#define	GCLK_SRC_DPLL1      		(0x8)   	/**< \brief (GCLK_GENCTRL) DPLL1 output */

	#define GCLK_GENEN					(1UL<<8)	/**< \brief (GCLK_GENCTRL) Generic Clock Generator Enable */
	#define GCLK_IDC        			(1UL<<9)	/**< \brief (GCLK_GENCTRL) Improve Duty Cycle */
	#define GCLK_OOV        			(1UL<<10)	/**< \brief (GCLK_GENCTRL) Output Off Value */
	#define GCLK_OE         			(1UL<<11)	/**< \brief (GCLK_GENCTRL) Output Enable */
	#define GCLK_DIVSEL     			(1UL<<12)	/**< \brief (GCLK_GENCTRL) Divide Selection */
	#define GCLK_RUNSTDBY   			(1UL<<13)	/**< \brief (GCLK_GENCTRL) Run in Standby */

	#define GCLK_DIV(value)				((value)<<16)

	#define GCLK_OSCCTRL_DFLL48			0	   	
	#define GCLK_OSCCTRL_FDPLL0			1
	#define GCLK_OSCCTRL_FDPLL1			2
	#define GCLK_OSCCTRL_FDPLL0_32K		3
	#define GCLK_OSCCTRL_FDPLL1_32K		3
	#define GCLK_SDHC0_SLOW				3
	#define GCLK_SDHC1_SLOW				3
	#define GCLK_SERCOM_SLOW			3
	#define GCLK_EIC					4
	#define GCLK_FREQM_MSR				5
	#define GCLK_FREQM_REF				6
	#define GCLK_SERCOM0_CORE			7
	#define GCLK_SERCOM1_CORE			8
	#define GCLK_TC0_TC1				9
	#define GCLK_USB					10
	#define GCLK_EVSYS0					11
	#define GCLK_EVSYS1					12
	#define GCLK_EVSYS2					13
	#define GCLK_EVSYS3					13
	#define GCLK_EVSYS4					15
	#define GCLK_EVSYS5					16
	#define GCLK_EVSYS6					17
	#define GCLK_EVSYS7					18
	#define GCLK_EVSYS8					19
	#define GCLK_EVSYS9					20
	#define GCLK_EVSYS10				21
	#define GCLK_EVSYS11				22
	#define GCLK_SERCOM2_CORE			23
	#define GCLK_SERCOM3_CORE			24
	#define GCLK_TCC0_TCC1 				25
	#define GCLK_TC2_TC3 				26
	#define GCLK_CAN0 					27
	#define GCLK_CAN1 					28
	#define GCLK_TCC2_TCC3 				29
	#define GCLK_TC4_TC5 				30
	#define GCLK_PDEC 					31
	#define GCLK_AC						32
	#define GCLK_CCL					33
	#define GCLK_SERCOM4_CORE			34
	#define GCLK_SERCOM5_CORE			35
	#define GCLK_SERCOM6_CORE			36
	#define GCLK_SERCOM7_CORE			37
	#define GCLK_TCC4					38
	#define GCLK_TC6_TC7				39
	#define GCLK_ADC0					40
	#define GCLK_ADC1					41
	#define GCLK_DAC					42
	#define GCLK_I2S0					43
	#define GCLK_I2S1					44
	#define GCLK_SDHC0 					45
	#define GCLK_SDHC1 					46
	#define GCLK_CM4_TRACE 				47
										 
	#define GCLK_GEN(value)				((value)&0xF)	/**< \brief (GCLK_CLKCTRL) Generic Clock Generator */
	#define GCLK_CHEN					(1<<6)					/**< \brief (GCLK_CLKCTRL) Clock Enable */
	#define GCLK_WRTLOCK				(1<<7)					/**< \brief (GCLK_CLKCTRL) Write Lock */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_PM
	{
		RW8     CTRLA;       /**< \brief Offset: 0x00 (R/W  8) Control A */
		RW8		SLEEPCFG;    /**< \brief Offset: 0x01 (R/W  8) Sleep Configuration */
		RO8					z_Reserved1[0x2];
		RW8		INTENCLR;    /**< \brief Offset: 0x04 (R/W  8) Interrupt Enable Clear */
		RW8		INTENSET;    /**< \brief Offset: 0x05 (R/W  8) Interrupt Enable Set */
		RW8		INTFLAG;     /**< \brief Offset: 0x06 (R/W  8) Interrupt Flag Status and Clear */
		RO8					z_Reserved2[0x1];
		RW8     STDBYCFG;    /**< \brief Offset: 0x08 (R/W  8) Standby Configuration */
		RW8     HIBCFG;      /**< \brief Offset: 0x09 (R/W  8) Hibernate Configuration */
		RW8     BKUPCFG;     /**< \brief Offset: 0x0A (R/W  8) Backup Configuration */
		RO8					z_Reserved3[0x7];
		RW8     PWSAKDLY;    /**< \brief Offset: 0x12 (R/W  8) Power Switch Acknowledge Delay */
	};

	/* ========== Instance parameters for PM peripheral ========== */
	
	#define PM_IORET					(1<<2)          /**< \brief (PM_CTRLA) I/O Retention */

	#define	PM_SLEEPMODE_IDLE0 			(0x0)   		/**< \brief (PM_SLEEPCFG) CPU clock is OFF */
	#define	PM_SLEEPMODE_IDLE1 			(0x1)   		/**< \brief (PM_SLEEPCFG) AHB clock is OFF */
	#define	PM_SLEEPMODE_IDLE2 			(0x2)   		/**< \brief (PM_SLEEPCFG) APB clock are OFF */
	#define	PM_SLEEPMODE_STANDBY		(0x4)   		/**< \brief (PM_SLEEPCFG) All Clocks are OFF */
	#define	PM_SLEEPMODE_HIBERNATE		(0x5)   		/**< \brief (PM_SLEEPCFG) Backup domain is ON as well as some PDRAMs */
	#define	PM_SLEEPMODE_BACKUP			(0x6)   		/**< \brief (PM_SLEEPCFG) Only Backup domain is powered ON */
	#define	PM_SLEEPMODE_OFF			(0x7)   		/**< \brief (PM_SLEEPCFG) All power domains are powered OFF */

	#define PM_SLEEPRDY					(1<<0)          /**< \brief (PM_INTENCLR) Sleep Mode Entry Ready Enable */

	#define	PM_RAMCFG_RET_Val      		(0x0)   		/**< \brief (PM_STDBYCFG) All the RAMs are retained */
	#define	PM_RAMCFG_PARTIAL_Val  		(0x1)   		/**< \brief (PM_STDBYCFG) Only the first 32K bytes are retained */
	#define	PM_RAMCFG_OFF_Val      		(0x2)   		/**< \brief (PM_STDBYCFG) All the RAMs are OFF */

	#define	PM_BRAMCFG_RET_Val      	(0x0<<2)   		/**< \brief (PM_STDBYCFG) All the RAMs are retained */
	#define	PM_BRAMCFG_PARTIAL_Val  	(0x1<<2)   		/**< \brief (PM_STDBYCFG) Only the first 32K bytes are retained */
	#define	PM_BRAMCFG_OFF_Val      	(0x2<<2)   		/**< \brief (PM_STDBYCFG) All the RAMs are OFF */

	#define	PM_FASTWKUP_NO 				(0x0<<4) 		/**< \brief Fast Wakeup is disabled.		*/
	#define	PM_FASTWKUP_NVM 			(0x1<<4) 		/**< \brief Fast Wakeup is enabled on NVM.		*/
	#define	PM_FASTWKUP_MAINVREG 		(0x2<<4) 		/**< \brief Fast Wakeup is enabled on the main voltage regulator (MAINVREG).		*/
	#define	PM_FASTWKUP_BOTH 			(0x3<<4) 		/**< \brief Fast Wakeup is enabled on both NVM and MAINVREG..		*/

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_MCLK
	{
		RO8                    z_Reserved1[0x1];
		RW8    		INTENCLR;    /**< \brief Offset: 0x01 (R/W  8) Interrupt Enable Clear */
		RW8    		INTENSET;    /**< \brief Offset: 0x02 (R/W  8) Interrupt Enable Set */
		RW8    		INTFLAG;     /**< \brief Offset: 0x03 (R/W  8) Interrupt Flag Status and Clear */
		RO8         HSDIV;       /**< \brief Offset: 0x04 (R/   8) HS Clock Division */
		RW8         CPUDIV;      /**< \brief Offset: 0x05 (R/W  8) CPU Clock Division */
		RO8                    z_Reserved2[0xA];
		RW32  		AHBMASK;     /**< \brief Offset: 0x10 (R/W 32) AHB Mask */
		RW32  		APBAMASK;    /**< \brief Offset: 0x14 (R/W 32) APBA Mask */
		RW32  		APBBMASK;    /**< \brief Offset: 0x18 (R/W 32) APBB Mask */
		RW32  		APBCMASK;    /**< \brief Offset: 0x1C (R/W 32) APBC Mask */
		RW32  		APBDMASK;    /**< \brief Offset: 0x20 (R/W 32) APBD Mask */
	};

	#define MCLK_CKRDY				(1<<0)            /**< \brief (MCLK_INTENCLR) Clock Ready Interrupt Enable */

	#define AHB_HPB0       			(1<<0)        	/**< \brief (MCLK_AHBMASK) HPB0 AHB Clock Mask */
	#define AHB_HPB1       			(1<<1)        	/**< \brief (MCLK_AHBMASK) HPB1 AHB Clock Mask */
	#define AHB_HPB2       			(1<<2)        	/**< \brief (MCLK_AHBMASK) HPB2 AHB Clock Mask */
	#define AHB_HPB3       			(1<<3)        	/**< \brief (MCLK_AHBMASK) HPB3 AHB Clock Mask */
	#define AHB_DSU        			(1<<4)        	/**< \brief (MCLK_AHBMASK) DSU AHB Clock Mask */
	#define AHB_HMATRIX    			(1<<5)        	/**< \brief (MCLK_AHBMASK) HMATRIX AHB Clock Mask */
	#define AHB_NVMCTRL    			(1<<6)        	/**< \brief (MCLK_AHBMASK) NVMCTRL AHB Clock Mask */
	#define AHB_HSRAM      			(1<<7)        	/**< \brief (MCLK_AHBMASK) HSRAM AHB Clock Mask */
	#define AHB_CMCC       			(1<<8)        	/**< \brief (MCLK_AHBMASK) CMCC AHB Clock Mask */
	#define AHB_DMAC       			(1<<9)        	/**< \brief (MCLK_AHBMASK) DMAC AHB Clock Mask */
	#define AHB_USB        			(1<<10)       	/**< \brief (MCLK_AHBMASK) USB AHB Clock Mask */
	#define AHB_BKUPRAM    			(1<<11)       	/**< \brief (MCLK_AHBMASK) BKUPRAM AHB Clock Mask */
	#define AHB_PAC        			(1<<12)       	/**< \brief (MCLK_AHBMASK) PAC AHB Clock Mask */
	#define AHB_QSPI       			(1<<13)       	/**< \brief (MCLK_AHBMASK) QSPI AHB Clock Mask */
	#define AHB_GMAC       			(1<<14)       	/**< \brief (MCLK_AHBMASK) GMAC AHB Clock Mask */
	#define AHB_SDHC0      			(1<<15)       	/**< \brief (MCLK_AHBMASK) SDHC0 AHB Clock Mask */
	#define AHB_SDHC1      			(1<<16)       	/**< \brief (MCLK_AHBMASK) SDHC1 AHB Clock Mask */
	#define AHB_ICM        			(1<<19)       	/**< \brief (MCLK_AHBMASK) ICM AHB Clock Mask */
	#define AHB_PUKCC      			(1<<20)       	/**< \brief (MCLK_AHBMASK) PUKCC AHB Clock Mask */
	#define AHB_QSPI_2X    			(1<<21)       	/**< \brief (MCLK_AHBMASK) QSPI_2X AHB Clock Mask */
	#define AHB_NVMCTRL_SMEEPROM	(1<<22)       	/**< \brief (MCLK_AHBMASK) NVMCTRL_SMEEPROM AHB Clock Mask */
	#define AHB_NVMCTRL_CACHE		(1<<23)       	/**< \brief (MCLK_AHBMASK) NVMCTRL_CACHE AHB Clock Mask */

	#define APBA_PAC       			(1<<0)        	/**< \brief (MCLK_APBAMASK) PAC APB Clock Enable */
	#define APBA_PM        			(1<<1)        	/**< \brief (MCLK_APBAMASK) PM APB Clock Enable */
	#define APBA_MCLK      			(1<<2)        	/**< \brief (MCLK_APBAMASK) MCLK APB Clock Enable */
	#define APBA_RSTC      			(1<<3)        	/**< \brief (MCLK_APBAMASK) RSTC APB Clock Enable */
	#define APBA_OSCCTRL   			(1<<4)        	/**< \brief (MCLK_APBAMASK) OSCCTRL APB Clock Enable */
	#define APBA_OSC32KCTRL			(1<<5)        	/**< \brief (MCLK_APBAMASK) OSC32KCTRL APB Clock Enable */
	#define APBA_SUPC      			(1<<6)        	/**< \brief (MCLK_APBAMASK) SUPC APB Clock Enable */
	#define APBA_GCLK      			(1<<7)        	/**< \brief (MCLK_APBAMASK) GCLK APB Clock Enable */
	#define APBA_WDT       			(1<<8)        	/**< \brief (MCLK_APBAMASK) WDT APB Clock Enable */
	#define APBA_RTC       			(1<<9)        	/**< \brief (MCLK_APBAMASK) RTC APB Clock Enable */
	#define APBA_EIC       			(1<<10)       	/**< \brief (MCLK_APBAMASK) EIC APB Clock Enable */
	#define APBA_FREQM     			(1<<11)       	/**< \brief (MCLK_APBAMASK) FREQM APB Clock Enable */
	#define APBA_SERCOM0   			(1<<12)       	/**< \brief (MCLK_APBAMASK) SERCOM0 APB Clock Enable */
	#define APBA_SERCOM1   			(1<<13)       	/**< \brief (MCLK_APBAMASK) SERCOM1 APB Clock Enable */
	#define APBA_TC0       			(1<<14)       	/**< \brief (MCLK_APBAMASK) TC0 APB Clock Enable */
	#define APBA_TC1       			(1<<15)       	/**< \brief (MCLK_APBAMASK) TC1 APB Clock Enable */
											  
	#define APBB_USB       			(1<<0)        	/**< \brief (MCLK_APBBMASK) USB APB Clock Enable */
	#define APBB_DSU       			(1<<1)        	/**< \brief (MCLK_APBBMASK) DSU APB Clock Enable */
	#define APBB_NVMCTRL   			(1<<2)        	/**< \brief (MCLK_APBBMASK) NVMCTRL APB Clock Enable */
	#define APBB_PORT      			(1<<4)        	/**< \brief (MCLK_APBBMASK) PORT APB Clock Enable */
	#define APBB_HMATRIX   			(1<<6)        	/**< \brief (MCLK_APBBMASK) HMATRIX APB Clock Enable */
	#define APBB_EVSYS     			(1<<7)        	/**< \brief (MCLK_APBBMASK) EVSYS APB Clock Enable */
	#define APBB_SERCOM2   			(1<<9)        	/**< \brief (MCLK_APBBMASK) SERCOM2 APB Clock Enable */
	#define APBB_SERCOM3   			(1<<10)       	/**< \brief (MCLK_APBBMASK) SERCOM3 APB Clock Enable */
	#define APBB_TCC0      			(1<<11)       	/**< \brief (MCLK_APBBMASK) TCC0 APB Clock Enable */
	#define APBB_TCC1      			(1<<12)       	/**< \brief (MCLK_APBBMASK) TCC1 APB Clock Enable */
	#define APBB_TC2       			(1<<13)       	/**< \brief (MCLK_APBBMASK) TC2 APB Clock Enable */
	#define APBB_TC3       			(1<<14)       	/**< \brief (MCLK_APBBMASK) TC3 APB Clock Enable */
	#define APBB_RAMECC    			(1<<16)       	/**< \brief (MCLK_APBBMASK) RAMECC APB Clock Enable */

	#define APBC_GMAC      			(1<<2)        	/**< \brief (MCLK_APBCMASK) GMAC APB Clock Enable */
	#define APBC_TCC2      			(1<<3)        	/**< \brief (MCLK_APBCMASK) TCC2 APB Clock Enable */
	#define APBC_TCC3      			(1<<4)        	/**< \brief (MCLK_APBCMASK) TCC3 APB Clock Enable */
	#define APBC_TC4       			(1<<5)        	/**< \brief (MCLK_APBCMASK) TC4 APB Clock Enable */
	#define APBC_TC5       			(1<<6)        	/**< \brief (MCLK_APBCMASK) TC5 APB Clock Enable */
	#define APBC_PDEC      			(1<<7)        	/**< \brief (MCLK_APBCMASK) PDEC APB Clock Enable */
	#define APBC_AC        			(1<<8)        	/**< \brief (MCLK_APBCMASK) AC APB Clock Enable */
	#define APBC_AES       			(1<<9)        	/**< \brief (MCLK_APBCMASK) AES APB Clock Enable */
	#define APBC_TRNG      			(1<<10)       	/**< \brief (MCLK_APBCMASK) TRNG APB Clock Enable */
	#define APBC_ICM       			(1<<11)       	/**< \brief (MCLK_APBCMASK) ICM APB Clock Enable */
	#define APBC_QSPI      			(1<<13)       	/**< \brief (MCLK_APBCMASK) QSPI APB Clock Enable */
	#define APBC_CCL       			(1<<14)       	/**< \brief (MCLK_APBCMASK) CCL APB Clock Enable */

	#define APBD_SERCOM4   			(1<<0)        	/**< \brief (MCLK_APBDMASK) SERCOM4 APB Clock Enable */
	#define APBD_SERCOM5   			(1<<1)        	/**< \brief (MCLK_APBDMASK) SERCOM5 APB Clock Enable */
	#define APBD_SERCOM6   			(1<<2)        	/**< \brief (MCLK_APBDMASK) SERCOM6 APB Clock Enable */
	#define APBD_SERCOM7   			(1<<3)        	/**< \brief (MCLK_APBDMASK) SERCOM7 APB Clock Enable */
	#define APBD_TCC4      			(1<<4)        	/**< \brief (MCLK_APBDMASK) TCC4 APB Clock Enable */
	#define APBD_TC6       			(1<<5)        	/**< \brief (MCLK_APBDMASK) TC6 APB Clock Enable */
	#define APBD_TC7       			(1<<6)        	/**< \brief (MCLK_APBDMASK) TC7 APB Clock Enable */
	#define APBD_ADC0      			(1<<7)        	/**< \brief (MCLK_APBDMASK) ADC0 APB Clock Enable */
	#define APBD_ADC1      			(1<<8)      	/**< \brief (MCLK_APBDMASK) ADC1 APB Clock Enable */
	#define APBD_DAC       			(1<<9)      	/**< \brief (MCLK_APBDMASK) DAC APB Clock Enable */
	#define APBD_I2S       			(1<<10)       	/**< \brief (MCLK_APBDMASK) I2S APB Clock Enable */
	#define APBD_PCC       			(1<<11)       	/**< \brief (MCLK_APBDMASK) PCC APB Clock Enable */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_OSCCTRL
	{
		struct S_DPLL
		{
			RW8		CTRLA;		/**< \brief Offset: 0x00 (R/W  8) DPLL Control A */
			RO8				z_Reserved1[0x3];
			RW32	RATIO;   	/**< \brief Offset: 0x04 (R/W 32) DPLL Ratio Control */
			RW32	CTRLB;   	/**< \brief Offset: 0x08 (R/W 32) DPLL Control B */
			RO32	SYNCBUSY;	/**< \brief Offset: 0x0C (R/  32) DPLL Synchronization Busy */
			RO32	STATUS;		/**< \brief Offset: 0x10 (R/  32) DPLL Status */
		};

		RW8  	EVCTRL;      /**< \brief Offset: 0x00 (R/W  8) Event Control */
		RO8  				z_Reserved1[0x3];
		RW32	INTENCLR;    /**< \brief Offset: 0x04 (R/W 32) Interrupt Enable Clear */
		RW32	INTENSET;    /**< \brief Offset: 0x08 (R/W 32) Interrupt Enable Set */
		RW32	INTFLAG;     /**< \brief Offset: 0x0C (R/W 32) Interrupt Flag Status and Clear */
		RW32	STATUS;      /**< \brief Offset: 0x10 (R/  32) Status */
		RW32	XOSC[2];	/**< \brief Offset: 0x14 (R/W 32) External Multipurpose Crystal Oscillator Control */
		RW8  	DFLLCTRLA;   /**< \brief Offset: 0x1C (R/W  8) DFLL48M Control A */
		RO8  				z_Reserved2[0x3];
		RW8  	DFLLCTRLB;   /**< \brief Offset: 0x20 (R/W  8) DFLL48M Control B */
		RO8  				z_Reserved3[0x3];
		RW32  	DFLLVAL;     /**< \brief Offset: 0x24 (R/W 32) DFLL48M Value */
		RW32  	DFLLMUL;     /**< \brief Offset: 0x28 (R/W 32) DFLL48M Multiplier */
		RW8  	DFLLSYNC;    /**< \brief Offset: 0x2C (R/W  8) DFLL48M Synchronization */
		RO8  				z_Reserved4[0x3];
       
		S_DPLL  DPLL[2];     /**< \brief Offset: 0x30 OscctrlDpll groups [DPLLS_NUM] */
	};


	#define XOSC_ENABLE     		(1<<1)            		/**< \brief (OSCCTRL_XOSCCTRL) Oscillator Enable */
	#define XOSC_XTALEN     		(1<<2)            		/**< \brief (OSCCTRL_XOSCCTRL) Crystal Oscillator Enable */
	#define XOSC_RUNSTDBY   		(1<<6)            		/**< \brief (OSCCTRL_XOSCCTRL) Run in Standby */
	#define XOSC_ONDEMAND   		(1<<7)            		/**< \brief (OSCCTRL_XOSCCTRL) On Demand Control */
	#define XOSC_LOWBUFGAIN			(1<<8)					/**< \brief (OSCCTRL_XOSCCTRL) Low Buffer Gain Enable */
	#define XOSC_IPTAT(value) 		(((value)&3) << 9)		/**< \brief (OSCCTRL_XOSCCTRL) Oscillator Current Reference */
	#define XOSC_IMULT(value) 		(((value)&0xF) << 11)
	#define XOSC_ENALC  			(1<<15)           		/**< \brief (OSCCTRL_XOSCCTRL) Automatic Loop Control Enable */
	#define XOSC_CFDEN  			(1<<16)           		/**< \brief (OSCCTRL_XOSCCTRL) Clock Failure Detector Enable */
	#define XOSC_SWBEN  			(1<<17)           		/**< \brief (OSCCTRL_XOSCCTRL) Xosc Clock Switch Enable */
	#define XOSC_STARTUP(value) 	(((value)&0xF) << 20)
	#define XOSC_CFDPRESC(value)	(((value)&0xF) << 24)   /**< \brief (OSCCTRL_XOSCCTRL) Clock Failure Detector Prescaler */


	#define DFLL_ENABLE 			(1<<1) 					/**< \brief (OSCCTRL_DFLLCTRLA) DFLL Enable */
	#define DFLL_RUNSTDBY 			(1<<6) 					/**< \brief (OSCCTRL_DFLLCTRLA) Run in Standby */
	#define DFLL_ONDEMAND 			(1<<7) 					/**< \brief (OSCCTRL_DFLLCTRLA) On Demand Control */

	#define DFLL_MODE				(1<<0)            		/**< \brief (OSCCTRL_DFLLCTRLB) Operating Mode Selection */
	#define DFLL_STABLE 			(1<<1)            		/**< \brief (OSCCTRL_DFLLCTRLB) Stable DFLL Frequency */
	#define DFLL_LLAW				(1<<2)            		/**< \brief (OSCCTRL_DFLLCTRLB) Lose Lock After Wake */
	#define DFLL_USBCRM 			(1<<3)            		/**< \brief (OSCCTRL_DFLLCTRLB) USB Clock Recovery Mode */
	#define DFLL_CCDIS 				(1<<4)            		/**< \brief (OSCCTRL_DFLLCTRLB) Chill Cycle Disable */
	#define DFLL_QLDIS 				(1<<5)            		/**< \brief (OSCCTRL_DFLLCTRLB) Quick Lock Disable */
	#define DFLL_BPLCKC 			(1<<6)            		/**< \brief (OSCCTRL_DFLLCTRLB) Bypass Coarse Lock */
	#define DFLL_WAITLOCK			(1<<7)            		/**< \brief (OSCCTRL_DFLLCTRLB) Wait Lock */


	#define DFLL_FINE(value)		(((value)&0xFF)<<0)		/**< \brief (SYSCTRL_DFLLVAL) Fine Value */
	#define DFLL_COARSE(value)		(((value)&0x3F)<<10)	/**< \brief (SYSCTRL_DFLLVAL) Coarse Value */
	#define DFLL_DIFF(value)		((value)<<16)			/**< \brief (SYSCTRL_DFLLVAL) Multiplication Ratio Difference */

	#define DFLL_MUL(value)			(((value)&0xFFFF)<<0)	/**< \brief (SYSCTRL_DFLLMUL) DFLL Multiply Factor */
	#define DFLL_FSTEP(value)		(((value)&0xFF)<<16)	/**< \brief (SYSCTRL_DFLLMUL) Fine Maximum Step */
	#define DFLL_CSTEP(value)		(((value))<<26)			/**< \brief (SYSCTRL_DFLLMUL) Coarse Maximum Step */

	#define DFLLSYNC_ENABLE			(1<<1)            		/**< \brief (OSCCTRL_DFLLSYNC) ENABLE Synchronization Busy */
	#define DFLLSYNC_DFLLCTRLB		(1<<2)            		/**< \brief (OSCCTRL_DFLLSYNC) DFLLCTRLB Synchronization Busy */
	#define DFLLSYNC_DFLLVAL 		(1<<3)            		/**< \brief (OSCCTRL_DFLLSYNC) DFLLVAL Synchronization Busy */
	#define DFLLSYNC_DFLLMUL 		(1<<4)            		/**< \brief (OSCCTRL_DFLLSYNC) DFLLMUL Synchronization Busy */


	#define DPLL_ENABLE				(1<<1)    				/**< \brief (SYSCTRL_DPLLCTRLA) DPLL Enable */
	#define DPLL_RUNSTDBY			(1<<6)    				/**< \brief (SYSCTRL_DPLLCTRLA) Run in Standby */
	#define DPLL_ONDEMAND			(1<<7)    				/**< \brief (SYSCTRL_DPLLCTRLA) On Demand Clock Activation */

	#define DPLL_LDR(value)			(((value)&0xFFF)<<0)	/**< \brief (SYSCTRL_DPLLRATIO) Loop Divider Ratio */
	#define DPLL_LDRFRAC(value)		(((value)&0xF)<<16)		/**< \brief (SYSCTRL_DPLLRATIO) Loop Divider Ratio Fractional Part */

	#define DPLL_FILTER(value)		(((value)&0xF)<<0)  	/**< \brief (OSCCTRL_DPLLCTRLB) Proportional Integral Filter Selection */
	#define DPLL_WUF				(1<<4)					/**< \brief (OSCCTRL_DPLLCTRLB) Wake Up Fast */
	#define DPLL_REFCLK_GCLK		(0x0<<5)   				/**< \brief (OSCCTRL_DPLLCTRLB) Dedicated GCLK clock reference */
	#define DPLL_REFCLK_XOSC32		(0x1<<5)   				/**< \brief (OSCCTRL_DPLLCTRLB) XOSC32K clock reference */
	#define DPLL_REFCLK_XOSC0		(0x2<<5)   				/**< \brief (OSCCTRL_DPLLCTRLB) XOSC0 clock reference */
	#define DPLL_REFCLK_XOSC1		(0x3<<5)   				/**< \brief (OSCCTRL_DPLLCTRLB) XOSC1 clock reference */
	#define DPLL_LTIME_DEFAULT		(0x0<<8)   				/**< \brief (OSCCTRL_DPLLCTRLB) No time-out. Automatic lock */
	#define DPLL_LTIME_800US		(0x4<<8)   				/**< \brief (OSCCTRL_DPLLCTRLB) Time-out if no lock within 800us */
	#define DPLL_LTIME_900US		(0x5<<8)   				/**< \brief (OSCCTRL_DPLLCTRLB) Time-out if no lock within 900us */
	#define DPLL_LTIME_1MS			(0x6<<8)   				/**< \brief (OSCCTRL_DPLLCTRLB) Time-out if no lock within 1ms */
	#define DPLL_LTIME_1P1MS		(0x7<<8)   				/**< \brief (OSCCTRL_DPLLCTRLB) Time-out if no lock within 1.1ms */
	#define DPLL_LBYPASS			(1<<11)					/**< \brief (OSCCTRL_DPLLCTRLB) Lock Bypass */
	#define DPLL_DCOFILTER(value)	(((value)&7)<<12)       /**< \brief (OSCCTRL_DPLLCTRLB) Sigma-Delta DCO Filter Selection */
	#define DPLL_DCOEN				(1<<15)					/**< \brief (OSCCTRL_DPLLCTRLB) DCO Filter Enable */
	#define DPLL_DIV(value)			(((value)&0x7FF)<<16)   /**< \brief (OSCCTRL_DPLLCTRLB) Clock Divider */

	#define DPLLSYNCBUSY_ENABLE		(1<<1)            		/**< \brief (OSCCTRL_DPLLSYNCBUSY) DPLL Enable Synchronization Status */
	#define DPLLSYNCBUSY_DPLLRATIO	(1<<2)            		/**< \brief (OSCCTRL_DPLLSYNCBUSY) DPLL Loop Divider Ratio Synchronization Status */

	#define DPLLSTATUS_LOCK			(1<<0)            		/**< \brief (OSCCTRL_DPLLSTATUS) DPLL Lock Status */
	#define DPLLSTATUS_CLKRDY		(1<<1)            		/**< \brief (OSCCTRL_DPLLSTATUS) DPLL Clock Ready */


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_OSC32KCTRL
	{
		RW32	INTENCLR;    /**< \brief Offset: 0x00 (R/W 32) Interrupt Enable Clear */
		RW32	INTENSET;    /**< \brief Offset: 0x04 (R/W 32) Interrupt Enable Set */
		RW32	INTFLAG;     /**< \brief Offset: 0x08 (R/W 32) Interrupt Flag Status and Clear */
		RO32	STATUS;      /**< \brief Offset: 0x0C (R/  32) Power and Clocks Status */
		RW8		RTCCTRL;     /**< \brief Offset: 0x10 (R/W  8) RTC Clock Selection */
		RO8                    z_Reserved1[0x3];
		RW16	XOSC32K;     /**< \brief Offset: 0x14 (R/W 16) 32kHz External Crystal Oscillator (XOSC32K) Control */
		RW8		CFDCTRL;     /**< \brief Offset: 0x16 (R/W  8) Clock Failure Detector Control */
		RW8		EVCTRL;      /**< \brief Offset: 0x17 (R/W  8) Event Control */
		RO8                    z_Reserved2[0x4];
		RW32	OSCULP32K;   /**< \brief Offset: 0x1C (R/W 32) 32kHz Ultra Low Power Internal Oscillator (OSCULP32K) Control */
	};

	#define OSC32_RDY				(1<<0)            /**< \brief (OSC32KCTRL_INTENCLR) XOSC32K Ready Interrupt Enable */
	#define OSC32_FAIL				(1<<2)            /**< \brief (OSC32KCTRL_INTENCLR) XOSC32K Clock Failure Detector Interrupt Enable */

	#define	OSC32_RTCSEL_ULP1K		(0x0)   /**< \brief (OSC32KCTRL_RTCCTRL) 1.024kHz from 32kHz internal ULP oscillator */
	#define	OSC32_RTCSEL_ULP32K 	(0x1)   /**< \brief (OSC32KCTRL_RTCCTRL) 32.768kHz from 32kHz internal ULP oscillator */
	#define	OSC32_RTCSEL_XOSC1K 	(0x4)   /**< \brief (OSC32KCTRL_RTCCTRL) 1.024kHz from 32.768kHz internal oscillator */
	#define	OSC32_RTCSEL_XOSC32K	(0x5)   /**< \brief (OSC32KCTRL_RTCCTRL) 32.768kHz from 32.768kHz external crystal oscillator */

	#define XOSC32K_ENABLE 			(1<<1)            	/**< \brief (OSC32KCTRL_XOSC32K) Oscillator Enable */
	#define XOSC32K_XTALEN 			(1<<2)            	/**< \brief (OSC32KCTRL_XOSC32K) Crystal Oscillator Enable */
	#define XOSC32K_EN32K			(1<<3)            	/**< \brief (OSC32KCTRL_XOSC32K) 32kHz Output Enable */
	#define XOSC32K_EN1K			(1<<4)            	/**< \brief (OSC32KCTRL_XOSC32K) 1kHz Output Enable */
	#define XOSC32K_RUNSTDBY 		(1<<6)            	/**< \brief (OSC32KCTRL_XOSC32K) Run in Standby */
	#define XOSC32K_ONDEMAND 		(1<<7)            	/**< \brief (OSC32KCTRL_XOSC32K) On Demand Control */
	#define XOSC32K_STARTUP(value)	(((value)&7)<<8)	/**< \brief (OSC32KCTRL_XOSC32K) Oscillator Start-Up Time */
	#define XOSC32K_WRTLOCK			(1<<12)				/**< \brief (OSC32KCTRL_XOSC32K) Write Lock */
	#define XOSC32K_CGM_XT   		(0x1<<13)   		/**< \brief (OSC32KCTRL_XOSC32K) Standard mode */
	#define XOSC32K_CGM_HS   		(0x2<<13)   		/**< \brief (OSC32KCTRL_XOSC32K) High Speed mode */

	#define OSC32_CFDEN				(1<<0)            	/**< \brief (OSC32KCTRL_CFDCTRL) Clock Failure Detector Enable */
	#define OSC32_SWBACK			(1<<1)            	/**< \brief (OSC32KCTRL_CFDCTRL) Clock Switch Back */
	#define OSC32_CFDPRESC			(1<<2)            	/**< \brief (OSC32KCTRL_CFDCTRL) Clock Failure Detector Prescaler */

	#define OSC32_CFDEO				(1<<0)            /**< \brief (OSC32KCTRL_EVCTRL) Clock Failure Detector Event Output Enable */

	#define OSCULP32K_EN32K 		(1<<1)            /**< \brief (OSC32KCTRL_OSCULP32K) Enable Out 32k */
	#define OSCULP32K_EN1K			(1<<2)            /**< \brief (OSC32KCTRL_OSCULP32K) Enable Out 1k */
	#define OSCULP32K_CALIB(value)	(((value)&0x3F)<<8)            /**< \brief (OSC32KCTRL_OSCULP32K) Oscillator Calibration */
	#define OSCULP32K_WRTLOCK		(1<<15)           /**< \brief (OSC32KCTRL_OSCULP32K) Write Lock */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_SUPC
	{
		RW32    INTENCLR;    /**< \brief Offset: 0x00 (R/W 32) Interrupt Enable Clear */
		RW32    INTENSET;    /**< \brief Offset: 0x04 (R/W 32) Interrupt Enable Set */
		RW32    INTFLAG;     /**< \brief Offset: 0x08 (R/W 32) Interrupt Flag Status and Clear */
		RO32    STATUS;      /**< \brief Offset: 0x0C (R/  32) Power and Clocks Status */
		RW32    BOD33;       /**< \brief Offset: 0x10 (R/W 32) BOD33 Control */
		RO8              Reserved1[0x4];
		RW32	VREG;        /**< \brief Offset: 0x18 (R/W 32) VREG Control */
		RW32	VREF;        /**< \brief Offset: 0x1C (R/W 32) VREF Control */
		RW32	BBPS;        /**< \brief Offset: 0x20 (R/W 32) Battery Backup Power Switch */
		RW32	BKOUT;       /**< \brief Offset: 0x24 (R/W 32) Backup Output Control */
		RO32	BKIN;        /**< \brief Offset: 0x28 (R/  32) Backup Input Control */
	};

	#define VREG_ENABLE    		(1<<1)      		/**< \brief (SUPC_VREG) Enable */
	#define VREG_SEL       		(1<<2)      		/**< \brief (SUPC_VREG) Voltage Regulator Selection */
	#define VREG_SEL_LDO   		(0x0<<2)			/**< \brief (SUPC_VREG) LDO selection */
	#define VREG_SEL_BUCK  		(0x1<<2)			/**< \brief (SUPC_VREG) Buck selection */
	#define VREG_RUNBKUP   		(1<<7)      		/**< \brief (SUPC_VREG) Run in Backup mode */
	#define VREG_VSEN      		(1<<16)         	/**< \brief (SUPC_VREG) Voltage Scaling Enable */
	#define VREG_VSPER(value)	(((value)&7)<<24)   /**< \brief (SUPC_VREG) Voltage Scaling Period */

	#define VREF_TSEN          (1<<1)        	/**< \brief (SUPC_VREF) Temperature Sensor Output Enable */
	#define VREF_VREFOE        (1<<2)        	/**< \brief (SUPC_VREF) Voltage Reference Output Enable */
	#define VREF_TSSEL         (1<<3)        	/**< \brief (SUPC_VREF) Temperature Sensor Selection */
	#define VREF_RUNSTDBY      (1<<6)        	/**< \brief (SUPC_VREF) Run during Standby */
	#define VREF_ONDEMAND      (1<<7)        	/**< \brief (SUPC_VREF) On Demand Contrl */
	#define	VREF_SEL_1V0       (0x0<<16)   		/**< \brief (SUPC_VREF) 1.0V voltage reference typical value */
	#define	VREF_SEL_1V1       (0x1<<16)   		/**< \brief (SUPC_VREF) 1.1V voltage reference typical value */
	#define	VREF_SEL_1V2       (0x2<<16)   		/**< \brief (SUPC_VREF) 1.2V voltage reference typical value */
	#define	VREF_SEL_1V25      (0x3<<16)   		/**< \brief (SUPC_VREF) 1.25V voltage reference typical value */
	#define	VREF_SEL_2V0       (0x4<<16)   		/**< \brief (SUPC_VREF) 2.0V voltage reference typical value */
	#define	VREF_SEL_2V2       (0x5<<16)   		/**< \brief (SUPC_VREF) 2.2V voltage reference typical value */
	#define	VREF_SEL_2V4       (0x6<<16)   		/**< \brief (SUPC_VREF) 2.4V voltage reference typical value */
	#define	VREF_SEL_2V5       (0x7<<16)   		/**< \brief (SUPC_VREF) 2.5V voltage reference typical value */

	#define BOD33_ENABLE           	(1<<1)               /**< (SUPC_BOD33) Enable Position */
	#define BOD33_ACTION_NONE      	(0x0<<2)             /**< (SUPC_BOD33) No action  */
	#define BOD33_ACTION_RESET     	(0x1<<2)             /**< (SUPC_BOD33) The BOD33 generates a reset  */
	#define BOD33_ACTION_INT       	(0x2<<2)             /**< (SUPC_BOD33) The BOD33 generates an interrupt  */
	#define BOD33_ACTION_BKUP      	(0x3<<2)             /**< (SUPC_BOD33) The BOD33 puts the device in backup sleep mode  */
	#define BOD33_STDBYCFG          (1<<4)               /**< (SUPC_BOD33) Configuration in Standby mode Position */
	#define BOD33_RUNSTDBY          (1<<5)               /**< (SUPC_BOD33) Run in Standby mode Position */
	#define BOD33_RUNHIB            (1<<6)               /**< (SUPC_BOD33) Run in Hibernate mode Position */
	#define BOD33_RUNBKUP           (1<<7)               /**< (SUPC_BOD33) Run in Backup mode Position */
	#define BOD33_HYST(value)       (((value)&15)<<8)
	#define BOD33_PSEL_NODIV        (0x0<<12)             /**< (SUPC_BOD33) Not divided  */
	#define BOD33_PSEL_DIV4         (0x1<<12)             /**< (SUPC_BOD33) Divide clock by 4  */
	#define BOD33_PSEL_DIV8         (0x2<<12)             /**< (SUPC_BOD33) Divide clock by 8  */
	#define BOD33_PSEL_DIV16        (0x3<<12)             /**< (SUPC_BOD33) Divide clock by 16  */
	#define BOD33_PSEL_DIV32        (0x4<<12)             /**< (SUPC_BOD33) Divide clock by 32  */
	#define BOD33_PSEL_DIV64        (0x5<<12)             /**< (SUPC_BOD33) Divide clock by 64  */
	#define BOD33_PSEL_DIV128       (0x6<<12)             /**< (SUPC_BOD33) Divide clock by 128  */
	#define BOD33_PSEL_DIV256       (0x7<<12)             /**< (SUPC_BOD33) Divide clock by 256  */
	#define BOD33_LEVEL(value)    	(((value)&0xFF)<<16)
	#define BOD33_VBATLEVEL(value)	(((value)&0xFF)<<24) 

	#define SUPC_BOD33RDY  			(1<<0)                                               /**< (SUPC_STATUS) BOD33 Ready Position */
	#define SUPC_BOD33DET  			(1<<1)                                               /**< (SUPC_STATUS) BOD33 Detection Position */
	#define SUPC_B33SRDY   			(1<<2)                                               /**< (SUPC_STATUS) BOD33 Synchronization Ready Position */
	#define SUPC_VREGRDY   			(1<<8)                                               /**< (SUPC_STATUS) Voltage Regulator Ready Position */
	#define SUPC_VCORERDY  			(1<<10)                                              /**< (SUPC_STATUS) VDDCORE Ready Position */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_WDT
	{
		RW8    CTRLA;       /**< \brief Offset: 0x0 (R/W  8) Control */
		RW8    CONFIG;      /**< \brief Offset: 0x1 (R/W  8) Configuration */
		RW8    EWCTRL;      /**< \brief Offset: 0x2 (R/W  8) Early Warning Interrupt Control */
		RO8					z__Reserved1[0x1];
		RW8    INTENCLR;    /**< \brief Offset: 0x4 (R/W  8) Interrupt Enable Clear */
		RW8    INTENSET;    /**< \brief Offset: 0x5 (R/W  8) Interrupt Enable Set */
		RW8    INTFLAG;     /**< \brief Offset: 0x6 (R/W  8) Interrupt Flag Status and Clear */
	    RO8					z__Reserved2[0x1];
		RO32	SYNCBUSY;    		/**< \brief Offset: 0x8 (R/  32) Synchronization Busy */
		RW8		CLEAR;       		/**< \brief Offset: 0xC ( /W  8) Clear */

		void Reset() { CLEAR = 0xA5; }
		void Update() { CLEAR = 0xA5; }
	};

	/* ========== Instance parameters for WDT peripheral ========== */
	#define WDT_GCLK_ID                 3

	#define	WDT_ENABLE         		(1<<1)            /**< \brief (WDT_CTRLA) Enable */
	#define	WDT_WEN            		(1<<2)            /**< \brief (WDT_CTRLA) Watchdog Timer Window Mode Enable */
	#define	WDT_ALWAYSON       		(1<<7)            /**< \brief (WDT_CTRLA) Always-On */
	#define	WDT_EW 					(1<<0)            /**< \brief (WDT_INTFLAG) Early Warning */
	#define	WDT_SYNCBUSY_ALWAYSON	(1<<3)            /**< \brief (WDT_SYNCBUSY) Always-On Synchronization Busy */
	#define	WDT_SYNCBUSY_CLEAR   	(1<<4)            /**< \brief (WDT_SYNCBUSY) Clear Synchronization Busy */

	#define	WDT_PER_CYC8          (0x0)   /**< \brief (WDT_CONFIG) 8 clock cycles */
	#define	WDT_PER_CYC16         (0x1)   /**< \brief (WDT_CONFIG) 16 clock cycles */
	#define	WDT_PER_CYC32         (0x2)   /**< \brief (WDT_CONFIG) 32 clock cycles */
	#define	WDT_PER_CYC64         (0x3)   /**< \brief (WDT_CONFIG) 64 clock cycles */
	#define	WDT_PER_CYC128        (0x4)   /**< \brief (WDT_CONFIG) 128 clock cycles */
	#define	WDT_PER_CYC256        (0x5)   /**< \brief (WDT_CONFIG) 256 clock cycles */
	#define	WDT_PER_CYC512        (0x6)   /**< \brief (WDT_CONFIG) 512 clock cycles */
	#define	WDT_PER_CYC1024       (0x7)   /**< \brief (WDT_CONFIG) 1024 clock cycles */
	#define	WDT_PER_CYC2048       (0x8)   /**< \brief (WDT_CONFIG) 2048 clock cycles */
	#define	WDT_PER_CYC4096       (0x9)   /**< \brief (WDT_CONFIG) 4096 clock cycles */
	#define	WDT_PER_CYC8192       (0xA)   /**< \brief (WDT_CONFIG) 8192 clock cycles */
	#define	WDT_PER_CYC16384      (0xB)   /**< \brief (WDT_CONFIG) 16384 clock cycles */

	#define	WDT_WINDOW_CYC8       (0x0<<4)   /**< \brief (WDT_CONFIG) 8 clock cycles */
	#define	WDT_WINDOW_CYC16      (0x1<<4)   /**< \brief (WDT_CONFIG) 16 clock cycles */
	#define	WDT_WINDOW_CYC32      (0x2<<4)   /**< \brief (WDT_CONFIG) 32 clock cycles */
	#define	WDT_WINDOW_CYC64      (0x3<<4)   /**< \brief (WDT_CONFIG) 64 clock cycles */
	#define	WDT_WINDOW_CYC128     (0x4<<4)   /**< \brief (WDT_CONFIG) 128 clock cycles */
	#define	WDT_WINDOW_CYC256     (0x5<<4)   /**< \brief (WDT_CONFIG) 256 clock cycles */
	#define	WDT_WINDOW_CYC512     (0x6<<4)   /**< \brief (WDT_CONFIG) 512 clock cycles */
	#define	WDT_WINDOW_CYC1024    (0x7<<4)   /**< \brief (WDT_CONFIG) 1024 clock cycles */
	#define	WDT_WINDOW_CYC2048    (0x8<<4)   /**< \brief (WDT_CONFIG) 2048 clock cycles */
	#define	WDT_WINDOW_CYC4096    (0x9<<4)   /**< \brief (WDT_CONFIG) 4096 clock cycles */
	#define	WDT_WINDOW_CYC8192    (0xA<<4)   /**< \brief (WDT_CONFIG) 8192 clock cycles */
	#define	WDT_WINDOW_CYC16384   (0xB<<4)   /**< \brief (WDT_CONFIG) 16384 clock cycles */

	#define	WDT_EWOFFSET_CYC8     (0x0)   /**< \brief (WDT_EWCTRL) 8 clock cycles */
	#define	WDT_EWOFFSET_CYC16    (0x1)   /**< \brief (WDT_EWCTRL) 16 clock cycles */
	#define	WDT_EWOFFSET_CYC32    (0x2)   /**< \brief (WDT_EWCTRL) 32 clock cycles */
	#define	WDT_EWOFFSET_CYC64    (0x3)   /**< \brief (WDT_EWCTRL) 64 clock cycles */
	#define	WDT_EWOFFSET_CYC128   (0x4)   /**< \brief (WDT_EWCTRL) 128 clock cycles */
	#define	WDT_EWOFFSET_CYC256   (0x5)   /**< \brief (WDT_EWCTRL) 256 clock cycles */
	#define	WDT_EWOFFSET_CYC512   (0x6)   /**< \brief (WDT_EWCTRL) 512 clock cycles */
	#define	WDT_EWOFFSET_CYC1024  (0x7)   /**< \brief (WDT_EWCTRL) 1024 clock cycles */
	#define	WDT_EWOFFSET_CYC2048  (0x8)   /**< \brief (WDT_EWCTRL) 2048 clock cycles */
	#define	WDT_EWOFFSET_CYC4096  (0x9)   /**< \brief (WDT_EWCTRL) 4096 clock cycles */
	#define	WDT_EWOFFSET_CYC8192  (0xA)   /**< \brief (WDT_EWCTRL) 8192 clock cycles */
	#define	WDT_EWOFFSET_CYC16384 (0xB)   /**< \brief (WDT_EWCTRL) 16384 clock cycles */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define	PORT_PMUXEN			(1<<16)
	#define	PORT_INEN			(1<<17)
	#define	PORT_PULLEN       	(1<<18)
	#define	PORT_DRVSTR       	(1<<22)
	#define PORT_PMUX(value)	(((value)&15)<<24)
	#define PORT_PMUX_A			(0<<24)
	#define PORT_PMUX_B			(1<<24)
	#define PORT_PMUX_C			(2<<24)
	#define PORT_PMUX_D			(3<<24)
	#define PORT_PMUX_E			(4<<24)
	#define PORT_PMUX_F			(5<<24)
	#define PORT_PMUX_G			(6<<24)
	#define PORT_PMUX_H			(7<<24)
	#define PORT_PMUX_I			(8<<24)
	#define PORT_PMUX_J			(9<<24)
	#define PORT_PMUX_K			(10<<24)
	#define PORT_PMUX_L			(11<<24)
	#define PORT_PMUX_M			(12<<24)
	#define PORT_PMUX_N			(13<<24)
	#define	PORT_WRPMUX       	(1<<28)
	#define	PORT_WRPINCFG      	(1<<30)
	#define	PORT_HWSEL_LO      	(0<<31)
	#define	PORT_HWSEL_HI      	(1UL<<31)

	struct S_PORT
	{
		RW32	DIR;         		/**< \brief Offset: 0x00 (R/W 32) Data Direction */
		RW32	DIRCLR;      		/**< \brief Offset: 0x04 (R/W 32) Data Direction Clear */
		RW32    DIRSET;      		/**< \brief Offset: 0x08 (R/W 32) Data Direction Set */
		RW32    DIRTGL;      		/**< \brief Offset: 0x0C (R/W 32) Data Direction Toggle */
		RW32    OUT;         		/**< \brief Offset: 0x10 (R/W 32) Data Output Value */
		RW32    OUTCLR;      		/**< \brief Offset: 0x14 (R/W 32) Data Output Value Clear */
		RW32    OUTSET;      		/**< \brief Offset: 0x18 (R/W 32) Data Output Value Set */
		RW32    OUTTGL;      		/**< \brief Offset: 0x1C (R/W 32) Data Output Value Toggle */
		RO32    IN;          		/**< \brief Offset: 0x20 (R/  32) Data Input Value */
		RW32    CTRL;        		/**< \brief Offset: 0x24 (R/W 32) Control */
		RW32    WRCONFIG;    		/**< \brief Offset: 0x28 ( /W 32) Write Configuration */

		union
		{
			RW32    R32;				/**< \brief Offset: 0x2C (R/W 32) Event Input Control */
			RW8		EV[4];	
		} EVCTRL;

		RW8     PMUX[16];    		/**< \brief Offset: 0x30 (R/W  8) Peripheral Multiplexing n */
		RW8     PINCFG[32];  		/**< \brief Offset: 0x40 (R/W  8) Pin Configuration n */

		RO8		z__Reserved2[0x20];

		void 	SET(u32 m) 			{ OUTSET = m; }
		void 	CLR(u32 m) 			{ OUTCLR = m; }
		void 	NOT(u32 m) 			{ OUTTGL = m; }
		void 	WBIT(u32 m, bool c) { if (c) SET(m); else CLR(m); }
		void 	BSET(u16 b) 		{ OUTSET = 1 << b; }
		void 	BCLR(u16 b) 		{ OUTCLR = 1 << b; }
		void 	BTGL(u16 b) 		{ OUTTGL = 1 << b; }

		void	OUT8(byte v)		{ *((byte*)&OUT) = v; }

		bool 	TBSET(u16 b) 		{ return IN & (1<<b); }
		bool 	TBCLR(u16 b) 		{ return (IN & (1<<b)) == 0; }

		//void 	SetPinMux(byte pinnum, byte value) { byte sh = (pinnum&1)<<2; pinnum >>= 1; PMUX[pinnum] = (PMUX[pinnum] & ~(0xF<<sh)) | (value<<sh); };
		
		void 	SetWRCONFIG(u32 mask, u32 mux) 
		{ 
			u32 t = (mux & 0xFFFF0000); 
			WRCONFIG = t | (mask & 0xFFFF); 
			WRCONFIG = t | (mask >> 16) | PORT_HWSEL_HI; };
	};										
											
	typedef S_PORT S_PIOA, S_PIOB, S_PIOC;

	#define PORT_PID0(value)	(((value)&0x1F)<<0)	/**< \brief (PORT_EVCTRL) PORT Event Pin Identifier 0 */
	#define	PORT_EVACT0_OUT   	(0x0<<5)			/**< \brief (PORT_EVCTRL) Event output to pin */
	#define	PORT_EVACT0_SET   	(0x1<<5)			/**< \brief (PORT_EVCTRL) Set output register of pin on event */
	#define	PORT_EVACT0_CLR   	(0x2<<5)			/**< \brief (PORT_EVCTRL) Clear output register of pin on event */
	#define	PORT_EVACT0_TGL   	(0x3<<5)			/**< \brief (PORT_EVCTRL) Toggle output register of pin on event */
	#define PORT_PORTEI0 		(1<<7)				/**< \brief (PORT_EVCTRL) PORT Event Input Enable 0 */

	#define PORT_PID1(value)	(((value)&0x1F)<<8)	/**< \brief (PORT_EVCTRL) PORT Event Pin Identifier 1 */
	#define	PORT_EVACT1_OUT   	(0x0<<13)			/**< \brief (PORT_EVCTRL) Event output to pin */
	#define	PORT_EVACT1_SET   	(0x1<<13)			/**< \brief (PORT_EVCTRL) Set output register of pin on event */
	#define	PORT_EVACT1_CLR   	(0x2<<13)			/**< \brief (PORT_EVCTRL) Clear output register of pin on event */
	#define	PORT_EVACT1_TGL   	(0x3<<13)			/**< \brief (PORT_EVCTRL) Toggle output register of pin on event */
	#define PORT_PORTEI1 		(1<<15)				/**< \brief (PORT_EVCTRL) PORT Event Input Enable 1 */

	#define PORT_PID2(value)	(((value)&0x1F)<<16)/**< \brief (PORT_EVCTRL) PORT Event Pin Identifier 2 */
	#define	PORT_EVACT2_OUT   	(0x0<<21)			/**< \brief (PORT_EVCTRL) Event output to pin */
	#define	PORT_EVACT2_SET   	(0x1<<21)			/**< \brief (PORT_EVCTRL) Set output register of pin on event */
	#define	PORT_EVACT2_CLR   	(0x2<<21)			/**< \brief (PORT_EVCTRL) Clear output register of pin on event */
	#define	PORT_EVACT2_TGL   	(0x3<<21)			/**< \brief (PORT_EVCTRL) Toggle output register of pin on event */
	#define PORT_PORTEI2 		(1<<23)				/**< \brief (PORT_EVCTRL) PORT Event Input Enable 2 */

	#define PORT_PID3(value)	(((value)&0x1F)<<24)/**< \brief (PORT_EVCTRL) PORT Event Pin Identifier 3 */
	#define	PORT_EVACT3_OUT   	(0x0<<29)			/**< \brief (PORT_EVCTRL) Event output to pin */
	#define	PORT_EVACT3_SET   	(0x1<<29)			/**< \brief (PORT_EVCTRL) Set output register of pin on event */
	#define	PORT_EVACT3_CLR   	(0x2<<29)			/**< \brief (PORT_EVCTRL) Clear output register of pin on event */
	#define	PORT_EVACT3_TGL   	(0x3<<29)			/**< \brief (PORT_EVCTRL) Toggle output register of pin on event */
	#define PORT_PORTEI3 		(1<<31)				/**< \brief (PORT_EVCTRL) PORT Event Input Enable 3 */

	#define PORT_PID(value)		(((value)&0x1F)<<0)	/**< \brief (PORT_EVCTRL) PORT Event Pin Identifier 0 */
	#define	PORT_EVACT_OUT   	(0x0<<5)			/**< \brief (PORT_EVCTRL) Event output to pin */
	#define	PORT_EVACT_SET   	(0x1<<5)			/**< \brief (PORT_EVCTRL) Set output register of pin on event */
	#define	PORT_EVACT_CLR   	(0x2<<5)			/**< \brief (PORT_EVCTRL) Clear output register of pin on event */
	#define	PORT_EVACT_TGL   	(0x3<<5)			/**< \brief (PORT_EVCTRL) Toggle output register of pin on event */
	#define PORT_PORTEI 		(1<<7)				/**< \brief (PORT_EVCTRL) PORT Event Input Enable 0 */

	#define	PINGFG_PMUXEN       (1<<0)
	#define	PINGFG_INEN			(1<<1)
	#define	PINGFG_PULLEN       (1<<2)
	#define	PINGFG_DRVSTR       (1<<6)
											
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_NVMCTRL
	{
		RW16    CTRLA;			/**< \brief Offset: 0x00 (R/W 16) Control A */

		RO8		z__Reserved1[0x2];

		RW16    CTRLB;			/**< \brief Offset: 0x04 ( /W 16) Control B */

		RO8     z__Reserved2[0x2];

		RO32    PARAM;			/**< \brief Offset: 0x08 (R/  32) NVM Parameter */
		RW16    INTENCLR;		/**< \brief Offset: 0x0C (R/W 16) Interrupt Enable Clear */
		RW16    INTENSET;		/**< \brief Offset: 0x0E (R/W 16) Interrupt Enable Set */
		RW16    INTFLAG;		/**< \brief Offset: 0x10 (R/W 16) Interrupt Flag Status and Clear */
		RO16    STATUS;			/**< \brief Offset: 0x12 (R/  16) Status */
		RW32    ADDR;			/**< \brief Offset: 0x14 (R/W 32) Address */
		RO32    RUNLOCK;		/**< \brief Offset: 0x18 (R/  32) Lock Section */
		RO32    PBLDATA[2];		/**< \brief Offset: 0x1C (R/  32) Page Buffer Load Data x */
		RO32    ECCERR;			/**< \brief Offset: 0x24 (R/  32) ECC Error Status Register */
		RW8     DBGCTRL;		/**< \brief Offset: 0x28 (R/W  8) Debug Control */

		RO8   	z__Reserved3[0x1];

		RW8   	SEECFG;			/**< \brief Offset: 0x2A (R/W  8) SmartEEPROM Configuration Register */

		RO8   	z__Reserved4[0x1];

		RO32  	SEESTAT;		/**< \brief Offset: 0x2C (R/  32) SmartEEPROM Status Register */
	};

	#define NVMCTRL_RWS(value)  				(((value)&0xF) << 1)	/**< \brief (NVMCTRL_CTRLB) NVM Read Wait States */
	#define NVMCTRL_RWS_SINGLE  				(0 << 1)
	#define NVMCTRL_RWS_HALF    				(1 << 1)
	#define NVMCTRL_RWS_DUAL    				(2 << 1)
	#define NVMCTRL_MANW						(1<<7)					/**< \brief (NVMCTRL_CTRLB) Manual Write */
	#define NVMCTRL_SLEEPPRM(value)				(((value)&3)<<8)        /**< \brief (NVMCTRL_CTRLB) Power Reduction Mode during Sleep */
	#define NVMCTRL_SLEEPPRM_WAKEONACCESS		(0 << 8)
	#define NVMCTRL_SLEEPPRM_WAKEUPINSTANT		(1 << 8)
	#define NVMCTRL_SLEEPPRM_DISABLED			(3 << 8)
	#define NVMCTRL_READMODE_Pos(value)			(((value)&3) << 16)     /**< \brief (NVMCTRL_CTRLB) NVMCTRL Read Mode */
	#define NVMCTRL_READMODE_NO_MISS_PENALTY	(0 << 16)
	#define NVMCTRL_READMODE_LOW_POWER			(1 << 16)
	#define NVMCTRL_READMODE_DETERMINISTIC		(2 << 16)
	#define NVMCTRL_CACHEDIS					(1<<18)					/**< \brief (NVMCTRL_CTRLB) Cache Disable */

	#define NVMCTRL_READY    					(1 << 0)            	/**< \brief (NVMCTRL_STATUS) Ready to accept a command */
	#define NVMCTRL_PRM      					(1 << 1)            	/**< \brief (NVMCTRL_STATUS) Power Reduction Mode */
	#define NVMCTRL_LOAD     					(1 << 2)            	/**< \brief (NVMCTRL_STATUS) NVM Page Buffer Active Loading */
	#define NVMCTRL_SUSP     					(1 << 3)            	/**< \brief (NVMCTRL_STATUS) NVM Write Or Erase Operation Is Suspended */
	#define NVMCTRL_AFIRST   					(1 << 4)            	/**< \brief (NVMCTRL_STATUS) BANKA First */
	#define NVMCTRL_BPDIS    					(1 << 5)            	/**< \brief (NVMCTRL_STATUS) Boot Loader Protection Disable */

	#define NVMCTRL_CMDEX_KEY					(0xA5<<8)				/**< \brief (NVMCTRL_CTRLB) Execution Key */

	#define   NVMCTRL_CMD_EP        	(NVMCTRL_CMDEX_KEY|0x0)   		/**< \brief (NVMCTRL_CTRLB) Erase Page - Only supported in the USER and AUX pages. */
	#define   NVMCTRL_CMD_EB        	(NVMCTRL_CMDEX_KEY|0x1)   		/**< \brief (NVMCTRL_CTRLB) Erase Block - Erases the block addressed by the ADDR register, not supported in the user page */
	#define   NVMCTRL_CMD_WP        	(NVMCTRL_CMDEX_KEY|0x3)   		/**< \brief (NVMCTRL_CTRLB) Write Page - Writes the contents of the page buffer to the page addressed by the ADDR register, not supported in the user page */
	#define   NVMCTRL_CMD_WQW       	(NVMCTRL_CMDEX_KEY|0x4)   		/**< \brief (NVMCTRL_CTRLB) Write Quad Word - Writes a 128-bit word at the location addressed by the ADDR register. */
	#define   NVMCTRL_CMD_SWRST     	(NVMCTRL_CMDEX_KEY|0x10)   		/**< \brief (NVMCTRL_CTRLB) Software Reset - Power-Cycle the NVM memory and replay the device automatic calibration procedure and resets the module configuration registers */
	#define   NVMCTRL_CMD_LR        	(NVMCTRL_CMDEX_KEY|0x11)   		/**< \brief (NVMCTRL_CTRLB) Lock Region - Locks the region containing the address location in the ADDR register. */
	#define   NVMCTRL_CMD_UR        	(NVMCTRL_CMDEX_KEY|0x12)   		/**< \brief (NVMCTRL_CTRLB) Unlock Region - Unlocks the region containing the address location in the ADDR register. */
	#define   NVMCTRL_CMD_SPRM      	(NVMCTRL_CMDEX_KEY|0x13)   		/**< \brief (NVMCTRL_CTRLB) Sets the power reduction mode. */
	#define   NVMCTRL_CMD_CPRM      	(NVMCTRL_CMDEX_KEY|0x14)   		/**< \brief (NVMCTRL_CTRLB) Clears the power reduction mode. */
	#define   NVMCTRL_CMD_PBC       	(NVMCTRL_CMDEX_KEY|0x15)   		/**< \brief (NVMCTRL_CTRLB) Page Buffer Clear - Clears the page buffer. */
	#define   NVMCTRL_CMD_SSB       	(NVMCTRL_CMDEX_KEY|0x16)   		/**< \brief (NVMCTRL_CTRLB) Set Security Bit */
	#define   NVMCTRL_CMD_BKSWRST   	(NVMCTRL_CMDEX_KEY|0x17)   		/**< \brief (NVMCTRL_CTRLB) Bank swap and system reset, if SMEE is used also reallocate SMEE data into the opposite BANK */
	#define   NVMCTRL_CMD_CELCK     	(NVMCTRL_CMDEX_KEY|0x18)   		/**< \brief (NVMCTRL_CTRLB) Chip Erase Lock - DSU.CE command is not available */
	#define   NVMCTRL_CMD_CEULCK    	(NVMCTRL_CMDEX_KEY|0x19)   		/**< \brief (NVMCTRL_CTRLB) Chip Erase Unlock - DSU.CE command is available */
	#define   NVMCTRL_CMD_SBPDIS    	(NVMCTRL_CMDEX_KEY|0x1A)   		/**< \brief (NVMCTRL_CTRLB) Sets STATUS.BPDIS, Boot loader protection is discarded until CBPDIS is issued or next start-up sequence */
	#define   NVMCTRL_CMD_CBPDIS    	(NVMCTRL_CMDEX_KEY|0x1B)   		/**< \brief (NVMCTRL_CTRLB) Clears STATUS.BPDIS, Boot loader protection is not discarded */
	#define   NVMCTRL_CMD_ASEES0    	(NVMCTRL_CMDEX_KEY|0x30)   		/**< \brief (NVMCTRL_CTRLB) Activate SmartEEPROM Sector 0, deactivate Sector 1 */
	#define   NVMCTRL_CMD_ASEES1    	(NVMCTRL_CMDEX_KEY|0x31)   		/**< \brief (NVMCTRL_CTRLB) Activate SmartEEPROM Sector 1, deactivate Sector 0 */
	#define   NVMCTRL_CMD_SEERALOC  	(NVMCTRL_CMDEX_KEY|0x32)   		/**< \brief (NVMCTRL_CTRLB) Starts SmartEEPROM sector reallocation algorithm */
	#define   NVMCTRL_CMD_SEEFLUSH  	(NVMCTRL_CMDEX_KEY|0x33)   		/**< \brief (NVMCTRL_CTRLB) Flush SMEE data when in buffered mode */
	#define   NVMCTRL_CMD_LSEE      	(NVMCTRL_CMDEX_KEY|0x34)   		/**< \brief (NVMCTRL_CTRLB) Lock access to SmartEEPROM data from any mean */
	#define   NVMCTRL_CMD_USEE      	(NVMCTRL_CMDEX_KEY|0x35)   		/**< \brief (NVMCTRL_CTRLB) Unlock access to SmartEEPROM data */
	#define   NVMCTRL_CMD_LSEER     	(NVMCTRL_CMDEX_KEY|0x36)   		/**< \brief (NVMCTRL_CTRLB) Lock access to the SmartEEPROM Register Address Space (above 64KB) */
	#define   NVMCTRL_CMD_USEER     	(NVMCTRL_CMDEX_KEY|0x37)   		/**< \brief (NVMCTRL_CTRLB) Unlock access to the SmartEEPROM Register Address Space (above 64KB) */

	#define NVMCTRL_INTFLAG_DONE    	(1 << 0)    					/**< \brief (NVMCTRL_INTFLAG) Command Done */
	#define NVMCTRL_INTFLAG_ADDRE   	(1 << 1)    					/**< \brief (NVMCTRL_INTFLAG) Address Error */
	#define NVMCTRL_INTFLAG_PROGE   	(1 << 2)    					/**< \brief (NVMCTRL_INTFLAG) Programming Error */
	#define NVMCTRL_INTFLAG_LOCKE   	(1 << 3)    					/**< \brief (NVMCTRL_INTFLAG) Lock Error */
	#define NVMCTRL_INTFLAG_ECCSE   	(1 << 4)    					/**< \brief (NVMCTRL_INTFLAG) ECC Single Error */
	#define NVMCTRL_INTFLAG_ECCDE   	(1 << 5)    					/**< \brief (NVMCTRL_INTFLAG) ECC Dual Error */
	#define NVMCTRL_INTFLAG_NVME    	(1 << 6)       					/**< \brief (NVMCTRL_INTFLAG) NVM Error */
	#define NVMCTRL_INTFLAG_SUSP    	(1 << 7)       					/**< \brief (NVMCTRL_INTFLAG) Suspended Write Or Erase Operation */
	#define NVMCTRL_INTFLAG_SEESFULL	(1 << 8)       					/**< \brief (NVMCTRL_INTFLAG) Active SEES Full */
	#define NVMCTRL_INTFLAG_SEESOVF		(1 << 9)       					/**< \brief (NVMCTRL_INTFLAG) Active SEES Overflow */
	#define NVMCTRL_INTFLAG_SEEWRC		(1 << 10)						/**< \brief (NVMCTRL_INTFLAG) SEE Write Completed */
											 
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_RTC
	{
		RW16	CTRLA;       /**< \brief Offset: 0x00 (R/W 16) MODE0 Control A */
		RW16	CTRLB;       /**< \brief Offset: 0x02 (R/W 16) MODE0 Control B */
		RW32	EVCTRL;      /**< \brief Offset: 0x04 (R/W 32) MODE0 Event Control */
		RW16	INTENCLR;    /**< \brief Offset: 0x08 (R/W 16) MODE0 Interrupt Enable Clear */
		RW16	INTENSET;    /**< \brief Offset: 0x0A (R/W 16) MODE0 Interrupt Enable Set */
		RW16	INTFLAG;     /**< \brief Offset: 0x0C (R/W 16) MODE0 Interrupt Flag Status and Clear */
		RW8		DBGCTRL;     /**< \brief Offset: 0x0E (R/W  8) Debug Control */
		RO8                    z__Reserved1[0x1];
		RO32	SYNCBUSY;    /**< \brief Offset: 0x10 (R/  32) MODE0 Synchronization Busy Status */
		RW8     FREQCORR;    /**< \brief Offset: 0x14 (R/W  8) Frequency Correction */
		RO8                    z__Reserved2[0x3];
		RW32    COUNT;       /**< \brief Offset: 0x18 (R/W 32) MODE0 Counter Value */
		RO8                    z__Reserved3[0x4];
		RW32    COMP[2];     /**< \brief Offset: 0x20 (R/W 32) MODE0 Compare n Value */
		RO8                    z__Reserved4[0x18];
		RW32    GP[4];       /**< \brief Offset: 0x40 (R/W 32) General Purpose */
		RO8                    z__Reserved5[0x10];
		RW32    TAMPCTRL;    /**< \brief Offset: 0x60 (R/W 32) Tamper Control */
		RO32	TIMESTAMP;   /**< \brief Offset: 0x64 (R/  32) MODE0 Timestamp */
		RW32    TAMPID;      /**< \brief Offset: 0x68 (R/W 32) Tamper ID */
		RO8                    z__Reserved6[0x14];
		RW32    BKUP[8];     /**< \brief Offset: 0x80 (R/W 32) Backup */
	};

	#define RTC_SWRST   		(1<<0)    	/**< \brief (RTC_MODE0_CTRLA) Software Reset */
	#define RTC_ENABLE  		(1<<1)    	/**< \brief (RTC_MODE0_CTRLA) Enable */
	#define RTC_MODE_COUNT32 	(0x0<<2)   	/**< \brief (RTC_MODE0_CTRLA) Mode 0: 32-bit Counter */
	#define RTC_MODE_COUNT16 	(0x1<<2)   	/**< \brief (RTC_MODE0_CTRLA) Mode 1: 16-bit Counter */
	#define RTC_MODE_CLOCK		(0x2<<2)   	/**< \brief (RTC_MODE0_CTRLA) Mode 2: Clock/Calendar */
	#define RTC_MATCHCLR		(1<<7)      /**< \brief (RTC_MODE0_CTRLA) Clear on Match */
	#define RTC_PRESC_OFF		(0x0<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/1 */
	#define RTC_PRESC_DIV1 		(0x1<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/1 */
	#define RTC_PRESC_DIV2 		(0x2<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/2 */
	#define RTC_PRESC_DIV4 		(0x3<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/4 */
	#define RTC_PRESC_DIV8 		(0x4<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/8 */
	#define RTC_PRESC_DIV16 	(0x5<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/16 */
	#define RTC_PRESC_DIV32 	(0x6<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/32 */
	#define RTC_PRESC_DIV64 	(0x7<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/64 */
	#define RTC_PRESC_DIV128 	(0x8<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/128 */
	#define RTC_PRESC_DIV256 	(0x9<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/256 */
	#define RTC_PRESC_DIV512 	(0xA<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/512 */
	#define RTC_PRESC_DIV1024	(0xB<<8)	/**< \brief (RTC_MODE0_CTRLA) CLK_RTC_CNT = GCLK_RTC/1024 */
	#define RTC_BKTRST  		(1<<13) 	/**< \brief (RTC_MODE0_CTRLA) BKUP Registers Reset On Tamper Enable */
	#define RTC_GPTRST			(1<<14) 	/**< \brief (RTC_MODE0_CTRLA) GP Registers Reset On Tamper Enable */
	#define RTC_COUNTSYNC		(1<<15) 	/**< \brief (RTC_MODE0_CTRLA) Count Read Synchronization Enable */


//	#define RTC_SWRST			(1<<0)    	/**< \brief (RTC_MODE0_SYNCBUSY) Software Reset Busy */
//	#define RTC_ENABLE			(1<<1)    	/**< \brief (RTC_MODE0_SYNCBUSY) Enable Bit Busy */
	#define RTC_FREQCORR		(1<<2)    	/**< \brief (RTC_MODE0_SYNCBUSY) FREQCORR Register Busy */
	#define RTC_COUNT 			(1<<3)    	/**< \brief (RTC_MODE0_SYNCBUSY) COUNT Register Busy */
	#define RTC_COMP0 			(1<<5)    	/**< \brief (RTC_MODE0_SYNCBUSY) COMP 0 Register Busy */
	#define RTC_COMP1 			(1<<6)    	/**< \brief (RTC_MODE0_SYNCBUSY) COMP 1 Register Busy */
//	#define RTC_COUNTSYNC		(1<<15)   	/**< \brief (RTC_MODE0_SYNCBUSY) Count Synchronization Enable Bit Busy */
	#define RTC_GP0  			(1<<16)   	/**< \brief (RTC_MODE0_SYNCBUSY) General Purpose 0 Register Busy */
	#define RTC_GP1  			(1<<17)   	/**< \brief (RTC_MODE0_SYNCBUSY) General Purpose 1 Register Busy */
	#define RTC_GP2  			(1<<18)   	/**< \brief (RTC_MODE0_SYNCBUSY) General Purpose 2 Register Busy */
	#define RTC_GP3  			(1<<19)   	/**< \brief (RTC_MODE0_SYNCBUSY) General Purpose 3 Register Busy */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_I2C
	{
		RW32    CTRLA;       		/**< \brief Offset: 0x00 (R/W 32) I2CM Control A */
		RW32    CTRLB;       		/**< \brief Offset: 0x04 (R/W 32) I2CM Control B */
		RO8     z__Reserved1[0x4];
		RW32    BAUD;				/**< \brief Offset: 0x0C (R/W 32) I2CM Baud Rate */
		RO8     z__Reserved2[0x4];
		RW8		INTENCLR;			/**< \brief Offset: 0x14 (R/W  8) I2CM Interrupt Enable Clear */
		RO8     z__Reserved3[0x1];
		RW8		INTENSET;			/**< \brief Offset: 0x16 (R/W  8) I2CM Interrupt Enable Set */
		RO8     z__Reserved4[0x1];
		RW8		INTFLAG;			/**< \brief Offset: 0x18 (R/W  8) I2CM Interrupt Flag Status and Clear */
		RO8     z__Reserved5[0x1];

		struct
		{
			RW16 BUSERR		:1;     /*!< bit:      0  Bus Error                          */
			RW16 ARBLOST	:1;     /*!< bit:      1  Arbitration Lost                   */
			RW16 RXNACK		:1;     /*!< bit:      2  Received Not Acknowledge           */
			RW16			:1;     /*!< bit:      3  Reserved                           */
			RW16 BUSSTATE	:2;     /*!< bit:  4.. 5  Bus State                          */
			RW16 LOWTOUT	:1;     /*!< bit:      6  SCL Low Timeout                    */
			RW16 CLKHOLD	:1;     /*!< bit:      7  Clock Hold                         */
			RW16 MEXTTOUT	:1;     /*!< bit:      8  Master SCL Low Extend Timeout      */
			RW16 SEXTTOUT	:1;     /*!< bit:      9  Slave SCL Low Extend Timeout       */
			RW16 LENERR		:1;     /*!< bit:     10  Length Error                       */
			//u16				:5;     /*!< bit: 11..15  Reserved                           */

			operator u16() { return *((u16*)this); }
			u16 operator=(u16 v) { return *((u16*)this) = v; }

			u16 Bits() { return *((u16*)this); } 

		} STATUS;	//RW16	STATUS;	/**< \brief Offset: 0x1A (R/W 16) I2CM Status */

		RO32	SYNCBUSY;    		/**< \brief Offset: 0x1C (R/  32) I2CM Syncbusy */
		RO8		z__Reserved6[0x4];
		RW32    ADDR;        		/**< \brief Offset: 0x24 (R/W 32) I2CM Address */
		RW8     DATA;        		/**< \brief Offset: 0x28 (R/W  8) I2CM Data */
		RO8     z__Reserved7[0x7];
		RW8		DBGCTRL;			/**< \brief Offset: 0x30 (R/W  8) I2CM Debug Control */
	};

	typedef S_I2C S_I2C0, S_I2C1, S_I2C2, S_I2C3, S_I2C4, S_I2C5, S_I2C6, S_I2C7;

	#define BUSSTATE_UNKNOWN	0
	#define BUSSTATE_IDLE		1
	#define BUSSTATE_OWNER		2
	#define BUSSTATE_BUSY		3

	#define	SERCOM_MODE_USART_EXT_CLK 	(0x0<<2)   /**< \brief USART mode with external clock */
	#define	SERCOM_MODE_USART_INT_CLK 	(0x1<<2)   /**< \brief USART mode with internal clock */
	#define	SERCOM_MODE_SPI_SLAVE		(0x2<<2)   /**< \brief SPI mode with external clock */
	#define	SERCOM_MODE_SPI_MASTER		(0x3<<2)   /**< \brief SPI mode with internal clock */
	#define	SERCOM_MODE_I2C_SLAVE		(0x4<<2)   /**< \brief I2C mode with external clock */
	#define	SERCOM_MODE_I2C_MASTER		(0x5<<2)   /**< \brief I2C mode with internal clock */

	#define	I2C_SWRST			(1<<0)            /**< \brief (SERCOM_I2CS_CTRLA) Software Reset */
	#define	I2C_ENABLE			(1<<1)            /**< \brief (SERCOM_I2CS_CTRLA) Enable */
	#define	I2C_RUNSTDBY		(1<<7)            /**< \brief (SERCOM_I2CM_CTRLA) Run in Standby */
	#define	I2C_PINOUT			(1<<16)           /**< \brief (SERCOM_I2CM_CTRLA) Pin Usage */
	#define	I2C_SDAHOLD_DIS		(0<<20)           /**< \brief (SERCOM_I2CM_CTRLA) SDA Hold Time */
	#define	I2C_SDAHOLD_75NS	(1<<20)           /**< \brief (SERCOM_I2CM_CTRLA) SDA Hold Time */
	#define	I2C_SDAHOLD_450NS	(2<<20)           /**< \brief (SERCOM_I2CM_CTRLA) SDA Hold Time */
	#define	I2C_SDAHOLD_600NS	(3<<20)           /**< \brief (SERCOM_I2CM_CTRLA) SDA Hold Time */
	#define	I2C_MEXTTOEN		(1<<22)           /**< \brief (SERCOM_I2CM_CTRLA) Master SCL Low Extend Timeout */
	#define	I2C_SEXTTOEN		(1<<23)           /**< \brief (SERCOM_I2CM_CTRLA) Slave SCL Low Extend Timeout */
	#define	I2C_SPEED_SM		(0<<24)           /**< \brief (SERCOM_I2CM_CTRLA) Transfer Speed */
	#define	I2C_SPEED_FM		(1<<24)           /**< \brief (SERCOM_I2CM_CTRLA) Transfer Speed */
	#define	I2C_SPEED_HSM		(2<<24)           /**< \brief (SERCOM_I2CM_CTRLA) Transfer Speed */
	#define	I2C_SCLSM			(1<<27)           /**< \brief (SERCOM_I2CM_CTRLA) SCL Clock Stretch Mode */
	#define	I2C_INACTOUT_DIS	(0<<28)           /**< \brief (SERCOM_I2CM_CTRLA) Inactive Time-Out */
	#define	I2C_INACTOUT_55US	(1<<28)           /**< \brief (SERCOM_I2CM_CTRLA) Inactive Time-Out */
	#define	I2C_INACTOUT_105US	(2<<28)           /**< \brief (SERCOM_I2CM_CTRLA) Inactive Time-Out */
	#define	I2C_INACTOUT_205US	(3<<28)           /**< \brief (SERCOM_I2CM_CTRLA) Inactive Time-Out */
	#define	I2C_LOWTOUTEN		(1<<30)           /**< \brief (SERCOM_I2CM_CTRLA) SCL Low Timeout Enable */

	#define I2C_MB		(1<<0)					/**< \brief (SERCOM_I2CM_INTFLAG) Master On Bus Interrupt */
	#define I2C_SB		(1<<1)					/**< \brief (SERCOM_I2CM_INTFLAG) Slave On Bus Interrupt */
	#define I2C_ERROR	(1<<7)					/**< \brief (SERCOM_I2CM_INTFLAG) Combined Error Interrupt */

	#define I2C_SMEN		(1<<8)            	/**< \brief (SERCOM_I2CM_CTRLB) Smart Mode Enable */
	#define I2C_QCEN		(1<<9)            	/**< \brief (SERCOM_I2CM_CTRLB) Quick Command Enable */
	#define I2C_CMD(value)	(((value)&3)<<16)	/**< \brief (SERCOM_I2CM_CTRLB) Command */
	#define I2C_CMD_1		(1<<16)           	/**< \brief (SERCOM_I2CM_CTRLB) Command */
	#define I2C_CMD_2		(2<<16)           	/**< \brief (SERCOM_I2CM_CTRLB) Command */
	#define I2C_CMD_STOP	(3<<16)           	/**< \brief (SERCOM_I2CM_CTRLB) Command */
	#define I2C_ACKACT		(1<<18)           	/**< \brief (SERCOM_I2CM_CTRLB) Acknowledge Action */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	struct S_SPI
	{
		RW32    CTRLA;       		/**< \brief Offset: 0x00 (R/W 32) SPI Control A */
		RW32    CTRLB;       		/**< \brief Offset: 0x04 (R/W 32) SPI Control B */
		RW32    CTRLC;       		/**< \brief Offset: 0x04 (R/W 32) SPI Control B */
		RW8		BAUD;				/**< \brief Offset: 0x0C (R/W  8) SPI Baud Rate */
		RO8     z__Reserved2[0x7];	
		RW8		INTENCLR;			/**< \brief Offset: 0x14 (R/W  8) SPI Interrupt Enable Clear */
		RO8     z__Reserved3[0x1];	
		RW8		INTENSET;			/**< \brief Offset: 0x16 (R/W  8) SPI Interrupt Enable Set */
		RO8     z__Reserved4[0x1];	
		RW8		INTFLAG;			/**< \brief Offset: 0x18 (R/W  8) SPI Interrupt Flag Status and Clear */
		RO8     z__Reserved5[0x1];	
		RW16	STATUS;      		/**< \brief Offset: 0x1A (R/W 16) SPI Status */
		RO32	SYNCBUSY;    		/**< \brief Offset: 0x1C (R/  32) SPI Syncbusy */
		RO8		z__Reserved6[0x2];	
		RW16	LENGTH;      		
		RW32    ADDR;        		/**< \brief Offset: 0x24 (R/W 32) SPI Address */
		RW32    DATA;        		/**< \brief Offset: 0x28 (R/W 32) SPI Data */
		RO8     z__Reserved7[0x4];	
		RW8		DBGCTRL;			/**< \brief Offset: 0x30 (R/W  8) SPI Debug Control */
	};

	typedef S_SPI S_SPI0, S_SPI1, S_SPI2, S_SPI3, S_SPI4, S_SPI5, S_SPI6, S_SPI7;

	#define SPI_SWRST  			(1<<0)            /**< \brief (SERCOM_SPI_CTRLA) Software Reset */
	#define SPI_ENABLE 			(1<<1)            /**< \brief (SERCOM_SPI_CTRLA) Enable */
	#define SPI_RUNSTDBY		(1<<7)            /**< \brief (SERCOM_SPI_CTRLA) Run during Standby */
	#define SPI_IBON			(1<<8)            /**< \brief (SERCOM_SPI_CTRLA) Immediate Buffer Overflow Notification */

	#define SPI_DOPO(value) 	(((value)&3) << 16)
	#define SPI_DIPO(value) 	(((value)&3) << 20)

	#define SPI_FORM_SPI		(0<<24)
	#define SPI_FORM_SPI_ADDR	(2<<24)
	#define SPI_CPHA   			(1<<28)   	/**< \brief (SERCOM_SPI_CTRLA) Clock Phase */
	#define SPI_CPOL   			(1<<29)   	/**< \brief (SERCOM_SPI_CTRLA) Clock Polarity */
	#define SPI_DORD   			(1<<30)   	/**< \brief (SERCOM_SPI_CTRLA) Data Order */

	#define SPI_CHSIZE_8BIT		(0<<0)		/**< \brief (SERCOM_SPI_CTRLB) Character Size */
	#define SPI_CHSIZE_9BIT		(1<<0)
	#define SPI_PLOADEN			(1<<6)    	/**< \brief (SERCOM_SPI_CTRLB) Data Preload Enable */
	#define SPI_SSDE			(1<<9)    	/**< \brief (SERCOM_SPI_CTRLB) Slave Select Low Detect Enable */
	#define SPI_MSSEN			(1<<13)   	/**< \brief (SERCOM_SPI_CTRLB) Master Slave Select Enable */
	#define SPI_AMODE_MASK		(0<<14)     /**< \brief (SERCOM_SPI_CTRLB) Address Mode */
	#define SPI_AMODE_2_ADDR	(1<<14)
	#define SPI_AMODE_RANGE		(2<<14)
	#define SPI_RXEN			(1<<17)		/**< \brief (SERCOM_SPI_CTRLB) Receiver Enable */
	#define SPI_FIFOCLR_NONE	(0<<22)
	#define SPI_FIFOCLR_TXFIFO	(1<<22)
	#define SPI_FIFOCLR_RXFIFO	(2<<22)
	#define SPI_FIFOCLR_BOTH	(3<<22)

	#define SPI_DRE 			(1<<0)		/**< \brief (SERCOM_SPI_INTENCLR) Data Register Empty Interrupt Disable */
	#define SPI_TXC 			(1<<1)		/**< \brief (SERCOM_SPI_INTENCLR) Transmit Complete Interrupt Disable */
	#define SPI_RXC 			(1<<2)		/**< \brief (SERCOM_SPI_INTENCLR) Receive Complete Interrupt Disable */
	#define SPI_SSL 			(1<<3)		/**< \brief (SERCOM_SPI_INTENCLR) Slave Select Low Interrupt Disable */
	#define SPI_ERROR			(1<<7)		/**< \brief (SERCOM_SPI_INTENCLR) Combined Error Interrupt Disable */
	#define SPI_BUFOVF			(1<<2)		/**< \brief (SERCOM_SPI_STATUS) Buffer Overflow */
	#define SPI_CTRLB			(1<<2)		/**< \brief (SERCOM_SPI_SYNCBUSY) CTRLB Synchronization Busy */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	struct S_USART
	{
		RW32   	CTRLA;       /**< \brief Offset: 0x00 (R/W 32) USART Control A */
		RW32   	CTRLB;       /**< \brief Offset: 0x04 (R/W 32) USART Control B */
		RW32    CTRLC;       /**< \brief Offset: 0x08 (R/W 32) USART Control C */
		RW16    BAUD;        /**< \brief Offset: 0x0C (R/W 16) USART Baud Rate */
		RW8		RXPL;        /**< \brief Offset: 0x0E (R/W  8) USART Receive Pulse Length */
		RO8                    z__Reserved1[0x5];
		RW8		INTENCLR;    /**< \brief Offset: 0x14 (R/W  8) USART Interrupt Enable Clear */
		RO8                    z__Reserved2[0x1];
		RW8		INTENSET;    /**< \brief Offset: 0x16 (R/W  8) USART Interrupt Enable Set */
		RO8                    z__Reserved3[0x1];
		RW8		INTFLAG;     /**< \brief Offset: 0x18 (R/W  8) USART Interrupt Flag Status and Clear */
		RO8                    z__Reserved4[0x1];
		RW16	STATUS;      /**< \brief Offset: 0x1A (R/W 16) USART Status */
		RO32	SYNCBUSY;    /**< \brief Offset: 0x1C (R/  32) USART Synchronization Busy */
		RO8		RXERRCNT;    /**< \brief Offset: 0x20 (R/   8) USART Receive Error Count */
		RO8                    z__Reserved5[0x1];
		RW16	LENGTH;      /**< \brief Offset: 0x22 (R/W 16) USART Length */
		RO8                    z__Reserved6[0x4];
		RW32    DATA;        /**< \brief Offset: 0x28 (R/W 32) USART Data */
		RO8                    z__Reserved7[0x4];
		RW8		DBGCTRL;     /**< \brief Offset: 0x30 (R/W  8) USART Debug Control */
	};

	typedef S_USART S_USART0, S_USART1, S_USART2, S_USART3, S_USART4, S_USART5, S_USART6, S_USART7;


	#define USART_SWRST				(1<<0)      /**< \brief (SERCOM_USART_CTRLA) Software Reset */
	#define USART_ENABLE			(1<<1)      /**< \brief (SERCOM_USART_CTRLA) Enable */
	#define	USART_MODE_EXT_CLK 		(0x0<<2)	/**< \brief USART mode with external clock */
	#define	USART_MODE_INT_CLK 		(0x1<<2)	/**< \brief USART mode with internal clock */

	#define USART_RUNSTDBY			(1<<7)		/**< \brief (SERCOM_USART_CTRLA) Run during Standby */
	#define USART_IBON				(1<<8)		/**< \brief (SERCOM_USART_CTRLA) Immediate Buffer Overflow Notification */
	#define USART_TXINV				(1<<9)		/**< \brief (SERCOM_USART_CTRLA) Transmit Data Invert */
	#define USART_RXINV				(1<<10)		/**< \brief (SERCOM_USART_CTRLA) Receive Data Invert */

	#define USART_SAMPR_16x_ARITH	(0<<13)
	#define USART_SAMPR_16x_FRACT	(1<<13)
	#define USART_SAMPR_8x_ARITH	(2<<13)
	#define USART_SAMPR_8x_FRACT	(3<<13)
	#define USART_SAMPR_3x_ARITH	(4<<13)

	#define USART_TXPO_0 			(0<<16)		/**< \brief (SERCOM_USART_CTRLA) Transmit Data Pinout */
	#define USART_TXPO_2 			(2<<16)		/**< \brief (SERCOM_USART_CTRLA) Transmit Data Pinout */
	#define USART_TXPO_3 			(3<<16)		/**< \brief (SERCOM_USART_CTRLA) Transmit Data Pinout */

	#define USART_RXPO_0 			(0<<20)		/**< \brief (SERCOM_USART_CTRLA) Receive Data Pinout */
	#define USART_RXPO_1 			(1<<20)		/**< \brief (SERCOM_USART_CTRLA) Receive Data Pinout */
	#define USART_RXPO_2 			(2<<20)		/**< \brief (SERCOM_USART_CTRLA) Receive Data Pinout */
	#define USART_RXPO_3 			(3<<20)		/**< \brief (SERCOM_USART_CTRLA) Receive Data Pinout */

	#define USART_SAMPA_0 			(0<<22)		/**< \brief (SERCOM_USART_CTRLA) Sample Adjustment */
	#define USART_SAMPA_1 			(1<<22)		/**< \brief (SERCOM_USART_CTRLA) Sample Adjustment */
	#define USART_SAMPA_2 			(2<<22)		/**< \brief (SERCOM_USART_CTRLA) Sample Adjustment */
	#define USART_SAMPA_3 			(3<<22)		/**< \brief (SERCOM_USART_CTRLA) Sample Adjustment */

	#define USART_FORM_USART		(0<<24)		/**< \brief (SERCOM_USART_CTRLA) Frame Format */
	#define USART_FORM_USART_PARITY	(1<<24)		/**< \brief (SERCOM_USART_CTRLA) Frame Format */

	#define USART_CMODE 			(1<<28)		/**< \brief (SERCOM_USART_CTRLA) Communication Mode */
	#define USART_CPOL				(1<<29)		/**< \brief (SERCOM_USART_CTRLA) Clock Polarity */
	#define USART_DORD				(1<<30)		/**< \brief (SERCOM_USART_CTRLA) Data Order */


	#define USART_CHSIZE_8 			(0<<0)            /**< \brief (SERCOM_USART_CTRLB) Character Size */
	#define USART_CHSIZE_9 			(1<<0)            /**< \brief (SERCOM_USART_CTRLB) Character Size */
	#define USART_CHSIZE_5 			(5<<0)            /**< \brief (SERCOM_USART_CTRLB) Character Size */
	#define USART_CHSIZE_6 			(6<<0)            /**< \brief (SERCOM_USART_CTRLB) Character Size */
	#define USART_CHSIZE_7 			(7<<0)            /**< \brief (SERCOM_USART_CTRLB) Character Size */

	#define USART_SBMODE  			(1<<6)            /**< \brief (SERCOM_USART_CTRLB) Stop Bit Mode */
	#define USART_COLDEN  			(1<<8)            /**< \brief (SERCOM_USART_CTRLB) Collision Detection Enable */
	#define USART_SFDE  			(1<<9)            /**< \brief (SERCOM_USART_CTRLB) Start of Frame Detection Enable */
	#define USART_ENC   			(1<<10)           /**< \brief (SERCOM_USART_CTRLB) Encoding Format */
	#define USART_PMODE 			(1<<13)           /**< \brief (SERCOM_USART_CTRLB) Parity Mode */
	#define USART_TXEN  			(1<<16)           /**< \brief (SERCOM_USART_CTRLB) Transmitter Enable */
	#define USART_RXEN  			(1<<17)           /**< \brief (SERCOM_USART_CTRLB) Receiver Enable */

	#define USART_DRE  				(1<<0)            /**< \brief (SERCOM_USART_INTFLAG) Data Register Empty Interrupt */
	#define USART_TXC  				(1<<1)            /**< \brief (SERCOM_USART_INTFLAG) Transmit Complete Interrupt */
	#define USART_RXC  				(1<<2)            /**< \brief (SERCOM_USART_INTFLAG) Receive Complete Interrupt */
	#define USART_RXS  				(1<<3)            /**< \brief (SERCOM_USART_INTFLAG) Receive Start Interrupt */
	#define USART_CTSIC  			(1<<4)            /**< \brief (SERCOM_USART_INTFLAG) Clear To Send Input Change Interrupt */
	#define USART_RXBRK  			(1<<5)            /**< \brief (SERCOM_USART_INTFLAG) Break Received Interrupt */
	#define USART_ERROR  			(1<<7)            /**< \brief (SERCOM_USART_INTFLAG) Combined Error Interrupt */


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_TCC
	{
		RW32	CTRLA;       		/**< \brief Offset: 0x00 (R/W 32) Control A */
		RW8     CTRLBCLR;    		/**< \brief Offset: 0x04 (R/W  8) Control B Clear */
		RW8     CTRLBSET;    		/**< \brief Offset: 0x05 (R/W  8) Control B Set */
		RO8     z__Reserved1[0x2];
		RO32    SYNCBUSY;    		/**< \brief Offset: 0x08 (R/  32) Synchronization Busy */
		RW32    FCTRLA;      		/**< \brief Offset: 0x0C (R/W 32) Recoverable Fault A Configuration */
		RW32    FCTRLB;      		/**< \brief Offset: 0x10 (R/W 32) Recoverable Fault B Configuration */
		RW32    WEXCTRL;     		/**< \brief Offset: 0x14 (R/W 32) Waveform Extension Configuration */
		RW32    DRVCTRL;     		/**< \brief Offset: 0x18 (R/W 32) Driver Control */
		RO8     z__Reserved2[0x2];
		RW8     DBGCTRL;			/**< \brief Offset: 0x1E (R/W  8) Debug Control */
		RO8     z__Reserved3[0x1];
		RW32	EVCTRL;      		/**< \brief Offset: 0x20 (R/W 32) Event Control */
		RW32	INTENCLR;    		/**< \brief Offset: 0x24 (R/W 32) Interrupt Enable Clear */
		RW32	INTENSET;    		/**< \brief Offset: 0x28 (R/W 32) Interrupt Enable Set */
		RW32	INTFLAG;     		/**< \brief Offset: 0x2C (R/W 32) Interrupt Flag Status and Clear */
		RW32	STATUS;      		/**< \brief Offset: 0x30 (R/W 32) Status */
		RW32    COUNT;       		/**< \brief Offset: 0x34 (R/W 32) Count */
		RW16	PATT;        		/**< \brief Offset: 0x38 (R/W 16) Pattern */
		RO8		z__Reserved4[0x2];
		RW32	WAVE;        		/**< \brief Offset: 0x3C (R/W 32) Waveform Control */
		RW32	PER;         		/**< \brief Offset: 0x40 (R/W 32) Period */
		RW32	CC[6];       		/**< \brief Offset: 0x44 (R/W 32) Compare and Capture */
		RO8     z__Reserved5[0x8];
		RW16    PATTBUF;			/**< \brief Offset: 0x64 (R/W 16) Pattern Buffer */
		RO8     z__Reserved6[0x6];
		RW32	PERBUF;      		/**< \brief Offset: 0x6C (R/W 32) Period Buffer */
		RW32	CCBUF[6];    		/**< \brief Offset: 0x70 (R/W 32) Compare and Capture Buffer */
	};

	typedef S_TCC S_TCC0, S_TCC1, S_TCC2, S_TCC3, S_TCC4;

	#define TCC_SWRST				(1<<0)            /**< \brief (TCC_CTRLA) Software Reset */
	#define TCC_ENABLE				(1<<1)            /**< \brief (TCC_CTRLA) Enable */
	#define TCC_RESOLUTION_NONE   	(0 << 5)
	#define TCC_RESOLUTION_DITH4  	(1 << 5)
	#define TCC_RESOLUTION_DITH5  	(2 << 5)
	#define TCC_RESOLUTION_DITH6  	(3 << 5)
	#define TCC_PRESCALER_DIV1    	(0 << 8)
	#define TCC_PRESCALER_DIV2    	(1 << 8)
	#define TCC_PRESCALER_DIV4    	(2 << 8)
	#define TCC_PRESCALER_DIV8    	(3 << 8)
	#define TCC_PRESCALER_DIV16   	(4 << 8)
	#define TCC_PRESCALER_DIV64   	(5 << 8)
	#define TCC_PRESCALER_DIV256  	(6 << 8)
	#define TCC_PRESCALER_DIV1024 	(7 << 8)
	#define TCC_RUNSTDBY_Pos      	(1 << 11)           /**< \brief (TCC_CTRLA) Run in Standby */
	#define TCC_PRESCSYNC_GCLK    	(0 << 12)
	#define TCC_PRESCSYNC_PRESC   	(1 << 12)
	#define TCC_PRESCSYNC_RESYNC  	(2 << 12)
	#define TCC_ALOCK         		(1<<14)           /**< \brief (TCC_CTRLA) Auto Lock */
	#define TCC_CPTEN0        		(1<<24)           /**< \brief (TCC_CTRLA) Capture Channel 0 Enable */
	#define TCC_CPTEN1        		(1<<25)           /**< \brief (TCC_CTRLA) Capture Channel 1 Enable */
	#define TCC_CPTEN2        		(1<<26)           /**< \brief (TCC_CTRLA) Capture Channel 2 Enable */
	#define TCC_CPTEN3        		(1<<27)           /**< \brief (TCC_CTRLA) Capture Channel 3 Enable */
		
	#define TCC_DIR        		(1<<0)            /**< \brief (TCC_CTRLBCLR) Counter Direction */
	#define TCC_LUPD       		(1<<1)            /**< \brief (TCC_CTRLBCLR) Lock Update */
	#define TCC_ONESHOT    		(1<<2)            /**< \brief (TCC_CTRLBCLR) One-Shot */
	#define TCC_IDXCMD_DISABLE 		(0<<3)
	#define TCC_IDXCMD_SET     		(1<<3)
	#define TCC_IDXCMD_CLEAR   		(2<<3)
	#define TCC_IDXCMD_HOLD    		(3<<3)
	#define TCC_CMD_NONE       		(0<<5)
	#define TCC_CMD_RETRIGGER  		(1<<5)
	#define TCC_CMD_STOP       		(2<<5)
	#define TCC_CMD_UPDATE     		(3<<5)
	#define TCC_CMD_READSYNC   		(4<<5)

	#define TCC_OVF        (1<<0)            /**< \brief (TCC_INTENCLR) Overflow Interrupt Enable */
	#define TCC_TRG        (1<<1)            /**< \brief (TCC_INTENCLR) Retrigger Interrupt Enable */
	#define TCC_CNT        (1<<2)            /**< \brief (TCC_INTENCLR) Counter Interrupt Enable */
	#define TCC_ERR        (1<<3)            /**< \brief (TCC_INTENCLR) Error Interrupt Enable */
	#define TCC_UFS        (1<<10)           /**< \brief (TCC_INTENCLR) Non-Recoverable Update Fault Interrupt Enable */
	#define TCC_DFS        (1<<11)           /**< \brief (TCC_INTENCLR) Non-Recoverable Debug Fault Interrupt Enable */
	#define TCC_FAULTA     (1<<12)           /**< \brief (TCC_INTENCLR) Recoverable Fault A Interrupt Enable */
	#define TCC_FAULTB     (1<<13)           /**< \brief (TCC_INTENCLR) Recoverable Fault B Interrupt Enable */
	#define TCC_FAULT0     (1<<14)           /**< \brief (TCC_INTENCLR) Non-Recoverable Fault 0 Interrupt Enable */
	#define TCC_FAULT1     (1<<15)           /**< \brief (TCC_INTENCLR) Non-Recoverable Fault 1 Interrupt Enable */
	#define TCC_MC0        (1<<16)           /**< \brief (TCC_INTENCLR) Match or Capture Channel 0 Interrupt Enable */
	#define TCC_MC1        (1<<17)           /**< \brief (TCC_INTENCLR) Match or Capture Channel 1 Interrupt Enable */
	#define TCC_MC2        (1<<18)           /**< \brief (TCC_INTENCLR) Match or Capture Channel 2 Interrupt Enable */
	#define TCC_MC3        (1<<19)           /**< \brief (TCC_INTENCLR) Match or Capture Channel 3 Interrupt Enable */

	#define TCC_NRE0         		(1<<0)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 0 Output Enable */
	#define TCC_NRE1         		(1<<1)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 1 Output Enable */
	#define TCC_NRE2         		(1<<2)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 2 Output Enable */
	#define TCC_NRE3         		(1<<3)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 3 Output Enable */
	#define TCC_NRE4         		(1<<4)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 4 Output Enable */
	#define TCC_NRE5         		(1<<5)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 5 Output Enable */
	#define TCC_NRE6         		(1<<6)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 6 Output Enable */
	#define TCC_NRE7         		(1<<7)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 7 Output Enable */
	#define TCC_NRV0         		(1<<8)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 0 Output Value */
	#define TCC_NRV1         		(1<<9)            /**< \brief (TCC_DRVCTRL) Non-Recoverable State 1 Output Value */
	#define TCC_NRV2         		(1<<10)           /**< \brief (TCC_DRVCTRL) Non-Recoverable State 2 Output Value */
	#define TCC_NRV3         		(1<<11)           /**< \brief (TCC_DRVCTRL) Non-Recoverable State 3 Output Value */
	#define TCC_NRV4         		(1<<12)           /**< \brief (TCC_DRVCTRL) Non-Recoverable State 4 Output Value */
	#define TCC_NRV5         		(1<<13)           /**< \brief (TCC_DRVCTRL) Non-Recoverable State 5 Output Value */
	#define TCC_NRV6         		(1<<14)           /**< \brief (TCC_DRVCTRL) Non-Recoverable State 6 Output Value */
	#define TCC_NRV7         		(1<<15)           /**< \brief (TCC_DRVCTRL) Non-Recoverable State 7 Output Value */
	#define TCC_INVEN0       		(1<<16)           /**< \brief (TCC_DRVCTRL) Output Waveform 0 Inversion */
	#define TCC_INVEN1       		(1<<17)           /**< \brief (TCC_DRVCTRL) Output Waveform 1 Inversion */
	#define TCC_INVEN2       		(1<<18)           /**< \brief (TCC_DRVCTRL) Output Waveform 2 Inversion */
	#define TCC_INVEN3       		(1<<19)           /**< \brief (TCC_DRVCTRL) Output Waveform 3 Inversion */
	#define TCC_INVEN4       		(1<<20)           /**< \brief (TCC_DRVCTRL) Output Waveform 4 Inversion */
	#define TCC_INVEN5       		(1<<21)           /**< \brief (TCC_DRVCTRL) Output Waveform 5 Inversion */
	#define TCC_INVEN6       		(1<<22)           /**< \brief (TCC_DRVCTRL) Output Waveform 6 Inversion */
	#define TCC_INVEN7       		(1<<23)           /**< \brief (TCC_DRVCTRL) Output Waveform 7 Inversion */
	#define TCC_FILTERVAL0(value) 	((value)&0xF << 24)
	#define TCC_FILTERVAL1(value) 	((value)&0xF << 28)

	#define TCC_EVACT0_OFF       	(0x0)		/**< \brief (TCC_EVCTRL) Event action disabled */
	#define TCC_EVACT0_RETRIGGER 	(0x1)		/**< \brief (TCC_EVCTRL) Start, restart or re-trigger counter on event */
	#define TCC_EVACT0_COUNTEV   	(0x2)		/**< \brief (TCC_EVCTRL) Count on event */
	#define TCC_EVACT0_START     	(0x3)		/**< \brief (TCC_EVCTRL) Start counter on event */
	#define TCC_EVACT0_INC       	(0x4)		/**< \brief (TCC_EVCTRL) Increment counter on event */
	#define TCC_EVACT0_COUNT     	(0x5)		/**< \brief (TCC_EVCTRL) Count on active state of asynchronous event */
	#define TCC_EVACT0_FAULT     	(0x7)		/**< \brief (TCC_EVCTRL) Non-recoverable fault */
	#define TCC_EVACT1_OFF       	(0x0<<3)	/**< \brief (TCC_EVCTRL) Event action disabled */
	#define TCC_EVACT1_RETRIGGER 	(0x1<<3)	/**< \brief (TCC_EVCTRL) Re-trigger counter on event */
	#define TCC_EVACT1_DIR       	(0x2<<3)	/**< \brief (TCC_EVCTRL) Direction control */
	#define TCC_EVACT1_STOP      	(0x3<<3)	/**< \brief (TCC_EVCTRL) Stop counter on event */
	#define TCC_EVACT1_DEC       	(0x4<<3)	/**< \brief (TCC_EVCTRL) Decrement counter on event */
	#define TCC_EVACT1_PPW       	(0x5<<3)	/**< \brief (TCC_EVCTRL) Period capture value in CC0 register, pulse width capture value in CC1 register */
	#define TCC_EVACT1_PWP       	(0x6<<3)	/**< \brief (TCC_EVCTRL) Period capture value in CC1 register, pulse width capture value in CC0 register */
	#define TCC_EVACT1_FAULT     	(0x7<<3)	/**< \brief (TCC_EVCTRL) Non-recoverable fault */
	#define TCC_CNTSEL_START     	(0x0<<6)	/**< \brief (TCC_EVCTRL) An interrupt/event is generated when a new counter cycle starts */
	#define TCC_CNTSEL_END       	(0x1<<6)	/**< \brief (TCC_EVCTRL) An interrupt/event is generated when a counter cycle ends */
	#define TCC_CNTSEL_BETWEEN   	(0x2<<6)	/**< \brief (TCC_EVCTRL) An interrupt/event is generated when a counter cycle ends, except for the first and last cycles */
	#define TCC_CNTSEL_BOUNDARY  	(0x3<<6)	/**< \brief (TCC_EVCTRL) An interrupt/event is generated when a new counter cycle starts or a counter cycle ends */
	#define TCC_OVFEO        		(1<<8)    	/**< \brief (TCC_EVCTRL) Overflow/Underflow Output Event Enable */
	#define TCC_TRGEO        		(1<<9)    	/**< \brief (TCC_EVCTRL) Retrigger Output Event Enable */
	#define TCC_CNTEO        		(1<<10)   	/**< \brief (TCC_EVCTRL) Timer/counter Output Event Enable */
	#define TCC_TCINV0       		(1<<12)   	/**< \brief (TCC_EVCTRL) Inverted Event 0 Input Enable */
	#define TCC_TCINV1       		(1<<13)   	/**< \brief (TCC_EVCTRL) Inverted Event 1 Input Enable */
	#define TCC_TCEI0        		(1<<14)   	/**< \brief (TCC_EVCTRL) Timer/counter Event 0 Input Enable */
	#define TCC_TCEI1        		(1<<15)   	/**< \brief (TCC_EVCTRL) Timer/counter Event 1 Input Enable */
	#define TCC_MCEI0        		(1<<16)   	/**< \brief (TCC_EVCTRL) Match or Capture Channel 0 Event Input Enable */
	#define TCC_MCEI1        		(1<<17)   	/**< \brief (TCC_EVCTRL) Match or Capture Channel 1 Event Input Enable */
	#define TCC_MCEI2        		(1<<18)   	/**< \brief (TCC_EVCTRL) Match or Capture Channel 2 Event Input Enable */
	#define TCC_MCEI3        		(1<<19)   	/**< \brief (TCC_EVCTRL) Match or Capture Channel 3 Event Input Enable */
	#define TCC_MCEO0        		(1<<24)   	/**< \brief (TCC_EVCTRL) Match or Capture Channel 0 Event Output Enable */
	#define TCC_MCEO1        		(1<<25)   	/**< \brief (TCC_EVCTRL) Match or Capture Channel 1 Event Output Enable */
	#define TCC_MCEO2        		(1<<26)   	/**< \brief (TCC_EVCTRL) Match or Capture Channel 2 Event Output Enable */
	#define TCC_MCEO3        		(1<<27)   	/**< \brief (TCC_EVCTRL) Match or Capture Channel 3 Event Output Enable */

	#define	TCC_WAVEGEN_NFRQ        (0x0)   /**< \brief (TCC_WAVE) Normal frequency */
	#define	TCC_WAVEGEN_MFRQ        (0x1)   /**< \brief (TCC_WAVE) Match frequency */
	#define	TCC_WAVEGEN_NPWM        (0x2)   /**< \brief (TCC_WAVE) Normal PWM */
	#define	TCC_WAVEGEN_DSCRITICAL  (0x4)   /**< \brief (TCC_WAVE) Dual-slope critical */
	#define	TCC_WAVEGEN_DSBOTTOM    (0x5)   /**< \brief (TCC_WAVE) Dual-slope with interrupt/event condition when COUNT reaches ZERO */
	#define	TCC_WAVEGEN_DSBOTH      (0x6)   /**< \brief (TCC_WAVE) Dual-slope with interrupt/event condition when COUNT reaches ZERO or TOP */
	#define	TCC_WAVEGEN_DSTOP       (0x7)   /**< \brief (TCC_WAVE) Dual-slope with interrupt/event condition when COUNT reaches TOP */
	#define	TCC_RAMP_1          	(0x0<<4)   /**< \brief (TCC_WAVE) RAMP1 operation */
	#define	TCC_RAMP_2A         	(0x1<<4)   /**< \brief (TCC_WAVE) Alternative RAMP2 operation */
	#define	TCC_RAMP_2          	(0x2<<4)   /**< \brief (TCC_WAVE) RAMP2 operation */
	#define	TCC_RAMP_2C         	(0x3<<4)   /**< \brief (TCC_WAVE) Critical RAMP2 operation */
	#define TCC_CIPEREN         	(1<<7)            /**< \brief (TCC_WAVE) Circular period Enable */
	#define TCC_CICCEN0         	(1<<8)            /**< \brief (TCC_WAVE) Circular Channel 0 Enable */
	#define TCC_CICCEN1         	(1<<9)            /**< \brief (TCC_WAVE) Circular Channel 1 Enable */
	#define TCC_CICCEN2         	(1<<10)           /**< \brief (TCC_WAVE) Circular Channel 2 Enable */
	#define TCC_CICCEN3         	(1<<11)           /**< \brief (TCC_WAVE) Circular Channel 3 Enable */
	#define TCC_POL0            	(1<<16)           /**< \brief (TCC_WAVE) Channel 0 Polarity */
	#define TCC_POL1            	(1<<17)           /**< \brief (TCC_WAVE) Channel 1 Polarity */
	#define TCC_POL2            	(1<<18)           /**< \brief (TCC_WAVE) Channel 2 Polarity */
	#define TCC_POL3         		(1<<19)           /**< \brief (TCC_WAVE) Channel 3 Polarity */
	#define TCC_POL4         		(1<<20)           /**< \brief (TCC_WAVE) Channel 4 Polarity */
	#define TCC_POL5         		(1<<21)           /**< \brief (TCC_WAVE) Channel 5 Polarity */
	#define TCC_SWAP0        		(1<<24)           /**< \brief (TCC_WAVE) Swap DTI Output Pair 0 */
	#define TCC_SWAP1           	(1<<25)           /**< \brief (TCC_WAVE) Swap DTI Output Pair 1 */
	#define TCC_SWAP2           	(1<<26)           /**< \brief (TCC_WAVE) Swap DTI Output Pair 2 */
	#define TCC_SWAP3           	(1<<27)           /**< \brief (TCC_WAVE) Swap DTI Output Pair 3 */

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_EIC
	{
		RW8   	CTRLA;       /**< \brief Offset: 0x00 (R/W  8) Control A */
		RW8   	NMICTRL;     /**< \brief Offset: 0x01 (R/W  8) Non-Maskable Interrupt Control */
		RW16	NMIFLAG;     /**< \brief Offset: 0x02 (R/W 16) Non-Maskable Interrupt Flag Status and Clear */
		RO32  	SYNCBUSY;    /**< \brief Offset: 0x04 (R/  32) Synchronization Busy */
		RW32  	EVCTRL;      /**< \brief Offset: 0x08 (R/W 32) Event Control */
		RW32  	INTENCLR;    /**< \brief Offset: 0x0C (R/W 32) Interrupt Enable Clear */
		RW32  	INTENSET;    /**< \brief Offset: 0x10 (R/W 32) Interrupt Enable Set */
		RW32  	INTFLAG;     /**< \brief Offset: 0x14 (R/W 32) Interrupt Flag Status and Clear */
		RW32  	ASYNCH;      /**< \brief Offset: 0x18 (R/W 32) External Interrupt Asynchronous Mode */
		RW32  	CONFIG[2];   /**< \brief Offset: 0x1C (R/W 32) External Interrupt Sense Configuration */
		RO8					z__Reserved1[0xC];
		RW32  	DEBOUNCEN;   /**< \brief Offset: 0x30 (R/W 32) Debouncer Enable */
		RW32  	DPRESCALER;  /**< \brief Offset: 0x34 (R/W 32) Debouncer Prescaler */
		RO32  	PINSTATE;    /**< \brief Offset: 0x38 (R/  32) Pin State */

		void SetConfig(byte intn, bool filt, byte sense) { byte n = intn / 8; intn &= 7; CONFIG[n] = (CONFIG[n] & ~(0xF << (intn*4))) | (filt << (intn*4+3)) | (sense << (intn*4)); }
	};

	#define EIC_SWRST_Pos       	0            /**< \brief (EIC_CTRLA) Software Reset */
	#define EIC_SWRST           	((0x1) << EIC_SWRST_Pos)
	#define EIC_ENABLE_Pos      	1         /**< \brief (EIC_CTRLA) Enable */
	#define EIC_ENABLE          	((0x1) << EIC_ENABLE_Pos)
	#define EIC_CKSEL_Pos       	4         /**< \brief (EIC_CTRLA) Clock Selection */
	#define EIC_CKSEL           	((0x1) << EIC_CKSEL_Pos)

	#define EIC_NMISENSE_Pos    	0            /**< \brief (EIC_NMICTRL) Non-Maskable Interrupt Sense Configuration */
	#define EIC_NMISENSE_Msk    	(_U_(0x7) << EIC_NMISENSE_Pos)
	#define EIC_NMISENSE(value) 	(EIC_NMISENSE_Msk & ((value) << EIC_NMISENSE_Pos))
	#define EIC_NMISENSE_NONE   	(0x0 << EIC_NMISENSE_Pos)
	#define EIC_NMISENSE_RISE   	(0x1 << EIC_NMISENSE_Pos)
	#define EIC_NMISENSE_FALL   	(0x2 << EIC_NMISENSE_Pos)
	#define EIC_NMISENSE_BOTH   	(0x3 << EIC_NMISENSE_Pos)
	#define EIC_NMISENSE_HIGH   	(0x4 << EIC_NMISENSE_Pos)
	#define EIC_NMISENSE_LOW    	(0x5 << EIC_NMISENSE_Pos)
	#define EIC_NMIFILTEN_Pos   	3            /**< \brief (EIC_NMICTRL) Non-Maskable Interrupt Filter Enable */
	#define EIC_NMIFILTEN       	((0x1) << EIC_NMIFILTEN_Pos)
	#define EIC_NMIASYNCH_Pos   	4            /**< \brief (EIC_NMICTRL) Asynchronous Edge Detection Mode */
	#define EIC_NMIASYNCH       	((0x1) << EIC_NMIASYNCH_Pos)

	#define EIC_NMI_Pos     		0            /**< \brief (EIC_NMIFLAG) Non-Maskable Interrupt */
	#define EIC_NMI         		((0x1) << EIC_NMI_Pos)

	#define EIC_SENSE_NONE 			(0x0)		/**< \brief (EIC_CONFIG) No detection */
	#define EIC_SENSE_RISE 			(0x1)		/**< \brief (EIC_CONFIG) Rising edge detection */
	#define EIC_SENSE_FALL 			(0x2)		/**< \brief (EIC_CONFIG) Falling edge detection */
	#define EIC_SENSE_BOTH 			(0x3)		/**< \brief (EIC_CONFIG) Both edges detection */
	#define EIC_SENSE_HIGH 			(0x4)		/**< \brief (EIC_CONFIG) High level detection */
	#define EIC_SENSE_LOW   		(0x5)		/**< \brief (EIC_CONFIG) Low level detection */
	//#define EIC_FILTEN(i)       	(0x1 << ((i)*4+3))		/**< \brief (EIC_CONFIG) Filter Enable i*/

	#define EIC_PRESCALER0_Pos 		0            /**< \brief (EIC_DPRESCALER) Debouncer Prescaler */
	#define EIC_PRESCALER0_Msk 		(_U_(0x7) << EIC_PRESCALER0_Pos)
	#define EIC_PRESCALER0(value)	(EIC_PRESCALER0_Msk & ((value) << EIC_PRESCALER0_Pos))
	#define EIC_STATES0_Pos			3            /**< \brief (EIC_DPRESCALER) Debouncer number of states */
	#define EIC_STATES0				(_U_(0x1) << EIC_STATES0_Pos)
	#define EIC_PRESCALER1_Pos		4            /**< \brief (EIC_DPRESCALER) Debouncer Prescaler */
	#define EIC_PRESCALER1_Msk		(_U_(0x7) << EIC_PRESCALER1_Pos)
	#define EIC_PRESCALER1(value)	(EIC_PRESCALER1_Msk & ((value) << EIC_PRESCALER1_Pos))
	#define EIC_STATES1_Pos			7            /**< \brief (EIC_DPRESCALER) Debouncer number of states */
	#define EIC_STATES1      		(_U_(0x1) << EIC_STATES1_Pos)
	#define EIC_TICKON_Pos   		16           /**< \brief (EIC_DPRESCALER) Pin Sampler frequency selection */
	#define EIC_TICKON       		(_U_(0x1) << EIC_TICKON_Pos)

	#define EIC_EXTINT0    	(1<<0)							
	#define EIC_EXTINT1    	(1<<1)							
	#define EIC_EXTINT2    	(1<<2)							
	#define EIC_EXTINT3    	(1<<3)							
	#define EIC_EXTINT4    	(1<<4)							
	#define EIC_EXTINT5    	(1<<5)							
	#define EIC_EXTINT6    	(1<<6)							
	#define EIC_EXTINT7    	(1<<7)							
	#define EIC_EXTINT8    	(1<<8)							
	#define EIC_EXTINT9    	(1<<9)							
	#define EIC_EXTINT10    (1<<10)							
	#define EIC_EXTINT11    (1<<11)							
	#define EIC_EXTINT12    (1<<12)							
	#define EIC_EXTINT13    (1<<13)							
	#define EIC_EXTINT14    (1<<14)							
	#define EIC_EXTINT15    (1<<15)							

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_EVSYS	
	{
		struct S_CH
		{
			RW32    CHANNEL;    /**< \brief Offset: 0x000 (R/W 32) Channel n Control */
			RW8     INTENCLR;  	/**< \brief Offset: 0x004 (R/W  8) Channel n Interrupt Enable Clear */
			RW8     INTENSET;  	/**< \brief Offset: 0x005 (R/W  8) Channel n Interrupt Enable Set */
			RW8     INTFLAG;   	/**< \brief Offset: 0x006 (R/W  8) Channel n Interrupt Flag Status and Clear */
			RO8		STATUS;    	/**< \brief Offset: 0x007 (R/   8) Channel n Status */
		};

		RW8     CTRLA;		/**< \brief Offset: 0x000 (R/W  8) Control */
		RO8					z__Reserved1[0x3];
		RW32    SWEVT;      /**< \brief Offset: 0x004 ( /W 32) Software Event */
		RW8     PRICTRL;    /**< \brief Offset: 0x008 (R/W  8) Priority Control */
		RO8					z__Reserved2[0x7];
		RW16    INTPEND;    /**< \brief Offset: 0x010 (R/W 16) Channel Pending Interrupt */
		RO8					z__Reserved3[0x2];
		RO32    INTSTATUS;	/**< \brief Offset: 0x014 (R/  32) Interrupt Status */
		RO32    BUSYCH;   	/**< \brief Offset: 0x018 (R/  32) Busy Channels */
		RO32    READYUSR; 	/**< \brief Offset: 0x01C (R/  32) Ready Users */
		S_CH	CH[32];		/**< \brief Offset: 0x020 EvsysChannel groups [CHANNELS] */
		RW32    USER[67];   /**< \brief Offset: 0x120 (R/W 32) User Multiplexer n */
	};

	#define EVSYS_SWRST			(1<<0)          /**< \brief (EVSYS_CTRLA) Software Reset */

	#define EVSYS_RREN			(1<<7)          /**< \brief (EVSYS_PRICTRL) Round-Robin Scheduling Enable */

	#define EVSYS_ID(value)		((value)&0xF)	/**< \brief (EVSYS_INTPEND) Channel ID */
	#define EVSYS_OVR        	(1<<8)        	/**< \brief (EVSYS_INTPEND) Channel Overrun */
	#define EVSYS_EVD        	(1<<9)        	/**< \brief (EVSYS_INTPEND) Channel Event Detected */
	#define EVSYS_READY      	(1<<14)       	/**< \brief (EVSYS_INTPEND) Ready */
	#define EVSYS_BUSY       	(1<<15)       	/**< \brief (EVSYS_INTPEND) Busy */


	#define EVSYS_EVGEN(value)			((value)&0x7F)	/**< \brief (EVSYS_CHANNEL) Event Generator Selection */
	#define EVSYS_PATH_SYNCHRONOUS 		(0x0<<8)   		/**< \brief (EVSYS_CHANNEL) Synchronous path */
	#define EVSYS_PATH_RESYNCHRONIZED 	(0x1<<8)   		/**< \brief (EVSYS_CHANNEL) Resynchronized path */
	#define EVSYS_PATH_ASYNCHRONOUS 	(0x2<<8)   		/**< \brief (EVSYS_CHANNEL) Asynchronous path */
	#define EVSYS_EDGSEL_NO_EVT_OUTPUT 	(0x0<<10)   	/**< \brief (EVSYS_CHANNEL) No event output when using the resynchronized or synchronous path */
	#define EVSYS_EDGSEL_RISING_EDGE 	(0x1<<10)   	/**< \brief (EVSYS_CHANNEL) Event detection only on the rising edge of the signal from the event generator when using the resynchronized or synchronous path */
	#define EVSYS_EDGSEL_FALLING_EDGE 	(0x2<<10)   	/**< \brief (EVSYS_CHANNEL) Event detection only on the falling edge of the signal from the event generator when using the resynchronized or synchronous path */
	#define EVSYS_EDGSEL_BOTH_EDGES 	(0x3<<10)   	/**< \brief (EVSYS_CHANNEL) Event detection on rising and falling edges of the signal from the event generator when using the resynchronized or synchronous path */
	#define EVSYS_RUNSTDBY   			(1<<14)       	/**< \brief (EVSYS_CHANNEL) Run in standby */
	#define EVSYS_ONDEMAND   			(1<<15)       	/**< \brief (EVSYS_CHANNEL) Generic Clock On Demand */






	#define EVGEN_NONE						0
	#define EVGEN_OSCCTRL_XOSC_FAIL_0		1
	#define EVGEN_OSCCTRL_XOSC_FAIL_1		2
	#define EVGEN_OSC32KCTRL_XOSC32K_FAIL	3
	#define EVGEN_RTC_PER_0      			4
	#define EVGEN_RTC_PER_1      			5
	#define EVGEN_RTC_PER_2      			6
	#define EVGEN_RTC_PER_3      			7
	#define EVGEN_RTC_PER_4      			8
	#define EVGEN_RTC_PER_5      			9
	#define EVGEN_RTC_PER_6      			10
	#define EVGEN_RTC_PER_7      			11
	#define EVGEN_RTC_CMP_0      			12
	#define EVGEN_RTC_CMP_1      			13
	#define EVGEN_RTC_CMP_2      			14
	#define EVGEN_RTC_CMP_3      			15
	#define EVGEN_RTC_TAMPER     			16
	#define EVGEN_RTC_OVF        			17
	#define EVGEN_EIC_EXTINT_0   			18
	#define EVGEN_EIC_EXTINT_1   			19
	#define EVGEN_EIC_EXTINT_2   			20
	#define EVGEN_EIC_EXTINT_3   			21
	#define EVGEN_EIC_EXTINT_4   			22
	#define EVGEN_EIC_EXTINT_5   			23
	#define EVGEN_EIC_EXTINT_6   			24
	#define EVGEN_EIC_EXTINT_7   			25
	#define EVGEN_EIC_EXTINT_8   			26
	#define EVGEN_EIC_EXTINT_9   			27
	#define EVGEN_EIC_EXTINT_10  			28
	#define EVGEN_EIC_EXTINT_11  			29
	#define EVGEN_EIC_EXTINT_12  			30
	#define EVGEN_EIC_EXTINT_13  			31
	#define EVGEN_EIC_EXTINT_14  			32
	#define EVGEN_EIC_EXTINT_15  			33
	#define EVGEN_DMAC_CH_0      			34
	#define EVGEN_DMAC_CH_1      			35
	#define EVGEN_DMAC_CH_2      			36
	#define EVGEN_DMAC_CH_3      			37
	#define EVGEN_PAC_ACCERR     			38
	#define EVGEN_TCC0_OVF       			41
	#define EVGEN_TCC0_TRG       			42
	#define EVGEN_TCC0_CNT       			43
	#define EVGEN_TCC0_MC_0      			44
	#define EVGEN_TCC0_MC_1      			45
	#define EVGEN_TCC0_MC_2      			46
	#define EVGEN_TCC0_MC_3      			47
	#define EVGEN_TCC0_MC_4      			48
	#define EVGEN_TCC0_MC_5      			49
	#define EVGEN_TCC1_OVF       			50
	#define EVGEN_TCC1_TRG       			51
	#define EVGEN_TCC1_CNT       			52
	#define EVGEN_TCC1_MC_0      			53
	#define EVGEN_TCC1_MC_1      			54
	#define EVGEN_TCC1_MC_2      			55
	#define EVGEN_TCC1_MC_3      			56
	#define EVGEN_TCC2_OVF       			57
	#define EVGEN_TCC2_TRG       			58
	#define EVGEN_TCC2_CNT       			59
	#define EVGEN_TCC2_MC_0      			60
	#define EVGEN_TCC2_MC_1      			61
	#define EVGEN_TCC2_MC_2      			62
	#define EVGEN_TCC3_OVF       			63
	#define EVGEN_TCC3_TRG       			64
	#define EVGEN_TCC3_CNT       			65
	#define EVGEN_TCC3_MC_0      			66
	#define EVGEN_TCC3_MC_1      			67
	#define EVGEN_TCC4_OVF       			68
	#define EVGEN_TCC4_TRG       			69
	#define EVGEN_TCC4_CNT       			70
	#define EVGEN_TCC4_MC_0      			71
	#define EVGEN_TCC4_MC_1      			72
	#define EVGEN_TC0_OVF        			73
	#define EVGEN_TC0_MC_0       			74
	#define EVGEN_TC0_MC_1       			75
	#define EVGEN_TC1_OVF        			76
	#define EVGEN_TC1_MC_0       			77
	#define EVGEN_TC1_MC_1       			78
	#define EVGEN_TC2_OVF        			79
	#define EVGEN_TC2_MC_0       			80
	#define EVGEN_TC2_MC_1       			81
	#define EVGEN_TC3_OVF        			82
	#define EVGEN_TC3_MC_0       			83
	#define EVGEN_TC3_MC_1       			84
	#define EVGEN_TC4_OVF        			85
	#define EVGEN_TC4_MC_0       			86
	#define EVGEN_TC4_MC_1       			87
	#define EVGEN_TC5_OVF        			88
	#define EVGEN_TC5_MC_0       			89
	#define EVGEN_TC5_MC_1       			90
	#define EVGEN_TC6_OVF        			91
	#define EVGEN_TC6_MC_0       			92
	#define EVGEN_TC6_MC_1       			93
	#define EVGEN_TC7_OVF        			94
	#define EVGEN_TC7_MC_0       			95
	#define EVGEN_TC7_MC_1       			96
	#define EVGEN_PDEC_OVF       			97
	#define EVGEN_PDEC_ERR       			98
	#define EVGEN_PDEC_DIR       			99
	#define EVGEN_PDEC_VLC       			100
	#define EVGEN_PDEC_MC_0      			101
	#define EVGEN_PDEC_MC_1      			102
	#define EVGEN_ADC0_RESRDY    			103
	#define EVGEN_ADC0_WINMON    			104
	#define EVGEN_ADC1_RESRDY    			105
	#define EVGEN_ADC1_WINMON    			106
	#define EVGEN_AC_COMP_0      			107
	#define EVGEN_AC_COMP_1      			108
	#define EVGEN_AC_WIN_0       			109
	#define EVGEN_DAC_EMPTY_0    			110
	#define EVGEN_DAC_EMPTY_1    			111
	#define EVGEN_DAC_RESRDY_0   			112
	#define EVGEN_DAC_RESRDY_1   			113
	#define EVGEN_GMAC_TSU_CMP   			114
	#define EVGEN_TRNG_READY     			115
	#define EVGEN_CCL_LUTOUT_0   			116
	#define EVGEN_CCL_LUTOUT_1   			117
	#define EVGEN_CCL_LUTOUT_2   			118
	#define EVGEN_CCL_LUTOUT_3   			119
									 
	#define EVSYS_USER_RTC_TAMPER    		0
	#define EVSYS_USER_PORT_EV_0     		1
	#define EVSYS_USER_PORT_EV_1     		2
	#define EVSYS_USER_PORT_EV_2     		3
	#define EVSYS_USER_PORT_EV_3     		4
	#define EVSYS_USER_DMAC_CH_0     		5
	#define EVSYS_USER_DMAC_CH_1     		6
	#define EVSYS_USER_DMAC_CH_2     		7
	#define EVSYS_USER_DMAC_CH_3     		8
	#define EVSYS_USER_DMAC_CH_4     		9
	#define EVSYS_USER_DMAC_CH_5     		10
	#define EVSYS_USER_DMAC_CH_6     		11
	#define EVSYS_USER_DMAC_CH_7     		12
	#define EVSYS_USER_CM4_TRACE_START		14
	#define EVSYS_USER_CM4_TRACE_STOP 		15
	#define EVSYS_USER_CM4_TRACE_TRIG 		16
	#define EVSYS_USER_TCC0_EV_0     		17
	#define EVSYS_USER_TCC0_EV_1     		18
	#define EVSYS_USER_TCC0_MC_0     		19
	#define EVSYS_USER_TCC0_MC_1     		20
	#define EVSYS_USER_TCC0_MC_2     		21
	#define EVSYS_USER_TCC0_MC_3     		22
	#define EVSYS_USER_TCC0_MC_4     		23
	#define EVSYS_USER_TCC0_MC_5     		24
	#define EVSYS_USER_TCC1_EV_0     		25
	#define EVSYS_USER_TCC1_EV_1     		26
	#define EVSYS_USER_TCC1_MC_0     		27
	#define EVSYS_USER_TCC1_MC_1     		28
	#define EVSYS_USER_TCC1_MC_2     		29
	#define EVSYS_USER_TCC1_MC_3     		30
	#define EVSYS_USER_TCC2_EV_0     		31
	#define EVSYS_USER_TCC2_EV_1     		32
	#define EVSYS_USER_TCC2_MC_0     		33
	#define EVSYS_USER_TCC2_MC_1     		34
	#define EVSYS_USER_TCC2_MC_2     		35
	#define EVSYS_USER_TCC3_EV_0     		36
	#define EVSYS_USER_TCC3_EV_1     		37
	#define EVSYS_USER_TCC3_MC_0     		38
	#define EVSYS_USER_TCC3_MC_1     		39
	#define EVSYS_USER_TCC4_EV_0     		40
	#define EVSYS_USER_TCC4_EV_1     		41
	#define EVSYS_USER_TCC4_MC_0     		42
	#define EVSYS_USER_TCC4_MC_1     		43
	#define EVSYS_USER_TC0_EVU       		44
	#define EVSYS_USER_TC1_EVU       		45
	#define EVSYS_USER_TC2_EVU       		46
	#define EVSYS_USER_TC3_EVU       		47
	#define EVSYS_USER_TC4_EVU       		48
	#define EVSYS_USER_TC5_EVU       		49
	#define EVSYS_USER_TC6_EVU       		50
	#define EVSYS_USER_TC7_EVU       		51
	#define EVSYS_USER_PDEC_EVU_0    		52
	#define EVSYS_USER_PDEC_EVU_1    		53
	#define EVSYS_USER_PDEC_EVU_2    		54
	#define EVSYS_USER_ADC0_START    		55
	#define EVSYS_USER_ADC0_SYNC     		56
	#define EVSYS_USER_ADC1_START    		57
	#define EVSYS_USER_ADC1_SYNC     		58
	#define EVSYS_USER_AC_SOC_0      		59
	#define EVSYS_USER_AC_SOC_1      		60
	#define EVSYS_USER_DAC_START_0   		61
	#define EVSYS_USER_DAC_START_1   		62
	#define EVSYS_USER_CCL_LUTIN_0   		63
	#define EVSYS_USER_CCL_LUTIN_1   		64
	#define EVSYS_USER_CCL_LUTIN_2   		65
	#define EVSYS_USER_CCL_LUTIN_3   		66

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_TC
	{
		RW32		CTRLA;       /**< \brief Offset: 0x00 (R/W 32) Control A */
		RW8       	CTRLBCLR;    /**< \brief Offset: 0x04 (R/W  8) Control B Clear */
		RW8       	CTRLBSET;    /**< \brief Offset: 0x05 (R/W  8) Control B Set */
		RW16		EVCTRL;      /**< \brief Offset: 0x06 (R/W 16) Event Control */
		RW8       	INTENCLR;    /**< \brief Offset: 0x08 (R/W  8) Interrupt Enable Clear */
		RW8       	INTENSET;    /**< \brief Offset: 0x09 (R/W  8) Interrupt Enable Set */
		RW8       	INTFLAG;     /**< \brief Offset: 0x0A (R/W  8) Interrupt Flag Status and Clear */
		RW8       	STATUS;      /**< \brief Offset: 0x0B (R/W  8) Status */
		RW8       	WAVE;        /**< \brief Offset: 0x0C (R/W  8) Waveform Generation Control */
		RW8       	DRVCTRL;     /**< \brief Offset: 0x0D (R/W  8) Control C */
		RO8						z__Reserved1[0x1];
		RW8       	DBGCTRL;     /**< \brief Offset: 0x0F (R/W  8) Debug Control */
		RO32		SYNCBUSY;    /**< \brief Offset: 0x10 (R/  32) Synchronization Status */
  
		union
		{
			struct
			{
				RW8       	COUNT8;       /**< \brief Offset: 0x14 (R/W  8) COUNT8 Count */
				RO8       				z__Reserved2[0x6];
				RW8       	PER8;         /**< \brief Offset: 0x1B (R/W  8) COUNT8 Period */
				RW8       	CC8[2];       /**< \brief Offset: 0x1C (R/W  8) COUNT8 Compare and Capture */
				RO8						z__Reserved3[0x11];
				RW8       	PERBUF8;      /**< \brief Offset: 0x2F (R/W  8) COUNT8 Period Buffer */
				RW8       	CCBUF8[2];    /**< \brief Offset: 0x30 (R/W  8) COUNT8 Compare and Capture Buffer */
			};

			struct
			{
				RW16		COUNT16;       /**< \brief Offset: 0x14 (R/W 16) COUNT16 Count */
				RO8						z__Reserved4[0x6];
				RW16		CC16[2];       /**< \brief Offset: 0x1C (R/W 16) COUNT16 Compare and Capture */
				RO8						z__Reserved5[0x10];
				RW16		CCBUF16[2];    /**< \brief Offset: 0x30 (R/W 16) COUNT16 Compare and Capture Buffer */
			};

			struct
			{
				RW32		COUNT32;       /**< \brief Offset: 0x14 (R/W 32) COUNT32 Count */
				RO8       				z__Reserved6[0x4];
				RW32		CC32[2];       /**< \brief Offset: 0x1C (R/W 32) COUNT32 Compare and Capture */
				RO8       				z__Reserved7[0xC];
				RW32		CCBUF32[2];    /**< \brief Offset: 0x30 (R/W 32) COUNT32 Compare and Capture Buffer */
			};
		};
	};

	typedef S_TC S_TC0, S_TC1, S_TC2, S_TC3, S_TC4, S_TC5, S_TC6, S_TC7;

	#define TC_SWRST          	(1<<0)            /**< \brief (TC_CTRLA) Software Reset */
	#define TC_ENABLE         	(1<<1)            /**< \brief (TC_CTRLA) Enable */
	#define TC_MODE_COUNT16   	(0x0<<2)   /**< \brief (TC_CTRLA) Counter in 16-bit mode */
	#define TC_MODE_COUNT8    	(0x1<<2)   /**< \brief (TC_CTRLA) Counter in 8-bit mode */
	#define TC_MODE_COUNT32   	(0x2<<2)   /**< \brief (TC_CTRLA) Counter in 32-bit mode */
	#define	TC_PRESCSYNC_GCLK     (0x0<<4)   /**< \brief (TC_CTRLA) Reload or reset the counter on next generic clock */
	#define	TC_PRESCSYNC_PRESC    (0x1<<4)   /**< \brief (TC_CTRLA) Reload or reset the counter on next prescaler clock */
	#define	TC_PRESCSYNC_RESYNC   (0x2<<4)   /**< \brief (TC_CTRLA) Reload or reset the counter on next generic clock and reset the prescaler counter */
	#define TC_RUNSTDBY       		(1<<6)            /**< \brief (TC_CTRLA) Run during Standby */
	#define TC_ONDEMAND       		(1<<7)            /**< \brief (TC_CTRLA) Clock On Demand */
	#define TC_PRESCALER_DIV1     	(0x0<<8)   /**< \brief (TC_CTRLA) Prescaler: GCLK_TC */
	#define TC_PRESCALER_DIV2     	(0x1<<8)   /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/2 */
	#define TC_PRESCALER_DIV4     	(0x2<<8)   /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/4 */
	#define TC_PRESCALER_DIV8     	(0x3<<8)   /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/8 */
	#define TC_PRESCALER_DIV16    	(0x4<<8)   /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/16 */
	#define TC_PRESCALER_DIV64    	(0x5<<8)   /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/64 */
	#define TC_PRESCALER_DIV256   	(0x6<<8)   /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/256 */
	#define TC_PRESCALER_DIV1024  	(0x7<<8)   /**< \brief (TC_CTRLA) Prescaler: GCLK_TC/1024 */
	#define TC_ALOCK          		(1<<11)           /**< \brief (TC_CTRLA) Auto Lock */
	#define TC_CAPTEN0        		(1<<16)           /**< \brief (TC_CTRLA) Capture Channel 0 Enable */
	#define TC_CAPTEN1        		(1<<17)           /**< \brief (TC_CTRLA) Capture Channel 1 Enable */
	#define TC_CAPTEN         		(1<<16)           /**< \brief (TC_CTRLA) Capture Channel x Enable */
	#define TC_COPEN0         		(1<<20)           /**< \brief (TC_CTRLA) Capture On Pin 0 Enable */
	#define TC_COPEN1         		(1<<21)           /**< \brief (TC_CTRLA) Capture On Pin 1 Enable */
	#define TC_CAPTMODE0_DEFAULT  	(0x0<<24)   /**< \brief (TC_CTRLA) Default capture */
	#define TC_CAPTMODE0_CAPTMIN  	(0x1<<24)   /**< \brief (TC_CTRLA) Minimum capture */
	#define TC_CAPTMODE0_CAPTMAX  	(0x2<<24)   /**< \brief (TC_CTRLA) Maximum capture */
	#define TC_CAPTMODE1_DEFAULT  	(0x0<<27)   /**< \brief (TC_CTRLA) Default capture */
	#define TC_CAPTMODE1_CAPTMIN  	(0x1<<27)   /**< \brief (TC_CTRLA) Minimum capture */
	#define TC_CAPTMODE1_CAPTMAX  	(0x2<<27)   /**< \brief (TC_CTRLA) Maximum capture */

	#define TC_DIR         			(1<<0)    	/**< \brief (TC_CTRLBCLR) Counter Direction */
	#define TC_LUPD        			(1<<1)    	/**< \brief (TC_CTRLBCLR) Lock Update */
	#define TC_ONESHOT     			(1<<2)    	/**< \brief (TC_CTRLBCLR) One-Shot on Counter */
	#define TC_CMD_NONE        		(0x0<<5)	/**< \brief (TC_CTRLBCLR) No action */
	#define TC_CMD_RETRIGGER   		(0x1<<5)	/**< \brief (TC_CTRLBCLR) Force a start, restart or retrigger */
	#define TC_CMD_STOP        		(0x2<<5)	/**< \brief (TC_CTRLBCLR) Force a stop */
	#define TC_CMD_UPDATE      		(0x3<<5)	/**< \brief (TC_CTRLBCLR) Force update of double-buffered register */
	#define TC_CMD_READSYNC    		(0x4<<5)	/**< \brief (TC_CTRLBCLR) Force a read synchronization of COUNT */
	#define TC_CMD_DMAOS       		(0x5<<5)	/**< \brief (TC_CTRLBCLR) One-shot DMA trigger */


	#define TC_EVACT_OFF        	(0x0)   	/**< \brief (TC_EVCTRL) Event action disabled */
	#define TC_EVACT_RETRIGGER  	(0x1)   	/**< \brief (TC_EVCTRL) Start, restart or retrigger TC on event */
	#define TC_EVACT_COUNT      	(0x2)   	/**< \brief (TC_EVCTRL) Count on event */
	#define TC_EVACT_START      	(0x3)   	/**< \brief (TC_EVCTRL) Start TC on event */
	#define TC_EVACT_STAMP      	(0x4)   	/**< \brief (TC_EVCTRL) Time stamp capture */
	#define TC_EVACT_PPW        	(0x5)   	/**< \brief (TC_EVCTRL) Period catured in CC0, pulse width in CC1 */
	#define TC_EVACT_PWP        	(0x6)   	/**< \brief (TC_EVCTRL) Period catured in CC1, pulse width in CC0 */
	#define TC_EVACT_PW         	(0x7)   	/**< \brief (TC_EVCTRL) Pulse width capture */
	#define TC_TCINV         		(1<<4)    	/**< \brief (TC_EVCTRL) TC Event Input Polarity */
	#define TC_TCEI          		(1<<5)    	/**< \brief (TC_EVCTRL) TC Event Enable */
	#define TC_OVFEO         		(1<<8)    	/**< \brief (TC_EVCTRL) Event Output Enable */
	#define TC_MCEO0         		(1<<12)   	/**< \brief (TC_EVCTRL) MC Event Output Enable 0 */
	#define TC_MCEO1         		(1<<13)   	/**< \brief (TC_EVCTRL) MC Event Output Enable 1 */

	#define TC_OVF         			(1<<0)    	/**< \brief (TC_INTENCLR) OVF Interrupt Disable */
	#define TC_ERR         			(1<<1)    	/**< \brief (TC_INTENCLR) ERR Interrupt Disable */
	#define TC_MC0         			(1<<4)    	/**< \brief (TC_INTENCLR) MC Interrupt Disable 0 */
	#define TC_MC1         			(1<<5)    	/**< \brief (TC_INTENCLR) MC Interrupt Disable 1 */

	#define TC_STOP          		(1<<0)    	/**< \brief (TC_STATUS) Stop Status Flag */
	#define TC_SLAVE         		(1<<1)    	/**< \brief (TC_STATUS) Slave Status Flag */
	#define TC_PERBUFV       		(1<<3)    	/**< \brief (TC_STATUS) Synchronization Busy Status */
	#define TC_CCBUFV0       		(1<<4)    	/**< \brief (TC_STATUS) Compare channel buffer 0 valid */
	#define TC_CCBUFV1       		(1<<5)    	/**< \brief (TC_STATUS) Compare channel buffer 1 valid */

	#define	TC_WAVEGEN_NFRQ         (0x0)   	/**< \brief (TC_WAVE) Normal frequency */
	#define	TC_WAVEGEN_MFRQ         (0x1)   	/**< \brief (TC_WAVE) Match frequency */
	#define	TC_WAVEGEN_NPWM         (0x2)   	/**< \brief (TC_WAVE) Normal PWM */
	#define	TC_WAVEGEN_MPWM         (0x3)   	/**< \brief (TC_WAVE) Match PWM */

	#define TC_INVEN0       		(1<<0)    	/**< \brief (TC_DRVCTRL) Output Waveform Invert Enable 0 */
	#define TC_INVEN1       		(1<<1)    	/**< \brief (TC_DRVCTRL) Output Waveform Invert Enable 1 */

	#define TC_DBGRUN				(1<<0)		/**< \brief (TC_DBGCTRL) Run During Debug */

	#define TC_CTRLB        		(1<<2)            /**< \brief (TC_SYNCBUSY) CTRLB */
	#define TC_STATUS       		(1<<3)            /**< \brief (TC_SYNCBUSY) STATUS */
	#define TC_COUNT        		(1<<4)            /**< \brief (TC_SYNCBUSY) Counter */
	#define TC_PER          		(1<<5)            /**< \brief (TC_SYNCBUSY) Period */
	#define TC_CC0          		(1<<6)            /**< \brief (TC_SYNCBUSY) Compare Channel 0 */
	#define TC_CC1          		(1<<7)            /**< \brief (TC_SYNCBUSY) Compare Channel 1 */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct DMADESC;

	typedef volatile DMADESC *RWDMADESC;
	
	struct DMADESC
	{
		RW16			BTCTRL;      /**< \brief Offset: 0x00 (R/W 16) Block Transfer Control */
		RW16			BTCNT;       /**< \brief Offset: 0x02 (R/W 16) Block Transfer Count */
		RO_PTR			SRCADDR;     /**< \brief Offset: 0x04 (R/W 32) Block Transfer Source Address */
		RW_PTR			DSTADDR;     /**< \brief Offset: 0x08 (R/W 32) Block Transfer Destination Address */
		RWDMADESC		DESCADDR;    /**< \brief Offset: 0x0C (R/W 32) Next Descriptor Address */
	};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_DMAC
	{
		struct S_DMAC_CH
		{
			RW32    CTRLA;     /**< \brief Offset: 0x00 (R/W 32) Channel n Control A */
			RW8		CTRLB;     /**< \brief Offset: 0x04 (R/W  8) Channel n Control B */
			RW8		PRILVL;    /**< \brief Offset: 0x05 (R/W  8) Channel n Priority Level */
			RW8		EVCTRL;    /**< \brief Offset: 0x06 (R/W  8) Channel n Event Control */
			RO8					z__Reserved1[0x5];
			RW8		INTENCLR;  /**< \brief Offset: 0x0C (R/W  8) Channel n Interrupt Enable Clear */
			RW8		INTENSET;  /**< \brief Offset: 0x0D (R/W  8) Channel n Interrupt Enable Set */
			RW8		INTFLAG;   /**< \brief Offset: 0x0E (R/W  8) Channel n Interrupt Flag Status and Clear */
			RW8		STATUS;    /**< \brief Offset: 0x0F (R/W  8) Channel n Status */
		};

		RW16   		CTRL;        /**< \brief Offset: 0x00 (R/W 16) Control */
		RW16   		CRCCTRL;     /**< \brief Offset: 0x02 (R/W 16) CRC Control */
		RW32   		CRCDATAIN;   /**< \brief Offset: 0x04 (R/W 32) CRC Data Input */
		RW32   		CRCCHKSUM;   /**< \brief Offset: 0x08 (R/W 32) CRC Checksum */
		RW8    		CRCSTATUS;   /**< \brief Offset: 0x0C (R/W  8) CRC Status */
		RW8    		DBGCTRL;     /**< \brief Offset: 0x0D (R/W  8) Debug Control */
		RO8     	            z__Reserved1[0x2];
		RW32  		SWTRIGCTRL;  /**< \brief Offset: 0x10 (R/W 32) Software Trigger Control */
		RW32  		PRICTRL0;    /**< \brief Offset: 0x14 (R/W 32) Priority Control 0 */
		RO8     	            z__Reserved2[0x8];
		RW16    	INTPEND;     /**< \brief Offset: 0x20 (R/W 16) Interrupt Pending */
		RO8						z__Reserved3[0x2];
		RO32  		INTSTATUS;   /**< \brief Offset: 0x24 (R/  32) Interrupt Status */
		RO32  		BUSYCH;      /**< \brief Offset: 0x28 (R/  32) Busy Channels */
		RO32  		PENDCH;      /**< \brief Offset: 0x2C (R/  32) Pending Channels */
		RO32  		ACTIVE;      /**< \brief Offset: 0x30 (R/  32) Active Channel and Levels */
		RWDMADESC	BASEADDR;    /**< \brief Offset: 0x34 (R/W 32) Descriptor Memory Section Base Address */
		RWDMADESC   WRBADDR;     /**< \brief Offset: 0x38 (R/W 32) Write-Back Memory Section Base Address */
		RO8                    z__Reserved4[0x4];
		S_DMAC_CH	CH[32];		/**< \brief Offset: 0x40 DmacChannel groups [CH_NUM] */
	};

	#define DMAC_SWRST         			(1<<0)            /**< \brief (DMAC_CTRL) Software Reset */
	#define DMAC_DMAENABLE     			(1<<1)            /**< \brief (DMAC_CTRL) DMA Enable */
	#define DMAC_CRCENABLE     			(1<<2)            /**< \brief (DMAC_CTRL) CRC Enable */
	#define DMAC_LVLEN0        			(1<<8)            /**< \brief (DMAC_CTRL) Priority Level 0 Enable */
	#define DMAC_LVLEN1        			(1<<9)            /**< \brief (DMAC_CTRL) Priority Level 1 Enable */
	#define DMAC_LVLEN2        			(1<<10)           /**< \brief (DMAC_CTRL) Priority Level 2 Enable */
	#define DMAC_LVLEN3        			(1<<11)           /**< \brief (DMAC_CTRL) Priority Level 3 Enable */

	#define DMAC_CRCBEATSIZE_BYTE		(0 << 0)
	#define DMAC_CRCBEATSIZE_HWORD		(1 << 0)
	#define DMAC_CRCBEATSIZE_WORD		(2 << 0)
	#define DMAC_CRCPOLY_CRC16			(0 << 2)
	#define DMAC_CRCPOLY_CRC32			(1 << 2)
	#define DMAC_CRCSRC(value)			(((value)&0x3F) << 8)
	#define DMAC_CRCSRC_NOACT   		(0 << 8)
	#define DMAC_CRCSRC_IO      		(1 << 8)

	#define DMAC_CRCMODE_DEFAULT   		(0 << 14)
	#define DMAC_CRCMODE_CRCMON   		(2 << 14)
	#define DMAC_CRCMODE_CRCGEN   		(3 << 14)

	#define DMAC_CRCBUSY  				(1<<0)            /**< \brief (DMAC_CRCSTATUS) CRC Module Busy */
	#define DMAC_CRCZERO  				(1<<1)            /**< \brief (DMAC_CRCSTATUS) CRC Zero */
	#define DMAC_CRCERR					(1<<2)            /**< \brief (DMAC_CRCSTATUS) CRC Error */

	#define DMAC_DBGRUN					(1<<0)            /**< \brief (DMAC_DBGCTRL) Debug Run */

	#define DMAC_INTPEND_TERR       	(1<<8)            /**< \brief (DMAC_INTPEND) Transfer Error */
	#define DMAC_INTPEND_TCMPL      	(1<<9)            /**< \brief (DMAC_INTPEND) Transfer Complete */
	#define DMAC_INTPEND_SUSP       	(1<<10)           /**< \brief (DMAC_INTPEND) Channel Suspend */
	#define DMAC_INTPEND_CRCERR     	(1<<12)           /**< \brief (DMAC_INTPEND) CRC Error */
	#define DMAC_INTPEND_FERR       	(1<<13)           /**< \brief (DMAC_INTPEND) Fetch Error */
	#define DMAC_INTPEND_BUSY       	(1<<14)           /**< \brief (DMAC_INTPEND) Busy */
	#define DMAC_INTPEND_PEND       	(1<<15)           /**< \brief (DMAC_INTPEND) Pending */


	#define DMCH_SWRST      			(1<<0)           /**< \brief (DMAC_CHCTRLA) Channel Software Reset */
	#define DMCH_ENABLE     			(1<<1)           /**< \brief (DMAC_CHCTRLA) Channel Enable */

	#define DMCH_RUNSTDBY				(1<<6)            /**< \brief (DMAC_CHCTRLA) Channel Run in Standby */
	#define DMCH_TRIGACT_BLOCK  		(0x0<<20)   /**< \brief (DMAC_CHCTRLA) One trigger required for each block transfer */
	#define DMCH_TRIGACT_BURST  		(0x2<<20)   /**< \brief (DMAC_CHCTRLA) One trigger required for each burst transfer */
	#define DMCH_TRIGACT_TRANSACTION	(0x3<<20)   /**< \brief (DMAC_CHCTRLA) One trigger required for each transaction */
	#define DMCH_BURSTLEN_SINGLE		(0x0<<24)   /**< \brief (DMAC_CHCTRLA) Single-beat burst length */
	#define DMCH_BURSTLEN_2BEAT 		(0x1<<24)   /**< \brief (DMAC_CHCTRLA) 2-beats burst length */
	#define DMCH_BURSTLEN_3BEAT 		(0x2<<24)   /**< \brief (DMAC_CHCTRLA) 3-beats burst length */
	#define DMCH_BURSTLEN_4BEAT 		(0x3<<24)   /**< \brief (DMAC_CHCTRLA) 4-beats burst length */
	#define DMCH_BURSTLEN_5BEAT 		(0x4<<24)   /**< \brief (DMAC_CHCTRLA) 5-beats burst length */
	#define DMCH_BURSTLEN_6BEAT 		(0x5<<24)   /**< \brief (DMAC_CHCTRLA) 6-beats burst length */
	#define DMCH_BURSTLEN_7BEAT 		(0x6<<24)   /**< \brief (DMAC_CHCTRLA) 7-beats burst length */
	#define DMCH_BURSTLEN_8BEAT 		(0x7<<24)   /**< \brief (DMAC_CHCTRLA) 8-beats burst length */
	#define DMCH_BURSTLEN_9BEAT 		(0x8<<24)   /**< \brief (DMAC_CHCTRLA) 9-beats burst length */
	#define DMCH_BURSTLEN_10BEAT 		(0x9<<24)   /**< \brief (DMAC_CHCTRLA) 10-beats burst length */
	#define DMCH_BURSTLEN_11BEAT 		(0xA<<24)   /**< \brief (DMAC_CHCTRLA) 11-beats burst length */
	#define DMCH_BURSTLEN_12BEAT 		(0xB<<24)   /**< \brief (DMAC_CHCTRLA) 12-beats burst length */
	#define DMCH_BURSTLEN_13BEAT 		(0xC<<24)   /**< \brief (DMAC_CHCTRLA) 13-beats burst length */
	#define DMCH_BURSTLEN_14BEAT 		(0xD<<24)   /**< \brief (DMAC_CHCTRLA) 14-beats burst length */
	#define DMCH_BURSTLEN_15BEAT 		(0xE<<24)   /**< \brief (DMAC_CHCTRLA) 15-beats burst length */
	#define DMCH_BURSTLEN_16BEAT 		(0xF<<24)   /**< \brief (DMAC_CHCTRLA) 16-beats burst length */
	#define DMCH_THRESHOLD_1BEAT		(0x0<<28)   /**< \brief (DMAC_CHCTRLA) Destination write starts after each beat source address read */
	#define DMCH_THRESHOLD_2BEATS 		(0x1<<28)   /**< \brief (DMAC_CHCTRLA) Destination write starts after 2-beats source address read */
	#define DMCH_THRESHOLD_4BEATS 		(0x2<<28)   /**< \brief (DMAC_CHCTRLA) Destination write starts after 4-beats source address read */
	#define DMCH_THRESHOLD_8BEATS 		(0x3<<28)   /**< \brief (DMAC_CHCTRLA) Destination write starts after 8-beats source address read */

	#define	DMCH_CMD_NOACT      		(0x0<<0)   /**< \brief (DMAC_CHCTRLB) No action */
	#define	DMCH_CMD_SUSPEND    		(0x1<<0)   /**< \brief (DMAC_CHCTRLB) Channel suspend operation */
	#define	DMCH_CMD_RESUME     		(0x2<<0)   /**< \brief (DMAC_CHCTRLB) Channel resume operation */

	#define	DMCH_PRILVL_LVL0   			(0x0<<0)   /**< \brief (DMAC_CHPRILVL) Channel Priority Level 0 (Lowest Level) */
	#define	DMCH_PRILVL_LVL1   			(0x1<<0)   /**< \brief (DMAC_CHPRILVL) Channel Priority Level 1 */
	#define	DMCH_PRILVL_LVL2   			(0x2<<0)   /**< \brief (DMAC_CHPRILVL) Channel Priority Level 2 */
	#define	DMCH_PRILVL_LVL3   			(0x3<<0)   /**< \brief (DMAC_CHPRILVL) Channel Priority Level 3 */

	#define DMCH_EVACT_NOACT    		(0<<0)		/**< \brief (DMAC_CHEVCTRL) No action */
	#define DMCH_EVACT_TRIG     		(1<<0)		/**< \brief (DMAC_CHEVCTRL) Transfer and periodic transfer trigger */
	#define DMCH_EVACT_CTRIG    		(2<<0)		/**< \brief (DMAC_CHEVCTRL) Conditional transfer trigger */
	#define DMCH_EVACT_CBLOCK   		(3<<0)		/**< \brief (DMAC_CHEVCTRL) Conditional block transfer */
	#define DMCH_EVACT_SUSPEND  		(4<<0)		/**< \brief (DMAC_CHEVCTRL) Channel suspend operation */
	#define DMCH_EVACT_RESUME   		(5<<0)		/**< \brief (DMAC_CHEVCTRL) Channel resume operation */
	#define DMCH_EVACT_SSKIP    		(6<<0)		/**< \brief (DMAC_CHEVCTRL) Skip next block suspend action */
	#define DMCH_EVACT_INCPRI    		(7<<0)		/**< \brief (DMAC_CHEVCTRL) Increase priority */

	#define DMCH_EVOMODE_DEFAULT 		(0x0<<4)   	/**< \brief (DMAC_CHEVCTRL) Block event output selection. Refer to BTCTRL.EVOSEL for available selections. */
	#define DMCH_EVOMODE_TRIGACT 		(0x1<<4)   	/**< \brief (DMAC_CHEVCTRL) Ongoing trigger action */
	#define DMCH_EVIE      				(1<<6)    	/**< \brief (DMAC_CHEVCTRL) Channel Event Input Enable */
	#define DMCH_EVOE      				(1<<7)    	/**< \brief (DMAC_CHEVCTRL) Channel Event Output Enable */

	#define DMCH_TRIGSRC_DISABLE		(0x00 << 8)
	#define DMCH_TRIGSRC_RTC			(0x01 << 8)
	#define DMCH_TRIGSRC_DSU_DCC0		(0x02 << 8)
	#define DMCH_TRIGSRC_DSU_DCC1		(0x03 << 8)
	#define DMCH_TRIGSRC_SERCOM0_RX		(0x04 << 8)
	#define DMCH_TRIGSRC_SERCOM0_TX		(0x05 << 8)
	#define DMCH_TRIGSRC_SERCOM1_RX		(0x06 << 8)
	#define DMCH_TRIGSRC_SERCOM1_TX		(0x07 << 8)
	#define DMCH_TRIGSRC_SERCOM2_RX		(0x08 << 8)
	#define DMCH_TRIGSRC_SERCOM2_TX		(0x09 << 8)
	#define DMCH_TRIGSRC_SERCOM3_RX		(0x0A << 8)
	#define DMCH_TRIGSRC_SERCOM3_TX		(0x0B << 8)
	#define DMCH_TRIGSRC_SERCOM4_RX		(0x0C << 8)
	#define DMCH_TRIGSRC_SERCOM4_TX		(0x0D << 8)
	#define DMCH_TRIGSRC_SERCOM5_RX		(0x0E << 8)
	#define DMCH_TRIGSRC_SERCOM5_TX		(0x0F << 8)
	#define DMCH_TRIGSRC_SERCOM6_RX		(0x10 << 8)
	#define DMCH_TRIGSRC_SERCOM6_TX		(0x11 << 8)
	#define DMCH_TRIGSRC_SERCOM7_RX		(0x12 << 8)
	#define DMCH_TRIGSRC_SERCOM7_TX		(0x13 << 8)
	#define DMCH_TRIGSRC_CAN0			(0x14 << 8)
	#define DMCH_TRIGSRC_CAN1			(0x15 << 8)
	#define DMCH_TRIGSRC_TCC0_OVF   	(0x16 << 8)
	#define DMCH_TRIGSRC_TCC0_MC0   	(0x17 << 8)
	#define DMCH_TRIGSRC_TCC0_MC1   	(0x18 << 8)
	#define DMCH_TRIGSRC_TCC0_MC2   	(0x19 << 8)
	#define DMCH_TRIGSRC_TCC0_MC3   	(0x1A << 8)
	#define DMCH_TRIGSRC_TCC0_MC4   	(0x1B << 8)
	#define DMCH_TRIGSRC_TCC0_MC5   	(0x1C << 8)
	#define DMCH_TRIGSRC_TCC1_OVF   	(0x1D << 8)
	#define DMCH_TRIGSRC_TCC1_MC0   	(0x1E << 8)
	#define DMCH_TRIGSRC_TCC1_MC1   	(0x1F << 8)
	#define DMCH_TRIGSRC_TCC1_MC2   	(0x20 << 8)
	#define DMCH_TRIGSRC_TCC1_MC3   	(0x21 << 8)
	#define DMCH_TRIGSRC_TCC2_OVF   	(0x22 << 8)
	#define DMCH_TRIGSRC_TCC2_MC0   	(0x23 << 8)
	#define DMCH_TRIGSRC_TCC2_MC1		(0x24 << 8)
	#define DMCH_TRIGSRC_TCC2_MC2		(0x25 << 8)
	#define DMCH_TRIGSRC_TCC3_OVF   	(0x26 << 8)
	#define DMCH_TRIGSRC_TCC3_MC0   	(0x27 << 8)
	#define DMCH_TRIGSRC_TCC3_MC1		(0x28 << 8)
	#define DMCH_TRIGSRC_TCC4_OVF   	(0x29 << 8)
	#define DMCH_TRIGSRC_TCC4_MC0   	(0x2A << 8)
	#define DMCH_TRIGSRC_TCC4_MC1		(0x2B << 8)
	#define DMCH_TRIGSRC_TC0_OVF 		(0x2C << 8)
	#define DMCH_TRIGSRC_TC0_MC0 		(0x2D << 8)
	#define DMCH_TRIGSRC_TC0_MC1 		(0x2E << 8)
	#define DMCH_TRIGSRC_TC1_OVF 		(0x2F << 8)
	#define DMCH_TRIGSRC_TC1_MC0 		(0x30 << 8)
	#define DMCH_TRIGSRC_TC1_MC1 		(0x31 << 8)
	#define DMCH_TRIGSRC_TC2_OVF 		(0x32 << 8)
	#define DMCH_TRIGSRC_TC2_MC0 		(0x33 << 8)
	#define DMCH_TRIGSRC_TC2_MC1 		(0x34 << 8)
	#define DMCH_TRIGSRC_TC3_OVF 		(0x35 << 8)
	#define DMCH_TRIGSRC_TC3_MC0 		(0x36 << 8)
	#define DMCH_TRIGSRC_TC3_MC1 		(0x37 << 8)
	#define DMCH_TRIGSRC_TC4_OVF 		(0x38 << 8)
	#define DMCH_TRIGSRC_TC4_MC0 		(0x39 << 8)
	#define DMCH_TRIGSRC_TC4_MC1 		(0x3A << 8)
	#define DMCH_TRIGSRC_TC5_OVF 		(0x3B << 8)
	#define DMCH_TRIGSRC_TC5_MC0 		(0x3C << 8)
	#define DMCH_TRIGSRC_TC5_MC1 		(0x3D << 8)
	#define DMCH_TRIGSRC_TC6_OVF 		(0x3E << 8)
	#define DMCH_TRIGSRC_TC6_MC0		(0x3F << 8)
	#define DMCH_TRIGSRC_TC6_MC1		(0x40 << 8)
	#define DMCH_TRIGSRC_TC7_OVF		(0x41 << 8)
	#define DMCH_TRIGSRC_TC7_MC0		(0x42 << 8)
	#define DMCH_TRIGSRC_TC7_MC1		(0x43 << 8)
	#define DMCH_TRIGSRC_ADC0_RESRDY	(0x44 << 8)
	#define DMCH_TRIGSRC_ADC0_SEQ		(0x45 << 8)
	#define DMCH_TRIGSRC_ADC1_RESRDY	(0x46 << 8)
	#define DMCH_TRIGSRC_ADC1_SEQ		(0x47 << 8)
	#define DMCH_TRIGSRC_DAC_EMPTY_0	(0x48 << 8)
	#define DMCH_TRIGSRC_DAC_EMPTY_1	(0x49 << 8)
	#define DMCH_TRIGSRC_DAC_RESRDY_0	(0x4A << 8)
	#define DMCH_TRIGSRC_DAC_RESRDY_1	(0x4B << 8)
	#define DMCH_TRIGSRC_I2S_RX_0		(0x4C << 8)
	#define DMCH_TRIGSRC_I2S_RX_1		(0x4D << 8)
	#define DMCH_TRIGSRC_I2S_TX_0		(0x4E << 8)
	#define DMCH_TRIGSRC_I2S_TX_1		(0x4F << 8)
	#define DMCH_TRIGSRC_PCC_RX			(0x50 << 8)
	#define DMCH_TRIGSRC_AES_WR			(0x51 << 8)
	#define DMCH_TRIGSRC_AES_RD			(0x52 << 8)
	#define DMCH_TRIGSRC_QSPI_RX		(0x53 << 8)
	#define DMCH_TRIGSRC_QSPI_TX		(0x54 << 8)

	#define DMCH_TERR    				(1<<0)            /**< \brief (DMAC_CHINTENCLR) Channel Transfer Error Interrupt Enable */
	#define DMCH_TCMPL   				(1<<1)            /**< \brief (DMAC_CHINTENCLR) Channel Transfer Complete Interrupt Enable */
	#define DMCH_SUSP    				(1<<2)            /**< \brief (DMAC_CHINTENCLR) Channel Suspend Interrupt Enable */

	#define DMCH_PEND       			(1<<0)            /**< \brief (DMAC_CHSTATUS) Channel Pending */
	#define DMCH_BUSY       			(1<<1)            /**< \brief (DMAC_CHSTATUS) Channel Busy */
	#define DMCH_FERR       			(1<<2)            /**< \brief (DMAC_CHSTATUS) Channel Fetch Error */
	#define DMCH_CRCERR     			(1<<3)            /**< \brief (DMAC_CHSTATUS) Channel CRC Error */

	#define DMDSC_VALID       			(1<<0)            /**< \brief (DMAC_BTCTRL) Descriptor Valid */
	#define DMDSC_EVOSEL_DISABLE  		(0<<1)
	#define DMDSC_EVOSEL_BLOCK    		(1<<1)
	#define DMDSC_EVOSEL_BEAT     		(3<<1)
	#define DMDSC_BLOCKACT_NOACT  		(0<<3)
	#define DMDSC_BLOCKACT_INT    		(1<<3)
	#define DMDSC_BLOCKACT_SUSPEND		(2<<3)
	#define DMDSC_BLOCKACT_BOTH			(3<<3)
	#define DMDSC_BEATSIZE_BYTE   		(0<<8)
	#define DMDSC_BEATSIZE_HWORD  		(1<<8)
	#define DMDSC_BEATSIZE_WORD   		(2<<8)
	#define DMDSC_SRCINC      			(1<<10)           /**< \brief (DMAC_BTCTRL) Source Address Increment Enable */
	#define DMDSC_DSTINC      			(1<<11)           /**< \brief (DMAC_BTCTRL) Destination Address Increment Enable */
	#define DMDSC_STEPSEL_DST     		(0<<12)
	#define DMDSC_STEPSEL_SRC     		(1<<12)
	#define DMDSC_STEPSIZE_X1     		(0<<13)
	#define DMDSC_STEPSIZE_X2     		(1<<13)
	#define DMDSC_STEPSIZE_X4     		(2<<13)
	#define DMDSC_STEPSIZE_X8     		(3<<13)
	#define DMDSC_STEPSIZE_X16    		(4<<13)
	#define DMDSC_STEPSIZE_X32    		(5<<13)
	#define DMDSC_STEPSIZE_X64    		(6<<13)
	#define DMDSC_STEPSIZE_X128   		(7<<13)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//struct S_RTT
	//{
	//	AT91_REG		MR;
	//	AT91_REG		AR;
	//	const AT91_REG	VR;
	//	const AT91_REG	SR;
	//};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	 
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_ADC
	{
	};


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//struct S_MATRIX
	//{
	//	AT91_REG		MCFG[16];
	//	AT91_REG		SCFG[16];
	//	struct { AT91_REG A; AT91_REG B; } PRAS[18];
	//	AT91_REG		zreserve;
	//	AT91_REG		SYSIO;
	//	AT91_REG		zreserve1;
	//	AT91_REG		SMCNFCS;
	//};

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//struct S_EFC
	//{
	//	AT91_REG		FMR;
	//	AT91_REG		FCR;
	//	AT91_REG		FSR;
	//	AT91_REG		FRR;
	//};

	//typedef S_EFC S_EFC0, S_EFC1;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_GMAC
	{
		union ADR
		{
			u64		U;
			u16		W[4];
			
			struct 
			{ 
				AT91_REG	B;	//	Bottom Register 
				AT91_REG	T;	//	Top Register
			};
		};

		RW32		NCR;         /**< \brief Offset: 0x000 (R/W 32) Network Control Register */
		RW32		NCFGR;       /**< \brief Offset: 0x004 (R/W 32) Network Configuration Register */
		RO32		NSR;         /**< \brief Offset: 0x008 (R/  32) Network Status Register */
		RW32		UR;          /**< \brief Offset: 0x00C (R/W 32) User Register */
		RW32		DCFGR;       /**< \brief Offset: 0x010 (R/W 32) DMA Configuration Register */
		RW32		TSR;         /**< \brief Offset: 0x014 (R/W 32) Transmit Status Register */
		RW32		RBQB;        /**< \brief Offset: 0x018 (R/W 32) Receive Buffer Queue Base Address */
		RW32		TBQB;        /**< \brief Offset: 0x01C (R/W 32) Transmit Buffer Queue Base Address */
		RW32		RSR;         /**< \brief Offset: 0x020 (R/W 32) Receive Status Register */
		RW32		ISR;         /**< \brief Offset: 0x024 (R/W 32) Interrupt Status Register */
		RW32		IER;         /**< \brief Offset: 0x028 ( /W 32) Interrupt Enable Register */
		RW32		IDR;         /**< \brief Offset: 0x02C ( /W 32) Interrupt Disable Register */
		RO32		IMR;         /**< \brief Offset: 0x030 (R/  32) Interrupt Mask Register */
		RW32		MAN;         /**< \brief Offset: 0x034 (R/W 32) PHY Maintenance Register */
		RO32		RPQ;         /**< \brief Offset: 0x038 (R/  32) Received Pause Quantum Register */
		RW32		TPQ;         /**< \brief Offset: 0x03C (R/W 32) Transmit Pause Quantum Register */
		RW32		TPSF;        /**< \brief Offset: 0x040 (R/W 32) TX partial store and forward Register */
		RW32		RPSF;        /**< \brief Offset: 0x044 (R/W 32) RX partial store and forward Register */
		RW32		RJFML;       /**< \brief Offset: 0x048 (R/W 32) RX Jumbo Frame Max Length Register */
		RO8						z__Reserved1[0x34];
		RW32		HRB;         /**< \brief Offset: 0x080 (R/W 32) Hash Register Bottom [31:0] */
		RW32		HRT;         /**< \brief Offset: 0x084 (R/W 32) Hash Register Top [63:32] */
		ADR			SA[4];       /**< \brief Offset: 0x088 GmacSa groups */
		RW32		TIDM[4];     /**< \brief Offset: 0x0A8 (R/W 32) Type ID Match Register */
		RW32		WOL;         /**< \brief Offset: 0x0B8 (R/W 32) Wake on LAN */
		RW32		IPGS;        /**< \brief Offset: 0x0BC (R/W 32) IPG Stretch Register */
		RW32		SVLAN;       /**< \brief Offset: 0x0C0 (R/W 32) Stacked VLAN Register */
		RW32		TPFCP;       /**< \brief Offset: 0x0C4 (R/W 32) Transmit PFC Pause Register */
		RW32		SAMB1;       /**< \brief Offset: 0x0C8 (R/W 32) Specific Address 1 Mask Bottom [31:0] Register */
		RW32		SAMT1;       /**< \brief Offset: 0x0CC (R/W 32) Specific Address 1 Mask Top [47:32] Register */
		RO8						z__Reserved2[0xC];
		RW32		NSC;         /**< \brief Offset: 0x0DC (R/W 32) Tsu timer comparison nanoseconds Register */
		RW32		SCL;         /**< \brief Offset: 0x0E0 (R/W 32) Tsu timer second comparison Register */
		RW32		SCH;         /**< \brief Offset: 0x0E4 (R/W 32) Tsu timer second comparison Register */
		RO32		EFTSH;       /**< \brief Offset: 0x0E8 (R/  32) PTP Event Frame Transmitted Seconds High Register */
		RO32		EFRSH;       /**< \brief Offset: 0x0EC (R/  32) PTP Event Frame Received Seconds High Register */
		RO32		PEFTSH;      /**< \brief Offset: 0x0F0 (R/  32) PTP Peer Event Frame Transmitted Seconds High Register */
		RO32		PEFRSH;      /**< \brief Offset: 0x0F4 (R/  32) PTP Peer Event Frame Received Seconds High Register */
		RO8						z__Reserved3[0x8];
		RO32		OTLO;        /**< \brief Offset: 0x100 (R/  32) Octets Transmitted [31:0] Register */
		RO32		OTHI;        /**< \brief Offset: 0x104 (R/  32) Octets Transmitted [47:32] Register */
		RO32		FT;          /**< \brief Offset: 0x108 (R/  32) Frames Transmitted Register */
		RO32		BCFT;        /**< \brief Offset: 0x10C (R/  32) Broadcast Frames Transmitted Register */
		RO32		MFT;         /**< \brief Offset: 0x110 (R/  32) Multicast Frames Transmitted Register */
		RO32		PFT;         /**< \brief Offset: 0x114 (R/  32) Pause Frames Transmitted Register */
		RO32		BFT64;       /**< \brief Offset: 0x118 (R/  32) 64 Byte Frames Transmitted Register */
		RO32		TBFT127;     /**< \brief Offset: 0x11C (R/  32) 65 to 127 Byte Frames Transmitted Register */
		RO32		TBFT255;     /**< \brief Offset: 0x120 (R/  32) 128 to 255 Byte Frames Transmitted Register */
		RO32		TBFT511;     /**< \brief Offset: 0x124 (R/  32) 256 to 511 Byte Frames Transmitted Register */
		RO32		TBFT1023;    /**< \brief Offset: 0x128 (R/  32) 512 to 1023 Byte Frames Transmitted Register */
		RO32		TBFT1518;    /**< \brief Offset: 0x12C (R/  32) 1024 to 1518 Byte Frames Transmitted Register */
		RO32		GTBFT1518;   /**< \brief Offset: 0x130 (R/  32) Greater Than 1518 Byte Frames Transmitted Register */
		RO32		TUR;         /**< \brief Offset: 0x134 (R/  32) Transmit Underruns Register */
		RO32		SCF;         /**< \brief Offset: 0x138 (R/  32) Single Collision Frames Register */
		RO32		MCF;         /**< \brief Offset: 0x13C (R/  32) Multiple Collision Frames Register */
		RO32		EC;          /**< \brief Offset: 0x140 (R/  32) Excessive Collisions Register */
		RO32		LC;          /**< \brief Offset: 0x144 (R/  32) Late Collisions Register */
		RO32		DTF;         /**< \brief Offset: 0x148 (R/  32) Deferred Transmission Frames Register */
		RO32		CSE;         /**< \brief Offset: 0x14C (R/  32) Carrier Sense Errors Register */
		RO32		ORLO;        /**< \brief Offset: 0x150 (R/  32) Octets Received [31:0] Received */
		RO32		ORHI;        /**< \brief Offset: 0x154 (R/  32) Octets Received [47:32] Received */
		RO32		FR;          /**< \brief Offset: 0x158 (R/  32) Frames Received Register */
		RO32		BCFR;        /**< \brief Offset: 0x15C (R/  32) Broadcast Frames Received Register */
		RO32		MFR;         /**< \brief Offset: 0x160 (R/  32) Multicast Frames Received Register */
		RO32		PFR;         /**< \brief Offset: 0x164 (R/  32) Pause Frames Received Register */
		RO32		BFR64;       /**< \brief Offset: 0x168 (R/  32) 64 Byte Frames Received Register */
		RO32		TBFR127;     /**< \brief Offset: 0x16C (R/  32) 65 to 127 Byte Frames Received Register */
		RO32		TBFR255;     /**< \brief Offset: 0x170 (R/  32) 128 to 255 Byte Frames Received Register */
		RO32		TBFR511;     /**< \brief Offset: 0x174 (R/  32) 256 to 511Byte Frames Received Register */
		RO32		TBFR1023;    /**< \brief Offset: 0x178 (R/  32) 512 to 1023 Byte Frames Received Register */
		RO32		TBFR1518;    /**< \brief Offset: 0x17C (R/  32) 1024 to 1518 Byte Frames Received Register */
		RO32		TMXBFR;      /**< \brief Offset: 0x180 (R/  32) 1519 to Maximum Byte Frames Received Register */
		RO32		UFR;         /**< \brief Offset: 0x184 (R/  32) Undersize Frames Received Register */
		RO32		OFR;         /**< \brief Offset: 0x188 (R/  32) Oversize Frames Received Register */
		RO32		JR;          /**< \brief Offset: 0x18C (R/  32) Jabbers Received Register */
		RO32		FCSE;        /**< \brief Offset: 0x190 (R/  32) Frame Check Sequence Errors Register */
		RO32		LFFE;        /**< \brief Offset: 0x194 (R/  32) Length Field Frame Errors Register */
		RO32		RSE;         /**< \brief Offset: 0x198 (R/  32) Receive Symbol Errors Register */
		RO32		AE;          /**< \brief Offset: 0x19C (R/  32) Alignment Errors Register */
		RO32		RRE;         /**< \brief Offset: 0x1A0 (R/  32) Receive Resource Errors Register */
		RO32		ROE;         /**< \brief Offset: 0x1A4 (R/  32) Receive Overrun Register */
		RO32		IHCE;        /**< \brief Offset: 0x1A8 (R/  32) IP Header Checksum Errors Register */
		RO32		TCE;         /**< \brief Offset: 0x1AC (R/  32) TCP Checksum Errors Register */
		RO32		UCE;         /**< \brief Offset: 0x1B0 (R/  32) UDP Checksum Errors Register */
		RO8						z__Reserved4[0x8];
		RW32		TISUBN;      /**< \brief Offset: 0x1BC (R/W 32) 1588 Timer Increment [15:0] Sub-Nanoseconds Register */
		RW32		TSH;         /**< \brief Offset: 0x1C0 (R/W 32) 1588 Timer Seconds High [15:0] Register */
		RO8						z__Reserved5[0x4];
		RW32		TSSSL;       /**< \brief Offset: 0x1C8 (R/W 32) 1588 Timer Sync Strobe Seconds [31:0] Register */
		RW32		TSSN;        /**< \brief Offset: 0x1CC (R/W 32) 1588 Timer Sync Strobe Nanoseconds Register */
		RW32		TSL;         /**< \brief Offset: 0x1D0 (R/W 32) 1588 Timer Seconds [31:0] Register */
		RW32		TN;          /**< \brief Offset: 0x1D4 (R/W 32) 1588 Timer Nanoseconds Register */
		RW32		TA;          /**< \brief Offset: 0x1D8 ( /W 32) 1588 Timer Adjust Register */
		RW32		TI;          /**< \brief Offset: 0x1DC (R/W 32) 1588 Timer Increment Register */
		RO32		EFTSL;       /**< \brief Offset: 0x1E0 (R/  32) PTP Event Frame Transmitted Seconds Low Register */
		RO32		EFTN;        /**< \brief Offset: 0x1E4 (R/  32) PTP Event Frame Transmitted Nanoseconds */
		RO32		EFRSL;       /**< \brief Offset: 0x1E8 (R/  32) PTP Event Frame Received Seconds Low Register */
		RO32		EFRN;        /**< \brief Offset: 0x1EC (R/  32) PTP Event Frame Received Nanoseconds */
		RO32		PEFTSL;      /**< \brief Offset: 0x1F0 (R/  32) PTP Peer Event Frame Transmitted Seconds Low Register */
		RO32		PEFTN;       /**< \brief Offset: 0x1F4 (R/  32) PTP Peer Event Frame Transmitted Nanoseconds */
		RO32		PEFRSL;      /**< \brief Offset: 0x1F8 (R/  32) PTP Peer Event Frame Received Seconds Low Register */
		RO32		PEFRN;       /**< \brief Offset: 0x1FC (R/  32) PTP Peer Event Frame Received Nanoseconds */
		RO8						z__Reserved6[0x70];
		RO32		RLPITR;      /**< \brief Offset: 0x270 (R/  32) Receive LPI transition Register */
		RO32		RLPITI;      /**< \brief Offset: 0x274 (R/  32) Receive LPI Time Register */
		RO32		TLPITR;      /**< \brief Offset: 0x278 (R/  32) Receive LPI transition Register */
		RO32		TLPITI;      /**< \brief Offset: 0x27C (R/  32) Receive LPI Time Register */
	};
	
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	struct S_CMCC
	{
		RO32    TYPE;        /**< \brief Offset: 0x00 (R/  32) Cache Type Register */
		RW32    CFG;         /**< \brief Offset: 0x04 (R/W 32) Cache Configuration Register */
		RW32    CTRL;        /**< \brief Offset: 0x08 ( /W 32) Cache Control Register */
		RO32  	SR;          /**< \brief Offset: 0x0C (R/  32) Cache Status Register */
		RW32  	LCKWAY;      /**< \brief Offset: 0x10 (R/W 32) Cache Lock per Way Register */
		RO8					z__Reserved1[0xC];
		RW32  	MAINT0;      /**< \brief Offset: 0x20 ( /W 32) Cache Maintenance Register 0 */
		RW32  	MAINT1;      /**< \brief Offset: 0x24 ( /W 32) Cache Maintenance Register 1 */
		RW32  	MCFG;        /**< \brief Offset: 0x28 (R/W 32) Cache Monitor Configuration Register */
		RW32  	MEN;         /**< \brief Offset: 0x2C (R/W 32) Cache Monitor Enable Register */
		RW32  	MCTRL;       /**< \brief Offset: 0x30 ( /W 32) Cache Monitor Control Register */
		RO32    MSR;         /**< \brief Offset: 0x34 (R/  32) Cache Monitor Status Register */
	};

	#define CMCC_CEN		(1<<0)            /**< \brief (CMCC_CTRL) Cache Controller Enable */
	#define CMCC_CSTS       (1<<0)            /**< \brief (CMCC_SR) Cache Controller Status */

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

};


namespace HW
{
	//namespace PID
	//{
	//	enum {	MKPID(SUPC, 0),		MKPID(RSTC, 1),		MKPID(RTC, 2),		MKPID(RTT, 3),		MKPID(WDT, 4),		MKPID(PMC, 5),		MKPID(EEFC0, 6),	MKPID(EEFC1, 7),
	//			MKPID(UART0, 8),	MKPID(UART1, 9),	MKPID(SMC, 10),		MKPID(PIOA, 11),	MKPID(PIOB, 12),	MKPID(PIOC, 13),	MKPID(USART0, 14),	MKPID(USART1, 15),
	//			MKPID(HSMCI, 18),	MKPID(TWI0, 19),	MKPID(TWI1, 20),	MKPID(SPI, 21),		MKPID(SSC, 22),		MKPID(TC0, 23),		MKPID(TC1, 24),		MKPID(TC2, 25),		MKPID(TC3, 26),		MKPID(TC4, 27), 
	//			MKPID(TC5, 28),		MKPID(ADC, 29),		MKPID(DACC, 30),	MKPID(PWM, 31),		MKPID(CRCCU, 32),	MKPID(ACC, 33),		MKPID(UDP, 34) };
	//};



	MK_PTR(GCLK,		0x40001C00);
	MK_PTR(PM,			0x40000400);
	MK_PTR(MCLK,		0x40000800);
	MK_PTR(WDT,			0x40002000);
	MK_PTR(OSCCTRL,		0x40001000);
	MK_PTR(OSC32KCTRL,	0x40001400);

	MK_PTR(PIOA,	0x41008000);
	MK_PTR(PIOB,	0x41008080);
	MK_PTR(PIOC,	0x41008100);


	MK_PTR(NVMCTRL,	0x41004000);
	MK_PTR(RTC,		0x40002400);

	MK_PTR(USART0,	0x40003000);
	MK_PTR(USART1,	0x40003400);
	MK_PTR(USART2,	0x41012000);
	MK_PTR(USART3,	0x41014000);
	MK_PTR(USART4,	0x43000000);
	MK_PTR(USART5,	0x43000400);
	MK_PTR(USART6,	0x43000800);
	MK_PTR(USART7,	0x43000C00);

	MK_PTR(SPI0,	0x40003000);
	MK_PTR(SPI1,	0x40003400);
	MK_PTR(SPI2,	0x41012000);
	MK_PTR(SPI3,	0x41014000);
	MK_PTR(SPI4,	0x43000000);
	MK_PTR(SPI5,	0x43000400);
	MK_PTR(SPI6,	0x43000800);
	MK_PTR(SPI7,	0x43000C00);

	MK_PTR(I2C0,	0x40003000);
	MK_PTR(I2C1,	0x40003400);
	MK_PTR(I2C2,	0x41012000);
	MK_PTR(I2C3,	0x41014000);
	MK_PTR(I2C4,	0x43000000);
	MK_PTR(I2C5,	0x43000400);
	MK_PTR(I2C6,	0x43000800);
	MK_PTR(I2C7,	0x43000C00);

	MK_PTR(TCC0,	0x41016000);
	MK_PTR(TCC1,	0x41018000);
	MK_PTR(TCC2,	0x42000C00);
	MK_PTR(TCC3,	0x42001000);
	MK_PTR(TCC4,	0x43001000);

	MK_PTR(EIC,		0x40002800);
	MK_PTR(EVSYS,	0x4100E000);

	MK_PTR(TC0,		0x40003800);
	MK_PTR(TC1,		0x40003C00);
	MK_PTR(TC2,		0x4101A000);
	MK_PTR(TC3,		0x4101C000);
	MK_PTR(TC4,		0x42001400);
	MK_PTR(TC5,		0x42001800);
	MK_PTR(TC6,		0x43001400);
	MK_PTR(TC7,		0x43001800);

	MK_PTR(DMAC, 	0x4100A000);

	MK_PTR(CMCC,	0x41006000);
	MK_PTR(SUPC,	0x40001800);

	MK_PTR(GMAC,	0x42000800);

	//MK_PTR(MATRIX,	0x400E0200);
	//MK_PTR(UART0,	0x400E0600);
	//MK_PTR(UART1,	0x400E0800);

	//MK_PTR(EFC1,	0x400E0C00);
	//MK_PTR(PIOC,	0x400E1200);

	//MK_PTR(RSTC,	0X400E1400);
	//MK_PTR(SUPC,	0X400E1410);
	//MK_PTR(RTT,		0X400E1430);

	//MK_PTR(SPI,		0X40008000);

	//MK_PTR(TC1,		0X40014000);

//	MK_PTR(PWM,		0X40094000);

	//MK_PTR(USART1,	0x40028000);

	//MK_PTR(ADC,		0X40038000);


	//MK_PTR(TWI0,	0x40018000);
	//MK_PTR(TWI1,	0x4001C000);


//	MK_PTR(UDP,		0x400A4000);



	inline void ResetWDT() { WDT->Reset(); }

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	inline bool RamCheck(void *ptr)
	{
		//u32 v = (u32)ptr;

		return ((u32)ptr & ~0x20003FFF) == 0; //(v >= 0x20000000 && v < 0x20004000);

	};

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	inline bool RomCheck(void *ptr)
	{
		//u32 v = (u32)ptr;

		return ((u32)ptr & ~0x1FFFF) == 0; //(v >= 0x000000 && v < 0x20000);

	};

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}; // namespace HW

extern T_HW::AT91_IHP VectorTableInt[16];
extern T_HW::AT91_IHP VectorTableExt[137];

extern T_HW::DMADESC DmaTable[32];
extern T_HW::DMADESC DmaWRB[32];

#undef MK_PTR
#undef MKPID



#endif // SAMD21_H__09_04_2019__08_14
