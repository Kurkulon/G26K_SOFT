#ifndef CORE_H__05_12_12__16_12
#define CORE_H__05_12_12__16_12

#define	CORETYPE_BF592

#define CLKIN 25000000
#define VRCTL_VALUE         0x0000
#define PLLCTL_VALUE        0x2000
#define PLLDIV_VALUE        0x0004
#define PLLLOCKCNT_VALUE    0x0000
#define PLLSTAT_VALUE       0x0000
#define RSICLK_DIV          0x0001

#define VCO_VALUE (CLKIN*((PLLCTL_VALUE>>9)&63))
#define SCLK (VCO_VALUE / (PLLDIV_VALUE & 15))
#define CCLK (VCO_VALUE >> ((PLLDIV_VALUE >> 4) & 3))
//#define MCK CCLK

#define MS2CLK(x) ((u32)(x*1.0*SCLK/1e3+0.5))
#define US2CLK(x) ((u32)(x*1.0*SCLK/1e6+0.5))
#define NS2CLK(x) ((u32)(x*1.0*SCLK/1e9+0.5))

#define MS2CCLK(x) ((u32)(x*1.0*CCLK/1e3+0.5))
#define US2CCLK(x) ((u32)(x*1.0*CCLK/1e6+0.5))
#define NS2CCLK(x) ((u32)(x*1.0*CCLK/1e9+0.5))

#include "bf592.h"

inline void ResetWDT() { *pWDOG_STAT = 0;}


#endif // CORE_H__05_12_12__16_12
