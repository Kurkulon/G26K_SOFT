#include "types.h"
#include "core.h"
//#include "at25df021.h"

#include <bfrom.h>
#include <ccblkfn.h>
#include <sysreg.h> 

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

section ("L1_code")

void initcode(ADI_BOOT_DATA* pBS)
{
    *pSIC_IWR0 = IRQ_PLL_WAKEUP;

	ADI_SYSCTRL_VALUES sysctrl = {	VRCTL_VALUE,
									PLLCTL_VALUE,		/* (25MHz CLKIN x (MSEL=16))::CCLK = 400MHz */
									PLLDIV_VALUE,		/* (400MHz/(SSEL=4))::SCLK = 100MHz */
									PLLLOCKCNT_VALUE,
									PLLSTAT_VALUE };


	bfrom_SysControl( SYSCTRL_WRITE | SYSCTRL_PLLCTL | SYSCTRL_PLLDIV, &sysctrl, 0);

	pBS->dFlags |= BFLAG_FASTREAD;
	*pSPI0_BAUD = 5;
	pBS->dClock = 5; /* required to keep dClock in pBS (-> ADI_BOOT_DATA) consistent */

    *pSIC_IWR0 = IWR0_ENABLE_ALL;

    sysreg_write(reg_LC0,0);
    sysreg_write(reg_LC1,0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

