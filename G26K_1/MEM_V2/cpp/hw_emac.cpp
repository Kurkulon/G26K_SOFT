#ifdef WIN32

#include <winsock2.h>
#include <WS2TCPIP.H>
#include <conio.h>
#include <stdio.h>

#else

#include "core.h"

#endif

#include "HW_EMAC_DEF.h"
#include "hw_emac.h"
#include "xtrap.h"
#include "list.h"
#include "hardware.h"

#include <emac.h>
#include <EMAC_DEF.h>
#include <tftp.h>

//#pragma diag_suppress 546,550,177

//#pragma O3
//#pragma Otime

#ifndef _DEBUG
	static const bool __debug = false;
#else
	static const bool __debug = true;
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/* Net_Config.c */

#define OUR_IP_ADDR   	IP32(192, 168, 3, 234)
#define OUR_IP_MASK   	IP32(255, 255, 255, 0)
#define DHCP_IP_ADDR   	IP32(192, 168, 3, 254)
#define PHYA 0

static const MAC hwAdr = {0x12345678, 0x9ABC};

static const u16 udpInPort = SWAP16(66);
static const u16 udpOutPort = SWAP16(66);

inline bool HW_EMAC_RequestUDP(EthBuf* mb) { return RequestTrap(mb); }
//inline bool HW_EMAC_RequestUDP(EthBuf* mb) { return RequestTFTP(mb); }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
List<SysEthBuf> SysEthBuf::freeList;
List<HugeTx>	HugeTx::freeList;

static SysEthBuf	sysTxBuf[16];
static HugeTx		hugeTxBuf[8];

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

static bool HW_EMAC_Init();

#ifdef CPU_SAME53	
	inline void ResetPHY()	{ HW::PIOC->DIRSET = (1 << 15); HW::PIOC->BCLR(15); };
	inline void EnablePHY() { HW::PIOC->DIRSET = (1 << 15); HW::PIOC->BSET(15); };
#elif defined(CPU_XMC48)
	inline void ResetPHY()	{ HW::P2->BCLR(10); };
	inline void EnablePHY() { HW::P2->BSET(10); };
#endif


#else

	//bool CheckStatusUDP(u32 stat) { return true; }
	//u16 HW_EMAC_GetUdpInPort()	{ return udpInPort;		}
	//u16 HW_EMAC_GetUdpOutPort()	{ return udpOutPort;	}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*--------------------------- init_ethernet ---------------------------------*/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <emac_imp.h>

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool HW_EMAC_Init()
{
#ifndef WIN32

	using namespace HW;
	
	/* Initialize the GMAC ethernet controller. */
	u32 id1,id2;

#ifdef CPU_SAME53	

	HW::MCLK->AHBMASK |= AHB_GMAC;
	HW::MCLK->APBCMASK |= APBC_GMAC;

	HW::GMAC->NCR = 0;

	HW::PIOA->SetWRCONFIG((0xF<<12)|(0x1F<<17),	PORT_PMUX(11)|PORT_DRVSTR|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX);
	HW::PIOC->SetWRCONFIG((1<<20),				PORT_PMUX(11)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX);

	HW::PIOC->DIRSET = (1<<15);
	HW::PIOC->SET((1<<15));

	HW::GMAC->NCFGR = GMAC_CLK_MCK_64; // MDC CLock MCK/48

#elif defined(CPU_XMC48)

    //Enable ETH0 peripheral clock

	HW::ETH_Enable();

    HW::PORT15->PDISC &= ~(PORT15_PDISC_PDIS8_Msk | PORT15_PDISC_PDIS9_Msk);
	EnablePHY();
	HW::ETH0_CON->CON = CON_INFSEL|CON_RXD0(0)|CON_RXD1(0)|CON_CLK_RMII(0)|CON_CRS_DV(2)|CON_RXER(0)|CON_MDIO(1);

	HW::ETH0->BUS_MODE |= ETH_BUS_MODE_SWR_Msk;

	while (HW::ETH0->BUS_MODE & ETH_BUS_MODE_SWR_Msk) HW::WDT->Update();

#endif

	EnableMDI();
  
	/* Put the DM9161 in reset mode */
	WritePHY(PHY_REG_BMCR, BMCR_RESET);

  /* Wait for hardware reset to end. */
	for (u32 tout = 0; tout < 0x800000; tout++)
	{
		if (!(ReadPHY(PHY_REG_BMCR) & BMCR_RESET))
		{
			break; /* Reset complete */
		};

		HW::WDT->Update();
	};

	WritePHY(PHY_REG_BMCR,		BMCR_ANENABLE|BMCR_FULLDPLX);
	WritePHY(PHY_REG_ANAR,		ANAR_NPAGE|ANAR_100FULL|ANAR_100HALF|ANAR_10FULL|ANAR_10HALF|ANAR_CSMA);
	WritePHY(PHY_REG_DRC,		DRC_PLL_OFF);
	WritePHY(PHY_REG_OMSO,		OMSO_RMII_OVERRIDE);
	WritePHY(PHY_REG_EXCON,		EXCON_EDPD_EN);
	WritePHY(PHY_REG_PHYCON1,	0);
	WritePHY(PHY_REG_PHYCON2,	PHYCON2_HP_MDIX|PHYCON2_JABBER_EN|PHYCON2_POWER_SAVING);

#endif

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++





