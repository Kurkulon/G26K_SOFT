#ifdef WIN32

#include <winsock2.h>
#include <WS2TCPIP.H>
#include <conio.h>
#include <stdio.h>

static SOCKET	lstnSocket;

static HANDLE handleTxThread;

DWORD txThreadCount = 0;

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

//#pragma diag_suppress 546,550,177

#pragma O3
#pragma Otime


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/* Net_Config.c */

#define OUR_IP_ADDR   	IP32(192, 168, 3, 234)
#define OUR_IP_MASK   	IP32(255, 255, 255, 0)
#define DHCP_IP_ADDR   	IP32(192, 168, 3, 254)

static const MAC _hwAdr = {0x12345678, 0x9ABC};
//static const MAC hwBroadCast = {0xFFFFFFFF, 0xFFFF};
//static const u32 ipAdr = OUR_IP_ADDR;//IP32(192, 168, 10, 1);
//static const u32 ipMask = IP32(255, 255, 255, 0);

static const u16 udpInPort = SWAP16(66);
static const u16 udpOutPort = SWAP16(66);


//bool EmacIsConnected() { return emacConnected; }

/* Local variables */


//enum	StateEM { LINKING, CONNECTED };	

//StateEM stateEMAC = LINKING;

static byte linkState = 0;


//u32 trp[4] = {-1};

//u16  txIpID = 0;

byte		HW_EMAC_GetAdrPHY()		{ return PHYA;			} 
const MAC&	HW_EMAC_GetHwAdr()		{ return _hwAdr;		}
u32  		HW_EMAC_GetIpAdr()		{ return OUR_IP_ADDR;	}
u32  		HW_EMAC_GetIpMask()		{ return OUR_IP_MASK;	}
u32  		HW_EMAC_GetDhcpIpAdr()	{ return DHCP_IP_ADDR;	}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32


#ifdef CPU_SAME53	
	inline void EnableMDI() { HW::GMAC->NCR |= GMAC_MPE; }
	inline void DisableMDI() { HW::GMAC->NCR &= ~GMAC_MPE; }
	inline void ResetPHY()	{ HW::PIOC->DIRSET = (1 << 15); HW::PIOC->BCLR(15); };
	inline void EnablePHY() { HW::PIOC->DIRSET = (1 << 15); HW::PIOC->BSET(15); };
#elif defined(CPU_XMC48)
	inline void EnableMDI() { /*HW::GMAC->NCR |= GMAC_MPE;*/ }
	inline void DisableMDI() { /*HW::GMAC->NCR &= ~GMAC_MPE;*/ }
	inline void ResetPHY() { HW::P2->BCLR(10); };
	inline void EnablePHY() { HW::P2->BSET(10); };
#endif

void HW_EMAC_StartLink() { linkState = 0; }

#else

	bool CheckStatusUDP(u32 stat) { return true; }

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*--------------------------- init_ethernet ---------------------------------*/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool HW_EMAC_Init()
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
#ifndef WIN32

bool HW_EMAC_UpdateLink()
{
	bool result = false;

	switch(linkState)
	{
		case 0:		

			ReqReadPHY(PHY_REG_BMSR);

			linkState++;

			break;

		case 1:

			if (IsReadyPHY())
			{
				if (ResultPHY() & BMSR_LINKST)
				{
					linkState++;
				}
				else
				{
					linkState = 0;
				};
			};

			break;

		case 2:

			ReqWritePHY(PHY_REG_BMCR, BMCR_ANENABLE|BMCR_FULLDPLX);

			linkState++;

			break;

		case 3:

			if (IsReadyPHY())
			{
//				ReqReadPHY(PHY_REG_BMSR);
				ReqReadPHY(PHY_REG_PHYCON1);

				linkState++;
			};

			break;

		case 4:

			if (IsReadyPHY())
			{
				if (ResultPHY() & PHYCON1_OP_MODE_MASK /*BMSR_LINKST*/)
				{
					#ifdef CPU_SAME53	
						HW::GMAC->NCFGR &= ~(GMAC_SPD|GMAC_FD);
					#elif defined(CPU_XMC48)
						HW::ETH0->MAC_CONFIGURATION &= ~(MAC_DM|MAC_FES);
					#endif

					if (ResultPHY() & PHYCON1_OP_MODE_100BTX /*ANLPAR_100*/)	// Speed 100Mbit is enabled.
					{
						#ifdef CPU_SAME53	
							HW::GMAC->NCFGR |= GMAC_SPD;
						#elif defined(CPU_XMC48)
							HW::ETH0->MAC_CONFIGURATION |= MAC_FES;
						#endif
					};

					if (ResultPHY() & 4 /*ANLPAR_DUPLEX*/)	//  Full duplex is enabled.
					{
						#ifdef CPU_SAME53	
							HW::GMAC->NCFGR |= GMAC_FD;
						#elif defined(CPU_XMC48)
							HW::ETH0->MAC_CONFIGURATION |= MAC_DM;
						#endif
					};

					result = true;

					linkState++;
				}
				else
				{
					linkState = 3;
				};
			};

			break;
	};

	return result;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool HW_EMAC_RequestUDP(EthBuf* mb)
{
	EthUdp *h = (EthUdp*)&mb->eth;

	bool c = false;

	if (mb != 0)
	{
		switch (h->udp.dst)
		{
			case udpInPort: c = RequestTrap(mb); break;
		};
	};

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif

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





