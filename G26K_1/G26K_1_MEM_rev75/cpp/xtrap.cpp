#include "xtrap.h"
#include "trap.h"
#include "trap_def.h"
#include "list.h"
#include "flash.h"
#include "vector.h"

#pragma diag_suppress 550,177

#pragma O3
#pragma Otime

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct TrapReq
{
	TrapReq	*next;
	
	MAC		srcHA;
	u32		srcIP;
	u16		srcPort;
	u16		dlen;

	Trap	trap;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static TrapReq  traps[10];

static List<TrapReq> freeTrapList;
static List<TrapReq> reqTrapList;

static List<SmallTx> txList;
//static List<SmallTx> txFree;

static MAC ComputerEmacAddr = {0,0};	// Our Ethernet MAC address and IP address
static u32 ComputerIpAddr	= IP32(192,168,3,254);
static u16 ComputerUDPPort	= 0;
static u32 ComputerOldIpAddr	= 0;
static u16 ComputerOldUDPPort	= 0;
bool ComputerFind = false;




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void HandleRxPacket(EthUdp *h, u32 stat)
//{
////	if((h->iph.hl_v & 0x0F) != 5) { return; }	// Если IP пакет имеет поля `OPTION`, явно не наш пакет
//
//	u16 l = (unsigned short)(ReverseWord(h->udp.len) - sizeof(UdpHdr));
//
//	bool recapture = false;
//
//	unsigned int i;
//
//	if (ComputerEmacAddr != h->eth.src || ComputerIpAddr != h->iph.src || ComputerUDPPort != ReverseWord(h->udp.src)) 
//	{
//		recapture = true;
//	};
//
//	if (recapture || !ComputerFind)
//	{
//		u32 new_ip = h->iph.src;
//		u32 old_ip = ComputerIpAddr;
//		u16 new_port = ReverseWord(h->udp.src);
//		u16 old_port = ComputerUDPPort;
//		
//		if (ComputerFind) 
//		{
//			TRAP_INFO_SendLostIP(new_ip, new_port);
//		}
//		else 
//		{
//			ComputerFind = true;
//		};
//		
//		ComputerEmacAddr = h->eth.src;
//		ComputerIpAddr  = h->iph.src;
//		ComputerUDPPort = h->udp.src;
//
//		TRAP_INFO_SendCaptureIP(old_ip, old_port);
//	}
//     
//	TRAP_HandleRxData((char*)(h) + sizeof(EthUdp), l);
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RequestTrap(EthUdp *h, u32 stat)
{
	if ((h->udp.len = ReverseWord(h->udp.len)) < 19) return;

	u16 len = h->udp.len - sizeof(h->udp);

	if (len > sizeof(Trap)) return;

	TrapReq* req = freeTrapList.Get();

	if (req == 0) return;

	req->srcHA = h->eth.src;
	req->srcIP = h->iph.src;
	req->srcPort = h->udp.src;
	req->dlen = len;

	EthTrap *t = (EthTrap*)h;

	__packed u32 *p = (u32*)&t->trap;
	u32 *d = (u32*)&req->trap;

	len = (len+3) >> 2;

	while (len > 0)	{ *d++ = *p++; len--; };

	reqTrapList.Add(req);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeSmallTxBuffer(EthBuf* b)
{
//	txFree.Add(b);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeHugeTxBuffer(EthBuf* b)
{
//	txFree.Add(b);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SendTrap(SmallTx *p)
{
	p->iph.off = 0;

	txList.Add(p);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SendFragTrap(SmallTx *p)
{
	txList.Add(p);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SmallTx* GetSmallTxBuffer()
{
	static byte		indSmallTx = 0;
	static SmallTx	smallTxBuf[8];

	SmallTx *p = &smallTxBuf[indSmallTx];

	if (p->len == 0)
	{
		p->len = 1;
		indSmallTx = (indSmallTx + 1) & 7;
		return p;
	}
	else
	{
		return 0;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

HugeTx* GetHugeTxBuffer()
{
	static byte		indHugeTx = 0;
	static HugeTx	hugeTxBuf[8];

	HugeTx *p = &hugeTxBuf[indHugeTx];

	if (p->len == 0)
	{
		indHugeTx = (indHugeTx + 1) & 7;
		return p;
	}
	else
	{
		return 0;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitTraps()
{
	for (u32 i = 0; i < ArraySize(traps); i++)
	{
		freeTrapList.Add(&traps[i]);
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateRequestTraps()
{
	static byte i = 0;
	static TrapReq *req = 0;

	switch(i)
	{
		case 0:

			req = reqTrapList.Get();

			if (req != 0)
			{
				i++;
			};

			break;

		case 1:

			if (!ComputerFind)
			{
				i = 3;
			}
			else if (ComputerEmacAddr != req->srcHA || ComputerIpAddr != req->srcIP || ComputerUDPPort != req->srcPort)
			{
				i = 2;
			}
			else
			{
				i = 5;
			};

			break;

		case 2:

			if (TRAP_INFO_SendLostIP(ReverseDword(req->srcIP), ReverseWord(req->srcPort)))
			{
				i++;
			};

			break;

		case 3:

			ComputerOldIpAddr	= ComputerIpAddr;
			ComputerOldUDPPort	= ComputerUDPPort;

			ComputerEmacAddr	= req->srcHA;
			ComputerIpAddr		= req->srcIP;
			ComputerUDPPort		= req->srcPort;


			ComputerFind = true;

			i++;

		case 4:

			i += (TRAP_INFO_SendCaptureIP(ReverseDword(ComputerOldIpAddr), ReverseWord(ComputerOldUDPPort))) ? 1 : 0;

			break;

		case 5:

			TRAP_HandleRxData(&req->trap, req->dlen);

			freeTrapList.Add(req);

			i = 0;

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSendTraps()
{
	static TM32 tm;

	if (!EmacIsConnected())
	{
		return;
	};

	SmallTx *t = txList.Get();

	if (t != 0)
	{
		t->eth.dest = ComputerEmacAddr;

		t->iph.dst = ComputerIpAddr;	

		if (t->iph.off == 0) { t->iph.id = GetIpID(); };

		TransmitFragUdp(t, ComputerUDPPort);
	};
	
	if (ComputerFind && tm.Check(500))
	{ 
		TRAP_CLOCK_SendMain();
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateTraps()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateRequestTraps()		);
		CALL( UpdateSendTraps()			);
		CALL( TRAP_Idle()				);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
