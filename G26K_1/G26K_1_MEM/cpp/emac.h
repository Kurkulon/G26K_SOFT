#ifndef EMAC_H__02_03_2015__16_38
#define EMAC_H__02_03_2015__16_38

#include "types.h"

extern bool	InitEMAC();
extern void	UpdateEMAC();
//extern bool EMAC_SendData(void *pData, u16 length);

inline u16 GetIpID() {extern u16 txIpID; return ReverseWord(txIpID++); }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline bool EmacIsConnected()
{
	extern bool emacConnected;
	return emacConnected;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct MAC
{
	u32 B;
	u16 T;

	inline void operator=(const MAC &v) { B = v.B; T = v.T; }
	inline bool operator==(const MAC &v) { return (B == v.B) && (T == v.T); }
	inline bool operator!=(const MAC &v) { return (B != v.B) || (T != v.T); }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53	

	struct Buf_Desc
	{
		u32		addr;
		u32		stat;
	};

#elif defined(CPU_XMC48)

	struct Receive_Desc
	{
		u32		stat;
		u32		ctrl;
		u32		addr1;
		u32		addr2;
	};

	struct Transmit_Desc
	{
		u32		stat;
		u32		ctrl;
		u32		addr1;
		u32		addr2;
	};

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthHdr
{
	MAC		dest;		/* Destination node		*/
	MAC		src;		/* Source node			*/
	u16		protlen;	/* Protocol or length		*/
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ArpHdr
{
	u16		hrd;		/* Format of hardware address	*/
	u16		pro;		/* Format of protocol address	*/
	byte	hln;		/* Length of hardware address	*/
	byte	pln;		/* Length of protocol address	*/
	u16		op;			/* Operation			*/
	MAC		sha;		/* Sender hardware address	*/
	u32		spa;		/* Sender protocol address	*/
	MAC		tha;		/* Target hardware address	*/
	u32		tpa;		/* Target protocol address	*/
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//* IP Header structure

__packed struct IPheader
{
	byte	hl_v;		/* header length and version	*/
	byte	tos;		/* type of service		*/
	u16		len;		/* total length			*/
	u16		id;			/* identification		*/
	u16		off;		/* fragment offset field	*/
	byte	ttl;		/* time to live			*/
	byte	p;			/* protocol			*/
	u16		sum;		/* checksum			*/
	u32		src;		/* Source IP address		*/
	u32		dst;		/* Destination IP address	*/
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//* Preudo IP Header
__packed struct IPPseudoheader
{
	u32		srcAdr;	/* Source IP address		*/
	u32		dstAdr;	/* Destination IP address	*/
	byte   	zero;
	byte   	proto;
	u16		size;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//* ICMP echo header structure

__packed struct IcmpEchoHdr
{
	byte	type;       /* type of message */
	byte	code;       /* type subcode */
	u16		cksum;      /* ones complement cksum of struct */
	u16		id;         /* identifier */
	u16		seq;        /* sequence number */
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//* UDP header structure
__packed struct UdpHdr
{
	u16	src;	/* UDP source port		*/
	u16	dst;	/* UDP destination port		*/
	u16	len;	/* Length of UDP packet		*/
	u16	xsum;	/* Checksum			*/
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthArp
{
	EthHdr	eth;
	ArpHdr	arp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthIp
{
	EthHdr		eth;
	IPheader	iph;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthIcmp
{
	EthHdr		eth;
	IPheader	iph;
	IcmpEchoHdr	icmp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthUdp
{
	EthHdr		eth;
	IPheader	iph;
	UdpHdr		udp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct DhcpHdr
{
	byte	op;
	byte	htype;
	byte	hlen;
	byte	hops;

	u32		xid;
	u16		secs;
	u16		flags;

	u32		ciaddr;
	u32		yiaddr;
	u32		siaddr;
	u32		giaddr;

	MAC		chaddr;
	byte	pad_chaddr[10];

	char	sname[64];
	char	file[128];
	u32		magic;
	byte	options[230];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthDhcp
{
	EthHdr		eth; // 14
	IPheader	iph; // 20
	UdpHdr		udp; // 8
	DhcpHdr		dhcp;// 
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union EthPtr
{
	EthHdr	*eth;
	EthArp	*earp;
	EthIp	*eip;
	EthIcmp *eicmp;
	EthUdp	*eudp;
	EthDhcp *edhcp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthBuf
{
//	typedef	void (*CB)(EthBuf *b);

	EthBuf*		next;

//	CB			CallBack;

	u32			len;
	EthHdr		eth;

	EthBuf() : next(0), /*CallBack(0),*/ len(0) {}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthArpBuf : public EthBuf
{
	ArpHdr	arp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthIpBuf : public EthBuf
{
	IPheader	iph;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthIcmpBuf : public EthIpBuf
{
	IcmpEchoHdr	icmp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct EthUdpBuf : public EthIpBuf
{
	UdpHdr	udp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern bool TransmitEth(EthBuf *b);
extern bool TransmitIp(EthIpBuf *b);
extern bool TransmitFragIp(EthIpBuf *b);
extern bool TransmitUdp(EthUdpBuf *b);
extern bool TransmitFragUdp(EthUdpBuf *b, u16 dst);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // EMAC_H__02_03_2015__16_38
