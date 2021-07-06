#ifndef XTRAP_H__12_03_20015__11_14
#define XTRAP_H__12_03_20015__11_14

#include "types.h"
#include "emac.h"
#include "trap_def.h"
#include "EMAC_DEF.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct SmallTx : public EthUdpBuf
{
	TrapHdr th;		// 9

	byte data[75];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct HugeTx : public SmallTx
{
	byte exdata[IP_MTU - sizeof(UdpHdr) - sizeof(TrapHdr) - 75];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void RequestTrap(EthUdp *h, u32 stat);
extern void InitTraps();
extern void UpdateTraps();

extern SmallTx*	GetSmallTxBuffer();
extern HugeTx*	GetHugeTxBuffer();
extern void SendTrap(SmallTx *p);
extern void SendFragTrap(SmallTx *p);
inline bool IsComputerFind() {	extern bool ComputerFind; return ComputerFind; }


#endif // TRAP_H__12_03_20015__11_14
