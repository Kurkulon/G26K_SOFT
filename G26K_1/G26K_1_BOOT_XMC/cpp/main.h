#ifndef MAIN_H__27_03_2020__09_53
#define MAIN_H__27_03_2020__09_53

#include "types.h"
#include "emac.h"

#define VERSION			0x0101

extern u32 FLASH_Read(u32 addr, byte *data, u32 size);
//extern bool IAP_EraseSector(u32 sa);


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct FLWB
{
	u32		adr;
	u32 	dataLen;
	u32 	dataOffset;
	byte	data[0];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct MEMB
{
	MEMB	*next;

	union
	{
		byte	data[ETH_RX_BUF_SIZE];
		FLWB	flwb;
	};
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern MEMB* AllocMemBuffer();
extern void FreeMemBuffer(MEMB* wb);
extern bool RequestFlashWrite(MEMB* b);
extern void InitFlashWrite();

inline u32 GetFlashWriteError() { extern u32 flash_write_error; return flash_write_error; }
inline u32 GetFlashWriteOK() { extern u32 flash_write_ok; return flash_write_ok; }

#endif // MAIN_H__27_03_2020__09_53
