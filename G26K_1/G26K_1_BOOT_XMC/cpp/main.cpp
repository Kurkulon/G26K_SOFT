#include <string.h>
#include "core.h"
//#include "xtrap.h"
//#include "flash.h"
#include "time.h"
#include "ComPort.h"
#include "list.h"
#include "system.h"
#include "main.h"
#include "crc16.h"
#include "emac.h"
#include "tftp.h"

#pragma diag_suppress 546,550,177

#define US2CLK(x) ((x)*(MCK/1000000))
#define MS2CLK(x) ((x)*(MCK/1000))

#define SGUID	0x4D8603031E5143bf
#define MGUID	0x8A90D21F96D7D991		
#define FRDY 1
#define FCMDE 2
#define FLOCKE 4
#define PAGESIZE 256
#define PAGEDWORDS (PAGESIZE>>2)
#define PAGES_IN_SECTOR 64
#define SECTORSIZE (PAGESIZE*PAGES_IN_SECTOR)
#define SECTORDWORDS (SECTORSIZE>>2)
#define PLANESIZE 0x200000
#define START_SECTOR 4
#define START_PAGE (START_SECTOR*PAGES_IN_SECTOR)

#define BOOTSIZE 0x10000	//(SECTORSIZE*START_SECTOR)
#define FLASH0_ADR 0x0C000000
#define FLASH_START (FLASH0_ADR+BOOTSIZE)

//#define FLASH1 (FLASH0+PLANESIZE)

#define FLASH_END (FLASH0_ADR+PLANESIZE)

#define IAP_LOCATION 0X1FFF1FF1
static u32 command_param[5];
static u32 status_result[4];
typedef void (*IAP)(unsigned int [],unsigned int[]);
#define iap_entry ((void(*)(u32[],u32[]))IAP_LOCATION)
//#define iap_entry ((IAP)IAP_LOCATION);

const u32 sectorAdr[] = {0x010000, 0x020000, 0x040000, 0x080000, 0x0C0000, 0x100000, 0x140000, 0x180000, 0x1C0000, 0x200000 };

//enum IAP_STATUS { CMD_SUCCESS = 0,  INVALID_COMMAND,  SRC_ADDR_ERROR,  DST_ADDR_ERROR,  SRC_ADDR_NOT_MAPPED,  DST_ADDR_NOT_MAPPED,  COUNT_ERROR,  INVALID_SECTOR,  SECTOR_NOT_BLANK, 
// SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION, COMPARE_ERROR, BUSY, ERR_ISP_IRC_NO_POWER , ERR_ISP_FLASH_NO_POWER,  ERR_ISP_FLASH_NO_CLOCK  };

const unsigned __int64 masterGUID = MGUID;
const unsigned __int64 slaveGUID = SGUID;

volatile u32* const _CMD_553C = (u32*)(FLASH0_ADR + 0x553C);
volatile u32* const _CMD_5554 = (u32*)(FLASH0_ADR + 0x5554);
volatile u32* const _CMD_55F0 = (u32*)(FLASH0_ADR + 0x55F0);
volatile u32* const _CMD_55F4 = (u32*)(FLASH0_ADR + 0x55F4);
volatile u32* const _CMD_AAA8 = (u32*)(FLASH0_ADR + 0xAAA8);

static void CMD_ResetToRead()		{ *_CMD_5554 = 0xF0; }
static void CMD_EnterPageMode()		{ *_CMD_5554 = 0x50; }
static void CMD_ClearStatus()		{ *_CMD_5554 = 0xF5; }

static TM32 timeOut;

enum STATEWRFL { WRITE_WAIT = 0, WRITE_START, WRITE_ERASE_SECTOR, WRITE_PAGE, WRITE_PAGE_0, WRITE_OK, WRITE_ERROR, WRITE_FINISH, WRITE_INIT };

STATEWRFL state_write_flash = WRITE_WAIT;
u32 flash_write_error = 0;
u32 flash_write_ok = 0;

static MEMB memBuffer[64];

static List<MEMB> freeMemBuf;
static List<MEMB> writeFlBuf;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitFlashBuffer()
{
	for (byte i = 0; i < ArraySize(memBuffer); i++)
	{
		freeMemBuf.Add(&memBuffer[i]);
	};							  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

MEMB* AllocMemBuffer()
{
	return freeMemBuf.Get();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeMemBuffer(MEMB* wb)
{
	freeMemBuf.Add(wb);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool RequestFlashWrite(MEMB* b)
{
	if ((b != 0) && (b->flwb.dataLen > 0))
	{
		writeFlBuf.Add(b);

		return true;
	}
	else
	{
		return false;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma push
#pragma O3
#pragma Otime

static void CopyDataDMA(volatile void *src, volatile void *dst, u16 len)
{
	using namespace HW;

	register u32 t __asm("r0");

	HW::GPDMA1->DMACFGREG = 1;

	HW::GPDMA1_CH3->CTLL = DST_INC|SRC_INC|TT_FC(0)|DEST_MSIZE(0)|SRC_MSIZE(0);
	HW::GPDMA1_CH3->CTLH = BLOCK_TS(len);

	HW::GPDMA1_CH3->SAR = (u32)src;
	HW::GPDMA1_CH3->DAR = (u32)dst;
	HW::GPDMA1_CH3->CFGL = 0;
	HW::GPDMA1_CH3->CFGH = PROTCTL(1);

	HW::GPDMA1->CHENREG = 0x101<<3;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CheckDataComplete()
{
	return (HW::GPDMA1->CHENREG & (1<<3)) == 0;
}

#pragma pop

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 GetSectorAdrLen(u32 sadr, u32 *radr)
{
	sadr += BOOTSIZE;

	if (sadr >= PLANESIZE)
	{
		return 0;
	};

	u32 len = 0;

	for (u32 i = 0; i < (ArraySize(sectorAdr)-1); i++)
	{
		if (sadr >= sectorAdr[i] && sadr < sectorAdr[i+1])
		{
			sadr = sectorAdr[i] - BOOTSIZE;
			len = sectorAdr[i+1] - sectorAdr[i];

			break;
		};
	};

	if (len != 0 && radr != 0)
	{
		*radr = sadr;
	};

	return len;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CMD_LoadPage(__packed const u32* data, u16 len)
{ 
	u16 l = len&1;

	len >>= 1;

	while (len > 0)
	{
		*_CMD_55F0 = *(data++);
		*_CMD_55F4 = *(data++);
		len -= 1;
	};

	if (l)
	{
		*_CMD_55F0 = *(data++);
		*_CMD_55F4 = 0;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CMD_WritePage(u32 pa)
{
	*_CMD_5554 = 0xAA; *_CMD_AAA8 = 0x55; *_CMD_5554 = 0xA0; *((u32*)(FLASH_START+pa)) = 0xAA;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CMD_ErasePhysicalSector(u32 sa)
{
	*_CMD_5554 = 0xAA; *_CMD_AAA8 = 0x55; *_CMD_5554 = 0x80; *_CMD_5554 = 0xAA; *_CMD_AAA8 = 0x55; *((u32*)(FLASH_START+sa)) = 0x40;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CMD_ResumeProtection()	{ *_CMD_5554 = 0x5E; }


static ComPort com;

//static ComPort* actCom = 0;


struct FL
{
	u32 id;
	u32 size;
	u32 pageSize;
	u32 nbPlane;
	u32 planeSize;
};

static FL flDscr;

//static bool run = false;
static bool runCom = false;
static bool runEmac = false;

//static u32 crcErrors = 0;
//static u32 lenErrors = 0;
//static u32 reqErrors = 0;

//static u32 lens[300] = {0};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqHS { unsigned __int64 guid; u16 crc; };
__packed struct RspHS { unsigned __int64 guid; u16 crc; };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct ReqMes
{
	u32 len;
	
	union
	{
		struct { u32 func; u32 sadr; u32 len;				u16 align; u16 crc; }	F1; // Get CRC
		struct { u32 func; u32 sadr;						u16 align; u16 crc; }	F2; // Erase sector
		struct { u32 func; u32 padr; u32 page[PAGEDWORDS];	u16 align; u16 crc; }	F3; // Programm page
	};

};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct RspMes
{
	u32 len;

	union
	{
		struct { u32 func; u32 sadr; u32 len;	u16 sCRC;	u16 crc; }	F1; // Get CRC
		struct { u32 func; u32 sadr; u32 status; u16 align;	u16 crc; } 	F2; // Erase sector
		struct { u32 func; u32 padr; u32 status; u16 align;	u16 crc; } 	F3; // Programm page
	};
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool IAP_WritePage(u32 pa, u32 *pbuf)
{
	CMD_ResetToRead();

	CMD_ClearStatus();

	CMD_EnterPageMode();

	while ((HW::FLASH0->FSR & (FLASH_FSR_PFPAGE_Msk|FLASH_FSR_SQER_Msk)) == 0);// { HW::WDT->Update(); };

	CMD_LoadPage(pbuf, PAGEDWORDS);

	CMD_WritePage(pa);

	while ((HW::FLASH0->FSR & (FLASH_FSR_PROG_Msk|FLASH_FSR_SQER_Msk)) == 0);// { HW::WDT->Update(); };

	while (HW::FLASH0->FSR & FLASH_FSR_PBUSY_Msk);// { HW::WDT->Update(); };

	return (HW::FLASH0->FSR & (FLASH_FSR_VER_Msk|FLASH_FSR_SQER_Msk)) == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool REQ_WritePage(u32 pa, __packed u32 *pbuf)
{
	CMD_ResetToRead();

	CMD_ClearStatus();

	CMD_EnterPageMode();

	while ((HW::FLASH0->FSR & (FLASH_FSR_PFPAGE_Msk|FLASH_FSR_SQER_Msk)) == 0);// { HW::WDT->Update(); };

	CMD_LoadPage(pbuf, PAGEDWORDS);

	CMD_WritePage(pa);

	while ((HW::FLASH0->FSR & (FLASH_FSR_PROG_Msk|FLASH_FSR_SQER_Msk)) == 0);// { HW::WDT->Update(); };

	return (HW::FLASH0->FSR & (FLASH_FSR_PROG_Msk|FLASH_FSR_SQER_Msk)) == FLASH_FSR_PROG_Msk;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool IsFlashReady()
{
	return (HW::FLASH0->FSR & FLASH_FSR_PBUSY_Msk) == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool IsFlashReqOK()
{
	return (HW::FLASH0->FSR & (FLASH_FSR_VER_Msk|FLASH_FSR_SQER_Msk)) == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool IAP_EraseSector(u32 sa)
{
	CMD_ResetToRead();
	
	CMD_ClearStatus();

	CMD_ErasePhysicalSector(sa);

	while ((HW::FLASH0->FSR & (FLASH_FSR_ERASE_Msk|FLASH_FSR_SQER_Msk)) == 0);// { HW::WDT->Update(); };

	while (HW::FLASH0->FSR & FLASH_FSR_PBUSY_Msk);// { HW::WDT->Update(); };

	return (HW::FLASH0->FSR & (FLASH_FSR_VER_Msk|FLASH_FSR_SQER_Msk)) == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool REQ_EraseSector(u32 sa)
{
	CMD_ResetToRead();
	
	CMD_ClearStatus();

	CMD_ErasePhysicalSector(sa);

	while ((HW::FLASH0->FSR & (FLASH_FSR_ERASE_Msk|FLASH_FSR_SQER_Msk)) == 0);// { HW::WDT->Update(); };

	return (HW::FLASH0->FSR & (FLASH_FSR_ERASE_Msk|FLASH_FSR_SQER_Msk)) == FLASH_FSR_ERASE_Msk;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 FLASH_Read(u32 addr, byte *data, u32 size) 
{
	while (HW::FLASH0->FSR & FLASH_FSR_PFPAGE_Msk);

	addr += FLASH_START;

	if (addr >= FLASH_END)
	{
		return 0;
	};

	if ((addr + size) >= FLASH_END)
	{
		size = FLASH_END - addr;	
	};

	CopyDataDMA((void*)addr, data, size);

	while (!CheckDataComplete());

	return size;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitFlashWrite()
{
	state_write_flash = WRITE_INIT;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateWriteFlash()
{
	static u32 secStartAdr = ~0;
	static u32 secEndAdr = ~0;
	static MEMB *curFlwb = 0;
	static FLWB *flwb = 0;
	static u32 wadr = 0;
	static u32 wlen = 0;
	static __packed u32 *wdata = 0;

	switch(state_write_flash)
	{
		case WRITE_WAIT:

			curFlwb = writeFlBuf.Get();

			if (curFlwb != 0)
			{
				flwb = &curFlwb->flwb;
				wadr = flwb->adr;
				wlen = flwb->dataLen;
				wdata = (__packed u32*)(flwb->data + flwb->dataOffset);

				state_write_flash = WRITE_START;
			};

			break;

		case WRITE_START:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
			
			if (wadr >= secStartAdr &&  wadr < secEndAdr)
			{
				state_write_flash = WRITE_PAGE;
			}
			else
			{
				u32 len = GetSectorAdrLen(wadr, &secStartAdr);

				if (len != 0)
				{
					secEndAdr = secStartAdr + len - 1;

					if (!REQ_EraseSector(secStartAdr)) len = 0;
				};

				state_write_flash = (len != 0) ? WRITE_ERASE_SECTOR : WRITE_ERROR;
			};

			break;
	
		case WRITE_ERASE_SECTOR:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (IsFlashReady())
			{
				state_write_flash = (IsFlashReqOK()) ? WRITE_PAGE : WRITE_ERROR;
			};

			break;

		case WRITE_PAGE:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			state_write_flash = (REQ_WritePage(wadr, wdata)) ? WRITE_PAGE_0 : WRITE_ERROR;

			break;

		case WRITE_PAGE_0:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (IsFlashReady())
			{
				if (IsFlashReqOK())
				{
					wadr += PAGESIZE;
					wdata += PAGEDWORDS;

					if (wlen >= PAGESIZE)
					{
						wlen -= PAGESIZE;
					}
					else
					{
						wlen = 0;	
					};

					state_write_flash = (wlen > 0) ? WRITE_START : WRITE_OK;
				}
				else
				{
					state_write_flash = WRITE_ERROR;
				};
			};

			break;

		case WRITE_OK:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			flash_write_ok++;

			state_write_flash = WRITE_FINISH;

			break;

		case WRITE_ERROR:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			flash_write_error++;

			state_write_flash = WRITE_FINISH;

			break;
		
		case WRITE_FINISH:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			FreeMemBuffer(curFlwb);

			curFlwb = 0;
			flwb = 0;

			state_write_flash = WRITE_WAIT;

			break;

		case WRITE_INIT:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (curFlwb != 0) 
			{
				FreeMemBuffer(curFlwb);
			};

			flash_write_error = 0;
			flash_write_ok = 0;

			secStartAdr = ~0;
			secEndAdr = ~0;

			curFlwb = 0;
			flwb = 0;

			state_write_flash = WRITE_WAIT;

			break;

	}; // switch(state_write_flash)
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool HandShake()
{
	static ReqHS req;
	static RspHS rsp;

	static ComPort::WriteBuffer wb = { false, sizeof(req), &req };

	static ComPort::ReadBuffer rb = { false, 0, sizeof(rsp), &rsp };

	req.guid = slaveGUID;
	req.crc = GetCRC16(&req, sizeof(req)-2);

	com.Connect(ComPort::ASYNC, 1, 115200, 0, 1);

	static byte i = 0;

	static TM32 tm;

	bool c = false;

	tm.Reset();

	while (!tm.Check(200) && !c)
	{
		HW::WDT->Update();

		switch (i)
		{
			case 0:

				com.Read(&rb, MS2RT(100), US2RT(500));

				i++;

				break;

			case 1:

				if (!com.Update())
				{
					if (rb.recieved && rb.len == sizeof(RspHS) && GetCRC16(rb.data, rb.len) == 0 && rsp.guid == masterGUID)
					{
						com.Write(&wb);

						i++;
					}
					else
					{
						i = 0;
					};
				};

				break;

			case 2:

				if (!com.Update())
				{
					runCom = c = true;

					timeOut.Reset();
				};

				break;
		};

		UpdateEMAC();

		if (EmacIsEnergyDetected() && EmacIsCableNormal())
		{
			runEmac = c = true;
			timeOut.Reset();
		};
	};

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request_F1_GetCRC(ReqMes &req, RspMes &rsp)
{
	u32 len = 0;
	u32 adr = 0;

	if (req.len == sizeof(req.F1) && (HW::FLASH0->FSR & FLASH_FSR_PFPAGE_Msk) == 0)
	{
		len = GetSectorAdrLen(req.F1.sadr, &adr);

		if (len != 0)
		{
			if (len > req.F1.len) len = req.F1.len;

			rsp.F1.sCRC = GetCRC16((void*)(FLASH_START+adr), len);
		};
	}
	else
	{
		adr = ~0;
	};

	rsp.F1.func = req.F1.func;
	rsp.F1.sadr = adr;
	rsp.F1.len = len;
	rsp.F1.crc = GetCRC16(&rsp.F1, sizeof(rsp.F1) - 2);
	rsp.len = sizeof(rsp.F1);

	return false;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request_F2_EraseSector(ReqMes &req, RspMes &rsp)
{
	bool c = false;

	if (req.len == sizeof(req.F2))
	{
		c = true; //IAP_EraseSector(req.F2.sadr);
	};

	rsp.F2.func = req.F2.func;
	rsp.F2.sadr = req.F2.sadr;
	rsp.F2.status = c;
	rsp.F2.crc = GetCRC16(&rsp.F2, sizeof(rsp.F2) - 2);
	rsp.len = sizeof(rsp.F2);

	return false;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request_F3_WritePage(MEMB *mb, RspMes &rsp)
{
	ReqMes &req = *((ReqMes*)mb->flwb.data);

	bool c = false;

	if (req.len == sizeof(req.F3) && flash_write_error == 0)
	{
		mb->flwb.adr = req.F3.padr;
		mb->flwb.dataLen = sizeof(req.F3.page);
		mb->flwb.dataOffset = (byte*)req.F3.page - mb->flwb.data;
		
		c = RequestFlashWrite(mb);
	};

	rsp.F3.func = req.F3.func;
	rsp.F3.padr = req.F3.padr;
	rsp.F3.status = c;
	rsp.F3.crc = GetCRC16(&rsp.F3, sizeof(rsp.F3) - 2);
	rsp.len = sizeof(rsp.F3);

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestHandler(MEMB *mb, RspMes &rsp)
{
	ReqMes &req = *((ReqMes*)mb->flwb.data);

	bool c = false;

	switch (req.F1.func)
	{
		case 1: c = Request_F1_GetCRC(req, rsp);		break;
		case 2: c = Request_F2_EraseSector(req, rsp);	break;
		case 3: c = Request_F3_WritePage(mb, rsp);		break;
	};

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateCom()
{
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;

//	static ReqMes req;
	static RspMes rsp;

	static byte i = 0;

//	static bool c = true;

	static TM32 tm;

	static MEMB *mb = 0;
	static ReqMes *req = 0;

	switch (i)
	{
		case 0:

			mb = AllocMemBuffer();

			if (mb != 0)
			{
				req = (ReqMes*)mb->flwb.data;
				i++;
			};

		case 1:

			rb.data = &req->F1;
			rb.maxLen = sizeof(*req);
			
			com.Read(&rb, MS2RT(200), MS2RT(2));

			i++;

			break;

		case 2:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0 && GetCRC16(rb.data, rb.len) == 0)
				{
					req->len = rb.len;

					if (RequestHandler(mb, rsp))
					{
						mb = 0;
						req = 0;
					};

					timeOut.Reset();

					i++;
				}
				else
				{
					if (timeOut.Check(2000))
					{
						runCom = false;
					};

					i = 1;
				};
			};

			break;

		case 3:

			if (tm.Check(2))
			{
				wb.data = &rsp.F1;
				wb.len = rsp.len;

				com.Write(&wb);

				i++;
			};

			break;

		case 4:

			if (!com.Update())
			{
				i = (mb == 0) ? 0 : 1;
			};

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void WDT_Init()
{
	#ifdef CPU_SAME53	

		HW::MCLK->APBAMASK |= APBA_WDT;

		HW::WDT->CONFIG = WDT_WINDOW_CYC512|WDT_PER_CYC1024;
	
		#ifndef _DEBUG
		//HW::WDT->CTRLA = WDT_ENABLE|WDT_WEN|WDT_ALWAYSON;
		#else
		//HW::WDT->CTRLA = WDT_ENABLE|WDT_WEN|WDT_ALWAYSON;
		#endif

		while(HW::WDT->SYNCBUSY);

	#elif defined(CPU_XMC48)

		HW::WDT_Enable();

		HW::WDT->WLB = OFI_FREQUENCY/2;
		HW::WDT->WUB = (3 * OFI_FREQUENCY)/2;
		HW::SCU_CLK->WDTCLKCR = 0|SCU_CLK_WDTCLKCR_WDTSEL_OFI;

		#ifndef _DEBUG
//		HW::WDT->CTR = WDT_CTR_ENB_Msk|WDT_CTR_DSP_Msk;
		#else
//		HW::WDT->CTR = WDT_CTR_ENB_Msk;
		#endif

	#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern "C" void _MainAppStart(u32 adr);

int main()
{
	//__breakpoint(0);

	Init_time();
	RTT_Init();
	WDT_Init();

	InitFlashBuffer();
	
	InitEMAC();

	HandShake();

	HW::SCU_RESET->ResetEnable(PID_WDT); HW::SCU_CLK->CLKCLR = SCU_CLK_CLKCLR_WDTCDI_Msk; HW::SCU_CLK->ClockDisable(PID_WDT);

	while(runCom || runEmac)
	{
		HW::P5->BSET(7);

		UpdateWriteFlash();

		if (runCom)
		{
			UpdateCom();
		}
		else
		{
			UpdateEMAC();
			runEmac = TFTP_Idle();

			if (!TFTP_Connected() && timeOut.Check(10000))
			{
				runEmac = false;
			};
		};

		HW::P5->BCLR(7);
	};

	//__breakpoint(0);

	__disable_irq();

	CM4::SysTick->CTRL = 0;

	HW::Peripheral_Disable(PID_DMA0);
	HW::Peripheral_Disable(PID_DMA1);

	HW::Peripheral_Disable(PID_USIC0);
	HW::Peripheral_Disable(PID_USIC1);

	_MainAppStart(FLASH_START);

	return FLASH_START;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

