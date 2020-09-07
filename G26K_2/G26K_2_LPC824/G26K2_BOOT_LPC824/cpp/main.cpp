#include "hardware.h"
#include "time.h"
#include "ComPort.h"
#include "crc16.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define US2CLK(x) (x*25)
#define MS2CLK(x) (x*25000)

#define SGUID	0x0A89D55DD5274785 
#define MGUID	0x9119CC18AC79DE35
#define FRDY 1
#define FCMDE 2
#define FLOCKE 4
#define PAGESIZE 64
#define PAGEDWORDS (PAGESIZE>>2)
#define PAGES_IN_SECTOR 16
#define SECTORSIZE (PAGESIZE*PAGES_IN_SECTOR)
#define SECTORDWORDS (SECTORSIZE>>2)
#define PLANESIZE 0x8000
#define START_SECTOR 8
#define START_PAGE (START_SECTOR*PAGES_IN_SECTOR)

#define BOOTSIZE (SECTORSIZE*START_SECTOR)
#define FLASH0 0x000000
#define FLASH_START (FLASH0+BOOTSIZE)

//#define FLASH1 (FLASH0+PLANESIZE)

#define FLASH_END (FLASH0+PLANESIZE)
#define FLASH_SIZE (FLASH_END-FLASH_START)

#define IAP_LOCATION 0X1FFF1FF1
static u32 command_param[5];
static u32 status_result[4];
typedef void (*IAP)(unsigned int [],unsigned int[]);
#define iap_entry ((void(*)(u32[],u32[]))IAP_LOCATION)
//#define iap_entry ((IAP)IAP_LOCATION);

enum IAP_STATUS { CMD_SUCCESS = 0,  INVALID_COMMAND,  SRC_ADDR_ERROR,  DST_ADDR_ERROR,  SRC_ADDR_NOT_MAPPED,  DST_ADDR_NOT_MAPPED,  COUNT_ERROR,  INVALID_SECTOR,  SECTOR_NOT_BLANK, 
 SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION, COMPARE_ERROR, BUSY, ERR_ISP_IRC_NO_POWER , ERR_ISP_FLASH_NO_POWER,  ERR_ISP_FLASH_NO_CLOCK  };

const unsigned __int64 masterGUID = MGUID;
const unsigned __int64 slaveGUID = SGUID;

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

static bool run;
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
	u32 func;

	union
	{
		struct { u32 flashLen;  u16 align; u16 crc; } F01; // Get Flash CRC
		struct { u32 padr; u32 page[PAGEDWORDS]; u16 align; u16 crc; } F02; // Write page
		struct { u16 align; u16 crc; } F03; // Exit boot loader
	};
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct RspMes
{
	u32 len;
	u32 func;

	union
	{
		struct { u32 flashLen; u16 flashCRC; u16 crc; } F01;
		struct { u32 padr; u32 status; u16 align; u16 crc; } F02;
		struct { u16 align; u16 crc; } F03;							// Exit boot loader
	};
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool IAP_PrepareSector(u32 sector)
{
	sector += START_SECTOR;

	command_param[0] = 50;
	command_param[1] = sector;
	command_param[2] = sector;
	command_param[3] = 0;

	iap_entry(command_param, status_result);

	return status_result[0] == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool IAP_WritePage(u32 pagenum, u32 *pbuf)
{
	command_param[0] = 51;
	command_param[1] = FLASH_START + pagenum*PAGESIZE;
	command_param[2] = (u32)pbuf;
	command_param[3] = PAGESIZE;
	command_param[4] = 0;

	iap_entry(command_param, status_result);

	return status_result[0] == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool IAP_WriteSector(u32 sector, u32 *pbuf)
{
	command_param[0] = 51;
	command_param[1] = FLASH_START + sector*SECTORSIZE;
	command_param[2] = (u32)pbuf;
	command_param[3] = SECTORSIZE;
	command_param[4] = 0;

	iap_entry(command_param, status_result);

	return status_result[0] == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool IAP_EraseSector(u32 sector)
{
	sector += START_SECTOR;

	command_param[0] = 52;
	command_param[1] = sector;
	command_param[2] = sector;
	command_param[3] = 0;

	iap_entry(command_param, status_result);

	return status_result[0] == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool IAP_ErasePage(u32 pagenum)
{
	pagenum += START_PAGE;

	command_param[0] = 59;
	command_param[1] = pagenum;
	command_param[2] = pagenum;
	command_param[3] = 0;

	iap_entry(command_param, status_result);

	return status_result[0] == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool VerifyPage(u32 pagenum, u32 *pbuf)
{
	bool c = true;

	u32 *p = (u32*)(FLASH_START + pagenum*PAGESIZE);

	for (u32 i = 0; i < PAGEDWORDS; i++)
	{
		if (p[i] != pbuf[i])
		{
			c = false;
			break;
		};
	};

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool WritePage(u32 pagenum, u32 *pbuf)
{
	u32 sector = pagenum/PAGES_IN_SECTOR;

	if ((pagenum & (PAGES_IN_SECTOR-1)) == 0)
	{
		if (!IAP_PrepareSector(sector))
		{
			return false;
		};
		if (!IAP_EraseSector(sector))
		{
			return false;
		};
	};

	if (!IAP_PrepareSector(sector))
	{
		return false;
	};

	if (!IAP_WritePage(pagenum, pbuf))
	{
		return false;
	};

	if (!VerifyPage(pagenum, pbuf))
	{
		return false;
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool VerifySector(u32 sector, u32 *pbuf)
{
	bool c = true;

	u32 *p = (u32*)(FLASH_START + sector*SECTORSIZE);

	for (u32 i = 0; i < SECTORDWORDS; i++)
	{
		if (p[i] != pbuf[i])
		{
			c = false;
			break;
		};
	};

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool WriteSector(u32 sector, u32 *pbuf)
{
	if (!VerifySector(sector, pbuf))
	{
		if (!IAP_PrepareSector(sector))
		{
			return false;
		};
		if (!IAP_EraseSector(sector))
		{
			return false;
		};

		if (!IAP_PrepareSector(sector))
		{
			return false;
		};

		if (!IAP_WriteSector(sector, pbuf))
		{
			return false;
		};

		if (!VerifySector(sector, pbuf))
		{
			return false;
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 bufSector[SECTORDWORDS];
static u32 pageMask = 0;
static u32 curSector = ~0;

static bool WritePageBuf(u32 pagenum, u32 *pbuf)
{
	u32 sector = pagenum/PAGES_IN_SECTOR;

	if (curSector != sector)
	{
		pageMask = 0;
		curSector = sector;
		
		u32 *p = bufSector;

		for (u32 i = ArraySize(bufSector); i > 0; i--)
		{
			*p++ = ~0;
		};
	};

	pagenum &= (PAGES_IN_SECTOR-1);

	if ((pageMask & (1<<pagenum)) == 0)
	{
		u32 *p = bufSector + pagenum*PAGEDWORDS;

		for (u32 i = PAGEDWORDS; i > 0; i--)
		{
			*p++ = *pbuf++;
		};

		pageMask |= 1<<pagenum;
	};

	if ((pageMask & (PAGES_IN_SECTOR-1)) == (PAGES_IN_SECTOR-1))
	{
		if (!WriteSector(sector, bufSector))
		{
			return false;
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool HandShake()
{
	static ReqHS req;
	static RspHS rsp;

	static ComPort::WriteBuffer wb = { false, sizeof(req), &req };

	static ComPort::ReadBuffer rb = { false, 0, sizeof(rsp), &rsp };

	req.guid = slaveGUID;
	req.crc = GetCRC16(&req, sizeof(req)-2);

	com.Connect(0, 1562500, 0);

	bool c = false;

	for (byte i = 0; i < 2; i++)
	{
		com.Read(&rb, MS2CLK(100), US2CLK(100));

		HW::GPIO->BSET(12);

		while(com.Update())
		{
			HW::ResetWDT();
		};

		HW::GPIO->BCLR(12);

		c = (rb.recieved && rb.len == sizeof(RspHS) && GetCRC16(rb.data, rb.len) == 0 && rsp.guid == masterGUID);

		if (c)
		{
			com.Write(&wb);

			while(com.Update()) { HW::ResetWDT() ; };

			break;
		};
	};

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request_01_GetFlashCRC(ReqMes &req, RspMes &rsp)
{

	//if (req.F01.flashLen > FLASH_SIZE) { req.F01.flashLen = FLASH_SIZE; };

	LIM(req.F01.flashLen, FLASH_SIZE);

	rsp.func = req.func;
	rsp.F01.flashLen = req.F01.flashLen;
	rsp.F01.flashCRC = GetCRC16((void*)FLASH_START, req.F01.flashLen);
	rsp.F01.crc = GetCRC16(&rsp.func, sizeof(rsp.F01) - 2 + sizeof(rsp.func));
	rsp.len = sizeof(rsp.F01) + sizeof(rsp.func);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request_02_WritePage(ReqMes &req, RspMes &rsp)
{
	bool c = false;

	if (req.len == (sizeof(req.F02) + sizeof(req.func)))
	{
		c = WritePage(req.F02.padr/PAGESIZE, req.F02.page);
	};

	rsp.func = req.func;
	rsp.F02.padr = req.F02.padr;
	rsp.F02.status = (c) ? 1 : 0;
	rsp.F02.align = ~req.F02.padr;
	rsp.F02.crc = GetCRC16(&rsp.func, sizeof(rsp.F02) - 2 + sizeof(rsp.func));
	rsp.len = sizeof(rsp.F02) + sizeof(rsp.func);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request_03_ExitBootLoader(ReqMes &req, RspMes &rsp)
{
	rsp.func = req.func;
	rsp.F03.align = 0x5555;
	rsp.F03.crc = GetCRC16(&rsp.func, sizeof(rsp.F03) - sizeof(rsp.F03.crc) + sizeof(rsp.func));
	rsp.len = sizeof(rsp.F03) + sizeof(rsp.func);

	return false;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request_Default(ReqMes &req, RspMes &rsp)
{
	rsp.func = req.func;
	rsp.F03.align = 0x5AAAA;
	rsp.F03.crc = GetCRC16(&rsp.func, sizeof(rsp.F03) - sizeof(rsp.F03.crc) + sizeof(rsp.func));
	rsp.len = sizeof(rsp.F03) + sizeof(rsp.func);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request_Handler(ReqMes &req, RspMes &rsp)
{
	bool c = false;

	switch (req.func)
	{
		case 1:		c = Request_01_GetFlashCRC(req, rsp);		break;
		case 2:  	c = Request_02_WritePage(req, rsp);			break;
		case 3:  	c = Request_03_ExitBootLoader(req, rsp);	break;
		default: 	c = Request_Default(req, rsp);				break;
	};

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


static void UpdateCom()
{
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;

	static ReqMes req;
	static RspMes rsp;

	static byte i = 0;

	static bool c = true;

	static TM32 tm;

	switch (i)
	{
		case 0:

			rb.data = &req.func;
			rb.maxLen = sizeof(req)-sizeof(req.len);
			
			com.Read(&rb, ~0, US2CLK(100));

			i++;

			break;

		case 1:

			if (!com.Update())
			{
				i++;
			};

			break;

		case 2:

			if (rb.recieved && rb.len > 0 && GetCRC16(rb.data, rb.len) == 0)
			{
				req.len = rb.len;

				c = Request_Handler(req, rsp);

				i++;
			}
			else
			{
				i = 0;
			};

			break;

		case 3:

			//while(!tm.Check(2)) ;

			wb.data = &rsp.func;
			wb.len = rsp.len;

			com.Write(&wb);

			i++;

			break;

		case 4:

			if (!com.Update())
			{
				i = 0;

				run = c;
			};

			break;

	};

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern "C" void _MainAppStart(u32 adr);

int main()
{
//	__breakpoint(0);

	HW::GPIO->BCLR(12);

	InitHardware();

	run = HandShake();

	while(run)
	{
		UpdateCom();

		HW::ResetWDT();
	};

//	__breakpoint(0);

	__disable_irq();

	_MainAppStart(FLASH0+BOOTSIZE);

	return FLASH0+BOOTSIZE;
}
