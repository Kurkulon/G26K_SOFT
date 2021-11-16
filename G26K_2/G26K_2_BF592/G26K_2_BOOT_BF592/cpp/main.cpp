#include "hardware.h"
#include "ComPort.h"
#include "CRC16.h"
#include "at25df021.h"
//#include "list.h"
#include <bfrom.h>

static ComPort com;

static u16 manReqWord = 0xAD00;
static u16 manReqMask = 0xFF00;

//static u16 numDevice = 1;
//static u16 verDevice = 0x101;

static u32 manCounter = 0;
static u32 err06 = 0;


static u32 timeOut = MS2RT(500);
static bool runMainApp = false;

static u16 flashCRC = 0;
static u32 flashLen = 0;
static u16 lastErasedBlock = ~0;
static bool flashOK = false;
static bool flashChecked = false;
static bool flashCRCOK = false;

static RTM32 tm32;

static void CheckFlash();

static byte buf[SECTOR_SIZE];

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RunMainApp()
{
	//if (!flashChecked) CheckFlash();

	//if (flashOK && flashCRCOK) bfrom_SpiBoot(FLASH_START_ADR, BFLAG_PERIPHERAL | BFLAG_NOAUTO | BFLAG_FASTREAD | BFLAG_TYPE3 | BAUD_RATE_DIVISOR, 0, 0);
	//
	//tm32.Reset(); timeOut = MS2RT(1000);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc_05(Req *r, u16 len, ComPort::WriteBuffer *wb)
{
	const ReqDsp05 *req = (ReqDsp05*)(&r->req);
	static RspDsp05 rsp;

	if (len < sizeof(ReqDsp05)/2) return  false;

	rsp.rw = req->rw;
	rsp.flashLen = flashLen;
	rsp.flashCRC = flashCRC;
	rsp.startAdr = FLASH_START_ADR;

	rsp.crc = GetCRC16(&rsp, sizeof(rsp)-2);

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	FreeReq(r);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc_06(Req *r, u16 len, ComPort::WriteBuffer *wb)
{
	const ReqDsp06 *req = (ReqDsp06*)(&r->req);
	static RspDsp06 rsp;

	u16 xl = req->len + sizeof(ReqDsp06) - sizeof(req->data);

	if (len < xl/2) return  false;

	FlashWriteReq(r);

	rsp.res = GetLastError();

	rsp.rw = req->rw;

	rsp.crc = GetCRC16(&rsp, sizeof(rsp)-2);

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	flashChecked = false;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc(Req *r, u16 len, ComPort::WriteBuffer *wb)
{
	bool result = false;

	u16 t = r->req.rw;

	if ((t & manReqMask) != manReqWord || len < 2)
	{
		return false;
	};

	manCounter += 1;

	t &= 0xFF;

	switch (t)
	{
		case 5: 	result = RequestFunc_05(r, len, wb); break;
		case 6: 	result = RequestFunc_06(r, len, wb); break;

		case 7: 		
		default:	RunMainApp(); break;
	};

	if (result)	tm32.Reset(), timeOut = MS2RT(10000);

	return result;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBlackFin()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	
	static Req *req = 0;

	ResetWDT();

	switch(i)
	{
		case 0:

			req = AllocReq();

			if (req != 0)
			{
				rb.data = &req->req;
				rb.maxLen = sizeof(req->req);
				com.Read(&rb, ~0, US2CCLK(500));
				i++;
			};

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0 && GetCRC16(rb.data, rb.len) == 0)
				{
					tm32.Reset();

					if (RequestFunc(req, rb.len, &wb))
					{
						com.Write(&wb);
						i++;
					}
					else
					{
						FreeReq(req);

						req = 0;

						i = 0;
					};
				}
				else
				{
					FreeReq(req);

					req = 0;

					i = 0;
				};
			};

			break;

		case 2:

			if (!com.Update())
			{
				i = 0;
			};

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CheckFlash()
{
	static ADI_BOOT_HEADER bh;
	static u16 crc = 0;

	u32 *p = (u32*)&bh;

	u32 adr = 0;
	
	flashOK = flashChecked = flashCRCOK = false;

	at25df021_Read(buf, FLASH_START_ADR, sizeof(buf));

	while (1)
	{
		at25df021_Read(&bh, FLASH_START_ADR + adr, sizeof(bh));

		u32 xor = p[0] ^ p[1] ^ p[2] ^ p[3];
		xor ^= xor >> 16; 
		xor = (xor ^ (xor >> 8)) & 0xFF; 

		if (((u32)(bh.dBlockCode) >> 24) == 0xAD && xor == 0)
		{
			adr += sizeof(bh);

			if ((bh.dBlockCode & BFLAG_FILL) == 0)
			{
				adr += bh.dByteCount;	
			};

			if (bh.dBlockCode & BFLAG_FINAL)
			{
				flashOK = true;

				break;
			};
		}
		else
		{
			break;
		};
	};

	flashLen = adr;

	at25df021_Read(&crc, FLASH_START_ADR + adr, sizeof(crc));

	if (flashLen > 0) flashCRC = at25df021_GetCRC16(FLASH_START_ADR, flashLen), flashCRCOK = (flashCRC == crc);

	if (!flashCRCOK) flashLen = 0;

	flashChecked = true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

i16 index_max = 0;

int main( void )
{
	static byte s = 0;
	static byte i = 0;

	static u32 pt = 0;

	//static RTM32 tm;

	InitHardware();

	FlashInit();

	com.Connect(6250000, 2);

	CheckFlash();

	tm32.Reset(); timeOut = MS2RT(10000);

	while (1)
	{
		*pPORTFIO_SET = 1<<7;
		
		UpdateBlackFin();
		FlashUpdate();

		//if (runMainApp || tm32.Check(timeOut)) RunMainApp();

		*pPORTFIO_CLEAR = 1<<7;

	};

//	return 0;
}
