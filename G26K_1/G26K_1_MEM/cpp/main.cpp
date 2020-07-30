#include "hardware.h"
//#include "options.h"
#include "emac.h"
#include "xtrap.h"
#include "flash.h"
#include "CRC16.h"
#include "ComPort.h"
#include "CRC16_CCIT.h"
#include "req.h"
#include "list.h"

#ifdef CPU_SAME53	
#elif defined(CPU_XMC48)
#endif

#define VERSION			0x0202

//#pragma O3
//#pragma Otime

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
u32 fps;
i16 tempClock = 0;
i16 cpu_temp = 0;

//inline u16 ReverseWord(u16 v) { __asm	{ rev16 v, v };	return v; }

static void* eepromData = 0;
static u16 eepromWriteLen = 0;
static u16 eepromReadLen = 0;
static u16 eepromStartAdr = 0;

static MTB mtb;

static u16 manBuf[16];

u16 manRcvData[10];
u16 manTrmData[50];

u16 txbuf[128 + 512 + 16];

static ComPort comdsp;
static ComPort commoto;

static RequestQuery qmoto(&commoto);
static RequestQuery qdsp(&comdsp);
//static RequestQuery qmem(&commem);

static R01 r02[8];

static R01* manVec40 = 0;
static R01* curManVec40 = 0;
static R01* manVec50 = 0;
static R01* curManVec50 = 0;

static RspMan60 rspMan60;

//static byte curRcv[3] = {0};
//static byte curVec[3] = {0};

static List<R01> freeR01;

//static RMEM rmem[4];
//static List<RMEM> lstRmem;
//static List<RMEM> freeRmem;

static byte fireType = 0;

static byte gain = 0;
static u16 sampleTime = 5;
static u16 sampleLen = 1024;
static u16 sampleDelay = 200;
static u16 deadTime = 400;
static u16 descriminant = 400;
static u16 freq = 500;

static byte gainRef = 0;
static u16 sampleTimeRef = 5;
static u16 sampleLenRef = 1024;
static u16 sampleDelayRef = 200;
static u16 deadTimeRef = 400;
static u16 descriminantRef = 400;
static u16 refFreq = 500;
static u16 filtrType = 0;
static u16 packType = 0;
//static u16 vavesPerRoundCM = 100;	
//static u16 vavesPerRoundIM = 100;
static u16 mode = 0;

static TM32 imModeTimeout;

static u16 motoEnable = 0;		// двигатель включить или выключить
static u16 motoTargetRPS = 0;	// заданные обороты двигателя
static u16 motoRPS = 0;			// обороты двигателя, об/сек
static u16 motoCur = 0;			// ток двигателя, мА
static u16 motoStat = 0;		// статус двигателя: 0 - выкл, 1 - вкл
static u16 motoCounter = 0;		// счётчик оборотов двигателя 1/6 оборота
static u16 cmSPR = 32;			// Количество волновых картин на оборот головки в режиме цементомера
static u16 imSPR = 100;			// Количество точек на оборот головки в режиме имиджера
static u16 *curSPR = &cmSPR;	// Количество импульсов излучателя на оборот в текущем режиме

static u32 dspMMSEC = 0;
static u32 shaftMMSEC = 0;

const u16 dspReqWord = 0xAD00;
const u16 dspReqMask = 0xFF00;

static u16 manReqWord = 0xAD00;
static u16 manReqMask = 0xFF00;

static u16 memReqWord = 0x3D00;
static u16 memReqMask = 0xFF00;

static u16 numDevice = 0;
static u16 verDevice = 0x100;

static u16 numMemDevice = 0;
static u16 verMemDevice = 0x100;

static u32 manCounter = 0;
static u32 fireCounter = 0;

static byte mainModeState = 0;
static byte dspStatus = 0;

u32 dspRcv40 = 0;
u32 dspRcv50 = 0;
//static u32 rcvCRCER = 0;

//static u32 chnlCount[4] = {0};

static u32 crcErr02 = 0;
//static u32 crcErr03 = 0;
static u32 crcErr04 = 0;

static u32 notRcv02 = 0;
static u32 lenErr02 = 0;

static i16 ax = 0, ay = 0, az = 0, at = 0;
i16 temperature = 0;
i16 cpuTemp = 0;
i16 temp = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update_RPS_SPR()
{
	curSPR = (mode == 0) ? &cmSPR : &imSPR;

	Set_Sync_Rot(motoTargetRPS, *curSPR);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetModeCM()
{
	if (mode != 0)
	{
		mode = 0;

		Update_RPS_SPR();
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetModeIM()
{
	if (mode == 0)
	{
		mode = 1;

		Update_RPS_SPR();
	};

	imModeTimeout.Reset();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Response_0(u16 rw, MTB &mtb)
{
	__packed struct Rsp {u16 rw; u16 device; u16 session; u32 rcvVec; u32 rejVec; u32 wrVec; u32 errVec; u16 wrAdr[3]; u16 numDevice; u16 version; u16 temp; byte status; byte flags; RTC rtc; };

	Rsp &rsp = *((Rsp*)&txbuf);

	rsp.rw = rw;
	rsp.device = GetDeviceID();  
	rsp.session = FLASH_Session_Get();	  
	rsp.rcvVec =  FLASH_Vectors_Recieved_Get();
	rsp.rejVec = FLASH_Vectors_Rejected_Get();
	rsp.wrVec = FLASH_Vectors_Saved_Get();
	rsp.errVec = FLASH_Vectors_Errors_Get();
	*((__packed u64*)rsp.wrAdr) = FLASH_Current_Adress_Get();
	rsp.temp = temp*5/2;
	rsp.status = FLASH_Status();

	GetTime(&rsp.rtc);

	mtb.data1 = txbuf;
	mtb.len1 = sizeof(rsp)/2;
	mtb.data2 = 0;
	mtb.len2 = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackDspReq01(REQ *q)
{
	RspDsp01 &rsp = *((RspDsp01*)q->rb->data);
	 
	if (rsp.rw == (dspReqWord|0x40))
	{
		q->crcOK = (q->rb->len == (rsp.CM.sl*2 + 19*2));

		dspStatus |= 1;
		dspRcv40++;
		
		dspMMSEC = rsp.time;
		shaftMMSEC = rsp.hallTime;

	}
	else if (rsp.rw == (dspReqWord|0x50))
	{
		q->crcOK = (q->rb->len == (rsp.IM.dataLen*4 + 11*2));

		dspStatus |= 1;
		dspRcv50++;

		dspMMSEC = rsp.time;
		shaftMMSEC = rsp.hallTime;
	}
	else
	{
		q->crcOK = false;

		if (q->rb->recieved)
		{
			//if ((rsp.rw & manReqMask) != manReqWord || (rsp.len*8+16) != q->rb->len)
			//{
			//	lenErr02++;
			//}
			//else
			//{
			//	crcErr02++;
			//};
		}
		else
		{
			notRcv02++;
		};


		if (q->tryCount > 0)
		{
			q->tryCount--;
			qdsp.Add(q);
		}
		else
		{
			dspStatus &= ~1; 

			R01* r = (R01*)q->ptr;
		
			if (r != 0)
			{
				freeR01.Add(r); 
			};
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

R01* CreateDspReq01(u16 tryCount)
{
	R01 *r01 = freeR01.Get();

	if (r01 == 0) return 0;

	R01 &r = *r01;

	ReqDsp01 &req = r.req;
	RspDsp01 &rsp = r.rsp;
	
	ComPort::WriteBuffer &wb = r.wb;
	ComPort::ReadBuffer	 &rb = r.rb;
	
	REQ &q = r.q;

	q.CallBack = CallBackDspReq01;
	q.rb = &rb;
	q.wb = &wb;
	q.preTimeOut = MS2RT(1);
	q.postTimeOut = 1;
	q.ready = false;
	q.tryCount = tryCount;
	q.ptr = &r;
	q.checkCRC = false;
	q.updateCRC = false;
	
	wb.data = &req;
	wb.len = sizeof(req);

	rb.data = &rsp;
	rb.maxLen = sizeof(rsp);
	rb.recieved = false;
	
	req.rw				= dspReqWord|1;
	req.mode 			= mode;
	req.motoCount		= motoCounter;
	req.gain 			= gain;
	req.st	 			= sampleTime;
	req.sl 				= sampleLen;
	req.sd 				= sampleDelay;
	req.thr				= descriminant;
	req.descr			= deadTime;
	req.refgain 		= gainRef;
	req.refst			= sampleTimeRef;
	req.refsl 			= sampleLenRef;
	req.refsd 			= sampleDelayRef;
	req.refthr			= deadTimeRef;
	req.refdescr		= descriminantRef;
	req.vavesPerRoundCM = cmSPR;
	req.vavesPerRoundIM = imSPR;
	req.filtrType		= filtrType;
	req.packType		= packType;

	req.crc	= GetCRC16(&req, sizeof(ReqDsp01)-2);

	return r01;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackDspReq05(REQ *q)
{
	if (!q->crcOK) 
	{
		crcErr04++;

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qdsp.Add(q);
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

REQ* CreateDspReq05(u16 tryCount)
{
	static ReqDsp05 req[2];
	static RspDsp05 rsp;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static REQ q;

	q.CallBack = CallBackDspReq05;
	q.preTimeOut = US2RT(500);
	q.postTimeOut = US2RT(100);
	q.rb = &rb;
	q.wb = &wb;
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	wb.data = req;
	wb.len = sizeof(req);
	
	rb.data = &rsp;
	rb.maxLen = sizeof(rsp);

	//req[1].len	= req[0].len	= sizeof(ReqDsp05) - 1;
	//req[1].func = req[0].func	= 5;

	//req[1].crc = req[0].crc = GetCRC16(&req[0].func, sizeof(ReqDsp05)-3);

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackDspReq06(REQ *q)
{
	if (!q->crcOK) 
	{
		crcErr04++;

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qdsp.Add(q);
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

REQ* CreateDspReq06(u16 stAdr, u16 count, void* data, u16 count2, void* data2, u16 tryCount)
{
	static ReqDsp06 req;
	static RspDsp06 rsp;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static REQ q;

	q.CallBack = CallBackDspReq06;
	q.preTimeOut = MS2RT(10);
	q.postTimeOut = US2RT(100);
	q.rb = &rb;
	q.wb = &wb;
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	rb.data = &rsp;
	rb.maxLen = sizeof(rsp);

	req.rw = dspReqWord|6;

	u16 max = sizeof(req.data)-2;

	if (count > max)
	{
		count = max;
		count2 = 0;
	}
	else if ((count + count2) > max)
	{
		count2 = max - count;
	};

	req.stAdr = stAdr;
	req.count = count+count2;

	//req.crc = GetCRC16(&req, sizeof(req)-2);

	byte *d = req.data;
	byte *s = (byte*)data;

	while(count > 0)
	{
		*d++ = *s++;
		count--;
	};

	if (data2 != 0)
	{
		s = (byte*)data2;

		while(count2 > 0)
		{
			*d++ = *s++;
			count2--;
		};
	};

	u16 len = sizeof(req) - 2 - sizeof(req.data) + req.count;

	u16 crc = GetCRC16(&req, len);

	d[0] = crc;
	d[1] = crc>>8;

	wb.data = &req;
	wb.len = len+2;

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

REQ* CreateDspReq07()
{
	static ReqDsp07 req;
	static ComPort::WriteBuffer wb;
	static REQ q;

	q.CallBack = 0;
	//q.preTimeOut = US2RT(500);
	//q.postTimeOut = US2RT(100);
	q.rb = 0;
	q.wb = &wb;
	q.ready = false;
	q.tryCount = 0;
	q.checkCRC = false;
	q.updateCRC = false;
	
	wb.data = &req;
	wb.len = sizeof(req);
	
	//rb.data = &rsp;
	//rb.maxLen = sizeof(rsp);

	req.rw	= dspReqWord|7;

	req.crc = GetCRC16(&req, sizeof(ReqDsp07)-2);

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CallBackMotoReq(REQ *q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qmoto.Add(q);
		};
	}
	else
	{
		RspMoto &rsp = *((RspMoto*)q->rb->data);

		if (rsp.rw == 0x101)
		{
			motoRPS = rsp.rpm;
			motoCur = rsp.current;
			motoStat = rsp.mororStatus;
			motoCounter = rsp.motoCounter;
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
RspMoto rspMoto;

static REQ* CreateMotoReq()
{
	static ReqMoto req;
//	static RspMoto rspMoto;

	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static REQ q;

	q.CallBack = CallBackMotoReq;
	q.rb = &rb;
	q.wb = &wb;
	q.preTimeOut = MS2RT(1);
	q.postTimeOut = 1;
	q.ready = false;
	q.checkCRC = true;
	q.updateCRC = false;
	q.tryCount = 1;
	
	wb.data = &req;
	wb.len = sizeof(req);

	rb.data = &rspMoto;
	rb.maxLen = sizeof(rspMoto);
	
	req.rw = 0x101;
	req.enableMotor	= 0;
	req.tRPM = 0;
	req.crc	= GetCRC16(&req, sizeof(req)-2);

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitRmemList()
{
	for (u16 i = 0; i < ArraySize(r02); i++)
	{
		freeR01.Add(&r02[i]);
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = (manReqWord & manReqMask) | 0;
	manTrmData[1] = numDevice;
	manTrmData[2] = verDevice;

	mtb->data1 = manTrmData;
	mtb->len1 = 3;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_10(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = (manReqWord & manReqMask) | 0x10;		//1. Ответное слово
	manTrmData[1] = gain;									//2. КУ (измерительный датчик)
	manTrmData[2] = sampleTime;								//3. Шаг оцифровки
	manTrmData[3] = sampleLen;								//4. Длина оцифровки
	manTrmData[4] = sampleDelay; 							//5. Задержка оцифровки
	manTrmData[5] = deadTime;								//6. Мертвая зона датчика
	manTrmData[6] = descriminant;							//7. Уровень дискриминации датчика
	manTrmData[7] = freq;
	manTrmData[8] = gainRef;								//8. КУ (опорный датчик)
	manTrmData[9] = sampleTimeRef;							//9. Шаг оцифровки
	manTrmData[10] = sampleLenRef;							//10. Длина оцифровки
	manTrmData[11] = sampleDelayRef; 						//11. Задержка оцифровки
	manTrmData[12] = deadTimeRef;							//12. Мертвая зона датчика
	manTrmData[13] = descriminantRef;						//13. Уровень дискриминации датчика
	manTrmData[14] = refFreq;
	manTrmData[15] = filtrType;								//14. Фильтр
	manTrmData[16] = packType;								//15. Упаковка
	manTrmData[17] = cmSPR;									//16. Количество волновых картин на оборот головки в режиме цементомера
	manTrmData[18] = imSPR;									//17. Количество точек на оборот головки в режиме имиджера

	mtb->data1 = manTrmData;
	mtb->len1 = 19;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = manReqWord|0x20;	//	1. ответное слово
	manTrmData[1] = dspMMSEC; 			//	2. Время (0.1мс). младшие 2 байта
	manTrmData[2] = dspMMSEC>>16;		//	3. Время. старшие 2 байта
	manTrmData[3] = shaftMMSEC;			//	4. Время датчика Холла (0.1мс). младшие 2 байта
	manTrmData[4] = shaftMMSEC>>16;		//	5. Время датчика Холла. старшие 2 байта
	manTrmData[5] = motoRPS;			//	6. Частота вращения двигателя (0.01 об/сек)
	manTrmData[6] = motoCur;			//	7. Ток двигателя (мА)
	manTrmData[7] = motoCounter;		//	8. Счётчик оборотов двигателя (1/6 об)
	manTrmData[8] = GetShaftRPS();		//	9. Частота вращения головки (0.01 об/сек)
	manTrmData[9] = GetShaftCount();	//	10. Счётчик оборотов головки (об)
	manTrmData[10] = -ax;				//	11. AX (уе)
	manTrmData[11] = az;				//	12. AY (уе)
	manTrmData[12] = -ay;				//	13. AZ (уе)
	manTrmData[13] = tempClock;			//	14. AT (short 0.01 гр)
	manTrmData[14] = temp;				//	15. Температура в приборе (short)(0.1гр)
 
	mtb->data1 = manTrmData;
	mtb->len1 = 15;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_40(u16 *data, u16 reqlen, MTB* mtb)
{
	__packed struct Req { u16 rw; u16 off; u16 len; };

	Req &req = *((Req*)data);

	if (data == 0 || reqlen == 0 || reqlen > 4 || mtb == 0) return false;

	//byte nf = ((req.rw>>4)-3)&3;
	//byte nr = req.rw & 7;

//	curRcv[nf] = nr;

	struct Rsp { u16 rw; };
	
	static Rsp rsp; 
	
	static u16 prevOff = 0;
	static u16 prevLen = 0;
	static u16 maxLen = 200;

	rsp.rw = req.rw;

	mtb->data1 = (u16*)&rsp;
	mtb->len1 = sizeof(rsp)/2;
	mtb->data2 = 0;
	mtb->len2 = 0;

	R01 *r01 = curManVec40;
	
	u16 sz = 18 + r01->rsp.CM.sl;

	if (reqlen == 1 || (reqlen >= 2 && data[1] == 0))
	{
		if (r01 != 0)
		{
			freeR01.Add(r01);

			curManVec40 = 0;
		};
		
		r01 = manVec40;

		if (r01 != 0/* && r01->rsp.rw == req.rw*/)
		{
			curManVec40 = r01;

			manVec40 = 0;

			mtb->data2 = ((u16*)&r01->rsp)+1;

			prevOff = 0;

			if (reqlen == 1)
			{
				mtb->len2 = sz;
				prevLen = sz;
			}
			else 
			{
				if (reqlen == 3) maxLen = data[2];

				u16 len = maxLen;

				if (len > sz) len = sz;

				mtb->len2 = len;

				prevLen = len;
			};
		};
	}
	else 
	{
		u16 off = prevOff + prevLen;
		u16 len = prevLen;

		if (reqlen == 3)
		{
			off = data[1];
			len = data[2];
		};

		if (sz >= off && r01 != 0)
		{
			u16 ml = sz - off;

			if (len > ml) len = ml;

			mtb->data2 = (u16*)&r01->rsp + data[1]+1;
			mtb->len2 = len;
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_30(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 3 || mtb == 0) return false;

	manTrmData[0] = data[0];	
 
	motoTargetRPS = (data[0]&15) * 100;
		
	Set_Sync_Rot(motoTargetRPS, *curSPR);

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_50(u16 *data, u16 reqlen, MTB* mtb)
{
	__packed struct Req { u16 rw; u16 off; u16 len; };

	Req &req = *((Req*)data);

	if (data == 0 || reqlen == 0 || reqlen > 4 || mtb == 0) return false;

	//byte nf = ((req.rw>>4)-3)&3;
	//byte nr = req.rw & 7;

//	curRcv[nf] = nr;

	struct Rsp { u16 rw; };
	
	static Rsp rsp; 
	
	static u16 prevOff = 0;
	static u16 prevLen = 0;
	static u16 maxLen = 200;

	SetModeIM();

	rsp.rw = req.rw;

	mtb->data1 = (u16*)&rsp;
	mtb->len1 = sizeof(rsp)/2;
	mtb->data2 = 0;
	mtb->len2 = 0;

	R01 *r01 = curManVec50;
	
	if (reqlen == 1 || (reqlen >= 2 && data[1] == 0))
	{
		if (r01 != 0)
		{
			freeR01.Add(r01);

			curManVec50 = 0;
		};
		
		r01 = manVec50;

		if (r01 != 0/* && r01->rsp.rw == req.rw*/)
		{
			curManVec50 = r01;

			manVec50 = 0;

			mtb->data2 = ((u16*)&r01->rsp)+1;

			prevOff = 0;

			u16 sz = 10 + r01->rsp.IM.dataLen*2;

			if (reqlen == 1)
			{
				mtb->len2 = sz;
				prevLen = sz;
			}
			else 
			{
				if (reqlen == 3) maxLen = data[2];

				u16 len = maxLen;

				if (len > sz) len = sz;

				mtb->len2 = len;

				prevLen = len;
			};
		};
	}
	else if (r01 != 0)
	{
		u16 off = prevOff + prevLen;
		u16 len = prevLen;
		u16 sz = 10 + r01->rsp.IM.dataLen*2;

		if (reqlen == 3)
		{
			off = data[1];
			len = data[2];
		};

		if (sz >= off && r01 != 0)
		{
			u16 ml = sz - off;

			if (len > ml) len = ml;

			mtb->data2 = (u16*)&r01->rsp + data[1]+1;
			mtb->len2 = len;
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_80(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch (data[1])
	{
		case 1:

			numDevice = data[2];

			break;

		case 2:

			SetTrmBoudRate(data[2]-1);

			break;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0x80;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_90(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch(data[1])
	{
		case 0x1:	gain			= data[2];	break;
		case 0x2:	sampleTime		= data[2];	break;
		case 0x3:	sampleLen		= data[2];	break;
		case 0x4:	sampleDelay 	= data[2];	break;
		case 0x5:	deadTime		= data[2];	break;
		case 0x6:	descriminant	= data[2];	break;
		case 0x7:	freq			= data[2];	break;

		case 0x11:	gainRef			= data[2];	break;
		case 0x12:	sampleTimeRef	= data[2];	break;
		case 0x13:	sampleLenRef	= data[2];	break;
		case 0x14:	sampleDelayRef 	= data[2];	break;
		case 0x15:	deadTimeRef		= data[2];	break;
		case 0x16:	descriminantRef	= data[2];	break;
		case 0x17:	refFreq			= data[2];	break;

		case 0x20:	filtrType		= data[2];	break;
		case 0x21:	packType		= data[2];	break;

		case 0x30:	cmSPR = data[2]; Update_RPS_SPR();	break;
		case 0x31:	imSPR = data[2]; Update_RPS_SPR();	break;

		default:

			return false;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0x90;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	SaveParams();

	manTrmData[0] = (manReqWord & manReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan(u16 *buf, u16 len, MTB* mtb)
{
	if (buf == 0 || len == 0 || mtb == 0) return false;

	bool r = false;

	byte i = (buf[0]>>4)&0xF;

	switch (i)
	{
		case 0: 	r = RequestMan_00(buf, len, mtb); break;
		case 1: 	r = RequestMan_10(buf, len, mtb); break;
		case 2: 	r = RequestMan_20(buf, len, mtb); break;
		case 3:		r = RequestMan_30(buf, len, mtb); break;
		case 4:		r = RequestMan_40(buf, len, mtb); break;
		case 5: 	r = RequestMan_50(buf, len, mtb); break;
		case 8: 	r = RequestMan_80(buf, len, mtb); break;
		case 9:		r = RequestMan_90(buf, len, mtb); break;
		case 0xF:	r = RequestMan_F0(buf, len, mtb); break;
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0;
	manTrmData[1] = numMemDevice;
	manTrmData[2] = verMemDevice;

	mtb->data1 = manTrmData;
	mtb->len1 = 3;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_10(u16 *data, u16 len, MTB* mtb)
{
	//__packed struct T { u16 g[8]; u16 st; u16 len; u16 delay; u16 voltage; };
	//__packed struct Rsp { u16 hdr; u16 rw; T t1, t2, t3; };
	
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0x10;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	__packed struct Rsp {u16 rw; u16 device; u16 session; u32 rcvVec; u32 rejVec; u32 wrVec; u32 errVec; u16 wrAdr[3]; u16 temp; byte status; byte flags; RTC rtc; };

	if (len != 1) return false;

	Rsp &rsp = *((Rsp*)&manTrmData);

	rsp.rw = (memReqWord & memReqMask)|0x20;
	rsp.device = GetDeviceID();  
	rsp.session = FLASH_Session_Get();	  
	rsp.rcvVec =  FLASH_Vectors_Recieved_Get();
	rsp.rejVec = FLASH_Vectors_Rejected_Get();
	rsp.wrVec = FLASH_Vectors_Saved_Get();
	rsp.errVec = FLASH_Vectors_Errors_Get();
	*((__packed u64*)rsp.wrAdr) = FLASH_Current_Adress_Get();
	rsp.temp = (temp+2)/4;
	rsp.status = FLASH_Status();

	GetTime(&rsp.rtc);

	mtb->data1 = manTrmData;
	mtb->len1 = 20;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_30(u16 *data, u16 len, MTB* mtb)
{
	if (len != 5) return false;

	SetClock(*(RTC*)(&data[1]));

	manTrmData[0] = (memReqWord & memReqMask)|0x30;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_31(u16 *data, u16 len, MTB* mtb)
{
	if (len != 1) return false;

	FLASH_WriteEnable();

	manTrmData[0] = (memReqWord & memReqMask)|0x31;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_32(u16 *data, u16 len, MTB* mtb)
{
	if (len != 1) return false;

	FLASH_WriteDisable();

	manTrmData[0] = (memReqWord & memReqMask)|0x32;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_33(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	// Erase all

	manTrmData[0] = (memReqWord & memReqMask)|0x33;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_80(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch (data[1])
	{
		case 1:

			numMemDevice = data[2];

			break;

		case 2:

			SetTrmBoudRate(data[2]-1);

			break;
	};

	manTrmData[0] = (memReqWord & memReqMask) | 0x80;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_90(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 1 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0x90;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	SaveParams();

	manTrmData[0] = (memReqWord & memReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


static bool RequestMem(u16 *buf, u16 len, MTB* mtb)
{
	if (buf == 0 || len == 0 || mtb == 0) return false;

	bool r = false;

	byte i = buf[0]&0xFF;

	switch (i)
	{
		case 0x00: 	r = RequestMem_00(buf, len, mtb); break;
		case 0x10: 	r = RequestMem_10(buf, len, mtb); break;
		case 0x20: 	r = RequestMem_20(buf, len, mtb); break;
		case 0x30: 	r = RequestMem_30(buf, len, mtb); break;
		case 0x31: 	r = RequestMem_31(buf, len, mtb); break;
		case 0x32: 	r = RequestMem_32(buf, len, mtb); break;
		case 0x33: 	r = RequestMem_33(buf, len, mtb); break;
		case 0x80: 	r = RequestMem_80(buf, len, mtb); break;
		case 0x90: 	r = RequestMem_90(buf, len, mtb); break;
		case 0xF0: 	r = RequestMem_F0(buf, len, mtb); break;
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMan()
{
	static MTB mtb;
	static MRB mrb;

	static byte i = 0;

	static RTM tm;


//	u16 c;

	switch (i)
	{
		case 0:

			mrb.data = manRcvData;
			mrb.maxLen = ArraySize(manRcvData);
			RcvManData(&mrb);

			i++;

			break;

		case 1:

			ManRcvUpdate();

			if (mrb.ready)
			{
				tm.Reset();

				if (mrb.OK && mrb.len > 0 &&	(((manRcvData[0] & manReqMask) == manReqWord && RequestMan(manRcvData, mrb.len, &mtb)) 
										||		((manRcvData[0] & memReqMask) == memReqWord && RequestMem(manRcvData, mrb.len, &mtb))))
				{
					i++;
				}
				else
				{
					i = 0;
				};
			}
			else if (mrb.len > 0)
			{

			};

			break;

		case 2:

			if (tm.Check(US2RT(100)))
			{
				SendManData(&mtb);

				i++;
			};

			break;

		case 3:

			if (mtb.ready)
			{
				i = 0;
			};

			break;

	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void MainMode()
{
	//fireType

	static byte rcv = 0;
//	static REQ *req = 0;
	static R01 *r01 = 0;
//	static RTM32 rt;
//	static TM32 rt2;

//	REQ *rm = 0;

	switch (mainModeState)
	{
		case 0:

			r01 = CreateDspReq01(1);

			if (r01 != 0)
			{
				qdsp.Add(&r01->q);

				mainModeState++;
			};

			break;

		case 1:

			if (r01->q.ready)
			{
				if (r01->q.crcOK)
				{
					if ((r01->rsp.rw & 0xFF) == 0x40)
					{
						r01->rsp.CM.ax = -ax;
						r01->rsp.CM.ay = az;
						r01->rsp.CM.az = -ay;
						r01->rsp.CM.at = at;
						r01->rsp.CM.pakType = 1;

						if (manVec40 != 0 && manVec40->rsp.CM.sensType == 0)
						{
							freeR01.Add(manVec40);
							
							manVec40 = 0;
						};
						
						if (manVec40 == 0)
						{
							manVec40 = r01;
						}
						else
						{
							freeR01.Add(r01);
						};
					}
					else if ((r01->rsp.rw & 0xFF) == 0x50)
					{
						r01->rsp.IM.ax = -ax;
						r01->rsp.IM.ay = az;
						r01->rsp.IM.az = -ay;
						r01->rsp.IM.at = at;

						//r01->rsp.IM.data[0] = 32767;

						if (manVec50 != 0)
						{
							freeR01.Add(manVec50);
						};
						
						manVec50 = r01;
					};
				
					manCounter++;
				}
				else
				{
					freeR01.Add(r01);
				};

				mainModeState = 0;
			};

			if (imModeTimeout.Check(10000))
			{
				SetModeCM();
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTemp()
{
	static byte i = 0;

	static DSCI2C dsc, dsc2;

	static byte reg = 0;
	static u16 rbuf = 0;
	static byte buf[10];

	static TM32 tm;

	switch (i)
	{
		case 0:

			if (tm.Check(100))
			{
				buf[0] = 0x11;

				dsc.adr = 0x68;
				dsc.wdata = buf;
				dsc.wlen = 1;
				dsc.rdata = &rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				if (I2C_AddRequest(&dsc))
				{
					i++;
				};
			};

			break;

		case 1:

			if (dsc.ready)
			{
				if (dsc.ack && dsc.readedLen == dsc.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);
					
					t = (t * 10 + 128) / 256;

					if (t < (-600))
					{
						t += 2560;
					};

					tempClock = t;
				}
				else
				{
					tempClock = -2730;
				};

				i++;
			};

			break;

		case 2:

			buf[0] = 0x0E;
			buf[1] = 0x20;
			buf[2] = 0xC8;

			dsc2.adr = 0x68;
			dsc2.wdata = buf;
			dsc2.wlen = 3;
			dsc2.rdata = 0;
			dsc2.rlen = 0;
			dsc2.wdata2 = 0;
			dsc2.wlen2 = 0;

			if (I2C_AddRequest(&dsc2))
			{
				i++;
			};

			break;

		case 3:

			if (dsc2.ready)
			{
				buf[0] = 0;

				dsc.adr = 0x49;
				dsc.wdata = buf;
				dsc.wlen = 1;
				dsc.rdata = &rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				if (I2C_AddRequest(&dsc))
				{
					i++;
				};
			};

			break;

		case 4:

			if (dsc.ready)
			{
				if (dsc.ack && dsc.readedLen == dsc.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);

					temp = (t * 10 + 64) / 128;
				}
				else
				{
					temp = -2730;
				};

#ifdef CPU_SAME53	

				i = 0;
			};

			break;

#elif defined(CPU_XMC48)

				HW::SCU_GENERAL->DTSCON = SCU_GENERAL_DTSCON_START_Msk;
				
				i++;
			};

			break;

		case 5:

			if (HW::SCU_GENERAL->DTSSTAT & SCU_GENERAL_DTSSTAT_RDY_Msk)
			{
				cpu_temp = ((i32)(HW::SCU_GENERAL->DTSSTAT & SCU_GENERAL_DTSSTAT_RESULT_Msk) - 605) * 1000 / 205;

				i = 0;
			};

			break;
#endif
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMoto()
{
	static TM32 tm;

	if (tm.Check(101))
	{
		qmoto.Add(CreateMotoReq());
	}
	else
	{
		qmoto.Update();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateParams()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( MainMode()		);
		CALL( UpdateMoto()		);
		CALL( UpdateTemp()		);
		CALL( UpdateMan(); 		);
		CALL( FLASH_Update();	);
		CALL( UpdateTraps();	);
		CALL( UpdateHardware();	);
//		CALL( I2C_Update();		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMisc()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateEMAC();		);
		CALL( qdsp.Update()		);
		CALL( UpdateParams();	);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( NAND_Idle();		);
		CALL( UpdateMisc();		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53

	#define FPS_PIN_SET()	HW::PIOA->BSET(25)
	#define FPS_PIN_CLR()	HW::PIOA->BCLR(25)

#elif defined(CPU_XMC48)

	#define FPS_PIN_SET()	HW::P2->BSET(13)
	#define FPS_PIN_CLR()	HW::P2->BCLR(13)

#endif

//static ComPort com1;

int main()
{
	static bool c = true;

	//static byte buf[100];

	//volatile byte * const FLD = (byte*)0x60000000;	
	
	static ComPort commem;

//	RTM32 tm;
//	Dbt db(100);

	//__breakpoint(0);

	DisableDSP();

	InitHardware();

	EnableDSP();

	InitEMAC();

	InitTraps();

	FLASH_Init();

	InitRmemList();

	Update_RPS_SPR();

	commoto.Connect(ComPort::ASYNC, 0, 1562500, 0, 1);
	comdsp.Connect(ComPort::ASYNC, 2, 6250000, 0, 1);
	commem.Connect(ComPort::ASYNC, 1, 6250000, 0, 1);

	u32 fc = 0;

	TM32 tm;
//	RTM rtm;

	tm.pt = 0;


	ComPort::WriteBuffer wb;

	while (1)
	{
		FPS_PIN_SET();

		Update();

		FPS_PIN_CLR();

		fc++;

		commem.Update();

		if (tm.Check(1000))
		{ 
			fps = fc; fc = 0; 

			//wb.data = buf;
			//wb.len = 5;
			//
			//commem.Write(&wb);

			//HW::ResetWDT();
		};

	}; // while (1)
}
