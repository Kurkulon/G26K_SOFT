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
#include "PointerCRC.h"

#ifdef CPU_SAME53	
#elif defined(CPU_XMC48)
#endif

#define VERSION			0x0202

//#pragma O3
//#pragma Otime

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct MainVars // NonVolatileVars  
{
	u16 numDevice;
	u16 numMemDevice;

	u16 gain;
	u16 sampleTime;
	u16 sampleLen;
	u16 sampleDelay;
	u16 deadTime;
	u16 descriminant;
	u16 freq;

	u16 gainRef;
	u16 sampleTimeRef;
	u16 sampleLenRef;
	u16 sampleDelayRef;
	u16 deadTimeRef;
	u16 descriminantRef;
	u16 refFreq;
	u16 filtrType;
	u16 packType;
	u16 cmSPR;
	u16 imSPR;
};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static MainVars mv;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 req40_count1 = 0;
u32 req40_count2 = 0;
u32 req40_count3 = 0;

u32 fps;
i16 tempClock = 0;
i16 cpu_temp = 0;
u32 i2cResetCount = 0;

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

static R01* manVec40[2] = {0};
static R01* curManVec40 = 0;
static R01* manVec50 = 0;
static R01* curManVec50 = 0;

static RspMan60 rspMan60;

//static byte curRcv[3] = {0};
//static byte curVec[3] = {0};

static List<R01> freeR01;
static List<R01> readyR01;

//static RMEM rmem[4];
//static List<RMEM> lstRmem;
//static List<RMEM> freeRmem;

static byte fireType = 0;

//static u16 gain = 0;
//static u16 sampleTime = 5;
//static u16 sampleLen = 1024;
//static u16 sampleDelay = 200;
//static u16 deadTime = 400;
//static u16 descriminant = 400;
//static u16 freq = 500;

//static u16 gainRef = 0;
//static u16 sampleTimeRef = 5;
//static u16 sampleLenRef = 1024;
//static u16 sampleDelayRef = 200;
//static u16 deadTimeRef = 400;
//static u16 descriminantRef = 400;
//static u16 refFreq = 500;
//static u16 filtrType = 0;
//static u16 packType = 0;
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
//static u16 cmSPR = 32;			// Количество волновых картин на оборот головки в режиме цементомера
//static u16 imSPR = 100;			// Количество точек на оборот головки в режиме имиджера
//static u16 *curSPR = &cmSPR;	// Количество импульсов излучателя на оборот в текущем режиме

static u32 dspMMSEC = 0;
static u32 shaftMMSEC = 0;

const u16 dspReqWord = 0xAD00;
const u16 dspReqMask = 0xFF00;

static u16 manReqWord = 0xAD00;
static u16 manReqMask = 0xFF00;

static u16 memReqWord = 0x3D00;
static u16 memReqMask = 0xFF00;

//static u16 numDevice = 0;
static u16 verDevice = 0x100;

//static u16 numMemDevice = 0;
static u16 verMemDevice = 0x100;

static u32 manCounter = 0;
static u32 fireCounter = 0;

static byte mainModeState = 0;
static byte dspStatus = 0;

static bool cmdWriteStart_00 = false;
static bool cmdWriteStart_10 = false;
static bool cmdWriteStart_20 = false;

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

static byte svCount = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SaveMainParams()
{
	svCount = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetNumDevice(u16 num)
{
	mv.numDevice = num;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern u16 GetNumDevice()
{
	return mv.numDevice;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update_RPS_SPR()
{
	Set_Sync_Rot(motoTargetRPS, (mode == 0) ? mv.cmSPR : mv.imSPR);
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
		q->crcOK = (q->rb->len == (rsp.CM.sl*2 + 10 + sizeof(rsp.CM)-sizeof(rsp.CM.data)));

		dspStatus |= 1;
		dspRcv40++;
		
		dspMMSEC = rsp.time;
		shaftMMSEC = rsp.hallTime;

		rsp.CM.ax = ax;
		rsp.CM.ay = ay;
		rsp.CM.az = az;
		rsp.CM.at = at;
		rsp.CM.pakType = 0;
	}
	else if (rsp.rw == (dspReqWord|0x50))
	{
		q->crcOK = (q->rb->len == (rsp.IM.dataLen*4 + 10 + sizeof(rsp.IM)-sizeof(rsp.IM.data)));

		dspStatus |= 1;
		dspRcv50++;

		dspMMSEC = rsp.time;
		shaftMMSEC = rsp.hallTime;

		rsp.IM.ax = ax;
		rsp.IM.ay = ay;
		rsp.IM.az = az;
		rsp.IM.at = at;
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
	};

	if (!q->crcOK)
	{
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
	req.gain 			= mv.gain;
	req.st	 			= mv.sampleTime;
	req.sl 				= mv.sampleLen;
	req.sd 				= mv.sampleDelay;
	req.thr				= mv.descriminant;
	req.descr			= mv.deadTime;
	req.freq			= mv.freq;
	req.refgain 		= mv.gainRef;
	req.refst			= mv.sampleTimeRef;
	req.refsl 			= mv.sampleLenRef;
	req.refsd 			= mv.sampleDelayRef;
	req.refthr			= mv.deadTimeRef;
	req.refdescr		= mv.descriminantRef;
	req.refFreq			= mv.refFreq;
	req.vavesPerRoundCM = mv.cmSPR;
	req.vavesPerRoundIM = mv.imSPR;
	req.filtrType		= mv.filtrType;
	req.packType		= mv.packType;

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
	static ReqDsp05 req;
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
	
	wb.data = &req;
	wb.len = sizeof(req);
	
	rb.data = &rsp;
	rb.maxLen = sizeof(rsp);

	req.rw = dspReqWord|5;
	req.crc	= GetCRC16(&req, sizeof(req)-2);

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
	req.enableMotor	= 1;
	req.tRPM = motoTargetRPS;
	req.crc	= GetCRC16(&req, sizeof(req)-2);

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackBootMotoReq01(REQ *q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qmoto.Add(q);
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

REQ* CreateBootMotoReq01(u16 flashLen, u16 tryCount)
{
	static ReqBootMoto req;
	static RspBootMoto rsp;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static REQ q;

	q.CallBack = CallBackBootMotoReq01;
	q.preTimeOut = MS2RT(10);
	q.postTimeOut = US2RT(100);
	q.rb = &rb;
	q.wb = &wb;
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	wb.data = &req;
	wb.len = sizeof(req.F01) + sizeof(req.func);
	
	rb.data = &rsp;
	rb.maxLen = sizeof(rsp);

	req.func = 1;
	req.F01.flashLen = flashLen;
	req.F01.align = ~flashLen;

	req.F01.crc	= GetCRC16(&req, wb.len - sizeof(req.F01.crc));

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackBootMotoReq02(REQ *q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qmoto.Add(q);
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

REQ* CreateBootMotoReq02(u16 stAdr, u16 count, const u32* data, u16 tryCount)
{
	static ReqBootMoto req;
	static RspBootMoto rsp;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static REQ q;

	q.CallBack = CallBackBootMotoReq02;
	q.preTimeOut = MS2RT(300);
	q.postTimeOut = US2RT(100);
	q.rb = &rb;
	q.wb = &wb;
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	rb.data = &rsp;
	rb.maxLen = sizeof(rsp);

	req.func = 2;

	u16 max = ArraySize(req.F02.page);

	if (count > max)
	{
		count = max;
	};

	u32 count2 = max - count;

	req.F02.padr = stAdr;

	u32 *d = req.F02.page;

	while(count > 0)
	{
		*d++ = *data++;
		count--;
	};

	if (count2 > 0)
	{
		*d++ = ~0;
		count2--;
	};

	u16 len = sizeof(req.F02) + sizeof(req.func) - sizeof(req.F02.crc);

	req.F02.align = 0xAAAA;
	req.F02.crc = GetCRC16(&req, len);

	wb.data = &req;
	wb.len = len+sizeof(req.F02.crc);

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

REQ* CreateBootMotoReq03()
{
	static ReqBootMoto req;
	static RspBootMoto rsp;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static REQ q;

	q.CallBack = 0;
	q.preTimeOut = MS2RT(10);
	q.postTimeOut = US2RT(100);
	q.rb = &rb;
	q.wb = &wb;
	q.ready = false;
	q.tryCount = 1;
	q.checkCRC = true;
	q.updateCRC = false;
	
	rb.data = &rsp;
	rb.maxLen = sizeof(rsp);

	
	req.func = 3;
	req.F03.align += 1; 

	u16 len = sizeof(req.F03) + sizeof(req.func) - sizeof(req.F03.crc);

	req.F03.crc = GetCRC16(&req, len);

	wb.data = &req;
	wb.len = len + sizeof(req.F03.crc);

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

static u32 InitRspMan_00(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= (manReqWord & manReqMask) | 0;
	*(data++)	= mv.numDevice;
	*(data++)	= verDevice;
	
	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_00(FLWB *flwb)
{
	__packed u16* data = (__packed u16*)flwb->vd.data;

	flwb->dataLen = InitRspMan_00(data) * 2;

	RequestFlashWrite(flwb, data[0]);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	InitRspMan_00(manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = 3;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_10(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= (manReqWord & manReqMask) | 0x10;		//1. Ответное слово
	*(data++)	= mv.gain;									//2. КУ (измерительный датчик)
	*(data++)	= mv.sampleTime;							//3. Шаг оцифровки
	*(data++)	= mv.sampleLen;							//4. Длина оцифровки
	*(data++)	= mv.sampleDelay; 							//5. Задержка оцифровки
	*(data++)	= mv.deadTime;								//6. Мертвая зона датчика
	*(data++)	= mv.descriminant;							//7. Уровень дискриминации датчика
	*(data++)	= mv.freq;
	*(data++)	= mv.gainRef;								//8. КУ (опорный датчик)
	*(data++)	= mv.sampleTimeRef;						//9. Шаг оцифровки
	*(data++)	= mv.sampleLenRef;							//10. Длина оцифровки
	*(data++)	= mv.sampleDelayRef; 						//11. Задержка оцифровки
	*(data++)	= mv.deadTimeRef;							//12. Мертвая зона датчика
	*(data++)	= mv.descriminantRef;						//13. Уровень дискриминации датчика
	*(data++)	= mv.refFreq;
	*(data++)	= mv.filtrType;							//14. Фильтр
	*(data++)	= mv.packType;								//15. Упаковка
	*(data++)	= mv.cmSPR;								//16. Количество волновых картин на оборот головки в режиме цементомера
	*(data++)	= mv.imSPR;								//17. Количество точек на оборот головки в режиме имиджера
	
	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_10(FLWB *flwb)
{
	__packed u16* data = (__packed u16*)flwb->vd.data;

	flwb->dataLen = InitRspMan_10(data) * 2;

	RequestFlashWrite(flwb, data[0]);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_10(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	InitRspMan_10(manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = 19;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_20(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= manReqWord|0x20;	//	1. ответное слово
	*(data++)	= dspMMSEC; 		//	2. Время (0.1мс). младшие 2 байта
	*(data++)	= dspMMSEC>>16;		//	3. Время. старшие 2 байта
	*(data++)  	= shaftMMSEC;		//	4. Время датчика Холла (0.1мс). младшие 2 байта
	*(data++)  	= shaftMMSEC>>16;	//	5. Время датчика Холла. старшие 2 байта
	*(data++)  	= motoRPS;			//	6. Частота вращения двигателя (0.01 об/сек)
	*(data++)  	= motoCur;			//	7. Ток двигателя (мА)
	*(data++)  	= motoCounter;		//	8. Счётчик оборотов двигателя (1/6 об)
	*(data++)  	= GetShaftRPS();	//	9. Частота вращения головки (0.01 об/сек)
	*(data++)  	= GetShaftCount();	//	10. Счётчик оборотов головки (об)
	*(data++)  	= ax;				//	11. AX (уе)
	*(data++)  	= ay;				//	12. AY (уе)
	*(data++)  	= az;				//	13. AZ (уе)
	*(data++)  	= at;				//	14. AT (short 0.01 гр)
	*(data++)	= temp;				//	15. Температура в приборе (short)(0.1гр)
	
	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_20(FLWB *flwb)
{
	__packed u16* data = (__packed u16*)flwb->vd.data;

	flwb->dataLen = InitRspMan_20(data) * 2;

	RequestFlashWrite(flwb, data[0]);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	InitRspMan_20(manTrmData);
 
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

	static byte sensInd = 0;

	rsp.rw = req.rw;

	mtb->data1 = (u16*)&rsp;
	mtb->len1 = sizeof(rsp)/2;
	mtb->data2 = 0;
	mtb->len2 = 0;

	R01 *r01 = curManVec40;
	
	//u16 sz = 18 + r01->rsp.CM.sl;

	if (reqlen == 1 || (reqlen >= 2 && data[1] == 0))
	{
		if (r01 != 0)
		{
			freeR01.Add(r01);

			curManVec40 = 0;
		};
		
		R01* &vec = manVec40[sensInd&1];
		r01 = vec;

		if (r01 != 0/* && r01->rsp.rw == req.rw*/)
		{

			u16 sz = 18 + r01->rsp.CM.sl;

			curManVec40 = r01;

			vec = 0;

			mtb->data2 = ((u16*)&r01->rsp)+1;

			prevOff = 0;

			if (reqlen == 1)
			{
				mtb->len2 = sz;
				prevLen = sz;
			}
			else 
			{
				req40_count1++;

				if (reqlen == 3) maxLen = data[2];

				u16 len = maxLen;

				if (len > sz) len = sz;

				mtb->len2 = len;

				prevLen = len;
			};
		};

		sensInd = (sensInd + 1) & 1;
	}
	else 
	{
		req40_count2++;

		u16 off = prevOff + prevLen;
		u16 len = prevLen;

		if (reqlen == 3)
		{
			off = data[1];
			len = data[2];
		};

		u16 sz = 18 + r01->rsp.CM.sl;

		if (sz >= off && r01 != 0)
		{
			req40_count3++;

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
		
	//Set_Sync_Rot(motoTargetRPS, *curSPR);

	Update_RPS_SPR();

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

			u16 sz = 12 + r01->rsp.IM.dataLen*2;

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
		u16 sz = 12 + r01->rsp.IM.dataLen*2;

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

			mv.numDevice = data[2];

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
		case 0x1:	mv.gain				= data[2];			break;
		case 0x2:	mv.sampleTime		= data[2];			break;
		case 0x3:	mv.sampleLen		= data[2];			break;
		case 0x4:	mv.sampleDelay 		= data[2];			break;
		case 0x5:	mv.deadTime			= data[2];			break;
		case 0x6:	mv.descriminant		= data[2];			break;
		case 0x7:	mv.freq				= data[2];			break;

		case 0x11:	mv.gainRef			= data[2];			break;
		case 0x12:	mv.sampleTimeRef	= data[2];			break;
		case 0x13:	mv.sampleLenRef		= data[2];			break;
		case 0x14:	mv.sampleDelayRef 	= data[2];			break;
		case 0x15:	mv.deadTimeRef		= data[2];			break;
		case 0x16:	mv.descriminantRef	= data[2];			break;
		case 0x17:	mv.refFreq			= data[2];			break;

		case 0x20:	mv.filtrType		= data[2];			break;
		case 0x21:	mv.packType			= data[2];			break;

		case 0x30:	mv.cmSPR = data[2]; Update_RPS_SPR();	break;
		case 0x31:	mv.imSPR = data[2]; Update_RPS_SPR();	break;

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

	SaveMainParams();

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
	manTrmData[1] = mv.numMemDevice;
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

	cmdWriteStart_00 = cmdWriteStart_10 = FLASH_WriteEnable();

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

			mv.numMemDevice = data[2];

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

	//SaveParams();

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

			HW::P5->BSET(7);

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
					HW::P5->BCLR(7);

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
	static R01 *r01 = 0;
	static FLWB *flwb = 0;
	static TM32 tm;

	switch (mainModeState)
	{
		case 0:

			r01 = readyR01.Get();

			if (r01 != 0)
			{
				mainModeState++;
			};

			break;

		case 1:
			
			flwb = AllocFlashWriteBuffer();

			if (flwb != 0)
			{
				flwb->dataLen = r01->rb.len;

				DSP_CopyDataDMA(&r01->rsp, flwb->vd.data, flwb->dataLen);

				mainModeState++;
			};

			break;

		case 2:

			if (DSP_CheckDataComplete())
			{
				if (!RequestFlashWrite(flwb, r01->rsp.rw))
				{
					FreeFlashWriteBuffer(flwb);
				};

				mainModeState++;
			};

			break;

		case 3:

			if ((r01->rsp.rw & 0xFF) == 0x40)
			{
				R01* &vec = manVec40[r01->rsp.CM.sensType&1];

				if (vec != 0)
				{
					freeR01.Add(vec);
					
					vec = 0;
				};
				
				if (vec == 0)
				{
					vec = r01;
				}
				else
				{
					freeR01.Add(r01);

					r01 = 0;
				};
			}
			else if ((r01->rsp.rw & 0xFF) == 0x50)
			{
				if (manVec50 != 0)
				{
					freeR01.Add(manVec50);
				};
				
				manVec50 = r01;
			};

			if (imModeTimeout.Check(10000))
			{
				SetModeCM();
			};

			mainModeState++;

			break;

		case 4:

			if (cmdWriteStart_00)
			{
				FLWB *b = AllocFlashWriteBuffer();

				if (b != 0)
				{
					RequestFlashWrite_00(b);

					cmdWriteStart_00 = false;
				};
			}
			else if (cmdWriteStart_10)
			{
				FLWB *b = AllocFlashWriteBuffer();

				if (b != 0)
				{
					RequestFlashWrite_10(b);

					cmdWriteStart_10 = false;
				};
			}
			else if (cmdWriteStart_20)
			{
				FLWB *b = AllocFlashWriteBuffer();

				if (b != 0)
				{
					RequestFlashWrite_20(b);

					cmdWriteStart_20 = false;
				};
			}
			else if (tm.Check(1001))
			{
				cmdWriteStart_20 = true;
			};

			mainModeState = 0;

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static DSCSPI dscAccel;

//static i16 ax = 0, ay = 0, az = 0, at = 0;


static u8 txAccel[25] = { 0 };
static u8 rxAccel[25];

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool AccelReadReg(byte reg, u16 count)
{
	dscAccel.adr = (reg<<1)|1;
	dscAccel.alen = 1;
	//dscAccel.baud = 8000000;
	dscAccel.csnum = 0;
	dscAccel.wdata = 0;
	dscAccel.wlen = 0;
	dscAccel.rdata = rxAccel;
	dscAccel.rlen = count;

	return SPI_AddRequest(&dscAccel);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool AccelWriteReg(byte reg, u16 count)
{
	dscAccel.adr = (reg<<1)|0;
	dscAccel.alen = 1;
	dscAccel.csnum = 0;
	dscAccel.wdata = txAccel;
	dscAccel.wlen = count;
	dscAccel.rdata = 0;
	dscAccel.rlen = 0;

	return SPI_AddRequest(&dscAccel);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateAccel()
{
	static byte i = 0; 
	static i32 x = 0, y = 0, z = 0, t = 0;
	static i32 fx = 0, fy = 0, fz = 0, ft = 0;

	static TM32 tm;

	switch (i)
	{
		case 0:

			txAccel[0] = 0x52;
			AccelWriteReg(0x2F, 1); // Reset

			i++;

			break;

		case 1:

			if (dscAccel.ready)
			{
				tm.Reset();

				i++;
			};

			break;

		case 2:

			if (tm.Check(35))
			{
				AccelReadReg(0, 4);

				i++;
			};

			break;

		case 3:

			if (dscAccel.ready)
			{
				txAccel[0] = 0;
				AccelWriteReg(0x28, 1); // FILTER SETTINGS REGISTER

				i++;
			};

			break;

		case 4:

			if (dscAccel.ready)
			{
				txAccel[0] = 0;
				AccelWriteReg(0x2D, 1); // CTRL Set PORST to zero

				i++;
			};

			break;

		case 5:

			if (dscAccel.ready)
			{
				AccelReadReg(0x2D, 1);

				i++;
			};

			break;

		case 6:

			if (dscAccel.ready)
			{
				if (rxAccel[0] != 0)
				{
					txAccel[0] = 0;
					AccelWriteReg(0x2D, 1); // CTRL Set PORST to zero
					i--; 
				}
				else
				{
					txAccel[0] = 0;
					AccelWriteReg(0x2E, 1); // Self Test

					tm.Reset();
					i++;
				};
			};

			break;

		case 7:

			if (dscAccel.ready)
			{
				i++;
			};

			break;

		case 8:

			if (tm.Check(10))
			{
				AccelReadReg(6, 11); // X_MSB 

				i++;
			};

			break;

		case 9:

			if (dscAccel.ready)
			{
				t = (rxAccel[0] << 8) | rxAccel[1];
				x = (rxAccel[2] << 24) | (rxAccel[3] << 16) | (rxAccel[4]<<8);
				y = (rxAccel[5] << 24) | (rxAccel[6] << 16) | (rxAccel[7]<<8);
				z = (rxAccel[8] << 24) | (rxAccel[9] << 16) | (rxAccel[10]<<8);

				//x /= 4096;
				//y /= 4096;
				//z /= 4096;

				fx += (x - fx) * 0.05f;
				fy += (y - fy) * 0.05f;
				fz += (z - fz) * 0.05f;

				ax = -(fz / 65536); 
				ay = (fy / 65536); 
				az = -(fx / 65536);

				at = 250 + ((1852 - t) * 1000 + 452) / 905;
				//at = 250 + (1852 - t) * 1.1049723756906077348066298342541f;

				i--;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

static void UpdateI2C()
{
	if (!comdsp.Update())
	{
		if (I2C_Update())
		{
			comdsp.InitHW();

			i2cResetCount++;
		};
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

static void UpdateDSP()
{
	static R01 *r01 = 0;

	static byte i = 0;
	static TM32 tm;

	switch (i)
	{
		case 0:

			r01 = CreateDspReq01(1);

			if (r01 != 0)
			{
				qdsp.Add(&r01->q);

				i++;
			};

			break;

		case 1:

			if (r01->q.ready)
			{
				if (r01->q.crcOK)
				{
//					if (tm.Check(2000))
					{
						readyR01.Add(r01);
					}
					//else
					//{
					//	freeR01.Add(r01);
					//};
				};
				
				i = 0;
			};

			break;
	};

	qdsp.Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const u32 dspFlashPages[] = {
#include "G26K1BF592.LDR.H"
};

u16 dspFlashLen = 0;
u16 dspFlashCRC = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashDSP()
{
	TM32 tm;

	REQ *req = 0;

	dspFlashLen = sizeof(dspFlashPages);
	dspFlashCRC = GetCRC16(dspFlashPages, dspFlashLen);

	tm.Reset();

	while (!tm.Check(100));

	req = CreateDspReq05(2);

	qdsp.Add(req); while(!req->ready) { qdsp.Update(); };

	if (req->crcOK)
	{
		RspDsp05 *rsp = (RspDsp05*)req->rb->data;

		//dspFlashLen = rsp->flashLen;
		//dspFlashCRC = rsp->flashCRC;

		if (rsp->flashCRC != dspFlashCRC || rsp->flashLen != dspFlashLen)
		{
			u16 count = dspFlashLen;
			u16 adr = 0;
			byte *p = (byte*)dspFlashPages;

			while (count > 0)
			{
				u16 len = (count > 256) ? 256 : count;

				req = CreateDspReq06(adr, len, p, 0, 0, 2);

				qdsp.Add(req); while(!req->ready) { qdsp.Update(); };

				count -= len;
				p += len;
				adr += len;
			};

			req = CreateDspReq07();

			qdsp.Add(req); while(!req->ready) { qdsp.Update();	};

			DisableDSP();
			
			tm.Reset();

			while (!tm.Check(1));

			EnableDSP();
			
			tm.Reset();

			while (!tm.Check(100));
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const u32 motoFlashPages[] = {
#include "G26K2LPC824.BIN.H"
};

u16 motoFlashLen = 0;
u16 motoFlashCRC = 0;

#define SGUID	0x0A89D55DD5274785 
#define MGUID	0x9119CC18AC79DE35

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashMoto()
{
	static ReqBootMotoHS		reqHS;
	static RspBootMotoHS		rspHS;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer	rb;

	const unsigned __int64 masterGUID = MGUID;
	const unsigned __int64 slaveGUID = SGUID;

	TM32 tm;

	REQ *req = 0;

	motoFlashLen = sizeof(motoFlashPages);
	motoFlashCRC = GetCRC16(motoFlashPages, motoFlashLen);

	tm.Reset();

	bool hs = true;

	while (!tm.Check(200))
	{
		reqHS.guid = masterGUID;
		reqHS.crc = GetCRC16(&reqHS, sizeof(reqHS) - sizeof(reqHS.crc));
		wb.data = &reqHS;
		wb.len = sizeof(reqHS);

		commoto.Write(&wb);

		while (commoto.Update());

		rb.data = &rspHS;
		rb.maxLen = sizeof(rspHS);
		commoto.Read(&rb, MS2RT(5), US2RT(100));

		while (commoto.Update());

		if (rb.recieved && rb.len == sizeof(rspHS) && GetCRC16(&rspHS, sizeof(rspHS)) == 0 && rspHS.guid == slaveGUID)
		{
			hs = true;
			break;
		};
	};

	if (hs)
	{
		req = CreateBootMotoReq01(motoFlashLen, 2);

		qmoto.Add(req); while(!req->ready) { qmoto.Update(); };

		if (req->crcOK)
		{
			RspBootMoto *rsp = (RspBootMoto*)req->rb->data;

			if (rsp->F01.flashCRC != motoFlashCRC || rsp->F01.flashLen != motoFlashLen)
			{
				u16 count = motoFlashLen/4;
				u32 adr = 0;
				const u32 *p = motoFlashPages;

				while (count > 0)
				{
					u16 len = (count > 16) ? 16 : count;

					for(u32 i = 3; i > 0; i--)
					{
						req = CreateBootMotoReq02(adr, len, p, 3);

						qmoto.Add(req); while(!req->ready) { qmoto.Update(); };

						RspBootMoto *rsp = (RspBootMoto*)req->rb->data;

						if (req->crcOK && rsp->F02.status) { break;	}
					};

					tm.Reset();

					while (!tm.Check(1));

					count -= len;
					p += len;
					adr += len*4;
				};
			};
		};

		req = CreateBootMotoReq03();

		qmoto.Add(req); while(!req->ready) { qmoto.Update();	};

		tm.Reset();

		while (!tm.Check(1));
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitMainVars()
{
	mv.numDevice		= 0;
	mv.numMemDevice		= 0;
	mv.gain				= 0; 
	mv.sampleTime		= 8; 
	mv.sampleLen		= 500; 
	mv.sampleDelay 		= 400; 
	mv.deadTime			= 400; 
	mv.descriminant		= 400; 
	mv.freq				= 500; 
	mv.gainRef			= 0; 
	mv.sampleTimeRef	= 8; 
	mv.sampleLenRef		= 500; 
	mv.sampleDelayRef 	= 400; 
	mv.deadTimeRef		= 400; 
	mv.descriminantRef	= 400; 
	mv.refFreq			= 500; 
	mv.filtrType		= 0;
	mv.packType			= 0;
	mv.cmSPR			= 32;
	mv.imSPR			= 100;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadVars()
{

	static DSCI2C dsc;
	static DSCSPI spi;
	static u16 romAdr = 0;
	
	byte buf[sizeof(mv)*2+4];

	//spi.adr = ((u32)ReverseWord(FRAM_SPI_MAINVARS_ADR)<<8)|0x9F;
	//spi.alen = 1;
	//spi.csnum = 1;
	//spi.wdata = 0;
	//spi.wlen = 0;
	//spi.rdata = buf;
	//spi.rlen = 9;

	//if (SPI_AddRequest(&spi))
	//{
	//	while (!spi.ready);
	//};

	bool loadVarsOk = false;

	spi.adr = ((u32)ReverseWord(FRAM_SPI_MAINVARS_ADR)<<8)|3;
	spi.alen = 4;
	spi.csnum = 1;
	spi.wdata = 0;
	spi.wlen = 0;
	spi.rdata = buf;
	spi.rlen = sizeof(buf);

	if (SPI_AddRequest(&spi))
	{
		while (!spi.ready);
	};

	PointerCRC p(buf);

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv, sizeof(mv)+2);

		if (p.CRC.w == 0) { loadVarsOk = true; break; };
	};

	if (!loadVarsOk)
	{
		romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

		dsc.wdata = &romAdr;
		dsc.wlen = sizeof(romAdr);
		dsc.wdata2 = 0;
		dsc.wlen2 = 0;
		dsc.rdata = buf;
		dsc.rlen = sizeof(buf);
		dsc.adr = 0x50;


		if (I2C_AddRequest(&dsc))
		{
			while (!dsc.ready) { I2C_Update(); };
		};

	//	bool c = false;

		p.b = buf;

		for (byte i = 0; i < 2; i++)
		{
			p.CRC.w = 0xFFFF;
			p.ReadArrayB(&mv, sizeof(mv)+2);

			if (p.CRC.w == 0) { loadVarsOk = true; break; };
		};
	};

	if (!loadVarsOk)
	{
		InitMainVars();

		svCount = 2;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SaveVars()
{
	static DSCI2C dsc;
	static DSCSPI spi,spi2;
	static u16 romAdr = 0;
	static byte buf[100];

	static byte i = 0;
	static TM32 tm;

	PointerCRC p(buf);

	switch (i)
	{
		case 0:

			if (svCount > 0)
			{
				svCount--;
				i++;
			};

			break;

		case 1:

			for (byte j = 0; j < 2; j++)
			{
				p.CRC.w = 0xFFFF;
				p.WriteArrayB(&mv, sizeof(mv));
				p.WriteW(p.CRC.w);
			};

			spi.adr = ((u32)ReverseWord(FRAM_SPI_MAINVARS_ADR)<<8)|2;
			spi.alen = 4;
			spi.csnum = 1;
			spi.wdata = buf;
			spi.wlen = p.b-buf;
			spi.rdata = 0;
			spi.rlen = 0;

			romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

			dsc.wdata = &romAdr;
			dsc.wlen = sizeof(romAdr);
			dsc.wdata2 = buf;
			dsc.wlen2 = p.b-buf;
			dsc.rdata = 0;
			dsc.rlen = 0;
			dsc.adr = 0x50;

			spi2.adr = 6;
			spi2.alen = 1;
			spi2.csnum = 1;
			spi2.wdata = 0;
			spi2.wlen = 0;
			spi2.rdata = 0;
			spi2.rlen = 0;

			tm.Reset();

			SPI_AddRequest(&spi2);

			i++;

			break;

		case 2:

			if (spi2.ready || tm.Check(10))
			{
				SPI_AddRequest(&spi);

				i++;
			};

			break;

		case 3:

			if (spi.ready || tm.Check(10))
			{
				I2C_AddRequest(&dsc);
				
				i++;
			};

			break;

		case 4:

			if (dsc.ready || tm.Check(100))
			{
				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
		CALL( UpdateHardware();	);
		CALL( UpdateAccel();	);
		CALL( UpdateI2C();		);
		CALL( SaveVars();		);
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
		CALL( UpdateDSP();		);
		CALL( UpdateParams();	);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update()
{
	NAND_Idle();	
	UpdateEMAC();
	UpdateTraps();
	UpdateMisc();
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

	static byte buf[100];

	//volatile byte * const FLD = (byte*)0x60000000;	
	
	static ComPort commem;

	DSCSPI dsc, dsc2;

	TM32 tm;

	//__breakpoint(0);

	DisableDSP();

	InitHardware();

	LoadVars();

	InitEMAC();

	InitTraps();

	FLASH_Init();

	InitRmemList();

	Update_RPS_SPR();

	commoto.Connect(ComPort::ASYNC, 0, 1562500, 0, 1);
	comdsp.Connect(ComPort::ASYNC, 2, 6250000, 2, 1);
	commem.Connect(ComPort::ASYNC, 1, 6250000, 0, 1);

	EnableDSP();

	FlashMoto();

	FlashDSP();

	u32 fc = 0;

	//ComPort::WriteBuffer wb;

	while (1)
	{
		FPS_PIN_SET();

		Update();

		FPS_PIN_CLR();

		fc++;

		//commem.Update();

		if (tm.Check(1000))
		{ 
			fps = fc; fc = 0; 

			//dsc.adr = 6;
			//dsc.alen = 1;
			//dsc.csnum = 1;
			//dsc.wdata = 0;
			//dsc.wlen = 0;
			//dsc.rdata = 0;
			//dsc.rlen = 0;

			//SPI_AddRequest(&dsc);
		};

	}; // while (1)
}
