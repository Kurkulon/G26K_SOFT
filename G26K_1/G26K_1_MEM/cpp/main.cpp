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

#include "SEGGER_RTT.h"

#ifdef WIN32

#include <conio.h>
//#include <stdio.h>

static const bool __WIN32__ = true;

#else

static const bool __WIN32__ = false;

//#pragma diag_suppress 546,550,177

#endif

enum { VERSION = 0x103 };

//#pragma O3
//#pragma Otime

#ifndef _DEBUG
	static const bool __debug = false;
#else
	static const bool __debug = true;
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct MainVars // NonVolatileVars  
{
	u32 timeStamp;

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
	u16 fireVoltage;
	u16 motoLimCur;
	u16 motoMaxCur;
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

//static void* eepromData = 0;
//static u16 eepromWriteLen = 0;
//static u16 eepromReadLen = 0;
//static u16 eepromStartAdr = 0;

//static MTB mtb;

//static u16 manBuf[16];

u16 manRcvData[10];
u16 manTrmData[50];
static u16 manTrmBaud = 0;
static u16 memTrmBaud = 0;

u16 txbuf[128 + 512 + 16];

static ComPort comdsp;
static ComPort commoto;

static RequestQuery qmoto(&commoto);
static RequestQuery qdsp(&comdsp);
//static RequestQuery qmem(&commem);

//static R01 r02[8];

static Ptr<UNIBUF> manVec40[2];

static Ptr<UNIBUF> curManVec40;
static Ptr<UNIBUF> manVec50;
static Ptr<UNIBUF> curManVec50;

//static RspMan60 rspMan60;

//static byte curRcv[3] = {0};
//static byte curVec[3] = {0};

//static List<R01> freeR01;
//static List<R01> readyR01;
static ListPtr<REQ> readyR01;

//static RMEM rmem[4];
//static List<RMEM> lstRmem;
//static List<RMEM> freeRmem;

//static byte fireType = 0;

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

//static u16 motoEnable = 0;		// ��������� �������� ��� ���������
static u16 motoTargetRPS = 0;		// �������� ������� ���������
static u16 motoRPS = 0;				// ������� ���������, ��/���
static u16 motoCur = 0;				// ��� ���������, ��
//static u16 motoStat = 0;			// ������ ���������: 0 - ����, 1 - ���
static u16 motoCounter = 0;			// ������� �������� ��������� 1/6 �������
//static u16 cmSPR = 32;			// ���������� �������� ������ �� ������ ������� � ������ �����������
//static u16 imSPR = 100;			// ���������� ����� �� ������ ������� � ������ ��������
//static u16 *curSPR = &cmSPR;		// ���������� ��������� ���������� �� ������ � ������� ������
static u16 motoVoltage = 90;
static u16 motoRcvCount = 0;

static u16 curFireVoltage = 500;

static u32 dspMMSEC = 0;
static u32 shaftMMSEC = 0;

const u16 dspReqWord = 0xAD00;
//const u16 dspReqMask = 0xFF00;

static u16 manReqWord = 0xAD00;
static u16 manReqMask = 0xFF00;

static u16 memReqWord = 0x3D00;
static u16 memReqMask = 0xFF00;

//static u16 numDevice = 0;
static u16 verDevice = VERSION;

//static u16 numMemDevice = 0;
static u16 verMemDevice = 0x100;

//static u32 manCounter = 0;
//static u32 fireCounter = 0;

static byte mainModeState = 0;
static byte dspStatus = 0;

static bool cmdWriteStart_00 = false;
static bool cmdWriteStart_10 = false;
static bool cmdWriteStart_20 = false;

static u32 dspRcv40 = 0;
static u32 dspRcv50 = 0;
static u16 dspRcvCount = 0;


//static u32 rcvCRCER = 0;

//static u32 chnlCount[4] = {0};

//static u32 crcErr02 = 0;
//static u32 crcErr03 = 0;
static u32 crcErr06 = 0;
static u32 wrtErr06 = 0;

static u32 notRcv02 = 0;
//static u32 lenErr02 = 0;

static i16 ax = 0, ay = 0, az = 0, at = 0;
i16 temperature = 0;
i16 cpuTemp = 0;
i16 temp = 0;

static byte svCount = 0;


struct AmpTimeMinMax { bool valid; u16 ampMax, ampMin, timeMax, timeMin; };

static AmpTimeMinMax sensMinMax[2] = { {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0} };
static AmpTimeMinMax sensMinMaxTemp[2] = { {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0} };

static u32 testDspReqCount = 0;

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

//static void Response_0(u16 rw, MTB &mtb)
//{
//	__packed struct Rsp {u16 rw; u16 device; u16 session; u32 rcvVec; u32 rejVec; u32 wrVec; u32 errVec; u16 wrAdr[3]; u16 numDevice; u16 version; u16 temp; byte status; byte flags; RTC rtc; };
//
//	Rsp &rsp = *((Rsp*)&txbuf);
//
//	rsp.rw = rw;
//	rsp.device = GetDeviceID();  
//	rsp.session = FLASH_Session_Get();	  
//	rsp.rcvVec =  FLASH_Vectors_Recieved_Get();
//	rsp.rejVec = FLASH_Vectors_Rejected_Get();
//	rsp.wrVec = FLASH_Vectors_Saved_Get();
//	rsp.errVec = FLASH_Vectors_Errors_Get();
//	*((__packed u64*)rsp.wrAdr) = FLASH_Current_Adress_Get();
//	rsp.temp = temp*5/2;
//	rsp.status = FLASH_Status();
//
//	GetTime(&rsp.rtc);
//
//	mtb.data1 = txbuf;
//	mtb.len1 = sizeof(rsp)/2;
//	mtb.data2 = 0;
//	mtb.len2 = 0;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackDspReq01(Ptr<REQ> &q)
{
	RspDsp01 &rsp = *((RspDsp01*)q->rb.data);
	 
	if (q->rb.recieved)
	{
		if (rsp.rw == (dspReqWord|0x40))
		{
			q->crcOK = (q->rb.len == (rsp.CM.sl*2 + 2 + sizeof(rsp.CM)-sizeof(rsp.CM.data)));
			q->rsp->dataLen = q->rb.len;

			dspStatus |= 1;
			dspRcv40++;
			dspRcvCount++;
			
			dspMMSEC = rsp.CM.time;
			shaftMMSEC = rsp.CM.hallTime;
		}
		else if (rsp.rw == (dspReqWord|0x50))
		{
			q->crcOK = (q->rb.len == (rsp.IM.dataLen*4 + 2 + sizeof(rsp.IM)-sizeof(rsp.IM.data)));
			q->rsp->dataLen = q->rb.len;

			dspStatus |= 1;
			dspRcv50++;
			dspRcvCount++;

			dspMMSEC = rsp.IM.time;
			shaftMMSEC = rsp.IM.hallTime;
		}
		else
		{
			q->crcOK = GetCRC16(q->rb.data, q->rb.len) == 0;

			if (q->crcOK && rsp.rw == (dspReqWord|1))
			{
				curFireVoltage = rsp.v01.fireVoltage;
				motoVoltage = rsp.v01.motoVoltage;
				
				dspStatus |= 1;
				dspRcvCount++;
			};
		};
	}
	else
	{
		q->crcOK = false;
		notRcv02++;
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

			//q.Free();
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Ptr<REQ> CreateDspReq01(u16 tryCount)
{
	Ptr<REQ> rq;
	
	rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp.Alloc();

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqDsp01 &req = *((ReqDsp01*)rq->reqData);
	RspDsp01 &rsp = *((RspDsp01*)(rq->rsp->GetDataPtr()));

	rq->rsp->dataLen = 0;
	
	REQ &q = *rq;

	q.CallBack = CallBackDspReq01;
	//q.rb = &rb;
	//q.wb = &wb;
	q.preTimeOut = MS2RT(1);
	q.postTimeOut = US2RT(100);
	q.ready = false;
	q.tryCount = tryCount;
	//q.ptr = &r;
	q.checkCRC = false;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);
	q.rb.recieved = false;
	
	req.rw				= dspReqWord|1;
	req.len				= sizeof(req);
	req.version			= req.VERSION;
	req.mode 			= mode;
	req.ax				= ax;
	req.ay				= ay;
	req.az				= az;
	req.at				= at;
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
	req.refthr			= mv.descriminantRef;
	req.refdescr		= mv.deadTimeRef;
	req.refFreq			= mv.refFreq;
	req.vavesPerRoundCM = mv.cmSPR;
	req.vavesPerRoundIM = mv.imSPR;
	req.filtrType		= mv.filtrType;
	req.packType		= mv.packType;
	req.fireVoltage		= mv.fireVoltage;

	req.crc	= GetCRC16(&req, sizeof(ReqDsp01)-2);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Ptr<UNIBUF> CreateTestDspReq01()
{
	Ptr<UNIBUF> rq;
	
	rq.Alloc();

	if (!rq.Valid()) { return rq; };

	RspDsp01 &rsp = *((RspDsp01*)(rq->GetDataPtr()));

	rsp.rw = manReqWord|0x40;
	rsp.CM.time += 1;
	rsp.CM.hallTime += 1;
	rsp.CM.motoCount += 1;
	rsp.CM.headCount += 1;
	rsp.CM.ax += 1;
	rsp.CM.ay += 1;
	rsp.CM.az += 1;
	rsp.CM.at += 1;
	rsp.CM.sensType = 0;
	rsp.CM.angle += 1;
	rsp.CM.maxAmp += 1;
	rsp.CM.fi_amp += 1;
	rsp.CM.fi_time += 1;
	rsp.CM.gain += 1;
	rsp.CM.st = 1;
	rsp.CM.sl = 4;
	rsp.CM.sd = 0;
	rsp.CM.pakType = 0;
	rsp.CM.pakLen = 0;
	rsp.CM.data[0] += 1;
	rsp.CM.data[1] += 1;
	rsp.CM.data[2] += 1;
	rsp.CM.data[3] += 1;

	rq->dataLen = (22+1500)*2;
	
	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackDspReq05(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qdsp.Add(q);
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Ptr<REQ> CreateDspReq05(u16 tryCount)
{
	Ptr<REQ> rq;
	
	rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp.Alloc();

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqDsp05 &req = *((ReqDsp05*)rq->reqData);
	RspDsp05 &rsp = *((RspDsp05*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackDspReq05;
	q.preTimeOut = MS2RT(10);
	q.postTimeOut = US2RT(100);
	//q.rb = &rb;
	//q.wb = &wb;
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.rw = dspReqWord|5;
	req.crc	= GetCRC16(&req, sizeof(req)-2);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackDspReq06(Ptr<REQ> &q)
{
	bool retry = false;

	if (!q->crcOK) 
	{
		crcErr06++;

		retry = true;
	}
	else
	{
		RspDsp06 &rsp = *((RspDsp06*)q->rb.data);

		if (rsp.res != 0) wrtErr06++, retry = true;
	};

	if (retry && q->tryCount > 0)
	{
		q->tryCount--;
		qdsp.Add(q);
	};

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Ptr<REQ> CreateDspReq06(u16 stAdr, u16 count, void* data, u16 count2, void* data2, u16 tryCount)
{
	Ptr<REQ> rq;
	
	rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp.Alloc();

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqDsp06 &req = *((ReqDsp06*)rq->reqData);
	RspDsp06 &rsp = *((RspDsp06*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackDspReq06;
	q.preTimeOut = MS2RT(500);
	q.postTimeOut = US2RT(100);
	//q.rb = &rb;
	//q.wb = &wb;
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

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

	q.wb.data = &req;
	q.wb.len = len+2;

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Ptr<REQ> CreateDspReq07()
{
	Ptr<REQ> rq;
	
	rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp.Alloc();

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqDsp07 &req = *((ReqDsp07*)rq->reqData);
	//RspDsp06 &rsp = *((RspDsp06*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = 0;
	//q.preTimeOut = US2RT(500);
	//q.postTimeOut = US2RT(100);
	//q.rb = 0;
	//q.wb = &wb;
	q.ready = false;
	q.tryCount = 0;
	q.checkCRC = false;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = 0;
	q.rb.maxLen = 0;
	
	//rb.data = &rsp;
	//rb.maxLen = sizeof(rsp);

	req.rw	= dspReqWord|7;

	req.crc = GetCRC16(&req, sizeof(ReqDsp07)-2);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CallBackMotoReq(Ptr<REQ> &q)
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
		RspMoto &rsp = *((RspMoto*)q->rb.data);

		if (rsp.rw == 0x101)
		{
			motoRPS = rsp.rpm;
			motoCur = rsp.current;
			//motoStat = rsp.mororStatus;
			motoCounter = rsp.motoCounter;
			motoRcvCount++;
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateMotoReq()
{
	Ptr<REQ> rq;
	
	rq.Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp.Alloc();

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqMoto &req = *((ReqMoto*)rq->reqData);
	RspMoto &rsp = *((RspMoto*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackMotoReq;
	//q.rb = &rb;
	//q.wb = &wb;
	q.preTimeOut = MS2RT(1);
	q.postTimeOut = 1;
	q.ready = false;
	q.checkCRC = true;
	q.updateCRC = false;
	q.tryCount = 1;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);
	
	req.rw = 0x101;
	req.enableMotor	= 1;
	req.tRPM = motoTargetRPS;
	req.limCurrent = mv.motoLimCur;
	req.maxCurrent = mv.motoMaxCur;
	req.crc	= GetCRC16(&req, sizeof(req)-2);

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackBootMotoReq01(Ptr<REQ> &q)
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

Ptr<REQ> CreateBootMotoReq01(u16 flashLen, u16 tryCount)
{
	Ptr<REQ> rq;
	
	rq.Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp.Alloc();

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqBootMoto &req = *((ReqBootMoto*)rq->reqData);
	RspBootMoto &rsp = *((RspBootMoto*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackBootMotoReq01;
	q.preTimeOut = MS2RT(10);
	q.postTimeOut = US2RT(100);
	//q.rb = &rb;
	//q.wb = &wb;
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req.F01) + sizeof(req.func);
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.func = 1;
	req.F01.flashLen = flashLen;
	req.F01.align = ~flashLen;

	req.F01.crc	= GetCRC16(&req, q.wb.len - sizeof(req.F01.crc));

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackBootMotoReq02(Ptr<REQ> &q)
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

Ptr<REQ> CreateBootMotoReq02(u16 stAdr, u16 count, const u32* data, u16 tryCount)
{
	Ptr<REQ> rq;
	
	rq.Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp.Alloc();

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqBootMoto &req = *((ReqBootMoto*)rq->reqData);
	RspBootMoto &rsp = *((RspBootMoto*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackBootMotoReq02;
	q.preTimeOut = MS2RT(300);
	q.postTimeOut = US2RT(100);
	//q.rb = &rb;
	//q.wb = &wb;
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

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

	q.wb.data = &req;
	q.wb.len = len+sizeof(req.F02.crc);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Ptr<REQ> CreateBootMotoReq03()
{
	Ptr<REQ> rq;
	
	rq.Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp.Alloc();

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqBootMoto &req = *((ReqBootMoto*)rq->reqData);
	RspBootMoto &rsp = *((RspBootMoto*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = 0;
	q.preTimeOut = MS2RT(10);
	q.postTimeOut = US2RT(100);
	//q.rb = &rb;
	//q.wb = &wb;
	q.ready = false;
	q.tryCount = 1;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);
	
	req.func = 3;
	req.F03.align += 1; 

	u16 len = sizeof(req.F03) + sizeof(req.func) - sizeof(req.F03.crc);

	req.F03.crc = GetCRC16(&req, len);

	q.wb.data = &req;
	q.wb.len = len + sizeof(req.F03.crc);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void InitRmemList()
//{
//	for (u16 i = 0; i < ArraySize(r02); i++)
//	{
//		freeR01.Add(&r02[i]);
//	};
//}

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

static void RequestFlashWrite_00(Ptr<UNIBUF> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->data+flwb->dataOffset);

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

	*(data++)	= (manReqWord & manReqMask) | 0x10;		//	1. �������� �����	
	*(data++)	= mv.gain;								//	2. ��(������������� ������)
	*(data++)	= mv.sampleTime;						//	3. ��� ���������
	*(data++)	= mv.sampleLen;							//	4. ����� ���������
	*(data++)	= mv.sampleDelay; 						//	5. �������� ���������
	*(data++)	= mv.deadTime;							//	6. ������� ���� �������
	*(data++)	= mv.descriminant;						//	7. ������� ������������� �������
	*(data++)	= mv.freq;								//	8. ������� ����������(���)
	*(data++)	= mv.gainRef;							//	9. ��(������� ������)
	*(data++)	= mv.sampleTimeRef;						//	10. ��� ���������
	*(data++)	= mv.sampleLenRef;						//	11. ����� ���������
	*(data++)	= mv.sampleDelayRef; 					//	12. �������� ���������
	*(data++)	= mv.deadTimeRef;						//	13. ������� ���� �������
	*(data++)	= mv.descriminantRef;					//	14. ������� ������������� �������
	*(data++)	= mv.refFreq;							//	15. ������� ����������(���)
	*(data++)	= mv.filtrType;							//	16. ������
	*(data++)	= mv.packType;							//	17. ��������
	*(data++)	= mv.cmSPR;								//	18. ���������� �������� ������ �� ������ ������� � ������ �����������
	*(data++)	= mv.imSPR;								//	19. ���������� ����� �� ������ ������� � ������ ��������
	*(data++)	= mv.fireVoltage;						//	20. ���������� ����������(�)
	*(data++)	= mv.motoLimCur;						//	21. ����������� ���� ��������� (��)
	*(data++)	= mv.motoMaxCur;						//	22. ��������� ��� ��������� (��)

	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_10(Ptr<UNIBUF> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->data+flwb->dataOffset);

	flwb->dataLen = InitRspMan_10(data) * 2;

	RequestFlashWrite(flwb, data[0]);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_10(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	len = InitRspMan_10(manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = len;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_20(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= manReqWord|0x20;				//	1. �������� �����	
	*(data++)	= dspMMSEC; 					//	2. �����(0.1��).������� 2 �����
	*(data++)	= dspMMSEC>>16;					//	3. �����.������� 2 �����
	*(data++)  	= shaftMMSEC;					//	4. ����� ������� �����(0.1��).������� 2 �����
	*(data++)  	= shaftMMSEC>>16;				//	5. ����� ������� �����.������� 2 �����
	*(data++)  	= motoRPS;						//	6. ������� �������� ���������(0.01 �� / ���)
	*(data++)  	= motoCur;						//	7. ��� ���������(��)
	*(data++)  	= motoCounter;					//	8. ������� �������� ���������(1 / 6 ��)
	*(data++)  	= GetShaftRPS();				//	9. ������� �������� �������(0.01 �� / ���)
	*(data++)  	= GetShaftCount();				//	10. ������� �������� �������(��)
	*(data++)  	= ax;							//	11. AX(��)
	*(data++)  	= ay;							//	12. AY(��)
	*(data++)  	= az;							//	13. AZ(��)
	*(data++)  	= at;							//	14. AT(short 0.01 ��)
	*(data++)	= temp;							//	15. ����������� � �������(short)(0.1��)
	*(data++)	= sensMinMax[0].ampMax;			//	16. ��������� �������������� ������� �������� �� ���� �����(�.�)
	*(data++)	= sensMinMax[0].ampMin;			//	17. ��������� �������������� ������� ������� �� ���� �����(�.�)
	*(data++)	= sensMinMax[0].timeMax;		//	18. ����� �������������� ������� �������� �� ������� ����������(0.05 ���)
	*(data++)	= sensMinMax[0].timeMin;		//	19. ����� �������������� ������� ������� �� ������� ����������(0.05 ���)
	*(data++)	= sensMinMax[1].ampMax;			//	20. ��������� �������� ������� �������� �� ���� �����(�.�)
	*(data++)	= sensMinMax[1].ampMin;			//	21. ��������� �������� ������� ������� �� ���� �����(�.�)
	*(data++)	= sensMinMax[1].timeMax;		//	22. ����� �������� ������� �������� �� ������� ����������(0.05 ���)
	*(data++)	= sensMinMax[1].timeMin;		//	23. ����� �������� ������� ������� �� ������� ����������(0.05 ���)
	*(data++)	= GetShaftState();				//	24. ��������� ������� �����(0, 1)
	*(data++)	= curFireVoltage;				//	25. ���������� ����������(�)
	*(data++)	= motoVoltage;					//	26. ���������� ���������(�)
	*(data++)	= dspRcvCount;					//	27. ������� �������� DSP
	*(data++)	= motoRcvCount;					//	28. ������� �������� ���������
	*(data++)	= GetRcvManQuality();			//	29. �������� ������� ������� ���������� (%)

	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_20(Ptr<UNIBUF> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->data+flwb->dataOffset);

	flwb->dataLen = InitRspMan_20(data) * 2;

	RequestFlashWrite(flwb, data[0]);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	if (sensMinMaxTemp[0].valid) { sensMinMax[0] = sensMinMaxTemp[0]; };
	if (sensMinMaxTemp[1].valid) { sensMinMax[1] = sensMinMaxTemp[1]; };

	len = InitRspMan_20(manTrmData);

	sensMinMaxTemp[0].ampMax = 0;
	sensMinMaxTemp[0].ampMin = ~0;
	sensMinMaxTemp[0].timeMax = 0;
	sensMinMaxTemp[0].timeMin = ~0;
	sensMinMaxTemp[0].valid = false;

	sensMinMaxTemp[1].ampMax = 0;
	sensMinMaxTemp[1].ampMin = ~0;
	sensMinMaxTemp[1].timeMax = 0;
	sensMinMaxTemp[1].timeMin = ~0;
	sensMinMaxTemp[1].valid = false;

	mtb->data1 = manTrmData;
	mtb->len1 = len;
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

	//Ptr<UNIBUF> &r01 = curManVec40;
	
	//u16 sz = 18 + r01->rsp.CM.sl;

	if (reqlen == 1 || (reqlen >= 2 && data[1] == 0))
	{
		curManVec40 = manVec40[sensInd&1];

		manVec40[sensInd&1].Free();

		if (!curManVec40.Valid())
		{
			sensInd = (sensInd + 1) & 1;

			curManVec40 = manVec40[sensInd];

			manVec40[sensInd&1].Free();
		};

		if (curManVec40.Valid())
		{
			RspDsp01 &rsp = *((RspDsp01*)(curManVec40->GetDataPtr()));

			u16 sz = 21 + rsp.CM.sl;

			mtb->data2 = ((u16*)&rsp)+1;

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
	else if (curManVec40.Valid())
	{
		RspDsp01 &rsp = *((RspDsp01*)(curManVec40->GetDataPtr()));

		req40_count2++;

		u16 off = prevOff + prevLen;
		u16 len = prevLen;

		if (reqlen == 3)
		{
			off = data[1];
			len = data[2];
		};

		u16 sz = 21 + rsp.CM.sl;

		if (sz >= off)
		{
			req40_count3++;

			u16 ml = sz - off;

			if (len > ml) len = ml;

			mtb->data2 = (u16*)&rsp + data[1]+1;
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

	if (reqlen == 1 || (reqlen >= 2 && data[1] == 0))
	{
		curManVec50 = manVec50;

		manVec50.Free();

		if (curManVec50.Valid())
		{
			RspDsp01 &rsp = *((RspDsp01*)curManVec50->GetDataPtr());

			mtb->data2 = ((u16*)&rsp)+1;

			prevOff = 0;

			u16 sz = 12 + rsp.IM.dataLen*2;

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
	else if (curManVec50.Valid())
	{
		RspDsp01 &rsp = *((RspDsp01*)curManVec50->GetDataPtr());

		u16 off = prevOff + prevLen;
		u16 len = prevLen;
		u16 sz = 12 + rsp.IM.dataLen*2;

		if (reqlen == 3)
		{
			off = data[1];
			len = data[2];
		};

		if (sz >= off)
		{
			u16 ml = sz - off;

			if (len > ml) len = ml;

			mtb->data2 = (u16*)&rsp + data[1]+1;
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

			manTrmBaud = data[2] - 1;	//SetTrmBoudRate(data[2]-1);

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

		case 0x30:	mv.cmSPR 			= data[2]; Update_RPS_SPR();	break;
		case 0x31:	mv.imSPR 			= data[2]; Update_RPS_SPR();	break;

		case 0x40:	mv.fireVoltage		= data[2];			break;
		case 0x41:	mv.motoLimCur		= data[2];			break;
		case 0x42:	mv.motoMaxCur		= data[2];			break;

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

	if (r) { mtb->baud = manTrmBaud; };

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
	rsp.temp = (temp+5)/10;
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

			memTrmBaud = data[2] - 1;	//SetTrmBoudRate(data[2]-1);

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

	if (r) { mtb->baud = memTrmBaud; };

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

//			HW::P5->BSET(7);

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
					//HW::P5->BCLR(7);

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
				//manTrmData[0] = 1;
				//manTrmData[1] = 0;
				//mtb.len1 = 2;
				//mtb.data1 = manTrmData;
				SendManData2(&mtb);

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
	static Ptr<REQ> rq;
	//static Ptr<UNIBUF> flwb;
	static TM32 tm;
	static RspDsp01 *rsp = 0;

	switch (mainModeState)
	{
		case 0:

			rq = readyR01.Get();

			if (rq.Valid())
			{
				rsp = (RspDsp01*)(rq->rsp->GetDataPtr());

				RequestFlashWrite(rq->rsp, rsp->rw);

				mainModeState++;
			};

			break;

		case 1:

			if ((rsp->rw & 0xFF) == 0x40)
			{
				byte n = rsp->CM.sensType & 1;

				manVec40[n] = rq->rsp;

				AmpTimeMinMax& mm = sensMinMaxTemp[n];

				u16 amp = rsp->CM.maxAmp;
				u16 time = rsp->CM.fi_time;

				if (amp > mm.ampMax) mm.ampMax = amp;
				if (amp < mm.ampMin) mm.ampMin = amp;
				if (time > mm.timeMax) mm.timeMax = time;
				if (time < mm.timeMin) mm.timeMin = time;

				mm.valid = true;
			}
			else if ((rsp->rw & 0xFF) == 0x50)
			{
				manVec50 = rq->rsp;
			};

			if (imModeTimeout.Check(10000))
			{
				SetModeCM();
			};

			rq.Free();

			mainModeState++;

			break;

		case 2:

			if (cmdWriteStart_00)
			{
				Ptr<UNIBUF> b; b.Alloc();

				if (b.Valid())
				{
					RequestFlashWrite_00(b);

					cmdWriteStart_00 = false;
				};
			}
			else if (cmdWriteStart_10)
			{
				Ptr<UNIBUF> b; b.Alloc();

				if (b.Valid())
				{
					RequestFlashWrite_10(b);

					cmdWriteStart_10 = false;
				};
			}
			else if (cmdWriteStart_20)
			{
				Ptr<UNIBUF> b; b.Alloc();

				if (b.Valid())
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
	static i32 fx = 0, fy = 0, fz = 0;

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

				fx += (x - fx) / 8;
				fy += (y - fy) / 8;
				fz += (z - fz) / 8;

				ay = -(fz / 65536); 
				ax = (fy / 65536); 
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

//	static byte reg = 0;
	static u16 rbuf = 0;
	static byte buf[10];

	static TM32 tm;

	switch (i)
	{
		case 0:

			if (tm.Check(100))
			{
#ifndef WIN32
				if (!__debug) { HW::WDT->Update(); };
#endif

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
				};
				//else
				//{
				//	tempClock = -2730;
				//};

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
				};
				//else
				//{
				//	temp = -2730;
				//};

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

#elif defined(WIN32)

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
	static Ptr<REQ> rq;
	static TM32 tm;

	static byte i = 0;

	switch (i)
	{
		case 0:

			rq = CreateMotoReq();

			if (rq.Valid())
			{
				qmoto.Add(rq);

				i++;
			};

			break;

		case 1:

			if (rq->ready)
			{
				rq.Free();

				i++;
			};

			break;

		case 2:

			if (tm.Check(101)) i = 0;

			break;
	};

	qmoto.Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTestFlashWrite()
{
	static Ptr<UNIBUF> ptr;
	static u32 count = 0;

	static RTM rtm;

	if (rtm.Check(MS2RT(1)))
	{
		testDspReqCount++;

		count = 1000;
	};

//	if (count != 0)
	{
		ptr = CreateTestDspReq01();

		if (ptr.Valid())
		{
			count--;

			RspDsp01 *rsp = (RspDsp01*)(ptr->GetDataPtr());
			RequestFlashWrite(ptr, rsp->rw);

		};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateDSP()
{
	static Ptr<REQ> rq;

	static byte i = 0;
//	static TM32 tm;

	switch (i)
	{
		case 0:

			if ((mv.fireVoltage == 0 && motoTargetRPS == 1500) || __WIN32__)
			{
				if (FLASH_Status() != 0) UpdateTestFlashWrite();
			}
			else
			{
				rq = CreateDspReq01(1);

				if (rq.Valid())	qdsp.Add(rq), i++;
			};

			break;

		case 1:

			if (rq->ready)
			{
				if (rq->crcOK)
				{
					readyR01.Add(rq);
				};
				
				i = 0;

				rq.Free();
			};

			break;
	};

	qdsp.Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

static const u32 dspFlashPages[] = {
#include "G26K1BF592.LDR.H"
};

//u16 dspFlashLen = 0;
//u16 dspFlashCRC = 0;

static const u32 dspBootFlashPages[] = {
#include "G26K_2_BOOT_BF592.LDR.H"
};

//u16 dspBootFlashLen = 0;
//u16 dspBootFlashCRC = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashDSP()
{
	TM32 tm;

	Ptr<REQ> req;

	tm.Reset();

	while (!tm.Check(100)) HW::WDT->Update();

	while (!req.Valid()) req = CreateDspReq05(2);

	qdsp.Add(req); while(!req->ready) { qdsp.Update(); HW::WDT->Update(); };

	if (req->crcOK)
	{
		RspDsp05 *rsp = (RspDsp05*)req->rb.data;

		u16 flen;
		const u32 *fpages;
		u16 flashCRC;
		u16 flashLen;
		u32 startAdr;

		if (req->rb.len < sizeof(RspDsp05))
		{
			startAdr = 0; flashLen = rsp->v1.flashLen; flashCRC = rsp->v1.flashCRC;
		}
		else
		{
			startAdr = rsp->v2.startAdr; flashLen = rsp->v2.flashLen; flashCRC = rsp->v2.flashCRC;
		};

		if (startAdr > 0)
		{
			flen = sizeof(dspFlashPages);
			fpages = dspFlashPages;
		}
		else
		{
			flen = sizeof(dspBootFlashPages);
			fpages = dspBootFlashPages;
		};

		u16 fcrc = GetCRC16(fpages, flen);

		if (flashCRC != fcrc || flashLen != flen)
		{
			u16 count = flen+2;
			u16 adr = 0;
			byte *p = (byte*)fpages;

			while (count > 0)
			{
				u16 len;
				
				if (count > 256)
				{
					len = 256;

					req = CreateDspReq06(adr, len, p, 0, 0, 10);
				}
				else
				{
					len = count;

					if (len > 2)
					{
						req = CreateDspReq06(adr, len-2, p, sizeof(fcrc), &fcrc, 10);
					}
					else
					{
						req = CreateDspReq06(adr, sizeof(fcrc), &fcrc, 0, 0, 10);
					};
				};

				qdsp.Add(req); while(!req->ready) { qdsp.Update(); HW::WDT->Update(); };

				count -= len;
				p += len;
				adr += len;
			};

			req = CreateDspReq07();

			qdsp.Add(req); while(!req->ready) { qdsp.Update();	};

			//DisableDSP();
			//
			//tm.Reset();

			//while (!tm.Check(1)) HW::WDT->Update();

			//EnableDSP();
			
			//tm.Reset();

			//while (!tm.Check(100)) HW::WDT->Update();
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

	Ptr<REQ> req;

	motoFlashLen = sizeof(motoFlashPages);
	motoFlashCRC = GetCRC16(motoFlashPages, motoFlashLen);

	tm.Reset();

	bool hs = false;

	while (!tm.Check(200))
	{
		reqHS.guid = masterGUID;
		reqHS.crc = GetCRC16(&reqHS, sizeof(reqHS) - sizeof(reqHS.crc));
		wb.data = &reqHS;
		wb.len = sizeof(reqHS);

		commoto.Write(&wb);

		while (commoto.Update()) HW::WDT->Update(); 

		rb.data = &rspHS;
		rb.maxLen = sizeof(rspHS);
		commoto.Read(&rb, MS2RT(5), US2RT(100));

		while (commoto.Update()) HW::WDT->Update();;

		if (rb.recieved && rb.len == sizeof(rspHS) && GetCRC16(&rspHS, sizeof(rspHS)) == 0 && rspHS.guid == slaveGUID)
		{
			hs = true;
			break;
		};
	};

	if (hs)
	{
		req = CreateBootMotoReq01(motoFlashLen, 2);

		qmoto.Add(req); while(!req->ready) { qmoto.Update(); HW::WDT->Update(); };

		if (req->crcOK)
		{
			RspBootMoto *rsp = (RspBootMoto*)req->rb.data;

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

						qmoto.Add(req); while(!req->ready) { qmoto.Update(); HW::WDT->Update(); };

						RspBootMoto *rsp = (RspBootMoto*)req->rb.data;

						if (req->crcOK && rsp->F02.status) { break;	}
					};

					tm.Reset();

					while (!tm.Check(1)) HW::WDT->Update();

					count -= len;
					p += len;
					adr += len*4;
				};
			};
		};

		req = CreateBootMotoReq03();

		qmoto.Add(req); while(!req->ready) { qmoto.Update(); HW::WDT->Update();	};

		tm.Reset();

		while (!tm.Check(1)) HW::WDT->Update();
	};
}

#endif
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
	mv.cmSPR			= 36;
	mv.imSPR			= 180;
	mv.fireVoltage		= 500;
	mv.motoLimCur		= 2000;
	mv.motoMaxCur		= 3000;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "Init Main Vars Vars ... OK\n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadVars()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "Load Vars ... ");

	static DSCI2C dsc;
	static DSCSPI spi;
	static u16 romAdr = 0;
	
	byte buf[sizeof(mv)*2+4];

	MainVars mv1, mv2;

	bool c1 = false, c2 = false;

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

	spi.adr = (ReverseDword(FRAM_SPI_MAINVARS_ADR) & ~0xFF) | 3;
	spi.alen = 4;
	spi.csnum = 1;
	spi.wdata = 0;
	spi.wlen = 0;
	spi.rdata = buf;
	spi.rlen = sizeof(buf);

	if (SPI_AddRequest(&spi))
	{
		while (!spi.ready) { SPI_Update(); };
	};

	PointerCRC p(buf);

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv1, sizeof(mv1));
		p.ReadW();

		if (p.CRC.w == 0) { c1 = true; break; };
	};

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
		p.ReadArrayB(&mv2, sizeof(mv2));
		p.ReadW();

		if (p.CRC.w == 0) { c2 = true; break; };
	};

	SEGGER_RTT_WriteString(0, "FRAM SPI - "); SEGGER_RTT_WriteString(0, (c1) ? (RTT_CTRL_TEXT_BRIGHT_GREEN "OK") : (RTT_CTRL_TEXT_BRIGHT_RED "ERROR"));

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE " ... FRAM I2C - "); SEGGER_RTT_WriteString(0, (c2) ? (RTT_CTRL_TEXT_BRIGHT_GREEN "OK\n") : (RTT_CTRL_TEXT_BRIGHT_RED "ERROR\n"));

	if (c1 && c2)
	{
		if (mv1.timeStamp > mv2.timeStamp)
		{
			c2 = false;
		}
		else if (mv1.timeStamp < mv2.timeStamp)
		{
			c1 = false;
		};
	};

	if (c1)	{ mv = mv1; } else if (c2) { mv = mv2; };

	loadVarsOk = c1 || c2;

	if (!c1 || !c2)
	{
		if (!loadVarsOk) InitMainVars();

		svCount = 2;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SaveVars()
{
	static DSCI2C dsc;
	static DSCSPI spi,spi2;
	static u16 romAdr = 0;
	static byte buf[sizeof(mv) * 2 + 8];

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

			mv.timeStamp = GetMilliseconds();

			for (byte j = 0; j < 2; j++)
			{
				p.CRC.w = 0xFFFF;
				p.WriteArrayB(&mv, sizeof(mv));
				p.WriteW(p.CRC.w);
			};

			spi.adr = (ReverseDword(FRAM_SPI_MAINVARS_ADR) & ~0xFF) | 2;
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

			if (spi2.ready || tm.Check(200))
			{
				SPI_AddRequest(&spi);

				i++;
			};

			break;

		case 3:

			if (spi.ready || tm.Check(200))
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
		CALL( MainMode()				);
		CALL( UpdateMoto()				);
		CALL( UpdateTemp()				);
		CALL( UpdateMan(); 				);
		CALL( FLASH_Update();			);
		CALL( UpdateHardware();			);
		CALL( UpdateAccel();			);
		CALL( UpdateI2C();				);
		CALL( SaveVars();				);
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
	//UpdateEMAC();

	if (EmacIsConnected())
	{
		UpdateEMAC();
		UpdateTraps();

#ifndef WIN32
		if (!__debug) { HW::WDT->Update(); };
#endif
	};
	
	if (!(IsComputerFind() && EmacIsConnected()))
	{
		UpdateMisc();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53

	#define FPS_PIN_SET()	HW::PIOA->BSET(25)
	#define FPS_PIN_CLR()	HW::PIOA->BCLR(25)

#elif defined(CPU_XMC48)

	#define FPS_PIN_SET()	HW::P2->BSET(13)
	#define FPS_PIN_CLR()	HW::P2->BCLR(13)

#elif defined(WIN32)

	#define FPS_PIN_SET()	
	#define FPS_PIN_CLR()	

#endif

//static ComPort com1;

int main()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_WHITE "main() start ...\n");

//	static bool c = true;

	//static byte buf[8192];

	//volatile byte * const FLD = (byte*)0x60000000;	
	
	//static ComPort commem;

	DSCSPI dsc, dsc2;

	TM32 tm;

	//__breakpoint(0);

#ifndef WIN32

	DisableDSP();

#endif

	InitHardware();

	LoadVars();

	InitEMAC();

	InitTraps();

	FLASH_Init();

//	InitRmemList();

	Update_RPS_SPR();

#ifndef WIN32

	commoto.Connect(ComPort::ASYNC, 0, 1562500, 0, 1);
	comdsp.Connect(ComPort::ASYNC, 2, 6250000, 2, 1);
	//commem.Connect(ComPort::ASYNC, 1, 6250000, 0, 1);

	EnableDSP();

	//__breakpoint(0);

	FlashMoto();

	FlashDSP();

#endif

	u32 fc = 0;

	//ComPort::WriteBuffer wb;

	//for (u32 i = 0; i < ArraySize(buf); i++) buf[i] = i;

	//fps = CRC_CCITT_DMA(buf, 8000, 0xFFFF);
	//fc = CRC_CCITT_PIO(buf, 8000, 0xFFFF);

	//fps = CRC_CCITT_DMA(buf, 6000, 0xFFFF);
	//fps = CRC_CCITT_DMA(buf+6000, 2000, fps);

	//fc = CRC_CCITT_PIO(buf, 6000, 0xFFFF);
	//fc = CRC_CCITT_PIO(buf+6000, 2000, fc);

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

#ifdef WIN32

			extern u32 txThreadCount;

			Printf(0, 0, 0xFC, "FPS=%9i", fps);
			Printf(0, 1, 0xF0, "%lu", testDspReqCount);
			Printf(0, 2, 0xF0, "%lu", txThreadCount);
#endif
		};

#ifdef WIN32

		UpdateDisplay();

		static TM32 tm2;

		byte key = 0;

		if (tm2.Check(50))
		{
			if (_kbhit())
			{
				key = _getch();

				if (key == 27) break;
			};

			if (key == 'w')
			{
				FLASH_WriteEnable();
			}
			else if (key == 'e')
			{
				FLASH_WriteDisable();
			}
			else if (key == 'p')
			{
				NAND_FullErase();
			};
		};

#endif

	}; // while (1)

#ifdef WIN32

	NAND_FlushBlockBuffers();

	I2C_Destroy();
	SPI_Destroy();

	//_fcloseall();

#endif

}
