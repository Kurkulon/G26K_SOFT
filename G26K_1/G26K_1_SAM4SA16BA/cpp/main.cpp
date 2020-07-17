#include "ComPort.h"
#include "time.h"
#include "CRC16.h"
#include "req.h"
#include "hardware.h"

#include "list.h"

#include "twi.h"

#include "PointerCRC.h"

#include "SPI.h"

//ComPort commem;
ComPort comdsp;
//ComPort combf;
ComPort commoto;

//ComPort::WriteBuffer wb;
//ComPort::ReadBuffer rb;

static const u32 flashPages[] = {
//#include "G26P.LDR.H"
};

//static bool runMainMode = false;

u16 flashLen = 0;
u16 flashCRC = 0;

u16 dspFlashLen;
u16 dspFlashCRC;

u32 fc = 0;

//static u32 bfCRCOK = 0;
//static u32 bfCRCER = 0;
static u32 bfURC = 0;
//static u32 bfERC = 0;

//static bool waitSync = false;
//static bool startFire = false;

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

static byte gainRef = 0;
static u16 sampleTimeRef = 5;
static u16 sampleLenRef = 1024;
static u16 sampleDelayRef = 200;
static u16 deadTimeRef = 400;
static u16 descriminantRef = 400;

static u16 motoEnable = 0; // двигатель включить или выключить
static u16 motoTargetRPS = 0; // заданные обороты двигателя
static u16 motoRPS = 0; // обороты двигателя, об/сек
static u16 motoCur = 0; // ток двигателя, мА
static u16 motoStat = 0; // статус двигателя: 0 - выкл, 1 - вкл
static u16 motoCounter = 0; // счётчик оборотов двигателя 1/6 оборота
static u16 cmSPR = 32;		// Количество волновых картин на оборот головки в режиме цементомера
static u16 imSPR = 32;		// Количество точек на оборот головки в режиме имиджера
static u16 curSPR = 32;		// Количество импульсов излучателя на оборот в текущем режиме

u16 manRcvData[10];
u16 manTrmData[50];

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

static u16 reqVoltage = 800;
static byte reqFireCountM = 3;
static byte reqFireCountXY = 2;

static u16 adcValue = 0;
static U32u filtrValue;
static u16 resistValue = 0;
static byte numStations = 0;
static u16 voltage = 0;
i16 temperature = 0;
i16 cpuTemp = 0;

static byte mainModeState = 0;

static byte dspStatus = 0;

//static u32 rcvCRCOK = 0;
//static u32 rcvCRCER = 0;

//static u32 chnlCount[4] = {0};

static u32 crcErr02 = 0;
//static u32 crcErr03 = 0;
static u32 crcErr04 = 0;

static u32 notRcv02 = 0;
static u32 lenErr02 = 0;

static byte savesCount = 0;

static TWI	twi;

//static DSCTWI dsc;
static byte framBuf[100];

//static u16 maxOff = 0;

static void UpdateI2C();

inline void SaveParams() { savesCount = 1; }


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//Response rsp;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static SPI spi;
static SPI::Buffer	bufAccel; 
//static SPI::Buffer	bufGyro;

static i16 ax = 0, ay = 0, az = 0, at = 0;

//static i32 ang_x = 0, ang_y = 0, ang_z = 0;

static u8 txAccel[25] = { 0 };
static u8 rxAccel[25];

//static u8 txGyro[25] = { 0 };
//static u8 rxGyro[25];

//static u8 gyro_WHO_AM_I = 0;

//static i32 gx = 0, gy = 0, gz = 0;
//static i32 fgx = 6100000, fgy = -5000000, fgz = 4180000;
//static i32 gYaw = 0, gPitch = 0, gRoll = 0;
//static i16 gt = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackAccel(SPI::Buffer *b)
{
	//DataPointer p(b->rxp);

	//union { float f; u16 w[2]; } u;

	//p.b += 1;

	//for (byte i = 0; i < 8; i++)
	//{
	//	u.f = (u16)(__rev(*p.d) >> 15) * 0.0003814697265625*1.00112267 - 0.00319055 - valueADC1[i];

	//	valueADC1[i] += u.f * (((u.w[1] & 0x7f80) < 0x3c00) ? 0.01 : 0.1);
	//	p.b += 3;
	//};

	//UpdatePWM();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackGyro(SPI::Buffer *b)
{
	//DataPointer p(b->rxp);

	//union { float f; u16 w[2]; } u;

	//p.b += 1;

	//for (byte i = 0; i < 8; i++)
	//{
	//	u.f = (u16)(__rev(*p.d) >> 15) * 0.0003814697265625*1.00112267 - 0.00319055 - valueADC2[i];

	//	valueADC2[i] += u.f * (((u.w[1] & 0x7f80) < 0x3c00) ? 0.01 : 0.1);
	//	p.b += 3;
	//};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void InitADC16()
//{
//	using namespace HW;
//
//	//PMC->PCER0 = PID::SPI0_M;
//
//	//SPI0->CR = 1; 
//	//SPI0->MR = 0xFF010005;
//	//SPI0->CSR[0] = 0x00095482;
//
//	bufADC1.txp = &txADC1;
//	bufADC1.rxp = &rxADC1;
//	bufADC1.count = 25;
//	bufADC1.CSR = 0x00092302;
//	bufADC1.DLYBCS = 0x9;
//	bufADC1.PCS = 1;
//	bufADC1.pCallBack = CallBackADC1;
//
//	bufADC2.txp = &txADC2;
//	bufADC2.rxp = &rxADC2;
//	bufADC2.count = 25;
//	bufADC2.CSR = 0x00092302;
//	bufADC2.DLYBCS = 0x9;
//	bufADC2.PCS = 0;
//	bufADC2.pCallBack = CallBackADC2;
//
//	spi.AddRequest(&bufADC1);
//	spi.AddRequest(&bufADC2);
//
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SendAccelBuf(u16 count)
{
	bufAccel.txp = &txAccel;
	bufAccel.rxp = &rxAccel;
	bufAccel.count = count;
	bufAccel.CSR = 0x00091402;
	bufAccel.DLYBCS = 0x9;
	bufAccel.PCS = 0;
	bufAccel.pCallBack = 0;
	bufAccel.pio = HW::PIOA;
	bufAccel.mask = 1<<24;
	
	spi.AddRequest(&bufAccel);
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void AccelReadReg(byte reg)
{
	txAccel[0] = reg;

	SendAccelBuf(2);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void AccelWriteReg(byte reg, byte v)
{
	txAccel[0] = reg;
	txAccel[1] = v;

	SendAccelBuf(2);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateAccel()
{
	static byte i = 0; 
	static i16 x = 0, y = 0, z = 0, t = 0;
	static i32 fx = 0, fy = 0, fz = 0, ft = 0;

	static TM32 tm;

	switch (i)
	{
		case 0:

			tm.Reset();
			i++;

			break;

		case 1:

			if (tm.Check(35))
			{
				AccelReadReg(0x58); // INT_STATUS

				i++;
			};

			break;

		case 2:

			if (bufAccel.ready)
			{
				AccelWriteReg(7, 0); // CTRL Set PORST to zero

				i++;
			};

			break;

		case 3:

			if (bufAccel.ready)
			{
				tm.Reset();

				i++;
			};

			break;

		case 4:

			if (tm.Check(10))
			{
				AccelReadReg(0x25); // X_MSB 

				i++;
			};

			break;

		case 5:

			if (bufAccel.ready)
			{
				z = rxAccel[1] << 8;

				AccelReadReg(0x20); // X_MSB

				i++;
			};

			break;

		case 6:

			if (bufAccel.ready)
			{
				z |= rxAccel[1];

//				z /= 4;

				fz += (((i32)z * 65536) - fz) / 16;

				az = (i32)fz / 23617;

				AccelReadReg(0x15); // X_MSB

				i++;
			};

			break;

		case 7:

			if (bufAccel.ready)
			{
				x = rxAccel[1] << 8;

				AccelReadReg(0x10); // X_MSB

				i++;
			};

			break;

		case 8:

			if (bufAccel.ready)
			{
				x |= rxAccel[1];

				//x /= 4;

				fx += (((i32)x * 65536) - fx) / 16;

				ax = (i32)fx / 23617;

				AccelReadReg(0x1C); // X_MSB

				i++;
			};

			break;

		case 9:

			if (bufAccel.ready)
			{
				y = rxAccel[1] << 8;

				AccelReadReg(0x19); // X_MSB

				i++;
			};

			break;

		case 10:

			if (bufAccel.ready)
			{
				y |= rxAccel[1];

				//y /= 4;

				fy += (((i32)y * 65536) - fy) / 16;

				ay = (i32)fy / 23617;

//				ang_y = ArcTan(gx, gz);

				AccelReadReg(0x4C); // X_MSB

				i++;
			};

			break;

		case 11:

			if (bufAccel.ready)
			{
				t = (rxAccel[1] & 0x3F) << 8;

				AccelReadReg(0x49); // X_MSB

				i++;
			};

			break;

		case 12:

			if (bufAccel.ready)
			{
				t |= rxAccel[1];

				ft += (((i32)t * 65536) - ft) / 32;

//				t /= 16;

				at = (ft - 512 * 65536 * 16) / 33554 + 2300;

				i = 4;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CopyData(void *src, void *dst, u16 len)
{
	T_HW::S_USART &u = *HW::USART1;

	u.CR = 0x1A0;	// Disable transmit and receive, reset status

	u.MR = 0x89C0; // LOCAL_LOOPBACK, SYNC, No parity, 
	u.BRGR = 30;
	u.PDC.TPR = src;
	u.PDC.TCR = len;
	u.PDC.RPR = dst;
	u.PDC.RCR = len;

	u.PDC.PTCR = 0x101;

	u.CR = 0x150;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline bool CheckCopyDataComplete()
{
	T_HW::S_USART &u = *HW::USART1;

	return 	u.PDC.TCR == 0 && u.PDC.RCR == 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackRcvReqFire(REQ *q)
{
//	waitSync = true;;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void CallBackDspReq01(REQ *q)
{
	RspDsp01 &rsp = *((RspDsp01*)q->rb->data);
	 
	if (rsp.rw == (dspReqWord|0x40))
	{
		q->crcOK = (q->rb->len == (rsp.CM.sl*2 + 19*2));

		dspStatus |= 1;
	}
	else if (rsp.rw == (dspReqWord|0x50))
	{
		q->crcOK = (q->rb->len == (rsp.IM.dataLen*2 + 11*2));

		dspStatus |= 1;
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
	
	req.rw			= dspReqWord|1;
	req.mode 		= 0;
	req.gain 		= gain;
	req.st	 		= sampleTime;
	req.sl 			= sampleLen;
	req.sd 			= sampleDelay;
	req.thr			= deadTime;
	req.descr		= descriminant;
	req.refgain 	= gainRef;
	req.refst		= sampleTimeRef;
	req.refsl 		= sampleLenRef;
	req.refsd 		= sampleDelayRef;
	req.refthr		= deadTimeRef;
	req.refdescr	= descriminantRef;

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
	//__packed struct T { u16 g[8]; u16 st; u16 len; u16 delay; u16 voltage; };
	//__packed struct Rsp { u16 hdr; u16 rw; T t1, t2, t3; };
	
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	u16* p = manTrmData+1;

	*(p++) =  reqVoltage;
	*(p++) =  reqFireCountM;
	*(p++) =  reqFireCountXY;

	//for (byte i = 0; i < 3; i++)
	//{
	//	*(p++) =  gain[0][i];
	//	*(p++) =  gain[1][i];
	//	*(p++) =  gain[2][i];
	//	*(p++) =  gain[3][i];
	//	*(p++) =  gain[4][i];
	//	*(p++) =  gain[5][i];
	//	*(p++) =  gain[6][i];
	//	*(p++) =  gain[7][i];
	//	*(p++) =  sampleTime[i];
	//	*(p++) =  sampleLen[i];
	//	*(p++) =  sampleDelay[i];
	//};

	manTrmData[0] = (manReqWord & manReqMask) | 0x10;

	mtb->data1 = manTrmData;
	mtb->len1 = 17;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = manReqWord|0x20;	//	1. ответное слово
	manTrmData[1] = 0;					//	2. Время (0.1мс). младшие 2 байта
	manTrmData[2] = 0;					//	3. Время. старшие 2 байта
	manTrmData[3] = 0;					//	4. Время датчика Холла (0.1мс). младшие 2 байта
	manTrmData[4] = 0;					//	5. Время датчика Холла. старшие 2 байта
	manTrmData[5] = motoRPS;			//	6. Частота вращения двигателя (0.01 об/сек)
	manTrmData[6] = motoCur;			//	7. Ток двигателя (мА)
	manTrmData[7] = motoCounter;		//	8. Счётчик оборотов двигателя (1/6 об)
	manTrmData[8] = GetShaftRPS();		//	9. Частота вращения головки (0.01 об/сек)
	manTrmData[9] = GetShaftCount();	//	10. Счётчик оборотов головки (об)
	manTrmData[10] = -ax;				//	11. AX (уе)
	manTrmData[11] = az;				//	12. AY (уе)
	manTrmData[12] = -ay;				//	13. AZ (уе)
	manTrmData[13] = at;				//	14. AT (short 0.01 гр)
	manTrmData[14] = cpuTemp;			//	15. Температура в приборе (short)(0.1гр)
 
	mtb->data1 = manTrmData;
	mtb->len1 = 15;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_40(u16 *data, u16 len, MTB* mtb)
{
	__packed struct Req { u16 rw; u16 off; u16 len; };

	Req &req = *((Req*)data);

	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	//byte nf = ((req.rw>>4)-3)&3;
	//byte nr = req.rw & 7;

//	curRcv[nf] = nr;

	struct Rsp { u16 rw; };
	
	static Rsp rsp; 

	rsp.rw = req.rw;

	mtb->data1 = (u16*)&rsp;
	mtb->len1 = sizeof(rsp)/2;
	mtb->data2 = 0;
	mtb->len2 = 0;

	R01 *r01 = curManVec40;
	
	u16 sz = 18 + r01->rsp.CM.sl;

	if (len < 3 || data[1] == 0)
	{
		if (r01 != 0)
		{
			freeR01.Add(r01);

			curManVec40 = 0;
		};
		
		r01 = manVec40;

		if (r01 != 0 && r01->rsp.rw == req.rw)
		{
			curManVec40 = r01;

			manVec40 = 0;

			mtb->data2 = ((u16*)&r01->rsp)+1;

			if (len < 3)
			{
				mtb->len2 = sz;
			}
			else
			{
				u16 len = data[2];

				if (len > sz) len = sz;

				mtb->len2 = len;
			};
		};
	}
	else if (sz >= data[1] && r01 != 0)
	{
		u16 maxlen = sz - data[1];
		u16 len = data[2];

		if (len > maxlen) len = maxlen;

		mtb->data2 = (u16*)&r01->rsp + data[1]+1;
		mtb->len2 = len;
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_30(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 3 || mtb == 0) return false;

	manTrmData[0] = data[0];	
 
	motoTargetRPS = (data[0]&15) * 100;
		
	Set_Sync_Rot(motoTargetRPS, curSPR);

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_50(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 3 || mtb == 0) return false;

	manTrmData[0] = manReqWord|0x50;	
 
	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

		case 0x11:	gainRef			= data[2];	break;
		case 0x12:	sampleTimeRef	= data[2];	break;
		case 0x13:	sampleLenRef	= data[2];	break;
		case 0x14:	sampleDelayRef 	= data[2];	break;
		case 0x15:	deadTimeRef		= data[2];	break;
		case 0x16:	descriminantRef	= data[2];	break;

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
		
		default:	bfURC++; 
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

	manTrmData[0] = (memReqWord & memReqMask)|0x20;


	mtb->data1 = manTrmData;
	mtb->len1 = 20;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_30(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

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
	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

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
	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

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
		
		default:	bfURC++; 
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

						if (manVec40 != 0)
						{
							freeR01.Add(manVec40);
						};
						
						manVec40 = r01;
					}
					else if ((r01->rsp.rw & 0xFF) == 0x50)
					{
						r01->rsp.IM.ax = -ax;
						r01->rsp.IM.ay = az;
						r01->rsp.IM.az = -ay;
						r01->rsp.IM.at = at;

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

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMan()
{
	static MTB mtb;
	static MRB mrb;

	static byte i = 0;

	static RTM32 tm;


//	u16 c;

	switch (i)
	{
		case 0:

			mrb.data = manRcvData;
			mrb.maxLen = 3;
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

static void UpdateTempCPU()
{
	static TM32 tm;

	if (tm.Check(103))
	{
		cpuTemp = (((i32)HW::ADC->CDR[15] - 1787) * 5617*5) / 16384 + 270;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitTempCPU()
{
	using namespace HW;

	PMC->PCER0 = PID::ADC_M;

	ADC->MR = 0x0001FF80;
	ADC->ACR = 0x10;
	ADC->CHER = 1<<15;
	ADC->CR = 2;
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

static void UpdateSlow()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateI2C()			);
		CALL( UpdateAccel()			);
		CALL( spi.Update()			);
		CALL( UpdateMoto()			);
		CALL( UpdateTempCPU()		);
		CALL( UpdateHardware()		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMisc()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( MainMode()			);
		CALL( UpdateMan()			);
		CALL( UpdateSlow()			);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateParams()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( qdsp.Update()		);
		CALL( UpdateMisc()		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadVars()
{
	static DSCTWI dsc;

	twi.Init(1);

	PointerCRC p(framBuf);

	dsc.MMR = 0x500200;
	dsc.IADR = 0;
	dsc.CWGR = 0x7575;
	dsc.data = framBuf;
	dsc.len = sizeof(framBuf);

	if (twi.Read(&dsc))
	{
		while (twi.Update());
	};

	bool c = false;

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		numDevice = p.ReadW();
		gain = p.ReadB();
		sampleTime = p.ReadW();
		sampleLen = p.ReadW();
		sampleDelay = p.ReadW();
		deadTime = p.ReadW();
		descriminant =  p.ReadW();

		gainRef = p.ReadB();
		sampleTimeRef = p.ReadW();
		sampleLenRef = p.ReadW();
		sampleDelayRef = p.ReadW();
		deadTimeRef = p.ReadW();
		descriminantRef =  p.ReadW();

		p.ReadW();

		if (p.CRC.w == 0) { c = true; break; };
	};

	if (!c)
	{
		numDevice = 0;

		gain = 0;
		sampleTime = 5;
		sampleLen = 1024;
		sampleDelay = 200;
		deadTime = 300;
		descriminant = 100;

		gainRef = 0;
		sampleTimeRef = 5;
		sampleLenRef = 1024;
		sampleDelayRef = 200;
		deadTimeRef = 300;
		descriminantRef = 100;

		savesCount = 2;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateI2C()
{
	PointerCRC p(framBuf);

	static byte i = 0;
	static TM32 tm;
	static DSCTWI dsc;
	static u16 tempBuf;

	switch (i)
	{
		case 0:

			if (savesCount > 0)
			{
				i++;
			}
			else if (tm.Check(107))
			{
				i = 3;
			};

			break;

		case 1:

			dsc.MMR = 0x500200;
			dsc.IADR = 0;
			dsc.CWGR = 0x07575; 
			dsc.data = framBuf;
			dsc.len = sizeof(framBuf);

			for (byte j = 0; j < 2; j++)
			{
				p.CRC.w = 0xFFFF;

				p.WriteW(numDevice);

				p.WriteB(gain);
				p.WriteW(sampleTime);
				p.WriteW(sampleLen);
				p.WriteW(sampleDelay);
				p.WriteW(deadTime);
				p.WriteW(descriminant);

				p.WriteB(gainRef);
				p.WriteW(sampleTimeRef);
				p.WriteW(sampleLenRef);
				p.WriteW(sampleDelayRef);
				p.WriteW(deadTimeRef);
				p.WriteW(descriminantRef);

				p.WriteW(p.CRC.w);
			};

			i = (twi.Write(&dsc)) ? (i+1) : 0;

			break;

		case 2:

			if (!twi.Update())
			{
				savesCount--;
				i = 0;
			};

			break;

		case 3:

			dsc.MMR = 0x491100;
			dsc.IADR = 0;
			dsc.CWGR = 0x07575; 
			dsc.data = &tempBuf;
			dsc.len = sizeof(tempBuf);

			i = (twi.Read(&dsc)) ? (i+1) : 0;

			break;

		case 4:

			if (!twi.Update())
			{
				temperature = (dsc.rlen == 2) ? (((i16)ReverseWord(tempBuf) * 5 + 32) / 64) : cpuTemp;

				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashRcv()
{
	REQ *req = 0;

	flashLen = sizeof(flashPages)+2;
	flashCRC = GetCRC16(flashPages, flashLen-2);

	req = CreateDspReq05(2);

	qdsp.Add(req); while(!req->ready) { qdsp.Update(); };

	if (req->crcOK)
	{
		RspDsp05 *rsp = (RspDsp05*)req->rb->data;

		dspFlashLen = rsp->flashLen;
		dspFlashCRC = rsp->flashCRC;

		if (rsp->flashCRC != 0 || rsp->flashLen != flashLen)
		{
			u16 count = flashLen;
			u16 adr = 0;
			byte *p = (byte*)flashPages;

			while (count > 0)
			{
				u16 len;
				
				if (count > 256)
				{
					len = 256;

					req = CreateDspReq06(adr, len, p, 0, 0, 2);
				}
				else
				{
					len = count;

					if (len > 2)
					{
						req = CreateDspReq06(adr, len-2, p, sizeof(flashCRC), &flashCRC, 2);
					}
					else
					{
						req = CreateDspReq06(adr, sizeof(flashCRC), &flashCRC, 0, 0, 2);
					};
				};

				qdsp.Add(req); while(!req->ready) { qdsp.Update(); };

				count -= len;
				p += len;
				adr += len;
			};

			req = CreateDspReq07();

			qdsp.Add(req); while(!req->ready) { qdsp.Update();	};
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main()
{
	InitHardware();

	EnableDSP();
	
	Init_time();

	//LoadVars();

//	InitNumStations();

	InitRmemList();

	InitTempCPU();

//	commem.Connect(0, 6250000, 0);
	commoto.Connect(2, 1562500, 0);
	comdsp.Connect(3, 6250000, 0);

//	FlashRcv();

//	InitRcv();


//	com1.Write(&wb);

	u32 fps = 0;

	TM32 tm;

//	__breakpoint(0);


	while(1)
	{
		HW::PIOA->SODR = 1UL<<23;

		UpdateParams();

		HW::PIOA->CODR = 1UL<<23;

		fps++;


		if (tm.Check(1000))
		{ 
			UpdateTempCPU();
//			qtrm.Add(CreateTrmReq03());
			fc = fps; fps = 0; 
//			startFire = true;
//			com1.TransmitByte(0);
		};

	};

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
