#include "hardware.h"
#include "ComPort.h"
#include "CRC16.h"
//#include "at25df021.h"
#include "list.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte build_date[512] = "\n" "G26K2BF592" "\n" __DATE__ "\n" __TIME__ "\n";

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ComPort com;

//struct Cmd
//{
//	byte cmd; 
//	byte chnl; 
//	byte clk; 
//	byte disTime; 
//	u16 enTime; 
//	byte chkSum; 
//	byte busy; 
//	byte ready; 
//};


static u16 manReqWord = 0xAD00;
static u16 manReqMask = 0xFF00;

static u16 numDevice = 1;
static u16 verDevice = 0x101;

static u32 manCounter = 0;

static bool startFire = false;

static u16 sampleDelay = 600;//800;
static u16 sampleTime = 8;
static u16 sampleLen = 512;
static u16 gain = 0;

static u16 vavesPerRoundCM = 100;	
static u16 vavesPerRoundIM = 100;
static u16 filtrType = 0;
static u16 packType = 0;

static List<DSCPPI> processedPPI;

static DSCPPI *curDsc = 0;

//static void SaveParams();

static u16 mode = 0; // 0 - CM, 1 - IM
static u16 imThr = 0;
static u16 imDescr = 0;
static u16 imDelay = 0;
static u16 refThr = 0;
static u16 refDescr = 0;
static u16 refDelay = 0;
static u16 refAmp = 0;
static u16 refTime = 0;

static i32 avrBuf[PPI_BUF_LEN] = {0};

static u16 flashCRC = 0;
static u32 flashLen = 0;
static u16 lastErasedBlock = ~0;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc_01(const u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static RspDsp01 rsp;

	ReqDsp01 *req = (ReqDsp01*)data;

	if (req->vavesPerRoundCM > 72) { req->vavesPerRoundCM = 72; }
	if (req->vavesPerRoundIM > 500) { req->vavesPerRoundIM = 500; }

	SetDspVars(req);

	mode = req->mode;
	imThr = req->mainSens.thr;
	imDescr = req->mainSens.descr;
	imDelay = req->mainSens.sd;

	refThr = req->refSens.thr;
	refDescr = req->refSens.descr;
	refDelay = req->refSens.sd;

	vavesPerRoundCM = req->vavesPerRoundCM;	
	vavesPerRoundIM = req->vavesPerRoundIM;
	filtrType = req->filtrType;
	packType = req->packType;

	SetFireVoltage(req->fireVoltage);

	if (wb == 0) return false;

	if (curDsc != 0)
	{
		FreeDscPPI(curDsc);

		curDsc = 0;
	};

	curDsc = processedPPI.Get();

	if (curDsc == 0)
	{
		rsp.rw = data[0];
		rsp.len = sizeof(rsp);
		rsp.version = rsp.VERSION;
		rsp.fireVoltage = GetFireVoltage();
		rsp.motoVoltage = GetMotoVoltage();
		rsp.crc = GetCRC16(&rsp, sizeof(rsp)-2);

		wb->data = &rsp;			 
		wb->len = sizeof(rsp);	 
	}
	else
	{
		wb->data = curDsc->data;			 
		wb->len = (curDsc->offset + curDsc->len)*2;	 
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//static bool RequestFunc_05(const u16 *data, u16 len, ComPort::WriteBuffer *wb)
//{
//	const ReqDsp05 *req = (ReqDsp05*)data;
//	static RspDsp05 rsp;
//
//	if (len < sizeof(ReqDsp05)/2) return  false;
//
//	rsp.rw = req->rw;
//	rsp.flashLen = flashLen;
//	rsp.flashCRC = flashCRC;
//
//	rsp.crc = GetCRC16(&rsp, sizeof(rsp)-2);
//
//	wb->data = &rsp;
//	wb->len = sizeof(rsp);
//
//	return true;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static bool RequestFunc_06(const u16 *data, u16 len, ComPort::WriteBuffer *wb)
//{
//	const ReqDsp06 *req = (ReqDsp06*)data;
//	static RspDsp06 rsp;
//
//	ERROR_CODE Result = NO_ERR;
//
//	u16 xl = req->len + sizeof(ReqDsp06) - sizeof(req->data);
//
//	if (len < xl/2) return  false;
//
//	u32 stAdr = FLASH_START_ADR + req->stAdr;
//
//	u16 block = stAdr/4096;
//
//	if (lastErasedBlock != block)
//	{
//		Result = EraseBlock(block);
//		lastErasedBlock = block;
//	};
//
//	if (Result == NO_ERR)
//	{
//		Result = at25df021_Write(req->data, stAdr, req->len, true);
//	};
//
//	rsp.res = Result;
//
//	rsp.rw = req->rw;
//
//	rsp.crc = GetCRC16(&rsp, sizeof(rsp)-2);
//
//	wb->data = &rsp;
//	wb->len = sizeof(rsp);
//
//	return true;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFunc_07(const u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	while(1) { };
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc(ComPort::WriteBuffer *wb, ComPort::ReadBuffer *rb)
{
	u16 *p = (u16*)rb->data;
	bool r = false;

	u16 t = p[0];

	if ((t & manReqMask) != manReqWord || rb->len < 2)
	{
//		bfERC++; 
		return false;
	};

	manCounter += 1;

	u16 len = (rb->len)>>1;

	t &= 0xFF;

	switch (t)
	{
		case 1: 	r = RequestFunc_01(p, len, wb); break;
//		case 5: 	r = RequestFunc_05(p, len, wb); break;
//		case 6: 	r = RequestFunc_06(p, len, wb); break;
		case 7: 		RequestFunc_07(p, len, wb); break;
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBlackFin()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	//static u16 buf[256];

	ResetWDT();

	switch(i)
	{
		case 0:

			rb.data = build_date;
			rb.maxLen = sizeof(build_date);
			com.Read(&rb, ~0, US2CCLK(25));
			i++;

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0 && GetCRC16(rb.data, rb.len) == 0)
				{
					if (RequestFunc(&wb, &rb))
					{
						com.Write(&wb);
						i++;
					}
					else
					{
						i = 0;
					};
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
				if (curDsc != 0)
				{
					FreeDscPPI(curDsc);
					
					curDsc = 0;
				};

				i = 0;
			};

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateBlackFin()	);
		CALL( UpdateHardware()	);
	};

	i &= 1; // i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void Filtr_Data(DSCPPI &dsc, u32 filtrType)
{
	//u16 amp, time;

	//GetAmpTimeIM(dsc, amp, time);

	u16 *d = dsc.data + dsc.offset;

	if (filtrType == 1)
	{
		i32 *ab = avrBuf;

		for (u32 i = dsc.len; i > 0; i--)
		{
			i16 v = d[4] - 2048;

			*(d++) = v -= *ab/32;

			*(ab++) += v;
		};
	}
	else if (filtrType == 2)
	{
		i32 av = 0;

		for (u32 i = dsc.len; i > 0; i--)
		{
			i16 v = d[4] - 2048;

			av += v - av/2;

			*(d++) = av / 2;
		};
	}
	else if (filtrType == 3)
	{
		i32 av = 0;
		i32 *ab = avrBuf;

		for (u32 i = dsc.len; i > 0; i--)
		{
			i16 v = d[4] - 2048;

			av += v - av/2;

			v = av / 2;

			*(d++) = v -= *ab/32;

			*(ab++) += v;
		};
	}
	else
	{
		for (u32 i = dsc.len; i > 0; i--)
		{
			*d = d[4] - 2048; d++;
		};
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void GetAmpTimeIM_old(DSCPPI &dsc, u16 &amp, u16 &time)
{
	amp = 0;
	time = 0;

	u16 *data = dsc.data + dsc.offset;
	
	u16 len1 = imDescr * NS2CLK(50) / dsc.ppiclkdiv;

	if (len1 > dsc.len) len1 = dsc.len;

	u16 len2 = dsc.len - len1;

	i32 max = -32768;
	i32 imax = -1;
	i32 ind = 0;

	for (u32 i = len1; i > 0; i--)
	{
		i32 t = (i16)(*(data++));

		if (t < 0) t = -t;

		if (t > max) { max = t; imax = ind; };
		
		ind++;
	};

	if ((imax < 0 || max < imThr) && len2 > 0)
	{
		max = -32768;
		imax = -1;

		for (u32 i = len2; i > 0; i--)
		{
			i32 t = (i16)(*(data++));

			if (t < 0) t = -t;

			if (t > max) { max = t; imax = ind; };

			ind++;
		};
	};

	if (imax >= 0)
	{
		amp = max;
		time = dsc.sampleDelay + imax * dsc.sampleTime;
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void GetAmpTimeIM_2_old(DSCPPI &dsc, u16 imDescr, u16 imDelay,  u16 imThr, u16 &amp, u16 &time)
{
	amp = 0;
	time = 0;

	u16 *data = dsc.data + dsc.offset;
	//i16 *ab = avrBuf;
	
	u16 descr = (imDescr > imDelay) ? (imDescr - imDelay) : 0;

	u16 ind = descr * NS2CLK(50) / dsc.ppiclkdiv;

	if (ind >= dsc.len) return;

	data += ind;
	//ab += ind;

	u16 len = dsc.len - ind;

	i32 max = -32768;
	i32 imax = -1;

	for (u32 i = len; i > 0; i--)
	{
		i32 v = (i16)(*(data++));

		if (v > imThr)
		{ 
			if (v > max) { max = v; imax = ind; };
		}
		else if (imax >= 0 && v < 0)
		{ 
			break;
		};
		
		ind++;
	};

	if (imax >= 0)
	{
		amp = max;
		time = dsc.sampleDelay + imax * dsc.sampleTime;
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void GetAmpTimeIM_3(DSCPPI &dsc, u16 imDescr, u16 imDelay,  u16 imThr, u16 &amp, u16 &time, u16 &max_amp)
{
	amp = 0;
	time = ~0;

	u16 *data = dsc.data + dsc.offset;
	
	u16 descr = (imDescr > imDelay) ? (imDescr - imDelay) : 0;

	u16 ind = descr * NS2CLK(50) / dsc.ppiclkdiv;

	if (ind >= dsc.len) return;

	data += ind;

	u16 len = dsc.len - ind;

	i32 max = -32768;
	i32 imax = -1;

	i32 ampmax = 0;

	for (u32 i = len; i > 0; i--)
	{
		i32 v = (i16)(*(data++));

		if (v > imThr)
		{ 
			if (v > max) { max = v; imax = ind; };
		}
		else if (imax >= 0 && v < 0)
		{ 
			ind++;
			break;
		};

		if (v < 0) v = -v;
		if (v > ampmax) ampmax = v;
		
		ind++;
	};

	if (imax >= 0)
	{
		amp = max;
		u32 t = dsc.sampleDelay + imax * dsc.sampleTime;
		time = (t < 0xFFFF) ? t : 0xFFFF;
	};

	if (dsc.len > ind)
	{
		for (u32 i = dsc.len - ind; i > 0; i--)
		{
			i32 v = (i16)(*(data++));

			if (v < 0) v = -v;
			if (v > ampmax) ampmax = v;
		};
	};

	max_amp = (ampmax < 0xFFFF) ? ampmax : 0xFFFF;
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void ProcessDataCM(DSCPPI &dsc)
{
	//u16 amp, time;

	//GetAmpTimeIM(dsc, amp, time);

	//u16 *d = dsc.data + dsc.offset;

	RspCM *rsp = (RspCM*)dsc.data;

	rsp->rw			= manReqWord|0x40;			//1. �������� �����
	rsp->mmsecTime	= dsc.mmsec;
	rsp->shaftTime	= dsc.shaftTime;
	rsp->motoCount	= dsc.motoCount;
	rsp->headCount	= dsc.shaftCount;
	rsp->ax			= dsc.ax;
	rsp->ay			= dsc.ay;
	rsp->az			= dsc.az;
	rsp->at			= dsc.at;
	rsp->sensType	= dsc.sensType;
	rsp->maxAmp		= dsc.maxAmp;
	rsp->fi_amp		= dsc.fi_amp;
	rsp->fi_time	= dsc.fi_time;
	rsp->gain		= dsc.gain;
	rsp->st 		= dsc.sampleTime;			//15. ��� ���������
	rsp->sl 		= dsc.len;					//16. ����� ��������� (���� 2028)
	rsp->sd 		= dsc.sampleDelay;			//17. �������� ���������  
	rsp->packType	= 0;						//18. ��������
	rsp->packLen	= 0;						//19. ������ ����������� ������
	
	//u32 t = dsc.shaftTime - dsc.shaftPrev;

	//if (t != 0) { t = (36000 * (dsc.mmsec - dsc.shaftTime) + t/2) / t; };

	u32 t = (72000 * dsc.rotCount + 74) / 147;

	if (t >= 36000) t -= 36000;

	rsp->angle = t;

	processedPPI.Add(&dsc);
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//#pragma optimize_for_speed
//
//static void ProcessDataIM_old(DSCPPI &dsc)
//{
//	static DSCPPI *imdsc = 0;
//	static u32 count = 0;
//	static u32 cmCount = 0;
//	static u32 i = 0;
//
//	if (imdsc == 0)
//	{
//		count = vavesPerRoundIM;
//		cmCount = (count + 4) / 8;
//		i = 0;
//
//		imdsc = AllocDscPPI();
//	};
//
//	if (imdsc != 0)
//	{
//		RspIM *rsp = (RspIM*)imdsc->data;
//
//		while (i < dsc.fireIndex)
//		{
//			*pPORTGIO_TOGGLE = 1<<6;
//			rsp->data[i+1] = rsp->data[i];
//			rsp->data[i+count+1] = rsp->data[i+count];
//			i++;
//		};
//
//		if (i < count)
//		{
//			u16 amp, time;
//
//			GetAmpTimeIM_2(dsc, amp, time);
//
//			rsp->data[i] = amp;
//			rsp->data[i+count] = time;
//			i++;
//		};
//
//		if (i >= count)
//		{
//			*pPORTGIO_SET = 1<<7;
//
//			RspIM *rsp = (RspIM*)imdsc->data;
//
//			rsp->rw = manReqWord|0x50;				//1. �������� �����
//			rsp->mmsecTime = 0;						//2. ����� (0.1��). ������� 2 �����
//			rsp->shaftTime = 0;						//4. ����� ������� ����� (0.1��). ������� 2 �����
//			rsp->gain = dsc.gain;					//10. ��
//			rsp->len = count;						//11. ����� (���� 1024)
//
//			imdsc->offset = (sizeof(*rsp) - sizeof(rsp->data)) / 2;
//			imdsc->len = count*2;
//
//			processedPPI.Add(imdsc);
//
//			imdsc = 0;
//
//			*pPORTGIO_CLEAR = 1<<7;
//		}
//		else if (i > (dsc.fireIndex+1))
//		{
//			FreeDscPPI(imdsc);
//
//			imdsc = 0;
//		};
//	};
//
//	if (cmCount == 0)
//	{
//		cmCount = (count + 4) / 8;
//
//		ProcessDataCM(dsc);
//	}
//	else
//	{
//		FreeDscPPI(&dsc);
//	};
//
//	cmCount -= 1;
//}
//
//#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void SendReadyDataIM(DSCPPI *dsc, u16 len)
{
	*pPORTGIO_SET = 1<<7;

	RspIM *rsp = (RspIM*)dsc->data;

	rsp->rw			= manReqWord|0x50;	//1. �������� �����
	rsp->mmsecTime	= dsc->mmsec;		//2. ����� (0.1��). ������� 2 �����
	rsp->shaftTime	= dsc->shaftTime;	//4. ����� ������� ����� (0.1��). ������� 2 �����
	rsp->ax			= dsc->ax;
	rsp->ay			= dsc->ay;
	rsp->az			= dsc->az;
	rsp->at			= dsc->at;
	rsp->gain		= dsc->gain;		//10. ��
	rsp->refAmp		= refAmp;
	rsp->refTime	= refTime;
	rsp->len		= len;				//11. ����� (���� 1024)

	dsc->offset = (sizeof(*rsp) - sizeof(rsp->data)) / 2;
	dsc->len = len*2;

	processedPPI.Add(dsc);

	*pPORTGIO_CLEAR = 1<<7;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void ProcessDataIM(DSCPPI &dsc)
{
	static DSCPPI *imdsc = 0;
	static u32 count = 0;
	static u32 cmCount = 0;
	static u32 i = 0;
	static u16 prevShaftCount = 0;

	if (dsc.shaftCount != prevShaftCount)
	{
		prevShaftCount = dsc.shaftCount;

		if (imdsc != 0)
		{
			SendReadyDataIM(imdsc, i);

			imdsc = 0;
		};
	};

	if (imdsc == 0)
	{
		count = 500; //vavesPerRoundIM;
		cmCount = vavesPerRoundIM / 8;
		i = 0;

		imdsc = AllocDscPPI();

		if (imdsc != 0)
		{
			imdsc->mmsec		= dsc.mmsec;
			imdsc->shaftTime	= dsc.shaftTime;
			imdsc->gain			= dsc.gain;
			imdsc->ax			= dsc.ax;
			imdsc->ay			= dsc.ay;
			imdsc->az			= dsc.az;
			imdsc->at			= dsc.at;

			RspIM *rsp = (RspIM*)imdsc->data;

			u16 *d = rsp->data;

			for (u32 i = 10; i > 0; i--) { *(d++) = 0; };
		};
	};

	if (dsc.sensType == 1)
	{
		ProcessDataCM(dsc);
	}
	else 
	{
		if (imdsc != 0)
		{
			RspIM *rsp = (RspIM*)imdsc->data;

			u16 *data = rsp->data + i*2;

			if (dsc.fireIndex < count)
			{
				while (i < dsc.fireIndex)
				{
					*(data++) = 0;
					*(data++) = 0;
					i++;
				};
			};

			if (i < count)
			{
				*(data++) = dsc.fi_amp;
				*(data++) = dsc.fi_time;
				i++;
			};

			if (i >= count)
			{
				SendReadyDataIM(imdsc, count);

				imdsc = 0;
			};
		};

		if (cmCount == 0)
		{
			cmCount = vavesPerRoundIM / 8;

			ProcessDataCM(dsc);
		}
		else
		{
			FreeDscPPI(&dsc);
		};

		cmCount -= 1;
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMode()
{
	DSCPPI *dsc = GetDscPPI();

	if (dsc != 0)
	{
		*pPORTFIO_SET = 1<<7;

		if (dsc->sensType == 0)
		{
			Filtr_Data(*dsc, filtrType);
			
			GetAmpTimeIM_3(*dsc, imDescr, imDelay, imThr, dsc->fi_amp, dsc->fi_time, dsc->maxAmp);

			switch (mode)
			{
				case 0:  ProcessDataCM(*dsc); break;
				case 1:  ProcessDataIM(*dsc); break;
			};
		}
		else
		{
			Filtr_Data(*dsc, (filtrType == 2 || filtrType == 3) ? 2 : 0);
			
			GetAmpTimeIM_3(*dsc, refDescr, refDelay, refThr, dsc->fi_amp, dsc->fi_time, dsc->maxAmp);

			refAmp	= dsc->fi_amp;
			refTime = dsc->fi_time;
			
			ProcessDataCM(*dsc);
		};

		//idle();
		*pPORTFIO_CLEAR = 1<<7;
	}
	else
	{
		Update();
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void CheckFlash()
//{
//	static BOOT_HEADER bh;
//
//	byte *p = (byte*)&bh;
//
////	u32 stAdr = 0x8000;
//	u32 adr = 0;
//
////	bool ready = false;
//
//	while (1)
//	{
//		at25df021_Read(p, FLASH_START_ADR + adr, sizeof(bh));
//
////		while(!ready) {};
//
//		adr += sizeof(bh);
//
//		if ((bh.blockCode & BFLAG_FILL) == 0)
//		{
//			adr += bh.byteCount;	
//		};
//
//		if (bh.blockCode & BFLAG_FINAL)
//		{
//			break;
//		};
//	};
//
//	flashLen = adr;
//
//	flashCRC = at25df021_GetCRC16(FLASH_START_ADR, flashLen);
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

i16 index_max = 0;

int main( void )
{
	static byte s = 0;
	static byte i = 0;

	static u32 pt = 0;

	//static RTM32 tm;

	InitHardware();

	com.Connect(6250000, 2);

	//CheckFlash();

	while (1)
	{
		*pPORTFIO_SET = 1<<8;
		
		UpdateMode();

		*pPORTFIO_CLEAR = 1<<8;

	};

//	return 0;
}
