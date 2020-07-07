#include "hardware.h"
#include "ComPort.h"
#include "CRC16.h"
#include "at25df021.h"


static ComPort com;
//static ComPort::WriteBuffer wb;
//

static u16 data[1024];

//static u16 spd[1024*13];

//static byte twiBuffer[14] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE};

//struct Req30 { u16 rw; u16 gain; u16 st; u16 sl; u16 sd; u16 flt; u16 data[1024]; };

//static Req30 req30[13];

struct Cmd
{
	byte cmd; 
	byte chnl; 
	byte clk; 
	byte disTime; 
	u16 enTime; 
	byte chkSum; 
	byte busy; 
	byte ready; 
};

static Cmd cmdreq = {0, 0, 52, 255, 25*2000, 0, 0, 0};
static Cmd cmdrsp;

//static u16 spd2[512*2];
//
//static i16 ch1[512];
//static i16 ch2[512];
//static i16 ch3[512];
//static i16 ch4[512];

//static bool ready1 = false, ready2 = false;

//static u32 CRCOK = 0;
//static u32 CRCER = 0;

//static byte sampleTime[3] = { 19, 19, 9};
//static byte gain[3] = { 7, 7, 7 };
//static byte netAdr = 1;

static u16 manReqWord = 0xA900;
static u16 manReqMask = 0xFF00;

static u16 numDevice = 1;
static u16 verDevice = 0x101;

static u32 manCounter = 0;

static bool startFire = false;

static u16 sampleDelay = 600;//800;
static u16 sampleTime = 8;
static u16 sampleLen = 512;
static u16 gain = 0;


static void SaveParams();


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateFire()
{
	static byte i = 0;
	static bool ready = false;
	static RTM32 tm;

	static byte count = 0;
	static byte fnum = 0;


	//Req30 &req = req30[fnum];


	//switch (i)
	//{
	//	case 0:

	//		if (startFire)
	//		{
	//			startFire = false;

	//			fnum = 0;

	//			SetGain(gain);

	//			tm.Reset();

	//			i++;
	//		};

	//		break;

	//	case 1:

	//		cmdreq.chnl = fnum;

	//		WriteTWI(&cmdreq, sizeof(cmdreq));

	//		count = 10;

	//		i++;

	//		break;

	//	case 2:

	//		if ((*pTWI_MASTER_CTL & MEN) == 0)
	//		{
	//			ReadTWI(&cmdrsp, sizeof(cmdrsp));

	//			i++;
	//		};

	//		break;

	//	case 3:

	//		if ((*pTWI_MASTER_CTL & MEN) == 0)
	//		{
	//			if ((count--) == 0)
	//			{
	//				i = 1;
	//			}
	//			else
	//			{
	//				i = (cmdrsp.busy || !cmdrsp.ready) ? (i-1) : (i+1);
	//			};
	//		};

	//		break;

	//	case 4:

	//		{
	//			i32 d = (i32)sampleDelay - (i32)sampleTime*2;

	//			if (d < 0) d = 0;

	//			ReadPPI(req.data, ArraySize(req.data), sampleTime, d, &ready);
	//		};

	//		i++;

	//		break;

	//	case 5:

	//		if (ready)
	//		{
	//			u16 *p = req.data;

	//			for (u16 j = ArraySize(req.data)-8; j > 0; j--)
	//			{
	//				*p = p[8] - 2048;
	//				p++;
	//			};

	//			for (u16 j = 8; j > 0; j--)
	//			{
	//				*p++ = 0;
	//			};

	//			req.rw = manReqWord + 0x30 + fnum;
	//			req.gain = gain;
	//			req.st = sampleTime;
	//			req.sl = sampleLen;
	//			req.sd = sampleDelay;
	//			req.flt = 0;

	//			i++;
	//		};

	//		break;

	//	case 6:

	//		if (tm.Check(US2CLK(2000)))
	//		{
	//			fnum++;
	//			
	//			i = (fnum < 13) ? 1 : 0;
	//		};

	//		break;
	//};

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_00(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 rsp[3];

	if (wb == 0) return false;

	rsp[0] = manReqWord;
	rsp[1] = numDevice;
	rsp[2] = verDevice;

	wb->data = rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_10(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	//1.1 Запрос параметров
	//	0xA910
	//	------

	static u16 rsp[6];

	if (wb == 0) return false;

	rsp[0] = manReqWord|0x10;	// 	1. ответное слово
	rsp[1] = gain;				//	2. КУ
	rsp[2] = sampleTime;		//	3. Шаг оцифровки
	rsp[3] = sampleLen;			//	4. Длина оцифровки
	rsp[4] = sampleDelay;		//	5. Задержка оцифровки
	rsp[5] = 0;					//	6. Фильтр

	wb->data = rsp;			 
	wb->len = sizeof(rsp);	 

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 rsp[8];

	if (wb == 0) return false;

	startFire = true;

	rsp[0] = manReqWord|0x20;			//	1. ответное слово	
	rsp[1] = GD(&manCounter, u16, 0);	//	2. счётчик. младшие 2 байта
	rsp[2] = GD(&manCounter, u16, 1);	//	3. счётчик. старшие 2 байта
	rsp[3] = 4;							//	4. статус
	rsp[4] = 5;							//	5. AX
	rsp[5] = 6;							//	6. AY
	rsp[6] = 7;							//	7. AZ
	rsp[7] = 8;							//	8. AT
 
	wb->data = rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_30(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
//	static u16 rsp[6+512];

	if (wb == 0) return false;

	byte num = data[0] & 15;

	if (num > 12) { num = 12; };

	//Req30 &req = req30[num];

	//rsp[0] = data[0];			// 	1. ответное слово
	//rsp[1] = 1;				 	//	2. КУ
	//rsp[2] = 2;		 			//	3. Шаг оцифровки
	//rsp[3] = 512;			 	//	4. Длина оцифровки
	//rsp[4] = 0;					//	5. Задержка оцифровки
	//rsp[5] = 0;					//	6. Фильтр
	//rsp[6] = 0;					//	7-?. данные (до 2048шт)

	//req.rw = data[0];			// 	1. ответное слово
	//req.gain = 1;				 	//	2. КУ
	//req.st = 8;		 			//	3. Шаг оцифровки
	//req.sl = 512;			 	//	4. Длина оцифровки
	//req.sd = 0;					//	5. Задержка оцифровки
	//req.flt = 0;					//	6. Фильтр
//	rsp[6] = 0;					//	7-?. данные (до 2048шт)

	//wb->data = &req;
	//wb->len = req.sl*2 + sizeof(req) - sizeof(req.data);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_80(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 rsp[1];

	if (wb == 0 || len < 3) return false;

	switch (data[1])
	{
		case 1:

			numDevice = data[2];

			break;
	};

	rsp[0] = manReqWord|0x80;
 
	wb->data = rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_90(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 rsp[1];

	if (wb == 0 || len < 3) return false;

	switch (data[1])
	{
		case 1:

			gain = data[2] & 0xF;

			break;

		case 2:

			sampleTime = data[2];

			break;

		case 3:

			sampleLen = data[2];

			if (sampleLen < 64)
			{
				sampleLen = 64;
			}
			else if (sampleLen > 1024)
			{
				sampleLen = 1024;
			};

			break;

		case 4:

			sampleDelay = data[2];

			break;

		case 5:

			break;

	};

	rsp[0] = manReqWord|0x90;
 
	wb->data = rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_F0(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 rsp[1];

	if (wb == 0) return false;

	SaveParams();

	rsp[0] = manReqWord|0xF0;
 
	wb->data = rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan(ComPort::WriteBuffer *wb, ComPort::ReadBuffer *rb)
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

	t = (t>>4) & 0xF;

	switch (t)
	{
		case 0: 	r = RequestMan_00(p, len, wb); break;
		case 1: 	r = RequestMan_10(p, len, wb); break;
		case 2: 	r = RequestMan_20(p, len, wb); break;
		case 3: 	r = RequestMan_30(p, len, wb); break;
		case 8: 	r = RequestMan_80(p, len, wb); break;
		case 9:		r = RequestMan_90(p, len, wb); break;
		case 0xF:	r = RequestMan_F0(p, len, wb); break;
		
//		default:	bfURC++; 
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBlackFin()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static byte buf[1024];

	ResetWDT();

	switch(i)
	{
		case 0:

			rb.data = buf;
			rb.maxLen = sizeof(buf);
			com.Read(&rb, (u32)-1, 10000);
			i++;

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0)
				{
					if (RequestMan(&wb, &rb))
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
//		CALL( UpdateFire()		);
		CALL( UpdateBlackFin()	);
	};

	i &= 1; // i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SaveParams()
{
	u16 buf[12];

	ERROR_CODE result;

	buf[0] = buf[6] = sampleDelay;
	buf[1] = buf[7] = sampleTime;
	buf[2] = buf[8] = sampleLen;
	buf[3] = buf[9] = gain;
	buf[4] = buf[10] = numDevice;
	buf[5] = buf[11] = GetCRC16(buf, 10);

	result = EraseBlock(0x20000);
	result = at25df021_Write((byte*)buf, 0x20000, sizeof(buf), false);

	result = EraseBlock(0x21000);
	result = at25df021_Write((byte*)buf, 0x21000, sizeof(buf), false);

	result = EraseBlock(0x22000);
	result = at25df021_Write((byte*)buf, 0x22000, sizeof(buf), false);

	result = EraseBlock(0x23000);
	result = at25df021_Write((byte*)buf, 0x23000, sizeof(buf), false);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadParams()
{
	u16 buf[12];

	ERROR_CODE result;

	u32 stAdr = 0x20000;


	bool crc = false;

	for (byte n = 0; n < 4; n++, stAdr += 0x1000)
	{
		result = at25df021_Read((byte*)buf, stAdr, sizeof(buf));

		if (result == NO_ERR)
		{
			u16 *p = buf;

			for (byte i = 0; i < 2; i++)
			{
				if (GetCRC16(p, 12) == 0)
				{
					sampleDelay = p[0];
					sampleTime = p[1];
					sampleLen = p[2];
					gain = p[3];
					numDevice = p[4];

					crc = true;

					break;
				}
				else
				{
					p += 6;
				};
			};
		};

		if (crc) break;
	};

	if(!crc)
	{
		sampleDelay = 600;
		sampleTime = 8;
		sampleLen = 512;
		gain = 0;
		numDevice = 1;

		SaveParams();
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

i16 TestProcessData(u16 *data, u16 len)
{
	i32 max = -32768;
	i32 imax = -1;

	for (u32 i = len; i > 0; i--)
	{
		i32 t = *(data++) - 2048;

		if (t > max)
		{
			max = t;
			imax = i;
		};
	};

	return len - imax;
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateProcessData()
{
	DSCPPI *dsc = GetDscPPI();

	if (dsc != 0)
	{
		*pPORTGIO_SET = 1<<5;

		TestProcessData(dsc->dst, dsc->len);

		FreeDscPPI(dsc);

		*pPORTGIO_CLEAR = 1<<5;

		idle();
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

i16 index_max = 0;

int main( void )
{
	static byte s = 0;
	static byte i = 0;

	static u32 pt = 0;

	static RTM32 tm;

	InitHardware();

	LoadParams();

	com.Connect(1000000, 2);

//	InitNetAdress();

	while (1)
	{
		UpdateProcessData();

		u32 t = GetCycles32();

		*pPORTFIO_TOGGLE = 1<<8;

		if ((t - pt) >= US2CCLK(500))
		{
			pt += US2CCLK(500);

			*pPORTGIO_TOGGLE = 1<<4;

			SetGain(0);
		};

		//Update();

		*pPORTFIO_TOGGLE = 1<<8;

	};

//	return 0;
}
