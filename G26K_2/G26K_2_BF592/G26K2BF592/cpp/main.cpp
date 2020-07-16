#include "hardware.h"
#include "ComPort.h"
#include "CRC16.h"
#include "at25df021.h"
#include "list.h"

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

static List<DSCPPI> processedPPI;

static DSCPPI *curDscPPI = 0;

//static void SaveParams();


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

static bool RequestMan_10(u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 rsp[6];

	SetDspVars((ReqDsp01*)data);
	
	if (wb == 0) return false;

	if (curDscPPI != 0)
	{
		FreeDscPPI(curDscPPI);

		curDscPPI = 0;
	};

	curDscPPI = processedPPI.Get();

	if (curDscPPI == 0)
	{
		rsp[0] = data[0];

		wb->data = rsp;			 
		wb->len = 1*2;	 
	}
	else
	{
		u16 *rsp = curDscPPI->data;

		rsp[0] = manReqWord|0x40;					//1. ответное слово
		//rsp[1] = 0;								//2. Время (0.1мс). младшие 2 байта
		//rsp[2] = 0;								//3. Время. старшие 2 байта
		//rsp[3] = 0;								//4. Время датчика Холла (0.1мс). младшие 2 байта
		//rsp[4] = 0;								//5. Время датчика Холла. старшие 2 байта
		rsp[5] = 0;									//6. Счётчик оборотов двигателя (1/6 об)
		rsp[6] = 0;									//7. Счётчик оборотов головки (об)
		rsp[7] = 0;									//8. AX (уе)
		rsp[8] = 0;									//9. AY (уе)
		rsp[9] = 0;									//10. AZ (уе)
		rsp[10] = 0;								//11. AT (short 0.01 гр)
		rsp[11] = 0;								//12. Тип датчика (0 - измерительный датчик, 1 - опорный датчик)
		rsp[12] = 0;								//13. Угол поворота (0.01гр)(ushort)
		rsp[13] = 0;								//14. КУ
		rsp[14] = curDscPPI->clkdiv/NS2CLK(50);		//15. Шаг оцифровки
		rsp[15] = curDscPPI->len;					//16. Длина оцифровки (макс 2028)
		rsp[16] = 0;								//17. Задержка оцифровки  
		rsp[17] = 1;								//18. Упаковка
		rsp[18] = 0;								//19. Размер упакованных данных
	
		wb->data = rsp;			 
		wb->len = (19 + curDscPPI->len)*2;	 
	};

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

	t &= 0xFF;

	switch (t)
	{
		case 1: 	r = RequestMan_10(p, len, wb); break;
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBlackFin()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static u32 buf[128];

	ResetWDT();

	switch(i)
	{
		case 0:

			rb.data = buf;
			rb.maxLen = sizeof(buf);
			com.Read(&rb, ~0, US2CCLK(10));
			i++;

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0 && GetCRC16(rb.data, rb.len) == 0)
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
				if (curDscPPI != 0)
				{
					FreeDscPPI(curDscPPI);
					
					curDscPPI = 0;
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
//		CALL( UpdateFire()		);
		CALL( UpdateBlackFin()	);
	};

	i &= 1; // i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
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

		TestProcessData(dsc->data, dsc->len);

		processedPPI.Add(dsc);

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

	//static RTM32 tm;

	InitHardware();

	//LoadParams();

	com.Connect(6250000, 0);

//	InitNetAdress();

	while (1)
	{
		UpdateProcessData();

		u32 t = GetCycles32();

		*pPORTFIO_TOGGLE = 1<<8;

		Update();

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
