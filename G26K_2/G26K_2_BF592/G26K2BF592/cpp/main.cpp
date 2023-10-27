#include "hardware.h"
#include "ComPort.h"
#include "CRC16.h"
//#include "at25df021.h"
#include "list.h"
#include "fdct.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte build_date[512] = "\n" "G26K2BF592" "\n" __DATE__ "\n" __TIME__ "\n";

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ComPort com;

enum Pack { PACK_NO = 0, PACK_BIT12, PACK_ULAW, PACK_ADPCM, PACK_DCT0, PACK_DCT1, PACK_DCT2 };

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

static u16 wavesPerRoundCM = 100;	
static u16 wavesPerRoundIM = 100;
//static u16 filtrType = 0;
//static u16 packType = 0;

static List<DSCPPI> processedPPI;
static List<DSCPPI> cmWave;

static DSCPPI *curDsc = 0;

//static void SaveParams();

static u16 mode = 0; // 0 - CM, 1 - IM

struct SensVars
{
	u16 threshold;
	u16 deadIndx;
	u16 deadTime;
	u16 delay;
	u16 filtr;
	u16 fi_type;
	u16 pack;
	u16 fragLen;
};

static SensVars sensVars[3] = {0}; //{{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

//static u16 imThr = 0;
//static u16 imDescr = 0;
//static u16 imDelay = 0;
//static u16 refThr = 0;
//static u16 refDescr = 0;
//static u16 refDelay = 0;

static u16 refAmp = 0;
static u16 refTime = 0;

static i32 avrBuf[PPI_BUF_LEN] = {0};

static u16 flashCRC = 0;
static u32 flashLen = 0;
static u16 lastErasedBlock = ~0;

//const i16 sin_Table[40] = {	0,		3196,	6270,	9102,	11585,	13623,	15137,	16069,
//							16384,	16069,	15137,	13623,	11585,	9102,	6270,	3196,
//							0,		-3196,	-6270,	-9102,	-11585,	-13623,	-15137,	-16069,
//							-16384,	-16069,	-15137,	-13623,	-11585,	-9102,	-6270,	-3196,
//							0,		3196,	6270,	9102,	11585,	13623,	15137,	16069 };

const i16 sin_Table[10] = {	0,	11585,	16384,	11585,	0,	-11585,	-16384,	-11585,	0,	11585 };

//const i16 sin_Table[10] = {	16384,	16384,	16384,	16384,	-16384,	-16384,	-16384,	-16384,	16384,	16384 };

//const i16 wavelet_Table[8] = { 328, 4922, 12442, 9053, -2522, -4922, -1616, -153 };
//const i16 wavelet_Table[8] = { 0, 4176, 12695, 11585, 0, -4176, -1649, -196};
//const i16 wavelet_Table[16] = { 0,509,1649,2352,0,-6526,-12695,-10869,0,10869,12695,6526,0,-2352,-1649,-509 };
//const i16 wavelet_Table[32] = {-1683,-3326,-3184,0,5304,9229,7777,0,-10037,-15372,-11402,0,11402,15372,10037,0,-7777,-9229,-5304,0,3184,3326,1683,0,-783,-720,-321,0,116,94,37,0};
//const i16 wavelet_Table[32] = {0,385,1090,1156,0,-1927,-3270,-2698,0,3468,5450,4239,0,-5010,-7630,-5781,0,6551,9810,7322,0,-8093,-11990,-8864,0,9634,14170,10405,0,-11176,-16350,-11947};
const i16 wavelet_Table[32] = {0,-498,-1182,-1320,0,2826,5464,5065,0,-7725,-12741,-10126,0,11476,16381,11290,0,-9669,-12020,-7223,0,4713,5120,2690,0,-1344,-1279,-588,0,226,188,76};
//const i16 wavelet_Table2[8] = {0,4738,0,-16283,0,16283,0,-4738};
//const i16 wavelet_Table2[16] = {0,276,0,-1649,0,5904,0,-12692,0,16380,0,-12692,0,5904,0,-1649};
const i16 wavelet_Table2[16] = {0,-1182,0,5464,0,-12741,0,16381,0,-12020,0,5120,0,-1279,0,188};



//#define K_DEC (1<<2)
//#define K_DEC_MASK (K_DEC-1)

static FDCT_DATA fdct_w[FDCT_N];

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const byte ulaw_0816_expenc[256] = {
	0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,
	4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
	5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
	5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const i8  adpcmima_0416_index_tab[16] = {-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};
static const u16 adpcmima_0416_stepsize_tab[89] = {
		 7,     8,     9,    10,    11,    12,    13,    14,    16,    17,
		19,    21,    23,    25,    28,    31,    34,    37,    41,    45,
		50,    55,    60,    66,    73,    80,    88,    97,   107,   118,
	   130,   143,   157,   173,   190,   209,   230,   253,   279,   307,
	   337,   371,   408,   449,   494,   544,   598,   658,   724,   796,
	   876,   963,  1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
	  2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,
	  5894,  6484,  7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
	 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void PreProcessDspVars(ReqDsp01 *v, bool forced = false)
{
	static u16 freq[SENS_NUM] = {0};
	static u16 st[SENS_NUM] = {0};

	for (byte n = 0; n < SENS_NUM; n++)
	{
		SENS &sens = v->sens[n];
		u16 &fr = freq[n];
		u16 &s = st[n];

		if (sens.st == 0) sens.st = 1;

		if (sens.pack >= PACK_DCT0) sens.sl = (sens.sl + FDCT_N - 1) & ~(FDCT_N-1);

		if (sens.fi_Type == 1)
		{
			if (fr != sens.freq || forced)
			{
				fr = sens.freq;

				u16 f = (fr > 400) ? 400 : fr;

				s = (20000/8 + f/2) / f;
			};

			sens.st = s;
		}
		else if (sens.fi_Type == 2)
		{
			if (fr != sens.freq || forced)
			{
				fr = sens.freq;

				u16 f = (fr > 400) ? 400 : fr;

				s = (20000/4 + f/2) / f;
			};

			sens.st = s;
		};
	};

	SetDspVars(v);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc_01(const u16 *data, u16 len, ComPort::WriteBuffer *wb)
{
	static RspDsp01 rsp;

	ReqDsp01 *req = (ReqDsp01*)data;

	if (req->wavesPerRoundCM > 72) { req->wavesPerRoundCM = 72; }
	if (req->wavesPerRoundIM > 500) { req->wavesPerRoundIM = 500; }

	bool forced = (manCounter&0x7F) == 0;

	PreProcessDspVars(req, forced);

	mode = req->mode;

	for (byte n = 0; n < 2; n++)
	{
		SensVars &sv = sensVars[n];
		SENS &rs = req->sens[n];

		sv.threshold	= rs.threshold;
		sv.filtr		= rs.filtr;
		sv.fi_type		= rs.fi_Type;
		sv.pack			= rs.pack;
		sv.fragLen		= rs.fragLen;

		if (sv.deadTime != rs.deadTime || sv.delay != rs.sd || forced)
		{
			sv.deadTime = rs.deadTime;
			sv.delay = rs.sd;

			u16 t = sv.deadTime;

			t = (t > sv.delay) ? (t - sv.delay) : 0;

			if (t != 0)
			{
				sv.deadIndx = (t + rs.st/2) / rs.st;
			}
			else
			{
				sv.deadIndx = 0;
			};
		};
	};

	wavesPerRoundCM = req->wavesPerRoundCM;	
	wavesPerRoundIM = req->wavesPerRoundIM;
	
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
		wb->len = curDsc->dataLen * 2;	 
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
	RspCM &rsp = *((RspCM*)dsc.data);

	u16 *d = rsp.data;

	if (filtrType == 1)
	{
		i32 *ab = avrBuf;

		for (u32 i = rsp.hdr.sl+32; i > 0; i--)
		{
			i16 v = d[0] - 2048;

			*(d++) = v -= *ab/32;

			*(ab++) += v;
		};
	}
	else if (filtrType == 2)
	{
		//i32 av = 0;

		for (u32 i = rsp.hdr.sl+32; i > 0; i--)
		{
			i16 v = (d[0] + d[1])/2 - 2048;

			//av += v - av/2;

			*(d++) = v;//av / 2;
		};

		u16 t = (rsp.hdr.st+1)/2;
		rsp.hdr.sd += t;

		//if (rsp.hdr.sd >= t) rsp.hdr.sd -= t; else rsp.hdr.sd = 0;
	}
	else if (filtrType == 3)
	{
		i32 av = 0;
		i32 *ab = avrBuf;

		for (u32 i = rsp.hdr.sl+32; i > 0; i--)
		{
			i16 v = (d[2] - d[0] + d[3] - d[1])/4;// - 2048;

			*(d++) = v;// -= *ab/32;
		};
	}
	//else if (filtrType == 3 && (rsp.hdr.st&1) == 0)
	//{
	//	u32 len = rsp.hdr.sl;

	//	if (len > 512) len = 512;

	//	u16 *p = d+(len+15)*2+2;

	//	d += len+15;

	//	for (u32 i = len+16; i > 0; i--)
	//	{
	//		u16 v = (d[0]+d[1]+1)/2+1;

	//		*(p--) = (i16)((d[1]+v)/2 - 2048);
	//		*(p--) = (i16)((d[0]+v)/2 - 2048);
	//		d--;
	//	};

	//	p[0] = p[1];

	//	dsc.dataLen -= rsp.hdr.sl;
	//	rsp.hdr.sl = len*2;
	//	dsc.dataLen += rsp.hdr.sl;
	//	rsp.hdr.st /= 2;
	//}
	else
	{
		for (u32 i = rsp.hdr.sl+32; i > 0; i--)
		{
			*d = d[0] - 2048; d++;
		};
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void Filtr_Wavelet(DSCPPI &dsc, u16 descrIndx)
{
	RspCM &rsp = *((RspCM*)dsc.data);

	i16 *d = (i16*)(rsp.data);

	i32 max = -32768;
	i32 imax = -1;

	//u16 descr = (imDescr > imDelay) ? (imDescr - imDelay) : 0;
	//i32 ind = (descr + rsp.hdr.st/2) / rsp.hdr.st;

	//ind = rsp.hdr.sl - ind;

	if (expected_true(descrIndx < rsp.hdr.sl))
	{
		//i16 *p = d+rsp.hdr.sl;

		//for (i32 i = ArraySize(wavelet_Table); i > 0; i--) *(p++) = 0;

		d += descrIndx;

		for (i32 i = rsp.hdr.sl - descrIndx; i > 0 ; i--)
		{
			i32 sum = 0;

			for (i32 j = 1; j < ArraySize(wavelet_Table); j += 2)
			{
				sum += (i32)d[j] * wavelet_Table[j]; //sin_Table[j&7];
			};

			sum /= 16384*2;
			
			d++; //*(d++) = sum;

			if (sum < 0) sum = -sum;

			if (sum > max) { max = sum; imax = i; };
		};
	};

	if (imax >= 0)
	{
		imax = rsp.hdr.sl - imax;
		u32 t = rsp.hdr.sd + imax * rsp.hdr.st;
		rsp.hdr.fi_time  = (t < 0xFFFF) ? t : 0xFFFF;
		rsp.hdr.fi_amp = max;
		rsp.hdr.maxAmp = max;
		dsc.fi_index = imax;
	}
	else
	{
		rsp.hdr.fi_time	= ~0;
		rsp.hdr.fi_amp	= 0;
		rsp.hdr.maxAmp	= 0;
		dsc.fi_index	= ~0;
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void Filtr_Wavelet2(DSCPPI &dsc, u16 descrIndx)
{
	RspCM &rsp = *((RspCM*)dsc.data);

	i16 *d = (i16*)(rsp.data);

	i32 max = -32768;
	i32 imax = -1;

	if (expected_true(descrIndx < rsp.hdr.sl))
	{
		d += descrIndx;

		for (i32 i = rsp.hdr.sl - descrIndx; i > 0 ; i--)
		{
			i32 sum = 0;

			for (i32 j = 1; j < ArraySize(wavelet_Table2); j += 2)
			{
				sum += (i32)d[j] * wavelet_Table2[j];
			};

			sum /= 16384*2;
			
			d++; //*(d++) = sum;

			if (sum < 0) sum = -sum;

			if (sum > max) { max = sum; imax = i; };
		};
	};

	if (imax >= 0)
	{
		imax = rsp.hdr.sl - imax;
		u32 t = rsp.hdr.sd + imax * rsp.hdr.st;
		rsp.hdr.fi_time  = (t < 0xFFFF) ? t : 0xFFFF;
		rsp.hdr.fi_amp = max;
		rsp.hdr.maxAmp = max;
		dsc.fi_index = imax;
	}
	else
	{
		rsp.hdr.fi_time	= ~0;
		rsp.hdr.fi_amp	= 0;
		rsp.hdr.maxAmp	= 0;
		dsc.fi_index	= ~0;
	};
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma optimize_for_speed

static void GetAmpTimeIM_3(DSCPPI &dsc, u16 ind, u16 imThr)
{
	RspCM &rsp = *((RspCM*)dsc.data);

	rsp.hdr.fi_amp = 0; 
	rsp.hdr.fi_time = ~0;
	dsc.fi_index = ~0;

	u16 *data = rsp.data;
	
	//u16 descr = (imDescr > imDelay) ? (imDescr - imDelay) : 0;

	//u16 ind = descr * NS2CLK(50) / dsc.ppiclkdiv;

	if (ind >= rsp.hdr.sl) return;

	data += ind;

	u16 len = rsp.hdr.sl - ind;

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
		rsp.hdr.fi_amp = max;
		u32 t = rsp.hdr.sd + imax * rsp.hdr.st;
		rsp.hdr.fi_time = (t < 0xFFFF) ? t : 0xFFFF;
		u16 z = 100 / rsp.hdr.st;
		dsc.fi_index = (imax>z) ? (imax-z) : 0;
	};

	if (rsp.hdr.sl > ind)
	{
		for (u32 i = rsp.hdr.sl - ind; i > 0; i--)
		{
			i32 v = (i16)(*(data++));

			if (v < 0) v = -v;
			if (v > ampmax) ampmax = v;
		};
	};

	rsp.hdr.maxAmp = (ampmax < 0xFFFF) ? ampmax : 0xFFFF;
}

//#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Pack_1_Bit12(DSCPPI *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = 1;
	rsp.hdr.packLen = ((rsp.hdr.sl+3)/4)*3;

	u16 *s = rsp.data;
	u16 *d = rsp.data;

	for (u32 i = (rsp.hdr.sl+3)/4; i > 0; i--)
	{
		*(d++) = (s[0]&0xFFF)|(s[1]<<12);
		*(d++) = ((s[1]>>4)&0xFF)|(s[2]<<8);
		*(d++) = ((s[2]>>8)&0xF)|(s[3]<<4);
		s += 4;
	};

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + rsp.hdr.packLen;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Pack_2_uLaw(DSCPPI *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = 2;
	rsp.hdr.packLen = (rsp.hdr.sl+1)/2;

	u16 *s = rsp.data;
	byte *d = (byte*)rsp.data;

    byte sign, exponent, mantissa, sample_out;

	for (u32 i = rsp.hdr.packLen*2; i > 0; i--)
	{
		u16 sample_in = *(s++);

		sign = 0;

		if ((i16)sample_in < 0)
		{
			sign = 0x80;
			sample_in = -sample_in;
		};

		//if (sample_in > ulaw_0816_clip) sample_in = ulaw_0816_clip;

		sample_in += 0x10;//ulaw_0816_bias;

		exponent = ulaw_0816_expenc[(sample_in >> 4) & 0xff];

		mantissa = (sample_in >> (exponent + 0)) & 0xf;

		sample_out = (sign | (exponent << 4) | mantissa);

		//if (sample_out == 0) sample_out = 2;

		*(d++) = sample_out;
	};

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + rsp.hdr.packLen;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void Pack_3_ADPCMIMA(DSCPPI *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	rsp.hdr.packType = 3;
	rsp.hdr.packLen = (rsp.hdr.sl+3)/4;

	u16 *s = rsp.data;
	byte *d = (byte*)rsp.data;

    u16 stepsize = 7;     		/* Quantizer step size */
    i16 predictedSample = 0;	/* Output of ADPCM predictor */
    i8  index = 0;			/* Index into step size table */
    u8	newSample;			/* Result of encoding */

	byte bits = 0;

	for (u32 i = rsp.hdr.packLen*4; i > 0; i--)
	{
		i16 originalSample = *(s++);

		i16 dq = originalSample - predictedSample;
		if (dq >= 0) newSample = 0; else { newSample = 8; dq = -dq;}

		//newSample = (dq >> 13) & 8;
		//dq *= (((~dq) >> 14) & 2)-1;

		//byte mask = 4;
		//u16 tempStepsize = stepsize;
		i16 diff = 0;

		if (dq >= stepsize) { newSample |= 4; dq -= stepsize; diff += stepsize; }; stepsize >>= 1;
		if (dq >= stepsize) { newSample |= 2; dq -= stepsize; diff += stepsize; }; stepsize >>= 1;
		if (dq >= stepsize) { newSample |= 1; dq -= stepsize; diff += stepsize; }; stepsize >>= 1;

		diff += stepsize;

		if (newSample & 8) diff = -diff;

		predictedSample += diff;

		if (predictedSample > 2047) predictedSample = 2047; else if (predictedSample < -2047) predictedSample = -2047;

		index += adpcmima_0416_index_tab[newSample];

		if (index < 0) index = 0; else if (index > 67) index = 67;

		stepsize = adpcmima_0416_stepsize_tab[index];

		bits |= newSample << ((i&1)*4);

		if (i&1) *(d++) = bits, bits = 0;
	};

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + rsp.hdr.packLen;
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void PackDataCM(DSCPPI *dsc, u16 pack)
{
	switch (pack)
	{
		case PACK_NO:								break;
		case PACK_BIT12:	Pack_1_Bit12(dsc);		break;
		case PACK_ULAW:		Pack_2_uLaw(dsc);		break;
		case PACK_ADPCM:	Pack_3_ADPCMIMA(dsc);	break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

static void ProcessDataCM(DSCPPI &dsc)
{
	*pPORTGIO_SET = 1<<6;

	//u16 amp, time;

	//GetAmpTimeIM(dsc, amp, time);

	//u16 *d = dsc.data + dsc.offset;

	const ReqDsp01 *req = GetDspVars();

	RspHdrCM &rsp = *((RspHdrCM*)dsc.data);

	rsp.rw			= manReqWord|0x40;			//1. ответное слово
	//rsp.mmsecTime	= dsc.mmsec;
	//rsp.shaftTime	= dsc.shaftTime;
	//rsp.motoCount	= dsc.motoCount;
	//rsp.headCount	= dsc.shaftCount;
	rsp.ax			= req->ax;
	rsp.ay			= req->ay;
	rsp.az			= req->az;
	rsp.at			= req->at;
	//rsp.sensType	= dsc.sensType;
	//rsp.maxAmp		= dsc.maxAmp;
	//rsp.fi_amp		= dsc.fi_amp;
	//rsp.fi_time		= dsc.fi_time;
	//rsp.gain		= dsc.gain;
	//rsp.st 			= dsc.sampleTime;			//15. Шаг оцифровки
	//rsp.sl 			= dsc.len;					//16. Длина оцифровки (макс 2028)
	//rsp.sd 			= dsc.sampleDelay;			//17. Задержка оцифровки  
	rsp.packType	= 0;						//18. Упаковка
	rsp.packLen		= 0;						//19. Размер упакованных данных
	
	//u32 t = dsc.shaftTime - dsc.shaftPrev;

	//if (t != 0) { t = (36000 * (dsc.mmsec - dsc.shaftTime) + t/2) / t; };

	//u32 t = (72000 * dsc.rotCount + 74) / 147;
	u32 t = (501551 * dsc.rotCount + 512) >> 10;

	if (t >= 36000) t -= 36000;

	rsp.angle = t;

	cmWave.Add(&dsc);

	*pPORTGIO_CLEAR = 1<<6;
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FragDataCM(DSCPPI *dsc)
{
	RspCM &rsp = *((RspCM*)dsc->data);

	u16 fragLen = sensVars[rsp.hdr.sensType].fragLen;

	u16 stind = dsc->fi_index;

	if (fragLen == 0 || stind >= rsp.hdr.sl) return;

	if (fragLen > rsp.hdr.sl)
	{
		fragLen = rsp.hdr.sl;
		stind = 0;
	}
	else if ((stind + fragLen) > rsp.hdr.sl)
	{
		stind = rsp.hdr.sl - fragLen;
	};

	if (stind > 0)
	{
		u16 *s = rsp.data + stind;
		u16 *d = rsp.data;

		for (u32 i = fragLen; i > 0; i--) *(d++) = *(s++);

		rsp.hdr.sd += stind * rsp.hdr.st;
	};

	dsc->dataLen = dsc->dataLen - rsp.hdr.sl + fragLen;
	
	rsp.hdr.sl = fragLen;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#pragma optimize_for_speed

static void SendReadyDataIM(DSCPPI *dsc, u16 len)
{
	*pPORTGIO_SET = 1<<7;

	const ReqDsp01 *req = GetDspVars();

	RspIM &rsp = *((RspIM*)dsc->data);

	rsp.hdr.rw			= manReqWord|0x50;	//1. ответное слово
	//rsp.hdr.mmsecTime	= dsc->mmsec;		//2. Время (0.1мс). младшие 2 байта
	//rsp.hdr.shaftTime	= dsc->shaftTime;	//4. Время датчика Холла (0.1мс). младшие 2 байта
	rsp.hdr.ax			= req->ax;
	rsp.hdr.ay			= req->ay;
	rsp.hdr.az			= req->az;
	rsp.hdr.at			= req->at;
	//rsp.hdr.gain		= dsc->gain;		//10. КУ
	rsp.hdr.refAmp		= refAmp;
	rsp.hdr.refTime		= refTime;
	rsp.hdr.len			= len;				//11. Длина (макс 1024)

	dsc->dataLen = sizeof(rsp.hdr)/2 + len*2;

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
	static u16 wpr = 180;

	RspCM &src = *((RspCM*)dsc.data);

	if (src.hdr.headCount != prevShaftCount)
	{
		prevShaftCount = src.hdr.headCount;

		if (imdsc != 0)
		{
			SendReadyDataIM(imdsc, i);

			imdsc = 0;
		};
	};

	if (imdsc == 0)
	{
		wpr = wavesPerRoundIM;
		count = wpr*9/8; if (count > 512) count = 512;
		cmCount = (wpr+8) / 16;
		i = 0;

		imdsc = AllocDscPPI();

		if (imdsc != 0)
		{
			RspIM *rsp = (RspIM*)imdsc->data;

			rsp->hdr.mmsecTime	= src.hdr.mmsecTime;
			rsp->hdr.shaftTime	= src.hdr.shaftTime;
			rsp->hdr.gain		= src.hdr.gain;
			rsp->hdr.ax			= src.hdr.ax;
			rsp->hdr.ay			= src.hdr.ay;
			rsp->hdr.az			= src.hdr.az;
			rsp->hdr.at			= src.hdr.at;

			u16 *d = rsp->data;

			for (u32 i = 10; i > 0; i--) { *(d++) = 0; };
		};
	};

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
			*(data++) = src.hdr.fi_amp;
			*(data++) = src.hdr.fi_time;
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
		cmCount = (wpr+4) / 8;

		ProcessDataCM(dsc);
	}
	else
	{
		FreeDscPPI(&dsc);
	};

	cmCount -= 1;
}

#pragma optimize_as_cmd_line

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Pack_DCT_uLaw(FDCT_DATA *s, byte *d, u16 len, byte scale)
{
    byte sign, exponent, mantissa, sample_out;

	for (; len > 0; len--)
	{
		u16 sample_in = (i16)((*(s++))>>scale);

		sign = 0;

		if ((i16)sample_in < 0)
		{
			sign = 0x80;
			sample_in = -sample_in;
		};

		//if (sample_in > ulaw_0816_clip) sample_in = ulaw_0816_clip;

		sample_in += 0x84;//ulaw_0816_bias;

		exponent = ulaw_0816_expenc[(sample_in >> 7) & 0xff];

		mantissa = (sample_in >> (exponent + 3)) & 0xf;

		sample_out = (sign | (exponent << 4) | mantissa);

		//if (sample_out == 0) sample_out = 2;

		*(d++) = sample_out;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateCM()
{
	static byte state = 0;
	static DSCPPI *dsc = 0;
	static u16 packLen = 0;
	static u16 index = 0;
	static byte OVRLAP = 3;
	static u16 scale = 0;

	switch (state)
	{
		case 0: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
			
			dsc = cmWave.Get();

			if (dsc != 0)
			{
				RspCM *rsp = (RspCM*)dsc->data;

				*pPORTFIO_SET = 1<<7;

				FragDataCM(dsc);

				*pPORTFIO_CLEAR = 1<<7;

				state++;
			};

			break;

		case 1: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		{
			RspCM *rsp = (RspCM*)dsc->data; 
	
			*pPORTFIO_SET = 1<<7;

			if (sensVars[rsp->hdr.sensType].pack < PACK_DCT0)
			{
				PackDataCM(dsc, sensVars[rsp->hdr.sensType].pack);

				//dsc->data[dsc->dataLen] = GetCRC16(&rsp->hdr, sizeof(rsp->hdr));
				//dsc->dataLen += 1;

				processedPPI.Add(dsc);

				state = 0;
			}
			else
			{
				index = 0;
				rsp->hdr.packType = sensVars[rsp->hdr.sensType].pack;
				rsp->hdr.packLen = 0;
				OVRLAP = (rsp->hdr.packType > PACK_DCT0) ? 7 : 3;
				state++;
			}

			*pPORTFIO_CLEAR = 1<<7;

			break;
		};

		case 2: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
		{
			RspCM *rsp = (RspCM*)dsc->data; 

			*pPORTFIO_SET = 1<<7;

			i16 *p = (i16*)(rsp->data + index);

			for (u32 n = 0; n < FDCT_N; n++) fdct_w[n] = *(p++);

			*pPORTFIO_CLEAR = 1<<7;
			*pPORTFIO_SET = 1<<7;

			//u32 t = cli();

			FastDctLee_transform(fdct_w, FDCT_LOG2N);

			//sti(t);

			*pPORTFIO_CLEAR = 1<<7;

			state++;

			break;
		};

		case 3: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
		{
			RspCM *rsp = (RspCM*)dsc->data; 

			*pPORTFIO_SET = 1<<7;

			//fdct_w[0] = 0;

			byte shift = 5 - (sensVars[rsp->hdr.sensType].pack- PACK_DCT0);

			FDCT_DATA max = 0;
			 
			for (u32 i = 0; i < FDCT_N; i++)
			{
				FDCT_DATA t = fdct_w[i];

				if (t < 0) t = -t;

				if (t > max) max = t;
			};

			FDCT_DATA *p = fdct_w + FDCT_N - 1;
			FDCT_DATA lim = max >> shift;

			scale = 0;

			while (max > 32000) { max /= 2; scale += 1; };

			packLen = FDCT_N;

			for (u32 i = FDCT_N; i > 0; i--)
			{
				FDCT_DATA t = p[0];

				if (t < 0) t = -t;

				if (t < lim)
				{
					*(p--) = 0;
				}
				else
				{
					packLen = i;
					break;
				};
			};

			packLen = (packLen+1) & ~1;

			*pPORTFIO_CLEAR = 1<<7;

			state++;

			break;
		};

		case 4: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
		{
			RspCM *rsp = (RspCM*)dsc->data; 

			PackDCT *pdct = (PackDCT*)(rsp->data+rsp->hdr.packLen);

			//*pPORTFIO_SET = 1<<7;

			//FastDctLee_inverseTransform(fdct_w, FDCT_LOG2N);

			//*pPORTFIO_CLEAR = 1<<7;
			*pPORTFIO_SET = 1<<7;

			Pack_DCT_uLaw(fdct_w, pdct->data, packLen, scale);
			pdct->len = packLen;
			pdct->scale = scale;
			rsp->hdr.packLen += 1 + packLen/2;

			index += FDCT_N - OVRLAP;

			if ((index+FDCT_N) <= rsp->hdr.sl)
			{
				state = 2;
			}
			else
			{
				dsc->dataLen = dsc->dataLen - rsp->hdr.sl + rsp->hdr.packLen;

				rsp->hdr.sl = index + OVRLAP;

				processedPPI.Add(dsc);

				state = 0;
			};

			*pPORTFIO_CLEAR = 1<<7;


			break;
		};
	}; // switch (i);
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMode()
{
	static byte i = 0;
	static DSCPPI *dsc = 0;

	switch (i)
	{
		case 0:

			dsc = GetDscPPI();

			if (dsc != 0)
			{
				i++;
			}
			else
			{
				Update();
			};

			break;

		case 1: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		{
			RspHdrCM *rsp = (RspHdrCM*)dsc->data;

			*pPORTFIO_SET = 1<<7;
			
			Filtr_Data(*dsc, sensVars[rsp->sensType].filtr);
			
			*pPORTFIO_CLEAR = 1<<7;

			i++;

			break;
		};

		case 2: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		{
			RspHdrCM *rsp = (RspHdrCM*)dsc->data;

			const SensVars &sens = sensVars[rsp->sensType];

			*pPORTFIO_SET = 1<<7;

			if (sens.fi_type == 0)
			{
				GetAmpTimeIM_3(*dsc, sens.deadIndx, sens.threshold);
			}
			else if (sens.fi_type == 1)
			{
				Filtr_Wavelet(*dsc, sens.deadIndx);
			}
			else
			{
				Filtr_Wavelet2(*dsc, sens.deadIndx);
			};

			*pPORTFIO_CLEAR = 1<<7;

			i++;

			break;
		};

		case 3: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		{				
			RspHdrCM *rsp = (RspHdrCM*)dsc->data;

			*pPORTFIO_SET = 1<<7;

			if (rsp->sensType == 0)
			{
				switch (mode)
				{
					case 0: ProcessDataCM(*dsc); break;
					case 1: ProcessDataIM(*dsc); break;
				};
			}
			else
			{
				refAmp	= rsp->fi_amp;
				refTime = rsp->fi_time;
				
				ProcessDataCM(*dsc);
			};

			*pPORTFIO_CLEAR = 1<<7;

			i = 0;

			break;
		};

	}; // switch (i);

	UpdateCM();

	if ((i&1) == 0) Update();
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

	com.Connect(6250000, 2);

	u32 t = cli();

	FDCT_Init();

	sti(t);

	while (1)
	{
		*pPORTFIO_SET = 1<<8;
		
		UpdateMode();

		*pPORTFIO_CLEAR = 1<<8;

	};

//	return 0;
}
