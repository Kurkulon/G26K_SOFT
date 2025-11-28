#ifndef REQ_H__09_10_2014__10_31
#define REQ_H__09_10_2014__10_31

#include "RequestQuery.h"

#include "ComPort\ComPort.h"

#include "list.h"
#include "FLASH\NandFlash.h"

//struct Request
//{
//	byte adr;
//	byte func;
//	
//	union
//	{
//		struct  { byte n; word crc; } f1;  // старт оцифровки
//		struct  { byte n; byte chnl; word crc; } f2;  // чтение вектора
//		struct  { byte dt[3]; byte ka[3]; word crc; } f3;  // установка периода дискретизации вектора и коэффициента усиления
//	};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqMoto
{
	u16 	rw;
	u16 	enableMotor; 
	u32		tRPM;		// время 1/6 оборота двигателя в мкс
	u16		limCurrent; // Ограничение тока двигателя (мА)
	u16		maxCurrent; // Аварийный ток двигателя (мА)
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspMoto
{
	u16 	rw;
	u16 	mororStatus; 
	u16		current;
	u16		rpm;
	u16		motoCounter;
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqBootMotoHS { unsigned __int64 guid; u16 crc; };
__packed struct RspBootMotoHS { unsigned __int64 guid; u16 crc; };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct ReqBootMoto
{
	u32 func;

	union
	{
		struct { u32 flashLen;  u16 align; u16 crc; } F01; // Get Flash CRC
		struct { u32 padr; u32 page[16]; u16 align; u16 crc; } F02; // Write page
		struct { u16 align; u16 crc; } F03; // Exit boot loader
	};
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct RspBootMoto
{
	u32 func;

	union
	{
		struct { u32 flashLen; u16 flashCRC; u16 crc; } F01;
		struct { u32 padr; u32 status; u16 align; u16 crc; } F02;
		struct { u16 align; u16 crc; } F03;							// Exit boot loader
	};
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct SENS
{
	u16 gain;
	u16 sampleTime;
	u16 sampleLen;
	u16 sampleDelay;
	u16 deadTime;
	u16 descriminant;
	u16 freq;
	u16 filtrType;
	u16 packType;
	u16 fi_Type;
	u16 fragLen;
};
	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct ReqDsp01_old	// чтение вектора
//{
//	u16 	rw;
//	u16 	mode; 
//	u32 	mmsecTime; 
//	u32		hallTime; 
//	u16		motoCount; 
//	u16		headCount;
//	u16		ax; 
//	u16		ay; 
//	u16		az; 
//	u16		at;
//	u16		sensType; 
//	u16		angle;
//	u16 	gain; 
//	u16 	st;	 
//	u16 	sl; 
//	u16 	sd; 
//	u16		thr;
//	u16		descr;
//	u16		freq;
//	u16 	refgain; 
//	u16 	refst;	 
//	u16 	refsl; 
//	u16 	refsd; 
//	u16		refthr;
//	u16		refdescr;
//	u16		refFreq;
//	u16		vavesPerRoundCM;
//	u16		vavesPerRoundIM;
//	u16		filtrType;
//	u16		packType;
//
//	u16 	crc;  
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqDsp01	// чтение вектора
{
	enum { VERSION = 1 };

	u16 	rw;
	u16		len;				// Длина структуры
	u16		version;			// Версия структуры

	u16 	mode; 
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	SENS	sens1;
	SENS	refSens;
	u16		vavesPerRoundCM;
	u16		vavesPerRoundIM;
	u16		fireVoltage;		// Напряжение излучателя (В)

	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspHdrCM	// 0xAD40
{
	u16 	rw;				//1. ответное слово	
	u32 	time;			//2. Время (0.1мс)
	u32		hallTime;		//4. Время датчика Холла (0.1мс)
	u16		motoCount; 		//6. Счётчик оборотов двигателя (1/6 об)
	u16		headCount;		//7. Счётчик оборотов головки (об)
	u16		ax; 			//8. AX (уе)
	u16		ay; 			//9. AY (уе)
	u16		az; 			//10. AZ (уе)
	u16		at;				//11. AT (short 0.01 гр)
	u16		sensType; 		//12. Тип датчика (0 - измерительный датчик, 1 - опорный датчик)
	u16		angle;			//13. Угол поворота (0.01гр)(ushort)
	u16		maxAmp;			//14. Амплитуда максимум по всей волне (у.е)
	u16		fi_amp;			//15. Амплитуда по первому вступлению (у.е)
	u16		fi_time;		//16. Время по первому вступлению (0.05 мкс)
	u16 	gain; 			//17. КУ
	u16 	st;	 			//18. Шаг оцифровки
	u16 	sl; 			//19. Длина оцифровки (макс 2028)
	u16 	sd; 			//20. Задержка оцифровки  
	u16		packType;		//21. Упаковка (1 - 4:3 по 12 бит, 2 - 2:1 u-Law, 3 - 4:1 ADPCM по 4 бита)
	u16		packLen;		//22. Размер упакованных данных
};							

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct PacketHdrCM	// 0xAD40
{
	u16 	deltatime;		//1. Дельта времени датчика Холла (0.1мс).
	u16 	angle;			//2. Угол поворота (0.01гр)(ushort)
	u16		fi_amp;			//3. Амплитуда по первому вступлению (у.е)
	u16		fi_time; 		//4. Время по первому вступлению (0.05 мкс)
	u16		packLen;		//5. Размер упакованных данных
	u16		data[0]; 		//6.... данные волновой картины 
};

#define MAX_WAVEPACKET_LEN 128

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspHdrIM	// 0xAD50
{
	u16 	rw;
	u32 	time;		//mmsecTime; 
	u32		hallTime;	//shaftTime; 
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16 	gain; 
	u16		refAmp;
	u16		refTime;
	u16		dataLen;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed union RspDsp01	// чтение вектора
{
	__packed struct { RspHdrCM hdr; u16 data[1024]; } CM;
	__packed struct { RspHdrIM hdr; u16 data[1024]; } IM;
	__packed struct { u16 rw; u16 len; u16 version; u16 fireVoltage; u16 motoVoltage; u16 crc; } v01;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
{ 
	u16		rw; 
	u16 	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
{ 
	u16		rw; 

	__packed union
	{
		__packed struct { u16 flashLen; u16 flashCRC; u16 crc; } v1;
		__packed struct { u16 flashLen; u32 startAdr; u16 flashCRC; u16 crc; } v2;
	};
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp06	// запись страницы во флэш
{ 
	u16		rw; 
	u16		stAdr; 
	u16		count; 
	byte	data[258]; 
	u16		crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspDsp06	// запись страницы во флэш
{ 
	u16		rw; 
	u16		res; 
	word	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp07	// перезагрузить блэкфин
{ 
	u16		rw; 
	word 	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union ReqUnion
{
	ReqDsp01 	dsp01;	
	ReqDsp05 	dsp05;	
	ReqDsp06 	dsp06;	
	ReqDsp07 	dsp07;	
	ReqMoto		moto;	
	ReqBootMoto bootMoto;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct REQ : public PtrItem<REQ>
//{
//	PTR_LIST_FRIENDS(REQ);
//
//	bool	ready;
//	bool	crcOK;
//	bool	checkCRC;
//	bool	updateCRC;
//
//	typedef void tRsp(Ptr<REQ> &q);
//
//	u16		tryCount;
//	
//	//REQ *next;
//
//	tRsp		*CallBack;
//	Ptr<MB>	rsp;
//
//	ComPort::WriteBuffer wb;
//	ComPort::ReadBuffer rb;
//
//	u32		preTimeOut, postTimeOut;
//
//	byte	reqData[(sizeof(ReqUnion)+64) & ~3];
//
//protected:
//
//	virtual void _FreeCallBack() { rsp.Free(); }
//
//public:
//
//	//void	Free() { if (this != 0) rsp.Free(), PtrItem<REQ>::Free(); }
//
//	REQ() : tryCount(0) { }
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//class RequestQuery
//{
//	//REQ*			_first;
//	//REQ*			_last;
//	Ptr<REQ>		_req;
//
//	ListPtr<REQ>	reqList;
//	
//	byte			_state;
//
//	u16				_crc;
//	u16 			_crcLen;
//
//	byte*			_crcPtr;
//
//
//	ComPort			*com;
//
//	//u32			count;
//
//	bool			_run;
//
//public:
//
//				RequestQuery(ComPort *p) : _state(0), com(p), _run(true) {}
//	void		Add(const Ptr<REQ>& req)	{ reqList.Add(req); }
//	Ptr<REQ>	Get()						{ return reqList.Get(); }
//	//bool Empty() { return reqList.Empty(); }
//	//bool Idle() { return (_first == 0) && (_req == 0); }
//	bool Stoped() { return !_req.Valid(); }
//	void Update();
//	void Stop() { _run = false; }
//	void Start() { _run = true; }
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct R01
//{
//	R01*	next;
//
//protected:
//
//	u32		count;
//
//	static List<R01> freeList;
//
//public:
//
////	bool memNeedSend;
//	//ComPort::WriteBuffer	wb;
//	//ComPort::ReadBuffer		rb;
//	REQ			q;
//	ReqDsp01	req;
//	RspDsp01	rsp;
//
//	R01() : next(0) { freeList.Add(this); }
//
//	static	R01*	Alloc()	{ R01* p = freeList.Get(); if (p != 0) p->count = 1; return p; };
//			void	Free()	{ if (this != 0 && count != 0) { count--; if (count == 0) freeList.Add(this); }; }
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/*struct RMEM
{
	RMEM* next;

//	R01*	r02;

	ComPort::WriteBuffer	wb;
	ComPort::ReadBuffer		rb;
	REQ		q;
	ReqMem	req;
	RspMem	rsp;
};*/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++






#endif //REQ_H__09_10_2014__10_31
