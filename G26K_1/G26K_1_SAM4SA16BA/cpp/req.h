#ifndef REQ_H__09_10_2014__10_31
#define REQ_H__09_10_2014__10_31

#include "ComPort.h"


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
	u32		tRPM; // время 1/6 оборота двигателя в мкс
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

__packed struct ReqDsp01	// старт оцифровки
{
	byte 	len;
	byte 	func;
	byte 	mode; 
	byte 	gain; 
	u16 	st;	 
	u16 	sl; 
	u16 	sd; 
	u16		thr;
	u16		descr;
	byte 	refgain; 
	u16 	refst;	 
	u16 	refsl; 
	u16 	refsd; 
	u16		refthr;
	u16		refdescr;

	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspDsp01	// старт оцифровки
{
	byte adr;
	byte func;
	u16 crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqDsp02	// чтение вектора
{
	byte 	len;
	byte 	func;
	byte 	mode; 
	byte 	gain; 
	u16 	st;	 
	u16 	sl; 
	u16 	sd; 
	u16		thr;
	u16		descr;
	byte 	refgain; 
	u16 	refst;	 
	u16 	refsl; 
	u16 	refsd; 
	u16		refthr;
	u16		refdescr;
	u16 	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspDsp02	// чтение вектора
{
	u16 rw; 
	u32 cnt; 
	u16 gain; 
	u16 st; 
	u16 len; 
	u16 delay; 
	u16 data[1024]; 
	u16 crc;
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
{ 
	byte 	len;
	byte 	func;
	word 	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
{ 
	byte 	adr;
	byte 	func;
	u16		flashLen; 
	u16		flashCRC;
	word 	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp06	// запись страницы во флэш
{ 
	byte 	len;
	byte 	func;
	u16		stAdr; 
	u16		count; 
	word	crc; 
	byte	data[258]; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspDsp06	// запись страницы во флэш
{ 
	byte 	adr;
	byte 	func;
	u16		res; 
	word	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp07	// перезагрузить блэкфин
{ 
	byte 	len;
	byte 	func;
	word 	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqTrm01	
{
	byte 	len;
	byte 	func;
	byte 	n; 
	word 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqTrm02	
{
	byte 	len;
	byte 	f;
	word 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspTrm02	
{
	byte f; 
	u16 hv; 
	u16 crc;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqTrm03	
{
	byte 	len;
	byte	f; 
	byte	fireCountM; 
	byte	fireCountXY; 
	u16		hv;
	word 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspTrm03	
{
	byte f; 
	u16 crc;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqMem
{
	u16 rw; 
	u32 cnt; 
	u16 gain; 
	u16 st; 
	u16 len; 
	u16 delay; 
	u16 data[1024*4]; 
	u16 crc;

	//byte adr;
	//byte func;
	
	//__packed union
	//{
	//	__packed struct  { word crc; } f1;  // Старт новой сессии
	//	__packed struct  { word crc; } f3;  
	//};
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspMem
{
	u16 rw; 
	u16 crc; 

	//byte	adr;
	//byte	func;
	
	//__packed union
	//{
	//	__packed struct  { word crc; } f1;  // Старт новой сессии
	//	__packed struct  { word crc; } f2;  // Запись вектора
	//	__packed struct  { word crc; } f3;  // 
	//	__packed struct  { word crc; } fFE;  // Ошибка CRC
	//	__packed struct  { word crc; } fFF;  // Неправильный запрос
	//};
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspMan60
{
	u16 rw; 
	u32 cnt; 
	u16 maxAmp[96]; 
	u16 power[96];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct REQ
{
	bool	ready;
	bool	crcOK;
	bool	checkCRC;
	bool	updateCRC;

	typedef void tRsp(REQ*);

	u16		tryCount;
	
	REQ *next;

	tRsp*	CallBack;
	void*	ptr;

	ComPort::WriteBuffer *wb;
	ComPort::ReadBuffer *rb;

	u32		preTimeOut, postTimeOut;

	REQ() : next(0), wb(0), rb(0), tryCount(0) {}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class RequestQuery
{
	REQ* _first;
	REQ* _last;
	REQ* _req;
	
	byte _state;

	u16	_crc;
	u16 _crcLen;

	byte* _crcPtr;


	ComPort *com;

	u32		count;

	bool _run;

public:

	RequestQuery(ComPort *p) : _first(0), _last(0), _run(true), _state(0), com(p), count(0) {}
	void Add(REQ* req);
	REQ* Get();
	bool Empty() { return _first == 0; }
	bool Idle() { return (_first == 0) && (_req == 0); }
	bool Stoped() { return _req == 0; }
	void Update();
	void Stop() { _run = false; }
	void Start() { _run = true; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct R02
{
	R02* next;

//	bool memNeedSend;
	ComPort::WriteBuffer	wb;
	ComPort::ReadBuffer		rb;
	REQ			q;
	ReqDsp02	req[2];
	RspDsp02	rsp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


struct RMEM
{
	RMEM* next;

//	R02*	r02;

	ComPort::WriteBuffer	wb;
	ComPort::ReadBuffer		rb;
	REQ		q;
	ReqMem	req;
	RspMem	rsp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++






#endif //REQ_H__09_10_2014__10_31
