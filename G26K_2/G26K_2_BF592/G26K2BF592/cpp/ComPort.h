#ifndef COMPORT__31_01_2007__15_32
#define COMPORT__31_01_2007__15_32

#include "types.h"
#include "core.h"


//#define COM_RS232 0
//#define COM_RS485 1


class ComPort
{
  public:

	struct ReadBuffer
	{
		bool	recieved;
		word	len;
		word	maxLen;
		void*	data;
	};

	struct WriteBuffer
	{
		bool	transmited;
		word	len;
		void*	data;
	};

  protected:

	enum STATUS485 { WRITEING = 0, WAIT_READ = 1, READING = 2, READ_END = 3 };

	bool			_connected;
	byte			_status485;
//	byte			_portNum;

	word			_prevDmaCounter;

	ReadBuffer		*_pReadBuffer;
	WriteBuffer		*_pWriteBuffer;



	word			_BaudRateRegister;

	dword			_ModeRegister;
//	dword			_maskRTS;

	dword			_startTransmitTime;
	dword			_startReceiveTime;
	dword			_preReadTimeout;
	dword			_postReadTimeout;


	void 		EnableTransmit(void* src, word count);
	void 		DisableTransmit();
	void 		EnableReceive(void* dst, word count);
	void 		DisableReceive();

	//static		bool _InitCom(byte i, ComPort* cp);
	//static		bool _DeinitCom(byte i, ComPort* cp);

	static ComPort *_objCom1;
	static ComPort *_objCom2;
	static ComPort *_objCom3;

	word 		BoudToPresc(dword speed);
	void		SetBoudRate(word presc);

  public:
	  
	ComPort() : _connected(false), _status485(READ_END) {}

	bool		Connect(dword speed, byte parity);
	bool		Disconnect();
	bool		Update();

	bool		Read(ComPort::ReadBuffer *readBuffer, dword preTimeout, dword postTimeout);
	bool		Write(ComPort::WriteBuffer *writeBuffer);


};

#endif // COMPORT__31_01_2007__15_32
