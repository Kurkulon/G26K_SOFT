#ifndef COMPORT__31_01_2007__15_32
#define COMPORT__31_01_2007__15_32


#include "types.h"
#include "core.h"

#ifdef WIN32
#include <windows.h>
#endif

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

	struct ComBase
	{
		bool used;
		T_HW::S_USART	*usart;
		const dword		maskRTS;

		//byte*		rdata;
		//u16			rlen;

		//byte*		wdata;
		//u16			wlen;

		//T_HW::LPC_IHP	ir;
		//T_HW::LPC_IHP	iw;

		//byte			ivect;

		//u32				tm;
	};

	static ComBase _bases[3];

	bool			_connected;
	byte			_status485;
	byte			_portNum;

	byte			_txDmaCh;
	u32				_txDmaMask;

	byte			_rxDmaCh;
	u32				_rxDmaMask;

	u16				_prevDmaCounter;

	ReadBuffer		*_pReadBuffer;
	WriteBuffer		*_pWriteBuffer;


	word			_BaudRateRegister;

	dword			_ModeRegister;
	dword			_maskRTS;

	//dword			_startTransmitTime;
	//dword			_startReceiveTime;
	dword			_preReadTimeout;
	dword			_postReadTimeout;

	T_HW::S_USART 	*_usart;

	void 		EnableTransmit(void* src, word count);
	void 		DisableTransmit();
	void 		EnableReceive(void* dst, word count);
	void 		DisableReceive();

	//static		bool _InitCom(byte i, ComPort* cp);
	//static		bool _DeinitCom(byte i, ComPort* cp);

	//static ComPort *_objCom1;
	//static ComPort *_objCom2;
	//static ComPort *_objCom3;

	word 		BoudToPresc(dword speed);

	//static __irq void ReadHandler_0();
	//static __irq void ReadHandler_1();
	//static __irq void ReadHandler_2();

	//static __irq void WriteHandler_0();
	//static __irq void WriteHandler_1();
	//static __irq void WriteHandler_2();



  public:
	  
	ComPort() : _connected(false), _status485(READ_END) {}

	bool		Connect(byte port, dword speed, byte parity);
	bool		Disconnect();
	bool		Update();

	bool		Read(ComPort::ReadBuffer *readBuffer, dword preTimeout, dword postTimeout);
	bool		Write(ComPort::WriteBuffer *writeBuffer);

	void		TransmitByte(byte v);
};

#endif // COMPORT__31_01_2007__15_32
