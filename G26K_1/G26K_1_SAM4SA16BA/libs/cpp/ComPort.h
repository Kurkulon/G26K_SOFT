#ifndef COMPORT__31_01_2007__15_32
#define COMPORT__31_01_2007__15_32

#include "types.h"
#include "core.h"

#define COM_RS232 0
#define COM_RS485 1

#ifdef WIN32
#include <windows.h>
#endif

class ComPort
{
  public:

	struct ReadBuffer
	{
		bool	recieved;
		bool	pretimeout;
		byte	CSR;
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

	union CUSART { void *vp; T_HW::S_UART *DU; T_HW::S_USART *SU; };

	struct ComBase
	{
		bool used;
		const CUSART HU;
		T_HW::S_PIO* const pm;
		const dword maskRTS;
	};

	static ComBase _bases[4];

	bool			_connected;
	byte			_status485;
	byte			_portNum;

	word			_prevDmaCounter;

	ReadBuffer		*_pReadBuffer;
	WriteBuffer		*_pWriteBuffer;



#ifndef WIN32

	word			_BaudRateRegister;

	dword			_ModeRegister;
	dword			_maskRTS;
	T_HW::S_PIO		*_pm;
	dword			_startTransmitTime;
	dword			_startReceiveTime;
	dword			_preReadTimeout;
	dword			_postReadTimeout;
	T_HW::S_USART 	*_SU;

#else

	dword		_readBytes;
	dword		_writeBytes;

	HANDLE		_comHandle;
	OVERLAPPED	_ovlRead;
	OVERLAPPED	_ovlWrite;
	COMMTIMEOUTS _cto;

	static int		_portTableSize;
	static dword	_portTable[16];

#endif

	void 		EnableTransmit(void* src, word count);
	void 		DisableTransmit();
	void 		EnableReceive(void* dst, word count);
	void 		DisableReceive();

	//static		bool _InitCom(byte i, ComPort* cp);
	//static		bool _DeinitCom(byte i, ComPort* cp);

	static ComPort *_objCom1;
	static ComPort *_objCom2;
	static ComPort *_objCom3;

#ifndef WIN32

	word 		BoudToPresc(dword speed);

#else

	void		BuildPortTable();

#endif


  public:
	  
#ifndef WIN32

	ComPort() : _connected(false), _status485(READ_END) {}

#else

	ComPort() : _connected(false), _status485(READ_END), _comHandle(INVALID_HANDLE_VALUE) {}
	bool		Connect(const char* comPort, dword bps, byte parity);

#endif

	bool		Connect(byte port, dword speed, byte parity);
	bool		Disconnect();
	bool		Update();

	bool		Read(ComPort::ReadBuffer *readBuffer, dword preTimeout, dword postTimeout);
	bool		Write(ComPort::WriteBuffer *writeBuffer);

	void		TransmitByte(byte v);

	static __irq void _IntHandlerCom1();
	static __irq void _IntHandlerCom2();
	static __irq void _IntHandlerCom3();

};

#endif // COMPORT__31_01_2007__15_32
