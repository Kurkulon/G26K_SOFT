#ifndef COMPORT__31_01_2007__15_32
#define COMPORT__31_01_2007__15_32

#include "types.h"
#include "core.h"
#include "time.h"

#ifdef CPU_XMC48
	#include "COM_DEF.h"
#endif

#define COM_RS232 0
#define COM_RS485 1

class ComPort
{
  public:

	enum CONNECT_TYPE { ASYNC = 0, SYNC_M, SYNC_S };

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

	#ifdef CPU_XMC48

	struct LLI
	{
		volatile void*	SAR;
		volatile void*	DAR;
		LLI*	LLP;
		u32		CTLL;
		u32		CTLH;
		u32		DSTAT;
	};

	#endif


  protected:

	enum STATUS485 { WRITEING = 0, WAIT_READ = 1, READING = 2, READ_END = 3 };

//	union CUSART { void *vp; T_HW::S_UART *DU; T_HW::S_USART *SU; };

	#ifdef CPU_SAME53

		struct ComBase
		{
			bool used;
			T_HW::S_USART* const HU;
			T_HW::S_PORT* const pm;
			const dword pinRTS;
			const u32 dmaCh;
			const u32 dmaTrgSrcTX;
			const u32 dmaTrgSrcRX;
		};

		T_HW::S_PORT			*_pm;
		T_HW::S_USART 			*_SU;
		T_HW::S_DMAC::S_DMAC_CH	*_chdma;
		T_HW::DMADESC			*_dmadsc;
		T_HW::DMADESC			*_dmawrb;

		u32 _CTRLA;
		u32 _CTRLB;
		u32 _CTRLC;
		u32 _dmaCh;
		u32 _dma_act_mask;
		u32 _dma_trgsrc_tx;
		u32 _dma_trgsrc_rx;

		bool IsTransmited() { return (_SU->INTFLAG & USART_TXC)/* && ((_chdma->CTRLA & DMCH_ENABLE) == 0 || (_chdma->INTFLAG & DMCH_TCMPL))*/; }
		u32	GetDmaCounter() { u32 t = HW::DMAC->ACTIVE; return ((t & 0x9F00) == _dma_act_mask) ? (t >> 16) : _dmawrb->BTCNT; }
		u16	GetRecievedLen() { return _pReadBuffer->maxLen - _prevDmaCounter; }

	#elif defined(CPU_XMC48)

		struct ComBase
		{
			bool used;
			const byte dsel;
			T_HW::USIC_CH_Type* const HU;
			T_HW::PORT_Type* const pm;
			const dword pinRTS;
			const u32 usic_pid;
			const u32 inpr_sr;
			T_HW::GPDMA_Type* const dma;
			const u32 dmaCh;
			const u32 dlr;
		};

		LLI					_lli[4];

		T_HW::PORT_Type		*_pm;
		T_HW::USIC_CH_Type 	*_SU;
		T_HW::GPDMA_Type	*_dma;
		T_HW::GPDMA_CH_Type	*_chdma;
		u32					_dmaChMask;
		u32					_dlr;
		u32					_inpr_sr;
		u32					_brg;
		u32					_pcr;


		bool IsTransmited() { return (_SU->PSR & BUSY) == 0 && !(_dma->CHENREG & _dmaChMask); }
//		u16	GetDmaCounter() { return BLOCK_TS(_chdma->CTLH); }
		u32	GetDmaCounter() { return _chdma->DAR; }
//		u16	GetRecievedLen() { return _pReadBuffer->maxLen - _prevDmaCounter; }
		u16	GetRecievedLen() { return _chdma->DAR - _startDmaCounter; }

	#elif defined(WIN32)

		bool IsTransmited() { return true; }
		u32	GetDmaCounter() { return 0; }
		u16	GetRecievedLen() { return 0; }

	#endif

#ifndef WIN32
	static ComBase	_bases[3];

	ComBase			*_cb;

#endif

	bool			_connected;
	byte			_status485;
	byte			_portNum;

	u32				_startDmaCounter;
	u32				_prevDmaCounter;

	ReadBuffer		*_pReadBuffer;
	WriteBuffer		*_pWriteBuffer;



	word			_BaudRateRegister;

	dword			_ModeRegister;
	dword			_pinRTS;
	dword			_startTransmitTime;

//	dword			_startReceiveTime;
//	dword			_preReadTimeout;
	dword			_postReadTimeout;
	dword			_readTimeout;

	RTM			_rtm;

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


  public:
	  
	ComPort() : _connected(false), _status485(READ_END) {}


	bool		Connect(CONNECT_TYPE ct, byte port, dword speed, byte parity, byte stopBits);
	bool		ConnectAsyn(byte port, dword speed, byte parity, byte stopBits);
	bool		ConnectSync(byte port, dword speed, byte parity, byte stopBits);
	bool		Disconnect();
	bool		Update();
	void		InitHW();

	bool		Read(ComPort::ReadBuffer *readBuffer, dword preTimeout, dword postTimeout);
	bool		Write(ComPort::WriteBuffer *writeBuffer);

	void		TransmitByte(byte v);

	//static __irq void _IntHandlerCom1();
	//static __irq void _IntHandlerCom2();
	//static __irq void _IntHandlerCom3();

};

#endif // COMPORT__31_01_2007__15_32
