#include <stdio.h>
//#include <conio.h>

#include "ComPort.h"
#include "time.h"
//#include "hardware.h"

#ifdef _DEBUG_
//	static const bool _debug = true;
#else
//	static const bool _debug = false;
#endif

extern dword millisecondsCount;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

ComPort::ComBase	ComPort::_bases[3] = { 
	{false, HW::USART0,	 1<<27	}, 
	{false, HW::USART1,	 0		},
	{false, HW::USART2,	 0		}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Connect(byte port, dword speed, byte parity)
{
	if (_connected || port > 2 || _bases[port].used)
	{
		return false;
	};

	_portNum = port;

	_rxDmaCh = port*2;
	_txDmaCh = _rxDmaCh+1;
	_rxDmaMask = 1<<_rxDmaCh;
	_txDmaMask = 1<<_txDmaCh;


	ComBase &cb = _bases[port];

	_usart = cb.usart;
	_maskRTS = cb.maskRTS;

	_BaudRateRegister = BoudToPresc(speed);

	_ModeRegister = 0x04;

	switch (parity)
	{
		case 0:		// нет четности
			_ModeRegister |= 0<<4;
			break;

		case 1:
			_ModeRegister |= 3<<4;
			break;

		case 2:
			_ModeRegister |= 2<<4;
			break;
	};

	_usart->CFG = _ModeRegister;
	_usart->BRG = _BaudRateRegister;
	_usart->OSR = 15;

	_status485 = READ_END;

	return _connected = cb.used = true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Disconnect()
{
	if (!_connected) return false;

	DisableReceive();
	DisableTransmit();

	_status485 = READ_END;

	_connected = _bases[_portNum].used = false;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

word ComPort::BoudToPresc(dword speed)
{
	if (speed == 0) return 0;

	return (MCK/16+speed/2) / speed - 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::EnableTransmit(void* src, word count)
{
	_usart->CFG &= ~1;	// Disable transmit and receive

	HW::GPIO->SET0 = _maskRTS;

	HW::DMA->CTRL = 1;

	HW::DMA->ENABLESET0 = _txDmaMask;	

//	HW::DMA->INTENSET0 = 1<<ch;		

	HW::DMA->CH[_txDmaCh].CFG = PERIPHREQEN | TRIGBURST_SNGL | CHPRIORITY(3);	

	DmaTable[_txDmaCh].SEA = (byte*)src + count - 1;	
	DmaTable[_txDmaCh].DEA = &_usart->TXDATA;			
	DmaTable[_txDmaCh].NEXT = 0;

	HW::DMA->CH[_txDmaCh].XFERCFG = CFGVALID | SETINTA | SWTRIG | WIDTH_8 | SRCINC_1 | DSTINC_0 | XFERCOUNT(count);

	HW::DMA->SETVALID0 = _txDmaMask;

	_usart->CFG |= 1;

	_status485 = WRITEING;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::DisableTransmit()
{
	_usart->CFG &= ~1;	// Disable transmit and receive
	_usart->INTENCLR = 4;
	HW::GPIO->CLR0 = _maskRTS;

	HW::DMA->ENABLECLR0 = _txDmaMask;
	HW::DMA->ABORT0 = _txDmaMask;
	HW::DMA->ERRINT0 = _txDmaMask;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::EnableReceive(void* dst, word count)
{
	_usart->CFG &= ~1;	// Disable transmit and receive

	HW::GPIO->CLR0 = _maskRTS;

	HW::DMA->CTRL = 1;

	HW::DMA->ENABLESET0 = _rxDmaMask;	

//	HW::DMA->INTENSET0 = 1<<ch;		

	HW::DMA->CH[_rxDmaCh].CFG = PERIPHREQEN | TRIGBURST_SNGL | CHPRIORITY(3);	


	DmaTable[_rxDmaCh].SEA = &_usart->RXDATA;		
	DmaTable[_rxDmaCh].DEA = (byte*)dst + count - 1;	
	DmaTable[_rxDmaCh].NEXT = 0;

	_prevDmaCounter = count - 1;

	HW::DMA->CH[_rxDmaCh].XFERCFG = CFGVALID | SWTRIG | WIDTH_8 | SRCINC_0 | DSTINC_1 | XFERCOUNT(count);

	HW::DMA->SETVALID0 = _rxDmaMask;

	_usart->CFG |= 1;

	_status485 = WAIT_READ;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::DisableReceive()
{
	_usart->CFG &= ~1;	// Disable transmit and receive
	_usart->INTENCLR = 1;

	HW::GPIO->CLR0 = _maskRTS;

	HW::DMA->ENABLECLR0 = _rxDmaMask;
	HW::DMA->ABORT0 = _rxDmaMask;
	HW::DMA->ERRINT0 = _rxDmaMask;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__irq void ComPort::ReadHandler_0()
//{
//	ComBase &cb = _bases[0];
//	
//	*(cb.rdata++) = cb.usart->RXDATA;
//	cb.rlen--;
//	
//	if (cb.rlen == 0)
//	{
//		cb.usart->INTENCLR = 1;
//	};
//
//	HW::MRT->Channel[0].INTVAL = cb.tm;
//}
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//__irq void ComPort::ReadHandler_1()
//{
//	ComBase &cb = _bases[1];
//	
//	*(cb.rdata++) = cb.usart->RXDATA;
//	cb.rlen--;
//	
//	if (cb.rlen == 0)
//	{
//		cb.usart->INTENCLR = 1;
//	};
//
//	HW::MRT->Channel[1].INTVAL = cb.tm;
//}
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//__irq void ComPort::ReadHandler_2()
//{
//	ComBase &cb = _bases[2];
//	
//	*(cb.rdata++) = cb.usart->RXDATA;
//	cb.rlen--;
//	
//	if (cb.rlen == 0)
//	{
//		cb.usart->INTENCLR = 1;
//	};
//
//	HW::MRT->Channel[2].INTVAL = cb.tm;
//}
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//__irq void ComPort::WriteHandler_0()
//{
//	ComBase &cb = _bases[0];
//	
//	cb.usart->TXDATA = *(cb.wdata++);
//	cb.wlen--;
//	
//	if (cb.wlen == 0)
//	{
//		cb.usart->INTENCLR = 4;
//	};
//}
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//__irq void ComPort::WriteHandler_1()
//{
//	ComBase &cb = _bases[1];
//	
//	cb.usart->TXDATA = *(cb.wdata++);
//	cb.wlen--;
//	
//	if (cb.wlen == 0)
//	{
//		cb.usart->INTENCLR = 4;
//	};
//}
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//__irq void ComPort::WriteHandler_2()
//{
//	ComBase &cb = _bases[2];
//	
//	cb.usart->TXDATA = *(cb.wdata++);
//	cb.wlen--;
//	
//	if (cb.wlen == 0)
//	{
//		cb.usart->INTENCLR = 4;
//	};
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Update()
{
	bool r = true;

	if (!_connected) 
	{
		_status485 = READ_END;
	};

//	ComBase &cb = _bases[_portNum];

	register u16 t;

	switch (_status485)
	{
		case WRITEING:

			if ((_usart->STAT & 8) && (HW::DMA->ACTIVE0 & _txDmaMask) == 0)
			{
				_pWriteBuffer->transmited = true;
				_status485 = READ_END;

				DisableTransmit();

				r = false;
			};

			break;

		case WAIT_READ:

			t = (HW::DMA->CH[_rxDmaCh].XFERCFG >> 16) & 0x3FF;

			if ((HW::MRT->Channel[_portNum].STAT & 1) || (HW::DMA->ACTIVE0 & _rxDmaMask) == 0)
			{
				DisableReceive();
				
				HW::MRT->Channel[_portNum].INTVAL = (1UL<<31);
				HW::MRT->Channel[_portNum].STAT = 1;

				_pReadBuffer->len = _pReadBuffer->maxLen - ((t + 1)&0x3FF);
				_pReadBuffer->recieved = _pReadBuffer->len > 0;
				_status485 = READ_END;
				r = false;
			}
			else if (_prevDmaCounter != t)
			{
				HW::MRT->Channel[_portNum].INTVAL = _postReadTimeout;
				_prevDmaCounter = t;
			};

			break;

		case READ_END:

			r = false;

			break;
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Read(ComPort::ReadBuffer *readBuffer, dword preTimeout, dword postTimeout)
{
	if (_status485 != READ_END || readBuffer == 0 || readBuffer->data == 0 || readBuffer->maxLen == 0)
	{
//		cputs("ComPort::Read falure\n\r");	
		return false;
	};


	_pReadBuffer = readBuffer;
	_pReadBuffer->recieved = false;
	_pReadBuffer->len = 0;

	_preReadTimeout = (1UL<<31)|preTimeout;
	_postReadTimeout = (1UL<<31)|postTimeout;

	HW::MRT->Channel[_portNum].CTRL = 2;
	HW::MRT->Channel[_portNum].INTVAL = _preReadTimeout; 
	HW::MRT->Channel[_portNum].STAT = 1;

	EnableReceive(_pReadBuffer->data, _pReadBuffer->maxLen);

//	cputs("ComPort::Read start\n\r");	

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Write(ComPort::WriteBuffer *writeBuffer)
{
	if (_status485 != READ_END || writeBuffer == 0 || writeBuffer->data == 0 || writeBuffer->len == 0)
	{
		return false;
	};

	_pWriteBuffer = writeBuffer;
	_pWriteBuffer->transmited = false;

	EnableTransmit(_pWriteBuffer->data, _pWriteBuffer->len);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::TransmitByte(byte v)
{
	HW::GPIO->SET0 = _maskRTS;

	_usart->CFG |= 1;
	_usart->TXDATA = v;

	while((_usart->STAT & 8) == 0);

	DisableTransmit();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
