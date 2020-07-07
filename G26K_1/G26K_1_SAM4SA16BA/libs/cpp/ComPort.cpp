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

#pragma O3
#pragma Otime

extern dword millisecondsCount;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

ComPort::ComBase	ComPort::_bases[4] = { 
	{false, HW::UART0,	 HW::PIOA, 1U<<31 }, 
	{false, HW::UART1,	 HW::PIOB, 1<<13 },
	{false, HW::USART0,	 HW::PIOA, 1<<4	 }, 
	{false, HW::USART1,	 HW::PIOA, 0	 }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Connect(byte port, dword speed, byte parity)
{
#ifndef WIN32

	if (_connected || port > 3 || _bases[port].used)
	{
		return false;
	};

	_portNum = port;

	ComBase &cb = _bases[port];

	_SU = cb.HU.SU;
	_pm = cb.pm;
	_maskRTS = cb.maskRTS;

	_BaudRateRegister = BoudToPresc(speed);

	_ModeRegister = 0xC0;

	switch (parity)
	{
		case 0:		// нет четности
			_ModeRegister |= 0x800;
			break;

		case 1:
			_ModeRegister |= 0x200;
			break;

		case 2:
			_ModeRegister |= 0x000;
			break;
	};

	_SU->CR = 3;
	_SU->MR = _ModeRegister;
	_SU->BRGR = _BaudRateRegister;

	_status485 = READ_END;

	return _connected = cb.used = true;

#else

    char buf[256];

    wsprintf(buf, "COM%u", port);

    return Connect(buf, speed, parity);

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Disconnect()
{
	if (!_connected) return false;

	DisableReceive();
	DisableTransmit();

	_status485 = READ_END;

#ifndef WIN32
	_connected = _bases[_portNum].used = false;
#else
	_connected = false;
#endif

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

word ComPort::BoudToPresc(dword speed)
{
	if (speed == 0) return 0;

	word presc;

	presc = (word)((MCK/16 + speed/2) / speed);

	return presc;
}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::TransmitByte(byte v)
{
#ifndef WIN32

	_SU->CR = 0xA0;	// Disable transmit and receive

	_pm->SODR = _maskRTS;

	_SU->CR = 0x40;

	_SU->THR = v;

	while ((_SU->CSR & 0x200) == 0);

	_pm->CODR = _maskRTS;

	_SU->CR = 0xA0;	// Disable transmit and receive

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::EnableTransmit(void* src, word count)
{
#ifndef WIN32

	_SU->CR = 0xA0;	// Disable transmit and receive

	_pm->SODR = _maskRTS;

	_SU->PDC.TPR = src;
	_SU->PDC.TCR = count;

	_SU->PDC.PTCR = 0x100;

	_startTransmitTime = GetRTT();
	_SU->CR = 0x40;

#else

	_ovlWrite.hEvent = 0;
	_ovlWrite.Internal = 0;
	_ovlWrite.InternalHigh = 0;
	_ovlWrite.Offset = 0;
	_ovlWrite.OffsetHigh = 0;
	_ovlWrite.Pointer = 0;

	WriteFile(_comHandle, src, count, &_writeBytes, &_ovlWrite);
    SetCommMask(_comHandle, EV_TXEMPTY);

#endif

	_status485 = WRITEING;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::DisableTransmit()
{
#ifndef WIN32
	
	_SU->CR = 0x80;
	_SU->PDC.PTCR = 0x200;
	_pm->CODR = _maskRTS;

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::EnableReceive(void* dst, word count)
{
#ifndef WIN32

	_SU->CR = 0x1A0;	// Disable transmit and receive, reset status

	_pm->CODR = _maskRTS;

	_SU->PDC.RPR = dst;
	_SU->PDC.RCR = count;

	_SU->PDC.PTCR = 1;

	_startReceiveTime = GetRTT();
	_SU->CR = 0x110;

#else

	_ovlRead.hEvent = 0;
	_ovlRead.Internal = 0;
	_ovlRead.InternalHigh = 0;
	_ovlRead.Offset = 0;
	_ovlRead.OffsetHigh = 0;
	_ovlRead.Pointer = 0;

	ReadFile(_comHandle, dst, count, &_readBytes, &_ovlRead);

#endif

	_status485 = WAIT_READ;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::DisableReceive()
{
#ifndef WIN32

	_SU->CR = 0x120;
	_SU->PDC.PTCR = 2;
	_pm->CODR = _maskRTS;

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Update()
{
	static u32 stamp = 0;

	bool r = true;
//	byte t;

	if (!_connected) 
	{
		_status485 = READ_END;
	};

	stamp = GetRTT();

	switch (_status485)
	{
		case WRITEING:

#ifndef WIN32
			if ((_SU->CSR&0x210) == 0x210)
#else
			if (HasOverlappedIoCompleted(&_ovlWrite))
#endif
			{
				_pWriteBuffer->transmited = true;
				_status485 = READ_END;

				DisableTransmit();
				DisableReceive();

				r = false;
			};

			break;

		case WAIT_READ:

#ifndef WIN32

			if ((_prevDmaCounter-_SU->PDC.RCR) == 0)
			{
				if ((stamp - _startReceiveTime) >= _preReadTimeout)
				{
					DisableReceive();
					_pReadBuffer->pretimeout = true;
					_pReadBuffer->len = _pReadBuffer->maxLen - _prevDmaCounter;
					_pReadBuffer->recieved = _pReadBuffer->len > 0;
					_status485 = READ_END;
					r = false;
				};
			}
			else
			{
				_prevDmaCounter = _SU->PDC.RCR;
				_startReceiveTime = stamp;
				_status485 = READING;
			};

#else
			if (HasOverlappedIoCompleted(&_ovlRead))
			{
				bool c = GetOverlappedResult(_comHandle, &_ovlRead, &_readBytes, false);
				_pReadBuffer->len = _readBytes;
				_pReadBuffer->recieved = _pReadBuffer->len > 0 && c;
				_status485 = READ_END;
				r = false;
			};
#endif

			break;

#ifndef WIN32

		case READING:

			//t = (_SU->CSR & 0xE4);

			//if (t) 
			//{
			//	_pReadBuffer->CSR = t;
			//	DisableReceive();
			//	_status485 = READ_END;
			//	return false;
			//};

			if ((_prevDmaCounter-_SU->PDC.RCR) == 0)
			{
				if ((stamp - _startReceiveTime) >= _postReadTimeout)
				{
					DisableReceive();
					_pReadBuffer->len = _pReadBuffer->maxLen - _prevDmaCounter;
					_pReadBuffer->recieved = _pReadBuffer->len > 0;
					_pReadBuffer->CSR = _SU->CSR & 0xE4;
					_status485 = READ_END;
					r = false;
				};
			}
			else
			{
				_prevDmaCounter = _SU->PDC.RCR;
				_startReceiveTime = stamp;
			};

			break;

#endif

		case READ_END:

			r = false;

			break;
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Read(ComPort::ReadBuffer *readBuffer, dword preTimeout, dword postTimeout)
{
	if (_status485 != READ_END || readBuffer == 0)
	{
//		cputs("ComPort::Read falure\n\r");	
		return false;
	};

#ifndef WIN32

	_preReadTimeout = preTimeout;
	_postReadTimeout = postTimeout;

#else
    _cto.ReadIntervalTimeout = postTimeout/32+1;
	_cto.ReadTotalTimeoutConstant = preTimeout/32+1;

    if (!SetCommTimeouts(_comHandle, &_cto))
    {
		return false;
	};

#endif

	_pReadBuffer = readBuffer;
	_pReadBuffer->recieved = false;
	_pReadBuffer->pretimeout = false;
	_pReadBuffer->len = 0;

	_prevDmaCounter = _pReadBuffer->maxLen;

	u32 t = (u32)_pReadBuffer->data;

	if (t < 0x20000000 || t >= 0x20028000)
	{
		__breakpoint(0);
	};

	EnableReceive(_pReadBuffer->data, _pReadBuffer->maxLen);

//	cputs("ComPort::Read start\n\r");	

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Write(ComPort::WriteBuffer *writeBuffer)
{
	if (_status485 != READ_END || writeBuffer == 0)
	{
		return false;
	};

	_pWriteBuffer = writeBuffer;
	_pWriteBuffer->transmited = false;

	EnableTransmit(_pWriteBuffer->data, _pWriteBuffer->len);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int  ComPort::_portTableSize = 0;
dword ComPort::_portTable[16];

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Connect(const char* comPort, dword bps, byte parity)
{
    if (_comHandle != INVALID_HANDLE_VALUE)
    {
        return false;
    };

	_comHandle = CreateFile(comPort, GENERIC_READ|GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);

    if (_comHandle == INVALID_HANDLE_VALUE)
    {
		return false;
    };

    DCB dcb;

    if (!GetCommState(_comHandle, &dcb))
    {
        Disconnect();
		return false;
    };

    dcb.DCBlength = sizeof(dcb);
    dcb.BaudRate = bps;
    dcb.ByteSize = 8;
    dcb.Parity = parity;
    dcb.fParity = (parity > 0);
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = true;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutxCtsFlow = false;
	dcb.fOutxDsrFlow = false;
    dcb.fDsrSensitivity = false;

	if (!SetCommState(_comHandle, &dcb))
    {
        Disconnect();
		return false;
    };

    _cto.ReadIntervalTimeout = 2;
	_cto.ReadTotalTimeoutMultiplier = 0;
	_cto.ReadTotalTimeoutConstant = -1;
	_cto.WriteTotalTimeoutConstant = 0;
	_cto.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(_comHandle, &_cto))
    {
        Disconnect();
    	return false;
    };

	printf("Connect to %s speed = %u\r\n", comPort, bps);

	return _connected = true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::BuildPortTable()
{
    if (_portTableSize != 0)
    {
        return;
    };

    char buf[256];

    COMMCONFIG cc;
    DWORD lcc = sizeof(cc);

    int k = 0;

    for (int i = 1; i < 17; i++)
    {
        wsprintf(buf, "COM%i", i);

        lcc = sizeof(cc);

        if (GetDefaultCommConfig(buf, &cc, &lcc))
        {
            _portTable[k++] = i;
        };

    };

    _portTableSize = k;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif
