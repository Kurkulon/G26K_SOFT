#include <stdio.h>
//#include <conio.h>

#include "hardware.h"
#include "ComPort.h"
#include "time.h"
//#include "hardware.h"

#ifdef _DEBUG_
//	static const bool _debug = true;
#else
//	static const bool _debug = false;
#endif

extern dword millisecondsCount;

#define MASKRTS (1<<10)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Connect(dword speed, byte parity)
{
#ifndef WIN32

	if (_connected)
	{
		return false;
	};

	_BaudRateRegister = BoudToPresc(speed);

	_ModeRegister = 3;

	switch (parity)
	{
		case 0:		// нет четности
			_ModeRegister |= 0;
			break;

		case 1:
			_ModeRegister |= 0x08;
			break;

		case 2:
			_ModeRegister |= 0x18;
			break;
	};

	*pUART0_GCTL = 1;
	*pUART0_LCR = _ModeRegister;
	SetBoudRate(_BaudRateRegister);

	_status485 = READ_END;

	return _connected = true;

#else

    char buf[256];

    wsprintf(buf, "COM%u", port+1);

    return Connect(buf, speed, parity);

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Disconnect()
{
	if (!_connected) return false;

	DisableReceive();
	DisableTransmit();

	*pUART0_GCTL = 0;

	_status485 = READ_END;

#ifndef WIN32
	_connected = false;
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

	presc = (word)((SCLK + (speed<<3)) / (speed << 4));

	return presc;
}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

void ComPort::SetBoudRate(word presc)
{
	union { word w; byte b[2]; };

	w = presc;

	*pUART0_LCR |= 128;
	*pUART0_DLL = b[0];
	*pUART0_DLH = b[1];
	*pUART0_LCR &= ~128;
}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::EnableTransmit(void* src, word count)
{
#ifndef WIN32

	*pPORTFIO_SET = MASKRTS;
	*pDMA8_CONFIG = 0;	// Disable transmit and receive
	*pUART0_IER = 0;

	*pDMA8_START_ADDR = src;
	*pDMA8_X_COUNT = count;
	*pDMA8_X_MODIFY = 1;
	*pDMA8_CONFIG = FLOW_STOP|WDSIZE_8|SYNC|DMAEN;

	_startTransmitTime = GetRTT();

	*pUART0_IER = ETBEI;



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
	
	*pDMA8_CONFIG = 0;	// Disable transmit and receive
	*pUART0_IER = 0;
	*pPORTFIO_CLEAR = MASKRTS;

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ComPort::EnableReceive(void* dst, word count)
{
#ifndef WIN32

	*pPORTFIO_CLEAR = MASKRTS;
	*pDMA7_CONFIG = 0;	// Disable transmit and receive
	*pUART0_IER = 0;

	*pDMA7_START_ADDR = dst;
	*pDMA7_CURR_X_COUNT = *pDMA7_X_COUNT = count;
	*pDMA7_X_MODIFY = 1;
	*pDMA7_CONFIG = WNR|FLOW_STOP|WDSIZE_8|SYNC|DMAEN;

	_startReceiveTime = GetRTT();

	count = *pUART0_RBR;
	count = *pUART0_LSR;
	*pUART0_IER = ERBFI;

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

	*pDMA7_CONFIG = 0;	// Disable transmit and receive
	*pUART0_IER = 0;
	*pPORTFIO_CLEAR = MASKRTS;

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ComPort::Update()
{
	static u32 stamp = 0;

	bool r = true;

	if (!_connected) 
	{
		_status485 = READ_END;
	};

	stamp = GetRTT();

	switch (_status485)
	{
		case WRITEING:

#ifndef WIN32
			if (*pUART0_LSR & TEMT)
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

			if ((_prevDmaCounter-*pDMA7_CURR_X_COUNT) == 0)
			{
				if ((stamp - _startReceiveTime) >= _preReadTimeout)
				{
					DisableReceive();
					_pReadBuffer->len = _pReadBuffer->maxLen - _prevDmaCounter;
					_pReadBuffer->recieved = _pReadBuffer->len > 0;
					_status485 = READ_END;
					r = false;
				};
			}
			else
			{
				_prevDmaCounter = *pDMA7_CURR_X_COUNT;
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

			//if (*pUART0_LSR & (BI|OE|PE|FE)) 
			//{
			//	DisableReceive();
			//	_status485 = READ_END;
			//	return false;
			//};

			if ((_prevDmaCounter-*pDMA7_CURR_X_COUNT) == 0)
			{
				if ((stamp - _startReceiveTime) >= _postReadTimeout)
				{
					DisableReceive();
					_pReadBuffer->len = _pReadBuffer->maxLen - _prevDmaCounter;
					_pReadBuffer->recieved = _pReadBuffer->len > 0;
					_status485 = READ_END;

					r = false;
				};
			}
			else
			{
				_prevDmaCounter = *pDMA7_CURR_X_COUNT;
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
    _cto.ReadIntervalTimeout = postTimeout/32;
	_cto.ReadTotalTimeoutConstant = preTimeout/32;

    if (!SetCommTimeouts(_comHandle, &_cto))
    {
		return false;
	};

#endif

	_pReadBuffer = readBuffer;
	_pReadBuffer->recieved = false;
	_pReadBuffer->len = 0;

	_prevDmaCounter = _pReadBuffer->maxLen;

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

