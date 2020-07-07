#pragma O3
#pragma Otime

#include "SPI.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//SPI spi;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SPI::Buffer *SPI::_request = 0;
SPI::Buffer *SPI::_current = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*SPI::SPI()
{

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SPI::~SPI()
{

}*/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SPI::Update()
{
	if (_current == 0)
	{
		if ((_current = GetRequest()) != 0)
		{
			HW::PMC->PCER0 = HW::PID::SPI_M;
			HW::SPI->CR = 1;

			HW::SPI->MR = 0x11 | _current->DLYBCS<<24 | _current->PCS<<16;
			HW::SPI->CSR[(_current->PCS>>2)&3] = _current->CSR;

			_current->pio->CODR = _current->mask;

			HW::SPI->PDC.RPR = _current->rxp;
			HW::SPI->PDC.RCR = HW::SPI->PDC.TCR = _current->count;
			HW::SPI->PDC.TPR = _current->txp;
			HW::SPI->PDC.PTCR = 0x101;
		};
	}
	else
	{
		if ((HW::SPI->SR & 0x2A2) == 0x2A2)
		{
			HW::SPI->PDC.PTCR = 0x202;

			_current->pio->SODR = _current->mask;

			_current->ready = true;
			_current->error = HW::SPI->PDC.RCR != 0;

			if (_current->pCallBack != 0) 
			{
				_current->pCallBack(_current); 
			};

			_current = 0;
		};
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SPI::AddRequest(Buffer *req)
{
	if (req == 0) return;

	if (_request == 0)
	{
		_request = req;
	}
	else
	{
		_request->next = req;
	};

	req->next = 0;
	req->ready = false;
	req->error = false;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SPI::Buffer* SPI::GetRequest()
{
	if (_request == 0) return 0;

	Buffer* req = _request;

	_request = req->next; 

	req->next = 0;

	return req;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//void SPI::AddRequest(Buffer *req)
//{
//	if (req == 0) return;
//
//	if (_last == 0)
//	{
//		_last = _first = req;
//	}
//	else
//	{
//		_last->next = req;
//		_last = req;
//	};
//
//	_last->next = 0;
//
//	req->ready = false;
//	req->error = false;
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//SPI::Buffer* SPI::GetRequest()
//{
//	if (_first == 0) return 0;
//
//	Buffer* req = _first;
//
//	if (_first->next == 0)
//	{
//		_first = _last = 0;
//	}
//	else
//	{
//		_first = _first->next;
//	};
//
//	req->next = 0;
//
//	return req;
//}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
