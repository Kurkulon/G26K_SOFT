#ifdef SPI_H__21_05_2009__14_23
#include "SPI.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SPI spi;

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
	using namespace HW;

	u32 register tmp;

	if (_current == 0)
	{
		if ((_current = GetRequest()) != 0)
		{
			PMC->PCER0 = HW::PID::SPI0_M;
			SPI0->CR = 2;

			SPI0->MR = 5 | _current->DLYBCS<<24 | _current->PCS<<16;
			SPI0->CSR[(_current->PCS>>2)&3] = _current->CSR;

			tmp = DMAC->EBCISR;

			DMAC->CH[0].SADDR = _current->txp;
			DMAC->CH[0].DADDR = &(SPI0->TDR);
			DMAC->CH[0].DSCR = 0x0;
			DMAC->CH[0].CTRLA = 0x00000000 | _current->count;
			DMAC->CH[0].CTRLB = 0x20200000;
			DMAC->CH[0].CFG = 0x00002010;

			DMAC->CH[1].SADDR = &(SPI0->RDR);
			DMAC->CH[1].DADDR = _current->rxp;
			DMAC->CH[1].DSCR = 0x0;
			DMAC->CH[1].CTRLA = 0x00000000 | _current->count;
			DMAC->CH[1].CTRLB = 0x02400000;
			DMAC->CH[1].CFG = 0x00000202;

			DMAC->CHER = (1<<0)|(1<<1);
			DMAC->EN = 1;
			SPI0->CR = 1;


			//SPI0->PDC.RPR = _current->rxp;
			//SPI0->PDC.RCR = SPI0->PDC.TCR = _current->count;
			//SPI0->PDC.TPR = _current->txp;
			//SPI0->PDC.PTCR = 0x101;
		}
		else
		{
			SPI0->CR = 2;
		};
	}
	else
	{
		if ((SPI0->SR & 0x202) == 0x202)
		{
			DMAC->CHDR = (1<<0)|(1<<1);
			SPI0->CR = 2;

			_current->ready = true;
			_current->error = ((u16)(DMAC->CH[0].CTRLA) != 0) || ((u16)(DMAC->CH[1].CTRLA) != 0);

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
#endif
