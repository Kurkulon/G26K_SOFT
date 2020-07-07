#pragma O3
#pragma Otime

#include "twi.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TWI::Init(byte num)
{
	using namespace HW;

	num &= 1;

	hw = TWI1;

	VectorTableExt[PID::TWI1_I] = Handler0;
	CM4::NVIC->ICPR[0] = PID::TWI1_M;
	CM4::NVIC->ISER[0] = PID::TWI1_M;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TWI::Write(DSCTWI *d)
{
	if (dsc != 0 || d == 0) { return false; };
	if (d->data == 0 || d->len == 0) { return false; }

//	smask = 1<<13;
	dsc = d;

	__enable_irq();

	dsc->ready = false;
//	hw->CR = 0x80;
	hw->CR = 0x24;
	hw->MMR = dsc->MMR & ~0x1000;
	hw->IADR = dsc->IADR;
	hw->CWGR = dsc->CWGR;
	hw->PDC.TPR = dsc->data;
	hw->PDC.TCR = dsc->len;
	hw->IER = 1<<13;
	hw->PDC.PTCR = 0x100;


	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TWI::Read(DSCTWI *d)
{
	if (dsc != 0 || d == 0) { return false; };
	if (d->data == 0 || d->len == 0) { return false; }

//	smask = 1<<12;
	dsc = d;

	__enable_irq();

	dsc->ready = false;
	hw->CR = 0x80;
	hw->MMR = dsc->MMR|0x1000;
	hw->IADR = dsc->IADR;
	hw->CWGR = dsc->CWGR;
	hw->CR = 0x24;

	hw->PDC.RPR = dsc->data;
	
	if (dsc->len > 1)
	{
		hw->PDC.RCR = dsc->len-1;
		hw->IER = 1<<12;
		hw->PDC.PTCR = 0x201;
		hw->CR = 0x1;
	}
	else
	{
		hw->IER = 2;
		hw->CR = 0x3;
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TWI::Update()
{
	if (dsc == 0)
	{ 
		return false; 
	}
	else if ((hw->SR & 1) != 0)
	{
		hw->PDC.PTCR = 0x202;
		dsc->ready = true;
		dsc->rlen = (hw->MMR & 0x1000) ? ((byte*)hw->PDC.RPR - (byte*)dsc->data) : 0;
		
		dsc = 0;

		return false;
	}
	else
	{
		return true;
	};

}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__irq void TWI::Handler0()
{
	using namespace HW;

//	__breakpoint(0);

	u32 a = TWI1->SR & TWI1->IMR;

	if(a & 0x100) // NACK
	{
		TWI1->PDC.PTCR = 0x202;;
		TWI1->IDR = 3<<12;
	}
	else if(a & (1<<13)) // ENDTX
	{
		TWI1->PDC.PTCR = 0x200;
		TWI1->IDR = 1<<13;

		TWI1->CR = 2;
	}
	else if(a & (1<<12)) // ENDRX
	{
		TWI1->PDC.PTCR = 2;
		TWI1->IDR = 1<<12;
		TWI1->CR = 2;
		TWI1->IER = 2;
	}
	else if(a & 2) // RXRDY
	{
		TWI1->IDR = 2;

		byte **p = (byte**)&TWI1->PDC.RPR;

		*((*p)++) = TWI1->RHR;
		//p[0]++;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
