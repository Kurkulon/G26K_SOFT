#ifndef TWI_H__24_04_13__11_25
#define TWI_H__24_04_13__11_25

#include "types.h"
#include "core.h"


struct DSCTWI
{
	u32		MMR;
	u32		IADR;
	u32		CWGR;
	void*	data;
	u16		len;
	u16		rlen; // recieved len
	bool	ready;
};

struct TWI
{
//	u32		smask;
	DSCTWI* dsc;

	T_HW::S_TWI *hw;

	TWI() : hw(0), dsc(0) {}

	bool Init(byte num);

	bool Write(DSCTWI *d);
	bool Read(DSCTWI *d);
	bool Update();

	static __irq void Handler0();



};

#endif // TWI_H__24_04_13__11_25
