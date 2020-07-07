#ifndef SPI_H__21_05_2009__14_23
#define SPI_H__21_05_2009__14_23

#pragma once

#include "types.h"
#include "core.h"

class SPI
{
public:

	struct Buffer
	{
		// byte align
		bool ready;
		bool error;
		byte DLYBCS;
		byte PCS;

		// word align

		word count; 

		//dword align

		void *txp; void *rxp; 
		void (*pCallBack)(Buffer*);
		Buffer *next; 
		dword CSR;

		T_HW::S_PIO *pio;
		u32 mask;
	};

protected:

	static Buffer *_request;
	static Buffer *_current;

	Buffer* GetRequest();

public:

	//SPI();
	//~SPI();

	void Update();
	void AddRequest(Buffer *req);


};

extern SPI spi;

#endif // SPI_H__21_05_2009__14_23
