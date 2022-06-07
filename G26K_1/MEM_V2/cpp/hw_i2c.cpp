#include "types.h"
#include "core.h"
#include "time.h"
#include "COM_DEF.h"
#include "CRC16_8005.h"
#include "list.h"
#include "PointerCRC.h"
#include "DMA.h"

#include "hardware.h"
#include "SEGGER_RTT.h"
#include "hw_conf.h"
#include "hw_rtm.h"


#ifdef WIN32

#include <windows.h>
#include <Share.h>
#include <conio.h>
#include <stdarg.h>
#include <stdio.h>
#include <intrin.h>
#include "CRC16_CCIT.h"
#include "list.h"

static HANDLE handleNandFile;
static const char nameNandFile[] = "NAND_FLASH_STORE.BIN";

static HANDLE handleWriteThread;
static HANDLE handleReadThread;

static byte nandChipSelected = 0;

static u64 curNandFilePos = 0;
//static u64 curNandFileBlockPos = 0;
static u32 curBlock = 0;
static u32 curRawBlock = 0;
static u16 curPage = 0;
static u16 curCol = 0;

static OVERLAPPED	_overlapped;
static u32			_ovlReadedBytes = 0;
static u32			_ovlWritenBytes = 0;

static void* nandEraseFillArray;
static u32 nandEraseFillArraySize = 0;
static byte nandReadStatus = 0x41;
static u32 lastError = 0;


static byte fram_I2c_Mem[0x10000];
static byte fram_SPI_Mem[0x40000];

static bool fram_spi_WREN = false;

static u16 crc_ccit_result = 0;



static volatile bool busyWriteThread = false;

#elif defined(CPU_SAME53)	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

List<DSCI2C>	i2c_ReqList;
DSCI2C*			i2c_dsc = 0;

#elif defined(CPU_XMC48)	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte *twi_wrPtr = 0;
static byte *twi_rdPtr = 0;
static u16 twi_wrCount = 0;
static u16 twi_rdCount = 0;
static byte *twi_wrPtr2 = 0;
static u16 twi_wrCount2 = 0;
static byte twi_adr = 0;
static DSCI2C* twi_dsc = 0;
static DSCI2C* twi_lastDsc = 0;

#endif 



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_XMC48

static __irq void I2C_Handler()
{
	using namespace HW;

//	HW::P6->BSET(2);

	u32 a = I2C->PSR_IICMode;

	if(a & ACK)
	{
		if (twi_wrCount > 0)
		{
			I2C->TBUF[0] = TDF_MASTER_SEND | *twi_wrPtr++;

			twi_wrCount--;

			twi_dsc->ack = true;

			if(twi_wrCount == 0 && twi_wrCount2 != 0)
			{
				twi_wrPtr = twi_wrPtr2;
				twi_wrCount = twi_wrCount2;
				twi_wrCount2 = 0;
			};
		}
		else if (twi_rdCount > 0)
		{
			if(a & (SCR|RSCR))
			{
				I2C->TBUF[0] = TDF_MASTER_RECEIVE_ACK; 
			}
			else
			{
				I2C->TBUF[0] = TDF_MASTER_RESTART | (twi_adr << 1) | 1;
			};
		}
		else
		{
			I2C->TBUF[0] = TDF_MASTER_STOP;
		};
	}
	else if (a & (RIF|AIF))
	{
		byte t = I2C->RBUF;

		if (twi_rdCount > 0)
		{
			*twi_rdPtr++ = t; // receive data
			twi_rdCount--;
		};
			
		I2C->TBUF[0] = (twi_rdCount > 0) ? TDF_MASTER_RECEIVE_ACK : TDF_MASTER_RECEIVE_NACK; 
	}
	else if ((a & PCR) == 0)
	{
		I2C->TBUF[0] = TDF_MASTER_STOP; 
	}
	else
	{
		twi_dsc->ready = true;
		twi_dsc->readedLen = twi_dsc->rlen - twi_rdCount;

//		state = 0;
		
		DSCI2C *ndsc = twi_dsc->next;

		if (ndsc != 0)
		{
			twi_dsc->next = 0;
			twi_dsc = ndsc;

			twi_dsc->ready = false;
			twi_dsc->ack = false;
			twi_dsc->readedLen = 0;

			twi_wrPtr = (byte*)twi_dsc->wdata;	
			twi_rdPtr = (byte*)twi_dsc->rdata;	
			twi_wrPtr2 = (byte*)twi_dsc->wdata2;	
			twi_wrCount = twi_dsc->wlen;
			twi_wrCount2 = twi_dsc->wlen2;
			twi_rdCount = twi_dsc->rlen;
			twi_adr = twi_dsc->adr;

			if (twi_wrPtr2 == 0) twi_wrCount2 = 0;

			//I2C->CCR |= RIEN|AIEN;
			//I2C->PCR_IICMode |= PCRIEN|NACKIEN|ARLIEN|SRRIEN|ERRIEN|ACKIEN;

			I2C->PSCR = ~0;

			I2C->TBUF[0] = TDF_MASTER_START | (twi_dsc->adr << 1) | ((twi_wrCount == 0) ? 1 : 0);
		}
		else
		{
			I2C->CCR = I2C__CCR;
			I2C->PCR_IICMode = I2C__PCR;

			twi_lastDsc = twi_dsc = 0;
		};

//		I2C->PSCR = PCR|NACK;
	};

	I2C->PSCR = a;

//	HW::P6->BCLR(2);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_XMC48

bool I2C_Write(DSCI2C *d)
{
	using namespace HW;

	if (twi_dsc != 0 || d == 0) { return false; };
	if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

	twi_dsc = d;

	twi_dsc->ready = false;
	twi_dsc->ack = false;
	twi_dsc->readedLen = 0;

	twi_wrPtr = (byte*)twi_dsc->wdata;	
	twi_rdPtr = (byte*)twi_dsc->rdata;	
	twi_wrPtr2 = (byte*)twi_dsc->wdata2;	
	twi_wrCount = twi_dsc->wlen;
	twi_wrCount2 = twi_dsc->wlen2;
	twi_rdCount = twi_dsc->rlen;
	twi_adr = twi_dsc->adr;

	if (twi_wrPtr2 == 0) twi_wrCount2 = 0;

	__disable_irq();

	I2C->PSCR = ~0;//RIF|AIF|TBIF|ACK|NACK|PCR;

	I2C->TBUF[0] = TDF_MASTER_START | (twi_dsc->adr << 1) | ((twi_wrCount == 0) ? 1 : 0);

	I2C->CCR |= RIEN|AIEN;
	I2C->PCR_IICMode |= PCRIEN|NACKIEN|ARLIEN|SRRIEN|ERRIEN|ACKIEN;

	__enable_irq();

	return true;
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_Update()
{
	bool result = false;

#ifdef CPU_SAME53	

	enum STATE { WAIT = 0, WRITE, READ, STOP };

	static STATE state = WAIT;
	static byte *wrPtr = 0;
	static byte *rdPtr = 0;
	static u16 	wrCount = 0;
	static u16 	rdCount = 0;
	static byte *wrPtr2 = 0;
	static u16	wrCount2 = 0;
	static byte adr = 0;

	switch (state)
	{
		case WAIT:

			i2c_dsc = i2c_ReqList.Get();

			if (i2c_dsc != 0)
			{
				DSCI2C &dsc = *i2c_dsc;

				dsc.ready = false;
				dsc.ack = false;
				dsc.readedLen = 0;

				wrPtr = (byte*)dsc.wdata;	
				rdPtr = (byte*)dsc.rdata;	
				wrPtr2 = (byte*)dsc.wdata2;	
				wrCount = dsc.wlen;
				wrCount2 = dsc.wlen2;
				rdCount = dsc.rlen;
				adr = dsc.adr;

				if (wrPtr2 == 0) wrCount2 = 0;

				I2C->CTRLB = I2C_SMEN;
				I2C->STATUS.BUSSTATE = BUSSTATE_IDLE;

				I2C->INTFLAG = ~0;

				//T_HW::DMADESC &dmadsc = DmaTable[I2C_DMACH];

				if (wrCount == 0)
				{
					I2C_DMA.ReadPeripheral(&I2C->DATA, rdPtr, rdCount, DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM3_RX, DMDSC_BEATSIZE_BYTE);

					//dmadsc.SRCADDR	= &I2C->DATA;
					//dmadsc.DSTADDR	= rdPtr + rdCount;
					//dmadsc.DESCADDR = 0;
					//dmadsc.BTCNT	= rdCount;
					//dmadsc.BTCTRL	= DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_DSTINC;

					//HW::DMAC->CH[I2C_DMACH].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM3_RX;

					I2C->ADDR = ((rdCount <= 255) ? (I2C_LEN(rdCount)|I2C_LENEN) : 0) | (adr << 1) | 1;
					state = READ; 
				}
				else
				{
					I2C_DMA.WritePeripheral(wrPtr, &I2C->DATA, wrCount, wrPtr2, wrCount2, DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM3_TX, DMDSC_BEATSIZE_BYTE);

					//dmadsc.SRCADDR	= wrPtr + wrCount;
					//dmadsc.DSTADDR	= &I2C->DATA;
					//dmadsc.BTCNT	= wrCount;
					//dmadsc.BTCTRL	= DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_SRCINC;

					//if (wrCount2 == 0)
					//{
					//	dmadsc.DESCADDR = 0;
					//}
					//else
					//{
					//	wr_dmadsc.SRCADDR	= wrPtr2 + wrCount2;
					//	wr_dmadsc.DSTADDR	= &I2C->DATA;
					//	wr_dmadsc.BTCNT		= wrCount2;
					//	wr_dmadsc.BTCTRL	= DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_SRCINC;
					//	dmadsc.DESCADDR		= &wr_dmadsc;
					//};

					//HW::DMAC->CH[I2C_DMACH].INTENCLR = ~0;
					//HW::DMAC->CH[I2C_DMACH].INTFLAG = ~0;
					//HW::DMAC->CH[I2C_DMACH].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM3_TX;

					I2C->ADDR = (adr << 1);
					state = WRITE; 
				};
			};

			break;

		case WRITE:

			if((I2C->INTFLAG & I2C_ERROR) || I2C->STATUS.RXNACK)
			{
				I2C->CTRLB = I2C_SMEN|I2C_CMD_STOP;
				
				state = STOP; 
			}
			else
			{
				DSCI2C &dsc = *i2c_dsc;

				__disable_irq();

				bool c = I2C_DMA.CheckComplete() && (I2C->INTFLAG & I2C_MB);
				
				__enable_irq();

				if (c)
				{
					dsc.ack = true;

					if (rdCount > 0)
					{
						I2C_DMA.ReadPeripheral(&I2C->DATA, rdPtr, rdCount, DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM3_RX, DMDSC_BEATSIZE_BYTE);

						//T_HW::DMADESC &dmadsc = DmaTable[I2C_DMACH];

						//dmadsc.SRCADDR	= &I2C->DATA;
						//dmadsc.DSTADDR	= rdPtr + rdCount;
						//dmadsc.DESCADDR = 0;
						//dmadsc.BTCNT	= rdCount;
						//dmadsc.BTCTRL	= DMDSC_VALID|DMDSC_BEATSIZE_BYTE|DMDSC_DSTINC;

						//HW::DMAC->CH[I2C_DMACH].INTENCLR = ~0;
						//HW::DMAC->CH[I2C_DMACH].INTFLAG = ~0;
						//HW::DMAC->CH[I2C_DMACH].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM3_RX;

						I2C->ADDR = ((rdCount <= 255) ? (I2C_LEN(rdCount)|I2C_LENEN) : 0) | (adr << 1) | 1;
		
						state = READ; 
					}
					else
					{
						I2C->CTRLB = I2C_SMEN|I2C_ACKACT|I2C_CMD_STOP;
						
						state = STOP; 
					};
				};
			};

			break;

		case READ:

			if((I2C->INTFLAG & I2C_ERROR) || I2C->STATUS.RXNACK)
			{
				I2C->CTRLB = I2C_SMEN|I2C_ACKACT|I2C_CMD_STOP;
				
				state = STOP; 
			}
			else
			{
				DSCI2C &dsc = *i2c_dsc;

				__disable_irq();

				bool c = I2C_DMA.CheckComplete();
				
				__enable_irq();

				if (c)
				{
					dsc.ack = true;

					I2C->CTRLB = I2C_SMEN|I2C_ACKACT|I2C_CMD_STOP;
						
					state = STOP; 
				};
			};

			i2c_dsc->readedLen = I2C_DMA.GetBytesReady(); //i2c_dsc->rlen - DmaWRB[I2C_DMACH].BTCNT;

			break;

		case STOP:

			if (I2C->STATUS.BUSSTATE == BUSSTATE_IDLE)
			{
				i2c_dsc->ready = true;
				
				i2c_dsc = 0;
				
				I2C->CTRLB = I2C_SMEN;

				state = WAIT; 
			}
			else if (I2C->SYNCBUSY == 0)
			{
				I2C->CTRLB = I2C_SMEN|I2C_ACKACT|I2C_CMD_STOP;
			};

			break;
	};

#elif defined(CPU_XMC48)

	using namespace HW;

	static TM32 tm;

	__disable_irq();

	if (twi_dsc != 0)
	{
		if (I2C->PSR_IICMode & (PCR|NACK|ACK|RIF|AIF))
		{
			tm.Reset();
		}
		else if (tm.Check(10))
		{
			result = true;

			HW::Peripheral_Disable(I2C_PID);

			I2C_Init();

			twi_dsc->ready = true;
			twi_dsc->readedLen = twi_dsc->rlen - twi_rdCount;

			DSCI2C *ndsc = twi_dsc->next;

			if (ndsc != 0)
			{
				twi_dsc->next = 0;
				twi_dsc = ndsc;

				twi_dsc->ready = false;
				twi_dsc->ack = false;
				twi_dsc->readedLen = 0;

				twi_wrPtr = (byte*)twi_dsc->wdata;	
				twi_rdPtr = (byte*)twi_dsc->rdata;	
				twi_wrPtr2 = (byte*)twi_dsc->wdata2;	
				twi_wrCount = twi_dsc->wlen;
				twi_wrCount2 = twi_dsc->wlen2;
				twi_rdCount = twi_dsc->rlen;
				twi_adr = twi_dsc->adr;

				if (twi_wrPtr2 == 0) twi_wrCount2 = 0;

				I2C->PSCR = ~0;//RIF|AIF|TBIF|ACK|NACK|PCR;

				I2C->CCR |= RIEN|AIEN;
				I2C->PCR_IICMode |= PCRIEN|NACKIEN|ARLIEN|SRRIEN|ERRIEN|ACKIEN;

				I2C->TBUF[0] = TDF_MASTER_START | (twi_dsc->adr << 1) | ((twi_wrCount == 0) ? 1 : 0);
			}
			else
			{
				I2C->CCR = I2C__CCR;
				I2C->PCR_IICMode = I2C__PCR;

				twi_lastDsc = twi_dsc = 0;
			};
		};
	}
	else
	{
		tm.Reset();
	};
	
	__enable_irq();

#endif

	return result;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_AddRequest(DSCI2C *d)
{
	if (d == 0) { return false; };
	if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

	d->next = 0;
	d->ready = false;
	if (d->wdata2 == 0) d->wlen2 = 0;

#ifdef CPU_SAME53

	i2c_ReqList.Add(d);

#elif defined(CPU_XMC48)

	__disable_irq();

	if (twi_lastDsc == 0)
	{
		twi_lastDsc = d;

		__enable_irq();

		return I2C_Write(d);
	}
	else
	{
		twi_lastDsc->next = d;
		twi_lastDsc = d;

		__enable_irq();
	};

#elif defined(WIN32)

	u16 adr;

	switch (d->adr)
	{
		case 0x49: //Temp

			if (d->rlen >= 2)
			{
				byte *p = (byte*)d->rdata;

				p[0] = 0;
				p[1] = 0;
			};
				
			d->readedLen = d->rlen;
			d->ack = true;
			d->ready = true;

			break;

		case 0x50: // FRAM

			d->readedLen = 0;

			if (d->wdata != 0 && d->wlen == 2)
			{
				adr = ReverseWord(*((u16*)d->wdata));

				adr %= sizeof(fram_I2c_Mem);

				if (d->wdata2 != 0 && d->wlen2 != 0)
				{
					u16 count = d->wlen2;
					byte *s = (byte*)d->wdata2;
					byte *d = fram_I2c_Mem + adr;

					while (count-- != 0) { *(d++) = *(s++); adr++; if (adr >= sizeof(fram_I2c_Mem)) { adr = 0; d = fram_I2c_Mem; }; };
				}
				else if (d->rdata != 0 && d->rlen != 0)
				{
					d->readedLen = d->rlen;
					u16 count = d->rlen;

					byte *p = (byte*)(d->rdata);
					byte *s = fram_I2c_Mem + adr;

					while (count-- != 0) { *(p++) = *(s++); adr++; if (adr >= sizeof(fram_I2c_Mem)) { adr = 0; s = fram_I2c_Mem; }; };
				};
			};

			d->ack = true;
			d->ready = true;

			break;
	};

#endif

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void I2C_Init()
{
#ifndef WIN32

	using namespace HW;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "I2C Init ... ");

	#ifdef CPU_SAME53	

		HW::GCLK->PCHCTRL[GCLK_SERCOM3_CORE]	= GCLK_GEN(GEN_25M)|GCLK_CHEN;	// 25 MHz

		MCLK->APBBMASK |= APBB_SERCOM3;

		PIO_I2C->SetWRCONFIG(SCL|SDA, PORT_PMUX_C | PORT_PMUXEN | PORT_WRPMUX | PORT_PULLEN | PORT_WRPINCFG);
		PIO_I2C->SET(SCL|SDA);

		I2C->CTRLA = I2C_SWRST;

		while(I2C->SYNCBUSY);

		I2C->CTRLA = SERCOM_MODE_I2C_MASTER;

		I2C->CTRLA = SERCOM_MODE_I2C_MASTER|I2C_INACTOUT_205US|I2C_SPEED_SM;
		I2C->CTRLB = I2C_SMEN;
		I2C->BAUD = 0x0018;

		I2C->CTRLA |= I2C_ENABLE;

		while(I2C->SYNCBUSY);

		I2C->STATUS = 0;
		I2C->STATUS.BUSSTATE = BUSSTATE_IDLE;

	#elif defined(CPU_XMC48)

		HW::Peripheral_Enable(I2C_PID);

 		P5->ModePin0(A1OD);
		P5->ModePin2(A1PP);

		I2C->KSCFG = MODEN|BPMODEN|BPNOM|NOMCFG(0);

		I2C->SCTR = I2C__SCTR;

		I2C->FDR = I2C__FDR;
		I2C->BRG = I2C__BRG;
	    
		I2C->TCSR = I2C__TCSR;

		I2C->PSCR = ~0;

		I2C->CCR = 0;

		I2C->DX0CR = I2C__DX0CR;
		I2C->DX1CR = I2C__DX1CR;

		I2C->CCR = I2C__CCR;


		I2C->PCR_IICMode = I2C__PCR;

		VectorTableExt[I2C_IRQ] = I2C_Handler;
		CM4::NVIC->CLR_PR(I2C_IRQ);
		CM4::NVIC->SET_ER(I2C_IRQ);

	#endif

#else

	HANDLE h;

	h = CreateFile("FRAM_I2C_STORE.BIN", GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	ReadFile(h, fram_I2c_Mem, sizeof(fram_I2c_Mem), &bytes, 0);
	CloseHandle(h);

#endif

	SEGGER_RTT_WriteString(0, "OK\n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

void I2C_Destroy()
{
	HANDLE h;

	h = CreateFile("FRAM_I2C_STORE.BIN", GENERIC_WRITE, 0, 0, OPEN_ALWAYS, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	if (!WriteFile(h, fram_I2c_Mem, sizeof(fram_I2c_Mem), &bytes, 0))
	{
		dword le = GetLastError();
	};

	CloseHandle(h);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
