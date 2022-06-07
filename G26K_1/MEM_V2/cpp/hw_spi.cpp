#include <core.h>
#include <time.h>
#include <spi.h>
#include <SEGGER_RTT.h>
#include <list.h>
#include <DMA.h>
#include "hw_conf.h"
#include "COM_DEF.h"


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


#elif defined(CPU_SAME53)

List<DSCSPI>	spi_ReqList;
DSCSPI*			spi_dsc = 0;

#elif defined(CPU_XMC48)

static byte *spi_wrPtr = 0;
static byte *spi_rdPtr = 0;
static u16 spi_wrCount = 0;
static u16 spi_count = 0;
static u16 spi_rdCount = 0;
static byte *spi_wrPtr2 = 0;
static u16 spi_wrCount2 = 0;
static u32 spi_adr = 0;
static DSCSPI* spi_dsc = 0;
static DSCSPI* spi_lastDsc = 0;

#endif 


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#ifndef WIN32
static u32 SPI_CS_MASK[2] = { CS0, CS1 };
#endif

static u32 spi_timestamp = 0;

//static bool SPI_Write(DSCSPI *d);
//static bool SPI_Read(DSCSPI *d);
static bool SPI_WriteRead(DSCSPI *d);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53	

//static __irq void SPI_Handler()
//{
//	using namespace HW;
//
//	byte state = SPI->INTFLAG & SPI->INTENSET;
//
//	if (state & SPI_DRE)
//	{
//		Pin_SPI_IRQ_Set();
//
//		if (spi_wrCount == 0)
//		{
//			if (spi_wrCount2 != 0)
//			{
//				HW::DMAC->CH[SPI_DMACH_TX].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_TX;
//				spi_wrCount2 = 0;
//				spi_rdCount = 0;
//			};
//
//			SPI->INTENCLR = ~0;
//			SPI->INTENSET = SPI_TXC;
//		}
//		else
//		{
//			SPI->DATA = *(spi_wrPtr++); 
//			spi_wrCount--;
//		};
//	}
//	else if (state & SPI_TXC)
//	{
//		Pin_SPI_IRQ_Set();
//		Pin_SPI_IRQ_Clr();
//		Pin_SPI_IRQ_Set();
//
//		if (spi_rdCount != 0)
//		{
//			spi_rdCount = 0;
//
//			//SPI->DATA = 0; 
//			SPI->INTFLAG = SPI_TXC;
//
//			HW::DMAC->CH[SPI_DMACH_TX].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_TX;
//			HW::DMAC->CH[SPI_DMACH_RX].CTRLA = DMCH_ENABLE|DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_RX;
//
//			SPI->CTRLB |= SPI_RXEN;
//		}
//		else
//		{
//			HW::DMAC->CH[SPI_DMACH_TX].CTRLA = 0;
//			HW::DMAC->CH[SPI_DMACH_RX].CTRLA = 0;
//
//			SPI->INTENCLR = ~0;
//			SPI->INTFLAG = ~0;
//
//			DSCSPI *ndsc = spi_dsc->next;
//				
//			spi_dsc->next = 0;
//
//			spi_dsc->ready = true;
//
//			SPI->CTRLB &= ~SPI_RXEN;
//
//			PIO_CS->SET(CS0|CS1);
//			
//			spi_dsc = 0;
//
//			if (ndsc != 0)
//			{
//				SPI_WriteRead(ndsc);
//			}
//			else
//			{
//				spi_lastDsc = 0;
//			};
//		};
//	};
//
//	Pin_SPI_IRQ_Clr();
//}

#elif defined(CPU_XMC48)

static __irq void SPI_Handler_Write()
{
	using namespace HW;

	Pin_SPI_IRQ_Set();

	volatile u32 a = SPI->PSR_SSCMode;
	
	a &= (SPI->CCR & (RIF|AIF))|MSLSEV;

	if(a & (RIF|AIF))
	{
		SPI->PSCR = RIF|DLIF|TSIF|MSLSEV;

		SPI->PCR_SSCMode |= MSLSIEN;
		SPI->CCR = SPI__CCR|TBIEN;
	}
	else if(a & MSLSEV)
	{
		SPI->PSCR = RIF|DLIF|TSIF|MSLSEV;

		SPI_DMA->CHENREG = SPI_DMA_CHDIS;

		SPI->TRBSCR = TRBSCR_FLUSHTB;

		SPI->PCR_SSCMode = SPI__PCR;

		SPI->TCSR = SPI__TCSR|TDSSM(1);

		SPI->CCR = SPI__CCR;

		SPI->PSCR = DLIF;

		DLR->LNEN &= ~SPI_DLR_LNEN;

		DSCSPI *ndsc = spi_dsc->next;
			
		spi_dsc->next = 0;

		spi_dsc->ready = true;

		PIO_CS->SET(CS0|CS1);
		
		spi_dsc = 0;

		if (ndsc != 0)
		{
			SPI_WriteRead(ndsc);
		}
		else
		{
			spi_lastDsc = 0;
		};
	};


	Pin_SPI_IRQ_Clr();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void SPI_Handler_Read()
{
	using namespace HW;

	Pin_SPI_IRQ_Set();

	volatile u32 a = SPI->PSR_SSCMode & SPI->CCR;

	if(a & TBIF)
	{
		SPI->PSCR = TBIF|TSIF;

		if (spi_count == 0)
		{
			a = SPI->RBUF;
			a = SPI->RBUF;

			SPI->PSCR = RIF|DLIF;
			SPI->CCR = SPI__CCR|RIEN|DLIEN;
			SPI->TBUF[0] = 0;
			SPI->TCSR = SPI__TCSR|TDSSM(0);
		}
		else
		{
			a = SPI->RBUF;
			a = SPI->RBUF;

			if (spi_wrCount > 0)
			{ 
				SPI->TBUF[0] = *(spi_wrPtr++); 
				spi_wrCount--;
			}
			else
			{
				SPI->TBUF[0] = 0;
			};

			spi_count--;
		};
	}
	else if(a & DLIF)
	{
		SPI_DMA->CHENREG = SPI_DMA_CHDIS;

		SPI->PCR_SSCMode = SPI__PCR;

		SPI->TCSR = SPI__TCSR|TDSSM(1);

		SPI->CCR = SPI__CCR;

		SPI->PSCR = ~0;

		DLR->LNEN &= ~SPI_DLR_LNEN;

		DSCSPI *ndsc = spi_dsc->next;
			
		spi_dsc->next = 0;

		spi_dsc->ready = true;

		PIO_CS->SET(CS0|CS1);
		
		spi_dsc = 0;

		if (ndsc != 0)
		{
			SPI_WriteRead(ndsc);
		}
		else
		{
			spi_lastDsc = 0;
		};
	};

	Pin_SPI_IRQ_Clr();
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool SPI_WriteRead(DSCSPI *d)
{
#ifndef WIN32

	using namespace HW;

	if (spi_dsc != 0 || d == 0) { return false; };
	//if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

	#ifdef CPU_SAME53

		spi_ReqList.Add(d);

	#elif defined(CPU_XMC48)

		spi_dsc = d;

		spi_dsc->ready = false;

		u32 alen = (spi_dsc->alen > 4) ? 4 : spi_dsc->alen; 

		spi_wrPtr = (byte*)&spi_dsc->adr;	
		spi_wrCount = spi_count = alen;

		spi_wrPtr2 = (byte*)spi_dsc->wdata;	
		spi_wrCount2 = spi_dsc->wlen;

		spi_rdPtr = (byte*)spi_dsc->rdata;	
		spi_rdCount = spi_dsc->rlen;

		spi_timestamp = GetMilliseconds();

		u32 adr = spi_dsc->adr;

		__disable_irq();

		PIO_CS->CLR(SPI_CS_MASK[spi_dsc->csnum]);

		//SPI->CCR = 0;

		HW::DLR->LNEN &= ~SPI_DLR_LNEN;

		SPI_DMA->CHENREG = SPI_DMA_CHDIS;
		SPI_DMA->DMACFGREG = 1;

		if (spi_wrCount2 != 0)
		{
			SPI_DMACH->CTLL = DINC(2)|SINC(0)|TT_FC(1)|DEST_MSIZE(0)|SRC_MSIZE(0);
			SPI_DMACH->CTLH = BLOCK_TS(spi_dsc->wlen);

			SPI_DMACH->SAR = (u32)spi_dsc->wdata;
			SPI_DMACH->DAR = (u32)&SPI->IN[4];
			SPI_DMACH->CFGL = HS_SEL_SRC;
			SPI_DMACH->CFGH = PROTCTL(1)|DEST_PER(SPI_DLR&7);

			SPI->TRBSCR = TRBSCR_FLUSHTB;
			SPI->TBCTR = TBCTR_SIZE8|TBCTR_LIMIT(0);

			SPI->TCSR = SPI__TCSR|TDSSM(1);

			//SPI->FDR = SPI__BAUD2FDR(spi_dsc->baud);
			SPI->CCR = SPI__CCR;
			SPI->PCR_SSCMode = SPI__PCR|SELO(1<<spi_dsc->csnum);

			VectorTableExt[SPI_IRQ] = SPI_Handler_Write;
			//CM4::NVIC->CLR_PR(SPI_IRQ);
			//CM4::NVIC->SET_ER(SPI_IRQ);
			
			SPI->PSCR = ~0;

			while(SPI->PSR_SSCMode & TBIF)
			{
				SPI->PSCR = ~0;
			};

			while(alen > 0)
			{
				SPI->IN[4] = (byte)adr;
				adr >>= 8;
				alen--;
			};

			HW::DLR->LNEN |= SPI_DLR_LNEN;
			SPI_DMA->CHENREG = SPI_DMA_CHEN;

			SPI->PSCR = ~0;
			SPI->CCR = SPI__CCR|TBIEN|RIEN;
			SPI->INPR = TBINP(SPI_INPR)|RINP(5)|PINP(5);
		}
		else if (spi_rdCount != 0)
		{
			volatile u32 t;

			SPI_DMACH->CTLL = DINC(0)|SINC(2)|TT_FC(2)|DEST_MSIZE(0)|SRC_MSIZE(0);
			SPI_DMACH->CTLH = BLOCK_TS(spi_dsc->rlen);

			SPI_DMACH->SAR = (u32)&SPI->RBUF;
			SPI_DMACH->DAR = (u32)spi_dsc->rdata;
			SPI_DMACH->CFGL = HS_SEL_DST;
			SPI_DMACH->CFGH = PROTCTL(1)|SRC_PER(SPI_DLR&7);

			SPI->RBCTR = 0;
			SPI->TBCTR = 0;

			SPI->TCSR = SPI__TCSR|TDSSM(1);

			SPI->CCR = SPI__CCR;
			SPI->PCR_SSCMode = SPI__PCR|SELO(1<<spi_dsc->csnum);

			SPI_DMA->CHENREG = SPI_DMA_CHEN;

			t = SPI->RBUF;
			t = SPI->RBUF;

	//		SPI->PSCR = ~0;

			VectorTableExt[SPI_IRQ] = SPI_Handler_Read;
			//CM4::NVIC->CLR_PR(SPI_IRQ);
			//CM4::NVIC->SET_ER(SPI_IRQ);

			HW::DLR->LNEN |= SPI_DLR_LNEN;

			SPI->INPR = RINP(0)|PINP(5)|TBINP(5);

			SPI->PSCR = ~0;
			
			while(SPI->PSR_SSCMode & TBIF)
			{
				SPI->PSCR = ~0;
			};

			SPI->CCR = SPI__CCR | TBIEN;
		
			SPI->TBUF[0] = *(spi_wrPtr++);
			spi_wrCount--;
		}
		else
		{
			SPI->TRBSCR = TRBSCR_FLUSHTB;
			SPI->TBCTR = TBCTR_SIZE8|TBCTR_LIMIT(0);

			SPI->TCSR = SPI__TCSR|TDSSM(1);

			SPI->CCR = SPI__CCR;
			SPI->PCR_SSCMode = SPI__PCR|SELO(1<<spi_dsc->csnum);

			VectorTableExt[SPI_IRQ] = SPI_Handler_Write;
			//CM4::NVIC->CLR_PR(SPI_IRQ);
			//CM4::NVIC->SET_ER(SPI_IRQ);
			
			SPI->PSCR = ~0;

			while(SPI->PSR_SSCMode & TBIF)
			{
				SPI->PSCR = ~0;
			};

			SPI->PSCR = ~0;
			SPI->CCR = SPI__CCR|RIEN|AIEN;
			SPI->INPR = TBINP(SPI_INPR)|AINP(5)|RINP(5)|PINP(5);

			while(alen > 0)
			{
				SPI->IN[4] = (byte)adr;
				adr >>= 8;
				alen--;
			};
		};

	#endif
		
	__enable_irq();

#else

#endif

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool SPI_AddRequest(DSCSPI *d)
{
	if (d == 0) { return false; };
	//if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

	d->next = 0;
	d->ready = false;

#ifdef CPU_SAME53

		spi_ReqList.Add(d);

#elif defined(CPU_XMC48)


	u32 t = __disable_irq();

	if (spi_lastDsc == 0)
	{
		spi_lastDsc = d;

		__enable_irq();

		return SPI_WriteRead(d);
	}
	else
	{
		spi_lastDsc->next = d;
		spi_lastDsc = d;

		__enable_irq();
	};

#else

	u32 adr;

	switch (d->csnum)
	{
		case 0x0: //Accel

			if (d->rlen >= 2)
			{
				byte *p = (byte*)d->rdata;

				p[0] = 0;
				p[1] = 0;
			};
				
			d->ready = true;

			break;

		case 0x1: // FRAM

			switch(d->adr&0xFF)
			{
				case 0x2: // WRITE

					if (d->alen == 4 && d->wdata != 0 && d->wlen != 0 && fram_spi_WREN)
					{
						adr = ReverseDword(d->adr & ~0xFF);
						adr %= sizeof(fram_SPI_Mem);

						u16 count = d->wlen;
						byte *s = (byte*)d->wdata;
						byte *d = fram_SPI_Mem + adr;

						while (count-- != 0) { *(d++) = *(s++); adr++; if (adr >= sizeof(fram_SPI_Mem)) { adr = 0; d = fram_SPI_Mem; }; };
					};

					fram_spi_WREN = false;

					break;

				case 0x3: // READ

					if (d->alen == 4 && d->rdata != 0 && d->rlen != 0)
					{
						adr = ReverseDword(d->adr & ~0xFF);
						adr %= sizeof(fram_SPI_Mem);

						u16 count = d->rlen;

						byte *p = (byte*)(d->rdata);
						byte *s = fram_SPI_Mem + adr;

						while (count-- != 0) { *(p++) = *(s++); adr++; if (adr >= sizeof(fram_SPI_Mem)) { adr = 0; s = fram_SPI_Mem; }; };
					};

					fram_spi_WREN = false;

					break;

				case 0x6: // WREN

					fram_spi_WREN = (d->alen == 1);

					break;

				default:	fram_spi_WREN = false; break;
			};

			d->ready = true;

			break;
	};

#endif

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool SPI_Update()
{
	bool result = false;

#ifdef CPU_SAME53

	enum STATE { WAIT = 0, WRITE, READ, STOP };

	static STATE state = WAIT;

	switch (state)
	{
		case WAIT:

			spi_dsc = spi_ReqList.Get();

			if (spi_dsc != 0)
			{
				PIO_CS->CLR(SPI_CS_MASK[spi_dsc->csnum]);

				SPI->INTFLAG = ~0;
				SPI->INTENCLR = ~0;
				SPI->CTRLB &= ~SPI_RXEN;

				DSCSPI &dsc = *spi_dsc;

				dsc.ready = false;

				if (dsc.alen == 0)
				{
					if (dsc.wdata != 0 && dsc.wlen > 0)
					{
						SPI_DMA_TX.WritePeripheral(dsc.wdata, &SPI->DATA, dsc.wlen, DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_TX, DMDSC_BEATSIZE_BYTE);

						state = WRITE; 
					}
					else if (dsc.rdata != 0 && dsc.rlen > 0)
					{
						SPI->CTRLB |= SPI_RXEN;
						SPI_DMA_RX.ReadPeripheral(&SPI->DATA, dsc.rdata, dsc.rlen, DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_RX, DMDSC_BEATSIZE_BYTE);
						SPI_DMA_TX.WritePeripheral(dsc.rdata, &SPI->DATA, dsc.rlen+1, DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_TX, DMDSC_BEATSIZE_BYTE);

						state = READ; 
					};
				}
				else
				{
					SPI_DMA_TX.WritePeripheral(&dsc.adr, &SPI->DATA, dsc.alen, dsc.wdata, dsc.wlen, DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_TX, DMDSC_BEATSIZE_BYTE);

					state = WRITE; 
				};
			};

			break;

		case WRITE:
		{
			DSCSPI &dsc = *spi_dsc;

			if (SPI_DMA_TX.CheckComplete() && (SPI->INTFLAG & SPI_TXC))
			{
				SPI->INTFLAG = ~0;

				if (dsc.rdata != 0 && dsc.rlen > 0)
				{
					SPI->CTRLB |= SPI_RXEN;

					SPI_DMA_RX.ReadPeripheral(&SPI->DATA, dsc.rdata, dsc.rlen, DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_RX, DMDSC_BEATSIZE_BYTE);
					SPI_DMA_TX.WritePeripheral(dsc.rdata, &SPI->DATA, dsc.rlen+1, DMCH_TRIGACT_BURST|DMCH_TRIGSRC_SERCOM0_TX, DMDSC_BEATSIZE_BYTE);

					state = READ; 
				}
				else
				{
					state = STOP; 
				};
			};

			break;
		};

		case READ:
		{
			DSCSPI &dsc = *spi_dsc;

			if (SPI_DMA_RX.CheckComplete())
			{
				state = STOP; 
			};

			break;
		};

		case STOP:
		{
			if (SPI_DMA_TX.CheckComplete() && SPI_DMA_RX.CheckComplete())
			{
				spi_dsc->ready = true;
				
				spi_dsc = 0;
				
				PIO_CS->SET(CS0|CS1);

				SPI->CTRLB &= ~SPI_RXEN;
				SPI->INTFLAG = ~0;
				SPI->INTENCLR = ~0;

				state = WAIT; 
			};

			break;
		};
	};

#elif defined(CPU_XMC48)

	using namespace HW;

	static TM32 tm;

	__disable_irq();

	if (spi_dsc != 0)
	{
		if (!spi_dsc->ready && (GetMilliseconds() - spi_timestamp) > 100)
		{
			result = true;

			HW::Peripheral_Disable(SPI_PID);

			DSCSPI *dsc = spi_dsc;

			spi_dsc = 0;

			SPI_Init();

			SPI_WriteRead(dsc);
		};
	};
	
	__enable_irq();

#endif

	return result;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SPI_Init()
{
#ifndef WIN32

	using namespace HW;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_GREEN "SPI Init ... ");

	#ifdef CPU_SAME53	

		HW::GCLK->PCHCTRL[GCLK_SERCOM0_CORE] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;	// 25 MHz

		MCLK->APBAMASK |= APBA_SERCOM0;

		PIO_SPCK->SetWRCONFIG(SPCK, PORT_PMUX_C|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX);
		PIO_MOSI->SetWRCONFIG(MOSI, PORT_PMUX_C|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX);
		PIO_MISO->SetWRCONFIG(MISO, PORT_PMUX_C|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX);
		PIO_CS->DIRSET = CS0|CS1; 
		PIO_CS->SetWRCONFIG(CS0|CS1, PORT_WRPINCFG|PORT_WRPMUX);
		PIO_CS->SET(CS0|CS1); 

		SPI->CTRLA = SPI_SWRST;

		while(SPI->SYNCBUSY);

		SPI->CTRLA = SERCOM_MODE_SPI_MASTER;

		SPI->CTRLA = SERCOM_MODE_SPI_MASTER|SPI_DIPO(2)|SPI_DOPO(0);
		SPI->CTRLB = 0;
		SPI->CTRLC = 1;
		SPI->BAUD = 12;

		SPI->DBGCTRL = 1;

		SPI->CTRLA |= SPI_ENABLE;

		while(SPI->SYNCBUSY);

		SPI->STATUS = ~0;

		//VectorTableExt[SERCOM0_0_IRQ] = SPI_Handler;
		//VectorTableExt[SERCOM0_1_IRQ] = SPI_Handler;
		//VectorTableExt[SERCOM0_3_IRQ] = SPI_Handler;
		//CM4::NVIC->CLR_PR(SERCOM0_0_IRQ);
		//CM4::NVIC->CLR_PR(SERCOM0_1_IRQ);
		//CM4::NVIC->CLR_PR(SERCOM0_3_IRQ);
		//CM4::NVIC->SET_ER(SERCOM0_0_IRQ);
		//CM4::NVIC->SET_ER(SERCOM0_1_IRQ);
		//CM4::NVIC->SET_ER(SERCOM0_3_IRQ);

	#elif defined(CPU_XMC48)

		HW::Peripheral_Enable(SPI_PID);

		SPI->KSCFG = MODEN|BPMODEN|BPNOM|NOMCFG(0);

		SPI->CCR = 0;

		SPI->FDR = SPI__FDR;
		SPI->BRG = SPI__BRG;
	    
		SPI->SCTR = SPI__SCTR;
		SPI->TCSR = SPI__TCSR;

		SPI->PCR_SSCMode = SPI__PCR;

		SPI->PSCR = ~0;

		SPI->CCR = 0;

		SPI->DX0CR = SPI__DX0CR;
		SPI->DX1CR = SPI__DX1CR;

		SPI->TBCTR = 0;// TBCTR_SIZE8|TBCTR_LIMIT(0);
		SPI->RBCTR = 0;//RBCTR_SIZE8|RBCTR_LIMIT(0);

		SPI->CCR = SPI__CCR;

		PIO_SPCK->ModePin(PIN_SPCK, A2PP);
		PIO_MOSI->ModePin(PIN_MOSI, A2PP);
 		PIO_MISO->ModePin(PIN_MISO, I0DNP);
		PIO_CS->ModePin(PIN_CS0, G_PP);
		PIO_CS->ModePin(PIN_CS1, G_PP);
		PIO_CS->SET(CS0|CS1);

		VectorTableExt[SPI_IRQ] = SPI_Handler_Read;
		CM4::NVIC->CLR_PR(SPI_IRQ);
		CM4::NVIC->SET_ER(SPI_IRQ);


		//SPI->PCR_SSCMode = SPI__PCR|SELO(1);
		
	//	SPI->PSCR |= TBIF;

	//	SPI->CCR = SPI__CCR|TBIEN;
	//	SPI->INPR = 0;

		//SPI->IN[0] = 0x55;
		//SPI->IN[0] = 0x55;

		//while ((SPI->PSR & TSIF) == 0);

	//	SPI->CCR = SPI__CCR;

	#endif
#else

	HANDLE h;

	h = CreateFile("FRAM_SPI_STORE.BIN", GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	ReadFile(h, fram_SPI_Mem, sizeof(fram_SPI_Mem), &bytes, 0);
	CloseHandle(h);

#endif

	SEGGER_RTT_WriteString(0, "OK\n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

void SPI_Destroy()
{
	HANDLE h;

	h = CreateFile("FRAM_SPI_STORE.BIN", GENERIC_WRITE, 0, 0, OPEN_ALWAYS, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	if (!WriteFile(h, fram_SPI_Mem, sizeof(fram_SPI_Mem), &bytes, 0))
	{
		dword le = GetLastError();
	};

	CloseHandle(h);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
