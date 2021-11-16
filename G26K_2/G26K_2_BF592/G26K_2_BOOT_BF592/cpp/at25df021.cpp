/*******************************************************************************/
/*                                                                             */
/*   (C) Copyright 2008 - Analog Devices, Inc.  All rights reserved.           */
/*                                                                             */
/*    FILE:     at25df021.c                                                       */
/*                                                                             */
/*    PURPOSE:  Performs operations specific to the M25P16 flash device.       */
/*                                                                             */
/*******************************************************************************/



//#include <cdefBF592-A.h>
//#include <ccblkfn.h>
//#include <stdio.h>
//#include <drivers\flash\util.h>
//#include <drivers\flash\Errors.h>



#include <sys\exception.h>
#include <bfrom.h>

#include "at25df021.h"

#include "types.h"
#include "hardware.h"
#include "CRC16.h"
#include "list.h"

//#pragma optimize_for_speed


static char 		*pFlashDesc =		"Atmel AT25DF021";
static char 		*pDeviceCompany	=	"Atmel Corporation";

static int			gNumSectors = NUM_SECTORS;

static u32			bufsect[(SECTOR_SIZE+3)/4];
static u16			lastErasedBlock = ~0;

static byte*		flashWritePtr = 0;
static u16			flashWriteLen = 0;
static u32			flashWriteAdr = 0;

#pragma instantiate List<Req>
static List<Req>	freeReq;
static List<Req>	readyReq;

static Req			_req[64];


static ERROR_CODE	lastError = NO_ERR;

enum FlashState 
{ 
	FLASH_STATE_WAIT = 0, 
	FLASH_STATE_ERASE_START, 
	FLASH_STATE_ERASE_WAIT,
	FLASH_STATE_ERASE_WAIT_2,
	FLASH_STATE_ERASE_CHECK,
	FLASH_STATE_WRITE_START, 
	FLASH_STATE_WRITE_PAGE, 
	FLASH_STATE_WRITE_PAGE_2, 
	FLASH_STATE_WRITE_PAGE_3, 
	FLASH_STATE_WRITE_PAGE_4, 
	FLASH_STATE_VERIFY_PAGE
};

static FlashState flashState = FLASH_STATE_WAIT;

#undef TIMEOUT
#undef DELAY

/* flash commands */
#define SPI_WREN            (0x06)  //Set Write Enable Latch
#define SPI_WRDI            (0x04)  //Reset Write Enable Latch
#define SPI_RDID            (0x9F)  //Read Identification
#define SPI_RDSR            (0x05)  //Read Status Register
#define SPI_WRSR            (0x01)  //Write Status Register
#define SPI_READ            (0x03)  //Read data from memory
#define SPI_FAST_READ       (0x0B)  //Read data from memory
#define SPI_PP              (0x02)  //Program Data into memory
#define SPI_SE              (0x20)  //Erase one sector in memory
#define SPI_BE              (0xC7)  //Erase all memory
#define RDY_BSY 			(0x1)	//Check the write in progress bit of the SPI status register
#define WEL					(0x2)	//Check the write enable bit of the SPI status register
#define SWP					(0xC)	//Software Protection Status
#define WPP					(0x10)	//Write Protect (WP) Pin Status
#define EPE					(0x20)	//Erase/Program Error
#define SPRL				(0x80)	//Sector Protection Registers Locked

#define SPI_UPS				(0x39)  //Unprotect Sector 
#define SPI_PRS				(0x36)  //Protect Sector 


#define SPI_PAGE_SIZE		(256)
//#define SPI_SECTORS		    (512)
//#define SPI_SECTOR_SIZE		(4224)
//#define SPI_SECTOR_DIFF		(3968)
//#define PAGE_BITS			(10)
//#define PAGE_SIZE_DIFF		(496)

#define DELAY				15
#define TIMEOUT        35000*64

//char			SPI_Page_Buffer[SPI_PAGE_SIZE];
//int 			SPI_Page_Index = 0;
//char            SPI_Status;


/* function prototypes */
//static ERROR_CODE SetupForFlash();
//static ERROR_CODE Wait_For_nStatus(void);
ERROR_CODE Wait_For_Status( char Statusbit );
static ERROR_CODE Wait_For_WEL(void);
byte ReadStatusRegister(void);
extern void SetupSPI();
extern void SPI_OFF(void);
void __SendSingleCommand( const int iCommand );
//unsigned long DataFlashAddress (unsigned long address);

static ERROR_CODE PollToggleBit(void);
static byte ReadFlash();
static void __WriteFlash(byte usValue);
static unsigned long GetFlashStartAddress( unsigned long ulAddr);
static void GlobalUnProtect();
static ERROR_CODE GetSectorStartEnd( unsigned long *ulStartOff, unsigned long *ulEndOff, int nSector );

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

ERROR_CODE GetLastError()
{
	return lastError;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


static void CmdWriteEnable()
{
	__SendSingleCommand(SPI_WREN);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CmdWriteDisable()
{
	__SendSingleCommand(SPI_WRDI);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CmdEraseSector(u32 sec)
{
	SetupSPI();

	__WriteFlash(SPI_SE);

	__WriteFlash(sec >> 16);
	__WriteFlash(sec >> 8);
	__WriteFlash(sec);

	SPI_OFF();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CmdWriteStatusReg(byte stat)
{
	SetupSPI();

	__WriteFlash(SPI_WRSR);
	__WriteFlash(stat);

	SPI_OFF();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 GetNumSectors()
{
	return gNumSectors;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 GetSectorSize()
{
	return SECTOR_SIZE;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SetupSPIDMA()
{
    volatile int i;

	/* PF8 - SPI0_SSEL2 */

	*pPORTF_FER   |= (PF13 | PF14 | PF15);
	*pPORTF_FER   &= ~(PF8);
	*pPORTF_MUX   &= ~(PF13 | PF14 | PF15);
   	*pPORTFIO_SET = PF8;
  	*pPORTFIO_DIR |= PF8;
   	*pPORTFIO_SET = PF8;

 //  	for(i=0; i<DELAY; i++)
	//{
		//asm("nop;");
		//asm("nop;");
		//asm("nop;");
		//asm("nop;");
		//asm("nop;");
//	}

	*pSPI0_BAUD = BAUD_RATE_DIVISOR;
	*pPORTFIO_CLEAR = PF8;

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//////////////////////////////////////////////////////////////
// void Wait_For_SPIF(void)
//
// Polls the SPIF (SPI single word transfer complete) bit
// of SPISTAT until the transfer is complete.
// Inputs - none
// returns- none
//
//////////////////////////////////////////////////////////////
static void Wait_For_SPIF(void)
{
	volatile int n;

	for(n=0; n<DELAY; n++)
	{
		asm("nop;");
	}

	while((*pSPI0_STAT & SPIF) == 0) { *pWDOG_STAT = 0; };
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Wait_For_RXS_SPIF(void)
{
	volatile int n;

	for(n=0; n<DELAY; n++)
	{
		asm("nop;");
	}

	while((*pSPI0_STAT & (SPIF|RXS)) != (SPIF|RXS)) { *pWDOG_STAT = 0; };
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 WaitReadSPI0()
{
	while ((*pSPI0_STAT & RXS) == 0) { *pWDOG_STAT = 0; };

	return *pSPI0_RDBR;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void WriteSyncDMA(void *data, u16 count)
{
	*pSPI0_CTL = COMMON_SPI_DMA_SETTINGS|3;	

	*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8/*|SYNC*/;
	*pDMA5_START_ADDR = (void*)data;
	*pDMA5_X_COUNT = count;
	*pDMA5_X_MODIFY = 1;

	*pDMA5_CONFIG |= DMAEN;
	*pSPI0_CTL |= SPE;

	asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");

	while ((*pDMA5_IRQ_STATUS & DMA_DONE) == 0) { *pWDOG_STAT = 0; };

	asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");

	while ((*pSPI0_STAT & SPIF) == 0 || (*pSPI0_STAT & TXS)) { *pWDOG_STAT = 0; };

	*pDMA5_IRQ_STATUS = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void WriteAsyncDMA(void *data, u16 count)
{
	*pSPI0_CTL = COMMON_SPI_DMA_SETTINGS|3;	

	*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8/*|SYNC*/;
	*pDMA5_START_ADDR = (void*)data;
	*pDMA5_X_COUNT = count;
	*pDMA5_X_MODIFY = 1;

	*pDMA5_IRQ_STATUS = DMA_DONE;

	*pDMA5_CONFIG |= DMAEN;
	*pSPI0_CTL |= SPE;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CheckWriteAsyncDMA()
{
	if ((*pDMA5_IRQ_STATUS & DMA_DONE) && (*pSPI0_STAT & SPIF) && (*pSPI0_STAT & TXS) == 0)
	{
		*pSPI0_CTL = 0;

		*pDMA5_CONFIG = 0;

		SPI_OFF();

		return true;
	}
	else
	{
		return false;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void ReadSyncDMA(void *data, u16 count)
{
	*pSPI0_CTL = COMMON_SPI_DMA_SETTINGS|2;
	*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8|WNR|SYNC;

	*pDMA5_START_ADDR = data;
	*pDMA5_X_COUNT = count;
	*pDMA5_X_MODIFY = 1;

	*pDMA5_CONFIG |= DMAEN;
	*pSPI0_CTL |= SPE;

	asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");asm("nop;");

	while ((*pDMA5_IRQ_STATUS & DMA_DONE) == 0) { *pWDOG_STAT = 0; };

	*pDMA5_IRQ_STATUS = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void ReadAsyncDMA(void *data, u32 stAdr, u16 count)
{
	SetupSPI();

	__WriteFlash(SPI_FAST_READ );

	__WriteFlash(stAdr >> 16);
	__WriteFlash(stAdr >> 8);
	__WriteFlash(stAdr);
	__WriteFlash(0);

	*pSPI0_CTL = COMMON_SPI_DMA_SETTINGS|2;
	*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8|WNR|SYNC;

	*pDMA5_START_ADDR = data;
	*pDMA5_X_COUNT = count;
	*pDMA5_X_MODIFY = 1;

	*pDMA5_IRQ_STATUS = 1;

	*pDMA5_CONFIG |= DMAEN;
	*pSPI0_CTL |= SPE;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CheckReadAsyncDMA()
{
	if (*pDMA5_IRQ_STATUS & DMA_DONE)
	{
		*pSPI0_CTL = 0;

		*pDMA5_CONFIG = 0;

		SPI_OFF();

		return true;
	}
	else
	{
		return false;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//EX_INTERRUPT_HANDLER(SPI0_WriteDMA_ISR)
//{
//	if (*pDMA5_IRQ_STATUS & 1)
//	{
//		*pDMA5_IRQ_STATUS = 1;
//		*pDMA5_CONFIG = 0;
//
////		*pIMASK &= ~EVT_IVG10; 
//		*pSIC_IMASK &= ~IRQ_DMA5;
//
//		SPI_OFF();
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 readCountSPI0 = 0;
static byte *readDataSPI0 = 0;
static bool *readReadySPI0 = 0;

static u16 writeCountSPI0 = 0;
static byte *writeDataSPI0 = 0;
static bool *writeReadySPI0 = 0;

static DataCRC CRC_SPI0 = { 0xFFFF };

static u16 *ptrCRC_SPI0 = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//EX_INTERRUPT_HANDLER(SPI0_GetCRC_ISR)
//{
//	u16 t = *pSPI0_STAT;
//
//	if (t & RXS)
//	{
//		*pSPI0_TDBR = 0;
//
//		readCountSPI0--;
//
//		CRC_SPI0.w = tableCRC[CRC_SPI0.b[0] ^ *pSPI0_RDBR] ^ CRC_SPI0.b[1];
//
//		if (readCountSPI0 == 0)
//		{
//			*pSPI0_CTL = 0;	
//			
//			*pSIC_IMASK &= ~IRQ_DMA5;
//
//			*readReadySPI0 = true;
//			*ptrCRC_SPI0 = CRC_SPI0.w;
//			
//			SPI_OFF();
//		};
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//EX_INTERRUPT_HANDLER(SPI0_ReadDMA_ISR)
//{
//	if (*pDMA5_IRQ_STATUS & 1)
//	{
//		*pSPI0_CTL = 0;
//
//		*pDMA5_IRQ_STATUS = 1;
//	
//		*pDMA5_CONFIG = 0;
//
//		*pSIC_IMASK &= ~IRQ_DMA5;
//
//		*readReadySPI0 = true;
//		
//		SPI_OFF();
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//EX_INTERRUPT_HANDLER(SPI0_ReadIRQ_ISR)
//{
//	u16 t = *pSPI0_STAT;
//
//	if (t & RXS)
//	{
//		if (readCountSPI0 == 1)
//		{
//			*pSPI0_CTL = 0;	
//			
//			*pSIC_IMASK &= ~IRQ_DMA5;
//
//			*readReadySPI0 = true;
//			
//			SPI_OFF();
//		};
//
//		*pSPI0_TDBR = 0;
//
//		readCountSPI0--;
//
//		*readDataSPI0++ = *pSPI0_RDBR;
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//EX_INTERRUPT_HANDLER(SPI0_StatusWRD_ISR)
//{
//	u16 t = *pSPI0_STAT;
//
//	if (t & RXS)
//	{
//		if (writeCountSPI0 == 0)
//		{
//			if (readCountSPI0 > 0)
//			{
//				if (ptrCRC_SPI0 != 0)
//				{
//					*pEVT10 = (void*)SPI0_GetCRC_ISR;
//				}
//				else
//				{
//					//*pEVT10 = (void*)SPI0_ReadIRQ_ISR;
//
//					//*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8|WNR/*|SYNC*/;
//
//					*pSPI0_CTL = COMMON_SPI_DMA_SETTINGS|2;
//
//					*pDMA5_START_ADDR = readDataSPI0;
//					*pDMA5_X_COUNT = readCountSPI0;
//					*pDMA5_X_MODIFY = 1;
//
//					*pEVT10 = (void*)SPI0_ReadDMA_ISR;
//
//					*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8|WNR|DMAEN;
//					*pSPI0_CTL |= SPE;
//
//					return;
//				};
//			}
//			else
//			{
//				*pSPI0_CTL = 0;	
//				
//				*pSIC_IMASK &= ~IRQ_DMA5;
//
//				*writeReadySPI0 = true;
//				
//				SPI_OFF();
//			};
//		}
//		else
//		{
//			*pSPI0_TDBR = *writeDataSPI0++;
//
//			writeCountSPI0--;
//		};
//
//		t = *pSPI0_RDBR;
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//ERROR_CODE at25df021_Read_IRQ(byte *data, u32 stAdr, u16 count, bool *ready)
//{
//	static byte buf[5];
//	static bool defReady = false;
//
// //   ERROR_CODE Result = NO_ERR;
//
//    buf[0] = SPI_FAST_READ;
//    buf[1] = stAdr >> 16;
//    buf[2] = stAdr >> 8;
//    buf[3] = stAdr;
//    buf[4] = 0;
//
//	readDataSPI0 = data;
//	readCountSPI0 = count;
//
//	readReadySPI0 = (ready != 0) ? ready : &defReady;
//
//	*readReadySPI0 = false;
//
//	writeDataSPI0 = buf;
//	writeCountSPI0 = sizeof(buf);
//
//	SetupSPIDMA();
//
//	*pEVT10 = (void*)SPI0_StatusWRD_ISR;
//	*pIMASK |= EVT_IVG10; 
//	*pSIC_IMASK |= IRQ_DMA5;
//
//	*pSPI0_CTL = COMMON_SPI_SETTINGS|0;
//
//	*pSPI0_TDBR = *writeDataSPI0++;
//	writeCountSPI0--;
//
//	u16 t = *pSPI0_RDBR;
//
//	return NO_ERR;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//EX_INTERRUPT_HANDLER(SPI0_WriteReadDMA_ISR)
//{
//	if (*pDMA5_IRQ_STATUS & 1)
//	{
//		*pDMA5_IRQ_STATUS = 1;
//
//		//while (*pDMA5_IRQ_STATUS & DMA_RUN);
//
//		//for (byte c = 2; c > 0; )
//		//{
//		//	c = (*pSPI0_STAT & TXS) ? 2 : (c-1);
//		//};
//
//		while ((*pSPI0_STAT & SPIF) == 0 || (*pSPI0_STAT & TXS));
//
//		*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8|WNR/*|SYNC*/;
//
//		*pDMA5_START_ADDR = readDataSPI0;
//		*pDMA5_X_COUNT = readCountSPI0;
//		*pDMA5_X_MODIFY = 1;
//
//		*pEVT10 = (void*)SPI0_ReadDMA_ISR;
//
//
//		*pSPI0_CTL = COMMON_SPI_DMA_SETTINGS|2;	
//
//		*pDMA5_CONFIG |= DMAEN;
//		*pSPI0_CTL |= SPE;
//	};
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//ERROR_CODE at25df021_Read_DMA(byte *data, u32 stAdr, u16 count, bool *ready)
//{
//	static byte buf[5];
//	static bool defReady = false;
//
// //   ERROR_CODE Result = NO_ERR;
//
//    buf[0] = SPI_FAST_READ;
//    buf[1] = stAdr >> 16;
//    buf[2] = stAdr >> 8;
//    buf[3] = stAdr;
//    buf[4] = 0;
//
//	readDataSPI0 = data;
//	readCountSPI0 = count;
//
//	readReadySPI0 = (ready != 0) ? ready : &defReady;
//
//	*readReadySPI0 = false;
//
//	SetupSPIDMA();
//
//	*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8/*|SYNC*/;
//	*pDMA5_START_ADDR = buf;
//	*pDMA5_X_COUNT = 5;
//	*pDMA5_X_MODIFY = 1;
//
//	*pEVT10 = (void*)SPI0_WriteReadDMA_ISR;
//	*pIMASK |= EVT_IVG10; 
//	*pSIC_IMASK |= IRQ_DMA5;
//
//	*pEVT7 = (void*)SPI0_StatusWRD_ISR;
//	*pIMASK |= EVT_IVG7; 
//	*pSIC_IMASK |= IRQ_SPI0_ERR;
//
//	*pSPI0_CTL = COMMON_SPI_DMA_SETTINGS|3;	
//
//	*pDMA5_CONFIG |= DMAEN;
//	*pSPI0_CTL |= SPE;
//
//	return NO_ERR;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

ERROR_CODE at25df021_Read(void *data, u32 stAdr, u16 count )
{
	static byte buf[5];

    buf[0] = SPI_FAST_READ;
    buf[1] = stAdr >> 16;
    buf[2] = stAdr >> 8;
    buf[3] = stAdr;
    buf[4] = 0;

	SetupSPIDMA();

	WriteSyncDMA(buf, sizeof(buf));

	ReadSyncDMA(data, count);

	*pSPI0_CTL = 0;

	*pDMA5_CONFIG = 0;

	SPI_OFF();

	return NO_ERR;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//ERROR_CODE at25df021_Read(byte *data, u32 stAdr, u32 count )
//{
//    ERROR_CODE Result = NO_ERR;
//
//	SetupSPI();
//
//        /* send the bulk erase command to the flash */
//    WriteFlash(SPI_FAST_READ);
//    WriteFlash((stAdr) >> 16);
//    WriteFlash((stAdr) >> 8);
//    WriteFlash(stAdr);
//    WriteFlash(0);
//
//	for ( ; count > 0; count--)
//	{
//		*data++ = ReadFlash();
//	};
//
//    SPI_OFF();
//
//	return(Result);
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//ERROR_CODE at25df021_GetCRC16_IRQ(u32 stAdr, u16 count, bool *ready, u16 *crc)
//{
//	CRC_SPI0.w = 0xFFFF;
//
//	ptrCRC_SPI0 = crc;
//
//	return at25df021_Read_IRQ(0,stAdr, count, ready);
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 at25df021_GetCRC16(u32 stAdr, u16 count)
{
	DataCRC crc;

	crc.w = 0xFFFF;

	u16 t = 0;

	static byte buf[5];

    buf[0] = SPI_FAST_READ;
    buf[1] = stAdr >> 16;
    buf[2] = stAdr >> 8;
    buf[3] = stAdr;
    buf[4] = 0;

	SetupSPIDMA();

	WriteSyncDMA(buf, sizeof(buf));

	*pDMA5_CONFIG = 0;

	*pSPI0_CTL = SPE|COMMON_SPI_DMA_SETTINGS|0;

	t = *pSPI0_RDBR;

	for ( ; count > 0; count--)
	{
		while ((*pSPI0_STAT & RXS) == 0);

		t = *pSPI0_RDBR;

		crc.w = tableCRC[crc.b[0] ^ t] ^ crc.b[1];
		
		*pWDOG_STAT = 0;
	};
	
	*pSPI0_CTL = 0;

	SPI_OFF();

	return crc.w;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ERROR_CODE WritePage(void *data, u32 stAdr, u16 count )
{
	static byte buf[4];

    ERROR_CODE Result = NO_ERR;

	if ((stAdr & 0xFF) != 0 || count > 256 || count == 0)
	{
		return INVALID_BLOCK;
	};

	CmdWriteEnable();

	Result = Wait_For_WEL();

    if(Result != NO_ERR)
	{
		return Result;
	}
    else
    {
		buf[0] = SPI_PP;
		buf[1] = stAdr >> 16;
		buf[2] = stAdr >> 8;
		buf[3] = stAdr;

		SetupSPIDMA();

		WriteSyncDMA(buf, sizeof(buf));

		WriteSyncDMA(data, count);

		*pSPI0_CTL = 0;

		*pDMA5_CONFIG = 0;

		SPI_OFF();
    };

	return Wait_For_Status(RDY_BSY);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void WritePageAsync(void *data, u32 stAdr, u16 count )
{
	SetupSPI();

	__WriteFlash(SPI_PP );

	__WriteFlash(stAdr >> 16);
	__WriteFlash(stAdr >> 8);
	__WriteFlash(stAdr);

	WriteAsyncDMA(data, count);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//ERROR_CODE WritePage(const byte *data, u32 stAdr, u16 count )
//{
//    ERROR_CODE Result = NO_ERR;
//
//	if ((stAdr & 0xFF) != 0 || count > 256 || count == 0)
//	{
//		return INVALID_BLOCK;
//	};
//
//    SendSingleCommand(SPI_WREN);
//
//    Result = Wait_For_WEL();
//
//    if( POLL_TIMEOUT == Result )
//	{
//		return Result;
//	}
//    else
//    {
//        SetupSPI();
//
//        /* send the bulk erase command to the flash */
//        WriteFlash(SPI_PP );
//        WriteFlash((stAdr) >> 16);
//        WriteFlash((stAdr) >> 8);
//        WriteFlash(stAdr);
//
//		for ( ; count > 0; count--)
//		{
//	        WriteFlash(*data++);
//		};
//
//        SPI_OFF();
//    };
//
//	return Wait_For_Status(WIP);
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//ERROR_CODE VerifyPage(const byte *data, u32 stAdr, u16 count )
//{
//    ERROR_CODE Result = NO_ERR;
//
//	if ((stAdr & 0xFF) != 0 || count > 256 || count == 0)
//	{
//		return INVALID_BLOCK;
//	};
//
//	u16 t = 0;
//
//	static byte buf[5];
//
//    buf[0] = SPI_FAST_READ;
//    buf[1] = stAdr >> 16;
//    buf[2] = stAdr >> 8;
//    buf[3] = stAdr;
//    buf[4] = 0;
//
//	SetupSPIDMA();
//
//	WriteSyncDMA(buf, sizeof(buf));
//
//	*pDMA5_CONFIG = 0;
//
//	*pSPI0_CTL = COMMON_SPI_SETTINGS|TIMOD01;
//
////	t = *pSPI0_RDBR;
//
//	for ( ; count > 0; count--)
//	{
////		t = WaitReadSPI0(); //while ((*pSPI0_STAT & RXS) == 0);
//
//		if (ReadFlash() != *data)
//		{
//			Result = VERIFY_WRITE;
//			break;
//		};
//	};
//	
//	*pSPI0_CTL = 0;
//
//	SPI_OFF();
//
//	return Result;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ERROR_CODE VerifyPage(const byte *data, u32 stAdr, u16 count )
{
    ERROR_CODE Result = NO_ERR;

	if ((stAdr & 0xFF) != 0 || count > 256 || count == 0)
	{
		return INVALID_BLOCK;
	};

	SetupSPI();

        /* send the bulk erase command to the flash */
    __WriteFlash(SPI_FAST_READ);
    __WriteFlash((stAdr) >> 16);
    __WriteFlash((stAdr) >> 8);
    __WriteFlash(stAdr);
    __WriteFlash(0);

	for ( ; count > 0; count--)
	{
		if (ReadFlash() != *data)
		{
			Result = VERIFY_WRITE;
			break;
		};

		data++;
	};

    SPI_OFF();

	return Result;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

ERROR_CODE at25df021_Write(const byte *data, u32 stAdr, u32 count, bool verify)
{
    ERROR_CODE Result = NO_ERR;

	u32 c;

	while (count > 0)
	{
		u16 c = (count >= 256) ? 256 : count;

		count -= c;

		Result = WritePage((void*)data, stAdr, c);

		if (Result != NO_ERR) break;

		if (verify)
		{
			Result = VerifyPage(data, stAdr, c);
			if (Result != NO_ERR) break;
		};

		data += c;
		stAdr += c;

    };

    return(Result);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//-----  H e l p e r   F u n c t i o n s	----//

//----------- R e s e t F l a s h  ( ) ----------//
//
//  PURPOSE
//  	Sends a "reset" command to the flash.
//
//	INPUTS
//		unsigned long ulStartAddr - flash start address
//
// 	RETURN VALUE
//  	ERROR_CODE - value if any error occurs
//  	NO_ERR     - otherwise

//static ERROR_CODE ResetFlash()
//{
//	SetupSPI();
//
//	//send the bulk erase command to the flash
//	WriteFlash(SPI_WRDI);
//
//	SPI_OFF();
//
//	return PollToggleBit();
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void GlobalUnProtect()
{
	CmdWriteEnable();

	CmdWriteStatusReg(0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//----------- E r a s e F l a s h  ( ) ----------//
//
//  PURPOSE
//  	Sends an "erase all" command to the flash.
//
//	INPUTS
//		unsigned long ulStartAddr - flash start address
//
// 	RETURN VALUE
//  	ERROR_CODE - value if any error occurs
//  	NO_ERR     - otherwise

//ERROR_CODE EraseFlash()
//{
//	ERROR_CODE ErrorCode = NO_ERR;	// tells us if there was an error erasing flash
////	int nBlock = 0;					// index for each block to erase
//
//	GlobalUnProtect();
//	GlobalUnProtect();
//
//	//A write enable instruction must previously have been executed
//	SendSingleCommand(SPI_WREN);
//
//	//The status register will be polled to check the write enable latch "WREN"
//	ErrorCode = Wait_For_WEL();
//
//	if( POLL_TIMEOUT == ErrorCode )
//	{
//		return ErrorCode;
//	}
//	else
//	{
//	    //The bulk erase instruction will erase the whole flash
//		SendSingleCommand(SPI_BE);
//
//		// Erasing the whole flash will take time, so the following bit must be polled.
//		//The status register will be polled to check the write in progress bit "WIP"
//		ErrorCode = Wait_For_Status(WIP);
//
//		printf("Error Code: %d", ErrorCode);
//
//
//	}
//
//	// erase should be complete
//	return ErrorCode;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static ERROR_CODE UnProtectBlock(u32 adr)
//{
//	ERROR_CODE 	  ErrorCode   = NO_ERR;		//tells us if there was an error erasing flash
//
//	SendSingleCommand(SPI_WREN );
//
//	SetupSPI();
//
//	WriteFlash(SPI_WRSR);
//	WriteFlash(0);
//
//	SPI_OFF();
//
//	// Poll the status register to check the Write in Progress bit
//	// Sector erase takes time
//	return ErrorCode;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//----------- E r a s e B l o c k ( ) ----------//
//
//  PURPOSE
//  	Sends an "erase block" command to the flash.
//
//	INPUTS
//		int nBlock - block to erase
//		unsigned long ulStartAddr - flash start address
//
// 	RETURN VALUE
//  	ERROR_CODE - value if any error occurs
//  	NO_ERR     - otherwise

ERROR_CODE EraseBlock(int nBlock)
{

	ERROR_CODE 	  ErrorCode   = NO_ERR;		//tells us if there was an error erasing flash
 	unsigned long ulSectStart = 0x0;		//stores the sector start offset
 	unsigned long ulSectEnd   = 0x0;		//stores the sector end offset(however we do not use it here)

	// Get the sector start offset
	// we get the end offset too however we do not actually use it for Erase sector
	GetSectorStartEnd( &ulSectStart, &ulSectEnd, nBlock );

	GlobalUnProtect();
	GlobalUnProtect();

	CmdWriteEnable();

	CmdEraseSector(ulSectStart);

	ErrorCode = Wait_For_Status(RDY_BSY);

	if (ErrorCode == NO_ERR)
	{
		ErrorCode == at25df021_Read(bufsect, ulSectStart, sizeof(bufsect));

		if (ErrorCode == NO_ERR)
		{
			for (u32 i = ArraySize(bufsect); i > 0; i--)
			{
				if (bufsect[i] != ~0) { ErrorCode = VERIFY_WRITE; break; };
			};
		};
	};

 	// block erase should be complete
	return ErrorCode;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//----------- P o l l T o g g l e B i t ( ) ----------//
//
//  PURPOSE
//  	Polls the toggle bit in the flash to see when the operation
//		is complete.
//
//	INPUTS
//	unsigned long ulAddr - address in flash
//
// 	RETURN VALUE
//  	ERROR_CODE - value if any error occurs
//  	NO_ERR     - otherwise


//static ERROR_CODE PollToggleBit(void)
//{
//	ERROR_CODE ErrorCode = NO_ERR;	// flag to indicate error
//	char status_register = 0;
//	int i;
//
//	for(i = 0; i < 500; i++)
//	{
//		status_register = ReadStatusRegister();
//		if( (status_register & WEL) )
//		{
//			ErrorCode = NO_ERR;
//
//		}
//		ErrorCode = POLL_TIMEOUT;	// Time out error
//	};
//
//	// we can return
//	return ErrorCode;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//----------- G e t C o d e s ( ) ----------//
//
//  PURPOSE
//  	Sends an "auto select" command to the flash which will allow
//		us to get the manufacturer and device codes.
//
//  INPUTS
//  	int *pnManCode - pointer to manufacture code
//		int *pnDevCode - pointer to device code
//		unsigned long ulStartAddr - flash start address
//
//	RETURN VALUE
//  	ERROR_CODE - value if any error occurs
//  	NO_ERR     - otherwise

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static ERROR_CODE GetCodes(int *pnManCode, int *pnDevCode)
//{
//	//Open the SPI, Deasserting CS
//	SetupSPI();
//
//	//Write the OpCode and Write address, 4 bytes.
//	WriteFlash( SPI_RDID );
//
//	// now we can read the codes
//	*pnManCode = ReadFlash();
//
//	*pnDevCode = ReadFlash();
//
//	SPI_OFF();
//	// ok
//	return NO_ERR;
//}

//----------- G e t S e c t o r N u m b e r ( ) ----------//
//
//  PURPOSE
//  	Gets a sector number based on the offset.
//
//  INPUTS
//  	unsigned long ulAddr - absolute address
//		int 	 *pnSector     - pointer to sector number
//
//	RETURN VALUE
//  	ERROR_CODE - value if any error occurs
//  	NO_ERR     - otherwise

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static ERROR_CODE GetSectorNumber( unsigned long ulAddr, int *pnSector )
//{
//	int nSector = 0;
//	int i;
//	int error_code = 1;
//	unsigned long ulMask;					//offset mask
//	unsigned long ulOffset;					//offset
//	unsigned long ulStartOff;
//	unsigned long ulEndOff;
//
//	ulMask      	  = 0x7ffffff;
//	ulOffset		  = ulAddr & ulMask;
//
//	for(i = 0; i < gNumSectors; i++)
//	{
//	    GetSectorStartEnd(&ulStartOff, &ulEndOff, i);
//		if ( (ulOffset >= ulStartOff)
//			&& (ulOffset <= ulEndOff) )
//		{
//			error_code = 0;
//			nSector = i;
//			break;
//		}
//	}
//
//	// if it is a valid sector, set it
//	if (error_code == 0)
//		*pnSector = nSector;
//	// else it is an invalid sector
//	else
//		return INVALID_SECTOR;
//
//	// ok
//	return NO_ERR;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//----------- G e t S e c t o r S t a r t E n d ( ) ----------//
//
//  PURPOSE
//  	Gets a sector start and end address based on the sector number.
//
//  INPUTS
//  	unsigned long *ulStartOff - pointer to the start offset
//		unsigned long *ulEndOff - pointer to the end offset
//		int nSector - sector number
//
//	RETURN VALUE
//		ERROR_CODE - value if any error occurs
//  	NO_ERR     - otherwise

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static ERROR_CODE GetSectorStartEnd( unsigned long *ulStartOff, unsigned long *ulEndOff, int nSector )
{
	u32 ulSectorSize = SECTOR_SIZE;

	if( ( nSector >= 0 ) && ( nSector < gNumSectors ) ) // 32 sectors
	{
		*ulStartOff = nSector * ulSectorSize;
		*ulEndOff = ( (*ulStartOff) + ulSectorSize - 1 );
	}
	else
	{
		return INVALID_SECTOR;
	};

	// ok
	return NO_ERR;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//----------- G e t F l a s h S t a r t A d d r e s s ( ) ----------//
//
//  PURPOSE
//  	Gets flash start address from an absolute address.
//
//  INPUTS
//  	unsigned long ulAddr - absolute address
//
//	RETURN VALUE
//		unsigned long - Flash start address

//static unsigned long GetFlashStartAddress( unsigned long ulAddr)
//{
//
//	unsigned long ulFlashStartAddr;			//flash start address
//
//	ulFlashStartAddr  =  0;
//
//	return(ulFlashStartAddr);
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//----------- R e a d F l a s h ( ) ----------//
//
//  PURPOSE
//  	Reads a value from an address in flash.
//
//  INPUTS
// 		unsigned long ulAddr - the address to read from
// 		int pnValue - pointer to store value read from flash
//
//	RETURN VALUE
//  	ERROR_CODE - value if any error occurs
//  	NO_ERR     - otherwise

static byte ReadFlash()
{
	asm("R0 = W[%0];" : : "p" (pSPI0_RDBR));
	Wait_For_SPIF();

	*pSPI0_TDBR = 0;
	Wait_For_RXS_SPIF();

	return *pSPI0_RDBR;	
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//----------- W r i t e F l a s h ( ) ----------//
//
//  PURPOSE
//  	Write a value to an address in flash.
//
//  INPUTS
//	 	unsigned long  ulAddr - address to write to
//		unsigned short nValue - value to write
//
//	RETURN VALUE
//  	ERROR_CODE - value if any error occurs
//  	NO_ERR     - otherwise

static void __WriteFlash(byte usValue )
{
	*pSPI0_TDBR = usValue;
	
	Wait_For_SPIF();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//////////////////////////////////////////////////////////////
// int ReadStatusRegister(void)
//
// Returns the 8-bit value of the status register.
// Inputs - none
// returns- second location of status_register[2],
//         first location is garbage.
// Core sends the command
//
//////////////////////////////////////////////////////////////

static byte ReadStatusRegister(void)
{
	SetupSPI(); // Turn on the SPI

	__WriteFlash(SPI_RDSR);

	byte usStatus = ReadFlash();

	SPI_OFF();		// Turn off the SPI

	return usStatus;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//////////////////////////////////////////////////////////////
// Wait_For_WEL(void)
//
// Polls the WEL (Write Enable Latch) bit of the Flash's status
// register.
// Inputs - none
// returns- none
//
//////////////////////////////////////////////////////////////

static ERROR_CODE Wait_For_WEL(void)
{
	volatile int n, i;
	char status_register = 0;
	ERROR_CODE ErrorCode = NO_ERR;	// tells us if there was an error erasing flash

	for(i = 0; i < 35; i++)
	{
		status_register = ReadStatusRegister();
		if( (status_register & WEL) )
		{
			ErrorCode = NO_ERR;	// tells us if there was an error erasing flash
			break;
		}

		for(n=0; n<DELAY; n++)
			asm("nop;");
		ErrorCode = POLL_TIMEOUT;	// Time out error

		*pWDOG_STAT = 0;
	}


	return ErrorCode;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//////////////////////////////////////////////////////////////
// Wait_For_Status(void)
//
// Polls the Status Register of the Flash's status
// register until the Flash is finished with its access. Accesses
// that are affected by a latency are Page_Program, Sector_Erase,
// and Block_Erase.
// Inputs - Statusbit
// returns- none
//
//////////////////////////////////////////////////////////////

static ERROR_CODE Wait_For_Status( char Statusbit )
{
	volatile int n, i;
	char status_register = 0xFF;
	ERROR_CODE ErrorCode = NO_ERR;	// tells us if there was an error erasing flash

	for(i = 0; i < TIMEOUT; i++)
	{
		status_register = ReadStatusRegister();
		if( !(status_register & Statusbit) )
		{
			ErrorCode = NO_ERR;	// tells us if there was an error erasing flash
			break;
		}

		for(n=0; n<DELAY; n++)
			asm("nop;");
		ErrorCode = POLL_TIMEOUT;	// Time out error

		*pWDOG_STAT = 0;
	};


	return ErrorCode;

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//////////////////////////////////////////////////////////////
// void SendSingleCommand( const int iCommand )
//
// Sends a single command to the SPI flash
// inputs - the 8-bit command to send
// returns- none
//
//////////////////////////////////////////////////////////////
static void __SendSingleCommand( const int iCommand )
{
	volatile int n;

	//turns on the SPI in single write mode
	SetupSPI();

	__WriteFlash(iCommand);

	//The SPI will be turned off
	SPI_OFF();

	//Pause before continuing
	for(n=0; n<DELAY; n++)
	{
		asm("nop;");
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//////////////////////////////////////////////////////////////
// Sets up the SPI for mode specified in spi_setting
// Inputs - spi_setting
// returns- none
//////////////////////////////////////////////////////////////
static void SetupSPI()
{
    volatile int i;

	/* PF8 - SPI0_SSEL2 */

	*pPORTF_FER   |= (PF13 | PF14 | PF15);
	*pPORTF_FER   &= ~(PF8);
	*pPORTF_MUX   &= ~(PF13 | PF14 | PF15);
   	*pPORTFIO_SET = PF8;
  	*pPORTFIO_DIR |= PF8;
   	*pPORTFIO_SET = PF8;

 //  	for(i=0; i<DELAY; i++)
	//{
		asm("nop;");
		asm("nop;");
		asm("nop;");
		asm("nop;");
		asm("nop;");
//	}

	*pSPI0_BAUD = BAUD_RATE_DIVISOR;
	*pSPI0_CTL = COMMON_SPI_SETTINGS|TIMOD01;	
	*pPORTFIO_CLEAR = PF8;

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//////////////////////////////////////////////////////////////
// Turns off the SPI
// Inputs - none
// returns- none
//
//////////////////////////////////////////////////////////////

static void SPI_OFF(void)
{
	volatile int i;

	*pPORTFIO_SET = PF8;
	*pSPI0_CTL = CPHA|CPOL;	// disable SPI
	*pSPI0_BAUD = 0;

	
	//for(i=0; i<DELAY; i++)
	//{
		asm("nop;");
		asm("nop;");
		asm("nop;");
		asm("nop;");
		asm("nop;");
//	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FlashUpdate()
{
	static RTM32 tm;

	//static u32 timeout = 0;

	static Req *request = 0;

	switch (flashState)
	{
		case FLASH_STATE_WAIT:

			if (request != 0)
			{
				FreeReq(request);
			};

			request = readyReq.Get();

			if (request != 0)
			{
				flashState = FLASH_STATE_WRITE_START;
			};

			break;

		case FLASH_STATE_ERASE_START:

			GlobalUnProtect();
			GlobalUnProtect();

			CmdWriteEnable();

			tm.Reset();

			flashState = FLASH_STATE_ERASE_WAIT;

			break;

		case FLASH_STATE_ERASE_WAIT:
		{ 
			byte st = ReadStatusRegister();

			if ((st & RDY_BSY) == 0 && (st & WEL) != 0)
			{
				lastError = NO_ERR;

				CmdEraseSector(lastErasedBlock);

				tm.Reset();

				flashState = FLASH_STATE_ERASE_WAIT_2;
			}
			else if (tm.Check(MS2RT(10)))
			{
				lastError = POLL_TIMEOUT;
				flashState = FLASH_STATE_WAIT;
			}; 

			break;
		};

		case FLASH_STATE_ERASE_WAIT_2:
		{
			byte st = ReadStatusRegister();

			if ((st & RDY_BSY) == 0)
			{
				if (st & EPE)
				{
					lastError = ERROR_ERASE;
					flashState = FLASH_STATE_WAIT;
				}
				else
				{
					lastError = NO_ERR;

					ReadAsyncDMA(bufsect, (u32)lastErasedBlock*SECTOR_SIZE, sizeof(bufsect));

					flashState = FLASH_STATE_ERASE_CHECK;
				};
			}
			else if (tm.Check(MS2RT(1000)))
			{
				lastError = POLL_TIMEOUT;
				flashState = FLASH_STATE_WAIT;
			};

			break;
		};

		case FLASH_STATE_ERASE_CHECK:

			if (CheckReadAsyncDMA())
			{
				bool c = false;

				for (u32 i = 0; i < ArraySize(bufsect); i++)
				{
					if (bufsect[i] != ~0)
					{
						c = true; break;
					};
				};

				if (c)
				{
					lastError = ERROR_ERASE;
					flashState = FLASH_STATE_WAIT;
				}
				else
				{
					flashState = (flashWritePtr != 0 && flashWriteLen != 0) ? FLASH_STATE_WRITE_PAGE : FLASH_STATE_ERASE_WAIT;
				};
			};

			break;

		case FLASH_STATE_WRITE_START:
		{
			ReqDsp06 &req = request->req;

			flashWriteAdr = FLASH_START_ADR + req.stAdr;
			flashWritePtr = req.data;
			flashWriteLen = req.len;

			u16 block = flashWriteAdr/SECTOR_SIZE;

			if (lastErasedBlock != block)
			{
				lastErasedBlock = block;

				flashState = FLASH_STATE_ERASE_START;

				break;
			};
		};

		case FLASH_STATE_WRITE_PAGE:

			CmdWriteEnable();

			tm.Reset();

			flashState = FLASH_STATE_WRITE_PAGE_2;

			break;

		case FLASH_STATE_WRITE_PAGE_2:
		{ 
			byte st = ReadStatusRegister();

			if ((st & RDY_BSY) == 0 && (st & WEL) != 0)
			{
				lastError = NO_ERR;

				WritePageAsync(flashWritePtr, flashWriteAdr, flashWriteLen);

				flashState = FLASH_STATE_WRITE_PAGE_3;
			}
			else if (tm.Check(MS2RT(10)))
			{
				lastError = POLL_TIMEOUT;
				flashState = FLASH_STATE_WAIT;
			}; 

			break;
		};

		case FLASH_STATE_WRITE_PAGE_3:

			if (CheckWriteAsyncDMA())
			{
				tm.Reset();

				flashState = FLASH_STATE_WRITE_PAGE_4;
			};

			break;

		case FLASH_STATE_WRITE_PAGE_4:
		{ 
			byte st = ReadStatusRegister();

			if ((st & RDY_BSY) == 0)
			{
				if (st & EPE)
				{
					lastError = ERROR_PROGRAM;
					flashState = FLASH_STATE_WAIT;
				}
				else
				{
					lastError = NO_ERR;

					flashState = FLASH_STATE_WAIT;
				};
			}
			else if (tm.Check(MS2RT(10)))
			{
				lastError = POLL_TIMEOUT;
				flashState = FLASH_STATE_WAIT;
			}; 

			break;
		};

		case FLASH_STATE_VERIFY_PAGE:



			break;


	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FlashInit()
{
	for (u32 i = 0; i < ArraySize(_req); i++)
	{
		freeReq.Add(_req+i);
	};

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Req* AllocReq()
{
	return freeReq.Get();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeReq(Req *req)
{
	freeReq.Add(req);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FlashWriteReq(Req *req)
{
	readyReq.Add(req);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
