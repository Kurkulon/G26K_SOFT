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

#include "at25df021.h"

#include "types.h"
#include "core.h"
#include "CRC16.h"

#pragma optimize_for_speed

#define NUM_SECTORS 	64			/* number of sectors in the flash device */
#define SECTOR_SIZE		4096

static char 	*pFlashDesc =		"Atmel AT25DF021";
static char 	*pDeviceCompany	=	"Atmel Corporation";

static int		gNumSectors = NUM_SECTORS;

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
#define WIP					(0x1)	//Check the write in progress bit of the SPI status register
#define WEL					(0x2)	//Check the write enable bit of the SPI status register
#define SPI_UPS				(0x39)  //Unprotect Sector 
#define SPI_PRS				(0x36)  //Protect Sector 


#define SPI_PAGE_SIZE		(256)
//#define SPI_SECTORS		    (512)
//#define SPI_SECTOR_SIZE		(4224)
//#define SPI_SECTOR_DIFF		(3968)
//#define PAGE_BITS			(10)
//#define PAGE_SIZE_DIFF		(496)

#define DELAY				15
#define TIMEOUT				35000

//char			SPI_Page_Buffer[SPI_PAGE_SIZE];
//int 			SPI_Page_Index = 0;
//char            SPI_Status;


/* function prototypes */
static ERROR_CODE SetupForFlash();
static ERROR_CODE Wait_For_nStatus(void);
ERROR_CODE Wait_For_Status( char Statusbit );
static ERROR_CODE Wait_For_WEL(void);
extern void SetupSPI();
extern void SPI_OFF(void);
void SendSingleCommand( const int iCommand );
//unsigned long DataFlashAddress (unsigned long address);

static ERROR_CODE PollToggleBit(void);
static byte ReadFlash();
static void WriteFlash(byte usValue);
static unsigned long GetFlashStartAddress( unsigned long ulAddr);
static void GlobalUnProtect();


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//u32 GetNumSectors()
//{
//	return gNumSectors;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//u32 GetSectorSize()
//{
//	return SECTOR_SIZE;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void SetupSPIDMA()
//{
//    volatile int i;
//
//	/* PF8 - SPI0_SSEL2 */
//
//	*pPORTF_FER   |= (PF13 | PF14 | PF15);
//	*pPORTF_FER   &= ~(PF8);
//	*pPORTF_MUX   &= ~(PF13 | PF14 | PF15);
//   	*pPORTFIO_SET = PF8;
//  	*pPORTFIO_DIR |= PF8;
//   	*pPORTFIO_SET = PF8;
//
// //  	for(i=0; i<DELAY; i++)
//	//{
//		//asm("nop;");
//		//asm("nop;");
//		//asm("nop;");
//		//asm("nop;");
//		//asm("nop;");
////	}
//
//	*pSPI0_BAUD = BAUD_RATE_DIVISOR;
//	*pPORTFIO_CLEAR = PF8;
//
//}

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
inline void Wait_For_SPIF(void)
{
	volatile int n;

	for(n=0; n<DELAY; n++)
	{
		asm("nop;");
	}

	while((*pSPI0_STAT & SPIF) == 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline void Wait_For_RXS_SPIF(void)
{
	volatile int n;

	for(n=0; n<DELAY; n++)
	{
		asm("nop;");
	}

	while((*pSPI0_STAT & (SPIF|RXS)) != (SPIF|RXS));
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//inline u16 WaitReadSPI0()
//{
//	while ((*pSPI0_STAT & RXS) == 0); 
//
//	return *pSPI0_RDBR;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//inline void WriteSyncDMA(byte *data, u16 count)
//{
//	*pSPI0_CTL = COMMON_SPI_DMA_SETTINGS|3;	
//
//	*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8/*|SYNC*/;
//	*pDMA5_START_ADDR = data;
//	*pDMA5_X_COUNT = count;
//	*pDMA5_X_MODIFY = 1;
//
//	*pDMA5_CONFIG |= DMAEN;
//	*pSPI0_CTL |= SPE;
//
////	while (*pDMA5_IRQ_STATUS & DMA_RUN);
//
//	while ((*pDMA5_IRQ_STATUS & DMA_DONE) == 0);
//
////	while ((*pDMA5_IRQ_STATUS & (DMA_RUN|DMA_DONE)) != DMA_DONE);
//
//	while ((*pSPI0_STAT & SPIF) == 0 || (*pSPI0_STAT & TXS));
//
//	*pDMA5_IRQ_STATUS = 1;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//inline void ReadSyncDMA(byte *data, u16 count)
//{
//	*pSPI0_CTL = COMMON_SPI_DMA_SETTINGS|2;
//	*pDMA5_CONFIG = FLOW_STOP|DI_EN|WDSIZE_8|WNR|SYNC;
//
//	*pDMA5_START_ADDR = data;
//	*pDMA5_X_COUNT = count;
//	*pDMA5_X_MODIFY = 1;
//
//	*pDMA5_CONFIG |= DMAEN;
//	*pSPI0_CTL |= SPE;
//
//	while ((*pDMA5_IRQ_STATUS & DMA_DONE) == 0);
//
//	*pDMA5_IRQ_STATUS = 1;
//}

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

//static u16 readCountSPI0 = 0;
//static byte *readDataSPI0 = 0;
//static bool *readReadySPI0 = 0;
//
//static u16 writeCountSPI0 = 0;
//static byte *writeDataSPI0 = 0;
//static bool *writeReadySPI0 = 0;
//
//static DataCRC CRC_SPI0 = { 0xFFFF };
//
//static u16 *ptrCRC_SPI0 = 0;

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

ERROR_CODE at25df021_Read(byte *data, u32 stAdr, u16 count )
{
    ERROR_CODE Result = NO_ERR;

	SetupSPI();

        /* send the bulk erase command to the flash */
    WriteFlash(SPI_FAST_READ);
    WriteFlash((stAdr) >> 16);
    WriteFlash((stAdr) >> 8);
    WriteFlash(stAdr);
    WriteFlash(0);

	for ( ; count > 0; count--)
	{
		*data++ = ReadFlash();
	};

    SPI_OFF();

	return(Result);
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

//u16 at25df021_GetCRC16(u32 stAdr, u16 count)
//{
//	DataCRC crc;
//
//	crc.w = 0xFFFF;
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
//	*pSPI0_CTL = SPE|COMMON_SPI_DMA_SETTINGS|0;
//
//	t = *pSPI0_RDBR;
//
//	for ( ; count > 0; count--)
//	{
//		while ((*pSPI0_STAT & RXS) == 0);
//
//		t = *pSPI0_RDBR;
//
//		crc.w = tableCRC[crc.b[0] ^ t] ^ crc.b[1];
//	};
//	
//	*pSPI0_CTL = 0;
//
//	SPI_OFF();
//
//	return crc.w;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

ERROR_CODE WritePage(byte *data, u32 stAdr, u16 count )
{
    ERROR_CODE Result = NO_ERR;

	if ((stAdr & 0xFF) != 0 || count > 256 || count == 0)
	{
		return INVALID_BLOCK;
	};

    SendSingleCommand(SPI_WREN);

    Result = Wait_For_WEL();

    if( POLL_TIMEOUT == Result )
	{
		return Result;
	}
    else
    {
        SetupSPI();

        /* send the bulk erase command to the flash */
        WriteFlash(SPI_PP );
        WriteFlash((stAdr) >> 16);
        WriteFlash((stAdr) >> 8);
        WriteFlash(stAdr);

		for ( ; count > 0; count--)
		{
	        WriteFlash(*data++);
		};

        SPI_OFF();
    };

	return Wait_For_Status(WIP);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//ERROR_CODE VerifyPage(byte *data, u32 stAdr, u16 count )
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
//	*pSPI0_CTL = SPE|COMMON_SPI_DMA_SETTINGS|0;
//
//	t = *pSPI0_RDBR;
//
//	for ( ; count > 0; count--)
//	{
//		t = WaitReadSPI0(); //while ((*pSPI0_STAT & RXS) == 0);
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

ERROR_CODE VerifyPage(byte *data, u32 stAdr, u16 count )
{
    ERROR_CODE Result = NO_ERR;

	SetupSPI();

        /* send the bulk erase command to the flash */
    WriteFlash(SPI_FAST_READ);
    WriteFlash((stAdr) >> 16);
    WriteFlash((stAdr) >> 8);
    WriteFlash(stAdr);
    WriteFlash(0);

	for ( ; count > 0; count--)
	{
		if (*data++ != ReadFlash())
		{
			Result = VERIFY_WRITE;
			break;
		};
	};

    SPI_OFF();

	return(Result);
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

ERROR_CODE at25df021_Write(byte *data, u32 stAdr, u32 count, bool verify)
{
    ERROR_CODE Result = NO_ERR;

	u32 c;

	while (count > 0)
	{
		u16 c = (count >= 256) ? 256 : count;

		count -= c;

		Result = WritePage(data, stAdr, c);

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

//ERROR_CODE ResetFlash()
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
	ERROR_CODE 	  ErrorCode   = NO_ERR;	

	SetupSPI();

	WriteFlash(SPI_WREN );

	SPI_OFF();

	SetupSPI();

	WriteFlash(SPI_WRSR);
	WriteFlash(0);

	SPI_OFF();
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

//ERROR_CODE UnProtectBlock(u32 adr)
//{
//	ERROR_CODE 	  ErrorCode   = NO_ERR;		//tells us if there was an error erasing flash
//
//	SetupSPI();
//
//	// send the write enable instruction
//	WriteFlash(SPI_WREN );
//
//	SPI_OFF();
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

ERROR_CODE EraseBlock(u32 adr)
{

	ERROR_CODE 	  ErrorCode   = NO_ERR;		//tells us if there was an error erasing flash
 	unsigned long ulSectStart = 0x0;		//stores the sector start offset
 	unsigned long ulSectEnd   = 0x0;		//stores the sector end offset(however we do not use it here)

	// Get the sector start offset
	// we get the end offset too however we do not actually use it for Erase sector
//	GetSectorStartEnd( &ulSectStart, &ulSectEnd, nBlock );

	GlobalUnProtect();
	GlobalUnProtect();

	SetupSPI();

	// send the write enable instruction
	WriteFlash(SPI_WREN );

	SPI_OFF();

	SetupSPI();

	//send the erase block command to the flash
	WriteFlash(SPI_SE );

	WriteFlash(adr>>16);
	WriteFlash(adr>>8);
	WriteFlash(adr);

	SPI_OFF();

	// Poll the status register to check the Write in Progress bit
	// Sector erase takes time
	ErrorCode = Wait_For_Status(WIP);



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


//ERROR_CODE PollToggleBit(void)
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

//ERROR_CODE GetCodes(int *pnManCode, int *pnDevCode)
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

//ERROR_CODE GetSectorNumber( unsigned long ulAddr, int *pnSector )
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

//ERROR_CODE GetSectorStartEnd( unsigned long *ulStartOff, unsigned long *ulEndOff, int nSector )
//{
//	u32 ulSectorSize = SECTOR_SIZE;
//
//	if( ( nSector >= 0 ) && ( nSector < gNumSectors ) ) // 32 sectors
//	{
//		*ulStartOff = nSector * ulSectorSize;
//		*ulEndOff = ( (*ulStartOff) + ulSectorSize - 1 );
//	}
//	else
//	{
//		return INVALID_SECTOR;
//	};
//
//	// ok
//	return NO_ERR;
//}

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

//unsigned long GetFlashStartAddress( unsigned long ulAddr)
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

byte ReadFlash()
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

void WriteFlash(byte usValue )
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

byte ReadStatusRegister(void)
{
	SetupSPI(); // Turn on the SPI

	WriteFlash(SPI_RDSR);

	//*pSPI0_TDBR = (SPI_RDSR);

	//while((*pSPI0_STAT & SPIF) == 0);

	byte usStatus = ReadFlash(); //*pSPI0_RDBR;

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

ERROR_CODE Wait_For_WEL(void)
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

ERROR_CODE Wait_For_Status( char Statusbit )
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

		for(n=0; n<DELAY; n++) asm("nop;");

		ErrorCode = POLL_TIMEOUT;	// Time out error

		ResetWDT();
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
void SendSingleCommand( const int iCommand )
{
	volatile int n;

	//turns on the SPI in single write mode
	SetupSPI();

	//sends the actual command to the SPI TX register

	*pSPI0_TDBR = iCommand;

	//The SPI status register will be polled to check the SPIF bit
	Wait_For_SPIF();

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
void SetupSPI()
{
    volatile int i;

	/* PF8 - SPI0_SSEL2 */

	*pPORTF_FER   |= (PF13 | PF14 | PF15);
	*pPORTF_FER   &= ~(PF8);
	*pPORTF_MUX   &= ~(PF13 | PF14 | PF15);
   	*pPORTFIO_SET = PF8;
  	*pPORTFIO_DIR |= PF8;
   	*pPORTFIO_SET = PF8;

   	for(i=0; i<DELAY; i++)
	{
		asm("nop;");
		asm("nop;");
		asm("nop;");
		asm("nop;");
		asm("nop;");
	}

	*pSPI0_BAUD = BAUD_RATE_DIVISOR;
	*pPORTFIO_CLEAR = PF8;

	*pSPI0_CTL = COMMON_SPI_SETTINGS|TIMOD01;	
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//////////////////////////////////////////////////////////////
// Turns off the SPI
// Inputs - none
// returns- none
//
//////////////////////////////////////////////////////////////

void SPI_OFF(void)
{
	volatile int i;

	*pPORTFIO_SET = PF8;
	*pSPI0_CTL = CPHA|CPOL;	// disable SPI
	*pSPI0_BAUD = 0;

	
	for(i=0; i<DELAY; i++)
	{
		asm("nop;");
		asm("nop;");
		asm("nop;");
		asm("nop;");
		asm("nop;");
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//////////////////////////////////////////////////////////////
// Wait_For_nStatus(void)
//
// Polls the WEL (Write Enable Latch) bit of the Flash's status
// register.
// Inputs - none
// returns- none
//
//////////////////////////////////////////////////////////////
//ERROR_CODE Wait_For_nStatus(void)
//{
//	volatile int i;
//	char status_register = 0;
//	ERROR_CODE ErrorCode = NO_ERR;	// tells us if there was an error erasing flash
//
//	for(i = 0; i < 500; i++)
//	{
//		status_register = ReadStatusRegister();
//		if( (status_register & WEL) )
//		{
//			ErrorCode = NO_ERR;
//			return ErrorCode;
//		}
//		ErrorCode = POLL_TIMEOUT;	// Time out error
//	}
//
//	return ErrorCode;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/////////////////////////////////////////////////////////////////
// unsigned long DataFlashAddress()
//
// Translates a binary address into the appropriate dataflash address.
//
// Inputs: unsigned long address - The binary address supplied
/////////////////////////////////////////////////////////////////
//unsigned long DataFlashAddress (unsigned long address)
//{
//	// Determine the page that it is on
//	unsigned long pageAddress = address / SPI_PAGE_SIZE;
//
//	// Determine the byte offset within the page
//	unsigned long bitAddress = address % SPI_PAGE_SIZE;
//
//	// Shift the page address the correct number of bits
//	pageAddress = pageAddress << PAGE_BITS;
//
//	// Return the combined Page and Byte offset address
//	return (pageAddress | bitAddress);
//}
//
