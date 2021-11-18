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

//static u32			bufsect[(SECTOR_SIZE+3)/4];
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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte bufpage[SPI_PAGE_SIZE] = "\n" "G26K_2_BOOT_BF592" "\n" __DATE__ "\n" __TIME__ "\n";

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


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

static void Delay(u32 us)
{
	for(u32 n=0; n < (us*NS2CCLK(10)); n++)
	{
		asm("nop;");
	};
};

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

	sec *= SECTOR_SIZE;

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
	*pSPI0_BAUD = BAUD_RATE_DIVISOR;
	*pPORTFIO_CLEAR = PF8;

	Delay(1);
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
	Delay(1);

	while((*pSPI0_STAT & SPIF) == 0) { *pWDOG_STAT = 0; };
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Wait_For_RXS_SPIF(void)
{
	Delay(1);

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

	Delay(10);

	while ((*pDMA5_IRQ_STATUS & DMA_DONE) == 0) { *pWDOG_STAT = 0; };

	Delay(10);

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

	Delay(10);

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

static void GlobalUnProtect()
{
	CmdWriteEnable();

	CmdWriteStatusReg(0);
}

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

	//if (ErrorCode == NO_ERR)
	//{
	//	ErrorCode == at25df021_Read(bufsect, ulSectStart, sizeof(bufsect));

	//	if (ErrorCode == NO_ERR)
	//	{
	//		for (u32 i = ArraySize(bufsect); i > 0; i--)
	//		{
	//			if (bufsect[i] != ~0) { ErrorCode = VERIFY_WRITE; break; };
	//		};
	//	};
	//};

 	// block erase should be complete
	return ErrorCode;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
		};

		Delay(DELAY);

		ErrorCode = POLL_TIMEOUT;	// Time out error

		*pWDOG_STAT = 0;
	};

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

		Delay(DELAY);

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
	SetupSPI();

	__WriteFlash(iCommand);

	SPI_OFF();

	Delay(1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//////////////////////////////////////////////////////////////
// Sets up the SPI for mode specified in spi_setting
// Inputs - spi_setting
// returns- none
//////////////////////////////////////////////////////////////
static void SetupSPI()
{
	*pSPI0_BAUD = BAUD_RATE_DIVISOR;
	*pSPI0_CTL = COMMON_SPI_SETTINGS|TIMOD01;	
	*pPORTFIO_CLEAR = PF8;

	Delay(1);
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

	Delay(1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FlashUpdate()
{
	static RTM32 tm;

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
				__breakpoint();
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
					__breakpoint();
					lastError = ERROR_ERASE;
					flashState = FLASH_STATE_WAIT;
				}
				else
				{
					lastError = NO_ERR;

					flashState = (flashWritePtr != 0 && flashWriteLen != 0) ? FLASH_STATE_WRITE_PAGE : FLASH_STATE_ERASE_WAIT;
				};
			}
			else if (tm.Check(MS2RT(1000)))
			{
				__breakpoint();
				lastError = POLL_TIMEOUT;
				flashState = FLASH_STATE_WAIT;
			};

			break;
		};

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
				__breakpoint();
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
					__breakpoint();
					lastError = ERROR_PROGRAM;
					flashState = FLASH_STATE_WAIT;
				}
				else
				{
					lastError = NO_ERR;

					ReadAsyncDMA(bufpage, flashWriteAdr, flashWriteLen);

					flashState = FLASH_STATE_VERIFY_PAGE;
				};
			}
			else if (tm.Check(MS2RT(10)))
			{
				__breakpoint();
				lastError = POLL_TIMEOUT;
				flashState = FLASH_STATE_WAIT;
			}; 

			break;
		};

		case FLASH_STATE_VERIFY_PAGE:

			if (CheckReadAsyncDMA())
			{
				bool c = false;

				for (u32 i = 0; i < flashWriteLen; i++)
				{
					if (flashWritePtr[i] != bufpage[i]) { c = true; break; };
				};

				if (c)
				{
					__breakpoint();

					lastError = VERIFY_WRITE;
				}
				else
				{
					lastError = NO_ERR;
				};

				flashWritePtr = 0;
				flashWriteLen = 0;

				flashState = FLASH_STATE_WAIT;
			};

			break;


	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool FlashBusy()
{
	return (flashState != FLASH_STATE_WAIT) || !readyReq.Empty();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FlashInit()
{
	for (u32 i = 0; i < ArraySize(_req); i++)
	{
		freeReq.Add(_req+i);
	};

	*pPORTF_FER   |= (PF13 | PF14 | PF15);
	*pPORTF_FER   &= ~(PF8);
	*pPORTF_MUX   &= ~(PF13 | PF14 | PF15);
   	*pPORTFIO_SET = PF8;
  	*pPORTFIO_DIR |= PF8;
   	*pPORTFIO_SET = PF8;
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
