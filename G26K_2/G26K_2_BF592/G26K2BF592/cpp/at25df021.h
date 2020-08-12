#ifndef AT25DF021_H__14_09_2016__08_29
#define AT25DF021_H__14_09_2016__08_29

#include "types.h"

#define FLASH_START_ADR 0 	

enum ERROR_CODE
{
	NO_ERR,					/* No Error */
	POLL_TIMEOUT,			/* Polling toggle bit failed */
	VERIFY_WRITE,			/* Verifying write to flash failed */
	INVALID_SECTOR,			/* Invalid Sector */
	INVALID_BLOCK,			/* Invalid Block */
	UNKNOWN_COMMAND,		/* Unknown Command */
	PROCESS_COMMAND_ERR,	/* Processing command */
	NOT_READ_ERROR,			/* Could not read memory from target */
	DRV_NOTAT_BREAK,		/* The drive was not at AFP_BreakReady */
	BUFFER_IS_NULL,			/* Could not allocate storage for the buffer */
	NO_ACCESS_SECTOR,		/* Cannot access the sector( could be locked or something is stored there that should not be touched ) */
	NUM_ERROR_CODES
};


#define BFLAG_FINAL         0x00008000   /* final block in stream */
#define BFLAG_FIRST         0x00004000   /* first block in stream */
#define BFLAG_INDIRECT      0x00002000   /* load data via intermediate buffer */
#define BFLAG_IGNORE        0x00001000   /* ignore block payload */
#define BFLAG_INIT          0x00000800   /* call initcode routine */
#define BFLAG_CALLBACK      0x00000400   /* call callback routine */
#define BFLAG_QUICKBOOT     0x00000200   /* boot block only when BFLAG_WAKEUP=0 */
#define BFLAG_FILL          0x00000100   /* fill memory with 32-bit argument value */
#define BFLAG_AUX           0x00000020   /* load auxiliary header -- reserved */
#define BFLAG_SAVE          0x00000010   /* save block on power down -- reserved */

struct BOOT_HEADER
{
	u32 blockCode;
	u32	targetAddress;
	u32 byteCount;
	u32 arg;
}; 

/* application definitions */
#define COMMON_SPI_SETTINGS (SPE|MSTR|CPOL|CPHA)  /* settings to the SPI_CTL */
#define TIMOD01 (0x01)                  /* sets the SPI to work with core instructions */

#define COMMON_SPI_DMA_SETTINGS (MSTR|CPOL|CPHA)  /* settings to the SPI_CTL */

#define BAUD_RATE_DIVISOR 	5
#define PE4 0x0010



extern ERROR_CODE at25df021_Read(byte *data, u32 stAdr, u16 count );
//extern ERROR_CODE at25df021_Read_DMA(byte *data, u32 stAdr, u16 count, bool *ready);
//extern ERROR_CODE at25df021_Read_IRQ(byte *data, u32 stAdr, u16 count, bool *ready);

extern ERROR_CODE at25df021_Write(const byte *data, u32 stAdr, u32 count, bool verify);

//extern ERROR_CODE at25df021_GetCRC16_IRQ(u32 stAdr, u16 count, bool *ready, u16 *crc);
extern u16 at25df021_GetCRC16(u32 stAdr, u16 count);


//extern ERROR_CODE GetCodes(int *pnManCode, int *pnDevCode);
//extern ERROR_CODE GetSectorStartEnd( unsigned long *ulStartOff, unsigned long *ulEndOff, int nSector );
//extern ERROR_CODE GetSectorNumber( unsigned long ulAddr, int *pnSector );
//extern ERROR_CODE EraseFlash();
extern ERROR_CODE EraseBlock(int nBlock);
//extern ERROR_CODE ResetFlash();
//extern u32 GetNumSectors();
//extern u32 GetSectorSize();



#endif // AT25DF021_H__14_09_2016__08_29 
