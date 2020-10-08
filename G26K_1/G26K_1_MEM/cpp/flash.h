#ifndef FLASH_H__09_08_2019__08_17
#define FLASH_H__09_08_2019__08_17

#include "hardware.h"

//#include "rtc.h"
//#include "vector.h"
#include "trap_def.h"


#define FLWB_LEN (8192+32)
#define FLRB_LEN 1536

//#define NAND_MAX_CHIP		8
//#define NAND_CHIP_MASK		7
//#define NAND_CHIP_BITS		3
//#define NAND_COL_BITS		11
//#define NAND_BLOCK_BITS		13
//#define NAND_PAGE_BITS		6
//#define NAND_RAWPAGE_MASK	((1 << (NAND_PAGE_BITS + NAND_CHIP_BITS + NAND_BLOCK_BITS)) - 1)
//#define NAND_RAWBLOCK_MASK	((1 << (NAND_CHIP_BITS + NAND_BLOCK_BITS)) - 1)
//#define NAND_RAWADR_MASK	(((u64)1 << (NAND_COL_BITS + NAND_PAGE_BITS + NAND_CHIP_BITS + NAND_BLOCK_BITS)) - 1)

enum  
{
	FLASH_STATUS_NONE = 0,
	FLASH_STATUS_BUSY,				// занят неизвестной для компьютера операцией (например записью в память)
	FLASH_STATUS_STOP,
	FLASH_STATUS_PAUSE,
	FLASH_STATUS_RESUME,
	FLASH_STATUS_ERASE,
	FLASH_STATUS_UNERASE,
	FLASH_STATUS_READ_SESSION_START,
	FLASH_STATUS_READ_SESSION_IDLE,
	FLASH_STATUS_READ_SESSION_FIND, // поиск вектора по несвязанной области
	FLASH_STATUS_READ_SESSION_ERROR,
	FLASH_STATUS_READ_SESSION_READY,
	FLASH_STATUS_READ_VECTOR_START,
	FLASH_STATUS_READ_VECTOR_IDLE,
	FLASH_STATUS_READ_VECTOR_FIND, // поиск вектора по несвязанной области
	FLASH_STATUS_READ_VECTOR_ERROR,
	FLASH_STATUS_READ_VECTOR_READY
};


enum flash_save_repeat_type
{
	FLASH_SAVE_REPEAT_NONE = 0,
	FLASH_SAVE_REPEAT_NORMAL = 2,	// одна перезапись, вторая - в новый блок
	FLASH_SAVE_REPEAT_HIGH = 4,	// .... 3 в новый блок
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct FLADR
{
	union
	{
		struct
		{
			u64		col		: NAND_COL_BITS;
			u64 	page	: NAND_PAGE_BITS;
			u64		chip	: NAND_CHIP_BITS;
			u64 	block	: NAND_BLOCK_BITS;

			u64		overflow : (64-(NAND_COL_BITS+NAND_PAGE_BITS+NAND_CHIP_BITS+NAND_BLOCK_BITS));
		};

		u64	raw;
	};

	enum { pg = (1<<NAND_COL_BITS) };
//	u32		rawpage;

//	const NandMemSize& sz;

	inline void operator=(const FLADR &a) { raw = a.raw; }

	FLADR() : raw(0) {}
	FLADR(u32 bl, u16 pg, u16 cl, byte ch) : block(bl), page(pg), col(cl), chip(ch) {}
	FLADR(u32 pg) : col(0) { SetRawPage(pg); }

	u32		GetRawPage() { return (raw & NAND_RAWADR_MASK) >> NAND_COL_BITS; }

	void	SetRawPage(u32 p) { raw = (u64)(p & NAND_RAWPAGE_MASK) << NAND_COL_BITS; };

	u32		GetRawBlock() { return (raw & NAND_RAWADR_MASK) >> (NAND_COL_BITS+NAND_PAGE_BITS); }

	void	SetRawBlock(u32 b) { raw = (u64)(b & NAND_RAWBLOCK_MASK) << (NAND_COL_BITS+NAND_PAGE_BITS); };

	u64		GetRawAdr()	{ return raw & NAND_RAWADR_MASK; };
	void	SetRawAdr(u64 a) { raw  = a & NAND_RAWADR_MASK; };

	void	NextPage()	{ col = 0; raw += 1 << NAND_COL_BITS; raw += NAND_GetMemSize()->chipOffsetNext[chip]; }
	void	NextBlock()	{ col = 0;page = 0;raw += 1 << (NAND_COL_BITS + NAND_PAGE_BITS); raw += NAND_GetMemSize()->chipOffsetNext[chip];}
	void	PrevPage()	{ raw -= 1 << NAND_COL_BITS; col = 0; raw -= NAND_GetMemSize()->chipOffsetPrev[chip]; }
	void	PrevBlock()	{ raw -= 1 << (NAND_COL_BITS + NAND_PAGE_BITS);col = 0;page = 0; raw -= NAND_GetMemSize()->chipOffsetPrev[chip];}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct VecData
{
	__packed struct Hdr
	{
		u16 		session;
		u16 		device;
		RTC_type	rtc;
		byte		flags;		// флажки
		u64			prVecAdr;	// адрес предыдущего блядовектора
		u16			dataLen;
		u16			crc;		// CRC16 всей этой хрени
	} h;

	byte		data[FLWB_LEN]; // Последние 2 байта CRC16
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct FLWB
{
	FLWB *next;

	//void	*data;
	u16 	dataLen;

	VecData vd;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct FLRB
{
	FLRB *next;

	bool	ready;
	bool	vecStart; // Найти начало вектора и считать hdr

	bool	useAdr;
	u64		adr;

	VecData::Hdr hdr;

	u16		maxLen;
	u16		len;

	byte	*data;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct FileDsc
{
	u16			session;	// file number
	i64			size;		//если 0 то сессия немного порченная
	RTC_type	start_rtc;	//если 0 то сессия немного порченная
	RTC_type	stop_rtc;  
	u32			startPage; 
	u32			lastPage;
	byte		flags;
};	

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct SpareArea
{
	u16		validPage;	// 0xFFFF - good page
	u16		validBlock;	// 0xFFFF - good block
	u16		badPages;	// 0xFFFF - all pages good

	u16		file;  		// file number
//	u32		lpn;		// logic page number
	u32		start;		// start page of file
	u32		fpn;		// file page number
	u32		prev;		// start page of previos file
	u32		rawPage;	// raw page num

	u32		vectorCount; // vector count in file

	u16		vecFstOff;	// first vector header offset. 0xFFFF - no vector header 
	u16		vecFstLen;	// first vector lenght. 0 - no vector

	u16		vecLstOff;	// last vector header offset. 0xFFFF - no vector header 
	u16		vecLstLen;	// last vector lenght. 0 - no vector

	//u32		vecPrPg;	// previos vector raw page
	//u16		vecPrOff;	// previos vector offset

	u16		fbb;		// file bad blocks count
	u16		fbp;		// file bad pages count

	byte	chipMask;	

	u16		crc;

	enum { CRC_SKIP = 6 };
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern FLWB*	AllocFlashWriteBuffer();
extern FLRB*	AllocFlashReadBuffer();
extern void		FreeFlashWriteBuffer(FLWB* b);
extern void		FreeFlashReadBuffer(FLRB* b);
extern bool		RequestFlashRead(FLRB* b);
extern bool		RequestFlashWrite(FLWB* b, u16 devID);

extern void NAND_Idle();
extern void NAND_FullErase();
//extern void NAND_NextSession();

inline u16 GetDeviceID() { extern u16 deviceID; return deviceID; }

extern void StartSendSession();
extern FileDsc* GetSessionInfo(u16 session, u64 adr);

//extern const SessionInfo* GetLastSessionInfo();


/*************************/
extern void FLASH_Init();
extern bool FLASH_Reset();
extern void FLASH_Update();
extern bool FLASH_Busy();

extern bool FLASH_WriteEnable();
extern void FLASH_WriteDisable();
extern byte FLASH_Status();

/*****************************************************************************/
//extern bool FLASH_Erase_Full();
//extern bool FLASH_UnErase_Full();
//extern bool FLASH_Write_Vector(u16 session, u16 device, RTC_type rtc, byte flags, byte *data, u16 size, flash_save_repeat_type repeat);
//extern bool FLASH_Read_Vector(u64 adr, u16 *size, bool *ready, byte **vector);

extern u32 FLASH_Vectors_Errors_Get();
extern u32 FLASH_Vectors_Saved_Get();
extern u32 FLASH_Vectors_Recieved_Get();
extern u32 FLASH_Vectors_Rejected_Get();
extern u32 FLASH_Session_Get();


//extern void FLASH_Vectors_Errors_Reset();
//extern void FLASH_Vectors_Saved_Reset();

extern u64 FLASH_Current_Adress_Get();
extern u64 FLASH_Full_Size_Get();
extern u16 FLASH_Chip_Mask_Get();
extern u64 FLASH_Used_Size_Get();

//extern i64 FLASH_Empty_Size_Get();


/*****************************************************************************/

#endif // FLASH_H__09_08_2019__08_17
