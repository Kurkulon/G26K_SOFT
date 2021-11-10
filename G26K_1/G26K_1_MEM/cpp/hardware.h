#ifndef HARDWARE_H__23_12_2013__11_37
#define HARDWARE_H__23_12_2013__11_37

#include "types.h"
//#include "core.h"
#include "time.h"

//#define NAND_SAMSUNG
#define NAND_MICRON

#define NAND_CHIP_BITS		3
#define NAND_MAX_CHIP		(1<<3)
#define NAND_CHIP_MASK		(NAND_MAX_CHIP-1)

#define K9K8_CHIP_BITS		3
#define K9K8_MAX_CHIP		(1<<3)
#define K9K8_CHIP_MASK		(K9K8_MAX_CHIP-1)
#define K9K8_COL_BITS		11
#define K9K8_BLOCK_BITS		13
#define K9K8_PAGE_BITS		6
#define K9K8_SPARE_SIZE		64
#define K9K8_PAGE_SIZE		(1 << K9K8_COL_BITS)
#define K9K8_RAWPAGE_MASK	((1 << (K9K8_PAGE_BITS + K9K8_CHIP_BITS + K9K8_BLOCK_BITS)) - 1)
#define K9K8_RAWBLOCK_MASK	((1 << (K9K8_CHIP_BITS + K9K8_BLOCK_BITS)) - 1)
#define K9K8_RAWADR_MASK	(((u64)1 << (K9K8_COL_BITS + K9K8_PAGE_BITS + K9K8_CHIP_BITS + K9K8_BLOCK_BITS)) - 1)

#define MT29_CHIP_BITS		3
#define MT29_MAX_CHIP		(1<<3)
#define MT29_CHIP_MASK		(MT29_MAX_CHIP-1)
#define MT29_COL_BITS		13
#define MT29_BLOCK_BITS		12
#define MT29_PAGE_BITS		7
#define MT29_SPARE_SIZE		448
#define MT29_PAGE_SIZE		(1 << MT29_COL_BITS)
#define MT29_RAWPAGE_MASK	((1 << (MT29_PAGE_BITS + MT29_CHIP_BITS + MT29_BLOCK_BITS)) - 1)
#define MT29_RAWBLOCK_MASK	((1 << (MT29_CHIP_BITS + MT29_BLOCK_BITS)) - 1)
#define MT29_RAWADR_MASK	(((u64)1 << (MT29_COL_BITS + MT29_PAGE_BITS + MT29_CHIP_BITS + MT29_BLOCK_BITS)) - 1)

#ifdef NAND_SAMSUNG

//#define NAND_MAX_CHIP			K9K8_MAX_CHIP	
//#define NAND_CHIP_MASK		K9K8_CHIP_MASK	
//#define NAND_CHIP_BITS		K9K8_CHIP_BITS	
#define NAND_COL_BITS		K9K8_COL_BITS			
#define NAND_BLOCK_BITS		K9K8_BLOCK_BITS		
#define NAND_PAGE_BITS		K9K8_PAGE_BITS		
#define NAND_SPARE_SIZE		K9K8_SPARE_SIZE		
#define NAND_PAGE_SIZE		K9K8_PAGE_SIZE		
#define NAND_RAWPAGE_MASK	K9K8_RAWPAGE_MASK	
#define NAND_RAWBLOCK_MASK	K9K8_RAWBLOCK_MASK	
#define NAND_RAWADR_MASK	K9K8_RAWADR_MASK	
#define nandType			K9K8 

#elif defined(NAND_MICRON)

//#define NAND_MAX_CHIP			MT29_MAX_CHIP
//#define NAND_CHIP_MASK		MT29_CHIP_MASK
//#define NAND_CHIP_BITS		MT29_CHIP_BITS
#define NAND_COL_BITS		MT29_COL_BITS		
#define NAND_BLOCK_BITS		MT29_BLOCK_BITS		
#define NAND_PAGE_BITS		MT29_PAGE_BITS		
#define NAND_SPARE_SIZE		MT29_SPARE_SIZE		
#define NAND_PAGE_SIZE		MT29_PAGE_SIZE		
#define NAND_RAWPAGE_MASK	MT29_RAWPAGE_MASK	
#define NAND_RAWBLOCK_MASK	MT29_RAWBLOCK_MASK	
#define NAND_RAWADR_MASK	MT29_RAWADR_MASK	
#define nandType			MT29 

#endif

#define FRAM_SPI_MAINVARS_ADR 0
#define FRAM_SPI_SESSIONS_ADR 0x200

#define FRAM_I2C_MAINVARS_ADR 0
#define FRAM_I2C_SESSIONS_ADR 0x200

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct NandMemSize
{
 	u64 ch;	// chip
 	u64 fl;	// full
// 	u32 bl;	// block
////	u32 row;
//	u16 pg;	// page
	u16 mask;
//	byte shPg; //(1 << x)
//	byte shBl; //(1 << x)
//	byte shCh;
//	//byte shRow;
//
//	byte bitCol;
//	byte bitPage; // 
//	byte bitBlock;
//
//	u16	pagesInBlock;
//
//
//	u16		maskPage;
//	u32		maskBlock;

	//byte	chipValidNext[NAND_MAX_CHIP]; // ≈сли чип битый, то по индексу находитс€ следующий хороший чип
	//byte	chipValidPrev[NAND_MAX_CHIP]; // ≈сли чип битый, то по индексу находитс€ предыдущий хороший чип

	//u32		chipOffsetNext[NAND_MAX_CHIP]; // ≈сли чип битый, то по индексу находитс€ смещение адреса на следующий хороший чип
	//u32		chipOffsetPrev[NAND_MAX_CHIP]; // ≈сли чип битый, то по индексу находитс€ смещение адреса на предыдущий хороший чип

	byte	chipDataBusMask[NAND_MAX_CHIP]; // ≈сли проблема по линии данных, то соответствующи бит равен 0

};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct FLADR
{
	union
	{
		struct K9K8
		{
			u64		col		: K9K8_COL_BITS;
			u64 	page	: K9K8_PAGE_BITS;
			u64		chip	: K9K8_CHIP_BITS;
			u64 	block	: K9K8_BLOCK_BITS;
			//u64		overflow : (64-(K9K8_COL_BITS+K9K8_PAGE_BITS+NAND_CHIP_BITS+K9K8_BLOCK_BITS));
		} K9K8;

		struct MT29
		{
			u64		col		: MT29_COL_BITS;
			u64 	page	: MT29_PAGE_BITS;
			u64		chip	: MT29_CHIP_BITS;
			u64 	block	: MT29_BLOCK_BITS;
			//u64		overflow : (64-(MT29_COL_BITS+MT29_PAGE_BITS+NAND_CHIP_BITS+MT29_BLOCK_BITS));
		} MT29;

		u32 DW[2];
		u64	raw;
	};

	static byte COL_BITS;
	static byte PAGE_BITS;		
	static byte BLOCK_BITS;		

	//static byte COL_OFF;	// = 0
	static byte PAGE_OFF;	// = COL_BITS;
	static byte CHIP_OFF;	// = PAGE_OFF + PAGE_BITS
	static byte BLOCK_OFF;	// = CHIP_OFF + NAND_CHIP_BITS 		

	static u32 COL_MASK;
	static u32 PAGE_MASK;		
	static u32 CHIP_MASK;		
	static u32 BLOCK_MASK;		

	static u32 RAWPAGE_MASK;	
	static u32 RAWBLOCK_MASK;	

	static u64 RAWADR_MASK;

	static u32 pg; //enum { pg = (1<<NAND_COL_BITS) };
//	u32		rawpage;

//	const NandMemSize& sz;

	static byte	chipValidNext[NAND_MAX_CHIP]; // ≈сли чип битый, то по индексу находитс€ следующий хороший чип
	static byte	chipValidPrev[NAND_MAX_CHIP]; // ≈сли чип битый, то по индексу находитс€ предыдущий хороший чип

	static u32	chipOffsetNext[NAND_MAX_CHIP]; // ≈сли чип битый, то по индексу находитс€ смещение адреса на следующий хороший чип
	static u32	chipOffsetPrev[NAND_MAX_CHIP]; // ≈сли чип битый, то по индексу находитс€ смещение адреса на предыдущий хороший чип

	inline void operator=(const FLADR &a) { raw = a.raw; }

	void SetCol(u32 c)		{ DW[0] &= ~COL_MASK;						DW[0] |= c & COL_MASK; }
	void SetPage(u32 p)		{ DW[0] &= ~(PAGE_MASK << PAGE_OFF);		DW[0] |= (p & PAGE_MASK) << PAGE_OFF; }
	void SetChip(u32 c)		{ DW[0] &= ~(CHIP_MASK << CHIP_OFF);		DW[0] |= (c & CHIP_MASK) << CHIP_OFF; }
	void SetBlock(u32 b)	{ raw   &= ~((u64)BLOCK_MASK << BLOCK_OFF);	raw   |= (u64)(b & BLOCK_MASK) << BLOCK_OFF; }

	u16 GetCol()	{ return DW[0] & COL_MASK; }
	u16 GetPage()	{ return (DW[0] >> PAGE_OFF) & PAGE_MASK; }
	u16 GetChip()	{ return (DW[0] >> CHIP_OFF) & CHIP_MASK; }
	u16 GetBlock()	{ return (raw >> BLOCK_OFF) & BLOCK_MASK; }
	

	FLADR() : raw(0) {}
	//FLADR(u32 bl, u16 pg, u16 cl, byte ch) : block(bl), page(pg), col(cl), chip(ch) {}
	FLADR(u32 pg) { SetRawPage(pg); }

	static void	InitVaildTables(u16 mask);

	u32		GetRawPage() { return (raw & RAWADR_MASK) >> PAGE_OFF; }

	void	SetRawPage(u32 p) { raw = (u64)(p & RAWPAGE_MASK) << PAGE_OFF; };

	u32		GetRawBlock() { return (raw & RAWADR_MASK) >> CHIP_OFF; }

	void	SetRawBlock(u32 b) { raw = (u64)(b & RAWBLOCK_MASK) << CHIP_OFF; };

	u64		GetRawAdr()	{ return raw & RAWADR_MASK; };
	void	SetRawAdr(u64 a) { raw  = a & RAWADR_MASK; };

	void	NextPage()	{ DW[0] &= ~COL_MASK; raw += 1UL << COL_BITS; raw += chipOffsetNext[GetChip()]; }
	void	NextBlock()	{ DW[0] &= ~((PAGE_MASK << PAGE_OFF)|COL_MASK); raw += 1UL << CHIP_OFF; raw += chipOffsetNext[GetChip()];}
	void	PrevPage()	{ raw -= 1UL << COL_BITS; DW[0] &= ~COL_MASK; raw -= chipOffsetPrev[GetChip()]; }
	void	PrevBlock()	{ raw -= 1UL << CHIP_OFF; DW[0] &= ~((PAGE_MASK << PAGE_OFF)|COL_MASK); raw -= chipOffsetPrev[GetChip()];}

	void	AddRaw(u32 v) { raw += v; raw += chipOffsetNext[GetChip()]; }
	void	SubRaw(u32 v) { raw -= v; raw -= chipOffsetPrev[GetChip()]; }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern bool NAND_BUSY(); 
extern bool NAND_CmdBusy();
extern void NAND_WriteDataDMA(volatile void *src, u16 len);
extern void NAND_WriteDataPIO(volatile void *src, u16 len);
extern void NAND_ReadDataDMA(volatile void *dst, u16 len);
extern void NAND_ReadDataDMA2(volatile void *dst, u16 len);
extern void NAND_ReadDataPIO(volatile void *dst, u16 len);
extern bool NAND_CheckDataComplete();
extern void NAND_Chip_Select(byte chip);
extern void NAND_Chip_Disable();
extern void NAND_WRITE(byte data);
extern void NAND_CopyDataDMA(volatile void *src, volatile void *dst, u16 len);
extern bool NAND_CheckCopyComplete();

extern void NAND_CmdEraseBlock(u32 bl);
extern void NAND_CmdRandomRead(u16 col);
extern void NAND_CmdReadPage(u16 col, u32 bl, u16 pg);
extern void NAND_CmdWritePage(u16 col, u32 bl, u16 pg);
extern void NAND_CmdWritePage2();
extern byte NAND_CmdReadStatus();

inline const NandMemSize* NAND_GetMemSize() { extern NandMemSize nandSize; return &nandSize; } 
inline u32 NAND_GetGoodChipMask() { extern NandMemSize nandSize; return nandSize.mask; } 


extern void Hardware_Init();

extern void UpdateHardware();

extern u16 CRC_CCITT_PIO(const void *data, u32 len, u16 init);
extern u16 CRC_CCITT_DMA(const void *data, u32 len, u16 init);
extern void CRC_CCITT_DMA_Async(const void* data, u32 len, u16 init);
extern bool CRC_CCITT_DMA_CheckComplete(u16* crc);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct DSCI2C
{
	DSCI2C*			next;
	void*			wdata;
	void*			rdata;
	void*			wdata2;
	u16				wlen;
	u16				wlen2;
	u16				rlen;
	u16				readedLen;
	byte			adr;
	volatile bool	ready;
	volatile bool	ack;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//extern bool Write_I2C(DSCI2C *d);
//inline bool Read_I2C(DSCI2C *d) { return Write_I2C(d); }
//extern bool Check_I2C_ready();
extern bool I2C_AddRequest(DSCI2C *d);
extern bool I2C_Update();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct DSCSPI
{
	DSCSPI*			next;
	//u32				baud;
	//u32				FDR;
	void*			wdata;
	void*			rdata;
	u32				adr;
	u16				alen;
	u16				wlen;
	u16				rlen;
	volatile bool	ready;
	byte			csnum;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern bool SPI_AddRequest(DSCSPI *d);
extern bool SPI_Update();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct MRB
{
	bool	ready;
	bool	OK;
	u16		len;
	u16		maxLen;
	u16		*data;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct MTB
{
	bool		ready;
	u16			baud;
	u16			len1;
	const u16	*data1;
	u16			len2;
	const u16	*data2;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern bool RcvManData(MRB *mrb);
extern bool SendManData(MTB *mtb);
extern bool SendManData2(MTB* mtb);
//extern void SetTrmBoudRate(byte i);
extern void ManRcvUpdate();
//extern void ManRcvStop();

extern bool SendMLT3(MTB *mtb);

inline u16 GetRcvManQuality() { extern u16 rcvManQuality; return rcvManQuality; }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();
extern void SetClock(const RTC &t);

extern void EnableDSP();	
extern void DisableDSP();	

inline u16 GetShaftRPS() { extern u16 shaftRPS; return shaftRPS; }
inline u16 GetShaftCount() { extern volatile u16 curShaftCounter; return curShaftCounter; }
extern u16 GetShaftState();

extern void Set_Sync_Rot(u16 RPS, u16 samplePerRound);

extern void EnableDSP();	
extern void DisableDSP();	
//extern void DSP_CopyDataDMA(volatile void *src, volatile void *dst, u16 len);
//extern bool DSP_CheckDataComplete();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__forceinline u32 Push_IRQ()
{
	register u32 t;

#ifndef WIN32

	register u32 primask __asm("primask");

	t = primask;

	__disable_irq();

#endif

	return t;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__forceinline void Pop_IRQ(u32 t)
{
#ifndef WIN32

	register u32 primask __asm("primask");

	primask = t;

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

extern void I2C_Destroy();
extern void SPI_Destroy();
extern void UpdateDisplay();
extern int PutString(u32 x, u32 y, byte c, const char *str);
extern int Printf(u32 x, u32 y, byte c, const char *format, ... );

#endif

#endif // HARDWARE_H__23_12_2013__11_37
