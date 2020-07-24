//#include <string.h>

//#include "common.h"
#include "flash.h"
//#include "fram.h"
#include "vector.h"
#include "core.h"
#include "list.h"
//#include <CRC16.h>
#include "ComPort.h"
#include "trap_def.h"
#include "trap.h"
//#include "twi.h"
#include "PointerCRC.h"


#pragma diag_suppress 550,177

//#pragma O3
//#pragma Otime

/**************************************************/
/*

Технология записи такова, что первая попытка при провале идёт по следующему адресу,
вторая по следующему блоку, трижды - ошибка.
можно сделать этот цикл бесконечным в целях более быстрого поиска живого места
можно также в случае битых блоков считать ИД памяти и понять а не сдохла ли она совсем, но это муторно реализовывать

*/
/**************************************************/

#pragma push
#pragma O3
#pragma Otime

static bool __memcmp(ConstDataPointer s, ConstDataPointer d, u32 len)
{
	while (len > 3)
	{
		if (*s.d++ != *d.d++) return false;
		len -= 4;
	};

	while (len > 0)
	{
		if (*s.b++ != *d.b++) return false;
		len -= 1;
	};

	return true;
}

#pragma pop

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//#define FLASH_SAVE_BUFFER_SIZE		8400
//#define FLASH_READ_BUFFER_SIZE		8400

static FLWB flashWriteBuffer[4];
static FLRB flashReadBuffer[4];

static List<FLWB> freeFlWrBuf;
static List<FLWB> writeFlBuf;

static List<FLRB> freeFlRdBuf;
static List<FLRB> readFlBuf;

static FLWB *curWrBuf = 0;
static FLRB *curRdBuf = 0;

//static SpareArea spareRead;
//static SpareArea spareWrite;
//static SpareArea spareErase;

static u32 pagesRead = 0;
static u32 pagesReadOK = 0;
static u32 pagesReadErr = 0;

static u64 adrLastVector = -1;
static u32 lastSessionBlock = -1;
static u32 lastSessionPage = -1;

static bool cmdCreateNextFile = false;
static bool cmdFullErase = false;
static bool cmdSendSession = false;

static bool writeFlashEnabled = false;
static bool flashFull = false;
static bool flashEmpty = false;

static bool testWriteFlash = false;

static const bool verifyWritePage = false; // Проверка записаной страницы, путём чтения страницы и сравнения с буфером

static SessionInfo lastSessionInfo;

u16 deviceID = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct NVV // NonVolatileVars  
{
	u16 numDevice;

	FileDsc f;

	u16 index;

	u32 prevFilePage;

	u32 badBlocks[4];
};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct NVSI // NonVolatileSessionInfo  
{
	FileDsc f;

	u16 crc;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static NVV nvv;

static NVSI nvsi[128];

static byte buf[sizeof(nvv)*2+4];

byte savesCount = 0;

byte savesSessionsCount = 0;

byte eraseSessionsCount = 0;

//static TWI	twi;

static void SaveVars();

static bool loadVarsOk = false;
static bool loadSessionsOk = false;

u32 verifyFlashErrors = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

const SessionInfo* GetLastSessionInfo()
{
	return &lastSessionInfo;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

enum flash_status_type
{
	FLASH_STATUS_WAIT = 0,
	FLASH_STATUS_WRITE,
	FLASH_STATUS_READ,
//	FLASH_STATUS_ERASE
};

static byte flashStatus = FLASH_STATUS_WAIT;

enum NandState
{
	NAND_STATE_WAIT = 0,
	NAND_STATE_WRITE_START,
	NAND_STATE_READ_START,
	NAND_STATE_FULL_ERASE_START,
	NAND_STATE_FULL_ERASE_0,
	NAND_STATE_CREATE_FILE,
	NAND_STATE_SEND_SESSION
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static NandState nandState = NAND_STATE_WAIT;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const NandMemSize *nandSZ;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32		invalidBlocks = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static ComPort com1;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetNumDevice(u16 num)
{
	nvv.numDevice = num;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern u16 GetNumDevice()
{
	return nvv.numDevice;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitFlashBuffer()
{
	for (byte i = 0; i < ArraySize(flashWriteBuffer); i++)
	{
		freeFlWrBuf.Add(&flashWriteBuffer[i]);
	};

	for (byte i = 0; i < ArraySize(flashReadBuffer); i++)
	{
		freeFlRdBuf.Add(&flashReadBuffer[i]);
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

FLWB* AllocFlashWriteBuffer()
{
	return freeFlWrBuf.Get();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

FLRB* AllocFlashReadBuffer()
{
	return freeFlRdBuf.Get();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeFlashReadBuffer(FLRB* b)
{
	freeFlRdBuf.Add(b);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool RequestFlashRead(FLRB* b)
{
	if ((b != 0) && (b->data != 0) && (b->maxLen > 0))
	{
		b->ready = false;
		b->len = 0;

		readFlBuf.Add(b);

		return true;
	}
	else
	{
		return false;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool RequestFlashWrite(FLWB* b)
{
	if ((b != 0) && (b->dataLen > 0))
	{
		writeFlBuf.Add(b);

		return true;
	}
	else
	{
		return false;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_FullErase()
{
	cmdFullErase = true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_NextSession()
{
	cmdCreateNextFile = true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ResetNandState()
{
	nandState = NAND_STATE_WAIT;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

NandState GetNandState()
{
	return nandState;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct EraseBlock
{
	enum {	WAIT = 0,ERASE_START,ERASE_0,ERASE_1,ERASE_2,ERASE_3,ERASE_4,ERASE_5 };

	SpareArea spare;
	
	FLADR er;

	byte state;
	bool force;		// стереть полюбасу
	bool check;		// Проверить результат стирания
	u16	errBlocks;

	EraseBlock() : state(WAIT), force(false), check(true), errBlocks(0), er(-1, -1, -1, -1) {}

	void Start(const FLADR &rd, bool frc, bool chk);
	bool Update();
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void EraseBlock::Start(const FLADR &rd, bool frc, bool chk)
{
	er = rd;

	force = frc;
	check = chk;

	errBlocks = 0;

	state = ERASE_START;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool EraseBlock::Update()
{
	switch(state)
	{
		case WAIT:
			
			return false;

		case ERASE_START:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++		
																																
			NAND_Chip_Select(er.chip);

			if (force)
			{
				NAND_CmdEraseBlock(er.block);																						
																															
				state = ERASE_2;																				
			}
			else
			{
				NAND_CmdReadPage(er.pg, er.block, 0);																					
																																	
				state = ERASE_0;																						
			};
																																
			break;																												
																																
		case ERASE_0:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++				
																																
			if(!NAND_BUSY())																									
			{																													
				NAND_ReadDataDMA(&spare, spare.CRC_SKIP);	

				state = ERASE_1;																					
			};																													
																																
			break;																												
																																
		case ERASE_1:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++				
																																
			if (NAND_CheckDataComplete())																							
			{																													
				if (spare.validPage != 0xFFFF && spare.validBlock != 0xFFFF)									
				{																												
					errBlocks += 1;	
					nvv.badBlocks[er.chip] += 1;
																																
					er.NextBlock();	

					state = ERASE_START; 
				}	
				else if (spare.badPages != 0xFFFF)
				{
					errBlocks += 1;	
																																
					NAND_CmdWritePage(er.pg, er.block, 0);																			
																																
					NAND_WRITE(0); NAND_WRITE(0); NAND_WRITE(0); NAND_WRITE(0); //*(u32*)FLD = 0;		// spareErase.validPage = 0; spareErase.validBlock = 0;																
																																
					NAND_CmdWritePage2();																							
																																
					state = ERASE_3;																				
				}
				else																											
				{																												
					NAND_CmdEraseBlock(er.block);																						
																																
					state = ERASE_2;																				
				};																												
			};																													
																																
			break;																												
																																
																																
		case ERASE_2:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++				
																																
			if(!NAND_BUSY())																									
			{																													
				if ((NAND_CmdReadStatus() & 1) != 0 && check) // erase error																	
				{																												
					errBlocks += 1;	
					nvv.badBlocks[er.chip] += 1;

//					__breakpoint(0);																							
																																
					NAND_CmdWritePage(er.pg, er.block, 0);																			
																																
					NAND_WRITE(0);NAND_WRITE(0);NAND_WRITE(0);NAND_WRITE(0);//*(u32*)FLD = 0;		// spareErase.validPage = 0; spareErase.validBlock = 0;																
																																
					NAND_CmdWritePage2();																							
																																
					state = ERASE_3;																				
				}																												
				else																											
				{																												
					//er_block = wr.block;														
					//er_chip = wr.chip;																							
																																
					state = WAIT;		

					return false;
				};																												
			};																													
																																
			break;																												
																																
		case ERASE_3:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++				
																																
			if(!NAND_BUSY())																									
			{																													
				er.NextBlock();	

				state = ERASE_START; 
			};

			break;
	};

	return true;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct Write
{
	enum {	WAIT = 0,				WRITE_START,			WRITE_BUFFER,			WRITE_PAGE,				WRITE_PAGE_0,	WRITE_PAGE_1,
			WRITE_PAGE_2,			WRITE_PAGE_3,			WRITE_PAGE_4,			WRITE_PAGE_5,			WRITE_PAGE_6,	WRITE_PAGE_7,	WRITE_PAGE_8,			
			ERASE,			
			WRITE_CREATE_FILE_0,	WRITE_CREATE_FILE_1,	WRITE_CREATE_FILE_2,	WRITE_CREATE_FILE_3,	WRITE_CREATE_FILE_4 };

	FLADR wr;

	//u16		wr_cur_col;
	//u32 	wr_cur_pg;

	byte	wr_pg_error;
	u16		wr_count;
	byte*	wr_data	;
	void*	wr_ptr	;

	u64		prWrAdr;
	u32		rcvVec ;
	u32		rejVec ;
	u32		errVec ;

	SpareArea spare;

//	SpareArea rspare;

	byte state;

	bool createFile;

	EraseBlock eraseBlock;

	u32	wrBuf[2048/4];
	u32	rdBuf[2112/4];


	bool Start();
	bool Update();
	void Vector_Make(VecData *vd, u16 size);
	void Finish();
	void Init();
	void Init(u32 bl, u32 file, u32 prfile);

	void SaveSession();

//	void BufWriteData(void *data, u16 len) { NAND_CopyDataDMA(data, (byte*)wrBuf+wr.col, len); }

	void CreateNextFile() { state = WRITE_CREATE_FILE_0; }


};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Write write;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Write::Init()
{
	wr.SetRawPage(nvv.f.lastPage);

	spare.file = nvv.f.session;  

	spare.prev = nvv.prevFilePage;		

	spare.start = nvv.f.startPage;		
	spare.fpn = 0;	

	spare.vectorCount = 0;

	spare.vecFstOff = -1;
	spare.vecFstLen = 0;

	spare.vecLstOff = -1;
	spare.vecLstLen = 0;

	spare.fbb = 0;		
	spare.fbp = 0;		

	spare.chipMask = nandSZ->mask;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Write::Init(u32 bl, u32 file, u32 prfile)
{
	wr.SetRawBlock(bl);

	spare.file = file;  
//	spare.lpn = wr.GetRawPage();

	spare.prev = prfile;		

	spare.start = wr.GetRawPage();		
	spare.fpn = 0;	

	spare.vectorCount = 0;

	spare.vecFstOff = -1;
	spare.vecFstLen = 0;

	spare.vecLstOff = -1;
	spare.vecLstLen = 0;

	spare.fbb = 0;		
	spare.fbp = 0;		

	spare.chipMask = nandSZ->mask;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Write::SaveSession()
{
	nvv.index = (nvv.index+1) & 127;

	FileDsc &s = nvv.f;
	NVSI &d = nvsi[nvv.index];

	d.f = nvv.f;

	d.crc = GetCRC16(&d.f, sizeof(d.f));

	s.session = spare.file;
	s.startPage = s.lastPage = wr.GetRawPage();
	s.size = 0;
	GetTime(&s.start_rtc);
	s.stop_rtc = s.start_rtc;
	s.flags = 0;

	SaveParams();
	savesSessionsCount = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Write::Vector_Make(VecData *vd, u16 size)
{
	vd->h.session = spare.file;
//	vd->h.device = 0;
	GetTime(&vd->h.rtc);

	vd->h.prVecAdr = prWrAdr; 
	vd->h.flags = 0;
	vd->h.dataLen = size;
	vd->h.crc = GetCRC16(&vd->h, sizeof(vd->h) - sizeof(vd->h.crc));
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool Write::Start()
{
	if ((curWrBuf = writeFlBuf.Get()) != 0)
	{
		rcvVec += 1;

		if (!writeFlashEnabled || flashFull)
		{
//			rejVec += 1;

//			curWrBuf->ready[0] = true;
			freeFlWrBuf.Add(curWrBuf);
			return false;
		};

		Vector_Make(&curWrBuf->vd, curWrBuf->dataLen); 

//		curWrBuf->dataLen += sizeof(curWrBuf->vd.vec);
//		curWrBuf->hdrLen = 0;

		
		prWrAdr = wr.GetRawAdr();

		wr_data = (byte*)&curWrBuf->vd;
		wr_count = curWrBuf->dataLen + sizeof(curWrBuf->vd.h);

//		wr_cur_col = wr.col;
//		wr_cur_pg = wr.GetRawPage();

		if (spare.vecFstOff == 0xFFFF)
		{
			spare.vecFstOff = wr.col;
			spare.vecFstLen = wr_count;
		};

		spare.vecLstOff = wr.col;
		spare.vecLstLen = wr_count;

		spare.vectorCount += 1;

		state = WRITE_START;

		return true;
	}
	else
	{
		return false;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Write::Finish()
{
	if (curWrBuf != 0)
	{
		nvv.f.size += curWrBuf->vd.h.dataLen;
		nvv.f.stop_rtc = curWrBuf->vd.h.rtc;
		nvv.f.lastPage = spare.rawPage;

		SaveParams();

//		curWrBuf->ready[0] = true;

		freeFlWrBuf.Add(curWrBuf);

		curWrBuf = 0;

		//wr_prev_col = wr_cur_col;
		//wr_prev_pg = wr_cur_pg;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool Write::Update()
{
//	u32 t;

	switch(state)
	{
		case WAIT:	return false;

		case WRITE_START:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
			
//			NAND_Chip_Select(wr.chip);

			{
				register u16 c = wr.pg - wr.col;


				if (wr.col == 0 && wr_count >= wr.pg) // писать сразу во флеш
				{
					wr_ptr = wr_data;
					wr_count -= wr.pg;
					wr_data += wr.pg;

					state = WRITE_PAGE;
				}
				else // писать в буфер
				{
					if (wr_count < c ) c = wr_count;

					NAND_CopyDataDMA(wr_data, (byte*)wrBuf+wr.col, c);	// BufWriteData(wr_data, c);

					wr.col += c;
					wr_data += c;
					wr_count -= c;

					state = WRITE_BUFFER;
				};
			};

			break;
	
		case WRITE_BUFFER:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (NAND_CheckDataComplete())
			{
				if (wr.col == 0)
				{
					wr_ptr = wrBuf;
//					wr.col = 0;

					state = WRITE_PAGE;
				}
				else if (wr_count > 0)
				{
					state = WRITE_START;
				}
				else
				{
					Finish();

					state = WAIT;

					return false;
				};
			};

			break;

		case WRITE_PAGE:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if(eraseBlock.er.GetRawBlock() != wr.GetRawBlock())	// новый блок
			{
				wr_pg_error = 0;

				eraseBlock.Start(wr, false, true);

	            state = ERASE;

				break;
			};

		case WRITE_PAGE_0:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			NAND_Chip_Select(wr.chip);

			NAND_CmdWritePage(0, wr.block, wr.page);

			NAND_WriteDataDMA(wr_ptr, wr.pg);
			
			state = WRITE_PAGE_1;

			break;

		case WRITE_PAGE_1:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (NAND_CheckDataComplete())
			{
				spare.validPage = -1;
				spare.validBlock = -1;
				spare.badPages = -1;
				spare.rawPage = wr.GetRawPage();
				spare.chipMask = nandSZ->mask;

				spare.crc = GetCRC16((void*)&spare.file, sizeof(spare) - spare.CRC_SKIP - sizeof(spare.crc));

				NAND_WriteDataDMA(&spare, sizeof(spare));

				state = WRITE_PAGE_2;
			};

			break;

		case WRITE_PAGE_2:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (NAND_CheckDataComplete())
			{
				NAND_CmdWritePage2();

				state = WRITE_PAGE_3;
			};

			break;

		case WRITE_PAGE_3:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if(!NAND_BUSY())
			{
				byte t = NAND_CmdReadStatus();

				if ((t & 1) != 0) // program error
				{
					spare.fbp += 1;

					NAND_CmdWritePage(wr.pg, wr.block, wr.page);

					NAND_WRITE(0);NAND_WRITE(0);//*(u16*)FLD = 0; // spareErase.validPage = 0;

					NAND_CmdWritePage2();

					state = WRITE_PAGE_4;
				}
				else if (verifyWritePage)
				{
					NAND_CmdReadPage(0, wr.block, wr.page);
					
					state = WRITE_PAGE_6;
				}
				else
				{
					state = WRITE_PAGE_8;
				};
			};

			break;

		case WRITE_PAGE_4:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++			

			if(!NAND_BUSY())																									
			{																													
				if (wr_pg_error == 2) // пометить блок как имеющий сбойные страницы
				{
					NAND_CmdWritePage(wr.pg + sizeof(spare.validPage) + sizeof(spare.validBlock), wr.block, 0);

					NAND_WRITE(0);NAND_WRITE(0);//*(u16*)FLD = 0; // spare.badPages = 0;

					NAND_CmdWritePage2();

					state = WRITE_PAGE_5;
				}
				else
				{
					state = WRITE_PAGE;																			
				};

				wr_pg_error += 1;

				wr.NextPage();		
			};																													

			break;

		case WRITE_PAGE_5:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++			
																																
			if(!NAND_BUSY())																									
			{																													
				state = WRITE_PAGE;																			
			};
																																
			break;	

		case WRITE_PAGE_6:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if(!NAND_BUSY())
			{
				NAND_ReadDataDMA(&rdBuf, sizeof(rdBuf));

				state = WRITE_PAGE_7;
			};

			break;	

		case WRITE_PAGE_7:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++			

			if (NAND_CheckDataComplete())
			{
				if (!__memcmp(wr_ptr, rdBuf, wr.pg) || !__memcmp(&spare, rdBuf+wr.pg/4, sizeof(spare)))
				{
					verifyFlashErrors += 1;

					spare.fbp += 1;

					NAND_CmdWritePage(wr.pg, wr.block, wr.page);

					NAND_WRITE(0);NAND_WRITE(0);//*(u16*)FLD = 0; // spareErase.validPage = 0;

					NAND_CmdWritePage2();

					state = WRITE_PAGE_4;
				}
				else
				{
					//if (wr_count == 0)
					//{
					//	Finish();

					//	state = (createFile) ? WRITE_CREATE_FILE_1 : WAIT;
					//}
					//else
					//{
					//	state = WRITE_START;
					//};

					//wr.NextPage();

					//spare.fpn += 1;

					//spare.vecFstOff = -1;
					//spare.vecFstLen = 0;

					//spare.vecLstOff = -1;
					//spare.vecLstLen = 0;

					state = WRITE_PAGE_8;
				};
			};

			break;

		case WRITE_PAGE_8:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			if (wr_count == 0)
			{
				Finish();

				state = (createFile) ? WRITE_CREATE_FILE_1 : WAIT;
			}
			else
			{
				state = WRITE_START;
			};

			wr.NextPage();

			spare.fpn += 1;

			spare.vecFstOff = -1;
			spare.vecFstLen = 0;

			spare.vecLstOff = -1;
			spare.vecLstLen = 0;

			break;	

		case ERASE:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++			
					
			if (!eraseBlock.Update())
			{
				wr = eraseBlock.er;

				spare.fbb += eraseBlock.errBlocks;

				state = WRITE_PAGE_0;																			
			};
																																
			break;																												

		case WRITE_CREATE_FILE_0:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (wr.col > 0)
			{
				wr_ptr = wrBuf;
				wr_count = 0;

				byte *p = (byte*)wrBuf;

				p += wr.col;

				for (u16 i = wr.col&3; i > 0; i--)
				{
					*(p++) = 0xff;
				};

				u32 *d = wrBuf + (wr.col+3)/4;

				for (u16 i = (sizeof(wrBuf)-wr.col)/4; i > 0; i--)
				{
					*(d++) = -1;
				};

				createFile = true;

				state = WRITE_PAGE_0;
				
				break;
			};


		case WRITE_CREATE_FILE_1:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			spare.file += 1;  		

			spare.prev = spare.start;		

			if (wr.page > 0)
			{
				wr.NextBlock();

				//if (wr.overflow != 0)
				//{
				//	flashFull = true;
				//};
			};

			spare.start = wr.GetRawPage();		
			spare.fpn = 0;
			spare.vectorCount = 0;

			//wr_prev_pg = -1;
			//wr_prev_col = 0;

			spare.fbb = 0;		
			spare.fbp = 0;		

			spare.chipMask = nandSZ->mask;	

			SaveSession();

			createFile = false;

			state = WAIT;

			return false;
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct ReadSpare
{
	enum { WAIT = 0, START, READ_1, READ_2, READ_3 };

	SpareArea	*spare;
	FLADR		*rd;

	byte state;

	ReadSpare() : spare(0), rd(0) {}

	bool Start(SpareArea *sp, FLADR *frd);
	bool Update();
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ReadSpare::Start(SpareArea *sp, FLADR *frd)
{
	if (sp == 0 || frd == 0)
	{
		return false;
	};

	spare = sp;
	rd = frd;

	state = START;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool ReadSpare::Update()
{
	switch(state)
	{
		case WAIT:

			return false;

		case START:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			NAND_Chip_Select(rd->chip);

			NAND_CmdReadPage(rd->pg, rd->block, rd->page);

			state = READ_1;

			break;

		case READ_1:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if(!NAND_BUSY())
			{
				NAND_ReadDataDMA(spare, sizeof(*spare));

				state = READ_2;
			};

			break;

		case READ_2:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (NAND_CheckDataComplete())
			{
				if (spare->validBlock != 0xFFFF)
				{
					rd->NextBlock();

					NAND_Chip_Select(rd->chip);
					NAND_CmdReadPage(rd->pg, rd->block, rd->page);

					state = READ_1;
				}				
				else if (spare->validPage != 0xFFFF)
				{
					rd->NextPage();

					NAND_Chip_Select(rd->chip);
					NAND_CmdReadPage(rd->pg, rd->block, rd->page);

					state = READ_1;
				}
				else
				{
					spare->crc = GetCRC16((void*)&spare->file, sizeof(*spare) - spare->CRC_SKIP);
				
					state = WAIT;

					return false;
				};
			};

			break;
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

namespace Read
{
	enum {	WAIT = 0,READ_START,READ_1, /*READ_2,*/ READ_PAGE,READ_PAGE_1,FIND_START,FIND_1,/*FIND_2,*/FIND_3/*,FIND_4*/};

	static FLADR rd(0, 0, 0, 0);
	static byte*	rd_data = 0;
	static u16		rd_count = 0;

	static u32 		sparePage = -1;

	static SpareArea spare;

	static ReadSpare readSpare;

	static bool vecStart = false;

	static byte state;

	static bool Start();
//	static bool Start(FLRB *flrb, FLADR *adr);
	static bool Update();
	static void End() { curRdBuf->ready = true; curRdBuf = 0; state = WAIT; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Read::Start()
{
	if ((curRdBuf = readFlBuf.Get()) != 0)
	{
		if (curRdBuf->useAdr) { rd.SetRawAdr(curRdBuf->adr); };

		vecStart = curRdBuf->vecStart;

		if (vecStart)
		{
			rd_data = (byte*)&curRdBuf->hdr;
			rd_count = sizeof(curRdBuf->hdr);
			curRdBuf->len = 0;	
		}
		else
		{
			rd_data = curRdBuf->data;
			rd_count = curRdBuf->maxLen;
			curRdBuf->len = 0;	
		};

		state = READ_START;

		return true;
	};

	return false;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Read::Update()
{
	switch(state)
	{
		case WAIT:	return false;

		case READ_START:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			NAND_Chip_Select(rd.chip);

			if (rd.GetRawPage() != sparePage)
			{
				readSpare.Start(&spare, &rd);
//				NAND_CmdReadPage(rd.pg, rd.block, rd.page);

				state = READ_1;
			}
			else
			{
				NAND_CmdReadPage(rd.col, rd.block, rd.page);

				state = READ_PAGE;
			};

			break;

		case READ_1:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (!readSpare.Update())
			{
				sparePage = rd.GetRawPage();

				NAND_CmdRandomRead(rd.col);

				state = READ_PAGE;
			};

			break;

		//case READ_2:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	


		//	break;

		case READ_PAGE:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if(!NAND_BUSY())
			{
				register u16 c = rd.pg - rd.col;

				if (rd_count < c) c = rd_count;

				NAND_ReadDataDMA(rd_data, c);

				rd_count -= c;
				rd.raw += c;
				rd_data += c;
				curRdBuf->len += c;

				state = READ_PAGE_1;
			};

			break;

		case READ_PAGE_1:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (NAND_CheckDataComplete())
			{
				if (rd_count == 0)
				{
					if (vecStart)
					{
						curRdBuf->hdr.crc = GetCRC16(&curRdBuf->hdr, sizeof(curRdBuf->hdr));

						if (curRdBuf->hdr.crc == 0)
						{
							rd_data = curRdBuf->data;
							rd_count = (curRdBuf->hdr.dataLen > curRdBuf->maxLen) ? curRdBuf->maxLen : curRdBuf->hdr.dataLen;
							curRdBuf->len = 0;	
							vecStart = false;

							if (rd_data == 0 || rd_count == 0)
							{
								End();

								return false;
							}
							else
							{
								state = READ_START;
							};
						}
						else
						{
							// Искать вектор

							state = FIND_START;
						};
					}
					else
					{
						End();

						return false;
					};
				}
				else
				{
					state = READ_START;
				};
			};

			break;

		case FIND_START:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (spare.start == -1 || spare.fpn == -1)
			{
				if (rd.page == 0)
				{
					// Вектора кончились
					state = FIND_3;
				}
				else
				{
					rd.NextPage();

					readSpare.Start(&spare, &rd);

					state = FIND_1;
				};
			}
			else if (spare.crc != 0 || spare.vecFstOff == 0xFFFF || spare.vecLstOff == 0xFFFF || rd.col > spare.vecLstOff)
			{
				rd.NextPage();

				readSpare.Start(&spare, &rd);

				state = FIND_1;
			}
			else 
			{
				if (rd.col <= spare.vecFstOff)
				{
					rd.col = spare.vecFstOff;
				}
				else if (rd.col <= (spare.vecFstOff+spare.vecFstLen))
				{
					rd.col = spare.vecFstOff+spare.vecFstLen;
				}
				else if (rd.col <= spare.vecLstOff)
				{
					rd.col = spare.vecLstOff;
				};

				rd_data = (byte*)&curRdBuf->hdr;
				rd_count = sizeof(curRdBuf->hdr);
				curRdBuf->len = 0;	

				state = READ_START;
			};

			break;

		case FIND_1:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			if (!readSpare.Update())	//(!NAND_BUSY())
			{
				sparePage = rd.GetRawPage();

				state = FIND_START;
			};

			break;

		case FIND_3:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	

			curRdBuf->len = 0;
			curRdBuf->hdr.dataLen = 0;

			End();

			break;
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void ReadSpareNow(SpareArea *spare, FLADR *rd, bool crc)
//{
//	//SpareArea *spare;
//	//FLADR *rd;
//
//	NAND_Chip_Select(rd->chip);
//
//	NAND_CmdReadPage(rd->pg, rd->block, rd->page);
//
//	while(!NAND_BUSY());
//	while(NAND_BUSY());
//
//	NAND_ReadDataDMA(spare, sizeof(*spare));
//
//	while (!NAND_CheckDataComplete());
//
//	if (crc)
//	{
//		spare->crc = GetCRC16((void*)&spare->file, sizeof(*spare) - spare->CRC_SKIP);
//	};
//}
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//static void ReadDataNow(void *dst, u16 len, FLADR *rd)
//{
//	NAND_Chip_Select(rd->chip);
//
//	NAND_CmdReadPage(rd->col, rd->block, rd->page);
//
//	while(!NAND_BUSY());
//	while(NAND_BUSY());
//
//	NAND_ReadDataDMA(dst, len);
//
//	while (!NAND_CheckDataComplete());
//}
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//static void ReadVecHdrNow(VecData::Hdr *h, FLADR *rd)
//{
//	ReadDataNow(h, sizeof(*h), rd);
//
//	h->crc = GetCRC16(h, sizeof(*h));
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitSessions()
{
//	__breakpoint(0);

	write.Init();

	if (nvv.f.size > 0)
	{
//		NAND_NextSession();

		write.CreateNextFile();

		while (write.Update()) ;
	};

	u32 ms = 0, me = 0, ls = -1;
	u32 sp = 0;

	bool bm = false, bl = false;

	//FLADR sb(0), eb(-1); // free start and end block

	for (u16 i = 128, ind = nvv.index; i > 0; i--, ind = (ind-1)&127)
	{
		FileDsc &f = nvsi[ind].f;

		if (f.size == 0) continue;

		if (bl)
		{
			if (f.lastPage < f.startPage)
			{
				f.size = 0;
			}
			else
			{
				if (f.lastPage < ls)
				{
					ls = f.startPage;

					if (bm)
					{
						if (f.lastPage <= me)
						{
							f.size = 0;
						}
						else if (f.startPage < me)
						{
							f.startPage = me+1;
						};
					};
				}
				else
				{
					f.size = 0;
				};
			};
		}
		else
		{
			if (f.lastPage < f.startPage)
			{
				bl = true;
				ls = f.startPage;
				sp = 0;
			}
			else
			{
				sp = f.startPage;
			};

			if (!bm)
			{
				bm = true;
				ms = sp;
				me = f.lastPage;
			}
			else if (f.lastPage < ms)
			{
				ms = sp;
			}
			else if (f.lastPage > me)
			{
				f.startPage = me+1;
				me = f.lastPage;
			}
			else
			{
				f.size = 0;
			};
		};



	}; // 	for (u16 i = 128, ind = nvv.index; i > 0; i--, ind = (ind-1)&127)
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//bool GetSessionNow(SessionInfo *si)
//{
//	static SpareArea spare;
//
//	static VecData::Hdr hdr;
//
//	FLADR rd(0,0,0,0);
//
//	rd.raw = adrLastVector;
//
//	ReadSpareNow(&spare, &rd, true);
//
//	si->size = (u64)spare.fpn << NAND_COL_BITS;
//
//	si->last_adress = adrLastVector;
//	si->session = spare.file;
//
//	ReadVecHdrNow(&hdr, &rd);
//
//	if (hdr.crc == 0)
//	{
//		si->stop_rtc = hdr.rtc;
//	}
//	else
//	{
//
//	};
//
//	rd.SetRawPage(spare.start);
//
//	ReadVecHdrNow(&hdr, &rd);
//
//	if (hdr.crc == 0)
//	{
//		si->start_rtc = hdr.rtc;
//	}
//	else
//	{
//
//	};
//
//	return true;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void StartSendSession()
{
	cmdSendSession = true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool UpdateSendSession()
{
	enum {	WAIT = 0, READ_START, READ_1, READ_2, READ_PAGE,READ_PAGE_1,FIND_START,FIND_1,FIND_2,FIND_3,FIND_4, READ_END};

	static byte i = WAIT;
	
	static u16 ind = 0;
	static u32 prgrss = 0;
	static u16 count = 0;
	static u32 lp = 0;
	static u32 sum = 0;
	static FLADR a(0);

	FileDsc &s = nvsi[ind].f;

	switch (i)
	{
		case 0:

			if (cmdSendSession)
			{
				ind = nvv.index;
				prgrss = 0;
				count = 128;

				i++;
			}
			else
			{
				return false;
			};

			break;

		case 1:

			if (s.size > 0)
			{
				if (!TRAP_MEMORY_SendSession(s.session, s.size, (u64)s.startPage * FLADR::pg, s.start_rtc, s.stop_rtc, s.flags))
				{
					break;
				};
			};

			ind = (ind-1)&127;

			prgrss += 0x100000000/128;

			count--;

			i++;

			break;

		case 2:

			if (TRAP_MEMORY_SendStatus(prgrss, FLASH_STATUS_READ_SESSION_IDLE))
			{
				if (count > 0)
				{
					i = 1;
				}
				else
				{
					i++;
				};
			};

			break;

		case 3:

			if (TRAP_MEMORY_SendStatus(-1, FLASH_STATUS_READ_SESSION_READY))
			{
				cmdSendSession = false;

				i = 0;
			};

			break;
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

FileDsc* GetSessionInfo(u16 session, u64 adr)
{
	u16 ind = nvv.index;

	FileDsc *s = 0;

	for (u16 i = 128; i > 0; i--)
	{
		s = &nvsi[ind].f;

		if (s->session == session)
		{
			return s;
		};

		ind = (ind-1)&127;
	};

	return 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void NAND_Idle()
{
//	register u32 t;

	static i32 t = 0;
	static i32 eb = 0;
	static TM32 tm;
	static FLADR er(0);
	static EraseBlock eraseBlock;

	switch (nandState)
	{
		case NAND_STATE_WAIT: 

			if (cmdCreateNextFile)
			{
				write.CreateNextFile();
				cmdCreateNextFile = false;
				nandState = NAND_STATE_CREATE_FILE;

				break;
			};

			if (cmdFullErase)
			{
				cmdFullErase = false;
				eraseSessionsCount = 1;
				//nandState = NAND_STATE_FULL_ERASE_START;

				break;
			};

			if (cmdSendSession)
			{
				nandState = NAND_STATE_SEND_SESSION;

				break;
			};

			if (write.Start())
			{
				nandState = NAND_STATE_WRITE_START;
			}
			else if (Read::Start())
			{
				nandState = NAND_STATE_READ_START;
			};

			if (tm.Check(500)) { TRAP_MEMORY_SendStatus(-1, FLASH_STATUS_NONE); };

			break;

		case NAND_STATE_WRITE_START:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			if (!write.Update())
			{
				nandState = NAND_STATE_WAIT;
			};

			break;

		case NAND_STATE_READ_START:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			if (!Read::Update())
			{
				nandState = NAND_STATE_WAIT;
			};

			break;

		case NAND_STATE_FULL_ERASE_START:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			t = eb = nandSZ->fl >> (NAND_COL_BITS + NAND_PAGE_BITS); // blocks count

			er.SetRawAdr(0);

			eraseBlock.Start(er, true, true);

			nandState = NAND_STATE_FULL_ERASE_0;

			break;

		case NAND_STATE_FULL_ERASE_0:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			if (!eraseBlock.Update())
			{
				invalidBlocks += eraseBlock.errBlocks;

				t -= eraseBlock.errBlocks;
				t -= 1;

				write.errVec = t;

				if (t > 0)
				{
					er.NextBlock();

					eraseBlock.Start(er, true, true);

					if (tm.Check(500)) { TRAP_MEMORY_SendStatus((eb-t)*((u64)0x100000000)/eb, FLASH_STATUS_BUSY); };
				}
				else
				{
					flashEmpty = true;

					write.Init(0, 1, -1);

					adrLastVector = -1;

					TRAP_MEMORY_SendStatus(-1, FLASH_STATUS_ERASE);
					nandState = NAND_STATE_WAIT;
				};
			};

			break;

		case NAND_STATE_CREATE_FILE:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			if (!write.Update())
			{
				nandState = NAND_STATE_WAIT;
			};

			break;

		case NAND_STATE_SEND_SESSION:	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			if (!UpdateSendSession())
			{
				if (TRAP_TRACE_PrintString("NAND Bad Blocks: %lu, %lu, %lu, %lu", nvv.badBlocks[0], nvv.badBlocks[1], nvv.badBlocks[2], nvv.badBlocks[3]))
				{
					nandState = NAND_STATE_WAIT;
				};
			};

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 FLASH_Vectors_Errors_Get()
{
	return write.errVec;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 FLASH_Vectors_Saved_Get()
{
	return write.spare.vectorCount;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 FLASH_Vectors_Recieved_Get()
{
	return write.rcvVec;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 FLASH_Vectors_Rejected_Get()
{
	return write.rejVec;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 FLASH_Session_Get()
{
	return write.spare.file;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u64 FLASH_Current_Adress_Get()
{
	return write.wr.GetRawAdr();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u64 FLASH_Full_Size_Get()
{
	return nandSZ->fl;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u64 FLASH_Used_Size_Get()
{
	i64 size = FLASH_Full_Size_Get();

	return size;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

i64 FLASH_Empty_Size_Get()
{
	return FLASH_Full_Size_Get() - FLASH_Used_Size_Get();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 FLASH_Chip_Mask_Get()
{
	return nandSZ->mask;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// удалить 
bool FLASH_Erase_Full()
{
	//if(flash_status_operation != FLASH_STATUS_OPERATION_WAIT) return false;
	//FRAM_Memory_Start_Adress_Set(FRAM_Memory_Current_Adress_Get());
	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// восстановить
bool FLASH_UnErase_Full()
{
	//if(flash_status_operation != FLASH_STATUS_OPERATION_WAIT) return false;
	//FRAM_Memory_Start_Adress_Set(-1);
	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct Req
{
	u16 rw; 
	u32 cnt; 
	u16 gain; 
	u16 st; 
	u16 len; 
	u16 delay; 
	u16 data[1024*4]; 
	u16 crc;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct Rsp
{
	byte	adr;
	byte	func;
	
	__packed union
	{
		__packed struct  { word crc; } f1;  // Старт новой сессии
		__packed struct  { word crc; } f2;  // Запись вектора
		__packed struct  { word crc; } f3;  // 
		__packed struct  { word crc; } fFE;  // Ошибка CRC
		__packed struct  { word crc; } fFF;  // Неправильный запрос
	};
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Rsp rspData;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CreateRsp01(ComPort::WriteBuffer *wb)
{
	static Rsp rsp;

	if (wb == 0)
	{
		return false;
	};

	rsp.adr = 1;
	rsp.func = 1;
	rsp.f1.crc = GetCRC16(&rsp, sizeof(rsp)-2);

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CreateRsp02(ComPort::WriteBuffer *wb)
{
	static Rsp rsp;

	if (wb == 0)
	{
		return false;
	};

	rsp.adr = 1;
	rsp.func = 2;
	rsp.f2.crc = GetCRC16(&rsp, sizeof(rsp)-2);

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CreateRsp03(ComPort::WriteBuffer *wb)
{
	static Rsp rsp;

	if (wb == 0)
	{
		return false;
	};

	rsp.adr = 1;
	rsp.func = 3;
	rsp.f3.crc = GetCRC16(&rsp, sizeof(rsp)-2);

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CreateRspErrCRC(ComPort::WriteBuffer *wb)
{
	static Rsp rsp;

	if (wb == 0)
	{
		return false;
	};

	rsp.adr = 1;
	rsp.func = 0xFE;
	rsp.fFE.crc = GetCRC16(&rsp, sizeof(rsp)-2);

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CreateRspErrReq(ComPort::WriteBuffer *wb)
{
	static Rsp rsp;

	if (wb == 0)
	{
		return false;
	};

	rsp.adr = 1;
	rsp.func = 0xFF;
	rsp.fFF.crc = GetCRC16(&rsp, sizeof(rsp)-2);

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static bool RequestFunc01(FLWB *fwb, ComPort::WriteBuffer *wb)
//{
//	VecData &vd = fwb->vd;
//
//	Req &req = *((Req*)vd.data);
//
//
//
//
//
//
//
//
//
//	freeFlWrBuf.Add(fwb);
//
//	return CreateRsp01(wb);
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc02(FLWB *fwb, ComPort::WriteBuffer *wb)
{
	VecData &vd = fwb->vd;

//	Req &req = *((Req*)vd.data);

	//byte n = vd.data[0] & 7;

	//vecCount[n] += 1;

	if (!RequestFlashWrite(fwb))
	{
		freeFlWrBuf.Add(fwb);
//		return false;
	};


	return CreateRsp02(wb);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static bool RequestFunc03(FLWB *fwb, ComPort::WriteBuffer *wb)
//{
//	VecData &vd = fwb->vd;
//
//	Req &req = *((Req*)vd.data);
//
//
//
//
//
//
//	freeFlWrBuf.Add(fwb);
//
//	return CreateRsp03(wb);
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte GetRequestCRC(byte *s, u32 len)
{
	byte crc = 0;

	while (len > 0)	{ crc += *(s++); len--;	};

	return crc;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc(FLWB *fwb, ComPort::WriteBuffer *wb)
{
	bool result = false;

	byte *d = (byte*)fwb->data;

	if (fwb == 0)
	{
//		freeReqList.Add(req);
	}
	else if (fwb->dataLen < 952)
	{
//		freeFlWrBuf.Add(fwb);
	}
	else
	{
		d[3] = -d[3];

		if (GetRequestCRC(d, fwb->dataLen) == 0) // (req.rw & 0xFF00) == 0xAA00) // 
		{
			fwb->dataLen -= fwb->vd.data - d;

			DataPointer p(fwb->vd.data);

			p.b += fwb->dataLen;

			*p.w = CRC_CCITT_DMA(fwb->vd.data, fwb->dataLen, 0xFFFF);
//			*p.w = GetCRC16(fwb->vd.data, fwb->dataLen);

			fwb->dataLen += 2;

			fwb->vd.h.device = deviceID = *((__packed u16*)(d+4));

			if (RequestFlashWrite(fwb))
			{
				result = CreateRsp02(wb);
			}
			else
			{
//				freeFlWrBuf.Add(fwb);
			};
		}
		else
		{
//			freeFlWrBuf.Add(fwb);
			write.rejVec++;
		};
	};

	return result;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestTestWrite(FLWB *fwb)
{
	__packed struct Req 
	{ 
		u32 cnt; 
		
		__packed struct Control	{ u16 version; u32 count; /* счётчик 50Гц измерений */ i16 voltage, temperature; byte status, flags; u16 errors; } con;
		__packed struct Gen		{ u16 version; i16 voltage, current, offset, temperature;	byte status, flags;	u16 errors;	} gen;
		__packed struct Incl	{ u16 version, ax, ay,az,fx,fy,fz, az_correction, temperature_sensor;	i16 temperature; byte status,flags;	u16 errors;	} incl;
		__packed struct Zond	{ u16 version; i16 compensation, temperature; byte status, flags; u16 errors; } zond;
		__packed struct Driver	{ u16 version,voltage,current; i16 temperature; byte status, flags; u16 errors;} driver;
		__packed struct Comm	{ u16 version;	i16 radius[4];	i16 temperature;byte status;byte flags;	u16 errors;	} comm[2];
		__packed struct Electrode {	u16 version; i16 current[24]; i16 phase[24]; i16 temperature; byte status; byte flags; u16 errors; } elect[8];

		u16 crc; 
	};

	static u32 pt = 0;

	if (fwb == 0)
	{
		return;
	};

	u32 t = GetMilliseconds();

	if (t == pt)
	{
		freeFlWrBuf.Add(fwb);

		return;
	};

	pt = t;

	static byte nr = 0;
	static byte nf = 0;
	static u32 count = 0;

	VecData &vd = fwb->vd;

	Req &req = *((Req*)vd.data);

	vd.h.device = 0xEC00;
	req.cnt = count;

	count += 1;

	fwb->dataLen = sizeof(req);

//	req.crc = GetCRC16(fwb->vd.data, fwb->dataLen - 2);
	req.crc = CRC_CCITT_DMA(fwb->vd.data, fwb->dataLen - 2, 0xFFFF);
	
	RequestFlashWrite(fwb);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void InitCom()
//{
//	com1.Connect(com1.SYNC_S, 0, 2000000, 2, 2);
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateCom()
//{
////	__packed struct Req { u16 rw; u32 cnt; u16 gain; u16 st; u16 len; u16 delay; u16 data[512*4]; };
//
//	static ComPort::WriteBuffer wb;
//	static ComPort::ReadBuffer rb;
//
//	static byte state = 0;
//
//	static FLWB *fwb;
////	static Req *req;
//	static VecData *vd;
//
//	static u32 count = 0;
//	static u16 v = 0;
//	static byte b = 0;
//
//	static TM32 tm;
//
//	switch (state)
//	{
//		case 0:
//
//			fwb = freeFlWrBuf.Get();
//
//			if (fwb != 0)
//			{
//				if (testWriteFlash)
//				{
//					if (!writeFlashEnabled && tm.Check(2000))
//					{
//						FLASH_WriteEnable();
//					}
//					else if (writeFlashEnabled && (nvv.f.size > 456789012))
//					{	
//						FLASH_WriteDisable();
//						tm.Reset();
//					};
//
//					RequestTestWrite(fwb);
//				}
//				else
//				{
//					state++;
//				};
//			};
//
//			break;
//
//		case 1:
//
//			vd = &fwb->vd;
//
//			rb.data = ((byte*)vd->data)-10;
//			rb.maxLen = 0xF00;//sizeof(vd->data);
//
//			com1.Read(&rb, MS2RT(50), US2RT(200));
//
//
//			state++;
//
//			break;
//
//		case 2:
//
//			if (!com1.Update())
//			{
//				if (rb.recieved)
//				{
//					fwb->data = rb.data;
//					fwb->dataLen = rb.len;
//
//					if (RequestFunc(fwb, &wb))
//					{
//						state++;
//					}
//					else
//					{
//						state = 1;
//					};
//
//				}
//				else
//				{
////					freeFlWrBuf.Add(fwb);
//
////					CreateRsp02(&wb);
//					state = 1;
//				};
//			};
//
//			break;
//
//		case 3:
//
//			if (!freeFlWrBuf.Empty())
//			{
//				com1.Write(&wb);
//
//				state++;
//			};
//
//			break;
//
//		case 4:
//			
//			if (!com1.Update())
//			{
//				state = 0;
//			};
//
//			break;
//	};
//
////	HW::PIOB->CODR = 1<<13;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FLASH_WriteEnable()
{
	if (!writeFlashEnabled)
	{
		nvv.f.size = 0;
		GetTime(&nvv.f.start_rtc);
	};

	writeFlashEnabled = true;
	flashStatus = FLASH_STATUS_WRITE;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FLASH_WriteDisable()
{
	if (writeFlashEnabled && nvv.f.size > 0)
	{
		NAND_NextSession();
	};

	writeFlashEnabled = false;

	flashStatus = FLASH_STATUS_WAIT;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

byte FLASH_Status()
{
	return flashStatus;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadVars()
{
//	twi.Init(1);

	PointerCRC p(buf);

	static DSCI2C dsc;
	static u16 romAdr = 0;

	romAdr = ReverseWord(0);

	dsc.wdata = &romAdr;
	dsc.wlen = sizeof(romAdr);
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;
	dsc.rdata = buf;
	dsc.rlen = sizeof(buf);
	dsc.adr = 0x50;


	if (I2C_AddRequest(&dsc))
	{
		while (!dsc.ready);
	};

//	bool c = false;

	loadVarsOk = false;

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&nvv, sizeof(nvv)+2);

		if (p.CRC.w == 0) { loadVarsOk = true; break; };
	};

	if (!loadVarsOk)
	{
		nvv.numDevice = 0;
		nvv.index = 0;
		nvv.prevFilePage = -1;

		nvv.f.session = 0;
		nvv.f.size = 0;
		nvv.f.startPage = 0;
		nvv.f.lastPage = 0;
		GetTime(&nvv.f.start_rtc);
		GetTime(&nvv.f.stop_rtc);
		nvv.f.flags = 0;

		for (u32 i = 0; i < ArraySize(nvv.badBlocks); i++)
		{
			nvv.badBlocks[i] = 0;
		};

		savesCount = 2;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SaveVars()
{
	const u16 sa = 0x100;

	PointerCRC p(buf);

	static DSCI2C dsc;
	static u16 romAdr = 0;

	static byte i = 0;
//	static RTM32 tm;

	switch (i)
	{
		case 0:

			if (savesCount > 0)
			{
				savesCount--;
				i++;
			}
			else if (savesSessionsCount > 0)
			{
				savesSessionsCount--;
				i = 3;
			}
			else if (eraseSessionsCount > 0)
			{
				eraseSessionsCount--;

				for (u16 n = 0; n < ArraySize(nvsi); n++)
				{
					nvsi[n].f.size = 0;
					nvsi[n].crc = 0;
				};

				nvv.f.session += 1;
				nvv.f.size = 0;
				//nvv.f.startPage = 0;
				//nvv.f.lastPage = 0;
				nvv.index = 0;

				savesCount = 1;

				i = 4;
			};

			break;

		case 1:


			romAdr = ReverseWord(0);

			dsc.wdata = &romAdr;
			dsc.wlen = sizeof(romAdr);
			dsc.wdata2 = buf;
			dsc.wlen2 = sizeof(buf);
			dsc.rdata = 0;
			dsc.rlen = 0;
			dsc.adr = 0x50;

			for (byte j = 0; j < 2; j++)
			{
				p.CRC.w = 0xFFFF;
				p.WriteArrayB(&nvv, sizeof(nvv));
				p.WriteW(p.CRC.w);
			};

			i = (I2C_AddRequest(&dsc)) ? (i+1) : 0;

			break;

		case 2:

			if (dsc.ready)
			{
				i = 0;
			};

			break;

		case 3:

			{
				NVSI &si = nvsi[nvv.index];

				//u32 adr = sa+sizeof(si)*nvv.index;

				romAdr = ReverseWord(sa+sizeof(si)*nvv.index);

				dsc.wdata = &romAdr;
				dsc.wlen = sizeof(romAdr);
				dsc.wdata2 = buf;
				dsc.wlen2 = sizeof(si);
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.adr = 0x50;

				p.CRC.w = 0xFFFF;
				p.WriteArrayB(&si, sizeof(si.f));
				p.WriteW(p.CRC.w);

				i = (I2C_AddRequest(&dsc)) ? 2 : 0;
			};

			break;

		case 4:

			romAdr = ReverseWord(sa);

			dsc.wdata = &romAdr;
			dsc.wlen = sizeof(romAdr);
			dsc.wdata2 = nvsi;
			dsc.wlen2 = sizeof(nvsi);
			dsc.rdata = 0;
			dsc.rlen = 0;
			dsc.adr = 0x50;

			i = (I2C_AddRequest(&dsc)) ? 2 : 0;

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadSessions()
{
	PointerCRC p(buf);

	static DSCI2C dsc;
	static u16 romAdr = 0;

	const u16 sa = 0x100;

	loadSessionsOk = true;

	for (u16 i = 0; i < ArraySize(nvsi); i++)
	{
		NVSI &si = nvsi[i];

		//u32 adr = sa+sizeof(si)*i;

		romAdr = ReverseWord(sa+sizeof(si)*i);

		dsc.wdata = &romAdr;
		dsc.wlen = sizeof(romAdr);
		dsc.wdata2 = 0;
		dsc.wlen2 = 0;
		dsc.rdata = &si;
		dsc.rlen = sizeof(si);
		dsc.adr = 0x50;

		if (I2C_AddRequest(&dsc))
		{
			while (!dsc.ready);
		};

		if (GetCRC16(&si, sizeof(si)) != 0)
		{
			loadSessionsOk = false;

			si.f.session = 0;
			si.f.size = 0;
			si.f.startPage = 0;
			si.f.lastPage = 0;
			si.f.flags = 0;
			si.f.start_rtc.date = 0;
			si.f.start_rtc.time = 0;
			si.f.stop_rtc.date = 0;
			si.f.stop_rtc.time = 0;
			si.crc = GetCRC16(&si, sizeof(si.f));

			dsc.wdata = &romAdr;
			dsc.wlen = sizeof(romAdr);
			dsc.wdata2 = &si;
			dsc.wlen2 = sizeof(si);
			dsc.rdata = 0;
			dsc.rlen = 0;
			dsc.adr = 0x50;

			I2C_AddRequest(&dsc);

			while (!dsc.ready);
		};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FLASH_Init()
{
	nandSZ = NAND_GetMemSize();

	LoadVars();

	LoadSessions();

	InitFlashBuffer();

//	InitCom();

	InitSessions();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FLASH_Update()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
//		CALL( UpdateCom();	);
		CALL( SaveVars();	);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
