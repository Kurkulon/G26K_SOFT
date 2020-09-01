#include "time.h"
#include <string.h>
#include "tftp_def.h"
//#include "flash.h"
#include "main.h"
#include "emac.h"
#include "list.h"

enum 
{
	TFTP_WAIT = 0, TFTP_READ_START, TFTP_READ_UPDATE, TFTP_WRITE_START,  TFTP_WRITE_UPDATE,
};


//unsigned char tftp_tx_buffer[TFTP_TX_BUFFER_SIZE];
//unsigned char tftp_data_buffer[TFTP_DATA_BUFFER_SIZE];

unsigned char tftp_read_command = TFTP_COMMAND_NONE;
unsigned short tftp_read_block = 0;
unsigned short tftp_read_block_ready = 0;
unsigned short tftp_read_block_size = 0;
unsigned char tftp_write_command = TFTP_COMMAND_NONE;
unsigned short tftp_write_block = 0;
unsigned short tftp_write_block_req = 0;
unsigned short tftp_write_block_ready = 0;
unsigned short tftp_write_block_size = 0;

u16 tftp_request = 0;
u16 tftp_processed = 0;
u16 tftp_err_opcode = 0;

//static byte tftp_state = 0;
static MAC ComputerEmacAddr = {0,0};	// Our Ethernet MAC address and IP address
static u32 ComputerIpAddr	= IP32(192,168,3,254);
static u16 ComputerUDPPort	= 0;
//static u32 ComputerOldIpAddr	= 0;
//static u16 ComputerOldUDPPort	= 0;
//static bool ComputerFind = false;

//static TrapReq  traps[10];

//static List<Receive_Desc> freeTrapList;
static List<MEMB> reqTrapList;

static TM32 tm;
static TM32 reboot;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct	TFTPHdr
{
	u16		opcode;

	__packed union
	{
		__packed struct { 				byte data[2]; } RRQ;
		__packed struct { 				byte data[2]; } WRQ;
		__packed struct { u16 block; 	byte data[0]; } DATA;
		__packed struct { u16 block; 	byte data[0]; } ACK;
		__packed struct { u16 error; 	char data[0]; } ERROR;
	};
};	

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct	EthTftp
{
	EthUdp	eudp;
	TFTPHdr	tftp;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct SmallTx : public EthUdpBuf
{
	TFTPHdr th;		// 4

	byte data[128];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct HugeTx : public SmallTx
{
	byte exdata[IP_MTU - sizeof(UdpHdr) - sizeof(TFTPHdr) - sizeof(data)];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SendFragTFTP(SmallTx *p)
{
	if (!EmacIsConnected())
	{
		return;
	};

	p->eth.dest = ComputerEmacAddr;

	p->iph.dst = ComputerIpAddr;	

	if (p->iph.off == 0) { p->iph.id = GetIpID(); };

	TransmitFragUdp(p, ComputerUDPPort);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SendTFTP(SmallTx *p)
{
	p->iph.off = 0;

	SendFragTFTP(p);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool RequestTFTP(MEMB* mb)
{
	EthUdp *h = (EthUdp*)mb->data;

	h->udp.len = ReverseWord(h->udp.len);

	reqTrapList.Add(mb);

	tftp_request++;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SmallTx* GetSmallTxBuffer()
{
	static byte		indSmallTx = 0;
	static SmallTx	smallTxBuf[16];

	SmallTx *p = &smallTxBuf[indSmallTx];

	if (p->len == 0)
	{
		p->len = 1;
	}
	else
	{
		p = 0;
	};

	indSmallTx += 1;

	if (indSmallTx >= ArraySize(smallTxBuf)) indSmallTx = 0;

	return p;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

HugeTx* GetHugeTxBuffer()
{
	static byte		indHugeTx = 0;
	static HugeTx	hugeTxBuf[8];

	HugeTx *p = &hugeTxBuf[indHugeTx];

	if (p->len == 0)
	{
		p->len = 1;
	}
	else
	{
		p = 0;
	};

	indHugeTx += 1;

	if (indHugeTx >= ArraySize(hugeTxBuf)) indHugeTx = 0;

	return p;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/*********************TX****************************************************/
static void TFTP_SendData(SmallTx *buf, u16 block, u16 size)
{
	if (buf == 0) return;

	buf->th.opcode = SWAP16(TFTP_OPCODE_DATA);
	buf->th.DATA.block = SWAP16(block);
	buf->len = sizeof(EthUdp) + sizeof(buf->th)+size;

	SendTFTP(buf);	
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void TFTP_SendAck(u16 block)
{
	SmallTx *buf = GetSmallTxBuffer();

	if (buf == 0) return;

	buf->th.opcode = SWAP16(TFTP_OPCODE_ACK);
	buf->th.ACK.block = SWAP16(block);
	buf->len = sizeof(EthUdp) + sizeof(buf->th);

	SendTFTP(buf);	
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void TFTP_SendError(u16 error, char *message)
{
	SmallTx *buf = GetSmallTxBuffer();

	if (buf == 0) return;

	buf->th.opcode = SWAP16(TFTP_OPCODE_ERROR);
	buf->th.ERROR.error = SWAP16(error);

	u32 l = strlcpy(buf->th.ERROR.data, message, sizeof(buf->data));

	buf->len = sizeof(EthUdp) + sizeof(buf->th) + l;

	SendTFTP(buf);	
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void TFTP_SendVersion(u16 version)
{
	SmallTx *buf = GetSmallTxBuffer();

	if (buf == 0) return;

	buf->th.opcode = SWAP16(TFTP_OPCODE_DATA);
	buf->th.DATA.block = SWAP16(1);
	
	DataPointer p(buf->th.DATA.data);

	p.w[0] = version;

	buf->len = sizeof(EthUdp) + sizeof(buf->th) + 2;

	SendTFTP(buf);	
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void TFTP_SendMode(byte mode)
{
	SmallTx *buf = GetSmallTxBuffer();

	if (buf == 0) return;

	buf->th.opcode = SWAP16(TFTP_OPCODE_DATA);
	buf->th.DATA.block = SWAP16(1);
	
	buf->th.DATA.data[0] = mode;

	buf->len = sizeof(EthUdp) + sizeof(buf->th) - sizeof(buf->th.DATA.data) + 1;

	SendTFTP(buf);	
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool TFTP_HandleRxData(MEMB *mb)
{
	if (mb == 0) return false;

	tftp_processed++;

	EthTftp *h = (EthTftp*)mb->data;
	
	u16 len = h->eudp.udp.len;

	if(len < sizeof(TFTPHdr)) 
	{ 
		TFTP_SendError(TFTP_ERRORCODE_UNKNOWN_ID, "");
		return false; 
	};

	bool c = false;

	TFTPHdr &tftp = h->tftp;

	switch (SWAP16(tftp.opcode))
	{
     	case TFTP_OPCODE_RRQ:

			if(strncmp((char*)tftp.RRQ.data, TFTP_FILE_VERSION, len) == 0)
			{
				TFTP_SendVersion(VERSION);
			}
			else if(strncmp((char*)tftp.RRQ.data, TFTP_FILE_MODE, len) == 0)
			{
				TFTP_SendMode(1/*FLASH_Get_Mode()*/);
			}
			else if(strncmp((char*)tftp.RRQ.data, TFTP_FILE_PROGRAMM, len) == 0)
			{
				tftp_read_command = TFTP_COMMAND_PROGRAMM;
				tftp_read_block = 1;
				tftp_read_block_ready = 0;
			}	
			else 
			{
				tftp_read_command = TFTP_COMMAND_NONE;
				TFTP_SendError(TFTP_ERRORCODE_FILE_NOT_FOUND, "");
			};

			break;

     	case TFTP_OPCODE_WRQ:

			if (strncmp((char*)tftp.WRQ.data, TFTP_FILE_VERSION, len) == 0)
			{
				TFTP_SendError(TFTP_ERRORCODE_ACCESS_VIOLATION, "");
			}
			else if (strncmp((char*)tftp.WRQ.data, TFTP_FILE_MODE, len) == 0)
			{
				tftp_write_command = TFTP_COMMAND_MODE;
				TFTP_SendAck(0);
			}
			else if(strncmp((char*)tftp.WRQ.data, TFTP_FILE_PROGRAMM, len) == 0)
			{
				tftp_write_command = TFTP_COMMAND_PROGRAMM;
				tftp_write_block = 0;
				tftp_write_block_ready = 0;
				tftp_write_block_req = 0;
				TFTP_SendAck(0);
				InitFlashWrite();
			}	
			else 
			{
				tftp_write_command = TFTP_COMMAND_NONE;
				TFTP_SendError(TFTP_ERRORCODE_FILE_NOT_FOUND, "");
			};

			break;

     	case TFTP_OPCODE_ACK:

			if((SWAP16(tftp.ACK.block) == tftp_read_block_ready) && tftp_read_block)
			{
				tftp_read_block ++;
			}
			else
			{
   				tftp_read_block = 0;
				tftp_read_block_ready = 0;
			};

			break;

     	case TFTP_OPCODE_DATA:

			if (tftp_write_command == TFTP_COMMAND_PROGRAMM)
			{
				if(SWAP16(tftp.DATA.block) == (tftp_write_block_req + 1))
				{
					tftp_write_block = tftp_write_block_req + 1;
					tftp_write_block_req++;
					tftp_write_block_size = h->eudp.udp.len - sizeof(h->eudp.udp) - sizeof(tftp) + sizeof(tftp.DATA.data);

					mb->flwb.adr = (tftp_write_block - 1) * TFTP_DATA_CHUNK_SIZE;
					mb->flwb.dataLen = tftp_write_block_size; //TFTP_DATA_CHUNK_SIZE;
					mb->flwb.dataOffset = h->tftp.DATA.data - mb->flwb.data;
					
					c = RequestFlashWrite(mb);

					tm.Reset();
				};
			}
			else if (tftp_write_command == TFTP_COMMAND_MODE)
			{
				if(SWAP16(tftp.DATA.block) == 1 && tftp.DATA.data[0] == 0)
				{
					TFTP_SendAck(1);
					tftp_write_command = TFTP_COMMAND_REBOOT;
					reboot.Reset();
				};
			}
			else
			{
				tftp_write_block = 0;
				tftp_write_block_ready = 0;
				TFTP_SendError(TFTP_ERRORCODE_ILLEGAL_OPERATION, "");
			};

			break;

     	case TFTP_OPCODE_ERROR:

			break;

		default:

			tftp_err_opcode++;

			break;
	};

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateRequestTFTP()
{
	static byte i = 0;
	static MEMB *mb = 0;

	EthUdp* h = (EthUdp*)mb->data;

	switch(i)
	{
		case 0:

			mb = reqTrapList.Get();

			if (mb != 0)
			{
				i++;
			};

			break;

		case 1:

			ComputerEmacAddr	= h->eth.src;
			ComputerIpAddr		= h->iph.src;
			ComputerUDPPort		= h->udp.src;

			if (!TFTP_HandleRxData(mb))
			{
				FreeMemBuffer(mb);
			};

			i = 0;

			break;
	};
} 

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TFTP_Idle()
{
	UpdateRequestTFTP();

	HugeTx* buf = 0;

	bool c = true;

    if(tftp_read_block > tftp_read_block_ready) // запрос на чтение
	{
		switch (tftp_read_command)
		{
			case TFTP_COMMAND_PROGRAMM:

				buf = GetHugeTxBuffer();

				if (buf != 0)
				{
					tftp_read_block_size = FLASH_Read((tftp_read_block - 1) * TFTP_DATA_CHUNK_SIZE, buf->th.DATA.data, TFTP_DATA_CHUNK_SIZE);

					TFTP_SendData(buf, tftp_read_block, tftp_read_block_size);
					tftp_read_block_ready = tftp_read_block;

					if (tftp_read_block_size < TFTP_DATA_CHUNK_SIZE) tftp_read_block = 0;
				};

				break;

			default:

				TFTP_SendError(TFTP_ERRORCODE_ILLEGAL_OPERATION, "");
				tftp_read_block = 0;
				break;
		};
	};

    if(tftp_write_block > tftp_write_block_ready) // запрос на запись
	{
		switch (tftp_write_command)
		{
			case TFTP_COMMAND_PROGRAMM:

				if (GetFlashWriteOK() > tftp_write_block_ready || tm.Check(500))
				{
					tftp_write_block_ready++;// = tftp_write_block;
		 			TFTP_SendAck(tftp_write_block_ready);
				}
				else if (GetFlashWriteError() != 0)
				{
					TFTP_SendError(TFTP_ERRORCODE_DISK_FULL, "");
					tftp_write_block = 0;
				};

				break;

			default:

				TFTP_SendError(TFTP_ERRORCODE_ILLEGAL_OPERATION, "");
				tftp_read_block = 0;

				break;
		};



	};

	if (tftp_write_command == TFTP_COMMAND_REBOOT && reboot.Timeout(100))
	{
		c = false;
	};

	return c;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void TFTP_Idle_old()
{
    if(tftp_read_block > tftp_read_block_ready) // запрос на чтение
	{
		switch (tftp_read_command)
		{
			case TFTP_COMMAND_VERSION:

//				*((unsigned short *)tftp_data_buffer) = VERSION;
                tftp_read_block_size = sizeof(unsigned short);
				break;

			case TFTP_COMMAND_MODE:

//				*((unsigned char *)tftp_data_buffer) = FLASH_Get_Mode();
				tftp_read_block_size = sizeof(unsigned char);
				break;

			case TFTP_COMMAND_PROGRAMM:
				
				//tftp_read_block_size = FLASH_Read((unsigned char *)((tftp_read_block - 1) * TFTP_DATA_CHUNK_SIZE), tftp_data_buffer, TFTP_DATA_CHUNK_SIZE);
				break;

			default:

				TFTP_SendError(TFTP_ERRORCODE_ILLEGAL_OPERATION, "");
				tftp_read_block = 0;
				break;
		};

		if(tftp_read_block)
		{
//			TFTP_SendData(tftp_read_block, tftp_data_buffer, tftp_read_block_size);
			tftp_read_block_ready = tftp_read_block;
			if(tftp_read_block_size < TFTP_DATA_CHUNK_SIZE) tftp_read_block = 0;
		}
	};

    if(tftp_write_block > tftp_write_block_ready) // запрос на запись
	{
		switch (tftp_write_command)
		{
			case TFTP_COMMAND_MODE:

				//FLASH_Set_Mode(*((unsigned char *)tftp_data_buffer));
				break;

			case TFTP_COMMAND_PROGRAMM:

//				if(!FLASH_Write((unsigned char *)((tftp_write_block - 1) * TFTP_DATA_CHUNK_SIZE), tftp_data_buffer, tftp_write_block_size))
				{
				    TFTP_SendError(TFTP_ERRORCODE_DISK_FULL, "");
					tftp_write_block = 0;
				};

				break;

			default:

				TFTP_SendError(TFTP_ERRORCODE_ILLEGAL_OPERATION, "");
				tftp_read_block = 0;

				break;
		};

		if(tftp_write_block)
		{
			tftp_write_block_ready = tftp_write_block;
		 	TFTP_SendAck(tftp_write_block_ready);
			if(tftp_write_block_size < TFTP_DATA_CHUNK_SIZE) tftp_write_block = 0;
		};
	};
}


