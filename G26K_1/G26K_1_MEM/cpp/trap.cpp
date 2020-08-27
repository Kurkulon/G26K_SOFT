#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "common.h"
#include "trap.h"
#include "main.h"
#include "flash.h"
#include "emac.h"
#include "hardware.h"

#include "trap_def.h"
#include "xtrap.h"

#pragma diag_suppress 2548,546,550,177

#pragma O3
#pragma Otime

enum trap_status
{
	TRAP_WAIT = 0,
	TRAP_SEND_SESSION,
	TRAP_SEND_VECTOR,
	TRAP_PAUSE_VECTOR
};

static trap_status trapStatus = TRAP_WAIT;


static const bool __trace = false;

//char TrapTxDataBuffer[TRAP_TX_DATA_BUFFER_SIZE];

u32 TrapRxCounter;
u32 TrapTxCounter;
u32 TrapRxLost;

static bool startSendVector = false;
static u16  startSession = 0;
static u64	startAdr = 0;

static bool startSendSession = false;
static bool stop = false;
static bool pause = false;


/******************************************************/
static void TRAP_MakePacketHeaders(char *data, bool need_ask, bool is_ask, char device);
static void MakePacketHeaders(TrapHdr *p, bool need_ask, bool is_ask, char device);

static void StartSendVector(u16 session, u64 adr);


/*********************** Info *************************/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_INFO_SendError(u32 error)
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

	TrapError &trap = (TrapError&)buf->th;

	MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_INFO_DEVICE);

	trap.hdr.cmd = TRAP_INFO_COMMAND_ERROR;
	trap.error = error;

	buf->len = sizeof(EthUdp) + sizeof(trap);
	
	SendTrap(buf);

	if (__trace) { TRAP_TRACE_PrintString(__func__); };

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_INFO_SendCaptureIP(u32 old_ip, u16 old_port)
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

	TrapIP &trap = (TrapIP&)buf->th;

	MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_INFO_DEVICE);
	trap.hdr.cmd = TRAP_INFO_CAPTURE_IP;
	trap.ip = old_ip;
	trap.port = old_port;

	buf->len = sizeof(EthUdp) + sizeof(trap);

	SendTrap(buf);

	if (__trace) { TRAP_TRACE_PrintString(__func__); };

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_INFO_SendLostIP(u32 new_ip, u16 new_port)
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

	TrapIP &trap = (TrapIP&)buf->th;

	MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_INFO_DEVICE);
	trap.hdr.cmd = TRAP_INFO_LOST_IP;
	trap.ip = new_ip;
	trap.port = new_port;

	buf->len = sizeof(EthUdp) + sizeof(trap);

	SendTrap(buf);

	if (__trace) { TRAP_TRACE_PrintString(__func__); };

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_INFO_SendInfo()
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

	TrapInfo &trap = (TrapInfo&)buf->th;

	MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_INFO_DEVICE);

	trap.hdr.cmd = TRAP_INFO_COMMAND_INFO;

	trap.version = VERSION;
	trap.number = GetNumDevice(); //FRAM_Main_Device_Number_Get();
	trap.memory_mask = FLASH_Chip_Mask_Get();
	trap.memory_size = FLASH_Full_Size_Get();
	trap.devices_mask = TRAP_DEVICES_MASK;
	trap.device_type = 1;// память 
	trap.device_telemetry = 2; // USART
	
	buf->len = sizeof(EthUdp) + sizeof(trap);

	SendTrap(buf);

	if (__trace) { TRAP_TRACE_PrintString(__func__); };

	return true;
}

/******************** CLOCK ******************************/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_CLOCK_SendMain(/*const RTC_type &rtc*/)
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

	TrapClock &trap = (TrapClock&)buf->th;

	MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_CLOCK_DEVICE);

	trap.hdr.cmd = TRAP_CLOCK_COMMAND_MAIN;
	
	GetTime(&trap.rtc);	

	buf->len = sizeof(EthUdp) + sizeof(trap);

	SendTrap(buf);

//	if (__trace) { TRAP_TRACE_PrintString(__func__); };

	return true;
}

/******************** TRACE ******************************/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_TRACE_SendData(const char *pData, u32 size)
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

//	Trap &trap = (TrapClock&)buf->th;

	MakePacketHeaders(&buf->th, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_TRACE_DEVICE);

	buf->th.cmd = TRAP_TRACE_COMMAND_MAIN;

	if (size > sizeof(buf->data))
	{
		size = sizeof(buf->data);
	};

	COPY(pData, (char*)buf->data, size);

	buf->len = sizeof(EthUdp) + sizeof(TrapHdr) + size;

	SendTrap(buf);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_TRACE_PrintString(const char *data, ...)
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

//	Trap &trap = (TrapClock&)buf->th;

	MakePacketHeaders(&buf->th, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_TRACE_DEVICE);

	buf->th.cmd = TRAP_TRACE_COMMAND_MAIN;

	va_list arglist;

    va_start(arglist, data);
    
	int i = vsnprintf((char*)buf->data, sizeof(buf->data) - 2, data, arglist);

//	u16 i = sizeof(buf->data) - 2;


	if (i < 0) i = 0;
	
	byte *dst = buf->data + i;

	//while((i > 0) && (*data != 0))
	//{
	//	*dst++ = *data++; i--;
	//};

	*dst++ = '\r';
	*dst++ = '\n';

	i += 2;

	buf->len = sizeof(EthUdp) + sizeof(TrapHdr) + i;

	SendTrap(buf);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void TRAP_TRACE_PrintChar(char data)
{
	TRAP_TRACE_SendData(&data, sizeof(data));
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void TRAP_TRACE_PrintDec(int number)
{
	char buf[11];
	byte i;
	u32 n = abs(number);
	for(i = 0; i < 11; i++)	
	{
	        buf[10 - i] = n - 10 * (n / 10) + 0x30;
		n /= 10;
	}
    	i = 0;
	while ((buf[i]=='0') && (i < 10)) i++;
	if(number < 0) 
	{
		i--;
		buf[i] = '-';
	}
	TRAP_TRACE_SendData(&buf[i], 11 - i);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void TRAP_TRACE_PrintHex(u32 number)
{
	char buf[10] = { '0' , 'x' };
	byte i;
	for(i = 0; i < 8; i++)	
	{
	        buf[7 - i + 2] = number - 16*(number/16);
		if(buf[7 - i + 2] >= 10) buf[7 - i + 2] += 0x41 - 10; else buf[7 - i + 2] += 0x30;
		number /= 16;
	}
	TRAP_TRACE_SendData(buf, sizeof(buf));
}

/******************** MEMORY ******************************/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_MEMORY_SendInfo()
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

	TrapMemInfo &trap = (TrapMemInfo&)buf->th;

	MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_MEMORY_DEVICE);

	trap.hdr.cmd = TRAP_MEMORY_COMMAND_INFO;

	trap.mask = FLASH_Chip_Mask_Get();
	trap.size = FLASH_Full_Size_Get();
	trap.size_used = FLASH_Used_Size_Get();

	buf->len = sizeof(EthUdp) + sizeof(trap);

	SendTrap(buf);

	if (__trace) { TRAP_TRACE_PrintString(__func__); };

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_MEMORY_SendStatus(u32 progress, byte status)
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

	TrapMemStatus &trap = (TrapMemStatus&)buf->th;

	MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_MEMORY_DEVICE);

	trap.hdr.cmd = TRAP_MEMORY_COMMAND_STATUS;

	trap.progress = progress;
	trap.status = status;

	buf->len = sizeof(EthUdp) + sizeof(trap);

	SendTrap(buf);

	if (__trace) { TRAP_TRACE_PrintString(__func__); };

	return true;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool TRAP_MEMORY_SendSession(u16 session, i64 size, i64 last_adress, RTC_type start_rtc, RTC_type stop_rtc, byte flags)
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

	TrapSession &trap = (TrapSession&)buf->th;

	MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_MEMORY_DEVICE);

	trap.hdr.cmd = TRAP_MEMORY_COMMAND_SESSION;

	trap.si.session = session;
	trap.si.size = size;
	trap.si.last_adress = last_adress;
	trap.si.start_rtc = start_rtc;
	trap.si.stop_rtc = stop_rtc;
	trap.si.flags = flags;

	buf->len = sizeof(EthUdp) + sizeof(trap);

	SendTrap(buf);

	if (__trace) { TRAP_TRACE_PrintString(__func__); };

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool TRAP_MEMORY_SendLastSession(const SessionInfo *si)
{
	return TRAP_MEMORY_SendSession(si->session, si->size, si->last_adress, si->start_rtc, si->stop_rtc, si->flags);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool TRAP_MEMORY_SendNullSession()
{
	SessionInfo si;

	si.size = 0;
	si.last_adress = -1;
	si.session = -1;

	return TRAP_MEMORY_SendSession(si.session, si.size, si.last_adress, si.start_rtc, si.stop_rtc, si.flags);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//bool TRAP_MEMORY_SendVector(u16 session, u16 device, RTC_type rtc, byte *data, u16 size, byte flags) 
//{
//	 // для скорости хорошо бы просто указать где лежит вектор, да тогда при прикреплении эзернет заголовка попортится сам вектор, а он нужен в дальнейшем, потому тупо копируем его
//	if(TRAP_TX_HEADERS_LEN + sizeof(TRAP_command_type) + sizeof(TRAP_MEMORY_vector_type) + size > sizeof(TrapTxDataBuffer)) return false;
//	TRAP_MakePacketHeaders((char *)TrapTxDataBuffer, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_MEMORY_DEVICE);
//	TRAP_command_type c;
//	c.command = TRAP_MEMORY_COMMAND_VECTOR;
//	COPY((char *)(&c.command), (char *)(TrapTxDataBuffer) + TRAP_TX_HEADERS_LEN, sizeof(TRAP_command_type));
//	TRAP_MEMORY_vector_type v;	
//	v.session = session;
//	v.device = device;
//	v.rtc = rtc;
//	v.flags = flags;
//	memmove((char *)(TrapTxDataBuffer) + TRAP_TX_HEADERS_LEN + sizeof(TRAP_command_type), (char *)(&v.session), sizeof(TRAP_MEMORY_vector_type));
//	memmove((char *)(TrapTxDataBuffer) + TRAP_TX_HEADERS_LEN + sizeof(TRAP_command_type) + sizeof(TRAP_MEMORY_vector_type), (char *)data, size);
//	EMAC_SendData((char *)TrapTxDataBuffer, TRAP_TX_HEADERS_LEN + sizeof(TRAP_command_type) + sizeof(TRAP_MEMORY_vector_type) + size);
//
//	if (__trace) { TRAP_TRACE_PrintString(__func__); };
//
//	return true;
//}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void TRAP_BATTERY_SendStatusMessage(u16 cmd)
{
	//TRAP_MakePacketHeaders((char *)TrapTxDataBuffer, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_BATTERY_DEVICE);
	//TRAP_command_type c;
	//c.command = cmd;
	//COPY((char *)(&c.command), (char *)(TrapTxDataBuffer) + TRAP_TX_HEADERS_LEN, sizeof(TRAP_command_type));
	//TRAP_BATTERY_status_type s;
	//FRAM_Power_Battery_Voltages_Get(&s.battery_setup_voltage, &s.battery_min_voltage, &s.battery_max_voltage);
	//FRAM_Power_Line_Voltages_Get(&s.line_setup_voltage, &s.line_min_voltage, &s.line_max_voltage);
	//FRAM_Power_Battery_Coeffs_Get(&s.battery_coeff_k, &s.battery_coeff_b);
	//FRAM_Power_Line_Coeffs_Get(&s.line_coeff_k, &s.line_coeff_b);
	//COPY((char *)(&s), (char *)(TrapTxDataBuffer) + TRAP_TX_HEADERS_LEN + sizeof(TRAP_command_type), sizeof(s));
	//EMAC_SendData((char *)TrapTxDataBuffer, TRAP_TX_HEADERS_LEN + sizeof(TRAP_command_type) + sizeof(s));
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/******************** SENSORS ******************************/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool TRAP_SendAsknowlege(byte device, u32 on_packet)
{
	SmallTx* buf = GetSmallTxBuffer();

	if (buf == 0) return false;

	TrapAsk &trap = (TrapAsk&)buf->th;

	MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_IS_ASK, device);

	trap.hdr.cmd = TRAP_COMMAND_ASKNOWLEGE;
	trap.on_packet = on_packet;

	buf->len = sizeof(EthUdp) + sizeof(TrapAsk);
	
	SendTrap(buf);

	if (__trace) { TRAP_TRACE_PrintString("%s('%c', 0x%08X)", __func__, device, on_packet); };

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void TRAP_HandleRxData(Trap *t, u32 size)
{
	if(size < TRAP_RX_HEADERS_LEN) { TRAP_INFO_SendError(TRAP_PACKET_ERROR_UNKNOW); return; }

//	TRAP_RX_PACKET_type *packet = (TRAP_RX_PACKET_type *)data;

	if((TrapRxCounter == 0) || (t->hdr.counter <= 1))
	{
		TrapRxLost = 0;
	}
	else
	{
		TrapRxLost += t->hdr.counter - TrapRxCounter - 1;	
	};

	TrapRxCounter = t->hdr.counter;

	bool need_ask = (((t->hdr.status)>>3)&0x1);

	bool is_ask = (((t->hdr.status)>>2)&0x1);

	byte version = t->hdr.version;

	if(version > TRAP_PACKET_VERSION)
	{
		TRAP_INFO_SendError(TRAP_PACKET_ERROR_VERSION);
		return;
	};

	if(is_ask == TRAP_PACKET_NO_ASK)
	{
//		TRAP_command_type *cmd = (TRAP_command_type *)(data + TRAP_RX_HEADERS_LEN);	

		if(size < TRAP_RX_HEADERS_LEN + sizeof(TRAP_command_type))
		{ 
			TRAP_INFO_SendError(TRAP_PACKET_ERROR_UNKNOW); 
			return; 
		};

		switch (t->hdr.device)
		{
			case TRAP_INFO_DEVICE: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

				TrapInfoSet &ts = (TrapInfoSet&)*t;

				switch (t->hdr.cmd)
				{
					case TRAP_INFO_COMMAND_GET_INFO:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_INFO_DEVICE, TrapRxCounter);					
						TRAP_INFO_SendInfo();
						break;

					case TRAP_INFO_COMMAND_SET_NUMBER:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_INFO_DEVICE, TrapRxCounter);
						SetNumDevice(ts.number);
						SaveMainParams();

						break;

					case TRAP_INFO_COMMAND_SET_TYPE:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_INFO_DEVICE, TrapRxCounter);
//						FRAM_Main_Device_Type_Set(ts.type);
						break;

					case TRAP_INFO_COMMAND_SET_TELEMETRY:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_INFO_DEVICE, TrapRxCounter);
//						FRAM_Main_Device_Telemetry_Set(ts.telemetry);
						break;

					default: 

						TRAP_INFO_SendError(TRAP_PACKET_ERROR_UNKNOW);
						break;
				};

				break;

			case TRAP_CLOCK_DEVICE: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

				switch (t->hdr.cmd)
				{
					case TRAP_CLOCK_COMMAND_GET:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_CLOCK_DEVICE, TrapRxCounter);					
						TRAP_CLOCK_SendMain(/*RTC_Get()*/);
						break;

					case TRAP_CLOCK_COMMAND_SET:

						TrapClock &tc = (TrapClock&)*t;

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_CLOCK_DEVICE, TrapRxCounter);	
						SetClock(tc.rtc);

						if (__trace) { TRAP_TRACE_PrintString(" TRAP_CLOCK_COMMAND_SET \r\n"); };

						break;

					default: 

						TRAP_INFO_SendError(TRAP_PACKET_ERROR_UNKNOW);
						break;
				}
				break;

			case TRAP_MEMORY_DEVICE: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

				switch (t->hdr.cmd)
				{
					case TRAP_MEMORY_COMMAND_GET_INFO:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_MEMORY_DEVICE, TrapRxCounter);					
						TRAP_MEMORY_SendInfo();
						break;

					case TRAP_MEMORY_COMMAND_READ_SESSION_START:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_MEMORY_DEVICE, TrapRxCounter);

						StartSendSession();

						//TRAP_MEMORY_SendLastSession(GetLastSessionInfo());
						//TRAP_MEMORY_SendStatus(-1, FLASH_STATUS_READ_SESSION_READY);

//						TRAP_MEMORY_SendNullSession();

//						Mode_Ethernet_Flash_Read_Session_Start();
						break;

					case TRAP_MEMORY_COMMAND_READ_VECTOR_START:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_MEMORY_DEVICE, TrapRxCounter);

						TrapReadVector &tr = (TrapReadVector&)*t;

						StartSendVector(tr.session, tr.last_adress);

//						Mode_Ethernet_Flash_Read_Vector_Start(tr.session, tr.last_adress);

						break;

					case TRAP_MEMORY_COMMAND_STOP:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_MEMORY_DEVICE, TrapRxCounter);					

						stop = true;

						TRAP_MEMORY_SendStatus(-1, FLASH_STATUS_STOP);

//						Mode_Ethernet_Flash_Stop();
						break;

					case TRAP_MEMORY_COMMAND_PAUSE:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_MEMORY_DEVICE, TrapRxCounter);	

						pause = true;

						TRAP_MEMORY_SendStatus(-1, FLASH_STATUS_PAUSE);

						break;

					case TRAP_MEMORY_COMMAND_RESUME:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_MEMORY_DEVICE, TrapRxCounter);

						pause = false;

						TRAP_MEMORY_SendStatus(-1, FLASH_STATUS_RESUME);

						break;

					case TRAP_MEMORY_COMMAND_ERASE:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_MEMORY_DEVICE, TrapRxCounter);	

						NAND_FullErase();

//						Mode_Ethernet_Flash_Erase();
						break;

					case TRAP_MEMORY_COMMAND_UNERASE:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_MEMORY_DEVICE, TrapRxCounter);					
//						Mode_Ethernet_Flash_UnErase();
						break;

					default: 

						TRAP_INFO_SendError(TRAP_PACKET_ERROR_UNKNOW);
						break;
				};

				break;

			case TRAP_BOOTLOADER_DEVICE: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

				switch (t->hdr.cmd)
				{
					case TRAP_BOOTLOADER_COMMAND_START:

						if(need_ask == TRAP_PACKET_NEED_ASK) TRAP_SendAsknowlege(TRAP_BOOTLOADER_DEVICE, TrapRxCounter);
//						BootLoader_Start_Delay();

						break;

					default: 

						TRAP_INFO_SendError(TRAP_PACKET_ERROR_UNKNOW);

						break;
				};

				break;



			default: //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

				TRAP_INFO_SendError(TRAP_PACKET_ERROR_UNKNOW);

				break;

		}
	}
	else 
	{
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void EMAC_HandleRxError()
{
	TRAP_INFO_SendError(TRAP_PACKET_ERROR_CHECKSUM);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void MakePacketHeaders(TrapHdr *p, bool need_ask, bool is_ask, char device)
{
//	TRAP_TX_PACKET_type p;
	p->counter = TrapTxCounter++;
	p->errors = (u16)TrapRxLost;
	p->version = TRAP_PACKET_VERSION;
	p->status = ((is_ask&0x1)<<2) | ((need_ask&0x1)<<3);
	p->device = device;
//        COPY((char *)(&p.counter), (char *)data, sizeof(TRAP_TX_PACKET_type));
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void TRAP_MakePacketHeaders(char *data, bool need_ask, bool is_ask, char device)
//{
//	TRAP_TX_PACKET_type p;
//	p.counter = TrapTxCounter++;
//	p.errors = (u16)TrapRxLost;
//	p.version = TRAP_PACKET_VERSION;
//	p.status = ((is_ask&0x1)<<2) | ((need_ask&0x1)<<3);
//	p.device = device;
//        COPY((char *)(&p.counter), (char *)data, sizeof(TRAP_TX_PACKET_type));
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void StartSendVector(u16 session, u64 adr)
{
	startSendVector = true;
	stop = false;
	pause = false;
	startSession = session;
	startAdr = adr;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool UpdateSendVector()
{
	static byte i = 0;
	static FLRB flrb;

	static HugeTx *t = 0;
//	static VecData::Hdr h;

	static u64 vecCount = 0;
	static u32 fragLen = 0;
	static u32 fragOff = 0;
	static u16 ipID = 0;
	static u16 crc = 0;

	static TM32 tm;

	static u16 ses = 0;
	static u64 adr = 0;
	static u64 size = 0;
	static bool useadr = false;

	static FileDsc *si = 0;

	__packed struct TRP { EthUdp eu; TrapVector tv; byte data[IP_MTU - sizeof(UdpHdr) - sizeof(TrapVector)]; };
	__packed struct FR  { EthIp  ei; byte data[IP_MTU]; };

	//TRP &et = *((TRP*)&t->eth);
	//FR  &ef = *((FR*)&t->eth);

	//TrapVector &trap = et.tv;

	//if (tm.Check(1000))
	//{
	//	while((t = GetHugeTxBuffer()) == 0);
	//	while((t2 = GetHugeTxBuffer()) == 0);

	//	ipID = GetIpID(); 

	//	t->iph.id = ipID;
	//	t->iph.off = 0x2000;
	//	t->len = sizeof(EthIp) + 1480;

	//	SendFragTrap(t);

	//	t2->iph.id = ipID;
	//	t2->iph.off = (1480/8);
	//	t2->len = sizeof(EthIp) + 1480;

	//	SendFragTrap(t2);

	//	return true;
	//}
	//else
	//{
	//	return true;
	//};


	switch (i)
	{
		case 0:

			if (startSendVector)
			{
				startSendVector = false;
				adr = startAdr;
				ses = startSession;
				useadr = true;

				si = GetSessionInfo(ses, adr);

				if (si != 0)
				{
					size = si->size;
				};

				vecCount = 0;

				i++;
			}
			else
			{
				return false;
			};

			break;

		case 1:

			if (tm.Check(200))
			{
				TRAP_MEMORY_SendStatus(vecCount * (1<<22) / (size/1024), FLASH_STATUS_READ_VECTOR_IDLE);
			}
			//else
			//{
			//	break;
			//};

			if (stop)
			{
				stop = false;

				i = 0;
			}
			else if (!pause)
			{
				t = GetHugeTxBuffer();

				if (t != 0)
				{
					TRP &et = *((TRP*)&t->eth);

					flrb.data = et.data;
					flrb.maxLen = sizeof(et.data);
					flrb.vecStart = true;
					flrb.useAdr = useadr;
					flrb.adr = adr;

					RequestFlashRead(&flrb);

					useadr = false;

					i++;
				};
			};

			break;

		case 2:

			if (flrb.ready)
			{
				if (flrb.len == 0 || flrb.hdr.session != ses)
				{
					t->len = 0;

					TRAP_MEMORY_SendStatus(-1, FLASH_STATUS_READ_VECTOR_READY);

					stop = false;
					pause = false;

					i = 0;
				}
				else // if (flrb.hdr.crc == 0)
				{
					TRP &et = *((TRP*)&t->eth);

					TrapVector &trap = et.tv;

					MakePacketHeaders(&trap.hdr, TRAP_PACKET_NO_NEED_ASK, TRAP_PACKET_NO_ASK, TRAP_MEMORY_DEVICE);

					trap.hdr.cmd = TRAP_MEMORY_COMMAND_VECTOR;
					trap.session = flrb.hdr.session;
					trap.device = flrb.hdr.device;
					trap.rtc = flrb.hdr.rtc;
					trap.flags = flrb.hdr.flags;

					//if (trap.rtc.msec == 0)
					//{
					//	__breakpoint(0);
					//};

					vecCount += flrb.hdr.dataLen;

					ipID = GetIpID(); 

					t->iph.id = ipID;
					t->iph.off = 0;

					t->len = sizeof(et.eu) + sizeof(et.tv) + flrb.len;

					crc = CRC_CCITT_DMA(flrb.data, flrb.len, 0xFFFF);	// GetCRC16(flrb.data, flrb.len, 0xFFFF, 0);

					if (flrb.hdr.dataLen > flrb.maxLen)
					{
						fragOff = t->len - sizeof(EthIp); //flrb.maxLen;
						fragLen = flrb.hdr.dataLen - flrb.maxLen;

						t->iph.off |= 0x2000;

						t->udp.len = sizeof(UdpHdr) + sizeof(trap) + flrb.hdr.dataLen - 2;

						i = 3;
					}
					else
					{
						t->len -=  (crc != 0) ? flrb.len : 2;

						i = 1;
					};

					SendFragTrap(t);
				};
				//else
				//{
				//	// найти следующий вектор
				//	__breakpoint(0);
				//};

			};

			break;

		case 3:

			t = GetHugeTxBuffer();

			if (t != 0)
			{
				FR  &ef = *((FR*)&t->eth);

				flrb.data = ef.data;
				flrb.maxLen = sizeof(ef.data);
				flrb.vecStart = false;
				flrb.useAdr = false;

				if (flrb.maxLen > fragLen) { flrb.maxLen = fragLen; };

				RequestFlashRead(&flrb);

				i++;
			};

			break;

		case 4:

			if (flrb.ready)
			{
				FR  &ef = *((FR*)&t->eth);

				t->iph.id = ipID;
				t->iph.off = (fragOff/8)&0x1FFF;

				fragLen -= flrb.len;
				fragOff += flrb.len;

				t->len = sizeof(ef.ei) + flrb.len;

				crc = CRC_CCITT_DMA(flrb.data, flrb.len, crc); //GetCRC16(flrb.data, flrb.len, crc, 0);

				if (fragLen > 0)
				{ 
					t->iph.off |= 0x2000; 
				}
				else if (crc != 0)
				{
					t->iph.off = 0;
				}
				else
				{
					t->len -= 2;
				};

				SendFragTrap(t);

				if (fragLen > 0)
				{
					i = 3;
				}
				else
				{
					i = 1;
				};
			};

			break;
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void TRAP_Init()
{
	TrapRxCounter = 0;
	TrapTxCounter = 0;
	TrapRxLost = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void TRAP_Idle()
{
	UpdateSendVector();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

