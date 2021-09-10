#ifndef MEMORY_AT91SAM7X256_TRAP_H
#define MEMORY_AT91SAM7X256_TRAP_H

#include "rtc.h"
//#include "power.h"
#include "trap_def.h"


/************ Common ******************************************/
extern void TRAP_HandleRxData(Trap *t, u32 size);
extern void TRAP_HandleRxError();
extern void TRAP_Init();
extern void TRAP_Idle();
/************ Info ********************************************/
extern bool TRAP_INFO_SendCaptureIP(u32 old_ip, u16 old_port);
extern bool TRAP_INFO_SendLostIP(u32 new_ip, u16 new_port);
/************ TRACE *******************************************/
extern bool TRAP_TRACE_SendData(const char *pData, u32 size);
extern bool TRAP_TRACE_PrintString(const char *data, ...);
extern void TRAP_TRACE_PrintChar(char data);
extern void TRAP_TRACE_PrintDec(int number);
extern void TRAP_TRACE_PrintHex(u32 number);
/************ MEMORY *******************************************/
extern bool TRAP_MEMORY_SendInfo();
extern bool TRAP_MEMORY_SendStatus(u32 progress, byte status);
extern bool TRAP_MEMORY_SendSession(u16 session, i64 size, i64 last_adress, RTC_type start_rtc, RTC_type stop_rtc, byte flags);
extern bool TRAP_MEMORY_SendVector(u16 session, u16 device, RTC_type rtc, byte *data, u16 size, byte flags);
/************ CLOCK *******************************************/
extern bool TRAP_CLOCK_SendMain(/*const RTC_type &rtc*/);
/************ BATTERY *************************************************/
extern void TRAP_BATTERY_SendMain();
extern void TRAP_BATTERY_SendStatus();
/************ SENSORS *************************************************/
extern void TRAP_SENSORS_SendMain();
/************ VECTOR **************************************************/
extern bool TRAP_VECTOR_SendVector(u16 command, u32 time_ms, int depth, int speed, byte flags, u16 size, byte *data);
/************ ONLINE **************************************************/
extern void TRAP_ONLINE_SendStatus(byte status);
/************ RDC **************************************************/
extern void TRAP_RDC_SendMain(u32 time_ms, int depth_sm, int speed_mh);
extern void TRAP_RDC_SendStatus(bool messaging, bool imitation);
/**************************************************************/

#endif


