#ifndef HARDWARE_H__23_12_2013__11_37
#define HARDWARE_H__23_12_2013__11_37

#include "types.h"
#include "core.h"

#ifdef WIN32
#include <windows.h>
#endif

struct SHAFTPOS
{
	i32 pos;

	SHAFTPOS() : pos(0) {}
//	SHAFTPOS(i32 v) : pos(v) {}

	operator i32() { return pos >> 8; }
	i32 operator=(i32 v) { pos = v<<8; return v; }
	i32 operator+=(i32 v) { return (pos += v<<8)>>8; }
	i32 operator-=(i32 v) { return (pos -= v<<8)>>8; }

};

struct Rsp30 { u16 rw; u16 dir; u16 st; u16 sl; u16 data[200]; };

extern void InitHardware();
extern void UpdateHardware();
extern u16 GetCRC(const void *data, u32 len);
//extern void StartValve(bool dir, u32 tacho, u32 time, u16 lim);

inline i32 GetShaftPos() { extern i32 shaftPos; return shaftPos; }

inline void SetDestShaftPos(i32 v) { extern i32 destShaftPos; destShaftPos = v; }

inline u32 GetTachoCount() { extern u32 tachoCount; return tachoCount; }

extern void SetTargetRPM(u32 v);

//extern void SetDutyPWMDir(i32 v);

extern void SetDutyPWM(u16 v);

//extern void OpenValve(bool forced = false);
//extern void CloseValve(bool forced = false);
//extern Rsp30* GetRsp30();

extern byte motorState;
extern SHAFTPOS closeShaftPos;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MSTEN			(0x1)
#define SLVEN			(0x2)
#define MSTPENDING		(0x1)
#define MSTSTATE		(0xe)
#define MSTST_IDLE		(0x0)
#define MSTST_RX 		(0x2)
#define MSTST_TX 		(0x4)
#define MSTST_NACK_ADDR (0x6)
#define MSTST_NACK_TX	(0x8)
#define SLVPENDING		(0x100)
#define SLVSTATE		(0x600)
#define SLVST_ADDR		(0x000)
#define SLVST_RX 		(0x200)
#define SLVST_TX 		(0x400)
#define MSTCONTINUE		(0x1)
#define MSTSTART		(0x2)
#define MSTSTOP			(0x4)
#define SLVCONTINUE		(0x1)
#define SLVNACK			(0x2)
#define SLVDMA			8
#define SLVPENDINGEN	(0x100)
#define SLVDESELEN		(1<<15)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct DSCTWI
{
	void*	wdata;
	void*	rdata;
	void*	wdata2;
	u16		wlen;
	u16		wlen2;
	u16		rlen;
	byte	adr;
	bool	ready;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool Init_TWI();
extern bool Write_TWI(DSCTWI *d);
inline bool Read_TWI(DSCTWI *d) { return Write_TWI(d); }
extern bool Check_TWI_ready();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct TWI
//{
//	DSCTWI* dsc;
//
//	static byte *wrPtr;
//	static byte *rdPtr;
//	static u16 wrCount;
//	static u16 rdCount;
//	static byte adr;
//
//	TWI() : dsc(0) {}
//
//	bool Init(byte num);
//
//	bool Write(DSCTWI *d);
//	bool Read(DSCTWI *d) { return Write(d); }
//	bool Update();
//
//	static __irq void Handler_TWI();
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline bool IsMotorIdle()
{
	extern byte motorState;
	return motorState == 0 || motorState == 2 || motorState == 4;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u32 GetMotorStopTime()
{
	extern u32 stopTime;
	return stopTime;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u16 GetAP()
{
	extern u16 vAP;
	return vAP;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u16 GetCurrent()
{
	extern u16 curADC;
	return curADC;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u16 GetAvrCurrent()
{
	extern u16 avrCurADC;
	return avrCurADC;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u16 GetRPM()
{
	extern u16 rpm;
	return rpm;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u32 GetmotoCounter()
{
	extern u32 motoCounter;
	return motoCounter;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//inline void OpenValve(u32 tacho, u32 time, u16 lim)
//{
//	StartValve(true, tacho, time, lim);
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//inline void CloseValve(u32 tacho, u32 time, u16 lim)
//{
//	StartValve(false, tacho, time, lim);
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline bool CalibrateShaftPos()
{
	extern byte motorState;

	if (IsMotorIdle())
	{
		motorState = 5;
		return true;
	};

	return false;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline bool InitShaftPos()
{
	extern byte motorState;

	motorState = 9;
	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#ifndef WIN32


#else


#endif

#endif // HARDWARE_H__23_12_2013__11_37
