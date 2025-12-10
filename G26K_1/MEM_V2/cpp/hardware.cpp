#include "hardware.h"

#include <types.h>
#include <core.h>
#include <time.h>
#include <CRC\CRC16_8005.h>
#include <DMA\DMA.h>
#include <SEGGER_RTT\SEGGER_RTT.h>


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

//#include <windows.h>
//#include <Share.h>
#include <conio.h>
#include <stdarg.h>
#include <stdio.h>
//#include <intrin.h>
#include <CRC16_CCIT.h>
#include <list.h>

static u16 crc_ccit_result = 0;

#else

//#pragma O3
//#pragma Otime

#endif 

#define GEAR_RATIO 12.25

const u16 pulsesPerHeadRoundFix4 = GEAR_RATIO * 6 * 16;

//const u16 testNandChipMask = 0xFFFF;

static volatile u32 shaftCounter = 0;
static volatile u32 shaftPrevTime = 0;
static volatile u32 shaftCount = 0;
static volatile u32 shaftTime = 0;
u16 shaftRPS = 0;
volatile u16 curShaftCounter = 0;
volatile u32 shaftCYCCNT = 0;


//static void I2C_Init();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//static void InitVectorTable();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

__forceinline 	void EnableVCORE()	{ PIO_ENVCORE->CLR(ENVCORE); 	}
__forceinline 	void DisableVCORE()	{ PIO_ENVCORE->SET(ENVCORE); 	}
				void EnableDSP()	{ PIO_RESET->CLR(RESET); 		}
				void DisableDSP()	{ PIO_RESET->SET(RESET); 		}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <ARM\system_imp.h>

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <Clock_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <CRC\CRC_CCITT_DMA_IMP.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <time_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*----------------------------------------------------------------------------
  Initialize the system
 *----------------------------------------------------------------------------*/



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void WDT_Init()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "WDT Init ... ");

	#ifdef CPU_SAME53	

		//HW::MCLK->APBAMASK |= APBA_WDT;

		//HW::WDT->CONFIG = WDT_WINDOW_CYC512|WDT_PER_CYC1024;
	
		#ifndef _DEBUG
//		HW::WDT->CTRLA = WDT_ENABLE|WDT_WEN|WDT_ALWAYSON;
		#else
//		HW::WDT->CTRLA = WDT_ENABLE|WDT_WEN|WDT_ALWAYSON;
		#endif

		//while(HW::WDT->SYNCBUSY);

	#elif defined(CPU_XMC48)

		#ifndef _DEBUG
	
		//HW::WDT_Enable();

		//HW::WDT->WLB = OFI_FREQUENCY/2;
		//HW::WDT->WUB = (3 * OFI_FREQUENCY)/2;
		//HW::SCU_CLK->WDTCLKCR = 0|SCU_CLK_WDTCLKCR_WDTSEL_OFI;

		//HW::WDT->CTR = WDT_CTR_ENB_Msk|WDT_CTR_DSP_Msk;

		#else

		HW::WDT_Disable();

		#endif

	#endif

	SEGGER_RTT_WriteString(0, "OK\n");
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53	

	#define SyncTmr				HW::SYNC_TCC
	#define RotTmr				HW::ROT_TCC
	#define SYNC_GEN			CONCAT2(GEN_,SYNC_TCC)
	#define SYNC_GEN_CLK		CONCAT2(CLK_,SYNC_TCC) 
	#define ROT_GEN				CONCAT2(GEN_,ROT_TCC)
	#define ROT_GEN_CLK			CONCAT2(CLK_,ROT_TCC) 

	#if (SYNC_GEN_CLK > 100000000)
			#define SYNC_PRESC_NUM		256
	#elif (SYNC_GEN_CLK > 50000000)
			#define SYNC_PRESC_NUM		64
	#elif (SYNC_GEN_CLK > 20000000)
			#define SYNC_PRESC_NUM		16
	#elif (SYNC_GEN_CLK > 10000000)
			#define SYNC_PRESC_NUM		8
	#elif (SYNC_GEN_CLK > 5000000)
			#define SYNC_PRESC_NUM		4
	#else
			#define SYNC_PRESC_NUM		1
	#endif

	#if (ROT_GEN_CLK > 100000000)
			#define ROT_PRESC_NUM		256
	#elif (ROT_GEN_CLK > 50000000)
			#define ROT_PRESC_NUM		64
	#elif (ROT_GEN_CLK > 20000000)
			#define ROT_PRESC_NUM		16
	#elif (ROT_GEN_CLK > 10000000)
			#define ROT_PRESC_NUM		8
	#elif (ROT_GEN_CLK > 5000000)
			#define ROT_PRESC_NUM		4
	#else
			#define ROT_PRESC_NUM		1
	#endif

	#define SYNC_PRESC_DIV		CONCAT2(TCC_PRESCALER_DIV,SYNC_PRESC_NUM)
	#define ROT_PRESC_DIV		CONCAT2(TCC_PRESCALER_DIV,ROT_PRESC_NUM)

	#define US2ROT(v)			(((v)*(ROT_GEN_CLK/1000/ROT_PRESC_NUM)+500)/1000)
	#define US2SYNC(v)			(((v)*(SYNC_GEN_CLK/1000/SYNC_PRESC_NUM)+500)/1000)


	inline void Sync_ClockEnable()  { HW::GCLK->PCHCTRL[CONCAT2(GCLK_,SYNC_TCC)] = SYNC_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(CONCAT2(PID_,SYNC_TCC)); }
	inline void Rot_ClockEnable()  { HW::GCLK->PCHCTRL[CONCAT2(GCLK_,ROT_TCC)]	 = ROT_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(CONCAT2(PID_,ROT_TCC)); }

#elif defined(CPU_XMC48)

	#define PIO_SYNC				HW::PORT_SYNC	
	#define PIO_ROT					HW::PORT_ROT	

	//P0_0_CCU
	#define SYNC_CCU_NUM			CONCAT4(PORT_SYNC,	_,	PIN_SYNC,	_CCU)
	#define ROT_CCU_NUM				CONCAT4(PORT_ROT,	_,	PIN_ROT,	_CCU)

	#define SYNC_CCU				CONCAT2(CCU, SYNC_CCU_NUM)
	#define ROT_CCU					CONCAT2(CCU, ROT_CCU_NUM)

	#if (SYNC_CCU_NUM >= 40) && (SYNC_CCU_NUM <= 43) && (SYNC_CCU_NUM == ROT_CCU_NUM)
		#define SYNC_ROT_CCU_NUM		SYNC_CCU_NUM
		#define SYNC_ROT_CCU_NAME		SYNC_CCU
	#else
		#error  SYNC_ROT_CCU ERROR!!!
	#endif

	#define SyncRotCCU_PID			CONCAT2(PID_,SYNC_ROT_CCU_NAME)

	#define SYNC_CC_NUM				CONCAT6(PORT_SYNC,	_,	PIN_SYNC,	_,	SYNC_ROT_CCU_NAME,	_CC)
	#define ROT_CC_NUM				CONCAT6(PORT_ROT,	_,	PIN_ROT,	_,	SYNC_ROT_CCU_NAME,	_CC)

	#define SYNC_CC					CONCAT3(SYNC_ROT_CCU_NAME, _CC4, SYNC_CC_NUM)
	#define ROT_CC					CONCAT3(SYNC_ROT_CCU_NAME, _CC4, ROT_CC_NUM)

									// P0_0_CCU40_OUT21
	#define SYNC_PINMODE			CONCAT7(PORT_SYNC,	_,	PIN_SYNC,	_,	SYNC_ROT_CCU_NAME,	_OUT,SYNC_CC_NUM)
	#define ROT_PINMODE				CONCAT7(PORT_ROT,	_,	PIN_ROT,	_,	SYNC_ROT_CCU_NAME,	_OUT,ROT_CC_NUM)

	#define SyncTmr					HW::SYNC_CC
	#define RotTmr					HW::ROT_CC
	#define SyncRotCCU				HW::SYNC_ROT_CCU_NAME
	#define Sync_GCSS				CONCAT3(CCU4_S, SYNC_CC_NUM,	SE)
	#define Rot_GCSS				CONCAT3(CCU4_S, ROT_CC_NUM,		SE)
	#define Sync_GIDLC				CONCAT3(CCU4_S, SYNC_CC_NUM,	I)
	#define Rot_GIDLC				CONCAT3(CCU4_S, ROT_CC_NUM,		I)

	#define SyncRot_GIDLC			(Sync_GIDLC|Rot_GIDLC|CCU4_PRB)
	//#define SyncRot_PSC				8					//1.28us
	//#define SyncRot_DIV				(1<<SyncRot_PSC)	

	#if (SYSCLK > 100000000)
			#define SYNC_PSC		8
			#define ROT_PSC			8
	#elif (SYSCLK > 50000000)
			#define SYNC_PSC		6
			#define ROT_PSC			6
	#elif (SYSCLK > 20000000)
			#define SYNC_PSC		4
			#define ROT_PSC			4
	#elif (SYSCLK > 10000000)
			#define SYNC_PSC		3
			#define ROT_PSC			3
	#elif (SYSCLK > 5000000)
			#define SYNC_PSC		2
			#define ROT_PSC			2
	#else
			#define SYNC_PSC		0
			#define ROT_PSC			0
	#endif

	#define SYNC_DIV				(1UL<<SYNC_PSC)	
	#define ROT_DIV					(1UL<<ROT_PSC)	

	#define US2ROT(v)				(((SYSCLK_MHz*(v)+ROT_DIV/2)/ROT_DIV))
	#define US2SYNC(v)				(((SYSCLK_MHz*(v)+SYNC_DIV/2)/SYNC_DIV))


#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Set_Sync_Rot(u16 RPS, u16 samplePerRound)
{
#ifndef WIN32

	u32 t = RPS;

	if (t == 0) t = 100;

	if (samplePerRound < 8) { samplePerRound = 8; };

	t *= samplePerRound;
	
	t = (100000000 + t/2) / t;
	
	t = US2SYNC(t);

	if (t > 0xFFFF) t = 0xFFFF;

	//u16 r = (samplePerRound + 36) / 72;
	
	u32 r = ((u32)RPS * pulsesPerHeadRoundFix4) >> 4;

	if (r != 0)
	{
		r = US2ROT((100000000 + r/2) / r);
	};

	if (r > 0xFFFF) r = 0xFFFF;

	#ifdef CPU_SAME53	

		SyncTmr->PER = t;
		SyncTmr->CC[0] = US2SYNC(10); 

		SyncTmr->CTRLA = (t != 0) ? TCC_ENABLE|SYNC_PRESC_DIV : SYNC_PRESC_DIV;

		RotTmr->CC[0] = r;

		RotTmr->CTRLA = (r != 0) ? TCC_ENABLE|ROT_PRESC_DIV : ROT_PRESC_DIV;

		SyncTmr->CTRLBSET = TCC_CMD_RETRIGGER;
		RotTmr->CTRLBSET = TCC_CMD_RETRIGGER;

	#elif defined(CPU_XMC48)

		PIO_SYNC->ModePin(PIN_SYNC, A3PP);
		PIO_ROT->ModePin(PIN_ROT, A3PP);

		HW::CCU_Enable(SyncRotCCU_PID);

		SyncRotCCU->GCTRL = 0;

		SyncRotCCU->GIDLC = SyncRot_GIDLC;

		SyncTmr->PRS = t-1;
		SyncTmr->CRS = US2SYNC(10)-1;
		SyncTmr->PSC = SYNC_PSC; 
		SyncTmr->PSL = 1; 

		if (t != 0) { SyncTmr->TCSET = CC4_TRBS; } else { SyncTmr->TCCLR = CC4_TRBC; };

		RotTmr->PRS = r-1;
		RotTmr->CRS = r/2;
		RotTmr->PSC = ROT_PSC; 
		RotTmr->TC = CC4_TCM;

		if (r != 0) { RotTmr->TCSET = CC4_TRBS; } else { RotTmr->TCCLR = CC4_TRBC; };

		SyncRotCCU->GCSS = Sync_GCSS|Rot_GCSS;  

	#endif

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32


static void Init_Sync_Rot()
{
	using namespace HW;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "Init_Sync_Rot ... ");

#ifdef CPU_SAME53	


	PIO_SYNC->SetWRCONFIG(SYNC, PMUX_SYNC|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_INEN);
	PIO_ROT->SetWRCONFIG(ROT,	PMUX_ROT|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_INEN);	

	Sync_ClockEnable();
	Rot_ClockEnable();

	//HW::GCLK->PCHCTRL[GCLK_TCC2_TCC3]	= GCLK_GEN(GEN_1M)|GCLK_CHEN;
	//HW::GCLK->PCHCTRL[GCLK_TCC4]		= GCLK_GEN(GEN_1M)|GCLK_CHEN;

	//HW::MCLK->APBCMASK |= APBC_TCC3;
	//HW::MCLK->APBDMASK |= APBD_TCC4;

	//PIO_MANCH->DIRSET = L1|H1|L2|H2;

	SyncTmr->CTRLA = TCC_SWRST;

	while(SyncTmr->SYNCBUSY);

	SyncTmr->CTRLA = SYNC_PRESC_DIV;
	SyncTmr->WAVE = TCC_WAVEGEN_NPWM;//|TCC_POL0;
	SyncTmr->DRVCTRL = 0;//TCC_NRE0|TCC_NRE1|TCC_NRV0|TCC_NRV1;
	SyncTmr->PER = 250;
	SyncTmr->CC[0] = 2; 
	//SyncTmr->CC[1] = 2; 

	SyncTmr->EVCTRL = 0;

	SyncTmr->CTRLA = TCC_ENABLE|SYNC_PRESC_DIV;

	RotTmr->CTRLA = TCC_SWRST;

	while(RotTmr->SYNCBUSY);

	RotTmr->CTRLA = ROT_PRESC_DIV;
	RotTmr->WAVE = TCC_WAVEGEN_MFRQ;//|TCC_POL0;
	RotTmr->DRVCTRL = 0;//TCC_NRE0|TCC_NRE1|TCC_NRV0|TCC_NRV1;
	RotTmr->CC[0] = 250;
	//RotTmr->CC[0] = 2; 
	//RotTmr->CC[1] = 2; 

	RotTmr->EVCTRL = 0;

	RotTmr->CTRLA = ROT_PRESC_DIV;//TCC_ENABLE;
	
	SyncTmr->CTRLBSET = TCC_CMD_RETRIGGER;
	RotTmr->CTRLBSET = TCC_CMD_RETRIGGER;


#elif defined(CPU_XMC48)

	PIO_SYNC->ModePin(PIN_SYNC, SYNC_PINMODE);
	PIO_ROT->ModePin(PIN_ROT, ROT_PINMODE);

	HW::CCU_Enable(SyncRotCCU_PID);

	SyncRotCCU->GCTRL = 0;

	SyncRotCCU->GIDLC = SyncRot_GIDLC;

	SyncTmr->PRS = US2SYNC(60)-1;
	SyncTmr->CRS = US2SYNC(10)-1;
	SyncTmr->PSC = SYNC_PSC; 
	SyncTmr->PSL = 1;
	SyncTmr->TCSET = CC4_TRBS;

	RotTmr->PRS = ~0;
	RotTmr->CRS = 0x7FFF;
	RotTmr->PSC = ROT_PSC; 
	RotTmr->TC = CC4_TCM;
	RotTmr->TCSET = CC4_TRBS;

	SyncRotCCU->GCSS = Sync_GCSS|Rot_GCSS;  

#endif

	SEGGER_RTT_WriteString(0, "OK\n");
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void ShaftIRQ()
{
	Pin_ShaftIRQ_Set();

#ifdef CPU_SAME53	

	HW::EIC->INTFLAG = 1<<SHAFT_EXTINT;

	SyncTmr->CTRLBSET = TCC_CMD_RETRIGGER;

#elif defined(CPU_XMC48)
	
	//SyncTmr->TCCLR = CC4_TCC;

	SyncTmr->TCCLR = CC4_TRBC;
	SyncTmr->TIMER = SyncTmr->PR >> 1;
	SyncTmr->TCSET = CC4_TRBS;

#endif

	shaftCounter++;
	curShaftCounter++;

	shaftCYCCNT = GetCYCCNT();

	u32 tm = GetMilliseconds();
	u32 dt = tm - shaftPrevTime;

	if (dt >= 1000)
	{
		shaftPrevTime = tm;
		shaftCount = shaftCounter;
		shaftTime = dt;
		shaftCounter = 0;
	};

//	rotCount = 0;

	Pin_ShaftIRQ_Clr();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateShaft()
{
	if (shaftCount != 0)
	{
		shaftRPS = shaftCount * 100000 / shaftTime;
		
		shaftCount = 0;
	}
	else if ((GetMilliseconds() - shaftPrevTime) > 1500)
	{
		shaftRPS = 0;
	};
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetShaftState()
{
#ifndef WIN32
	return PIO_SHAFT->TBCLR(PIN_SHAFT);
#else
	return 0;
#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

static void InitShaft()
{
	using namespace HW;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_GREEN "Init Shaft ... ");

	VectorTableExt[IRQ_SHAFT] = ShaftIRQ;
	CM4::NVIC->CLR_PR(IRQ_SHAFT);
	CM4::NVIC->SET_ER(IRQ_SHAFT);	

#ifdef CPU_SAME53	

	EIC->CTRLA = 0;
	while(EIC->SYNCBUSY);

	EIC->EVCTRL |= EIC_EXTINT0<<SHAFT_EXTINT;
	EIC->SetConfig(SHAFT_EXTINT, 1, EIC_SENSE_RISE);
	EIC->INTENSET = EIC_EXTINT0<<SHAFT_EXTINT;
	EIC->CTRLA = EIC_ENABLE;

	PIO_SHAFT->SetWRCONFIG(SHAFT, PORT_PMUX(0)|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_INEN);

#elif defined(CPU_XMC48)

	PIO_SHAFT->ModePin(PIN_SHAFT, I1DPD);

	// Event Request Select (ERS)
	
	ERU0->EXISEL = 2<<ERU_EXISEL_EXS3B_Pos;
	
	// Event Trigger Logic (ETL)

	ERU0->EXICON[3] = ERU_PE|ERU_RE|ERU_OCS(0)|ERU_SS_B;

	// Cross Connect Matrix

	// Output Gating Unit (OGU)

	ERU0->EXOCON[0] = ERU_GP(1);

#endif

	SEGGER_RTT_WriteString(0, "OK\n");
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

//extern dword DI;
//extern dword DO;
char pressedKey;
//extern  dword maskLED[16];

//byte _array1024[0x60000]; 

HWND  hMainWnd;

HCURSOR arrowCursor;
HCURSOR handCursor;

HBRUSH	redBrush;
HBRUSH	yelBrush;
HBRUSH	grnBrush;
HBRUSH	gryBrush;

RECT rectLed[10] = { {20, 41, 33, 54}, {20, 66, 33, 79}, {20, 91, 33, 104}, {21, 117, 22, 118}, {20, 141, 33, 154}, {218, 145, 219, 146}, {217, 116, 230, 129}, {217, 91, 230, 104}, {217, 66, 230, 79}, {217, 41, 230, 54}  }; 
HBRUSH* brushLed[10] = { &yelBrush, &yelBrush, &yelBrush, &gryBrush, &grnBrush, &gryBrush, &redBrush, &redBrush, &redBrush, &redBrush };

//int x,y,Depth;

HFONT font1;
HFONT font2;

HDC memdc;
HBITMAP membm;

//HANDLE facebitmap;

static const u32 secBufferWidth = 80;
static const u32 secBufferHeight = 12;
static const u32 fontWidth = 12;
static const u32 fontHeight = 16;


static char secBuffer[secBufferWidth*secBufferHeight*2];

static u32 pallete[16] = {	0x000000,	0x800000,	0x008000,	0x808000,	0x000080,	0x800080,	0x008080,	0xC0C0C0,
							0x808080,	0xFF0000,	0x00FF00,	0xFFFF00,	0x0000FF,	0xFF00FF,	0x00FFFF,	0xFFFFFF };

const char lpAPPNAME[] = "¿—76÷_”œ–";

int screenWidth = 0, screenHeight = 0;

LRESULT CALLBACK WindowProc (HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

static __int64		tickCounter = 0;

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

static void InitDisplay()
{
	WNDCLASS		    wcl;

	wcl.hInstance		= NULL;
	wcl.lpszClassName	= lpAPPNAME;
	wcl.lpfnWndProc		= WindowProc;
	wcl.style	    	= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;

	wcl.hIcon	    	= NULL;
	wcl.hCursor	    	= NULL;
	wcl.lpszMenuName	= NULL;

	wcl.cbClsExtra		= 0;
	wcl.cbWndExtra		= 0;
	wcl.hbrBackground	= NULL;

	RegisterClass (&wcl);

	int sx = screenWidth = GetSystemMetrics (SM_CXSCREEN);
	int sy = screenHeight = GetSystemMetrics (SM_CYSCREEN);

	hMainWnd = CreateWindowEx (0, lpAPPNAME, lpAPPNAME,	WS_DLGFRAME|WS_POPUP, 0, 0,	640, 480, NULL,	NULL, NULL, NULL);

	if(!hMainWnd) 
	{
		cputs("Error creating window\r\n");
		exit(0);
	};

	RECT rect;

	if (GetClientRect(hMainWnd, &rect))
	{
		MoveWindow(hMainWnd, 0, 0, 641 + 768 - rect.right - 2, 481 - rect.bottom - 1 + 287, true);
	};

	ShowWindow (hMainWnd, SW_SHOWNORMAL);

	font1 = CreateFont(30, 14, 0, 0, 100, false, false, false, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, PROOF_QUALITY, FIXED_PITCH|FF_DONTCARE, "Lucida Console");
	font2 = CreateFont(fontHeight, fontWidth-2, 0, 0, 100, false, false, false, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, PROOF_QUALITY, FIXED_PITCH|FF_DONTCARE, "Lucida Console");

	if (font1 == 0 || font2 == 0)
	{
		cputs("Error creating font\r\n");
		exit(0);
	};

	GetClientRect(hMainWnd, &rect);
	HDC hdc = GetDC(hMainWnd);
    memdc = CreateCompatibleDC(hdc);
	membm = CreateCompatibleBitmap(hdc, rect.right - rect.left + 1, rect.bottom - rect.top + 1 + secBufferHeight*fontHeight);
    SelectObject(memdc, membm);
	ReleaseDC(hMainWnd, hdc);


	arrowCursor = LoadCursor(0, IDC_ARROW);
	handCursor = LoadCursor(0, IDC_HAND);

	if (arrowCursor == 0 || handCursor == 0)
	{
		cputs("Error loading cursors\r\n");
		exit(0);
	};

	LOGBRUSH lb;

	lb.lbStyle = BS_SOLID;
	lb.lbColor = RGB(0xFF, 0, 0);

	redBrush = CreateBrushIndirect(&lb);

	lb.lbColor = RGB(0xFF, 0xFF, 0);

	yelBrush = CreateBrushIndirect(&lb);

	lb.lbColor = RGB(0x7F, 0x7F, 0x7F);

	gryBrush = CreateBrushIndirect(&lb);

	lb.lbColor = RGB(0, 0xFF, 0);

	grnBrush = CreateBrushIndirect(&lb);

	for (u32 i = 0; i < sizeof(secBuffer); i+=2)
	{
		secBuffer[i] = 0x20;
		secBuffer[i+1] = 0xF0;
	};
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

void UpdateDisplay()
{
	static byte curChar = 0;
	//static byte c = 0, i = 0;
	//static byte flashMask = 0;
	static const byte a[4] = { 0x80, 0x80+0x40, 0x80+20, 0x80+0x40+20 };

	MSG msg;

	static dword pt = 0;
	static TM32 tm;

	u32 t = GetTickCount();

	if ((t-pt) >= 2)
	{
		pt = t;

		while(PeekMessage (&msg, NULL, 0, 0, PM_NOREMOVE))
		{
			GetMessage (&msg, NULL, 0, 0);

			TranslateMessage (&msg);

			DispatchMessage (&msg);
		};
	};

	static char buf[80];
	static char buf1[sizeof(secBuffer)];

	if (tm.Check(20))
	{
		bool rd = true;

		if (rd)
		{
			//SelectObject(memdc, font1);
			//SetBkColor(memdc, 0x074C00);
			//SetTextColor(memdc, 0x00FF00);
			//TextOut(memdc, 443, 53, buf, 20);
			//TextOut(memdc, 443, 30*1+53, buf+20, 20);
			//TextOut(memdc, 443, 30*2+53, buf+40, 20);
			//TextOut(memdc, 443, 30*3+53, buf+60, 20);

			SelectObject(memdc, font2);

			for (u32 j = 0; j < secBufferHeight; j++)
			{
				for (u32 i = 0; i < secBufferWidth; i++)
				{
					u32 n = (j*secBufferWidth+i)*2;

					u8 t = secBuffer[n+1];

					SetBkColor(memdc, pallete[(t>>4)&0xF]);
					SetTextColor(memdc, pallete[t&0xF]);
					TextOut(memdc, i*fontWidth, j*fontHeight+287, secBuffer+n, 1);
				};
			};

			RedrawWindow(hMainWnd, 0, 0, RDW_INVALIDATE);

		}; // if (rd)

	}; // if ((t-pt) > 10)

}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

LRESULT CALLBACK WindowProc (HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int i;
	HDC hdc;
	PAINTSTRUCT ps;
	bool c;
	static int x, y;
	int x0, y0, r0;
	static char key = 0;
	static char pkey = 0;

	RECT rect;

	static bool move = false;
	static int movex, movey = 0;

//	char *buf = (char*)screenBuffer;

    switch (message)
	{
        case WM_CREATE:

            break;

	    case WM_DESTROY:

		    break;

        case WM_PAINT:

			if (GetUpdateRect(hWnd, &rect, false)) 
			{
				hdc = BeginPaint(hWnd, &ps);

				//printf("Update RECT: %li, %li, %li, %li\r\n", ps.rcPaint.left, ps.rcPaint.top, ps.rcPaint.right, ps.rcPaint.bottom);

				c = BitBlt(hdc, ps.rcPaint.left, ps.rcPaint.top, ps.rcPaint.right - ps.rcPaint.left + 1, ps.rcPaint.bottom - ps.rcPaint.top + 1, memdc, ps.rcPaint.left, ps.rcPaint.top, SRCCOPY);
	 
				EndPaint(hWnd, &ps);
			};

            break;

        case WM_CHAR:

			pressedKey = wParam;

			if (pressedKey == '`')
			{
				GetWindowRect(hWnd, &rect);

				if ((rect.bottom-rect.top) > 340)
				{
					MoveWindow(hWnd, rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top-fontHeight*secBufferHeight, true); 
				}
				else
				{
					MoveWindow(hWnd, rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top+fontHeight*secBufferHeight, true); 
				};
			};

            break;

		case WM_MOUSEMOVE:

			x = LOWORD(lParam); y = HIWORD(lParam);

			if (move)
			{
				GetWindowRect(hWnd, &rect);
				SetWindowPos(hWnd, HWND_TOP, rect.left+x-movex, rect.top+y-movey, 0, 0, SWP_NOSIZE); 
			};

			return 0;

			break;

		case WM_MOVING:

			return TRUE;

			break;

		case WM_SYSCOMMAND:

			return 0;

			break;

		case WM_LBUTTONDOWN:

			move = true;
			movex = x; movey = y;

			SetCapture(hWnd);

			return 0;

			break;

		case WM_LBUTTONUP:

			move = false;

			ReleaseCapture();

			return 0;

			break;

		case WM_MBUTTONDOWN:

			ShowWindow(hWnd, SW_MINIMIZE);

			return 0;

			break;

		case WM_ACTIVATE:

			if (HIWORD(wParam) != 0 && LOWORD(wParam) != 0)
			{
				ShowWindow(hWnd, SW_NORMAL);
			};

			break;

		case WM_CLOSE:

			//run = false;

			break;
	};
    
	return DefWindowProc(hWnd, message, wParam, lParam);
}

#endif		
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

int PutString(u32 x, u32 y, byte c, const char *str)
{
	char *dst = secBuffer+(y*secBufferWidth+x)*2;
	dword i = secBufferWidth-x;

	while (*str != 0 && i > 0)
	{
		*(dst++) = *(str++);
		*(dst++) = c;
		i -= 1;
	};

	return secBufferWidth-x-i;
}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef	WIN32

int Printf(u32 xx, u32 yy, byte c, const char *format, ... )
{
	char buf[1024];

	va_list arglist;

    va_start(arglist, format);
    vsprintf(buf, format, arglist);
    va_end(arglist);

	return PutString(xx, yy, c, buf);
}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

i32	Get_FRAM_SPI_SessionsAdr() { return FRAM_SPI_SESSIONS_ADR; }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

i32	Get_FRAM_I2C_SessionsAdr() { return FRAM_I2C_SESSIONS_ADR; }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "Hardware Init ... ");

#ifdef CPU_SAME53	
	
	using namespace HW;

//	HW::PIOA->BSET(13);

	HW::GCLK->GENCTRL[GEN_32K]		= GCLK_DIV(1)	|GCLK_SRC_OSCULP32K	|GCLK_GENEN;

	HW::GCLK->GENCTRL[GEN_1M]		= GCLK_DIV(25)	|GCLK_SRC_XOSC1		|GCLK_GENEN		|GCLK_OE;

	HW::GCLK->GENCTRL[GEN_25M]		= GCLK_DIV(1)	|GCLK_SRC_XOSC1		|GCLK_GENEN;

//	HW::GCLK->GENCTRL[GEN_500K] 	= GCLK_DIV(50)	|GCLK_SRC_XOSC1		|GCLK_GENEN;

	PIO_32kHz->SetWRCONFIG(1UL<<PIN_32kHz, PORT_PMUX_M|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_PULLEN);

	HW::GCLK->GENCTRL[GEN_EXT32K]	= GCLK_DIV(1)	|GCLK_SRC_GCLKIN	|GCLK_GENEN		;


	HW::MCLK->APBAMASK |= APBA_EIC;
	HW::GCLK->PCHCTRL[GCLK_EIC] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;

	HW::MCLK->APBBMASK |= APBB_EVSYS;

	HW::GCLK->PCHCTRL[GCLK_SERCOM_SLOW]		= GCLK_GEN(GEN_32K)|GCLK_CHEN;	// 32 kHz
	//HW::GCLK->PCHCTRL[GCLK_SERCOM5_CORE]	= GCLK_GEN(GEN_MCK)|GCLK_CHEN;	
	//HW::GCLK->PCHCTRL[GCLK_SERCOM6_CORE]	= GCLK_GEN(GEN_MCK)|GCLK_CHEN;	
	//HW::GCLK->PCHCTRL[GCLK_SERCOM7_CORE]	= GCLK_GEN(GEN_MCK)|GCLK_CHEN;	

	//HW::MCLK->APBDMASK |= APBD_SERCOM5|APBD_SERCOM6|APBD_SERCOM7;

#endif

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_GREEN "OK\n");

	Init_time();
	I2C_Init();
	SPI_Init();

#ifndef WIN32

	InitClock();

	//InitManTransmit();
	InitManRecieve();
	Init_CRC_CCITT_DMA();

	InitManTransmit();

	Init_Sync_Rot();

	InitShaft();

	EnableVCORE();

	WDT_Init();

#else

	InitDisplay();

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
#ifndef WIN32

	static byte i = 0;

	static Deb db(false, 20);

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateShaft();	);
		CALL( SPI_Update();		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
