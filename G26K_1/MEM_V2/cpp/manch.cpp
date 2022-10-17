#include "types.h"
#include "core.h"
//#include "COM_DEF.h"
//#include "CRC16_8005.h"

//#include "hardware.h"

#include "SEGGER_RTT.h"
#include "hw_conf.h"
#include "hardware.h"

#ifdef CPU_SAME53	
#define MAN_TRANSMIT_V1
#elif defined(CPU_XMC48)
#define MAN_TRANSMIT_V1
#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <manch_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
