#ifndef HW_NAND_H__03_05_2022__19_36
#define HW_NAND_H__03_05_2022__19_36

#include "types.h"
//#include "core.h"

#define NAND_CHIP_BITS		3
#define NAND_MAX_CHIP		(1<<NAND_CHIP_BITS)
#define NAND_CHIP_MASK		(NAND_MAX_CHIP-1)

extern void	HW_NAND_Init();

#ifdef WIN32

extern void NAND_FlushBlockBuffers();
extern void NAND_ReqFlushBlockBuffers();

#endif


#endif // HW_NAND_H__03_05_2022__19_36
