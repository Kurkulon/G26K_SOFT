#ifndef ALLOCMEM_H__04_08_2009__13_09
#define ALLOCMEM_H__04_08_2009__13_09

#include "types.h"

void* AllocMem(dword size);
void* AllocMemNV(dword size);

#ifdef WIN32
void SaveMemNV();
void LoadMemNV();
#endif


#endif // ALLOCMEM_H__04_08_2009__13_09
