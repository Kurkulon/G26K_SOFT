#ifndef WAVELET_H__INCLUDED
#define WAVELET_H__INCLUDED

#include "types.h"

#define FWT_LOG2N 7
#define FWT_N (1UL<<FWT_LOG2N)

#define FWT_DAUB8

#define FWT_INTEGER

#ifdef FWT_INTEGER

typedef i32 FWT_DATA;
typedef i16 FWT_FLTR;

#define FWT_FIXBITS 14

#define FLOAT_FWT(x)	((int)((x)*(1<<FWT_FIXBITS)+.5))
#define MULT_FWT(x)	(((x)+(1<<(FWT_FIXBITS-1)))>>FWT_FIXBITS)

#else

typedef float FWT_DATA;
typedef float FWT_FLTR;

#define FLOAT_FWT(x)	(x)
#define MULT_FWT(x)		(x)

#endif 

extern void Wavelet(FWT_DATA *f, u16 log2n);
extern void InvWavelet(FWT_DATA *f, u16 log2n);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif

