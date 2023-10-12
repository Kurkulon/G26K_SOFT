/********************************************************************
 *                                                                  *
 * THIS FILE IS PART OF THE OggVorbis SOFTWARE CODEC SOURCE CODE.   *
 * USE, DISTRIBUTION AND REPRODUCTION OF THIS LIBRARY SOURCE IS     *
 * GOVERNED BY A BSD-STYLE SOURCE LICENSE INCLUDED WITH THIS SOURCE *
 * IN 'COPYING'. PLEASE READ THESE TERMS BEFORE DISTRIBUTING.       *
 *                                                                  *
 * THE OggVorbis SOURCE CODE IS (C) COPYRIGHT 1994-2002             *
 * by the XIPHOPHORUS Company http://www.xiph.org/                  *
 *                                                                  *
 ********************************************************************

 function: modified discrete cosine transform prototypes
 last mod: $Id: mdct.h 7187 2004-07-20 07:24:27Z xiphmont $

 ********************************************************************/

#ifndef _OGG_mdct_H_
#define _OGG_mdct_H_

#include "types.h"

#define MDCT_INTEGERIZED //  <- be warned there could be some hurt left here
#ifdef MDCT_INTEGERIZED

#define DATA_TYPE			i16
#define DATA_TYPE_IN		i16
#define DATA_TYPE_T			i16
#define DATA_TYPE_BITREV	u16
#define REG_TYPE  register i32
#define TRIGBITS 14
#define cPI3_8 6270
#define cPI2_8 11585
#define cPI1_8 15137

#define FLOAT_CONV(x) ((int)((x)*(1<<TRIGBITS)+.5))
#define MULT_NORM(x) ((x)>>TRIGBITS)
#define HALVE(x) ((x)>>1)

#else

#define DATA_TYPE float
#define REG_TYPE  float
#define cPI3_8 .38268343236508977175F
#define cPI2_8 .70710678118654752441F
#define cPI1_8 .92387953251128675613F

#define FLOAT_CONV(x) (x)
#define MULT_NORM(x) (x)
#define HALVE(x) ((x)*.5f)

#endif


struct MDCT_LookUp
{
  i32 n;
  i32 log2n;
  
  DATA_TYPE_T		*trig;
  DATA_TYPE_BITREV	*bitrev;

  DATA_TYPE scale;
};

extern void mdct_init(MDCT_LookUp *lookup, int log2n, int n, DATA_TYPE_BITREV *bitrev, DATA_TYPE_T *T);
//extern void mdct_clear(MDCT_LookUp *l);
extern void mdct_forward(MDCT_LookUp *init, DATA_TYPE_IN *in, DATA_TYPE_IN *out, DATA_TYPE *w);
extern void mdct_backward(MDCT_LookUp *init, DATA_TYPE *in, DATA_TYPE *out);

#endif












