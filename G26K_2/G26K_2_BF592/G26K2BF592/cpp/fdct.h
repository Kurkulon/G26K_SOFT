/* 
 * Fast discrete cosine transform algorithms (C)
 * 
 * Copyright (c) 2018 Project Nayuki. (MIT License)
 * https://www.nayuki.io/page/fast-discrete-cosine-transform-algorithms
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * - The above copyright notice and this permission notice shall be included in
 *   all copies or substantial portions of the Software.
 * - The Software is provided "as is", without warranty of any kind, express or
 *   implied, including but not limited to the warranties of merchantability,
 *   fitness for a particular purpose and noninfringement. In no event shall the
 *   authors or copyright holders be liable for any claim, damages or other
 *   liability, whether in an action of contract, tort or otherwise, arising from,
 *   out of or in connection with the Software or the use or other dealings in the
 *   Software.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "types.h"

#define FDCT_LOG2N 6
#define FDCT_N (1UL<<FDCT_LOG2N)

#define FDCT_INTEGER

#ifdef FDCT_INTEGER

typedef i32 FDCT_DATA;
typedef i16 FDCT_TRIG;

#define TRIGBITS 10

#define FLOAT_TRIG(x) ((int)((x)*(1<<TRIGBITS)+.5))
#define MULT_TRIG(x) ((((x)>>(TRIGBITS-1))+1)>>1)
//#define MULT_TRIG(x) ((x)>>TRIGBITS)

#else

typedef float FDCT_DATA;
typedef float FDCT_TRIG;

#define FLOAT_TRIG(x) (x)
#define MULT_TRIG(x) ((x)+0.5f)

#endif 

extern bool FastDctLee_transform(FDCT_DATA vector[], u16 log2n);
extern bool FastDctLee_inverseTransform(FDCT_DATA vector[], u16 log2n);

extern void FDCT_Init();


