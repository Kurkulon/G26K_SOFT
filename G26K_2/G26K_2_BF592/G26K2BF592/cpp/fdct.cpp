/* 
 * Fast discrete cosine transform algorithms (C)
 * 
 * Copyright (c) 2021 Project Nayuki. (MIT License)
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

//#pragma optimize_for_speed

#include <math.h>
//#include <stdint.h>
//#include <stdlib.h>
#include "fdct.h"

#ifndef M_PI
#define M_PI 3.141592653
#endif 

static void forwardTransform(FDCT_DATA vector[restrict], FDCT_DATA temp[restrict], u16 len);
static void inverseTransform(FDCT_DATA vector[restrict], FDCT_DATA temp[restrict], u16 len);
static void forwardTransform_v2(FDCT_DATA vector[restrict], FDCT_DATA temp[restrict], u16 len);
static void forwardTransform_v3(FDCT_DATA vector[restrict], FDCT_DATA temp[restrict], u16 len);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static FDCT_TRIG fdct_trig[FDCT_N] = {0};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FDCT_Init()
{
	u32 len = ArraySize(fdct_trig)/2;

	for (u32 len = 1; len < ArraySize(fdct_trig); len *= 2)
	{
		FDCT_TRIG *trig = fdct_trig + len;

		for (u32 i = 0; i < len; i++)
		{
			trig[i] = FLOAT_TRIG(0.5f / cos((i + 0.5) * M_PI / (len*2)));
		};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// DCT type II, unscaled. Algorithm by Byeong Gi Lee, 1984.
// See: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.118.3056&rep=rep1&type=pdf#page=34

bool FastDctLee_transform(FDCT_DATA vector[], u16 log2n)
{
	if (log2n < 3 || log2n > FDCT_LOG2N) return false;  // Length is not power of 2

	FDCT_DATA temp[FDCT_N];

	u16 len = 1UL<<log2n;

	forwardTransform_v3(vector, temp, len);

	#ifdef FDCT_INTEGER
		//for (u16 i = 0; i < len; i++) vector[i] >>= log2n-3;
	#else
		for (u16 i = 0; i < len; i++) vector[i] /= len/8;
	#endif

	vector[0] /= 2;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void forwardTransform2(FDCT_DATA vector[restrict])
{
	FDCT_DATA x = vector[0];
	FDCT_DATA y = vector[1];

	vector[0] = x + y;
	vector[1] = MULT_TRIG((x - y) * fdct_trig[1]); 
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline void forwardTransform4(FDCT_DATA vector[restrict] )
{
	FDCT_DATA temp0 =				vector[0] + vector[3];
	FDCT_DATA temp2 =	MULT_TRIG((	vector[0] - vector[3]	) * fdct_trig[2]);

	FDCT_DATA temp1 =				vector[1] + vector[2];
	FDCT_DATA temp3 =	MULT_TRIG((	vector[1] - vector[2]	) * fdct_trig[3]);

	vector[0] = temp0 + temp1;
	vector[2] = MULT_TRIG((temp0 - temp1) * fdct_trig[1]); 
	vector[3] = MULT_TRIG((temp2 - temp3) * fdct_trig[1]); 
	vector[1] = temp2 + temp3 + vector[3]; 
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline void forwardTransform8(FDCT_DATA vector[restrict])
{
	FDCT_DATA temp0 =				vector[0] + vector[7];
	FDCT_DATA temp4 = MULT_TRIG((	vector[0] - vector[7]) * fdct_trig[4]); // / (cos((i + 0.5) * M_PI / len) * 2);

	FDCT_DATA temp1 =				vector[1] + vector[6];
	FDCT_DATA temp5 = MULT_TRIG((	vector[1] - vector[6]) * fdct_trig[5]); // / (cos((i + 0.5) * M_PI / len) * 2);

	FDCT_DATA temp2 =				vector[2] + vector[5];
	FDCT_DATA temp6 = MULT_TRIG((	vector[2] - vector[5]) * fdct_trig[6]); // / (cos((i + 0.5) * M_PI / len) * 2);

	FDCT_DATA temp3 =				vector[3] + vector[4];
	FDCT_DATA temp7 = MULT_TRIG((	vector[3] - vector[4]) * fdct_trig[7]); // / (cos((i + 0.5) * M_PI / len) * 2);

	FDCT_DATA t0 =				temp0 + temp3;
	FDCT_DATA x = MULT_TRIG((	temp0 - temp3) * fdct_trig[2]);

	FDCT_DATA t1 =				temp1 + temp2;
	FDCT_DATA y = MULT_TRIG((	temp1 - temp2) * fdct_trig[3]);

	vector[0] =				t0 + t1;								//temp0
	vector[4] = MULT_TRIG((	t0 - t1) * fdct_trig[1]);	// temp2

	vector[6] = MULT_TRIG((	x - y) * fdct_trig[1]); 
	vector[2] =				x + y + vector[6];	// temp1

	t0 =			temp4 + temp7;
	x = MULT_TRIG((	temp4 - temp7) * fdct_trig[2]);

	t1 =			temp5 + temp6;
	y = MULT_TRIG((	temp5 - temp6) * fdct_trig[3]);

	temp4 =				t0 + t1;
	temp6 = MULT_TRIG((	t0 - t1) * fdct_trig[1]); 

	t0 = MULT_TRIG((	x - y) * fdct_trig[1]); 
	temp5 =				x + y + t0; 

	vector[1] = temp4 + temp5;
	vector[3] = temp5 + temp6;
	vector[5] = temp6 + t0;
	vector[7] = t0;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void forwardTransform(FDCT_DATA vector[restrict], FDCT_DATA temp[restrict], u16 len)
{
	//if (len == 1) return;

	u16 halfLen = len / 2;

	FDCT_TRIG *trig = fdct_trig + halfLen;

	for (u16 i = 0; i < halfLen; i++)
	{
		FDCT_DATA x = vector[i];
		FDCT_DATA y = vector[len - 1 - i];

		temp[i] = x + y;
		temp[i + halfLen] = MULT_TRIG((x - y) * trig[i]); // / (cos((i + 0.5) * M_PI / len) * 2);
	};

	if (halfLen != 8)
	{
		forwardTransform(temp,			vector, halfLen);
		forwardTransform(temp+halfLen,	vector, halfLen);
	}
	else
	{
		forwardTransform8(temp			);
		forwardTransform8(temp+halfLen	);
	}

	for (u16 i = 0; i < halfLen - 1; i++)
	{
		vector[i * 2 + 0] = temp[i];
		vector[i * 2 + 1] = temp[i + halfLen] + temp[i + halfLen + 1];
	};

	vector[len - 2] = temp[halfLen - 1];
	vector[len - 1] = temp[len - 1];
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void forwardTransform_v2(FDCT_DATA vector[restrict], FDCT_DATA temp[restrict], u16 len)
{
	//u16 halfLen = len / 2;

	FDCT_DATA *vec = temp;
	FDCT_DATA *tmp = vector;

    for (u16 halfLen = len / 2; halfLen > 0; halfLen /= 2)
    {
		FDCT_TRIG *trig = fdct_trig + halfLen;

		FDCT_DATA *t = tmp; tmp = vec; vec = t;

		for (u16 off = 0; off < len; off += halfLen*2)
        {
            for (u16 i = 0; i < halfLen; i++)
            {
                FDCT_DATA x = vec[off + i];
                FDCT_DATA y = vec[off + halfLen*2 - 1 - i];
                tmp[off + i] = x + y;
                tmp[off + i + halfLen] = MULT_TRIG((x - y) * trig[i]);
            };
        };
        
    };

    for (u16 halfLen = 1; halfLen < len; halfLen *= 2)
    {
        for (u16 off = 0; off < len; off += halfLen*2)
        {
            for (u16 i = 0; i < halfLen - 1; i++)
            {
                vec[off + i * 2 + 0] = tmp[off + i];
                vec[off + i * 2 + 1] = tmp[off + i + halfLen] + tmp[off + i + halfLen + 1];
            };
        
            vec[off + halfLen*2 - 2] = tmp[off + halfLen - 1];
            vec[off + halfLen*2 - 1] = tmp[off + halfLen*2 - 1];
        };

		FDCT_DATA *t = tmp; tmp = vec; vec = t;
    };
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void forwardTransform_v3(FDCT_DATA vector[restrict], FDCT_DATA temp[restrict], u16 len)
{
	//u16 halfLen = len / 2;

	FDCT_DATA *vec = temp;
	FDCT_DATA *tmp = vector;

    for (u16 halfLen = len / 2; halfLen > 4; halfLen /= 2)
    {
		FDCT_TRIG *trig = fdct_trig + halfLen;

		FDCT_DATA *t = tmp; tmp = vec; vec = t;

		for (u16 off = 0; off < len; off += halfLen*2)
        {
            for (u16 i = 0; i < halfLen; i++)
            {
                FDCT_DATA x = vec[off + i];
                FDCT_DATA y = vec[off + halfLen*2 - 1 - i];
                tmp[off + i] = x + y;
                tmp[off + i + halfLen] = MULT_TRIG((x - y) * trig[i]);
            };
        };
        
    };

	for (u16 i = 0; i < len; i += 8) forwardTransform8(tmp+i);

    for (u16 halfLen = 8; halfLen < len; halfLen *= 2)
    {
        for (u16 off = 0; off < len; off += halfLen*2)
        {
            for (u16 i = 0; i < halfLen - 1; i++)
            {
                vec[off + i * 2 + 0] = tmp[off + i];
                vec[off + i * 2 + 1] = tmp[off + i + halfLen] + tmp[off + i + halfLen + 1];
            };
        
            vec[off + halfLen*2 - 2] = tmp[off + halfLen - 1];
            vec[off + halfLen*2 - 1] = tmp[off + halfLen*2 - 1];
        };

		FDCT_DATA *t = tmp; tmp = vec; vec = t;
    };
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// DCT type III, unscaled. Algorithm by Byeong Gi Lee, 1984.
// See: https://www.nayuki.io/res/fast-discrete-cosine-transform-algorithms/lee-new-algo-discrete-cosine-transform.pdf

bool FastDctLee_inverseTransform(FDCT_DATA vector[], u16 log2n)
{
	if (log2n < 3 || log2n > FDCT_LOG2N) return false;  // Length is not power of 2

	FDCT_DATA temp[FDCT_N];

	vector[0] /= 2;

	u16 len = 1UL<<log2n;

	inverseTransform(vector, temp, len);

	#ifdef FDCT_INTEGER
		for (u16 i = 0; i < len; i++) vector[i] >>= 2;
	#else
		for (u16 i = 0; i < len; i++) vector[i] /= 4;
	#endif

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void inverseTransform2(FDCT_DATA vector[restrict])
{
	FDCT_DATA x = vector[0];
	FDCT_DATA y = MULT_TRIG(vector[1] * fdct_trig[1]);

	vector[0] = x + y;
	vector[1] = x - y;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline void inverseTransform4(FDCT_DATA vector[restrict])
{
	//FDCT_DATA temp3 = vector[1] + vector[3];

	FDCT_DATA x = vector[0];
	FDCT_DATA y = MULT_TRIG(vector[2] * fdct_trig[1]);

	FDCT_DATA temp0 = x + y;
	FDCT_DATA temp1 = x - y;

	y = MULT_TRIG((vector[1] + vector[3]) * fdct_trig[1]);

	FDCT_DATA temp2 = vector[1] + y;
	FDCT_DATA temp3 = vector[1] - y;

	y = MULT_TRIG(temp2 * fdct_trig[2]); // / (cos((i + 0.5) * M_PI / len) * 2);

	vector[0] = temp0 + y;
	vector[3] = temp0 - y;

	y = MULT_TRIG(temp3 * fdct_trig[3]); // / (cos((i + 0.5) * M_PI / len) * 2);

	vector[1] = temp1 + y;
	vector[2] = temp1 - y;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void inverseTransform(FDCT_DATA vector[restrict], FDCT_DATA temp[restrict], u16 len)
{
	if (len == 1) return;

	u16 halfLen = len / 2;

	temp[0] = vector[0];
	temp[halfLen] = vector[1];
	
	for (u16 i = 1; i < halfLen; i++)
	{
		temp[i] = vector[i * 2];
		temp[i + halfLen] = vector[i * 2 - 1] + vector[i * 2 + 1];
	};

	if (halfLen != 4)
	{
		inverseTransform(temp,			vector, halfLen);
		inverseTransform(temp+halfLen,	vector, halfLen);
	}
	else
	{
		inverseTransform4(temp			);
		inverseTransform4(temp+halfLen	);
	};

	FDCT_TRIG *trig = fdct_trig + halfLen;

	for (u16 i = 0; i < halfLen; i++)
	{
		FDCT_DATA x = temp[i];
		FDCT_DATA y = MULT_TRIG(temp[i + halfLen] * trig[i]); // / (cos((i + 0.5) * M_PI / len) * 2);

		vector[i]			= x + y;
		vector[len - 1 - i]	= x - y;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
