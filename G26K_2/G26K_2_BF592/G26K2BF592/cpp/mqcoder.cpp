#include "types.h"


/*
** Simple predictive arithmetic audio coding, (c) 2004 Thomas Richter,
** THOR Software.
** $Id: arthdeco.c,v 1.4 2004-10-10 20:06:33 thor Exp $
**
*/

//#include <stdio.h>
#include <string.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
** context of the arithmetic coder: index into the probability table,
** most probable symbol
*/
struct MQContext
{
	byte index : 7;
	byte mp : 1;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
** Arithmetic coder structure.
** The MQ coder is patented software, (c) IBM corp.
*/
struct MQCoder
{
	i32		a;       /* the inverval */
	i32		c;       /* computation register */
	i32		ct;      /* bit counter */
	i32		flag;
	byte	b;       /* b output register */

	byte	*src;
	byte	*dst;
	u32		srclen;
	u32		dstlen;

	void putchar(byte v) {*(dst++) = v; dstlen++; }
	byte getchar() { if ( srclen > 0) { srclen--; return *(src++); } else { return ~0; } }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
** MQ coder lookup tables
*/
const u16 Qe_Value[] = 
{
	0x5601,0x3401,0x1801,0x0ac1,0x0521,0x0221,0x5601,0x5401,0x4801,0x3801,
	0x3001,0x2401,0x1c01,0x1601,0x5601,0x54ff,0x5401,0x527d,0x5101,0x4c5f,
	0x4801,0x3f80,0x3801,0x35f7,0x3401,0x31f6,0x3001,0x2801,0x2401,0x2201,
	0x1c01,0x1801,0x1601,0x1401,0x1201,0x1101,0x0ac1,0x09c1,0x08a1,0x0521,
	0x0441,0x02a1,0x0221,0x0141,0x0111,0x0085,0x0049,0x0025,0x0015,0x0009,
	0x0005,0x0001,0x5601
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
** MSB/LSB switch flags
*/
const byte Qe_Switch[] = {
	1,0,0,0,0,0,1,0,0,0,
	0,0,0,0,1,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
** Next state on MPS coding
*/
const byte Qe_NMPS[] = {
	1,2,3,4,5,44,7,8,9,10,
	11,12,13,35,15,16,17,18,19,20,
	21,22,23,24,25,26,27,28,29,30,
	31,32,33,34,35,36,37,38,39,40,
	41,42,43,44,45,45,47,48,49,50,
	51,51,52
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
** Next state on LPS coding
*/
const byte Qe_NLPS[] = {
	1,6,9,12,35,39,6,14,14,14,
	20,22,25,27,14,14,14,15,16,17,
	18,19,20,21,22,23,24,25,26,27,
	28,29,30,31,32,33,34,35,36,37,
	38,39,40,41,42,43,44,45,46,47,
	48,49,52
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
** Open the MQ coder for writing
*/
static inline void OpenForWrite(struct MQCoder *self, byte *dst)
{
	self->a			= 0x8000;
	self->c			= 0;
	self->ct		= 12;
	self->b			= 0;
	self->flag		= 0;
	self->dstlen	= 0;
	self->dst		= dst;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void OpenForRead(struct MQCoder *self, byte *src, u32 len)
{
	self->src		= src;
	self->srclen	= len;
	self->b			= self->getchar();
	self->c			= self->b << 16;
	byte t			= self->getchar();
	self->ct		= 8;

	if (self->b == 0xff)
	{
		if (t < 0x90)
		{
			self->c += t << 8;
			self->ct--;
		};
	};

	self->c  += t << 8;
	self->b   = t;
	self->c <<= 7;
	self->ct -= 7;
	self->a   = 0x8000;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static i32 GetBit(struct MQCoder *self,struct MQContext *ctx)
{
	u32 q = Qe_Value[ctx->index];
	
	byte d;

	self->a -= q;

	if ((self->c >> 16) >= q)
	{
		/* MPS case */
		self->c -= q << 16;

		if (self->a & 0x8000)
		{
			/* short MPS case, return immediately. */
			return ctx->mp;
		};

		/* MPS exchange here */
		d = (self->a < q);  /* d == 1 on LPS */
	}
	else
	{
		/* LPS exchange here */
		d = (self->a >= q); /* d == 1 on LPS */
		self->a = q;
	};
	
	if (d)
	{
		/* LPS decoding. Check for MPS/LPS exchange, adjust index */
		d ^= ctx->mp;
		
		if (Qe_Switch[ctx->index]) ctx->mp = d;

		ctx->index = Qe_NLPS[ctx->index];
	}
	else
	{
		/* MPS decoding */
		d = ctx->mp;
		ctx->index = Qe_NMPS[ctx->index];
	}
	
	
	do // Now renormalize 
	{
		if (self->ct == 0)
		{
			byte t = self->getchar();

			self->ct = 8;

			if (self->b == 0XFF)
			{
				if (t < 0x90)
				{
					self->c += t << 8;
					self->ct--;
				};
			};

			self->c += t << 8;
			self->b = t;
		};

		self->a <<= 1;
		self->c <<= 1;
		self->ct--;

	} while ((self->a & 0x8000) == 0);

	return d;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
** Put out a bit thru the MQ coder.
*/
static void PutBit(struct MQCoder *self,struct MQContext *ctx,i32 bitf)
{
	u32 q = Qe_Value[ctx->index];

	byte bit = (bitf) ? 1 : 0;

	self->a -= q;
	
	// Check for MPS/LPS coding

	if (bit == ctx->mp)
	{
		// MPS coding

		if (self->a & 0x8000)
		{
			/* Short MPS case, no renormalization */
			self->c += q;
			return;
		}
		else
		{
			/* context change */
			
			if (self->a < q)
			{
				/* MPS/LPS exchange case */
				self->a  = q;
			}
			else
			{
				self->c += q;
			};

			ctx->index = Qe_NMPS[ctx->index];
		};
	}
	else
	{
		/* LPS coding here */
	
		if (self->a < q)
		{
			self->c += q;
		}
		else
		{
			self->a  = q;
		};

		ctx->mp   ^= Qe_Switch[ctx->index];
		ctx->index = Qe_NLPS[ctx->index];
	};

	 
	// Renormalize now. 

	do
	{
		self->a <<= 1;
		self->c <<= 1;

		if (--self->ct == 0)
		{
			if (self->b < 0xff)
			{
				if (self->c & 0x8000000)
				{
					/* Overflow into the b register, remove
					** carry bit and go on */
					self->b++;
					self->c &= 0x7ffffff;
				};
			};

			if (self->b == 0xff)
			{
				/* We either have an 0xff here, or generated one due to carry. 
				** in either case, must have buffered something or the overflow
				** could not have happened.
				*/
				self->putchar(0xff);
				self->b  = (byte)(self->c >> 20);
				self->c &= 0xfffff;
				self->ct = 7;
			}
			else
			{
				if (self->flag) self->putchar((byte)self->b);
				self->b  = self->c >> 19;
				self->c &= 0x7ffff;
				self->ct = 8;
			};

			self->flag = 1;
		};

	} while ((self->a & 0x8000) == 0);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Flush out the remaining bits

static void Flush(struct MQCoder *self)
{
	i32 k;	// Number of bits to push out is then in k.

	self->c <<= self->ct;

	for(k = 12 - self->ct;k > 0;k -= self->ct,self->c <<= self->ct)
	{
		if (self->b < 0xff)
		{
			if (self->c & 0x8000000)
			{
				self->b++;
				self->c &= 0x7ffffff;
			};
		};

		if (self->b == 0xff)
		{
			self->putchar(0xff);
			self->b  = self->c >> 20;
			self->c &= 0xfffff;
			self->ct = 7;
		}
		else
		{
			if (self->flag) self->putchar((byte)self->b);

			self->b  = self->c >> 19;
			self->c &= 0x7ffff;
			self->ct = 8;
		};

		self->flag = 1;
	};

	if (self->b < 0xff)
	{
		if (self->c & 0x8000000)
		{
			self->b++;
		};
	};

	if (self->b != 0xff && self->flag) self->putchar((byte)self->b);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 MQcompress(byte* src, u32 srclen, byte* dst)
{
	struct MQContext ctxt[256*8];
	struct MQContext *cx;
	struct MQCoder   coder;
	i32 code,i;
	i32 lb   = 0;

	/* reset all contexts */
	memset(&ctxt,0,sizeof(ctxt));
	OpenForWrite(&coder, dst);

	while (srclen > 0)
	{
		code  = *src++;
		cx    = ctxt + (lb << 3); 
		//code ^= (code >> 1);
		
		for(i = 0;i < 8;i++,cx++)
		{
			PutBit(&coder, cx/*+((lb>>(i&4))&15)*/,code & (1 << i));
		};

		lb = code;

		srclen--;
	};

	Flush(&coder);

	return coder.dstlen;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

i32 MQdecompress(byte* src, u32 srclen, byte* dst, u32 dstlen)
{  
	//byte grey[256];
	struct MQContext ctxt[256*8];
	struct MQContext *cx;
	struct MQCoder   coder;
	i32 code,i,lb = 0;

	/* reset all contexts */
	memset(&ctxt,0,sizeof(ctxt));
	/*
	** Initialize the inverse grey coding table.
	*/
	//for(i = 0;i < 256;i++) grey[i ^ (i >> 1)] = (byte)i;

	OpenForRead(&coder, src, srclen);

	while(dstlen > 0)
	{ 
		cx    = ctxt + (lb << 3);

		for(i = 0, code = 0; i < 8; i++,cx++)
		{
			if (GetBit(&coder, cx/*+((lb>>(i&4))&15)*/)) code |= (1 << i);
		};

		//code  = grey[code];
		lb    = code;
		*dst++ = (byte)code; dstlen--;

		//srclen--;
	};

	return dstlen; 
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 MQcompressFast(byte* src, u32 srclen, byte* dst)
{
	MQContext ctxt[256*8];
	MQContext *cx;
	MQCoder   coder;

	i32 code,i;
	i32 lb   = 0;

	/* reset all contexts */
	memset(&ctxt,0,sizeof(ctxt));
		
	//OpenForWrite(&coder, dst);

	coder.a			= 0x8000;
	coder.c			= 0;
	coder.ct		= 12;
	coder.b			= 0;
	coder.flag		= 0;
	coder.dstlen	= 0;
	coder.dst		= dst;

	while (srclen > 0)
	{
		code  = *src++;
		cx    = ctxt;// + (lb << 3); 
		//code ^= (code >> 1);
		lb = code;
		
		for(i = 0;i < 8;i++/*,cx++*/)
		{
			//PutBit(&coder, cx/*+((lb>>(i&4))&15)*/, code&1);

			i32 q = Qe_Value[cx->index];

			byte bit = code&1; code >>= 1;

			coder.a -= q;
			
			// Check for MPS/LPS coding

			if (bit == cx->mp)
			{
				// MPS coding

				if (coder.a & 0x8000)
				{
					/* Short MPS case, no renormalization */
					coder.c += q;
					continue;// goto __end;
				}
				else
				{
					/* context change */
					
					if (coder.a < q)
					{
						/* MPS/LPS exchange case */
						coder.a  = q;
					}
					else
					{
						coder.c += q;
					};

					cx->index = Qe_NMPS[cx->index];
				};
			}
			else
			{
				/* LPS coding here */
			
				if (coder.a < q)
				{
					coder.c += q;
				}
				else
				{
					coder.a  = q;
				};

				cx->mp   ^= Qe_Switch[cx->index];
				cx->index = Qe_NLPS[cx->index];
			};
			 
			// Renormalize now. 

			do
			{
				coder.a <<= 1;
				coder.c <<= 1;

				if (--coder.ct == 0)
				{
					if (coder.b < 0xff)
					{
						if (coder.c & 0x8000000)
						{
							/* Overflow into the b register, remove
							** carry bit and go on */
							coder.b++;
							coder.c &= 0x7ffffff;
						};
					};

					if (coder.b == 0xff)
					{
						/* We either have an 0xff here, or generated one due to carry. 
						** in either case, must have buffered something or the overflow
						** could not have happened.
						*/
						coder.putchar(0xff);
						coder.b  = (byte)(coder.c >> 20);
						coder.c &= 0xfffff;
						coder.ct = 7;
					}
					else
					{
						if (coder.flag) coder.putchar((byte)coder.b);
						coder.b  = coder.c >> 19;
						coder.c &= 0x7ffff;
						coder.ct = 8;
					};

					coder.flag = 1;
				};

			} while ((coder.a & 0x8000) == 0);
//__end:
			
		};

		srclen--;
	};

	Flush(&coder);

	return coder.dstlen;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
