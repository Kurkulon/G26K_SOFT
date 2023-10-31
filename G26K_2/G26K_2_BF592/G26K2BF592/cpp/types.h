#ifndef TYPES_H__15_05_2009__14_37
#define TYPES_H__15_05_2009__14_37

#include <ccblkfn.h>

//#ifdef _WIN32
//#ifndef WIN32
//#define WIN32
//#endif
//#endif

#ifdef WIN32

#define __packed /**/
#define __softfp /**/
#define __irq /**/

inline void __disable_irq() {}
inline void __enable_irq() {}

#endif 

#ifdef __DEBUG
inline void __breakpoint() { asm("EMUEXCPT;"); }
#else
inline void __breakpoint() { asm("EMUEXCPT;"); }
#endif

typedef unsigned char byte, u8;
typedef unsigned short int word, u16;
typedef unsigned long int dword, u32;
typedef signed char i8;
typedef signed short int i16;
typedef signed long int i32;
typedef signed long long int i64;
typedef unsigned long long int u64;

#define ArraySize(x) (sizeof(x)/sizeof(x[0]))

#ifndef NAN
static const dword __NAN_dword = 0xFFFFFFFF;
#define NAN (*((const float*)(&__NAN_dword)))
#endif

inline float ABS(float v) { *((u32*)&v) &= 0x7FFFFFFF; return v; }

inline bool fIsValid(float v) { return (((u16*)&v)[2] & 0x7F80) != 0x7F80; }
inline bool dIsValid(float v) { return (((u32*)&v)[2] & 0x7FF0) != 0x7FF0; }

#define GD(adr, t, i) (*(((t*)adr)+i))
#define GB(adr,i) (*(((byte*)adr)+i))


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
union DataCRC
{
	word	w;
	byte	b[2];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union U16u 
{
	word w; byte b[2]; 

	U16u(word v) {w = v;}
	operator word() {return w;}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union U32u 
{ 
	dword d; word w[2]; byte b[4]; 

	U32u(dword v) {d = v;}
	U32u() {d = 0;}
	operator dword() {return d;}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union F32u
{ 
	float f; dword d; word w[2]; byte b[4]; 

	F32u (float v) {f = v;}
	operator float() {return f;}
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union DataPointer
{
	void	*v;
	byte	*b;
	word 	*w;
	dword	*d;
	float	*f;

	DataPointer(void *p) { v = p; } 

	void operator=(void *p) { v = p; } 

	void WW(word a) { misaligned_store16(v, a); }
	word RW() { return misaligned_load16(v); }
} ;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union ConstDataPointer
{
	const void		*v;
	const byte		*b;
	const word		*w;
	const dword	*d;
	const float	*f;

	ConstDataPointer(const void *p) { v = p; } 

	void operator=(const void *p) { v = p; } 
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#endif // TYPES_H__15_05_2009__14_37
