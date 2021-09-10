#ifndef	POINTERCRC_H__26_05_2009__15_56
#define	POINTERCRC_H__26_05_2009__15_56

#include "types.h"
#include "CRC16.h"

struct PointerCRC
{
	union
	{
		__packed void	*v;
		__packed byte 	*b;
		__packed word 	*w;
		__packed dword 	*d;
		__packed float 	*f;
	};

	union { word w; byte b[2]; } CRC;

	PointerCRC() {v = 0; CRC.w = 0xFFFF; }
	PointerCRC(void *p) { v = p; CRC.w = 0xFFFF; } 
	PointerCRC(byte *p) { b = p; CRC.w = 0xFFFF; } 
	PointerCRC(word *p) { w = p; CRC.w = 0xFFFF; } 
	PointerCRC(dword *p) { d = p; CRC.w = 0xFFFF; } 
	PointerCRC(float *p) { f = p; CRC.w = 0xFFFF; } 

	void operator=(void *p) { v = p; } 
	void operator=(byte *p) { b = p; } 
	void operator=(word *p) { w = p; } 
	void operator=(dword *p) { d = p; } 
	void operator=(float *p) { f = p; } 

	void WriteB(byte var) {*(b++) = var; CRC.w = tableCRC[CRC.b[0] ^ var] ^ CRC.b[1]; }
	void WriteW(U16u var) {*(w++) = var; CRC.w = tableCRC[CRC.b[0] ^ var.b[0]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ var.b[1]] ^ CRC.b[1];}
	void WriteD(U32u var){*(d++) = var; 
	
	CRC.w = tableCRC[CRC.b[0] ^ var.b[0]] ^ CRC.b[1]; 
	CRC.w = tableCRC[CRC.b[0] ^ var.b[1]] ^ CRC.b[1]; 
	CRC.w = tableCRC[CRC.b[0] ^ var.b[2]] ^ CRC.b[1]; 
	CRC.w = tableCRC[CRC.b[0] ^ var.b[3]] ^ CRC.b[1]; }
	
	void WriteF(F32u var){*(d++) = var.d; 	CRC.w = tableCRC[CRC.b[0] ^ var.b[0]] ^ CRC.b[1];CRC.w = tableCRC[CRC.b[0] ^ var.b[1]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ var.b[2]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ var.b[3]] ^ CRC.b[1]; }

	byte ReadB() { CRC.w = tableCRC[CRC.b[0] ^ *b] ^ CRC.b[1]; return *(b++); }
	word ReadW() { CRC.w = tableCRC[CRC.b[0] ^ b[0]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ b[1]] ^ CRC.b[1]; return *(w++); 	}
	dword ReadD(){ CRC.w = tableCRC[CRC.b[0] ^ b[0]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ b[1]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ b[2]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ b[3]] ^ CRC.b[1]; return *(d++); }
	float ReadF(){ CRC.w = tableCRC[CRC.b[0] ^ b[0]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ b[1]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ b[2]] ^ CRC.b[1]; CRC.w = tableCRC[CRC.b[0] ^ b[3]] ^ CRC.b[1]; return *(f++); }

	void WriteArrayB(ConstDataPointer p, word count);
	void WriteArrayW(ConstDataPointer p, word count);
	void WriteArrayD(ConstDataPointer p, word count);
	void WriteArrayF(ConstDataPointer p, word count);

	void ReadArrayB(DataPointer p, word count);
	void ReadArrayW(DataPointer p, word count);
	void ReadArrayD(DataPointer p, word count);
	void ReadArrayF(DataPointer p, word count);
};

#endif	// POINTERCRC_H__26_05_2009__15_56
