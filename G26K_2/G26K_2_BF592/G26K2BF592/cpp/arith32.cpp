//Entropy Coding Source code
//By Sachin Garg, 2006
//
//Includes range coder based upon the carry-less implementation 
//by Dmitry Subbotin, and arithmetic coder based upon Mark Nelson's
//DDJ code.
// 
//Modified to use 64-bit variables for improved performance.
//32-bit reference implementations also included.
//
//For details:
//http://www.sachingarg.com/compression/entropy_coding/64bit


//#include "entropy/arith32.h"
#include "types.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Constants
#define True 1
#define False 0
#define Yes 1
#define No 0

#ifndef NULL
#define NULL 0
#endif

#define NotFound -1

namespace SG
{
	//Compiler/implementation dependent typedefs
	typedef unsigned char Byte;
	typedef unsigned short Word;
	typedef unsigned long DWord;
	typedef unsigned long long QWord;		//MS platform
										//typedef unsigned long long QWord;	//Linux and other Unices
	//typedef long double Real;

	typedef Byte Binary;	//reperesents 0 or 1
	typedef bool Boolean;	//reperesents true or false

	typedef DWord Counter;
	typedef Word SmallCounter;
	typedef QWord BigCounter;

	typedef signed long Num;
	typedef unsigned int FastInt;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace SG
{
	namespace stdx
	{
		//To ease operating on bits.

		//Checks if BitNo of data is set(1) or not. Returns 1 if yes, 0 otherwise.
		SG::Binary GetBit(SG::Byte BitNo,SG::Byte Data);

		//Sets BitNo of data as X (X = 0 or 1)
		void SetBit(SG::Byte BitNo,SG::Byte &Data,SG::Binary X);

		//Similar to above functions - for vector<SG::Byte>
		//inline SG::Binary GetArr(SG::Counter BitNo,std::vector<Byte> &Data);
		//inline void SetArr(SG::Counter BitNo,std::vector<Byte> &Data,SG::stdx::Type::Binary X);
	}
}

SG::Binary SG::stdx::GetBit(SG::Byte BitNo,SG::Byte Data)
{
	return (1<<BitNo)&Data?1:0;
}

void SG::stdx::SetBit(SG::Byte BitNo,SG::Byte &Data,SG::Binary X)
{
	if(X)
		Data=Data|(1<<BitNo);
	else
		Data=Data&(~(1<<BitNo));
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

namespace SG
{
	namespace io
	{
		class InputStream
		{
		protected:

		public:
			InputStream(){}
			virtual ~InputStream(){}

			virtual int ReadByte()=0;
			virtual SG::Boolean Ended()=0;
		};

		class OutputStream
		{
		protected:

		public:
			OutputStream(){}
			virtual ~OutputStream(){Flush();}

			virtual void WriteByte(SG::Byte Value)=0;
			virtual void Flush(){};
		};
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

namespace SG
{
	namespace io
	{
		class BitInputStream
		{
		protected:
			SG::Byte _Position;
			SG::Byte _Buffer;
			SG::io::InputStream &_Under;

		public:
			BitInputStream(SG::io::InputStream &Under);

			SG::Binary ReadBit();
			SG::Boolean Ended();
		};

		class BitOutputStream
		{
		protected:

			SG::Byte _Position;
			SG::Byte _Buffer;
			SG::io::OutputStream &_Under;

		public:
			BitOutputStream(SG::io::OutputStream &Under);
			~BitOutputStream();

			void WriteBit(SG::Binary Value);
			void Flush();
		};
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
SG::io::BitInputStream::BitInputStream(SG::io::InputStream &Under) :
	_Under(Under),
	_Position(0)
{
	_Buffer=_Under.ReadByte(); 
}

SG::Binary SG::io::BitInputStream::ReadBit()
{
	SG::Binary Result=stdx::GetBit(_Position,_Buffer);

	if(_Position==7)
	{
		_Position=0;
		_Buffer=_Under.ReadByte();
	}
	else
	{
		++_Position;
	}

	return Result;
}

SG::io::BitOutputStream::BitOutputStream(SG::io::OutputStream &Under) :
	_Under(Under),
	_Position(0)
{
}

SG::io::BitOutputStream::~BitOutputStream()
{
	Flush();
}

void SG::io::BitOutputStream::WriteBit(SG::Binary Value)
{
	stdx::SetBit(_Position,_Buffer,Value);

	if(_Position==7)
	{
		_Position=0;
		_Under.WriteByte(_Buffer);
	}
	else
	{
		++_Position;
	}
}

void SG::io::BitOutputStream::Flush()
{
	if(_Position!=0)
	{
		_Under.WriteByte(_Buffer);
		_Position=0;
		_Buffer=0;
	}
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

namespace SG
{
	namespace io
	{
		class ArrayInputStream : public SG::io::InputStream
		{
		private:
			SG::Counter _Offset;
			SG::Byte *_Array;
			SG::Counter _Size;

		public:
			ArrayInputStream(SG::Byte *Array,SG::Counter Size,SG::Counter InitialOffset=0);
			~ArrayInputStream(){}

			int ReadByte();
			SG::Boolean Ended();

			SG::Counter Tell(){return _Offset;}
			void Seek(SG::Counter Offset){_Offset=Offset;}
		};

		class ArrayOutputStream : public SG::io::OutputStream
		{
		private:
			SG::Counter _Offset;
			SG::Byte *_Array;
			SG::Counter _Size;

		public:
			ArrayOutputStream(SG::Byte *Array,SG::Counter Size,SG::Counter InitialOffset=0);
			~ArrayOutputStream(){}

			void WriteByte(SG::Byte Value);
			void Flush(){};

			SG::Counter Tell(){return _Offset;}
			void Seek(SG::Counter Offset){_Offset=Offset;}
		};
	}
}

SG::io::ArrayInputStream::ArrayInputStream(SG::Byte *Array,SG::Counter Size,SG::Counter InitialOffset) :
	_Array(Array),
	_Size(Size),
	_Offset(InitialOffset)
{
}

int SG::io::ArrayInputStream::ReadByte()
{
	int Result=_Offset<_Size ? _Array[_Offset]:(-1);
	++_Offset;
	return Result;
}

SG::Boolean SG::io::ArrayInputStream::Ended()
{
	return _Offset>=_Size;
}

SG::io::ArrayOutputStream::ArrayOutputStream(SG::Byte *Array,SG::Counter Size,SG::Counter InitialOffset) :
	_Array(Array),
	_Size(Size),
	_Offset(InitialOffset)
{
}

void SG::io::ArrayOutputStream::WriteByte(SG::Byte Value)
{
	//if(_Offset>=_Size) throw stdx::Exception("Buffer Overflow","SG::io::ArrayOutputStream::WriteByte");
	_Array[_Offset]=Value;
	++_Offset;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

namespace SG
{
	namespace Entropy
	{
		/*	Code for arithmetic coding
		derived from work by Mark Nelson, Tom st Denis, Charles Bloom

		author : Sachin Garg
		*/
		class ArithmeticCoder32
		{
		public:
			static const SG::DWord MaxRange;

		protected:

			ArithmeticCoder32();
			SG::DWord High,Low,UnderflowCount;
			SG::DWord TempRange;
		};

		class ArithmeticEncoder32 : public ArithmeticCoder32
		{
		private:
			SG::Boolean Flushed;
			SG::io::BitOutputStream &Output;

		public:
			ArithmeticEncoder32(SG::io::BitOutputStream &BitOStream);
			~ArithmeticEncoder32();

			void EncodeRange(SG::DWord SymbolLow,SG::DWord SymbolHigh,SG::DWord TotalRange);
			void Flush();
		};

		class ArithmeticDecoder32 : public ArithmeticCoder32
		{
		private:
			SG::DWord Code;
			SG::io::BitInputStream &Input;

		public:
			ArithmeticDecoder32(SG::io::BitInputStream &BitIStream);

			SG::DWord GetCurrentCount(SG::DWord TotalRange);
			void RemoveRange(SG::DWord SymbolLow,SG::DWord SymbolHigh,SG::DWord TotalRange);
		};
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

const SG::DWord	SG::Entropy::ArithmeticCoder32::MaxRange=0x3FFF;

SG::Entropy::ArithmeticCoder32::ArithmeticCoder32() :
	High(0xFFFF),
	Low(0),
	UnderflowCount(0),
	TempRange(0)
{
}

SG::Entropy::ArithmeticEncoder32::ArithmeticEncoder32(SG::io::BitOutputStream &BitOStream) :
	Flushed(false),
	Output(BitOStream)
{
}

SG::Entropy::ArithmeticEncoder32::~ArithmeticEncoder32()
{
	if(!Flushed) Flush();
}

void SG::Entropy::ArithmeticEncoder32::EncodeRange(SG::DWord SymbolLow,SG::DWord SymbolHigh,SG::DWord TotalRange)
{
	TempRange=(High-Low)+1;
	High=Low + ((TempRange*SymbolHigh)/TotalRange)-1;
	Low	=Low + ((TempRange*SymbolLow )/TotalRange);

	for(;;)
	{
		if((High & 0x8000)==(Low & 0x8000))
		{
			Output.WriteBit(High>>15);
			while(UnderflowCount)
			{
				Output.WriteBit((High>>15)^1);
				UnderflowCount--;
			}
		}
		else
		{
			if((Low	& 0x4000) && !(High	& 0x4000))
			{
				UnderflowCount++;

				Low	 &=	0x3FFF;
				High |=	0x4000;
			}
			else
				return;
		}

		Low	=(Low<<1) &	0xFFFF;
		High=((High<<1)|1) & 0xFFFF;
	}
}

void SG::Entropy::ArithmeticEncoder32::Flush()
{
	if(!Flushed)
	{
		Output.WriteBit((Low>>14)&1);
		UnderflowCount++;

		while(UnderflowCount)
		{
			Output.WriteBit(((Low>>14)^1)&1);
			UnderflowCount--;
		}

		Output.Flush();
		Flushed=true;
	}
}

SG::Entropy::ArithmeticDecoder32::ArithmeticDecoder32(SG::io::BitInputStream &BitIStream) :
	Code(0),
	Input(BitIStream)
{
	for(SG::FastInt I=0;I<16;I++)
	{
		Code<<=1;
		Code+=Input.ReadBit();;
	}
}

SG::DWord SG::Entropy::ArithmeticDecoder32::GetCurrentCount(SG::DWord TotalRange)
{
	TempRange=(High-Low)+1;
	return (SG::DWord)(((((Code-Low)+1)*(SG::QWord)TotalRange)-1)/TempRange);
}

void SG::Entropy::ArithmeticDecoder32::RemoveRange(SG::DWord SymbolLow,SG::DWord SymbolHigh,SG::DWord TotalRange)
{
	TempRange=(High-Low)+1;
	High=Low+((TempRange*SymbolHigh)/TotalRange)-1;
	Low	=Low+((TempRange*SymbolLow )/TotalRange);

	for(;;)
	{
		if((High & 0x8000) == (Low & 0x8000))
		{
		}
		else
		{
			if((Low	& 0x4000) && !(High	& 0x4000))
			{
				Code ^=	0x4000;
				Low	 &=	0x3FFF;
				High |=	0x4000;
			}
			else
				return;
		}
		Low	 = (Low	<< 1) &	0xFFFF;
		High = ((High<<1) |	1) & 0xFFFF;

		Code <<=1;
		Code|=Input.ReadBit();
		Code &=0xFFFF;
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

using namespace std;
using namespace SG;

void Rescale(Counter *Frequency, int size) {
	for(int i=1;i<size;i++) {
		Frequency[i]/=2;
		if(Frequency[i]<=Frequency[i-1]) Frequency[i]=Frequency[i-1]+1;
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//A quick test for entropty coders. Uses order-0 model.
u32 ArithEncode32(byte *Inp, u32 inpLen, byte *Out, u32 outLen)
{
	//Counter FileSizeB;
	//Counter Seconds, OutputSize;

	io::ArrayInputStream ByteStream(Inp, inpLen);
	io::ArrayOutputStream  OutputStream(Out,outLen);
	io::BitOutputStream  BitStream(OutputStream);

	//for(int i=0;i<sizeof(Counter);i++) OutputStream.WriteByte(((Byte*)&FileSizeB)[i]);

	SG::Entropy::ArithmeticEncoder32 EntropyCoder(BitStream);

	Counter Freq[257];

	for(int i=0;i<ArraySize(Freq);i++) Freq[i]=i;

	for(int i=0;i<inpLen;i++)
	{
		Byte ch=ByteStream.ReadByte();
		EntropyCoder.EncodeRange(Freq[ch],Freq[ch+1],Freq[ArraySize(Freq)-1]);

		for(int j=ch+1;j<ArraySize(Freq);j++) Freq[j]++;

		if(Freq[ArraySize(Freq)-1]>=EntropyCoder.MaxRange) Rescale(Freq, ArraySize(Freq));
	};

	EntropyCoder.Flush();
	BitStream.Flush();

	return OutputStream.Tell();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 ArithDecode32(byte *Inp, u32 inpLen, byte *Out, u32 outLen)
{
	io::ArrayInputStream InputStream(Inp, inpLen);
	io::ArrayOutputStream ByteStream(Out, outLen);

	//for(int i=0;i<sizeof(Counter);i++) ((Byte *)&OutputSize)[i]=InputStream.ReadByte();
	
	io::BitInputStream BitStream(InputStream);

	SG::Entropy::ArithmeticDecoder32 EntropyCoder(BitStream);

	Counter Freq[257];
	
	for(int i=0;i<ArraySize(Freq);i++) Freq[i]=i;

	for(int i=0;i<outLen;i++)
	{
		Counter Count = EntropyCoder.GetCurrentCount(Freq[ArraySize(Freq)-1]);

		Byte Symbol;
		for(Symbol=255;Freq[Symbol]>Count;Symbol--);
		//Symbol--;

		ByteStream.WriteByte(Symbol);
		EntropyCoder.RemoveRange(Freq[Symbol],Freq[Symbol+1],Freq[ArraySize(Freq)-1]);

		for(int j=Symbol+1;j<ArraySize(Freq);j++) Freq[j]++;

		if(Freq[ArraySize(Freq)-1]>=EntropyCoder.MaxRange) Rescale(Freq, ArraySize(Freq));
	}

	//Fout.write((char*)OutputFile,OutputSize);

	return 0;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//A quick test for entropty coders. Uses order-0 model.
u32 ArithEncode32_4(byte *Inp, u32 inpLen, byte *Out, u32 outLen)
{
	//Counter FileSizeB;
	//Counter Seconds, OutputSize;

	//io::ArrayInputStream ByteStream(Inp, inpLen);
	io::ArrayOutputStream  OutputStream(Out,outLen);
	io::BitOutputStream  BitStream(OutputStream);

	//for(int i=0;i<sizeof(Counter);i++) OutputStream.WriteByte(((Byte*)&FileSizeB)[i]);

	SG::Entropy::ArithmeticEncoder32 EntropyCoder(BitStream);

	Counter Freq[17];

	for(int i=0;i<ArraySize(Freq);i++) Freq[i]=i;

	for(int i=0;i<inpLen*2;i++)
	{
		Byte ch = (Inp[i>>1]>>(i&1))&15 ;//ByteStream.ReadByte();

		EntropyCoder.EncodeRange(Freq[ch],Freq[ch+1],Freq[ArraySize(Freq)-1]);

		for(int j=ch+1;j<ArraySize(Freq);j++) Freq[j]++;

		if(Freq[ArraySize(Freq)-1]>=EntropyCoder.MaxRange) Rescale(Freq, ArraySize(Freq));
	};

	EntropyCoder.Flush();
	BitStream.Flush();

	return OutputStream.Tell();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
