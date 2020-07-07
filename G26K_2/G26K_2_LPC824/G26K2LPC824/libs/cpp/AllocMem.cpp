#include "types.h"
#include "time.h"
#include "AllocMem.h"

#ifdef WIN32
#include <windows.h>
#include "core.h"

T_HW::LPC_IHP VectorTableInt[16];
T_HW::LPC_IHP VectorTableExt[45];
byte core_sys_array[0x100000];
byte CM3_sys_array[0x1000];

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

static byte* memptr = (byte*)0x60000000;
static byte* nvmemptr = (byte*)0x60100000;

#else

static byte* memptr = 0;
static byte* nvmemptr = 0;

static byte* s_nvmemptr = 0;

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void* AllocMem(dword size)
{
#ifdef WIN32

	if (s_nvmemptr == 0)
	{
		s_nvmemptr = memptr = (byte*)VirtualAlloc(0, 0x200000, MEM_COMMIT, PAGE_READWRITE);
		nvmemptr = memptr + 0x100000;
		LoadMemNV();
	};

#endif

	size = (size+3) & 0xFFFFFFF8;

	byte* p = memptr;
	memptr += size;
	return p;

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void* AllocMemNV(dword size)
{
#ifdef WIN32

	if (s_nvmemptr == 0)
	{
		s_nvmemptr = memptr = (byte*)VirtualAlloc(0, 0x200000, MEM_COMMIT, PAGE_READWRITE);
		nvmemptr = memptr + 0x100000;
		LoadMemNV();
	};

#endif

	size = (size+3) & 0xFFFFFFF8;

	byte* p = nvmemptr;
	nvmemptr += size;
	return p;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

void SaveMemNV()
{
	if (s_nvmemptr != 0)
	{
		HANDLE h;

		h = CreateFile("STORE.BIN", GENERIC_WRITE, 0, 0, OPEN_ALWAYS, 0, 0);

		if (h == INVALID_HANDLE_VALUE)
		{
			return;
		};

		dword bytes;

		if (!WriteFile(h, s_nvmemptr, 0x200000, &bytes, 0))
		{
			dword le = GetLastError();
		};

		CloseHandle(h);
	};
}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef WIN32

void LoadMemNV()
{
	if (s_nvmemptr == 0)
	{
		s_nvmemptr = memptr = (byte*)VirtualAlloc(0, 0x200000, MEM_COMMIT, PAGE_READWRITE);
		nvmemptr = memptr + 0x100000;
	};

	HANDLE h;

	h = CreateFile("STORE.BIN", GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	ReadFile(h, s_nvmemptr, 0x200000, &bytes, 0);
	CloseHandle(h);
}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
