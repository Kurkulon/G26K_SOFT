#include "list.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> T* List<T>::Get()
{
	u32 i = cli();

	T* r = first;

	if (r != 0)
	{
		first = (T*)r->next;

//		r->next = 0;

		if (first == 0)
		{
			last = 0;
		};
	};

	sti(i);

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> void List<T>::Add(T* r)
{
	if (r == 0)
	{
		return;
	};

	u32 i = cli();

	if (last == 0)
	{
		first = last = r;
	}
	else
	{
		last->next = r;
		last = r;
	};

	r->next = 0;

	sti(i);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

