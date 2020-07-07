#ifndef LIST_H__03_02_2016__14_38
#define LIST_H__03_02_2016__14_38

#include <types.h>

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> struct List
{

protected:

	T *first;
	T *last;

	u32 count;

  public:

	List() : first(0), last(0), count(0) {}

	T*		Get();
	void	Add(T* r);

	bool	Empty() { return first == 0; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> T* List<T>::Get()
{
	T* r = first;

	if (r != 0)
	{
		first = (T*)r->next;

//		r->next = 0;

		if (first == 0)
		{
			last = 0;
		};

		count--;
	};

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> void List<T>::Add(T* r)
{
	if (r == 0)
	{
		return;
	};

	if (last == 0)
	{
		first = last = r;
	}
	else
	{
		last->next = r;
		last = r;
	};

	count++;

	r->next = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // LIST_H__03_02_2016__14_38
