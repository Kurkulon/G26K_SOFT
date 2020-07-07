#ifndef LIST_H__03_02_2016__14_38
#define LIST_H__03_02_2016__14_38

#include "types.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> struct List
{

protected:

	T *first;
	T *last;

	u32 counter;

  public:

	List() : first(0), last(0), counter(0) {}

	T*		Get();
	void	Add(T* r);

	bool	Empty() { return first == 0; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // LIST_H__03_02_2016__14_38
