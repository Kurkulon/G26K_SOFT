#ifndef LIST_H__03_02_2016__14_38
#define LIST_H__03_02_2016__14_38

#include <types.h>

#ifdef WIN32

#include <intrin.h>

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> struct List
{

protected:

	T *first;
	T *last;

	List* _selfPtr;

  public:

	void Init() { if (_selfPtr != this) first = 0, last = 0, _selfPtr = this; }
	
	List() { Init(); }

	T*		Get();
	void	Add(T* r);

	bool	Empty() { return first == 0; }


#ifdef WIN32

public:

	u32		locks_count;

protected:

	long	locked;

    enum {LOCK_IS_FREE = 0, LOCK_IS_TAKEN = 1};

	void Lock()		{ while (_InterlockedCompareExchange(&locked, LOCK_IS_TAKEN, LOCK_IS_FREE) == LOCK_IS_TAKEN) locks_count++; }
	void Unlock()	{ _InterlockedExchange(&locked, LOCK_IS_FREE);  }

#else

	void Lock()		{ }
	void Unlock()	{ }

#endif

};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> T* List<T>::Get()
{
	Lock();

	T* r = first;

	if (r != 0)
	{
		first = (T*)r->next;

		if (first == 0)
		{
			last = 0;
		};
	};

	Unlock();
	
	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> void List<T>::Add(T* r)
{
	if (r == 0)
	{
		return;
	};

	Lock();

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

	Unlock();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct ListItem
{
	//friend class List<ListItem>;

	ListItem	*next;
	void		*item;

	static	List<ListItem> freeItemList;

	static	ListItem*	Alloc()	{ return freeItemList.Get(); }
			void		Free()	{ freeItemList.Add(this); }

			ListItem() : next(0), item(0) { freeItemList.Init(); freeItemList.Add(this); }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> struct ListPtr;
template <class T> struct ListRef;

template <class T> class Ptr
{
	friend class ListPtr<T>;
	friend class ListRef<T>;

protected:

	T* ptr;

public:

	Ptr() : ptr(0)					{}
	Ptr(T* p) : ptr(p)				{ if (ptr != 0) ptr->count++; }
	Ptr(const Ptr& p) : ptr(p.ptr)	{ if (ptr != 0) ptr->count++; }

	Ptr& operator=(const Ptr& p)	{ if (ptr != p.ptr) { if (ptr != 0) ptr->Free(); ptr = p.ptr; if (ptr != 0) ptr->count++; }; return *this; }
	~Ptr()							{ if (ptr != 0) ptr->Free(); }
	bool Valid() const				{ return ptr != 0; }
	void Alloc()					{ if (ptr != 0) ptr->Free(); ptr = T::Alloc(); }
	void Free()						{ if (ptr != 0) ptr->Free(), ptr = 0; }

	T* operator->()					{ return ptr; }
	T& operator*()					{ return *ptr; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> struct ListPtr
{

protected:

	T* first;
	T* last;

	ListPtr* _selfPtr;

public:

	void Init() { if (_selfPtr != this) first = 0, last = 0, _selfPtr = this; }

	ListPtr() { Init(); }

	Ptr<T> Get();

	void	Add(const Ptr<T> &r);

	//bool	Empty() { return first == 0; }
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> Ptr<T> ListPtr<T>::Get()
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

		//counter--;

		r->count--;
	};

	return Ptr<T>(r);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> void ListPtr<T>::Add(const Ptr<T>& r)
{
	if (r.ptr == 0)
	{
		return;
	};

	if (last == 0)
	{
		first = last = r.ptr;
	}
	else
	{
		last->next = r.ptr;
		last = r.ptr;
	};

	r.ptr->count++;

	//counter++;

	r.ptr->next = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> struct ListRef
{

protected:

	ListItem* first;
	ListItem* last;

	ListRef* _selfPtr;

public:

	void Init() { if (_selfPtr != this) first = 0, last = 0, _selfPtr = this; }

	ListRef() { Init(); }

	Ptr<T> Get();

	bool Add(const Ptr<T> &r);
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> Ptr<T> ListRef<T>::Get()
{
	T* r = 0;

	ListItem* i = first;

	if (i != 0)
	{
		r = (T*)(i->item);

		first = i->next;

		if (first == 0)
		{
			last = 0;
		};

		if (r != 0) r->count--;
		
		i->Free();
	};

	return Ptr<T>(r);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class T> bool ListRef<T>::Add(const Ptr<T>& r)
{
	if (!r.Valid()) return false;

	ListItem *item = ListItem::Alloc(); 

	if (item == 0) return false;

	item->item = r.ptr;

	if (last == 0)
	{
		first = last = item;
	}
	else
	{
		last->next = item;
		last = item;
	};

	r.ptr->count++;
	item->next = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <class T> struct PtrItem;

template <class T> struct PtrItem
{
	friend class Ptr<T>;
	friend class List<T>;
	friend class ListPtr<T>;

	PtrItem*	next;
	u32	count;

protected:

	static List<PtrItem> _freeList;

	static	T*		Alloc()	{ T* p = (T*)_freeList.Get(); if (p != 0) p->count = 1; return p; };

	virtual	void	_FreeCallBack() {}
			void	Free()	{ if (count != 0) { count--; if (count == 0) _FreeCallBack(), _freeList.Add(this); }; }

public:

	PtrItem() : next(0), count(0) { _freeList.Init(); _freeList.Add(this); }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // LIST_H__03_02_2016__14_38
