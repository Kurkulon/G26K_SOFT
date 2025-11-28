#include "hw_conf.h"
#include "req.h"

#include "CRC\CRC16.h"
#include "time.h"

//#pragma O3
//#pragma Otime

//List<R01> R01::freeList;
//static R01 r02[8];

//List< PtrItem<REQ> > PtrItem<REQ>::_freeList;
//template <class T> List< PtrItem<T> > PtrItem<T>::_freeList;
//static REQ reqArray[8];

#define REQUEST_DATA_LEN (sizeof(ReqUnion))

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "RequestQuery_imp.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
