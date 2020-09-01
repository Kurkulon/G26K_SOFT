#ifndef TFTP_H__25_03_2020__18_36
#define TFTP_H__25_03_2020__18_36

#include "emac.h"
#include "main.h"

extern bool TFTP_Idle();
extern bool RequestTFTP(MEMB* mb);
inline bool TFTP_Connected() { extern u16 tftp_processed; return tftp_processed != 0; }  

#endif // TFTP_H__25_03_2020__18_36


