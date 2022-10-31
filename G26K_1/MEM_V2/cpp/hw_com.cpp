//#pragma O3
//#pragma Otime

//#include <stdio.h>
//#include <conio.h>

#include <ComPort.h>
#include "hw_conf.h"
#include "hw_com.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef CPU_SAME53

ComPort commoto	(UART0_LPC, PIO_UTXD0, PIO_URXD0, PIO_RTS0, UTXD0, URXD0, RTS0, PMUX_UTXD0, PMUX_URXD0, UART0_TXPO, UART0_RXPO, UART0_GEN_SRC, UART0_GEN_CLK, &UART0_DMA);
ComPort comdsp	(UART1_DSP, PIO_UTXD1, PIO_URXD1, PIO_RTS1, UTXD1, URXD1, RTS1, PMUX_UTXD1, PMUX_URXD1, UART1_TXPO, UART1_RXPO, UART1_GEN_SRC, UART1_GEN_CLK, &UART2_DMA);

#elif defined(CPU_XMC48)

ComPort commoto	(UART0_USIC_NUM, PIO_USCK0, PIO_UTXD0, PIO_URXD0, PIO_RTS0, PIN_USCK0, PIN_UTXD0, PIN_URXD0, PIN_RTS0, MUX_USCK0, MUX_UTXD0, UART0_DX0CR, UART0_DX1CR, &UART0_DMA, UART0_DRL);	
ComPort comdsp	(UART1_USIC_NUM, PIO_USCK1, PIO_UTXD1, PIO_URXD1, PIO_RTS1, PIN_USCK1, PIN_UTXD1, PIN_URXD1, PIN_RTS1, MUX_USCK1, MUX_UTXD1, UART1_DX0CR, UART1_DX1CR, &UART1_DMA, UART1_DRL);		

#elif defined(WIN32)

ComPort commoto;	
ComPort comdsp;		

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <ComPort_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
