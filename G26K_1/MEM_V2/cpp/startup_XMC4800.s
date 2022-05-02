;*******************************************************************************
;* @file     startup_XMC4800.s
;* @brief    CMSIS Core Device Startup File for
;*           Infineon XMC4800 Device Series
;* @version  V1.0
;* @date     22 May 2015
;*
;* @cond
;*********************************************************************************************************************
;* Copyright (c) 2015-2016, Infineon Technologies AG
;* All rights reserved.                        
;*                                             
;* Redistribution and use in source and binary forms, with or without modification,are permitted provided that the 
;* following conditions are met:   
;*                                                                              
;* Redistributions of source code must retain the above copyright notice, this list of conditions and the following 
;* disclaimer.                        
;* 
;* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following 
;* disclaimer in the documentation and/or other materials provided with the distribution.                       
;* 
;* Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote 
;* products derived from this software without specific prior written permission.                                           
;*                                                                              
;* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
;* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE  
;* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
;* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR  
;* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
;* WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
;* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                  
;*                                                                              
;* To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes with 
;* Infineon Technologies AG dave@infineon.com).                                                          
;*********************************************************************************************************************
;*
;************************** Version History ************************************
; V1.00, 22 May 2015, First version
;*******************************************************************************
;* @endcond

; ------------------ <<< Use Configuration Wizard in Context Menu >>> ------------------
           
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000800

                AREA    STACK, NOINIT, READWRITE, ALIGN=8
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3, LINKORDER=STACK
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

VecTableIntSize		EQU		16*4	
VecTableExtSize		EQU		112*4	
SCB_VTOR			EQU		0xE000ED08
SeggerRttCB_size	EQU		0x2000

				AREA	||.ARM.__AT_0x20000000||, DATA, NOINIT, ALIGN=7
				EXPORT  VectorTableInt
                EXPORT  VectorTableExt
VectorTableInt	SPACE	VecTableIntSize				
VectorTableExt	SPACE	VecTableExtSize				

                ALIGN	128

                EXPORT  SeggerRttCB
SeggerRttCB		SPACE	SeggerRttCB_size - (SeggerRttCB-VectorTableInt)		

                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, CODE, READONLY, ALIGN=4
                
__Vectors       DCD     __initial_sp              ; 0 Top of Stack
                DCD     Reset_Handler             ; 1 Reset Handler


                ALIGN	16
                DCB		"\n", __PROGNAME__, "\n"
                DCB		__DATE__, "\n"
                DCB		__TIME__, "\n"

                ALIGN	16

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

_MainAppStart	PROC
				EXPORT	_MainAppStart
				
				LDR		R1, [R0]
				MOV		SP, R1
				LDR		R0, [R0, #4]
				BX		R0

                ENDP

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler           [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                
                MOVS	R0, #(VecTableIntSize+VecTableExtSize-4)
                LDR		R1, =VectorTableInt
                LDR		R2, =Dummy_Handler
|L1.16|
				STR		R2, [R1,R0]
                SUBS	R0, #4
                BNE		|L1.16|
                
                LDR		R0, =SCB_VTOR 
                STR		R1, [R0]		;CM0::SCB->VTOR = VectorTableInt
                
                MOVS	R0, #(2*4)
                LDR		R2, =NMI_Handler
                STR		R2, [R1,R0]
                
                ADD		R0, #4
                LDR		R2, =HardFault_Handler
                STR		R2, [R1,R0]
                
                ADD		R0, #4
                LDR		R2, =MemManage_Handler
                STR		R2, [R1,R0]
                
                ADD		R0, #4
                LDR		R2, =BusFault_Handler
                STR		R2, [R1,R0]
                
                ADD		R0, #4
                LDR		R2, =UsageFault_Handler
                STR		R2, [R1,R0]
                
                ADD		R0, #(4*5)
                LDR		R2, =SVC_Handler
                STR		R2, [R1,R0]
                
                ADD		R0, #4
                LDR		R2, =DebugMon_Handler
                STR		R2, [R1,R0]
                
                ADD		R0, #8
                LDR		R2, =PendSV_Handler
                STR		R2, [R1,R0]
                
                
;                LDR		R0, =(Stack_Size-4)
;                LDR		R1, =Stack_Mem
;                MOVS	R2, #(0xAAAAAAAA)
;|L1.17|
;				STR		R2, [R1,R0]
;                SUBS	R0, #4
;                BNE		|L1.17|
                

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

NMI_Handler			PROC	;2
					LDR		R0,[SP,#0x18]	; R0 = return Address
					MRS		R1, IPSR		; R1 = Exception number
					BKPT	#0
					BX		lr
					ENDP
					
HardFault_Handler   PROC	;3	
					LDR		R0,[SP,#0x18]	; R0 = return Address
					MRS		R1, IPSR		; R1 = Exception number
					BKPT	#0
					BX		lr
					ENDP
					
MemManage_Handler   PROC	;4	
					LDR		R0,[SP,#0x18]	; R0 = return Address
					MRS		R1, IPSR		; R1 = Exception number
					BKPT	#0
					BX		lr
					ENDP
					
BusFault_Handler	PROC	;5	
					LDR		R0,[SP,#0x18]	; R0 = return Address
					MRS		R1, IPSR		; R1 = Exception number
					BKPT	#0
					BX		lr
					ENDP
					
UsageFault_Handler  PROC	;6	
					LDR		R0,[SP,#0x18]	; R0 = return Address
					MRS		R1, IPSR		; R1 = Exception number
					BKPT	#0
					BX		lr
					ENDP

SVC_Handler			PROC	;11	
					LDR		R0,[SP,#0x18]	; R0 = return Address
					MRS		R1, IPSR		; R1 = Exception number
					BKPT	#0
					BX		lr
					ENDP

DebugMon_Handler	PROC	;12	
					LDR		R0,[SP,#0x18]	; R0 = return Address
					MRS		R1, IPSR		; R1 = Exception number
					BKPT	#0
					BX		lr
					ENDP
					
PendSV_Handler		PROC	;14	
					LDR		R0,[SP,#0x18]	; R0 = return Address
					MRS		R1, IPSR		; R1 = Exception number
					BKPT	#0
					BX		lr
					ENDP



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

; Dummy Exception Handlers (infinite loops which can be modified)

;[SP+0x00] = R[0];
;[SP+0x04] = R[1];
;[SP+0x08] = R[2];
;[SP+0x0C] = R[3];
;[SP+0x10] = R[12];
;[SP+0x14] = LR;
;[SP+0x18] = ReturnAddress();
;[SP+0x1C] = xPSR;


Dummy_Handler   PROC

				LDR		R0,[SP,#0x18]	; R0 = return Address
				MRS		R1, IPSR		; R1 = Exception number
				BKPT	#0
				BX		lr
                ENDP

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
             
                ALIGN

; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
