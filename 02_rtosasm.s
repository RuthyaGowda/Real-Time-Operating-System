

	.def SETPSP
	.def GETPSP
	.def PUSHPSP
	.def SETPSP6
	.def POPPSP
	.def PUSH_XPSR
	.def GET_SVC_NUMBER
	.def GET_R0
	.def GET_R1
	.def GET_R2
	.def GET_R3

.thumb
.const

.text

SETPSP:

	    MRS R1,CONTROL
	    ORR R1,R1,#2
	    MSR CONTROL,R1
	    MSR PSP,R0
	    BX  LR

GETPSP:
		 MRS R0,PSP
    	 BX LR

PUSHPSP:
		 MRS R0,PSP
         STMDB R0!,{R4-R11}
         MSR PSP,R0
         BX LR

SETPSP6:
		 MSR PSP,R0
    	 BX LR

POPPSP:
		 MRS R0,PSP
         LDMIA R0!,{R4-R11}
         MSR PSP,R0
         BX LR

PUSH_XPSR:
		 MOV R3,R0
		 MRS R0,PSP
		 MOV R2,#0x01000000
		 SUB R0,#4    ;xpsr
		 str R2,[R0]
         SUB R0,#4    ;PC
         str R3,[R0]
         SUB R0,#4    ;LR
         SUB R0,#4    ;R12
       	 SUB R0,#4    ;R3
       	 SUB R0,#4    ;R2
       	 SUB R0,#4     ;R1
       	 SUB R0,#4    ;R0
       	 MSR PSP,R0
         BX LR

GET_SVC_NUMBER:
		 MRS R0,PSP
	     ADD R0,#24
         LDR R2,[R0]
         SUB R2,#2
         LDRB R3,[R2]
         MOV R0,R3
         BX LR

GET_R0:
		 MRS R0,PSP
         LDR R0,[R0]
         ;MOV R0,R3
         BX LR

GET_R1:
		MRS R0,PSP
        LDR R0,[R0,#4]
        BX LR

GET_R2:
		MRS R0,PSP
        LDR R0,[R0,#8]
        BX LR

GET_R3:
		MRS R0,PSP
        LDR R0,[R0,#12]
        BX LR



.endm
