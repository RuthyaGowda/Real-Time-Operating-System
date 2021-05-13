// RTOS Framework - Spring 2021
// J Losh

// Student Name: Manjunath Elechithaya Dinesh
//               Ruthya Chikkaputte Gowda
// TO DO: Add your name(s) on this line.
//        Do not include your ID number in the file.

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files (except uart0.x and wait.x)
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Orange: PE2
// Yellow: PE3
// Green:  PE4
// PBs on these pins
// PB0:    PA7
// PB1:    PA6
// PB2:    PA5
// PB3:    PA4
// PB4:    PA3
// PB5:    PA2
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
//#include <stdlib.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED  Port F
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED  Port E
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board green LED Port E
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED Port E
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board orange LED Port E

#define PB0   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define PB1   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define PB2   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define PB3   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PB4   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PB5   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))

#define BLUE_LED_MASK         4
#define RED_LED_MASK         2
#define GREEN_LED_MASK       16
#define YELLOW_LED_MASK      8
#define ORANGE_LED_MASK      4

#define PB0_MASK             128
#define PB1_MASK             64
#define PB2_MASK             32
#define PB3_MASK             16
#define PB4_MASK             8
#define PB5_MASK             4
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)(void);

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
uint32_t heap[12][512];

#define MAX_CHARS 80
#define MAX_ARGS 5


extern void SETPSP(uint32_t p);
extern void* GETPSP(void);
extern void PUSHPSP(void);
extern void POPPSP(void);
extern void SETPSP6(void*);
extern void PUSH_XPSR(void*);
extern uint16_t GET_SVC_NUMBER(void);
extern uint32_t GET_R0(void);
extern int GET_R1(void);
extern int GET_R2(void);
extern int GET_R3(void);

typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];
#define keyPressed 0
#define keyReleased 1
#define flashReq    2
#define resource    3

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint8_t scheduler=0, pre;

uint32_t time1[2][MAX_TASKS];
uint32_t buff=0;
uint32_t whole[MAX_TASKS];
uint32_t decimal[MAX_TASKS];

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on multiples of 4 bytes

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 7=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    uint32_t timeout;
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];


enum SVCNUMBERS
{
    yields = 19,
    sleeps = 21,
    waits = 23,
    posts = 25,
    //pss=29,
    //deletes = 35,
    resume = 30

};


//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos(void)
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler(void)
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    uint8_t currtask;
    uint8_t i;
    uint8_t priorityLevel = 0;
    if(scheduler == 0)    // by default priority scheduler
    {
        while (!ok)
        {
                    //Fetch Current Task
            currtask = taskCurrent;
                    //If priority level
            if(priorityLevel >= 7)
                priorityLevel = 0;
                    //loop MAX_TASKS times to check for lowest priority READY task
            for(i = 1;i <= MAX_TASKS;i++)
            {
                //increment on every loop
                currtask = currtask + 1;
                if(currtask >= MAX_TASKS)
                    currtask = 0;
                if(tcb[currtask].priority == priorityLevel)
                {
                    ok = (tcb[currtask].state == STATE_READY || tcb[currtask].state == STATE_UNRUN);
                    if(ok)
                    {
                        return currtask;
                    }
                }
            }
            priorityLevel++;
        }
    }
    else if(scheduler == 1)    // Round robin scheduler
    {
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
        return task;
    }
    return 0;
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0,j=0;
    bool found = false;
  //  int j=0;
    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            while(name[j]!='\0')
            {
                tcb[i].name[j] = name[j];
                j++;
            }
           // tcb[i].name[r]='\0';
            tcb[i].sp = &heap[i][512];
            tcb[i].spInit=tcb[i].sp;
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    // __asm("  SVC #30");
    uint32_t R0V=GET_R0();
    int i;
    for(i=0;i<MAX_TASKS;i++)
             {
               if(tcb[i].pid == (_fn)R0V)
                {
                   tcb[i].sp = tcb[i].spInit;
                   //taskCount++;
                   tcb[i].state = STATE_UNRUN;
                   break;
                 }
             }
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
   //  __asm("  SVC #35");
    uint32_t R0V=GET_R0();
    struct _semaphore *killsema;
    int i;

for(i = 0;i < MAX_TASKS;i++)
{
    if(tcb[i].pid==(_fn)R0V)
    {
        if(tcb[i].state==STATE_BLOCKED)
        {
            killsema=tcb[i].semaphore;
            killsema->queueSize--;
            killsema->processQueue[killsema->queueSize]=0 ;


        }
        tcb[i].state =STATE_INVALID ;
        tcb[i].timeout=0;
    }
}
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    int i;
    for(i=0;i<MAX_TASKS;i++)
    {
        if(tcb[i].pid==fn)
        {
            tcb[i].priority=priority;
            tcb[i].currentPriority=priority;
            break;
        }
    }
}

bool createSemaphore(uint8_t semaphore, uint8_t count)
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos(void)
{
    taskCurrent= rtosScheduler();
    tcb[taskCurrent].state = STATE_READY;
    SETPSP((uint32_t)tcb[taskCurrent].sp);

    // Timer Configuration
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 4294967295;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

   _fn fn=(_fn)(tcb[taskCurrent].pid);
   (*fn)();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield(void)
{
    __asm("  SVC #19");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm("  SVC #21");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(int8_t semaphore)
{
    __asm("  SVC #23");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm("  SVC #25");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr(void)
{
    static uint16_t timecounter = 1000;
    int i = 0;


        //Support for sleep command
        for(i = 0; i < MAX_TASKS; i++)
        {
            if(tcb[i].state == STATE_DELAYED)
            {
                if(tcb[i].timeout == 0)
                {
                    tcb[i].state = STATE_READY;
                }
                tcb[i].timeout--;
            }
        }
        // timer t store cpu time
        timecounter--;
        if(timecounter == 0)
        {
            buff ^= 1;
            timecounter = 1000;
            int j=0;
            for(j = 0;j < MAX_TASKS;j++)
            {
                time1[buff][j] = 0;
            }
        }
        // preemption
        if(pre == 1)
        {
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr(void)
{
    int T;
    PUSHPSP();
    tcb[taskCurrent].sp = GETPSP();

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    T=TIMER1_TAV_R;
    time1[buff][taskCurrent]+=T;


    taskCurrent=rtosScheduler();
    SETPSP6(tcb[taskCurrent].sp);


    if(tcb[taskCurrent].state == STATE_UNRUN)
    {
        tcb[taskCurrent].state = STATE_READY;
        PUSH_XPSR(tcb[taskCurrent].pid);
    }
    else if(tcb[taskCurrent].state == STATE_READY)
        {
        POPPSP();
        }

    TIMER1_TAV_R=0;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr(void)
{
    uint8_t N;
    N = GET_SVC_NUMBER();
    uint32_t R0V=GET_R0();

    switch(N)
    {
    case(yields) :

            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

    case(sleeps) :

            tcb[taskCurrent].timeout = R0V;
            tcb[taskCurrent].state = STATE_DELAYED;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

    case(waits) :
           // timeout = GET_R0();
            if(semaphores[R0V].count > 0)

            {

                semaphores[R0V].count--;
            }
            else
            {
                tcb[taskCurrent].state = STATE_BLOCKED;
                tcb[taskCurrent].semaphore = &semaphores[R0V];
                semaphores[R0V].processQueue[semaphores[R0V].queueSize] = taskCurrent;
                semaphores[R0V].queueSize++;
            }
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

    case(posts):

         semaphores[R0V].count++;

         if(semaphores[R0V].queueSize > 0)
         {
             semaphores[R0V].count--;
             tcb[semaphores[R0V].processQueue[0]].state = STATE_READY;


             int l=0;
             for(l = 0;l < semaphores[R0V].queueSize;l++)
             {
                 semaphores[R0V].processQueue[l] = semaphores[R0V].processQueue[l+1];


             }
             semaphores[R0V].queueSize--;

         }
         NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
         break;
    }

}



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw(void)
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
            SYSCTL_GPIOHBCTL_R = 0;

            // Enable Clocks
            SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5;
            SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
            SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
            _delay_cycles(3);

            //Configure LED and pushbuttons
            GPIO_PORTE_DIR_R |= GREEN_LED_MASK | RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;
            GPIO_PORTF_DIR_R |= BLUE_LED_MASK;
            GPIO_PORTF_DR2R_R |= BLUE_LED_MASK;
            GPIO_PORTE_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;
            GPIO_PORTE_DEN_R |= GREEN_LED_MASK | RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;
            GPIO_PORTF_DEN_R |= BLUE_LED_MASK;
            GPIO_PORTA_DIR_R &=~ PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK | PB4_MASK | PB5_MASK;
            GPIO_PORTA_DEN_R |= PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK | PB4_MASK | PB5_MASK;
            GPIO_PORTA_PUR_R |= PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK | PB4_MASK | PB5_MASK;

             //systick configuration

            NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE;
            NVIC_ST_RELOAD_R |= 39999;      //(system clock(40MHz)/1kHz)-1
            NVIC_ST_CURRENT_R |=  NVIC_ST_CURRENT_M;
 }

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs(void)
{
    if(PB0 == 0)
       {
           return 1;
       }
       if(PB1 == 0)
       {
           return 2;
       }
       if(PB2 == 0)
       {
           return 4;
       }
       if(PB3 == 0)
       {
           return 8;
       }
       if(PB4 == 0)
       {
           return 16;
       }
       if(PB5 == 0)
       {
           return 32;
       }
       return 0;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}


// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose








// REQUIRED: Remove this after task switching working
//void idle2(void)
//{
//    while(true)
//    {
//        RED_LED = 1;
//        waitMicrosecond(1000);
//        RED_LED = 0;
//        yield();
//    }
//}

// REQUIRED: Add other tasks here after task switching is complete
//           These will be provided in class

// REQUIRED: add processing for the shell commands through the UART here
void shell(void)
{
    char strbuffer[MAX_CHARS+1];
    uint8_t argcount;
    uint8_t index[MAX_CHARS];
    char type[MAX_CHARS];

    while (true)
    {
        while(!kbhitUart0())
        {
            yield();
        }
        getsUart0(strbuffer);
        putsUart0("\n\r");

        argcount=parse(strbuffer,index,type);
        bool valid=false;
        uint16_t i=0,j=0;

//-------------------------------ps Command--------------------------------------//
        if(iscommand("ps",0,strbuffer,argcount,index,type))
            {
            //ps
                uint32_t tau=0;

                uint64_t time2,pn;
                uint16_t i=0, j=0;
                uint32_t a,b,c,d;
               char pid[100];
               char wholes[100];
               char decimals[100];
               char prio[100];


               for(j = 0;j < taskCount; j++)
              {
                  tau += time1[!buff][j];
              }
              for(i = 0;i < taskCount; i++)
              {
                  time2 = time1[!buff][i];
                  time2 = time2 * 10000;
                  pn = (time2)/tau;
                  whole[i] = pn/100;
                  decimal[i] = pn%100;
              }

               putsUart0("Pid\t\tThread Name\t\t%Cpu time\tState\t\tPriority\n\r\n");
               for(i = 0;i < 10;i++)
               {

                   a = (uint32_t)tcb[i].pid;
                   itoa(pid,a);
                   putsUart0(pid);
                   putsUart0("\t\t");

                   putsUart0(tcb[i].name);
                   if(i==0 ||i==3|i==7||i==8||i==9)
                   {
                       putsUart0("\t\t\t");
                   }
                   else
                   {
                       putsUart0("\t\t");
                   }

                   b = (uint32_t)whole[i];
                   itoa(wholes, b);
                   c = (uint32_t)decimal[i];
                   itoa(decimals, c);
                   putsUart0(wholes);
                   putcUart0('.');
                   putsUart0(decimals);
                   putcUart0('%');
                   putsUart0("\t\t");

                   if(tcb[i].state== STATE_INVALID)
                   {
                       putsUart0("INVALID");
                   }
                   else if(tcb[i].state==STATE_UNRUN)
                   {
                       putsUart0("UNRUN");
                   }
                   else if(tcb[i].state==STATE_READY)
                   {
                       putsUart0("READY");
                   }
                   else if(tcb[i].state==STATE_DELAYED)
                   {
                       putsUart0("DELAYED");
                   }
                   else if(tcb[i].state==STATE_BLOCKED)
                   {
                       putsUart0("BLOCKED");
                   }

                   putsUart0("\t\t");

                   d=tcb[i].priority;
                   itoa(prio,d);
                   putsUart0(prio);

                  putsUart0("\r\n");
               }
                valid=true;
            }

//-------------------------------ipcs Command--------------------------------------//

        if(iscommand("ipcs",0,strbuffer,argcount,index,type))
        {
           uint32_t q,w;
           char qs[100];
           char ws[100];

            putsUart0("SEMAPHORE NAME\t\tCOUNT\tQUEUE SIZE\tWAITING\n\r\n");

            for( i=0;i<MAX_SEMAPHORES-1;i++)
            {
                if(i == keyPressed)
                   putsUart0("Key Pressed");
               else if(i == keyReleased)
                   putsUart0("Key Released");
               else if(i == flashReq)
                   putsUart0("Flash Req");
               else if(i == resource)
                   putsUart0("Resource");

                putsUart0("\t\t");

                q = (uint32_t)semaphores[i].count;
                itoa(qs,q);
                putsUart0(qs);
                putsUart0("\t");

                w = (uint32_t)semaphores[i].queueSize;
                itoa(ws,w);
                putsUart0(ws);
                putsUart0("\t\t");

                for( j=0;j<MAX_TASKS;j++)
                {
                    if(tcb[j].state==STATE_BLOCKED ||tcb[j].state==STATE_READY||tcb[j].state==STATE_DELAYED)
                    {
                        if(tcb[j].semaphore==semaphores+i)
                        {
                            putsUart0(tcb[j].name);
                            break;
                        }
                    }
                }

                putsUart0("\r\n");
            }
            valid=true;
    }

//------------------------------------kill Command----------------------------------//

        if(iscommand("kill",1,strbuffer,argcount,index,type))
        {
            uint32_t killed;
            killed = getvalue(0,strbuffer,argcount,index,type);
            destroyThread((_fn)killed);
            valid=true;
        }

//------------------------------------reboot Command--------------------------------//

        if(iscommand("reboot",0,strbuffer,argcount,index,type))
        {
           // __asm(" SVC #40");
            NVIC_APINT_R |= NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            valid=true;
        }

//------------------------------------pidof Command---------------------------------//
        if(iscommand("pidof",1,strbuffer,argcount,index,type))
        {
            uint32_t x;
            char y[100];
            for(i=0; i<MAX_TASKS;i++)
            {
               if(stringcmp(getstring(0,strbuffer,argcount,index,type),tcb[i].name)==1)
               {
                   x=(uint32_t)tcb[i].pid;
                   itoa(y,x);
                   putsUart0(y);

               }
            }
            valid=true;

        }

//------------------------------------taskname& Command--------------------------------//

        char *a;
        a="&";
        uint32_t nametask;
        if(stringcmp(getstring(0, strbuffer, argcount, index, type),a)==1)
        {
            for(i=0;i<MAX_TASKS;i++)
            {
                if(stringcmp(getstring(1, strbuffer, argcount, index, type), tcb[i].name))
                    {
                        valid=true;
                         nametask = (uint32_t)tcb[i].pid;
                         restartThread((_fn)nametask);
                         break;
                    }
            }
        }

//------------------------------------scheduler Command---------------------------------//

        if(iscommand("prio",1,strbuffer,argcount,index,type))
        {
//            char *r;//,*b;
//            r="ON";
           // b="OFF";
            if(stringcmp(getstring(0, strbuffer, argcount, index, type), "on")==true)
            {
                scheduler=0;
                putsUart0("Priority Scheduler is ON");
                putsUart0("\n\r");
            }
            if(stringcmp(getstring(0, strbuffer, argcount, index, type), "off")==true)
            {
                scheduler = 1;
                putsUart0("Round Robin Scheduler is ON");
                putsUart0("\r\n");
            }
            valid = true;
        }

//-------------------------------------preemption Command------------------------------//

        if(iscommand("preem",1,strbuffer,argcount,index,type))
        {
            if(stringcmp(getstring(0, strbuffer, argcount, index, type), "off")==true)
            {
                pre=0;
                putsUart0("Preemption is OFF");
                putsUart0("\n\r");
            }

            if(stringcmp(getstring(0, strbuffer, argcount, index, type), "on")==true)
            {
                pre = 1;
                putsUart0("Preemption is ON");
                putsUart0("\n\r");
            }
            valid = true;
        }
        if(!valid)
        {
            putsUart0("enter valid command\n\r");
        }
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initUart0();
    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    createSemaphore(keyPressed, 1);
    createSemaphore(keyReleased, 0);
    createSemaphore(flashReq, 5);
    createSemaphore(resource, 1);


    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);
    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);
    ok &= createThread(oneshot, "OneShot", 2, 1024);
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);
    ok &= createThread(debounce, "Debounce", 6, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 6, 1024);
    ok &= createThread(errant, "Errant", 6, 1024);
    ok &= createThread(shell, "Shell", 6, 4096);


    // Add other processes
   // ok &=  createThread(flash4Hz, "Flash4Hz", 6, 1024);
    //ok &=  createThread(oneshot, "OneShot", 3, 1024);

// REQUIRED: create threads for additional tasks here
//    ok &= createThread(shell, "Shell", 4, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
