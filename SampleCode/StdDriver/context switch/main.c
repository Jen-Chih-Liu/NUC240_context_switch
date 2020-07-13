#include <stdio.h>
#include "NUC230_240.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define PLL_CLOCK   72000000
typedef uint32_t os_stack_t;
/* The maximum number of tasks: */
#define OS_CONFIG_MAX_TASKS	10
typedef enum {
	OS_TASK_STATUS_IDLE = 1,
	OS_TASK_STATUS_ACTIVE
} os_task_status_t;

typedef struct {
	/* The stack pointer (sp) has to be the first element as it is located
	   at the same address as the structure itself (which makes it possible
	   to locate it safely from assembly implementation of PendSV_Handler).
	   The compiler might add padding between other structure elements. */
	volatile uint32_t sp;
	void (*handler)(void);
	volatile os_task_status_t status;
} os_task_t;

static struct {
	os_task_t tasks[OS_CONFIG_MAX_TASKS];
	volatile uint32_t current_task;
	uint32_t size;
} m_task_table;

volatile os_task_t *os_curr_task;
volatile os_task_t *os_next_task;


static void task_finished(void)
{
	/* This function is called when some task handler returns. */
	volatile uint32_t i = 0;
	while (1)
		i++;
}

void os_init(void)
{
	memset(&m_task_table, 0, sizeof(m_task_table));
}

bool os_task_init(void (*handler)(void), os_stack_t *p_stack, uint32_t stack_size)
{
	if (m_task_table.size >= OS_CONFIG_MAX_TASKS-1)
		return false;

	/* Initialize the task structure and set SP to the top of the stack
	   minus 16 words (64 bytes) to leave space for storing 16 registers: */
	os_task_t *p_task = &m_task_table.tasks[m_task_table.size];
	p_task->handler = handler;
	p_task->sp = (uint32_t)(p_stack+stack_size-16);
	p_task->status = OS_TASK_STATUS_IDLE;

	/* Save special registers which will be restored on exc. return:
	   - XPSR: Default value (0x01000000)
	   - PC: Point to the handler function
	   - LR: Point to a function to be called when the handler returns */
	p_stack[stack_size-1] = 0x01000000;
	p_stack[stack_size-2] = (uint32_t)handler;
	p_stack[stack_size-3] = (uint32_t) &task_finished;

#ifdef OS_CONFIG_DEBUG
	uint32_t base = (m_task_table.size+1)*1000;
	p_stack[stack_size-4] = base+12;  /* R12 */
	p_stack[stack_size-5] = base+3;   /* R3  */
	p_stack[stack_size-6] = base+2;   /* R2  */
	p_stack[stack_size-7] = base+1;   /* R1  */
	p_stack[stack_size-8] = base+0;   /* R0  */
	p_stack[stack_size-9] = base+7;   /* R7  */
	p_stack[stack_size-10] = base+6;  /* R6  */
	p_stack[stack_size-11] = base+5;  /* R5  */
	p_stack[stack_size-12] = base+4;  /* R4  */
	p_stack[stack_size-13] = base+11; /* R11 */
	p_stack[stack_size-14] = base+10; /* R10 */
	p_stack[stack_size-15] = base+9;  /* R9  */
	p_stack[stack_size-16] = base+8;  /* R8  */
#endif

	m_task_table.size++;

	return true;
}


void config_systick(uint32_t us)  
{
    SysTick->LOAD =  (36*us)-1;      //systick clock=hclk/2=72000000/2=36000000;        
    SysTick->VAL   =     0;                  
    SysTick->CTRL  |=  SysTick_CTRL_TICKINT_Msk   | SysTick_CTRL_ENABLE_Msk; 
}

bool os_start(void)
{
	NVIC_SetPriority(PendSV_IRQn, 0xff); /* Lowest possible priority */
	NVIC_SetPriority(SysTick_IRQn, 0x00); /* Highest possible priority */


  config_systick(10000);
	
	
	/* Start the first task: */
	os_curr_task = &m_task_table.tasks[m_task_table.current_task];

	__set_PSP(os_curr_task->sp+64); /* Set PSP to the top of task's stack */
	__set_CONTROL(0x03); /* Switch to PSP, unprivilleged mode */
	__ISB(); /* Exec. ISB after changing CONTORL (recommended) */

	os_curr_task->handler();

	return true;
}



void SysTick_Handler(void)
{
	os_curr_task = &m_task_table.tasks[m_task_table.current_task];
	os_curr_task->status = OS_TASK_STATUS_IDLE;

	/* Select next task: */
	m_task_table.current_task++;
	if (m_task_table.current_task >= m_task_table.size)
		m_task_table.current_task = 0;

	os_next_task = &m_task_table.tasks[m_task_table.current_task];
	os_next_task->status = OS_TASK_STATUS_ACTIVE;

	/* Trigger PendSV which performs the actual context switch: */
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}


__asm void PendSV_Handler(void)
{
  IMPORT  os_curr_task
  IMPORT  os_next_task

	/*
	Exception frame saved by the NVIC hardware onto stack:
	+------+
	|      | <- SP before interrupt (orig. SP)
	| xPSR |
	|  PC  |
	|  LR  |
	|  R12 |
	|  R3  |
	|  R2  |
	|  R1  |
	|  R0  | <- SP after entering interrupt (orig. SP + 32 bytes)
	+------+

	Registers saved by the software (PendSV_Handler):
	+------+
	|  R7  |
	|  R6  |
	|  R5  |
	|  R4  |
	|  R11 |
	|  R10 |
	|  R9  |
	|  R8  | <- Saved SP (orig. SP + 64 bytes)
	+------+
	*/
	
		/* Save registers R4-R11 (32 bytes) onto current PSP (process stack
	   pointer) and make the PSP point to the last stacked register (R8):
	   - The MRS/MSR instruction is for loading/saving a special registers.
	   - The STMIA inscruction can only save low registers (R0-R7), it is
	     therefore necesary to copy registers R8-R11 into R4-R7 and call
	     STMIA twice. */
	mrs	r0, psp
	subs	r0, #16
	stmia	r0!,{r4-r7}
	mov	r4, r8
	mov	r5, r9
	mov	r6, r10
	mov	r7, r11
	subs	r0, #32
	stmia	r0!,{r4-r7}
	subs	r0, #16

	/* Save current task's SP: */
	ldr	r2, =os_curr_task
	ldr	r1, [r2]
	str	r0, [r1]

	/* Load next task's SP: */
	ldr	r2, =os_next_task
	ldr	r1, [r2]
	ldr	r0, [r1]

	/* Load registers R4-R11 (32 bytes) from the new PSP and make the PSP
	   point to the end of the exception stack frame. The NVIC hardware
	   will restore remaining registers after returning from exception): */
	ldmia	r0!,{r4-r7}
	mov	r8, r4
	mov	r9, r5
	mov	r10, r6
	mov	r11, r7
	ldmia	r0!,{r4-r7}
	msr	psp, r0

	/* EXC_RETURN - Thread mode with PSP: */
	ldr r0, =0xFFFFFFFD

	/* Enable interrupts: */
	cpsie	i

	bx	r0
	
}


static void delay(volatile uint32_t time)
{
	while (time > 0)
		time--;
}
static void task1_handler(void)
{
	while (1) {
		__disable_irq();
		//LED_GPIOx->ODR ^= (1U << LED_GPIO_PIN);
		__enable_irq();

		delay(100000);
	}
}

static void task2_handler(void)
{
	while (1) {
		__disable_irq();
		//LED_GPIOx->ODR ^= (1U << LED_GPIO_PIN);
		__enable_irq();

		delay(50000);
	}
}

static void task3_handler(void)
{
	while (1) {
		__disable_irq();

		__enable_irq();

		 delay(10000);
	}
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);


}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
	    /* Unlock protected registers */
    SYS_UnlockReg();
	
	    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();
	
		/* Initialize task stacks: */
	static os_stack_t stack1[128];
	static os_stack_t stack2[128];
	static os_stack_t stack3[128];
	os_init();

	os_task_init(&task1_handler, stack1, 128);
	os_task_init(&task2_handler, stack2, 128);
	os_task_init(&task3_handler, stack3, 128);
	/* Context switch every second: */
	os_start();
		/* The program should never reach there: */
	while (1);
   
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
