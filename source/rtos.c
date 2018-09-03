/**
 * @file rtos.c
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 *
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */

#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"

#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#endif
/**********************************************************************************/
// Module defines
/**********************************************************************************/

#define FORCE_INLINE 	__attribute__((always_inline)) inline

#define STACK_FRAME_SIZE			8
#define STACK_LR_OFFSET				2
#define STACK_PSR_OFFSET			1
#define STACK_PSR_DEFAULT			0x01000000

/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define CAT_STRING(x,y)  		x##y
#define alive_GPIO(x)			CAT_STRING(GPIO,x)
#define alive_PORT(x)			CAT_STRING(PORT,x)
#define alive_CLOCK(x)			CAT_STRING(kCLOCK_Port,x)
static void init_is_alive(void);
static void refresh_is_alive(void);
#endif

/**********************************************************************************/
// Type definitions
/**********************************************************************************/

typedef enum
{
	S_READY = 0, S_RUNNING, S_WAITING, S_SUSPENDED
} task_state_e;
typedef enum
{
	kFromISR = 0, kFromNormalExec
} task_switch_type_e;

typedef struct
{
	uint8_t priority;
	task_state_e state;
	uint32_t *sp;
	void (*task_body)();
	rtos_tick_t local_tick;
	uint32_t reserved[10];
	uint32_t stack[RTOS_STACK_SIZE];
} rtos_tcb_t;

/**********************************************************************************/
// Global (static) task list
/**********************************************************************************/

struct
{
	uint8_t nTasks;
	rtos_task_handle_t current_task;
	rtos_task_handle_t next_task;
	rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS + 1];
	rtos_tick_t global_tick;
} task_list =
{ 0 };

/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void reload_systick(void);
static void dispatcher(task_switch_type_e type);
static void activate_waiting_tasks();
FORCE_INLINE static void context_switch(task_switch_type_e type);
static void idle_task(void);

/**********************************************************************************/
// API implementation
/**********************************************************************************/

void rtos_start_scheduler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	init_is_alive();
#endif
	task_list.global_tick = 0;
	task_list.current_task = -1;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
	        | SysTick_CTRL_ENABLE_Msk;
	reload_systick();
	for (;;)
		;
}

rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,
		rtos_autostart_e autostart)
{
	rtos_task_handle_t retval = -1; //Valor de retorno por defecto
	if(RTOS_MAX_NUMBER_OF_TASKS>task_list.nTasks) //Valida que no se vayan a crear más tareas de las que definimos en RTOS_MAX_NUMBER_OF_TASKS
	{
		if(kAutoStart==autostart)
		{
			task_list.tasks[task_list.nTasks].state = S_READY;
		}
		else
		{
			task_list.tasks[task_list.nTasks].state = S_SUSPENDED;
		}
		task_list.tasks[task_list.nTasks].sp = &(task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE]);
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE-STACK_PSR_OFFSET] = STACK_PSR_DEFAULT;
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE-STACK_PSR_OFFSET-1] = (uint32_t)task_body;
		task_list.tasks[task_list.nTasks].local_tick=0;
		task_list.tasks[task_list.nTasks].priority=priority;
		retval = task_list.nTasks;
		task_list.nTasks++;
	}
}

rtos_tick_t rtos_get_clock(void)
{
	return 0;
}

void rtos_delay(rtos_tick_t ticks)
{

}

void rtos_suspend_task(void)
{
	task_list.tasks[task_list.current_task].state = S_SUSPENDED; //Cambias el estado de la tarea actual a suspendida
	dispatcher(kFromNormalExec); //Llamas al dispatcher para que cambie de contexto
}

void rtos_activate_task(rtos_task_handle_t task)
{
	task_list.tasks[task].state = S_READY; //Ponemos la tarea en ready
	dispatcher(kFromNormalExec); //Llamas al dispatcher para que cambie de contexto
}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void reload_systick(void)
{
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
}

static void dispatcher(task_switch_type_e type)
{
	uint8_t counter; //Creamos un contador para recorrer las tareas
	int8_t max = -1; //Creamos el valor de la prioridad máxima que se tiene, de momento, en la tarea (este cambiará en el for en función de la mayor prioridad que se tenga)
	for(counter = 0; counter < task_list.nTasks; counter++) //Hacemos el ciclo para recorrer todas las tareas
	{
		//Revisamos si la tarea que se está revisando en el ciclo tenga mayor prioridad que la máxima prioridad que se tiene, y que esté en estado de lista o corriendo
		if((task_list.tasks[counter].priority > max) && (S_READY == task_list.tasks[counter].state || S_RUNNING == task_list.tasks[counter].state))
		{
			max = task_list.tasks[counter].priority; //Se guarda el nuevo valor máximo de prioridad
			task_list.next_task = counter; //Se pasa a la siguiente tarea
		}
	}
	if(task_list.next_task!=task_list.current_task) //Verifica si hay que hacer un cambio de contexto
	{
		context_switch(type); //Cambiamos de contexto
	}
}

FORCE_INLINE static void context_switch(task_switch_type_e type)
{
	register uint32_t sp asm("sp"); //Asignas el valor del Stack Pointer a la variable "sp", con "ensamblador"
	static uint8_t first = 1;
	if(!first) //Verifica si es la primera vez que entramos, ya que si es la primera vez, no necesiramos respaldar el SP de una tarea que no sea ha creado todavía
	{
		task_list.tasks[task_list.current_task].sp = (uint8_t*)sp - 9; //Guardas el valor del stack pointer en donde le corresponde a la estructura de la tarea (-9 porque el compilador mueve la dirección)
	}
	else
	{
		first = 0;
	}

	task_list.current_task = task_list.next_task; //Te pasas a la siguiente tarea
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;//Activas la bandera del PENDSV para llamar la interrupción.
}

static void activate_waiting_tasks()
{

}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void idle_task(void)
{
	for (;;)
	{

	}
}

/****************************************************/
// ISR implementation
/****************************************************/

void SysTick_Handler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	refresh_is_alive();
#endif
	task_list.global_tick++; //Se incrementa el global_tick
	dispatcher(kFromISR);
	activate_waiting_tasks();
	reload_systick();
}

void PendSV_Handler(void)
{
	register int32_t r0 asm("r0"); //Variable del registro r0
	SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk; //Limpias la bandera de la interrupción
	r0 = (uint32_t)task_list.tasks[task_list.current_task].sp; //Le asignamos al registro r0 el valor del stack que se tiene en la tarea actual
	asm("mov r7,r0"); //Como el compilador pone en r7 el valor del stack pointer, se lo tenemos que asignar de esta forma.
}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void)
{
	gpio_pin_config_t gpio_config =
	{ kGPIO_DigitalOutput, 1, };

	port_pin_config_t port_config =
	{ kPORT_PullDisable, kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	        kPORT_UnlockRegister, };
	CLOCK_EnableClock(alive_CLOCK(RTOS_IS_ALIVE_PORT));
	PORT_SetPinConfig(alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &port_config);
	GPIO_PinInit(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &gpio_config);
}

static void refresh_is_alive(void)
{
	static uint8_t state = 0;
	static uint32_t count = 0;
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
	if (RTOS_IS_ALIVE_PERIOD_IN_US / RTOS_TIC_PERIOD_IN_US - 1 == count)
	{
		GPIO_WritePinOutput(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
		        state);
		state = state == 0 ? 1 : 0;
		count = 0;
	} else //
	{
		count++;
	}
}
#endif
///
