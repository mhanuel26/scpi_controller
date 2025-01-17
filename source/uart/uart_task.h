#ifndef UART_TASK_H_
#define UART_TASK_H_

#include "FreeRTOS.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Task parameters for UART Task. */
#define UART_TASK_PRIORITY       (2)
#define UART_TASK_STACK_SIZE     (1024 * 3)

/* application dependent UART settings */
#define UART_BUFFER_SIZE 26

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void uart_task(void *pvParameters);

void initUART();
void UART_Isr();
void UART_write(const char * data, uint32_t len);


#endif /* UART_TASK_H_ */
