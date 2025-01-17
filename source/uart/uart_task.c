/*
 * uart_task.c
 */

/* FreeRTOS header files */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "uart_task.h"
#include "telemetry_queue.h"

#include "hardware/uart.h"
#include "hardware/irq.h"

#include "hardware/gpio.h"

#include"scpi-def.h"

//#include "pico/stdio.h"
#include <stdio.h>
#include "pico/stdlib.h"




uint8_t rxChar;



/******************************************************************************
 * Function Name: uart_task
 ******************************************************************************
 * Summary:
 *  Task for handling initialization & connection of UART.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/

/* 
the PURE UART version of the code is compiled if the STDIO_UART lib is used.
this is default for the SDK, except when you put these commands in the CMake script:
# enable usb output, disable uart output
 pico_enable_stdio_usb(scpi_switch 1)
 pico_enable_stdio_uart(scpi_switch 0)
*/
#if LIB_PICO_STDIO_UART


#define UART_ID uart0

/* Stores the handle of the task that will be notified when the
 receive is complete. */
volatile TaskHandle_t xTaskToNotify_UART = NULL;


void UART_receive();

void uart_task(void *pvParameters) {

    /* To avoid compiler warnings */
    (void) pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART = NULL;
    scpi_instrument_init();


    // TODO semaphore

    while (true) {

        /* Start the receiving from UART. */
        UART_receive();
        /* Wait to be notified that the receive is complete.  Note
         the first parameter is pdTRUE, which has the effect of clearing
         the task's notification value back to 0, making the notification
         value act like a binary (rather than a counting) semaphore.  */
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (ulNotificationValue == 1) {
            /* Handle received data */
            while (uart_is_readable(UART_ID)) 
            {
                rxChar = uart_getc(UART_ID);
                if (rxChar == 255) break;
                scpi_instrument_input((const char *)&rxChar, 1);
                
                /* TODO this will be the character handler

                if (telemetry_queue) {
                    // queue needs to be generated}
                    if (xQueueSend(telemetry_queue,
                                   (void *)(&rxChar),
                                   (TickType_t)10) != pdPASS)
                    {
                        // Failed to post the message, even after 10 ticks.
                        // Force an assert
                        configASSERT((volatile void *)NULL);                    }
                }
                */
            }
        }
    }
}

void initUART() {
    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);
    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // TODO set USB handler if needed 
    // irq_has_shared_handler(USBCTRL_IRQ)

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, UART_Isr);
    irq_set_enabled(UART_IRQ, true);
}

// UART interrupt handler
void UART_Isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Now disable the UART to send interrupts
    uart_set_irq_enables(UART_ID, false, false);

    if (xTaskToNotify_UART != NULL) {

        /* Notify the task that the receive is complete. */
        vTaskNotifyGiveFromISR(xTaskToNotify_UART, &xHigherPriorityTaskWoken);
        /* There are no receive in progress, so no tasks to notify. */
        xTaskToNotify_UART = NULL;

        /* If xHigherPriorityTaskWoken is now set to pdTRUE then a
         context switch should be performed to ensure the interrupt
         returns directly to the highest priority task.  The macro used
         for this purpose is dependent on the port in use and may be
         called portEND_SWITCHING_ISR(). */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// UART activate a receive with interrupt. Wait for ever for UART_BUFFER_SIZE bytes
void UART_receive() {
    /* At this point xTaskToNotify should be NULL as no receive
     is in progress.  A mutex can be used to guard access to the
     peripheral if necessary. */
    configASSERT(xTaskToNotify_UART == NULL);

    /* Store the handle of the calling task. */
    xTaskToNotify_UART = xTaskGetCurrentTaskHandle();
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}

void UART_write(const char * data, uint32_t len) {
    uart_puts(UART_ID, data);
}
#endif

/* 
the TINYUSB UART version of the code is compiled if the STDIO_USB lib is used.
this is the case when you put these commands in the CMake script:
# enable usb output, disable uart output
 pico_enable_stdio_usb(scpi_switch 1)
 pico_enable_stdio_uart(scpi_switch 0)
*/

#if LIB_PICO_STDIO_USB
void uart_task(void *pvParameters) {

    /* To avoid compiler warnings */
    (void) pvParameters;
    scpi_instrument_init();


    // TODO semaphore
    bool bHasChar;
    while (true) {
        vTaskDelay((uint32_t)(50 / portTICK_PERIOD_MS)); // sleep 50 ms
        bHasChar = true;
        while (bHasChar) {
            rxChar = getchar_timeout_us(0); // don't wait for characters
            if (rxChar == 255) {
                bHasChar = false;
            } else {
                scpi_instrument_input((const char *)&rxChar, 1);
            }
        }
    }
}

void initUART() {
    // USB mode does not need additional settings. It's polling vs interrupt
    return;
}

void UART_write(const char * data, uint32_t len) {
    printf(data);
}

#endif

