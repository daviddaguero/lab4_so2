#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Environment includes. */
#include "DriverLib.h"

/* Macros */
#define mainBAUD_RATE               19200    // Baud rate for serial communication
#define TIMER_LOAD_VALUE            1500     // Initial load value for the timer
#define TRUE                        1        // Boolean value representing true
#define FILTER_WINDOW_SIZE          5       // Window size (last N samples)
#define TASK_DELAY_MS               100     // Delay in milliseconds for task execution    


QueueHandle_t xSensorDataQueue;
unsigned long ulHighFrequencyTimerTicks;

/* Function prototypes */
void Timer0IntHandler( void );
void prvSetupTimer( void );
void vUARTSetup(void);
unsigned long ulGetHighFrequencyTimerTicks( void );
void vUARTSend(const char *string);
void formatString(char *buffer, const char *prefix, int value, const char *suffix);
/* Task prototypes */
void vSimulateTemperatureSensorTask(void *pvParameters);
void vLowPassFilterTask(void *pvParameters);

void vUART_ISR(void)
{
    while (TRUE)
    {
        /* code */
    }
    
}

/**
 * @brief Configures and initializes Timer0 for a 32-bit periodic timer.
 *
 * This function sets up Timer0 by enabling the required peripheral, enabling
 * interrupts, configuring the timer in 32-bit mode, loading the initial timer
 * value, registering the interrupt handler, and enabling the timer. The timer
 * is configured to trigger an interrupt upon timeout.
 *
 * @note Ensure that the `Timer0IntHandler` function is implemented to handle
 * the timer interrupts properly.
 */
void prvSetupTimer( void )
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	IntMasterEnable();
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerConfigure(TIMER0_BASE,TIMER_CFG_32_BIT_TIMER);
	TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_LOAD_VALUE);
	TimerIntRegister(TIMER0_BASE,TIMER_A, Timer0IntHandler);
	TimerEnable(TIMER0_BASE,TIMER_A);
}

/**
 * @brief Retrieves the current high-frequency timer tick count.
 *
 * This function returns the value of the `ulHighFrequencyTimerTicks` variable,
 * which represents the number of ticks counted by the high-frequency timer.
 *
 * @return The current high-frequency timer tick count as an unsigned long.
 */
unsigned long ulGetHighFrequencyTimerTicks(void)
{
	return ulHighFrequencyTimerTicks;
}

int main(void)
{
    vUARTSetup();

    // Crear la cola para pasar valores de temperatura
    xSensorDataQueue = xQueueCreate(10, sizeof(int));
    if (xSensorDataQueue == NULL) {
        // Manejo de error si no se pudo crear la cola
        vUARTSend("Error: No se pudo crear la cola.\n");
        for (;;);
    }

    xTaskCreate(vSimulateTemperatureSensorTask, "TempSensorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vLowPassFilterTask, "FilterTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();

    // Should never reach this point
    for(;;);

    return 0;
}

/**
 * @brief Simulates a temperature sensor task
 *
 * This task generates pseudo-random temperature values between 15 and 35 degrees Celsius. 
 * The task runs continuously in an infinite loop. 
 * It operates at a frequency of 10 Hz (every 100 milliseconds).
 *
 * @param pvParameters Pointer to task-specific parameters (not used in this implementation).
 *                     Included to satisfy the task function prototype requirements.
 */
void vSimulateTemperatureSensorTask(void *pvParameters)
{
    (void)pvParameters; // Avoid compiler warnings for unused parameter
    static unsigned int counter = 0; 

    while (TRUE)
    {
        // Increment counter to introduce variability
        counter++;

        // Generate a pseudo-random temperature value between 15 and 35 degrees Celsius
        int temperature = 15 + ((counter * 17) % 21);

        // Formatear la cadena utilizando la función general
        char buffer[50];
        formatString(buffer, "Temperatura: ", temperature, " °C\n");

        vUARTSend(buffer);

        if (xQueueSend(xSensorDataQueue, &temperature, portMAX_DELAY) != pdPASS) {
            vUARTSend("Error: temperature read couldn't be sent through the queue\n");
        }

        // Delay task to achieve a frequency of 10 Hz (100 ms)
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
    }
}

/**
 * @brief Implements a low-pass filter task to process temperature readings.
 *
 * This task receives temperature data from a queue and applies a low-pass filter
 * using a moving average algorithm. The
 * low-pass filter is implemented with a circular buffer to efficiently calculate
 * the moving average of the most recent temperature values.
 *
 * @param pvParameters Pointer to task-specific parameters (not used in this implementation).
 *                     Included to satisfy the task function prototype requirements.
 */
void vLowPassFilterTask(void *pvParameters)
{
    (void)pvParameters; // Avoid compiler warnings for unused parameter

    int window[FILTER_WINDOW_SIZE] = {0}; // Circular window
    int index = 0; // Current window index
    int sum = 0; // Cumulative sum of the window
    int count = 0; // Number of values ​​processed (for initialization)

    while (TRUE)
    {
        int temperature;
        char buffer[50];

        if (xQueueReceive(xSensorDataQueue, &temperature, portMAX_DELAY) == pdPASS) {
            // formatString(buffer, "Temperatura recibida: ", temperature, " °C\n");
            // vUARTSend(buffer);
            // Adjust the current cumulative sum by removing the oldest value and adding the new one
            sum -= window[index];
            // formatString(buffer, "sum: ", sum, "\n");
            // vUARTSend(buffer);
            window[index] = temperature;
            // for (size_t i = 0; i < FILTER_WINDOW_SIZE; i++)
            // {
            //     formatString(buffer, "window[index]: ", window[i], "\n");
            //     vUARTSend(buffer);
            // }
            
            sum += temperature;
            // formatString(buffer, "sum: ", sum, "\n");
            // vUARTSend(buffer);

            // Advance the index of the circular window
            index = (index + 1) % FILTER_WINDOW_SIZE;

            // Calculate the average (consider the number of initial values)
            if (count < FILTER_WINDOW_SIZE) {
                count++;
            }
            int filteredValue = sum / count;

            formatString(buffer, "Valor filtrado: ", filteredValue, " °C\n");

            vUARTSend(buffer);
        }
    }
}

/**
 * @brief Interrupt handler for Timer0 time-out event.
 *
 * This function is triggered when Timer0 reaches its time-out condition.
 * It clears the interrupt flag for Timer0 and increments the 
 * `ulHighFrequencyTimerTicks` counter, which tracks high-frequency timer ticks.
 */
void Timer0IntHandler( void )
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	ulHighFrequencyTimerTicks++;
}

/**
 * @brief Configures and initializes UART0 for communication.
 *
 * This function enables the UART0 peripheral, sets its configuration parameters
 * (baud rate, word length, stop bits, and parity), enables UART interrupts, and 
 * sets the interrupt priority. It also globally enables the UART0 interrupt to 
 * ensure proper operation.
 */
void vUARTSetup(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTConfigSet(UART0_BASE, mainBAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTIntEnable(UART0_BASE, UART_INT_RX);
    IntPrioritySet(INT_UART0, configKERNEL_INTERRUPT_PRIORITY);
    IntEnable(INT_UART0);
}

/**
 * @brief Sends a null-terminated string via UART0.
 *
 * This function transmits a string character by character using UART0. It
 * loops through each character of the provided null-terminated string and
 * sends it until the string is fully transmitted.
 *
 * @param string Pointer to the null-terminated string to be sent.
 */
void vUARTSend(const char *string)
{
    while (*string)
    {
        UARTCharPut(UART0_BASE, *string++);
    }
}

void formatString(char *buffer, const char *prefix, int value, const char *suffix)
{
    char *ptr = buffer;

    // Agregar el prefijo
    while (*prefix) {
        *ptr++ = *prefix++;
    }

    // Convertir el valor numérico a cadena
    char tempBuffer[10];
    char *tempPtr = tempBuffer + sizeof(tempBuffer) - 1;
    *tempPtr = '\0';

    int temp = value;
    do {
        *--tempPtr = (temp % 10) + '0'; // Convertir dígito a carácter
        temp /= 10;
    } while (temp > 0);

    // Copiar el número convertido al buffer principal
    while (*tempPtr) {
        *ptr++ = *tempPtr++;
    }

    // Agregar el sufijo
    while (*suffix) {
        *ptr++ = *suffix++;
    }

    *ptr = '\0'; // Terminar la cadena
}
