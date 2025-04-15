#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Environment includes. */
#include "DriverLib.h"

/* Macros */
#define mainBAUD_RATE               19200       // Baud rate for serial communication
#define TIMER_LOAD_VALUE            1500        // Initial load value for the timer
#define TRUE                        1           // Boolean value representing true
#define FILTER_WINDOW_SIZE          5           // Window size (last N samples)
#define TASK_DELAY_MS               100         // Delay in milliseconds for task execution    
#define BUFFER_SIZE                 50          // Buffer size for string formatting
#define MIN_TEMPERATURE             15          // Minimum temperature value
#define TEMPERATURE_RANGE           20          // Range of temperature values (35 - 15)
#define QUEUE_LENGTH                10          // Length of the queue for temperature values
#define MAX_HEIGHT                  15          // Maximum height or output range
#define DISPLAY_BUFFER_OFFSET       96          // Offset used in the display buffer operations
#define PIXEL_ON                    1           // Pixel state for ON
#define Y_AXIS_HEIGHT               16          // Total height of the Y-axis
#define X_AXIS_POSITION             15          // Row position for the X-axis (bottom row)
#define IMAGE_X_START               0           // Starting X-coordinate for image drawing
#define IMAGE_Y_START               0           // Starting Y-coordinate for image drawing
#define IMAGE_HEIGHT_PAGES          2           // Height of the image in display pages
#define DECIMAL_BASE                10          // Base used for decimal number conversion
#define MULTIPLIER                  1103515245  // Factor used in the linear congruential generator
#define INCREMENT                   12345       // Increment added in the generator formula
#define SHIFT_BITS                  16          // Number of bits to shift for extracting the result
#define RESULT_MASK                 0x7FFF      // Mask to obtain the 15 least significant bits of the result

/* Global variables */
QueueHandle_t xSensorDataQueue;
QueueHandle_t xFilteredDataQueue;
unsigned long ulHighFrequencyTimerTicks;

unsigned char ucDisplayBuffer[96 * 2] = {0}; // 96 columns, 2 pages (16px height)
static unsigned int seed = 12345;           // Pseudoaleatory numbers generator (LCG - Linear Congruential Generator)

/* Function prototypes */
void Timer0IntHandler( void );
void prvSetupTimer( void );
void vUARTSetup(void);
unsigned long ulGetHighFrequencyTimerTicks( void );
void vUARTSend(const char *string);
void formatString(char *buffer, const char *prefix, int value, const char *suffix);
void setPixel(int x, int y, int on);
int pseudo_random(void);

/* Task prototypes */
void vSimulateTemperatureSensorTask(void *pvParameters);
void vLowPassFilterTask(void *pvParameters);
void vDisplayGraphTask(void *pvParameters);

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

    // Queue to pass temperature values
    xSensorDataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(int));
    if (xSensorDataQueue == NULL) {
        // Handle error if the queue could not be created
        vUARTSend("Error: The queue couldn't be created.\n");
        for (;;);
    }

    xFilteredDataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(int));
    if (xFilteredDataQueue == NULL) {
        vUARTSend("Error: Could not create queue for filtered values.\n");
        for (;;);
    }

    OSRAMInit(TRUE);  // Initializes the display with fast speed (400 kbps)
    OSRAMDisplayOn(); // Turn on the display

    xTaskCreate(vSimulateTemperatureSensorTask, "TempSensorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vLowPassFilterTask, "FilterTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vDisplayGraphTask, "GraphTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    // Should never reach this point
    for(;;);

    return 0;
}

/* ------------------------------------- Tasks --------------------------------------------- */

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

    while (TRUE)
    {
        int temperature = MIN_TEMPERATURE + (pseudo_random() % (TEMPERATURE_RANGE + 1));  // Between 15 and 35 degrees Celsius

        char buffer[BUFFER_SIZE];
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
        char buffer[BUFFER_SIZE];

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

            formatString(buffer, "Filtered value: ", filteredValue, " °C\n");

            vUARTSend(buffer);

            if (xQueueSend(xFilteredDataQueue, &filteredValue, portMAX_DELAY) != pdPASS) {
                vUARTSend("Error: The filtered value could not be sent to the queue.\n");
            }
        }
    }
}

/**
 * @brief Displays a real-time temperature graph on the screen.
 *
 * This task receives filtered temperature values from a queue, scales the values 
 * to fit within the graphical display's range, and updates a visual graph. The graph 
 * includes a Y-axis for temperature and an X-axis for time, with new values being 
 * plotted at regular intervals. Numeric temperature values are also displayed above 
 * the graph.
 *
 * @param pvParameters Pointer to task-specific parameters (not used in this implementation).
 *                     Included to satisfy the task function prototype requirements.
 *
 * @details
 * - Scales temperature values from a range of 15-35°C to a displayable height (0-15 pixels).
 * - Implements a scrolling graph by shifting existing graphical data to the left.
 * - Draws Y and X axes to enhance visual reference.
 * - Updates the graphical buffer and displays it on the screen.
 * - Displays the latest numeric temperature value as text.
 */
void vDisplayGraphTask(void *pvParameters)
{
    (void)pvParameters;
    int graphIndex = 0;
    char buffer[BUFFER_SIZE];

    while (TRUE)
    {
        int value;
        if (xQueueReceive(xFilteredDataQueue, &value, portMAX_DELAY) == pdPASS)
        {
            // We scale the value from 15-35°C to a range of 0-15 (display height)
            int y = (value - MIN_TEMPERATURE) * MAX_HEIGHT / TEMPERATURE_RANGE;
            if (y < 0) y = 0;
            if (y > MAX_HEIGHT) y = MAX_HEIGHT;

            // Shift buffer to the left
            for (int i = 0; i < DISPLAY_BUFFER_OFFSET - 1; i++) {
                ucDisplayBuffer[i] = ucDisplayBuffer[i + 1];
                ucDisplayBuffer[i + DISPLAY_BUFFER_OFFSET] = ucDisplayBuffer[i + 1 + DISPLAY_BUFFER_OFFSET];
            }

            // Clear last column
            ucDisplayBuffer[DISPLAY_BUFFER_OFFSET - 1] = 0;
            ucDisplayBuffer[(DISPLAY_BUFFER_OFFSET - 1) + DISPLAY_BUFFER_OFFSET] = 0;

            // Set the new value in the last column
            setPixel(DISPLAY_BUFFER_OFFSET - 1, MAX_HEIGHT - y, PIXEL_ON); // Inverted because the Y axis starts at the top.

            // Draw Y axis (column 0)
            for (int i = 0; i < Y_AXIS_HEIGHT; i++) {
                setPixel(0, i, PIXEL_ON);
            }

            // Draw X axis (bottom row)
            for (int i = 0; i < DISPLAY_BUFFER_OFFSET; i++) {
                setPixel(i, X_AXIS_POSITION, PIXEL_ON);
            }

            OSRAMClear();
            OSRAMImageDraw(ucDisplayBuffer, IMAGE_X_START, IMAGE_Y_START, DISPLAY_BUFFER_OFFSET,IMAGE_HEIGHT_PAGES);

            // Show the numeric value above the graph
            formatString(buffer, "T: ", value, "C");
            OSRAMStringDraw(buffer, IMAGE_X_START, IMAGE_Y_START);
        }
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
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

/**
 * @brief Formats a string by combining a prefix, a numeric value, and a suffix.
 *
 * This function generates a formatted string by appending a prefix, converting
 * a numeric value to its string representation, and adding a suffix. The result
 * is stored in the provided buffer.
 *
 * @param buffer Pointer to the character array where the formatted string will be stored.
 *               The caller must ensure the buffer is large enough to hold the resulting string.
 * @param prefix Pointer to the null-terminated string to be appended at the beginning.
 * @param value Integer value to be converted to its string representation and appended.
 * @param suffix Pointer to the null-terminated string to be appended at the end.
 *
 * @details
 * - The `value` parameter is converted to a string using base-10 representation.
 * - The function ensures null termination of the resulting string in the buffer.
 * - It uses an intermediate buffer for the numeric conversion before merging into the main buffer.
 *
 * @note
 * - Ensure `buffer` is large enough to hold the concatenated strings and the converted number.
 */
void formatString(char *buffer, const char *prefix, int value, const char *suffix)
{
    char *ptr = buffer;

    // Add the prefix
    while (*prefix) {
        *ptr++ = *prefix++;
    }

    // Convert the numeric value to string
    char tempBuffer[10];
    char *tempPtr = tempBuffer + sizeof(tempBuffer) - 1;
    *tempPtr = '\0';

    int temp = value;
    do {
        *--tempPtr = (temp % DECIMAL_BASE) + '0'; // Convert digit to char
        temp /= DECIMAL_BASE;
    } while (temp > 0);

    // Cppy the converted number to the main buffer
    while (*tempPtr) {
        *ptr++ = *tempPtr++;
    }

    // Add sufix
    while (*suffix) {
        *ptr++ = *suffix++;
    }

    *ptr = '\0'; // Finish the string
}

/**
 * @brief Sets or clears a pixel in the display buffer.
 *
 * This function modifies the state of a specific pixel in the display buffer
 * based on its X and Y coordinates and a flag indicating whether the pixel 
 * should be turned on or off. The display buffer is organized in pages, with 
 * each page representing 8 vertical pixels.
 *
 * @param x The X-coordinate of the pixel (horizontal position).
 *          Valid range: 0 to DISPLAY_BUFFER_OFFSET - 1.
 * @param y The Y-coordinate of the pixel (vertical position).
 *          Valid range: 0 to Y_AXIS_HEIGHT - 1.
 * @param on Flag indicating whether the pixel should be turned on or off.
 *           Use PIXEL_ON to turn the pixel on, and PIXEL_OFF to clear it.
 *
 * @details
 * - The function calculates the page and bit positions within the buffer based
 *   on the Y-coordinate.
 * - If the X or Y coordinates are outside the valid range, the function returns
 *   immediately without modifying the buffer.
 * - Pixels are manipulated using bitwise operations:
 *   - `|=` is used to set a specific bit to turn the pixel on.
 *   - `&=~` is used to clear a specific bit to turn the pixel off.
 */
void setPixel(int x, int y, int on) {
    if (x < 0 || x >= DISPLAY_BUFFER_OFFSET || y < 0 || y >= Y_AXIS_HEIGHT) return;
    int page = y / (Y_AXIS_HEIGHT/2);
    int bit = y % (Y_AXIS_HEIGHT/2);
    if (on)
        ucDisplayBuffer[x + (page * DISPLAY_BUFFER_OFFSET)] |= (1 << bit);
    else
        ucDisplayBuffer[x + (page * DISPLAY_BUFFER_OFFSET)] &= ~(1 << bit);
}

/**
 * @brief Generates a pseudo-random number using a Linear Congruential Generator (LCG).
 *
 * This function produces a pseudo-random number based on the LCG algorithm. 
 * The generated number is masked to return a 15-bit value for use in various applications. 
 * It updates a global seed value to ensure sequential pseudo-random number generation.
 *
 * @return A 15-bit pseudo-random integer in the range [0, 32767].
 *
 * @details
 * - The function employs the LCG formula: `seed = seed * MULTIPLIER + INCREMENT`.
 * - The updated seed is shifted right by a defined number of bits (`SHIFT_BITS`) 
 *   to discard the less significant bits.
 * - A mask (`RESULT_MASK`) is applied to restrict the result to 15 bits.
 * 
 * @note
 * - This function is deterministic, meaning the sequence of numbers generated
 *   will always be the same for a given initial seed value.
 */
int pseudo_random(void)
{
    seed = seed * MULTIPLIER + INCREMENT; // LCG formula
    return (seed >> SHIFT_BITS) & RESULT_MASK; // Returns a 15 bits number
}
