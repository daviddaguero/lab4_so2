/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <string.h>

/* Environment includes. */
#include "DriverLib.h"

/* Macros */
#define mainBAUD_RATE               115200      // Baud rate for serial communication
#define TIMER_LOAD_VALUE            1500        // Initial load value for the timer
#define TRUE                        1           // Boolean value representing true
#define DELAY_100_MS                100         // 100 milliseconds delay for task execution   
#define DELAY_10_MS                 10          // 10 milliseconds delay for task execution 
#define DELAY_5_SECONDS             5000        // 5 seconds delay for task execution
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
#define MAX_WINDOW_SIZE             10          // Maximum allowed filter window size
#define MIN_WINDOW_SIZE             2           // Minimum allowed filter window size
#define INPUT_BUFFER_SIZE           10          // Size of the input buffer for UART reading
#define STACK_SIZE_TEMP_SENSOR      96          // Stack size for the temperature sensor simulation task
#define STACK_SIZE_FILTER           96          // Stack size for the low-pass filter processing task
#define STACK_SIZE_GRAPH            96          // Stack size for the graph display task
#define STACK_SIZE_UART_READER      96          // Stack size for the UART reader task
#define STACK_SIZE_MONITOR_STACK    64          // Stack size for the stack monitoring task
#define BUFFER_SIZE_STATS           128         // Buffer size for task statistics formatting
#define BASE_DECIMAL                10          // Base 10 for converting numerical values to string representation
#define BUFFER_SIZE_TEMP            16          // Buffer size for temporary string storage

/* Global variables */
QueueHandle_t xSensorDataQueue;
QueueHandle_t xFilteredDataQueue;
SemaphoreHandle_t xFilterMutex;
unsigned long ulHighFrequencyTimerTicks;

TaskHandle_t xTempSensorHandle = NULL;
TaskHandle_t xFilterHandle = NULL;
TaskHandle_t xGraphHandle = NULL;
TaskHandle_t xUARTReaderHandle = NULL;

unsigned char ucDisplayBuffer[96 * 2] = {0}; // 96 columns, 2 pages (16px height)
static unsigned int seed = 12345;           // Pseudoaleatory numbers generator (LCG - Linear Congruential Generator)
volatile int filter_window_size = 5;        // Window size (last N samples)

/* Function prototypes */
void Timer0IntHandler( void );
void prvSetupTimer( void );
void vUARTSetup(void);
unsigned long ulGetHighFrequencyTimerTicks( void );
void vUARTSend(const char *string);
void formatString(char *buffer, const char *prefix, int value, const char *suffix);
void setPixel(int x, int y, int on);
int pseudo_random(void);
int stringToInt(const char *str);
void formatTaskStats(char *buffer, TaskStatus_t *task, uint32_t totalRunTime);
char *utoa(unsigned int value, char *str, int base);

/* Task prototypes */
void vSimulateTemperatureSensorTask(void *pvParameters);
void vLowPassFilterTask(void *pvParameters);
void vDisplayGraphTask(void *pvParameters);
void vUARTReaderTask(void *pvParameters);
void vMonitorStackTask(void *pvParameters);
void vTopLikeTask(void *pvParameters);

/**
 * @brief Handles stack overflow detection in FreeRTOS tasks.
 *
 * This function is called automatically when FreeRTOS detects a stack overflow 
 * in any task. It sends a status character via UART and enters an infinite loop 
 * to halt execution, preventing further issues caused by the overflow.
 *
 * @param xTask Handle to the task that experienced the stack overflow.
 * @param pcTaskName Pointer to the name of the task that overflowed (null-terminated string).
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    UARTCharNonBlockingPut(UART0_BASE, 'S');
    while (TRUE);
}

void vUART_ISR(void)
{}

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
    vUARTSetup();  // Configures UART and enables interrupts.

    // Queue to pass temperature values
    xSensorDataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(int));
    if (xSensorDataQueue == NULL) {
        // Handle error if the queue could not be created
        vUARTSend("Error: The queue couldn't be created.\n");
        for (;;);
    }

    // Queue to pass filtered values
    xFilteredDataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(int));
    if (xFilteredDataQueue == NULL) {
        vUARTSend("Error: Could not create queue for filtered values.\n");
        for (;;);
    }

    // Mutex for accessing the filter window size
    xFilterMutex = xSemaphoreCreateBinary();
    if (xFilterMutex == NULL) {
        vUARTSend("Error: Filter mutex couldn't be created.\n");
        for(;;);
    }

    // Initialize the filter mutex
    xSemaphoreGive(xFilterMutex);

    vUARTSend("Starting...\n");

    OSRAMInit(TRUE);  // Initializes the display with fast speed (400 kbps)
    OSRAMDisplayOn(); // Turn on the display

    xTaskCreate(vSimulateTemperatureSensorTask, "TempSensorTask", STACK_SIZE_TEMP_SENSOR, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vLowPassFilterTask, "FilterTask", STACK_SIZE_FILTER, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vDisplayGraphTask, "GraphTask", STACK_SIZE_GRAPH, NULL, tskIDLE_PRIORITY + 3, NULL);
    BaseType_t result = xTaskCreate(vUARTReaderTask, "UARTReader", STACK_SIZE_UART_READER, NULL, tskIDLE_PRIORITY + 4, NULL);
    if (result != pdPASS) {
        vUARTSend("❌ UARTReaderTask couldn't be created.\n");
    }
    xTaskCreate(vMonitorStackTask, "MonitorStack", STACK_SIZE_MONITOR_STACK, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vTopLikeTask, "TopTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

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

        if (xQueueSend(xSensorDataQueue, &temperature, portMAX_DELAY) != pdPASS) {
            vUARTSend("Error: temperature read couldn't be sent through the queue\n");
        }

        // Delay task to achieve a frequency of 10 Hz (100 ms)
        vTaskDelay(pdMS_TO_TICKS(DELAY_100_MS));
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

    int current_size;
    if (xSemaphoreTake(xFilterMutex, portMAX_DELAY)) {
        current_size = filter_window_size;
        xSemaphoreGive(xFilterMutex);
    }
    // int *window = pvPortMalloc(current_size * sizeof(int));
    int *window = pvPortMalloc(MAX_WINDOW_SIZE * sizeof(int));

    int index = 0; // Current window index
    int sum = 0; // Cumulative sum of the window
    int count = 0; // Number of values ​​processed (for initialization)

    while (TRUE)
    {
        int new_size;

        // Get current filter size
        if (xSemaphoreTake(xFilterMutex, portMAX_DELAY)) {
            new_size = filter_window_size;
            xSemaphoreGive(xFilterMutex);
        }

        // If the size changed, we reallocate
        if (new_size != current_size) {
            current_size = new_size;
            index = 0;
            sum = 0;
            count = 0;
            memset(window, 0, MAX_WINDOW_SIZE * sizeof(int));
            
            vUARTSend("\nFilter set to new N.\n");
        }

        int temperature;
        char buffer[BUFFER_SIZE];

        if (xQueueReceive(xSensorDataQueue, &temperature, portMAX_DELAY) == pdPASS) {
            // Adjust the current cumulative sum by removing the oldest value and adding the new one
            sum -= window[index];
            window[index] = temperature;
            
            sum += temperature;

            // Advance the index of the circular window
            index = (index + 1) % current_size;

            // Calculate the average (consider the number of initial values)
            if (count < current_size) {
                count++;
            }
            int filteredValue = sum / count;

            if (xQueueSend(xFilteredDataQueue, &filteredValue, portMAX_DELAY) != pdPASS) {
                vUARTSend("\nError: The filtered value could not be sent to the queue.\n");
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
        vTaskDelay(pdMS_TO_TICKS(DELAY_100_MS));
    }
}

/**
 * @brief Task for reading and processing UART input.
 *
 * This FreeRTOS task continuously monitors the UART interface, receiving 
 * and processing incoming characters. It supports numeric input for configuring 
 * a filter window size, provides user feedback via UART, and handles invalid input gracefully.
 *
 * @param pvParameters Pointer to task-specific parameters (unused in this implementation).
 */
void vUARTReaderTask(void *pvParameters) {
    (void)pvParameters;

    char c;
    char inputBuffer[INPUT_BUFFER_SIZE];
    int inputIndex = 0;

    for (;;) {
        if (UARTCharsAvail(UART0_BASE)) {
            c = UARTCharGet(UART0_BASE);  // Reads an available character (blocks if none, but we already checked with UARTCharsAvail)
            UARTCharPut(UART0_BASE, c);   // Direct echo to UART

            if (c >= '0' && c <= '9') {
                if (inputIndex < sizeof(inputBuffer) - 1) {
                    inputBuffer[inputIndex++] = c;
                } else {
                    inputIndex = 0;
                    vUARTSend("\n❗ Very long entry. Try again.\r\n");
                }
            } else if (c == '\r' || c == '\n') {
                vUARTSend("\r\n");
                inputBuffer[inputIndex] = '\0';

                if (inputIndex > 0) {
                    int newN = stringToInt(inputBuffer);
                    if (newN >= MIN_WINDOW_SIZE && newN <= MAX_WINDOW_SIZE) {
                        filter_window_size = newN;
                        vUARTSend("\n✅ Filter now N = ");
                        vUARTSend(inputBuffer);
                        vUARTSend("\r\n");
                    } else {
                        vUARTSend("\n❗ Invalid N (2-10).\r\n");
                    }
                } else {
                    vUARTSend("\n⚠️ Empty buffer.\r\n");
                }
                inputIndex = 0;
            } else {
                inputIndex = 0;
                vUARTSend("\n❗ Non numeric character.\r\n");
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(DELAY_10_MS));  // Avoid saturating the CPU if there is no data
        }
    }
}

/**
 * @brief Monitors the stack usage of various FreeRTOS tasks.
 *
 * This FreeRTOS task periodically retrieves and reports the stack high water mark 
 * for multiple tasks, providing insights into stack consumption and potential overflow risks.
 *
 * @param pvParameters Pointer to task-specific parameters (unused in this implementation).
 */
void vMonitorStackTask(void *pvParameters)
{
    (void)pvParameters;
    char buffer[BUFFER_SIZE];

    while (TRUE)
    {
        UBaseType_t stackTemp = uxTaskGetStackHighWaterMark(xTempSensorHandle);
        UBaseType_t stackFilter = uxTaskGetStackHighWaterMark(xFilterHandle);
        UBaseType_t stackGraph = uxTaskGetStackHighWaterMark(xGraphHandle);
        UBaseType_t stackUART = uxTaskGetStackHighWaterMark(xUARTReaderHandle);

        vUARTSend("\n📊 Stack High Water Marks:\n");

        formatString(buffer, "TempSensor HWM: ", stackTemp, "\n");
        vUARTSend(buffer);

        formatString(buffer, "FilterTask HWM: ", stackFilter, "\n");
        vUARTSend(buffer);

        formatString(buffer, "GraphTask HWM: ", stackGraph, "\n");
        vUARTSend(buffer);

        formatString(buffer, "UARTReader HWM: ", stackUART, "\n");
        vUARTSend(buffer);

        vTaskDelay(pdMS_TO_TICKS(DELAY_5_SECONDS)); // every 5 seconds
    }
}

/**
 * @brief Monitors system tasks, tracks free heap space, and logs task statistics.
 *
 * This task periodically retrieves system task statuses, monitors heap usage,
 * and reports statistics via UART. If the number of tasks increases, it dynamically
 * resizes the task status array.
 *
 * @param pvParameters Unused parameter, maintained for FreeRTOS compliance.
 */
void vTopLikeTask(void *pvParameters)
{
    TaskStatus_t *pxTaskStatusArray;
    static UBaseType_t uxMaxTasks = 0;

    UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime;
    char buffer[BUFFER_SIZE_STATS];  

    while (TRUE)
    {
        uxArraySize = uxTaskGetNumberOfTasks();

        utoa(xPortGetFreeHeapSize(), buffer, BASE_DECIMAL);
        vUARTSend("\n📉 Free Heap: ");
        vUARTSend(buffer);
        vUARTSend("\n");

        if (uxArraySize > uxMaxTasks)
        {
            // Resize only if there are more tasks than before
            if (pxTaskStatusArray != NULL)
                vPortFree(pxTaskStatusArray);

            pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

            if (pxTaskStatusArray != NULL)
                uxMaxTasks = uxArraySize;
            else
            {
                vUARTSend("❌ Could not allocate memory for pxTaskStatusArray\n");
                vTaskDelay(pdMS_TO_TICKS(DELAY_5_SECONDS));
                continue;
            }
        }

        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        vUARTSend("\n🔍 Task Stats:\n");

        for (x = 0; x < uxArraySize; x++)
        {
            formatTaskStats(buffer, &pxTaskStatusArray[x], ulTotalRunTime);
            vUARTSend(buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(DELAY_5_SECONDS));
    }
}

/* ------------------------------------- Functions --------------------------------------------- */

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
    IntMasterEnable();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSet(UART0_BASE, mainBAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTIntDisable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntClear(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTEnable(UART0_BASE);
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
void vUARTSend(const char *string) {
    while (*string) {
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

/**
 * @brief Converts a numeric string to an integer.
 *
 * This function parses a null-terminated string containing numeric characters ('0' to '9') 
 * and converts it into an integer value. If the string contains any non-numeric characters, 
 * the function returns an error code (-1).
 *
 * @param str Pointer to the null-terminated string representing a numeric value.
 * @return The converted integer value if the input is valid, or -1 if the string contains non-numeric characters.
 */
int stringToInt(const char *str) {
    int value = 0;
    while (*str) {
        if (*str < '0' || *str > '9') {
            return -1; // Error: non-valid character
        }
        value = value * 10 + (*str - '0');
        str++;
    }
    return value;
}

/**
 * @brief Formats and stores task statistics in a buffer.
 *
 * This function generates a formatted string containing details about a FreeRTOS task,
 * including its name, CPU usage percentage, available stack space, and current state.
 * The formatted output is stored in the provided buffer for further transmission or logging.
 *
 * @param buffer Pointer to a character array where the formatted statistics will be stored.
 * @param task Pointer to the TaskStatus_t structure containing information about the task.
 * @param totalRunTime Total runtime of all tasks, used to compute CPU usage percentage.
 */
void formatTaskStats(char *buffer, TaskStatus_t *task, uint32_t totalRunTime) {
    char temp[BUFFER_SIZE_TEMP];
    uint32_t cpu = 0;

    buffer[0] = '\0';  // Clear buffer

    strcat(buffer, "📌 Name: ");
    strcat(buffer, task->pcTaskName);

    strcat(buffer, " | CPU: ");

    if (totalRunTime > 0) {
        cpu = (task->ulRunTimeCounter * 100UL) / totalRunTime;
    }

    utoa(cpu, temp, BASE_DECIMAL);  // Convert number to string
    strcat(buffer, temp);

    strcat(buffer, "% | Stack Free: ");
    utoa(task->usStackHighWaterMark, temp, BASE_DECIMAL);
    strcat(buffer, temp);

    strcat(buffer, " | State: ");
    utoa(task->eCurrentState, temp, BASE_DECIMAL);
    strcat(buffer, temp);

    strcat(buffer, "\n");
}

/**
 * @brief Converts an unsigned integer to a string representation in the specified base.
 *
 * This function converts a given unsigned integer into a null-terminated string
 * using the specified numerical base (between 2 and 16). The conversion is done
 * in reverse order, and the result is stored in the provided buffer.
 *
 * @param value The unsigned integer to convert.
 * @param str Pointer to a character array where the converted string will be stored.
 * @param base Numerical base for conversion (valid range: 2 to 16).
 * @return Pointer to the resulting string buffer.
 */
char *utoa(unsigned int value, char *str, int base) {
    char *ptr = str;
    char *ptr1 = str;
    char tmp_char;
    unsigned int tmp_value;

    // Only valid bases
    if (base < 2 || base > 16) {
        // Invalid base, return empty string
        *str = '\0';
        return str;
    }

    // Convert number to string in reverse order.
    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "0123456789ABCDEF"[tmp_value % base];
    } while (value);

    // End string
    *ptr-- = '\0';

    // Invert string (because we built it backwards)
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }
    return str;
}
