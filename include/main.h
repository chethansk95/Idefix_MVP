
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include <string.h>
#include "esp_log.h"


#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_UART_PORT_NUM       (UART_NUM_1)
#define DEBUG_UART_PORT_NUM      (UART_NUM_0)
#define ECHO_UART_BAUD_RATE     (38400)    //38400
#define DEBUG_UART_BAUD_RATE    (115200)
#define ECHO_TASK_STACK_SIZE    (1000)
#define BUF_SIZE (1024)

uint8_t *crc_16;

void motor_valve_control(void);
void uartinit(void);
