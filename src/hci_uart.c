
/******************************************************************************/
/***        include files                                                   ***/
/******************************************************************************/

#include "hci_uart.h"

#include <driver/uart.h>
#include <driver/gpio.h>
// #include "esp_log.h"
#include "esp32-hal-log.h"

#include <string.h>
// #include <termios.h>

/******************************************************************************/
/***        macro definitions                                               ***/
/******************************************************************************/

#define TAG "hci_uart"
#define BUF_SIZE (256)
#define RD_BUF_SIZE (BUF_SIZE)

/******************************************************************************/
/***        type definitions                                                ***/
/******************************************************************************/

typedef struct hci_data
{
    uint8_t data[RD_BUF_SIZE];
    size_t size;
} hci_data_t ;

/******************************************************************************/
/***        local function prototypes                                       ***/
/******************************************************************************/

static void uart_event_task(void *pvParameters);

/******************************************************************************/
/***        exported variables                                              ***/
/******************************************************************************/

QueueHandle_t recv_queue;

/******************************************************************************/
/***        local variables                                                 ***/
/******************************************************************************/

static QueueHandle_t uart_queue;
static TaskHandle_t uart_handle;

/******************************************************************************/
/***        exported functions                                              ***/
/******************************************************************************/

void uart_init(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_1, BUF_SIZE * 5, BUF_SIZE * 5, 20, &uart_queue, 0);
    uart_param_config(UART_NUM_1, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM_1, GPIO_NUM_18, GPIO_NUM_19, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    recv_queue = xQueueCreate(10, sizeof(hci_data_t));

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, &uart_handle);
}

void uart_send(const void* src, size_t size)
{
    uart_write_bytes(UART_NUM_1, src, size);
}


BaseType_t uart_recv(uint8_t *data, size_t *size)
{
    BaseType_t res;
    hci_data_t hci_data;
    bzero(&hci_data, sizeof(hci_data_t));
    res = xQueueReceive(recv_queue, &hci_data, portMAX_DELAY);
    memcpy(data, hci_data.data, hci_data.size);
    *size = hci_data.size;
    return res;
}


void uart_deinit(void)
{
    vTaskDelete(uart_handle);
    uart_driver_delete(UART_NUM_1);
    gpio_reset_pin(GPIO_NUM_18);
    // gpio_set_level(GPIO_NUM_18, 0);
}

/******************************************************************************/
/***        local functions                                                 ***/
/******************************************************************************/

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    // size_t buffered_size = 0;
    // uint8_t recvdata[RD_BUF_SIZE] = { 0 };
    hci_data_t hci_data;

    for(;;)
    {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {
            // bzero(recvdata, RD_BUF_SIZE);
            log_i("uart[%d] event:", UART_NUM_1);
            switch(event.type)
            {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    log_i("[UART DATA]: %d", event.size);
                    bzero(&hci_data, sizeof(hci_data_t));
                    hci_data.size = event.size;
                    uart_read_bytes(UART_NUM_1, hci_data.data, event.size, portMAX_DELAY);
                    xQueueSend(recv_queue, &hci_data, 10);
                break;

                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    log_i("hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                break;

                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    log_i("ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart_queue);
                break;

                //Event of UART RX break detected
                case UART_BREAK:
                    log_i("uart rx break");
                break;

                //Event of UART parity check error
                case UART_PARITY_ERR:
                    log_i("uart parity error");
                break;

                //Event of UART frame error
                case UART_FRAME_ERR:
                    log_i("uart frame error");
                break;

                default:
                    log_i("uart event type: %d", event.type);
                break;
            }
        }
    }

    vTaskDelete(NULL);
}


/******************************************************************************/
/***        END OF FILE                                                     ***/
/******************************************************************************/