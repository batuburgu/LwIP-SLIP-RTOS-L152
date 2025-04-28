#include "sio.h"
#include "cmsis_os.h"
#include "string.h"

#define RXBUFFER_SIZE 2500
extern UART_HandleTypeDef huart1;
extern osSemaphoreId UARTSemHandle;
extern osMessageQId UARTDataQueueHandle;

u8_t rx_byte;
u8_t temp [RXBUFFER_SIZE];
u8_t rx_buffer [RXBUFFER_SIZE];
int read_index = 0;
int write_index = 0;

sio_fd_t sio_open(u8_t devnum) {

	return (sio_fd_t)&huart1;

}
void sio_send(u8_t* c, sio_fd_t fd, u16_t byte_count) {

	UART_HandleTypeDef *uart = (UART_HandleTypeDef *)fd;

	HAL_UART_Transmit_DMA(uart, c, byte_count);

	while (uart->gState != HAL_UART_STATE_READY)
	{
		taskYIELD();
	}
}

u8_t sio_recv(sio_fd_t fd) {

    UART_HandleTypeDef *uart = (UART_HandleTypeDef *)fd;
    u8_t received_byte;
    HAL_UART_Receive(uart, &received_byte, 1, HAL_MAX_DELAY);
    return received_byte;
}

u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len) {

    UART_HandleTypeDef *uart = (UART_HandleTypeDef *)fd;
    HAL_UART_Receive(uart, data, len, HAL_MAX_DELAY);
    return len;
}

u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len) {
    u32_t i = 0;
    taskENTER_CRITICAL();
    while (read_index != write_index && i < len) {
        data[i] = rx_buffer[read_index];
        read_index = (read_index + 1) % RXBUFFER_SIZE;
        i++;
    }
    taskEXIT_CRITICAL();

    return i;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {

        u32_t next_write_index = (write_index + 1) % RXBUFFER_SIZE;
        if (next_write_index != read_index) {
            rx_buffer[write_index] = rx_byte;
            write_index = next_write_index;
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
