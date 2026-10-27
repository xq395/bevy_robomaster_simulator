#include <stdint.h>
typedef struct { void *Instance; } UART_HandleTypeDef;
#define UART_FLAG_TC 0
#define __HAL_UART_GET_FLAG(h, f) 1
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n);
extern UART_HandleTypeDef huart6;
