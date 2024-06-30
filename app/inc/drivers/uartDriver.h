/**
 * @file uartDriver.h
 * @author luis (luishenrique.8804@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "stdint.h"
#include "queue.h"

typedef enum
{
    UART1,
    UART2
} uInst;

typedef struct
{
    uInst inst;
    uint8_t rxData;
    Queue rx;
    Queue tx;
} uartDriver_t;

void udInit(uartDriver_t *self, uInst instance);
uint8_t udTx(uartDriver_t *self);
uint8_t udRx(uartDriver_t *self, uint8_t* data);
