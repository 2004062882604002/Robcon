#pragma once

#include "usart.h"
#include "MT.h"
#include "spi.h"


#ifdef __cplusplus

#include <functional>
#include "rdk/core/transfer/serial_port.h"
#include "rdk/core/transfer/circular_buffer.h"
#include "rdk/core/transfer/crc.h"
#include "rdk/core/transfer/simple_binary_transfer.h"
#include "rdk/core/transfer/reliable_binary_transfer.h"

extern CircularBuffer circular_buffer;
extern SerialPort serial_port;
extern Crc8Calculator crc8_calculator;
extern ReliableBinaryTransfer transfer;


#endif



#ifdef __cplusplus

extern "C"{
#endif
    void RobotInit();
    void RobotTest();
    void OnHAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size);
    void OnHAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
    void OnHAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
    void OnHAL_UART_ErrorCallback(UART_HandleTypeDef* huart);


#ifdef __cplusplus
};
#endif