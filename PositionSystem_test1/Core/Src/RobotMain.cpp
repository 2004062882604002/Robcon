#include "RobotMain.h"

CircularBuffer circular_buffer;
SerialPort serial_port(&huart1,&circular_buffer);
Crc8Calculator crc8_calculator(0x31);
ReliableBinaryTransfer transfer(&serial_port, &crc8_calculator);

void RobotInit()
{
    HAL_UART_Init(&huart1);
    serial_port.start();
    mt6816Init(&hspi1);
}

void RobotTest()
{
    //串口发送
    /*char msg[128]="Usart Test1!";
    serial_port.write((uint8_t*)msg,sizeof (msg));*/

    float demo=Rec_Data(&hspi1);
    char buffer[50]; // 定义一个足够大的缓冲区来存储浮点数的字符串表示
    int len = snprintf(buffer, sizeof(buffer), "Received value: %f\n",demo); // 将浮点数转换为字符串
    if (len > 0) {
        serial_port.write((uint8_t*)buffer,len);
    }
    HAL_Delay(1000);
    /*//串口发送与接收
    uint8_t message[128];
    std::size_t bytesRead=serial_port.read(message,sizeof (message),1000);
    serial_port.write(message,bytesRead,1000);*/
}

void OnHAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    serial_port.OnHAL_UARTEx_RxEventCallback(huart,size);
}
void OnHAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port.OnHAL_UART_TxCpltCallback(huart);
}
void OnHAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port.OnHAL_UART_RxCpltCallback(huart);
}
void OnHAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    serial_port.OnHAL_UART_ErrorCallback(huart);
}
