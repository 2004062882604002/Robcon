/*
 * @author BusyBox
 * @date 2024/4/27
 * @version 1.0
 */

#pragma once

#include "usart.h"
#include "can.h"
#include "tim.h"

#ifdef __cplusplus

#include <functional>
#include <memory>

#include "rdk/core/transfer/serial_port.h"
#include "rdk/core/transfer/reliable_binary_transfer.h"
#include "rdk/core/motor/dji/c6xx_controller.h"
#include "rdk/core/motor/dji/m2006_motor.h"
#include "rdk/core/motor/dji/m3508_motor.h"
#include "rdk/core/underpan/omnidirectional_motion.h"
#include "rdk/core/remote/dr16.h"
#include "rdk/core/servo/feetech/feetech_protocol.h"
#include "rdk/core/servo/feetech/feetech_SCS.h"
#include "rdk/core/servo/servo.h"

extern std::shared_ptr<SerialPort> serial_port;
extern std::shared_ptr<SerialPort> remoteSerial;
extern std::shared_ptr<SerialPort>ftSerial;
extern std::shared_ptr<ReliableBinaryTransfer> transfer;
extern std::shared_ptr<C6xxController> c6xx_controller1;
extern std::shared_ptr<C6xxController> c6xx_controller2;
extern std::shared_ptr<M3508Motor> m3508_motor1;
extern std::shared_ptr<M3508Motor> m3508_motor2;
extern std::shared_ptr<M3508Motor> m3508_motor3;
extern std::shared_ptr<M3508Motor> m3508_motor4;
extern std::shared_ptr<M3508Motor> m3508_motor5;
extern std::shared_ptr<M3508Motor> m3508_motor6;
extern std::shared_ptr<M3508Motor> m3508_motor7;
extern std::shared_ptr<M3508Motor> m3508_motor8;
extern std::shared_ptr<M3508Motor> m3508_motor9;
extern std::shared_ptr<M3508Motor> m3508_motor10;
extern std::shared_ptr<M3508Motor> m3508_motor11;
extern std::shared_ptr<M3508Motor> m3508_motor12;
extern std::shared_ptr<OmnidirectionalMotion> motion;
extern std::shared_ptr<DR16> dr16;
extern std::shared_ptr<FeetechProtocol> FEE;
extern std::shared_ptr<FeetechSCS> ft_scs1;
extern std::shared_ptr<FeetechSCS> ft_scs2;
extern std::shared_ptr<FeetechSCS> ft_scs3;
extern std::shared_ptr<FeetechSCS> ft_scs4;
extern std::shared_ptr<FeetechSCS> ft_scs5;
extern std::shared_ptr<FeetechSCS> ft_scs6;
extern std::shared_ptr<FeetechSCS> ft_scs7;
extern std::shared_ptr<FeetechSCS> ft_scs8;


#endif



#ifdef __cplusplus
extern "C" {
#endif

    void RobotInit();
    void RobotTick();

    void RobotTest();
    void RobotMain();

    void Robot_DbusMove();
    void Robot_Dbus_s11_s21();
    void Robot_Dbus_s11_s22();
    void Robot_Dbus_s11_s23();
    void Robot_Dbus_s12();
    void Robot_Dbus_s13();



    void OnHAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size);
    void OnHAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
    void OnHAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
    void OnHAL_UART_ErrorCallback(UART_HandleTypeDef* huart);
    void OnHAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
    void OnHAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
    void RecvDR16Thread();


#ifdef __cplusplus
};
#endif