/*
 * @author BusyBox
 * @date 2024/4/27
 * @version 1.0
 */

#pragma once

#include "usart.h"
#include "can.h"

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
extern std::shared_ptr<SerialPort> serial_port;
extern std::shared_ptr<SerialPort> remoteSerial;
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
extern std::shared_ptr<OmnidirectionalMotion> motion;
extern std::shared_ptr<DR16> dr16;


enum class MasterCmdType : uint8_t
{
    Ping = 0x00, //通信测试
    Motion = 0x01,  //控制底盘移动
    Launching = 0x02, //发射
    Delivery = 0x03, //送球
    GetMotor4Info = 0x04, //获取送球机构电机位置转速
    SetLaunchingPID = 0x05, //设置发射机构PID
    GetLaunchingRPM = 0x06, //获取发射机构电机转速
};

struct MasterCmd
{
    MasterCmdType cmd_type;
};

struct MasterCmdMotion
{
    MasterCmdType cmd_type;
    int16_t x_speed;
    int16_t y_speed;
    int16_t z_speed;
};

struct MasterCmdLaunching
{
    MasterCmdType cmd_type;
    int16_t speed;
};

struct MasterCmdDelivery
{
    MasterCmdType cmd_type;
    int16_t pos;
    int16_t speed;
};

struct MasterCmdGetMotor4Info
{
    MasterCmdType cmd_type;
    double pos;
    double speed;
};

struct MasterCmdSetLaunchingPID
{
    MasterCmdType cmd_type;
    double kp;
    double ki;
    double kd;
};

struct MasterCmdGetLaunchingRPM
{
    MasterCmdType cmd_type;
    double rpm1;
    double rpm2;
    double rpm3;
    double rpm4;
};


#endif



#ifdef __cplusplus
extern "C" {
#endif

    void RobotInit();
    void RobotTick();

    void RobotTest();
    void RobotMain();

    void Robot_DbusInit();
    void Robot_DbusMove();
    void Robot_Dbus_s11_s21();
    void Robot_Dbus_s11_s22();
    void Robot_Dbus_s12();
    void Robot_Dbus_s13_s23();
    void Robot_Dbus_s13_s21();

    void RobotRecvMasterCmdThread();

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