/*
 * @author BusyBox
 * @date 2024/4/27
 * @version 1.0
 */

#include "RobotMain.h"

std::shared_ptr<SerialPort> serial_port;
std::shared_ptr<SerialPort> remoteSerial;
std::shared_ptr<ReliableBinaryTransfer> transfer;
std::shared_ptr<C6xxController> c6xx_controller1;
std::shared_ptr<C6xxController> c6xx_controller2;
std::shared_ptr<M3508Motor> m3508_motor1;
std::shared_ptr<M3508Motor> m3508_motor2;
std::shared_ptr<M3508Motor> m3508_motor3;
std::shared_ptr<M3508Motor> m3508_motor4;
std::shared_ptr<M3508Motor> m3508_motor5;
std::shared_ptr<M3508Motor> m3508_motor6;
std::shared_ptr<M3508Motor> m3508_motor7;
std::shared_ptr<M3508Motor> m3508_motor8;
std::shared_ptr<M3508Motor> m3508_motor9;
std::shared_ptr<M3508Motor> m3508_motor10;
std::shared_ptr<M3508Motor> m3508_motor11;
std::shared_ptr<OmnidirectionalMotion> motion;
std::shared_ptr<DR16> dr16;
uint32_t motion_timeout = 0;

uint32_t step_count = 0;
void RobotInit()
{
    serial_port = std::make_shared<SerialPort>(&huart8);
    serial_port->start();
    remoteSerial = std::make_shared<SerialPort>(&huart1);
    remoteSerial->start();
    transfer = std::make_shared<ReliableBinaryTransfer>(serial_port);

    dr16 = std::make_shared<DR16>(remoteSerial);

    c6xx_controller1 = std::make_shared<C6xxController>(&hcan1);
    c6xx_controller2 = std::make_shared<C6xxController>(&hcan2);

    m3508_motor1 = std::make_shared<M3508Motor>(c6xx_controller1, 1, DjiMotor::Mode::SPEED);
    m3508_motor2 = std::make_shared<M3508Motor>(c6xx_controller1, 2, DjiMotor::Mode::SPEED);
    m3508_motor3 = std::make_shared<M3508Motor>(c6xx_controller1, 3, DjiMotor::Mode::SPEED);
    m3508_motor4 = std::make_shared<M3508Motor>(c6xx_controller1, 4, DjiMotor::Mode::SPEED);

    m3508_motor5 = std::make_shared<M3508Motor>(c6xx_controller2, 1, DjiMotor::Mode::SPEED);
    m3508_motor6 = std::make_shared<M3508Motor>(c6xx_controller2, 2, DjiMotor::Mode::SPEED);
    m3508_motor7 = std::make_shared<M3508Motor>(c6xx_controller2, 3, DjiMotor::Mode::SPEED);
    m3508_motor8 = std::make_shared<M3508Motor>(c6xx_controller2, 4, DjiMotor::Mode::SPEED);






    //底盘
    /*m3508_motor1->set_reverse(true);
    m3508_motor2->set_reverse(true);
    m3508_motor3->set_reverse(true);
    m3508_motor4->set_reverse(true);*/
    m3508_motor1->set_target_rpm(0);
    m3508_motor2->set_target_rpm(0);
    m3508_motor3->set_target_rpm(0);
    m3508_motor4->set_target_rpm(0);

    //摩擦轮
    m3508_motor5->set_target_rpm(0);
    m3508_motor6->set_target_rpm(0);
    m3508_motor7->set_target_rpm(0);
    m3508_motor8->set_target_rpm(0);

    motion = std::make_shared<OmnidirectionalMotion>(m3508_motor1, m3508_motor2, m3508_motor3, m3508_motor4);
    motion_timeout = HAL_GetTick();

}
void RobotMain()
{
    //Robot_DbusInit();
    while(true)
    {
        HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);
        HAL_Delay(100);


        if(dr16->alive() )
        {

            while(dr16->get_s2()==1)
            {
                while(dr16->get_s1()==1)
                {
                    Robot_Dbus_s11_s21();
                }
                while(dr16->get_s1()==3)
                {
                    Robot_Dbus_s11_s22();
                }

            }
            while(dr16->get_s2()==3)
            {
                Robot_Dbus_s12();
            }
            while(dr16->get_s2()==2)
            {
                if(dr16->get_s1()==3)
                {
                    Robot_Dbus_s13_s23();
                }
                else if(dr16->get_s1()==1)
                {
                    Robot_Dbus_s13_s21();
                }
            }

        }



    }

}
void RobotTest()
{
    while (true) {
        HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);
        //m3508_motor2->set_target_rpm(dr16->get_channel_1());
        //m3508_motor2->set_target_rpm(0);
        int channel0=dr16->get_channel_1();
        float rpm1=(channel0-1024)/660*480;
        //m3508_motor2->set_target_rpm(rpm1);


        char buff[128];

        snprintf(buff, 128, "0:%d 1:%d 2:%d 3:%d\r\n", dr16->get_channel_0(), dr16->get_channel_1(),
                 dr16->get_channel_2(), dr16->get_channel_3());
        serial_port->write(reinterpret_cast<uint8_t*>(buff), strlen(buff));

        snprintf(buff, 128, "s1:%d s2:%d alive:%d\r\n", dr16->get_s1(), dr16->get_s2(), dr16->alive());
        serial_port->write(reinterpret_cast<uint8_t*>(buff), strlen(buff));
    }
}

void HandlePingRequest(MasterCmd* cmd)
{
    transfer->send_binary(reinterpret_cast<uint8_t*>(cmd), sizeof(MasterCmd));
}

void HandleMotionRequest(MasterCmd* cmd)
{
    MasterCmdMotion* motion_cmd = reinterpret_cast<MasterCmdMotion*>(cmd);
    motion->clear();
    motion->add_x_speed(motion_cmd->x_speed);
    motion->add_y_speed(motion_cmd->y_speed);
    motion->add_z_speed(motion_cmd->z_speed);
    motion->commit();
    motion_timeout = HAL_GetTick();
}



void HandleDeliveryRequest(MasterCmd* cmd)
{
    MasterCmdDelivery* delivery_cmd = reinterpret_cast<MasterCmdDelivery*>(cmd);
    m3508_motor4->set_target_rpm(delivery_cmd->speed);
    m3508_motor4->set_target_pos(delivery_cmd->pos);
}

void HandleGetMotor4InfoRequest(MasterCmd* cmd)
{
    MasterCmdGetMotor4Info* cmd2 = reinterpret_cast<MasterCmdGetMotor4Info*>(cmd);
    cmd2->pos = m3508_motor4->get_pos();
    cmd2->speed = c6xx_controller1->read_angle(4);
    transfer->send_binary(reinterpret_cast<uint8_t*>(cmd2), sizeof(MasterCmdGetMotor4Info));
}

void HandleSetLaunchingPIDRequest(MasterCmd* cmd)
{
    MasterCmdSetLaunchingPID* cmd2 = reinterpret_cast<MasterCmdSetLaunchingPID*>(cmd);
    m3508_motor5->set_speed_pid(cmd2->kp, cmd2->ki, cmd2->kd);
    m3508_motor6->set_speed_pid(cmd2->kp, cmd2->ki, cmd2->kd);
    m3508_motor10->set_speed_pid(cmd2->kp, cmd2->ki, cmd2->kd);
    m3508_motor11->set_speed_pid(cmd2->kp, cmd2->ki, cmd2->kd);
}



void RobotRecvMasterCmdThread()
{
    while (true) {
        uint8_t recv_buff[256];
        std::size_t recv_len = transfer->recv_binary(recv_buff, 256, 200);
        if (recv_len == 0) continue;
        HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);

        MasterCmd* ret_cmd = reinterpret_cast<MasterCmd*>(recv_buff);
        switch (ret_cmd->cmd_type) {
            case MasterCmdType::Ping:
                HandlePingRequest(ret_cmd);
                break;
            case MasterCmdType::Motion:
                HandleMotionRequest(ret_cmd);
                break;
            case MasterCmdType::Delivery:
                HandleDeliveryRequest(ret_cmd);
                break;
            case MasterCmdType::GetMotor4Info:
                HandleGetMotor4InfoRequest(ret_cmd);
                break;
            case MasterCmdType::SetLaunchingPID:
                HandleSetLaunchingPIDRequest(ret_cmd);
        }
    }
}

void RobotTick()
{
    c6xx_controller1->tick();
    c6xx_controller2->tick();

    motion->tick();

    m3508_motor5->tick();
    m3508_motor6->tick();
    m3508_motor7->tick();
    m3508_motor8->tick();


    //如果超过1秒没有接收到上位机控制指令，则让底盘立刻停止运动
    /*if (abs(HAL_GetTick() - motion_timeout) > 1000) {
        motion->clear();
        motion->commit();
    }*/


}


void OnHAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    serial_port->OnHAL_UARTEx_RxEventCallback(huart, size);
    remoteSerial->OnHAL_UARTEx_RxEventCallback(huart, size);
}

void OnHAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port->OnHAL_UART_TxCpltCallback(huart);
    remoteSerial->OnHAL_UART_TxCpltCallback(huart);
}

void OnHAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port->OnHAL_UART_RxCpltCallback(huart);
    remoteSerial->OnHAL_UART_RxCpltCallback(huart);
}

void OnHAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    serial_port->OnHAL_UART_ErrorCallback(huart);
    remoteSerial->OnHAL_UART_ErrorCallback(huart);
}

void OnHAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    c6xx_controller1->OnHAL_CAN_RxFifo0MsgPendingCallback(hcan);
    c6xx_controller2->OnHAL_CAN_RxFifo0MsgPendingCallback(hcan);
}

void OnHAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    c6xx_controller1->OnHAL_CAN_RxFifo1MsgPendingCallback(hcan);
    c6xx_controller2->OnHAL_CAN_RxFifo1MsgPendingCallback(hcan);
}

void RecvDR16Thread()
{
    dr16->start();
}

void Robot_DbusInit()
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);

    m3508_motor5->set_target_rpm(0);
    m3508_motor6->set_target_rpm(0);
    m3508_motor7->set_target_rpm(0);
    m3508_motor8->set_target_rpm(0);
}

/*
 * @brief 左摇杆-底盘（前后左右）
 * @details
 */
void Robot_DbusMove()
{
    if (dr16->alive())
    {
        while(dr16->get_s2()==1&&dr16->get_s1()==1 || dr16->get_s2()==1&&dr16->get_s1()==3)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);

            int channel0=dr16->get_channel_0();
            int channel3=dr16->get_channel_3();
            int channel2=dr16->get_channel_2();

            float x_speed=(float)(channel3-1024)*1.2;
            float y_speed=(float)(channel2-1024)*1.2;
            float z_speed=(float)(channel0-1024)*1.2;

            motion->clear();

            motion->add_x_speed(x_speed);
            motion->add_y_speed(y_speed);
            motion->add_x_speed(z_speed);

            motion->commit();
            motion_timeout = HAL_GetTick();

        }
        HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);

        int channel3=dr16->get_channel_3();
        int channel2=dr16->get_channel_2();

        float x_speed=(float)(channel3-1024)*1.2;
        float y_speed=(float)(channel2-1024)*1.2;

        motion->clear();

        motion->add_x_speed(x_speed);
        motion->add_y_speed(y_speed);

        motion->commit();
        motion_timeout = HAL_GetTick();
    }
    else
    {
        HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
        motion->clear();
        motion->commit();
    }



}


/*
 * @brief 上：摩擦轮加速
          下：夹爪收
          左：底盘逆时针
          右：底盘顺时针
 * @details
 */
void Robot_Dbus_s11_s21()
{

    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);

    int channel1=dr16->get_channel_1();
    float f_speed=(float)(channel1-1024)*1.2;

    motion->clear();

    m3508_motor5->set_target_rpm(f_speed);
    m3508_motor6->set_target_rpm(f_speed);
    m3508_motor7->set_target_rpm(f_speed-50);
    m3508_motor8->set_target_rpm(f_speed-50);

    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
}

/*
 * @brief 上：摩擦轮停
          下：夹爪收
          左：底盘逆时针
          右：底盘顺时针
 * @details
 */
void Robot_Dbus_s11_s22()
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);

    m3508_motor5->set_target_rpm(0);
    m3508_motor6->set_target_rpm(0);
    m3508_motor7->set_target_rpm(0);
    m3508_motor8->set_target_rpm(0);

    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
}

/*
 * @brief 上：左夹爪-收
          下：右夹爪-闭
          左：左夹爪-开
          右：右夹爪-闭
 * @details
 */
void Robot_Dbus_s12()
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
}

/*
 * @brief 上：右丝杆-上
          下：右丝杆-下
          左：右13爪-开
          右：右24爪-开
 * @details
 */
void Robot_Dbus_s13_s23()
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_SET);
}

/*
 * @brief 上：左丝杆-上
          下：左丝杆-下
          左：左13爪-开
          右：左24爪-开
 * @details
 */
void Robot_Dbus_s13_s21()
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
    if(dr16->get_channel_1()>1024)
    {
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
        HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,400);
    }
    else if(dr16->get_channel_1()<1024)
    {
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
        HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,400);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
    }
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
}

void Robot_StepMove()
{
    if(dr16->get_s2()==2&&dr16->get_s1()==1&&dr16->get_channel_1()!=1024)
    {
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,400);
        const uint32_t target_steps = dr16->get_channel_1();  //
        step_count++;
        if(step_count>target_steps)
        {
            HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
        }
    }
    else if(dr16->get_s2()==2&&dr16->get_s1()==2&&dr16->get_channel_1()!=1024)
    {
        //HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
        const uint32_t target_steps = 3000;  //
        step_count++;
        if(step_count>target_steps)
        {
            //HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
        }
    }
}
