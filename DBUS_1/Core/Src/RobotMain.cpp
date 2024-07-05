/*
 * @author BusyBox
 * @date 2024/4/27
 * @version 1.0
 */

#include "RobotMain.h"

std::shared_ptr<SerialPort> serial_port;
std::shared_ptr<SerialPort> remoteSerial;
std::shared_ptr<SerialPort> ftSerial;
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
std::shared_ptr<M3508Motor> m3508_motor12;
std::shared_ptr<OmnidirectionalMotion> motion;
std::shared_ptr<DR16> dr16;
std::shared_ptr<FeetechProtocol> FEE;
std::shared_ptr<FeetechSCS> ft_scs1;
std::shared_ptr<FeetechSCS> ft_scs2;
std::shared_ptr<FeetechSCS> ft_scs3;
std::shared_ptr<FeetechSCS> ft_scs4;
std::shared_ptr<FeetechSCS> ft_scs5;
std::shared_ptr<FeetechSCS> ft_scs6;
std::shared_ptr<FeetechSCS> ft_scs7;
std::shared_ptr<FeetechSCS> ft_scs8;

uint32_t motion_timeout = 0;

void RobotInit()
{
    serial_port = std::make_shared<SerialPort>(&huart8);
    serial_port->start();
    remoteSerial = std::make_shared<SerialPort>(&huart1);
    remoteSerial->start();
    ftSerial=std::make_shared<SerialPort>(&huart6);
    ftSerial->start();
    transfer = std::make_shared<ReliableBinaryTransfer>(serial_port);

    dr16 = std::make_shared<DR16>(remoteSerial);

    c6xx_controller1 = std::make_shared<C6xxController>(&hcan1);
    c6xx_controller2 = std::make_shared<C6xxController>(&hcan2);

    FEE =std::make_shared<FeetechProtocol>(ftSerial);

    m3508_motor1 = std::make_shared<M3508Motor>(c6xx_controller1, 1, DjiMotor::Mode::SPEED);
    m3508_motor2 = std::make_shared<M3508Motor>(c6xx_controller1, 2, DjiMotor::Mode::SPEED);
    m3508_motor3 = std::make_shared<M3508Motor>(c6xx_controller1, 3, DjiMotor::Mode::SPEED);
    m3508_motor4 = std::make_shared<M3508Motor>(c6xx_controller1, 4, DjiMotor::Mode::SPEED);

    m3508_motor9 = std::make_shared<M3508Motor>(c6xx_controller1,5,DjiMotor::Mode::SPEED_POS);
    m3508_motor10 = std::make_shared<M3508Motor>(c6xx_controller1,6,DjiMotor::Mode::SPEED_POS);

    m3508_motor5 = std::make_shared<M3508Motor>(c6xx_controller2, 1, DjiMotor::Mode::SPEED);
    m3508_motor6 = std::make_shared<M3508Motor>(c6xx_controller2, 2, DjiMotor::Mode::SPEED);
    m3508_motor7 = std::make_shared<M3508Motor>(c6xx_controller2, 3, DjiMotor::Mode::SPEED);
    m3508_motor8 = std::make_shared<M3508Motor>(c6xx_controller2, 4, DjiMotor::Mode::SPEED);

    m3508_motor11 = std::make_shared<M3508Motor>(c6xx_controller2,5,DjiMotor::Mode::SPEED_POS);
    m3508_motor12 = std::make_shared<M3508Motor>(c6xx_controller2,6,DjiMotor::Mode::SPEED_POS);

    ft_scs1=std::make_shared<FeetechSCS>(FEE,1);
    ft_scs2=std::make_shared<FeetechSCS>(FEE,2);
    ft_scs3=std::make_shared<FeetechSCS>(FEE,3);
    ft_scs4=std::make_shared<FeetechSCS>(FEE,4);
    ft_scs5=std::make_shared<FeetechSCS>(FEE,5);
    ft_scs6=std::make_shared<FeetechSCS>(FEE,6);
    ft_scs7=std::make_shared<FeetechSCS>(FEE,7);
    ft_scs8=std::make_shared<FeetechSCS>(FEE,8);



    //底盘
    m3508_motor1->set_target_rpm(0);
    m3508_motor2->set_target_rpm(0);
    m3508_motor3->set_target_rpm(0);
    m3508_motor4->set_target_rpm(0);

    //摩擦轮
    m3508_motor5->set_target_rpm(0);
    m3508_motor6->set_target_rpm(0);
    m3508_motor7->set_target_rpm(0);
    m3508_motor8->set_target_rpm(0);

    //爪子
    //左 -280 280
    ft_scs1->write_position_speed(0,500);
    ft_scs2->write_position_speed(0,500);
    ft_scs3->write_position_speed(0,500);
    ft_scs4->write_position_speed(0,500);
    m3508_motor9->set_target_rpm(100);
    //m3508_motor9->set_target_pos(-290);
    m3508_motor10->set_target_rpm(100);
    //m3508_motor10->set_target_pos(290);
    m3508_motor9->set_target_pos(0);
    m3508_motor10->set_target_pos(0);
    //右
    ft_scs5->write_position_speed(0,500);
    ft_scs6->write_position_speed(0,500);
    ft_scs7->write_position_speed(0,500);
    ft_scs8->write_position_speed(0,500);
    m3508_motor11->set_target_rpm(100);
    //m3508_motor11->set_target_pos(-300);
    m3508_motor12->set_target_rpm(100);
    //m3508_motor12->set_target_pos(300);
    m3508_motor9->set_target_pos(0);
    m3508_motor10->set_target_pos(0);

    motion = std::make_shared<OmnidirectionalMotion>(m3508_motor1, m3508_motor2, m3508_motor3, m3508_motor4);
    motion_timeout = HAL_GetTick();

}
void RobotMain()
{
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
                if(dr16->get_s1()==2)
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

        /*HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7);
        HAL_Delay(100); //
        ft_scs1->write_position_speed(1024,500); //位置*/

        //Robot_Dbus_s12();





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

    m3508_motor9->tick();
    m3508_motor10->tick();
    m3508_motor11->tick();
    m3508_motor12->tick();
}


void OnHAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    serial_port->OnHAL_UARTEx_RxEventCallback(huart, size);
    remoteSerial->OnHAL_UARTEx_RxEventCallback(huart, size);
    ftSerial->OnHAL_UARTEx_RxEventCallback(huart, size);
}

void OnHAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port->OnHAL_UART_TxCpltCallback(huart);
    remoteSerial->OnHAL_UART_TxCpltCallback(huart);
    ftSerial->OnHAL_UART_TxCpltCallback(huart);
}

void OnHAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    serial_port->OnHAL_UART_RxCpltCallback(huart);
    remoteSerial->OnHAL_UART_RxCpltCallback(huart);
    ftSerial->OnHAL_UART_RxCpltCallback(huart);
}

void OnHAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    serial_port->OnHAL_UART_ErrorCallback(huart);
    remoteSerial->OnHAL_UART_ErrorCallback(huart);
    ftSerial->OnHAL_UART_ErrorCallback(huart);
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
          下：收集臂展开
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

    if(channel1>1024)
    {
        m3508_motor5->set_target_rpm(f_speed+50);
        m3508_motor6->set_target_rpm(f_speed+50);
        m3508_motor7->set_target_rpm(f_speed);
        m3508_motor8->set_target_rpm(f_speed);
    }
    else if(channel1==1024)
    {
        m3508_motor5->set_target_rpm(0);
        m3508_motor6->set_target_rpm(0);
        m3508_motor7->set_target_rpm(0);
        m3508_motor8->set_target_rpm(0);
    }
    else if(channel1<1024)
    {
        m3508_motor11->set_target_pos(0);
        m3508_motor12->set_target_pos(0);
        m3508_motor9->set_target_pos(0);
        m3508_motor10->set_target_pos(0);
    }


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
 * @brief 上：左夹爪-闭
          下：右夹爪-闭
          左：左夹爪-开
          右：右夹爪-开
 * @details
 */
void Robot_Dbus_s12()
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
    if(dr16->get_channel_1()>1024)
    {

    }
    else if(dr16->get_channel_1()<1024)
    {

    }
    else if(dr16->get_channel_0()>1024)
    {

    }
    else if(dr16->get_channel_0()<1024)
    {

    }
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
    if(dr16->get_channel_1()>1224)
    {
        HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,400);
    }
    else if(dr16->get_channel_1()<824)
    {
        HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,400);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim9,TIM_CHANNEL_2);
    }
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
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
    if(dr16->get_channel_1()>1224)
    {
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,400);
    }
    else if(dr16->get_channel_1()<824)
    {
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,400);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
    }
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
}

