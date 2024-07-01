#include "MT.h"

mt6816 MT6816_1;

void mt6816Init(SPI_HandleTypeDef* spi1)
{
    HAL_SPI_Init(spi1);


}

float Rec_Data(SPI_HandleTypeDef* spi1)
{
    MT6816_1.data_t[0]=0x80|0x03<8;
    MT6816_1.data_t[1]=0x80|0x04<8;
    MT6816_1.data_t[2]=0x80|0x05<8;

    mt6816_SPI1_CS_L();
    HAL_SPI_TransmitReceive(spi1,(uint8_t*) &MT6816_1.data_t[0],(uint8_t*) &MT6816_1.data_r[0],1,HAL_MAX_DELAY);
    mt6816_SPI1_CS_H();
    mt6816_SPI1_CS_L();
    HAL_SPI_TransmitReceive(spi1,(uint8_t*) &MT6816_1.data_t[1],(uint8_t*) &MT6816_1.data_r[1],1,HAL_MAX_DELAY);
    mt6816_SPI1_CS_H();
    mt6816_SPI1_CS_L();
    HAL_SPI_TransmitReceive(spi1,(uint8_t*) &MT6816_1.data_t[2],(uint8_t*) &MT6816_1.data_r[2],1,HAL_MAX_DELAY);
    mt6816_SPI1_CS_H();

    MT6816_1.angle=((MT6816_1.data_r[0]&0x00FF)<<8)|(MT6816_1.data_r[1]&0x00FF);

    MT6816_1.count=0;
    for(uint8_t i=0;i<16;i++)
    {
        if(MT6816_1.angle&(0x0001<<i))
        {
            MT6816_1.count++;
        }
    }
    if(MT6816_1.count & 0x01)
    {
        MT6816_1.pc_flg= false;
    }
    else
    {
        MT6816_1.pc_flg= true;
    }

    if(MT6816_1.pc_flg)
    {
        MT6816_1.theta=(((float)MT6816_1.angle /4)/16384*360);
    }
    /*uint16_t no_msg_worning = (MT6816_1.data_r[1] >> 1) & 0x01;
    uint8_t Over_speed = (MT6816_1.data_r[2] >> 3) & 0x01;

    if(Over_speed==1)
    {
        MT6816_1.theta=380;
    }
    else
    {
        if(MT6816_1.pc_flg && no_msg_worning==0)
        {
            MT6816_1.theta=(((float)MT6816_1.angle /4)/16384*360);
        }
        else if(no_msg_worning==1 && MT6816_1.pc_flg== false)
        {
            MT6816_1.theta=37012;
        }
        else if(no_msg_worning==0 && MT6816_1.pc_flg== false)
        {
            MT6816_1.theta=37002;
        }
        else if(no_msg_worning==1 && MT6816_1.pc_flg== true)
        {
            MT6816_1.theta=37010;
        }
    }*/
    return MT6816_1.theta;
}

int32_t CycAvg(int32_t a,int32_t b,int32_t cyc)
{
    int32_t sub_data;
    int32_t ave_data;

    sub_data=a-b;
    ave_data=(a+b)/2;

    if(std::abs(sub_data)>(cyc/2))
    {
        if(ave_data>=(cyc/2)) ave_data-=(cyc/2);
        else ave_data+=(cyc/2);
    }
    return ave_data;
}

int32_t CycSub(int32_t a,int32_t b,int32_t cyc)
{
    int32_t sub_data;

    sub_data=a-b;
    if(sub_data>(cyc>>1)) sub_data-=cyc;
    if(sub_data<(-cyc>>1)) sub_data+=cyc;
    return sub_data;
}

