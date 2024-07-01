#pragma once

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>

#define mt6816_SPI1_CS_L() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)
#define mt6816_SPI1_CS_H() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)
#define mt6816_SPI2_CS_L() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET)
#define mt6816_SPI2_CS_H() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET)

struct mt6816{
    uint16_t angle;
    uint16_t data_t[3];
    uint16_t data_r[3];
    bool pc_flg;
    float theta;
    uint8_t count;
};
void mt6816Init(SPI_HandleTypeDef* spi1);
float Rec_Data(SPI_HandleTypeDef* spi1);
int32_t CycAvg(int32_t a,int32_t b,int32_t cyc);
int32_t CycSub(int32_t a,int32_t b,int32_t cyc);