#include "led.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/4/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

//��ʼ��PB1Ϊ���.��ʹ��ʱ��
//LED IO��ʼ��
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOF_CLK_ENABLE(); //����GPIOFʱ��

    GPIO_Initure.Pin = GPIO_PIN_6 | GPIO_PIN_7; //PA6,7
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;    //�������
    GPIO_Initure.Pull = GPIO_PULLUP;            //����
    GPIO_Initure.Speed = GPIO_SPEED_HIGH;       //����
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); //PA6��1��Ĭ�ϳ�ʼ�������
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //PA7��1��Ĭ�ϳ�ʼ�������
}
