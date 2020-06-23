#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
/************************************************
 ALIENTEK ̽����STM32F407������ʵ��0-1
 Template����ģ��-�½������½�ʹ��-HAL��汾
 ����֧�֣�www.openedv.com
 �Ա����̣� http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/
int main(void)
{
    HAL_Init();                     //��ʼ��HAL��
    Stm32_Clock_Init(336, 8, 2, 7); //����ʱ��,168Mhz
    delay_init(168);                //��ʼ����ʱ����
    LED_Init();                     //��ʼ��LED

    while (1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //LED0��Ӧ����PA6���ͣ�������ͬ��LED0(0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   //LED1��Ӧ����PA7���ߣ��𣬵�ͬ��LED1(1)
        delay_ms(500);                                        //��ʱ500ms
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   //LED0��Ӧ����PA6���ߣ��𣬵�ͬ��LED0(1)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //LED1��Ӧ����PA7���ͣ�������ͬ��LED1(0)
        delay_ms(500);                                        //��ʱ500ms
    }
}
