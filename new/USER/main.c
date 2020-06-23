#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
/************************************************
 ALIENTEK 探索者STM32F407开发板实验0-1
 Template工程模板-新建工程章节使用-HAL库版本
 技术支持：www.openedv.com
 淘宝店铺： http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
int main(void)
{
    HAL_Init();                     //初始化HAL库
    Stm32_Clock_Init(336, 8, 2, 7); //设置时钟,168Mhz
    delay_init(168);                //初始化延时函数
    LED_Init();                     //初始化LED

    while (1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //LED0对应引脚PA6拉低，亮，等同于LED0(0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   //LED1对应引脚PA7拉高，灭，等同于LED1(1)
        delay_ms(500);                                        //延时500ms
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   //LED0对应引脚PA6拉高，灭，等同于LED0(1)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //LED1对应引脚PA7拉低，亮，等同于LED1(0)
        delay_ms(500);                                        //延时500ms
    }
}
