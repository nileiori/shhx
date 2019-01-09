/*
* File      : gpio.h
* This file is part of RT-Thread RTOS
* COPYRIGHT (C) 2015, RT-Thread Development Team
*
* The license and distribution terms for this file may be
* found in the file LICENSE in this distribution or at
* http://www.rt-thread.org/license/LICENSE
*
* Change Logs:
* Date           Author       Notes
* 2015-01-05     Bernard      the first version
*/
#ifndef GPIO_H__
#define GPIO_H__

#include "Rtdef.h"
#include "stm32f2xx.h"

struct stm32_hw_pin_userdata
{
  int pin;
  uint32_t mode;
};

#define PIN_USERDATA_END {-1,0}

extern struct stm32_hw_pin_userdata stm32_pins[];

//pin no
enum
{
  	PIN_NO_CAN_PWR					= 37,			//can1_pwr	DO
  	PIN_NO_CAN2_EN 					= 53,			//can2_en  
  	PIN_NO_EC20_PWRON				= 58,			//ec20 pwron	DO
  	PIN_NO_EC20_DTR 				= 59,			//ec20 dtr	DO
  	PIN_NO_EC20_AP_READY 			= 60,			//ec20处理器睡眠状态检测   DI
  	PIN_NO_EC20_PWRKEY 				= 61,			//ec20 pwrkey		DO
	PIN_NO_CAN1_EN					= 67, 			//can1_en

};

int stm32_hw_pin_init(void);

/****************函数定义********************************************
//函数名称	:gpio_out_init
//功能		:初始化GPIO输出引脚
//输入		:pin:gpio管脚编号
//输出		:
//使用资源	:
//全局变量	:   
//调用函数	:
//中断资源	:  
//返回		:
//备注		:
*********************************************************************/
void gpio_out_init(rt_base_t pin);
/****************函数定义********************************************
//函数名称	:gpio_out_on
//功能		:设置GPIO输出引脚为高(即数字1)
//输入		:pin:gpio管脚编号
//输出		:
//使用资源	:
//全局变量	:   
//调用函数	:
//中断资源	:  
//返回		:
//备注		:
*********************************************************************/
void gpio_out_on(rt_base_t pin);
/****************函数定义********************************************
//函数名称	:gpio_out_off
//功能		:设置GPIO输出引脚为低(即数字0)
//输入		:pin:gpio管脚编号
//输出		:
//使用资源	:
//全局变量	:   
//调用函数	:
//中断资源	:  
//返回		:
//备注		:
*********************************************************************/
void gpio_out_off(rt_base_t pin);
/****************函数定义********************************************
//函数名称	:gpio_in_init
//功能		:初始化GPIO输入引脚
//输入		:pin:gpio管脚编号
//输出		:
//使用资源	:
//全局变量	:   
//调用函数	:
//中断资源	:  
//返回		:
//备注		:
*********************************************************************/
void gpio_in_init(rt_base_t pin);
/****************函数定义********************************************
//函数名称	:gpio_pin_read
//功能		:获取输入引脚状态
//输入		:pin:gpio管脚编号
//输出		:
//使用资源	:
//全局变量	:   
//调用函数	:
//中断资源	:  
//返回		:当前IO口状态值,0或1
//备注		:
*********************************************************************/
int gpio_pin_read(rt_base_t pin);

/****************函数定义********************************************
//函数名称	:gpio_irq_install
//功能		:gpio终端装载函数
//输入		:IoNum 输入值,参考GPIO_OUT_IN_NUM枚举
//输出		:
//使用资源	:
//全局变量	:   
//调用函数	:
//中断资源	:  
//返回		:
//备注		:
*********************************************************************/
rt_err_t gpio_irq_install(rt_base_t pin, rt_base_t pinMode, void (*hdr)(void *args),rt_uint32_t irqMode,void  *args);

#endif
