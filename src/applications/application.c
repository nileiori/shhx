/*
* File      : application.c
* This file is part of RT-Thread RTOS
* COPYRIGHT (C) 2006, RT-Thread Development Team
*
* The license and distribution terms for this file may be
* found in the file LICENSE in this distribution or at
* http://www.rt-thread.org/license/LICENSE
*
* Change Logs:
* Date           Author       Notes
* 2009-01-05     Bernard      the first version
*/

/**
* @addtogroup STM32
*/
/*@{*/

#include <board.h>
#include <rtthread.h>


#include <sfud.h>
#include <spi_flash.h>
#include <spi_flash_sfud.h>
#include <spi_master.h>
#include <spi_msd.h>

#if defined ( RT_USING_IDE_IAR )
#include "canapp.h"
#endif

#include <ec20.h>
#include <cm_backtrace.h>
#ifdef	EC20_DEBUG
/******************************************************************
**      函数名：            MCO_OUT_Cfg()
**      输入：
**      功能描述：          MCO输出频率
**      返回：              
**                          
**      备注：              
***************************************************************/
void MCO_OUT_Cfg(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_1);
}
#endif

void rt_init_thread_entry(void *parameter)
{
  
#ifdef RT_USING_COMPONENTS_INIT
  /* initialization RT-Thread Components */
  rt_components_init();
#endif  
#if defined ( RT_USING_IDE_IAR )
  extern int finsh_system_init(void);
  finsh_system_init();
  rt_can_app_init();
#endif  

  	ec20_comm_mount("uart1");
	cm_backtrace_init("shhx", "v1.0", "v1.0.1");
#ifdef	EC20_DEBUG
  MCO_OUT_Cfg();
#endif
  for(; ;)
  {
    rt_thread_delay( RT_TICK_PER_SECOND * 3 );
  }
}

int rt_application_init()
{
  rt_thread_t tid;
  
  tid = rt_thread_create("init",
                         rt_init_thread_entry, RT_NULL,
                         2048, RT_THREAD_PRIORITY_MAX / 3, 20);
  
  if (tid != RT_NULL)
    rt_thread_startup(tid);
  
  return 0;
}

/*@}*/
