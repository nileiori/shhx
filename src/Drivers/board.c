/*
* File      : board.c
* This file is part of RT-Thread RTOS
* COPYRIGHT (C) 2009 RT-Thread Develop Team
*
* The license and distribution terms for this file may be
* found in the file LICENSE in this distribution or at
* http://www.rt-thread.org/license/LICENSE
*
* Change Logs:
* Date           Author       Notes
* 2009-01-05     Bernard      first implementation
*/

#include <rthw.h>
#include <rtthread.h>
#include "usart.h"
#include "board.h"
#include "gpio.h"
#include "bxcan.h"
#if defined ( RT_USING_IDE_IAR )
#include "canapp.h"
#endif
/**
* @addtogroup STM32
*/

/*@{*/
/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(uint32_t priorGrp,uint32_t addrOffset)
{
#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, addrOffset);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, addrOffset);
  /*Set NVIC Priority Group*/
  NVIC_PriorityGroupConfig(priorGrp);
#endif
}

/*******************************************************************************
* Function Name  : SysTick_Configuration
* Description    : Configures the SysTick for OS tick.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  SysTick_Configuration(void)
{
  RCC_ClocksTypeDef  rcc_clocks;
  rt_uint32_t         cnts;
  
  RCC_GetClocksFreq(&rcc_clocks);
  
  cnts = (rt_uint32_t)rcc_clocks.HCLK_Frequency / RT_TICK_PER_SECOND;
  
  SysTick_Config(cnts);
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}

/**
* This is the timer interrupt service routine.
*
*/
void SysTick_Handler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();
  
  rt_tick_increase();
  
  /* leave interrupt */
  rt_interrupt_leave();
}
extern int rt_hw_spi_init(void);
/**
* This function will initial STM32 board.
*/
void rt_hw_board_init()
{
  //SystemInit();
  //SystemCoreClockUpdate();
  RCC_ClocksTypeDef  RCC_Clocks;
        
	//≥ı ºªØ ±÷”
	RCC_GetClocksFreq(&RCC_Clocks);
  /* NVIC Configuration */
  NVIC_Configuration(NVIC_PriorityGroup_0,0x0000);
  
  /* Configure the SysTick */
  SysTick_Config( RCC_Clocks.SYSCLK_Frequency/ RT_TICK_PER_SECOND );//1000Hz
  
  /* Configure the SysTick */
  //SysTick_Configuration();
  
  //rt_hw_usart_init();
  stm32_hw_usart_init();
  rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
  
  rt_hw_spi_init();
  
#if defined ( RT_USING_IDE_IAR )
  stm32_hw_pin_init();
  
  stm32_bxcan_init();
#endif

#ifdef RT_USING_COMPONENTS_INIT
  rt_components_board_init();
#endif

}

/*@}*/
