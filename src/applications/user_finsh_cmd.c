/*
 * user_finsh_cmd.c
 *
 *  Created on: 2013年12月7日
 *      Author: Armink
 */


#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stm32f2xx.h>
#include <finsh.h>
#include <sfud.h>
#include <shell.h>
#include <spi_flash.h>

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
//自定义的一些msh命令
static void reboot(uint8_t argc, char **argv) 
{
    NVIC_SystemReset();
}
FINSH_FUNCTION_EXPORT(reboot, Reboot System);
MSH_CMD_EXPORT(reboot, Reboot System);
long sysclock(void)
{
	RCC_ClocksTypeDef RCC_ClocksStatus;
  
  	RCC_GetClocksFreq(&RCC_ClocksStatus);

    rt_kprintf("SYSCLK_Frequency :%d Hz\n",RCC_ClocksStatus.SYSCLK_Frequency);
	rt_kprintf("HCLK_Frequency 	 :%d Hz\n",RCC_ClocksStatus.HCLK_Frequency);
	rt_kprintf("PCLK1_Frequency  :%d Hz\n",RCC_ClocksStatus.PCLK1_Frequency);
	rt_kprintf("PCLK2_Frequency  :%d Hz\n",RCC_ClocksStatus.PCLK2_Frequency);

	return 0;
}
FINSH_FUNCTION_EXPORT(sysclock, show sysclock freq);
MSH_CMD_EXPORT(sysclock, show sysclock freq);
static void canst(uint8_t argc, char **argv)
{
	rt_device_t candev;
	struct rt_can_msg msg;
	if (argc < 2) 
	{
		rt_kprintf("parameter error,Please check it carefully :\n");
        return;
  	} 
	else 
	{
		rt_memset(&msg,0,sizeof(msg));
		const char *operator = argv[1];			

		if (!rt_strcmp(operator, "bxcan1")) 
		{
			candev = rt_device_find("bxcan1");			
		}
		else
		{
			candev = rt_device_find("bxcan2");
		}
		rt_uint8_t i;
		msg.ide = atoi(argv[2]);				
		rt_uint8_t size = atoi(argv[3]);
        rt_uint8_t *data = rt_malloc(8);				
		msg.len = size>8?8:size;
		if (data) 
		{
	        for (i = 0; i < msg.len; i++) 
			{
				data[i] = atoi(argv[4 + i]);
	        }
        }	
		rt_memcpy(msg.data,data,msg.len);
        rt_device_write(candev, 0, &msg, sizeof(msg));

		rt_free(data);
	}	
}
MSH_CMD_EXPORT(canst, can selftest);

#endif



