#ifndef __SPI_MASTER_H__
#define __SPI_MASTER_H__

#include <stdlib.h>
#include <rtthread.h>
#include "sfud_def.h"
#include "rtdevice.h"

#ifdef __cplusplus
extern "C"{
#endif

#define	FLASH_AT26DF161
//#define	E2PROM_25LC320A

#if defined (FLASH_AT26DF161) && defined (E2PROM_25LC320A)
#error	It can not be set at the same time  
#endif

  //*************SD卡电源电压控制*********
#define  SD_POWER_ON()      GPIO_WriteBit(GPIOG,GPIO_Pin_3,Bit_RESET)
#define  SD_POWER_OFF()     GPIO_WriteBit(GPIOG,GPIO_Pin_3,Bit_SET)
  
struct stm32_spi_bus
{
    struct rt_spi_bus parent;
    SPI_TypeDef * SPI;
};

struct stm32_spi_cs
{
    GPIO_TypeDef * GPIOx;
    uint16_t GPIO_Pin;
};

sfud_err spi_read_status(const char *spi_dev_name, uint8_t *status);
sfud_err spi_write_status(const char *spi_dev_name, bool is_volatile, uint8_t status);
sfud_err spi_erase(const char *spi_dev_name, uint32_t addr, size_t size);
sfud_err spi_chip_erase(const char *spi_dev_name);
sfud_err spi_write(const char *spi_dev_name, uint32_t addr, size_t size, const uint8_t *data);
sfud_err spi_read(const char *spi_dev_name, uint32_t addr, size_t size, uint8_t *data);



#ifdef __cplusplus
}
#endif

#endif





