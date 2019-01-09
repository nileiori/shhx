#include <board.h>
#include <rtthread.h>
#include <sfud.h>
#include <spi_flash.h>
#include <spi_flash_sfud.h>
#include <spi_master.h>



sfud_err spi_read_status(const char *spi_dev_name, uint8_t *status) {
  sfud_flash *sfud_dev = rt_sfud_flash_find(spi_dev_name);
  
  return sfud_read_status(sfud_dev, status);
}
sfud_err spi_write_status(const char *spi_dev_name, bool is_volatile, uint8_t status) {
  sfud_flash *sfud_dev = rt_sfud_flash_find(spi_dev_name);
  
  return sfud_write_status(sfud_dev, is_volatile, status);
}
sfud_err spi_erase(const char *spi_dev_name, uint32_t addr, size_t size) {
  sfud_flash *sfud_dev = rt_sfud_flash_find(spi_dev_name);
  
  return sfud_erase(sfud_dev, addr, size);
}
sfud_err spi_chip_erase(const char *spi_dev_name) {
  sfud_flash *sfud_dev = rt_sfud_flash_find(spi_dev_name);
  
  return sfud_chip_erase(sfud_dev);
}
sfud_err spi_read(const char *spi_dev_name, uint32_t addr, size_t size, uint8_t *data) {
  sfud_flash *sfud_dev = rt_sfud_flash_find(spi_dev_name);
  
  return sfud_read(sfud_dev,addr, size, data);
}
sfud_err spi_write(const char *spi_dev_name, uint32_t addr, size_t size, const uint8_t *data) {
  sfud_flash *sfud_dev = rt_sfud_flash_find(spi_dev_name);
  
  return sfud_write(sfud_dev,addr, size, data);
}



/* private rt-thread spi ops function */
static rt_err_t stm32_spi_configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration);
static rt_uint32_t stm32_spi_xfer(struct rt_spi_device* device, struct rt_spi_message* message);

static struct rt_spi_ops stm32_spi_ops =
{
  stm32_spi_configure,
  stm32_spi_xfer
};

static rt_err_t stm32_spi_configure(struct rt_spi_device* device,
                                    struct rt_spi_configuration* configuration)
{
  struct stm32_spi_bus * stm32_spi_bus = (struct stm32_spi_bus *)device->bus;
  SPI_InitTypeDef SPI_InitStructure;
  
  SPI_StructInit(&SPI_InitStructure);
  
  /* data_width */
  if(configuration->data_width <= 8)
  {
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  }
  else if(configuration->data_width <= 16)
  {
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  }
  else
  {
    return RT_EIO;
  }
  
  /* baudrate */
  {
    uint32_t SPI_APB_CLOCK;
    uint32_t stm32_spi_max_clock;
    uint32_t max_hz;
    
    stm32_spi_max_clock = 18000000;
    max_hz = configuration->max_hz;
#if defined(STM32F4XX)
    stm32_spi_max_clock = 37500000;
#elif defined(STM32F2XX)
    stm32_spi_max_clock = 30000000;
#endif
    
    if(max_hz > stm32_spi_max_clock)
    {
      max_hz = stm32_spi_max_clock;
    }
    
    if(stm32_spi_bus->SPI == SPI1)
    {
      SPI_APB_CLOCK = SystemCoreClock / 2;
    }
    else
    {
      SPI_APB_CLOCK = SystemCoreClock / 4;
    }
    /* STM32F2xx SPI MAX 30Mhz */
    /* STM32F4xx SPI MAX 37.5Mhz */
    if(max_hz >= SPI_APB_CLOCK/2 && SPI_APB_CLOCK/2 <= 30000000)
    {
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    }
    else if(max_hz >= SPI_APB_CLOCK/4)
    {
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    }
    else if(max_hz >= SPI_APB_CLOCK/8)
    {
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    }
    else if(max_hz >= SPI_APB_CLOCK/16)
    {
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    }
    else if(max_hz >= SPI_APB_CLOCK/32)
    {
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    }
    else if(max_hz >= SPI_APB_CLOCK/64)
    {
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    }
    else if(max_hz >= SPI_APB_CLOCK/128)
    {
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    }
    else
    {
      /*  min prescaler 256 */
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    }
  } /* baudrate */
  
  /* CPOL */
  if(configuration->mode & RT_SPI_CPOL)
  {
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  }
  else
  {
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  }
  /* CPHA */
  if(configuration->mode & RT_SPI_CPHA)
  {
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  }
  else
  {
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  }
  /* MSB or LSB */
  if(configuration->mode & RT_SPI_MSB)
  {
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  }
  else
  {
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
  }
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;
  //SPI_InitStructure.SPI_CRCPolynomial = 7;
  /* init SPI */
  SPI_I2S_DeInit(stm32_spi_bus->SPI);
  SPI_Init(stm32_spi_bus->SPI, &SPI_InitStructure);
  /* Enable SPI_MASTER */
  SPI_Cmd(stm32_spi_bus->SPI, ENABLE);
  SPI_CalculateCRC(stm32_spi_bus->SPI, DISABLE);
  
  uint8_t send = 0xff;
  uint8_t recv;
  struct rt_spi_message message;
  /* initial message */
  message.send_buf = &send;
  message.recv_buf = &recv;
  message.length = 1;
  message.cs_take = message.cs_release = 0;

  /* transfer message */
  device->bus->ops->xfer(device, &message);
    
  return RT_EOK;
};

static rt_uint32_t stm32_spi_xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
  struct stm32_spi_bus * stm32_spi_bus = (struct stm32_spi_bus *)device->bus;
  struct rt_spi_configuration * config = &device->config;
  SPI_TypeDef * SPI = stm32_spi_bus->SPI;
  struct stm32_spi_cs * stm32_spi_cs = device->parent.user_data;
  rt_uint32_t size = message->length;
  
  /* take CS */
  if(message->cs_take)
  {
    GPIO_ResetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
    //rt_hw_us_delay(50);
  }
  
  if(config->data_width <= 8)
  {
    const rt_uint8_t * send_ptr = message->send_buf;
    rt_uint8_t * recv_ptr = message->recv_buf;
    
    while(size--)
    {
      rt_uint8_t data = 0xFF;
      
      if(send_ptr != RT_NULL)
      {
        data = *send_ptr++;
      }
      
      //Wait until the transmit buffer is empty
      while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
      // Send the byte
      SPI_I2S_SendData(SPI, data);
      
      //Wait until a data is received
      while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
      // Get the received data
      data = SPI_I2S_ReceiveData(SPI);
      
      if(recv_ptr != RT_NULL)
      {
        *recv_ptr++ = data;
      }
    }
  }
  else if(config->data_width <= 16)
  {
    const rt_uint16_t * send_ptr = message->send_buf;
    rt_uint16_t * recv_ptr = message->recv_buf;
    
    while(size--)
    {
      rt_uint16_t data = 0xFF;
      
      if(send_ptr != RT_NULL)
      {
        data = *send_ptr++;
      }
      
      //Wait until the transmit buffer is empty
      while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
      // Send the byte
      SPI_I2S_SendData(SPI, data);
      
      //Wait until a data is received
      while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
      // Get the received data
      data = SPI_I2S_ReceiveData(SPI);
      
      if(recv_ptr != RT_NULL)
      {
        *recv_ptr++ = data;
      }
    }
  }
  
  /* release CS */
  if(message->cs_release)
  {
    GPIO_SetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
    //rt_hw_us_delay(50);
  }
  
  return message->length;
};

/** \brief init and register stm32 spi bus.
*
* \param SPI: STM32 SPI, e.g: SPI1,SPI2,SPI3.
* \param stm32_spi: stm32 spi bus struct.
* \param spi_bus_name: spi bus name, e.g: "spi1"
* \return
*
*/
rt_err_t stm32_spi_register(SPI_TypeDef * SPI,
                            struct stm32_spi_bus * stm32_spi,
                            const char * spi_bus_name)
{
  if(SPI == SPI1)
  {
    stm32_spi->SPI = SPI1;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  }
  else if(SPI == SPI2)
  {
    stm32_spi->SPI = SPI2;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  }
  else if(SPI == SPI3)
  {
    stm32_spi->SPI = SPI3;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  }
  else
  {
    return RT_ENOSYS;
  }
  
  return rt_spi_bus_register(&stm32_spi->parent, spi_bus_name, &stm32_spi_ops);
}

/*
SPI1: PA5/6/7
CS: PA4
SPI3: PB3/4/5
CS: PA15
*/
int rt_hw_spi_init(void)
{
#ifdef RT_USING_SPI1
  /* register spi bus */
  {
    static struct stm32_spi_bus stm32_spi;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    
    /*!< SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    stm32_spi_register(SPI1, &stm32_spi, "spi1");
  }
  
  /* attach cs */
  {
    static struct rt_spi_device spi_device;
    static struct stm32_spi_cs  spi_cs;
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    
#ifdef FLASH_AT26DF161
    /* spi10: PA4 */
    spi_cs.GPIOx = GPIOA;
    spi_cs.GPIO_Pin = GPIO_Pin_4;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;       
    GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);
    
    GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
#endif
    
#ifdef E2PROM_25LC320A
    /* spi10: PG4 */
    spi_cs.GPIOx = GPIOG;
    spi_cs.GPIO_Pin = GPIO_Pin_4;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;       
    GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);
    
    GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
#endif
    
    rt_spi_bus_attach_device(&spi_device, "spi10", "spi1", (void*)&spi_cs);
  }
#endif
  
#ifdef RT_USING_SPI3
  /* register spi bus */
  {
    static struct stm32_spi_bus stm32_spi;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOG, ENABLE);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; //GPIO_PuPd_NOPULL; GPIO_PuPd_DOWN;
    
    /*!< SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    
    SD_POWER_OFF();
    
    stm32_spi_register(SPI3, &stm32_spi, "spi3");
  }
  
  /* attach cs */
  {
    static struct rt_spi_device spi_device;
    static struct stm32_spi_cs  spi_cs;
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    /* spi30: PA15 */
    spi_cs.GPIOx = GPIOD;
    spi_cs.GPIO_Pin = GPIO_Pin_14;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;        
    GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);
    GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
    
    //!< Configure SD_CD pin in Input pushpull mode 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    SD_POWER_ON();
    
    rt_spi_bus_attach_device(&spi_device, "spi30", "spi3", (void*)&spi_cs);
  }
#endif
  return 0;
}
INIT_BOARD_EXPORT(rt_hw_spi_init);




