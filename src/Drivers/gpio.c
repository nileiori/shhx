/*
* File      : gpio.c
* This file is part of RT-Thread RTOS
* COPYRIGHT (C) 2015, RT-Thread Development Team
*
* The license and distribution terms for this file may be
* found in the file LICENSE in this distribution or at
* http://www.rt-thread.org/license/LICENSE
*
* Change Logs:
* Date           Author            Notes
* 2015-03-24     Bright            the first version
* 2016-05-23     Margguo@gmail.com Add  48 pins IC define
* 2018-07-23     jiezhi320         Add GPIO Out_OD mode config
*/

#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <gpio.h>

#ifdef RT_USING_PIN

#define STM32F2XX_PIN_NUMBERS 100 //[48, 64, 100, 144 ]

#define __STM32_PIN(index, rcc, gpio, gpio_index) { 0, RCC_##rcc##Periph_GPIO##gpio, GPIO##gpio, GPIO_Pin_##gpio_index, EXTI_PortSourceGPIO##gpio, EXTI_PinSource##gpio_index}
#define __STM32_PIN_DEFAULT {-1, 0, 0, 0, 0}

/* STM32 GPIO driver */
struct pin_index
{
  int index;
  uint32_t rcc;
  GPIO_TypeDef *gpio;
  uint32_t pin;
  uint8_t port_source;
  uint8_t pin_source;
};

static const struct pin_index pins[] =
{
#if (STM32F2XX_PIN_NUMBERS == 48)
  __STM32_PIN_DEFAULT,
  __STM32_PIN_DEFAULT,
  __STM32_PIN(2, AHB1, C, 13),
  __STM32_PIN(3, AHB1, C, 14),
  __STM32_PIN(4, AHB1, C, 15),
  __STM32_PIN_DEFAULT,
  __STM32_PIN_DEFAULT,
  __STM32_PIN_DEFAULT,
  __STM32_PIN_DEFAULT,
  __STM32_PIN_DEFAULT,
  __STM32_PIN(10, AHB1, A, 0),
  __STM32_PIN(11, AHB1, A, 1),
  __STM32_PIN(12, AHB1, A, 2),
  __STM32_PIN(13, AHB1, A, 3),
  __STM32_PIN(14, AHB1, A, 4),
  __STM32_PIN(15, AHB1, A, 5),
  __STM32_PIN(16, AHB1, A, 6),
  __STM32_PIN(17, AHB1, A, 7),
  __STM32_PIN(18, AHB1, B, 0),
  __STM32_PIN(19, AHB1, B, 1),
  __STM32_PIN(20, AHB1, B, 2),
  __STM32_PIN(21, AHB1, B, 10),
  __STM32_PIN(22, AHB1, B, 11),
  __STM32_PIN_DEFAULT,
  __STM32_PIN_DEFAULT,
  __STM32_PIN(25, AHB1, B, 12),
  __STM32_PIN(26, AHB1, B, 13),
  __STM32_PIN(27, AHB1, B, 14),
  __STM32_PIN(28, AHB1, B, 15),
  __STM32_PIN(29, AHB1, A, 8),
  __STM32_PIN(30, AHB1, A, 9),
  __STM32_PIN(31, AHB1, A, 10),
  __STM32_PIN(32, AHB1, A, 11),
  __STM32_PIN(33, AHB1, A, 12),
  __STM32_PIN(34, AHB1, A, 13),
  __STM32_PIN_DEFAULT,
  __STM32_PIN_DEFAULT,
  __STM32_PIN(37, AHB1, A, 14),
  __STM32_PIN(38, AHB1, A, 15),
  __STM32_PIN(39, AHB1, B, 3),
  __STM32_PIN(40, AHB1, B, 4),
  __STM32_PIN(41, AHB1, B, 5),
  __STM32_PIN(42, AHB1, B, 6),
  __STM32_PIN(43, AHB1, B, 7),
  __STM32_PIN_DEFAULT,
  __STM32_PIN(45, AHB1, B, 8),
  __STM32_PIN(46, AHB1, B, 9),
  __STM32_PIN_DEFAULT,
  __STM32_PIN_DEFAULT,
  
#endif
#if (STM32F2XX_PIN_NUMBERS == 64)
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(2, AHB1, C, 13),
__STM32_PIN(3, AHB1, C, 14),
__STM32_PIN(4, AHB1, C, 15),
__STM32_PIN(5, AHB1, D, 0),
__STM32_PIN(6, AHB1, D, 1),
__STM32_PIN_DEFAULT,
__STM32_PIN(8, AHB1, C, 0),
__STM32_PIN(9, AHB1, C, 1),
__STM32_PIN(10, AHB1, C, 2),
__STM32_PIN(11, AHB1, C, 3),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(14, AHB1, A, 0),
__STM32_PIN(15, AHB1, A, 1),
__STM32_PIN(16, AHB1, A, 2),
__STM32_PIN(17, AHB1, A, 3),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(20, AHB1, A, 4),
__STM32_PIN(21, AHB1, A, 5),
__STM32_PIN(22, AHB1, A, 6),
__STM32_PIN(23, AHB1, A, 7),
__STM32_PIN(24, AHB1, C, 4),
__STM32_PIN(25, AHB1, C, 5),
__STM32_PIN(26, AHB1, B, 0),
__STM32_PIN(27, AHB1, B, 1),
__STM32_PIN(28, AHB1, B, 2),
__STM32_PIN(29, AHB1, B, 10),
__STM32_PIN(30, AHB1, B, 11),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(33, AHB1, B, 12),
__STM32_PIN(34, AHB1, B, 13),
__STM32_PIN(35, AHB1, B, 14),
__STM32_PIN(36, AHB1, B, 15),
__STM32_PIN(37, AHB1, C, 6),
__STM32_PIN(38, AHB1, C, 7),
__STM32_PIN(39, AHB1, C, 8),
__STM32_PIN(40, AHB1, C, 9),
__STM32_PIN(41, AHB1, A, 8),
__STM32_PIN(42, AHB1, A, 9),
__STM32_PIN(43, AHB1, A, 10),
__STM32_PIN(44, AHB1, A, 11),
__STM32_PIN(45, AHB1, A, 12),
__STM32_PIN(46, AHB1, A, 13),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(49, AHB1, A, 14),
__STM32_PIN(50, AHB1, A, 15),
__STM32_PIN(51, AHB1, C, 10),
__STM32_PIN(52, AHB1, C, 11),
__STM32_PIN(53, AHB1, C, 12),
__STM32_PIN(54, AHB1, D, 2),
__STM32_PIN(55, AHB1, B, 3),
__STM32_PIN(56, AHB1, B, 4),
__STM32_PIN(57, AHB1, B, 5),
__STM32_PIN(58, AHB1, B, 6),
__STM32_PIN(59, AHB1, B, 7),
__STM32_PIN_DEFAULT,
__STM32_PIN(61, AHB1, B, 8),
__STM32_PIN(62, AHB1, B, 9),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
#endif
#if (STM32F2XX_PIN_NUMBERS == 100)
__STM32_PIN_DEFAULT,
__STM32_PIN(1, AHB1, E, 2),
__STM32_PIN(2, AHB1, E, 3),
__STM32_PIN(3, AHB1, E, 4),
__STM32_PIN(4, AHB1, E, 5),
__STM32_PIN(5, AHB1, E, 6),
__STM32_PIN_DEFAULT,
__STM32_PIN(7, AHB1, C, 13),
__STM32_PIN(8, AHB1, C, 14),
__STM32_PIN(9, AHB1, C, 15),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(15, AHB1, C, 0),
__STM32_PIN(16, AHB1, C, 1),
__STM32_PIN(17, AHB1, C, 2),
__STM32_PIN(18, AHB1, C, 3),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(23, AHB1, A, 0),
__STM32_PIN(24, AHB1, A, 1),
__STM32_PIN(25, AHB1, A, 2),
__STM32_PIN(26, AHB1, A, 3),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(29, AHB1, A, 4),
__STM32_PIN(30, AHB1, A, 5),
__STM32_PIN(31, AHB1, A, 6),
__STM32_PIN(32, AHB1, A, 7),
__STM32_PIN(33, AHB1, C, 4),
__STM32_PIN(34, AHB1, C, 5),
__STM32_PIN(35, AHB1, B, 0),
__STM32_PIN(36, AHB1, B, 1),
__STM32_PIN(37, AHB1, B, 2),
__STM32_PIN(38, AHB1, E, 7),
__STM32_PIN(39, AHB1, E, 8),
__STM32_PIN(40, AHB1, E, 9),
__STM32_PIN(41, AHB1, E, 10),
__STM32_PIN(42, AHB1, E, 11),
__STM32_PIN(43, AHB1, E, 12),
__STM32_PIN(44, AHB1, E, 13),
__STM32_PIN(45, AHB1, E, 14),
__STM32_PIN(46, AHB1, E, 15),
__STM32_PIN(47, AHB1, B, 10),
__STM32_PIN(48, AHB1, B, 11),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(51, AHB1, B, 12),
__STM32_PIN(52, AHB1, B, 13),
__STM32_PIN(53, AHB1, B, 14),
__STM32_PIN(54, AHB1, B, 15),
__STM32_PIN(55, AHB1, D, 8),
__STM32_PIN(56, AHB1, D, 9),
__STM32_PIN(57, AHB1, D, 10),
__STM32_PIN(58, AHB1, D, 11),
__STM32_PIN(59, AHB1, D, 12),
__STM32_PIN(60, AHB1, D, 13),
__STM32_PIN(61, AHB1, D, 14),
__STM32_PIN(62, AHB1, D, 15),
__STM32_PIN(63, AHB1, C, 6),
__STM32_PIN(64, AHB1, C, 7),
__STM32_PIN(65, AHB1, C, 8),
__STM32_PIN(66, AHB1, C, 9),
__STM32_PIN(67, AHB1, A, 8),
__STM32_PIN(68, AHB1, A, 9),
__STM32_PIN(69, AHB1, A, 10),
__STM32_PIN(70, AHB1, A, 11),
__STM32_PIN(71, AHB1, A, 12),
__STM32_PIN(72, AHB1, A, 13),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(76, AHB1, A, 14),
__STM32_PIN(77, AHB1, A, 15),
__STM32_PIN(78, AHB1, C, 10),
__STM32_PIN(79, AHB1, C, 11),
__STM32_PIN(80, AHB1, C, 12),
__STM32_PIN(81, AHB1, D, 0),
__STM32_PIN(82, AHB1, D, 1),
__STM32_PIN(83, AHB1, D, 2),
__STM32_PIN(84, AHB1, D, 3),
__STM32_PIN(85, AHB1, D, 4),
__STM32_PIN(86, AHB1, D, 5),
__STM32_PIN(87, AHB1, D, 6),
__STM32_PIN(88, AHB1, D, 7),
__STM32_PIN(89, AHB1, B, 3),
__STM32_PIN(90, AHB1, B, 4),
__STM32_PIN(91, AHB1, B, 5),
__STM32_PIN(92, AHB1, B, 6),
__STM32_PIN(93, AHB1, B, 7),
__STM32_PIN_DEFAULT,
__STM32_PIN(95, AHB1, B, 8),
__STM32_PIN(96, AHB1, B, 9),
__STM32_PIN(97, AHB1, E, 0),
__STM32_PIN(98, AHB1, E, 1),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
#endif
#if (STM32F2XX_PIN_NUMBERS == 144)
__STM32_PIN_DEFAULT,
__STM32_PIN(1, AHB1, E, 2),
__STM32_PIN(2, AHB1, E, 3),
__STM32_PIN(3, AHB1, E, 4),
__STM32_PIN(4, AHB1, E, 5),
__STM32_PIN(5, AHB1, E, 6),
__STM32_PIN_DEFAULT,
__STM32_PIN(7, AHB1, C, 13),
__STM32_PIN(8, AHB1, C, 14),
__STM32_PIN(9, AHB1, C, 15),

__STM32_PIN(10, AHB1, F, 0),
__STM32_PIN(11, AHB1, F, 1),
__STM32_PIN(12, AHB1, F, 2),
__STM32_PIN(13, AHB1, F, 3),
__STM32_PIN(14, AHB1, F, 4),
__STM32_PIN(15, AHB1, F, 5),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(18, AHB1, F, 6),
__STM32_PIN(19, AHB1, F, 7),
__STM32_PIN(20, AHB1, F, 8),
__STM32_PIN(21, AHB1, F, 9),
__STM32_PIN(22, AHB1, F, 10),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(26, AHB1, C, 0),
__STM32_PIN(27, AHB1, C, 1),
__STM32_PIN(28, AHB1, C, 2),
__STM32_PIN(29, AHB1, C, 3),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(34, AHB1, A, 0),
__STM32_PIN(35, AHB1, A, 1),
__STM32_PIN(36, AHB1, A, 2),
__STM32_PIN(37, AHB1, A, 3),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(40, AHB1, A, 4),
__STM32_PIN(41, AHB1, A, 5),
__STM32_PIN(42, AHB1, A, 6),
__STM32_PIN(43, AHB1, A, 7),
__STM32_PIN(44, AHB1, C, 4),
__STM32_PIN(45, AHB1, C, 5),
__STM32_PIN(46, AHB1, B, 0),
__STM32_PIN(47, AHB1, B, 1),
__STM32_PIN(48, AHB1, B, 2),
__STM32_PIN(49, AHB1, F, 11),
__STM32_PIN(50, AHB1, F, 12),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(53, AHB1, F, 13),
__STM32_PIN(54, AHB1, F, 14),
__STM32_PIN(55, AHB1, F, 15),
__STM32_PIN(56, AHB1, G, 0),
__STM32_PIN(57, AHB1, G, 1),
__STM32_PIN(58, AHB1, E, 7),
__STM32_PIN(59, AHB1, E, 8),
__STM32_PIN(60, AHB1, E, 9),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(63, AHB1, E, 10),
__STM32_PIN(64, AHB1, E, 11),
__STM32_PIN(65, AHB1, E, 12),
__STM32_PIN(66, AHB1, E, 13),
__STM32_PIN(67, AHB1, E, 14),
__STM32_PIN(68, AHB1, E, 15),
__STM32_PIN(69, AHB1, B, 10),
__STM32_PIN(70, AHB1, B, 11),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(73, AHB1, B, 12),
__STM32_PIN(74, AHB1, B, 13),
__STM32_PIN(75, AHB1, B, 14),
__STM32_PIN(76, AHB1, B, 15),
__STM32_PIN(77, AHB1, D, 8),
__STM32_PIN(78, AHB1, D, 9),
__STM32_PIN(79, AHB1, D, 10),
__STM32_PIN(80, AHB1, D, 11),
__STM32_PIN(81, AHB1, D, 12),
__STM32_PIN(82, AHB1, D, 13),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(85, AHB1, D, 14),
__STM32_PIN(86, AHB1, D, 15),
__STM32_PIN(87, AHB1, G, 2),
__STM32_PIN(88, AHB1, G, 3),
__STM32_PIN(89, AHB1, G, 4),
__STM32_PIN(90, AHB1, G, 5),
__STM32_PIN(91, AHB1, G, 6),
__STM32_PIN(92, AHB1, G, 7),
__STM32_PIN(93, AHB1, G, 8),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(96, AHB1, C, 6),
__STM32_PIN(97, AHB1, C, 7),
__STM32_PIN(98, AHB1, C, 8),
__STM32_PIN(99, AHB1, C, 9),
__STM32_PIN(100, AHB1, A, 8),
__STM32_PIN(101, AHB1, A, 9),
__STM32_PIN(102, AHB1, A, 10),
__STM32_PIN(103, AHB1, A, 11),
__STM32_PIN(104, AHB1, A, 12),
__STM32_PIN(105, AHB1, A, 13),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(109, AHB1, A, 14),
__STM32_PIN(110, AHB1, A, 15),
__STM32_PIN(111, AHB1, C, 10),
__STM32_PIN(112, AHB1, C, 11),
__STM32_PIN(113, AHB1, C, 12),
__STM32_PIN(114, AHB1, D, 0),
__STM32_PIN(115, AHB1, D, 1),
__STM32_PIN(116, AHB1, D, 2),
__STM32_PIN(117, AHB1, D, 3),
__STM32_PIN(118, AHB1, D, 4),
__STM32_PIN(119, AHB1, D, 5),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(122, AHB1, D, 6),
__STM32_PIN(123, AHB1, D, 7),
__STM32_PIN(124, AHB1, G, 9),
__STM32_PIN(125, AHB1, G, 10),
__STM32_PIN(126, AHB1, G, 11),
__STM32_PIN(127, AHB1, G, 12),
__STM32_PIN(128, AHB1, G, 13),
__STM32_PIN(129, AHB1, G, 14),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
__STM32_PIN(132, AHB1, G, 15),
__STM32_PIN(133, AHB1, B, 3),
__STM32_PIN(134, AHB1, B, 4),
__STM32_PIN(135, AHB1, B, 5),
__STM32_PIN(136, AHB1, B, 6),
__STM32_PIN(137, AHB1, B, 7),
__STM32_PIN_DEFAULT,
__STM32_PIN(139, AHB1, B, 8),
__STM32_PIN(140, AHB1, B, 9),
__STM32_PIN(141, AHB1, E, 0),
__STM32_PIN(142, AHB1, E, 1),
__STM32_PIN_DEFAULT,
__STM32_PIN_DEFAULT,
#endif
};

struct pin_irq_map
{
  rt_uint16_t            pinbit;
  rt_uint32_t            irqbit;
  enum IRQn              irqno;
};
static const  struct pin_irq_map pin_irq_map[] =
{
  {GPIO_Pin_0,  EXTI_Line0,  EXTI0_IRQn    },
  {GPIO_Pin_1,  EXTI_Line1,  EXTI1_IRQn    },
  {GPIO_Pin_2,  EXTI_Line2,  EXTI2_IRQn    },
  {GPIO_Pin_3,  EXTI_Line3,  EXTI3_IRQn    },
  {GPIO_Pin_4,  EXTI_Line4,  EXTI4_IRQn    },
  {GPIO_Pin_5,  EXTI_Line5,  EXTI9_5_IRQn  },
  {GPIO_Pin_6,  EXTI_Line6,  EXTI9_5_IRQn  },
  {GPIO_Pin_7,  EXTI_Line7,  EXTI9_5_IRQn  },
  {GPIO_Pin_8,  EXTI_Line8,  EXTI9_5_IRQn  },
  {GPIO_Pin_9,  EXTI_Line9,  EXTI9_5_IRQn  },
  {GPIO_Pin_10, EXTI_Line10, EXTI15_10_IRQn},
  {GPIO_Pin_11, EXTI_Line11, EXTI15_10_IRQn},
  {GPIO_Pin_12, EXTI_Line12, EXTI15_10_IRQn},
  {GPIO_Pin_13, EXTI_Line13, EXTI15_10_IRQn},
  {GPIO_Pin_14, EXTI_Line14, EXTI15_10_IRQn},
  {GPIO_Pin_15, EXTI_Line15, EXTI15_10_IRQn},
};
struct rt_pin_irq_hdr pin_irq_hdr_tab[] =
{
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},
  {-1, 0, RT_NULL, RT_NULL},                  
};

#define ITEM_NUM(items) sizeof(items)/sizeof(items[0])
const struct pin_index *get_pin(uint8_t pin)
{
  const struct pin_index *index;
  
  if (pin < ITEM_NUM(pins))
  {
    index = &pins[pin];
    if (index->index == -1)
      index = RT_NULL;
  }
  else
  {
    index = RT_NULL;
  }
  
  return index;
};

void stm32_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
  const struct pin_index *index;
  
  index = get_pin(pin);
  if (index == RT_NULL)
  {
    return;
  }
  
  if (value == PIN_LOW)
  {
    GPIO_ResetBits(index->gpio, index->pin);
  }
  else
  {
    GPIO_SetBits(index->gpio, index->pin);
  }
}

int stm32_pin_read(rt_device_t dev, rt_base_t pin)
{
  int value;
  const struct pin_index *index;
  
  value = PIN_LOW;
  
  index = get_pin(pin);
  if (index == RT_NULL)
  {
    return value;
  }
  
  if (GPIO_ReadInputDataBit(index->gpio, index->pin) == Bit_RESET)
  {
    value = PIN_LOW;
  }
  else
  {
    value = PIN_HIGH;
  }
  
  return value;
}

void stm32_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
  const struct pin_index *index;
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  index = get_pin(pin);
  if (index == RT_NULL)
  {
    return;
  }
  
  /* GPIO Periph clock enable */
  RCC_AHB1PeriphClockCmd(index->rcc, ENABLE);
  
  /* Configure GPIO_InitStructure */
  GPIO_InitStructure.GPIO_Pin     = index->pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  if (mode == PIN_MODE_OUTPUT)
  {
    /* output setting */
    GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
  }
  else if (mode == PIN_MODE_OUTPUT_OD)
  {
    /* output setting: od. */
    GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  = GPIO_OType_OD;
  }			
  else if (mode == PIN_MODE_INPUT_ANL)
  {
    /* input setting: analog input . */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
  }
  else if (mode == PIN_MODE_INPUT)
  {
    /* input setting: not pull. */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  }
  else if (mode == PIN_MODE_INPUT_PULLUP)
  {
    /* input setting: pull up. */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  }
  else if (mode == PIN_MODE_INPUT_PULLDOWN)
  {
    /* input setting: pull up. */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  }
  else
  {
    /* input setting:default. */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  }
  GPIO_Init(index->gpio, &GPIO_InitStructure);
}

rt_inline rt_int32_t bit2bitno(rt_uint32_t bit)
{
  int i;
  for(i = 0; i < 32; i++)
  {
    if((0x01 << i) == bit)
    {
      return i;
    }
  }
  return -1;
}
rt_inline const struct pin_irq_map *get_pin_irq_map(uint32_t pinbit)
{
  rt_int32_t mapindex = bit2bitno(pinbit);
  if(mapindex < 0 || mapindex >= ITEM_NUM(pin_irq_map))
  {
    return RT_NULL;
  }
  return &pin_irq_map[mapindex];
};
rt_err_t stm32_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                              rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
  const struct pin_index *index;
  rt_base_t level;
  rt_int32_t irqindex = -1;
  
  index = get_pin(pin);
  if (index == RT_NULL)
  {
    return -RT_ENOSYS;
  }
  irqindex = bit2bitno(index->pin);
  if(irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
  {
    return -RT_ENOSYS;
  }
  
  level = rt_hw_interrupt_disable();
  if(pin_irq_hdr_tab[irqindex].pin == pin   &&
     pin_irq_hdr_tab[irqindex].hdr == hdr   &&
       pin_irq_hdr_tab[irqindex].mode == mode &&
         pin_irq_hdr_tab[irqindex].args == args
           )
  {
    rt_hw_interrupt_enable(level);
    return RT_EOK;
  }
  if(pin_irq_hdr_tab[irqindex].pin != -1)
  {
    rt_hw_interrupt_enable(level);
    return -RT_EBUSY;
  }
  pin_irq_hdr_tab[irqindex].pin = pin;
  pin_irq_hdr_tab[irqindex].hdr = hdr;
  pin_irq_hdr_tab[irqindex].mode = mode;
  pin_irq_hdr_tab[irqindex].args = args;
  rt_hw_interrupt_enable(level);
  
  return RT_EOK;
}
rt_err_t stm32_pin_detach_irq(struct rt_device *device, rt_int32_t pin)
{
  const struct pin_index *index;
  rt_base_t level;
  rt_int32_t irqindex = -1;
  
  index = get_pin(pin);
  if (index == RT_NULL)
  {
    return -RT_ENOSYS;
  }
  irqindex = bit2bitno(index->pin);
  if(irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
  {
    return -RT_ENOSYS;
  }
  
  level = rt_hw_interrupt_disable();
  if(pin_irq_hdr_tab[irqindex].pin == -1)
  {
    rt_hw_interrupt_enable(level);
    return RT_EOK;
  }
  pin_irq_hdr_tab[irqindex].pin = -1;
  pin_irq_hdr_tab[irqindex].hdr = RT_NULL;
  pin_irq_hdr_tab[irqindex].mode = 0;
  pin_irq_hdr_tab[irqindex].args = RT_NULL;
  rt_hw_interrupt_enable(level);
  
  return RT_EOK;
}
rt_err_t stm32_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                              rt_uint32_t enabled)
{
  const struct pin_index *index;
  const struct pin_irq_map *irqmap;
  rt_base_t level;
  rt_int32_t irqindex = -1;
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
  index = get_pin(pin);
  if (index == RT_NULL)
  {
    return -RT_ENOSYS;
  }
  if(enabled == PIN_IRQ_ENABLE)
  {
    irqindex = bit2bitno(index->pin);
    if(irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
      return -RT_ENOSYS;
    }
    level = rt_hw_interrupt_disable();
    if(pin_irq_hdr_tab[irqindex].pin == -1)
    {
      rt_hw_interrupt_enable(level);
      return -RT_ENOSYS;
    }
    irqmap = &pin_irq_map[irqindex];
    /* GPIO Periph clock enable */
    RCC_APB2PeriphClockCmd(index->rcc, ENABLE);
    /* Configure GPIO_InitStructure */
    GPIO_InitStructure.GPIO_Pin     = index->pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(index->gpio, &GPIO_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel= irqmap->irqno;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 2;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2; 
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    SYSCFG_EXTILineConfig(index->port_source, index->pin_source);
    EXTI_InitStructure.EXTI_Line = irqmap->irqbit;  
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    switch(pin_irq_hdr_tab[irqindex].mode)
    {
    case PIN_IRQ_MODE_RISING:
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
      break;
    case PIN_IRQ_MODE_FALLING:
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
      break;
    case PIN_IRQ_MODE_RISING_FALLING:
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
      break;
    }
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
    EXTI_Init(&EXTI_InitStructure);
    rt_hw_interrupt_enable(level);
  }
  else if(enabled == PIN_IRQ_DISABLE)
  {
    irqmap = get_pin_irq_map(index->pin);
    if(irqmap == RT_NULL)
    {
      return -RT_ENOSYS;
    }
    EXTI_InitStructure.EXTI_Line = irqmap->irqbit;  
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;  
    EXTI_Init(&EXTI_InitStructure);
  }
  else
  {
    return -RT_ENOSYS;
  }
  
  return RT_EOK;
}
const static struct rt_pin_ops _stm32_pin_ops =
{
  stm32_pin_mode,
  stm32_pin_write,
  stm32_pin_read,
  stm32_pin_attach_irq,
  stm32_pin_detach_irq,
  stm32_pin_irq_enable,
};

int stm32_hw_pin_init(void)
{
  int result;
  
  result = rt_device_pin_register("pin", &_stm32_pin_ops, RT_NULL);
  return result;
}
INIT_BOARD_EXPORT(stm32_hw_pin_init);

rt_inline void pin_irq_hdr(int irqno)
{
  EXTI_ClearITPendingBit(pin_irq_map[irqno].irqbit);
  if(pin_irq_hdr_tab[irqno].hdr)
  {
    pin_irq_hdr_tab[irqno].hdr(pin_irq_hdr_tab[irqno].args);
  }
}

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
void gpio_out_init(rt_base_t pin)
{
  rt_pin_mode(pin, PIN_MODE_OUTPUT);
}
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
void gpio_out_on(rt_base_t pin)
{
  rt_pin_write(pin, PIN_HIGH);
}
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
void gpio_out_off(rt_base_t pin)
{
  rt_pin_write(pin, PIN_LOW);
}
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
void gpio_in_init(rt_base_t pin)
{
  rt_pin_mode(pin, PIN_MODE_INPUT);
}
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
int gpio_pin_read(rt_base_t pin)
{
  return rt_pin_read(pin);
}
/****************函数定义********************************************
//函数名称	:gpio_irq_install
//功能		:gpio中断装载函数
//输入		:pin:gpio管脚编号
pinMode:gpio管脚模式
irqMode:gpio中断模式
hdr:中断回调函数
args:中断回调函数参数 
//输出		:
//使用资源	:
//全局变量	:   
//调用函数	:
//中断资源	:  
//返回		:
//备注		:
*********************************************************************/
rt_err_t gpio_irq_install(rt_base_t pin, rt_base_t pinMode, void (*hdr)(void *args),rt_uint32_t irqMode,void  *args)
{
  rt_pin_mode(pin, pinMode);
  // 绑定中断
  rt_pin_attach_irq(pin, irqMode, hdr, args);
  // 使能中断
  return rt_pin_irq_enable(pin, PIN_IRQ_ENABLE);
}

void EXTI0_IRQHandler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();
  pin_irq_hdr(0);
  /* leave interrupt */
  rt_interrupt_leave();
}
void EXTI1_IRQHandler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();
  pin_irq_hdr(1);
  /* leave interrupt */
  rt_interrupt_leave();
}
void EXTI2_IRQHandler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();
  pin_irq_hdr(2);
  /* leave interrupt */
  rt_interrupt_leave();
}
void EXTI3_IRQHandler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();
  pin_irq_hdr(3);
  /* leave interrupt */
  rt_interrupt_leave();
}
void EXTI4_IRQHandler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();
  pin_irq_hdr(4);
  /* leave interrupt */
  rt_interrupt_leave();
}
void EXTI9_5_IRQHandler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
    pin_irq_hdr(5);
  }
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
    pin_irq_hdr(6);
  }
  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
    pin_irq_hdr(7);
  }
  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    pin_irq_hdr(8);
  }
  if(EXTI_GetITStatus(EXTI_Line9) != RESET)
  {
    pin_irq_hdr(9);
  }
  /* leave interrupt */
  rt_interrupt_leave();
}
void EXTI15_10_IRQHandler(void)
{
  /* enter interrupt */
  rt_interrupt_enter();
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    pin_irq_hdr(10);
  }
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {
    pin_irq_hdr(11);
  }
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    pin_irq_hdr(12);
  }
  if(EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    pin_irq_hdr(13);
  }
  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
    pin_irq_hdr(14);
  }
  if(EXTI_GetITStatus(EXTI_Line15) != RESET)
  {
    pin_irq_hdr(15);
  }
  /* leave interrupt */
  rt_interrupt_leave();
}

#endif
