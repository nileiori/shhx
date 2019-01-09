#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <ec20.h>
#include <gpio.h>



//ALIGN(RT_ALIGN_SIZE)

#define	EC20_THREAD_PRIORITY	21

#define	EC20_DATA_PKG_SZ		( 128 )
#define EC20_WAIT_CHR_TICK 		( 20 )

#define	EC20_WAKEUP_TIMES	1000



static struct ec20_ctx *_ec20_the_ctx;


static rt_size_t _ec20_read_data(
                                 struct ec20_ctx *ctx,
                                 rt_size_t len);
static rt_size_t _ec20_write_data(
                                  struct ec20_ctx *ctx,
                                  rt_uint8_t *buf,
                                  rt_size_t len);
static void _ec20_inti(struct ec20_ctx *ctx)
{
  //pull high ec20pwron pin
  rt_pin_mode(ctx->modempwronpin, PIN_MODE_OUTPUT);
  rt_pin_write(ctx->modempwronpin, PIN_HIGH);

  rt_thread_delay(50);//delay 50ms
  
  //pull high ec20pwrkey pin
  rt_pin_mode(ctx->modempwrkeypin, PIN_MODE_OUTPUT);
  rt_pin_write(ctx->modempwrkeypin, PIN_HIGH);
  //pull low ec20dtr pin
  rt_pin_mode(ctx->modemdtrpin, PIN_MODE_OUTPUT);
  rt_pin_write(ctx->modemdtrpin, PIN_LOW);

  rt_pin_mode(ctx->modemaprdypin, PIN_MODE_INPUT);
  
  rt_timer_start(ctx->timer);//start soft timer 
  
}
static rt_bool_t _ec20_tick_timeout(rt_tick_t tick_start, rt_tick_t tick_long)
{
    rt_tick_t tick_end = tick_start + tick_long;
    rt_tick_t tick_now = rt_tick_get();
    rt_bool_t result = RT_FALSE;

    if (tick_end >= tick_start)
    {
        if (tick_now >= tick_end)
        {
            result = RT_TRUE;
        }
        else
        {
            result = RT_FALSE;
        }
    }
    else
    {
        if ((tick_now < tick_start) && (tick_now >= tick_end))
        {
            result = RT_TRUE;
        }
        else
        {
            result = RT_FALSE;
        }
    }
	
    return result;
}
//通过如下步骤使模块进入睡眠模式
//1,用 AT+QSCLK=1 命令使能睡眠功能。2,拉高 DTR 引脚。
rt_err_t _ec20_sleep(struct ec20_ctx *ctx)
{
	rt_tick_t tick_start;
	int pinsta;

	_ec20_write_data(ctx,"AT+QSCLK=1\r\n",rt_strlen("AT+QSCLK=1\r\n"));
	rt_pin_write(ctx->modemdtrpin, PIN_HIGH);
	
	tick_start = rt_tick_get();
    while (1)
    {
        pinsta = rt_pin_read(ctx->modemaprdypin);
        if (!pinsta)
        {
            break;
        }

        if (_ec20_tick_timeout(tick_start, rt_tick_from_millisecond(EC20_WAKEUP_TIMES)))
        {
            return RT_ERROR;
        }
    }
	return RT_EOK;
}
rt_err_t _ec20_wakeup(struct ec20_ctx *ctx)
{
	rt_tick_t tick_start;
	int pinsta;
	rt_pin_write(ctx->modemdtrpin, PIN_LOW);
	tick_start = rt_tick_get();

    while (1)
    {
        pinsta = rt_pin_read(ctx->modemaprdypin);
        if (pinsta)
        {
            break;
        }

        if (_ec20_tick_timeout(tick_start, rt_tick_from_millisecond(EC20_WAKEUP_TIMES)))
        {
            return RT_ERROR;
        }
    }
	
	return RT_EOK;
}
//AT+CFUN=0： 最少功能模式（关闭 RF 和(U)SIM 卡）。
//AT+CFUN=1： 全功能模式（默认）。
//AT+CFUN=4： 关闭 RF 功能（飞行模式）。
rt_err_t _ec20_enterflymode(struct ec20_ctx *ctx,rt_uint8_t mode)
{
	char buf[20] = {'\0',};
	
	rt_sprintf(buf,"AT+CFUN=%d",mode);
	_ec20_write_data(ctx,(rt_uint8_t*)buf,rt_strlen(buf));
	
	return RT_EOK;
}

static enum ec20_stage _ec20_process_00(struct ec20_ctx *ctx)
{
  if(rt_strncmp("\r\nRDY",(char*)ctx->revbuf,5) == 0)
  {
    _ec20_write_data(ctx,"at\r\n",rt_strlen("at\r\n"));
    
    ctx->stage = EC20_STAGE_01;
  }
  
  return ctx->stage;
}
static enum ec20_stage _ec20_process_01(struct ec20_ctx *ctx)
{
  if(rt_strncmp("at",(char*)ctx->revbuf,2) == 0)
  {
    
    ctx->stage = EC20_STAGE_02;
  }
  
  return ctx->stage;
}

#define ec20_process_tab_size		sizeof(ec20_process_tab) / sizeof(ec20_process_tab[0])
//ec20任务表
static const struct ec20_process ec20_process_tab[] =
{
  {EC20_STAGE_RDY,_ec20_process_00},
  {EC20_STAGE_01,_ec20_process_01},
  {EC20_STAGE_02,RT_NULL},
  
};

static rt_err_t _ec20_rx_ind(rt_device_t dev, rt_size_t size)
{
  return rt_sem_release(_ec20_the_ctx->sem);
}


//软件定时器
static void _ec20_timeout(void *arg)
{
  struct ec20_ctx *ec20 = (struct ec20_ctx *)arg;
  //pull low ec20pwrkey pin
  if(rt_pin_read(ec20->modempwrkeypin))
  {
    rt_pin_write(ec20->modempwrkeypin, PIN_LOW);
    rt_timer_control(ec20->timer,RT_TIMER_CTRL_SET_TIME,&ec20->periodicticks);
    rt_timer_control(ec20->timer,RT_TIMER_CTRL_SET_PERIODIC,RT_NULL);//设置成周期定时器
  }
  else
  {
    //do something....
    //rt_kprintf("\n");
    //rt_kprintf("EC20当前进程号:%d\n",ec20->stage);
    //rt_kprintf("EC20软件定时器单次tick:%d\n",ec20->onceticks);
    //rt_kprintf("EC20软件定时器周期性tick:%d\n",ec20->periodicticks);
    //rt_kprintf("EC20电源管脚号:%d\n",ec20->modempwrpin);
    //rt_kprintf("EC20信号量tick:%d\n",ec20->semtick);
    //_ec20_write_data(_ec20_the_ctx,"at\r\n",rt_strlen("AT\r\n"));

  }
}
//读取通讯口的数据
static rt_size_t _ec20_read_data(
                                 struct ec20_ctx *ctx,
                                 rt_size_t len)
{
  /* we should already have had the code */
  rt_uint8_t *buf = ctx->revbuf;
  rt_size_t readlen = 0;
  
  do
  {
    readlen += rt_device_read(ctx->dev,
                              0, buf+readlen, len-readlen);
    if (readlen >= len)
      return readlen;
  } while (rt_sem_take(ctx->sem, ctx->semtick) == RT_EOK);
  
  return readlen;
}
static rt_size_t _ec20_write_data(
                                  struct ec20_ctx *ctx,
                                  rt_uint8_t *buf,
                                  rt_size_t len)
{
  
  return rt_device_write(ctx->dev, 0, buf, len);
}
static void ec20_thread_entry(void *parameter)
{
  enum ec20_stage cmdcode;
  rt_size_t revlen = 0;
  struct ec20_ctx *ctx = (struct ec20_ctx *)parameter;
  RT_ASSERT(ctx);
  
  _ec20_inti(ctx);
  
  while( 1 )
  {
    revlen = _ec20_read_data(ctx,EC20_DATA_PKG_SZ);//read data
    if(revlen)
    {
      if(ctx->ec20_proc[ctx->stage].ec20_cb)
      {
        cmdcode = ctx->ec20_proc[ctx->stage].ec20_cb(ctx);
      }     
      rt_kprintf("%s...%d...%d\n",(char*)ctx->revbuf,revlen,cmdcode);
      revlen = 0;
    }
  }
}
static rt_err_t ec20_connect_to_device(
                                       struct ec20_ctx *ctx,
                                       rt_device_t dev,
                                       rt_uint16_t oflag,
                                       const struct ec20_process	*ec20_proc)
{
  rt_err_t res;
  rt_err_t (*odev_rx_ind)(rt_device_t dev, rt_size_t size);
  rt_uint16_t odev_flag;
  int int_lvl;
  
  
  ctx->onceticks = 1000;
  ctx->periodicticks = 2000;
  
  ctx->modempwronpin 	= PIN_NO_EC20_PWRON;
  ctx->modempwrkeypin   = PIN_NO_EC20_PWRKEY;
  ctx->modemdtrpin   	= PIN_NO_EC20_DTR;
  ctx->modemaprdypin    = PIN_NO_EC20_AP_READY;
  
  ctx->semtick = EC20_WAIT_CHR_TICK;
  
  ctx->stage = EC20_STAGE_RDY;
  ctx->ec20_proc = ec20_proc;
  ctx->dev      = dev;
  ctx->sem   = rt_sem_create("ec20sem", 0, RT_IPC_FLAG_FIFO);
  ctx->event = rt_event_create("ec20event", RT_IPC_FLAG_FIFO);
  
  ctx->timer = rt_timer_create("ec20timer",
                               _ec20_timeout,
                               (void *)ctx,
                               ctx->onceticks,
                               RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
  
  ctx->revbuf = rt_malloc(EC20_DATA_PKG_SZ);
  
  //ctx->sendbuf = rt_malloc(EC20_DATA_PKG_SZ);
  
  odev_rx_ind = dev->rx_indicate;
  /* no data should be received before the device has been fully setted up.
  */
  int_lvl = rt_hw_interrupt_disable();
  rt_device_set_rx_indicate(dev, _ec20_rx_ind);
  
  odev_flag = dev->flag;
  /* make sure the device don't change the content. */
  dev->flag &= ~RT_DEVICE_FLAG_STREAM;
  rt_hw_interrupt_enable(int_lvl);
  
  res = rt_device_open(dev, oflag);
  if (res != RT_EOK)
    goto __exit;
  
  ctx->tid = rt_thread_create("ec20_thread",
                              ec20_thread_entry,
                              (void*)ctx,
                              ( rt_uint32_t)512,
                              ( rt_uint8_t )EC20_THREAD_PRIORITY,
                              ( rt_uint32_t )5);
  if(ctx->tid != RT_NULL)
    rt_thread_startup(ctx->tid);
  
  return res;
__exit:
  /* no rx_ind should be called before the callback has been fully detached.
  */
  int_lvl = rt_hw_interrupt_disable();
  rt_sem_detach(ctx->sem);
  rt_event_detach(ctx->event);
  
  dev->flag = odev_flag;
  rt_device_set_rx_indicate(dev, odev_rx_ind);
  rt_hw_interrupt_enable(int_lvl);
  
  rt_free(ctx->revbuf);
  //rt_free(ctx->sendbuf);
  _ec20_the_ctx = RT_NULL;
  
  return res;
}
//将模块挂载到相应串口
rt_err_t ec20_comm_mount(const char *devname)
{
  
  rt_device_t dev = rt_device_find(devname);
  if (!dev)
  {
    rt_kprintf("could not find device %s\n", devname);
    return -1;
  }
  
  _ec20_the_ctx = (struct ec20_ctx *)rt_malloc(sizeof(struct ec20_ctx));
  RT_ASSERT(_ec20_the_ctx);
  
  return ec20_connect_to_device(_ec20_the_ctx, dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                                ec20_process_tab);
}


