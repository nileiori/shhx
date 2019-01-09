#include <rthw.h>
#include <rtthread.h>
#include <version.h>



void ei_show_version(void)
{
    rt_kprintf("\n \\ | /\n");
    rt_kprintf("- EI -     Happy New Year\n");
    rt_kprintf(" / | \\     %d.%d.%d build %s\n",
               EI_VERSION, EI_SUBVERSION, EI_REVISION, __DATE__);
    rt_kprintf(" 2006 - 2018 Copyright by Shenzhen EI company\n");
}

