/* drivers/i2c/chips/max77819.c - MAX77819 motion sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <math.h>

#include "max77819_periph.h"

#define MAX77819_BL_DEBUG

#ifdef MAX77819_BL_DEBUG
/*----------------------------------------------------------------------------*/
#define MAXBL_TAG                  "[MAX77819_BL] "
#define MAXBL_FUN(f)               printk(KERN_ERR MAXBL_TAG"%s\n", __FUNCTION__)
#define MAXBL_ERR(fmt, args...)    printk(KERN_ERR MAXBL_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MAXBL_LOG(fmt, args...)    printk(KERN_ERR MAXBL_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
#else
/*----------------------------------------------------------------------------*/
#define MAXBL_TAG                  "[MAX77819_BL] "
#define MAXBL_FUN(f)               printk(KERN_INF MAXBL_TAG"%s\n", __FUNCTION__)
#define MAXBL_ERR(fmt, args...)    printk(KERN_ERR MAXBL_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MAXBL_LOG(fmt, args...)    printk(KERN_INF MAXBL_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
#endif

int max77819_set_backlight_level(int level)
{
    u8 dataBuffer = 0;
    
    level = level & 0xff;
	
    MAXBL_LOG("[max77819_set_backlight] level:%d\n",level);
    
    max77819_write_byte(MAX77819_IWLED,level);
#ifdef MAX77819_BL_DEBUG
{
    u8 dataBuffer = 0;
    max77819_read_byte(0x9C,&dataBuffer);
    MAXBL_LOG("[max77819_set_backlight] addr:0x9C,dataBuffer:0x%x\n", dataBuffer);
    
    max77819_read_byte(MAX77819_IWLED,&dataBuffer);
    MAXBL_LOG("[max77819_set_backlight] addr:0x%x,dataBuffer:0x%x\n", MAX77819_IWLED,dataBuffer);
    
    max77819_read_byte(MAX77819_WLEDBSTCNTL,&dataBuffer);
    MAXBL_LOG("[max77819_set_backlight] addr:0x%x,dataBuffer:0x%x\n", MAX77819_WLEDBSTCNTL,dataBuffer);
}
#endif
}
EXPORT_SYMBOL_GPL(max77819_set_backlight_level);

int max77819_set_backlight_cntl_mode(unsigned char mode)
{	    
    max77819_write_byte(MAX77819_WLEDBSTCNTL,mode);//0xC2
}
EXPORT_SYMBOL_GPL(max77819_set_backlight_cntl_mode);

int max77819_set_backlight_enable(unsigned char on)
{
	printk("max77819_set_backlight_enable:kernel: on =%d", on);
	if (on)
	{
	    max77819_set_backlight_cntl_mode(MAX77819_WLEDOVP  | MAX77819_WLED2EN | MAX77819_WLED1EN | MAX77819_WLEDPWM1EN | MAX77819_WLEDPWM2EN);
		max77819_set_backlight_level(0xEF);  //Set current to midian
	}else{
		max77819_set_backlight_cntl_mode(0x00);
	}
}

EXPORT_SYMBOL_GPL(max77819_set_backlight_enable);