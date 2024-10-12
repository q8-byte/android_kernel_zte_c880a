#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>


/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;

//#define FLASH_GPIO_ENF GPIO12
//#define FLASH_GPIO_ENT GPIO13

//static int g_bLtVersion=0;

/*****************************************************************************
Functions
*****************************************************************************/

/* Registers */
#define MAX77819_IFLASH			0x00
#define MAX77819_ITORCH			0x02
#define MAX77819_TORCH_TMR		0x03
#define MAX77819_FLASH_TMR		0x04
#define MAX77819_FLASH_EN		0x05
#define MAX77819_MAX_FLASH1		0x06
#define MAX77819_MAX_FLASH2		0x07
#define MAX77819_MAX_FLASH3		0x08
#define MAX77819_VOUT_CNTL		0x0A
#define MAX77819_VOUT_FLASH		0x0B
#define MAX77819_FLASH_INT		0x0E
#define MAX77819_FLASH_INT_MASK		0x0F
#define MAX77819_FLASH_STATUS		0x10

/* MAX77819_IFLASH */
#define MAX77819_FLASH_I		0x3F

/* MAX77819_ITORCH */
#define MAX77819_TORCH_I		0x0F

/* MAX77819_TORCH_TMR */
#define MAX77819_TORCH_TMR_DUR		0x0F
#define MAX77819_DIS_TORCH_TMR		0x40
#define MAX77819_TORCH_TMR_MODE		0x80
#define MAX77819_TORCH_TMR_ONESHOT	0x00
#define MAX77819_TORCH_TMR_MAXTIMER	0x80

/* MAX77819_FLASH_TMR */
#define MAX77819_FLASH_TMR_DUR		0x0F
#define MAX77819_FLASH_TMR_MODE		0x80
#define MAX77819_FLASH_TMR_ONESHOT	0x00
#define MAX77819_FLASH_TMR_MAXTIMER	0x80

/* MAX77819_FLASH_EN */
#define MAX77819_TORCH_FLED_EN		0x0C
#define MAX77819_FLASH_FLED_EN		0xC0
#define MAX77819_OFF			0x00
#define MAX77819_BY_FLASHEN		0x01
#define MAX77819_BY_TORCHEN		0x02
#define MAX77819_BY_I2C			0X03

/* MAX77819_MAX_FLASH1 */
#define MAX77819_MAX_FLASH_HYS		0x03
#define MAX77819_MAX_FLASH_TH		0x7C
#define MAX77819_MAX_FLASH_TH_FROM_VOLTAGE(uV) \
		((((uV) - 2400000) / 33333) << M2SH(MAX77819_MAX_FLASH_TH))
#define MAX77819_MAX_FL_EN		0x80

/* MAX77819_MAX_FLASH2 */
#define MAX77819_LB_TMR_F		0x07
#define MAX77819_LB_TMR_R		0x38
#define MAX77819_LB_TME_FROM_TIME(uSec) ((uSec) / 256)

/* MAX77819_MAX_FLASH3 */
#define MAX77819_FLED_MIN_OUT		0x3F
#define MAX77819_FLED_MIN_MODE		0x80

/* MAX77819_VOUT_CNTL */
#define MAX77819_BOOST_FLASH_MDOE	0x07
#define MAX77819_BOOST_FLASH_MODE_OFF	0x00
#define MAX77819_BOOST_FLASH_MODE_FLED1	0x01
#define MAX77819_BOOST_FLASH_MODE_FIXED	0x04

/* MAX77819_VOUT_FLASH */
#define MAX77819_BOOST_VOUT_FLASH	0x7F
#define MAX77819_BOOST_VOUT_FLASH_FROM_VOLTAGE(uV)				\
		((uV) <= 3300000 ? 0x00 :					\
		((uV) <= 5500000 ? (((mV) - 3300000) / 25000 + 0x0C) : 0x7F))

/* MAX77819_FLASH_INT_MASK */
#define MAX77819_FLED_OPEN_M		0x04
#define MAX77819_FLED_SHORT_M		0x08
#define MAX77819_MAX_FLASH_M		0x10
#define MAX77819_FLED_FAIL_M		0x20

/* MAX77819_FLASH_STATAUS */
#define MAX77819_TORCH_ON_STAT		0x04
#define MAX77819_FLASH_ON_STAT		0x08

#define MAX_FLASH_CURRENT	1000	// 1000mA(0x1f)
#define MAX_TORCH_CURRENT	250	// 250mA(0x0f)   
#define MAX_FLASH_DRV_LEVEL	63	/* 15.625 + 15.625*63 mA */
#define MAX_TORCH_DRV_LEVEL	15	/* 15.625 + 15.625*15 mA */

#define MAX_LIGHT_MODE_FLASH 0
#define MAX_LIGHT_MODE_TORCH 1

//Define GPIO number
#define MAX_LIGHT_GPIO_FLASH GPIO_P1_0
#define MAX_LIGHT_GPIO_TORCH GPIO_P1_1

unsigned char current_setting = 0; //it is used by mtk to remeber their brightness setting

#define FL_TAG                  "[MAX77819_FLASHLIGHT] "
#define FL_FUN(f)               printk(KERN_ERR FL_TAG"%s\n", __FUNCTION__)
#define FL_ERR(fmt, args...)    printk(KERN_ERR FL_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define FL_LOG(fmt, args...)    printk(KERN_ERR FL_TAG fmt, ##args)

static int max77819_flashCur[] = { 11, 13,  15,  20,  24,  27,  31,  34,  40, 45, 50, 55};
static int max77819_torchEn [] = { 3, 8,  10,  0,  0,  0,  0,  0,  0,  0,  0,  0};

static unsigned char max77819_light_mode = MAX_LIGHT_MODE_TORCH;

extern int max77819_pmic_read_byte(kal_uint8 cmd, kal_uint8 *returnData);
extern int max77819_pmic_reg_write(kal_uint8 reg, kal_uint8 val);
extern int max77819_pmic_reg_read(kal_uint8 reg, kal_uint8 *val);
extern int max77819_pmic_reg_update_bits(kal_uint8 reg, kal_uint8 mask, kal_uint8 val);


static void max77819_set_gpio_out(unsigned int pin_num, unsigned char val)
{
    mt_set_gpio_mode(pin_num,0);
	mt_set_gpio_dir(pin_num,1);
	mt_set_gpio_pull_enable(pin_num,1);
	mt_set_gpio_pull_select(pin_num,val);
	mt_set_gpio_out(pin_num,val);
}

void max77819_flash_set_duty(unsigned char duty)
{
	FL_FUN();
	//mtk duty range:0-13
	FL_LOG("duty:%d",duty);
	if(duty>=0 && duty <=13)
		current_setting = duty;
	else
		current_setting = 1;  //Default set as torch mode
}
void max77819_flash_preOn(void)
{
	  FL_FUN();
	  //set cuurent with duty_setting
		FL_LOG("current_setting:%d",current_setting);
    if(max77819_torchEn[current_setting]!= 0)
		{
				//Tourch Mode
				FL_LOG("Tourch Mode\n");
				max77819_light_mode = MAX_LIGHT_MODE_TORCH;
				max77819_pmic_reg_write(MAX77819_ITORCH,max77819_torchEn[current_setting]);
				//max77819_pmic_reg_update_bits(MAX77819_FLASH_EN,MAX77819_TORCH_FLED_EN,MAX77819_BY_TORCHEN<<2);
		}
		else
		{
		    //Flash Mode
		    FL_LOG("Flash Mode\n");
		    max77819_light_mode = MAX_LIGHT_MODE_FLASH;
		    max77819_pmic_reg_write(MAX77819_IFLASH,max77819_flashCur[current_setting]);
		    //max77819_pmic_reg_update_bits(MAX77819_FLASH_EN,MAX77819_FLASH_FLED_EN,MAX77819_BY_FLASHEN<<6);
		}
		
}
void max77819_flash_init(void)
{
	unsigned char val;
	kal_uint8 max77819_reg = 0;
	/* wangjianping 20140429 added for clear Flash Interrupt register, start */
    int check_count = 5;

	do{
    	max77819_pmic_reg_read(0x0E,&val);
    	FL_LOG("[max77819_flash_init] read 0x0E: 0x%x, check_count:%d\n",val, check_count);
		check_count--;
    }while((val != 0) && (check_count));
	/* wangjianping 20140429 added for clear Flash Interrupt register, end */	
	max77819_pmic_reg_read(0x20,&val);
	FL_LOG("[max77819_flash_init] read 0x20: 0x%x",val);
	max77819_pmic_reg_read(0x0E,&val);
	FL_LOG("[max77819_flash_init] read 0x0E: 0x%x",val);
	max77819_pmic_reg_read(0x0E,&val);
	FL_LOG("[max77819_flash_init] read 0x0E: 0x%x",val);
	max77819_pmic_reg_read(0x0E,&val);
	FL_LOG("[max77819_flash_init] read 0x0E: 0x%x",val);
	max77819_pmic_reg_write(MAX77819_TORCH_TMR,0xcf);
	max77819_pmic_reg_write(MAX77819_FLASH_TMR,0x8f);
	max77819_pmic_reg_write(MAX77819_FLASH_INT_MASK,0xff);

	 mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE_PIN,GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE_PIN,GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN,0);
	 mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN,GPIO_CAMERA_FLASH_EN_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,0);
	/*pca9575_set_gpio_dir(MAX_LIGHT_GPIO_TORCH,0);
	pca9575_set_gpio_output(MAX_LIGHT_GPIO_TORCH,0);
	 pca9575_set_gpio_dir(MAX_LIGHT_GPIO_FLASH,0);
	pca9575_set_gpio_output(MAX_LIGHT_GPIO_FLASH,0);*/
	max77819_pmic_reg_update_bits(MAX77819_FLASH_EN,MAX77819_TORCH_FLED_EN,MAX77819_BY_TORCHEN<<2);
	max77819_pmic_reg_update_bits(MAX77819_FLASH_EN,MAX77819_FLASH_FLED_EN,MAX77819_BY_FLASHEN<<6);
	max77819_pmic_reg_write(0x06,0xfe);
	max77819_pmic_reg_write(0x07,0x38);
	//fixed mode
	//max77819_pmic_reg_write(MAX77819_VOUT_CNTL,0x04);
	max77819_pmic_reg_write(MAX77819_VOUT_CNTL,0x01);
	
}

void max77819_flash_enable(void)
{
	FL_FUN();
		    kal_uint8 max77819_reg = 0;
    int ret = 0;

	//max77819_pmic_reg_update_bits(MAX77819_FLASH_EN,MAX77819_TORCH_FLED_EN,MAX77819_BY_TORCHEN<<2);
	//max77819_pmic_reg_update_bits(MAX77819_FLASH_EN,MAX77819_FLASH_FLED_EN,MAX77819_BY_FLASHEN<<6);

	   /* max77819_pmic_read_byte(0x00, &max77819_reg);
    FL_LOG("[max77819_pmic_reg_update_bits] Before Reg[%x]=0x%x\n", 0x00, max77819_reg);
	    max77819_pmic_read_byte(0x04, &max77819_reg);
    FL_LOG("[max77819_pmic_reg_update_bits] Before Reg[%x]=0x%x\n", 0x04, max77819_reg);
	    max77819_pmic_read_byte(0x05, &max77819_reg);
    FL_LOG("[max77819_pmic_reg_update_bits] Before Reg[%x]=0x%x\n", 0x05, max77819_reg);
		    max77819_pmic_read_byte(0x0a, &max77819_reg);
    FL_LOG("[max77819_pmic_reg_update_bits] Before Reg[%x]=0x%x\n", 0x0a, max77819_reg);
		    max77819_pmic_read_byte(0x0b, &max77819_reg);
    FL_LOG("[max77819_pmic_reg_update_bits] Before Reg[%x]=0x%x\n", 0x0b, max77819_reg);*/
	
	//max77819_pmic_reg_write(MAX77819_FLASH_EN,0xc0);

	max77819_flash_preOn();
	
	if(max77819_light_mode == MAX_LIGHT_MODE_TORCH){  //Enable Torch
		 /*pca9575_set_gpio_dir(MAX_LIGHT_GPIO_TORCH,0);
		pca9575_set_gpio_output(MAX_LIGHT_GPIO_TORCH,1);
		 pca9575_set_gpio_dir(MAX_LIGHT_GPIO_FLASH,0);
	        pca9575_set_gpio_output(MAX_LIGHT_GPIO_FLASH,0);*/
	 mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE_PIN,GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE_PIN,GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN,0);
	 mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN,GPIO_CAMERA_FLASH_EN_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,1);
  }else{ //Enable Flashlight
	 /*pca9575_set_gpio_dir(MAX_LIGHT_GPIO_TORCH,0);
	pca9575_set_gpio_output(MAX_LIGHT_GPIO_TORCH,0);
	 pca9575_set_gpio_dir(MAX_LIGHT_GPIO_FLASH,0);
	pca9575_set_gpio_output(MAX_LIGHT_GPIO_FLASH,1);*/
	 mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE_PIN,GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE_PIN,GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN,1);
	 mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN,GPIO_CAMERA_FLASH_EN_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,0);
  }

}
	
void max77819_flash_disable(void)
{
	FL_FUN();
	 /*pca9575_set_gpio_dir(MAX_LIGHT_GPIO_TORCH,0);
	pca9575_set_gpio_output(MAX_LIGHT_GPIO_TORCH,0);
	 pca9575_set_gpio_dir(MAX_LIGHT_GPIO_FLASH,0);
	pca9575_set_gpio_output(MAX_LIGHT_GPIO_FLASH,0);*/
	 mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE_PIN,GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE_PIN,GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN,0);
	 mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN,GPIO_CAMERA_FLASH_EN_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,0);
}

void max77819_flash_deinit(void)
{
	FL_FUN();
	max77819_flash_disable();
	max77819_pmic_reg_write(0x05,0x00);
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    //FL_Disable();
	max77819_flash_disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{ 
  static int init_flag;
  if (init_flag==0)
  {
	init_flag=1;
    INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;
   }
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("LM3642 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		//FL_dim_duty(arg);
			max77819_flash_set_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{

    		    int s;
    		    int ms;
    		    if(g_timeOutTimeMs>1000)
            	{
            		s = g_timeOutTimeMs/1000;
            		ms = g_timeOutTimeMs - s*1000;
            	}
            	else
            	{
            		s = 0;
            		ms = g_timeOutTimeMs;
            	}

				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( s, ms*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			//FL_Enable();
				max77819_flash_enable();
    		}
    		else
    		{
    			//FL_Disable();
				max77819_flash_disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    //FL_Init();
		max77819_flash_init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	//FL_Uninit();
		max77819_flash_deinit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


