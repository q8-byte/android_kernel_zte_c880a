#ifndef __GFX1XM_SPI_H
#define __GFX1XM_SPI_H

#include <linux/types.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>
#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <mach/eint.h>
#include <cust_eint.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>

/****************GFX1XM Macro interface*******************/
#define FW_UPDATE               1
#define ESD_PROTECT             1
#define CFG_UPDATE              1
#define SPI_ASYNC   1
#define GFX1XM_DEBUG               1
#define CONFIG_HAS_EARLYSUSPEND  1
#define GFX1XM_DRIVER_VERSION   "V1.0<20140811>"
#define PROCESSOR_64_BIT  1
//#define SPI_TRANSFER_256
//#define SPI_TRANSFER_512
#define SPI_TRANSFER_1024

#define FP_FF_MODE_EN   1
//#define GF_FF_KEY_EN 
#define ESD_TIME    2 * HZ  //  HZ/2
//#define GF_PULLKEY 

/********************GFX1XM Mapping**********************/
#define GFX1XM_BASE             (0x8000)
#define GFX1XM_OFFSET(x)        (GFX1XM_BASE + x)

#define GFX1XM_VERSION	        GFX1XM_OFFSET(0)
#define GFX1XM_CONFIG_DATA      GFX1XM_OFFSET(0x40)
#define GFX1XM_CFG_ADDR	        GFX1XM_OFFSET(0x47)
#define GFX1XM_MODE_STATUS      GFX1XM_OFFSET(0x043)
//#define GFX1XM_MIXER_DATA     GFX1XM_OFFSET(0x140)
#define GFX1XM_BUFFER_STATUS	GFX1XM_OFFSET(0x140)
#define GFX1XM_KEY_DATA         GFX1XM_OFFSET(0x142)
#define GFX1XM_NOISE_DATA       GFX1XM_OFFSET(0x144)
#define GFX1XM_LONG_PRESS_STDP  GFX1XM_OFFSET(0x146)
#define GFX1XM_BUFFER_DATA      GFX1XM_OFFSET(0x141)

#ifdef GF_PULLKEY
#define GF_FORCE_VALUE_ADDR     GFX1XM_OFFSET(0x146)
#define GF_LBSTATUS_ADDR        GFX1XM_OFFSET(0x141)
#endif

#define GFX1XM_BUF_STA_MASK     (0x1<<7)
#define	GFX1XM_BUF_STA_READY	(0x1<<7)
#define	GFX1XM_BUF_STA_BUSY     (0x0<<7)

#define	GFX1XM_IMAGE_MASK       (0x1<<6)
#define	GFX1XM_IMAGE_ENABLE     (0x1)
#define	GFX1XM_IMAGE_DISABLE	(0x0)

#define	GFX1XM_KEY_MASK	        (GFX1XM_HOME_KEY_MASK | \
                                 GFX1XM_MENU_KEY_MASK | \
                                 GFX1XM_BACK_KEY_MASK )

//home key
#define	GFX1XM_HOME_KEY_MASK	(0x1<<5)
#define	GFX1XM_HOME_KEY_ENABL   (0x1)
#define	GFX1XM_HOME_KEY_DISABLE (0x0)

#define	GFX1XM_HOME_KEY_STA     (0x1<<4)
//menu key
#define	GFX1XM_MENU_KEY_MASK    (0x1<<3)
#define	GFX1XM_MENU_KEY_ENABLE	(0x1)
#define	GFX1XM_MENU_KEY_DISABLE	(0x0)

#define	GFX1XM_MENU_KEY_STA	(0x1<<2)
//back key
#define	GFX1XM_BACK_KEY_MASK    (0x1<<1)
#define	GFX1XM_BACK_KEY_ENABLE  (0x1)
#define	GFX1XM_BACK_KEY_DISABLE (0x0)

#define	GFX1XM_BACK_KEY_STA     (0x1<<0)

#ifdef GF_PULLKEY
#define GF_LONGPRESS_MASK   (0x1<<2)
#define GF_FCE_KEY_MASK (0x1<<3)
#define GF_NVG_KEY_MASK (0x1<<6)
#define GF_NVG_KEY_STA  (0x3<<4)
#define GF_NGKEY_MASK   (GF_NVG_KEY_MASK | GF_FCE_KEY_MASK | GF_LONGPRESS_MASK)

#define NGV_VALUE_UP        0x00 
#define NGV_VALUE_DOWN  0x01
#define NGV_VALUE_LEFT  0x02
#define NGV_VALUE_RIGHT     0x03
#endif

#define	GFX1XM_IMAGE_MODE       (0x00)
#define	GFX1XM_KEY_MODE	        (0x01)
#define GFX1XM_SLEEP_MODE       (0x02)
#define GFX1XM_FF_MODE		(0x03)
#define GFX1XM_DEBUG_MODE       (0x56)

/**********************GFX1XM ops****************************/
#define GFX1XM_W                0xF0
#define GFX1XM_R                0xF1
#define GFX1XM_WDATA_OFFSET         (0x3)
#define GFX1XM_RDATA_OFFSET         (0x5)
#define GFX1XM_CFG_LEN                  (249)   /*config data length*/

/**********************************************************/

#define GFX1XM_FASYNC 		1//If support fasync mechanism.
//#undef GFX1XM_FASYNC

/*************************************************************/
struct gfx1xm_dev {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct input_dev        *input;

	struct workqueue_struct *spi_wq;
	struct work_struct     spi_work;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned		users;
	u8			*buffer;		
	u8  		buf_status;
	u8 mode;
	u8           isPowerOn;
	u8 timer_state;
	struct timer_list   gfx1xm_timer;
#ifdef GFX1XM_FASYNC
	struct  fasync_struct *async;
#endif

#if CONFIG_HAS_EARLYSUSPEND
    struct early_suspend    early_fp;
#endif
	//struct notifier_block notifier;
	//char fb_black;			  
	struct mutex fb_lock;
	struct wake_lock		gx_wake_lock;
};

/**********************IO Magic**********************/
#define  GFX1XM_IOC_MAGIC    'g'  //define magic number
struct gfx1xm_ioc_transfer {
	u8	cmd;
	u8 reserve;
	u16	addr;
	u32 len;
	#if PROCESSOR_64_BIT
	u64  buf;
	//u32  buf;
	#else
	u8 *buf;
	#endif
};
//define commands
/*read/write GFX1XM registers*/
#define  GFX1XM_IOC_CMD	_IOWR(GFX1XM_IOC_MAGIC, 1, struct gfx1xm_ioc_transfer)
#define  GFX1XM_IOC_REINIT	_IO(GFX1XM_IOC_MAGIC, 0)
#define  GFX1XM_IOC_SETSPEED	_IOW(GFX1XM_IOC_MAGIC, 2, u32)
#define  GF_IOC_POWER_OFF   _IO(GFX1XM_IOC_MAGIC, 3)
#define  GF_IOC_POWER_ON    _IO(GFX1XM_IOC_MAGIC, 4)

#define  GFX1XM_IOC_MAXNR    5

/*******************Refering to hardware platform*****************************/
#define 	GFX1XM_LDO_EN_PIN		GPIO_FPC_EN_PIN
#define 	GFX1XM_LDO_EN_PIN_M_GPIO   GPIO_MODE_00

#define 	GFX1XM_LDO2_8V_EN_PIN		(GPIO59 | 0x80000000)
#define 	GFX1XM_LDO2_8V_EN_PIN_M_GPIO   GPIO_MODE_00


#define		GFX1XM_IRQ_NUM		198
#define		GFX1XM_IRQ_PIN_NUM		198
#define 	GFX1XM_IRQ_PIN 		(GPIO198 | 0x80000000)
#define 	GFX1XM_IRQ_PIN_M_GPIO   GPIO_MODE_00
#define 	GFX1XM_IRQ_PIN_M_EINT   GPIO_MODE_04

#define 	GFX1XM_RST_PIN		(GPIO199 | 0x80000000)
#define 	GFX1XM_RST_PIN_M_GPIO   GPIO_MODE_00
#define 	GFX1XM_RST_PIN_M_DAIPCMOUT   GPIO_MODE_01

#define		GFX1XM_SCK_PIN		(GPIO66 | 0x80000000)            
#define		GFX1XM_SCK_PIN_M_GPIO	GPIO_MODE_00
#define		GFX1XM_SCK_PIN_M_SCK	GPIO_MODE_01

#define		GFX1XM_CS_PIN		(GPIO65 | 0x80000000)
#define		GFX1XM_CS_PIN_M_GPIO	GPIO_MODE_00
#define		GFX1XM_CS_PIN_M_CS	GPIO_MODE_01
	
#define		GFX1XM_MOSI_PIN		(GPIO68 | 0x80000000)
#define		GFX1XM_MOSI_PIN_M_GPIO	GPIO_MODE_00
#define		GFX1XM_MOSI_PIN_M_MOSI	GPIO_MODE_01

#define		GFX1XM_MISO_PIN		(GPIO67 | 0x80000000)
#define		GFX1XM_MISO_PIN_M_GPIO	GPIO_MODE_00
#define		GFX1XM_MISO_PIN_M_MISO	GPIO_MODE_01


inline static void gfx1xm_get_gpio_pull(int gpio_num)
{
	int state=0;

	state = mt_get_gpio_pull_enable(gpio_num);

	printk("gfx1xm_get_irq_pull(%x)\n",state);
}


/*Confure the IRQ pin for GFX1XM irq if necessary*/
inline static void gfx1xm_spi_pins_config(void)
{
	/*cs*/
    mt_set_gpio_mode(GFX1XM_CS_PIN, GFX1XM_CS_PIN_M_CS);
    mt_set_gpio_pull_enable(GFX1XM_CS_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GFX1XM_CS_PIN, GPIO_PULL_UP);
	/*sck*/
    mt_set_gpio_mode(GFX1XM_SCK_PIN, GFX1XM_SCK_PIN_M_SCK);
    mt_set_gpio_pull_enable(GFX1XM_SCK_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GFX1XM_SCK_PIN, GPIO_PULL_DOWN);
	/*miso*/

    mt_set_gpio_mode(GFX1XM_MISO_PIN, GFX1XM_MISO_PIN_M_MISO);
	//mt_set_gpio_dir(GFX1XM_MISO_PIN, GPIO_DIR_IN); 
    mt_set_gpio_pull_enable(GFX1XM_MISO_PIN, GPIO_PULL_DISABLE);
	//mt_set_gpio_pull_enable(GFX1XM_MISO_PIN, GPIO_PULL_ENABLE);
    //mt_set_gpio_pull_select(GFX1XM_MISO_PIN, GPIO_PULL_UP);

	/*mosi*/
    mt_set_gpio_mode(GFX1XM_MOSI_PIN, GFX1XM_MOSI_PIN_M_MOSI);
    mt_set_gpio_pull_enable(GFX1XM_MOSI_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GFX1XM_MOSI_PIN, GPIO_PULL_UP);

    msleep(1);
}

inline static void gfx1xm_pin_cfg(void)
{
	#if 0
	/*cs*/
    mt_set_gpio_mode(GFX1XM_CS_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GFX1XM_CS_PIN, GPIO_DIR_OUT); 
	mt_set_gpio_pull_enable(GFX1XM_CS_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_out(GFX1XM_CS_PIN, GPIO_OUT_ZERO); 			 
	/*sck*/
    mt_set_gpio_mode(GFX1XM_SCK_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GFX1XM_SCK_PIN, GPIO_DIR_OUT); 
	mt_set_gpio_pull_enable(GFX1XM_SCK_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_out(GFX1XM_SCK_PIN, GPIO_OUT_ZERO); 			 
	/*miso*/

    mt_set_gpio_mode(GFX1XM_MISO_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GFX1XM_MISO_PIN, GPIO_DIR_OUT); 
	mt_set_gpio_pull_enable(GFX1XM_MISO_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_out(GFX1XM_MISO_PIN, GPIO_OUT_ZERO); 			 

	/*mosi*/
    mt_set_gpio_mode(GFX1XM_MOSI_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GFX1XM_MOSI_PIN, GPIO_DIR_OUT); 
	mt_set_gpio_pull_enable(GFX1XM_MOSI_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_out(GFX1XM_MOSI_PIN, GPIO_OUT_ZERO); 			 

//rst
	mt_set_gpio_mode(GFX1XM_RST_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GFX1XM_RST_PIN, GPIO_DIR_OUT); 
	mt_set_gpio_pull_enable(GFX1XM_RST_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_out(GFX1XM_RST_PIN, GPIO_OUT_ZERO); 			 
	#endif
//irq
#if 0
	mt_set_gpio_mode(GFX1XM_IRQ_PIN, GPIO_MODE_00);
	mt_set_gpio_pull_enable(GFX1XM_IRQ_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_dir(GFX1XM_IRQ_PIN, GPIO_DIR_OUT); 
	mt_set_gpio_out(GFX1XM_IRQ_PIN, GPIO_OUT_ONE);


#endif
	gfx1xm_get_gpio_pull(GFX1XM_IRQ_PIN);

	mt_set_gpio_mode(GFX1XM_IRQ_PIN, GPIO_MODE_00);
	mt_set_gpio_pull_enable(GFX1XM_IRQ_PIN, GPIO_PULL_DISABLE);
	//mt_set_gpio_dir(GFX1XM_IRQ_PIN, GPIO_DIR_IN);  			 
	//mt_set_gpio_pull_enable(GFX1XM_IRQ_PIN, GPIO_PULL_UP); //must disable pull
	gfx1xm_get_gpio_pull(GFX1XM_IRQ_PIN);

}

inline static void gfx1xm_irq_cfg(void)
{
#if 0
	/*Config IRQ pin, referring to platform.*/
	mt_set_gpio_mode(GFX1XM_IRQ_PIN, GFX1XM_IRQ_PIN_M_EINT);
	mt_set_gpio_dir(GFX1XM_IRQ_PIN, GPIO_DIR_IN); 
	mt_set_gpio_pull_enable(GFX1XM_IRQ_PIN, GPIO_PULL_DISABLE);
#endif
mt_set_gpio_mode(GFX1XM_IRQ_PIN, GPIO_MODE_00);
mt_set_gpio_dir(GFX1XM_IRQ_PIN, GPIO_DIR_IN); 
mt_set_gpio_pull_enable(GFX1XM_IRQ_PIN, GPIO_PULL_DISABLE);
}


inline static void gfx1xm_miso_pullup(void)
{
	/*Config MISO pin, referring to platform.*/
    mt_set_gpio_mode(GFX1XM_MISO_PIN, GFX1XM_MISO_PIN_M_MISO);
    mt_set_gpio_pull_enable(GFX1XM_MISO_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GFX1XM_MISO_PIN, GPIO_PULL_UP);
}



inline static void gfx1xm_miso_backnal(void)
{
	/*Config miso pin, referring to platform.*/	
    mt_set_gpio_mode(GFX1XM_MISO_PIN, GFX1XM_MISO_PIN_M_MISO);
    mt_set_gpio_pull_enable(GFX1XM_MISO_PIN, GPIO_PULL_DISABLE);
    //mt_set_gpio_pull_select(GFX1XM_MISO_PIN, GPIO_PULL_UP);
}


/********************************************************************
*CPU output low level in RST pin to reset GFX1XM. This is the MUST action for GFX1XM.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
inline static void gfx1xm_hw_reset(void)
{		/*rst pin referring to samsung KIT.*/
	mt_set_gpio_mode(GFX1XM_RST_PIN, GFX1XM_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GFX1XM_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GFX1XM_RST_PIN, GPIO_OUT_ZERO); 			 
	msleep(5);  //delay for power to reset  typical:10ms max:50ms
	mt_set_gpio_out(GFX1XM_RST_PIN, GPIO_OUT_ONE);
    msleep(60);
	
}

int gfx1xm_spi_read_bytes(struct gfx1xm_dev *gfx1xm_dev,
                                u16 addr, u32 data_len, u8 *rx_buf);

int gfx1xm_spi_write_bytes(struct gfx1xm_dev *gfx1xm_dev,
                                u16 addr, u32 data_len, u8 *tx_buf);
int gfx1xm_fw_update(struct gfx1xm_dev* gfx1xm_dev, unsigned char *buf, unsigned short len);

#endif
