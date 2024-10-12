/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *	
 * MediaTek Inc. (C) 2012. All rights reserved. 
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 *
 * Version: V2.5
 * Release Date: 2015/01/21
 */

#include "tpd.h"
#include "tpd_custom_gt9xx.h"

#ifndef TPD_NO_GPIO
#include "cust_gpio_usage.h" 
#endif
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif
#if GTP_SUPPORT_I2C_DMA
#include <linux/dma-mapping.h>
#endif
#ifdef CONFIG_CREATE_SYS_INTERFACE
#include "tpd_fw.h"
#endif

#define GTP_INFO(fmt,arg...)           printk("[GTP_D] "fmt"\n",##arg)

extern struct tpd_device *tpd;
extern u8 gtp_loading_fw;

static int tpd_flag = 0; 
int tpd_halt = 0;
//static int g_tpd_debug_point = 0;
static unsigned  long report_num = 0;
static unsigned  long tp_irq_num = 0;
static unsigned  long all_up=0;

static struct goodix_point_desc  tpd_point_desc[6]={0};

static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_rw_mutex);

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if GTP_GESTURE_WAKEUP
typedef enum
{
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct i2c_client *client);
#endif

#if GTP_CHARGER_SWITCH
	#ifdef MT6573
		#define CHR_CON0	  (0xF7000000+0x2FA00)
	#else
		extern kal_bool upmu_is_chr_det(void);
	#endif
	static void gtp_charger_switch(s32 dir_update);
#endif 

#if GTP_HAVE_TOUCH_KEY
const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]	= TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
//static int tpd_calmat_local[8]	 = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#if GTP_SUPPORT_I2C_DMA
s32 i2c_dma_write(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len);
s32 i2c_dma_read(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len);

static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
#endif

s32 gtp_send_cfg(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
static void tpd_eint_interrupt_handler(void);
static int touch_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);

#ifndef MT6572
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									 kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									 kal_bool auto_umask);
#endif

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif

struct i2c_client *i2c_client_point = NULL;
static const struct i2c_device_id tpd_i2c_id[] = {{"gt9xx", 0}, {}};
static unsigned short force[] = {0, 0xBA, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};
static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("gt9xx", (0xBA >> 1))};
//static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("gt9xx", 0xBA )};

static struct i2c_driver tpd_i2c_driver =
{
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.detect = tpd_i2c_detect,
	.driver.name = "gt9xx",
	.id_table = tpd_i2c_id,
	.address_list = (const unsigned short *) forces,
};


static u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
	= {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
#if GTP_CHARGER_SWITCH
static u8 gtp_charger_config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
	= {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
#endif

#pragma pack(1)
typedef struct
{
	u16 pid;				 //product id	//
	u16 vid;				 //version id	//
} st_tpd_info;
#pragma pack()

st_tpd_info tpd_info;
u8 int_type = 0;
u32 abs_x_max = 0;
u32 abs_y_max = 0;
u8 gtp_rawdiff_mode = 0;
u8 cfg_len = 0;
u8 pnl_init_error = 0;

/* proc file system */
s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *, size_t, loff_t *);

static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.read = gt91xx_config_read_proc,
	.write = gt91xx_config_write_proc,
};

#define VELOCITY_CUSTOM
#ifdef VELOCITY_CUSTOM
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

// for magnify velocity********************************************
#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x = TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y = TPD_VELOCITY_CUSTOM_Y;
static int tpd_misc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int tpd_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
							   unsigned long arg)
{
	//char strbuf[256];
	void __user *data;

	long err = 0;

	if (_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;

			if (data == NULL)
			{
				err = -EINVAL;
				break;
			}

			if (copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}

			break;

		case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;

			if (data == NULL)
			{
				err = -EINVAL;
				break;
			}

			if (copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}

			break;

		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;

	}

	return err;
}


static struct file_operations tpd_fops =
{
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};

//**********************************************
#endif

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, "mtk-tpd");
	return 0;
}

static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	char *ptr = page;
	char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0};
	int i;
	
	if (*ppos)	// CMD call again
	{
		return 0;
	}
	
	ptr += sprintf(ptr, "==== GT9XX config init value====\n");

	for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
	{
		ptr += sprintf(ptr, "0x%02X ", config[i + 2]);

		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}

	ptr += sprintf(ptr, "\n");

	ptr += sprintf(ptr, "==== GT9XX config real value====\n");
	i2c_read_bytes(i2c_client_point, GTP_REG_CONFIG_DATA, temp_data, GTP_CONFIG_MAX_LENGTH);

	for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
	{
		ptr += sprintf(ptr, "0x%02X ", temp_data[i]);

		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}
	*ppos += ptr - page;
	return (ptr - page);
}

static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
	s32 ret = 0;

	GTP_DEBUG("write count %ld\n", count);

	if (count > GTP_CONFIG_MAX_LENGTH)
	{
		GTP_ERROR("size not match [%d:%ld]\n", GTP_CONFIG_MAX_LENGTH, count);
		return -EFAULT;
	}

	if (copy_from_user(&config[2], buffer, count))
	{
		GTP_ERROR("copy from user fail\n");
		return -EFAULT;
	}

	ret = gtp_send_cfg(i2c_client_point);
	abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
	abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
	int_type = (config[TRIGGER_LOC]) & 0x03;

	if (ret < 0)
	{
		GTP_ERROR("send config failed.");
	}

	return count;
}

#if GTP_SUPPORT_I2C_DMA
s32 i2c_dma_read(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len)
{
	int ret;
	s32 retry = 0;
	u8 buffer[2];

	struct i2c_msg msg[2] =
	{
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = buffer,
			.len = 2,
			.timing = I2C_MASTER_CLOCK
		},
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = (u8*)gpDMABuf_pa,	 
			.len = len,
			.timing = I2C_MASTER_CLOCK
		},
	};
	
	buffer[0] = (addr >> 8) & 0xFF;
	buffer[1] = addr & 0xFF;

	if (rxbuf == NULL)
		return -1;

	//GTP_DEBUG("dma i2c read: 0x%04X, %d bytes(s)", addr, len);
	for (retry = 0; retry < I2C_RETRY_TIMES; ++retry) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0) {
			continue;
		}
		memcpy(rxbuf, gpDMABuf_va, len);
		return 0;
	}
	GTP_ERROR("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
	return ret;
}


s32 i2c_dma_write(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len)
{
	int ret;
	s32 retry = 0;
	u8 *wr_buf = gpDMABuf_va;
	
	struct i2c_msg msg =
	{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = (u8*)gpDMABuf_pa,
		.len = 2 + len,
		.timing = I2C_MASTER_CLOCK
	};
	
	wr_buf[0] = (u8)((addr >> 8) & 0xFF);
	wr_buf[1] = (u8)(addr & 0xFF);

	if (txbuf == NULL)
		return -1;
	
	//GTP_DEBUG("dma i2c write: 0x%04X, %d bytes(s)", addr, len);
	memcpy(wr_buf+2, txbuf, len);
	for (retry = 0; retry < I2C_RETRY_TIMES; ++retry) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			continue;
		}
		return 0;
	}
	GTP_ERROR("Dma I2C Write Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
	return ret;
}

s32 i2c_read_bytes_dma(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len)
{
	s32 left = len;
	s32 read_len = 0;
	u8 *rd_buf = rxbuf;
	s32 ret = 0;

	mutex_lock(&i2c_rw_mutex);
	//GTP_DEBUG("Read bytes dma: 0x%04X, %d byte(s)", addr, len);
	while (left > 0) {
		if (left > GTP_DMA_MAX_TRANSACTION_LENGTH) {
			read_len = GTP_DMA_MAX_TRANSACTION_LENGTH;
		} else {
			read_len = left;
		}
		ret = i2c_dma_read(client, addr, rd_buf, read_len);
		if (ret < 0) {
			GTP_ERROR("dma read failed");
			ret = -1;
			goto out;
		}
		
		left -= read_len;
		addr += read_len;
		rd_buf += read_len;
	}
	ret = 0;
out:
	mutex_unlock(&i2c_rw_mutex);
	return ret;
}

s32 i2c_write_bytes_dma(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len)
{

	s32 ret = 0;
	s32 write_len = 0;
	s32 left = len;
	u8 *wr_buf = txbuf;

	mutex_lock(&i2c_rw_mutex);
	//GTP_DEBUG("Write bytes dma: 0x%04X, %d byte(s)", addr, len);
	while (left > 0) {
		if (left > GTP_DMA_MAX_I2C_TRANSFER_SIZE)
		{
			write_len = GTP_DMA_MAX_I2C_TRANSFER_SIZE;
		} else {
			write_len = left;
		}
		ret = i2c_dma_write(client, addr, wr_buf, write_len);
		if (ret < 0) {
			GTP_ERROR("dma i2c write failed!");
			ret = -1;
			goto out;
		}
		
		left -= write_len;
		addr += write_len;
		wr_buf += write_len;
	}
	ret = 0;
out:
	mutex_unlock(&i2c_rw_mutex);
	return ret;
}
#endif


int i2c_read_bytes_non_dma(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	u8 buffer[GTP_ADDR_LENGTH];
	u8 retry;
	u16 left = len;
	u16 offset = 0;
	int retval = -1;
	struct i2c_msg msg[2] =
	{
		{
			.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
			//.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
			.flags = 0,
			.buf = buffer,
			.len = GTP_ADDR_LENGTH,
			.timing = I2C_MASTER_CLOCK
		},
		{
			.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
			//.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
			.flags = I2C_M_RD,
			.timing = I2C_MASTER_CLOCK
		},
	};

	mutex_lock(&i2c_rw_mutex);

	if (rxbuf == NULL) {
		retval = -1;
		goto out;
	}

	//GTP_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

	while (left > 0) {
		buffer[0] = ((addr + offset) >> 8) & 0xFF;
		buffer[1] = (addr + offset) & 0xFF;

		msg[1].buf = &rxbuf[offset];

		if (left > MAX_TRANSACTION_LENGTH) {
			msg[1].len = MAX_TRANSACTION_LENGTH;
			left -= MAX_TRANSACTION_LENGTH;
			offset += MAX_TRANSACTION_LENGTH;
		} else {
			msg[1].len = left;
			left = 0;
		}

		retry = 0;
		while (i2c_transfer(client->adapter, &msg[0], 2) != 2) {
			retry++;
			//if (retry == 20)
			if (retry == I2C_RETRY_TIMES) {
				GTP_ERROR("I2C read 0x%X length=%d failed\n", addr + offset, len);
				retval = -1;
				goto out;
			}
		}
	}
	retval = 0;
out:
	mutex_unlock(&i2c_rw_mutex);
	return retval;
}

int i2c_write_bytes_non_dma(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
	u8 buffer[MAX_TRANSACTION_LENGTH];
	u16 left = len;
	u16 offset = 0;
	u8 retry = 0;
	int retval = -1;
	struct i2c_msg msg =
	{
		.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
		//.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
		.flags = 0,
		.buf = buffer,
		.timing = I2C_MASTER_CLOCK,
	};

	mutex_lock(&i2c_rw_mutex);
	
	if (txbuf == NULL){
		retval = -1;
		goto out;
	}
	//GTP_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

	while (left > 0)
	{
		retry = 0;

		buffer[0] = ((addr + offset) >> 8) & 0xFF;
		buffer[1] = (addr + offset) & 0xFF;

		if (left > MAX_I2C_TRANSFER_SIZE) {
			memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], MAX_I2C_TRANSFER_SIZE);
			msg.len = MAX_TRANSACTION_LENGTH;
			left -= MAX_I2C_TRANSFER_SIZE;
			offset += MAX_I2C_TRANSFER_SIZE;
		} else {
			memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], left);
			msg.len = left + GTP_ADDR_LENGTH;
			left = 0;
		}

		//GTP_DEBUG("byte left %d offset %d\n", left, offset);
		while (i2c_transfer(client->adapter, &msg, 1) != 1) {
			retry++;

			//if (retry == 20)
			if (retry == I2C_RETRY_TIMES) {
				GTP_ERROR("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
				retval = -1;
				goto out;
			}
		}
	}
	retval = 0;
out:
	mutex_unlock(&i2c_rw_mutex);
	return retval;
}

int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
#if GTP_SUPPORT_I2C_DMA
	return i2c_write_bytes_dma(client, addr, txbuf, len);
#else
	return i2c_write_bytes_non_dma(client, addr, txbuf, len);
#endif
}

int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
#if GTP_SUPPORT_I2C_DMA
	return i2c_read_bytes_dma(client, addr, rxbuf, len);
#else
	return i2c_read_bytes_non_dma(client, addr, rxbuf, len);
#endif
}

s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
	s32 ret = -1;
	u16 addr = (buf[0] << 8) + buf[1];

	ret = i2c_write_bytes(client, addr, &buf[2], len - 2);
	if (!ret) {
		return 1;
	} else 
	{
#if GTP_GESTURE_WAKEUP
		if (DOZE_ENABLED == doze_status) {
			return ret;
		}
#endif

			gtp_reset_guitar(client, 20);

		return ret;
	}
}

s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
	s32 ret = -1;
	u16 addr = (buf[0] << 8) + buf[1];

	ret = i2c_read_bytes(client, addr, &buf[2], len - 2);
	if (!ret) {
		return 2;
	} else {
#if GTP_GESTURE_WAKEUP
		if (DOZE_ENABLED == doze_status) {
			return ret;
		}
#endif
			gtp_reset_guitar(client, 20);

		return ret;
	}
}

s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	u8 retry = 0;
	
	while (retry++ < 3)
	{
		memset(buf, 0xAA, 16);
		buf[0] = (u8)(addr >> 8);
		buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, buf, len + 2);
		
		memset(confirm_buf, 0xAB, 16);
		confirm_buf[0] = (u8)(addr >> 8);
		confirm_buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, confirm_buf, len + 2);
		
		if (!memcmp(buf, confirm_buf, len+2))
		{
			memcpy(rxbuf, confirm_buf+2, len);
			return SUCCESS;
		}
	}	 
	GTP_ERROR("i2c read 0x%04X, %d bytes, double check failed!\n", addr, len);
	return FAIL;
}

static int tpd_get_tplcd_res(struct goodix_ts_data *goodix_data)
{ 
	int tp_resx = 0, tp_resy = 0;
	u8 buf[8] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
	int ret = -1;
	
	ret = gtp_i2c_read(goodix_data->i2c_client, buf, sizeof(buf));
	if(ret < 0) {
		GTP_ERROR("Read tpd res failed.");
		tp_resx = TPD_RES_X;
		tp_resy = TPD_RES_Y;
	} else {
		tp_resx = buf[4] << 8 | buf[3];
		tp_resy = buf[6] << 8 | buf[5];
	}
	GTP_INFO("Enter %s tp (%d, %d) lcd(%d, %d).\n", __func__, tp_resx, tp_resy, TPD_RES_X, TPD_RES_Y);
	goodix_data->tp_resx = tp_resx;
	goodix_data->tp_resy = tp_resy;
	goodix_data->lcd_resx = TPD_RES_X;
	goodix_data->lcd_resy = TPD_RES_Y;

	return 0;
}

#ifdef CONFIG_CREATE_SYS_INTERFACE
#ifdef CONFIG_PROC_FS
#define	TOUCH_PROC_FILE	"driver/tpd_touch"
static struct proc_dir_entry *tpd_debug_proc_entry;

static ssize_t proc_read_val(struct file *file,
	char __user *buffer, size_t count, loff_t *offset)
{
	ssize_t len = 0;
	char data[800];
	//unsigned char uc_reg_value = 0;
	struct goodix_ts_data *ts_data = i2c_get_clientdata(i2c_client_point);
	int rst_gpio_dir, rst_gpio_pullen, rst_gpio_pullsel, rst_gpio_inver, rst_gpio_outval, rst_gpio_inval, rst_gpio_mode;
	int int_gpio_dir, int_gpio_pullen, int_gpio_pullsel, int_gpio_inver, int_gpio_outval, int_gpio_inval, int_gpio_mode;

	len += sprintf(data + len, "Goodix Touchscreen 20150728.\n");
	len += sprintf(data + len, "tpd is suspend:%d , need stay awake:%d , bsg enable? :%d .\n", ts_data->tpd_suspend, ts_data->need_stay_awake, 
		ts_data->enable_wakeup_gesture);
	len += sprintf(data + len, "tp (%d, %d) lcd(%d, %d).\n", ts_data->tp_resx,  ts_data->tp_resy, 
		ts_data->lcd_resx, ts_data->lcd_resy);
	
	rst_gpio_dir = mt_get_gpio_dir(GPIO_CTP_RST_PIN);
	rst_gpio_pullen = mt_get_gpio_pull_enable(GPIO_CTP_RST_PIN);
	rst_gpio_pullsel = mt_get_gpio_pull_select(GPIO_CTP_RST_PIN);
	rst_gpio_inver = mt_get_gpio_inversion(GPIO_CTP_RST_PIN);
	rst_gpio_outval = mt_get_gpio_out(GPIO_CTP_RST_PIN);
	rst_gpio_inval = mt_get_gpio_in(GPIO_CTP_RST_PIN);
	rst_gpio_mode = mt_get_gpio_dir(GPIO_CTP_RST_PIN);
	
	int_gpio_dir = mt_get_gpio_dir(GPIO_CTP_EINT_PIN);
	int_gpio_pullen = mt_get_gpio_pull_enable(GPIO_CTP_EINT_PIN);
	int_gpio_pullsel = mt_get_gpio_pull_select(GPIO_CTP_EINT_PIN);
	int_gpio_inver = mt_get_gpio_inversion(GPIO_CTP_EINT_PIN);
	int_gpio_outval = mt_get_gpio_out(GPIO_CTP_EINT_PIN);
	int_gpio_inval = mt_get_gpio_in(GPIO_CTP_EINT_PIN);
	int_gpio_mode = mt_get_gpio_dir(GPIO_CTP_EINT_PIN);
	
	len += sprintf(data + len, "rst pin 0x%x, mode:%d, dir:%d, pullen:%d, sel:%d, inver:%d, outval:%d, in:%d.\n",
		GPIO_CTP_RST_PIN, rst_gpio_mode, rst_gpio_dir, rst_gpio_pullen, rst_gpio_pullsel, rst_gpio_inver, rst_gpio_outval, rst_gpio_inval);
	len += sprintf(data + len, "int pin 0x%x, mode:%d, dir:%d, pullen:%d, sel:%d, inver:%d, outval:%d, in:%d.\n",
		GPIO_CTP_EINT_PIN, int_gpio_mode, int_gpio_dir, int_gpio_pullen, int_gpio_pullsel, int_gpio_inver, int_gpio_outval, int_gpio_inval);

	return simple_read_from_buffer(buffer, count, offset, data, len);
}

static ssize_t proc_write_val(struct file *filp,
					 const char *buff, size_t len,
					 loff_t * off)
{
	return len;
}

static struct file_operations tpd_touch_proc_ops = {
	.read = proc_read_val,
	.write = proc_write_val,
};

static void create_tpd_debug_proc_entry(void *data)
{
	tpd_debug_proc_entry = proc_create(TOUCH_PROC_FILE, 0644, NULL, &tpd_touch_proc_ops);
	if (tpd_debug_proc_entry) {
		//tpd_debug_proc_entry->data = (void*)data;
		//g_data = data;
		printk(KERN_INFO "create proc file sucess!\n");
	} else
		printk(KERN_INFO "create proc file failed!\n");
}
#endif
s32 gtp_read_version(struct i2c_client *client, u16 *version);
static int gtp_read_cfg_version(struct i2c_client *client, u16* version)
{
	int ret = 0;
	u8 buf[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

	//GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version) {
		*version = buf[2];
	}
	GTP_INFO("cfg version: 0x%x\n", *version);

	return ret;
}
static int tpd_init_tpinfo(struct tpd_classdev_t *cdev)
{
	struct goodix_ts_data *data = (struct goodix_ts_data*) cdev->private;
	u16 version_info = 0x00;
	u16 version_cfg_info = 0x00;

	gtp_read_version(data->i2c_client, &version_info);
	gtp_read_cfg_version(data->i2c_client, &version_cfg_info);

	if(9157 == data->chip_id ||970 ==data->chip_id ||615 == data->chip_id ||915 == data->chip_id  ){
	}else{
		data->fw_version = 0;
	}
	strcpy(cdev->ic_tpinfo.tp_name, "Goodix");
	cdev->ic_tpinfo.chip_id = data->chip_id;
	cdev->ic_tpinfo.vendor_id = 0x05;    //data->vendor_id;;  //truly tp
	cdev->ic_tpinfo.chip_ver = version_cfg_info;
	cdev->ic_tpinfo.firmware_ver= data->fw_version;
	cdev->ic_tpinfo.i2c_type = 0;
	cdev->ic_tpinfo.i2c_addr = data->i2c_client->addr;

	tpd_get_tplcd_res(data);
	
	return 0;
}

extern int tpd_goodix_update_proc(struct tpd_classdev_t *cdev, unsigned char * data, unsigned int size);
extern int tpd_goodix_compare_fw(struct tpd_classdev_t *cdev, unsigned char * data, unsigned int size, int force_upg);
static int tpd_compare_tp(struct tpd_classdev_t *cdev, unsigned char *data)
{
	return tpd_goodix_compare_fw(cdev, data, cdev->tp_fw.size, 0);
}
/*retval: 0:success -1:failed 0xff:noneed update*/
static int tpd_flash_firmware(struct tpd_classdev_t *cdev, unsigned char * data, unsigned int size, int force_upg)
{
	int  i_ret = -1;
	struct goodix_ts_data *goodix_data = (struct goodix_ts_data*) cdev->private;
	
//#ifdef TPD_UPDATE_FIRMWARE
	GTP_INFO("%s in, fw size is:0x%x, force:0x%x.\n", __func__, size, force_upg);
	if(cdev == NULL || data == NULL || 0 == size){
		GTP_INFO("%s maybe read sdcard failed\n", __func__);
		return -1;
	}
	goodix_data->need_stay_awake = true;
	if(goodix_data->tpd_suspend) {
		printk("tpd %s ts in suspend mode, wait 5 sec \n", __func__);
		msleep(5000);
		if(goodix_data->tpd_suspend) {
			printk("tpd %s ts still in suspend mode, return. \n", __func__);
			i_ret = 0;
			goto out;
		}
	}

	i_ret = tpd_goodix_compare_fw(cdev, data, cdev->tp_fw.size, force_upg);
	if( i_ret != 0)	{
		GTP_INFO("%s line:%d RMI check firmware finished, return...\n", __func__, __LINE__);
		goto out;
	}

	i_ret = tpd_goodix_update_proc(cdev, data, size);	
	if (i_ret == 0) {
		GTP_INFO("upgrade to new version\n");
	} else {
		GTP_INFO("Upgrade firmware failed, try again!\n");
		gtp_reset_guitar(goodix_data->i2c_client, 10);
		i_ret = tpd_goodix_update_proc(cdev, data, size);
		if( 0 != i_ret )	{
			GTP_INFO("upgrade failed ret=%d.\n", i_ret);	
		} else {
			GTP_INFO("Upgrade firware success at second try.\n");
		}
	}
	tpd_init_tpinfo(cdev);

out:
//#endif
	goodix_data->need_stay_awake = false;
	return i_ret;
}

static int tpd_read_block(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len)
{
	int ret = 0;
	int i = 0;
	struct goodix_ts_data *data = (struct goodix_ts_data*) cdev->private;

	ret = i2c_read_bytes(data->i2c_client, addr, buf, len);
	GTP_INFO("Read from addr:0x%x val=", addr);
	for(i = 0; i < (len < 8? len : 8); i++)
	{
		GTP_INFO("0x%x ", buf[i]);
	}
	GTP_INFO("\n");
	
	return ret;
}

static int tpd_write_block(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len)
{
	int ret = 0;
	struct goodix_ts_data *data = (struct goodix_ts_data*) cdev->private;

	ret = i2c_write_bytes(data->i2c_client, addr, buf, len);
	
	return ret;
}
static int tpd_get_gesture(struct tpd_classdev_t *cdev)
{
	struct goodix_ts_data *data = (struct goodix_ts_data*) cdev->private;
	GTP_INFO("%s bsg_gesture %d.\n", __func__, data->bsg_gesture);
	
	return data->bsg_gesture;
}
static int tpd_set_enable_bsg_gesture(struct tpd_classdev_t *cdev, int enable)
{
	struct goodix_ts_data *data = (struct goodix_ts_data*) cdev->private;
	GTP_INFO("%s val %d.\n", __func__, enable);

	data->enable_wakeup_gesture= enable;
	
	return enable;
}

static int tpd_register_fw_class(struct goodix_ts_data *data)
{	
	tpd_fw_cdev.name = "touchscreen";
	tpd_fw_cdev.private = (void*)data;
	tpd_fw_cdev.flash_fw = tpd_flash_firmware;
	tpd_fw_cdev.read_block = tpd_read_block;
	tpd_fw_cdev.write_block = tpd_write_block;
	tpd_fw_cdev.compare_tp = tpd_compare_tp;
	tpd_fw_cdev.get_tpinfo = tpd_init_tpinfo;
	
	tpd_fw_cdev.get_gesture = tpd_get_gesture;
	tpd_fw_cdev.wake_gesture = tpd_set_enable_bsg_gesture;
	//for black wakeup gesture.
	tpd_classdev_register(&(data->i2c_client->dev), &tpd_fw_cdev);
	tpd_init_tpinfo(&tpd_fw_cdev);
	
#ifdef CONFIG_PROC_FS
	create_tpd_debug_proc_entry(data);
#endif
	return 0;
}
#endif

/*******************************************************
Function:
	Send config Function.

Input:
	client: i2c client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
	s32 ret = 1;

#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;

	if (pnl_init_error)
	{
		GTP_INFO("Error occurred in init_panel, no config sent!");
		return 0;
	}
	
	GTP_INFO("Driver Send Config");
	for (retry = 0; retry < 5; retry++)
	{
		ret = gtp_i2c_write(client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);

		if (ret > 0)
		{
			break;
		}
	}
#endif
	return ret;
}
#if GTP_CHARGER_SWITCH
static int gtp_send_chr_cfg(struct i2c_client *client)
{
	s32 ret = 1;
#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;

	if (pnl_init_error) {
		GTP_INFO("Error occurred in init_panel, no config sent!");
		return 0;
	}
	
	GTP_INFO("Driver Send Config");
	for (retry = 0; retry < 5; retry++) {
		ret = gtp_i2c_write(client, gtp_charger_config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0) {
			break;
		}
	}
#endif	
	return ret;
}
#endif
static u8 ascii2hex(u8 a)
{
	s8 value = 0;

	if(a >= '0' && a <= '9') {
		value = a - '0';
	} else if (a >= 'A' && a <= 'F') {
		value = a - 'A' + 0x0A;
	} else if (a >= 'a' && a <= 'f') {
		value = a - 'a' + 0x0A;
	} else {
		value = 0xff;
	}
	
	return value;
}
/*******************************************************
Function:
	Read goodix touchscreen version function.

Input:
	client: i2c client struct.
	version:address to store version info

Output:
	Executive outcomes.0---succeed.
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16 *version)
{
	s32 ret = -1;
	s32 i;
	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};
	struct goodix_ts_data *data = i2c_get_clientdata(client);

	//GTP_DEBUG_FUNC();
	GTP_INFO("Enter  %s,LINE=%d",__func__,__LINE__);
	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version) {
		*version = (buf[7] << 8) | buf[6];
	}
	data->fw_version = *version;

	tpd_info.vid = *version;
	tpd_info.pid = 0x00;

	for (i = 0; i < 4; i++) {
		if (buf[i + 2] < 0x30)break;

		tpd_info.pid |= ((buf[i + 2] - 0x30) << ((3 - i) * 4));
	}

	if (buf[5] == 0x00) {
		data->chip_id = ascii2hex(buf[2]) * 100 + ascii2hex(buf[3]) * 10 + ascii2hex(buf[4]);
		GTP_INFO("IC VERSION_1: %c%c%c_%02x%02x",
			 buf[2], buf[3], buf[4], buf[7], buf[6]);  
	} else {
		data->chip_id = ascii2hex(buf[2]) * 1000 + ascii2hex(buf[3]) * 100 + ascii2hex(buf[4]) * 10 + ascii2hex(buf[5]);
		GTP_INFO("IC VERSION_2:%c%c%c%c_%02x%02x",
			 buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
	}
	return ret;
}

/*******************************************************
Function:
	GTP initialize function.

Input:
	client: i2c client private struct.

Output:
	Executive outcomes.0---succeed.
*******************************************************/
static s32 gtp_init_panel(struct i2c_client *client)
{
	s32 ret = 0;

#if GTP_DRIVER_SEND_CFG
	s32 i;
	u8 check_sum = 0;
	u8 opr_buf[16];
	u8 sensor_id = 0;
	 u8 drv_cfg_version;
	 u8 flash_cfg_version;

	u8 cfg_info_group0[] = CTP_CFG_GROUP0;
	u8 cfg_info_group1[] = CTP_CFG_GROUP1;
	u8 cfg_info_group2[] = CTP_CFG_GROUP2;
	u8 cfg_info_group3[] = CTP_CFG_GROUP3;
	u8 cfg_info_group4[] = CTP_CFG_GROUP4;
	u8 cfg_info_group5[] = CTP_CFG_GROUP5;
	u8 *send_cfg_buf[] = {cfg_info_group0, cfg_info_group1, cfg_info_group2,
						cfg_info_group3, cfg_info_group4, cfg_info_group5};
	u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group0), 
						  CFG_GROUP_LEN(cfg_info_group1),
						  CFG_GROUP_LEN(cfg_info_group2),
						  CFG_GROUP_LEN(cfg_info_group3), 
						  CFG_GROUP_LEN(cfg_info_group4),
						  CFG_GROUP_LEN(cfg_info_group5)};
	GTP_INFO("Enter  %s,%d",__func__,__LINE__);
	GTP_INFO("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d", 
		cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
		cfg_info_len[4], cfg_info_len[5]);

	ret = gtp_i2c_read_dbl_check(client, GTP_REG_SENSOR_ID, &sensor_id, 1);
	if (SUCCESS == ret) {
		if (sensor_id >= 0x06) {
			GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
		}
	} else {
		GTP_ERROR("Failed to get sensor_id, No config sent!");
	}
	GTP_INFO("Sensor_ID: %d", sensor_id);

	if ((!cfg_info_len[1]) && (!cfg_info_len[2]) && 
		(!cfg_info_len[3]) && (!cfg_info_len[4]) && 
		(!cfg_info_len[5]))
	{
		sensor_id = 0; 
	} else {
		ret = gtp_i2c_read_dbl_check(client, GTP_REG_SENSOR_ID, &sensor_id, 1);
		if (SUCCESS == ret) {
			if (sensor_id >= 0x06) {
				GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
				pnl_init_error = 1;
				return -1;
			}
		} else {
			GTP_ERROR("Failed to get sensor_id, No config sent!");
			pnl_init_error = 1;
			return -1;
		}
		GTP_INFO("Enter  %s,%d,Sensor_ID: %d\n",__func__,__LINE__,sensor_id);
	}
	
	cfg_len = cfg_info_len[sensor_id];
	
	GTP_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id, cfg_len);
	
	if (cfg_len < GTP_CONFIG_MIN_LENGTH)
	{
		GTP_ERROR("CTP_CONFIG_GROUP%d is INVALID CONFIG GROUP! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id);
		pnl_init_error = 1;
		return -1;
	}
		{
		ret = gtp_i2c_read_dbl_check(client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);	  
		if (ret == SUCCESS)
		{
			GTP_DEBUG("CFG_CONFIG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id, 
						send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);
	
			flash_cfg_version = opr_buf[0];
			drv_cfg_version = send_cfg_buf[sensor_id][0];		// backup  config version
			
			if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {
				send_cfg_buf[sensor_id][0] = 0x00;
			}
		}
		else
		{
			GTP_ERROR("Failed to get ic config version!No config sent!");
			return -1;
		}
	}  
	memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], cfg_len);

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < cfg_len; i++)
	{
		check_sum += config[i];
	}
	config[cfg_len] = (~check_sum) + 1;  
#else // DRIVER NOT SEND CONFIG
	cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret = gtp_i2c_read(client, config, cfg_len + GTP_ADDR_LENGTH);
	if (ret < 0)
	{
		GTP_ERROR("Read Config Failed, Using DEFAULT Resolution & INT Trigger!");
		abs_x_max = GTP_MAX_WIDTH;
		abs_y_max = GTP_MAX_HEIGHT;
		int_type = GTP_INT_TRIGGER;
	}
#endif // GTP_DRIVER_SEND_CFG

	GTP_DEBUG_FUNC();
	if ((abs_x_max == 0) && (abs_y_max == 0))
	{
		abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
		abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
		int_type = (config[TRIGGER_LOC]) & 0x03; 
	}
		{
#if GTP_DRIVER_SEND_CFG
		ret = gtp_send_cfg(client);
		if (ret < 0)
		{
			GTP_ERROR("Send config error.");
		}
	  {
		/* for resume to send config */
		if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {		
			config[GTP_ADDR_LENGTH] = drv_cfg_version;
			check_sum = 0;
			for (i = GTP_ADDR_LENGTH; i < cfg_len; i++)
			{
				check_sum += config[i];
			}
			config[cfg_len] = (~check_sum) + 1;
		}
	  }
#endif
		GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
			abs_x_max,abs_y_max,int_type);
	}
	
	msleep(10);
	return 0;
}

static s8 gtp_i2c_test(struct i2c_client *client)
{

	u8 retry = 0;
	s8 ret = -1;
	u32 hw_info = 0;

	//GTP_DEBUG_FUNC();

	while (retry < 3) {
		ret = i2c_read_bytes(client, GTP_REG_HW_INFO, (u8 *)&hw_info, sizeof(hw_info));
		if ((!ret) && (hw_info == 0x00900600)) {				//20121212
			return ret;
		}

		GTP_ERROR("GTP_REG_HW_INFO : %08X\n", hw_info);
		GTP_ERROR("GTP i2c test failed time %d.\n", retry);
		msleep(10);
		retry++;
	}

	return -1;
}



/*******************************************************
Function:
	Set INT pin  as input for FW sync.

Note:
  If the INT is high, It means there is pull up resistor attached on the INT pin.
  Pull low the INT pin manaully for FW sync.
*******************************************************/
void gtp_int_sync(s32 ms)
{
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(ms);
	GTP_GPIO_AS_INT(GTP_INT_PORT);
}

void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
	GTP_INFO("GTP RESET!,%d\n",(client->addr == 0x14));
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(ms);
	GTP_GPIO_OUTPUT(GTP_INT_PORT, client->addr == 0x14);

	msleep(2);
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

	msleep(6);						//must >= 6ms


	gtp_int_sync(50); 
}

static int tpd_power_on(struct i2c_client *client)
{
	int ret = 0;
	//int reset_count = 0;

	GTP_INFO("Enter %s, %d,client-address=0x%x\n", __FUNCTION__, __LINE__,client->addr);

//reset_proc:
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);	
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(10);

#ifdef MT6573
	// power on CTP
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#else	// ( defined(MT6575) || defined(MT6577) || defined(MT6589) )
#ifdef TPD_POWER_SOURCE_CUSTOM							 
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_3000, "TP");    
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif
#endif

	gtp_reset_guitar(client, 20);
	{
		ret = gtp_i2c_test(client);
		if (ret < 0) {
			GTP_ERROR("I2C communication ERROR!");
			//if (reset_count < TPD_MAX_RESET_COUNT) {
			//	reset_count++;
			//	goto reset_proc;
			//}
		}
	}
	 GTP_INFO("Exit %s, %d\n", __FUNCTION__, __LINE__);
	return ret;
}

//**************** For GT9XXF Start ********************//
//************* For GT9XXF End **********************//
static s32 tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 err = 0;
	s32 ret = 0;
	u16 version_info;
#if GTP_HAVE_TOUCH_KEY
	int idx = 0;
#endif
	struct goodix_ts_data *goodix_data;

	GTP_INFO("Enter %s, %d\n", __FUNCTION__, __LINE__);

	goodix_data = kzalloc(sizeof(*goodix_data), GFP_KERNEL);
	if (!goodix_data) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for goodix_data\n",
				__func__);
		return -ENOMEM;
	}
#if GTP_SUPPORT_I2C_DMA
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	if(!gpDMABuf_va){
		GTP_INFO("[Error] Allocate DMA I2C Buffer failed!\n");
	}
	memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
#endif

	goodix_data->i2c_client = client;
	i2c_set_clientdata(client, goodix_data);

	i2c_client_point = client;

	tpd_power_already_on = 1;
	ret = tpd_power_on(client);
	if (ret < 0) {
		GTP_ERROR("I2C communication ERROR!");
		goto error_i2c_failed;
	}
	
#ifdef VELOCITY_CUSTOM
	if ((err = misc_register(&tpd_misc_device))) {
		printk("mtk_tpd: tpd_misc_device register failed\n");
	}

#endif
	ret = gtp_read_version(client, &version_info);
	if (ret < 0) {
		GTP_ERROR("Read version failed.");
	}	 
	
	ret = gtp_init_panel(client);
	if (ret < 0) {
		GTP_ERROR("GTP init panel failed.");
	}
	
	// Create proc file system
	gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0640, NULL, &config_proc_ops);
	if (gt91xx_config_proc == NULL) {
		GTP_ERROR("create_proc_entry %s failed\n", GT91XX_CONFIG_PROC_FILE);
	} else {
		GTP_INFO("create proc entry %s success", GT91XX_CONFIG_PROC_FILE);
	}

#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		GTP_INFO(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}
	
#if GTP_HAVE_TOUCH_KEY
	for (idx = 0; idx < GTP_MAX_KEY_NUM; idx++) {
		input_set_capability(tpd->dev, EV_KEY, touch_key_array[idx]);
	}
#endif
	goodix_data->need_stay_awake = false;
        #if GTP_GESTURE_WAKEUP
	goodix_data->enable_wakeup_gesture = false;
	goodix_data->tpd_suspend = false;
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOUBLE_CLICK); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_UP); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_LEFT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_RIGHT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_C); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_E); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_M); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_U); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_Z);
#endif
	// set INT mode
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);

	msleep(50);

#ifdef MT6572
	if (!int_type) {	//EINTF_TRIGGER
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);
	} else {
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	}
#else
	// mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, MT_LEVEL_SENSITIVE);
	//mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);

	if (!int_type) {
		//mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1);
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM,  EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);
	} else {
		//mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, tpd_eint_interrupt_handler, 1);
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	}
#endif
	//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	 mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#if GTP_AUTO_UPDATE
	ret = gup_init_update_proc(client);
	if (ret < 0) {
		GTP_ERROR("Create update thread error.");
	}
#endif
#ifdef CONFIG_CREATE_SYS_INTERFACE
	tpd_register_fw_class(goodix_data);
#endif
	tpd_get_tplcd_res(goodix_data);

	tpd_load_status = 1;
	GTP_INFO("Exit %s, %d\n", __FUNCTION__, __LINE__);
	return 0;

error_i2c_failed:
	kfree(goodix_data);
	if(gpDMABuf_va){
		dma_free_coherent(&tpd->dev->dev, GTP_DMA_MAX_TRANSACTION_LENGTH,  gpDMABuf_va, gpDMABuf_pa);
		gpDMABuf_va = NULL;
		gpDMABuf_pa = 0;
		GTP_INFO("[DMA][release] Allocate DMA I2C Buffer release!\n");
	}
	return -1;
}

static void tpd_eint_interrupt_handler(void)
{
	TPD_DEBUG_PRINT_INT;
	
	tpd_flag = 1;
	//g_tpd_debug_point ++;
	tp_irq_num++;

	wake_up_interruptible(&waiter);
}
static int tpd_i2c_remove(struct i2c_client *client)
{
#if GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

	return 0;
}
#if (GTP_ESD_PROTECT || GTP_COMPATIBLE_MODE)
static void force_reset_guitar(void)
{
	s32 i = 0;
	s32 ret = 0;

	GTP_INFO("force_reset_guitar");
	
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);	
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
#ifdef MT6573
	//Power off TP
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);  
	msleep(30);
	//Power on TP
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(30);
#else			// ( defined(MT6575) || defined(MT6577) || defined(MT6589) )
	// Power off TP
	#ifdef TPD_POWER_SOURCE_CUSTOM
		hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
	#else
		hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
	#endif
		msleep(30); 

	// Power on TP
	#ifdef TPD_POWER_SOURCE_CUSTOM
		hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
	#else
		hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	#endif
		msleep(30);

#endif

	for (i = 0; i < 5; i++)
	{
		{
			//Reset Guitar
			gtp_reset_guitar(i2c_client_point, 20);
			msleep(50);
			//Send config
			ret = gtp_send_cfg(i2c_client_point);
	
			if (ret < 0)
			{
				continue;
			}
		}
		break;
	}
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	
	if (i >= 5)
	{
		GTP_ERROR("Failed to reset guitar.");
		return;
	}
	GTP_INFO("Esd recovery successful");
	return;
}
#endif


static void tpd_down(s32 x, s32 y, s32 size, s32 id)
{
	if ((!size) && (!id))
	{
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	}
	else
	{
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
		/* track id Start 0 */
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	}

	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, id, 1);

#if (defined(MT6575)||defined(MT6577))

	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
	{
		tpd_button(x, y, 1);
	}

#endif

 #if  1 /*for debug*/
     tpd_point_desc[id].downnum++;
     tpd_point_desc[id].x=x;
     tpd_point_desc[id].y=y;
     tpd_point_desc[id].finger_status=1;
#endif

}

static void tpd_up(s32 x, s32 y, s32 id)
{
       int tpindex=0;
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, id, 0);

#if (defined(MT6575) || defined(MT6577))

	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
	{
		tpd_button(x, y, 0);
	}

#endif

       #if	1	/*for debug*/
	for(tpindex=0;tpindex<5;tpindex++)
	{
		if(1==tpd_point_desc[tpindex].finger_status)
		{
	tpd_point_desc[tpindex].finger_status=0;
		      GTP_INFO("[UP],ID:[%d],LO:[%d,%d],T:[%d] Up\n",tpindex,tpd_point_desc[tpindex].x,tpd_point_desc[tpindex].y,tpd_point_desc[tpindex].downnum);
		}
	tpd_point_desc[tpindex].downnum=0;
	tpd_point_desc[tpindex].x=0;
	tpd_point_desc[tpindex].y=0;

	}
	all_up=1;
	 //GTP_INFO("[ALL],All up\n");
       #endif

}


static void tpd_blackwakeup_func(struct goodix_ts_data *data, u8 gesture)
{
	unsigned int code = KEY_RESERVED;

	//a,b,c,d,e,g,h,m,o,q,s,v,w,y,z,^
	if ((gesture == 'c') || (gesture == 'e') || (gesture == 'm') || 
		(gesture == 's') || (gesture == 'w') ) {
		
		if (gesture == 'c'){
			code = KEY_GESTURE_C;
		}
		if (gesture == 'e'){
			code = KEY_GESTURE_E;                    
		}  
		if (gesture == 'm'){
			code = KEY_GESTURE_M;
		}
		if (gesture == 's'){
			code = KEY_GESTURE_S;
		}
		if (gesture == 'w'){
			code = KEY_GESTURE_W;
		}
	}else if ( (gesture == 0xAA) || (gesture == 0xBB) ||
		(gesture == 0xAB) || (gesture == 0xBA) ) {
		
		char *direction[4] = {"Right", "Down", "Up", "Left"};
		u8 type = ((gesture & 0x0F) - 0x0A) + (((gesture >> 4) & 0x0F) - 0x0A) * 2;

		GTP_INFO("%s slide to light up the screen!", direction[type]);
		if (type == 0){
			code = KEY_GESTURE_RIGHT;
		}
		if (type == 1){
			code = KEY_GESTURE_DOWN;
		}
		if (type == 2){
			code = KEY_GESTURE_UP;
		}
		if (type == 3){
			code = KEY_GESTURE_LEFT;
		}
	} else if (0xCC == gesture) {
		code = KEY_GESTURE_DOUBLE_CLICK;
	}
	input_report_key(tpd->dev, code, 1);
	input_sync(tpd->dev);
	input_report_key(tpd->dev, code, 0);
	input_sync(tpd->dev);
}

static void to_lcd_point(int *x, int *y)
{
	struct goodix_ts_data *data = i2c_get_clientdata(i2c_client_point);
	if(0 == data->tp_resx || 0 == data->tp_resy) {
		return;
	}

	*x = (*x) * data->lcd_resx / data->tp_resx;
	*y = (*y) * data->lcd_resy / data->tp_resy;
}
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	u8	end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	u8	point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
	u8	touch_num = 0;
	u8	finger = 0;
	static u8 pre_touch = 0;
	static u8 pre_key = 0;
	u8	key_value = 0;
	u8 *coor_data = NULL;
	s32 input_x = 0;
	s32 input_y = 0;
	s32 input_w = 0;
	s32 id = 0;
	s32 i  = 0;
	s32 ret = -1;
	struct goodix_ts_data *data = i2c_get_clientdata(i2c_client_point);

#if GTP_GESTURE_WAKEUP
	u8 doze_buf[3] = {0x81, 0x4B};
#endif

	sched_setscheduler(current, SCHED_RR, &param);
	do
	{
		set_current_state(TASK_INTERRUPTIBLE);
		
		while (tpd_halt)
		{
#if GTP_GESTURE_WAKEUP
			if (DOZE_ENABLED == doze_status) {
				break;
			}
#endif
			tpd_flag = 0;
			msleep(20);
		       GTP_INFO("Enter %s, LINE=%d,Waitfor tp resume\n", __FUNCTION__, __LINE__);
		}

		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);

#if GTP_CHARGER_SWITCH
		gtp_charger_switch(0);
#endif
		//GTP_INFO("Enter %s, %d\n", __FUNCTION__, __LINE__);
#if GTP_GESTURE_WAKEUP
		if (DOZE_ENABLED == doze_status)
		{
			ret = gtp_i2c_read(i2c_client_point, doze_buf, 3);
			GTP_INFO("0x814B = 0x%02X", doze_buf[2]);
			if (ret > 0)
			{				
				if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') || (doze_buf[2] == 'c') ||
					(doze_buf[2] == 'd') || (doze_buf[2] == 'e') || (doze_buf[2] == 'g') || 
					(doze_buf[2] == 'h') || (doze_buf[2] == 'm') || (doze_buf[2] == 'o') ||
					(doze_buf[2] == 'q') || (doze_buf[2] == 's') || (doze_buf[2] == 'v') || 
					(doze_buf[2] == 'w') || (doze_buf[2] == 'y') || (doze_buf[2] == 'z') ||
					(doze_buf[2] == 0x5E) /* ^ */
					)
				{
					if (doze_buf[2] != 0x5E) {
						GTP_INFO("Wakeup by gesture(%c), light up the screen!", doze_buf[2]);
					} else {
						GTP_INFO("Wakeup by gesture(^), light up the screen!");
					}
					doze_status = DOZE_WAKEUP;
					tpd_blackwakeup_func(data, doze_buf[2]);
					// clear 0x814B
					doze_buf[2] = 0x00;
					gtp_i2c_write(i2c_client_point, doze_buf, 3);
				} else if ( (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
					(doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA) )
				{
					char *direction[4] = {"Right", "Down", "Up", "Left"};
					u8 type = ((doze_buf[2] & 0x0F) - 0x0A) + (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;
					
					GTP_INFO("%s slide to light up the screen!", direction[type]);
					doze_status = DOZE_WAKEUP;
					tpd_blackwakeup_func(data, doze_buf[2]);
					// clear 0x814B
					doze_buf[2] = 0x00;
					gtp_i2c_write(i2c_client_point, doze_buf, 3);
				} else if (0xCC == doze_buf[2]) {
					GTP_INFO("Double click to light up the screen!");
					doze_status = DOZE_WAKEUP;
					tpd_blackwakeup_func(data, doze_buf[2]);
					// clear 0x814B
					doze_buf[2] = 0x00;
					gtp_i2c_write(i2c_client_point, doze_buf, 3);
				} else {
					GTP_INFO("tpd unkown blackscreen gesture.");
					// clear 0x814B
					doze_buf[2] = 0x00;
					gtp_i2c_write(i2c_client_point, doze_buf, 3);
					
				}
				gtp_enter_doze(i2c_client_point);
			}
			continue;
		}
#endif
		ret = gtp_i2c_read(i2c_client_point, point_data, 12);
		if (ret < 0) {
			GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
			continue;
		}
		finger = point_data[GTP_ADDR_LENGTH];
		if (finger == 0x00) 	{
			GTP_INFO("Enter %s, line=%d,finger==0\n", __FUNCTION__, __LINE__);
			continue;
		}
		if ((finger & 0x80) == 0) {
			GTP_INFO("Enter %s, line=%d,exit_work_func\n", __FUNCTION__, __LINE__);
			goto exit_work_func;
		}
		
		touch_num = finger & 0x0f;
		if (touch_num > GTP_MAX_TOUCH) {
			GTP_INFO("Enter %s, line=%d,exit_work_func\n", __FUNCTION__, __LINE__);
			goto exit_work_func;
		}
		if (touch_num > 1) {
			u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

			ret = gtp_i2c_read(i2c_client_point, buf, 2 + 8 * (touch_num - 1));
			memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
		}

#if (GTP_HAVE_TOUCH_KEY || GTP_PEN_HAVE_BUTTON)
		key_value = point_data[3 + 8 * touch_num];

		if (key_value || pre_key) {
#if GTP_PEN_HAVE_BUTTON
			if (key_value == 0x40) {
				GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Down.");
				input_report_key(pen_dev, BTN_STYLUS, 1);
				input_report_key(pen_dev, BTN_STYLUS2, 1);
				pen_active = 1;
			} else if (key_value == 0x10) {
				GTP_DEBUG("BTN_STYLUS Down, BTN_STYLUS2 Up.");
				input_report_key(pen_dev, BTN_STYLUS, 1);
				input_report_key(pen_dev, BTN_STYLUS2, 0);
				pen_active = 1;
			} else if (key_value == 0x20) {
				GTP_DEBUG("BTN_STYLUS Up, BTN_STYLUS2 Down.");
				input_report_key(pen_dev, BTN_STYLUS, 0);
				input_report_key(pen_dev, BTN_STYLUS2, 1);
				pen_active = 1;
			} else {
				GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Up.");
				input_report_key(pen_dev, BTN_STYLUS, 0);
				input_report_key(pen_dev, BTN_STYLUS2, 0);
				if ( (pre_key == 0x40) || (pre_key == 0x20) ||(pre_key == 0x10) ) {
					pen_active = 1;
				}
			}
			if (pen_active) {
				touch_num = 0;		// shield pen point
				//pre_touch = 0;	// clear last pen status
			}
#endif
#if GTP_HAVE_TOUCH_KEY
			if (!pre_touch) {
				printk("tpd gtp keyvalue=%d.\n",key_value);
				for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
					input_report_key(tpd->dev, touch_key_array[i], key_value & (0x01 << i));
				}
				touch_num = 0;	// shiled fingers
			}
#endif
		}
#endif
		pre_key = key_value;

		//GTP_DEBUG("pre_touch:%02x, finger:%02x.touch_num=0x%x\n", pre_touch, finger,touch_num);
		
		if (touch_num) {
			report_num++;
			for (i = 0; i < touch_num; i++) {
				coor_data = &point_data[i * 8 + 3];

				id = coor_data[0] & 0x0F;	   
				input_x  = coor_data[1] | coor_data[2] << 8;
				input_y  = coor_data[3] | coor_data[4] << 8;
				input_w  = coor_data[5] | coor_data[6] << 8;

				input_x = TPD_WARP_X(abs_x_max, input_x);
				input_y = TPD_WARP_Y(abs_y_max, input_y);
				//to_lcd_point(&input_x, &input_y);

				//GTP_DEBUG(" (%d)(%d, %d)[%d]", id, input_x, input_y, input_w);
				tpd_down(input_x, input_y, input_w, id);

				//if((g_tpd_debug_point - 1) % 160 == 0) {
					//GTP_INFO("tpd:%d%d %d,%d\n", id, input_w, input_x, input_y);
				//}
				 //if(1==report_num%90)
				  if((1==all_up)||(1==report_num%90))
				  {
				   if(1==all_up) all_up=0;
				   GTP_INFO("[D],REP:[%ld],INT:[%ld],ID:[%d],LO:[%d,%d],T:[%d]\n",report_num,tp_irq_num,id, input_x, input_y,tpd_point_desc[i].downnum);
				}
				  
			}
		} else {
			if (pre_touch)	{
				//GTP_DEBUG("Touch Release!");
				tpd_up(0, 0, 0);
			}
		}
		
		pre_touch = touch_num;
		input_sync(tpd->dev);

exit_work_func:

		if (!gtp_rawdiff_mode) {
			ret = gtp_i2c_write(i2c_client_point, end_cmd, 3);
			if (ret < 0) {
				GTP_INFO("I2C write end_cmd  error!\n");
			}
		}
	} while (!kthread_should_stop());

	return 0;
}

static int tpd_local_init(void)
{
	GTP_INFO("Enter %s, %d\n", __FUNCTION__, __LINE__);
	
#if GTP_ESD_PROTECT
	clk_tick_cnt = 2 * HZ;	 // HZ: clock ticks in 1 second generated by system
	GTP_DEBUG("Clock ticks for an esd cycle: %d", clk_tick_cnt);
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
	spin_lock_init(&esd_lock);			// 2.6.39 & later
	// esd_lock = SPIN_LOCK_UNLOCKED;	// 2.6.39 & before
#endif

	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		GTP_INFO("unable to add i2c driver.\n");
		return -1;
	}

#if  0	/*zhengxiahong*/
	if (tpd_load_status == 0) //if(tpd_load_status == 0) // disable auto load touch driver for linux3.0 porting
	{
		GTP_INFO("add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
#endif
#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif

	// set vendor string
	tpd->dev->id.vendor = 0x00;
	tpd->dev->id.product = tpd_info.pid;
	tpd->dev->id.version = tpd_info.vid;

	GTP_INFO("Exit %s, %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

#if GTP_GESTURE_WAKEUP
static s8 gtp_enter_doze(struct i2c_client *client)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

	//GTP_DEBUG_FUNC();

	GTP_INFO("Entering gesture mode...\n");
	while(retry++ < 5)
	{
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x46;
		ret = gtp_i2c_write(client, i2c_control_buf, 3);
		if (ret < 0)
		{
			GTP_INFO("Failed to set gesture flag into 0x8046, %d\n", retry);
			continue;
		}
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x40;
		ret = gtp_i2c_write(client, i2c_control_buf, 3);
		if (ret > 0)
		{
			doze_status = DOZE_ENABLED;
			GTP_INFO("Gesture mode enabled.\n");
			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send gesture cmd failed.\n");
	return ret;
}

#endif
/*******************************************************
Function:
	Eter sleep function.

Input:
	client:i2c_client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_enter_sleep(struct i2c_client *client)
{
#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == gtp_chip_type)
	{
		u8 i2c_status_buf[3] = {0x80, 0x44, 0x00};
		s32 ret = 0;
	  
		ret = gtp_i2c_read(client, i2c_status_buf, 3);
		if(ret <= 0)
		{
			 GTP_ERROR("[gtp_enter_sleep]Read ref status reg error.");
		}
		
		if (i2c_status_buf[2] & 0x80)
		{
			//Store bak ref
			ret = gtp_bak_ref_proc(client, GTP_BAK_REF_STORE);
			if(FAIL == ret)
			{
				GTP_ERROR("[gtp_enter_sleep]Store bak ref failed.");
			}		 
		}
	}
#endif
#if GTP_POWER_CTRL_SLEEP

	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);	
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(10);

#ifdef MT6573
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);  
	msleep(30);
#else				// ( defined(MT6575) || defined(MT6577) || defined(MT6589) )

	#ifdef TPD_POWER_SOURCE_1800
		hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
	#endif
	
	#ifdef TPD_POWER_SOURCE_CUSTOM
		hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
	#else
		hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
	#endif
#endif
   
	GTP_INFO("GTP enter sleep by poweroff!");
	return 0;
	
#else
	{
		s8 ret = -1;
		s8 retry = 0;
		u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};
		
		
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
		msleep(5);
	
		while (retry++ < 5) {
			ret = gtp_i2c_write(client, i2c_control_buf, 3);
			if (ret > 0) {
				GTP_INFO("GTP enter sleep! try %d\n",retry);
				return ret;
			}
	
			msleep(10);
		}
	
		GTP_ERROR("GTP send sleep cmd failed.");
		return ret;
	}
#endif
}

/*******************************************************
Function:
	Wakeup from sleep mode Function.

Input:
	client:i2c_client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_wakeup_sleep(struct i2c_client *client)
{
	u8 retry = 0;
	s8 ret = -1;
	struct goodix_ts_data *data = i2c_get_clientdata(i2c_client_point);

	GTP_INFO("GTP wakeup begin.");

#if (GTP_POWER_CTRL_SLEEP)	 
	while (retry++ < 5)
	{
		ret = tpd_power_on(client);

		if (ret < 0)
		{
			GTP_ERROR("I2C Power on ERROR!");
			continue;
		}
		GTP_INFO("Ic wakeup by poweron");
		return 0;
	}
#else
	while (retry++ < 10)
	{
#if GTP_GESTURE_WAKEUP
		if(data->enable_wakeup_gesture) {
			if (DOZE_WAKEUP != doze_status) {
				GTP_INFO("Powerkey wakeup.");
			} else {
				GTP_INFO("Gesture wakeup try %d.\n",retry);
			}
			doze_status = DOZE_DISABLED;

			mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
			gtp_reset_guitar(client, 20);
			mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		} else {
			GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
			msleep(5);
		}
#else
                GTP_INFO("No Gesture wakeup.");
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
		msleep(5);
#endif
		ret = gtp_i2c_test(client);
		if (ret >= 0) {
			GTP_INFO("GTP wakeup sleep.");
#if GTP_GESTURE_WAKEUP
			if(!data->enable_wakeup_gesture) {
				gtp_int_sync(25);
			}
#if GTP_ESD_PROTECT
				gtp_init_ext_watchdog(client);
#endif
#endif
			return ret;
		}
		gtp_reset_guitar(client, 20);
	}
#endif
	GTP_ERROR("GTP wakeup sleep failed.");
	return ret;
}

/* Function to manage low power suspend */
static void tpd_suspend(struct early_suspend *h)
{
	s32 ret = -1;
	int id=0;
	struct goodix_ts_data *data = i2c_get_clientdata(i2c_client_point);
	
	GTP_INFO("TPD goodix ts suspend.");
	if(data->need_stay_awake) {
		printk("%s need_stay_awake return.\n", __func__);
		return;
	}
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1) {
		return ;
	}
#endif

	tpd_halt = 1;
	data->tpd_suspend = true;
#if GTP_ESD_PROTECT
	gtp_esd_switch(i2c_client_point, SWITCH_OFF);
#endif
	
#if GTP_GESTURE_WAKEUP
	if(data->enable_wakeup_gesture) {
		ret = gtp_enter_doze(i2c_client_point);
	} else {
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		ret = gtp_enter_sleep(i2c_client_point);
	}
#else
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	ret = gtp_enter_sleep(i2c_client_point);
#endif

      #if   1   /*for debug*/
       report_num = 0;
       tp_irq_num = 0;
	for(id=0;id<5;id++)
	{
		tpd_point_desc[id].finger_status=0;
		tpd_point_desc[id].downnum=0;
		tpd_point_desc[id].x=0;
		tpd_point_desc[id].y=0;
	}
	#endif
	if (ret < 0)
	{
		GTP_ERROR("GTP early suspend failed.");
	}
	// to avoid waking up while not sleeping, delay 48 + 10ms to ensure reliability 
	msleep(58);
}

/* Function to manage power-on resume */
static void tpd_resume(struct early_suspend *h)
{
	s32 ret = -1;
	struct goodix_ts_data *data = i2c_get_clientdata(i2c_client_point);

	GTP_INFO("TPD goodix ts resume.");
	if(data->need_stay_awake) {
		printk("%s need_stay_awake return.\n", __func__);
		return;
	}
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1) {
		return ;
	}
#endif
	ret = gtp_wakeup_sleep(i2c_client_point);
	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
	{
		gtp_send_cfg(i2c_client_point);
	}
	
#if GTP_CHARGER_SWITCH
	gtp_charger_switch(1);	// force update
#endif

	tpd_halt = 0;
	data->tpd_suspend = false;
#if GTP_GESTURE_WAKEUP
	doze_status = DOZE_DISABLED;
	if(!data->enable_wakeup_gesture) {
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	}
#else 
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif

}

static struct tpd_driver_t tpd_device_driver =
{
	.tpd_device_name = "gt9xx",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	GTP_INFO("MediaTek gt91xx touch panel driver init\n");
#ifdef MT6572
1
	i2c_register_board_info(I2C_BUS_NUMBER, &i2c_tpd, 1);
#else
	i2c_register_board_info(1, &i2c_tpd, 1);
#endif
	if (tpd_driver_add(&tpd_device_driver) < 0)
		GTP_INFO("add generic driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	GTP_INFO("MediaTek gt91xx touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

