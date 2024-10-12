/* linux/drivers/hwmon/lis33de.c
 *
 * (C) Copyright 2008 
 * MediaTek <www.mediatek.com>
 *
 * MAX77819 driver for MT6516
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef MAX77819_H
#define MAX77819_H 
	 
#include <linux/ioctl.h>
#include <mach/mt_typedefs.h>

#define MAX77819_I2C_SLAVE_ADDR		0x90


#define WLED_MAX_BRIGHTNESS     0xFF

/* registers */
#define MAX77819_WLED_INT		0x9B
#define MAX77819_WLED_INT_MASK	0x9C
#define MAX77819_WLEDBSTCNTL	0x98
#define MAX77819_IWLED			0x99

/* MAX77819_WLED_INT */
#define MAX77819_WLEDOL			0x10
#define MAX77819_WLEDOVP_I		0x80

/* MAX77819_WLED_INT_MASK */
#define MAX77819_WLEDOL_M		0x10
#define MAX77819_WLEDOVP_M		0x80

/* MAX77819_WLEDBSTCNTL */
#define MAX77819_WLEDOVP		0x02
#define MAX77819_WLEDFOSC		0x0C
#define MAX77819_WLEDPWM2EN		0x10
#define MAX77819_WLEDPWM1EN		0x20
#define MAX77819_WLED2EN		0x40
#define MAX77819_WLED1EN		0x80

/* MAX77819_IWLED */
#define MAX77819_CUR			0xFF

	 
	 
#define MAX77819_SUCCESS						0
#define MAX77819_ERR_I2C						-1
#define MAX77819_ERR_STATUS					-3
#define MAX77819_ERR_SETUP_FAILURE			-4
#define MAX77819_ERR_GETGSENSORDATA			-5
#define MAX77819_ERR_IDENTIFICATION			-6
	 
	 
	 
#define MAX77819_BUFSIZE				256


#define M2SH(m) ((m) & 0x0F ? ((m) & 0x03 ? ((m) & 0x01 ? 0 : 1) : ((m) & 0x04 ? 2 : 3)) : \
                   ((m) & 0x30 ? ((m) & 0x10 ? 4 : 5) : ((m) & 0x40 ? 6 : 7)))


/**********************************************************
  *
  *   [I2C Function For Read/Write max77819] 
  *
  *********************************************************/
extern int max77819_read_byte(u8 cmd, u8 *returnData);
extern int max77819_write_byte(u8 cmd, u8 writeData);
extern u32 max77819_read_interface (u8 RegNum, u8 *val, u8 MASK, u8 SHIFT);
extern u32 max77819_config_interface (u8 RegNum, u8 val, u8 MASK, u8 SHIFT);
extern int max77819_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
extern int max77819_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
extern int max77819_periph_reg_update_bits(kal_uint8 reg, kal_uint8 mask, kal_uint8 val);

//Backlight
extern int max77819_set_backlight_cntl_mode(unsigned char mode);
extern int max77819_set_backlight_level(int level);
extern int max77819_set_backlight_enable(unsigned char on);
//Vibrator
extern int max77819_vib_set(bool on);
extern int max77819_vib_set_sinkcurrent(int curr);
	 
#endif

