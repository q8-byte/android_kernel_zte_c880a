#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h> 
	#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_pm_ldo.h>
    #include <mach/mt_gpio.h>
#endif
#include <cust_gpio_usage.h>
#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_i2c.h>
#endif
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif


static const unsigned int BL_MIN_LEVEL =20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmd_by_cmdq(handle,cmd,count,ppara,force_update)    lcm_util.dsi_set_cmdq_V22(handle,cmd,count,ppara,force_update);
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0
#ifndef CONFIG_FPGA_EARLY_PORTING
#define GPIO_65132_ENP  GPIO201  //GPIO_LCD_BIAS_ENP_PIN  liuyi
#define GPIO_65132_ENN  GPIO200 //GPIO_LCD_BIAS_ENN_PIN  liuyi
#endif
#define LCM_ID_OTM1283 0x40

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#if 1
static struct LCM_setting_table lcm_initialization_setting[] = {
{0x00,1,{0x00}},
{0xff,3,{0x12,0x84,0x01}},	//EXTC=1
{0x00,1,{0x80}},	            //Orise mode enable
{0xff,2,{0x12,0x84}},

{0x00,1,{0x92}},	            //Orise mode enable
{0xff,2,{0x30,0x02}},

{0x00,1,{0x80}},             //TCON Setting
{0xc0,9,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},

{0x00,1,{0x90}},             //Panel Timing Setting
{0xc0,6,{0x00,0x5c,0x00,0x01,0x00,0x04}},

{0x00,1,{0xb3}},             //Interval Scan Frame: 0 frame, column inversion
{0xc0,2,{0x00,0x55}},

{0x00,1,{0x81}},             //frame rate:60Hz
{0xc1,1,{0x55}},

{0x00,1,{0xa0}},             //dcdc setting
{0xc4,14,{0x05,0x10,0x06,0x02,0x05,0x15,0x10,0x05,0x10,0x07,0x02,0x05,0x15,0x10}},

{0x00,1,{0xb0}},             //clamp voltage setting
{0xc4,2,{0x00,0x00}},

{0x00,1,{0x91}},             //VGH=15V, VGL=-10V, pump ratio:VGH=6x, VGL=-5x
{0xc5,2,{0x46,0x42}},

{0x00,1,{0x00}},             //GVDD=4.87V, NGVDD=-4.87V
{0xd8,2,{0xbc,0xbc}},

{0x00,1,{0x00}},            
{0xd9,1,{0x8d}},

{0x00,1,{0xb3}},             //VDD_18V=1.7V, LVDSVDD=1.6V
{0xc5,1,{0x84}},

{0x00,1,{0xbb}},             //LVD voltage level setting
{0xc5,1,{0x8a}},

{0x00,1,{0x82}},		// chopper
{0xC4,1,{0x0a}}, 

{0x00,1,{0xc6}},		// debounce
{0xB0,1,{0x03}}, 

{0x00,1,{0xc2}},             //precharge disable
{0xf5,1,{0x40}},

{0x00,1,{0xc3}},             //sample hold gvdd
{0xf5,1,{0x85}},

{0x00,1,{0x87}},             //en op
{0xc4,1,{0x18}},

{0x00,1,{0x00}},             //ID1
{0xd0,1,{0x40}},

{0x00,1,{0x00}},             //ID2, ID3
{0xd1,2,{0x00,0x00}},

{0x00,1,{0x80}},             //panel timing state control
{0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},             //panel timing state control
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xa0}},             //panel timing state control
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xb0}},             //panel timing state control
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xc0}},             //panel timing state control
{0xcb,15,{0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xd0}},             //panel timing state control
{0xcb,15,{0xff,0xff,0xff,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00}},

{0x00,1,{0xe0}},             //panel timing state control
{0xcb,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x05,0x05,0x05}},

{0x00,1,{0xf0}},             //panel timing state control
{0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},

{0x00,1,{0x80}},             //panel pad mapping control
{0xcc,15,{0x02,0x0a,0x0c,0x0e,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},             //panel pad mapping control
{0xcc,15,{0x00,0x00,0x00,0x00,0x2e,0x2d,0x06,0x01,0x09,0x0b,0x0d,0x0f,0x00,0x00,0x00}},

{0x00,1,{0xa0}},             //panel pad mapping control
{0xcc,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2e,0x2d,0x05}},

{0x00,1,{0xb0}},             //panel pad mapping control
{0xcc,15,{0x05,0x0f,0x0d,0x0b,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xc0}},             //panel pad mapping control
{0xcc,15,{0x00,0x00,0x00,0x00,0x2d,0x2e,0x01,0x06,0x10,0x0e,0x0c,0x0a,0x00,0x00,0x00}},

{0x00,1,{0xd0}},             //panel pad mapping control
{0xcc,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2d,0x2e,0x02}},

{0x00,1,{0x80}},             //panel VST setting
{0xce,12,{0x87,0x03,0x10,0x86,0x03,0x10,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},             //panel VEND setting
{0xce,14,{0x35,0x01,0x10,0x35,0x02,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xa0}},             //panel CLKA1/2 setting
{0xce,14,{0x38,0x03,0x04,0xf8,0x00,0x10,0x00,0x38,0x02,0x04,0xf9,0x00,0x10,0x00}},

{0x00,1,{0xb0}},             //panel CLKA3/4 setting
{0xce,14,{0x38,0x01,0x04,0xfa,0x00,0x10,0x00,0x38,0x00,0x04,0xfb,0x00,0x10,0x00}},

{0x00,1,{0xc0}},             //panel CLKb1/2 setting
{0xce,14,{0x30,0x00,0x04,0xfc,0x00,0x10,0x00,0x30,0x01,0x04,0xfd,0x00,0x10,0x00}},

{0x00,1,{0xd0}},             //panel CLKb3/4 setting
{0xce,14,{0x30,0x02,0x04,0xfe,0x00,0x10,0x00,0x30,0x03,0x04,0xff,0x00,0x10,0x00}},

{0x00,1,{0x80}},             //panel CLKc1/2 setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},             //panel CLKc3/4 setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xa0}},             //panel CLKd1/2 setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xb0}},             //panel CLKd3/4 setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xc0}},             //panel ECLK setting
{0xcf,11,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x81,0x00,0x03,0x08}}, //gate pre. ena.

{0x00,1,{0xb5}},             //TCON_GOA_OUT Setting
{0xc5,6,{0x3f,0xff,0xff,0x3f,0xff,0xff}},

{0x00,1,{0x90}},             //Mode-3
{0xf5,4,{0x02,0x11,0x02,0x15}},

{0x00,1,{0x90}},             //2//xVPNL
{0xc5,1,{0x50}},

{0x00,1,{0x94}},             //Freq.
{0xc5,1,{0x66}},

{0x00,1,{0xb2}},             //VGLO1
{0xf5,2,{0x00,0x00}},

{0x00,1,{0xb4}},             //VGLO1_S
{0xf5,2,{0x00,0x00}},

{0x00,1,{0xb6}},             //VGLO2
{0xf5,2,{0x00,0x00}},

{0x00,1,{0xb8}},             //VGLO2_S
{0xf5,2,{0x00,0x00}},

{0x00,1,{0x94}},             //VCL pump dis
{0xf5,2,{0x00,0x00}},

{0x00,1,{0xd2}},             //VCL reg. en
{0xf5,2,{0x06,0x15}},

{0x00,1,{0xb4}},             //VGLO1/2 Pull low setting
{0xc5,1,{0xcc}},				//d[7] vglo1 d[6] vglo2 => 0: pull vss, 1: pull vgl


{0x00,1,{0x00}},                                                                                    ////
{0xE1,20,{0x00,0x11,0x1e,0x2b,0x3b,0x49,0x4c,0x7a,0x6e,0x8b,0x75,0x5d,0x6c,0x47,0x42,0x33,0x25,0x16,0x09,0x00}},
	
{0x00,1,{0x00}},
{0xE2,20,{0x00,0x11,0x1e,0x2b,0x3b,0x49,0x4c,0x7a,0x6e,0x8b,0x73,0x5b,0x6c,0x42,0x42,0x33,0x25,0x16,0x09,0x00}},

// ***********************CELEVER COLOR SETTING FOR CE ****************************************//
{0x00,1,{0xA0}},
{0xD6,12,{0x01,0x0d,0x01,0x0d,0x01,0xa6,0x01,0xa6,0x01,0xa6,0x01,0xa6}},

{0x00,1,{0xB0}},
{0xD6,12,{0x01,0xa6,0x01,0xa6,0x01,0xa6,0x01,0xa6,0x01,0xa6,0x01,0xa6}},

{0x00,1,{0xC0}},
{0xD6,12,{0x55,0x11,0x09,0x6f,0x11,0x6f,0x6f,0x11,0x6f,0x6f,0x11,0x6f}},

{0x00,1,{0xD0}},
{0xD6,6,{0x6f,0x11,0x6f,0x6f,0x11,0x6f}},

{0x00,1,{0xE0}},
{0xD6,12,{0x2b,0x11,0x37,0x37,0x11,0x37,0x37,0x11d,0x37,0x37,0x11,0x37}},

{0x00,1,{0xF0}},
{0xD6,6,{0x37,0x11,0x37,0x37,0x11,0x37}},


//CE on High
{0x00,1,{0x00}},             
{0x81,1,{0x83}},	//CE - High
/*
//CE on Middle
{0x00,,{0x00}},             
{0x81,,{0x81}},	//CE - Middle

//CE on Low
{0x00,0x00}},             
{0x81,0x80}},	//CE - Low

//CE off
{0x00,0x00}},             
{0x81,0x00}},	//CE  off
*/

{0x00,1,{0x00}},             //Orise mode disable
{0xff,3,{0xff,0xff,0xff}},	

};
#endif

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},

    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_backlight_level_setting[] = {
{0x51, 1, {0xFF}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_init_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//mt6325_upmu_set_rg_vgp1_en(1);  //liuyi
#else
	printk("%s, begin\n", __func__);
	//hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	//liuyi
	printk("%s, end\n", __func__);
#endif
#endif
}

static void lcm_suspend_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//mt6325_upmu_set_rg_vgp1_en(0);//liuyi
#else
	printk("%s, begin\n", __func__);
	//hwPowerDown(MT6325_POWER_LDO_VGP1, "LCM_DRV");	//liuyi
	printk("%s, end\n", __func__);
#endif
#endif
}

static void lcm_resume_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//mt6325_upmu_set_rg_vgp1_en(1);  //liuyi
#else
	printk("%s, begin\n", __func__);
	//hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	 //liuyi
	printk("%s, end\n", __func__);
#endif
#endif
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
            //The following defined the fomat for data coming from LCD engine.
            params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
            params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
            params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
            params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

            // Highly depends on LCD driver capability.
                 params->dsi.packet_size=256;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 14;
		params->dsi.vertical_frontporch					= 16;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2;
		params->dsi.horizontal_backporch				= 42;
		params->dsi.horizontal_frontporch				= 44;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

#ifndef CONFIG_FPGA_EARLY_PORTING
                params->dsi.PLL_CLOCK = 190;//220; //this value must be in MTK suggested table
#else
                params->dsi.pll_div1 = 0;
                params->dsi.pll_div2 = 0;
                params->dsi.fbk_div = 0x1;
#endif
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable = 0;
		params->dsi.lcm_esd_check_table[0].cmd 			= 0x0a;
		params->dsi.lcm_esd_check_table[0].count 		= 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
		params->dsi.vertical_vfp_lp = 100;

}
#define GPIO_LCD_ID1 59 // 61|0x80000000
#define GPIO_PCD_ID0  60 //62|0x80000000
static unsigned int lcm_compare_id(void)
{
	int pin_lcd_id0=0;	
	int pin_lcd_id1=0;
	
  #if 0
	mt_set_gpio_mode(GPIO_LCD_ID1,0);//Set GPIO as GPIO Mode ,Input dir, Disable Inpull for LCM automatic recognition 
	mt_set_gpio_dir(GPIO_LCD_ID1,0);	
	mt_set_gpio_pull_enable(GPIO_LCD_ID1,1);
	
	mt_set_gpio_mode(GPIO_PCD_ID0,0);//Set GPIO as GPIO Mode ,Input dir, Disable Inpull for LCM automatic recognition 
	mt_set_gpio_dir(GPIO_PCD_ID0,0);	
	mt_set_gpio_pull_enable(GPIO_PCD_ID0,1);
  #endif
       pin_lcd_id1 = mt_get_gpio_in(GPIO_LCD_ID1);
	pin_lcd_id0= mt_get_gpio_in(GPIO_PCD_ID0);
#ifdef BUILD_LK
	printf("%s, lead id otm1284A, pin_lcd_id0= %d, pin_lcd_id1 = %d \n", __func__, pin_lcd_id0, pin_lcd_id1);
#else
	printk("%s, lead id otm1284A, pin_lcd_id0= %d, pin_lcd_id1 = %d \n", __func__, pin_lcd_id0, pin_lcd_id1);
#endif
	if (pin_lcd_id1 == 0 && pin_lcd_id0 == 1){
		return  1;  
	}else{
		return 0;
	}
}
#if 1
static void lcm_init_registers()
{
	unsigned int data_array[16];
#if 0
	
	data_array[0] = 0x00002300;
	dsi_set_cmdq(&data_array, 1, 1);//EXTC = 1
	
	data_array[0] = 0x00042902;
	data_array[1] = 0x018312FF;
	dsi_set_cmdq(&data_array, 2, 1);//EXTC = 1
	
	data_array[0] = 0x80002300;
	dsi_set_cmdq(&data_array, 1, 1);	//Orise mode enable
	
	data_array[0] = 0x00032902;
	data_array[1] = 0x008312FF;
	dsi_set_cmdq(&data_array, 2, 1);
	
	/*===================panel setting====================*/
	data_array[0] = 0x80002300;
	dsi_set_cmdq(&data_array, 1, 1);	//TCON Setting
	
	data_array[0] = 0x000A2902;
	data_array[1] = 0x006400C0;
	data_array[2] = 0x6400120e;
	data_array[3] = 0x0000120e;
	dsi_set_cmdq(&data_array, 4, 1);

	data_array[0] = 0xb4002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x55C02300;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x81002300;
	dsi_set_cmdq(&data_array, 1, 1);	//frame rate: 60Hz
	
	data_array[0] = 0x55C12300;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x81002300;				//Source bias 0.75uA
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x82C42300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x90002300;				//clock delay for data latch
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x49C42300;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x82002300;				//clock delay for data latch
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x02C42300;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0xc6002300;				//clock delay for data latch
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x03b02300;
	dsi_set_cmdq(&data_array, 1, 1);

	
	/*================Power setting===============*/
	data_array[0] = 0xA0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000F2902;	//DCDC setting
	data_array[1] = 0x061005C4;
	data_array[2] = 0x10150502;
	data_array[3] = 0x02071005;
	data_array[4] = 0x00101505;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xB0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//Clamp coltage setting
	data_array[1] = 0x000000C4;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0xBB002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x80c52300;	//LVD voltage level setting
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x91002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VGH=12v, VGL=-12v, pump ratio: VGH=6x, VGL=-5x
	data_array[1] = 0x005016c5;
	dsi_set_cmdq(&data_array, 2, 1);
	
	data_array[0] = 0x00002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//GVDD=4.87v, NGVDD = -4.87V
	data_array[1] = 0x00aeaed8;
	dsi_set_cmdq(&data_array, 2, 1);
	
	data_array[0] = 0xB0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VDD_18v=1.6v, LVDSVDD=1.55v
	data_array[1] = 0x00b804c5;
	dsi_set_cmdq(&data_array, 2, 1);
	
	/*=========================Panel Timming State Control================*/
	data_array[0] = 0x90002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00052902;	//mode-3
	data_array[1] = 0x021102f5;
	data_array[2] = 0x00000011;
	dsi_set_cmdq(&data_array, 3, 1);
	
		data_array[0] = 0x90002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x50c52300;	//2xVPNL, 1.5*=00, 2*=50, 3*=A0
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x94002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x66c52300;	//Frequency
	dsi_set_cmdq(&data_array, 1, 1);
	
	/*===============VGL01/02 disable================*/
	data_array[0] = 0xb2002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VGL01
	data_array[1] = 0x000000f5;
	dsi_set_cmdq(&data_array, 2, 1);
	
	data_array[0] = 0xb4002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VGL01_S
	data_array[1] = 0x000000f5;
	dsi_set_cmdq(&data_array, 2, 1);
	
	data_array[0] = 0xb6002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VGL02
	data_array[1] = 0x000000f5;
	dsi_set_cmdq(&data_array, 2, 1);
	
	data_array[0] = 0xb8002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VGL02_S
	data_array[1] = 0x000000f5;
	dsi_set_cmdq(&data_array, 2, 1);


	data_array[0] = 0x94002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VCL ON
	data_array[1] = 0x000002f5;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0xBA002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VSP ON
	data_array[1] = 0x000003f5;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0xB2002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VCL ON
	data_array[1] = 0x000040C5;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0xB4002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00032902;	//VCL ON
	data_array[1] = 0x0000C0C5;
	dsi_set_cmdq(&data_array, 2, 1);
	
	data_array[0] = 0x80002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000C2902;	//Panel timing state control
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	dsi_set_cmdq(&data_array, 4, 1);
	
	data_array[0] = 0x90002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00102902;	//Panel timing state control
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xa0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00102902;	//Panel timing state control
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xb0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00102902;	//Panel timing state control
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xc0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00102902;	//Panel timing state control
	data_array[1] = 0x050505cb;
	data_array[2] = 0x00050505;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xd0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00102902;	//Panel timing state control
	data_array[1] = 0x000000cb;
	data_array[2] = 0x05050500;
	data_array[3] = 0x05050505;
	data_array[4] = 0x00000505;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xe0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000F2902;	//Panel timing state control
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00050505;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xf0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000c2902;	//Panel timing state control
	data_array[1] = 0xffffffcb;
	data_array[2] = 0xffffffff;
	data_array[3] = 0xfffffffC;
	dsi_set_cmdq(&data_array, 4, 1);
	
	/*===============Panel pad mapping control===============*/
	data_array[0] = 0x80002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00102902;	//Panel pad mapping control
	data_array[1] = 0x100A0Ccc;
	data_array[2] = 0x0004020e;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0x90002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00102902;	//Panel pad mapping control
	data_array[1] = 0x000000cc;
	data_array[2] = 0x2d2e0600;
	data_array[3] = 0x0d0f090b;
	data_array[4] = 0x00000301;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xa0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel pad mapping control
	data_array[1] = 0x000000cc;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x002d2e05;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xb0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00102902;	//Panel pad mapping control
	data_array[1] = 0x090f0dcc;
	data_array[2] = 0x0001030b;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xc0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00102902;	//Panel pad mapping control
	data_array[1] = 0x000000cc;
	data_array[2] = 0x2e2d0600;
	data_array[3] = 0x0c0a100e;
	data_array[4] = 0x00000204;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xd0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel pad mapping control
	data_array[1] = 0x000000cc;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x002e2d05;
	dsi_set_cmdq(&data_array, 5, 1);
	
	/*===============Panel Timing Setting====================*/
	data_array[0] = 0x80002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000d2902;	//Panel VST Setting
	data_array[1] = 0x18038bce;
	data_array[2] = 0x8918038a;
	data_array[3] = 0x03881803;
	data_array[4] = 0x00000018;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0x90002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel vend setting
	data_array[1] = 0x180f38ce;
	data_array[2] = 0x00180e38;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xa0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel clka1/2 setting
	data_array[1] = 0x050738ce;
	data_array[2] = 0x00180000;
	data_array[3] = 0x01050638;
	data_array[4] = 0x00001800;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xb0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel clka3/4 setting
	data_array[1] = 0x050538ce;
	data_array[2] = 0x00180002;
	data_array[3] = 0x03050438;
	data_array[4] = 0x00001800;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xc0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel clkb1/2 setting
	data_array[1] = 0x050338ce;
	data_array[2] = 0x00180004;
	data_array[3] = 0x05050238;
	data_array[4] = 0x00001800;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xd0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel clkb3/4 setting
	data_array[1] = 0x050138ce;
	data_array[2] = 0x00180006;
	data_array[3] = 0x07050038;
	data_array[4] = 0x00001800;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0x80002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel clkc1/2 setting
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0x90002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel clkc3/4 setting
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xa0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel clkd1/2 setting
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xb0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000f2902;	//Panel clkd3/4 setting
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);
	
	data_array[0] = 0xc0002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x000c2902;	//gate pre. ena
	data_array[1] = 0x200101cf;
	data_array[2] = 0x02000020;
	data_array[3] = 0x08030081;
	dsi_set_cmdq(&data_array, 4, 1);
	
	data_array[0] = 0xb5002300;
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0] = 0x00072902;	//normal output with VGH/VGL
	data_array[1] = 0x7f1138c5;
	data_array[2] = 0x007f1138;
	dsi_set_cmdq(&data_array, 3, 1);
	
	/*===================Gamma====================*/
	data_array[0] = 0x00002300;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x00112902;
	data_array[1] = 0x1c1605e1;
	data_array[2] = 0x0c11060d;
	data_array[3] = 0x0807020b;
	data_array[4] = 0x0b100e07;
	data_array[5] = 0x00000002;
	dsi_set_cmdq(&data_array, 6 ,1);
	
	data_array[0] = 0x00002300;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x00112902;
	data_array[1] = 0x1C1605e2;
	data_array[2] = 0x0c12060d;
	data_array[3] = 0x0907020b;
	data_array[4] = 0x0b110e07;
	data_array[5] = 0x00000002;
	dsi_set_cmdq(&data_array, 6 ,1);
	
	data_array[0] = 0x92002300;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x00032902;
	data_array[1] = 0x000230ff;
	dsi_set_cmdq(&data_array, 2 ,1);
	
	data_array[0] = 0x00002300;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x00022902;
	data_array[1] = 0x000070d9;
	dsi_set_cmdq(&data_array, 2 ,1);
#else
data_array[0]= 0x00023902;
data_array[1]= 0x00000000;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00043902;
data_array[1]= 0x018412ff;
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;
data_array[1]= 0x00008000;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00033902;
data_array[1]= 0x008412ff;
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;
data_array[1]= 0x00009200;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00033902;
data_array[1]= 0x000230ff;
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;
data_array[1]= 0x00008000;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x000A3902;
data_array[1]= 0x006400c0;
data_array[2]= 0x64001010;
data_array[3]= 0x00001010;
dsi_set_cmdq(data_array, 4, 1);
	
data_array[0]= 0x00023902;
data_array[1]= 0x00009000;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;
data_array[1]= 0x005c00c0;
	data_array[2] = 0x00040001;
dsi_set_cmdq(data_array, 3, 1);
	
data_array[0]= 0x00023902;
data_array[1]= 0x0000b300;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00033902;
data_array[1]= 0x005500c0;
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;
data_array[1]= 0x00008100;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;
data_array[1]= 0x000055c1;
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;
data_array[1]= 0x0000a000;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x000f3902;
data_array[1]= 0x061005c4;
	data_array[2] = 0x10150502;
	data_array[3] = 0x02071005;
	data_array[4] = 0x00101505;
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000b000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00033902;     
data_array[1]= 0x000000c4;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00009100;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00033902;     
data_array[1]= 0x004246c5;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00000000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00033902;     
	data_array[1] = 0x00bcbcd8;
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00000000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;     
data_array[1]= 0x000085d9;  //d9-8d     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000b300;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;     
data_array[1]= 0x000084c5;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000bb00;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;     
data_array[1]= 0x00008ac5;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00008200;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;     
data_array[1]= 0x00000ac4;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000c600;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;     
data_array[1]= 0x000003b0;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000c200;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;     
data_array[1]= 0x000040f5;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000c300;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;     
data_array[1]= 0x000085f5;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00008700;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;     
data_array[1]= 0x000018c4;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00000000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;     
data_array[1]= 0x000040d0;     
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00000000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00033902;     
	data_array[1] = 0x000000d1;
dsi_set_cmdq(data_array, 2, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00008000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x000c3902;     
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
dsi_set_cmdq(data_array, 4, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00009000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00103902;     
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000a000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00103902;     
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000b000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00103902;     
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000c000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00103902;     
	data_array[1] = 0x050505cb;
data_array[2]= 0x00000505;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000d000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00103902;     
data_array[1]= 0xffffffcb;
data_array[2]= 0x05050500;
	data_array[3] = 0x05050505;
data_array[4]= 0x00000005;   
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000e000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x000f3902;     
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
data_array[3]= 0x00ffffff; 
data_array[4]= 0x00050505;   
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000f000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x000c3902;     
data_array[1]= 0xffffffff;
	data_array[2] = 0xffffffff;
	data_array[3] = 0xffffffff;
dsi_set_cmdq(data_array, 4, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00008000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00103902;     
data_array[1]= 0x0c0a02cc;     
data_array[2]= 0x0000100e;     
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x00009000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00103902;     
	data_array[1] = 0x000000cc;
data_array[2]= 0x062d2e00;     
data_array[3]= 0x0d0b0901;     
data_array[4]= 0x0000000f;     
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000a000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x000f3902;     
	data_array[1] = 0x000000cc;
data_array[2]= 0x00000000;     
	data_array[3] = 0x00000000;
data_array[4]= 0x00052d2e;     
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000b000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00103902;     
data_array[1]= 0x0d0f05cc;     
data_array[2]= 0x0000090b;     
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);         
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000c000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00103902;     
	data_array[1] = 0x000000cc;
data_array[2]= 0x012e2d00;     
data_array[3]= 0x0c0e1006;     
data_array[4]= 0x0000000a;     
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;     
data_array[1]= 0x0000d000;     
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x000f3902;     
	data_array[1] = 0x000000cc;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
data_array[4]= 0x00022e2d;     
dsi_set_cmdq(data_array, 5, 1);
	
data_array[0]= 0x00023902;      
data_array[1]= 0x00008000;      
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x000d3902;      
data_array[1]= 0x100387ce;      
data_array[2]= 0x00100386;      
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 4, 1); 
	
data_array[0]= 0x00023902;      
data_array[1]= 0x00009000;      
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x000f3902;      
data_array[1]= 0x100135ce;      
data_array[2]= 0x00100235;      
data_array[3]= 0x00000000;      
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1); 
	
data_array[0]= 0x00023902;      
data_array[1]= 0x0000a000;      
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x000f3902;      
data_array[1]= 0x040338ce;      
data_array[2]= 0x001000f8;      
data_array[3]= 0xf9040238;      
data_array[4]= 0x00001000;      
dsi_set_cmdq(data_array, 5, 1); 
	
data_array[0]= 0x00023902;      
data_array[1]= 0x0000b000;      
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x000f3902;      
data_array[1]= 0x040138ce;      
data_array[2]= 0x001000fa;      
data_array[3]= 0xfb040038;      
data_array[4]= 0x00001000;      
dsi_set_cmdq(data_array, 5, 1); 
	
data_array[0]= 0x00023902;      
data_array[1]= 0x0000c000;      
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x000f3902;      
data_array[1]= 0x040030ce;      
data_array[2]= 0x001000fc;      
data_array[3]= 0xfd040130;      
data_array[4]= 0x00001000;      
dsi_set_cmdq(data_array, 5, 1); 
	
data_array[0]= 0x00023902;      
data_array[1]= 0x0000d000;      
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x000f3902;      
data_array[1]= 0x040230ce;      
data_array[2]= 0x001000fe;      
data_array[3]= 0xff040330;      
data_array[4]= 0x00001000;      
dsi_set_cmdq(data_array, 5, 1); 
	
data_array[0]= 0x00023902;        
data_array[1]= 0x00008000;        
dsi_set_cmdq(data_array, 2, 1);   
data_array[0]= 0x000f3902;        
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 4, 1);   
	
data_array[0]= 0x00023902;        
data_array[1]= 0x00009000;        
dsi_set_cmdq(data_array, 2, 1);   
data_array[0]= 0x000f3902;        
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);   
	
data_array[0]= 0x00023902;        
data_array[1]= 0x0000a000;        
dsi_set_cmdq(data_array, 2, 1);   
data_array[0]= 0x000f3902;        
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);   
	
data_array[0]= 0x00023902;        
data_array[1]= 0x0000b000;        
dsi_set_cmdq(data_array, 2, 1);   
data_array[0]= 0x000f3902;        
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
dsi_set_cmdq(data_array, 5, 1);   
	
data_array[0]= 0x00023902;        
data_array[1]= 0x0000c000;        
dsi_set_cmdq(data_array, 2, 1);   
data_array[0]= 0x000c3902;        
	data_array[1] = 0x200101cf;
	data_array[2] = 0x01000020;
	data_array[3] = 0x08030081;
dsi_set_cmdq(data_array, 4, 1);   
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000b500;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00073902;                              
data_array[1]= 0xffff3fc5;                              
data_array[2]= 0x00ffff3f;                                                          
dsi_set_cmdq(data_array, 3, 1); 
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x00009000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00053902;                              
	data_array[1] = 0x021102f5;
data_array[2]= 0x00000015;                                                          
dsi_set_cmdq(data_array, 3, 1); 
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x00009000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00023902;                              
data_array[1]= 0x000050c5;                                                                                      
dsi_set_cmdq(data_array, 2, 1);    
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x00009400;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00023902;                              
data_array[1]= 0x000066c5;                                                                                      
dsi_set_cmdq(data_array, 2, 1);  
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000b200;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00033902;                              
data_array[1]= 0x000000f5;                                                                                      
dsi_set_cmdq(data_array, 2, 1);  
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000b400;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00033902;                              
data_array[1]= 0x000000f5;                                                                                      
dsi_set_cmdq(data_array, 2, 1);    
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000b600;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00033902;                              
data_array[1]= 0x000000f5;                                                                                      
dsi_set_cmdq(data_array, 2, 1);   
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000b800;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00033902;                              
	data_array[1] = 0x000000f5;
dsi_set_cmdq(data_array, 2, 1); 
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x00009400;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00033902;                              
	data_array[1] = 0x000000f5;
dsi_set_cmdq(data_array, 2, 1);   
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000d200;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00033902;                              
data_array[1]= 0x001506f5;                                                                                      
dsi_set_cmdq(data_array, 2, 1);   
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000b400;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000ccc5;                                                                                      
dsi_set_cmdq(data_array, 2, 1);      
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x00000000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00153902;                              
data_array[1]= 0x221600e1;  
data_array[2]= 0x53504132;                              
data_array[3]= 0x6f917582; 
data_array[4]= 0x41446858;                              
data_array[5]= 0x09162533; 
data_array[6]= 0x00000000;                                                                                                                 
dsi_set_cmdq(data_array, 7, 1); 
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x00000000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00153902;                              
data_array[1]= 0x221500e2;  
data_array[2]= 0x53504132;                              
data_array[3]= 0x6f917582; 
data_array[4]= 0x41446858;                              
data_array[5]= 0x09162533; 
data_array[6]= 0x00000000;                                                                                                                 
dsi_set_cmdq(data_array, 7, 1); 
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000a000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x000d3902;                              
data_array[1]= 0x010d01d6;
data_array[2]= 0x01a6010d;                              
data_array[3]= 0x01a601a6;
data_array[4]= 0x000000a6;                                                                                  
dsi_set_cmdq(data_array, 5, 1);    
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000b000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x000d3902;                              
data_array[1]= 0x01a601d6;
data_array[2]= 0x01a601a6;                              
data_array[3]= 0x01a601a6;
data_array[4]= 0x000000a6;                                                                                  
dsi_set_cmdq(data_array, 5, 1);  
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000c000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x000d3902;                              
data_array[1]= 0x091155d6;
data_array[2]= 0x6f6f116f;                              
data_array[3]= 0x116f6f11;
data_array[4]= 0x0000006f;                                                                                  
dsi_set_cmdq(data_array, 5, 1); 

data_array[0]= 0x00023902;                              
data_array[1]= 0x0000d000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00073902;                              
data_array[1]= 0x6f116fd6;
data_array[2]= 0x006f116f;                                                                                                                
dsi_set_cmdq(data_array, 3, 1); 
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x0000e000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x000d3902;                              
data_array[1]= 0x37112bd6;
data_array[2]= 0x37371137;                              
data_array[3]= 0x11373711;
data_array[4]= 0x00000037;                                                                                  
dsi_set_cmdq(data_array, 5, 1); 

data_array[0]= 0x00023902;                              
data_array[1]= 0x0000f000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00073902;                              
data_array[1]= 0x371137d6;
data_array[2]= 0x00371137;                                                                                                                
dsi_set_cmdq(data_array, 3, 1); 
	
data_array[0]= 0x00023902;                              
data_array[1]= 0x00000000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00023902;                              
data_array[1]= 0x00008181;                                                                                                                
dsi_set_cmdq(data_array, 2, 1); 

data_array[0]= 0x00023902;                              
data_array[1]= 0x00000000;                              
dsi_set_cmdq(data_array, 2, 1);                         
data_array[0]= 0x00033902;                              
data_array[1]= 0x00ffffff;                                                                                                                
dsi_set_cmdq(data_array, 2, 1); 	
#endif
}
#endif


static void lcm_init(void)
{
	unsigned int data_array[16];
	printk("%s, lead  otm1284A \n", __func__);
	SET_RESET_PIN(1);
	MDELAY(10);//5
	SET_RESET_PIN(0);
	MDELAY(10);//50
	SET_RESET_PIN(1);
	MDELAY(10);//100

	lcm_init_registers();

	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(5);
    
    SET_RESET_PIN(0);
	MDELAY(1);
    SET_RESET_PIN(1);
}



static void lcm_resume(void)
{

	lcm_init();
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);	
}
#endif
static void lcm_setbacklight(unsigned int level)
{
#ifdef BUILD_LK
	dprintf(0,"%s,lk nt35595 backlight: level = %d\n", __func__, level);
#else
	printk("%s, kernel nt35595 backlight: level = %d\n", __func__, level);
#endif
	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = level;
	
	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);

}

static void lcm_set_cmd(void* handle,int *lcm_cmd,unsigned int cmd_num)
{
#ifdef BUILD_LK
	dprintf(0,"%s,lk nt35595 set cmd: num = %d\n", __func__, cmd_num);
#else
	printk("%s, kernel nt35595 set cmd: num = %d\n", __func__, cmd_num);
    if(cmd_num==0)
		return;
//customize example 
	unsigned int cmd = 0x51;
	unsigned int count =1;
    unsigned int level = lcm_cmd[0];
	printk("[lcm_set_cmd mode = %d\n", lcm_cmd[0]);
	dsi_set_cmd_by_cmdq(handle, cmd, count, &level, 1);
//	
#endif
}
LCM_DRIVER otm1284a_hd720_dsi_vdo_tm_lcm_drv = 
{
    .name			= "otm1284a_dsi_vdo_tm",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.init_power		= lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
	.set_backlight	= lcm_setbacklight,
	     .set_cmd = lcm_set_cmd,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};

