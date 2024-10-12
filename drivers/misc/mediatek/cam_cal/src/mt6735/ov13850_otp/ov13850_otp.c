/*
 * Driver for CAM_CAL
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "ov13850_otp.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


#define CAM_CALGETDLT_DEBUG
//#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALDB printk
#else
#define CAM_CALDB(x,...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 0

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x6d>>1)};

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
//spin_lock(&g_CAM_CALLock);
//spin_unlock(&g_CAM_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);

#define LSCOTPDATASIZE 512

int otp_flag=0;
//int otp_lsc_flag=1;
extern int otp_flag_cali;


static kal_uint8 lscotpdata[LSCOTPDATASIZE];
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

extern  void write_cmos_sensor(kal_uint32 addr, kal_uint32 para);
 extern kal_uint16 read_cmos_sensor(kal_uint32 addr);

extern kal_uint16 OV13850_R2A_read_i2c(kal_uint32 addr);


extern kal_uint16 OV13850_R2A_write_i2c(addr, para);




#if 0
#define iWriteCAM_CAL(addr, bytes, para,) iWriteReg((u16) addr , (u32) para , 1, OV13850OTP_DEVICE_ID)
#else

/*******************************************************************************
*
********************************************************************************/
//Address: 2Byte, Data: 1Byte
static int iWriteCAM_CAL(u16 a_u2Addr  , u32 a_u4Bytes, u8 puDataInBytes)
{   

 //  CAM_CALDB("[CAM_CAL][iWriteCAM_CAL] Write 0x%x=0x%x \n",a_u2Addr, puDataInBytes);
 //  iWriteReg(a_u2Addr,puDataInBytes,1,OV13850OTP_DEVICE_ID);
   return 0;
}
#endif

//Address: 2Byte, Data: 1Byte
static int iReadCAM_CAL(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
#if 0
    	int  i4RetValue = 0;

	i4RetValue=iReadReg(a_u2Addr,a_puBuff,OV13850OTP_DEVICE_ID);
	
	if (i4RetValue !=0)
		{
			CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
			return -1;
		} 	
	CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
#endif	
    return 0;
}

int iReadCAM_CAL_8(u8 a_u2Addr, u8 * a_puBuff, u16 i2c_id)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)(a_u2Addr)};

#if 0
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient->addr = i2c_id>>1;
	g_pstI2Cclient->timing=400;
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);
	
    if (i4RetValue != 1)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
	    CAM_CALDB("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !! \n", a_u2Addr,  *a_puBuff );
        return -1;
    }

    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);
	CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
    if (i4RetValue != 1)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    } 
#endif
    return 0;
}

//go to page
#if 1
static kal_uint8 OV13850_GotoPage(kal_uint8 page)
{
  return 1;
}


static kal_uint8 check_OV13850_otp_valid_AFPage(void)
{	
#if 0
    kal_uint8 AF_OK = 0x00;
	u8 readbuff, i;

	//iReadCAM_CAL_8(0x28,&readbuff,0xA0);
	iReadCAM_CAL_8(0x22,&readbuff,0xA0);
	
	AF_OK = readbuff;
	
	if (AF_OK==1)
	{
		CAM_CALDB("can read AF otp from page\n");
	}
	else
	    return KAL_FALSE;
#endif	
	return KAL_TRUE;

}


static kal_uint8 check_OV13850_otp_valid_AWBPage(void)
{
#if 0
	kal_uint8 AWB_OK = 0x00;
	u8 readbuff, i;
	kal_uint16 LSC_lengthL,LSC_lengthH;

//	iReadCAM_CAL_8(0x11,&readbuff,0xA0);
iReadCAM_CAL_8(0x21,&readbuff,0xA0);
	
	AWB_OK = readbuff;
	
	if (AWB_OK==1)
	{
		CAM_CALDB("can read AWB otp from page\n");
	}
	else
	    return KAL_FALSE;
#endif	
	return KAL_TRUE;

}

#endif

kal_bool check_OV13850_otp_valid_LSC_Page(kal_uint8 page)
{
	kal_uint8 LSC_OK = 0x00;
	u8 readbuff, i;
	
	return KAL_TRUE;
}


 void OV13850_ReadOtp(kal_uint8 page,kal_uint16 address,unsigned char *iBuffer,unsigned int buffersize)
{
#if 0
		kal_uint16 i = 0;
		u8 readbuff, base_add;
		int ret ;
		//base_add=(address/10)*16+(address%10);
		printk("==zlx==OV13850_ReadOtp\n");
		//printk("[CAM_CAL]ENTER page:0x%x address:0x%x buffersize:%d\n ",page, address, buffersize);
		if (OV13850_GotoPage(page))
		{
			for(i = 0; i<buffersize; i++)
			{				
				ret= iReadCAM_CAL_8(address+i,&readbuff,0xA0);
				//CAM_CALDB("[CAM_CAL]address+i = 0x%x,readbuff = 0x%x\n",(address+i),readbuff );
				//	printk("==zlx==address+i = 0x%x,readbuff = 0x%x\n",(address+i),readbuff );
	
				*(iBuffer+i) =(unsigned char)readbuff;
			}
		}
#endif
}


 void  read_af_start(unsigned char *data)
{
       int otp_flag=0, temp=0, VCM_start=0, addr=0;
	// OTP VCM Calibration
	otp_flag = OV13850_R2A_read_i2c(0x73ac);
	printk(" VCM calibration data : %x \n", otp_flag);
	//addr = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x73ad; // base address of VCM Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x73b0; // base address of VCM Calibration group 2
	}
	if(addr != 0) {
		//(*otp_ptr).flag |= 0x20;
		temp = OV13850_R2A_read_i2c(addr + 2);
		VCM_start = (OV13850_R2A_read_i2c(addr)<<2) | ((temp>>6) & 0x03);
	//	(* otp_ptr).VCM_end = (OV13850_R2A_read_i2c(addr + 1) << 2) | ((temp>>4) & 0x03);
		//(* otp_ptr).VCM_dir = (temp>>2) & 0x03;
	}
	else {
		VCM_start = 0;
	//	(* otp_ptr).VCM_end = 0;
	//	(* otp_ptr).VCM_dir = 0;
	}

	//*data =(unsigned char)VCM_start;
 	memcpy(data, (int *) &VCM_start, 2);
 	//*data = VCM_start & 0xff;
	//*(data + 1) = (VCM_start >> 8) & 0xff;

	printk("VCM_start : %d----lhl-*data=%d-- \n", VCM_start,*data);
}

int  read_af_end(unsigned char *data)
{
       int otp_flag=0, temp=0, VCM_end=0, addr=0;

	// OTP VCM Calibration
	otp_flag = OV13850_R2A_read_i2c(0x73ac);
	printk(" VCM calibration data : %x \n", otp_flag);
	//addr = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x73ad; // base address of VCM Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x73b0; // base address of VCM Calibration group 2
	}
	if(addr != 0) {
		//(*otp_ptr).flag |= 0x20;
		temp = OV13850_R2A_read_i2c(addr + 2);
	//	(* otp_ptr).VCM_start = (OV13850_R2A_read_i2c(addr)<<2) | ((temp>>6) & 0x03);
		VCM_end = (OV13850_R2A_read_i2c(addr + 1) << 2) | ((temp>>4) & 0x03);
	//	(* otp_ptr).VCM_dir = (temp>>2) & 0x03;
	}
	else {
	//	(* otp_ptr).VCM_start = 0;
		VCM_end = 0;
	//	(* otp_ptr).VCM_dir = 0;
	}
	//*data =(unsigned char)VCM_end;
	memcpy(data, (unsigned char *) &VCM_end, 2);
	//*data = VCM_end & 0xff;
	//*(data + 1) = (VCM_end >> 8) & 0xff;

	printk(" ----VCM_end=%d--lhl-*data=%d---- \n", VCM_end,*data);
}


//Burst Write Data
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
}

//Burst Read Data
 int iReadData(kal_uint16 ui4_offset, unsigned int  ui4_length, unsigned char *pinputdata)
{
#if 1
   int  i4RetValue = 0;
    kal_uint8 page = 0, pageS=0,pageE=4;;
	//1. check which page is valid
	
	if(ui4_length ==2)	//af macor
    {
     
	    read_af_end(pinputdata);
   	
   	}
   	if(ui4_length ==4)	//af inf
    {
     
	    read_af_start(pinputdata);
   	
   	}

    // 2. read otp

  //  for(i4RetValue = 0;i4RetValue<ui4_length;i4RetValue++){
 //   printk( "[[CAM_CAL]]pinputdata[%d]=%x\n", i4RetValue,*(pinputdata+i4RetValue));}
  //  printk(" [[CAM_CAL]]page = %d,ui4_length = %d,ui4_offset =%d\n ",page,ui4_length,ui4_offset);
   // printk("[S24EEPORM] iReadData done\n" );
#endif




   return 0;
}


#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    printk("===zlx===[CAMERA SENSOR] OV13850_Ioctl_Compat\n");
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	 // printk("===zlx===[CAMERA SENSOR] ov13850_Ioctl_Compat,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
       printk("===lhl----0819---==[CAMERA SENSOR] COMPAT_CAM_CALIOC_G_READ\n");

        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            printk("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}


#endif


/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
	
#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif
 printk("==llhl==[ov13850---CAM_CAL_Ioctl] CAM_CAL_Ioctl\n");
    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
    // printk("[S24CAM_CAL] init Working buffer address 0x%8x  command is 0x%8x\n", (u32)pWorkingBuff, (u32)a_u4Command);

  printk("===zlx===[S24CAM_CAL] CAM_CAL_Ioctl, command is 0x%8x\n",(u32)a_u4Command);

    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
            printk("[S24CAM_CAL] Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif            
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            
            break;
        case CAM_CALIOC_G_READ:
            printk("[S24CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif     
            printk("[CAM_CAL] offset %d---CAM_CALIOC_G_READ-----lhl---0819---- \n", ptempbuf->u4Offset);
            printk("[CAM_CAL] length %d \n", ptempbuf->u4Length);
          //  printk("[CAM_CAL] Before read Working buffer address 0x%8x \n", (u32)pWorkingBuff);
            i4RetValue = iReadData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
			
         //   printk("[S24CAM_CAL] After read Working buffer data  0x%4x \n", *pWorkingBuff);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            

            break;
        default :
      	     printk("[S24CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[S24CAM_CAL] to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("[S24CAM_CAL] to user  Working buffer address 0x%8x \n", (u32)pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            CAM_CALDB("[S24CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    printk("===zlx===[S24CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[S24CAM_CAL] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
     //.ioctl = CAM_CAL_Ioctl
    #ifdef CONFIG_COMPAT
    .compat_ioctl = Ioctl_Compat,
   #endif
   .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 0
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[S24CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[S24CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};   
#if 0 //test110314 Please use the same I2C Group ID as Sensor
static unsigned short force[] = {CAM_CAL_I2C_GROUP_ID, OV13850OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#else
//static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, OV5647OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#endif
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,}; 


static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,                                   
    .remove = CAM_CAL_i2c_remove,                           
//   .detect = CAM_CAL_i2c_detect,                           
    .driver.name = CAM_CAL_DRVNAME,
    .id_table = CAM_CAL_i2c_id,                             
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
int i4RetValue = 0;
    CAM_CALDB("[S24CAM_CAL] Attach I2C \n");
//    spin_lock_init(&g_CAM_CALLock);
printk("==zlx==CAM_CAL_i2c_probe\n");
    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = OV13850OTP_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("[S24CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }


    CAM_CALDB("[S24CAM_CAL] Attached!! \n");
    return 0;                                                                                       
} 

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
printk("==zlx==CAM_CAL_probe\n");

    return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{
    i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
printk("==zlx==CAM_CAL_i2C_init\n");
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register S24CAM_CAL driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register S24CAM_CAL driver, 2nd time\n");
        return -ENODEV;
    }	

    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


