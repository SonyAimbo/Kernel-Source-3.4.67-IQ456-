#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include "tpd_custom_ft6306.h"

#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#else
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>
#endif

#include "cust_gpio_usage.h"

//lenovo_sw liaohj merged from putaoya 2012-09-12 

//#define FTS_CTL_IIC
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
 
extern int tpd_firmware_version[2]; 
extern struct tpd_device *tpd;
 
struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
static void tpd_eint_interrupt_handler(void);
 
#ifdef MT6575 
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
								  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
								  kal_bool auto_umask);
#endif
#ifdef MT6577
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 
static int boot_mode = 0;
static int tpd_halt=0; 
static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;
static bool discard_resume_first_eint = KAL_FALSE;
static int tpd_state = 0;
//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID     0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12

/*add for portugal evt*/
/* Lenovo-sw yexm1 modify, 2012-10-18, open the FW upgrade fun */
//#define CONFIG_SUPPORT_FTS_CTP_UPG

//#define ESD_CHECK

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 3

#ifdef ESD_CHECK
static struct delayed_work ctp_read_id_work;
static struct workqueue_struct * ctp_read_id_workqueue = NULL;
#endif

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

//#define VELOCITY_CUSTOM_FT6306

#ifdef VELOCITY_CUSTOM_FT6306
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;
static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
	file->private_data = adxl345_i2c_client;
	if(file->private_data == NULL)
	{
		printk("tpd: null pointer!!\n");
		return -EINVAL;
	}
	*/
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
	//char strbuf[256];
	void __user *data;
	
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
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


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[5];
    int x[5];
    int p[5];
	int id[5];
    int count;
};
 
static const struct i2c_device_id FT6306_tpd_id[] = {{"FT6306",0},{}};
//unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata FT6306_i2c_tpd={ I2C_BOARD_INFO("FT6306", (0x70>>1))};

static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = "FT6306",//.name = TPD_DEVICE,
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = FT6306_tpd_id,
  .detect = tpd_detect,
//  .address_data = &addr_data,
};

int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

//read ctp sw version

/***********************************************************************************************
Name	:	ft5x0x_read_reg 
Input	:	addr
                     pdata
Output	:	
function	:	read register of ft5x0x
***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2] = {0};

	buf[0] = addr;
	struct i2c_msg msgs[] = {
		{
			.addr	= i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
		},
	};

    //msleep(1);
	ret = i2c_transfer(i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
  
}

/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver
Input	:	 void
                     
Output	:	 firmware version 	
function	:	 read TP firmware version
***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(0xa6, &ver);
    
    tpd_firmware_version[0] = ver;
    
	return(ver);
}
static unsigned char ft5x0x_read_ID_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(0xa8, &ver);
    
    tpd_firmware_version[1] = ver;
    
	return(ver);
}
 
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
static u8 *CTPI2CDMABuf_va = NULL;
static u32 CTPI2CDMABuf_pa = NULL;
typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS        0x70

/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata 
Input	:	*rxdata
                     *length
Output	:	ret
function	:	
***********************************************************************************************/
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}
/***********************************************************************************************
Name	:	 
Input	:	
                     
Output	:	
function	:	
***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= i2c_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};
   	//msleep(1);
	ret = i2c_transfer(i2c_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_write_reg
Input	:	addr -- address
                     para -- parameter
Output	:	
function	:	write register of ft5x0x
***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}

void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}

/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(i2c_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(i2c_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

static int CTPDMA_i2c_write(FTS_BYTE slave,FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
	int i = 0;
	for(i = 0 ; i < dw_len; i++)
	{
		CTPI2CDMABuf_va[i] = pbt_buf[i];
	}

	if(dw_len <= 8)
	{
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
		return i2c_master_send(i2c_client, pbt_buf, dw_len);
	}
	else
	{
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
		return i2c_master_send(i2c_client, CTPI2CDMABuf_pa, dw_len);
	}    
}

static int CTPDMA_i2c_read(FTS_BYTE slave, FTS_BYTE *buf, FTS_DWRD len)
{
	int i = 0, err = 0;

	if(len < 8)
	{
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
		//MSE_ERR("Sensor non-dma read timing is %x!\r\n", this_client->timing);
		return i2c_master_recv(i2c_client, buf, len);
	}
	else
	{
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		//MSE_ERR("Sensor dma read timing is %x!\r\n", this_client->timing);
		err = i2c_master_recv(i2c_client, CTPI2CDMABuf_pa, len);
		
	    if(err < 0)
	    {
			return err;
		}

		for(i = 0; i < len; i++)
		{
			buf[i] = CTPI2CDMABuf_va[i];
		}
	}
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error. 
*/


#define FT_UPGRADE_AA	0xAA
#define FT_UPGRADE_55 	0x55

/*upgrade config of FT6X06*/
#define FT6X06_UPGRADE_AA_DELAY 	60
#define FT6X06_UPGRADE_55_DELAY 	10
#define FT6X06_UPGRADE_ID_1			0x79
#define FT6X06_UPGRADE_ID_2			0x08
#define FT6X06_UPGRADE_READID_DELAY 10
#define FT6X06_UPGRADE_EARSE_DELAY	2000

#define FTS_PACKET_LENGTH       128
#define FTS_SETTING_BUF_LEN     128

#define FTS_UPGRADE_LOOP	    10

#define FTS_FACTORYMODE_VALUE	0x40
#define FTS_WORKMODE_VALUE		0x00

static unsigned char CTPM_FW[]=
{
#include "ver12_20130410_11204_v990_id0x57_app.i"
};

int ft6x06_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ft6x06_i2c_Write(client, buf, sizeof(buf));
}

int ft6x06_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return ft6x06_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

int fts_ctpm_fw_read_app(u8 *pbt_buf, u32 dw_lenth)
{
	u32 packet_number;
	u32 j = 0;
	u32 temp;
	u32 lenght = 0;
	u8 *pReadBuf = NULL;
	u8 auc_i2c_write_buf[10];
	int i_ret;

	dw_lenth = dw_lenth - 2;

	pReadBuf = kmalloc(dw_lenth + 1, GFP_ATOMIC);
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;

	/*Read flash*/
	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		auc_i2c_write_buf[2] = (u8) (temp >> 8);
		auc_i2c_write_buf[3] = (u8) temp;

              CTPDMA_i2c_write(0x70,auc_i2c_write_buf, 4);
		
		i_ret = CTPDMA_i2c_read(0x70, pReadBuf+lenght, FTS_PACKET_LENGTH);
		if (i_ret < 0)
			return -EIO;
		msleep(FTS_PACKET_LENGTH / 6 + 1);
		lenght += FTS_PACKET_LENGTH;
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		auc_i2c_write_buf[2] = (u8) (temp >> 8);
		auc_i2c_write_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;

		CTPDMA_i2c_write(0x70,auc_i2c_write_buf, 4);
		i_ret = CTPDMA_i2c_read(0x70,	pReadBuf+lenght, temp);
		if (i_ret < 0)
			return -EIO;
		msleep(FTS_PACKET_LENGTH / 6 + 1);
		lenght += temp;
	}

	/*read the last six byte */
	temp = 0x6ffa + j;
	auc_i2c_write_buf[2] = (u8) (temp >> 8);
	auc_i2c_write_buf[3] = (u8) temp;
	temp = 6;
	i_ret = CTPDMA_i2c_write( 0x70,auc_i2c_write_buf, 4); 
	byte_read(pReadBuf+lenght, temp);
	if (i_ret < 0)
		return -EIO;
	msleep(FTS_PACKET_LENGTH / 6 + 1);
	lenght += temp;

       for (j=0; j<dw_lenth-2; j++) {
	   	printk("0x%0x",pReadBuf[j]);
		if((j+1)%16 == 0)
			printk("\n");
       	}
	/*read app from flash and compart*/
	for (j=0; j<dw_lenth-2; j++) {
		if(pReadBuf[j] != pbt_buf[j]) {
			kfree(pReadBuf);
			return -EIO;
		}
	}

	kfree(pReadBuf);
	return 0;
}

int fts_ctpm_fw_upgrade(struct i2c_client *client,u8 *pbt_buf,u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	int k ;
    int ret;
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xbc */
#if 1
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
    msleep(3);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(3);  
    //mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(40);
#else
    ft6x06_write_reg(client, 0xbc, FT_UPGRADE_AA);
    msleep(FT6X06_UPGRADE_AA_DELAY);
    /*write 0x55 to register 0xbc */
    ft6x06_write_reg(client, 0xbc, FT_UPGRADE_55);
    msleep(FT6X06_UPGRADE_55_DELAY);
#endif
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		do {
			k++;
			i_ret = ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && k< 5);


		/*********Step 3:check READ-ID***********************/
		msleep(200);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == FT6X06_UPGRADE_ID_1
			&& reg_val[1] == FT6X06_UPGRADE_ID_2) {
			//dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				//reg_val[0], reg_val[1]);
			printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i == FTS_UPGRADE_LOOP)
		return -EIO;
	auc_i2c_write_buf[0] = 0xcd;

	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);

	/*Step 4:erase app and panel paramenter area*/
	printk("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(FT6X06_UPGRADE_EARSE_DELAY);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = 0x63;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        ret=CTPDMA_i2c_write(0x70, &packet_buf[0],FTS_PACKET_LENGTH + 6);
 //             printk("[TSP] 111 ret 0x%x \n", ret);
        msleep(5);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
     //         printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
//             printk("[TSP]temp 0x%x \n", temp);
        ret = CTPDMA_i2c_write(0x70, &packet_buf[0],temp+6);    
 //            printk("[TSP] 222 ret 0x%x \n", ret);
        msleep(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        CTPDMA_i2c_write(0x70,&packet_buf[0],7);  
        msleep(20);
    }

	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	printk("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
#if 0
	printk("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */
#else
 	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(1);  
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif

	return 0;
}

static int fts_ctpm_fw_upgrade_with_i_file(void)
{
    FTS_BYTE*     pbt_buf = FTS_NULL;
    int i_ret;
    unsigned char version=0;
    FTS_BYTE flag;
    FTS_DWRD i = 0;
    //=========FW upgrade========================*/
    pbt_buf = CTPM_FW;

    int fw_len = sizeof(CTPM_FW);
	//printk("version=%x ,pbt_buf[sizeof(CTPM_FW)-2]=%d\n",version,pbt_buf[sizeof(CTPM_FW)-2]);
	//printk("[TSP]ID_ver=%x, fw_ver=%x\n", ft5x0x_read_ID_ver(), ft5x0x_read_fw_ver());
#if 0 /*zhouwl, temp disable this line*/
    if(0xa8 != ft5x0x_read_ID_ver())
    {
    	if(ft5x0x_read_ID_ver() != pbt_buf[sizeof(CTPM_FW)-1])
    	{
            return;
    	}
    	
        do
        {
            i ++;
            version =ft5x0x_read_fw_ver();
            delay_qt_ms(2);
        }while( i < 5 );
        
    	if(version==pbt_buf[sizeof(CTPM_FW)-2])
    	{
    		return;
    	}
    }
#endif
	if ((CTPM_FW[fw_len - 8] ^ CTPM_FW[fw_len - 6]) == 0xFF
		&& (CTPM_FW[fw_len - 7] ^ CTPM_FW[fw_len - 5]) == 0xFF
		&& (CTPM_FW[fw_len - 3] ^ CTPM_FW[fw_len - 4]) == 0xFF) {
		/*FW upgrade */
		/*call the upgrade function */
		   i_ret =  fts_ctpm_fw_upgrade(i2c_client,pbt_buf,sizeof(CTPM_FW));
		if (i_ret != 0)
			dev_err(&i2c_client->dev, "%s:upgrade failed. err.\n",__func__);
#ifdef AUTO_CLB
		else
			fts_ctpm_auto_clb(client);	/*start auto CLB */
#endif
	} else {
		printk(&i2c_client->dev, "%s:FW format error\n", __func__);
		return -EBADFD;
	}

	msleep(200);  
    ft5x0x_write_reg(0xfc,0x04);
	msleep(4000);
	flag=0;
	i2c_smbus_read_i2c_block_data(i2c_client, 0xFC, 1, &flag);
	//printk("flag=%d\n",flag);
    return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

static int touch_firmware_upgrade_thread(void *unused)
{
    u8 reg_version = 0, reg_value, lib_version = 0;
    int err = 0, reg_val[2];
    
    msleep(200);
    unsigned char*     pbt_buf = FTS_NULL;
    pbt_buf = CTPM_FW;

    reg_version = ft5x0x_read_fw_ver();
    lib_version = pbt_buf[sizeof(CTPM_FW)-2];
	
	printk("version=%x ,pbt_buf[sizeof(CTPM_FW)-2]=%d\n",reg_version,pbt_buf[sizeof(CTPM_FW)-2]);
    printk("read lib file version=0x%x ,read chip register FT6306_version= 0x%x by cheehwa\n", lib_version, reg_version);

    if((lib_version > reg_version) )
    {
        printk("upgrading,the verison not match 0x%2x != 0x%2x\n", lib_version, reg_version);
        msleep(200);
        err = fts_ctpm_fw_upgrade_with_i_file();
        printk("Return fts_ctpm_fw_upgrade_with_i_file_ err=%d by cheehwa\n", err);

        if(err == 0)
        {
            printk("[TSP] ugrade successfuly.\n");
            msleep(300);
            //msleep(50);
            reg_value = ft5x0x_read_fw_ver();
            printk("FTS_DBG from old_version 0x%2x to new_version = 0x%2x\n", reg_version, reg_value);
        }
        else
        {
            printk("[TSP]  ugrade fail err=%d, line = %d.\n",
                   err, __LINE__);
        }

       	// msleep(4000);
        msleep(500);
    }

	kthread_should_stop(); 	
}
#endif

#ifdef ESD_CHECK	
static void ESD_read_id_workqueue(struct work_struct *work)
{
	char data;
	if(tpd_halt) 
		return; 
	i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &data);
//	TPD_DEBUG("ESD_read_id_workqueue data: %d\n", data);
	printk("ESD_read_id_workqueue data: %d\n", data);
	if((data > 5)&&(data < 10))
	{
		//add_timer();
	}
	else
	{
        mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        if(tpd_state)
        {
            input_mt_sync(tpd->dev);
            input_sync(tpd->dev);
            tpd_state = 0;
        }
        msleep(5);  
	
//#ifdef MT6575
		    //power on, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
#else
        hwPowerDown(MT65XX_POWER_LDO_VGP2,  "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
        hwPowerDown(TPD_POWER_SOURCE_1800,  "TP");
#endif    
        msleep(5);  
#ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
        hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
        hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif    
//#endif	
        msleep(100);
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
        msleep(10);  
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 

        msleep(200);
	}
	if(tpd_halt) 
		mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 
	else 
		queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection					
}
#endif

static  void tpd_down(int x, int y, int p) {
    //if (RECOVERY_BOOT != get_boot_mode()){
        // input_report_abs(tpd->dev, ABS_PRESSURE, p);
        input_report_key(tpd->dev, BTN_TOUCH, 1);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
        input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
        input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
        /* Lenovo-sw yexm1, optimize the code, 2012-9-19 begin */
        // printk("D[%4d %4d %4d] ", x, y, p);
        /* Lenovo-sw yexm1, optimize the code, 2012-9-19 end */
        /* track id Start 0 */
        input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
        input_mt_sync(tpd->dev);
    //}
    //if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
    //{   
    //    tpd_button(x, y, 1);  
    //}
    if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
    {
        /* Lenovo-sw yexm1 modify, 2012-10-15, delete the delay */
        //msleep(50);
        printk("D virtual key \n");
    }
	TPD_EM_PRINT(x, y, x, y, p-1, 1);
}

static  void tpd_up(int x, int y,int *count) {
    //if (RECOVERY_BOOT != get_boot_mode()){
	 //if(*count>0) {
		 //input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
		 //input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		 //printk("U[%4d %4d %4d] ", x, y, 0);
		 input_mt_sync(tpd->dev);
		 TPD_EM_PRINT(x, y, x, y, 0, 0);
	//	 (*count)--;
     //}
     //if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     //{   
     //   tpd_button(x, y, 0); 
     //}   		 
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
    int i = 0;
	
	char data[40] = {0};

    u16 high_byte,low_byte;
	u8 report_rate =0;

	p_point_num = point_num;

	/* Lenovo-sw yexm1, optimize the code, 2012-9-19 begin */
	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[32]));
	/* Lenovo-sw yexm1, optimize the code, 2012-9-19 end */
	i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &report_rate);
	//TPD_DEBUG("received raw data from touch panel as following:\n");
	TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
	//TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	//TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);

	//we have  to re update report rate
    //TPD_DMESG("report rate =%x\n",report_rate);
	if(report_rate < 8)
	{
        report_rate = 0x8;
        if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
        {
           TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
        }
	 }
	
	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if((data[0] & 0x70) != 0) return false; 

	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;
	
	//TPD_DEBUG("point_num =%d\n",point_num);
	
//	if(point_num == 0) return false;

	   //TPD_DEBUG("Procss raw data...\n");

		for(i = 0; i < point_num; i++)
		{
			cinfo->p[i] = data[3+6*i] >> 6; //event flag 
            cinfo->id[i] = data[3+6*i+2]>>4; //touch id
	       /*get the X coordinate, 2 bytes*/
			high_byte = data[3+6*i];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i + 1];
			cinfo->x[i] = high_byte |low_byte;

			/*get the Y coordinate, 2 bytes*/
			
			high_byte = data[3+6*i+2];
			
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i+3];
			cinfo->y[i] = high_byte |low_byte;

			cinfo->count++;
		}
		//TPD_DEBUG(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);	
		//TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
		//TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	
		  
	 return true;
};

static int touch_event_handler(void *unused)
{
    struct touch_info cinfo, pinfo;
    int i=0;

    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

    do
    {
        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
        set_current_state(TASK_INTERRUPTIBLE); 
        wait_event_interruptible(waiter,tpd_flag!=0);
        		 
        tpd_flag = 0;

        set_current_state(TASK_RUNNING);

        if (tpd_touchinfo(&cinfo, &pinfo)) 
        {
            //TPD_DEBUG("point_num = %d\n",point_num);
            TPD_DEBUG_SET_TIME;
            if(point_num >0) 
            {
                for(i =0; i<point_num && i<5; i++)//only support 3 point
                {
                    tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
                }
                input_sync(tpd->dev);
            }

            else  
            {
                tpd_up(cinfo.x[0], cinfo.y[0], 0);
                //TPD_DEBUG("release --->\n"); 
                //input_mt_sync(tpd->dev);
                input_sync(tpd->dev);
            }
        }

        if(tpd_mode==12)
        {
            //power down for desence debug
            //power off, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
            hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
#else
            hwPowerDown(MT65XX_POWER_LDO_VGP2,  "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
            hwPowerDown(TPD_POWER_SOURCE_1800,  "TP");
#endif    
            msleep(20);
        }
    }while(!kthread_should_stop());

    return 0;
}
 
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
    strcpy(info->type, TPD_DEVICE);	
    return 0;
}
 
static void tpd_eint_interrupt_handler(void)
{
    //TPD_DEBUG("TPD interrupt has been triggered\n");
    TPD_DEBUG_PRINT_INT;
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}
 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	int retval = TPD_OK;
#ifdef ESD_CHECK	
	int ret;
#endif
	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;

reset_proc:   
	i2c_client = client;

#if (defined (MT6575)||defined(MT6577))
    //power on, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
    hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
    hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
    hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif    
#endif	

#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
    hwPowerDown(TPD_POWER_SOURCE,"TP");
    hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
    msleep(100);
#else
#ifdef MT6573
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
    msleep(100);
#endif
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(10);  

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(200);
#endif

    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

    msleep(100);
    
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	CTPI2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &CTPI2CDMABuf_pa, GFP_KERNEL);
    	if(!CTPI2CDMABuf_va)
	{
    		printk("[TSP] dma_alloc_coherent error\n");
	}
#endif		
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( reset_count < TPD_MAX_RESET_COUNT )
        {
            reset_count++;
            goto reset_proc;
        }
#endif
        return -1; 
	}

	//set report rate 80Hz
	report_rate = 0x8; 
	if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	{
	    if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	    {
		   TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
	    }
	}

	tpd_load_status = 1;

	#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(i2c_client) < 0)
		TPD_DMESG(TPD_DEVICE, "%s:[FTS] create fts control iic driver failed\n",__func__);
	#endif

	#ifdef VELOCITY_CUSTOM_FT6306
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
		
	}
	#endif

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
    printk("I171_ft6306_upgrade_switch\r\n");

    kthread_run(touch_firmware_upgrade_thread, 0, TPD_DEVICE);
#endif
    msleep(200);
    //update firmware version for factory mode
	printk("[TSP]ID_ver=%x, fw_ver=%x\n", ft5x0x_read_ID_ver(), ft5x0x_read_fw_ver());
    

#ifdef ESD_CHECK	
    ctp_read_id_workqueue = create_workqueue("ctp_read_id");
    INIT_DELAYED_WORK(&ctp_read_id_work, ESD_read_id_workqueue);
    ret = queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection					
    printk("[TSP] ret =%d\n",ret);
#endif

    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread))
    { 
        retval = PTR_ERR(thread);
        TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
    }

    TPD_DMESG("FT6306 Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
    return 0;
}

static int __devexit tpd_remove(struct i2c_client *client)
{
    TPD_DEBUG("TPD removed\n");
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	if(CTPI2CDMABuf_va)
	{
		dma_free_coherent(NULL, 4096, CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
		CTPI2CDMABuf_va = NULL;
		CTPI2CDMABuf_pa = 0;
	}
#endif	
#ifdef ESD_CHECK	
	destroy_workqueue(ctp_read_id_workqueue);
#endif	

#ifdef FTS_CTL_IIC
    ft_rw_iic_drv_exit();
#endif

   return 0;
}
 
static int tpd_local_init(void)
{
    TPD_DMESG("Focaltech FT6306 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

    if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("FT6306 unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("FT6306 add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
    TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
    tpd_type_cap = 1;

    return 0; 
}

static void tpd_resume( struct early_suspend *h )
{
    //int retval = TPD_OK;
    char data;

    TPD_DMESG("TPD wake up\n");
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
    hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP"); 
#else
#ifdef MT6573
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif	
	discard_resume_first_eint = KAL_TRUE;

#ifdef MT6575
    //power on, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
    hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
    hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
    hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif    
#endif	
    //msleep(100);

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(10);  
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
    msleep(100);//add this line 
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
#ifdef ESD_CHECK	
    msleep(1);  
    queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection					
#endif
	
    msleep(20);
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);
	}
	tpd_halt = 0;//add this line 
	tpd_up(0,0,0);
	input_sync(tpd->dev);
	TPD_DMESG("TPD wake up done\n");
	 //return retval;
}

static void tpd_suspend( struct early_suspend *h )
{
    //int retval = TPD_OK;
    static char data = 0x3;
#ifdef ESD_CHECK	
    cancel_delayed_work_sync(&ctp_read_id_work);
#endif
    TPD_DMESG("TPD enter sleep\n");
    tpd_halt = 1; //add this line 
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
    hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
    i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
#ifdef MT6573
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
#endif
#ifdef MT6575
    //power down, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
    hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
#else
    hwPowerDown(MT65XX_POWER_LDO_VGP2,  "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
    hwPowerDown(TPD_POWER_SOURCE_1800,  "TP");
#endif    
#endif	
#endif
    TPD_DMESG("TPD enter sleep done\n");
    //return retval;
} 

static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "FT6306",
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
static int __init tpd_driver_init(void) {
	 printk("MediaTek FT6306 touch panel driver init\n");
	   i2c_register_board_info(1, &FT6306_i2c_tpd, 1);
		 if(tpd_driver_add(&tpd_device_driver) < 0)
			 TPD_DMESG("add FT6306 driver failed\n");
	 return 0;
}
 
/* should never be called */
static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek FT6306 touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
