/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
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
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2005
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   Sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/



/*#####################################################


superpix    sensor   30m  BF3905 .   sensorID = 0X0A       SLAVE ADDR= 0X42 



#####################################################*/

 
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "bf3905mipi_Sensor.h"
#include "bf3905mipi_Camera_Sensor_para.h"
#include "bf3905mipi_CameraCustomized.h"

static MSDK_SENSOR_CONFIG_STRUCT BF3905MIPISensorConfigData;
static struct BF3905MIPI_Sensor_Struct BF3905MIPI_Sensor_Driver;

#define BF3905MIPIYUV_DEBUG
#ifdef BF3905MIPIYUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#define __SENSOR_CONTROL__
#ifdef __SENSOR_CONTROL__
#define CAMERA_CONTROL_FLOW(para1,para2) printk("[%s:%d]::para1=0x%x,para1=0x%x\n\n",__FUNCTION__,__LINE__,para1,para2)
#else
#define CAMERA_CONTROL_FLOW(para1, para2)
#endif

#define BF3905MIPI_CAPPRE_20_10FPS                                    0//capture preview daylight
#define BF3905MIPI_CAPPRE_20_10FPS_MAXGAIN                    0x78
#define BF3905MIPI_CAPPRE_20_08FPS                                    1//capture preview daylight

kal_uint8 isBf3905Banding = 1; // 0: 50hz  1:60hz

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
/*************************************************************************
* FUNCTION
*    BF3905_MIPI_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
void BF3905_MIPI_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
	char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	iWriteRegI2C(puSendCmd, 2, BF3905MIPI_WRITE_ID);
}

/*************************************************************************
* FUNCTION
*    BF3905_MIPI_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
kal_uint16 BF3905_MIPI_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint16 get_byte = 0;
	char puSendCmd = { (char)(addr & 0xFF) };
	iReadRegI2C(&puSendCmd, 1, (u8*)&get_byte, 1, BF3905MIPI_WRITE_ID);

    return get_byte;
}


void BF3905MIPI_Set_Dummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
                BF3905_MIPI_write_cmos_sensor(0x92, iLines&0xff); //default 510
		BF3905_MIPI_write_cmos_sensor(0x93, (iLines>>8)&0xff); 	// Extra V-Blanking
		BF3905_MIPI_write_cmos_sensor(0x2b, (iPixels+16)&0xff); //default 784
		BF3905_MIPI_write_cmos_sensor(0x2a, (((iPixels+16)>>8)<<4)|((BF3905_MIPI_read_cmos_sensor(0x2a))&0x0f)); 	// Extra H-Blanking
}   /*  BF3905MIPI_Set_Dummy    */


/*************************************************************************
* FUNCTION
*	BF3905MIPI_write_reg
*
* DESCRIPTION
*	This function set the register of BF3905MIPI.
*
* PARAMETERS
*	addr : the register index of OV76X0
*  para : setting parameter of the specified register of OV76X0
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

//static void BF3905MIPI_write_reg(kal_uint32 addr, kal_uint32 para)
//{
//	BF3905_MIPI_write_cmos_sensor(addr,para);
//}	/* BF3905MIPI_write_reg() */

/*************************************************************************
* FUNCTION
*	ov7670_read_cmos_sensor
*
* DESCRIPTION
*	This function read parameter of specified register from OV76X0.
*
* PARAMETERS
*	addr : the register index of OV76X0
*
* RETURNS
*	the data that read from OV76X0
*
* GLOBALS AFFECTED
*
*************************************************************************/
//static kal_uint32 BF3905MIPI_read_reg(kal_uint32 addr)
//{
//	return (BF3905_MIPI_read_cmos_sensor(addr));
//}	/* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	BF3905MIPI_NightMode
*
* DESCRIPTION
*	This function night mode of BF3905MIPI.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void BF3905MIPI_night_mode(kal_bool bEnable)
{
// kal_uint8 temp = BF3905_MIPI_read_cmos_sensor(0x3B);


  if (!BF3905MIPI_Sensor_Driver.MODE_CAPTURE) 
  { 
	if(bEnable)//night mode
	{ 
		   BF3905MIPI_Sensor_Driver.bNight_mode = KAL_TRUE;

	   if(BF3905MIPI_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
		{
				if(isBf3905Banding== 0)
				{
				printk("video 50Hz night\n");	
				//Video record night 24M 50hz 12-12FPS maxgain:0x8c
				  //26m 50hz 12-12                      



				//dbg_print(" video 50Hz night\r\n");
				}
				else if(isBf3905Banding == 1)
				{
				//Video record night 24M 60Hz 12-12FPS maxgain:0x8c
								  //26m 60hz 12-12


						
				printk(" video 60Hz night\r\n");
				}
   		  	}	
			    else 
			   {
			//	dbg_print(" BF3905MIPI_banding=%x\r\n",BF3905MIPI_banding);
			       if(isBf3905Banding== 0)
				{
				//capture preview night 24M 50hz 20-6FPS maxgain:0x78	 
								   //26m 50hz 6-20



				printk(" priview 50Hz night\r\n");	
				}  
				else if(isBf3905Banding== 1)
				{
				//capture preview night 24M 60hz 20-6FPS maxgain:0x78


								  //26m 60hz 6-20 



				printk(" priview 60Hz night\r\n");	
				}
			       } 		
	}
	else    // daylight mode
	{
		BF3905MIPI_Sensor_Driver.bNight_mode = KAL_FALSE;
		                   
	    if(BF3905MIPI_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
	    {
				//dbg_print(" BF3905MIPI_banding=%x\r\n",BF3905MIPI_banding);
				if(isBf3905Banding== 0)
				{
				//Video record daylight 24M 50hz 20-20FPS maxgain:0x8c
									  //26m 50hz 20-20                      



				printk(" video 50Hz normal\r\n");				
				}
				else if(isBf3905Banding == 1)
				{
				//Video record daylight 24M 60Hz 20-20FPS maxgain:0x8c

								  //26 m 60hz 20-20 


				printk(" video 60Hz normal\r\n");	
				}
			   }
		else 
			{
			//	dbg_print(" BF3905MIPI_banding=%x\r\n",BF3905MIPI_banding);
			       if(isBf3905Banding== 0)
				{
				



				printk(" priview 50Hz normal\r\n");
				}
				else if(isBf3905Banding== 1)
				{
				#if BF3905MIPI_CAPPRE_20_10FPS
					//capture preview daylight 24M 60hz 20-10FPS maxgain:0x78
					
					#endif
				#if BF3905MIPI_CAPPRE_20_08FPS
				//capture preview daylight 24M 60hz 20-8FPS maxgain:0x70   
					//26m 60hz 8-20                         
					
	

					#endif
				printk(" priview 60Hz normal\r\n");
				}
			       }
	   
	}  
	}
}	/*	BF3905MIPI_NightMode	*/

/*
static void BF3905MIPI_set_isp_driving_current(kal_uint8 current)
{
    //#define CONFIG_BASE      	(0xF0001000)     
//  iowrite32((0xE << 12)|(0 << 28)|0x8880888, 0xF0001500);
}
*/

static void BF3905MIPI_Sensor_Driver_Init(void)
{
  #if 0
	//BF3905_MIPI_write_cmos_sensor(0x12,0x80);
	BF3905_MIPI_write_cmos_sensor(0x09,0x00);
	BF3905_MIPI_write_cmos_sensor(0x12,0x00);
	BF3905_MIPI_write_cmos_sensor(0x3a,0x03); //0x20
	BF3905_MIPI_write_cmos_sensor(0x17,0x00);
	BF3905_MIPI_write_cmos_sensor(0x18,0xa0);
	BF3905_MIPI_write_cmos_sensor(0x19,0x00);
	BF3905_MIPI_write_cmos_sensor(0x1a,0x78);
	BF3905_MIPI_write_cmos_sensor(0x03,0xa0);
	BF3905_MIPI_write_cmos_sensor(0x5d,0xb3);
	BF3905_MIPI_write_cmos_sensor(0xbf,0x08);
	BF3905_MIPI_write_cmos_sensor(0xc3,0x08);
	BF3905_MIPI_write_cmos_sensor(0xca,0x10);
	BF3905_MIPI_write_cmos_sensor(0x15,0x12);
	BF3905_MIPI_write_cmos_sensor(0x62,0x00);
	BF3905_MIPI_write_cmos_sensor(0x63,0x00);
	BF3905_MIPI_write_cmos_sensor(0x1e,0x40);
	BF3905_MIPI_write_cmos_sensor(0x2a,0x00); //dummy pixel
	BF3905_MIPI_write_cmos_sensor(0x2b,0x10); //dummy pixel
	BF3905_MIPI_write_cmos_sensor(0x92,0x1f); //dummy line
	BF3905_MIPI_write_cmos_sensor(0x93,0x00); //dummy line
	BF3905_MIPI_write_cmos_sensor(0xd9,0x25);
	BF3905_MIPI_write_cmos_sensor(0xdf,0x25);
	BF3905_MIPI_write_cmos_sensor(0x4a,0x0c);
	BF3905_MIPI_write_cmos_sensor(0xda,0x00);
	BF3905_MIPI_write_cmos_sensor(0xdb,0xa2);
	BF3905_MIPI_write_cmos_sensor(0xdc,0x00);
	BF3905_MIPI_write_cmos_sensor(0xdd,0x7a);
	BF3905_MIPI_write_cmos_sensor(0xde,0x00);
	BF3905_MIPI_write_cmos_sensor(0x1b,0x0e);
	BF3905_MIPI_write_cmos_sensor(0x60,0xe5);
	BF3905_MIPI_write_cmos_sensor(0x61,0xf2);
	BF3905_MIPI_write_cmos_sensor(0x6d,0xc0);
	BF3905_MIPI_write_cmos_sensor(0xb9,0x00);
	BF3905_MIPI_write_cmos_sensor(0x64,0x00);
	BF3905_MIPI_write_cmos_sensor(0xbb,0x10);
	BF3905_MIPI_write_cmos_sensor(0x08,0x02);
	BF3905_MIPI_write_cmos_sensor(0x20,0x09);
	BF3905_MIPI_write_cmos_sensor(0x21,0x4f);
	BF3905_MIPI_write_cmos_sensor(0x3e,0x83);
	BF3905_MIPI_write_cmos_sensor(0x16,0xa1);
	BF3905_MIPI_write_cmos_sensor(0x2f,0xc4);
	BF3905_MIPI_write_cmos_sensor(0x13,0x00);
	BF3905_MIPI_write_cmos_sensor(0x01,0x15);
	BF3905_MIPI_write_cmos_sensor(0x02,0x23);
	BF3905_MIPI_write_cmos_sensor(0x9d,0x20);
	BF3905_MIPI_write_cmos_sensor(0x8c,0x03);
	BF3905_MIPI_write_cmos_sensor(0x8d,0x11);
	BF3905_MIPI_write_cmos_sensor(0x33,0x10);
	BF3905_MIPI_write_cmos_sensor(0x34,0x1d);
	BF3905_MIPI_write_cmos_sensor(0x36,0x45);
	BF3905_MIPI_write_cmos_sensor(0x6e,0x20);
	BF3905_MIPI_write_cmos_sensor(0xbc,0x0d);
	BF3905_MIPI_write_cmos_sensor(0x35,0x30);
	BF3905_MIPI_write_cmos_sensor(0x65,0x2a);
	BF3905_MIPI_write_cmos_sensor(0x66,0x2a);
	BF3905_MIPI_write_cmos_sensor(0xbd,0xf4);
	BF3905_MIPI_write_cmos_sensor(0xbe,0x44);
	BF3905_MIPI_write_cmos_sensor(0x9b,0xf4);
	BF3905_MIPI_write_cmos_sensor(0x9c,0x44);
	BF3905_MIPI_write_cmos_sensor(0x37,0xf4);
	BF3905_MIPI_write_cmos_sensor(0x38,0x44);
	BF3905_MIPI_write_cmos_sensor(0x70,0x0b);
	BF3905_MIPI_write_cmos_sensor(0x71,0x0f);
	BF3905_MIPI_write_cmos_sensor(0x72,0x4c);
	BF3905_MIPI_write_cmos_sensor(0x73,0x27);
	BF3905_MIPI_write_cmos_sensor(0x75,0x8a);
	BF3905_MIPI_write_cmos_sensor(0x76,0x98);
	BF3905_MIPI_write_cmos_sensor(0x77,0x5a);
	BF3905_MIPI_write_cmos_sensor(0x78,0xff);
	BF3905_MIPI_write_cmos_sensor(0x79,0x24);
	BF3905_MIPI_write_cmos_sensor(0x7a,0x12);
	BF3905_MIPI_write_cmos_sensor(0x7b,0x58);
	BF3905_MIPI_write_cmos_sensor(0x7c,0x55);
	BF3905_MIPI_write_cmos_sensor(0x7d,0x00);
	BF3905_MIPI_write_cmos_sensor(0x7e,0x84);
	BF3905_MIPI_write_cmos_sensor(0x7f,0x3c);
	BF3905_MIPI_write_cmos_sensor(0x13,0x07);
	BF3905_MIPI_write_cmos_sensor(0x24,0x4f);
	BF3905_MIPI_write_cmos_sensor(0x25,0x88);
	BF3905_MIPI_write_cmos_sensor(0x80,0x92);
	BF3905_MIPI_write_cmos_sensor(0x81,0x00);
	BF3905_MIPI_write_cmos_sensor(0x82,0x2a);
	BF3905_MIPI_write_cmos_sensor(0x83,0x54);
	BF3905_MIPI_write_cmos_sensor(0x84,0x39);
	BF3905_MIPI_write_cmos_sensor(0x85,0x5d);
	BF3905_MIPI_write_cmos_sensor(0x86,0x88);
	BF3905_MIPI_write_cmos_sensor(0x89,0x63);
	BF3905_MIPI_write_cmos_sensor(0x8a,0xa2);//96->a2
	BF3905_MIPI_write_cmos_sensor(0x8b,0x7d);
	BF3905_MIPI_write_cmos_sensor(0x8f,0x82);
	BF3905_MIPI_write_cmos_sensor(0x94,0x92);//42->92 avoid ae vabration
	BF3905_MIPI_write_cmos_sensor(0x95,0x84);
	BF3905_MIPI_write_cmos_sensor(0x96,0xb3);
	BF3905_MIPI_write_cmos_sensor(0x97,0x40);
	BF3905_MIPI_write_cmos_sensor(0x98,0x8a);
	BF3905_MIPI_write_cmos_sensor(0x99,0x10);
	BF3905_MIPI_write_cmos_sensor(0x9a,0x50);
	BF3905_MIPI_write_cmos_sensor(0x9f,0x64);
	
	BF3905_MIPI_write_cmos_sensor(0X39,0XA8);//0X98
	BF3905_MIPI_write_cmos_sensor(0X3f,0XA8);//0X98
	BF3905_MIPI_write_cmos_sensor(0x90,0x20);
	BF3905_MIPI_write_cmos_sensor(0x91,0xd0);
	//gamma default
	BF3905_MIPI_write_cmos_sensor(0X40,0X3b);
	BF3905_MIPI_write_cmos_sensor(0X41,0X36);
	BF3905_MIPI_write_cmos_sensor(0X42,0X2b);
	BF3905_MIPI_write_cmos_sensor(0X43,0X1d);
	BF3905_MIPI_write_cmos_sensor(0X44,0X1a);
	BF3905_MIPI_write_cmos_sensor(0X45,0X14);
	BF3905_MIPI_write_cmos_sensor(0X46,0X11);
	BF3905_MIPI_write_cmos_sensor(0X47,0X0e);
	BF3905_MIPI_write_cmos_sensor(0X48,0X0d);
	BF3905_MIPI_write_cmos_sensor(0X49,0X0c);
	BF3905_MIPI_write_cmos_sensor(0X4b,0X0b);
	BF3905_MIPI_write_cmos_sensor(0X4c,0X09);
	BF3905_MIPI_write_cmos_sensor(0X4e,0X08);
	BF3905_MIPI_write_cmos_sensor(0X4f,0X07);
	BF3905_MIPI_write_cmos_sensor(0X50,0X07);
	
	BF3905_MIPI_write_cmos_sensor(0x5a,0x56);
	
	BF3905_MIPI_write_cmos_sensor(0x51,0x13);
	BF3905_MIPI_write_cmos_sensor(0x52,0x05);
	BF3905_MIPI_write_cmos_sensor(0x53,0x91);
	BF3905_MIPI_write_cmos_sensor(0x54,0x72);
	BF3905_MIPI_write_cmos_sensor(0x57,0x96);
	BF3905_MIPI_write_cmos_sensor(0x58,0x35);
	
	BF3905_MIPI_write_cmos_sensor(0x5a,0xd6);
	BF3905_MIPI_write_cmos_sensor(0x51,0x29);
	BF3905_MIPI_write_cmos_sensor(0x52,0x0D);
	BF3905_MIPI_write_cmos_sensor(0x53,0x91);
	BF3905_MIPI_write_cmos_sensor(0x54,0x81);
	BF3905_MIPI_write_cmos_sensor(0x57,0x56);
	BF3905_MIPI_write_cmos_sensor(0x58,0x09);
	BF3905_MIPI_write_cmos_sensor(0x5b,0x02);
	BF3905_MIPI_write_cmos_sensor(0x5c,0x30);
	
	BF3905_MIPI_write_cmos_sensor(0xb0,0xe0);
	BF3905_MIPI_write_cmos_sensor(0xb3,0x58);
	BF3905_MIPI_write_cmos_sensor(0xb4,0xe3);//a light
	BF3905_MIPI_write_cmos_sensor(0xb1,0xef);
	BF3905_MIPI_write_cmos_sensor(0xb2,0xef);
	
	BF3905_MIPI_write_cmos_sensor(0xb4,0x63);//
	BF3905_MIPI_write_cmos_sensor(0xb1,0xb0);//0xba
	BF3905_MIPI_write_cmos_sensor(0xb2,0xa0);//0xaa
	BF3905_MIPI_write_cmos_sensor(0x55,0x00);
	BF3905_MIPI_write_cmos_sensor(0x56,0x40);
	BF3905_MIPI_write_cmos_sensor(0x6a,0x81);
	
	BF3905_MIPI_write_cmos_sensor(0x69,0x00); // effect normal 
	BF3905_MIPI_write_cmos_sensor(0x67,0x80); // Normal, 
	BF3905_MIPI_write_cmos_sensor(0x68,0x80); // Normal, 
	BF3905_MIPI_write_cmos_sensor(0xb4,0x63); // Normal, 
	
	BF3905_MIPI_write_cmos_sensor(0x23,0x55);
	BF3905_MIPI_write_cmos_sensor(0xa0,0x00);
	BF3905_MIPI_write_cmos_sensor(0xa1,0x31);
	BF3905_MIPI_write_cmos_sensor(0xa2,0x0d);
	BF3905_MIPI_write_cmos_sensor(0xa3,0x27);
	BF3905_MIPI_write_cmos_sensor(0xa4,0x0a);
	BF3905_MIPI_write_cmos_sensor(0xa5,0x2c);
	BF3905_MIPI_write_cmos_sensor(0xa6,0x04);
	BF3905_MIPI_write_cmos_sensor(0xa7,0x1a);
	BF3905_MIPI_write_cmos_sensor(0xa8,0x18);
	BF3905_MIPI_write_cmos_sensor(0xa9,0x13);
	BF3905_MIPI_write_cmos_sensor(0xaa,0x18);
	BF3905_MIPI_write_cmos_sensor(0xab,0x24);
	BF3905_MIPI_write_cmos_sensor(0xac,0x3c);
	BF3905_MIPI_write_cmos_sensor(0xad,0xf0);
	BF3905_MIPI_write_cmos_sensor(0xae,0x59);
	BF3905_MIPI_write_cmos_sensor(0xc5,0xaa);
	BF3905_MIPI_write_cmos_sensor(0xc6,0xbb);
	BF3905_MIPI_write_cmos_sensor(0xc7,0x30);
	BF3905_MIPI_write_cmos_sensor(0xc8,0x0d);
	BF3905_MIPI_write_cmos_sensor(0xc9,0x10);
	BF3905_MIPI_write_cmos_sensor(0xd0,0xa3);
	BF3905_MIPI_write_cmos_sensor(0xd1,0x00);
	BF3905_MIPI_write_cmos_sensor(0xd2,0x58);
	BF3905_MIPI_write_cmos_sensor(0xd3,0x09);
	BF3905_MIPI_write_cmos_sensor(0xd4,0x24);
	BF3905_MIPI_write_cmos_sensor(0xee,0x30);
	BF3905_MIPI_write_cmos_sensor(0x6c,0xc2);
	BF3905_MIPI_write_cmos_sensor(0x24,0x48);
 #else
	BF3905_MIPI_write_cmos_sensor(0x12,0x80);
	BF3905_MIPI_write_cmos_sensor(0x15,0x12);
	BF3905_MIPI_write_cmos_sensor(0x09,0x01);
	BF3905_MIPI_write_cmos_sensor(0x12,0x00);
	BF3905_MIPI_write_cmos_sensor(0x3a,0x03);//20
	BF3905_MIPI_write_cmos_sensor(0x1e,0x70);
	BF3905_MIPI_write_cmos_sensor(0x1b,0x0e);
	BF3905_MIPI_write_cmos_sensor(0x2a,0x00);
	BF3905_MIPI_write_cmos_sensor(0x2b,0x10);
	BF3905_MIPI_write_cmos_sensor(0x92,0x09);
	BF3905_MIPI_write_cmos_sensor(0x93,0x00);
	BF3905_MIPI_write_cmos_sensor(0x8a,0x9c);
	BF3905_MIPI_write_cmos_sensor(0x8b,0x82);
	BF3905_MIPI_write_cmos_sensor(0x13,0x00);
	BF3905_MIPI_write_cmos_sensor(0x01,0x15);
	BF3905_MIPI_write_cmos_sensor(0x02,0x23);
	BF3905_MIPI_write_cmos_sensor(0x9d,0x20);
	BF3905_MIPI_write_cmos_sensor(0x8c,0x02);
	BF3905_MIPI_write_cmos_sensor(0x8d,0xee);
	BF3905_MIPI_write_cmos_sensor(0x13,0x07);

	BF3905_MIPI_write_cmos_sensor(0x5d,0xb3);
	BF3905_MIPI_write_cmos_sensor(0xbf,0x08);
	BF3905_MIPI_write_cmos_sensor(0xc3,0x08);
	BF3905_MIPI_write_cmos_sensor(0xca,0x10);
	BF3905_MIPI_write_cmos_sensor(0x62,0x00);
	BF3905_MIPI_write_cmos_sensor(0x63,0x00);
	BF3905_MIPI_write_cmos_sensor(0xb9,0x00);
	BF3905_MIPI_write_cmos_sensor(0x64,0x00);
	BF3905_MIPI_write_cmos_sensor(0xbb,0x10);
	BF3905_MIPI_write_cmos_sensor(0x08,0x02);
	BF3905_MIPI_write_cmos_sensor(0x0e,0x10);
	BF3905_MIPI_write_cmos_sensor(0x22,0x12);	
	BF3905_MIPI_write_cmos_sensor(0x20,0x09);
	BF3905_MIPI_write_cmos_sensor(0x21,0x4f);
	BF3905_MIPI_write_cmos_sensor(0x3e,0x83);
	BF3905_MIPI_write_cmos_sensor(0x2f,0x84);
	BF3905_MIPI_write_cmos_sensor(0x16,0xa1);
	BF3905_MIPI_write_cmos_sensor(0x6c,0xc2);
	BF3905_MIPI_write_cmos_sensor(0x71,0x0f);
	BF3905_MIPI_write_cmos_sensor(0x7e,0x84);
	BF3905_MIPI_write_cmos_sensor(0x7f,0x3c);
	BF3905_MIPI_write_cmos_sensor(0x60,0xe5);
	BF3905_MIPI_write_cmos_sensor(0x61,0xf2);
	BF3905_MIPI_write_cmos_sensor(0x6d,0xc0);
	BF3905_MIPI_write_cmos_sensor(0x1e,0x70);
	BF3905_MIPI_write_cmos_sensor(0xd9,0x25);
	BF3905_MIPI_write_cmos_sensor(0xdf,0x26);
	BF3905_MIPI_write_cmos_sensor(0x17,0x00);
	BF3905_MIPI_write_cmos_sensor(0x18,0xa0);
	BF3905_MIPI_write_cmos_sensor(0x19,0x00);
	BF3905_MIPI_write_cmos_sensor(0x1a,0x78);
	BF3905_MIPI_write_cmos_sensor(0x03,0xa0);
	BF3905_MIPI_write_cmos_sensor(0x4a,0x0c);
	BF3905_MIPI_write_cmos_sensor(0xda,0x00);
	BF3905_MIPI_write_cmos_sensor(0xdb,0xa2);
	BF3905_MIPI_write_cmos_sensor(0xdc,0x00);
	BF3905_MIPI_write_cmos_sensor(0xdd,0x7a);
	BF3905_MIPI_write_cmos_sensor(0xde,0x00);
	BF3905_MIPI_write_cmos_sensor(0x34,0x1d);
	BF3905_MIPI_write_cmos_sensor(0x36,0x45);
	BF3905_MIPI_write_cmos_sensor(0x6e,0x20);
	BF3905_MIPI_write_cmos_sensor(0xbc,0x0d);
	BF3905_MIPI_write_cmos_sensor(0x35,0x30);
	BF3905_MIPI_write_cmos_sensor(0x65,0x24);
	BF3905_MIPI_write_cmos_sensor(0x66,0x19);
	BF3905_MIPI_write_cmos_sensor(0xbd,0xf4);
	BF3905_MIPI_write_cmos_sensor(0xbe,0x44);
	BF3905_MIPI_write_cmos_sensor(0x9b,0xf4);
	BF3905_MIPI_write_cmos_sensor(0x9c,0x44);
	BF3905_MIPI_write_cmos_sensor(0x37,0xf4);
	BF3905_MIPI_write_cmos_sensor(0x38,0x44);
	BF3905_MIPI_write_cmos_sensor(0xf1,0x00);
	BF3905_MIPI_write_cmos_sensor(0x70,0x0b);
	BF3905_MIPI_write_cmos_sensor(0x73,0x27);
	BF3905_MIPI_write_cmos_sensor(0x79,0x24);
	BF3905_MIPI_write_cmos_sensor(0x7a,0x34); //0x23 //2014.1.22 ycm
	BF3905_MIPI_write_cmos_sensor(0x75,0xaa);
	BF3905_MIPI_write_cmos_sensor(0x76,0x98);
	BF3905_MIPI_write_cmos_sensor(0x77,0x2a);
	BF3905_MIPI_write_cmos_sensor(0x7b,0x58);
	BF3905_MIPI_write_cmos_sensor(0x7d,0x00);
	BF3905_MIPI_write_cmos_sensor(0x13,0x07);
	BF3905_MIPI_write_cmos_sensor(0x24,0x40);
	BF3905_MIPI_write_cmos_sensor(0x25,0x88); ///0xff   20131128 Fix AE unstable
	BF3905_MIPI_write_cmos_sensor(0x97,0x45); //0x3c 201311281806 modified.
	BF3905_MIPI_write_cmos_sensor(0x98,0xb8); //0xbf //2014.1.22 ycm
	BF3905_MIPI_write_cmos_sensor(0x80,0xd2);//0xd0
	BF3905_MIPI_write_cmos_sensor(0x81,0x00);
	BF3905_MIPI_write_cmos_sensor(0x82,0x2a);
	BF3905_MIPI_write_cmos_sensor(0x83,0x54);
	BF3905_MIPI_write_cmos_sensor(0x84,0x39);
	BF3905_MIPI_write_cmos_sensor(0x85,0x5d);
	BF3905_MIPI_write_cmos_sensor(0x86,0xc0); ///0x88
	BF3905_MIPI_write_cmos_sensor(0x89,0x75); ///0x63   20131128 Fix AE unstable
	BF3905_MIPI_write_cmos_sensor(0x94,0x22); ////0xc2 //2014.1.22 ycm
	BF3905_MIPI_write_cmos_sensor(0x96,0xbb);////0xb3 for avoiding greenish outdoor 201310221448
	BF3905_MIPI_write_cmos_sensor(0x9a,0x50);
	BF3905_MIPI_write_cmos_sensor(0x99,0x10);
	BF3905_MIPI_write_cmos_sensor(0x9f,0x64);
	BF3905_MIPI_write_cmos_sensor(0x39,0xb0);//0x98  0xAa
	BF3905_MIPI_write_cmos_sensor(0x3f,0xb0);//0x98 0xAa
	BF3905_MIPI_write_cmos_sensor(0x90,0x20);
	BF3905_MIPI_write_cmos_sensor(0x91,0xf7);
	#if 0
	//default gamma
	BF3905_MIPI_write_cmos_sensor(0x40,0x3b);
	BF3905_MIPI_write_cmos_sensor(0x41,0x36);
	BF3905_MIPI_write_cmos_sensor(0x42,0x2b);
	BF3905_MIPI_write_cmos_sensor(0x43,0x1d);
	BF3905_MIPI_write_cmos_sensor(0x44,0x1a);
	BF3905_MIPI_write_cmos_sensor(0x45,0x14);
	BF3905_MIPI_write_cmos_sensor(0x46,0x11);
	BF3905_MIPI_write_cmos_sensor(0x47,0x0f);
	BF3905_MIPI_write_cmos_sensor(0x48,0x0e);
	BF3905_MIPI_write_cmos_sensor(0x49,0x0d);
	BF3905_MIPI_write_cmos_sensor(0x4b,0x0c);
	BF3905_MIPI_write_cmos_sensor(0x4c,0x0b);
	BF3905_MIPI_write_cmos_sensor(0x4e,0x0a);
	BF3905_MIPI_write_cmos_sensor(0x4f,0x09);
	BF3905_MIPI_write_cmos_sensor(0x50,0x09);
	#else
	//low niose
	BF3905_MIPI_write_cmos_sensor(0x40,0x24);
	BF3905_MIPI_write_cmos_sensor(0x41,0x30);
	BF3905_MIPI_write_cmos_sensor(0x42,0x24);
	BF3905_MIPI_write_cmos_sensor(0x43,0x1d);
	BF3905_MIPI_write_cmos_sensor(0x44,0x1a);
	BF3905_MIPI_write_cmos_sensor(0x45,0x14);
	BF3905_MIPI_write_cmos_sensor(0x46,0x11);
	BF3905_MIPI_write_cmos_sensor(0x47,0x0e);
	BF3905_MIPI_write_cmos_sensor(0x48,0x0d);
	BF3905_MIPI_write_cmos_sensor(0x49,0x0c);
	BF3905_MIPI_write_cmos_sensor(0x4b,0x0b);
	BF3905_MIPI_write_cmos_sensor(0x4c,0x09);
	BF3905_MIPI_write_cmos_sensor(0x4e,0x08); //0x09 //2014.1.22 ycm
	BF3905_MIPI_write_cmos_sensor(0x4f,0x07); //0x08 //2014.1.22 ycm
	BF3905_MIPI_write_cmos_sensor(0x50,0x06); //0x07 //2014.1.22 ycm
	#endif
	//outdoor colormatrix
	BF3905_MIPI_write_cmos_sensor(0x5a,0x56);
	#if 0
	BF3905_MIPI_write_cmos_sensor(0x51,0x12);
	BF3905_MIPI_write_cmos_sensor(0x52,0x0d);
	BF3905_MIPI_write_cmos_sensor(0x53,0x92);
	BF3905_MIPI_write_cmos_sensor(0x54,0x7d);
	BF3905_MIPI_write_cmos_sensor(0x57,0x97);
	BF3905_MIPI_write_cmos_sensor(0x58,0x43);
	#else ////201310301109 avoiding over-saturation outdoor
	BF3905_MIPI_write_cmos_sensor(0x51,0x01);
	BF3905_MIPI_write_cmos_sensor(0x52,0x0b);
	BF3905_MIPI_write_cmos_sensor(0x53,0x68);
	BF3905_MIPI_write_cmos_sensor(0x54,0x2c);
	BF3905_MIPI_write_cmos_sensor(0x57,0x56);
	BF3905_MIPI_write_cmos_sensor(0x58,0x54);
	#endif
	//indoor colormatrix
	BF3905_MIPI_write_cmos_sensor(0x5a,0xd6);
	
       #if 0
	BF3905_MIPI_write_cmos_sensor(0x51,0x39);
	BF3905_MIPI_write_cmos_sensor(0x52,0x0f);
	BF3905_MIPI_write_cmos_sensor(0x53,0x3b);
	BF3905_MIPI_write_cmos_sensor(0x54,0x55);
	BF3905_MIPI_write_cmos_sensor(0x57,0x7e);
	BF3905_MIPI_write_cmos_sensor(0x58,0x05);
       #else
        
       //fusehao
	BF3905_MIPI_write_cmos_sensor(0x51,0x17);
	BF3905_MIPI_write_cmos_sensor(0x52,0x13);
	BF3905_MIPI_write_cmos_sensor(0x53,0x5e);
	BF3905_MIPI_write_cmos_sensor(0x54,0x38);
	BF3905_MIPI_write_cmos_sensor(0x57,0x38);
	BF3905_MIPI_write_cmos_sensor(0x58,0x02);
	/*
	//default
	BF3905_MIPI_write_cmos_sensor(0x51,0x28);
	BF3905_MIPI_write_cmos_sensor(0x52,0x23);
	BF3905_MIPI_write_cmos_sensor(0x53,0x9f);
	BF3905_MIPI_write_cmos_sensor(0x54,0x73);
	BF3905_MIPI_write_cmos_sensor(0x57,0x50);
	BF3905_MIPI_write_cmos_sensor(0x58,0x08);
	*/
       #endif	
	BF3905_MIPI_write_cmos_sensor(0x5c,0x26);/////////////zj  26//3fzj20140123
	BF3905_MIPI_write_cmos_sensor(0x6a,0x81);//e1//81  zj20140109
	BF3905_MIPI_write_cmos_sensor(0x23,0x55);
	BF3905_MIPI_write_cmos_sensor(0xa1,0x31);
	BF3905_MIPI_write_cmos_sensor(0xa2,0x0d);//0x0d zj
	BF3905_MIPI_write_cmos_sensor(0xa3,0x25);
	BF3905_MIPI_write_cmos_sensor(0xa4,0x0b);//
	BF3905_MIPI_write_cmos_sensor(0xa5,0x25); ///0x23   20131128 Fix AE unstable
	BF3905_MIPI_write_cmos_sensor(0xa6,0x06);//04 zj
	BF3905_MIPI_write_cmos_sensor(0xa7,0x17);
	BF3905_MIPI_write_cmos_sensor(0xa8,0x1a);
	BF3905_MIPI_write_cmos_sensor(0xa9,0x13);//20  zj20140109 13
	BF3905_MIPI_write_cmos_sensor(0xaa,0x18);//20  zj20140109 18
	BF3905_MIPI_write_cmos_sensor(0xab,0x1c);//20  zj20140109 1c
	BF3905_MIPI_write_cmos_sensor(0xac,0x3c);
	BF3905_MIPI_write_cmos_sensor(0xad,0xf0);
	BF3905_MIPI_write_cmos_sensor(0xae,0x7f);////201310221146
	BF3905_MIPI_write_cmos_sensor(0xd0,0xa4);
	BF3905_MIPI_write_cmos_sensor(0xd1,0x00);
	BF3905_MIPI_write_cmos_sensor(0xd2,0x58);
	BF3905_MIPI_write_cmos_sensor(0xc5,0xaa);
	BF3905_MIPI_write_cmos_sensor(0xc6,0xaa);//0xca
	BF3905_MIPI_write_cmos_sensor(0xc7,0x30);
	BF3905_MIPI_write_cmos_sensor(0xc8,0x0a);///0x0d 201310291500 outdoor
	BF3905_MIPI_write_cmos_sensor(0xc9,0x0d); ///0x10 201310291500
	BF3905_MIPI_write_cmos_sensor(0xd3,0x06); //0x09 201310291500
	BF3905_MIPI_write_cmos_sensor(0xd4,0x1a); //0x24 201310291500 
	BF3905_MIPI_write_cmos_sensor(0xee,0x30);
	BF3905_MIPI_write_cmos_sensor(0xb0,0xe0);///////////zj  //0xe0 //00 zj20140123
	BF3905_MIPI_write_cmos_sensor(0xb3,0x48);
	BF3905_MIPI_write_cmos_sensor(0xb4,0xe3);
	BF3905_MIPI_write_cmos_sensor(0xb1,0xff);
	BF3905_MIPI_write_cmos_sensor(0xb2,0xff); ///0xa0
	BF3905_MIPI_write_cmos_sensor(0xb4,0x63);//63   
	BF3905_MIPI_write_cmos_sensor(0xb1,0xc0); //0xb3
	BF3905_MIPI_write_cmos_sensor(0xb2,0xb0);//0xa0//zj 0xa0
	BF3905_MIPI_write_cmos_sensor(0x55,0x00);
	BF3905_MIPI_write_cmos_sensor(0x56,0x40);//0x40  //2014.1.22 ycm
	BF3905_MIPI_write_cmos_sensor(0x24,0x40);//0x45 //2014.1.22 ycm
	
	BF3905_MIPI_write_cmos_sensor(0x8a,0x93);//96->a2
	BF3905_MIPI_write_cmos_sensor(0x8b,0x7b);
	#endif
}


/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	BF3905MIPIOpen
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint32 BF3905MIPIOpen(void)
{
	kal_uint16 sensor_id = 0; 
	int retry = 10; 

    SENSORDB("BF3905MIPIOpen_start \n");

//	BF3905MIPI_Sensor_Driver.i2c_clit.addr=BF3905MIPI_WRITE_ID;
//	BF3905MIPI_Sensor_Driver.i2c_clit = i2c_clit;
//    BF3905MIPI_Sensor_Driver.i2c_clit->addr = BF3905MIPI_WRITE_ID;

#if 0 
	BF3905_MIPI_write_cmos_sensor(0x12, 0x80);
	mDELAY(10);
#endif 

	// check if sensor ID correct
	do {

	    sensor_id=((BF3905_MIPI_read_cmos_sensor(0xfc)<<8)|(BF3905_MIPI_read_cmos_sensor(0xfd)));
    	    if (sensor_id == BF3905_MIPI_SENSOR_ID) {
                 break; 
    	    }
         SENSORDB("BF3905MIPIOpen Read Sensor ID Fail = 0x%x\n", sensor_id); 
    	    
    	    retry--; 
	}while (retry > 0); 
	
	if (sensor_id != BF3905_MIPI_SENSOR_ID) {
	    return ERROR_SENSOR_CONNECT_FAIL;
	}

  memset(&BF3905MIPI_Sensor_Driver, 0, sizeof(struct BF3905MIPI_Sensor_Struct)); 
	BF3905MIPI_Sensor_Driver.MPEG4_encode_mode=KAL_FALSE;
	BF3905MIPI_Sensor_Driver.dummy_pixels=0;
	BF3905MIPI_Sensor_Driver.dummy_lines=0;
	BF3905MIPI_Sensor_Driver.extra_exposure_lines=0;
	BF3905MIPI_Sensor_Driver.exposure_lines=0;
	BF3905MIPI_Sensor_Driver.MODE_CAPTURE=KAL_FALSE;
		
	BF3905MIPI_Sensor_Driver.bNight_mode =KAL_FALSE; // to distinguish night mode or auto mode, default: auto mode setting
	BF3905MIPI_Sensor_Driver.bBanding_value = AE_FLICKER_MODE_50HZ; // to distinguish between 50HZ and 60HZ.
		
	BF3905MIPI_Sensor_Driver.fPV_PCLK = 24; //26000000;
	BF3905MIPI_Sensor_Driver.iPV_Pixels_Per_Line = 0;

//	BF3905MIPI_set_isp_driving_current(1);
	// initail sequence write in
//    BF3905_MIPI_write_cmos_sensor(0x12, 0x80);
    mDELAY(10);
    BF3905MIPI_Sensor_Driver_Init();		
    SENSORDB("BF3905MIPIOpen_end \n");
    
    return ERROR_NONE;
}   /* BF3905MIPIOpen  */



/*************************************************************************
* FUNCTION
*	BF3905MIPI_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 BF3905MIPI_GetSensorID(UINT32 *sensorID)
{
	kal_uint16 sensor_id = 0;
	//Read sensor ID to adjust I2C is OK?
	sensor_id = ((BF3905_MIPI_read_cmos_sensor(0xfc)<<8)|(BF3905_MIPI_read_cmos_sensor(0xfd)));
	
	SENSORDB("BF3905MIPI Sensor Read ID 0x%x\n", sensor_id);
	
	if(BF3905_MIPI_SENSOR_ID != sensor_id) 
	{
	  *sensorID = 0xFFFFFFFF;
	  SENSORDB("BF3905MIPI Sensor Read ID Failed\n");
	  return ERROR_SENSOR_CONNECT_FAIL;
	}
	*sensorID = sensor_id;
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	BF3905MIPIClose
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 BF3905MIPIClose(void)
{
	kal_uint8 tmp1;
   // tmp1 = closed;
	//CAMERA_CONTROL_FLOW(tmp1,closed++);
   SENSORDB("BF3905MIPIClose\n");
	return ERROR_NONE;
}   /* BF3905MIPIClose */

static void BF3905MIPI_HVMirror(ACDK_SENSOR_IMAGE_MIRROR_ENUM SensorImageMirror)
{
	//volatile kal_uint32 temp_reg2=BF3905_MIPI_read_cmos_sensor(0x1E), temp_reg1=(temp_reg2&0x0F);
	kal_uint32 iTemp;
	kal_uint32 iTemp2;	
    kal_uint8 temp_r = 0;
	
	temp_r = BF3905_MIPI_read_cmos_sensor(0x1e);
	
      #if 0
	#if defined(AGOLD_BF3905MIPI_YUV_HV_MIRROR) //[Agold][xxd]
		SensorImageMirror  = IMAGE_HV_MIRROR;
	#elif defined(AGOLD_BF3905MIPI_YUV_H_MIRROR)
		SensorImageMirror   = IMAGE_H_MIRROR;
	#elif defined(AGOLD_BF3905MIPI_YUV_V_MIRROR)
		SensorImageMirror   = IMAGE_V_MIRROR;
	#endif 
                SensorImageMirror ^= IMAGE_HV_MIRROR; //[Agold][xxd][add for qq video rotate 180]   
      #endif
	iTemp2= iTemp;
	switch (SensorImageMirror)
	{
		case IMAGE_NORMAL:
			BF3905_MIPI_write_cmos_sensor(0x1e, temp_r & 0xCF); // 0x1e[4] vertical flip, 0:normal, 1:vertical				                                                    //0x1e[5] mirror, 0:normal, 1:mirror			
			break;

		case IMAGE_H_MIRROR:			 
			BF3905_MIPI_write_cmos_sensor(0x1e, temp_r | 0x20); 
			break;

		case IMAGE_V_MIRROR:			 
            BF3905_MIPI_write_cmos_sensor(0x1e, temp_r | 0x10); 
			break;

		case IMAGE_HV_MIRROR:						 
            BF3905_MIPI_write_cmos_sensor(0x1e, temp_r | 0x30); 
			break;
	}


}
/*************************************************************************
* FUNCTION
* BF3905MIPI_Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint32 BF3905MIPI_Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
	BF3905MIPI_Sensor_Driver.fPV_PCLK=24000000;//26000000
	BF3905MIPI_Sensor_Driver.MODE_CAPTURE=KAL_FALSE;

	if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO){
		BF3905MIPI_Sensor_Driver.MPEG4_encode_mode = KAL_TRUE;  // MPEG4 Encode Mode
	}else{
		BF3905MIPI_Sensor_Driver.MPEG4_encode_mode = KAL_FALSE;  
	}


	// BF3905MIPI_HVMirror(sensor_config_data->SensorImageMirror);
	BF3905MIPI_HVMirror(IMAGE_HV_MIRROR);//IMAGE_NORMAL//IMAGE_HV_MIRROR

//	BF3905MIPI_Sensor_Driver.dummy_pixels = 0;
//	BF3905MIPI_Sensor_Driver.dummy_lines = 42;
//	BF3905MIPI_Sensor_Driver.iPV_Pixels_Per_Line =BF3905_VGA_PERIOD_PIXEL_NUMS+BF3905MIPI_Sensor_Driver.dummy_pixels;  
//	BF3905MIPI_Set_Dummy(BF3905MIPI_Sensor_Driver.dummy_pixels, BF3905MIPI_Sensor_Driver.dummy_lines);

	
	image_window->GrabStartX= BF3905_IMAGE_SENSOR_VGA_INSERTED_PIXELS;
	image_window->GrabStartY= BF3905_IMAGE_SENSOR_VGA_INSERTED_LINES;
	image_window->ExposureWindowWidth = BF3905_IMAGE_SENSOR_PV_WIDTH;
	image_window->ExposureWindowHeight =BF3905_IMAGE_SENSOR_PV_HEIGHT;

	if(KAL_TRUE == BF3905MIPI_Sensor_Driver.bNight_mode) // for nd 128 noise,decrease color matrix
	{
	}

	// copy sensor_config_data
	memcpy(&BF3905MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;

}   /*  BF3905MIPI_Preview   */

UINT32 BF3905MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	kal_uint8 tmp1;
//    tmp1 = res;
//	CAMERA_CONTROL_FLOW(tmp1,res++);

	pSensorResolution->SensorFullWidth=BF3905_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight=BF3905_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=BF3905_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=BF3905_IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=BF3905_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorVideoHeight=BF3905_IMAGE_SENSOR_PV_HEIGHT;
	return ERROR_NONE;
}	/* BF3905MIPIGetResolution() */

UINT32 BF3905MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	
	SENSORDB("BF3905MIPIGetInfo \n");
		pSensorInfo->SensorPreviewResolutionX = BF3905_IMAGE_SENSOR_PV_WIDTH;
		pSensorInfo->SensorPreviewResolutionY = BF3905_IMAGE_SENSOR_PV_HEIGHT;
		pSensorInfo->SensorFullResolutionX = BF3905_IMAGE_SENSOR_PV_WIDTH;
		pSensorInfo->SensorFullResolutionY = BF3905_IMAGE_SENSOR_PV_HEIGHT;
	
		pSensorInfo->SensorCameraPreviewFrameRate=30;
		pSensorInfo->SensorVideoFrameRate=30;
		pSensorInfo->SensorStillCaptureFrameRate=30;
		pSensorInfo->SensorWebCamCaptureFrameRate=30;
		pSensorInfo->SensorResetActiveHigh=FALSE;
		pSensorInfo->SensorResetDelayCount=1;
#if 0
		pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YVYU;
		pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorInterruptDelayLines = 1;
		pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;
		pSensorInfo->SensorMasterClockSwitch = 0;
#endif 
		pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
	
		pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
		pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
		pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;

		pSensorInfo->SensorInterruptDelayLines = 1;
		pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

	    pSensorInfo->YUVAwbDelayFrame = 2;
	    pSensorInfo->YUVEffectDelayFrame = 2;

	    pSensorInfo->CaptureDelayFrame = 1;
	    pSensorInfo->PreviewDelayFrame = 0;
	    pSensorInfo->VideoDelayFrame = 4;
		pSensorInfo->SensorMasterClockSwitch = 0; 
	
	
		switch (ScenarioId)
		{
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				pSensorInfo->SensorClockFreq=24;//26;
				pSensorInfo->SensorClockDividCount= 3;
				pSensorInfo->SensorClockRisingCount= 0;
				pSensorInfo->SensorClockFallingCount= 2;
				pSensorInfo->SensorPixelClockCount= 3;
				pSensorInfo->SensorDataLatchCount= 2;
				pSensorInfo->SensorGrabStartX = 1; 
				pSensorInfo->SensorGrabStartY = 1;

				pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
				pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
				pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 4; 
				pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
				pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
				pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
				pSensorInfo->SensorPacketECCOrder = 1;
				break;
			
			default:			
				pSensorInfo->SensorClockFreq=24;//26;
				pSensorInfo->SensorClockDividCount= 3;
				pSensorInfo->SensorClockRisingCount= 0;
				pSensorInfo->SensorClockFallingCount= 2;
				pSensorInfo->SensorPixelClockCount= 3;
				pSensorInfo->SensorDataLatchCount= 2;
				pSensorInfo->SensorGrabStartX = 1; 
				pSensorInfo->SensorGrabStartY = 1;
				break;
		}

	memcpy(pSensorConfigData, &BF3905MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	
	return ERROR_NONE;
}	/* BF3905MIPIGetInfo() */


UINT32 BF3905MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	CAMERA_CONTROL_FLOW(ScenarioId,ScenarioId);

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			BF3905MIPI_Preview(pImageWindow, pSensorConfigData);
		break;
		
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			//BF3905MIPI_Capture(pImageWindow, pSensorConfigData);
			BF3905MIPI_Preview(pImageWindow, pSensorConfigData);
		break;

		case MSDK_SCENARIO_ID_CAMERA_ZSD:		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: 
		break;

		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* MT9P012Control() */

static BOOL BF3905MIPI_set_param_wb(UINT16 para)
{
	kal_uint8  temp_reg;
        kal_uint8 temp_r=0;
	          temp_r=BF3905_MIPI_read_cmos_sensor(0x13);
	if(BF3905MIPI_Sensor_Driver.u8Wb_value==para)
		return FALSE;

	
	BF3905MIPI_Sensor_Driver.u8Wb_value = para;

	switch (para)
	 {
		 case AWB_MODE_OFF:
		 //BF3905_MIPI_write_cmos_sensor(0xfd,0x00);				   
		 //BF3905_MIPI_write_cmos_sensor(0x32,0x05);	   
		 break;
			 
		 case AWB_MODE_AUTO:
			BF3905_MIPI_write_cmos_sensor(0x13, temp_r|0x02); //bit[1]:AWB Auto:1 menual:0
			//BF3905_MIPI_write_cmos_sensor(0x13, 0x05);
			BF3905_MIPI_write_cmos_sensor(0x01, 0x15);
			BF3905_MIPI_write_cmos_sensor(0x02, 0x23);		
			 break;
	
		 case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			BF3905_MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			BF3905_MIPI_write_cmos_sensor(0x01, 0x10);
			BF3905_MIPI_write_cmos_sensor(0x02, 0x28);											   
			 break;
	
		 case AWB_MODE_DAYLIGHT: //sunny
		 // BF3905MIPI_reg_WB_auto	
			BF3905_MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			BF3905_MIPI_write_cmos_sensor(0x01, 0x11);
			BF3905_MIPI_write_cmos_sensor(0x02, 0x26);  														   
			 break;
	
		 case AWB_MODE_INCANDESCENT: //office
			BF3905_MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			BF3905_MIPI_write_cmos_sensor(0x01, 0x1f);
			BF3905_MIPI_write_cmos_sensor(0x02, 0x15); 															
			 break;
	
		 case AWB_MODE_TUNGSTEN: //home
			BF3905_MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			BF3905_MIPI_write_cmos_sensor(0x01, 0x1a);
			BF3905_MIPI_write_cmos_sensor(0x02, 0x1e); 														   
			 break;
			 
		 case AWB_MODE_FLUORESCENT:
			BF3905_MIPI_write_cmos_sensor(0x13, temp_r&0xfd); //bit[1]:AWB Auto:1 menual:0
			BF3905_MIPI_write_cmos_sensor(0x01, 0x1a);
			BF3905_MIPI_write_cmos_sensor(0x02, 0x1e); 														   
			 break;
	
		 default:
			 return FALSE;
	 }


	return TRUE;
} /* BF3905MIPI_set_param_wb */

//whl ok
BOOL BF3905MIPI_set_param_effect(UINT16 para)
{
	kal_uint32 ret = KAL_TRUE;

	if(para==BF3905MIPI_Sensor_Driver.u8Effect_value)
		return FALSE;

	
	BF3905MIPI_Sensor_Driver.u8Effect_value = para;
    switch (para)
    {
        case MEFFECT_OFF: 

			BF3905_MIPI_write_cmos_sensor(0xb4,0xe3);
			BF3905_MIPI_write_cmos_sensor(0xb1,0xff);
			BF3905_MIPI_write_cmos_sensor(0xb2,0xff); //0xa0
			BF3905_MIPI_write_cmos_sensor(0xb4,0x63); 
			BF3905_MIPI_write_cmos_sensor(0xb1,0xc0); //0xb3
			BF3905_MIPI_write_cmos_sensor(0xb2,0xb0); //0xa0		
			BF3905_MIPI_write_cmos_sensor(0x56,0x40);  

			BF3905_MIPI_write_cmos_sensor(0x70,0x0b); 
			BF3905_MIPI_write_cmos_sensor(0x69,0x00);  
			BF3905_MIPI_write_cmos_sensor(0x67,0x80);   
			BF3905_MIPI_write_cmos_sensor(0x68,0x80);  
			BF3905_MIPI_write_cmos_sensor(0xb4,0x63);  
            break;

        case MEFFECT_SEPIA:  
			BF3905_MIPI_write_cmos_sensor(0xb4,0xe3); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb1,0xef); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb2,0xef); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb4,0x63); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb1,0xba); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb2,0xaa); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x56,0x40); // Normal,
			
			BF3905_MIPI_write_cmos_sensor(0x70,0x0b); // Normal, 			
			BF3905_MIPI_write_cmos_sensor(0x69,0x20); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x67,0x60); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x68,0xa0); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb4,0x03); // Normal,

            break;

        case MEFFECT_NEGATIVE: 
			BF3905_MIPI_write_cmos_sensor(0xb4,0xe3); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb1,0xef); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb2,0xef); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb4,0x63); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb1,0xba); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb2,0xaa); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x56,0x40); // Normal, 
			
			BF3905_MIPI_write_cmos_sensor(0x70,0x0b); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x69,0x01); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x67,0x80); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x68,0x80); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb4,0x03); // Normal, 
            break;

        case MEFFECT_SEPIAGREEN:

            break;

        case MEFFECT_SEPIABLUE:


            break;
			
		case MEFFECT_MONO: //B&W
			BF3905_MIPI_write_cmos_sensor(0xb4,0xe3); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb1,0xef); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb2,0xef); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb4,0x63); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb1,0xba); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb2,0xaa); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x56,0x40); // Normal, 
			
			BF3905_MIPI_write_cmos_sensor(0x70,0x0b); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x69,0x20); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x67,0x80); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0x68,0x80); // Normal, 
			BF3905_MIPI_write_cmos_sensor(0xb4,0x03); // Normal, 

			break;
        default:
            return FALSE;
    }

	return ret;

} /* BF3905MIPI_set_param_effect */

void BF3905MIPI_set_banding_for_50Hz(void)
{
   BF3905_MIPI_write_cmos_sensor(0x80,0xd2);//0xd0
   BF3905_MIPI_write_cmos_sensor(0x8a,0x93);//96->a2
	
printk("BF3905MIPI_set_banding_for_50Hz\n");

}


void BF3905MIPI_set_banding_for_60Hz(void)
{
printk("BF3905MIPI_set_banding_for_60Hz\n");
   BF3905_MIPI_write_cmos_sensor(0x80,0xd0);//0xd0
   BF3905_MIPI_write_cmos_sensor(0x8b,0x7b);
}

BOOL BF3905MIPI_set_param_banding(UINT16 para)
{
	//if(BF3905MIPI_Sensor_Driver.bBanding_value == para)
	//	return TRUE;
	
	BF3905MIPI_Sensor_Driver.bBanding_value = para;
	
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			isBf3905Banding = 0;
			printk("BF3905MIPI_set_param_banding_50hz\n");
			//BF3905MIPI_set_banding_for_50Hz();
			break;
		case AE_FLICKER_MODE_60HZ:
			isBf3905Banding = 1;
			printk("BF3905MIPI_set_param_banding_60hz\n");
			//BF3905MIPI_set_banding_for_60Hz();
			break;
		default:
			return FALSE;
	}

	return TRUE;
} /* BF3905MIPI_set_param_banding */

//whl ok
BOOL BF3905MIPI_set_param_exposure(UINT16 para)
{
	if(para == BF3905MIPI_Sensor_Driver.u8Ev_value)
		return FALSE;

	BF3905MIPI_Sensor_Driver.u8Ev_value = para;

    switch (para)
    {
        case AE_EV_COMP_n13:
		BF3905_MIPI_write_cmos_sensor(0x24, 0x10);

            break;

        case AE_EV_COMP_n10:
		BF3905_MIPI_write_cmos_sensor(0x24, 0x20);

            break;

        case AE_EV_COMP_n07:
		BF3905_MIPI_write_cmos_sensor(0x24, 0x38);

            break;

        case AE_EV_COMP_n03:
		BF3905_MIPI_write_cmos_sensor(0x24, 0x40);

            break;

        case AE_EV_COMP_00:
                BF3905_MIPI_write_cmos_sensor(0x24, 0x40); //EV 0

            break;

        case AE_EV_COMP_03:
		BF3905_MIPI_write_cmos_sensor(0x24, 0x50);

            break;

        case AE_EV_COMP_07:
		BF3905_MIPI_write_cmos_sensor(0x24, 0x60);

            break;

        case AE_EV_COMP_10:
		BF3905_MIPI_write_cmos_sensor(0x24, 0x68);

            break;

        case AE_EV_COMP_13:
		BF3905_MIPI_write_cmos_sensor(0x24, 0x7F);

            break;

        default:
            return FALSE;
    }

	return TRUE;
} /* BF3905MIPI_set_param_exposure */

kal_uint32 BF3905MIPI_YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
/*
CMD: AE Strob(4)->AE Flick(0) -> AF Mode(6) -> AF Metring(7) -> AE metring(1)
-> EV(3) -> AWB(5) -> ISO(2) -> AE Scene Mode(13) ->Brightness(1) -> Hue(9)
-> Saturation(10) -> Edge(8) -> Contrast(12) -> Scene Mode(14) -> Effect(15)

For Current: Banding->EV->WB->Effect
*/
    //printk("[BF3905MIPI_YUVSensorSetting], Cmd = 0x%x, Para = 0x%x\n", iCmd, iPara); 
	//CAMERA_CONTROL_FLOW(iCmd,iPara);

	switch (iCmd) {
		case FID_SCENE_MODE:
                //printk("\n\nBF3905MIPIYUVSensorSetting:para=%d\n\n",iPara);
			
		    if (iPara == SCENE_MODE_OFF){
		        BF3905MIPI_night_mode(FALSE); 
		    }else if (iPara == SCENE_MODE_NIGHTSCENE){
               BF3905MIPI_night_mode(TRUE); 
		    }	    
		  
		    break; 
		case FID_AWB_MODE:
			BF3905MIPI_set_param_wb(iPara);
		break;
		case FID_COLOR_EFFECT:
			BF3905MIPI_set_param_effect(iPara);
		break;
		case FID_AE_EV:	
			BF3905MIPI_set_param_exposure(iPara);
		break;
		case FID_AE_FLICKER:
			BF3905MIPI_set_param_banding(iPara);
			
			 if (BF3905MIPI_Sensor_Driver.bNight_mode == KAL_FALSE){
		        BF3905MIPI_night_mode(FALSE); 
		    }else if (BF3905MIPI_Sensor_Driver.bNight_mode == KAL_TRUE){
               	BF3905MIPI_night_mode(TRUE); 
		    }	    
		break;
		default:
		break;
	}
	
	return TRUE;
}   /* BF3905MIPI_YUVSensorSetting */

kal_uint32 BF3905MIPI_YUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 temp ;//= BF3905_MIPI_read_cmos_sensor(0x3B);
    BF3905MIPI_Sensor_Driver.MPEG4_encode_mode = KAL_TRUE; 

    if (u2FrameRate == 30)
    {
    }
    else if (u2FrameRate == 15)       
    {
    }
    else 
    {
        printk("Wrong frame rate setting \n");
    }   
    
	printk("\n BF3905MIPI_YUVSetVideoMode:u2FrameRate=%d\n\n",u2FrameRate);
    return TRUE;
}

UINT32 BF3905MIPISetSoftwarePWDNMode(kal_bool bEnable)
{
#if 0
    SENSORDB("[BF3905MIPISetSoftwarePWDNMode] Software Power down enable:%d\n", bEnable);
    
    if(bEnable) {   // enable software sleep mode   
	 BF3905_MIPI_write_cmos_sensor(0x09, 0x80);
    } else {
        BF3905_MIPI_write_cmos_sensor(0x09, 0x00);  
    }
#endif
    return TRUE;
}

/*************************************************************************
* FUNCTION
*    BF3905MIPI_get_size
*
* DESCRIPTION
*    This function return the image width and height of image sensor.
*
* PARAMETERS
*    *sensor_width: address pointer of horizontal effect pixels of image sensor
*    *sensor_height: address pointer of vertical effect pixels of image sensor
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
void BF3905MIPI_get_size(kal_uint16 *sensor_width, kal_uint16 *sensor_height)
{
  *sensor_width = BF3905_IMAGE_SENSOR_FULL_WIDTH; /* must be 4:3 */
  *sensor_height = BF3905_IMAGE_SENSOR_FULL_HEIGHT;
}

/*************************************************************************
* FUNCTION
*    BF3905MIPI_get_period
*
* DESCRIPTION
*    This function return the image width and height of image sensor.
*
* PARAMETERS
*    *pixel_number: address pointer of pixel numbers in one period of HSYNC
*    *line_number: address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
void BF3905MIPI_get_period(kal_uint16 *pixel_number, kal_uint16 *line_number)
{
  *pixel_number = BF3905_VGA_PERIOD_PIXEL_NUMS+BF3905MIPI_Sensor_Driver.dummy_pixels;
  *line_number = BF3905_VGA_PERIOD_LINE_NUMS+BF3905MIPI_Sensor_Driver.dummy_lines;
}

void BF3905MIPIGetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
    SENSORDB("BF3905MIPIGetAFMaxNumFocusAreas, *pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);
}


void BF3905MIPIGetAFMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
    SENSORDB("BF3905MIPIGetAFMaxNumMeteringAreas,*pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);
}

/*************************************************************************
* FUNCTION
*    BF3905MIPI_feature_control
*
* DESCRIPTION
*    This function control sensor mode
*
* PARAMETERS
*    id: scenario id
*    image_window: image grab window
*    cfg_data: config data
*
* RETURNS
*    error code
*
* LOCAL AFFECTED
*
*************************************************************************/
UINT32 BF3905MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM id, kal_uint8 *para, kal_uint32 *len)
{
	UINT32 *pFeatureData32=(UINT32 *) para;

	switch (id)
	{
		case SENSOR_FEATURE_GET_RESOLUTION: /* no use */
			BF3905MIPI_get_size((kal_uint16 *)para, (kal_uint16 *)(para + sizeof(kal_uint16)));
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			BF3905MIPI_get_period((kal_uint16 *)para, (kal_uint16 *)(para + sizeof(kal_uint16)));
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*(kal_uint32 *)para = BF3905MIPI_Sensor_Driver.fPV_PCLK;
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE: 
			BF3905MIPI_night_mode((kal_bool)*(kal_uint16 *)para);
			break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			BF3905_MIPI_write_cmos_sensor(((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER: /* 10 */
			((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData = BF3905_MIPI_read_cmos_sensor(((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr);
			break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
			memcpy(&BF3905MIPI_Sensor_Driver.eng.CCT, para, sizeof(BF3905MIPI_Sensor_Driver.eng.CCT));
			break;
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_GET_CONFIG_PARA: /* no use */
			break;
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
			break;
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
		case SENSOR_FEATURE_GET_GROUP_INFO: /* 20 */
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*
		* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
		* if EEPROM does not exist in camera module.
		*/
			*(kal_uint32 *)para = LENS_DRIVER_ID_DO_NOT_CARE;
			*len = sizeof(kal_uint32);
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
	//		BF3905MIPI_YUVSensorSetting((FEATURE_ID)(UINT32 *)para, (UINT32 *)(para+1));
			
			BF3905MIPI_YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
#if 0		    		
		case SENSOR_FEATURE_QUERY:
			BF3905MIPI_Query(pSensorFeatureInfo);
			*pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
			break;		
		case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
			/* update yuv capture raw support flag by *pFeatureData16 */
			break;		
#endif 			
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			BF3905MIPI_YUVSetVideoMode(*para);
			break;
			
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
             BF3905MIPI_GetSensorID(pFeatureData32); 
             break; 	
        case SENSOR_FEATURE_SET_SOFTWARE_PWDN:
             BF3905MIPISetSoftwarePWDNMode((BOOL)*pFeatureData32);        	        	
             break;

	    case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
	 		BF3905MIPIGetAFMaxNumFocusAreas(pFeatureData32);
	 		*len=4;
	 		break;
	 
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
	 		BF3905MIPIGetAFMaxNumMeteringAreas(pFeatureData32);
	 		*len=4;
	 		break;

		default:
			break;
	}
	return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT	SensorFuncBF3905MIPI=
{
	BF3905MIPIOpen,
	BF3905MIPIGetInfo,
	BF3905MIPIGetResolution,
	BF3905MIPIFeatureControl,
	BF3905MIPIControl,
	BF3905MIPIClose
};

UINT32 BF3905_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncBF3905MIPI;

	return ERROR_NONE;
}	/* SensorInit() */

