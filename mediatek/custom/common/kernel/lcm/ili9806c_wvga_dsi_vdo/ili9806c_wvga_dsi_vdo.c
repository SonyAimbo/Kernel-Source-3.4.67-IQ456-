/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
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
*  permission of MediaTek Inc. (C) 2008
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


#ifdef BUILD_LK
#include "platform/mt_gpio.h"
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (480)
#define FRAME_HEIGHT                                        (854)

#define REGFLAG_DELAY                                       0XFE
#define REGFLAG_END_OF_TABLE                                0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_ILI9806C 0x9816

#define LCM_DSI_CMD_MODE                                    0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static unsigned int lcm_esd_test = TRUE;      ///only for ESD test
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting_0[] = {
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}}, // Change to Page 1
	{0x08,1,{0x10}},                     // output SDA
	{0x21,1,{0x01}},                 // DE = 1 Active
	{0x30,1,{0x01}},                 // 480 X 854
	{0x31,1,{0x02}},                 // 2-Dot Inversion
	{0x60,1,{0x07}},                 // SDTI
	{0x61,1,{0x00}},                 // CRTI
	{0x62,1,{0x08}},                 // EQTI
	{0x63,1,{0x00}},                 // PCTI
	{0x40,1,{0x10}},                 // BT  +2.5/-2.5 pump for DDVDH-L
	{0x41,1,{0x33}},                 // DVDDH DVDDL clamp AVDD5.8 5.2 AVEE -5.4  
	{0x42,1,{0x02}},                 // VGH/VGL 
	{0x43,1,{0x09}},                 // VGH/VGL 
	{0x44,1,{0x09}},                 // VGH/VGL  
	{0x50,1,{0x60}},                 // VGMP(+4.2)
	{0x51,1,{0x60}},                 // VGMN(-4.2)
	{0x52,1,{0x00}},                 // Flicker MSB
	{0x53,1,{0x39}},                 // Flicker LSB
	{0x57,1,{0x50}},                 // LOW VOLTAGE DETECTION

	//++++++++++++++++++ Gamma Setting ++++++++++++++++++//
	{0xA0,1,{0x00}}, // Gamma 0 /255
	{0xA1,1,{0x11}}, // Gamma 4 /251
	{0xA2,1,{0x18}}, // Gamma 8 /247
	{0xA3,1,{0x0D}}, // Gamma 16 /239
	{0xA4,1,{0x06}}, // Gamma 24 /231
	{0xA5,1,{0x09}}, // Gamma 52 / 203
	{0xA6,1,{0x08}}, // Gamma 80 / 175
	{0xA7,1,{0x06}}, // Gamma 108 /147
	{0xA8,1,{0x08}}, // Gamma 147 /108
	{0xA9,1,{0x0A}}, // Gamma 175 / 80
	{0xAA,1,{0x13}}, // Gamma 203 / 52
	{0xAB,1,{0x08}}, // Gamma 231 / 24
	{0xAC,1,{0x0E}}, // Gamma 239 / 16
	{0xAD,1,{0x17}}, // Gamma 247 / 8
	{0xAE,1,{0x0D}}, // Gamma 251 / 4
	{0xAF,1,{0x00}}, // Gamma 255 / 0
	//==============Negative
	{0xC0,1,{0x00}}, // Gamma 0 
	{0xC1,1,{0x11}}, // Gamma 4
	{0xC2,1,{0x18}}, // Gamma 8
	{0xC3,1,{0x0D}}, // Gamma 16
	{0xC4,1,{0x06}}, // Gamma 24
	{0xC5,1,{0x09}}, // Gamma 52
	{0xC6,1,{0x08}}, // Gamma 80
	{0xC7,1,{0x06}}, // Gamma 108
	{0xC8,1,{0x08}}, // Gamma 147
	{0xC9,1,{0x0A}}, // Gamma 175
	{0xCA,1,{0x13}}, // Gamma 203
	{0xCB,1,{0x08}}, // Gamma 231
	{0xCC,1,{0x0E}}, // Gamma 239
	{0xCD,1,{0x17}}, // Gamma 247
	{0xCE,1,{0x0D}}, // Gamma 251
	{0xCF,1,{0x00}}, // Gamma 255

	//****************************** Page 6 Command ******************************//
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}}, // Change to Page 6
	{0x00,1,{0x20}},
	{0x01,1,{0x04}},
	{0x02,1,{0x00}},    
	{0x03,1,{0x00}},
	{0x04,1,{0x01}},
	{0x05,1,{0x01}},
	{0x06,1,{0x88}},    
	{0x07,1,{0x04}},
	{0x08,1,{0x01}},
	{0x09,1,{0x90}},    
	{0x0A,1,{0x03}},    
	{0x0B,1,{0x01}},    
	{0x0C,1,{0x01}},
	{0x0D,1,{0x01}},
	{0x0E,1,{0x00}},
	{0x0F,1,{0x00}},
	{0x10,1,{0x55}},
	{0x11,1,{0x53}},
	{0x12,1,{0x01}},
	{0x13,1,{0x0D}},
	{0x14,1,{0x0D}}, 
	{0x15,1,{0x43}}, 
	{0x16,1,{0x0B}}, 
	{0x17,1,{0x00}}, 
	{0x18,1,{0x00}}, 
	{0x19,1,{0x00}}, 
	{0x1A,1,{0x00}}, 
	{0x1B,1,{0x00}}, 
	{0x1C,1,{0x00}}, 
	{0x1D,1,{0x00}}, 
	{0x20,1,{0x01}}, 
	{0x21,1,{0x23}}, 
	{0x22,1,{0x45}}, 
	{0x23,1,{0x67}}, 
	{0x24,1,{0x01}}, 
	{0x25,1,{0x23}}, 
	{0x26,1,{0x45}}, 
	{0x27,1,{0x67}}, 
	{0x30,1,{0x02}}, 
	{0x31,1,{0x22}}, 
	{0x32,1,{0x11}},
	{0x33,1,{0xAA}},
	{0x34,1,{0xBB}},
	{0x35,1,{0x66}},
	{0x36,1,{0x00}},
	{0x37,1,{0x22}},
	{0x38,1,{0x22}},
	{0x39,1,{0x22}},
	{0x3A,1,{0x22}},
	{0x3B,1,{0x22}},
	{0x3C,1,{0x22}},
	{0x3D,1,{0x22}},
	{0x3E,1,{0x22}},
	{0x3F,1,{0x22}},
	{0x40,1,{0x22}},
	{0x52,1,{0x10}},
	{0x53,1,{0x10}},  //VGLO refer VGL_REG
	     
	//****************************** Page 7 Command ******************************//
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}}, // Change to Page 7
	{0x17,1,{0x22}},  // VGL_REG ON
	{0xE1,1,{0x79}},
	{0x02,1,{0x77}},
	{0x06,1,{0x13}}, //VCL = -2 VCI
	{REGFLAG_DELAY,10,{}}, //0x0A
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}}, // Change to Page 0

	{0x55,1,{0xB0}}, // Write Content Adaptive Brightness Control Value
	{0x11,1,{0x00}},
	{REGFLAG_DELAY,120,{}}, //0x78
	 // Display ON
	{0x29,1,{0x00}},
	{REGFLAG_DELAY,50,{}}, //0x32
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x08}},

	 {REGFLAG_END_OF_TABLE, 0x00, {}}	
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}}, // Change to Page 1
	{0x08,1,{0x10}},				// output SDA
	{0x21,1,{0x01}},				// DE = 1 Active
	{0x30,1,{0x01}},				// 480 X 854
	{0x31,1,{0x02}},				// 2-Dot Inversion
	{0x40,1,{0x15}},				// DDVDH/L BT 17 -10 5.94 -5.47 -2.3 1.6 43.4 21.88 0.17
	{0x41,1,{0x33}},				// DVDDH DVDDL clamp AVDD5.8 5.2 AVEE -5.4  
	{0x42,1,{0x03}},				// VGH/VGL 
	{0x43,1,{0x09}},				// VGH/VGL 
	{0x44,1,{0x09}},				// VGH/VGL  
	{0x50,1,{0x78}},				// VGMP(+4.2)
	{0x51,1,{0x66}},				// VGMN(-4.2)
	{0x60,1,{0x07}},				// SDTI
	{0x61,1,{0x00}},				// CRTI
	{0x62,1,{0x08}},				// EQTI
	{0x63,1,{0x00}},				// PCTI
	{0x52,1,{0x00}},				// Flicker MSB
	{0x53,1,{0x46}},				// Flicker LSB
	{0x57,1,{0x50}},				// LOW VOLTAGE DETECTION

	//++++++++++++++++++ Gamma Setting ++++++++++++++++++//
	{0xA0,1,{0x00}}, // Gamma 0 /255
	{0xA1,1,{0x06}}, // Gamma 4 /251
	{0xA2,1,{0x00}}, // Gamma 8 /247
	{0xA3,1,{0x13}}, // Gamma 16 /239
	{0xA4,1,{0x10}}, // Gamma 24 /231
	{0xA5,1,{0x1C}}, // Gamma 52 / 203
	{0xA6,1,{0x0F}}, // Gamma 80 / 175
	{0xA7,1,{0x0F}}, // Gamma 108 /147
	{0xA8,1,{0x01}}, // Gamma 147 /108
	{0xA9,1,{0x06}}, // Gamma 175 / 80
	{0xAA,1,{0x04}}, // Gamma 203 / 52
	{0xAB,1,{0x08}}, // Gamma 231 / 24
	{0xAC,1,{0x0E}}, // Gamma 239 / 16
	{0xAD,1,{0x33}}, // Gamma 247 / 8
	{0xAE,1,{0x31}}, // Gamma 251 / 4
	{0xAF,1,{0x00}}, // Gamma 255 / 0
	//==============Negative
	{0xC0,1,{0x00}}, // Gamma 0 
	{0xC1,1,{0x01}}, // Gamma 4
	{0xC2,1,{0x0C}}, // Gamma 8
	{0xC3,1,{0x0A}}, // Gamma 16
	{0xC4,1,{0x01}}, // Gamma 24
	{0xC5,1,{0x14}}, // Gamma 52
	{0xC6,1,{0x06}}, // Gamma 80
	{0xC7,1,{0x03}}, // Gamma 108
	{0xC8,1,{0x04}}, // Gamma 147
	{0xC9,1,{0x08}}, // Gamma 175
	{0xCA,1,{0x07}}, // Gamma 203
	{0xCB,1,{0x01}}, // Gamma 231
	{0xCC,1,{0x08}}, // Gamma 239
	{0xCD,1,{0x28}}, // Gamma 247
	{0xCE,1,{0x1F}}, // Gamma 251
	{0xCF,1,{0x00}}, // Gamma 255

	//****************************** Page 6 Command ******************************//
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}}, // Change to Page 6
	{0x00,1,{0x21}},
	{0x01,1,{0x06}},
	{0x02,1,{0x00}},    
	{0x03,1,{0x00}},
	{0x04,1,{0x01}},
	{0x05,1,{0x01}},
	{0x06,1,{0x80}},    
	{0x07,1,{0x02}},
	{0x08,1,{0x05}},
	{0x09,1,{0x00}},    
	{0x0A,1,{0x00}},    
	{0x0B,1,{0x00}},    
	{0x0C,1,{0x01}},
	{0x0D,1,{0x01}},
	{0x0E,1,{0x00}},
	{0x0F,1,{0x00}},
	{0x10,1,{0xF0}},
	{0x11,1,{0xF4}},
	{0x12,1,{0x00}},
	{0x13,1,{0x00}},
	{0x14,1,{0x00}}, 
	{0x15,1,{0xC0}}, 
	{0x16,1,{0x08}}, 
	{0x17,1,{0x00}}, 
	{0x18,1,{0x00}}, 
	{0x19,1,{0x00}}, 
	{0x1A,1,{0x00}}, 
	{0x1B,1,{0x00}}, 
	{0x1C,1,{0x00}}, 
	{0x1D,1,{0x00}}, 
	{0x20,1,{0x02}}, 
	{0x21,1,{0x13}}, 
	{0x22,1,{0x45}}, 
	{0x23,1,{0x67}}, 
	{0x24,1,{0x01}}, 
	{0x25,1,{0x23}}, 
	{0x26,1,{0x45}}, 
	{0x27,1,{0x67}}, 
	{0x30,1,{0x13}}, 
	{0x31,1,{0x22}}, 
	{0x32,1,{0x22}},
	{0x33,1,{0x22}},
	{0x34,1,{0x22}},
	{0x35,1,{0xBB}},
	{0x36,1,{0xAA}},
	{0x37,1,{0xDD}},
	{0x38,1,{0xCC}},
	{0x39,1,{0x66}},
	{0x3A,1,{0x77}},
	{0x3B,1,{0x22}},
	{0x3C,1,{0x22}},
	{0x3D,1,{0x22}},
	{0x3E,1,{0x22}},
	{0x3F,1,{0x22}},
	{0x40,1,{0x22}},
	{0x52,1,{0x10}},
	{0x53,1,{0x10}},  //VGLO refer VGL_REG
	     
	//****************************** Page 7 Command ******************************//
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}}, // Change to Page 7
	//{0x18,1,{0x1D}},
	{0x17,1,{0x22}},  // VGL_REG ON
	{0x02,1,{0x77}},
	{0xE1,1,{0x79}},
	//{0x06,1,{0x13}}, //VCL = -2 VCI
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}}, // Change to Page 0

	{0x55,1,{0x80}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY,120,{}}, //0x78
	 // Display ON
	{0x29,1,{0x00}},
	{REGFLAG_DELAY,50,{}}, //0x32
	{0x3A,1,{0x66}},
	{0x36,1, {0x00}},
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x08}},

	 {REGFLAG_END_OF_TABLE, 0x00, {}}	
	
};

static struct LCM_setting_table lcm_ese_pre_setting[] = {
{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
{REGFLAG_END_OF_TABLE, 0x00, {}}	
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x00}},
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 200, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x08}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x00}},
    {REGFLAG_DELAY, 1, {}}, 
    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x00}},
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 200, {}},

    {0xFF, 5, {0xFF,0x98,0x06,0x04,0x01}},
    {0x58, 1, {0x91}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_mode_setting[] = {
	{0x55, 1, {0x1}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

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
//  LCM Driver Implementations#ifdef BUILD_LK
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, 508);
    
    params->type   = LCM_TYPE_DSI;
    
    params->width  = FRAME_WIDTH;  
    params->height = FRAME_HEIGHT; 
    

    params->dbi.te_mode			= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity	= LCM_POLARITY_RISING;
    
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;	

    params->dsi.LANE_NUM		= LCM_TWO_LANE;

    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format	    = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting	
    params->dsi.intermediat_buffer_num = 2;
    
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count = 480*3;	

    params->dsi.vertical_sync_active		= 4;
    params->dsi.vertical_backporch		= 16;
    params->dsi.vertical_frontporch		= 8;	
    params->dsi.vertical_active_line		= FRAME_HEIGHT; 
    
    params->dsi.horizontal_sync_active		= 6; 
    params->dsi.horizontal_backporch		= 37; 
    params->dsi.horizontal_frontporch		= 37; 
    params->dsi.horizontal_blanking_pixel	= 60;
    params->dsi.horizontal_active_pixel		= FRAME_WIDTH; 
    params->dsi.compatibility_for_nvk		= 1; 		

    params->dsi.pll_div1 = 1;		
    params->dsi.pll_div2 = 1;		
    params->dsi.fbk_div = 31; 	

    // Недостающие параметры
    params->dpi.rgb_order = 1;
    params->dsi.fbk_sel = 1;
    params->dsi.cont_clock = 1;
    params->dpi.embsync = 2;
    params->dpi.lvds_tx_en = 0;
    params->dpi.io_driving_current = 0;
    params->dpi.lsb_io_driving_current = 2;
    params->dsi.ufoe_enable = 0;
    params->dsi.lcm_int_te_period = 0;
    //params->dsi.lcm_ext_te_monitor = 2; // WARNING: Если расскоментировать, то катинка плавно гаснет и потом просто черный экран
}

#ifndef BUILD_LK 
extern int lcd_firmware_version[2];
#endif

static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[5];
    char id_high=0;
    char id_low=0;
    int id=0;
    int i;
    int uiHighCnt=0, uiLowCnt=0; 

    mt_set_gpio_mode(GPIO21, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO21, GPIO_DIR_IN);
	for (i = 0; i < 6; i++)  
	{      
	    if (mt_get_gpio_in(GPIO21))        //LCM ID Pin:119       
	    {            
	        uiHighCnt++;     
	    }        
	    else        
	    {          
	        uiLowCnt++;       
	    }    
	}
	if (uiHighCnt > uiLowCnt)//IPSÆÁ
	{
		return 1;
	} 
	else//TNÆÁ
	{
		return 0;
	}
}

static void lcm_init(void)
{
    unsigned int id = 0;

    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(120);
	
    id = lcm_compare_id();

    #ifdef BUILD_LK
        printf("ILI9806C lk %s id=%d\n", __func__,id);
    #else
        printk("ILI9806C kernel %s id=%d\n", __func__,id);
    #endif

    if(0 == id)
    	push_table(lcm_initialization_setting_0, sizeof(lcm_initialization_setting_0) / sizeof(struct LCM_setting_table), 1);
    else
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    
}

static unsigned int lcm_esd_check(void)
{
    unsigned char buffer[3] = {0};
    unsigned int array[3] = {0};

    UDELAY(600);

    array[0] = 0x00063902;
    array[1] = 0x0698ffff;
    array[2] = 0x00000004;
    dsi_set_cmdq(array, 3, 1);
    //---------------------------------
    // Set Maximum Return Size
    //---------------------------------

    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x0A, &buffer[0], 1);

    array[0] = 0x00063902;
    array[1] = 0x0698ffff;
    array[2] = 0x00000804;
    dsi_set_cmdq(array, 3, 1);

    if(0x9c == buffer[0])
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

static unsigned int lcm_esd_recover(void)
{     
    #ifndef BUILD_LK   
	    printk("lcm_esd_recover enter \n");
	    lcm_init(); 
    #endif 

    return TRUE;
}

static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
    lcm_init();    
}

static unsigned char s_ucColorMode = 1;
 
LCM_DRIVER ili9806c_wvga_dsi_vdo_lcm_drv = 
{
    	.name          = "ili9806c_dsi_vdo",	// 80055218
	.set_util_funcs = lcm_set_util_funcs,	// 8003AF71
	.get_params     = lcm_get_params,		// 8003AECD
	.init           = lcm_init,				// 8003AFDD
	.suspend        = lcm_suspend,			// 8003B155
	.resume         = lcm_resume,			// 8003B049
	.compare_id    =  lcm_compare_id,		// 8003ADE9
	//.esd_check      = lcm_esd_check,		// ? 8003AE75
	.esd_recover    = lcm_esd_recover,	// ? 8003AE71
	//.update         = lcm_update,			// ? 8003AE91
	//.set_backlight	= lcm_setbacklight, // ? 8003B04D
	//.set_backlight_mode = lcm_setbacklight_mode,
};
