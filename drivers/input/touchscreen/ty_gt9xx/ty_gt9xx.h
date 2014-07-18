/* drivers/input/touchscreen/gt9xx.h
 * 
 * 2010 - 2013 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 */

/*TYDRV:liujie update this file from 2.0 to 2.2*/


#ifndef _GOODIX_GT9XX_H_
#define _GOODIX_GT9XX_H_

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <mach/gpio.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define GOODIX_SUSPEND_LEVEL 1
#endif

//***************************PART1:ON/OFF define*******************************
#define GTP_CUSTOM_CFG        0
#define GTP_CHANGE_X2Y        0
#define GTP_DRIVER_SEND_CFG   1
#define GTP_HAVE_TOUCH_KEY    1
#define GTP_POWER_CTRL_SLEEP  1
#define GTP_ICS_SLOT_REPORT   1

#define TYN_VIRTAUL_KEY_FRAMEWORK	1 
#define TY_TP_IOCTRL	1
#define TY_TP_SYSFS	1
#define GTP_AUTO_UPDATE       1   // auto update fw by .bin file as default
#define GTP_HEADER_FW_UPDATE  0    // auto update fw by gtp_default_FW in gt9xx_firmware.h, function together with GTP_AUTO_UPDATE
#define GTP_AUTO_UPDATE_CFG   0    // auto update config by .cfg file, function together with GTP_AUTO_UPDATE

#define GTP_COMPATIBLE_MODE   0    // compatible with GT9XXF

#define GTP_CREATE_WR_NODE    1
#define GTP_ESD_PROTECT       0    // esd protection with a cycle of 2 seconds
#define GTP_WITH_PEN          0
#define GTP_PEN_HAVE_BUTTON   0    // active pen has buttons, function together with GTP_WITH_PEN

#define GTP_GESTURE_WAKEUP    0    // gesture wakeup
#define GTP_DBL_CLK_WAKEUP    0    // double-click wakeup, function together with GTP_SLIDE_WAKEUP

#define GTP_DEBUG_ON          0
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0

#if GTP_COMPATIBLE_MODE
typedef enum
{
    CHIP_TYPE_GT9  = 0,
    CHIP_TYPE_GT9F = 1,
} CHIP_TYPE_T;
#endif
#define GOODIX_MAX_CFG_GROUP	6
#define GTP_FW_NAME_MAXSIZE	50
struct goodix_ts_platform_data {
	int irq_gpio;
	u32 irq_gpio_flags;
	int reset_gpio;
	u32 reset_gpio_flags;
	const char *product_id;
	const char *fw_name;
	u32  panel_minx;
	u32  panel_miny;
	u32  panel_maxx;
	u32  panel_maxy;
	int  abs_x_min;
	int  abs_y_min;
	int  abs_x_max;
    int  abs_y_max;
	int  max_touch_num;
    int  int_trigger_type;
	bool no_force_update;
	bool i2c_pull_up;
	bool enable_power_off;
	int cfg_count[GOODIX_MAX_CFG_GROUP+1];
	size_t config_data_len[GOODIX_MAX_CFG_GROUP];
	u8 *config_data[GOODIX_MAX_CFG_GROUP];
};
struct goodix_ts_data {
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;
    struct hrtimer timer;
    struct work_struct  work;
	struct goodix_ts_platform_data *pdata; //liujie
    s32 irq_is_disable;
    s32 use_irq;
    u8  green_wake_mode;
    u8  enter_update;
    u8  gtp_is_suspend;
	u8 *config_data;
    u8  fixed_cfg;
    u8  fw_error;
    u8  pnl_init_error;
	u8  cfg_reset;  //liujie add it
	int  gtp_cfg_len;
	bool  gtp_rawdiff_mode; //liujie change it from u8 to bool
    struct regulator *vdd;
	struct regulator *vcc_i2c;
	bool power_on;
/*TYDRV:liujie add these for update up fw&cfg 20140707*/
#if TY_TP_IOCTRL
    bool is_compare_version;
	int  fw_total_len;
	int  fw_burned_len;
	char *fw_buf;
#endif
#if GTP_WITH_PEN
    struct input_dev *pen_dev;
#endif

#if GTP_ESD_PROTECT
    spinlock_t esd_lock;
    u8  esd_running;
    s32 clk_tick_cnt;
#endif

#if GTP_COMPATIBLE_MODE
    u16 bak_ref_len;
    s32 ref_chk_fs_times;
    s32 clk_chk_fs_times;
    CHIP_TYPE_T chip_type;
    u8 rqst_processing;
    u8 is_950;
#endif
	struct mutex lock;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
    
};

extern u16 show_len;
extern u16 total_len;


#define TRUlY_VENDOR_ID_NO		0x5a
#define TRUlY_VENDOR_ID_STR		"TRULY_TP"
#define TRUlY_VENDOR_ID_NO_2		0xda
#define TRUlY_VENDOR_ID_STR_2		"TRULY_TP-2"
#define BYD_VENDOR_ID_NO		0x59
#define BYD_VENDOR_ID_STR		"BYD_TP"
#define O_FILM_VENDOR_ID_NO		0x51
#define O_FILM_VENDOR_ID_STR		"O-FILM_TP"
#define O_FILM_VENDOR_ID_NO_2		0xd1
#define O_FILM_VENDOR_ID_STR_2		"O-FILM_TP-2"
#define BM_VENDOR_ID_NO		0x5d
#define BM_VENDOR_ID_STR		"BM_TP"
#define LCE_VENDOR_ID_NO		0x87
#define LCE_VENDOR_ID_STR		"LCE_TP"
#define V_INTERFACE__VENDOR_ID_STR	" V-INTERFACE_TP"
//*************************** PART2:TODO define **********************************

// STEP_2(REQUIRED): Customize your I/O ports & I/O operations
#define GTP_RST_PORT    16
#define GTP_INT_PORT    17
#define GTP_INT_IRQ     gpio_to_irq(GTP_INT_PORT)
//#define GTP_INT_CFG     S3C_GPIO_SFN(0xF)

#define GTP_GPIO_AS_INPUT(pin)          do{\
                                            gpio_direction_input(pin);\
                                        }while(0)
#define GTP_GPIO_AS_INT(pin)            do{\
                                            GTP_GPIO_AS_INPUT(pin);\
                                        }while(0)
#define GTP_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
#define GTP_GPIO_OUTPUT(pin,level)      gpio_direction_output(pin,level)
#define GTP_GPIO_REQUEST(pin, label)    gpio_request(pin, label)
#define GTP_GPIO_FREE(pin)              gpio_free(pin)
//#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}

// STEP_3(optional): Specify your special config info if needed
#if GTP_CUSTOM_CFG
#if defined(TYQ_TP_THIRD_MENU_540x960)
#define GTP_MAX_HEIGHT	 960
#define GTP_MAX_WIDTH	  540

#elif defined (TYQ_TP_THIRD_MENU_480x800)	
#define GTP_MAX_HEIGHT	 800
 #define GTP_MAX_WIDTH	  480

#elif defined (TYQ_TP_THIRD_MENU_480x854)	
#define GTP_MAX_HEIGHT	 854
 #define GTP_MAX_WIDTH	  480

#elif defined(TYQ_TP_THIRD_MENU_720x1280)
#define GTP_MAX_HEIGHT	 1280
 #define GTP_MAX_WIDTH	  720

#elif defined(TYQ_TP_THIRD_MENU_1080x1920)
#define GTP_MAX_HEIGHT	 1920
 #define GTP_MAX_WIDTH	  1080

#else
  #define GTP_MAX_HEIGHT   1280
  #define GTP_MAX_WIDTH    720
#endif
  #define GTP_INT_TRIGGER  0            // 0: Rising 1: Falling
#else
  #define GTP_MAX_HEIGHT   4096
  #define GTP_MAX_WIDTH    4096
  #define GTP_INT_TRIGGER  1
#endif
#define GTP_MAX_TOUCH         5

// STEP_4(optional): If keys are available and reported as keys, config your key info here                             
#if GTP_HAVE_TOUCH_KEY
    #define GTP_KEY_TAB  {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#endif

//***************************PART3:OTHER define*********************************
#define GTP_DRIVER_VERSION          "V2.2<2014/01/14>"
#define GTP_I2C_NAME                "Goodix-TS"
#define GT91XX_CONFIG_PROC_FILE     "gt9xx_config"
#define GTP_POLL_TIME         10    
#define GTP_ADDR_LENGTH       2
#define GTP_CONFIG_MIN_LENGTH 186
#define GTP_CONFIG_MAX_LENGTH 240
#define FAIL                  0
#define SUCCESS               1
#define SWITCH_OFF            0
#define SWITCH_ON             1

//******************** For GT9XXF Start **********************//
#define GTP_REG_BAK_REF                 0x99D0
#define GTP_REG_MAIN_CLK                0x8020
#define GTP_REG_CHIP_TYPE               0x8000
#define GTP_REG_HAVE_KEY                0x804E
#define GTP_REG_MATRIX_DRVNUM           0x8069     
#define GTP_REG_MATRIX_SENNUM           0x806A

#define GTP_FL_FW_BURN              0x00
#define GTP_FL_ESD_RECOVERY         0x01
#define GTP_FL_READ_REPAIR          0x02

#define GTP_BAK_REF_SEND                0
#define GTP_BAK_REF_STORE               1
#define CFG_LOC_DRVA_NUM                29
#define CFG_LOC_DRVB_NUM                30
#define CFG_LOC_SENS_NUM                31

#define GTP_CHK_FW_MAX                  40
#define GTP_CHK_FS_MNT_MAX              300
#define GTP_BAK_REF_PATH                "/data/gtp_ref.bin"
#define GTP_MAIN_CLK_PATH               "/data/gtp_clk.bin"
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

//******************** For GT9XXF End **********************//
// Registers define
#define GTP_READ_COOR_ADDR    0x814E
#define GTP_REG_SLEEP         0x8040
#define GTP_REG_SENSOR_ID     0x814A
#define GTP_REG_CONFIG_DATA   0x8047
#define GTP_REG_VERSION       0x8140

#define RESOLUTION_LOC        3
#define TRIGGER_LOC           8

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))
// Log define
#define GTP_INFO(fmt,arg...)           printk("<<-GTP-INFO->> "fmt"\n",##arg)
#define GTP_ERROR(fmt,arg...)          printk("<<-GTP-ERROR->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                         if(GTP_DEBUG_ON)\
                                         printk("<<-GTP-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)
#define GTP_DEBUG_ARRAY(array, num)    do{\
                                         s32 i;\
                                         u8* a = array;\
                                         if(GTP_DEBUG_ARRAY_ON)\
                                         {\
                                            printk("<<-GTP-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printk("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printk("\n");\
                                                }\
                                            }\
                                            printk("\n");\
                                        }\
                                       }while(0)
#define GTP_DEBUG_FUNC()               do{\
                                         if(GTP_DEBUG_FUNC_ON)\
                                         printk("<<-GTP-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)
#define GTP_SWAP(x, y)                 do{\
                                         typeof(x) z = x;\
                                         x = y;\
                                         y = z;\
                                       }while (0)

//*****************************End of Part III********************************

#endif /* _GOODIX_GT9XX_H_ */
