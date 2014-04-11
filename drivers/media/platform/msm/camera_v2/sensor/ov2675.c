/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "ov2675.h"

#define CONFIG_MSMB_CAMERA_DEBUG

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define OV2675_SENSOR_NAME "ov2675"
DEFINE_MSM_MUTEX(ov2675_mut);

static struct msm_sensor_ctrl_t ov2675_s_ctrl;
static unsigned int SAT_U = 0x40; /* DEFAULT SATURATION VALUES*/
static unsigned int SAT_V = 0x40; /* DEFAULT SATURATION VALUES*/
#define ov2675_MASTER_CLK_RATE             24000000

static int cur_effect = MSM_CAMERA_EFFECT_MODE_OFF ;

static int cur_exposure = -1;
static int cur_wb = -1 ;
//static int cur_brightness = -1 ;
static int cur_saturation = -1 ;
static int cur_contrast = -1;
static int cur_ISO = -1;
static int cur_sharpenness = -1 ;
static int cur_res = -1 ;

static unsigned int ov2675_preview_shutter;
static unsigned int ov2675_preview_gain16;
static unsigned short ov2675_preview_binning;
static unsigned int ov2675_preview_sysclk;
static unsigned int ov2675_preview_HTS;
static uint16_t ov2675_preview_R_gain;
static uint16_t ov2675_preview_G_gain;
static uint16_t ov2675_preview_B_gain;
enum CAM_RES_MODE{
     RES_SNAPSHOT=0,
     RES_PREVIEW,
     RES_ZSL,
};

static struct msm_sensor_power_setting ov2675_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 50,
	},

	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
	},

	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 5,
	},
};

static struct msm_camera_i2c_reg_conf ov2675_start_settings[] = {
	{0x309e, 0x08},
};

static struct msm_camera_i2c_reg_conf ov2675_stop_settings[] = {
	{0x309e, 0x18},
};

static struct msm_camera_i2c_reg_conf ov2675_recommend_settings[] = {

    {0x3012, 0x80},
    {0x308c, 0x80},
    {0x308d, 0x0e},
    {0x360b, 0x00},
    {0x30b0, 0xff},
    {0x30b1, 0xff},
    {0x30b2, 0x04},
    {0x300e, 0x34},
    {0x300f, 0xa6},
    {0x3010, 0x81},
    {0x3082, 0x01},
    {0x30f4, 0x01},
    {0x3090, 0x43},
    {0x3091, 0xc0},
    {0x30ac, 0x42},
    {0x30d1, 0x08},
    {0x30a8, 0x54},
    {0x3015, 0x52},
    {0x3093, 0x00},
    {0x307e, 0xe5},
    {0x3079, 0x00},
    {0x30aa, 0x52},
    {0x3017, 0x40},
    {0x30f3, 0x83},
    {0x306a, 0x0c},
    {0x306d, 0x00},
    {0x336a, 0x3c},
    {0x3076, 0x6a},
    {0x30d9, 0x95},
    {0x3016, 0x52},
    {0x3601, 0x30},
    {0x304e, 0x88},
    {0x30f1, 0x82},
    {0x306f, 0x14},
    {0x302a, 0x02},
    {0x302b, 0x6a},
    {0x3012, 0x10},
    {0x3011, 0x01},
    {0x30af, 0x00},
    {0x3048, 0x1f},
    {0x3049, 0x4e},
    {0x304a, 0x20},
    {0x304f, 0x20},
    {0x304b, 0x02},
    {0x304c, 0x00},
    {0x304d, 0x02},
    {0x304f, 0x20},
    {0x30a3, 0x10},
    {0x3013, 0xe7},
    {0x3014, 0x84},
    {0x302c, 0x00},
    {0x302d, 0x00},
    {0x302e, 0x00},
    {0x3071, 0x00},
    {0x3070, 0x5d},
    {0x3073, 0x00},
    {0x3072, 0x4d},
    {0x301c, 0x07},
    {0x301d, 0x08},
    {0x304d, 0x42},
    {0x304a, 0x40},
    {0x304f, 0x40},
    {0x3095, 0x07},
    {0x3096, 0x16},
    {0x3097, 0x1d},
    {0x3020, 0x01},
    {0x3021, 0x18},
    {0x3022, 0x00},
    {0x3023, 0x06},
    {0x3024, 0x06},
    {0x3025, 0x58},
    {0x3026, 0x02},
    {0x3027, 0x61},
    {0x3088, 0x03},
    {0x3089, 0x20},
    {0x308a, 0x02},
    {0x308b, 0x58},
    {0x3316, 0x64},
    {0x3317, 0x25},
    {0x3318, 0x80},
    {0x3319, 0x08},
    {0x331a, 0x28},
    {0x331b, 0x1e},
    {0x331c, 0x00},
    {0x331d, 0x38},
    {0x3100, 0x00},
    {0x307c, 0x10},
    //awb
    {0x3320, 0xf8},
    {0x3321, 0x11},
    {0x3322, 0x92},
    {0x3323, 0x01},
    {0x3324, 0x97},
    {0x3325, 0x02},
    {0x3326, 0xff},
    {0x3327, 0x14},
    {0x3328, 0x10},
    {0x3329, 0x12},
    {0x332a, 0x58},
    {0x332b, 0x57},
    {0x332c, 0xac},
    {0x332d, 0xb7},
    {0x332e, 0x36},
    {0x332f, 0x31},
    {0x3330, 0x4d},
    {0x3331, 0x42},
    {0x3332, 0xff},
    {0x3333, 0x00},
    {0x3334, 0xf0},
    {0x3335, 0xf0},
    {0x3336, 0xf0},
    {0x3337, 0x40},
    {0x3338, 0x40},
    {0x3339, 0x40},
    {0x333a, 0x00},
    {0x333b, 0x00},
    //cmx
    {0x3380, 0x26},
    {0x3381, 0x52},
    {0x3382, 0x09},
    {0x3383, 0x28},
    {0x3384, 0xa4},
    {0x3385, 0xcc},
    {0x3386, 0xab},
    {0x3387, 0xa0},
    {0x3388, 0x0b},
    {0x3389, 0x98},
    {0x338a, 0x01},
    //gamma
    {0x3340, 0x0c},
    {0x3341, 0x18},
    {0x3342, 0x30},
    {0x3343, 0x3d},
    {0x3344, 0x4b},
    {0x3345, 0x59},
    {0x3346, 0x67},
    {0x3347, 0x71},
    {0x3348, 0x7d},
    {0x3349, 0x8e},
    {0x334a, 0x9b},
    {0x334b, 0xa6},
    {0x334c, 0xb9},
    {0x334d, 0xc6},
    {0x334e, 0xd9},
    {0x334f, 0x34},
    //lenc
    //r
    {0x3350, 0x30},
    {0x3351, 0x24},
    {0x3352, 0xc8},
    {0x3353, 0x2d},
    {0x3354, 0x00},
    {0x3355, 0x85},
    //g
    {0x3356, 0x32},
    {0x3357, 0x24},
    {0x3358, 0x00},
    {0x3359, 0x26},
    {0x335a, 0x00},
    {0x335b, 0x85},
    //b
    {0x335c, 0x30},
    {0x335d, 0x24},
    {0x335e, 0x84},
    {0x335f, 0x24},
    {0x3360, 0x00},
    {0x3361, 0x85},
    {0x3362, 0x80},
    {0x3363, 0x70},
    {0x3364, 0x7f},
    {0x3365, 0x00},
    {0x3366, 0x00},
    {0x3301, 0xff},
    //{0x338b, 0x12},
    {0x338b, 0x28}, //0x12 neil modify for low saturation
    {0x338c, 0x10},
    {0x338d, 0x40},
    //sharpness&DNS
    {0x3370, 0xd0},
    {0x3371, 0x00},
    {0x3372, 0x00},
    {0x3374, 0x10},
    {0x3375, 0x10},
    {0x3376, 0x0a},
    {0x3377, 0x01},
    {0x3378, 0x04},
    {0x3379, 0x50},
    {0x3069, 0x80},
    {0x307c, 0x10},
    {0x3087, 0x02},
    {0x3090, 0x03},
    {0x30a8, 0x54},
    {0x30aa, 0x82},
    {0x30a3, 0x91},
    {0x30a1, 0x41},
    {0x3300, 0xfc},
    {0x3302, 0x11},
    {0x3400, 0x00},
    {0x3606, 0x20},
    {0x3601, 0x30},
    {0x300e, 0x34},
    {0x30f3, 0x83},
    {0x304e, 0x88},
    {0x3391, 0x06},
    {0x3018, 0x88},
    {0x3019, 0x78},
    {0x301a, 0xd4},
    {0x3640, 0x30},
    {0x3645, 0xf0},
    {0x3643, 0x21},
    {0x3645, 0x0a},
    {0x3655, 0x04},
    {0x3306,0x10},
#ifdef TYQ_FCAM_OV2675_FLIP
    {0x307c,0x11},
    {0x3090,0x0b},
#else
    {0x307c,0x10},
    {0x3090,0x03},
#endif
};

#if 1
static struct msm_camera_i2c_reg_conf ov2675_VGA_settings[] = {
    {0x3011, 0x00},
    {0x3012, 0x10},
    {0x3015, 0x52},
    {0x3014, 0x84},
    {0x3016, 0x82},
    {0x3023, 0x06},
    {0x3026, 0x02},
    {0x3027, 0x5e},
    {0x302a, 0x02},
    {0x302b, 0x6a},
    {0x330c, 0x00},
    {0x3301, 0xff},
    {0x3069, 0x80},
    {0x306f, 0x14},
    {0x3088, 0x03},
    {0x3089, 0x20},
    {0x308a, 0x02},
    {0x308b, 0x58},
    {0x308e, 0x00},
    {0x30a1, 0x41},
    {0x30a3, 0x80},
    {0x30d9, 0x95},
    {0x3302, 0x11},
    {0x3317, 0x25},
    {0x3318, 0x80},
    {0x3319, 0x08},
    {0x331d, 0x38},
    {0x3373, 0x40},//wangda mondify
    //{0x3376, 0x15},
    {0x3376, 0x0a}, //0x15 neil modify strong sharpness in video mode
    {0x3362, 0x90},
    {0x3302, 0x11},
    {0x3088, 0x03},
    {0x3089, 0x20},
    {0x308a, 0x02},
    {0x308b, 0x58},
    {0x331a, 0x28},
    {0x331b, 0x1e},
    {0x331c, 0x00},
    {0x3302, 0x11},
    {0x363b, 0x01},
    {0x309e, 0x08},
    {0x3606, 0x00},
    {0x3630, 0x31},
    {0x304e, 0x04},
    {0x363b, 0x01},
    {0x309e, 0x08},
    {0x3606, 0x00},
    {0x3084, 0x01},
    {0x3634, 0x26},
    {0x3071, 0x00},
    {0x3070, 0x5d},
    {0x3073, 0x00},
    {0x3072, 0x4d},
    {0x301c, 0x05},
    {0x301d, 0x07},
    {0x3000, 0x15},
    {0x3002, 0x02},
    {0x3003, 0x28},
    {0x3640, 0x30},
    {0x3645, 0xf0},
    {0x3015, 0x22},
    {0x3640, 0x53},
    {0x3070,0xba},// ; B50
    {0x3072,0x9a},// ; B60
    {0x301c,0x02},// ; max step 50
    {0x301d,0x03},// ; max step 60
    {0x3011, 0x00},
    {0x3390, 0x41},
    {0x339a, 0x10},
};

//   UXGA
static struct msm_camera_i2c_reg_conf ov2675_UXGA_settings[] = {
    {0x3014, 0x84},
    {0x3012, 0x00},
    {0x301c, 0x0f},
    {0x301d, 0x0f},
    {0x3020, 0x01},
    {0x3021, 0x18},
    {0x3022, 0x00},
    {0x3023, 0x0A},
    {0x3024, 0x06},
    {0x3025, 0x58},
    {0x3026, 0x04},
    {0x3027, 0xbc},
    {0x302a, 0x04},
    {0x302b, 0xd4},
    {0x302d, 0x00},
    {0x302e, 0x00},
    {0x306f, 0x54},
    {0x3362, 0x80},
    {0x3070, 0x5d},
    {0x3072, 0x5d},
    {0x3088, 0x06},
    {0x3089, 0x40},
    {0x308a, 0x04},
    {0x308b, 0xb0},
    {0x3316, 0x64},
    {0x3317, 0x4b},
    {0x3318, 0x00},
    {0x3319, 0x4c},
    {0x331A, 0x64},
    {0x331B, 0x4B},
    {0x331C, 0x00},
    {0x331D, 0x4C},
    {0x3373, 0x40},
    //{0x3376, 0x02},
    {0x3376, 0x06}, //0x02 neil modify for capture low sharpness issue
    {0x3377, 0x01},
    {0x3378, 0x04},
    {0x3379, 0x50},
    {0x3302, 0x01},
    {0x3640, 0x10},
    {0x3645, 0xc0},
    {0x309e, 0x08},
    {0x3606, 0x00},
    {0x3084, 0x01},
    {0x3634, 0x26},
    {0x3015, 0x52},
    {0x3645, 0xf0},
    {0x300e, 0x34},
    {0x300f, 0xa6},
    {0x3010, 0x81},
    {0x3011, 0x01},
    {0x3640, 0x53},
};
/*TYRD wang_gj add for sending yuv sensor stream type to kernel begin*/
#ifdef TYQ_YUV_SENSOR_STREAM_TYPE_SUPPORT
static struct msm_camera_i2c_reg_conf ov2675_zsl_settings[] = {
    {0x3014, 0x84},
    {0x3012, 0x00},
    {0x301c, 0x0a},
    {0x301d, 0x0a},
    {0x3020, 0x01},
    {0x3021, 0x18},
    {0x3022, 0x00},
    {0x3023, 0x0A},
    {0x3024, 0x06},
    {0x3025, 0x58},
    {0x3026, 0x04},
    {0x3027, 0xbc},
    {0x302a, 0x04},
    {0x302b, 0xd4},
    {0x302d, 0x00},
    {0x302e, 0x00},
    {0x306f, 0x54},
    {0x3362, 0x80},
    {0x3070, 0x7b},
    {0x3072, 0x71},
    {0x3088, 0x06},
    {0x3089, 0x40},
    {0x308a, 0x04},
    {0x308b, 0xb0},
    {0x3316, 0x64},
    {0x3317, 0x4b},
    {0x3318, 0x00},
    {0x3319, 0x4c},
    {0x331A, 0x64},
    {0x331B, 0x4B},
    {0x331C, 0x00},
    {0x331D, 0x4C},
    {0x3373, 0x40},
    //{0x3376, 0x02},
    {0x3376, 0x06}, //0x02 neil modify for low sharpness
    {0x3377, 0x00},
    {0x3302, 0x01},
    {0x3640, 0x10},
    {0x3645, 0xc0},
    {0x309e, 0x08},
    {0x3606, 0x00},
    {0x3084, 0x01},
    {0x3634, 0x26},
    {0x3015, 0x52},
    {0x300e, 0x34},
    {0x300f, 0xa6},
    {0x3010, 0x81},
    {0x3011, 0x01},
    {0x3640, 0x30},
    {0x3645, 0xf0},
    {0x300e, 0x34},
    {0x300f, 0xa6},
    {0x3010, 0x81},
    {0x3011, 0x01},
  //  {0x3011, 0x00},
    {0x3640, 0x53},
    {0x3645, 0x96},
    {0x3013, 0xf7},
    {0x3070,0xba},// ; B50
    {0x3072,0x9a},// ; B60
    {0x301c,0x02},// ; max step 50
    {0x301d,0x03},// ; max step 60
};
/*TYRD wang_gj add for sending yuv sensor stream type to kernel end*/
#endif 

#endif


static struct v4l2_subdev_info ov2675_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order  = 0,
	},
};

static const struct i2c_device_id ov2675_i2c_id[] = {
	{OV2675_SENSOR_NAME, (kernel_ulong_t)&ov2675_s_ctrl},
	{ }
};

static int32_t msm_ov2675_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	CDBG("%s, E.", __func__);

	return msm_sensor_i2c_probe(client, id, &ov2675_s_ctrl);
}

static struct i2c_driver ov2675_i2c_driver = {
	.id_table = ov2675_i2c_id,
	.probe  = msm_ov2675_i2c_probe,
	.driver = {
		.name = OV2675_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov2675_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov2675_dt_match[] = {
	{.compatible = "qcom,ov2675", .data = &ov2675_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov2675_dt_match);

static struct platform_driver ov2675_platform_driver = {
	.driver = {
		.name = "qcom,ov2675",
		.owner = THIS_MODULE,
		.of_match_table = ov2675_dt_match,
	},
};

static int32_t ov2675_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	CDBG("%s, E.", __func__);
	match = of_match_device(ov2675_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov2675_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov2675_platform_driver,
		ov2675_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov2675_i2c_driver);
}

static void __exit ov2675_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov2675_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov2675_s_ctrl);
		platform_driver_unregister(&ov2675_platform_driver);
	} else
		i2c_del_driver(&ov2675_i2c_driver);
	return;
}

static int ov2675_read(struct msm_sensor_ctrl_t *s_ctrl, 
						unsigned short add, unsigned short *val)
{
	return s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
							 s_ctrl->sensor_i2c_client,
							 add,
							 val, MSM_CAMERA_I2C_BYTE_DATA);
}

static int ov2675_write(struct msm_sensor_ctrl_t *s_ctrl, 
						unsigned short add, unsigned short val)
{
	return s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
							 s_ctrl->sensor_i2c_client,
							 add,
							 val, MSM_CAMERA_I2C_BYTE_DATA);
}
static int ov2675_no_effect(struct msm_sensor_ctrl_t *s_ctrl)
{
	ov2675_write(s_ctrl, 0x3391, 0x06);
	ov2675_write(s_ctrl, 0x3390, 0x41);
	return 0;
}
static int ov2675_group_hold_on(struct msm_sensor_ctrl_t *s_ctrl)
{
	//ov2675_write(s_ctrl, 0x308c, 0x88);
	return 0;
}
static int ov2675_group_hold_off(struct msm_sensor_ctrl_t *s_ctrl)
{
	//ov2675_write(s_ctrl, 0x308c, 0x80);
	//ov2675_write(s_ctrl, 0x30ff, 0xff);
	return 0;
}
#if 1
static int ov2675_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int contrast)
{
	unsigned short temp;
	printk("[OV2675]set contrast %d\n", contrast);
	
	ov2675_read(s_ctrl, 0x3391, &temp);
	switch (contrast) {
	case CONTRAST_0:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x32);
		ov2675_write(s_ctrl, 0x3399, 0x32);
		break;
	case CONTRAST_1:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x30);
		ov2675_write(s_ctrl, 0x3399, 0x30);
		break;
	case CONTRAST_2:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x2e);
		ov2675_write(s_ctrl, 0x3399, 0x2e);
		break;
	case CONTRAST_3:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x2c);
		ov2675_write(s_ctrl, 0x3399, 0x2c);
		break;
	case CONTRAST_4:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x28);
		ov2675_write(s_ctrl, 0x3399, 0x28);
		break;
	case CONTRAST_5:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x24);
		ov2675_write(s_ctrl, 0x3399, 0x24);
		break;
	case CONTRAST_6:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x20);
		ov2675_write(s_ctrl, 0x3399, 0x20);
		break;
	case CONTRAST_7:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x1e);
		ov2675_write(s_ctrl, 0x3399, 0x1e);
		break;
	case CONTRAST_8:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x1c);
		ov2675_write(s_ctrl, 0x3399, 0x1c);
		break;
	case CONTRAST_9:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x1a);
		ov2675_write(s_ctrl, 0x3399, 0x1a);
		break;
	case CONTRAST_10:
		ov2675_write(s_ctrl, 0x3390, 0x45);
		ov2675_write(s_ctrl, 0x3398, 0x18);
		ov2675_write(s_ctrl, 0x3399, 0x18);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif
#if 0
static int ov2675_set_brightness(struct msm_sensor_ctrl_t *s_ctrl, int brightness)
{
	unsigned short temp;
	printk("[OV2675]set brightness %d\n", brightness);
	ov2675_read(s_ctrl, 0x3391, &temp);
	switch (brightness) {
	case BRIGHTNESS_0:
		ov2675_write(s_ctrl, 0x3390, 0x49);
		ov2675_write(s_ctrl, 0x339a, 0x40);
		break;
	case BRIGHTNESS_1:
		ov2675_write(s_ctrl, 0x3390, 0x49);
		ov2675_write(s_ctrl, 0x339a, 0x30);		
		break;
	case BRIGHTNESS_2:
		ov2675_write(s_ctrl, 0x3390, 0x49);
		ov2675_write(s_ctrl, 0x339a, 0x20);
		break;
	case BRIGHTNESS_3:
		ov2675_write(s_ctrl, 0x3390, 0x49);
		ov2675_write(s_ctrl, 0x339a, 0x10);
		break;
	case BRIGHTNESS_4:
		ov2675_write(s_ctrl, 0x3390, 0x41);
		ov2675_write(s_ctrl, 0x339a, 0x00);
		break;
	case BRIGHTNESS_5:
		ov2675_write(s_ctrl, 0x3390, 0x41);
		ov2675_write(s_ctrl, 0x339a, 0x00);
		break;
	case BRIGHTNESS_6:
		ov2675_write(s_ctrl, 0x3390, 0x41);
		ov2675_write(s_ctrl, 0x339a, 0x20);
		break;
	case BRIGHTNESS_7:
		ov2675_write(s_ctrl, 0x3390, 0x41);
		ov2675_write(s_ctrl, 0x339a, 0x30);
		break;
	case BRIGHTNESS_8:
		ov2675_write(s_ctrl, 0x3390, 0x41);
		ov2675_write(s_ctrl, 0x339a, 0x40);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#endif
static int ov2675_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int contrast)
{
	unsigned short temp;

	printk("[OV2675]set contrast %d\n", contrast);
	contrast = contrast / 6;
	ov2675_read(s_ctrl, 0x3391, &temp);
	switch (contrast) {
	case CONTRAST_6:
		ov2675_write(s_ctrl, 0x3376,0x16);
		ov2675_write(s_ctrl, 0x3377,0x01);
		ov2675_write(s_ctrl, 0x3378,0x04);
		ov2675_write(s_ctrl, 0x3379,0x50);
		break;	
	case CONTRAST_5:
		ov2675_write(s_ctrl, 0x3376,0x12);
		ov2675_write(s_ctrl, 0x3377,0x01);
		ov2675_write(s_ctrl, 0x3378,0x04);
		ov2675_write(s_ctrl, 0x3379,0x50);
		break;	
	case CONTRAST_4:
		ov2675_write(s_ctrl, 0x3376,0x0d);
		ov2675_write(s_ctrl, 0x3377,0x01);
		ov2675_write(s_ctrl, 0x3378,0x04);
		ov2675_write(s_ctrl, 0x3379,0x50);
		break;
	case CONTRAST_3:
		ov2675_write(s_ctrl, 0x3376,0x08);
		ov2675_write(s_ctrl, 0x3377,0x01);
		ov2675_write(s_ctrl, 0x3378,0x04);
		ov2675_write(s_ctrl, 0x3379,0x50);
		break;	
	case CONTRAST_2:
		ov2675_write(s_ctrl, 0x3376,0x06);
		ov2675_write(s_ctrl, 0x3377,0x01);
		ov2675_write(s_ctrl, 0x3378,0x04);
		ov2675_write(s_ctrl, 0x3379,0x50);
		break;
	case CONTRAST_1:
		ov2675_write(s_ctrl, 0x3376,0x03);
		ov2675_write(s_ctrl, 0x3377,0x00);
		ov2675_write(s_ctrl, 0x3378,0x04);
		ov2675_write(s_ctrl, 0x3379,0x50);
		break;
	case CONTRAST_0:
		ov2675_write(s_ctrl, 0x3376, 0x01);
		ov2675_write(s_ctrl, 0x3377, 0x00);
		ov2675_write(s_ctrl, 0x3378, 0x04);
		ov2675_write(s_ctrl, 0x3379, 0x50);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ov2675_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int effect)
{
	unsigned short temp;
	printk("[OV2675]set effect %d\n", effect);
	ov2675_read(s_ctrl, 0x3391, &temp);
	temp &= ~0x78;
	switch(effect){
		case MSM_CAMERA_EFFECT_MODE_OFF:
			ov2675_write(s_ctrl, 0x3391,0x06);
			ov2675_write(s_ctrl, 0x3396,0x80);
			ov2675_write(s_ctrl, 0x3397,0x80);
			break;
		case MSM_CAMERA_EFFECT_MODE_MONO:
			ov2675_write(s_ctrl, 0x3391,0x26);
			ov2675_write(s_ctrl, 0x3396,0x80);
			ov2675_write(s_ctrl, 0x3397,0x80);
			break;
		case MSM_CAMERA_EFFECT_MODE_NEGATIVE:
			ov2675_write(s_ctrl, 0x3391,0x4a);
			ov2675_write(s_ctrl, 0x3396,0x44);
			ov2675_write(s_ctrl, 0x3397,0x44);
			break;
		case MSM_CAMERA_EFFECT_MODE_SOLARIZE:
			ov2675_write(s_ctrl, 0x3391,0x06);
			ov2675_write(s_ctrl, 0x3396,0x41);
			ov2675_write(s_ctrl, 0x3397,0x41);
			break;
		case MSM_CAMERA_EFFECT_MODE_SEPIA:
			ov2675_write(s_ctrl, 0x3391, 0x1e);
			ov2675_write(s_ctrl, 0x3396, 0x40);
			ov2675_write(s_ctrl, 0x3397, 0xa6);
			break;
		case MSM_CAMERA_EFFECT_MODE_POSTERIZE:
			ov2675_write(s_ctrl, 0x3391, 0x1e);
			ov2675_write(s_ctrl, 0x3396, 0x41);
			ov2675_write(s_ctrl, 0x3397, 0x41);
			break;
		case MSM_CAMERA_EFFECT_MODE_WHITEBOARD:
			ov2675_write(s_ctrl, 0x3391, 0x2a);
			ov2675_write(s_ctrl, 0x3396, 0x24);
			ov2675_write(s_ctrl, 0x3397, 0x24);
			break;
		case MSM_CAMERA_EFFECT_MODE_BLACKBOARD:
			ov2675_write(s_ctrl, 0x3391, 0x2a);
			ov2675_write(s_ctrl, 0x3396, 0x24);
			ov2675_write(s_ctrl, 0x3397, 0x24);
			break;
		case MSM_CAMERA_EFFECT_MODE_AQUA:
			ov2675_write(s_ctrl, 0x3391,0x06);
			ov2675_write(s_ctrl, 0x3396,0x41);
			ov2675_write(s_ctrl, 0x3397,0x41);
			break;
		default:
			return -EINVAL;
	}
	return 0;
}
static int ov2675_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, int saturation)
{
	unsigned short temp;	
	printk("[OV2675]set saturation %d\n", saturation);
	
	ov2675_read(s_ctrl, 0x3391, &temp);
	switch (saturation) {
	case SATURATION_0:
		ov2675_write(s_ctrl, 0x3394, 0x10);
		ov2675_write(s_ctrl, 0x3395, 0x10);
		break;
	case SATURATION_1:
		ov2675_write(s_ctrl, 0x3394, 0x18);
		ov2675_write(s_ctrl, 0x3395, 0x18);
		break;
	case SATURATION_2:
		ov2675_write(s_ctrl, 0x3394, 0x20);
		ov2675_write(s_ctrl, 0x3395, 0x20);
		break;
	case SATURATION_3:
		ov2675_write(s_ctrl, 0x3394, 0x28);
		ov2675_write(s_ctrl, 0x3395, 0x28);
		break;	
	case SATURATION_4:
		ov2675_write(s_ctrl, 0x3394, 0x30);
		ov2675_write(s_ctrl, 0x3395, 0x30);
		break;
	case SATURATION_5:
		ov2675_write(s_ctrl, 0x3394, 0x38);
		ov2675_write(s_ctrl, 0x3395, 0x38);
		break;
	case SATURATION_6:
		ov2675_write(s_ctrl, 0x3394, 0x40);
		ov2675_write(s_ctrl, 0x3395, 0x40);
		break;
	case SATURATION_7:
		ov2675_write(s_ctrl, 0x3394, 0x48);
		ov2675_write(s_ctrl, 0x3395, 0x48);
		break;
	case SATURATION_8:
		ov2675_write(s_ctrl, 0x3394, 0x50);
		ov2675_write(s_ctrl, 0x3395, 0x50);
		break;
	case SATURATION_9:
		ov2675_write(s_ctrl, 0x3394, 0x58);
		ov2675_write(s_ctrl, 0x3395, 0x58);
		break;
	case SATURATION_10:
		ov2675_write(s_ctrl, 0x3394, 0x60);
		ov2675_write(s_ctrl, 0x3395, 0x60);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int ov2675_set_exposure(struct msm_sensor_ctrl_t *s_ctrl, int exposure)
{
	printk("[OV2675]set exposure %d\n", exposure);
	exposure = (exposure + 12) / 6;
	switch (exposure) {
	case EXPOSURE_0:
		ov2675_write(s_ctrl, 0x3018, 0x58);
		ov2675_write(s_ctrl, 0x3019, 0x48);
		ov2675_write(s_ctrl, 0x301a, 0x73);
		break;
	case EXPOSURE_1:
		ov2675_write(s_ctrl, 0x3018, 0x60);
		ov2675_write(s_ctrl, 0x3019, 0x50);
		ov2675_write(s_ctrl, 0x301a, 0x74);
		break;
	case EXPOSURE_2:
		ov2675_write(s_ctrl, 0x3018, 0x68);
		ov2675_write(s_ctrl, 0x3019, 0x58);
		ov2675_write(s_ctrl, 0x301a, 0x84);
		break;
	case EXPOSURE_3:
		ov2675_write(s_ctrl, 0x3018, 0x70);
		ov2675_write(s_ctrl, 0x3019, 0x60);
		ov2675_write(s_ctrl, 0x301a, 0x84);
		break;
	case EXPOSURE_4:
		ov2675_write(s_ctrl, 0x3018, 0x78);
		ov2675_write(s_ctrl, 0x3019, 0x68);
		ov2675_write(s_ctrl, 0x301a, 0x95);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int ov2675_set_white_balance(struct msm_sensor_ctrl_t *s_ctrl, int balance)
{
	unsigned short temp;
	printk("[OV2675]set balance %d\n", balance);
	ov2675_read(s_ctrl, 0x3306, &temp);
	temp &= ~0x02;
	switch(balance){
		case MSM_CAMERA_WB_MODE_AUTO:
			ov2675_write(s_ctrl, 0x3306, 0x10);
			break;
		case MSM_CAMERA_WB_MODE_INCANDESCENT:
			ov2675_write(s_ctrl, 0x3306, 0x12);
			ov2675_write(s_ctrl, 0x3337, 0x40);
			ov2675_write(s_ctrl, 0x3338, 0x40);
			ov2675_write(s_ctrl, 0x3339, 0x5d);
			break;
		case MSM_CAMERA_WB_MODE_FLUORESCENT:
			ov2675_write(s_ctrl, 0x3306, 0x12);
			ov2675_write(s_ctrl, 0x3337, 0x50);
			ov2675_write(s_ctrl, 0x3338, 0x40);
			ov2675_write(s_ctrl, 0x3339, 0x53);
			break;
		case MSM_CAMERA_WB_MODE_WARM_FLUORESCENT:
			break;
		case MSM_CAMERA_WB_MODE_DAYLIGHT:
			ov2675_write(s_ctrl, 0x3306, 0x12);
			ov2675_write(s_ctrl, 0x3337, 0x5f);
			ov2675_write(s_ctrl, 0x3338, 0x40);
			ov2675_write(s_ctrl, 0x3339, 0x44);
			break;
		case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT:
			ov2675_write(s_ctrl, 0x3306, 0x12);
			ov2675_write(s_ctrl, 0x3337, 0x64);
			ov2675_write(s_ctrl, 0x3338, 0x40);
			ov2675_write(s_ctrl, 0x3339, 0x40);
			break;
		case MSM_CAMERA_WB_MODE_TWILIGHT:
			break;
		case MSM_CAMERA_WB_MODE_SHADE:
			break;
		default:
			return -EINVAL;
	}
	return 0;
}
#if 0
static int ov2675_set_flicker(struct msm_sensor_ctrl_t *s_ctrl, int flicker)
{
	unsigned short temp;
	printk("[OV2675]set flicker %d\n", flicker);
	ov2675_read(s_ctrl, 0x3013, &temp);
	
	switch (flicker) {
	case FLICKER_AUTO:
		ov2675_write(s_ctrl, 0x3014, 0x44);
		temp &= 0xdf;
		ov2675_write(s_ctrl, 0x3013, temp);
		break;
	case FLICKER_50Hz:
		ov2675_write(s_ctrl, 0x3014, 0x84);
		temp |= 0x20;
		ov2675_write(s_ctrl, 0x3013, temp);		
		break;
	case FLICKER_60Hz:
		ov2675_write(s_ctrl, 0x3014, 0x04);
		temp |= 0x20;
		ov2675_write(s_ctrl, 0x3013, temp);		
		break;
	case FLICKER_OFF:		
		temp &= 0xdf;
		ov2675_write(s_ctrl, 0x3013, temp);		
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif
static int ov2675_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int bs_mode)
{
	return 0;
}
#if 0
static int ov2675_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int iso)
{
	unsigned short temp;
	printk("[OV2675]set iso %d\n", iso);
	
	ov2675_read(s_ctrl, 0x3391, &temp);
	switch (iso) {
	case ISO_AUTO:
		ov2675_write(s_ctrl, 0x3015, 0x03);
		break;
	case ISO_AUTO_HJR:
		ov2675_write(s_ctrl, 0x3015, 0x03);
		break;
	case ISO_100:
		ov2675_write(s_ctrl, 0x3015, 0x00);
		break;
	case ISO_200:
		ov2675_write(s_ctrl, 0x3015, 0x01);
		break;
	case ISO_400:
		ov2675_write(s_ctrl, 0x3015, 0x02);
		break;
	case ISO_800:
		ov2675_write(s_ctrl, 0x3015, 0x03);
		break;
	case ISO_1600:
		ov2675_write(s_ctrl, 0x3015, 0x04);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif
#if 1 //wang_gj
//sharpness set stop
//func 1
int ov2675_SDE_msm_sensor_s_ctrl_by_enum(
        struct msm_sensor_ctrl_t *s_ctrl,
        enum msm_sensor_cfg_type_t  ctrl_info, int value)
{
    int rc = 0;

    uint16_t isp_ctrl_addr;
    uint16_t isp_ctrl = 0;
    uint16_t isp_ctrl_mask;


    if(ctrl_info == CFG_SET_SATURATION) {
        if (value <= SATURATION_6)
            SAT_U = SAT_V = value * 0x10;
    }
    if (cur_effect == MSM_CAMERA_EFFECT_MODE_OFF) {

        switch(ctrl_info) {
        case CFG_SET_CONTRAST:
            if(value == cur_contrast) {
                return rc ;
            }
            cur_contrast = value ;
            isp_ctrl_addr = 0x3391;
            isp_ctrl_mask = 0x06;
            break;
        case CFG_SET_SATURATION:
            if(value == cur_saturation){
                return rc ;
            }
            cur_saturation = value ;
            isp_ctrl_addr = 0x3391;
            isp_ctrl_mask = 0x06;
            break;
        default:
            return -EINVAL;
        }

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                s_ctrl->sensor_i2c_client,
                isp_ctrl_addr, &isp_ctrl,
                MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("%s: read isp_ctrl_reg failed\n", __func__);
        }

        isp_ctrl = isp_ctrl | isp_ctrl_mask;
        //s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                s_ctrl->sensor_i2c_client,
                isp_ctrl_addr, isp_ctrl,
                MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("%s: write isp_ctrl_reg failed\n", __func__);
        }
		switch(ctrl_info) {
				case CFG_SET_CONTRAST:
					//cur_brightness =value;
					ov2675_set_contrast(s_ctrl,  value);
					break;
				case CFG_SET_SATURATION:
					ov2675_set_saturation(s_ctrl,  value);
					break;
				default:
					return -EINVAL;
				}

       // s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
    }

    CDBG("--CAMERA-- %s ...(End)\n", __func__);

    return rc;
}
//func 2
int ov2675_wb_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
        enum msm_sensor_cfg_type_t  ctrl_info, int value)
{
    int rc = 0;
    uint16_t isp_ctrl_addr = 0x3306;
    uint16_t isp_ctrl = 0;
    uint16_t isp_ctrl_mask = 0x02;

    cur_wb = value ;
    isp_ctrl_addr = 0x3306 ;
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            isp_ctrl_addr, &isp_ctrl,
            MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: read isp_ctrl_reg failed\n", __func__);
    }

    CDBG("%s: isp_ctrl = 0x%x, value=%d\n", __func__, isp_ctrl,value);
    if(value <= MSM_CAMERA_WB_MODE_AUTO) {
        isp_ctrl_mask = 0x02;
        isp_ctrl = isp_ctrl & ~isp_ctrl_mask;
    } else {
        isp_ctrl_mask = 0x02;
        isp_ctrl = isp_ctrl | isp_ctrl_mask;
    }
    ov2675_group_hold_on(s_ctrl);
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                s_ctrl->sensor_i2c_client,
                isp_ctrl_addr, isp_ctrl,
                MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: write isp_ctrl_reg failed\n", __func__);
    }


	ov2675_set_white_balance(s_ctrl,  value);

    ov2675_group_hold_off(s_ctrl);
    if (rc < 0) {
        CDBG("write faield\n");
    }
    CDBG("--CAMERA-- %s ...(End)\n", __func__);
    return rc;
}

//func 3
int ov2675_effect_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
        enum msm_sensor_cfg_type_t  ctrl_info, int value)
{
    int rc = 0;
    if(value == cur_effect) {
        return rc ;
    }

    cur_effect = value ;
    ov2675_group_hold_on(s_ctrl);
    if (cur_effect == MSM_CAMERA_EFFECT_MODE_OFF) {
        rc = ov2675_no_effect(s_ctrl);
        if (rc < 0) {
            CDBG("write faield\n");
        }
        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3394, SAT_U,
            MSM_CAMERA_I2C_BYTE_DATA);
        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3395, SAT_V,
            MSM_CAMERA_I2C_BYTE_DATA);
    } else {
	       ov2675_set_effect( s_ctrl, value);
    }
    ov2675_group_hold_off(s_ctrl);
    CDBG("--CAMERA-- %s ...(End)\n", __func__);
    return rc;
}

//func 4
static unsigned int ov2675_get_sysclk(struct msm_sensor_ctrl_t *s_ctrl)
{
  // calculate sysclk
    unsigned int  XVCLK;
    uint16_t temp1, temp2;
    unsigned int Indiv2x, Bit8Div, FreqDiv2x, PllDiv, SensorDiv, ScaleDiv,DvpDiv, ClkDiv, VCO, sysclk;
    unsigned int Indiv2x_map[] = { 2, 3, 4, 6, 4, 6, 8, 12};
    unsigned int Bit8Div_map[] = { 1, 1, 4, 5};
    unsigned int FreqDiv2x_map[] = { 2, 3, 4, 6};
    unsigned int DvpDiv_map[] = { 1, 2, 8, 16};

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x300e, &temp1,
        MSM_CAMERA_I2C_BYTE_DATA);
    // bit[5:0] PllDiv
    PllDiv = 64 - (temp1 & 0x3f);

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x300f, &temp1,
        MSM_CAMERA_I2C_BYTE_DATA);
    // bit[2:0] Indiv
    temp2 = temp1 & 0x07;
    Indiv2x = Indiv2x_map[temp2];

    // bit[5:4] Bit8Div
    temp2 = (temp1 >> 4) & 0x03;
    Bit8Div = Bit8Div_map[temp2];

    // bit[7:6] FreqDiv
    temp2 = temp1 >> 6;
    FreqDiv2x = FreqDiv2x_map[temp2];

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3010, &temp1,
        MSM_CAMERA_I2C_BYTE_DATA);

    //bit[3:0] ScaleDiv
    temp2 = temp1 & 0x0f;
    if(temp2==0) {
        ScaleDiv = 1;
    } else {
        ScaleDiv = temp2 * 2;
    }

    // bit[4] SensorDiv
    if(temp1 & 0x10) {
        SensorDiv = 2;
    } else {
        SensorDiv = 1;
    }

    // bit[5] LaneDiv
    // bit[7:6] DvpDiv
    temp2 = temp1 >> 6;
    DvpDiv = DvpDiv_map[temp2];
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3011, &temp1,
        MSM_CAMERA_I2C_BYTE_DATA);

    // bit[5:0] ClkDiv
    temp2 = temp1 & 0x3f;
    ClkDiv = temp2 + 1;
    XVCLK = ov2675_MASTER_CLK_RATE/10000;

    VCO = XVCLK * Bit8Div * FreqDiv2x * PllDiv / Indiv2x ;
    sysclk = VCO / Bit8Div / SensorDiv / ClkDiv / 4;

    return sysclk;
}

static unsigned int ov2675_get_HTS(struct msm_sensor_ctrl_t *s_ctrl)
{
  // read HTS from register settings
    unsigned int HTS, extra_HTS;
    uint16_t ret_l,ret_h;
    ret_l = ret_h = 0;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3028, &ret_h,
        MSM_CAMERA_I2C_BYTE_DATA);
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3029, &ret_l,
        MSM_CAMERA_I2C_BYTE_DATA);
    HTS = (ret_h << 8) | (ret_l & 0xff) ;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x302c, &ret_l,
        MSM_CAMERA_I2C_BYTE_DATA);
    extra_HTS = ret_l;

    return HTS + extra_HTS;
}

static unsigned int ov2675_get_VTS(struct msm_sensor_ctrl_t *s_ctrl)
{
  // read VTS from register settings
    unsigned int VTS, extra_VTS;
    uint16_t ret_l,ret_h;
    ret_l = ret_h = 0;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x302a, &ret_h,
        MSM_CAMERA_I2C_BYTE_DATA);
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x302b, &ret_l,
        MSM_CAMERA_I2C_BYTE_DATA);
    VTS = (ret_h << 8) | (ret_l & 0xff) ;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x302d, &ret_h,
        MSM_CAMERA_I2C_BYTE_DATA);
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x302e, &ret_l,
        MSM_CAMERA_I2C_BYTE_DATA);
    extra_VTS = (ret_h << 8) | (ret_l & 0xff) ;

    return VTS + extra_VTS;
}

static int ov2675_set_VTS(struct msm_sensor_ctrl_t *s_ctrl,unsigned int VTS)
{
    // write VTS to registers
    int rc = 0;
    uint16_t temp;
    temp = VTS & 0xff;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x302b, temp,
        MSM_CAMERA_I2C_BYTE_DATA);
    temp = VTS>>8;

    rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x302a, temp,
        MSM_CAMERA_I2C_BYTE_DATA);
    return rc;
}

static int ov2675_set_bandingfilter(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
    unsigned int preview_VTS;
    //int dummyline = 0;
    uint16_t band_step60, max_band60, band_step50, max_band50;
    // read preview PCLK
    ov2675_preview_sysclk = ov2675_get_sysclk(s_ctrl);
    // read preview HTS
    ov2675_preview_HTS = ov2675_get_HTS(s_ctrl);
    // read preview VTS
    preview_VTS = ov2675_get_VTS(s_ctrl);
    // calculate banding filter

    // 60Hz
    band_step60 = ov2675_preview_sysclk * 100/ov2675_preview_HTS * 100/120;
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
        0x3073, (band_step60 >> 8),MSM_CAMERA_I2C_BYTE_DATA);
    rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
        0x3072, (band_step60 & 0xff),MSM_CAMERA_I2C_BYTE_DATA);
    max_band60 = ((preview_VTS-4)/band_step60);
    rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
        0x301d,max_band60-1,MSM_CAMERA_I2C_BYTE_DATA);
    // 50Hz

    band_step50 = ov2675_preview_sysclk * 100/ov2675_preview_HTS;
    rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3071,
        (band_step50 >> 8),MSM_CAMERA_I2C_BYTE_DATA);
    rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3070,
        (band_step50 & 0xff),MSM_CAMERA_I2C_BYTE_DATA);
    max_band50 = ((preview_VTS-4)/band_step50);
    rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x301c,
        max_band50-1,MSM_CAMERA_I2C_BYTE_DATA);

    //dummyline = ov2675_get_dummyline();
    return rc;
}

int ov2675_antibanding_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
        enum msm_sensor_cfg_type_t  ctrl_info, int value)
{
    int rc = 0;

    uint16_t isp_ctrl_addr = 0x3014;
    uint16_t isp_ctrl = 0;

    switch(value){
        case FLICKER_60Hz:
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                    s_ctrl->sensor_i2c_client,
                    isp_ctrl_addr, &isp_ctrl,
                    MSM_CAMERA_I2C_BYTE_DATA);
               if (rc < 0) {
                         pr_err("%s: write isp_ctrl_reg failed\n", __func__);
                }
            isp_ctrl = isp_ctrl & 0x3f ;
            pr_err("%s: MSM_V4L2_POWER_LINE_60HZ\n", __func__);
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                        s_ctrl->sensor_i2c_client,
                        isp_ctrl_addr, isp_ctrl,
                        MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0) {
                pr_err("%s: write isp_ctrl_reg failed\n", __func__);
            }
            break ;
           case FLICKER_50Hz:
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                    s_ctrl->sensor_i2c_client,
                    isp_ctrl_addr, &isp_ctrl,
                    MSM_CAMERA_I2C_BYTE_DATA);
               if (rc < 0) {
                         pr_err("%s: write isp_ctrl_reg failed\n", __func__);
                }
            isp_ctrl = isp_ctrl | 0x80 ;
            isp_ctrl = isp_ctrl & 0xbf ;
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                        s_ctrl->sensor_i2c_client,
                        isp_ctrl_addr, isp_ctrl,
                        MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0) {
                pr_err("%s: write isp_ctrl_reg failed\n", __func__);
            }
            pr_err("%s: MSM_V4L2_POWER_LINE_50HZ\n", __func__);
               break ;
        case FLICKER_OFF:
            pr_err("%s: MSM_V4L2_POWER_LINE_OFF\n", __func__);
            break ;
        default:
            pr_err("%s: MSM_V4L2_POWER_LINE_OTHERS\n", __func__);
            break ;
    }


    isp_ctrl_addr = 0x3013 ;
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            isp_ctrl_addr, &isp_ctrl,
            MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: read isp_ctrl_reg failed\n", __func__);
    }
    if(value == FLICKER_60Hz ||
        value == FLICKER_50Hz){
           isp_ctrl = isp_ctrl | 0x20 ;
    }else{
              isp_ctrl = isp_ctrl & 0xdf ;
    }
        //s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                s_ctrl->sensor_i2c_client,
                isp_ctrl_addr, isp_ctrl,
                MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: write isp_ctrl_reg failed\n", __func__);
    }
        ov2675_group_hold_off(s_ctrl);

    //ov2675_set_bandingfilter(s_ctrl) ;
    //
    CDBG("--CAMERA-- %s ...(End)\n", __func__);//*/
    return rc;
}


//func 5
int ov2675_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
        enum msm_sensor_cfg_type_t  ctrl_info, int value)
{
    int rc = 0;


    if(value == cur_exposure){
        return rc ;
    }
    cur_exposure = value ;
    ov2675_group_hold_on(s_ctrl);
    ov2675_set_exposure( s_ctrl, value);
    ov2675_group_hold_off(s_ctrl);

    if (rc < 0) {
        CDBG("write faield\n");
    }
    CDBG("--CAMERA-- %s ...(End)\n", __func__);
    return rc;
}
//func 6
int ov2675_ISO_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
        enum msm_sensor_cfg_type_t  ctrl_info, int value)
{
    int rc = 0 ;
       uint16_t isp_ctrl_addr = 0x3015;
    uint16_t isp_ctrl = 0;
	
    if(value == cur_ISO){
        return rc ;
    }
    cur_ISO = value ;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
               s_ctrl->sensor_i2c_client,
               isp_ctrl_addr, &isp_ctrl,
               MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
         pr_err("%s: read isp_ctrl_reg failed\n", __func__);
    }

    isp_ctrl = isp_ctrl & 0xf8 ;
    switch(value){
        case ISO_AUTO:
            isp_ctrl = isp_ctrl |0x02 ;
            CDBG("%s: MSM_V4L2_ISO_AUTO\n", __func__);
            break ;
        case ISO_100:
            isp_ctrl = isp_ctrl |0x00 ;
            CDBG("%s: MSM_V4L2_ISO_100\n", __func__);
            break ;
        case ISO_200:
            isp_ctrl = isp_ctrl |0x01 ;
            CDBG("%s: MSM_V4L2_ISO_200\n", __func__);
            break ;
        case ISO_400:
            isp_ctrl = isp_ctrl |0x02 ;
            CDBG("%s: MSM_V4L2_ISO_400\n", __func__);
            break ;
        case ISO_800:
            isp_ctrl = isp_ctrl |0x03 ;
            CDBG("%s: MSM_V4L2_ISO_800\n", __func__);
            break ;
        case ISO_1600:
            isp_ctrl = isp_ctrl |0x04 ;
            CDBG("%s: MSM_V4L2_ISO_1600\n", __func__);
            break ;
        default:
            break ;

    }
        ov2675_group_hold_on(s_ctrl);

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                s_ctrl->sensor_i2c_client,
                isp_ctrl_addr, isp_ctrl,
                MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: write isp_ctrl_reg failed\n", __func__);
    }
    ov2675_group_hold_off(s_ctrl);
    CDBG("--CAMERA-- %s ...(End)\n", __func__);
    return rc ;
}

//func 7
int ov2675_sharpness_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
        enum msm_sensor_cfg_type_t  ctrl_info, int value)
{
    int rc = 0 ;
    uint16_t isp_ctrl_addr = 0x3306;
    uint16_t isp_ctrl = 0;
    uint16_t isp_ctrl_mask = 0xf7;
    if(value == cur_sharpenness){
        return rc ;
    }
    cur_sharpenness = value ;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
               s_ctrl->sensor_i2c_client,
               isp_ctrl_addr, &isp_ctrl,
               MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
         pr_err("%s: read isp_ctrl_reg failed\n", __func__);
    }

    isp_ctrl = isp_ctrl & isp_ctrl_mask ;
    ov2675_group_hold_on(s_ctrl);

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                s_ctrl->sensor_i2c_client,
                isp_ctrl_addr, isp_ctrl,
                MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: write isp_ctrl_reg failed\n", __func__);
    }

    ov2675_set_sharpness( s_ctrl,value);

    if (rc < 0) {
        pr_err("%s: write isp_ctrl_reg failed\n", __func__);
    }
    ov2675_group_hold_off(s_ctrl);
    CDBG("--CAMERA-- %s ...(End)\n", __func__);
    return rc ;

}

void ov2675_get_rgb_gain(struct msm_sensor_ctrl_t *s_ctrl)
{
     if(cur_wb == MSM_CAMERA_WB_MODE_AUTO)
    {
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x330c, 0x02,
                MSM_CAMERA_I2C_BYTE_DATA);
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x330f, &
                ov2675_preview_R_gain,MSM_CAMERA_I2C_BYTE_DATA);
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x330c, 0x03,
                MSM_CAMERA_I2C_BYTE_DATA);
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x330f, &
                ov2675_preview_G_gain,MSM_CAMERA_I2C_BYTE_DATA);
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x330c, 0x04,
                MSM_CAMERA_I2C_BYTE_DATA);
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x330f, &
                ov2675_preview_B_gain,MSM_CAMERA_I2C_BYTE_DATA);
        }

}
void ov2675_set_rgb_gain(struct msm_sensor_ctrl_t *s_ctrl)
{
        if(cur_wb == MSM_CAMERA_WB_MODE_AUTO)
         {
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3306,
                0x12,MSM_CAMERA_I2C_BYTE_DATA);
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3337,
                ov2675_preview_R_gain,MSM_CAMERA_I2C_BYTE_DATA);
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3338,
                ov2675_preview_G_gain,MSM_CAMERA_I2C_BYTE_DATA);
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3339,
                ov2675_preview_B_gain,MSM_CAMERA_I2C_BYTE_DATA);
        }

}

static unsigned int ov2675_get_shutter(struct msm_sensor_ctrl_t *s_ctrl)
{
  // read shutter, in number of line period
    unsigned int shutter = 0, extra_line = 0;
    uint16_t ret_l,ret_h;
    ret_l = ret_h = 0;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3002, &ret_h,
        MSM_CAMERA_I2C_BYTE_DATA);
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3003, &ret_l,
        MSM_CAMERA_I2C_BYTE_DATA);

    shutter = (ret_h << 8) | (ret_l & 0xff) ;
    ret_l = ret_h = 0;
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x302d, &ret_h,
        MSM_CAMERA_I2C_BYTE_DATA);
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x302e, &ret_l,
        MSM_CAMERA_I2C_BYTE_DATA);
    extra_line = (ret_h << 8) | (ret_l & 0xff) ;

    return shutter + extra_line;
}

static int ov2675_set_shutter(struct msm_sensor_ctrl_t *s_ctrl,unsigned int shutter)
{
  // write shutter, in number of line period
    int rc = 0;
    uint16_t temp;
    shutter = shutter & 0xffff;

    temp = shutter & 0xff;
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3003, temp,
        MSM_CAMERA_I2C_BYTE_DATA);

    temp = shutter >> 8;
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3002, temp,
        MSM_CAMERA_I2C_BYTE_DATA);

    return rc;
}

static unsigned int ov2675_get_gain16(struct msm_sensor_ctrl_t *s_ctrl)
{
    unsigned int gain16;
    uint16_t temp;

    temp = 0;
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3000, &temp,
    MSM_CAMERA_I2C_BYTE_DATA);
    CDBG("%s:Reg(0x3000) = 0x%x\n",__func__,temp);
    gain16 = ((temp>>4) + 1) * (16 + (temp & 0x0f));

    return gain16;
}

static int ov2675_set_gain16(struct msm_sensor_ctrl_t *s_ctrl,unsigned int gain16)
{
    int rc = 0;
    uint16_t reg;
    gain16 = gain16 & 0x1ff;    // max gain is 32x
    reg = 0;
    if (gain16 > 32){
        gain16 = gain16 /2;
        reg = 0x10;
    }

    if (gain16 > 32){
        gain16 = gain16 /2;
        reg = reg | 0x20;
    }

    if (gain16 > 32){
        gain16 = gain16 /2;
        reg = reg | 0x40;
    }

    if (gain16 > 32){
        gain16 = gain16 /2;
        reg = reg | 0x80;
    }

    reg = reg | (gain16 -16);

    rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3000,reg,
    MSM_CAMERA_I2C_BYTE_DATA);
    return rc;
}
static unsigned int ov2675_get_binning(struct msm_sensor_ctrl_t *s_ctrl)
{
    // write VTS to registers
    unsigned int  binning;
    uint16_t temp;
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x300b, &temp,
        MSM_CAMERA_I2C_BYTE_DATA);
    if(temp==0x52){
        // OV2650
        binning = 2;
    } else {
        // ov2675
        binning = 1;
    }
	 CDBG("--CAMERA-- %s ...  binning = %d  (End)\n", __func__,binning);
    return binning;
}
static unsigned int ov2675_get_light_frequency(struct msm_sensor_ctrl_t *s_ctrl)
{
    // get banding filter value
    unsigned int  light_frequency;
    uint16_t temp;
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3014, &temp,
    MSM_CAMERA_I2C_BYTE_DATA);
    if (temp & 0x40) {
        // auto
        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x508e, &temp,
            MSM_CAMERA_I2C_BYTE_DATA);
        if (temp & 0x01){
            light_frequency = 50;
        } else {
            light_frequency = 60;
        }
    } else {
        // manual
        if (temp & 0x80){
            // 50Hz
            light_frequency = 50;
        } else {
            // 60Hz
            light_frequency = 60;
        }
     }

    return light_frequency;
}
static int ov2675_dummyline = 0;
static int ov2675_get_dummyline(struct msm_sensor_ctrl_t *s_ctrl)
{
    // read shutter, in number of line period
    int dummyline;
    uint16_t temp;
    dummyline = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x302d, &
    temp,MSM_CAMERA_I2C_BYTE_DATA);

    //ov2675_read_i2c(0x302d) ;
    dummyline = (dummyline<<8) + s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,
    0x302e, &temp,MSM_CAMERA_I2C_BYTE_DATA);
    //ov2675_read_i2c(0x302e);
    return dummyline;
}
static int ov2675_set_nightmode(struct msm_sensor_ctrl_t *s_ctrl,int NightMode)

{
    int rc = 0;
    uint16_t temp;

    CDBG("%s:NightMode = %d\n",__func__,NightMode);
    switch (NightMode) {
        case 0://Off
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3014, &temp,
                MSM_CAMERA_I2C_BYTE_DATA);
            temp = temp & 0xf7;            // night mode off, bit[3] = 0
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3014, temp,
                MSM_CAMERA_I2C_BYTE_DATA);
            // clear dummy lines
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x302d, 0,
                MSM_CAMERA_I2C_BYTE_DATA);
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x302e, 0,
                MSM_CAMERA_I2C_BYTE_DATA);

            break;
        case 1: {// On
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3014, &temp,
                MSM_CAMERA_I2C_BYTE_DATA);
            temp = temp | 0x08;            // night mode on, bit[3] = 1
            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3014, temp,
                MSM_CAMERA_I2C_BYTE_DATA);
        }
        break;
        default:
        break;
    }
    return rc;
}
static int ov2675_get_preview_exposure_gain(struct msm_sensor_ctrl_t *s_ctrl)
{
  int rc = 0;
  ov2675_preview_shutter = ov2675_get_shutter(s_ctrl);
  // read preview gain
  ov2675_preview_gain16 = ov2675_get_gain16(s_ctrl);
  ov2675_preview_binning = ov2675_get_binning(s_ctrl);
  // turn off night mode for capture
  rc = ov2675_set_nightmode(s_ctrl,0);
  ov2675_dummyline = ov2675_get_dummyline(s_ctrl);
    pr_err("%s:shutter=%d,gain=%d\n",__func__,ov2675_preview_shutter,ov2675_preview_gain16) ;
  return rc;
}

static int ov2675_set_preview_exposure_gain(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
    rc = ov2675_set_shutter(s_ctrl,ov2675_preview_shutter);
    rc = ov2675_set_gain16(s_ctrl,ov2675_preview_gain16);
    pr_err("%s:shutter=%d,gain=%d\n",__func__,ov2675_preview_shutter,ov2675_preview_gain16) ;
    return rc;
}
static int ov2675_set_capture_exposure_gain(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
    unsigned int capture_shutter, capture_gain16, capture_sysclk, capture_HTS, capture_VTS;
    unsigned int light_frequency, capture_bandingfilter, capture_max_band;
    unsigned long capture_gain16_shutter;
    uint16_t temp;
    //Step3: calculate and set capture exposure and gain
    // turn off AEC, AGC
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3013, &temp,
    MSM_CAMERA_I2C_BYTE_DATA);
    temp = temp & 0xfa;
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3013, temp,
    MSM_CAMERA_I2C_BYTE_DATA);
    // read capture sysclk
    capture_sysclk = ov2675_get_sysclk(s_ctrl);
    // read capture HTS
    capture_HTS = ov2675_get_HTS(s_ctrl);
    // read capture VTS
    capture_VTS = ov2675_get_VTS(s_ctrl);
    // calculate capture banding filter
    light_frequency = ov2675_get_light_frequency(s_ctrl);
    if (light_frequency == 60) {
      // 60Hz
      capture_bandingfilter = capture_sysclk * 100 / capture_HTS * 100 / 120;
    } else {
      // 50Hz
      capture_bandingfilter = capture_sysclk * 100 / capture_HTS;
     }

    capture_max_band = ((capture_VTS-4)/capture_bandingfilter);
    // calculate capture shutter

    capture_gain16 = ov2675_preview_gain16;
    capture_shutter = ov2675_preview_shutter * capture_sysclk/ov2675_preview_sysclk * ov2675_preview_HTS/capture_HTS * ov2675_preview_binning;
    if (capture_shutter < 1) {
        capture_shutter = 1;
    }
    if (capture_gain16 < 16) {
        capture_gain16 = 16;
     }
    capture_gain16_shutter = capture_gain16 * capture_shutter;
    if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
        // shutter < 1/100
        capture_shutter = capture_gain16_shutter/16;
        capture_gain16 = capture_gain16_shutter/capture_shutter;
    } else {
        if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
            // exposure reach max
            capture_shutter = capture_bandingfilter*capture_max_band;
            capture_gain16 = capture_gain16_shutter / capture_shutter;
        } else {
            // 1/100 < capture_shutter < max, capture_shutter = n/100
            capture_shutter = (capture_gain16_shutter/16/capture_bandingfilter)
              * capture_bandingfilter;
            capture_gain16 = capture_gain16_shutter / capture_shutter;
        }
     }
    // write capture gain
    rc |= ov2675_set_gain16(s_ctrl,capture_gain16);
    // write capture shutter
    if (capture_shutter > (capture_VTS - 4)) {
      capture_VTS = capture_shutter + 4;
      rc |= ov2675_set_VTS(s_ctrl,capture_VTS);
     }
     rc |= ov2675_set_shutter(s_ctrl,capture_shutter);
     return rc;
 }
int32_t ov2675_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    //first we must close mipi interface
        //first we must close mipi interface
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
        0x30ab,0x00,MSM_CAMERA_I2C_BYTE_DATA) ;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
        0x30ad,0x0a,MSM_CAMERA_I2C_BYTE_DATA) ;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
        0x30ae,0x27,MSM_CAMERA_I2C_BYTE_DATA) ;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
        0x363b,0x01,MSM_CAMERA_I2C_BYTE_DATA) ;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
        0x363c,0xb2,MSM_CAMERA_I2C_BYTE_DATA) ;
    cur_effect = MSM_CAMERA_EFFECT_MODE_OFF ;
    cur_exposure = -1;
    cur_wb = -1 ;
    //cur_brightness = -1 ;
    cur_saturation = -1 ;
    cur_contrast = -1;
    cur_ISO = -1;
    cur_sharpenness = -1 ;
    cur_res = -1 ;

    return msm_sensor_power_down(s_ctrl);

}

#endif
int32_t ov2675_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
/*TYRD wang_gj add for sending yuv sensor stream type to kernel begin*/
#ifdef TYQ_YUV_SENSOR_STREAM_TYPE_SUPPORT
	struct msm_sensor_yuv_resolution_t {
  	enum msm_sensor_resolution_t res;
	uint32_t stream_mask;
  	}yuv_res;
/*TYRD wang_gj add for sending yuv sensor stream type to kernel end*/
#endif 
	enum msm_sensor_resolution_t res = MSM_SENSOR_INVALID_RES;	
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
		/* Write Recommend settings */
		pr_err("%s, sensor write init setting!!", __func__);
#if 1
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0x3012,
			0x80, MSM_CAMERA_I2C_BYTE_DATA);
        msleep(5);
#endif
#if 1
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			ov2675_recommend_settings,
			ARRAY_SIZE(ov2675_recommend_settings),
			MSM_CAMERA_I2C_BYTE_DATA);		     
#endif		
	        ov2675_preview_shutter = ov2675_get_shutter(s_ctrl);
	        ov2675_preview_gain16 = ov2675_get_gain16(s_ctrl);
		break;
	case CFG_SET_RESOLUTION:
		res = *(int*)(cdata->cfg.setting);
/*TYRD wang_gj add for sending yuv sensor stream type to kernel begin*/
#ifdef TYQ_YUV_SENSOR_STREAM_TYPE_SUPPORT
		yuv_res = *(struct msm_sensor_yuv_resolution_t*)(cdata->cfg.setting);
		pr_err("%s, sensor write resolution setting res %d  stream_mask %x ! !", __func__, res,yuv_res.stream_mask);
#endif
/*TYRD wang_gj add for sending yuv sensor stream type to kernel end*/

		if (res == MSM_SENSOR_RES_FULL) {
/*TYRD wang_gj add for sending yuv sensor stream type to kernel begin*/
#ifdef TYQ_YUV_SENSOR_STREAM_TYPE_SUPPORT
			if(yuv_res.stream_mask == 0xc)//snapshot
				{
				
				pr_err("%s snapshot setting ! !", __func__);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
								   ov2675_UXGA_settings,
								   ARRAY_SIZE(ov2675_UXGA_settings),
								   MSM_CAMERA_I2C_BYTE_DATA);	
			ov2675_set_rgb_gain(s_ctrl) ;
			ov2675_set_capture_exposure_gain(s_ctrl);
			cur_res = RES_SNAPSHOT ;
			msleep(200);	
				}
			else if(yuv_res.stream_mask == 0xa)//ZSL
				{
				pr_err("%s ZSL setting ! !", __func__);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
								   ov2675_zsl_settings,
								   ARRAY_SIZE(ov2675_zsl_settings),
								   MSM_CAMERA_I2C_BYTE_DATA);	
			cur_res = RES_ZSL;
			msleep(134);	
				}
			else//hardcode to snapshot
				{
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
								   ov2675_UXGA_settings,
								   ARRAY_SIZE(ov2675_UXGA_settings),
								   MSM_CAMERA_I2C_BYTE_DATA);	
			ov2675_set_rgb_gain(s_ctrl) ;
			ov2675_set_capture_exposure_gain(s_ctrl);
			cur_res = RES_SNAPSHOT ;
			msleep(200);	
				}
#else
	{
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
							   ov2675_UXGA_settings,
							   ARRAY_SIZE(ov2675_UXGA_settings),
							   MSM_CAMERA_I2C_BYTE_DATA);	
		ov2675_set_rgb_gain(s_ctrl) ;
		ov2675_set_capture_exposure_gain(s_ctrl);
		msleep(200);	
	}

#endif
/*TYRD wang_gj add for sending yuv sensor stream type to kernel end*/

		} else if (res == MSM_SENSOR_RES_QTR) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
								   ov2675_VGA_settings,
								   ARRAY_SIZE(ov2675_VGA_settings),
								   MSM_CAMERA_I2C_BYTE_DATA);	
			rc |= ov2675_set_bandingfilter(s_ctrl) ;
			ov2675_write(s_ctrl, 0x3013, 0xf0);
			ov2675_set_preview_exposure_gain(s_ctrl) ;
			msleep(134) ;
			//turn on AEC/AGC
			ov2675_write(s_ctrl, 0x3013, 0xf7);
			cur_res = RES_PREVIEW ;
		}
		else {
			pr_err("%s, unsupport resolution res %d ! !", __func__, res);
		}
		break;
	case CFG_SET_STOP_STREAM:
		pr_err("%s, sensor stop stream!!", __func__);
		if(cur_res == RES_PREVIEW || cur_res == -1){
		ov2675_get_preview_exposure_gain(s_ctrl);
		ov2675_get_rgb_gain(s_ctrl) ;
		}
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			ov2675_stop_settings,
			ARRAY_SIZE(ov2675_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		break;
	case CFG_SET_START_STREAM:
		pr_err("%s, sensor start stream!!", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			ov2675_start_settings,
			ARRAY_SIZE(ov2675_start_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_camera_power_ctrl_t *p_ctrl;
		uint16_t size;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
		    (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		p_ctrl = &s_ctrl->sensordata->power_info;
		size = sensor_slave_info.power_setting_array.size;
		if (p_ctrl->power_setting_size < size) {
			struct msm_sensor_power_setting *tmp;
			tmp = kmalloc(sizeof(struct msm_sensor_power_setting)
				      * size, GFP_KERNEL);
			if (!tmp) {
				pr_err("%s: failed to alloc mem\n", __func__);
				rc = -ENOMEM;
				break;
			}
			kfree(p_ctrl->power_setting);
			p_ctrl->power_setting = tmp;
		}
		p_ctrl->power_setting_size = size;

		rc = copy_from_user(p_ctrl->power_setting, (void *)
			sensor_slave_info.power_setting_array.power_setting,
			size * sizeof(struct msm_sensor_power_setting));
		if (rc) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			p_ctrl->power_setting_size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				p_ctrl->power_setting[slave_index].seq_type,
				p_ctrl->power_setting[slave_index].seq_val,
				p_ctrl->power_setting[slave_index].config_val,
				p_ctrl->power_setting[slave_index].delay);
		}
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(
				s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
		    (void *)reg_setting, stop_setting->size *
		    sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}

	case CFG_SET_SATURATION: {
		int32_t sat_lev;
		if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);
		ov2675_SDE_msm_sensor_s_ctrl_by_enum(s_ctrl, CFG_SET_SATURATION,sat_lev);
		break;
	}
	case CFG_SET_CONTRAST: {
		int32_t con_lev;
		if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);
		ov2675_SDE_msm_sensor_s_ctrl_by_enum(s_ctrl, CFG_SET_CONTRAST,con_lev);
		break;
	}
	case CFG_SET_SHARPNESS: {
		int32_t shp_lev;
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Sharpness Value is %d", __func__, shp_lev);
		ov2675_sharpness_msm_sensor_s_ctrl_by_enum(s_ctrl, CFG_SET_SHARPNESS,shp_lev);
		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: ISO Value is %d", __func__, iso_lev);
		ov2675_ISO_msm_sensor_s_ctrl_by_enum(s_ctrl,CFG_SET_ISO, iso_lev);
		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
		ov2675_msm_sensor_s_ctrl_by_enum(s_ctrl, CFG_SET_EXPOSURE_COMPENSATION,ec_lev);
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Effect mode is %d", __func__, effect_mode);
		ov2675_effect_msm_sensor_s_ctrl_by_enum(s_ctrl, CFG_SET_EFFECT,effect_mode);
		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: anti-banding mode is %d", __func__,
			antibanding_mode);
		ov2675_antibanding_msm_sensor_s_ctrl_by_enum(s_ctrl, CFG_SET_ANTIBANDING,antibanding_mode);
		break;
	}
	case CFG_SET_BESTSHOT_MODE: {
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: best shot mode is %d", __func__, bs_mode);
		ov2675_set_scene_mode(s_ctrl, bs_mode);
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: white balance is %d", __func__, wb_mode);
		ov2675_wb_msm_sensor_s_ctrl_by_enum(s_ctrl,CFG_SET_WHITE_BALANCE, wb_mode);
		break;
	}
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int32_t ov2675_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	//return 0;

	CDBG("%s, E. calling i2c_read:, i2c_addr:%d, id_reg_addr:%d\n",
		__func__,
		s_ctrl->sensordata->slave_info->sensor_slave_addr,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x300a,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("%s: read id: %x expected id 0x16:\n", __func__, chipid);
	if (chipid != 0x26) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}

	chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x300b,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("%s: read id: %x expected id 0x28:\n", __func__, chipid);
	if (chipid != 0x56) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}

	return rc;
}


static struct msm_sensor_fn_t ov2675_sensor_func_tbl = {
	.sensor_config = ov2675_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = ov2675_sensor_power_down,
	.sensor_match_id = ov2675_match_id,
};

static struct msm_sensor_ctrl_t ov2675_s_ctrl = {
	.sensor_i2c_client = &ov2675_sensor_i2c_client,
	.power_setting_array.power_setting = ov2675_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov2675_power_setting),
	.msm_sensor_mutex = &ov2675_mut,
	.sensor_v4l2_subdev_info = ov2675_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov2675_subdev_info),
	.func_tbl = &ov2675_sensor_func_tbl,
};

module_init(ov2675_init_module);
module_exit(ov2675_exit_module);
MODULE_DESCRIPTION("Aptina 1.26MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
