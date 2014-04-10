/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#define OV9760_SENSOR_NAME "ov9760_sunny_q1v04a"
DEFINE_MSM_MUTEX(ov9760_sunny_q1v04a_mut);

//#define OV9760_OTP_KERNEL_SUPPORT
//#define  CDBG

#ifdef CDBG
#undef CDBG
#define CDBG printk
#endif

#ifdef OV9760_OTP_KERNEL_SUPPORT
#define BG_Ratio_Typical 0x173
#define RG_Ratio_Typical 0x14e
struct msm_sensor_ctrl_t *extern_s_ctrl;
static uint16_t OV9760_read_i2c(uint16_t reg_addr)
{
	uint16_t data;
	 extern_s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			extern_s_ctrl->sensor_i2c_client,
			reg_addr,
			&data, MSM_CAMERA_I2C_BYTE_DATA);

	CDBG("%s:Data[0x%x] = 0x%x\n",__func__, reg_addr,data);

	 return data;
}
static uint16_t OV9760_write_i2c(uint16_t reg_addr,uint16_t data)
{
	 extern_s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			extern_s_ctrl->sensor_i2c_client,
			reg_addr,
			data, MSM_CAMERA_I2C_BYTE_DATA);

	 CDBG("%s:Data[0x%x] = 0x%x\n", __func__,reg_addr,data);

	 return 1;
}

struct otp_struct {
	int module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int light_rg;
	int light_bg;
	int user_data[5];
	int lenc[24];
};
// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_wb(int index)
{
	int flag, i;
	int bank, address;

	// select bank index
	bank = 0xc0 | index;
	OV9760_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV9760_write_i2c(0x3d81, 0x01);
	mdelay(5);
	// read flag
	address = 0x3d00;
	flag = OV9760_read_i2c(address);
	flag = flag & 0xc0;
	// clear otp buffer
	for (i=0;i<16;i++) {
		OV9760_write_i2c(0x3d00 + i, 0x00);
	}
	if (flag == 0x00) {
		return 0;
	}
	else if (flag & 0x80) {
		return 1;
	}
	else {
		return 2;
	}
}

// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_lenc(int index)
{
	int flag, i;
	int bank, address;

	// select bank index
	bank = 0xc0 | ((index+1) *2);
	OV9760_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV9760_write_i2c(0x3d81, 0x01);
	mdelay(5);
	// read flag
	address = 0x3d00;
	flag = OV9760_read_i2c(address);
	flag = flag & 0xc0;
	// clear otp buffer
	for (i=0;i<16;i++) {
		OV9760_write_i2c(0x3d00 + i, 0x00);
	}
	if (flag == 0x00) {
		return 0;
	}
	else if (flag & 0x80) {
		return 1;
	}
	else {
		return 2;
	}
}

	// index: index of otp group. (1, 2, 3)
	// otp_ptr: pointer of otp_struct
	// return: 0, 
int read_otp_wb(int index,  struct otp_struct *otp_ptr)
{
	int i, bank;
	int address;
	int temp;

	// select bank index
	bank = 0xc0 | index;
	OV9760_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV9760_write_i2c(0x3d81, 0x01);
	mdelay(5);
	//print otp buffer
	for (i=0;i<16;i++) {
		CDBG("-->%s:Data[%d] = 0x%x",__func__,i,OV9760_read_i2c(0x3d00 + i));
	}
	
	address = 0x3d00;
	(*otp_ptr).module_integrator_id = OV9760_read_i2c(address + 1);
	(*otp_ptr).lens_id = OV9760_read_i2c(address + 2);
	(*otp_ptr).production_year = OV9760_read_i2c(address + 3);
	(*otp_ptr).production_month = OV9760_read_i2c(address + 4);
	(*otp_ptr).production_day = OV9760_read_i2c(address + 5);
	temp = OV9760_read_i2c(address + 10);
	(*otp_ptr).rg_ratio = (OV9760_read_i2c(address + 6)<<2) + ((temp>>6) & 0x03);
	(*otp_ptr).bg_ratio = (OV9760_read_i2c(address + 7)<<2) + ((temp>>4) & 0x03);
	(*otp_ptr).light_rg = (OV9760_read_i2c(address + 8) <<2) + ((temp>>2) & 0x03);
	(*otp_ptr).light_bg = (OV9760_read_i2c(address + 9)<<2) +  (temp & 0x03);
	(*otp_ptr).user_data[0] = OV9760_read_i2c(address + 11);
	(*otp_ptr).user_data[1] = OV9760_read_i2c(address + 12);
	(*otp_ptr).user_data[2] = OV9760_read_i2c(address + 13);
	(*otp_ptr).user_data[3] = OV9760_read_i2c(address + 14);
	(*otp_ptr).user_data[4] = OV9760_read_i2c(address + 15);

	CDBG("rg_ratio = 0x%x\n",(*otp_ptr).rg_ratio);
	CDBG("bg_ratio = 0x%x\n",(*otp_ptr).bg_ratio);
	CDBG("light_rg = 0x%x\n",(*otp_ptr).light_rg);
	CDBG("light_bg = 0x%x\n",(*otp_ptr).light_bg);
	// clear otp buffer
	for (i=0;i<16;i++) {
		OV9760_write_i2c(0x3d00 + i, 0x00);
	}
	return 0;
}


// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0, 
int read_otp_lenc(int index,  struct otp_struct *otp_ptr)
{
	int i, bank;
	int address;
	int temp;

	// select bank index
	bank = 0xc0 | ((index+1) *2);
	OV9760_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV9760_write_i2c(0x3d81, 0x01);
	mdelay(5);

	//print otp buffer
	for (i=0;i<16;i++) {
		CDBG("-->%s:BANK:Data[%d] = 0x%x",__func__,i,OV9760_read_i2c(0x3d00 + i));
	}
	
	address = 0x3d01;
	temp = OV9760_read_i2c(address + 2);
	(*otp_ptr).lenc[0] = (temp & 0x39)>>3; // Red_X0[10:8]
	(*otp_ptr).lenc[1] = OV9760_read_i2c(address); // Red_X0[7:0]
	(*otp_ptr).lenc[2] = temp & 0x07; // Red_Y0[10;8}
	(*otp_ptr).lenc[3] = OV9760_read_i2c(address + 1); // Red_Y0[7:0]
	(*otp_ptr).lenc[4] = OV9760_read_i2c(address + 3); // Red_A1
	temp = OV9760_read_i2c(address + 5);
	(*otp_ptr).lenc[5] = temp>>4; // Red_A2
	(*otp_ptr).lenc[6] = OV9760_read_i2c(address + 4); // Red_B1
	(*otp_ptr).lenc[7] = temp & 0x0f; // Red_B2
	temp = OV9760_read_i2c(address + 8);
	(*otp_ptr).lenc[8] = (temp & 0x39)>>3; // Green_X0[10:8]
	(*otp_ptr).lenc[9] = OV9760_read_i2c(address + 6); // Green_X0[7:0]
	(*otp_ptr).lenc[10] = temp & 0x07; // Green_Y0[10;8}
	(*otp_ptr).lenc[11] = OV9760_read_i2c(address + 7); // Green_Y0[7:0]
	(*otp_ptr).lenc[12] = OV9760_read_i2c(address + 9); // Green_A1
	temp = OV9760_read_i2c(address + 11);
	(*otp_ptr).lenc[13] = temp>>4; // Green_A2
	(*otp_ptr).lenc[14] = OV9760_read_i2c(address + 10); // Green_B1
	(*otp_ptr).lenc[15] = temp & 0x0f; // Green_B2
	temp = OV9760_read_i2c(address + 14);
	(*otp_ptr).lenc[16] = (temp & 0x39)>>3; // Blue_X0[10:8]
	(*otp_ptr).lenc[17] = OV9760_read_i2c(address + 12); // Blue_X0[7:0]
	(*otp_ptr).lenc[18] = temp & 0x07; // Blue_Y0[10;8}
	(*otp_ptr).lenc[19] = OV9760_read_i2c(address + 13); // Blue_Y0[7:0]
	bank ++;
	OV9760_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV9760_write_i2c(0x3d81, 0x01);

	//print otp buffer
	for (i=0;i<16;i++) {
		CDBG("-->%s:BANK++:Data[%d] = 0x%x",__func__,i,OV9760_read_i2c(0x3d00 + i));
	}
	
	address = 0x3d00;
	(*otp_ptr).lenc[20] = OV9760_read_i2c(address);// Blue_A1
	temp = OV9760_read_i2c(address + 2);
	(*otp_ptr).lenc[21] = temp>>4; // Blue_A2
	(*otp_ptr).lenc[22] = OV9760_read_i2c(address + 1); // Blue_B1
	(*otp_ptr).lenc[23] = temp & 0x0f; // Blue_B2
	// clear otp buffer
	for (i=0;i<16;i++) {
		OV9760_write_i2c(0x3d00 + i, 0x00);
	}

	//print lenc
	for (i=0;i<24;i++) {
		CDBG("-->%s:(*otp_ptr).lenc[%d] = 0x%x",__func__,i,(*otp_ptr).lenc[i]);
	}

	return 0;
}

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
int update_awb_gain(int R_gain, int G_gain, int B_gain)
{
	if (R_gain>0x400) {
		OV9760_write_i2c(0x5180, R_gain>>8);
		OV9760_write_i2c(0x5181, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
		OV9760_write_i2c(0x5182, G_gain>>8);
		OV9760_write_i2c(0x5183, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
		OV9760_write_i2c(0x5184, B_gain>>8);
		OV9760_write_i2c(0x5185, B_gain & 0x00ff);
	}
	return 0;
}
// call this function after OV9760 initialization
// return value:  0 update success
// 1, no OTP
int update_otp_wb(void)
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	int rg,bg;
	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	//sunny q1v04a  valid data from bank1
	//o-film oty1f01 valid data from bank0
	for(i=1;i<=3;i++) {
		temp = check_otp_wb(i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i>3) {
		// no valid wb OTP data
		return 1;
	}
	read_otp_wb(otp_index, &current_otp);
	if(current_otp.light_rg==0) {
		// no light source information in OTP, light factor = 1
		rg = current_otp.rg_ratio;
	}
	else {
		rg = current_otp.rg_ratio * ((current_otp.light_rg +512) / 1024);
	}
	if(current_otp.light_bg==0) {
		// not light source information in OTP, light factor = 1
		bg = current_otp.bg_ratio;
	}
	else {
		bg = current_otp.bg_ratio * ((current_otp.light_bg +512) / 1024);
	}
	//calculate G gain
	//0x400 = 1x gain
	if(bg < BG_Ratio_Typical) {
		if (rg< RG_Ratio_Typical) {
			// current_otp.bg_ratio < BG_Ratio_typical &&  
			// current_otp.rg_ratio < RG_Ratio_typical
    			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical / bg;
    			 R_gain = 0x400 * RG_Ratio_Typical / rg; 
		}
		else {
			// current_otp.bg_ratio < BG_Ratio_typical &&  
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else {
		if (rg < RG_Ratio_Typical) {
			// current_otp.bg_ratio >= BG_Ratio_typical &&  
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = 0x400;
			G_gain = 0x400 * bg / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / rg;
		}
		else {
			// current_otp.bg_ratio >= BG_Ratio_typical &&  
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x400 * bg / BG_Ratio_Typical;
			G_gain_R = 0x400 * rg / RG_Ratio_Typical;
			if(G_gain_B > G_gain_R ) {
         			B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
  			 }
			else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / bg;
			}
     		}    
	}

	CDBG("-->R_gain = 0x%x,G_gain = 0x%x,B_gain = 0x%x",
				R_gain,G_gain,B_gain);
	update_awb_gain(R_gain, G_gain, B_gain);
	return 0;
}
// otp_ptr: pointer of otp_struct
int update_lenc(struct otp_struct * otp_ptr)
{
	int i, temp;
	temp = OV9760_read_i2c(0x5000);
	temp = 0x80 | temp;
	OV9760_write_i2c(0x5000, temp);
	for(i=0;i<24;i++) {
		OV9760_write_i2c(0x5800 + i, (*otp_ptr).lenc[i]);
	}
	return 0;
}
 
 
 
 
 
 // call this function after OV9760 initialization
// return value: 0 update success
// 1, no OTP
int update_otp_lenc(void)
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	// check first lens correction OTP with valid data
	//sunny q1v04a  valid data from bank1
	//o-film oty1f01 valid data from bank0
	for(i=1;i<=3;i++) {
		temp = check_otp_lenc(i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i>3) {
		// no valid WB OTP data
		return 1;
	}
	read_otp_lenc(otp_index, &current_otp);
	update_lenc(&current_otp);
	// success
	return 0;
}
// call this function after OV9760 initialization
// return value:  1 use CP data from REG3D0A
// 2 use Module data from REG3D0A
// 0 data ErRoR
int update_blc_ratio(void)
{
	int K,i;
	int temp;
	OV9760_write_i2c(0x3d84, 0xdf);
	OV9760_write_i2c(0x3d81, 0x01);
	mdelay(5);
	
	//print otp buffer
	for (i=0;i<16;i++) {
		CDBG("-->%s:Data[%d] = 0x%x\n",__func__,i,OV9760_read_i2c(0x3d00 + i));
	}
	
	K = OV9760_read_i2c(0x3d0b);
	if (K != 0) {
		if (K >= 0x15 && K <= 0x40) {
			// auto load mode
			temp = OV9760_read_i2c(0x4000);
			temp &= 0x9f;
			OV9760_write_i2c(0x4000, temp);
			return 2;
		}
	}
	K = OV9760_read_i2c(0x3d0a);
	if (K >= 0x10 && K <= 0x40) {
		// manual load mode
		OV9760_write_i2c(0x4006, K);
		temp = OV9760_read_i2c(0x4000);
		temp |= 0x40;
		temp &= 0xdf;
		OV9760_write_i2c(0x4000, temp);
		return 1;
	}
	else {
		// set to default
		OV9760_write_i2c(0x4006, 0x20);
		temp = OV9760_read_i2c(0x4000);
		temp |= 0x40;
		temp &= 0xdf;
		OV9760_write_i2c(0x4000, temp);
	return 0;
	}
}
#endif

static struct msm_sensor_ctrl_t ov9760_sunny_q1v04a_s_ctrl;

static struct msm_sensor_power_setting ov9760_sunny_q1v04a_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},

	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay =1,
	},

	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
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
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
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
		.delay = 10,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay =10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov9760_sunny_q1v04a_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};     

static int32_t msm_ov9760_sunny_q1v04a_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov9760_sunny_q1v04a_s_ctrl);
}

static const struct i2c_device_id ov9760_sunny_q1v04a_i2c_id[] = {
	{OV9760_SENSOR_NAME, (kernel_ulong_t)&ov9760_sunny_q1v04a_s_ctrl},
	{ }
};

static struct i2c_driver ov9760_sunny_q1v04a_i2c_driver = {
	.id_table = ov9760_sunny_q1v04a_i2c_id,
	.probe  = msm_ov9760_sunny_q1v04a_i2c_probe,
	.driver = {
		.name = OV9760_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov9760_sunny_q1v04a_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};
#ifdef OV9760_OTP_KERNEL_SUPPORT
#if 0
static struct msm_camera_i2c_reg_conf ov9760_recommend_settings[] = {
{0x0340,0x4 },//VTS
{0x0341,0x93 },//VTS, 03/05/2012
{0x0342,0x6 },//HTS, 03/05/2012
{0x0343,0xAB },//HTS, 03/05/2012
{0x0344,0x0 },//x_addr_start
{0x0345,0x08 },//x_addr_start, 03/01/2012
{0x0346,0x0 },//y_addr_start
{0x0347,0x02},//y_addr_start
{0x0348,0x5 },//x_addr_end
{0x0349,0xdf },//x_addr_end, 03/01/2012
{0x034a,0x4 },//y_addr_end
{0x034b,0x65},//y_addr_end
{0x3811,0x4 },//x_offset
{0x3813,0x4},// y_offset
{0x034c,0x5 },//x_output_size
{0x034d,0xC0 },//x_output_size
{0x034e,0x4 },//y_output_size
{0x034f,0x50 },//y_output_size
{0x0383,0x1 },//x_odd_inc
{0x0387,0x1 },//y_odd_inc
{0x3820,0x0 },//V bin
{0x3821,0x0 },//H bin
{0x3660,0x80 },//Analog control, 03/01/2012
{0x3680,0xf4 },//Analog control, 03/01/2012
{0x0100,0x0 },//Mode select - stop streaming
#ifdef TYQ_CAM_OV9760_FLIP_SUPPORT
{0x0101,0x1 },//Orientation
#else
{0x0101,0x2 },//Orientation
#endif
{0x3002,0x80 },//IO control
{0x3012,0x8 },//MIPI control
{0x3014,0x4 },//MIPI control
{0x3022,0x2 },//Analog control
{0x3023,0x0f },//Analog control
{0x3080,0x0 },//PLL control
{0x3090,0x2 },//PLL control
{0x3091,0x28 },//PLL control
{0x3092,0x2 },//
{0x3093,0x2 },//PLL control
{0x3094,0x0 },//PLL control
{0x3095,0x0 },//PLL control
{0x3096,0x1 },//PLL control
{0x3097,0x0 },//PLL control
{0x3098,0x4 },//PLL control
{0x3099,0x14 },//PLL control
{0x309a,0x3 },//PLL control
{0x309c,0x0 },//PLL control
{0x309d,0x0 },//PLL control
{0x309e,0x1 },//PLL control
{0x309f,0x0 },//PLL control
{0x30a2,0x1 },//PLL control
{0x30b0,0x0a },//v05
{0x30b3,0x32 },//PLL control
{0x30b4,0x2 },//PLL control
{0x30b5,0x0 },//PLL control
{0x3503,0x17 },//Auto gain/exposure, //set as 0x17 become manual mode
{0x3509,0x10 },//AEC control
{0x3600,0x7c },//Analog control
{0x3621,0xb8 },//v04
{0x3622,0x23 },//Analog control
{0x3631,0xe2 },//Analog control
{0x3634,0x3 },//Analog control
{0x3662,0x14 },//Analog control
{0x366b,0x3 },//Analog control
{0x3682,0x82 },//Analog control
{0x3705,0x20 },//
{0x3708,0x64 },//
{0x371b,0x60 },//Sensor control
{0x3732,0x40 },//Sensor control
{0x3745,0x0 },//Sensor control
{0x3746,0x18 },//Sensor control
{0x3780,0x2a },//Sensor control
{0x3781,0x8c },//Sensor control
{0x378f,0xf5 },//Sensor control
{0x3823,0x37 },//Internal timing control
{0x383d,0x88 },//Adjust starting black row for BLC calibration to avoid FIFO empty condition, 03/01/2012
{0x4000,0x23 },//BLC control, 03/06/2012, disable DCBLC for production test
{0x4001,0x4 },//BLC control
{0x4002,0x45 },//BLC control
{0x4004,0x8 },//BLC control
{0x4005,0x40 },//BLC for flashing
{0x4006,0x40 },//BLC control
{0x4009,0x40 },//BLC
{0x404F,0x8F },//BLC control to improve black level fluctuation, 03/01/2012
{0x4058,0x44 },//BLC control
{0x4101,0x32 },//BLC control
{0x4102,0xa4 },//BLC control
{0x4520,0xb0 },//For full res
{0x4580,0x8 },//Bypassing HDR gain latch, 03/01/2012
{0x4582,0x0 },//Bypassing HDR gain latch, 03/01/2012
{0x4307,0x30 },//MIPI control
{0x4605,0x0 },//VFIFO control v04 updated
{0x4608,0x2 },//VFIFO control v04 updated
{0x4609,0x0 },//VFIFO control v04 updated
{0x4801,0x0f },//MIPI control
{0x4819,0xB6 },//MIPI control v05 updated
{0x4837,0x21 },//MIPI control
{0x4906,0xff },//Internal timing control
{0x4d00,0x04 },//Temperature sensor
{0x4d01,0x4b },//Temperature sensor
{0x4d02,0xfe },//Temperature sensor
{0x4d03,0x09 },//Temperature sensor
{0x4d04,0x1e },//Temperature sensor
{0x4d05,0xb7 },//Temperature sensor
{0x5781,0x17},
{0x5792,0x00},
//TY wuchx remove for start stream error begin
//{0x0100,0x1  },
//{0x3501,0x45},
//{0x3502,0x80},
//{0x350b,0xf0},
//TY wuchx remove for start stream error end	
};
#endif
int32_t ov9760_sunny_q1v04a_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;

	extern_s_ctrl = s_ctrl;

	CDBG("Enter %s\n",__func__);
	CDBG("%s, E. calling i2c_read:, i2c_addr:0x%x, id_reg_addr:0x%x,sensor_id:0x%x\n",
		__func__,
		s_ctrl->sensordata->slave_info->sensor_slave_addr,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
		s_ctrl->sensordata->slave_info->sensor_id);
	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("-%s: read id: %x expected id 0x%x:\n", __func__, chipid,s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}


	//Writing init settings:
	CDBG("-0 %s\n",__func__);	
		//  {0x0103, 0x01},
		//s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0103, 0x1,
		//				MSM_CAMERA_I2C_BYTE_DATA);
		//msleep(5);
		//  {0x0100, 0x01},
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x100, 0x1,
						MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		#if 0
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov9760_recommend_settings,
			ARRAY_SIZE(ov9760_recommend_settings),
			MSM_CAMERA_I2C_BYTE_DATA);	
	msleep(20);
	#endif
	CDBG("-1%s\n",__func__);	

	//Read otp
	update_otp_wb();  
	update_otp_lenc();
	update_blc_ratio();
	CDBG("-2%s\n",__func__);	


	CDBG("-Exit%s\n",__func__);	
	return rc;
}

static struct msm_sensor_fn_t ov9760_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = ov9760_sunny_q1v04a_match_id,
};
#endif
static struct msm_sensor_ctrl_t ov9760_sunny_q1v04a_s_ctrl = {
#ifdef OV9760_OTP_KERNEL_SUPPORT
	.func_tbl = &ov9760_func_tbl,
#endif
	.sensor_i2c_client = &ov9760_sunny_q1v04a_sensor_i2c_client,
	.power_setting_array.power_setting = ov9760_sunny_q1v04a_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov9760_sunny_q1v04a_power_setting),
	.msm_sensor_mutex = &ov9760_sunny_q1v04a_mut,
	.sensor_v4l2_subdev_info = ov9760_sunny_q1v04a_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov9760_sunny_q1v04a_subdev_info),
};

static const struct of_device_id ov9760_sunny_q1v04a_dt_match[] = {
	{.compatible = "qcom,ov9760_sunny_q1v04a", .data = &ov9760_sunny_q1v04a_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov9760_sunny_q1v04a_dt_match);
static struct platform_driver ov9760_sunny_q1v04a_platform_driver = {
	.driver = {
		.name = "qcom,ov9760_sunny_q1v04a",
		.owner = THIS_MODULE,
		.of_match_table = ov9760_sunny_q1v04a_dt_match,
	},
};

static int32_t ov9760_sunny_q1v04a_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(ov9760_sunny_q1v04a_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov9760_sunny_q1v04a_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&ov9760_sunny_q1v04a_platform_driver,
		ov9760_sunny_q1v04a_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&ov9760_sunny_q1v04a_i2c_driver);
}

static void __exit ov9760_sunny_q1v04a_exit_module(void)
{
	if (ov9760_sunny_q1v04a_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov9760_sunny_q1v04a_s_ctrl);
		platform_driver_unregister(&ov9760_sunny_q1v04a_platform_driver);
	} else
		i2c_del_driver(&ov9760_sunny_q1v04a_i2c_driver);
	return;
}

module_init(ov9760_sunny_q1v04a_init_module);
module_exit(ov9760_sunny_q1v04a_exit_module);
MODULE_DESCRIPTION("ov9760_sunny_q1v04a");
MODULE_LICENSE("GPL v2");
