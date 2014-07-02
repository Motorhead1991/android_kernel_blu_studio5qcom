/*
 *
 * FocalTech ft5x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/input/ft5x06_ts.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#define FT_DRIVER_VERSION	0x02

#define FT_META_REGS		3
#define FT_ONE_TCH_LEN		6
#define FT_TCH_LEN(x)		(FT_META_REGS + FT_ONE_TCH_LEN * x)

#define FT_PRESS		0x7F
#define FT_MAX_ID		0x0F
#define FT_TOUCH_X_H_POS	3
#define FT_TOUCH_X_L_POS	4
#define FT_TOUCH_Y_H_POS	5
#define FT_TOUCH_Y_L_POS	6
#define FT_TD_STATUS		2
#define FT_TOUCH_EVENT_POS	3
#define FT_TOUCH_ID_POS		5
#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

/*register address*/
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

/* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	50

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55
#define FT6X06_ID		0x06

#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

#define FT_FW_MIN_SIZE		8
#define FT_FW_MAX_SIZE		32768

/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FT_FW_FILE_MIN_VER(x)	0
#define FT_FW_FILE_SUB_MIN_VER(x) 0

#define FT_FW_CHECK(x)		\
	(((x)->data[(x)->size - 8] ^ (x)->data[(x)->size - 6]) == 0xFF \
	&& (((x)->data[(x)->size - 7] ^ (x)->data[(x)->size - 5]) == 0xFF \
	&& (((x)->data[(x)->size - 3] ^ (x)->data[(x)->size - 4]) == 0xFF)))

#define FT_MAX_TRIES		5
#define FT_RETRY_DLY		20

#define FT_MAX_WR_BUF		10
#define FT_MAX_RD_BUF		2
#define FT_FW_PKT_LEN		128
#define FT_FW_PKT_META_LEN	6
#define FT_FW_PKT_DLY_MS	20
#define FT_FW_LAST_PKT		0x6ffa
#define FT_EARSE_DLY_MS		100
#define FT_55_AA_DLY_NS		5000

#define FT_UPGRADE_LOOP		30
#define FT_CAL_START		0x04
#define FT_CAL_FIN		0x00
#define FT_CAL_STORE		0x05
#define FT_CAL_RETRY		100
#define FT_REG_CAL		0x00
#define FT_CAL_MASK		0x70

#define FT_INFO_MAX_LEN		512

#define FT_BLOADER_SIZE_OFF	12
#define FT_BLOADER_NEW_SIZE	30
#define FT_DATA_LEN_OFF_OLD_FW	8
#define FT_DATA_LEN_OFF_NEW_FW	14
#define FT_FINISHING_PKT_LEN_OLD_FW	6
#define FT_FINISHING_PKT_LEN_NEW_FW	12
#define FT_MAGIC_BLOADER_Z7	0x7bfa
#define FT_MAGIC_BLOADER_LZ4	0x6ffa
#define FT_MAGIC_BLOADER_GZF_30	0x7ff4
#define FT_MAGIC_BLOADER_GZF	0x7bf4

enum {
	FT_BLOADER_VERSION_LZ4 = 0,
	FT_BLOADER_VERSION_Z7 = 1,
	FT_BLOADER_VERSION_GZF = 2,
};

enum {
	FT_FT5336_FAMILY_ID_0x11 = 0x11,
	FT_FT5336_FAMILY_ID_0x12 = 0x12,
	FT_FT5336_FAMILY_ID_0x13 = 0x13,
	FT_FT5336_FAMILY_ID_0x14 = 0x14,
};

#define FT_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FT_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= 0x%x\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FT_DRIVER_VERSION, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)

#define FT_DEBUG_DIR_NAME	"ts_debug"

struct ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct ft5x06_ts_platform_data *pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
};

#ifndef CONFIG_TOUCHSCREEN_GEN_VKEYS
#define TYN_VIRTAUL_KEY_FRAMEWORK
#ifdef TYN_VIRTAUL_KEY_FRAMEWORK
static int fts_creat_virtual_key_sysfs(void);
#endif
#endif
/*TYDRV:liujie remove the macro restriction 20140519*/
unsigned char ft_probe_flag = 0;

#if defined (TYQ_FOCALTECH_TP_CHARGEING_INTERFERENCE)
 char tp_usb_charge_flag = 0;
 char is_tp_resum = 1;
 void  fts_ctpm_charge_mode(char flag);
#endif

#define CONFIG_SUPPORT_FTS_CTP_UPG

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
#define FT_FIVE_FINGERS_SUPPORT
static int touch_ctrl_init(void *touch);

static struct ft5x06_ts_data* g_stpTouchData = NULL;
static char is_upgrading = 0;
#endif

static int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
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
			dev_err(&client->dev, "%s: i2c read error.\n",
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

static int ft5x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return ft5x06_i2c_write(client, buf, sizeof(buf));
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return ft5x06_i2c_read(client, &addr, 1, val, 1);
}
/*TYRD wuxh add for ft firmware compatible(HD,QHD)*/
#ifdef TYQ_FT_HDTOQHD
int tyq_ft_hdqhd_firmware(struct i2c_client *client)
{
	int ret=0;
	ret=ft5x0x_write_reg(client,0xc5,0x1);
	printk("tyq_ft_hdqhd_firmware qhd\r\n");
	return ret;
}
#endif
static void ft5x06_update_fw_ver(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");

	reg_addr = FT_REG_FW_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		dev_err(&client->dev, "fw minor version read failed");

	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		dev_err(&client->dev, "fw sub minor version read failed");

	dev_info(&client->dev, "Firmware version = %d.%d.%d\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

static irqreturn_t ft5x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x06_ts_data *data = dev_id;
	struct input_dev *ip_dev;
	int rc, i;
	u32 id, x, y, status, num_touches;
	u8 reg = 0x00, *buf;
	bool update_input = false;

	if (!data) {
		pr_err("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}

	ip_dev = data->input_dev;
	buf = data->tch_data;

	rc = ft5x06_i2c_read(data->client, &reg, 1,
			buf, data->tch_data_len);
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		return IRQ_HANDLED;
	}

	for (i = 0; i < data->pdata->num_max_touches; i++) {
		id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
		if (id >= FT_MAX_ID)
			break;

		update_input = true;

		x = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
		y = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);

		status = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;

		num_touches = buf[FT_TD_STATUS] & FT_STATUS_NUM_TP_MASK;

		/* invalid combination */
		if (!num_touches && !status && !id)
			break;

		input_mt_slot(ip_dev, id);
		if (status == FT_TOUCH_DOWN || status == FT_TOUCH_CONTACT) {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
		} else {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		}
	}

	if (update_input) {
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}

	return IRQ_HANDLED;
}

static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		regulator_enable(data->vdd);
	}

	return rc;
}

static int ft5x06_power_init(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

#ifdef CONFIG_PM
static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2], i;
	int err;

	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

	disable_irq(data->client->irq);

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		txbuf[0] = FT_REG_PMODE;
		txbuf[1] = FT_PMODE_HIBERNATE;
		ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
	}

	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	} else {
		err = ft5x06_power_on(data, false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	}

	data->suspended = true;
	#if defined (TYQ_FOCALTECH_TP_CHARGEING_INTERFERENCE)
	is_tp_resum = 0;
	#endif
	return 0;

pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	enable_irq(data->client->irq);
	return err;
}

static int ft5x06_ts_resume(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	msleep(data->pdata->soft_rst_dly);
/*TYRD wuxh add for ft firmware compatible(HD,QHD)*/
#ifdef TYQ_FT_HDTOQHD
	tyq_ft_hdqhd_firmware(data->client);
#endif
	#if defined (TYQ_FOCALTECH_TP_CHARGEING_INTERFERENCE)
	is_tp_resum = 1;
	/*TYDRV:add by liujie .only then the flag is 1,then we wirte it*/
	if(tp_usb_charge_flag)
	{
		fts_ctpm_charge_mode(tp_usb_charge_flag);
	}
	#endif
	enable_irq(data->client->irq);

	data->suspended = false;

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);
	#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	if(is_upgrading == 1)
		return 0;
	#endif
	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			ft5x06_ts_resume(&ft5x06_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			ft5x06_ts_suspend(&ft5x06_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_resume(&data->client->dev);
}
#endif

static const struct dev_pm_ops ft5x06_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = ft5x06_ts_suspend,
	.resume = ft5x06_ts_resume,
#endif
};
#endif

static int ft5x06_auto_cal(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
		/*return to normal mode, calibration finish */
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}

static int ft5x06_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 is_5336_new_bootloader = false;
	u8 is_5336_fwsize_30 = false;
	u8 fw_ecc;

	/* determine firmware size */
	if (*(data + data_len - FT_BLOADER_SIZE_OFF) == FT_BLOADER_NEW_SIZE)
		is_5336_fwsize_30 = true;
	else
		is_5336_fwsize_30 = false;

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);
		/* reset - write 0xaa and 0x55 to reset register */
		if (ts_data->family_id == FT6X06_ID)
			reset_reg = FT_RST_CMD_REG2;
		else
			reset_reg = FT_RST_CMD_REG1;

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
		msleep(info.delay_aa);

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(info.delay_55 + i * 3);
		else
			msleep(info.delay_55 - (i - (FT_UPGRADE_LOOP / 2)) * 2);

		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, w_buf, 1);
		usleep(FT_55_AA_DLY_NS);
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, w_buf, 1);

		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1],
				info.upgrade_id_1, info.upgrade_id_2);
		} else
			break;
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
		((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	dev_dbg(&client->dev, "bootloader type=%d, r_buf=0x%x, family_id=0x%x\n",
		is_5336_new_bootloader, r_buf[0], ts_data->family_id);
	/* is_5336_new_bootloader = FT_BLOADER_VERSION_GZF; */

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	if (is_5336_fwsize_30) {
		w_buf[0] = FT_ERASE_PANEL_REG;
		ft5x06_i2c_write(client, w_buf, 1);
	}
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4
		|| is_5336_new_bootloader == FT_BLOADER_VERSION_Z7)
		data_len = data_len - FT_DATA_LEN_OFF_OLD_FW;
	else
		data_len = data_len - FT_DATA_LEN_OFF_NEW_FW;

	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4 ||
		is_5336_new_bootloader == FT_BLOADER_VERSION_Z7) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_OLD_FW; i++) {
			if (is_5336_new_bootloader  == FT_BLOADER_VERSION_Z7)
				temp = FT_MAGIC_BLOADER_Z7 + i;
			else if (is_5336_new_bootloader ==
						FT_BLOADER_VERSION_LZ4)
				temp = FT_MAGIC_BLOADER_LZ4 + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);
		}
	} else if (is_5336_new_bootloader == FT_BLOADER_VERSION_GZF) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_NEW_FW; i++) {
			if (is_5336_fwsize_30)
				temp = FT_MAGIC_BLOADER_GZF_30 + i;
			else
				temp = FT_MAGIC_BLOADER_GZF + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);

		}
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade successful\n");

	return 0;
}

static int ft5x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min;
	bool fw_upgrade = false;

	if (data->suspended) {
		dev_info(dev, "Device is in suspend state: Exit FW upgrade\n");
		return -EBUSY;
	}

	rc = request_firmware(&fw, data->fw_name, dev);
	if (rc < 0) {
		dev_err(dev, "Request firmware failed - %s (%d)\n",
						data->fw_name, rc);
		return rc;
	}

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%d)\n", fw->size);
		rc = -EIO;
		goto rel_fw;
	}

	fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	dev_info(dev, "Current firmware: %d.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: %d.%d.%d", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	if (force)
		fw_upgrade = true;
	else if (data->fw_ver[0] < fw_file_maj)
		fw_upgrade = true;

	if (!fw_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
	if (FT_FW_CHECK(fw)) {
		rc = ft5x06_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
		else if (data->pdata->info.auto_cal)
			ft5x06_auto_cal(data->client);
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}

	ft5x06_update_fw_ver(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
rel_fw:
	release_firmware(fw);
	return rc;
}

static ssize_t ft5x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft5x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_update_fw_store);

static ssize_t ft5x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_force_update_fw_store);

static ssize_t ft5x06_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t ft5x06_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);

static bool ft5x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int ft5x06_debug_data_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_data_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft5x06_debug_data_get,
			ft5x06_debug_data_set, "0x%02llX\n");

static int ft5x06_debug_addr_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	if (ft5x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft5x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft5x06_debug_addr_get,
			ft5x06_debug_addr_set, "0x%02llX\n");

static int ft5x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft5x06_ts_suspend(&data->client->dev);
	else
		ft5x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft5x06_debug_suspend_get,
			ft5x06_debug_suspend_set, "%lld\n");

static int ft5x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft5x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
				struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		#if defined (TYQ_TP_THIRD_MENU_480x800)
		pdata->x_min =0;
		pdata->y_min = 0;
		pdata->x_max = 480;
		pdata->y_max =  800;
		#elif  defined (TYQ_TP_THIRD_MENU_480x854)
		pdata->x_min =0;
		pdata->y_min = 0;
		pdata->x_max = 480;
		pdata->y_max =  854;
		#elif  defined (TYQ_TP_THIRD_MENU_540x960)
		pdata->x_min =0;
		pdata->y_min = 0;
		pdata->x_max = 540;
		pdata->y_max =  960;
		#elif  defined (TYQ_TP_THIRD_MENU_720x1280)
		pdata->x_min =0;
		pdata->y_min = 0;
		pdata->x_max = 720;
		pdata->y_max =  1280;
		#else
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
		#endif
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
						"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
						"focaltech,ignore-id-check");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int ft5x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft5x06_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	g_stpTouchData = data;
#endif
	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
				data->tch_data_len, GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "ft5x06_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, pdata->num_max_touches);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}

	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = ft5x06_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "ft5x06_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "ft5x06_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	/* check the controller id */
	reg_addr = FT_REG_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		goto free_reset_gpio;
	}
	/*TYDRV:liujie remove the macro limit 20140519*/
	ft_probe_flag = 1;
	
	/*TYRD wuxh add for ft firmware compatible(HD,QHD)*/
#ifdef TYQ_FT_HDTOQHD
	tyq_ft_hdqhd_firmware(client);
#endif

	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
		goto free_reset_gpio;
	}

	data->family_id = pdata->family_id;

	err = request_threaded_irq(client->irq, NULL,
				ft5x06_ts_interrupt,
				pdata->irqflags | IRQF_ONESHOT,
				client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_reset_gpio;
	}

	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto irq_free;
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}

	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	data->ts_info = devm_kzalloc(&client->dev,
				FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}

	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	dev_dbg(&client->dev, "touch threshold = %d\n", reg_value * 4);

	ft5x06_update_fw_ver(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft5x06_ts_early_suspend;
	data->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

#ifndef CONFIG_TOUCHSCREEN_GEN_VKEYS
#ifdef TYN_VIRTAUL_KEY_FRAMEWORK
 fts_creat_virtual_key_sysfs();
#endif
#endif

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	touch_ctrl_init(data);
#endif
	return 0;

free_debug_dir:
	debugfs_remove_recursive(data->dir);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
irq_free:
	free_irq(client->irq, data);
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
pwr_off:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
unreg_inputdev:
	input_unregister_device(input_dev);
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);
	return err;
}

static int __devexit ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);

	debugfs_remove_recursive(data->dir);
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	device_remove_file(&client->dev, &dev_attr_fw_name);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft5x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft5x06_power_init(data, false);

	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = __devexit_p(ft5x06_ts_remove),
	.driver = {
		   .name = "ft5x06_ts",
		   .owner = THIS_MODULE,
		.of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
		   .pm = &ft5x06_ts_pm_ops,
#endif
		   },
	.id_table = ft5x06_ts_id,
};

#ifndef CONFIG_TOUCHSCREEN_GEN_VKEYS
#ifdef TYN_VIRTAUL_KEY_FRAMEWORK
///////////////////////////////////////////////////////////////////////

/*
sysfs file format:
key tyep:key value:key center x:key center y:key width:key height

#define TOUCH_Y_MAX 836
#define SCREEN_Y_MAX 800

*/
static ssize_t ty_touch_virtual_keys_show(struct kobject *kobj,
										  struct kobj_attribute *attr, char *buf)
{
#if defined(TYQ_TP_THIRD_MENU_540x960)
	return snprintf(buf, 200,
		__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":120:1010:100:80"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":240:1010:100:80"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":360:1010:100:80" 			   
	   "\n");
#elif defined (TYQ_TP_THIRD_MENU_480x800)		
	return snprintf(buf, 200,
			__stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":240:850:100:80"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":120:850:100:80"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":360:850:100:80"				   
		   "\n");	
#elif defined (TYQ_TP_THIRD_MENU_480x854)		
	return snprintf(buf, 200,
			__stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":240:900:100:80"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":120:900:100:80"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":360:900:100:80"				   
		   "\n");	
	//tydrv pengwei addfor fwvga virtual key
#elif defined (TYQ_TP_FOUR_MENU_480x854)		
	return snprintf(buf, 200,
		__stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)  ":67:1000:135:60"
		":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)	":202:1000:135:60"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":337:1000:135:60"
		":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH)   ":472:1000:135:60"
		"\n");
	
#elif defined(TYQ_TP_THIRD_MENU_720x1280)
#if 0// defined(TYQ_TBW5933_SUPPORT)
	return snprintf(buf, 200,
		__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":90:1007:100:80"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":263:1007:100:80"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":428:1007:100:80" 			   
	   "\n");
#else
	return snprintf(buf, 200,
		__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":120:1343:100:80"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":350:1343:100:80"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":570:1343:100:80" 			   
	   "\n");
#endif
#else
	return snprintf(buf, 200,
		__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":120:1010:100:80"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":240:1010:100:80"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":360:1010:100:80" 			   
	   "\n");
#endif

}

static struct kobj_attribute ty_touch_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ft5x06_ts",
		.mode = S_IRUGO,
	},
	.show = &ty_touch_virtual_keys_show,
};


//**********************************************************************//
static struct attribute *ty_touch_properties_attrs[] = {
	&ty_touch_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group ty_touch_properties_attr_group = {
	.attrs = ty_touch_properties_attrs,
};

//**********************************************************************//

static int fts_creat_virtual_key_sysfs(void)
{
	struct kobject *properties_kobj;
	int ret = 0;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
								 &ty_touch_properties_attr_group);
	if (!properties_kobj || ret)
		printk("failed to create board_properties\n");

	return ret;
}

////////////////////////////////////////////////////////////////////////////


#endif
#endif
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL               0x0
#define FTS_TRUE               0x01
#define FTS_FALSE              0x0
#define CFG_MAX_TOUCH_POINTS	5
#define I2C_CTPM_ADDRESS       0x76
#define FT_DEFAULT_POINTERS     CFG_MAX_TOUCH_POINTS
//interrupt mode setting
#define INTERRUPT_TRIGGER_MODE   1
#define INTERRUPT_QUERY_MODE     0 
#define INTERRUPT_MODE_TRIGGER

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




typedef enum
{
	ERR_OK,
	ERR_MODE,
	ERR_READID,
	ERR_ERASE,
	ERR_STATUS,
	ERR_ECC
}E_UPGRADE_ERR_TYPE;

struct ty_touch_fmupgrade_S
{
  unsigned int	touchType;
  unsigned long ulCRC;
  unsigned long ulReserved;
  unsigned long bufLength;
  void* 		bufAddr;
};

DEFINE_SEMAPHORE(upgrade_lock);
static int g_iINTMode =  INTERRUPT_TRIGGER_MODE;

static int   g_iMaxFingers = CFG_MAX_TOUCH_POINTS;

static int g_bConfigPointer = FTS_FALSE;
static int g_bConfigINTMode = FTS_FALSE;
u8   g_u8UpgradeFWResult = 0;

static int fts_i2c_read(
	struct ft5x06_ts_data* hTouch,
    u8  reg,
    u8* buffer,
    u32 len)
{
	int ret;
	ret = ft5x06_i2c_read(hTouch->client,&reg,1, buffer, len);
	if(ret<0)
		return ret;
	else
	 	return FTS_TRUE;
}

static int fts_i2c_write(
    struct ft5x06_ts_data* hTouch,
    u8  reg,
    u8* buffer,
    u32 len)
{
	int ret;
	ret = ft5x06_i2c_write(hTouch->client, buffer,len);
	if(ret < 0 ) 
		return ret;
	else
	 	return FTS_TRUE;
}

int ty_ft5x0x_write_reg(struct ft5x06_ts_data* hTouch, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_write(hTouch,0,buf,sizeof(buf));
}


int ty_ft5x0x_read_reg(struct ft5x06_ts_data* hTouch, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_read(hTouch,regaddr,regvalue,1);//ft5x0x_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

#if defined (TYQ_FOCALTECH_TP_CHARGEING_INTERFERENCE)
void  fts_ctpm_charge_mode(char flag)
{
	//char value;
	ty_ft5x0x_write_reg(g_stpTouchData,0x8b,flag);
	//ft5x0x_read_reg(g_stpTouchData,0x8b,&value);
	//printk("****************************fts_ctpm_charge_mode flag = %d ***********************\n",flag);
	
}
#endif

int fts_ctpm_auto_clb(struct ft5x06_ts_data* hTouch)
{
	unsigned char uc_temp = 0x00;
	int  i = 0;

	/*start auto CLB */
	msleep(500);

	ty_ft5x0x_write_reg(hTouch, 0, 0x40);
	/*make sure already enter factory mode */
	msleep(300);
	/*write command to start calibration */
	ty_ft5x0x_write_reg(hTouch, 2, 0x4);
	msleep(300);
	for (i = 0; i < 300; i++) {
		ty_ft5x0x_read_reg(hTouch, 0, &uc_temp);
		/*return to normal mode, calibration finish */
		if (0x0 == ((uc_temp & 0x70) >> 4))
		{
			printk("ft calibration finish !\n");
			break;
		}
	}

	//msleep(200);
	/*calibration OK */
	msleep(300);
	ty_ft5x0x_write_reg(hTouch, 0, 0x40);	/*goto factory mode for store */
	msleep(100);	/*make sure already enter factory mode */
	ty_ft5x0x_write_reg(hTouch, 2, 0x5);	/*store CLB result */
	msleep(300);
	ty_ft5x0x_write_reg(hTouch, 0, 0x00);	/*return to normal mode */
	msleep(300);

	/*store CLB result OK */
	return 0;
}

void delay_qt_ms(unsigned long  w_ms)
{


	msleep(w_ms);
	return;
}

#ifdef TP_UPGRADE_DEBUG

#define fts_debug(fmt, args...) \
	do {\
			printk("fts: "fmt, ##args);\
	} while (0)
#else

#define fts_debug(fmt, args...)

#endif 


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
static FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
	
    //ret=fts_i2c_read(g_stpTouchData, pbt_buf[0],&pbt_buf[1], dw_lenth-1);
    ret = i2c_master_recv(g_stpTouchData->client,pbt_buf, dw_lenth);
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
static FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
	//ret = fts_i2c_write(g_stpTouchData, pbt_buf[0],&pbt_buf[1], dw_lenth-1);
	ret = i2c_master_send(g_stpTouchData->client,pbt_buf, dw_lenth);
	if(ret < 0)
	{
		
		printk("%s fail err = %x!!!\n", __func__,  ret);
		return 0;
	}
	
    return 1; 
}


/*
[function]: 
	write a value to register.
[parameters]:
	e_reg_name[in]	:register name;
	pbt_buf[in]		:the returned register value;
[return]:
	FTS_TRUE	:success;
	FTS_FALSE	:io fail;
*/
#if 0
static FTS_BOOL fts_register_write(FTS_BYTE e_reg_name, FTS_BYTE b_data)
{
	FTS_BYTE write_cmd[2] = {0};

	write_cmd[0] = e_reg_name;
	write_cmd[1] = b_data;

	/*call the write callback function*/
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, 2);
	
}
#endif

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
static FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
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
static FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
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
static FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

static FTS_BOOL fts_Upgraded(void)
{
	u8   u8Val;
    u8   u8SleepCommand[2] = {0}; 

	if(!g_stpTouchData)
	{
		printk("%s:g_stpTouchData is null.\n",__func__);
		return 0;
	}

	fts_debug("enter %s.\n",__func__);	
	//NvOsSleepMS(50);
	
#ifdef FT_FIVE_FINGERS_SUPPORT

	mdelay(1);
	u8Val = 0xFE;
	
	//version
	if(!fts_i2c_read(g_stpTouchData, 0xA6, &u8Val, 1))
	{
		fts_debug("%s:get FW ver faild.\n",__func__);
		u8Val = 0x01;
	}
	fts_debug("FT_Upgraded:Version=0x%x \n",u8Val);
	
	if((u8Val >= 0x1a))
	{
		mdelay(1);
		u8Val = 0xFE;
		if( FTS_TRUE == fts_i2c_read(g_stpTouchData, 0x90, &u8Val, 1))
		{
			if(g_iMaxFingers != u8Val)
			{
				mdelay(1);
				g_bConfigPointer = FTS_TRUE;
				
				u8SleepCommand[0] = 0x90;
				u8SleepCommand[1] = g_iMaxFingers;
			    fts_i2c_write(g_stpTouchData, 0x0, u8SleepCommand, 2);
				fts_debug("FT_Upgraded:Fingers=0x%x,fwfingers=0x%x\n",g_iMaxFingers,u8Val);
			}
		}
		else
		{
			fts_debug("%s:get fingers faild.\n",__func__);
			u8Val = FT_DEFAULT_POINTERS;
		}
		fts_debug("FT_Upgraded:Fingers=0x%x \n",g_iMaxFingers);
	}
#endif	
	
	u8Val = 0xFE;
	if(!fts_i2c_read(g_stpTouchData, 0xA4, &u8Val, 1))
	{
		printk("%s:get FW ver faild.\n",__func__);
	}
	fts_debug("fts_Upgraded:IntMode=0x%x \n",u8Val);
	
#ifdef INTERRUPT_MODE_QUERY
	if(u8Val == INTERRUPT_TRIGGER_MODE )
	{
		msleep(50);
		//write
		u8SleepCommand[0] = 0xa4;
		u8SleepCommand[1] = INTERRUPT_QUERY_MODE; //QUERY
	    fts_i2c_write(g_stpTouchData, 0x0, u8SleepCommand, 2);	

		msleep(50);
		
		//read&check
		u8Val = 20;
		fts_i2c_read(g_stpTouchData, 0xa4, &u8Val, 1);
		fts_debug("fts_Upgraded:intmode=0x%x\n",u8Val);
		g_iINTMode = INTERRUPT_QUERY_MODE;
		g_bConfigINTMode = FTS_TRUE;
	}
#endif

#ifdef INTERRUPT_MODE_TRIGGER
	if(u8Val == INTERRUPT_QUERY_MODE )
	{
		msleep(50);

		//write
		u8SleepCommand[0] = 0xa4;
		u8SleepCommand[1] = INTERRUPT_TRIGGER_MODE; 
	    fts_i2c_write(g_stpTouchData, 0x0, u8SleepCommand, 2);	

		msleep(50);
		//read&check
		u8Val = 20;
		fts_i2c_read(g_stpTouchData, 0xa4, &u8Val, 1);
		fts_debug("fts_Upgraded:intmode=0x%x\n",u8Val);
		g_iINTMode = INTERRUPT_TRIGGER_MODE;
		g_bConfigINTMode = FTS_TRUE;
	}
#endif

	return FTS_TRUE;
	
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


#define    TY_FTS_PACKET_LENGTH        32
#if 0
static unsigned char CTPM_FW_FILE[]=
{
//#include "ft_app.i"
};
#endif
#define BL_VERSION_LZ4	0
#define BL_VERSION_Z7	1
#define BL_VERSION_GZF	2 

static E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;
    u8 is_5336_new_bootloader = 0;
    u8 is_5336_fwsize_30 = 0;
   u8 readid=0;
    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp = 0;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[TY_FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_is_new_protocol = 0;
    int      i_ret;
////////////////////////
unsigned char au_delay_timing[44] = {30, 33, 36, 39, 42, 45, 27, 24,21,18,15,20,21,22,23,25,26,28,29,
								31,32,34,35,37,38,40,10,11,12,13,14,16,17,18,19,1,2,3,4,5,6,7,8,9};
    unsigned char jj = 0;
///////////////////////

	if(pbt_buf[dw_lenth-12] == 30)
	{
		is_5336_fwsize_30 = 1;
	}
	else 
	{
		is_5336_fwsize_30 = 0;
	}

	fts_debug("[TSP] :enter %s\n",__func__);
/////////////////
UPG_START:
////////////
	
    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    //fts_register_write(0xfc,0xaa);
/*********Step 1:HardReset  CTPM wang_gj  add begin*****/
	gpio_direction_output(16, 0);
	delay_qt_ms(50);
	gpio_direction_output(16, 1);
/*********Step 1:HardReset  CTPM wang_gj  add end*****/
	//delay_qt_ms(50);
    fts_debug("fts_register_write:%d\n",__LINE__);
     /*write 0x55 to register 0xfc*/
    //fts_register_write(0xfc,0x55);
    fts_debug("[TSP] Step 1: Reset CTPM test\n");
    fts_debug("fts_register_write:%d\n",__LINE__);
    //delay_qt_ms(10);
/*#if defined (TYQ_TBW5925_SUPPORT)   
  //  delay_qt_ms(30); //25~40    30
#else
   delay_qt_ms(20);
#endif
*/
////////////////////////////////////////////////
	delay_qt_ms(au_delay_timing[jj]);
	printk("********jj=%d***********\n",jj);
/////////////////////////////////////////
    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = i2c_write_interface(I2C_CTPM_ADDRESS, auc_i2c_write_buf, 2);
        fts_debug("i=%d ",i);
        delay_qt_ms(5); //25
    }while(i_ret <= 0 && i < 10 );

	fts_debug("\ni=%d\n",i);

    if (i > 1)
    {
        i_is_new_protocol = 1;
    }
	msleep(1);
    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
  //  #if defined (TYQ_TBW5925_SUPPORT)
    if (((reg_val[0] == 0x79) && (reg_val[1] == 0x7 ||reg_val[1] == 0x3 || reg_val[1] == 0x11)) || ((reg_val[0] == 0x79) && (reg_val[1] == 0x8)))
   // #else
  //  if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
   // #endif
    {
    	if((reg_val[0] == 0x79) && (reg_val[1] == 0x11))
			readid = 1;
    	printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
        fts_debug("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
    	/*printk("error:ERR_READID,0x%x,0x%x,%d\n",reg_val[0],reg_val[1],__LINE__);
        return ERR_READID;
	*/
if (jj < 43)
        {
             jj ++;
		printk("***************jj=%d**************\n",jj);
             msleep(200);
             goto UPG_START; 
        }
        else
        {
		printk("error:ERR_READID,0x%x,0x%x,%d\n",reg_val[0],reg_val[1],__LINE__);
            return ERR_READID;
        }
/////////////////////////////////////////
        //i_is_new_protocol = 1;
    }

	auc_i2c_write_buf[0] = 0xcd;
	i2c_write_interface(I2C_CTPM_ADDRESS, auc_i2c_write_buf, 1);
	  byte_read(reg_val,1);
	//ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);

	/*********0705 mshl ********************/
	/*if (reg_val[0] > 4)
		is_5336_new_bootloader = 1;*/

	if (reg_val[0] <= 4)
	{
		is_5336_new_bootloader = BL_VERSION_LZ4 ;
	}
	else if(reg_val[0] == 7)
	{
		is_5336_new_bootloader = BL_VERSION_Z7 ;
	}
	else if(reg_val[0] >= 0x0f)
	{
		is_5336_new_bootloader = BL_VERSION_GZF ;
	}

	printk("****reg_val[0] =%d,is_5336_new_bootloader =%d*****is_5336_fwsize_30=%d**********\n",reg_val[0],is_5336_new_bootloader,is_5336_fwsize_30);
     /*********Step 4:erase app*******************************/
    if (is_5336_fwsize_30 && (readid ==1))
    {
      // auc_i2c_write_buf[0] = 0x61;
		//ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1); /*erase app area*/	
		 cmd_write(0x61,0x00,0x00,0x00,1);
   		 msleep(1500); 

		// auc_i2c_write_buf[0] = 0x63;
		//ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1); /*erase app area*/
		cmd_write(0x63,0x00,0x00,0x00,1);
   		 msleep(50);
    }
    else
    {
        cmd_write(0x61,0x00,0x00,0x00,1);
	 delay_qt_ms(2000);
    }
    //delay_qt_ms(1500);
   
    fts_debug("[TSP] Step 4: erase. \n");
    fts_debug("[TSP] TY_FTS_PACKET_LENGTH = %d\n", TY_FTS_PACKET_LENGTH);

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    fts_debug("[TSP] Step 5: start upgrade. \n");
  if(readid == 0 || is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
  {
   	 dw_lenth = dw_lenth - 8;
  }
  else if(is_5336_new_bootloader == BL_VERSION_GZF) dw_lenth = dw_lenth - 14;
  
    packet_number = (dw_lenth) / TY_FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * TY_FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = TY_FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<TY_FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*TY_FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],TY_FTS_PACKET_LENGTH + 6);
        delay_qt_ms(TY_FTS_PACKET_LENGTH/6 + 8);
        if ((j * TY_FTS_PACKET_LENGTH % 1024) == 0)
        {
              fts_debug("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * TY_FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % TY_FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * TY_FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % TY_FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*TY_FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        delay_qt_ms(20);
    }

    //send the last six byte
    if(readid == 0 || is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
	{
    for (i = 0; i<6; i++)
    {
    	if(readid &&( is_5336_new_bootloader  == BL_VERSION_Z7))
		 temp = 0x7bfa + i;
	else if(readid ==0 || is_5336_new_bootloader == BL_VERSION_LZ4)
        	temp = 0x6ffa + i;
	
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        delay_qt_ms(20);
    }
    }
	else if(is_5336_new_bootloader == BL_VERSION_GZF)
	{
		for (i = 0; i<12; i++)
		{
			if (is_5336_fwsize_30 && readid) 
			{
				temp = 0x7ff4 + i;
			}
			else if (readid) 
			{
				temp = 0x7bf4 + i;
			}
			packet_buf[2] = (u8)(temp>>8);
			packet_buf[3] = (u8)temp;
			temp =1;
			packet_buf[4] = (u8)(temp>>8);
			packet_buf[5] = (u8)temp;
			packet_buf[6] = pbt_buf[ dw_lenth + i]; 
			bt_ecc ^= packet_buf[6];
  			 byte_write(&packet_buf[0],7);  
			//ft5x0x_i2c_Write(client, packet_buf, 7);
			msleep(10);

		}
	}
    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    fts_debug("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

	//added by wuz
	delay_qt_ms(300);
	fts_Upgraded();
	//
    return ERR_OK;
}

/**********************************************************
*CTPM_FW:firmware file,the size less often 26k byte
***********************************************************/
static int fts_ctpm_fw_upgrade_with_i_file(unsigned char* CTPM_FW,int len)
//int fts_ctpm_fw_upgrade_with_i_file()
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
   g_u8UpgradeFWResult = 0;
   
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,len);
   g_u8UpgradeFWResult = i_ret;
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
       printk("upgrade the tp firmware failed.\n");
   }

   return i_ret;
}
#if 0
static int fts_ctpm_fw_upgrade_inline(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
   g_u8UpgradeFWResult = 0;
   
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW_FILE;
   printk("%s:enter,0x%x \n",__func__,*pbt_buf); //wang_gj add *
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW_FILE));
   g_u8UpgradeFWResult = i_ret;
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
       printk("upgrade the tp firmware failed.\n");
   }

   return i_ret;
}

static unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW_FILE);
    if (ui_sz > 2)
    {
        return CTPM_FW_FILE[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}
#endif

/*
* registry the touch as the char device
*/
#define	NV_TOUCH_FT

//Tianyu ADD
#define	TYN_TOUCH_FMUPGRADE 0x6501
#define	TYN_TOUCH_FWVER     (TYN_TOUCH_FMUPGRADE +1 )
#define   TYN_TOUCH_VENDOR   (TYN_TOUCH_FMUPGRADE +2 )
#define TYN_TOUCH_IC_TYPE	(TYN_TOUCH_FMUPGRADE +5)

//tp vendor
#define TYN_TOUCH_FOCALTECH    1
#define TYN_TOUCH_ILITEK	   2
//tp ic type
#define MSTAR_TP_ID	1
#define CYTTSP_TP_ID	0
#define FTS_TP_ID	2//wang_gj
#define MSG_TP_ID	3 
#define GOODIX_TP_ID	4

static struct touch_ctrl_dev
{
  dev_t devno;
  struct cdev cdev;
  struct class *class;
}ty_touch_ctrl;

static int touch_ctrl_open(struct inode *inode, struct file *filp) 
{ 
  printk("%s\n", __func__); 
  return 0;   
} 
 
static ssize_t touch_ctrl_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos) 
{		  
  printk("%s, calibration %s %d\n", __func__, buf, count);	
  
  //NvOdmTouchSetAutoCalibration(touch_context->hTouchDevice, FTS_TRUE);
  return 0;
} 
static ssize_t touch_ctrl_read (struct file *file, char *buf, size_t count, loff_t *offset)
{
  printk("touch_ctrl_read calibration\n"); 
  return 0; 
}

//int (*ioctl) (struct inode *, struct file *, unsigned int, unsigned long);
//struct serial_struct __user *

static int fts_GetFWVer(unsigned long * pu32)
{
	
	int ret;
	if((NULL == g_stpTouchData) || (NULL == pu32))
	{
		printk("%s:g_stpTouchData is null.\n",__func__);
		return 0;
	}
	
	msleep(50);
	ret = fts_i2c_read(g_stpTouchData, 0xA6, (u8*)pu32, 1);//wang_gj add u8*
	if(ret< 0)
	{
		printk("%s:i2c error.\n",__func__);
		return ret;
	}
	msleep(50);

	printk("%s:ver=0x%x.\n",__func__,(unsigned int)(*pu32));//wang_gj modify
	
	return 1;

}

unsigned char FTS_CTPM_GetVendorID_info(unsigned char* ptr_vendor_id)
{
	 FTS_BYTE reg_val[2] = {0};
		FTS_DWRD i = 0;
		FTS_BYTE  auc_i2c_write_buf[10];
		int 	 i_is_new_protocol = 0;
		int 	 i_ret;
	unsigned char au_delay_timing[44] = {30, 33, 36, 39, 42, 45, 27, 24,21,18,15,20,21,22,23,25,26,28,29,
									31,32,34,35,37,38,40,10,11,12,13,14,16,17,18,19,1,2,3,4,5,6,7,8,9};
	unsigned char jj = 0;
		fts_debug("[TSP] :enter %s\n",__func__);
	UPG_START:
		
	/*********Step 1:HardReset	CTPM wang_gj  add begin*****/
		gpio_direction_output(16, 0);
		delay_qt_ms(50);
		gpio_direction_output(16, 1);
	/*********Step 1:HardReset	CTPM wang_gj  add end*****/
		delay_qt_ms(au_delay_timing[jj]);
	
	/////////////////////////////////////////
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = 0x55;
		auc_i2c_write_buf[1] = 0xaa;
		do
		{
			i ++;
			i_ret = i2c_write_interface(I2C_CTPM_ADDRESS, auc_i2c_write_buf, 2);
			fts_debug("i=%d ",i);
			delay_qt_ms(5); //25
		}while(i_ret <= 0 && i < 10 );
	
		fts_debug("\ni=%d\n",i);
	
		if (i > 1)
		{
			i_is_new_protocol = 1;
		}
	
		/*********Step 3:check READ-ID***********************/		  
		cmd_write(0x90,0x00,0x00,0x00,4);
		byte_read(reg_val,2);
	  //  #if defined (TYQ_TBW5925_SUPPORT)
		if (((reg_val[0] == 0x79) && (reg_val[1] == 0x7 ||reg_val[1] == 0x3))||((reg_val[0] == 0x79) && (reg_val[1] == 0x8)) || ((reg_val[0] == 0x79) && (reg_val[1] == 0x11)))
	   // #else
	  //  if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
	   // #endif
		{
			fts_debug("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
		}
		else
		{
			/*printk("error:ERR_READID,0x%x,0x%x,%d\n",reg_val[0],reg_val[1],__LINE__);
			return ERR_READID;
		*/
	//////////////////////////////////////////
	if (jj < 43)
			{
				 jj ++;
			printk("***************jj=%d**************\n",jj);
			printk("error:ERR_READID,0x%x,0x%x,%d\n",reg_val[0],reg_val[1],__LINE__);
				 msleep(200);
				 goto UPG_START; 
			}
			else
			{
			printk("error:ERR_READID,0x%x,0x%x,%d\n",reg_val[0],reg_val[1],__LINE__);
				return ERR_READID;
			}
	/////////////////////////////////////////
			//i_is_new_protocol = 1;
		}
		fts_i2c_read(g_stpTouchData, 0xcd, reg_val, 1);
		printk("bootloader version =0x%x\n",reg_val[0]);
		if(reg_val[1] == 0x11)                    //ft5336 readid : 0x79,0x11 
			cmd_write(0x03,0x00,0x07,0xb4,4);
		else
			cmd_write(0x03,0x00,0x78,0x04,4);
		byte_read(ptr_vendor_id,1);
		auc_i2c_write_buf[0]=0x07;
		byte_write(auc_i2c_write_buf,1);
		msleep(300);
		return 0;

}
static long touch_ctrl_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
	struct ty_touch_fmupgrade_S  bufarg;
	int iret;
	unsigned char *pBuf;
	u8 bRet = 0;
	unsigned long  ulVer;
	u8 vendor = 0;
	//unsigned long i;
	if (down_interruptible(&upgrade_lock)) {
		printk("obtain upgrade_lock faild!\n ");
		return  -EFAULT;
	}
	printk("**********************entery touch_ctrl_ioctl****************************\n");
	if (0 == arg)
	{
		printk("%s:arg null pointer.\n",__func__);
		up(&upgrade_lock);
		return -EFAULT;
	}
	
	if (copy_from_user(&bufarg, (void *) arg, sizeof(bufarg))) 
	{
		printk("%s:null pointer.\n",__func__);
		up(&upgrade_lock);
		return -EFAULT;
	}

	printk("%s:cmd=0x%x.\n",__func__,cmd);
	
	switch (cmd) 
	{
		case TYN_TOUCH_FWVER:
			printk("%s:getver.\n",__func__);
			if (NULL == bufarg.bufAddr)
			{
				printk("%s:bufAddr null pointer.\n",__func__);
				up(&upgrade_lock);
				return -EFAULT;
			}
			printk("%s:getver 1.\n",__func__);
			
			if ((0>= bufarg.bufLength )||( sizeof(unsigned long) < bufarg.bufLength ))
			{
				printk("%s:bufLength null pointer.\n",__func__);
				up(&upgrade_lock);
				return -EFAULT;
			}			
			//printk("%s:lenth=%x,addr=0x%x\n",__func__,(unsigned int)bufarg.bufLength,(bufarg.bufAddr));
			printk("%s:getver 2\n",__func__);
			
			pBuf = kzalloc(bufarg.bufLength, GFP_KERNEL);
			if (0 == pBuf)
			{
				printk("%s:alloc buffer failed.\n",__func__);
				up(&upgrade_lock);
				return 0;
			}

			if (copy_from_user(pBuf, (void *)bufarg.bufAddr, bufarg.bufLength)) 
			{
				printk("%s:get buffer error.\n",__func__);
				up(&upgrade_lock);
				return -EFAULT;
			}
			printk("%s:getver 3.\n",__func__);

			
			ulVer = 0;
			
			//get firmware version
	#if defined(NV_TOUCH_FT)
			bRet = fts_GetFWVer(&ulVer);
			if(0 > bRet) 
			{
				*(unsigned long*)bufarg.bufAddr = 0;
				printk("%s:FT_GetFWVer failed. \n",__func__);
				kfree(pBuf);
				up(&upgrade_lock);
				return -EFAULT;
			}
			
	#endif
			if(ulVer==0)
			{
				printk("*******tp firmware maby be damaged*******\n");
				ulVer = ulVer+0xA6;
			}
			*(( unsigned long *)bufarg.bufAddr) = ulVer;
			
			//printk("%s:ver=0x%x \n",__func__,*(unsigned long*)bufarg.bufAddr);
			if( pBuf ) kfree(pBuf);
			
			break;
		case TYN_TOUCH_IC_TYPE:
			printk("%s:getver.\n",__func__);
			if (NULL == bufarg.bufAddr)
			{
				printk("%s:bufAddr null pointer.\n",__func__);
				up(&upgrade_lock);
				return -EFAULT;
			}
			printk("%s:getver 1.\n",__func__);
			
			if ((0>= bufarg.bufLength )||( sizeof(unsigned long) < bufarg.bufLength ))
			{
				printk("%s:bufLength null pointer.\n",__func__);
				up(&upgrade_lock);
				return -EFAULT;
			}			
			//printk("%s:lenth=%x,addr=0x%x\n",__func__,(unsigned int)bufarg.bufLength,(bufarg.bufAddr));
			printk("%s:get tp ic type 2\n",__func__);
			
			pBuf = kzalloc(bufarg.bufLength, GFP_KERNEL);
			if (0 == pBuf)
			{
				printk("%s:alloc buffer failed.\n",__func__);
				up(&upgrade_lock);
				return 0;
			}

			if (copy_from_user(pBuf, (void *)bufarg.bufAddr, bufarg.bufLength)) 
			{
				printk("%s:get buffer error.\n",__func__);
				up(&upgrade_lock);
				return -EFAULT;
			}

			*(( unsigned long *)bufarg.bufAddr) = FTS_TP_ID;
			
			//printk("%s:ver=0x%x \n",__func__,*(unsigned long*)bufarg.bufAddr);
			if( pBuf ) kfree(pBuf);
			
			break;
		
		case TYN_TOUCH_FMUPGRADE:
			fts_GetFWVer(&ulVer);
			printk("old tp_version = 0x%x\n",(u8)ulVer);
			if (0 == bufarg.bufAddr)
			{
				printk("%s:null pointer.\n",__func__);
				up(&upgrade_lock);
				return 0;
			}
			
			pBuf = kzalloc(bufarg.bufLength, GFP_KERNEL);
			if (0 == pBuf)
			{
				printk("%s:alloc buffer failed.\n",__func__);
				up(&upgrade_lock);
				return 0;
			}
			
			if (copy_from_user(pBuf, (void *) bufarg.bufAddr, bufarg.bufLength)) 
			{
				printk("%s:get buffer error.\n",__func__);
				up(&upgrade_lock);
				return -EAGAIN;
			}

			//get crc and check crc 	  
			//
			
			//printk("%s:type=0x%x,len=0x%x \n",__func__,bufarg.touchType,bufarg.bufLength);
			//printk("ver=0x%x,0x%x,0x%x \n",pBuf[bufarg.bufLength-3],pBuf[bufarg.bufLength-2],pBuf[bufarg.bufLength-1]);
			
     #if defined(NV_TOUCH_FT)
			if( TYN_TOUCH_FOCALTECH == bufarg.touchType)
			{
				#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
					is_upgrading = 1;
				#endif
				iret = fts_ctpm_fw_upgrade_with_i_file(pBuf,bufarg.bufLength);
				#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
					is_upgrading = 0;
				#endif
				if(iret)
				{
					printk("update focaltech firmware failed.\n");
				}
				kfree(pBuf);
				fts_ctpm_auto_clb(g_stpTouchData);
				
				/*TYRD wuxh add for ft firmware compatible(HD,QHD)*/
				#ifdef TYQ_FT_HDTOQHD
				tyq_ft_hdqhd_firmware(g_stpTouchData->client);
				#endif
				
				fts_GetFWVer(&ulVer);
				printk("current tp_version = 0x%x\n",(u8)ulVer);
				up(&upgrade_lock);
				return iret;
			}
			
    #endif
			if( pBuf ) kfree(pBuf);
			break;
		case TYN_TOUCH_VENDOR:
			msleep(200);
			ty_ft5x0x_read_reg(g_stpTouchData,0xa8,&vendor);
			if(vendor == 0xa8 || vendor ==0)
			{
				FTS_CTPM_GetVendorID_info(&vendor);
			}
			printk("pengwei test touch vendor id is %d",vendor);
			if(vendor==TRUlY_VENDOR_ID_NO)
			{
				if(copy_to_user(bufarg.bufAddr,TRUlY_VENDOR_ID_STR,strlen(TRUlY_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(vendor==TRUlY_VENDOR_ID_NO_2)
			{
				if(copy_to_user(bufarg.bufAddr,TRUlY_VENDOR_ID_STR_2,strlen(TRUlY_VENDOR_ID_STR_2)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(vendor==BYD_VENDOR_ID_NO)
			{
				if(copy_to_user(bufarg.bufAddr,BYD_VENDOR_ID_STR,strlen(BYD_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(vendor==O_FILM_VENDOR_ID_NO)
			{
				if(copy_to_user(bufarg.bufAddr,O_FILM_VENDOR_ID_STR,strlen(O_FILM_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(vendor==O_FILM_VENDOR_ID_NO_2)
			{
				if(copy_to_user(bufarg.bufAddr,O_FILM_VENDOR_ID_STR_2,strlen(O_FILM_VENDOR_ID_STR_2)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(vendor==BM_VENDOR_ID_NO)
			{
				if(copy_to_user(bufarg.bufAddr,BM_VENDOR_ID_STR,strlen(BM_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(vendor==LCE_VENDOR_ID_NO)
			{
				if(copy_to_user(bufarg.bufAddr,LCE_VENDOR_ID_STR,strlen(LCE_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else
			{
				if(copy_to_user(bufarg.bufAddr,"unknown_tp",strlen("unknown_tp")))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			break;
		default:
			printk("%s:Invalid command.\n",__func__);
	}
	up(&upgrade_lock);
	return 0;
}

static int touch_ctrl_close(struct inode *inode, struct file *filp) 
{ 
  printk("%s\n", __func__); 
  return 0; 
} 

struct file_operations ty_touch_ctrl_fops = 
{ 
	.write = touch_ctrl_write,
	.read = touch_ctrl_read, 
	.open = touch_ctrl_open, 
	.release = touch_ctrl_close, 
	.unlocked_ioctl	 = touch_ctrl_ioctl,//wang_gj
}; 

static int touch_ctrl_init(void *touch) 
{ 
  int ret = 0; 
  struct device *fts_chr_device;

  ret = alloc_chrdev_region(&ty_touch_ctrl.devno, 0, 1, "touch_ctrl"); 
  if(ret)
  { 
	printk("%s, can't alloc chrdev\n", __func__); 
  } 
  printk("%s, register chrdev(%d, %d)\n", __func__, MAJOR(ty_touch_ctrl.devno), MINOR(ty_touch_ctrl.devno)); 
		 
  cdev_init(&ty_touch_ctrl.cdev, &ty_touch_ctrl_fops);
 
  ty_touch_ctrl.cdev.owner = THIS_MODULE; 
  ty_touch_ctrl.cdev.ops = &ty_touch_ctrl_fops;
  
  ret = cdev_add(&ty_touch_ctrl.cdev, ty_touch_ctrl.devno, 1); 
  if(ret < 0)
  { 
	printk("%s, add char devive error, ret %d\n", __func__, ret); 
  } 

  ty_touch_ctrl.class = class_create(THIS_MODULE, "fts_touch_ctrl"); 
  if(IS_ERR(ty_touch_ctrl.class)){ 
	printk("%s, creating class error\n", __func__); 
  } 

  fts_chr_device=device_create(ty_touch_ctrl.class, NULL, ty_touch_ctrl.devno, NULL, "fts_fw_entry");
  if(NULL == fts_chr_device){
	printk("%s, create device, error\n", __func__);
	class_destroy(ty_touch_ctrl.class);
  }

  return (int)touch; 
} 

#endif
static int __init ft5x06_ts_init(void)
{
	return i2c_add_driver(&ft5x06_ts_driver);
}
module_init(ft5x06_ts_init);

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");
