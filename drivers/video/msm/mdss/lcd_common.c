/*wuchx@k-touch.cn
*Register a misc device for KTCIT
* Function:Getting LCD ID
*/
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include "lcd_common.h"

char lcd_panel_dtsi_name[MAX_DTSI_NAME_SIZE];

/*Adding new panel to lcd_panel_table: modify begin*/
struct panel_info lcd_panel_table[]=
{
	{"no this panel","no this panel"},
	{"lk_dtsi_name_long","lk_dtsi_name_long"},
	{"5.5inch HD truly hx8394","qcom,mdss_dsi_hx8394a_720p_video"},
	{"5Inch HD Truly SSD2080M","qcom,mdss_dsi_ssd2080m_720p_video"},
/*jinxr add for truly qhd lcd*/
#if defined(TY_CONFIG_TRULY_LCD_HX8389B_QHD_SUPPORT)
    {"5inch JDI truly-hx8989b","qcom,mdss_dsi_ty_hx8389b_qhd_video"},
	{"5inch LG truly-hx8989b","qcom,mdss_dsi_ty_hx8389b_trulylg_qhd_video"},
#endif
/*jinxr add for ofilm qhd lcd*/
#if defined(TY_CONFIG_OFILM_LCD_OTM9605A_QHD_SUPPORT)
	{"Ofilm QHD otm9605a","qcom,mdss_dsi_ty_ofilm_otm9605a_qhd_video"},
#endif
//xiangdong add for tianma fwvga panel
#ifdef TY_CONFIG_TIANMA_LCD_FWVGA_OTM8018B_SUPPORT
	{"4.5 FWVGA tianma_otm8018b","qcom,mdss_dsi_ty_tianma_otm8018b_fwvga_video"},
	{"4.5 FWVGA trust_ips_otm8018b","qcom,mdss_dsi_ty_45ips_trust_otm8018b_fwvga_video"},
	{"4.5 FWVGA trust_otm8018b","qcom,mdss_dsi_ty_45inch_trust_otm8018b_fwvga_video"},
	/*TYRD: for holitech 4.5inch fwvga lcd---add by liujie 20140403*/
	{"4.5 FWVGA holitech_otm8018b","qcom,mdss_dsi_ty_holitech_45inch_otm8018b_fwvga_video"},
#endif
//xiangdong add for fwvga panel
#ifdef TY_CONFIG_45INCH_TN_FWVGA_SUPPORT
	{"4.5 FWVGA trust_otm8018b","qcom,mdss_dsi_ty_45inch_trust_otm8018b_fwvga_video"},
	{"4.5 FWVGA ofilm_hx8379a","qcom,mdss_dsi_ty_ofilm_hx8379a_fwvga_video"},	
	{"4.5 FWVGA truly_hx8379a","qcom,mdss_dsi_ty_truly_hx8379a_fwvga_video"},
#endif

/*TYRD wuxh add for 5 inch fwvga LCD on 20131226*/
#ifdef TYQ_5INCH_OFILM_FWVGA_OTM8018B_LCD_SUPPORT
	{"5inch FWVGA Ofilm_otm8018b","qcom,mdss_dsi_ofilm_5inch_otm8018b_fwvga_video"},
	{"5inch FWVGA trust_otm8018b","qcom,mdss_dsi_5inch_trust_otm8018b_fwvga_video"},	
	{"5inch FWVGA truly_hx8379a","qcom,mdss_dsi_ty_5inch_truly_hx8379a_fwvga_video"},
	/*TYRD: for holitech 5inch fwvga lcd---add by liujie 20140320*/
	{"5inch FWVGA holitech_otm8018b","qcom,mdss_dsi_5inch_holitech_otm8018b_fwvga_video"},
#endif
/*lichm add for ofilm qhd lcd*/
#if defined(TY_5_5INCH_OFILM_OTM9605A_QHD_SUPPORT)
	{"5.5 inch Ofilm QHD otm9605a","qcom,mdss_dsi_ty_5_5_inch_ofilm_otm9605a_qhd_video"},
#endif
/*TYRD wuchx add for TRULY 5.5inch QHD lcd on 20140117*/
#if defined(TY_LCD_5_5INCH_TRULY_HX8389B_QHD_SUPPORT)
	{"5.5 QHD  truly-hx8989b","qcom,mdss_dsi_ty_5_5inch_truly_hx8389b_qhd_video"},
#endif
/*TYRD wuchx add for TIANMA 5.5inch QHD lcd on 20140317*/
#if defined(TY_LCD_5_5_TIANMA_HX8389B_QHD_SUPPORT)
	{"5.5 QHD  tianma-hx8389b","qcom,mdss_dsi_ty_5_5_tianma_hx8389b_qhd_video"},
#endif
/*TYRD wuchx add for BOE 5.5inch QHD lcd on 20140317*/
#if defined(TY_LCD_5_5_BOE_NT35517_QHD_SUPPORT)
	{"5.5 QHD  Boe-nt35517","qcom,mdss_dsi_ty_5_5_boe_nt35517_qhd_video"},
#endif
/*TYDRV:chenjp 2014.01.21 add for 5inch ofilm otm1283a 720p lcd support*/
#if defined(TYQ_5INCH_OFILM_OTM1283A_720P_LCD_SUPPORT)
	{"ofilm otm1283a 720p","qcom,mdss_dsi_ofilm_otm1283a_720p_video"},
#endif
/*TYDRV:lichm 20140426 add for 5inch truly hx8394a 720p lcd support*/
#if defined(TYQ_CONFIG_5INCH_TRYLY_HX8394A_HD_LCD_SUPPORT)
	{"hx8394a 5inch hd ty video mode dsi panel","qcom,mdss_dsi_ty_5inch_hx8394a_hd_video"},
#endif
};
/*Adding new panel to lcd_panel_table: modify end*/

static int lcd_common_open(struct inode *inode, struct file *file)
{

	return nonseekable_open(inode, file);
}
static long lcd_common_ioctl(struct file *file, unsigned int cmd,
		   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	long ret = 0;
	int i = 0,index = 0;
	int panel_table_size = 0;

	printk("===>Enter %s:\n",__func__);

	switch (cmd) {
	case LCD_IOCTL_GET_LCD_ID:
		printk("===>Enter %s:LCD_IOCTL_GET_LCD_ID\n",__func__);
		// copy panel_name from lcd_panel_dtsi_name
		panel_table_size = sizeof(lcd_panel_table)/sizeof(struct panel_info);
		for(i = 0; i < panel_table_size; i ++){
			if(!strcmp(lcd_panel_table[i].panel_dtsi_name,lcd_panel_dtsi_name)){
				index = i;
				break;
			}				
		}
		printk("===>LCD_IOCTL_GET_LCD_ID: index = %d,name = %s\n",
				index,lcd_panel_table[index].panel_name);
		if(copy_to_user(argp, lcd_panel_table[index].panel_name,MAX_NAME_SIZE)) {
			printk("===>Enter %s:copy_to_user(%d) failed:\n",__func__,__LINE__);
			ret = -EFAULT;
			break;
		}
		
		break;
	default:
		break;
	}

	return ret;
}
static struct file_operations lcd_common_fops = {
	.owner = THIS_MODULE,
	.open = lcd_common_open,
	.unlocked_ioctl = lcd_common_ioctl,
};
static struct miscdevice lcd_common_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lcd_common_device",
	.fops = &lcd_common_fops,
};

static int __init lcd_common_init(void)
{
	printk("===>Enter %s:\n",__func__);
	misc_register(&lcd_common_device);
	return 0;
}

module_init( lcd_common_init);
