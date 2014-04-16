#ifndef _LCD_COMMON_H_
#define _LCD_COMMON_H_
#include <linux/io.h>
#include <asm/uaccess.h>

#define MAX_NAME_SIZE 40
#define MAX_DTSI_NAME_SIZE 100

#define LCDIO 0x1f
#define LCD_IOCTL_GET_LCD_ID _IOR(LCDIO, 0x02, char[MAX_NAME_SIZE])

struct panel_info{
	char panel_name[MAX_NAME_SIZE];
	char panel_dtsi_name[MAX_DTSI_NAME_SIZE ];
};

extern char lcd_panel_dtsi_name[];
extern struct panel_info lcd_panel_table[];
#endif
