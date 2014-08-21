
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>

#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <mach/gpio.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/input.h>
#include <linux/semaphore.h>

#define TY_PMU_CTL	1
#if	defined  TY_PMU_CTL
#include <linux/regulator/consumer.h>
#endif
#define MSG_2133_AND_2133A
static unsigned char chip_id;
static unsigned char is_firmware_ok =1;
static unsigned int msg_vendor_id =0;
static bool is_suspended = false;
#if defined(TYQ_MSG_TP_I2C_SPEED_100K)
extern int ty_chage_i2c_speed_rate(struct i2c_adapter *adap,int speed);
#endif
//-------------------------------------------------------------------
//-------------------------------------------------------------------
#define TYQ_TP_ANDROID4_0_SUPPORT

#if defined(CONFIG_TOUCHSCREEN_FT5X06) && defined(CONFIG_TY_TOUCHSCREEN_MSTAR) 
extern unsigned char ft_probe_flag;
#endif
#define TYN_VIRTAUL_KEY_FRAMEWORK     //add by gaohw
#define u8         unsigned char
#define u32        unsigned int
#define s32        signed int

#define REPORT_PACKET_LENGTH    (8)
#define TY_TP_INT 17
#define TY_TP_RST 16

#define MSG21XX_INT_GPIO       (TY_TP_INT)         //gaohw
#define MSG21XX_RESET_GPIO     (TY_TP_RST)    //gaohw

#define MS_TS_MSG21XX_X_MAX   (480)
#define MS_TS_MSG21XX_Y_MAX   (854)

/*#define	TOUCH_KEY_HOME	      (102)
#define	TOUCH_KEY_MENU        (139)
#define	TOUCH_KEY_BACK        (158)
*/



//Tianyu ADD
#define	TYN_TOUCH_FMUPGRADE 0x6501
#define	TYN_TOUCH_FWVER     (TYN_TOUCH_FMUPGRADE +1 )
#define   TYN_TOUCH_VENDOR   (TYN_TOUCH_FMUPGRADE +2 )
#define   TYN_TOUCH_CHIPID   (TYN_TOUCH_FMUPGRADE +3 )
#define TYN_TOUCH_IC_TYPE	(TYN_TOUCH_FMUPGRADE +5)

#define	NV_TOUCH_FT
#define TYN_TOUCH_FOCALTECH    1

//tp ic type
#define MSTAR_TP_ID	1
#define CYTTSP_TP_ID	0
#define FTS_TP_ID	2//wang_gj
#define MSG_TP_ID	3 
#define GOODIX_TP_ID	4


#define TRUlY_VENDOR_ID_NO		0x01
#define TRUlY_VENDOR_ID_STR		"TRULY_TP"
#define BYD_VENDOR_ID_NO		0x02
#define BYD_VENDOR_ID_STR		"BYD_TP"
#define O_FILM_VENDOR_ID_NO		0x09
#define O_FILM_VENDOR_ID_STR		"O-FILM_TP"
#define TRUST_VENDOR_ID_NO		0x0d
#define TRUST_VENDOR_ID_STR		"TRUST"
#define HOLITECH_VENDOR_ID_NO		0x0A
#define HOLITECH_VENDOR_ID_STR		"HOLITECH_TP"

struct ty_touch_fmupgrade_S
{
  unsigned int	touchType;
  unsigned long ulCRC;
  unsigned long ulReserved;
  unsigned long bufLength;
  void* 		bufAddr;
};
static struct touch_ctrl_dev
{
  dev_t devno;
  struct cdev cdev;
  struct class *class;
}ty_touch_ctrl;




static int msg21xx_irq = 0;
static struct i2c_client     *msg21xx_i2c_client;
static struct work_struct    msg21xx_wq;


#if defined(CONFIG_FB)
struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static struct early_suspend  early_suspend;
#endif
static struct input_dev     *input = NULL;
/*********start::add by gaohw***********/
struct mstar_tp_irq_status {
	volatile unsigned long irqen;
};
static struct mstar_tp_irq_status mstar_tp_status;
/**********end:: add by gaohw********/
struct msg21xx_ts_data
{
	uint16_t            addr;
	struct i2c_client  *client;
	struct input_dev   *input_dev;
	int                 use_irq;
	struct work_struct  work;
	int (*power)(int on);
	int (*get_int_status)(void);
	void (*reset_ic)(void);
};


#define MAX_TOUCH_FINGER 2
typedef struct
{
    u16 X;
    u16 Y;
} TouchPoint_t;

typedef struct
{
    u8 nTouchKeyMode;
    u8 nTouchKeyCode;
    u8 nFingerNum;
    TouchPoint_t Point[MAX_TOUCH_FINGER];
} TouchScreenInfo_t;

//---------------------------------------------------------------------------
#define	__FIRMWARE_UPDATE__
#ifdef __FIRMWARE_UPDATE__
/*#if defined(TYQ_TBW5913B_SUPPORT)
#define AUTO_UPDATE
#endif
*/
#ifdef AUTO_UPDATE
static struct work_struct    msg21xx_wq_gao;
#endif

#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)
#define TP_DEBUG	printk
static  char *fw_version;

static unsigned short major_version;
static unsigned short minor_version;
#ifdef AUTO_UPDATE
static unsigned short fm_major_version;
static unsigned short fm_minor_version;
#endif
static u8 temp[94][1024];
static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;

static int touch_ctrl_init(void *touch);

#if defined TY_PMU_CTL
struct regulator *vdd;
struct regulator *vcc_i2c;

#define MSTAR_VTG_MIN_UV	2800000
#define MSTAR_VTG_MAX_UV	3300000
#define MSTAR_I2C_VTG_MIN_UV	1800000
#define MSTAR_I2C_VTG_MAX_UV	1800000
#define MSTAR_VDD_LOAD_MIN_UA	0
#define MSTAR_VDD_LOAD_MAX_UA	10000
#define MSTAR_VIO_LOAD_MIN_UA	0
#define MSTAR_VIO_LOAD_MAX_UA	10000


/**
 * mstar_power_on - Turn device power ON
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int mstar_power_on(void)
{
	
	int rc;
	
		rc = regulator_enable( vdd);
		if (rc) {
			dev_err(&msg21xx_i2c_client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
	
		rc = regulator_enable( vcc_i2c);
		if (rc) {
			dev_err(& msg21xx_i2c_client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			regulator_disable( vdd);
		}
	
		return rc;
}
#if 1
/**
 * mstar_power_off - Turn device power OFF
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int mstar_power_off(void)
{
	int rc;
	rc = regulator_disable( vdd);
	if (rc) {
		dev_err(& msg21xx_i2c_client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable( vcc_i2c);
	if (rc) {
		dev_err(&msg21xx_i2c_client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		regulator_enable( vdd);
	}

	return rc;
}
#endif

/**
 * mstar_power_init - Initialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int mstar_power_init(void)
{
	int rc;


	vdd = regulator_get(&msg21xx_i2c_client->dev, "vdd");
	if (IS_ERR(vdd)) {
		rc = PTR_ERR(vdd);
		dev_err(&msg21xx_i2c_client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vdd) > 0) {
		rc = regulator_set_voltage(vdd, MSTAR_VTG_MIN_UV,
					   MSTAR_VTG_MAX_UV);
		if (rc) {
			dev_err(&msg21xx_i2c_client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	vcc_i2c = regulator_get(&msg21xx_i2c_client->dev, "vcc_i2c");
	if (IS_ERR(vcc_i2c)) {
		rc = PTR_ERR(vcc_i2c);
		dev_err(&msg21xx_i2c_client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(vcc_i2c) > 0) {
		rc = regulator_set_voltage(vcc_i2c, MSTAR_I2C_VTG_MIN_UV,
					   MSTAR_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&msg21xx_i2c_client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(vdd) > 0)
		regulator_set_voltage(vdd, 0, MSTAR_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(vdd);
	return rc;
}
#if 1
/**
 * mstar_power_deinit - Deinitialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int mstar_power_deinit(void)
{
	if (regulator_count_voltages(vdd) > 0)
		regulator_set_voltage(vdd, 0, MSTAR_VTG_MAX_UV);

	regulator_put(vdd);

	if (regulator_count_voltages(vcc_i2c) > 0)
		regulator_set_voltage(vcc_i2c, 0, MSTAR_I2C_VTG_MAX_UV);

	regulator_put(vcc_i2c);
	return 0;
}
#endif
#endif
static void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u16 size)
{
   //according to your platform.
   	int rc;

	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = read_data,
		},
	};

	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	if( rc < 0 )
    {
		printk("HalTscrCReadI2CSeq error %d\n", rc);
	}
}

static void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    //according to your platform.
   	int rc;

	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};
	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	if( rc < 0 )
    {
		printk("HalTscrCDevWriteI2CSeq error %d,addr = %d\n", rc,addr);
	}
}

static void dbbusDWIICEnterSerialDebugMode(void)
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 5);
}

static void dbbusDWIICStopMCU(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICUseBus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICReshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICNotUseBus(void)
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICNotStopMCU(void)
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICExitSerialDebugMode(void)
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);

    // Delay some interval to guard the next transaction
    //udelay ( 200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode(void)
{
    u8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };

    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);
}

static u8 drvISP_Read(u8 n, u8* pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
    u8 Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0};
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &Read_cmd, 1);
    udelay(100);        // delay about 100us*****
    if (n == 1)
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
        *pDataToRead = dbbus_rx_data[0];
    }
    else
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, pDataToRead, n);
    }

    return 0;
}

static void drvISP_WriteEnable(void)
{
    u8 bWriteData[2] =
    {
        0x10, 0x06
    };
    u8 bWriteData1 = 0x12;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
}


static void drvISP_ExitIspMode(void)
{
    u8 bWriteData = 0x24;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 1);
}

static u8 drvISP_ReadStatus(void)
{
    u8 bReadData = 0;
    u8 bWriteData[2] =
    {
        0x10, 0x05
    };
    u8 bWriteData1 = 0x12;

    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    udelay(100);        // delay about 100us*****
    drvISP_Read(1, &bReadData);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    return bReadData;
}


static void drvISP_ChipErase(void)
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
		u32 timeOutCount=0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 3);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
		udelay(100);        // delay about 100us*****
    timeOutCount=0;
		while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
		{
		   timeOutCount++;
	     if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	  }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

  	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
		udelay(100);        // delay about 100us*****
		timeOutCount=0;
		while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
		{
		   timeOutCount++;
	     if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
	  }
}

static void drvISP_Program(u16 k, u8* pDataToWrite)
{
    u16 i = 0;
    u16 j = 0;
    //u16 n = 0;
    u8 TX_data[133];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
		u32 timeOutCount=0;
    for (j = 0; j < 8; j++)   //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);
        for (i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
        udelay(100);        // delay about 100us*****
       
        timeOutCount=0;
				while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
				{
		   			timeOutCount++;
	     			if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	  		}
        
        
        
        drvISP_WriteEnable();
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, TX_data, 133);   //write 133 byte per cycle
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    }
}


static void drvISP_Verify(u16 k, u8* pDataToVerify)
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index=0;
    u32 timeOutCount;
    for (j = 0; j < 8; j++)   //128*8 cycle
    {
        bWriteData[2] = (u8)((addr + j * 128) >> 16);
        bWriteData[3] = (u8)((addr + j * 128) >> 8);
        bWriteData[4] = (u8)(addr + j * 128);
        udelay(100);        // delay about 100us*****
        
        
        timeOutCount=0;
				while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
				{
		   		timeOutCount++;
	     		if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	  		}
        
        
        
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
        udelay(100);        // delay about 100us*****
        drvISP_Read(128, RX_data);
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
        for (i = 0; i < 128; i++)   //log out if verify error
        {
        if((RX_data[i]!=0)&&index<10)
		{
        //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
        index++;
		}
            if (RX_data[i] != pDataToVerify[128 * j + i])
            {
                TP_DEBUG("k=%d,j=%d,i=%d===============Update Firmware Error================",k,j,i);
            }
        }
    }
}

static ssize_t firmware_update_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", fw_version);
}

static void _HalTscrHWReset(void)
{
	// gpio_request(MSG21XX_RESET_GPIO, "reset");           //add by gaohw
	  gpio_direction_output(MSG21XX_RESET_GPIO, 1);
	  gpio_set_value(MSG21XX_RESET_GPIO, 1);
	  gpio_set_value(MSG21XX_RESET_GPIO, 0);
	    mdelay(10);  /* Note that the RST must be in LOW 10ms at least */
	  gpio_set_value(MSG21XX_RESET_GPIO, 1);
    /* Enable the interrupt service thread/routine for INT after 50ms */
   	 mdelay(300);
//	gpio_free(MSG21XX_RESET_GPIO);
}
static u8 get_chip_id(void)
{
  u8 dbbus_tx_data[4];
   unsigned char dbbus_rx_data[2] = {0};
 //  disable_irq(msg21xx_irq);

 _HalTscrHWReset();
   // Erase TP Flash first
   dbbusDWIICEnterSerialDebugMode();
   dbbusDWIICStopMCU();
   dbbusDWIICIICUseBus();
   dbbusDWIICIICReshape();
   mdelay ( 300 );

   // Disable the Watchdog
   dbbus_tx_data[0] = 0x10;
   dbbus_tx_data[1] = 0x3C;
   dbbus_tx_data[2] = 0x60;
   dbbus_tx_data[3] = 0x55;
   HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
   dbbus_tx_data[0] = 0x10;
   dbbus_tx_data[1] = 0x3C;
   dbbus_tx_data[2] = 0x61;
   dbbus_tx_data[3] = 0xAA;
   HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
   // Stop MCU
   #ifdef doubleeee
   dbbus_tx_data[0] = 0x10;
   dbbus_tx_data[1] = 0x3C;
   dbbus_tx_data[2] = 0xE4;
   HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
   #else
   dbbus_tx_data[0] = 0x10;
   dbbus_tx_data[1] = 0x0F;
   dbbus_tx_data[2] = 0xE6;
   dbbus_tx_data[3] = 0x01;
   HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
   #endif
   /////////////////////////
   // Difference between C2 and C3
   /////////////////////////
   // c2:2133 c32:2133a(2) c33:2138
   //check id
   dbbus_tx_data[0] = 0x10;
   dbbus_tx_data[1] = 0x1E;
   dbbus_tx_data[2] = 0xCC;
   HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
   HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
   _HalTscrHWReset();
//    enable_irq(msg21xx_irq);
   return  dbbus_rx_data[0] ;
 
}
#ifdef MSG_2133_AND_2133A
u32 crc_tab[256];
static u8 g_dwiic_info_data[1024];   // Buffer for info data

static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
    ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    return ulCRC ;
}
static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

typedef enum
{
    EMEM_ALL = 0,
    EMEM_MAIN,
    EMEM_INFO,
} EMEM_TYPE_t;

static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 4 );
}
static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 5 );
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

/*
static int drvTP_erase_emem_c32 ( void )
{
    /////////////////////////
    //Erase  all
    /////////////////////////
    
    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}
*/
static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}
/*
static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, u8 *p, size_t set_pce_high )
{
    u32 i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}

*/
#if 1
static int drvTP_read_info_dwiic_c33 ( void )
{
    //u8 i;
    u8  dwiic_tx_data[5];
  //  u8  dwiic_rx_data[4];
    u16 reg_data=0;
    mdelay ( 300 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

    drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
    mdelay ( 1 );
    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );   
    mdelay ( 100 );

    do{
		printk("*********gaohw::reg_data != 0x5B58********\n");
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );
	printk("( reg_data != 0x5B58 );\n");

	#if 0
    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );

    mdelay ( 50 );

    // recive info data
    #if 0
    for (i=0;i<256;i++)
    {
    	mdelay(20);
	printk("i=%d\n",i);
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[i*4], 4 );
    	}
	#endif
	
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 8 );

	#else
	dwiic_tx_data[0] = 0x72;

   // dwiic_tx_data[3] = 0x04;
  //  dwiic_tx_data[4] = 0x00;	
    dwiic_tx_data[3] = 0x00;
    dwiic_tx_data[4] = 0x08;	

    for(reg_data=0;reg_data<128;reg_data++)
    {
    	dwiic_tx_data[1] = (((reg_data*8)&0xff00)>>8)+0x80;           // address High
    	dwiic_tx_data[2] = (reg_data*8)&0x00ff;             // address low

    	HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );
	
    	mdelay (50 );

    // recive info data
    	HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[reg_data*8], 8);
    }
	#endif
    return ( 1 );
}
#endif
static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )
{
  //  u8  dbbus_tx_data[4];
    //u8  dbbus_rx_data[2] = {0};
    u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp,crc_temp;
    u32 crc_info, crc_info_tp;
  
    int update_pass = 1;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    drvTP_read_info_dwiic_c33();
    printk("drvTP_read_info_dwiic_c33\n");
   if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        //drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

        g_dwiic_info_data[8]=temp[32][8];
        g_dwiic_info_data[9]=temp[32][9];
        g_dwiic_info_data[10]=temp[32][10];
        g_dwiic_info_data[11]=temp[32][11];
        // updata life counter
        life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
        life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
        g_dwiic_info_data[12]=life_counter[0];
        g_dwiic_info_data[13]=life_counter[1];
        //drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
        drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

        mdelay ( 1500 );
        printk("TP SW reset still busy ******************\n");
        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );

        }
        while ( reg_data != 0x2F43 );

       printk("2F43 still busy ******************\n");
        // transmit lk info data
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &g_dwiic_info_data[0], 1024 );
	printk("transmit lk info data\n");
	mdelay(100);
        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
	printk("0xD0BC still busy ******************\n");

    }

    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
	printk("erase main\n");
    mdelay ( 1000 );

    //ResetSlave();
   _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }
  printk("Program\n");
    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }

    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );
	printk("polling 0x3CE4 is 0x2F43\n");
    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( emem_type == EMEM_INFO )
            i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }

		    crc_temp=crc_main;
		  crc_temp=crc_temp ^ 0xffffffff;

		  for(j=0;j<4;j++)
		  {
		  /**************************modify by sam 20131224**************************/
		  	temp[i][1023-j]=(crc_temp>>8*j)&0xFF;
		  /**************************end of modify***********************************/
			printk("Upate crc32 into bin buffer temp[%d][%d]=%x\n",i,(1020+j),temp[i][1020+j]);
		  }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }
    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();

    update_pass = 1;
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }

    if ( !update_pass )
    {
        printk ( "update FAILED\n" );
      _HalTscrHWReset();
        FwDataCnt = 0;
     //   enable_irq(msg21xx_irq);
        return ( 0 );
    }

    printk ( "update OK\n" );
   _HalTscrHWReset();
    FwDataCnt = 0;
    //enable_irq(msg21xx_irq);
    return size;
}

static ssize_t firmware_update_c2 ( struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    TP_DEBUG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
  // enable_irq(msg21xx_irq);

    return size;
}

static u32 _CalMainCRC32(void)
{
    u32 ret=0;
    u16  reg_data=0;

    _HalTscrHWReset();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 100 );

    //Stop MCU
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );
	
    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );
  
    //cmd
    drvDB_WriteReg ( 0x3C, 0xE4, 0xDF4C ); 
    drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

    //MCU run 	
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0000 );

    //polling 0x3CE4	
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
		printk(" reg_data != 0x9432\n");
    }while ( reg_data != 0x9432 );
        
    // Cal CRC Main from TP
    ret = drvDB_ReadReg ( 0x3C, 0x80 );
	printk("**************drvDB_ReadReg\n");
    ret = ( ret << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );
    
    printk("[21xxA]:Current main crc32=0x%x\n",ret);
    return (ret);
}

static u32 _ReadBinConfig ( void )
{
    u8 i;
    u32 ret=0;
    u8  dbbus_tx_data[5]={0};
    u8  dbbus_rx_data[4]={0};
    u16 reg_data=0;

    _HalTscrHWReset();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 100 );
	
    //Stop MCU
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );
	
    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    //cmd	
    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );
    drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

    //MCU run	
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0000 );

   //polling 0x3CE4
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );

    dbbus_tx_data[0] = 0x72;
    dbbus_tx_data[1] = 0x7F;
    dbbus_tx_data[2] = 0xFC;
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &dbbus_tx_data[0], 5 );
    // recive info data
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4 );

    for(i=0;i<4;i++)
    {
        printk("[21xxA]:Bin Config dbbus_rx_data[%d]=%x\n",i,dbbus_rx_data[i]);
    }
/**************************modify by sam 20131224**************************/
    ret=dbbus_rx_data[0];
    ret=(ret<<8)|dbbus_rx_data[1];
    ret=(ret<<8)|dbbus_rx_data[2];
    ret=(ret<<8)|dbbus_rx_data[3];
/**************************end of modify************************************/
    printk("[21xxA]:crc32 from bin config=%x\n",ret);
    return (ret);
}

static u16 _GetVendorID ( EMEM_TYPE_t emem_type )
{
    u8 i;
    u16 ret=0;
    u8  dbbus_tx_data[5]={0};
    u8  dbbus_rx_data[4]={0};
    u16 reg_data=0;
    u16 addr_id;

    _HalTscrHWReset();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 100 );
	
    //Stop MCU
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );
	
    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    //cmd	
    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );
    drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

    //MCU run	
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0000 );

   //polling 0x3CE4
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );

    if(emem_type == EMEM_MAIN)
    {
        addr_id = 0x7F55;
    }
    else if(emem_type == EMEM_INFO)
    {
        addr_id = 0x830D;
    }

    dbbus_tx_data[0] = 0x72;
    dbbus_tx_data[1] = (addr_id>>8)&0xFF;
    dbbus_tx_data[2] = addr_id&0xFF;
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &dbbus_tx_data[0], 5 );
    // recive info data
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4 );

    for(i=0;i<4;i++)
    {
        printk("[21xxA]:Vendor id dbbus_rx_data[%d]=0x%x,%d\n",i,dbbus_rx_data[i],(dbbus_rx_data[i]-0x30));
    }

#if(0)
    ret=dbbus_rx_data[3];
    ret=(ret<<8)|dbbus_rx_data[2];
    ret=(ret<<8)|dbbus_rx_data[1];
    ret=(ret<<8)|dbbus_rx_data[0];
#endif
	
    if((dbbus_rx_data[0]>=0x30 && dbbus_rx_data[0]<=0x39)
    &&(dbbus_rx_data[1]>=0x30 && dbbus_rx_data[1]<=0x39)
    &&(dbbus_rx_data[2]>=0x30 && dbbus_rx_data[2]<=0x39))  
    {
    	ret=(dbbus_rx_data[0]-0x30)*100+(dbbus_rx_data[1]-0x30)*10+(dbbus_rx_data[2]-0x30);
    }


    if(emem_type == EMEM_MAIN)
    {
        printk("[21xxA]:Vendor id from main block=%d\n",ret);
    }
    else if(emem_type == EMEM_INFO)
    {
        printk("[21xxA]:Vendor id from info. block=%d\n",ret);
    }
	
    return (ret);
}

static u8 _CheckFwIntegrity(void)
{
    u8 ret=0;
    u32 cal_crc32, bin_conf_crc32;

    cal_crc32=_CalMainCRC32();
    bin_conf_crc32=_ReadBinConfig();

    if((cal_crc32==bin_conf_crc32))
    {
         ret=1;
    }

    return (ret);
}

static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
   // u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    //disable_irq(msg21xx_irq);

  _HalTscrHWReset();

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    #ifdef doubleeee
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0xE4;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    #else
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    #endif
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
    // c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    if ( dbbus_rx_data[0] == 2 )
    {
       		printk("***chip id =0x%x\n",dbbus_rx_data[0]);
                return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
           
    }
    else
    {
      		  printk("chip id =0x%x\n",dbbus_rx_data[0]);
            return firmware_update_c2 ( dev, attr, buf, size );
      
    } 
}

#else
static ssize_t firmware_update_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};


		_HalTscrHWReset();
		//1.Erase TP Flash first
	  dbbusDWIICEnterSerialDebugMode();
		dbbusDWIICStopMCU();
		dbbusDWIICIICUseBus();
		dbbusDWIICIICReshape();
		mdelay(300);
		
				
		// Disable the Watchdog
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x3C;
		dbbus_tx_data[2] = 0x60;
		dbbus_tx_data[3] = 0x55;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x3C;
		dbbus_tx_data[2] = 0x61;
		dbbus_tx_data[3] = 0xAA;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	  //Stop MCU
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x0F;
		dbbus_tx_data[2] = 0xE6;
		dbbus_tx_data[3] = 0x01;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	  //set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);



		//set MCU clock,SPI clock =FRO
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x22;
		dbbus_tx_data[2] = 0x00;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
		
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x23;
		dbbus_tx_data[2] = 0x00;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

		
	  // Enable slave's ISP ECO mode
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x08;
		dbbus_tx_data[2] = 0x0c;
		dbbus_tx_data[3] = 0x08;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	  //Enable SPI Pad
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x02;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
		HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
		TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
		dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	  //WP overwrite
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x0E;
		dbbus_tx_data[3] = 0x02;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	  //set pin high
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x10;
		dbbus_tx_data[3] = 0x08;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

		dbbusDWIICIICNotUseBus();
		dbbusDWIICNotStopMCU();
		dbbusDWIICExitSerialDebugMode();
		
		
		
    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay(300);
    
    //2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
		dbbusDWIICStopMCU();
		dbbusDWIICIICUseBus();
		dbbusDWIICIICReshape();


		
		// Disable the Watchdog
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x3C;
		dbbus_tx_data[2] = 0x60;
		dbbus_tx_data[3] = 0x55;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x3C;
		dbbus_tx_data[2] = 0x61;
		dbbus_tx_data[3] = 0xAA;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	  //Stop MCU
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x0F;
		dbbus_tx_data[2] = 0xE6;
		dbbus_tx_data[3] = 0x01;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	  //set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);



		//set MCU clock,SPI clock =FRO
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x22;
		dbbus_tx_data[2] = 0x00;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
		
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x23;
		dbbus_tx_data[2] = 0x00;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	  // Enable slave's ISP ECO mode
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x08;
		dbbus_tx_data[2] = 0x0c;
		dbbus_tx_data[3] = 0x08;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
		
	  //Enable SPI Pad
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x02;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
		HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
		TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
		dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	  //WP overwrite
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x0E;
		dbbus_tx_data[3] = 0x02;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	  //set pin high
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x10;
		dbbus_tx_data[3] = 0x08;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

		dbbusDWIICIICNotUseBus();
		dbbusDWIICNotStopMCU();
		dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for (i = 0; i < 94; i++)   // total  94 KB : 1 byte per R/W
    {
        drvISP_Program(i, temp[i]);    // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
		TP_DEBUG("update OK\n");
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    return size;
}
#endif
static DEVICE_ATTR(update, 0744, firmware_update_show, firmware_update_store);


/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    u16 k=0,i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
	u32 addr = 0;
		u32 timeOutCount=0;
	for (k = 0; k < 94; i++)   // total  94 KB : 1 byte per R/W
   {
      addr = k * 1024;
    for (j = 0; j < 8; j++)   //128*8 cycle
    {
        bWriteData[2] = (u8)((addr + j * 128) >> 16);
        bWriteData[3] = (u8)((addr + j * 128) >> 8);
        bWriteData[4] = (u8)(addr + j * 128);
        udelay(100);        // delay about 100us*****

        timeOutCount=0;
				while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
				{
		   		timeOutCount++;
	     		if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	  		}
        
        
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
        udelay(100);        // delay about 100us*****
        drvISP_Read(128, RX_data);
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
        for (i = 0; i < 128; i++)   //log out if verify error
        {
            if (RX_data[i] != 0xFF)
            {
                TP_DEBUG("k=%d,j=%d,i=%d===============erase not clean================",k,j,i);
            }
        }
    }
	}
   TP_DEBUG("read finish\n");
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{

		u8 dbbus_tx_data[4];
		unsigned char dbbus_rx_data[2] = {0};

		_HalTscrHWReset();
		dbbusDWIICEnterSerialDebugMode();
		dbbusDWIICStopMCU();
		dbbusDWIICIICUseBus();
		dbbusDWIICIICReshape();


		
		// Disable the Watchdog
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x3C;
		dbbus_tx_data[2] = 0x60;
		dbbus_tx_data[3] = 0x55;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x3C;
		dbbus_tx_data[2] = 0x61;
		dbbus_tx_data[3] = 0xAA;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	  //Stop MCU
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x0F;
		dbbus_tx_data[2] = 0xE6;
		dbbus_tx_data[3] = 0x01;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	  //set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);



		//set MCU clock,SPI clock =FRO
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x22;
		dbbus_tx_data[2] = 0x00;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
		
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x23;
		dbbus_tx_data[2] = 0x00;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

		
	  // Enable slave's ISP ECO mode
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x08;
		dbbus_tx_data[2] = 0x0c;
		dbbus_tx_data[3] = 0x08;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	  //Enable SPI Pad
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x02;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
		HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
		TP_DEBUG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
		dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	  //WP overwrite
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x0E;
		dbbus_tx_data[3] = 0x02;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	  //set pin high
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0x10;
		dbbus_tx_data[3] = 0x08;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

		dbbusDWIICIICNotUseBus();
		dbbusDWIICNotStopMCU();
		dbbusDWIICExitSerialDebugMode();


    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
	  TP_DEBUG("chip erase+\n");
    drvISP_ChipErase();
	  TP_DEBUG("chip erase-\n");
    drvISP_ExitIspMode();
    return size;
}

static DEVICE_ATTR(clear, 0744, firmware_clear_show, firmware_clear_store);

/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    TP_DEBUG("*** firmware_version_show fw_version = %s***\n", fw_version);
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    //unsigned short major=0, minor=0;

    fw_version = kzalloc(sizeof(char), GFP_KERNEL);
		//SM-BUS GET FW VERSION
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
   if(chip_id==0x02)
    	dbbus_tx_data[2] = 0x2a;  //0x74      
    else
    dbbus_tx_data[2] = 0x74;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

	major_version = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor_version= (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

    TP_DEBUG("***major = %d ***\n", major_version);
    TP_DEBUG("***minor = %d ***\n", minor_version);
    sprintf(fw_version,"%03d%03d", major_version, minor_version);
	
  /*  major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

    TP_DEBUG("***major = %d ***\n", major);
    TP_DEBUG("***minor = %d ***\n", minor);
    sprintf(fw_version,"%03d%03d", major, minor);
    */
    TP_DEBUG("***fw_version = %s ***\n", fw_version);


    return size;
}
static DEVICE_ATTR(version, 0744, firmware_version_show, firmware_version_store);

static int fts_GetFWVer(unsigned long * pu32)
{
	unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
   // unsigned short major=0, minor=0;
		//SM-BUS GET FW VERSION
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
  if(chip_id==0x02)
      dbbus_tx_data[2] = 0x2a; //0x74
   else 
    dbbus_tx_data[2] = 0x74;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    major_version= (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor_version= (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

   *pu32 = (major_version<<16) | minor_version;
   printk("major_version = %d,minor_version=%d,\n",major_version,minor_version);
   //*pu32 = 0;
    return 0;
}


static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{

    int i;
	TP_DEBUG("***FwDataCnt = %d ***\n", FwDataCnt);
    for (i = 0; i < 1024; i++)
    {
        memcpy(temp[FwDataCnt], buf, 1024);
    }
    FwDataCnt++;
    return size;
}
static DEVICE_ATTR(data, 0744, firmware_data_show, firmware_data_store);
#endif


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
				__stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":240:900:100:80"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":120:900:100:80"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":360:900:100:80"				   
			   "\n");	
#endif

}

static struct kobj_attribute ty_touch_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ms-msg21xx",
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



#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)

static void msg21xx_suspend(void)
{
	printk("Enter %s \n",__func__);
	if (is_suspended) {
		printk("  goodix_ts Already in suspend state\n");
		return ;
	}
	if((msg21xx_irq)&&(test_and_clear_bit(0,&mstar_tp_status.irqen)))                //gaohw
		disable_irq_nosync(msg21xx_irq);
		
	cancel_work_sync(&msg21xx_wq);
	//gpio_request(MSG21XX_RESET_GPIO, "reset");   //add by gaohw
	gpio_direction_output(MSG21XX_RESET_GPIO, 0);   //add by gaohw
	gpio_set_value(MSG21XX_RESET_GPIO, 0);
	//gpio_free(MSG21XX_RESET_GPIO);         //add by gaohw
	input_mt_sync(input);
	input_sync(input);
	#if defined TY_PMU_CTL
	if ( mstar_power_off()) {
			printk("  msg21xx_ts power off failed");
			return ;
	}
	#endif
	is_suspended = true;
	printk("TP exit %s \n",__func__);
}

static void msg21xx_resume(void)
{
	 if (!is_suspended) {
		printk(" msg21xx_ts Already in awake state\n");
		return ;
	}
	printk("Enter %s \n",__func__);
	#if defined TY_PMU_CTL
	 if (mstar_power_on()) {
			printk("msg21xx_ts power on failed\n");
			return ;
 	  }
	#endif
	/*input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(input, ABS_MT_POSITION_X, 3000);
	input_report_abs(input, ABS_MT_POSITION_Y, 3000);
	input_mt_sync(input);
	input_sync(input);
	*/
	#if 1//defined(TYQ_TBW5913B_SUPPORT) || defined(TYQ_TBW5913C_SUPPORT) 
	//gpio_request(MSG21XX_RESET_GPIO, "reset");   //add by gaohw
	gpio_direction_output(MSG21XX_RESET_GPIO, 1);   //add by gaohw
	/*TY:niuli insert 2 msleep(5) to add delay time when system resume, 
	so lcd will not white background when resume on 2012*/
	msleep(5);
	gpio_set_value(MSG21XX_RESET_GPIO, 1);
	msleep(5);
	//gpio_free(MSG21XX_RESET_GPIO);         //add by gaohw
	#else
	gpio_direction_input(MSG21XX_RESET_GPIO);
	/*TY:niuli insert 2 msleep(5) to add delay time when system resume, 
	so lcd will not white background when resume on 2012*/
	msleep(10);
	#endif
	if((msg21xx_irq)&&(!test_and_set_bit(0,&mstar_tp_status.irqen)))          //add by  gaohw
		enable_irq(msg21xx_irq);
	
	is_suspended= false;
}
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	#if 0
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);
	#endif
	#if 0
	if(is_upgrading == 1)
		return 0;
	#endif
	if (evdata && evdata->data && event == FB_EVENT_BLANK ) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			msg21xx_resume();
		else if (*blank == FB_BLANK_POWERDOWN)
			msg21xx_suspend();
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)

//---------------------------------------------------------------------------
static void msg21xx_early_suspend(struct early_suspend *h)
{
	printk("Enter %s \n",__func__);;
	msg21xx_suspend();
	printk("TP exit %s \n",__func__);
}

static void msg21xx_early_resume(struct early_suspend *h)
{

	printk("Enter %s \n",__func__);
	msg21xx_resume();
	printk("TP exit %s \n",__func__);
}

#endif
#endif
//#if defined(CONFIG_TOUCHSCREEN_FT5X06) && defined(CONFIG_TY_TOUCHSCREEN_MSTAR) 

//#else
static void msg21xx_chip_init(void)
{
	#if 1 //defined(TYQ_TBW5913B_SUPPORT) || defined(TYQ_TBW5913C_SUPPORT)
    /* After the LCD is on, power on the TP controller */
	//gpio_request(MSG21XX_RESET_GPIO, "reset");   //add by gaohw
	gpio_direction_output(MSG21XX_RESET_GPIO, 1);
	gpio_set_value(MSG21XX_RESET_GPIO, 1);
	gpio_set_value(MSG21XX_RESET_GPIO, 0);
    mdelay(200);  /* Note that the RST must be in LOW 10ms at least */
	gpio_set_value(MSG21XX_RESET_GPIO, 1);
    /* Enable the interrupt service thread/routine for INT after 50ms */
    mdelay(50);
	//gpio_free(MSG21XX_RESET_GPIO);         //add by gaohw
	#else
	/* After the LCD is on, power on the TP controller */
	//gpio_request(MSG21XX_RESET_GPIO, "reset");   //add by gaohw
	//gpio_direction_output(MSG21XX_RESET_GPIO, 1);
	gpio_direction_output(MSG21XX_RESET_GPIO, 0);
	//gpio_set_value(MSG21XX_RESET_GPIO, 1);
	gpio_set_value(MSG21XX_RESET_GPIO, 0);
    mdelay(200);  /* Note that the RST must be in LOW 10ms at least */
	//gpio_set_value(MSG21XX_RESET_GPIO, 1);
	gpio_direction_input(MSG21XX_RESET_GPIO);
    /* Enable the interrupt service thread/routine for INT after 50ms */
    mdelay(50);
	//gpio_free(MSG21XX_RESET_GPIO);         //add by gaohw
	#endif
}
//#endif

static u8 Calculate_8BitsChecksum( u8 *msg, s32 s32Length )
{
    s32 s32Checksum = 0;
    s32 i;

    for ( i = 0 ; i < s32Length; i++ )
    {
        s32Checksum += msg[i];
    }

    return (u8)( ( -s32Checksum ) & 0xFF );
}


#ifdef AUTO_UPDATE
static void msg21xx_data_disposal_gao(struct work_struct *work)
{
	int i =0;
	TP_DEBUG("current fw_version is %s,wrong version !! \n",fw_version);
	
	for(i=0;i<94;i++)
 	{
 		firmware_data_store(firmware_cmd_dev,0,MSG_2133_BIN+(i*1024),(size_t)i);		
 	}
 	 firmware_update_store(firmware_cmd_dev,0,0,(size_t)i);

}
#endif


static void msg21xx_data_disposal(struct work_struct *work)
{
    u8 val[8] = {0};
    u8 Checksum = 0;
	u8 i;
    u32 delta_x = 0, delta_y = 0;
    u32 u32X = 0;
    u32 u32Y = 0;
	u8 touchkeycode = 0;
	TouchScreenInfo_t *touchData=NULL;
	static u32 preKeyStatus=0;
	//static u32 preFingerNum=0;

/*#define SWAP_X_Y   (1)
#ifdef SWAP_X_Y
    int tempx;
    int tempy;
#endif
*/
	touchData = kzalloc(sizeof(TouchScreenInfo_t), GFP_KERNEL);

	/* tianyu quhl add 2012-10-19 */
	if(touchData==NULL)
	{
		printk("=====%s:request memory failed!!!!=====\n",__func__);
		return;
	}
    memset(touchData, 0, sizeof(TouchScreenInfo_t));

    //_TRACE((TSCR_LEVEL_C_TYP, "DrvTscrCGetData in"));
    i2c_master_recv(msg21xx_i2c_client,&val[0],REPORT_PACKET_LENGTH);
    Checksum = Calculate_8BitsChecksum(&val[0], 7); //calculate checksum

	
    //printk("primary:: val[0]=%d,val[1]=%d,val[2]=%d,val[3]=%d,val[4]=%d,val[5]=%d,val[6]=%d,val[7]=%d\n",val[0],val[1],val[2],val[3],val[4],val[5],val[6],val[7]);
    if ((Checksum == val[7]) && (val[0] == 0x52))   //check the checksum  of packet
    {
        u32X = (((val[1] & 0xF0) << 4) | val[2]);         //parse the packet to coordinates
        u32Y = (((val[1] & 0x0F) << 8) | val[3]);

        delta_x = (((val[4] & 0xF0) << 4) | val[5]);
        delta_y = (((val[4] & 0x0F) << 8) | val[6]);
  //  printk("primary:: u32X=%d,u32Y=%d\n",u32X,u32Y);
/*    #ifdef SWAP_X_Y
		tempy = u32X;
		tempx = u32Y;
        u32X = tempx;
        u32Y = tempy;

		tempy = delta_x;
		tempx = delta_y;
        delta_x = tempx;
        delta_y = tempy;
	#endif
*/
        //DBG("[HAL] u32X = %x, u32Y = %x", u32X, u32Y);
        //DBG("[HAL] delta_x = %x, delta_y = %x", delta_x, delta_y);

        if ((val[1] == 0xFF) && (val[2] == 0xFF) && (val[3] == 0xFF) && (val[4] == 0xFF) && (val[6] == 0xFF))
        {
            touchData->Point[0].X = 0; // final X coordinate
            touchData->Point[0].Y = 0; // final Y coordinate

          /*  if (val[5] != 0x0)
            {
                touchData->nTouchKeyMode = 1; //TouchKeyMode
				touchData->nTouchKeyCode = val[5]; //TouchKeyCode
                touchData->nFingerNum = 1;
            }
            else
            {
                touchData->nFingerNum = 0; //touch end
                touchData->nTouchKeyCode = 0; //TouchKeyMode
                touchData->nTouchKeyMode = 0; //TouchKeyMode
            }*/

	    if((val[5]==0x0)||(val[5]==0xFF))
            {
                touchData->nFingerNum = 0; //touch end
                touchData->nTouchKeyCode = 0; //TouchKeyMode
                touchData->nTouchKeyMode = 0; //TouchKeyMode
            }
            else
            {
                touchData->nTouchKeyMode = 1; //TouchKeyMode
                touchData->nTouchKeyCode = val[5]; //TouchKeyCode
                touchData->nFingerNum = 1;
		 // printk("&&&&&&&&useful key code report touch key code = %d\n",touchData->nTouchKeyCode);		
            }

		  
        }
		else
		{
		    touchData->nTouchKeyMode = 0; //Touch on screen...

			if ((delta_x == 0) && (delta_y == 0))
			{
				touchData->nFingerNum = 1; //one touch
				touchData->Point[0].X = (u32X * MS_TS_MSG21XX_X_MAX) / 2048;
				touchData->Point[0].Y = (u32Y * MS_TS_MSG21XX_Y_MAX ) / 2048;//1781;

				/* Calibrate if the touch panel was reversed in Y */
	#if 0
				touchData->Point[0].Y = (MS_TS_MSG21XX_Y_MAX - touchData->Point[0].Y);
	#endif
			}
			else
			{
				u32 x2, y2;

				touchData->nFingerNum = 2; //two touch

				/* Finger 1 */
				touchData->Point[0].X = (u32X * MS_TS_MSG21XX_X_MAX) / 2048;
				touchData->Point[0].Y = (u32Y * MS_TS_MSG21XX_Y_MAX ) / 2048;//1781;

				/* Finger 2 */
				if (delta_x > 2048)     //transform the unsigh value to sign value
				{
					delta_x -= 4096;
				}
				if (delta_y > 2048)
				{
					delta_y -= 4096;
				}

				x2 = (u32)(u32X + delta_x);
				y2 = (u32)(u32Y + delta_y);

				touchData->Point[1].X = (x2 * MS_TS_MSG21XX_X_MAX) / 2048;
				touchData->Point[1].Y = (y2 * MS_TS_MSG21XX_Y_MAX ) /2048;// 1781;

				/* Calibrate if the touch panel was reversed in Y */
	#if 0
				touchData->Point[1].Y = (MS_TS_MSG21XX_Y_MAX - y2);
	#endif
			}
		}

		//report...
		if(touchData->nTouchKeyMode)
		{
			
			if (touchData->nTouchKeyCode == 1)
				#ifdef TYQ_TP_ANDROID4_0_SUPPORT
				touchkeycode = KEY_HOMEPAGE;
				#else
				touchkeycode = KEY_HOME;
				#endif
			if (touchData->nTouchKeyCode == 2)
				touchkeycode = KEY_MENU;
			if (touchData->nTouchKeyCode == 4)
				touchkeycode = KEY_BACK;
			#if 0
			if (touchData->nTouchKeyCode == 8)
				touchkeycode = KEY_SEARCH;
			#endif
			if(preKeyStatus!=touchkeycode)
			{
				preKeyStatus=touchkeycode;
				//input_report_key(input, touchkeycode, 1);
				//printk("&&&&&&&&useful key code report touch key code = %d\n",touchkeycode);
					#ifdef TYQ_TP_ANDROID4_0_SUPPORT
					if(touchkeycode == KEY_HOMEPAGE)
					#else
					if(touchkeycode == KEY_HOME)
					#endif
					{
						//input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
						input_report_abs(input, ABS_MT_POSITION_X, 240);
						input_report_abs(input, ABS_MT_POSITION_Y, 900);
						input_mt_sync(input);
				
					}

					if(touchkeycode == KEY_MENU)
					{
					//	input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
						input_report_abs(input, ABS_MT_POSITION_X, 120);
						input_report_abs(input, ABS_MT_POSITION_Y, 900);
						input_mt_sync(input);
				
					}

					if(touchkeycode == KEY_BACK)
					{
					//	input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
						input_report_abs(input, ABS_MT_POSITION_X, 360);
						input_report_abs(input, ABS_MT_POSITION_Y, 900);
						input_mt_sync(input);
				
					}
					#if 0
					if(touchkeycode == KEY_SEARCH)
					{
					//	input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
						input_report_abs(input, ABS_MT_POSITION_X, 420);
						input_report_abs(input, ABS_MT_POSITION_Y, 863);
						input_mt_sync(input);
				
					}
					#endif
					
			}
			
			input_sync(input);
		}
        else
        {
		    preKeyStatus=0; //clear key status..

            if((touchData->nFingerNum) == 0)   //touch end
            {
				//preFingerNum=0;
			/*	input_report_key(input, KEY_MENU, 0);
				input_report_key(input, KEY_HOME, 0);
				input_report_key(input, KEY_BACK, 0);
				input_report_key(input, KEY_SEARCH, 0);
			*/
			//	input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
				input_mt_sync(input);
				input_sync(input);
            }
            else //touch on screen
            {
			    /*
				if(preFingerNum!=touchData->nFingerNum)   //for one touch <--> two touch issue
				{
					printk("langwenlong number has changed\n");
					preFingerNum=touchData->nFingerNum;
					input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
				    input_mt_sync(input);
				    input_sync(input);
				}*/

				for(i = 0;i < (touchData->nFingerNum);i++)
				{
				//	input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
					input_report_abs(input, ABS_MT_POSITION_X, touchData->Point[i].X);
					input_report_abs(input, ABS_MT_POSITION_Y, touchData->Point[i].Y);
					input_mt_sync(input);
				//	printk("X=%d,Y=%d\n",touchData->Point[i].X,touchData->Point[i].Y);
				}

				input_sync(input);
			}
		}
    }
    else
    {
        //DBG("Packet error 0x%x, 0x%x, 0x%x", val[0], val[1], val[2]);
        //DBG("             0x%x, 0x%x, 0x%x", val[3], val[4], val[5]);
        //DBG("             0x%x, 0x%x, 0x%x", val[6], val[7], Checksum);
		printk(KERN_ERR "err status in tp\n");
    }
	if((msg21xx_irq)&&(!test_and_set_bit(0,&mstar_tp_status.irqen)))          //add by  gaohw
		enable_irq(msg21xx_irq);
   // enable_irq(msg21xx_irq);

	//tianyu quhl add 2012-10-19
	if(touchData)
	{
		kfree(touchData);
	}
}



static int msg21xx_ts_open(struct input_dev *dev)
{
	return 0;
}

static void msg21xx_ts_close(struct input_dev *dev)
{
	printk("msg21xx_ts_close\n");
}


static int msg21xx_init_input(void)
{
	int err;

	input = input_allocate_device();
	input->name ="ms-msg21xx";
	input->phys = "I2C";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &msg21xx_i2c_client->dev;
	input->open = msg21xx_ts_open;
	input->close = msg21xx_ts_close;

	set_bit(EV_ABS, input->evbit);
	set_bit(EV_SYN, input->evbit);
	
	#ifdef TYQ_TP_ANDROID4_0_SUPPORT
	set_bit(INPUT_PROP_DIRECT,input->propbit);
	/*TYDRV:liujie add this for qcom factory test 20140821*/
	set_bit(EV_KEY, input->evbit);
	#else
	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	#endif
	set_bit(KEY_BACK, input->keybit);
	set_bit(KEY_MENU, input->keybit);
	#ifdef TYQ_TP_ANDROID4_0_SUPPORT
		set_bit(KEY_HOMEPAGE, input->keybit);
	#else
		set_bit(KEY_HOME, input->keybit);
	#endif
	//set_bit(KEY_SEARCH, input->keybit);
	
//	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, MS_TS_MSG21XX_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, MS_TS_MSG21XX_Y_MAX, 0, 0);

	err = input_register_device(input);
	if (err)
		goto fail_alloc_input;

fail_alloc_input:
	return 0;
}
static irqreturn_t msg21xx_interrupt(int irq, void *dev_id)
{
	if((msg21xx_irq)&&(test_and_clear_bit(0,&mstar_tp_status.irqen)))          //gaohw
		disable_irq_nosync(msg21xx_irq);
	schedule_work(&msg21xx_wq);
	return IRQ_HANDLED;
}


static int __devinit msg21xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int  err = 0;
	#ifdef AUTO_UPDATE
	int i = 0;
	#endif
	#if defined(CONFIG_TOUCHSCREEN_FT5X06) && defined(CONFIG_TY_TOUCHSCREEN_MSTAR) 
	//u8 val[9] = {0};
	if(ft_probe_flag ==1){
		printk("other tp has been  detected successfully,so I (mstar) quit\n");
		return -1;
	}
	#endif
	msg21xx_i2c_client = client;
	#if defined TY_PMU_CTL
	err = mstar_power_init();
		if (err) {
			dev_err(&client->dev, "Mstar power init failed\n");
			//goto exit_free_client_data;
		}
	
		err = mstar_power_on();
		if (err) {
			dev_err(&client->dev, "Mstar power on failed\n");
			goto exit_deinit_power;
		}
	#endif
	gpio_request(MSG21XX_RESET_GPIO, "reset");   //add by gaohw
	gpio_request(MSG21XX_INT_GPIO, "interrupt");
//	gpio_request(MSG21XX_RESET_GPIO, "reset");
	gpio_direction_input(MSG21XX_INT_GPIO);
	//gpio_free(MSG21XX_INT_GPIO);         //add by gaohw
//	gpio_set_value(MSG21XX_INT_GPIO, 1);
//#if defined(CONFIG_TOUCHSCREEN_FT5X06) && defined(CONFIG_TY_TOUCHSCREEN_MSTAR) 
 // tp have been reseted  in board-7x27a.c 
//#else
	
#if defined(TYQ_MSG_TP_I2C_SPEED_100K)
	ty_chage_i2c_speed_rate(client->adapter,50000);
#endif
#if 1
	chip_id=get_chip_id();
	printk("mstar chip id = %x\n",chip_id);
	if(chip_id == 1 || chip_id == 2){
		printk("Mstar tp detected successfully!\n");
	}
	else{
		err = -1;
		goto exit0;
	}
#endif
	
	if (_CheckFwIntegrity()){
		 is_firmware_ok = 1;
	  	//get vendor id from main block.
		msg_vendor_id = _GetVendorID(EMEM_MAIN);
	}
	else{
		 is_firmware_ok = 0;
		 printk("mstar tp firmware is bad in ic\n");
		msg_vendor_id = _GetVendorID(EMEM_INFO);

	}
#if defined(TYQ_MSG_TP_I2C_SPEED_100K)
			ty_chage_i2c_speed_rate(client->adapter,100000);
#endif
	msg21xx_chip_init();
//#endif
	#if 0  //defined(CONFIG_TOUCHSCREEN_FT5X06) && defined(CONFIG_TY_TOUCHSCREEN_MSTAR) 
	//msleep(200);
	//printk("********************msg21xx_probe********************************\n");
	err = i2c_master_recv(msg21xx_i2c_client,&val[0],REPORT_PACKET_LENGTH);
	if(err < 0)
	{
		printk("Mstar tp detected failure!\n");
		goto exit0;
	}
	printk("Mstar tp detected successfully!\n");
	#endif
	INIT_WORK(&msg21xx_wq, msg21xx_data_disposal);
	
	#ifdef AUTO_UPDATE
	INIT_WORK(&msg21xx_wq_gao, msg21xx_data_disposal_gao);
	#endif
	
	msg21xx_init_input();
	msg21xx_irq = client->irq;   //gaohw
	set_bit(0,&mstar_tp_status.irqen); //add by gaohw
	err = request_irq(msg21xx_irq, msg21xx_interrupt,
							IRQF_TRIGGER_RISING, "msg21xx", NULL);          //gaohw no modify
	if (err != 0) {
		printk("%s: cannot register irq\n", __func__);
		goto exit;
	}
	#if defined(CONFIG_FB)
	fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",err);
	
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +1;              //gao +1?
	early_suspend.suspend = msg21xx_early_suspend;
	early_suspend.resume = msg21xx_early_resume;
	register_early_suspend(&early_suspend);
	#endif

#ifdef TYN_VIRTAUL_KEY_FRAMEWORK
	fts_creat_virtual_key_sysfs();               //add by gaohw
#endif
	/*frameware upgrade*/
#ifdef __FIRMWARE_UPDATE__
	touch_ctrl_init(0);
	firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
    if (IS_ERR(firmware_class))
        pr_err("Failed to create class(firmware)!\n");
    firmware_cmd_dev = device_create(firmware_class,
                                     NULL, 0, NULL, "device");
    if (IS_ERR(firmware_cmd_dev))
        pr_err("Failed to create device(firmware_cmd_dev)!\n");

    // version
    if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
	// clear
    if (device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);

	dev_set_drvdata(firmware_cmd_dev, NULL);
#endif

	#ifdef AUTO_UPDATE
  #ifndef __MSG_2133_BIN__
  	#error "please include firmware array"
  #else
  fm_major_version = MSG_2133_BIN[0x7f50] + (MSG_2133_BIN[0x7f4f] << 8);//0x7f4d-0x7f50
  fm_minor_version = MSG_2133_BIN[0x7f4e] + (MSG_2133_BIN[0x7f4d] << 8);
  mdelay(100);
 firmware_version_store(firmware_cmd_dev,0,0,(size_t)i);
 printk("fw_version =%s  major_version =%d  minor_version =%d fm_major_version =%d fm_minor_version=%d\n", fw_version,major_version,minor_version,fm_major_version,fm_minor_version);
  if((major_version == fm_major_version) &&(minor_version < fm_minor_version)) 
// if(fw_version[2]!=0x31 && fw_version[4]!=0x31 && fw_version[5]!=0x31)
 {
 	schedule_work(&msg21xx_wq_gao);
 	/*TP_DEBUG("current fw_version is %s,wrong version !! \n",fw_version);
	for(i=0;i<94;i++)
 	{
 		firmware_data_store(firmware_cmd_dev,0,MSG_2133_BIN+(i*1024),(size_t)i);		
 	}
 	 firmware_update_store(firmware_cmd_dev,0,0,(size_t)i);
 	*/
 }
 else
 {
	TP_DEBUG("firmware version check okay \n"); 		
 }
 
 //mdelay(10000);
 	#endif
#endif

	return 0;

exit0:
	 gpio_free(MSG21XX_RESET_GPIO);
	 gpio_free(MSG21XX_INT_GPIO);
#if defined TY_PMU_CTL
	 mstar_power_off();
 exit_deinit_power:
    	 mstar_power_deinit();
#endif

exit:
	return err;
}


/*******************************start irmware updae control*******************************/

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

DEFINE_SEMAPHORE(mstar_upgrade_lock);

static long touch_ctrl_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
	struct ty_touch_fmupgrade_S  bufarg;
	int iret;
	unsigned char *pBuf;
	u8 bRet = 0;
	unsigned long  ulVer;
	//unsigned long i;
	int mVer;

	if (down_interruptible(&mstar_upgrade_lock)) {
		printk("obtain mstar_upgrade_lock faild!\n ");
		return  -EFAULT;
	}
	if (0 == arg)
	{
		printk("%s:arg null pointer.\n",__func__);
		up(&mstar_upgrade_lock);
		return -EFAULT;
	}
	
	if (copy_from_user(&bufarg, (void *) arg, sizeof(bufarg))) 
	{
		printk("%s:null pointer.\n",__func__);
		up(&mstar_upgrade_lock);
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
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}
			printk("%s:getver 1.\n",__func__);
			
			if ((0>= bufarg.bufLength )||( sizeof(unsigned long) < bufarg.bufLength ))
			{
				printk("%s:bufLength null pointer.\n",__func__);
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}			
			//printk("%s:lenth=%x,addr=0x%x\n",__func__,(unsigned int)bufarg.bufLength,(bufarg.bufAddr));
			printk("%s:getver 2\n",__func__);
			
			pBuf = kzalloc(bufarg.bufLength, GFP_KERNEL);
			if (0 == pBuf)
			{
				printk("%s:alloc buffer failed.\n",__func__);
				up(&mstar_upgrade_lock);
				return 0;
			}

			if (copy_from_user(pBuf, (void *)bufarg.bufAddr, bufarg.bufLength)) 
			{
				printk("%s:get buffer error.\n",__func__);
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}
			printk("%s:getver 3.\n",__func__);

			
			ulVer = 0;
			
			//get firmware version
	#if defined(NV_TOUCH_FT)
		if( is_firmware_ok == 1){
			printk("mstar tp firmware check ok\n");
			mdelay(200);
			bRet = fts_GetFWVer(&ulVer);
			if(0 > bRet) 
			{
				*(unsigned long*)bufarg.bufAddr = 0;
				printk("%s:FT_GetFWVer failed. \n",__func__);
				kfree(pBuf);
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}
			
				
		}
		else{
			ulVer = 0xa6;
		}
	#endif

			*(( unsigned long *)bufarg.bufAddr) = ulVer;
			
			//printk("%s:ver=0x%x \n",__func__,*(unsigned long*)bufarg.bufAddr);
			if( pBuf ) kfree(pBuf);
			
			break;
		case TYN_TOUCH_IC_TYPE:
			printk("%s:getver.\n",__func__);
			if (NULL == bufarg.bufAddr)
			{
				printk("%s:bufAddr null pointer.\n",__func__);
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}
			printk("%s:getver 1.\n",__func__);
			
			if ((0>= bufarg.bufLength )||( sizeof(unsigned long) < bufarg.bufLength ))
			{
				printk("%s:bufLength null pointer.\n",__func__);
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}			
			//printk("%s:lenth=%x,addr=0x%x\n",__func__,(unsigned int)bufarg.bufLength,(bufarg.bufAddr));
			printk("%s:get tp ic type 2\n",__func__);
			
			pBuf = kzalloc(bufarg.bufLength, GFP_KERNEL);
			if (0 == pBuf)
			{
				printk("%s:alloc buffer failed.\n",__func__);
				up(&mstar_upgrade_lock);
				return 0;
			}

			if (copy_from_user(pBuf, (void *)bufarg.bufAddr, bufarg.bufLength)) 
			{
				printk("%s:get buffer error.\n",__func__);
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}

			*(( unsigned long *)bufarg.bufAddr) = MSG_TP_ID;
			
			//printk("%s:ver=0x%x \n",__func__,*(unsigned long*)bufarg.bufAddr);
			if( pBuf ) kfree(pBuf);
			
			break;
		case TYN_TOUCH_CHIPID:
			printk("%s:getchipidr.\n",__func__);
			if (NULL == bufarg.bufAddr)
			{
				printk("%s:bufAddr null pointer.\n",__func__);
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}
			printk("%s:getver 1.\n",__func__);
			
			if ((0>= bufarg.bufLength )||( sizeof(unsigned long) < bufarg.bufLength ))
			{
				printk("%s:bufLength null pointer.\n",__func__);
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}			
			//printk("%s:lenth=%x,addr=0x%x\n",__func__,(unsigned int)bufarg.bufLength,(bufarg.bufAddr));
			printk("%s:getver 2\n",__func__);
			
			pBuf = kzalloc(bufarg.bufLength, GFP_KERNEL);
			if (0 == pBuf)
			{
				printk("%s:alloc buffer failed.\n",__func__);
				up(&mstar_upgrade_lock);
				return 0;
			}

			if (copy_from_user(pBuf, (void *)bufarg.bufAddr, bufarg.bufLength)) 
			{
				printk("%s:get buffer error.\n",__func__);
				up(&mstar_upgrade_lock);
				return -EFAULT;
			}
			printk("%s:getver 3.\n",__func__);

			

			*(( unsigned long *)bufarg.bufAddr) = (unsigned long)chip_id;
			
			//printk("%s:ver=0x%x \n",__func__,*(unsigned long*)bufarg.bufAddr);
			if( pBuf ) kfree(pBuf);
			
			break;
			
		case TYN_TOUCH_FMUPGRADE:
			if (0 == bufarg.bufAddr)
			{
				printk("%s:null pointer.\n",__func__);
				up(&mstar_upgrade_lock);
				return 0;
			}
			
			pBuf =&temp[0][0]; //kzalloc(bufarg.bufLength, GFP_KERNEL);
			if (0 == pBuf)
			{
				printk("%s:alloc buffer failed.\n",__func__);
				up(&mstar_upgrade_lock);
				return 0;
			}
			
			if (copy_from_user(pBuf, (void *) bufarg.bufAddr, bufarg.bufLength)) 
			{
				printk("%s:get buffer error.\n",__func__);
				up(&mstar_upgrade_lock);
				return -EAGAIN;
			}
			mdelay(200);
			
     #if defined(NV_TOUCH_FT)
			if( TYN_TOUCH_FOCALTECH == bufarg.touchType)
			{
				iret = firmware_update_store(0,0,0,0);//fts_ctpm_fw_upgrade_with_i_file(pBuf,bufarg.bufLength);
				if(iret)
				{
					is_firmware_ok = 0;
					printk("update focaltech firmware failed.\n");
				}
				else{
					is_firmware_ok = 1 ;
				}
				up(&mstar_upgrade_lock);
				//kfree(pBuf);
				return iret;
			}
    #endif
		//	if( pBuf ) kfree(pBuf);
			break;
		case TYN_TOUCH_VENDOR:
			ulVer = 0;
			  mVer = msg_vendor_id;
			  printk("msg_vendor_id = %d\n",msg_vendor_id);
			if(mVer == TRUlY_VENDOR_ID_NO)
			{
				if(copy_to_user(bufarg.bufAddr,TRUlY_VENDOR_ID_STR,strlen(TRUlY_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(mVer == BYD_VENDOR_ID_NO)
			{
				if(copy_to_user(bufarg.bufAddr,BYD_VENDOR_ID_STR,strlen(BYD_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(mVer == O_FILM_VENDOR_ID_NO )
			{	
				if(copy_to_user(bufarg.bufAddr,O_FILM_VENDOR_ID_STR,strlen(O_FILM_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(mVer == TRUST_VENDOR_ID_NO )
			{	
				if(copy_to_user(bufarg.bufAddr,TRUST_VENDOR_ID_STR,strlen(TRUST_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else if(mVer == HOLITECH_VENDOR_ID_NO )
			{	
				if(copy_to_user(bufarg.bufAddr,HOLITECH_VENDOR_ID_STR,strlen(HOLITECH_VENDOR_ID_STR)))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			else
			{
				if(copy_to_user(bufarg.bufAddr,"unknown(mstar)",strlen("unknown(mstar)")))
				{
					printk("%s:get tp_vendor info err!\n",__func__);
				}
			}
			break;
		default:
			printk("%s:Invalid command.\n",__func__);
			
	}
	up(&mstar_upgrade_lock);
	return 0;
}

static int touch_ctrl_close(struct inode *inode, struct file *filp) 
{ 
  printk("%s\n", __func__); 
  return 0; 
} 

struct file_operations ty_touch_ctrl_fops_msg = 
{ 
	.write = touch_ctrl_write,
	.read = touch_ctrl_read, 
	.open = touch_ctrl_open, 
	.release = touch_ctrl_close, 
	.unlocked_ioctl	 = touch_ctrl_ioctl,
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
		 
  cdev_init(&ty_touch_ctrl.cdev, &ty_touch_ctrl_fops_msg);
 
  ty_touch_ctrl.cdev.owner = THIS_MODULE; 
  ty_touch_ctrl.cdev.ops = &ty_touch_ctrl_fops_msg;
  
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


/*******************************end irmware updae*******************************/


static int __devexit msg21xx_remove(struct i2c_client *client)
{
	#if defined TY_PMU_CTL
	 mstar_power_off();
    	mstar_power_deinit();
	#endif
	return 0;
}



static const struct i2c_device_id msg21xx_id[] = {
	{ "ms-msg21xx", 0x26 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, msg21xx_id);

static struct of_device_id mstar_match_table[] = {
	{ .compatible = "mstar,ms-msg21xx", },
	{ },
};


static struct i2c_driver msg21xx_driver = {
	.driver = {
		   .name = "ms-msg21xx",
		   .owner = THIS_MODULE,
		   .of_match_table = mstar_match_table,
	},
	.probe = msg21xx_probe,
	.remove = __devexit_p(msg21xx_remove),
	.id_table = msg21xx_id,
};





static int __init msg21xx_init(void)
{
	int err;
	err = i2c_add_driver(&msg21xx_driver);
	if (err) {
		printk(KERN_WARNING "msg21xx  driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          msg21xx_driver.driver.name);
	}
	return err;
}

static void __exit msg21xx_cleanup(void)
{
	i2c_del_driver(&msg21xx_driver);
}


module_init(msg21xx_init);
module_exit(msg21xx_cleanup);

MODULE_AUTHOR("Mstar semiconductor");
MODULE_DESCRIPTION("Driver for msg21xx Touchscreen Controller");
MODULE_LICENSE("GPL");

