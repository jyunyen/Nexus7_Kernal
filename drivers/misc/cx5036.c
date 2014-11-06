/*
 * This file is part of the CX5036, AP3212C and AP3216C sensor driver.
 * CX5036 is combined proximity and ambient light sensor.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *	    Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: cx5036.c
 *
 * Summary:
 *	CX5036 device driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 15/09/14 Templeton Tsai       1. Init CX5036
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/workqueue.h>
#include <linux/timer.h>
#include "cx5036.h"

#define CX5036_DRV_NAME		"cx5036"
#define DRIVER_VERSION		"1"


#define PL_TIMER_DELAY 10
#define POLLING_MODE 1
#define INTERRUPT_MODE 0
#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("CX5036: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif
static void cx5036_work_handler(struct work_struct *w);
#if POLLING_MODE
static void cx5036_timer_callback(unsigned long pl_data);
#endif
static int cx5036_set_phthres(struct i2c_client *client, int val);
static int cx5036_set_plthres(struct i2c_client *client, int val);
static int cx5036_set_ahthres(struct i2c_client *client, int val);
static int cx5036_set_althres(struct i2c_client *client, int val);
static void cx5036_change_ls_threshold(struct i2c_client *client);
static int cx5036_init_mgs(struct i2c_client *client);

struct cx5036_data {
    struct i2c_client *client;
    u8 reg_cache[CX5036_NUM_CACHABLE_REGS];//TO-DO
    u8 power_state_before_suspend;
    int irq;
    struct input_dev	*psensor_input_dev;
    struct input_dev	*lsensor_input_dev;
    struct input_dev	*hsensor_input_dev;
    struct input_dev	*msensor_input_dev;
    struct workqueue_struct *plsensor_wq;
    struct work_struct plsensor_work;
#if POLLING_MODE
    struct timer_list pl_timer;
#endif
};

static struct cx5036_data *cx5036_data_g = NULL;
// CX5036 register
static u8 cx5036_reg[CX5036_NUM_CACHABLE_REGS] = 
{
    0x00, 0x01, 0x02, 0x06, 0x07, 0x08, 0x0a, 0x0c, 0x0d, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x71
};

// CX5036 range
static int cx5036_range[5] = {65536,16384,3072,2048,1024};


static u16 cx5036_threshole[8] = {28,444,625,888,1778,3555,7222,0xffff};

static u8 *reg_array;
static int *range;
static int reg_num = 0;

static int cali = 100;
static int misc_ps_opened = 0;
static int misc_ls_opened = 0;
//Motion Valuables
static int motion = 0;
#define MOTION_NONE    0
#define MOTION_UP      1
#define MOTION_DOWN    2
#define MOTION_LEFT    3
#define MOTION_RIGHT   4
static int motion_interrupt = 0;
static DEFINE_MUTEX(cx5026_motion_intrrupt_lock);


#define ADD_TO_IDX(addr,idx)	{														\
    int i;												\
    for(i = 0; i < reg_num; i++)						\
    {													\
	if (addr == reg_array[i])						\
	{												\
	    idx = i;									\
	    break;										\
	}												\
    }													\
}


/*
 * register access helpers
 */

static int __cx5036_read_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift)
{
    struct cx5036_data *data = i2c_get_clientdata(client);
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx)
	return (data->reg_cache[idx] & mask) >> shift;
}

static int __cx5036_write_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift, u8 val)
{
    struct cx5036_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 tmp;
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx)
	if (idx >= reg_num)
	    return -EINVAL;

    tmp = data->reg_cache[idx];
    tmp &= ~mask;
    tmp |= val << shift;

    ret = i2c_smbus_write_byte_data(client, reg, tmp);
    if (!ret)
	data->reg_cache[idx] = tmp;

    return ret;
}

/*
 * internally used functions
 */

/* range */
static int cx5036_get_range(struct i2c_client *client)
{
    u8 idx = __cx5036_read_reg(client, CX5036_REG_ALS_CON,
	    CX5036_REG_ALS_ALGAIN_CON_MASK, CX5036_REG_ALS_ALGAIN_CON_SHIFT); 
    return range[idx];
}

static int cx5036_set_range(struct i2c_client *client, int range)
{
    return __cx5036_write_reg(client, CX5036_REG_ALS_CON,
	    CX5036_REG_ALS_ALGAIN_CON_MASK, CX5036_REG_ALS_ALGAIN_CON_SHIFT, range);
}


/* mode */
static int cx5036_get_mode(struct i2c_client *client)
{
    int ret;

    ret = __cx5036_read_reg(client, CX5036_REG_SYS_CON,
	    CX5036_REG_SYS_CON_MASK, CX5036_REG_SYS_CON_SHIFT);
    return ret;
}

static int cx5036_set_mode(struct i2c_client *client, int mode)
{
    int ret;
    if(mode == CX5036_SYS_PS_ENABLE) {
	LDBG("mode = %x\n", mode);
	ret = __cx5036_write_reg(client, CX5036_REG_PS_GAIN,
		CX5036_REG_PS_GAIN_MASK, CX5036_REG_PS_GAIN_SHIFT, 1);
	ret = __cx5036_write_reg(client, CX5036_REG_PS_TIME,
		CX5036_REG_PS_TIME_MASK, CX5036_REG_PS_TIME_SHIFT, 3);
	ret = cx5036_set_plthres(client, 100);
	ret = cx5036_set_phthres(client, 500);
	misc_ps_opened = 1;
    } else if(mode == CX5036_SYS_ALS_ENABLE) {

	LDBG("mode = %x\n", mode);
	ret = __cx5036_write_reg(client, CX5036_REG_ALS_CON,
		CX5036_REG_ALS_ALGAIN_CON_MASK, CX5036_REG_ALS_ALGAIN_CON_SHIFT, 1);
	ret = __cx5036_write_reg(client, CX5036_REG_ALS_TIME,
		CX5036_REG_ALS_TIME_MASK, CX5036_REG_ALS_TIME_SHIFT, 0x3F);//resolution
	ret = cx5036_set_althres(client, 9830);
	ret = cx5036_set_ahthres(client, 55704);
	misc_ls_opened = 1;
    } else if(mode == (CX5036_SYS_PS_ENABLE | CX5036_SYS_ALS_ENABLE)) {
	LDBG("mode = %x\n", mode);
	ret = __cx5036_write_reg(client, CX5036_REG_PS_GAIN,
		CX5036_REG_PS_GAIN_MASK, CX5036_REG_PS_GAIN_SHIFT, 1);
	ret = __cx5036_write_reg(client, CX5036_REG_ALS_CON,
		CX5036_REG_ALS_ALGAIN_CON_MASK, CX5036_REG_ALS_ALGAIN_CON_SHIFT, 4);
	ret = __cx5036_write_reg(client, CX5036_REG_ALS_TIME,
		CX5036_REG_ALS_TIME_MASK, CX5036_REG_ALS_TIME_SHIFT, 0x3F);
	ret = cx5036_set_plthres(client, 100);
	ret = cx5036_set_phthres(client, 500);
	ret = cx5036_set_althres(client, 9830);
	ret = cx5036_set_ahthres(client, 55704);
	misc_ps_opened = 1;
	misc_ls_opened = 1;
    } else if(mode == CX5036_SYS_MGS_ENABLE) { 
	LDBG("mode = %x\n", mode);
	ret = __cx5036_write_reg(client, CX5036_REG_SYS_CON,
		CX5036_REG_SYS_CON_MASK, CX5036_REG_SYS_CON_SHIFT, CX5036_SYS_RST_ENABLE);
	ret = cx5036_init_mgs(client);

    } else if(mode == CX5036_SYS_DEV_DOWN) {
	LDBG("mode = %x\n", mode);
	misc_ps_opened = 0;
	misc_ls_opened = 0;
    }



    ret = __cx5036_write_reg(client, CX5036_REG_SYS_CON,
	    CX5036_REG_SYS_CON_MASK, CX5036_REG_SYS_CON_SHIFT, mode);
    return ret;
}

/* ALS low threshold */
static int cx5036_get_althres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __cx5036_read_reg(client, CX5036_REG_AL_ALTHL_LOW,
	    CX5036_REG_AL_ALTHL_LOW_MASK, CX5036_REG_AL_ALTHL_LOW_SHIFT);
    msb = __cx5036_read_reg(client, CX5036_REG_AL_ALTHL_HIGH,
	    CX5036_REG_AL_ALTHL_HIGH_MASK, CX5036_REG_AL_ALTHL_HIGH_SHIFT);
    return ((msb << 8) | lsb);
}

static int cx5036_set_althres(struct i2c_client *client, int val)
{

    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & CX5036_REG_AL_ALTHL_LOW_MASK;

    err = __cx5036_write_reg(client, CX5036_REG_AL_ALTHL_LOW,
	    CX5036_REG_AL_ALTHL_LOW_MASK, CX5036_REG_AL_ALTHL_LOW_SHIFT, lsb);
    if (err)
	return err;

    err = __cx5036_write_reg(client, CX5036_REG_AL_ALTHL_HIGH,
	    CX5036_REG_AL_ALTHL_HIGH_MASK, CX5036_REG_AL_ALTHL_HIGH_SHIFT, msb);

    return err;
}

/* ALS high threshold */
static int cx5036_get_ahthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __cx5036_read_reg(client, CX5036_REG_AL_ALTHH_LOW,
	    CX5036_REG_AL_ALTHH_LOW_MASK, CX5036_REG_AL_ALTHH_LOW_SHIFT);
    msb = __cx5036_read_reg(client, CX5036_REG_AL_ALTHH_HIGH,
	    CX5036_REG_AL_ALTHH_HIGH_MASK, CX5036_REG_AL_ALTHH_HIGH_SHIFT);
    return ((msb << 8) | lsb);
}

static int cx5036_set_ahthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & CX5036_REG_AL_ALTHH_LOW_MASK;

    err = __cx5036_write_reg(client, CX5036_REG_AL_ALTHH_LOW,
	    CX5036_REG_AL_ALTHH_LOW_MASK, CX5036_REG_AL_ALTHH_LOW_SHIFT, lsb);
    if (err)
	return err;

    err = __cx5036_write_reg(client, CX5036_REG_AL_ALTHH_HIGH,
	    CX5036_REG_AL_ALTHH_HIGH_MASK, CX5036_REG_AL_ALTHH_HIGH_SHIFT, msb);

    return err;
}

/* PS low threshold */
static int cx5036_get_plthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __cx5036_read_reg(client, CX5036_REG_PS_PSTHL_LOW,
	    CX5036_REG_PS_PSTHL_LOW_MASK, CX5036_REG_PS_PSTHL_LOW_SHIFT);
    msb = __cx5036_read_reg(client, CX5036_REG_PS_PSTHL_HIGH,
	    CX5036_REG_PS_PSTHL_HIGH_MASK, CX5036_REG_PS_PSTHL_HIGH_SHIFT);
    return ((msb << 8) | lsb);
}

static int cx5036_set_plthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & CX5036_REG_PS_PSTHL_LOW_MASK;

    err = __cx5036_write_reg(client, CX5036_REG_PS_PSTHL_LOW,
	    CX5036_REG_PS_PSTHL_LOW_MASK, CX5036_REG_PS_PSTHL_LOW_SHIFT, lsb);
    if (err)
	return err;

    err = __cx5036_write_reg(client, CX5036_REG_PS_PSTHL_HIGH,
	    CX5036_REG_PS_PSTHL_HIGH_MASK, CX5036_REG_PS_PSTHL_HIGH_SHIFT, msb);

    return err;
}

/* PX high threshold */
static int cx5036_get_phthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __cx5036_read_reg(client, CX5036_REG_PS_PSTHH_LOW,
	    CX5036_REG_PS_PSTHL_LOW_MASK, CX5036_REG_PS_PSTHL_LOW_SHIFT);
    msb = __cx5036_read_reg(client, CX5036_REG_PS_PSTHH_HIGH,
	    CX5036_REG_PS_PSTHL_HIGH_MASK, CX5036_REG_PS_PSTHL_HIGH_SHIFT);
    return ((msb << 8) | lsb);
}

static int cx5036_set_phthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & CX5036_REG_PS_PSTHH_LOW_MASK;

    err = __cx5036_write_reg(client, CX5036_REG_PS_PSTHH_LOW,
	    CX5036_REG_PS_PSTHH_LOW_MASK, CX5036_REG_PS_PSTHH_LOW_SHIFT, lsb);
    if (err)
	return err;

    err = __cx5036_write_reg(client, CX5036_REG_PS_PSTHH_HIGH,
	    CX5036_REG_PS_PSTHH_HIGH_MASK, CX5036_REG_PS_PSTHH_HIGH_SHIFT, msb);

    return err;
}

static int cx5036_get_als_value(struct i2c_client *client)
{
    unsigned int lsb, msb, val;
#ifdef LSC_DBG
    unsigned int tmp,range;
#endif
    int err;


    err = __cx5036_write_reg(client, CX5036_REG_LUM_COEFR,
	    CX5036_REG_LUM_COEFR_MASK, CX5036_REG_LUM_COEFR_SHIFT, 1);
    if(err < 0) {
	return err;
    }
    err = __cx5036_write_reg(client, CX5036_REG_LUM_COEFG,
	    CX5036_REG_LUM_COEFG_MASK, CX5036_REG_LUM_COEFG_SHIFT, 1);
    if(err < 0) {
	return err;
    }
    err = __cx5036_write_reg(client, CX5036_REG_LUM_COEFB,
	    CX5036_REG_LUM_COEFB_MASK, CX5036_REG_LUM_COEFB_SHIFT, 1);
    if(err < 0) {
	return err;
    }
    lsb = i2c_smbus_read_byte_data(client, CX5036_REG_L_DATA_LOW);

    if (lsb < 0) {
	return lsb;
    }

    msb = i2c_smbus_read_byte_data(client, CX5036_REG_L_DATA_HIGH);

    if (msb < 0)
	return msb;

#ifdef LSC_DBG
    range = cx5036_get_range(client);
    tmp = (((msb << 8) | lsb) * range) >> 16;
    tmp = tmp * cali / 100;
    //LDBG("ALS val=%d lux\n",tmp);
#endif
    val = msb << 8 | lsb;

    return val;
}


static int cx5036_get_object(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, CX5036_REG_SYS_INTSTATUS);
    LDBG("Object = %x\n", val);
    val &= CX5036_REG_SYS_INT_OBJ_MASK;

    return val >> CX5036_REG_SYS_INT_OBJ_SHIFT;
}

static int cx5036_get_intstat(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, CX5036_REG_SYS_INTSTATUS);

    return val;
}


static int cx5036_get_px_value(struct i2c_client *client)
{
    int lsb, msb;

    lsb = i2c_smbus_read_byte_data(client, CX5036_REG_PS_DATA_LOW);

    if (lsb < 0) {
	return lsb;
    }

    msb = i2c_smbus_read_byte_data(client, CX5036_REG_PS_DATA_HIGH);

    if (msb < 0)
	return msb;

    return (u32)(((msb & CX5036_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & CX5036_REG_PS_DATA_LOW_MASK));
}


static int cx5036_init_mgs(struct i2c_client *client)
{
    int ret = 0;
    //---------------------------------------
    // mgs configuration
    //---------------------------------------
    ret = __cx5036_write_reg(client, CX5036_REG_WAITING_TIME, 
	    CX5036_REG_WAITING_TIME_WTIME_MASK, CX5036_REG_WAITING_TIME_WTIME_SHIFT, 0);//Register 0x06 configure wait_time
    ret = __cx5036_write_reg(client, CX5036_REG_MGS_CON,
	    CX5036_REG_MGS_GAIN_MASK, CX5036_REG_MGS_GAIN_SHIFT, 1);//Register 0x11 mgs gain      = x8   
    ret = __cx5036_write_reg(client, CX5036_REG_MGS_PERS, 
	    CX5036_REG_MGS_PERS_MASK, CX5036_REG_MGS_PERS_SHIFT, 0);//Register 0x12 configure mgs Z axis persist= every time
    ret = __cx5036_write_reg(client, CX5036_REG_LED_CON, 
	    CX5036_REG_MGSLDR_LED_CON_MASK, CX5036_REG_MGSLDR_LED_CON_SHIFT, 1);//Register 0x17 configure mgs LED width = 31T
    ret = __cx5036_write_reg(client, CX5036_REG_LED_CON, 
	    CX5036_REG_MGSLPUW_LED_CON_MASK, CX5036_REG_MGSLPUW_LED_CON_SHIFT, 55);//Register 0x17 configure mgs LED width = 31T
    ret = __cx5036_write_reg(client, 0x16, 0xff, 0, 0x40);//Register 0x17 configure mgs LED width = 31T
    ret = __cx5036_write_reg(client, 0x71, 0xff, 0, 0x01);//Register 0x71 configure mgs error	 = enable flag

    if(ret < 0)
	LDBG("Init MGS Failed\n");

    return ret;
}
static int cx5036_get_mgs_x_value(struct i2c_client *client)
{
    return (u32)i2c_smbus_read_byte_data(client, CX5036_REG_MGS_DATA_X);
}

static int cx5036_get_mgs_y_value(struct i2c_client *client)
{
    return (u32)i2c_smbus_read_byte_data(client, CX5036_REG_MGS_DATA_Y);

}

static int cx5036_get_mgs_z_value(struct i2c_client *client)
{
    return (u32)i2c_smbus_read_byte_data(client, CX5036_REG_MGS_DATA_Z);
}

static int cx5036_get_mgs_d_value(struct i2c_client *client)
{
    return (u32)i2c_smbus_read_byte_data(client, 0x23);
}
static int cx5036_get_mgs_err_flag(struct i2c_client *client)
{
    return (u32)i2c_smbus_read_byte_data(client, CX5036_REG_ERR);
}

static int cx5036_lsensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = cx5036_get_mode(client);
    if((mode & CX5036_SYS_ALS_ENABLE) == 0){
	mode |= CX5036_SYS_ALS_ENABLE;
	ret = cx5036_set_mode(client,mode);
    }

    return ret;
}

static int cx5036_lsensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = cx5036_get_mode(client);
    if(mode & CX5036_SYS_ALS_ENABLE){
	mode &= ~CX5036_SYS_ALS_ENABLE;
	if(mode == CX5036_SYS_RST_ENABLE)
	    mode = CX5036_SYS_DEV_DOWN;
	ret = cx5036_set_mode(client,mode);
    }

    return ret;
}

static int cx5036_register_lsensor_device(struct i2c_client *client, struct cx5036_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device lsensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for lsensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->lsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "lightsensor-level";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_MISC, 0, 8, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for lsensor\n", __FUNCTION__);
	goto done;
    }
done:
    return rc;
}


static void cx5036_unregister_lsensor_device(struct i2c_client *client, struct cx5036_data *data)
{
    input_unregister_device(data->lsensor_input_dev);
}
static int cx5036_register_heartbeat_sensor_device(struct i2c_client *client, struct cx5036_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device heartbeat sensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for heartbeat sensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->hsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "heartbeat";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_WHEEL, 0, 8, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for heartbeat sensor\n", __FUNCTION__);
	goto done;
    }
done:
    return rc;
}

static void cx5036_unregister_heartbeat_device(struct i2c_client *client, struct cx5036_data *data)
{
    input_unregister_device(data->hsensor_input_dev);
}

static int cx5036_register_motion_sensor_device(struct i2c_client *client, struct cx5036_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device motion sensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for motion sensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->msensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "motion";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_KEY, input_dev->evbit);
    input_set_capability(input_dev, EV_KEY, KEY_UP);
    input_set_capability(input_dev, EV_KEY, KEY_DOWN);
    input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
    input_set_capability(input_dev, EV_KEY, KEY_LEFT);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for motion sensor\n", __FUNCTION__);
	goto done;
    }
done:
    return rc;
}

static void cx5036_unregister_motion_device(struct i2c_client *client, struct cx5036_data *data)
{
    input_unregister_device(data->msensor_input_dev);
}
static void cx5036_change_ls_threshold(struct i2c_client *client)
{
    struct cx5036_data *data = i2c_get_clientdata(client);
    int value;

    value = cx5036_get_als_value(client);
    if(value > 0){
	cx5036_set_althres(client,cx5036_threshole[value-1]);
	cx5036_set_ahthres(client,cx5036_threshole[value]);
    }
    else{
	cx5036_set_althres(client,0);
	cx5036_set_ahthres(client,cx5036_threshole[value]);
    }

    input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
    input_sync(data->lsensor_input_dev);

}



static int cx5036_psensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = cx5036_get_mode(client);
    if((mode & CX5036_SYS_PS_ENABLE) == 0){
	mode |= CX5036_SYS_PS_ENABLE;
	ret = cx5036_set_mode(client,mode);
    }

    return ret;
}

static int cx5036_psensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = cx5036_get_mode(client);
    if(mode & CX5036_SYS_PS_ENABLE){
	mode &= ~CX5036_SYS_PS_ENABLE;
	if(mode == CX5036_SYS_RST_ENABLE)
	    mode = CX5036_SYS_DEV_DOWN;
	ret = cx5036_set_mode(client,mode);
    }
    return ret;
}


static int cx5036_register_psensor_device(struct i2c_client *client, struct cx5036_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device psensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for psensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->psensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "proximity";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for psensor\n", __FUNCTION__);
	goto done;
    }

    return 0;

done:
    return rc;
}

static void cx5036_unregister_psensor_device(struct i2c_client *client, struct cx5036_data *data)
{
    input_unregister_device(data->psensor_input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend cx5036_early_suspend;
static void cx5036_suspend(struct early_suspend *h)
{

    if (misc_ps_opened)
	cx5036_psensor_disable(cx5036_data_g -> client);
    if (misc_ls_opened)
	cx5036_lsensor_disable(cx5036_data_g -> client);
}

static void cx5036_resume(struct early_suspend *h)
{

    if (misc_ls_opened)
	cx5036_lsensor_enable(cx5036_data_g -> client);
    if (misc_ps_opened)
	cx5036_psensor_enable(cx5036_data_g -> client);
}
#endif


/* range */
static ssize_t cx5036_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%i\n", cx5036_get_range(data->client));
}

static ssize_t cx5036_store_range(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct cx5036_data *data = cx5036_data_g;
    unsigned long val;
    int ret;

    if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
	return -EINVAL;

    ret = cx5036_set_range(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* motion */
static ssize_t cx5036_show_motion(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", motion);
}

static ssize_t cx5036_store_motion(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct cx5036_data *data = cx5036_data_g;
    unsigned long val;

    if ((strict_strtoul(buf, 10, &val) < 0))
	return -EINVAL;
    motion = val;
    switch(val) {
	case MOTION_UP:
	    input_report_key(data -> msensor_input_dev, KEY_UP, 1);
	    input_report_key(data -> msensor_input_dev, KEY_UP, 0);
	    break;
	case MOTION_DOWN:
	    input_report_key(data -> msensor_input_dev, KEY_DOWN, 1);
	    input_report_key(data -> msensor_input_dev, KEY_DOWN, 0);
	    break;
	case MOTION_RIGHT:
	    input_report_key(data -> msensor_input_dev, KEY_RIGHT, 1);
	    input_report_key(data -> msensor_input_dev, KEY_RIGHT, 0);
	    break;
	case MOTION_LEFT:
	    input_report_key(data -> msensor_input_dev, KEY_LEFT, 1);
	    input_report_key(data -> msensor_input_dev, KEY_LEFT, 0);
	    break;
	case MOTION_NONE:
	    //input_report_key(data -> msensor_input_dev, KEY_UP, 0);
	    //input_report_key(data -> msensor_input_dev, KEY_DOWN, 0);
	    //input_report_key(data -> msensor_input_dev, KEY_LEFT, 0);
	    //input_report_key(data -> msensor_input_dev, KEY_RIGHT, 0);
	    break;
    }
    input_sync(data -> msensor_input_dev);
    return count;
}
/*MGS X*/
static ssize_t cx5036_show_mgs_x(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_mgs_x_value(data->client));
}

/*MGS Y*/
static ssize_t cx5036_show_mgs_y(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_mgs_y_value(data->client));
}

/*MGS Z*/
static ssize_t cx5036_show_mgs_z(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_mgs_z_value(data->client));
}

/*MGS Z*/
static ssize_t cx5036_show_mgs_d(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_mgs_d_value(data->client));
}

/*MGS Error Flag*/
static ssize_t cx5036_show_err_flag(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_mgs_err_flag(data->client));
}

/*MGS INT*/
static ssize_t cx5036_show_motion_int(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    int val = 0; 
    mutex_lock(&cx5026_motion_intrrupt_lock);
    val = motion_interrupt;
    motion_interrupt = 0;
    mutex_unlock(&cx5026_motion_intrrupt_lock);
    return sprintf(buf, "%d\n", val);
}
/* mode */
static ssize_t cx5036_show_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_mode(data->client));
}

static ssize_t cx5036_store_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct cx5036_data *data = cx5036_data_g;
    unsigned long val;
    int ret;

    if ((strict_strtoul(buf, 10, &val) < 0))
	return -EINVAL;
    ret = cx5036_set_mode(data->client, val);

    if (ret < 0)
	return ret;
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies);
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(!ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}



/* lux */
static ssize_t cx5036_show_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;

    /* No LUX data if power down */
    if (cx5036_get_mode(cx5036_data_g->client) != CX5036_SYS_ALS_ENABLE)
	return sprintf((char*) buf, "%s\n", "Please power up first!");

    return sprintf(buf, "%d\n", cx5036_get_als_value(data->client));
}



/* Px data */
static ssize_t cx5036_show_pxvalue(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;

    /* No Px data if power down */
    if (cx5036_get_mode(cx5036_data_g->client) != CX5036_SYS_PS_ENABLE)
	return -EBUSY;

    return sprintf(buf, "%d\n", cx5036_get_px_value(data->client));
}



/* proximity object detect */
static ssize_t cx5036_show_object(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_object(data->client));
}



/* ALS low threshold */
static ssize_t cx5036_show_althres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_althres(data->client));
}

static ssize_t cx5036_store_althres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct cx5036_data *data = cx5036_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = cx5036_set_althres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* ALS high threshold */
static ssize_t cx5036_show_ahthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_ahthres(data->client));
}

static ssize_t cx5036_store_ahthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct cx5036_data *data = cx5036_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = cx5036_set_ahthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}


/* PS low threshold */
static ssize_t cx5036_show_plthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_plthres(data->client));
}

static ssize_t cx5036_store_plthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct cx5036_data *data = cx5036_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = cx5036_set_plthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}


/* PS high threshold */
static ssize_t cx5036_show_phthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    return sprintf(buf, "%d\n", cx5036_get_phthres(data->client));
}

static ssize_t cx5036_store_phthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct cx5036_data *data = cx5036_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = cx5036_set_phthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* calibration */
static ssize_t cx5036_show_calibration_state(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    return sprintf(buf, "%d\n", cali);
}

static ssize_t cx5036_store_calibration_state(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct cx5036_data *data = cx5036_data_g;
    int stdls, lux; 
    char tmp[10];

    LDBG("DEBUG cx5036_store_calibration_state..\n");

    /* No LUX data if not operational */
    if (cx5036_get_mode(data->client) == CX5036_SYS_DEV_DOWN)
    {
	printk("Please power up first!");
	return -EINVAL;
    }

    cali = 100;
    sscanf(buf, "%d %s", &stdls, tmp);

    if (!strncmp(tmp, "-setcv", 6))
    {
	cali = stdls;
	return -EBUSY;
    }

    if (stdls < 0)
    {
	printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
		Set calibration factor to 100.\n", stdls);
	return -EBUSY;
    }

    lux = cx5036_get_als_value(data->client);
    cali = stdls * 100 / lux;

    return -EBUSY;
}


#ifdef LSC_DBG
/* engineer mode */
static ssize_t cx5036_em_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct cx5036_data *data = cx5036_data_g;
    int i;
    u8 tmp;

    LDBG("DEBUG cx5036_em_read..\n");

    for (i = 0; i < reg_num; i++)
    {
	tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);

	LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
    }

    return 0;
}

static ssize_t cx5036_em_write(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct cx5036_data *data = cx5036_data_g;
    u32 addr,val,idx=0;
    int ret = 0;

    LDBG("DEBUG cx5036_em_write..\n");

    sscanf(buf, "%x%x", &addr, &val);

    printk("Write [%x] to Reg[%x]...\n",val,addr);

    ret = i2c_smbus_write_byte_data(data->client, addr, val);
    ADD_TO_IDX(addr,idx)
	if (!ret)
	    data->reg_cache[idx] = val;

    return count;
}
#endif


static struct device_attribute attributes[] = {
    __ATTR(range, S_IWUSR | S_IRUGO, cx5036_show_range, cx5036_store_range),
    __ATTR(mode, 0666, cx5036_show_mode, cx5036_store_mode),
    __ATTR(lux, S_IRUGO, cx5036_show_lux, NULL),
    __ATTR(pxvalue, S_IRUGO, cx5036_show_pxvalue, NULL),
    __ATTR(object, S_IRUGO, cx5036_show_object, NULL),
    __ATTR(althres, S_IWUSR | S_IRUGO, cx5036_show_althres, cx5036_store_althres),
    __ATTR(ahthres, S_IWUSR | S_IRUGO, cx5036_show_ahthres, cx5036_store_ahthres),
    __ATTR(plthres, S_IWUSR | S_IRUGO, cx5036_show_plthres, cx5036_store_plthres),
    __ATTR(phthres, S_IWUSR | S_IRUGO, cx5036_show_phthres, cx5036_store_phthres),
    __ATTR(calibration, S_IWUSR | S_IRUGO, cx5036_show_calibration_state, cx5036_store_calibration_state),
    __ATTR(motion, 0666, cx5036_show_motion, cx5036_store_motion),
    __ATTR(motion_int, 0666, cx5036_show_motion_int, NULL),
    __ATTR(mgs_x, 0666, cx5036_show_mgs_x, NULL),
    __ATTR(mgs_y, 0666, cx5036_show_mgs_y, NULL),
    __ATTR(mgs_z, 0666, cx5036_show_mgs_z, NULL),
    __ATTR(mgs_d, 0666, cx5036_show_mgs_d, NULL),
    __ATTR(err_flag, 0666, cx5036_show_err_flag, NULL),
#ifdef LSC_DBG
    __ATTR(em, S_IWUSR | S_IRUGO, cx5036_em_read, cx5036_em_write),
#endif

};

static int cx5036_init_client(struct i2c_client *client)
{
    struct cx5036_data *data = i2c_get_clientdata(client);
    int i;

    LDBG("DEBUG cx5036_init_client..\n");


    /* read all the registers once to fill the cache.
     * if one of the reads fails, we consider the init failed */
    for (i = 0; i < reg_num; i++) {
	int v = i2c_smbus_read_byte_data(client, reg_array[i]);
	if (v < 0)
	    return -ENODEV;

	data->reg_cache[i] = v;
    }

    /* set defaults */
    cx5036_set_range(client, CX5036_ALS_GAIN_1);
    cx5036_set_mode(client, CX5036_SYS_RST_ENABLE);

    return 0;
}

#if POLLING_MODE
static void cx5036_timer_callback(unsigned long pl_data)
{
    struct cx5036_data *data;
    int ret =0;

    data = cx5036_data_g;
    queue_work(data->plsensor_wq, &data->plsensor_work);

    ret = mod_timer(&cx5036_data_g->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");

}
#endif
static void cx5036_work_handler(struct work_struct *w)
{

    struct cx5036_data *data =
	container_of(w, struct cx5036_data, plsensor_work);
    u8 int_stat;
    int pxvalue;
    int Pval;
    int ret;

    int_stat = cx5036_get_intstat(data->client);


    // ALS int
    if (int_stat & CX5036_REG_SYS_INT_AL_MASK)
    {
	LDBG("LS INT Status: %0x\n", int_stat);
	cx5036_change_ls_threshold(data->client);
	ret = __cx5036_write_reg(data->client, CX5036_REG_SYS_INTSTATUS,
		CX5036_REG_SYS_INT_AL_MASK, CX5036_REG_SYS_INT_AL_SHIFT, 0);
    }

    // PX int
    if (int_stat & CX5036_REG_SYS_INT_PS_MASK)
    {
	int_stat = cx5036_get_intstat(data->client);
	LDBG("PS INT Status: %0x\n", int_stat);
	Pval = cx5036_get_object(data->client);
	LDBG("%s\n", Pval ? "obj near":"obj far");
	input_report_abs(data->psensor_input_dev, ABS_DISTANCE, Pval);

	input_sync(data->psensor_input_dev);
	pxvalue = cx5036_get_px_value(data->client); 
	LDBG("pxvalue = %d\n", pxvalue);
	input_report_abs(data->hsensor_input_dev, ABS_WHEEL, pxvalue);
	input_sync(data->hsensor_input_dev);

	ret = __cx5036_write_reg(data->client, CX5036_REG_SYS_INTSTATUS,
		CX5036_REG_SYS_INT_PS_MASK, CX5036_REG_SYS_INT_PS_SHIFT, 0);
    }

    //Motion Int
    if (int_stat & CX5036_REG_SYS_INT_MGS_MASK)
    {
	int_stat = cx5036_get_intstat(data->client);
	mutex_lock(&cx5026_motion_intrrupt_lock);
	motion_interrupt = 1;
	mutex_unlock(&cx5026_motion_intrrupt_lock);
//	LDBG("1.MGS INT Status: %0x\n", int_stat);

	ret = __cx5036_write_reg(data->client, CX5036_REG_SYS_INTSTATUS,
		CX5036_REG_SYS_INT_POR_MASK, CX5036_REG_SYS_INT_POR_SHIFT, 0);
	ret = __cx5036_write_reg(data->client, CX5036_REG_SYS_INTSTATUS,
		CX5036_REG_SYS_INT_MGS_MASK, CX5036_REG_SYS_INT_MGS_SHIFT, 0);
	int_stat = cx5036_get_intstat(data->client);
    }
#if !defined(POLLING_MODE)
    enable_irq(data->client->irq);
#endif
}

#ifdef INTERRUPT_MODE
static irqreturn_t cx5036_irq(int irq, void *data_)
{
    struct cx5036_data *data = data_;

    LDBG("interrupt\n");
    disable_irq_nosync(data->client->irq);
    queue_work(data->plsensor_wq, &data->plsensor_work);

    return IRQ_HANDLED;
}
#endif
static int create_sysfs_interfaces(struct cx5036_data *sensor)
{
    int i;
    struct class *cx5036_class = NULL;
    struct device *cx5036_dev = NULL;
    int ret;

    cx5036_class = class_create(THIS_MODULE, "sensors");
    if (IS_ERR(cx5036_class)) {
	ret = PTR_ERR(cx5036_class);
	cx5036_class = NULL;
	LDBG("%s: could not allocate cx5036_class, ret = %d\n", __func__, ret);
	goto cx5036_class_error;
    }

    cx5036_dev= device_create(cx5036_class,
	    NULL, 0, "%s", "di_sensors");

    if(cx5036_dev == NULL)
	goto cx5036_device_error;
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
	if (device_create_file(cx5036_dev, attributes + i))
	    goto cx5036_create_file_error;

    return 0;

cx5036_create_file_error:
    for ( ; i >= 0; i--)
	device_remove_file(cx5036_dev, attributes + i);

cx5036_device_error:
    class_destroy(cx5036_class);
cx5036_class_error:
    dev_err(&sensor->client->dev, "%s:Unable to create interface\n", __func__);
    return -1;
}

static int __devinit cx5036_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct cx5036_data *data;
    int err = 0;

    LDBG("cx5036_probe\n");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
	err = -EIO;
	goto exit_free_gpio;
    }

    reg_array = cx5036_reg;
    range = cx5036_range;
    reg_num = CX5036_NUM_CACHABLE_REGS;

    data = kzalloc(sizeof(struct cx5036_data), GFP_KERNEL);
    if (!data){
	err = -ENOMEM;
	goto exit_free_gpio;
    }

    data->client = client;
    i2c_set_clientdata(client, data);
    data->irq = client->irq;

    /* initialize the CX5036 chip */
    err = cx5036_init_client(client);
    if (err)
	goto exit_kfree;

    err = cx5036_register_lsensor_device(client,data);
    if (err){
	dev_err(&client->dev, "failed to register_lsensor_device\n");
	goto exit_kfree;
    }

    err = cx5036_register_psensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_psensor_device\n");
	goto exit_free_ls_device;
    }

    err = cx5036_register_heartbeat_sensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_heartbeat_sensor_device\n");
	goto exit_free_ps_device;
    }

    err = cx5036_register_motion_sensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_motion_sensor_device\n");
	goto exit_free_heartbeat_device;
    }
    /* register sysfs hooks */

    err = create_sysfs_interfaces(data);
    if (err)
	goto exit_free_motion_device;

#ifdef CONFIG_HAS_EARLYSUSPEND
    cx5036_early_suspend.suspend = cx5036_suspend;
    cx5036_early_suspend.resume  = cx5036_resume;
    cx5036_early_suspend.level   = 0x02;
    register_early_suspend(&cx5036_early_suspend);
#endif

#ifdef INTERRUPT_MODE
    err = request_threaded_irq(client->irq, NULL, cx5036_irq,
	    IRQF_TRIGGER_FALLING,
	    "cx5036", data);
    if (err) {
	dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,client->irq);
	goto exit_free_ps_device;
    }
#endif
    data->plsensor_wq = create_singlethread_workqueue("plsensor_wq");
    if (!data->plsensor_wq) {
	LDBG("%s: create workqueue failed\n", __func__);
	err = -ENOMEM;
	goto err_create_wq_failed;
    }

    INIT_WORK(&data->plsensor_work, cx5036_work_handler);

#if POLLING_MODE
    LDBG("Timer module installing\n");
    setup_timer(&data->pl_timer, cx5036_timer_callback, 0);
#endif


    cx5036_data_g = data;
    dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);
    return 0;
err_create_wq_failed:
#if POLLING_MODE
    if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
#endif
    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
exit_free_motion_device:
    cx5036_unregister_motion_device(client,data);
exit_free_heartbeat_device:
    cx5036_unregister_heartbeat_device(client,data);
exit_free_ps_device:
    cx5036_unregister_psensor_device(client,data);

exit_free_ls_device:
    cx5036_unregister_lsensor_device(client,data);

exit_kfree:
    kfree(data);

exit_free_gpio:
    return err;
}

static int __devexit cx5036_remove(struct i2c_client *client)
{
    struct cx5036_data *data = i2c_get_clientdata(client);
    free_irq(data->irq, data);

    cx5036_unregister_psensor_device(client,data);
    cx5036_unregister_lsensor_device(client,data);
    cx5036_unregister_heartbeat_device(client,data);
    cx5036_unregister_motion_device(client,data);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&cx5036_early_suspend);
#endif

    cx5036_set_mode(client, CX5036_SYS_DEV_DOWN);
    kfree(i2c_get_clientdata(client));

    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
#if POLLING_MODE
    if(&data->pl_timer)
	del_timer(&data->pl_timer);
#endif
    return 0;
}

static const struct i2c_device_id cx5036_id[] = {
    { CX5036_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, cx5036_id);

static struct i2c_driver cx5036_driver = {
    .driver = {
	.name	= CX5036_DRV_NAME,
	.owner	= THIS_MODULE,
    },
    .probe	= cx5036_probe,
    .remove	= __devexit_p(cx5036_remove),
    .id_table = cx5036_id,
};

static int __init cx5036_init(void)
{
    int ret;

    LDBG("cx5036_init\n");
    ret = i2c_add_driver(&cx5036_driver);
    return ret;	

}

static void __exit cx5036_exit(void)
{
    i2c_del_driver(&cx5036_driver);
}

MODULE_AUTHOR("Templeton Tsai Dyna-Image Corporation.");
MODULE_DESCRIPTION("CX5036 driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(cx5036_init);
module_exit(cx5036_exit);



