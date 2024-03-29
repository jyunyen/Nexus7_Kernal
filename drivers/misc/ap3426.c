/*
 * This file is part of the AP3426, AP3212C and AP3216C sensor driver.
 * AP3426 is combined proximity and ambient light sensor.
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
 * Filename: ap3426.c
 *
 * Summary:
 *	AP3426 device driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 02/02/12 YC       1. Modify irq function to seperate two interrupt routine. 
 *					 2. Fix the index of reg array error in em write. 
 * 02/22/12 YC       3. Merge AP3426 and AP3216C into the same driver. (ver 1.8)
 * 03/01/12 YC       Add AP3212C into the driver. (ver 1.8)
 * 07/25/14 John	  Ver.2.1 , ported for Nexus 7
 * 08/21/14 Templeton AP3426 Ver 1.0, ported for Nexus 7
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
#include "ap3426.h"

#define AP3426_DRV_NAME		"ap3426"
#define DRIVER_VERSION		"1"


#define PL_TIMER_DELAY 2000
#define POLLING_MODE 1

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("AP3426: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif
static void plsensor_work_handler(struct work_struct *w);
#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data);
#endif
static int ap3426_set_phthres(struct i2c_client *client, int val);
static int ap3426_set_plthres(struct i2c_client *client, int val);

struct ap3426_data {
    struct i2c_client *client;
    u8 reg_cache[AP3426_NUM_CACHABLE_REGS];//TO-DO
    u8 power_state_before_suspend;
    int irq;
    int hsensor_enable;
    int old_mode;
    struct input_dev	*psensor_input_dev;
    struct input_dev	*lsensor_input_dev;
    struct input_dev	*hsensor_input_dev;
    struct workqueue_struct *plsensor_wq;
    struct work_struct plsensor_work;
#if POLLING_MODE
    struct timer_list pl_timer;
#endif
};

static struct ap3426_data *ap3426_data_g = NULL;
// AP3426 register
static u8 ap3426_reg[AP3426_NUM_CACHABLE_REGS] = 
{0x00,0x01,0x02,0x06,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
    0x10,0x14,0x1A,0x1B,0x1C,0x1D,
    0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x28,0x29,0x2A,0x2B,0x2C,0x2D, 0x30, 0x32};

// AP3426 range
static int ap3426_range[4] = {32768,8192,2048,512};



static u8 *reg_array;
static int *range;
static int reg_num = 0;

static int cali = 100;
static int misc_ps_opened = 0;
static int misc_ls_opened = 0;

static DEFINE_MUTEX(ap3426_lock);
static DEFINE_MUTEX(ap3426_ls_lock);
static DEFINE_MUTEX(ap3426_ps_lock);
static DEFINE_MUTEX(ap3426_heartbeat_lock);
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

static int __ap3426_read_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    u8 idx = 0xff;
    int val = 0;

    val = i2c_smbus_read_byte_data(client, reg);
	return (val & mask) >> shift;
}

static int __ap3426_write_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift, u8 val)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
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
static int ap3426_get_range(struct i2c_client *client)
{
    u8 idx = __ap3426_read_reg(client, AP3426_REG_ALS_CONF,
	    AP3426_ALS_RANGE_MASK, AP3426_ALS_RANGE_SHIFT); 
    return idx;
}

static int ap3426_set_range(struct i2c_client *client, int range)
{
    return __ap3426_write_reg(client, AP3426_REG_ALS_CONF,
	    AP3426_ALS_RANGE_MASK, AP3426_ALS_RANGE_SHIFT, range);
}


/* mode */
static int ap3426_get_mode(struct i2c_client *client)
{
    int ret;

    ret = __ap3426_read_reg(client, AP3426_REG_SYS_CONF,
	    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT);
    return ret;
}

static int ap3426_set_mode(struct i2c_client *client, int mode)
{
    int ret;

    ret = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
	    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, mode);

    return ret;
}

/* ALS low threshold */
static int ap3426_get_althres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_ALS_THDL_L,
	    AP3426_REG_ALS_THDL_L_MASK, AP3426_REG_ALS_THDL_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_ALS_THDL_H,
	    AP3426_REG_ALS_THDL_H_MASK, AP3426_REG_ALS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_althres(struct i2c_client *client, int val)
{

    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_ALS_THDL_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDL_L,
	    AP3426_REG_ALS_THDL_L_MASK, AP3426_REG_ALS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDL_H,
	    AP3426_REG_ALS_THDL_H_MASK, AP3426_REG_ALS_THDL_H_SHIFT, msb);

    return err;
}

/* ALS high threshold */
static int ap3426_get_ahthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_ALS_THDH_L,
	    AP3426_REG_ALS_THDH_L_MASK, AP3426_REG_ALS_THDH_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_ALS_THDH_H,
	    AP3426_REG_ALS_THDH_H_MASK, AP3426_REG_ALS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_ahthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_ALS_THDH_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDH_L,
	    AP3426_REG_ALS_THDH_L_MASK, AP3426_REG_ALS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDH_H,
	    AP3426_REG_ALS_THDH_H_MASK, AP3426_REG_ALS_THDH_H_SHIFT, msb);

    return err;
}

/* PX low threshold */
static int ap3426_get_plthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_PS_THDL_L,
	    AP3426_REG_PS_THDL_L_MASK, AP3426_REG_PS_THDL_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_PS_THDL_H,
	    AP3426_REG_PS_THDL_H_MASK, AP3426_REG_PS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_plthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_THDL_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDL_L,
	    AP3426_REG_PS_THDL_L_MASK, AP3426_REG_PS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDL_H,
	    AP3426_REG_PS_THDL_H_MASK, AP3426_REG_PS_THDL_H_SHIFT, msb);

    return err;
}

/* PX high threshold */
static int ap3426_get_phthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_PS_THDH_L,
	    AP3426_REG_PS_THDH_L_MASK, AP3426_REG_PS_THDH_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_PS_THDH_H,
	    AP3426_REG_PS_THDH_H_MASK, AP3426_REG_PS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_phthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_THDH_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDH_L,
	    AP3426_REG_PS_THDH_L_MASK, AP3426_REG_PS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDH_H,
	    AP3426_REG_PS_THDH_H_MASK, AP3426_REG_PS_THDH_H_SHIFT, msb);

    return err;
}

static int ap3426_get_ir_value(struct i2c_client *client)
{
    unsigned int lsb, msb, val;


    lsb = __ap3426_read_reg(client, AP3426_REG_IR_DATA_LOW,
	    AP3426_REG_IR_DATA_LOW_MASK, AP3426_REG_IR_DATA_LOW_SHIFT);
    LDBG("%s, IR = %d\n", __func__, (u32)(lsb));
    msb = __ap3426_read_reg(client, AP3426_REG_IR_DATA_HIGH,
	    AP3426_REG_IR_DATA_HIGH_MASK, AP3426_REG_IR_DATA_HIGH_SHIFT);
    LDBG("%s, IR = %d\n", __func__, (u32)(msb));


//    lsb = i2c_smbus_read_byte_data(client, AP3426_REG_IR_DATA_LOW);
  //  msb = i2c_smbus_read_byte_data(client, AP3426_REG_IR_DATA_HIGH);
    val = (msb << 8) | lsb;
    return val;
}

static int ap3426_get_adc_value(struct i2c_client *client)
{
    unsigned int lsb, msb, val;
#ifdef LSC_DBG
    unsigned int tmp,range_idx;
#endif

    lsb = i2c_smbus_read_byte_data(client, AP3426_REG_ALS_DATA_LOW);

    if (lsb < 0) {
	return -1;
    }

    msb = i2c_smbus_read_byte_data(client, AP3426_REG_ALS_DATA_HIGH);

    if (msb < 0)
	return -1;

    //range_idx = ap3426_get_range(client);

    //tmp = (((msb << 8) | lsb) * range[range_idx]) >> 16;
    //tmp = tmp * cali / 100;
    val = (msb << 8) | lsb;
    return val;
}


static int ap3426_get_object(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP3426_OBJ_COMMAND);
    val &= AP3426_OBJ_MASK;

    return val >> AP3426_OBJ_SHIFT;
}

static int ap3426_get_intstat(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP3426_REG_SYS_INTSTATUS);
    val &= AP3426_REG_SYS_INT_MASK;

    return val >> AP3426_REG_SYS_INT_SHIFT;
}


static int ap3426_get_px_value(struct i2c_client *client)
{
    int lsb, msb;

    lsb = i2c_smbus_read_byte_data(client, AP3426_REG_PS_DATA_LOW);

    if (lsb < 0) {
	return lsb;
    }

    //LDBG("%s, IR = %d\n", __func__, (u32)(lsb));
    msb = i2c_smbus_read_byte_data(client, AP3426_REG_PS_DATA_HIGH);

    if (msb < 0)
	return msb;

    //LDBG("%s, IR = %d\n", __func__, (u32)(msb));

    return (u32)(((msb & AL3426_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & AL3426_REG_PS_DATA_LOW_MASK));
}




static int ap3426_lsensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if((mode & AP3426_SYS_ALS_ENABLE) == 0){
	mode |= AP3426_SYS_ALS_ENABLE;
	ret = ap3426_set_mode(client,mode);
    }

    return ret;
}

static int ap3426_lsensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if(mode & AP3426_SYS_ALS_ENABLE){
	mode &= ~AP3426_SYS_ALS_ENABLE;
	if(mode == AP3426_SYS_DEV_RESET)
	    mode = 0;
	ret = ap3426_set_mode(client,mode);
    }

    return ret;
}

static int ap3426_register_lsensor_device(struct i2c_client *client, struct ap3426_data *data)
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


static void ap3426_unregister_lsensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->lsensor_input_dev);
}
static int ap3426_register_heartbeat_sensor_device(struct i2c_client *client, struct ap3426_data *data)
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
    set_bit(EV_REL, input_dev->evbit);
    input_set_capability(input_dev, EV_REL, ABS_WHEEL);
    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for heartbeat sensor\n", __FUNCTION__);
	goto done;
    }
done:
    return rc;
}

static void ap3426_unregister_heartbeat_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->hsensor_input_dev);
}


static int ap3426_psensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if((mode & AP3426_SYS_PS_ENABLE) == 0){
	mode |= AP3426_SYS_PS_ENABLE;
	ret = ap3426_set_mode(client,mode);
    }

    return ret;
}

static int ap3426_psensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if(mode & AP3426_SYS_PS_ENABLE){
	mode &= ~AP3426_SYS_PS_ENABLE;
	if(mode == AP3426_SYS_DEV_RESET)
	    mode = AP3426_SYS_DEV_DOWN;
	ret = ap3426_set_mode(client,mode);
    }
    return ret;
}


static int ap3426_register_psensor_device(struct i2c_client *client, struct ap3426_data *data)
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

static void ap3426_unregister_psensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->psensor_input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend ap3426_early_suspend;
static void ap3426_suspend(struct early_suspend *h)
{

    if (misc_ps_opened)
	ap3426_psensor_disable(ap3426_data_g->client);
    if (misc_ls_opened)
	ap3426_lsensor_disable(ap3426_data_g->client);
}

static void ap3426_resume(struct early_suspend *h)
{

    if (misc_ls_opened)
	ap3426_lsensor_enable(ap3426_data_g->client);
    if (misc_ps_opened)
	ap3426_psensor_enable(ap3426_data_g->client);
}
#endif


/* range */
static ssize_t ap3426_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%i\n", ap3426_get_range(data->client));
}

static ssize_t ap3426_store_range(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
	return -EINVAL;

    ret = ap3426_set_range(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}




/* mode */
static ssize_t ap3426_show_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_mode(data->client));
}

static ssize_t ap3426_store_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_mode(data->client, val);

    if (ret < 0)
	return ret;
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}


static ssize_t ap3426_ls_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long mode;
    int ret;

    LDBG("mode = %s,%s\n", __func__,buf);
    if (strict_strtoul(buf, 10, &mode) < 0)
	return -EINVAL;

    if(mode == AP3426_SYS_ALS_ENABLE){
	ap3426_set_althres(data->client, 1000);
	ap3426_set_ahthres(data->client, 2000);

	ret = ap3426_set_range(data->client, AP3426_ALS_RANGE_2);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_LEDD,
		AP3426_REG_PS_LEDD_MASK, AP3426_REG_PS_LEDD_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_MEAN,
		AP3426_REG_PS_MEAN_MASK, AP3426_REG_PS_MEAN_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_INTEGR,
		AP3426_REG_PS_INTEGR_MASK, AP3426_REG_PS_INTEGR_SHIFT, 0);

	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_PS_ENABLE);
	misc_ls_opened = 1;
	misc_ps_opened = 1;
	if (ret < 0)
	    return ret;
    } else if (mode == AP3426_SYS_DEV_DOWN){
	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_RESET);
	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_DOWN);
	misc_ls_opened = 0;
	misc_ps_opened = 0;
    }
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
	/*
    mutex_lock(&ap3426_ls_lock);

    if(!(data -> hsensor_enable)){
	data -> old_mode = ap3426_get_mode(data->client);
	if((mode == AP3426_SYS_ALS_ENABLE) && data -> old_mode == AP3426_SYS_PS_ENABLE) {
	    ap3426_set_althres(data->client, 1000);
	    ap3426_set_ahthres(data->client, 2000);
	    misc_ls_opened = 1;
	    ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_PS_ENABLE);
	    if (ret < 0)
		return ret;
	} else if(mode == AP3426_SYS_ALS_ENABLE && data -> old_mode == AP3426_SYS_DEV_DOWN){
	    ap3426_set_althres(data->client, 1000);
	    ap3426_set_ahthres(data->client, 2000);
	    misc_ls_opened = 1;
	    ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_ENABLE);
	    if (ret < 0)
		return ret;
	} else if (mode == AP3426_SYS_DEV_DOWN){
	    if(data -> old_mode != AP3426_SYS_ALS_PS_ENABLE && data-> hsensor_enable != 1) {
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_RESET);
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_DOWN);
		misc_ls_opened = 0;
		misc_ps_opened = 0;
	    } else {
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_PS_ENABLE);
		misc_ls_opened = 0;

	    }
	}
	data -> old_mode = ap3426_get_mode(data->client);
    } else {
	if(mode == AP3426_SYS_DEV_DOWN && misc_ps_opened == 1) {
	    data -> old_mode = AP3426_SYS_PS_ENABLE;
	    misc_ls_opened = 0;
	} else if(mode == AP3426_SYS_ALS_ENABLE) {
	    data -> old_mode = AP3426_SYS_ALS_ENABLE;
	    misc_ls_opened = 1;

	} else {
	    data -> old_mode = AP3426_SYS_DEV_DOWN;
	    misc_ps_opened = 0;
	    misc_ls_opened = 0;
	}

    }
    LDBG("1.data -> old_mode = %d\n", data -> old_mode);
    mutex_unlock(&ap3426_ls_lock);
*/
    return count;
}


static ssize_t ap3426_ps_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long mode;
    int ret;
    

    LDBG("mode = %s,%s\n", __func__,buf);
    if (strict_strtoul(buf, 10, &mode) < 0)
	return -EINVAL;

    mutex_lock(&ap3426_ps_lock);




    if(!(data -> hsensor_enable)) {
	data -> old_mode = ap3426_get_mode(data->client);
	if(mode == AP3426_SYS_PS_ENABLE && data -> old_mode == AP3426_SYS_ALS_ENABLE) {
	    ret = ap3426_set_plthres(data->client, 100);
	    ret = ap3426_set_phthres(data->client, 500);
	    misc_ps_opened = 1;
	    ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_PS_ENABLE);
	    if (ret < 0)
		return ret;


	} else if(mode == AP3426_SYS_PS_ENABLE && data -> old_mode == AP3426_SYS_DEV_DOWN){
	    ret = ap3426_set_plthres(data->client, 100);
	    ret = ap3426_set_phthres(data->client, 500);
	    misc_ps_opened = 1;
	    ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_PS_ENABLE);
	    if (ret < 0)
		return ret;
	} else if(mode == AP3426_SYS_DEV_DOWN){
	    if(data -> old_mode != AP3426_SYS_ALS_PS_ENABLE && data-> hsensor_enable != 1) {
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_RESET);
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_DOWN);
		misc_ps_opened = 0;
		misc_ls_opened = 0;
	    } else {
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_ENABLE);
		misc_ps_opened = 0;

	    }
	}
	data -> old_mode = ap3426_get_mode(data->client);
    } else {
	if(mode == AP3426_SYS_DEV_DOWN && misc_ls_opened == 1) {
	    data -> old_mode = AP3426_SYS_ALS_ENABLE;
	    misc_ps_opened = 0;
	} else if(mode == AP3426_SYS_PS_ENABLE) {
	    data -> old_mode = AP3426_SYS_PS_ENABLE;
	    misc_ls_opened = 1;

	} else {
	    data -> old_mode = AP3426_SYS_DEV_DOWN;
	    misc_ps_opened = 0;
	    misc_ls_opened = 0;
	}
    }
    LDBG("2.data -> old_mode = %d\n", data -> old_mode);
    mutex_unlock(&ap3426_ps_lock);
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}

static ssize_t ap3426_hs_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long mode;
    int ret;

    LDBG("mode = %s,%s\n", __func__,buf);
    if (strict_strtoul(buf, 10, &mode) < 0)
	return -EINVAL;

    mutex_lock(&ap3426_heartbeat_lock);


    if(mode == AP3426_SYS_HS_ENABLE) {
	data -> hsensor_enable = 1;

	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_CONF,
		AP3426_REG_PS_CONF_MASK, AP3426_REG_PS_CONF_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_DC_1,
		AP3426_REG_PS_DC_1_MASK, AP3426_REG_PS_DC_1_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_DC_2,
		AP3426_REG_PS_DC_2_MASK, AP3426_REG_PS_DC_2_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_LEDD,
		AP3426_REG_PS_LEDD_MASK, AP3426_REG_PS_LEDD_SHIFT, 1);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_MEAN,
		AP3426_REG_PS_MEAN_MASK, AP3426_REG_PS_MEAN_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_PERSIS,
		AP3426_REG_PS_PERSIS_MASK, AP3426_REG_PS_PERSIS_SHIFT, 0);
	ret = ap3426_set_plthres(data->client, 0);
	ret = ap3426_set_phthres(data->client, 535);
	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_PS_ENABLE);

	if (ret < 0)
	    return ret;
    } else {
	data -> hsensor_enable = 0;
	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_RESET);
	LDBG("3.data -> old_mode = %d\n", data -> old_mode);
	switch(data -> old_mode) {
	    case AP3426_SYS_PS_ENABLE:
		ret = ap3426_set_plthres(data->client, 100);
		ret = ap3426_set_phthres(data->client, 500);
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_PS_ENABLE);
		data -> old_mode = AP3426_SYS_PS_ENABLE;
		misc_ps_opened = 1;
		misc_ls_opened = 0;
		break;
	    case AP3426_SYS_ALS_ENABLE:
		ap3426_set_althres(data->client, 1000);
		ap3426_set_ahthres(data->client, 2000);
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_ENABLE);
		data -> old_mode = AP3426_SYS_ALS_ENABLE;
		misc_ls_opened = 1;
		misc_ps_opened = 0;
		break;
	    case AP3426_SYS_ALS_PS_ENABLE:
		ret = ap3426_set_plthres(data->client, 100);
		ret = ap3426_set_phthres(data->client, 500);
		ap3426_set_althres(data->client, 1000);
		ap3426_set_ahthres(data->client, 2000);
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_PS_ENABLE);
		data -> old_mode = AP3426_SYS_ALS_PS_ENABLE;
		misc_ps_opened = 1;
		misc_ls_opened = 1;
		break;
	    default:
		ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
			AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_DOWN);
		data -> old_mode = AP3426_SYS_DEV_DOWN;
		misc_ps_opened = 0;
		misc_ls_opened = 0;

	}
    }
    mutex_unlock(&ap3426_heartbeat_lock);
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}

/* lux */
static ssize_t ap3426_show_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;

    /* No LUX data if power down */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
	return sprintf((char*) buf, "%s\n", "Please power up first!");

    return sprintf(buf, "%d\n", ap3426_get_adc_value(data->client));
}


/* ir data */
static ssize_t ap3426_show_irdata(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;

    /* No LUX data if power down */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
	return sprintf((char*) buf, "%s\n", "Please power up first!");

    return sprintf(buf, "%d\n", ap3426_get_ir_value(data->client));
}

/* Px data */
static ssize_t ap3426_show_pxvalue(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;

    /* No Px data if power down */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
	return -EBUSY;

    return sprintf(buf, "%d\n", ap3426_get_px_value(data->client));
}



/* proximity object detect */
static ssize_t ap3426_show_object(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_object(data->client));
}



/* ALS low threshold */
static ssize_t ap3426_show_althres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_althres(data->client));
}

static ssize_t ap3426_store_althres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_althres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* ALS high threshold */
static ssize_t ap3426_show_ahthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_ahthres(data->client));
}

static ssize_t ap3426_store_ahthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_ahthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}


/* Px low threshold */
static ssize_t ap3426_show_plthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_plthres(data->client));
}

static ssize_t ap3426_store_plthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_plthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}


/* Px high threshold */
static ssize_t ap3426_show_phthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_phthres(data->client));
}

static ssize_t ap3426_store_phthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_phthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* calibration */
static ssize_t ap3426_show_calibration_state(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    return sprintf(buf, "%d\n", cali);
}

static ssize_t ap3426_store_calibration_state(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    int stdls, lux; 
    char tmp[10];

    LDBG("DEBUG ap3426_store_calibration_state..\n");

    /* No LUX data if not operational */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
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

    lux = ap3426_get_adc_value(data->client);
    cali = stdls * 100 / lux;

    return -EBUSY;
}


#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap3426_em_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    int i;
    u8 tmp;

    LDBG("DEBUG ap3426_em_read..\n");

    for (i = 0; i < reg_num; i++)
    {
	tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);

	LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
    }

    return 0;
}

static ssize_t ap3426_em_write(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    u32 addr,val,idx=0;
    int ret = 0;

    LDBG("DEBUG ap3426_em_write..\n");

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
    __ATTR(range, 0666, ap3426_show_range, ap3426_store_range),
    __ATTR(mode, 0666, ap3426_show_mode, ap3426_store_mode),
    __ATTR(lsensor, 0666, ap3426_show_mode, ap3426_ls_enable),
    __ATTR(psensor, 0666, ap3426_show_mode, ap3426_ps_enable),
    __ATTR(hsensor, 0666, ap3426_show_mode, ap3426_hs_enable),
    __ATTR(lux, 0666, ap3426_show_lux, NULL),
    __ATTR(irdata, 0666, ap3426_show_irdata, NULL),
    __ATTR(pxvalue, S_IRUGO, ap3426_show_pxvalue, NULL),
    __ATTR(object, S_IRUGO, ap3426_show_object, NULL),
    __ATTR(althres, S_IWUSR | S_IRUGO, ap3426_show_althres, ap3426_store_althres),
    __ATTR(ahthres, S_IWUSR | S_IRUGO, ap3426_show_ahthres, ap3426_store_ahthres),
    __ATTR(plthres, S_IWUSR | S_IRUGO, ap3426_show_plthres, ap3426_store_plthres),
    __ATTR(phthres, S_IWUSR | S_IRUGO, ap3426_show_phthres, ap3426_store_phthres),
    __ATTR(calibration, S_IWUSR | S_IRUGO, ap3426_show_calibration_state, ap3426_store_calibration_state),
#ifdef LSC_DBG
    __ATTR(em, S_IWUSR | S_IRUGO, ap3426_em_read, ap3426_em_write),
#endif

};

static int create_sysfs_interfaces(struct ap3426_data *sensor)
{
    int i;
    struct class *ap3426_class = NULL;
    struct device *ap3426_dev = NULL;
    int ret;

    ap3426_class = class_create(THIS_MODULE, "sensors");
    if (IS_ERR(ap3426_class)) {
	ret = PTR_ERR(ap3426_class);
	ap3426_class = NULL;
	LDBG("%s: could not allocate ap3426_class, ret = %d\n", __func__, ret);
	goto ap3426_class_error;
    }

    ap3426_dev= device_create(ap3426_class,
	    NULL, 0, "%s", "di_sensors");

    if(ap3426_dev == NULL)
	goto ap3426_device_error;
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
	if (device_create_file(ap3426_dev, attributes + i))
	    goto ap3426_create_file_error;

    return 0;

ap3426_create_file_error:
    for ( ; i >= 0; i--)
	device_remove_file(ap3426_dev, attributes + i);

ap3426_device_error:
    class_destroy(ap3426_class);
ap3426_class_error:
    dev_err(&sensor->client->dev, "%s:Unable to create interface\n", __func__);
    return -1;
}
static int ap3426_init_client(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    int i;

    LDBG("DEBUG ap3426_init_client..\n");

    /* read all the registers once to fill the cache.
     * if one of the reads fails, we consider the init failed */
    for (i = 0; i < reg_num; i++) {
	int v = i2c_smbus_read_byte_data(client, reg_array[i]);
	if (v < 0)
	    return -ENODEV;

	data->reg_cache[i] = v;
    }
    /* set defaults */

    ap3426_set_range(client, AP3426_ALS_RANGE_0);
    ap3426_set_mode(data->client, AP3426_SYS_DEV_DOWN);

    return 0;
}

#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data)
{
    struct ap3426_data *data;
    int ret =0;

    data = ap3426_data_g;
    queue_work(data->plsensor_wq, &data->plsensor_work);

    ret = mod_timer(&ap3426_data_g->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");

}
#endif
static void plsensor_work_handler(struct work_struct *w)
{

    struct ap3426_data *data =
	container_of(w, struct ap3426_data, plsensor_work);
    u8 int_stat;
    int pxvalue;
    int obj;
    int ret;
    int value;

    int_stat = ap3426_get_intstat(data->client);


    // ALS int
    if (int_stat & AP3426_REG_SYS_INT_AMASK)
    {
	//LDBG("LS INT Status: %0x\n", int_stat);

	value = ap3426_get_adc_value(data->client);
	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_INTSTATUS,
		AP3426_REG_SYS_INT_AMASK, AP3426_REG_SYS_INT_LS_SHIFT, 0);
	input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
	input_sync(data->lsensor_input_dev);
    }

    // PX int
    if (int_stat & AP3426_REG_SYS_INT_PMASK)
    {
	LDBG("PS INT Status: %0x\n", int_stat);
	obj = ap3426_get_object(data->client);
	pxvalue = ap3426_get_px_value(data->client); 

	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_INTSTATUS,
		AP3426_REG_SYS_INT_PMASK, AP3426_REG_SYS_INT_PS_SHIFT, 0);
	LDBG("%s\n", obj ? "obj near":"obj far");
	input_report_abs(data->psensor_input_dev, ABS_DISTANCE, obj);
	input_sync(data->psensor_input_dev);

	if(data->hsensor_enable) {
	    LDBG("pxvalue = %d\n", pxvalue);
	    input_report_rel(data->hsensor_input_dev, ABS_WHEEL, pxvalue);
	    input_sync(data->hsensor_input_dev);
	}

    }

    enable_irq(data->client->irq);
}
/*
 * I2C layer
 */

static irqreturn_t ap3426_irq(int irq, void *data_)
{
    struct ap3426_data *data = data_;

    LDBG("interrupt\n");
    disable_irq_nosync(data->client->irq);
    queue_work(data->plsensor_wq, &data->plsensor_work);

    return IRQ_HANDLED;
}

static int __devinit ap3426_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct ap3426_data *data;
    int err = 0;

    LDBG("ap3426_probe\n");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
	err = -EIO;
	goto exit_free_gpio;
    }

    reg_array = ap3426_reg;
    range = ap3426_range;
    reg_num = AP3426_NUM_CACHABLE_REGS;

    data = kzalloc(sizeof(struct ap3426_data), GFP_KERNEL);
    if (!data){
	err = -ENOMEM;
	goto exit_free_gpio;
    }

    data->client = client;
    i2c_set_clientdata(client, data);
    data->irq = client->irq;

    /* initialize the AP3426 chip */
    err = ap3426_init_client(client);
    if (err)
	goto exit_kfree;

    err = ap3426_register_lsensor_device(client,data);
    if (err){
	dev_err(&client->dev, "failed to register_lsensor_device\n");
	goto exit_kfree;
    }

    err = ap3426_register_psensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_psensor_device\n");
	goto exit_free_ls_device;
    }

    err = ap3426_register_heartbeat_sensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_heartbeatsensor_device\n");
	goto exit_free_heartbeats_device;
    }

    err = create_sysfs_interfaces(data);
    if (err)
	goto exit_free_ps_device;

#ifdef CONFIG_HAS_EARLYSUSPEND
    ap3426_early_suspend.suspend = ap3426_suspend;
    ap3426_early_suspend.resume  = ap3426_resume;
    ap3426_early_suspend.level   = 0x02;
    register_early_suspend(&ap3426_early_suspend);
#endif


    err = request_threaded_irq(client->irq, NULL, ap3426_irq,
	    IRQF_TRIGGER_FALLING,
	    "ap3426", data);
    if (err) {
	dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,client->irq);
	goto exit_free_ps_device;
    }

    data->plsensor_wq = create_singlethread_workqueue("plsensor_wq");
    if (!data->plsensor_wq) {
	LDBG("%s: create workqueue failed\n", __func__);
	err = -ENOMEM;
	goto err_create_wq_failed;
    }

    INIT_WORK(&data->plsensor_work, plsensor_work_handler);

#if POLLING_MODE
    LDBG("Timer module installing\n");
    setup_timer(&data->pl_timer, pl_timer_callback, 0);
#endif


    ap3426_data_g = data;
    dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);
    return 0;
err_create_wq_failed:
#if POLLING_MODE
    if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
#endif
    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
exit_free_ps_device:
    ap3426_unregister_psensor_device(client,data);

exit_free_heartbeats_device:
    ap3426_unregister_heartbeat_device(client,data);
exit_free_ls_device:
    ap3426_unregister_lsensor_device(client,data);

exit_kfree:
    kfree(data);

exit_free_gpio:
    return err;
}

static int __devexit ap3426_remove(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    free_irq(data->irq, data);

    ap3426_unregister_psensor_device(client,data);
    ap3426_unregister_lsensor_device(client,data);
    ap3426_unregister_heartbeat_device(client,data);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ap3426_early_suspend);
#endif

    ap3426_set_mode(data->client, 0);
    kfree(i2c_get_clientdata(client));

    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
#if POLLING_MODE
    if(&data->pl_timer)
	del_timer(&data->pl_timer);
#endif
    return 0;
}

static const struct i2c_device_id ap3426_id[] = {
    { AP3426_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ap3426_id);

static struct i2c_driver ap3426_driver = {
    .driver = {
	.name	= AP3426_DRV_NAME,
	.owner	= THIS_MODULE,
    },
    .probe	= ap3426_probe,
    .remove	= __devexit_p(ap3426_remove),
    .id_table = ap3426_id,
};

static int __init ap3426_init(void)
{
    int ret;

    LDBG("ap3426_init\n");
    ret = i2c_add_driver(&ap3426_driver);
    return ret;	

}

static void __exit ap3426_exit(void)
{
    i2c_del_driver(&ap3426_driver);
}

MODULE_AUTHOR("Templeton Tsai Dyna-Image Corporation.");
MODULE_DESCRIPTION("AP3426 driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(ap3426_init);
module_exit(ap3426_exit);



