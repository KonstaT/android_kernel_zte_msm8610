/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name		: l3g4200d_gyr_sysfs.c
* Authors		: MH - C&I BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.1.3 sysfs
* Date			: 2011/Jun/24
* Description		: L3G4200D digital output gyroscope sensor API
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS		| DESCRIPTION
* 1.0		| 2010/11/19	| Carmine Iascone	| First Release
* 1.1.0		| 2011/02/28	| Matteo Dameno		| Self Test Added
* 1.1.1		| 2011/05/25	| Matteo Dameno		| Corrects Polling Bug
* 1.1.2		| 2011/05/30	| Matteo Dameno		| Corrects ODR Bug
* 1.1.3		| 2011/06/24	| Matteo Dameno		| Corrects ODR Bug
*******************************************************************************/

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input-polldev.h>
#include <linux/input.h>
#include <linux/slab.h>

#include <linux/l3g4200d.h>

//yaotierui
#include	<linux/module.h>
#include <linux/proc_fs.h> // added by yangze for create proc file 20121011
//[ECID:000000] ZTEBSP wanghaifei 20130906 start, for device tree driver
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h> 
#include <linux/gpio.h>
//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver
#include <linux/delay.h>

/** Maximum polled-device-reported rot speed value value in dps*/
#define FS_MAX			32768

/* l3g4200d gyroscope registers */
#define WHO_AM_I        0x0F

#define CTRL_REG1       0x20    /* CTRL REG1 */
#define CTRL_REG2       0x21    /* CTRL REG2 */
#define CTRL_REG3       0x22    /* CTRL_REG3 */
#define CTRL_REG4       0x23    /* CTRL_REG4 */
#define CTRL_REG5       0x24    /* CTRL_REG5 */

/* CTRL_REG1 */
#define PM_OFF		0x00
#define PM_NORMAL	0x08
#define ENABLE_ALL_AXES	0x07
#define BW00		0x00
#define BW01		0x10
#define BW10		0x20
#define BW11		0x30
#define ODR100		0x00  /* ODR = 100Hz */
#define ODR200		0x40  /* ODR = 200Hz */
#define ODR400		0x80  /* ODR = 400Hz */
#define ODR800		0xC0  /* ODR = 800Hz */

/* CTRL_REG4 bits */
#define	FS_MASK				0x30

#define	SELFTEST_MASK			0x06
#define L3G4200D_SELFTEST_DIS		0x00
#define L3G4200D_SELFTEST_EN_POS	0x02
#define L3G4200D_SELFTEST_EN_NEG	0x04

#define AXISDATA_REG			0x28

#define FUZZ			0
#define FLAT			0
#define AUTO_INCREMENT		0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RESUME_ENTRIES		5


//#define DEBUG 1

/** Registers Contents */
#define WHOAMI_L3G4200D		0x00D3	/* Expected content for WAI register*/

/*
 * L3G4200D gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */

struct l3g4200d_triple {
	short	x,	/* x-axis angular rate data. */
		y,	/* y-axis angluar rate data. */
		z;	/* z-axis angular rate data. */
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {
	{	2,	ODR800|BW10},
	{	3,	ODR400|BW01},
	{	5,	ODR200|BW00},
	{	10,	ODR100|BW00},
};

struct l3g4200d_data {
	struct i2c_client *client;
	struct l3g4200d_gyr_platform_data *pdata;

	struct mutex lock;

	struct input_polled_dev *input_poll_dev;
	int hw_initialized;
	int selftest_enabled;
	atomic_t enabled;
//[ECID:000000] ZTEBSP wanghaifei 20130906 start, for device tree driver
#ifdef CONFIG_OF
        bool    vdd_enabled;
        bool    vio_enabled;
        struct regulator *vdd;
        struct regulator *vio;
#endif
//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver


	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];
};

//[ECID:000000] ZTEBSP wanghaifei 20130906 start, for device tree driver
static int chipid = 0xff;
#ifdef CONFIG_OF
#define L3G4200D_VDD_MIN_UV        2700000
#define L3G4200D_VDD_MAX_UV        3300000
#define L3G4200D_VIO_MIN_UV        1700000
#define L3G4200D_VIO_MAX_UV        1950000
#endif
//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver

static int l3g4200d_i2c_read(struct l3g4200d_data *gyro,
				  u8 *buf, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = gyro->client->addr,
		 .flags = gyro->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = gyro->client->addr,
		 .flags = (gyro->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	err = i2c_transfer(gyro->client->adapter, msgs, 2);

	if (err != 2) {
		dev_err(&gyro->client->dev, "read transfer error: %d\n",err);
		return -EIO;
	}

	return 0;
}

static int l3g4200d_i2c_write(struct l3g4200d_data *gyro,
						u8 *buf,
						int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = gyro->client->addr,
		 .flags = gyro->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	err = i2c_transfer(gyro->client->adapter, msgs, 1);

	if (err != 1) {
		dev_err(&gyro->client->dev, "write transfer error\n");
		return -EIO;
	}

	return 0;
}

static int l3g4200d_register_write(struct l3g4200d_data *gyro, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = l3g4200d_i2c_write(gyro, buf, 1);
		if (err < 0)
			return err;

	return err;
}

static int l3g4200d_register_read(struct l3g4200d_data *gyro, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = l3g4200d_i2c_read(gyro, buf, 1);
	return err;
}

static int l3g4200d_register_update(struct l3g4200d_data *gyro, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = l3g4200d_register_read(gyro, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = l3g4200d_register_write(gyro, buf, reg_address,
				updated_val);
	}
	return err;
}


static int l3g4200d_update_fs_range(struct l3g4200d_data *gyro,
							u8 new_fs)
{
	int res ;
	u8 buf[2];

	buf[0] = CTRL_REG4;

	res = l3g4200d_register_update(gyro, buf, CTRL_REG4,
							FS_MASK, new_fs);

	if (res < 0) {
		pr_err("%s : failed to update fs:0x%02x\n",
			__func__, new_fs);
		return res;
	}
	gyro->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs ) |
		( ~FS_MASK & gyro->resume_state[RES_CTRL_REG4]));

	return res;
}


static int l3g4200d_selftest(struct l3g4200d_data *gyro, u8 enable)
{
	int err = -1;
	u8 buf[2] = {0x00,0x00};
	char reg_address, mask, bit_values;

	reg_address = CTRL_REG4;
	mask = SELFTEST_MASK;
	if (enable > 0)
		bit_values = L3G4200D_SELFTEST_EN_POS;
	else
		bit_values = L3G4200D_SELFTEST_DIS;
	if (atomic_read(&gyro->enabled)) {
		mutex_lock(&gyro->lock);
		err = l3g4200d_register_update(gyro, buf, reg_address,
				mask, bit_values);
		gyro->selftest_enabled = enable;
		mutex_unlock(&gyro->lock);
		if (err < 0)
			return err;
		gyro->resume_state[RES_CTRL_REG4] = ((mask & bit_values) |
				( ~mask & gyro->resume_state[RES_CTRL_REG4]));
	}
	return err;
}


static int l3g4200d_update_odr(struct l3g4200d_data *gyro,
				int poll_interval)
{
	int err = -1;
	int i;
	u8 config[2];

	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_interval) || (i == 0)){
			break;
		}

	}

//[ECID:000000] ZTEBSP wanghaifei start 20120423, don't change pm here
//	config[1] = odr_table[i].mask;
//	config[1] |= (ENABLE_ALL_AXES + PM_NORMAL);
	config[1] = (gyro->resume_state[RES_CTRL_REG1] & 0x0f) | odr_table[i].mask;
//[ECID:000000] ZTEBSP wanghaifei end 20120423, don't change pm here

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&gyro->enabled)) {
		config[0] = CTRL_REG1;
		err = l3g4200d_i2c_write(gyro, config, 1);
		if (err < 0)
			return err;
		gyro->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;
}

/* gyroscope data readout */
static int l3g4200d_get_data(struct l3g4200d_data *gyro,
			     struct l3g4200d_triple *data)
{
	int err;
	unsigned char gyro_out[6];
	/* y,p,r hardware data */
	s16 hw_d[3] = { 0 };

	gyro_out[0] = (AUTO_INCREMENT | AXISDATA_REG);

	err = l3g4200d_i2c_read(gyro, gyro_out, 6);

	if (err < 0)
		return err;

	hw_d[0] = (s16) (((gyro_out[1]) << 8) | gyro_out[0]);
	hw_d[1] = (s16) (((gyro_out[3]) << 8) | gyro_out[2]);
	hw_d[2] = (s16) (((gyro_out[5]) << 8) | gyro_out[4]);

	data->x = ((gyro->pdata->negate_x) ? (-hw_d[gyro->pdata->axis_map_x])
		   : (hw_d[gyro->pdata->axis_map_x]));
	data->y = ((gyro->pdata->negate_y) ? (-hw_d[gyro->pdata->axis_map_y])
		   : (hw_d[gyro->pdata->axis_map_y]));
	data->z = ((gyro->pdata->negate_z) ? (-hw_d[gyro->pdata->axis_map_z])
		   : (hw_d[gyro->pdata->axis_map_z]));

#ifdef DEBUG
	/* pr_info("gyro_out: y = %d p = %d r= %d\n",
		data->y, data->p, data->r);*/
#endif

	return err;
}

static void l3g4200d_report_values(struct l3g4200d_data *l3g,
						struct l3g4200d_triple *data)
{
	struct input_dev *input = l3g->input_poll_dev->input;
	input_report_abs(input, ABS_X, data->x);
	input_report_abs(input, ABS_Y, data->y);
	input_report_abs(input, ABS_Z, data->z);
	input_sync(input);
}

static int l3g4200d_hw_init(struct l3g4200d_data *gyro)
{
	int err = -1;
	u8 buf[6];

	pr_info("%s hw init\n", L3G4200D_GYR_DEV_NAME);

	buf[0] = (AUTO_INCREMENT | CTRL_REG1);
	buf[1] = gyro->resume_state[RES_CTRL_REG1];
	buf[2] = gyro->resume_state[RES_CTRL_REG2];
	buf[3] = gyro->resume_state[RES_CTRL_REG3];
	buf[4] = gyro->resume_state[RES_CTRL_REG4];
	buf[5] = gyro->resume_state[RES_CTRL_REG5];

	err = l3g4200d_i2c_write(gyro, buf, 5);
	if (err < 0)
		return err;

	gyro->hw_initialized = 1;

	return err;
}

//[ECID:000000] ZTEBSP wanghaifei 20130906 start, for device tree driver
#ifdef CONFIG_OF
static int l3g4200d_power_init(struct l3g4200d_data *data, bool on)
{
        int rc;
        int vol_uv;
        int enabled = 0; 

        if (!on) {
                if (data->vdd_enabled) {
                        regulator_disable(data->vdd);
                        data->vdd_enabled = false;
                }    
                if (data->vio_enabled) {
                        regulator_disable(data->vio);
                        data->vio_enabled = false;
                }    

                if (regulator_count_voltages(data->vdd) > 0) 
                        regulator_set_voltage(data->vdd, 0, L3G4200D_VDD_MAX_UV);

                regulator_put(data->vdd);

                if (regulator_count_voltages(data->vio) > 0) 
                        regulator_set_voltage(data->vio, 0, L3G4200D_VIO_MAX_UV);

                regulator_put(data->vio);
        } else {
                data->vdd = regulator_get(&data->client->dev, "vdd");
                if (IS_ERR(data->vdd)) {
                        rc = PTR_ERR(data->vdd);
                        dev_err(&data->client->dev,
                                        "Regulator get failed vdd rc=%d\n", rc); 
                        return rc;
                }    
                vol_uv = regulator_get_voltage(data->vdd);
                enabled = regulator_is_enabled(data->vdd);
                dev_info(&data->client->dev, "vdd current voltate is %duv, enabled = %d ", vol_uv, enabled);
                if ((vol_uv >= L3G4200D_VDD_MIN_UV) && (vol_uv <= L3G4200D_VDD_MAX_UV)) {
                        dev_info(&data->client->dev,"no need to set vdd voltage again");
                } else if (regulator_count_voltages(data->vdd) > 0) { 
                        rc = regulator_set_voltage(data->vdd, L3G4200D_VDD_MIN_UV,
                                        L3G4200D_VDD_MAX_UV);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator set failed vdd rc=%d\n",
                                                rc);
                                goto reg_vdd_put;
                        }
                }
                if (enabled == 1) {
                        dev_info(&data->client->dev,"no need to enable vdd voltage again");
                } else if (!data->vdd_enabled) {
                        rc = regulator_enable(data->vdd);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator vdd enable failed rc=%d\n", rc);
                                goto reg_vdd_put;
                        }
                        data->vdd_enabled = true;
                }

                data->vio = regulator_get(&data->client->dev, "vio");
                if (IS_ERR(data->vio)) {
                        rc = PTR_ERR(data->vio);
                        dev_err(&data->client->dev,
                                        "Regulator get failed vio rc=%d\n", rc);
                        goto reg_vdd_dis;
                }

                vol_uv = regulator_get_voltage(data->vio);
                enabled = regulator_is_enabled(data->vio);
                dev_info(&data->client->dev, "vio current voltate is %duv, enabled = %d ", vol_uv, enabled);
                if ((vol_uv >= L3G4200D_VIO_MIN_UV) && (vol_uv <= L3G4200D_VIO_MAX_UV)) {
                        dev_info(&data->client->dev,"no need to set vio voltage again");
                } else if (regulator_count_voltages(data->vio) > 0) {
                        rc = regulator_set_voltage(data->vio, L3G4200D_VIO_MIN_UV,
                                        L3G4200D_VIO_MAX_UV);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator set failed vio rc=%d\n", rc);
                                goto reg_vio_put;
                        }
                }
                if (enabled == 1) {
                        dev_info(&data->client->dev,"no need to enable vio voltage again");
                } else if (!data->vio_enabled) {
                        rc = regulator_enable(data->vio);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator vio enable failed rc=%d\n", rc);
                                goto reg_vio_put;
                        }
                        data->vio_enabled = true;
                        msleep(10);
                }
        }
        return 0;

reg_vio_put:
        regulator_put(data->vio);
reg_vdd_dis:
        if (data->vdd_enabled){
                regulator_disable(data->vdd);
                data->vdd_enabled = false;
        }
reg_vdd_put:
        regulator_put(data->vdd);
        return rc;
}

#else

static int l3g4200d_power_init(struct l3g4200d_data *data, bool on)
{
        return 0;
}
#endif
//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver

static void l3g4200d_device_power_off(struct l3g4200d_data *dev_data)
{
	int err;
	u8 buf[2];

	pr_info("%s power off\n", L3G4200D_GYR_DEV_NAME);

	buf[0] = CTRL_REG1;
	buf[1] = PM_OFF;
	err = l3g4200d_i2c_write(dev_data, buf, 1);
	if (err < 0)
		dev_err(&dev_data->client->dev, "soft power off failed\n");

	if (dev_data->pdata->power_off) {
		dev_data->pdata->power_off();
		dev_data->hw_initialized = 0;
	}

	if (dev_data->hw_initialized)
		dev_data->hw_initialized = 0;

}

static int l3g4200d_device_power_on(struct l3g4200d_data *dev_data)
{
	int err;

	if (dev_data->pdata->power_on) {
		err = dev_data->pdata->power_on();
		if (err < 0)
			return err;
	}


	if (!dev_data->hw_initialized) {
		err = l3g4200d_hw_init(dev_data);
		if (err < 0) {
			l3g4200d_device_power_off(dev_data);
			return err;
		}
	}

	return 0;
}

static int l3g4200d_enable(struct l3g4200d_data *dev_data)
{
	int err;

	if (!atomic_cmpxchg(&dev_data->enabled, 0, 1)) {

		err = l3g4200d_device_power_on(dev_data);
		if (err < 0) {
			atomic_set(&dev_data->enabled, 0);
			return err;
		}
	}

	return 0;
}

static int l3g4200d_disable(struct l3g4200d_data *dev_data)
{
	if (atomic_cmpxchg(&dev_data->enabled, 1, 0))
		l3g4200d_device_power_off(dev_data);

	return 0;
}

static ssize_t attr_polling_rate_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	mutex_lock(&gyro->lock);
	val = gyro->input_poll_dev->poll_interval;
	mutex_unlock(&gyro->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max((unsigned int)interval_ms,gyro->pdata->min_interval);
	mutex_lock(&gyro->lock);
	gyro->input_poll_dev->poll_interval = interval_ms;
	gyro->pdata->poll_interval = interval_ms;
	l3g4200d_update_odr(gyro, interval_ms);
	mutex_unlock(&gyro->lock);
	return size;
}

static ssize_t attr_range_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int range = 0;
	char val;
	mutex_lock(&gyro->lock);
	val = gyro->pdata->fs_range;
	switch (val) {
	case L3G4200D_GYR_FS_250DPS:
		range = 250;
		break;
	case L3G4200D_GYR_FS_500DPS:
		range = 500;
		break;
	case L3G4200D_GYR_FS_2000DPS:
		range = 2000;
		break;
	}
	mutex_unlock(&gyro->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&gyro->lock);
	gyro->pdata->fs_range = val;
	l3g4200d_update_fs_range(gyro, val);
	mutex_unlock(&gyro->lock);
	return size;
}

static ssize_t attr_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int val = atomic_read(&gyro->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
       printk("gyro enable \n");
	if (val)
		l3g4200d_enable(gyro);
	else
		l3g4200d_disable(gyro);

	return size;
}

static ssize_t attr_get_selftest(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	mutex_lock(&gyro->lock);
	val = gyro->selftest_enabled;
	mutex_unlock(&gyro->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_selftest(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	l3g4200d_selftest(gyro, val);

	return size;
}

//#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&gyro->lock);
	x[0] = gyro->reg_addr;
	mutex_unlock(&gyro->lock);
	x[1] = val;
	rc = l3g4200d_i2c_write(gyro, x, 1);
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&gyro->lock);
	data = gyro->reg_addr;
	mutex_unlock(&gyro->lock);
	rc = l3g4200d_i2c_read(gyro, &data, 1);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);

	gyro->reg_addr = val;

	mutex_unlock(&gyro->lock);

	return size;
}
//#endif /* DEBUG */

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0664, attr_polling_rate_show, attr_polling_rate_store),
	__ATTR(enable_device, 0664, attr_enable_show, attr_enable_store),

	__ATTR(range, 0664, attr_range_show, attr_range_store),
	__ATTR(enable_selftest, 0664, attr_get_selftest, attr_set_selftest),

	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),

};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void l3g4200d_input_poll_func(struct input_polled_dev *dev)
{
	struct l3g4200d_data *gyro = dev->private;

	struct l3g4200d_triple data_out;

	int err;

	/* dev_data = container_of((struct delayed_work *)work,
				 struct l3g4200d_data, input_work); */

//[ECID:000000] ZTEBSP wanghaifei start 20120423, return if not enabled
	if (!atomic_read(&gyro->enabled)) {
		return;
	}
//[ECID:000000] ZTEBSP wanghaifei end 20120423, return if not enabled
	mutex_lock(&gyro->lock);
	err = l3g4200d_get_data(gyro, &data_out);
	if (err < 0)
		dev_err(&gyro->client->dev, "get_gyroscope_data failed\n");
	else
	{
    //       printk("gyro x=%d, y=%d, z=%d\n",data_out.x,data_out.y,data_out.z);
           l3g4200d_report_values(gyro, &data_out);
	}
//[ECID:000000] ZTEBSP wanghaifei start 20120423, change ODR rate if poll interval changed 
	if (gyro->pdata->poll_interval != gyro->input_poll_dev->poll_interval) {
		gyro->pdata->poll_interval = gyro->input_poll_dev->poll_interval;
		l3g4200d_update_odr(gyro, gyro->pdata->poll_interval);
	}
//[ECID:000000] ZTEBSP wanghaifei end 20120423, change ODR rate if poll interval changed 

	mutex_unlock(&gyro->lock);

}

//[ECID:000000] ZTEBSP wanghaifei start 20120423, remove open and close
/*
int l3g4200d_input_open(struct input_dev *input)
{
	struct l3g4200d_data *gyro = input_get_drvdata(input);

	return l3g4200d_enable(gyro);
}

void l3g4200d_input_close(struct input_dev *dev)
{
	struct l3g4200d_data *gyro = input_get_drvdata(dev);

	l3g4200d_disable(gyro);
}
*/
//[ECID:000000] ZTEBSP wanghaifei end 20120423, remove open and close

static int l3g4200d_validate_pdata(struct l3g4200d_data *gyro)
{
	/* checks for correctness of minimal polling period */
	gyro->pdata->min_interval =
		max((unsigned int) L3G4200D_MIN_POLL_PERIOD_MS,
						gyro->pdata->min_interval);

	gyro->pdata->poll_interval = max(gyro->pdata->poll_interval,
			gyro->pdata->min_interval);

	if (gyro->pdata->axis_map_x > 2 ||
	    gyro->pdata->axis_map_y > 2 ||
	    gyro->pdata->axis_map_z > 2) {
		dev_err(&gyro->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			gyro->pdata->axis_map_x,
			gyro->pdata->axis_map_y,
			gyro->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (gyro->pdata->negate_x > 1 ||
	    gyro->pdata->negate_y > 1 ||
	    gyro->pdata->negate_z > 1) {
		dev_err(&gyro->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			gyro->pdata->negate_x,
			gyro->pdata->negate_y,
			gyro->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (gyro->pdata->poll_interval < gyro->pdata->min_interval) {
		dev_err(&gyro->client->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int l3g4200d_input_init(struct l3g4200d_data *gyro)
{
	int err = -1;
	struct input_dev *input;


	gyro->input_poll_dev = input_allocate_polled_device();
	if (!gyro->input_poll_dev) {
		err = -ENOMEM;
		dev_err(&gyro->client->dev,
			"input device allocate failed\n");
		goto err0;
	}

	gyro->input_poll_dev->private = gyro;
	gyro->input_poll_dev->poll = l3g4200d_input_poll_func;
	gyro->input_poll_dev->poll_interval = gyro->pdata->poll_interval;
	gyro->input_poll_dev->poll_interval_max = 200;

	input = gyro->input_poll_dev->input;

//[ECID:000000] ZTEBSP wanghaifei start 20120423, remove open and close
//	input->open = l3g4200d_input_open;
//	input->close = l3g4200d_input_close;
//[ECID:000000] ZTEBSP wanghaifei end 20120423, remove open and close

	input->id.bustype = BUS_I2C;
	input->dev.parent = &gyro->client->dev;

	input_set_drvdata(gyro->input_poll_dev->input, gyro);

	set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_X, -FS_MAX, FS_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Y, -FS_MAX, FS_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Z, -FS_MAX, FS_MAX, FUZZ, FLAT);

	//input->name = L3G4200D_GYR_DEV_NAME;
	input->name = "zte_gyro";

	err = input_register_polled_device(gyro->input_poll_dev);
	if (err) {
		dev_err(&gyro->client->dev,
			"unable to register input polled device %s\n",
			gyro->input_poll_dev->input->name);
		goto err1;
	}

	return 0;

err1:
	input_free_polled_device(gyro->input_poll_dev);
err0:
	return err;
}

static void l3g4200d_input_cleanup(struct l3g4200d_data *gyro)
{
	input_unregister_polled_device(gyro->input_poll_dev);
	input_free_polled_device(gyro->input_poll_dev);
}

//[ECID:000000] ZTEBSP wanghaifei 20130906 start, for device tree driver
#ifdef CONFIG_OF
static int l3g4200d_parse_dt(struct device *dev,
                                struct l3g4200d_gyr_platform_data *pdata)
{
        struct device_node *np = dev->of_node;
        u32 temp_val;
        int rc;

        rc = of_property_read_u32(np, "st,min-interval", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read min-interval\n");
                return rc;
        } else {
                pdata->min_interval = temp_val;
        }

        rc = of_property_read_u32(np, "st,poll-interval", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read init-interval\n");
                return rc;
        } else {
                pdata->poll_interval = temp_val;
        }

        rc = of_property_read_u32(np, "st,axis-map-x", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read axis-map_x\n");
                return rc;
        } else {
                pdata->axis_map_x = (u8)temp_val;
        }

        rc = of_property_read_u32(np, "st,axis-map-y", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read axis_map_y\n");
                return rc;
        } else {
                pdata->axis_map_y = (u8)temp_val;
        }

        rc = of_property_read_u32(np, "st,axis-map-z", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read axis-map-z\n");
                return rc;
        } else {
                pdata->axis_map_z = (u8)temp_val;
        }

        rc = of_property_read_u32(np, "st,fs-range", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read fs-range\n");
                return rc;
        } else {
                switch (temp_val) {
                case 250:
                        pdata->fs_range = L3G4200D_GYR_FS_250DPS;
                        break;
                case 500:
                        pdata->fs_range = L3G4200D_GYR_FS_500DPS;
                        break;
                case 2000:
                        pdata->fs_range = L3G4200D_GYR_FS_2000DPS;
                        break;
                default:
                        pdata->fs_range = L3G4200D_GYR_FS_2000DPS;
                        break;
                }
        }

        pdata->negate_x = of_property_read_bool(np, "st,negate-x");

        pdata->negate_y = of_property_read_bool(np, "st,negate-y");

        pdata->negate_z = of_property_read_bool(np, "st,negate-z");

        temp_val = of_get_named_gpio(np, "st,gpio-irq", 0);
        if (gpio_is_valid(temp_val)) {
                rc = gpio_request(temp_val, "l3g4200d_irq");
                if (rc) {
                        dev_err(dev, "irq gpio request failed");
                } else
                        gpio_direction_input(temp_val);
        }  

        return 0;
}
#else
static int l3g4200d_parse_dt(struct device *dev,
                                struct l3g4200d_gyr_platform_data *pdata)
{
        return -ENODEV;
}
#endif /* !CONFIG_OF */
//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver

static int l3g4200d_probe(struct i2c_client *client,
					const struct i2c_device_id *devid)
{
	struct l3g4200d_data *gyro;
	struct l3g4200d_gyr_platform_data *pdata = NULL;

	int err = -1;
	u8 data;


	pr_info("TI-ST:%s: probe start.\n", L3G4200D_GYR_DEV_NAME);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable:1\n");
		err = -ENODEV;
		goto err0;
	}

//[ECID:000000] ZTEBSP wanghaifei 20130906 start, for device tree driver
        if (client->dev.of_node) {
                printk("l3g4200d use device tree\n");
                pdata = devm_kzalloc(&client->dev, sizeof(struct l3g4200d_gyr_platform_data), GFP_KERNEL);
                if (!pdata) {
                        dev_err(&client->dev, "Failed to allocate memory\n");
                        return -ENOMEM;
                }    
                err = l3g4200d_parse_dt(&client->dev, pdata);
                if (err) {
                        dev_err(&client->dev,
                                "Unable to parse platfrom data err=%d\n", err);
                        return err; 
                }    
        } else {
                printk("l3g4200d not use device tree\n");
                pdata = client->dev.platform_data;
        }  

	if (pdata == NULL) {
//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err0;
	}


	gyro = kzalloc(sizeof(*gyro), GFP_KERNEL);
	if (gyro == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&gyro->lock);
	mutex_lock(&gyro->lock);
	gyro->client = client;



	gyro->pdata = kmalloc(sizeof(*gyro->pdata), GFP_KERNEL);
	if (gyro->pdata == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err1;
	}
	memcpy(gyro->pdata, pdata, sizeof(*gyro->pdata));

	err = l3g4200d_validate_pdata(gyro);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, gyro);

	if (gyro->pdata->init) {
		err = gyro->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err1_1;
		}
	}

	memset(gyro->resume_state, 0, ARRAY_SIZE(gyro->resume_state));

	gyro->resume_state[RES_CTRL_REG1] = 0x0F; // [ECID:000000] ZTEBSP wanghaifei 20120423
	gyro->resume_state[RES_CTRL_REG2] = 0x00;
	gyro->resume_state[RES_CTRL_REG3] = 0x00;
	gyro->resume_state[RES_CTRL_REG4] = 0x00;
	gyro->resume_state[RES_CTRL_REG5] = 0x00;

//[ECID:000000] ZTEBSP wanghaifei 20130906 start, for device tree driver
	gyro->vdd_enabled = false;
	gyro->vio_enabled = false;
        gyro->reg_addr = 0x0F;
	data = gyro->reg_addr;
	l3g4200d_i2c_read(gyro, &data, 1);
	printk("TI-ST: gyro address is %x\n",data);
	if (data != WHOAMI_L3G4200D) {
		dev_err(&client->dev,
			"device id is 0x%x don't match 0x%x\n", data, WHOAMI_L3G4200D);
		goto err2;
	}
	chipid = data;

       err = l3g4200d_power_init(gyro, true);
        if (err < 0) {
                dev_err(&gyro->client->dev, "power init failed! err=%d", err);
                goto err2;
        }
//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver

	err = l3g4200d_device_power_on(gyro);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2_1;
	}

	atomic_set(&gyro->enabled, 1);

	err = l3g4200d_update_fs_range(gyro, gyro->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto err2_1;
	}

	err = l3g4200d_update_odr(gyro, gyro->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2_1;
	}

	err = l3g4200d_input_init(gyro);
	if (err < 0)
		goto err3;

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
			"%s device register failed\n", L3G4200D_GYR_DEV_NAME);
		goto err4;
	}

	l3g4200d_device_power_off(gyro);

	/* As default, do not report information */
	atomic_set(&gyro->enabled, 0);

	mutex_unlock(&gyro->lock);
	printk("gyro init successful\n");

#ifdef DEBUG
	pr_info("%s probed: device created successfully\n",
							L3G4200D_GYR_DEV_NAME);
#endif

	return 0;

err4:
	l3g4200d_input_cleanup(gyro);
err3:
	l3g4200d_device_power_off(gyro);
err2_1:
	l3g4200d_power_init(gyro, false);
err2:
	if (gyro->pdata->exit)
		gyro->pdata->exit();
err1_1:
	mutex_unlock(&gyro->lock);
	kfree(gyro->pdata);
err1:
	kfree(gyro);
err0:
		pr_err("%s: Driver Initialization failed\n",
							L3G4200D_GYR_DEV_NAME);
		return err;
}

static int l3g4200d_remove(struct i2c_client *client)
{
	struct l3g4200d_data *gyro = i2c_get_clientdata(client);
#ifdef DEBUG
	pr_info(KERN_INFO "L3G4200D driver removing\n");
#endif
	l3g4200d_input_cleanup(gyro);
	l3g4200d_device_power_off(gyro);
	l3g4200d_power_init(gyro, false);
	remove_sysfs_interfaces(&client->dev);

	kfree(gyro->pdata);
	kfree(gyro);
	return 0;
}

static int l3g4200d_suspend(struct device *dev)
{
#if 0
#ifdef CONFIG_SUSPEND
	struct i2c_client *client = to_i2c_client(dev);
	struct l3g4200d_data *gyro = i2c_get_clientdata(client);
#ifdef DEBUG
	pr_info(KERN_INFO "l3g4200d_suspend\n");
#endif /* DEBUG */
	/* TO DO */
#endif /*CONFIG_SUSPEND*/
	return 0;
#endif
	return 0;
}

static int l3g4200d_resume(struct device *dev)
{
#if 0
#ifdef CONFIG_SUSPEND
	struct i2c_client *client = to_i2c_client(dev);
	struct l3g4200d_data *gyro = i2c_get_clientdata(client);
#ifdef DEBUG
	pr_info(KERN_INFO "l3g4200d_resume\n");
#endif /*DEBUG */
	/* TO DO */
#endif /*CONFIG_SUSPEND*/
	return 0;
#endif
return 0;
}


static const struct i2c_device_id l3g4200d_id[] = {
	{ L3G4200D_GYR_DEV_NAME , 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, l3g4200d_id);

//[ECID:000000] ZTEBSP wanghaifei 20130906 start, for device tree driver
static struct of_device_id l3g4200d_match_table[] = {
         { .compatible = "st,l3g4200d", },
         { }, 
};
//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver

//[ECID 000000] yangze add for ic information add 20121121 begin
static ssize_t gyro_info_read_proc(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	return sprintf(page, "0x%x\n", chipid);
}

static struct proc_dir_entry *gyro_info_proc_file;
static int __init create_gyro_info_proc_file(void)
{
        if(0xff == chipid) {
                printk("not create l3g4200d proc file due to unconnected\n");
                return 0;
        }   
  gyro_info_proc_file = create_proc_entry("driver/gyro", 0644, NULL);
  printk("goes to create_gyro_info_proc_file\n");
  if (gyro_info_proc_file) {
			gyro_info_proc_file->read_proc = gyro_info_read_proc;
   } else{
	printk(KERN_INFO "l3g4200d gyro proc file create failed!\n");
   }
	return 0;
}


//[ECID 000000] yangze add for ic information add 20121121 end

static struct dev_pm_ops l3g4200d_pm = {
	.suspend = l3g4200d_suspend,
	.resume = l3g4200d_resume,
};

static struct i2c_driver l3g4200d_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = L3G4200D_GYR_DEV_NAME,
			.pm = &l3g4200d_pm,
			.of_match_table = l3g4200d_match_table,//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver
	},
	.probe = l3g4200d_probe,
	.remove = __devexit_p(l3g4200d_remove),
	.id_table = l3g4200d_id,

};

//[ECID:000000] ZTEBSP wanghaifei 20130906 start, for device tree driver
late_initcall(create_gyro_info_proc_file);
module_i2c_driver(l3g4200d_driver);
//[ECID:000000] ZTEBSP wanghaifei 20130906 end, for device tree driver

MODULE_DESCRIPTION("l3g4200d digital gyroscope sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");
