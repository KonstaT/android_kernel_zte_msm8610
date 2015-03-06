/* drivers/input/misc/kxtik.c - KXTIK accelerometer driver
 *
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kxtik.h>
#include <linux/input-polldev.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/proc_fs.h> // added by yangze for create proc file 20121111

#define NAME			"kxtik"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTIK_IEL		(1 << 3)
#define KXTIK_IEA		(1 << 4)
#define KXTIK_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxtik_odr_table[] = {
	{ 3,	ODR800F },
	{ 5,	ODR400F },
	{ 10,	ODR200F },
	{ 20,	ODR100F },
	{ 40,	ODR50F  },
	{ 80,	ODR25F  },
	{ 0,	ODR12_5F},
};

struct kxtik_data {
	struct i2c_client *client;
	struct kxtik_platform_data pdata;
	struct input_dev *input_dev;
	struct workqueue_struct *workqueue;
	struct work_struct irq_work;
	struct input_polled_dev *poll_dev;
	struct mutex mutex; 
	int enable; 
#ifdef CONFIG_OF
	bool    vdd_enabled;
	bool    vio_enabled;
	struct regulator *vdd;
        struct regulator *vio;
#endif
	unsigned int poll_interval;
	unsigned int poll_delay;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
};

static int chipid = 0xff;
#ifdef CONFIG_OF
#define KXTF9_VDD_MIN_UV        2700000
#define KXTF9_VDD_MAX_UV        3300000
#define KXTF9_VIO_MIN_UV        1700000
#define KXTF9_VIO_MAX_UV        2000000
#endif

static int kxtik_i2c_read(struct kxtik_data *tik, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tik->client->addr,
			.flags = tik->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tik->client->addr,
			.flags = tik->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tik->client->adapter, msgs, 2);
}

static void kxtik_report_acceleration_data(struct kxtik_data *tik)
{
	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;

	mutex_lock(&tik->mutex);
	err = kxtik_i2c_read(tik, XOUT_L, (u8 *)acc_data, 6);
	mutex_unlock(&tik->mutex);
	if (err < 0)
		dev_err(&tik->client->dev, "accelerometer data read failed\n");

	x = ((s16) le16_to_cpu(acc_data[tik->pdata.axis_map_x])) >> tik->shift;
	y = ((s16) le16_to_cpu(acc_data[tik->pdata.axis_map_y])) >> tik->shift;
	z = ((s16) le16_to_cpu(acc_data[tik->pdata.axis_map_z])) >> tik->shift;

	input_report_abs(tik->input_dev, ABS_X, tik->pdata.negate_x ? -x : x);
	input_report_abs(tik->input_dev, ABS_Y, tik->pdata.negate_y ? -y : y);
	input_report_abs(tik->input_dev, ABS_Z, tik->pdata.negate_z ? -z : z);
	input_sync(tik->input_dev);
}

static irqreturn_t kxtik_isr(int irq, void *dev)
{
	struct kxtik_data *tik = dev;
	queue_work(tik->workqueue, &tik->irq_work);
	return IRQ_HANDLED;
}

static void kxtik_irq_work(struct work_struct *work)
{
	struct kxtik_data *tik = container_of(work,	struct kxtik_data, irq_work);
	int err;

	/* data ready is the only possible interrupt type */
	kxtik_report_acceleration_data(tik);

	err = i2c_smbus_read_byte_data(tik->client, INT_REL);
	if (err < 0)
		dev_err(&tik->client->dev,
			"error clearing interrupt status: %d\n", err);
}

static int kxtik_update_g_range(struct kxtik_data *tik, u8 new_g_range)
{
	switch (new_g_range) {
	case KXTIK_G_2G:
		tik->shift = 4;
		break;
	case KXTIK_G_4G:
		tik->shift = 3;
		break;
	case KXTIK_G_8G:
		tik->shift = 2;
		break;
	default:
		return -EINVAL;
	}

	tik->ctrl_reg1 &= 0xe7;
	tik->ctrl_reg1 |= new_g_range;

	return 0;
}

static int kxtik_update_odr(struct kxtik_data *tik, unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxtik_odr_table); i++) {
		tik->data_ctrl = kxtik_odr_table[i].mask;
		if (poll_interval < kxtik_odr_table[i].cutoff)
			break;
	}

	err = i2c_smbus_write_byte_data(tik->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tik->client, DATA_CTRL, tik->data_ctrl);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tik->client, CTRL_REG1, tik->ctrl_reg1);
	if (err < 0)
		return err;

	return 0;
}

#ifdef CONFIG_OF
static int kxtf9_power_init(struct kxtik_data *data, bool on)
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
                        regulator_set_voltage(data->vdd, 0, KXTF9_VDD_MAX_UV);

                regulator_put(data->vdd);

                if (regulator_count_voltages(data->vio) > 0)
                        regulator_set_voltage(data->vio, 0, KXTF9_VIO_MAX_UV);

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
                if ((vol_uv >= KXTF9_VDD_MIN_UV) && (vol_uv <= KXTF9_VDD_MAX_UV)) {
                        dev_info(&data->client->dev,"no need to set vdd voltage again");
                } else if (regulator_count_voltages(data->vdd) > 0) {
                        rc = regulator_set_voltage(data->vdd, KXTF9_VDD_MIN_UV,
                                        KXTF9_VDD_MAX_UV);
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
                if ((vol_uv >= KXTF9_VIO_MIN_UV) && (vol_uv <= KXTF9_VIO_MAX_UV)) {
                        dev_info(&data->client->dev,"no need to set vio voltage again");
                } else if (regulator_count_voltages(data->vio) > 0) {
                        rc = regulator_set_voltage(data->vio, KXTF9_VIO_MIN_UV,
                                        KXTF9_VIO_MAX_UV);
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
static int kxtf9_power_init(struct kxtik_data *data, bool on)
{
        return 0;
}

#endif

static int kxtik_device_power_on(struct kxtik_data *tik)
{
	if (tik->pdata.power_on)
		return tik->pdata.power_on();

	return 0;
}

static void kxtik_device_power_off(struct kxtik_data *tik)
{
	int err;

	tik->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tik->client, CTRL_REG1, tik->ctrl_reg1);
	if (err < 0)
		dev_err(&tik->client->dev, "soft power off failed\n");

	if (tik->pdata.power_off)
		tik->pdata.power_off();
}

static int kxtik_enable(struct kxtik_data *tik)
{
	int err;

	err = kxtik_device_power_on(tik);
	if (err < 0)
		return err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tik->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (tik->client->irq) {
		err = i2c_smbus_write_byte_data(tik->client,
						INT_CTRL1, tik->int_ctrl);
		if (err < 0)
			return err;
	}

	err = kxtik_update_g_range(tik, tik->pdata.g_range);
	if (err < 0)
		return err;

	/* turn on outputs */
	tik->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tik->client, CTRL_REG1, tik->ctrl_reg1);
	if (err < 0)
		return err;

	err = kxtik_update_odr(tik, tik->poll_interval);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
	if (tik->client->irq) {
		err = i2c_smbus_read_byte_data(tik->client, INT_REL);
		if (err < 0) {
			dev_err(&tik->client->dev,
				"error clearing interrupt: %d\n", err);
			goto fail;
		}
	}

	return 0;

fail:
	kxtik_device_power_off(tik);
	return err;
}

static void kxtik_disable(struct kxtik_data *tik)
{
	kxtik_device_power_off(tik);
}

static int kxtik_input_open(struct input_dev *input)
{
	struct kxtik_data *tik = input_get_drvdata(input);

	return kxtik_enable(tik);
}

static void kxtik_input_close(struct input_dev *dev)
{
	struct kxtik_data *tik = input_get_drvdata(dev);

	kxtik_disable(tik);
}

static void __devinit kxtik_init_input_device(struct kxtik_data *tik,
					      struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	input_dev->name = "zte_acc";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &tik->client->dev;
}

static int __devinit kxtik_setup_input_device(struct kxtik_data *tik)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tik->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	tik->input_dev = input_dev;

	input_dev->open = kxtik_input_open;
	input_dev->close = kxtik_input_close;
	input_set_drvdata(input_dev, tik);

	kxtik_init_input_device(tik, input_dev);

	err = input_register_device(tik->input_dev);
	if (err) {
		dev_err(&tik->client->dev,
			"unable to register input polled device %s: %d\n",
			tik->input_dev->name, err);
		input_free_device(tik->input_dev);
		return err;
	}

	return 0;
}

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */

/* Returns currently selected poll interval (in ms) */
static ssize_t kxtik_get_poll(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtik_data *tik = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", tik->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtik_set_poll(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtik_data *tik = i2c_get_clientdata(client);
	struct input_dev *input_dev = tik->input_dev;
	unsigned int interval;

	#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
	int error;
	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;
	#else
	interval = (unsigned int)simple_strtoul(buf, NULL, 10);
	#endif

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	disable_irq(client->irq);

	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	tik->poll_interval = max(interval, tik->pdata.min_interval);
	tik->poll_delay = msecs_to_jiffies(tik->poll_interval);

	kxtik_update_odr(tik, tik->poll_interval);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

/* Allow users to enable/disable the device */
static ssize_t kxtik_set_enable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtik_data *tik = i2c_get_clientdata(client);
	struct input_dev *input_dev = tik->input_dev;
	unsigned int enable;

	#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
	int error;
	error = kstrtouint(buf, 10, &enable);
	if (error < 0)
		return error;
	#else
	enable = (unsigned int)simple_strtoul(buf, NULL, 10);
	#endif

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(enable)
		kxtik_enable(tik);
	else
		kxtik_disable(tik);

	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, kxtik_get_poll, kxtik_set_poll);
static DEVICE_ATTR(poll_enable, S_IWUSR, NULL, kxtik_set_enable);

static struct attribute *kxtik_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_poll_enable.attr,
	NULL
};

static struct attribute_group kxtik_attribute_group = {
	.attrs = kxtik_attributes
};


static void kxtik_poll(struct input_polled_dev *dev)
{
	struct kxtik_data *tik = dev->private;
	unsigned int poll_interval = dev->poll_interval;

	if (!tik->enable) {
		return;
	}
	kxtik_report_acceleration_data(tik);

	if (poll_interval != tik->poll_interval) {
		kxtik_update_odr(tik, poll_interval);
		tik->poll_interval = poll_interval;
	}
}

#if 0
static void kxtik_polled_input_open(struct input_polled_dev *dev)
{
	struct kxtik_data *tik = dev->private;

	kxtik_enable(tik);
}

static void kxtik_polled_input_close(struct input_polled_dev *dev)
{
	struct kxtik_data *tik = dev->private;

	kxtik_disable(tik);
}
#endif


static ssize_t attr_polling_rate_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct kxtik_data  *tik = dev_get_drvdata(dev);
	mutex_lock(&tik->mutex);
	val = tik->pdata.poll_interval;
	mutex_unlock(&tik->mutex);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct kxtik_data  *tik = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max((unsigned int)interval_ms, tik->pdata.min_interval);
	mutex_lock(&tik->mutex);
	if (tik->enable)
		tik->poll_dev->poll_interval = interval_ms;
	tik->pdata.poll_interval = interval_ms;
	kxtik_update_odr(tik, interval_ms);
	mutex_unlock(&tik->mutex);
	return size;
}

static ssize_t attr_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct kxtik_data *tik = dev_get_drvdata(dev);
	int val;
	mutex_lock(&tik->mutex);
	val = tik->enable;
	mutex_unlock(&tik->mutex);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct kxtik_data *tik = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	printk("kxtik set poll enable %ld\n", val);
	mutex_lock(&tik->mutex);
	if (val){
		kxtik_enable(tik);
		tik->enable = 1;
	}else{
		kxtik_disable(tik);
		tik->enable = 0;
	}
	mutex_unlock(&tik->mutex);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0644, attr_polling_rate_show,
						attr_polling_rate_store),
	//__ATTR(range, 0644, attr_range_show, attr_range_store),
	__ATTR(enable_device, 0644, attr_enable_show, attr_enable_store),
	//__ATTR(enable_polling, 0644, attr_polling_mode_show,attr_polling_mode_store),
	//__ATTR(fifo_samples, 0644, attr_watermark_show, attr_watermark_store),
	//__ATTR(fifo_mode, 0644, attr_fifomode_show, attr_fifomode_store),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for (; i >= 0; i--)
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

static int __devinit kxtik_setup_polled_device(struct kxtik_data *tik)
{
	int err;
	struct input_polled_dev *poll_dev;
	poll_dev = input_allocate_polled_device();

	if (!poll_dev) {
		dev_err(&tik->client->dev,
			"Failed to allocate polled device\n");
		return -ENOMEM;
	}

	tik->poll_dev = poll_dev;
	tik->input_dev = poll_dev->input;

	poll_dev->private = tik;
	poll_dev->poll = kxtik_poll;
//	poll_dev->open = kxtik_polled_input_open;
//	poll_dev->close = kxtik_polled_input_close;
        poll_dev->poll_interval = tik->pdata.poll_interval;


	kxtik_init_input_device(tik, poll_dev->input);

	err = input_register_polled_device(poll_dev);
	if (err) {
		dev_err(&tik->client->dev,
			"Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return err;
	}



	return 0;
}

static void __devexit kxtik_teardown_polled_device(struct kxtik_data *tik)
{
	input_unregister_polled_device(tik->poll_dev);
	input_free_polled_device(tik->poll_dev);
}


static int __devinit kxtik_verify(struct kxtik_data *tik)
{
	int retval;

	retval = kxtik_device_power_on(tik);
	if (retval < 0)
		return retval;

	retval = i2c_smbus_write_byte_data(tik->client, 0x1d, 0x80);
	if (retval < 0){
		printk(KERN_ERR "kxtik write reg 0x1d error\n");
		goto out;
	}
	
	mdelay(10);
	
	retval = i2c_smbus_read_byte_data(tik->client, WHO_AM_I);
	if (retval < 0) {
		dev_err(&tik->client->dev, "read err int source\n");
		goto out;
	}
	printk("kxtik reg WHO_AM_I is 0x%x after write reg 0x1d\n", retval);	
	
	chipid = retval;
	
	retval = ((retval != 0x05)&&(retval != 0x11)) ? -EIO : 0;

out:
	kxtik_device_power_off(tik);
	return retval;
}

#ifdef CONFIG_OF
static int kxtf9_parse_dt(struct device *dev,
                                struct kxtik_platform_data *kxtik_pdata)
{
        struct device_node *np = dev->of_node;
        u32 temp_val;
        int rc;

        rc = of_property_read_u32(np, "kionix,min-interval", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read min-interval\n");
                return rc;
        } else {
                kxtik_pdata->min_interval = temp_val;
        }

        rc = of_property_read_u32(np, "kionix,poll-interval", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read init-interval\n");
                return rc;
        } else {
                kxtik_pdata->poll_interval = temp_val;
        }

        rc = of_property_read_u32(np, "kionix,axis-map-x", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read axis-map_x\n");
                return rc;
        } else {
                kxtik_pdata->axis_map_x = (u8)temp_val;
        }

        rc = of_property_read_u32(np, "kionix,axis-map-y", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read axis_map_y\n");
                return rc;
        } else {
                kxtik_pdata->axis_map_y = (u8)temp_val;
        }

        rc = of_property_read_u32(np, "kionix,axis-map-z", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read axis-map-z\n");
                return rc;
        } else {
                kxtik_pdata->axis_map_z = (u8)temp_val;
        }

        rc = of_property_read_u32(np, "kionix,g-range", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read g-range\n");
                return rc;
        } else {
                switch (temp_val) {
                case 2:
                        kxtik_pdata->g_range = KXTIK_G_2G;
                        break;
                case 4:
                        kxtik_pdata->g_range = KXTIK_G_4G;
                        break;
                case 8:
                        kxtik_pdata->g_range = KXTIK_G_8G;
                        break;
                default:
                        kxtik_pdata->g_range = KXTIK_G_2G;
                        break;
                }
        }

        kxtik_pdata->negate_x = of_property_read_bool(np, "kionix,negate-x");

        kxtik_pdata->negate_y = of_property_read_bool(np, "kionix,negate-y");

        kxtik_pdata->negate_z = of_property_read_bool(np, "kionix,negate-z");

        if (of_property_read_bool(np, "kionix,res-12bit"))
                kxtik_pdata->res_12bit = RES_12BIT;
        else
                kxtik_pdata->res_12bit = RES_8BIT;

        return 0;
}

static int kxtf9_parse_gpio(struct device *dev)
{
        struct device_node *np = dev->of_node;
        u32 temp_val;
	int rc;

        temp_val = of_get_named_gpio(np, "kionix,gpio-irq1", 0);
        if (gpio_is_valid(temp_val)) {
                rc = gpio_request(temp_val, "kionix_irq1");
                if (rc) {
                        dev_err(dev, "irq1 gpio request failed");
                } else
                        gpio_direction_input(temp_val);
        }  

        temp_val = of_get_named_gpio(np, "kionix,gpio-irq2", 0);
        if (gpio_is_valid(temp_val)) {
                rc = gpio_request(temp_val, "kionix_irq2");
                if (rc) {
                        dev_err(dev, "irq2 gpio request failed");
                } else
                        gpio_direction_input(temp_val);
        }
	return 0;
}
#else
static int kxtf9_parse_dt(struct device *dev,
                                struct kxtik_platform_data *kxtik_pdata)
{       
        return -ENODEV;
}       
static int kxtf9_parse_gpio(struct device *dev)
{       
        return 0;
}       
#endif /* !CONFIG_OF */


static int __devinit kxtik_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct kxtik_platform_data *pdata = NULL;
	struct kxtik_data *tik;
	int err;

	printk("kxtik probe begin\n");
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	if (client->dev.of_node) {
		printk("kxtik use device tree\n");
		pdata = devm_kzalloc(&client->dev, sizeof(struct kxtik_platform_data), GFP_KERNEL);
		if (!pdata) {
                        dev_err(&client->dev, "Failed to allocate memory\n");
                        return -ENOMEM;
                } 
                err = kxtf9_parse_dt(&client->dev, pdata);
                if (err) {
                        dev_err(&client->dev,
                                "Unable to parse platfrom data err=%d\n", err);
                        return err;
                }   
        } else {
		printk("kxtik not use device tree\n");
		pdata = client->dev.platform_data;
        }  

	if (!pdata) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		return -EINVAL;
	}

	tik = kzalloc(sizeof(*tik), GFP_KERNEL);
	if (!tik) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	tik->client = client;
	tik->pdata = *pdata;

	if (pdata->init) {
		err = pdata->init();
		if (err < 0)
			goto err_free_mem;
	}


	tik->vdd_enabled = false;
	tik->vio_enabled = false;
        err = kxtf9_power_init(tik, true);
        if (err < 0) {
                dev_err(&tik->client->dev, "power init failed! err=%d", err);
                goto err_pdata_exit;
        }
	printk("kxtf9 irq %d\n", client->irq);

	err = kxtik_verify(tik);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_power_off;
	} else {
                kxtf9_parse_gpio(&client->dev);
	}

	i2c_set_clientdata(client, tik);

	tik->ctrl_reg1 = tik->pdata.res_12bit | tik->pdata.g_range;
	tik->poll_interval = tik->pdata.poll_interval;
	tik->poll_delay = msecs_to_jiffies(tik->poll_interval);

	if (client->irq) {
		err = kxtik_setup_input_device(tik);
		if (err)
			goto err_power_off;

		tik->workqueue = create_workqueue("KXTIK Workqueue");
		INIT_WORK(&tik->irq_work, kxtik_irq_work);
		/* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
		tik->int_ctrl |= KXTIK_IEN | KXTIK_IEA;
		tik->ctrl_reg1 |= DRDYE;

		err = request_threaded_irq(client->irq, NULL, kxtik_isr,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "kxtik-irq", tik);
		if (err) {
			dev_err(&client->dev, "request irq failed: %d\n", err);
			goto err_destroy_input;
		}

		err = sysfs_create_group(&client->dev.kobj, &kxtik_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
			goto err_free_irq;
		}

	} else {
		err = kxtik_setup_polled_device(tik);
		if (err)
			goto err_pdata_exit;

		 // added by yangze 20120827
		err = create_sysfs_interfaces(&client->dev);
		if (err < 0) {
			dev_err(&client->dev,
				"%s device register failed\n", NAME);
			goto err_pdata_exit;
		}
		//added by ytangze 20120827
		
		mutex_init(&tik->mutex); 
		tik->enable = 0; 
	}

	return 0;

err_free_irq:
	free_irq(client->irq, tik);
err_destroy_input:
	input_unregister_device(tik->input_dev);
	destroy_workqueue(tik->workqueue);
err_power_off:
	kxtf9_power_init(tik, false);	
err_pdata_exit:
	if (tik->pdata.exit)
		tik->pdata.exit();
err_free_mem:
	kfree(tik);
	return err;
}

static int __devexit kxtik_remove(struct i2c_client *client)
{
	struct kxtik_data *tik = i2c_get_clientdata(client);

	if (client->irq) {
		sysfs_remove_group(&client->dev.kobj, &kxtik_attribute_group);
		free_irq(client->irq, tik);
		input_unregister_device(tik->input_dev);
		destroy_workqueue(tik->workqueue);
	} else {
		kxtik_teardown_polled_device(tik);
	}
	kxtf9_power_init(tik, false);
	remove_sysfs_interfaces(&client->dev);

	if (tik->pdata.exit)
		tik->pdata.exit();

	kfree(tik);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int suspend_power = 0;
static int kxtik_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtik_data *tik = i2c_get_clientdata(client);

        int reg;

        reg = i2c_smbus_read_byte_data(tik->client, CTRL_REG1);
        if (reg < 0)
                dev_err(&tik->client->dev, "kxtik_suspend read power failed\n");
        if (reg & PC1_ON) {
                suspend_power = 1;
                kxtik_disable(tik);
        }  
	printk("kxtik: suspend with reg = 0x%x, standby in suspend %d\n", reg, suspend_power);
	return 0;
}

static int kxtik_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtik_data *tik = i2c_get_clientdata(client);
	int retval = 0;

        if (suspend_power == 1) {
                suspend_power = 0;
                kxtik_enable(tik);
        }

	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(kxtik_pm_ops, kxtik_suspend, kxtik_resume);

static const struct i2c_device_id kxtik_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kxtik_id);

static struct of_device_id kxtf9_match_table[] = {
         { .compatible = "kionix,kxtf9", },
         { },
};


//[ECID 000000] yangze add for ic information add 20121121 begin
static ssize_t acc_info_read_proc(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	return sprintf(page, "0x%x\n", chipid);
}

static struct proc_dir_entry *acc_info_proc_file;
static int __init create_acc_info_proc_file(void)
{
	if(0xff == chipid) {
		printk("not create kxtf9 proc file due to unconnected\n");
		return 0;
	}
	acc_info_proc_file = create_proc_entry("driver/accel", 0644, NULL);
	printk("goes to create_acc_info_proc_file\n");
	if (acc_info_proc_file) {
		acc_info_proc_file->read_proc = acc_info_read_proc;
	} else{
		printk(KERN_INFO "proc file create failed!\n");
	}
	return 0;
}
//[ECID 000000] yangze add for ic information add 20121121 end

static struct i2c_driver kxtik_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
		.of_match_table = kxtf9_match_table, 
		.pm	= &kxtik_pm_ops,
	},
	.probe		= kxtik_probe,
	.remove		= __devexit_p(kxtik_remove),
	.id_table	= kxtik_id,
};

late_initcall(create_acc_info_proc_file);
module_i2c_driver(kxtik_driver);

MODULE_DESCRIPTION("KXTIK accelerometer driver");
MODULE_AUTHOR("Kuching Tan <kuchingtan@kionix.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.6.0");
