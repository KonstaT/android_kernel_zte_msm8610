/* Lite-On LTR-558ALS Android Driver
 *
 * Copyright (C) 2011 Lite-On Technology Corp (Singapore)
 * Copyright (C) 2012 ZTE modify
 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
//#include <linux/i2c/ltr558als.h>
#include "ltr558als.h"

#define DRIVER_VERSION "1.0"
#define PARTID 0x80
#define MANUID 0x05

#define I2C_RETRY 5

#define DEVICE_NAME "ltr558als"

struct ltr558_data {
	/* Device */
	struct i2c_client *i2c_client;
	struct input_dev *als_ps_input_dev;
	struct mutex mutex;
	struct wake_lock ltr558_wake_lock;
	/* Device mode
	 * 0 = ALS
	 * 1 = PS
	 */
	int mode;

	/* ALS */
	int als_enable_flag;
	uint16_t als_lowthresh;
	uint16_t als_highthresh;
	uint16_t default_als_lowthresh;
	uint16_t default_als_highthresh;
	uint16_t als_ch0_value;

	/* PS */
	int ps_enable_flag;
	int ps_on_flag;
	uint16_t ps_lowthresh;
	uint16_t ps_highthresh;
	uint16_t default_ps_lowthresh;
	uint16_t default_ps_highthresh;

	/* LED */
	int led_pulse_freq;
	int led_duty_cyc;
	int led_peak_curr;
	int led_pulse_count;

	/* Interrupt */
	int irq;
	int is_suspended;

	int open_num;
};

struct ltr558_data *sensor_info = NULL;

/* I2C Read */
static int I2C_Read(char *rxData, int length)
{
	int index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 2) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("LTR558: I2C Read Fail !!!!\n");
		return -EIO;
	}
	return 0;
}

/* I2C Write */
static int I2C_Write(char *txData, int length)
{
	int index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 1) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("LTR558: I2C Write Fail !!!!\n");
		return -EIO;
	}
	return 0;
}

/* Set register bit */
static int _ltr558_set_bit(struct i2c_client *client, u8 set, u8 cmd, u8 data)
{
	u8 value;
	int ret = 0;

	value = i2c_smbus_read_byte_data(client, cmd);
	if (value < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, value);
		return ret;
	}


	if (set == SET_BIT)
		value |= data;
	else
		value &= ~data;

	ret = i2c_smbus_write_byte_data(client, cmd, value);
	if (ret < 0) {
		dev_err(&client->dev, "%s |ltr558: 0x%02X", __func__, value);
		return -EIO;
	}

	return ret;
}

static uint16_t lux_formula(uint16_t ch0_adc, uint16_t ch1_adc)
{
	uint16_t luxval = 0;
	int ch0_coeff = 0;
	int ch1_coeff = 0;
	int ratio, ret, gain;
	struct ltr558_data *ltr558 = sensor_info;


	if ((ch1_adc * ch0_adc) == 0) {
		ratio = 100;
	} else {
		ratio = (100 * ch1_adc)/(ch1_adc + ch0_adc);
	}

	if (ratio < 45)
	{
		ch0_coeff = 17743;
		ch1_coeff = -11059;
	}
	else if ((ratio >= 45) && (ratio < 64))
	{
		ch0_coeff = 37725;
		ch1_coeff = 13363;
	}
	else if ((ratio >= 64) && (ratio < 85))
	{
		ch0_coeff = 16900;
		ch1_coeff = 1690;
	}
	else if (ratio >= 85)
	{
		ch0_coeff = 0;
		ch1_coeff = 0;
	}

	luxval = ((ch0_adc * ch0_coeff) - (ch1_adc * ch1_coeff))/10000;

	ret = i2c_smbus_read_byte_data(ltr558->i2c_client, LTR558_ALS_PS_STATUS);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, ret);
		return ret;
	}
	gain =ret & 0x10;

	if ((gain == 16) && (luxval > 0)) {
		luxval = luxval / 200;
	}


	return luxval;
}

/* Read ADC Value */
static uint16_t read_adc_value(struct ltr558_data *ltr558)
{
	int ret = -99;
	uint16_t value = -99;
	uint16_t ps_val;
	uint16_t ch0_val;
	uint16_t ch1_val;
	
	char buffer[4];

	switch (ltr558->mode) {
		case ALS :
			/* ALS */
			buffer[0] = LTR558_ALS_DATA_CH1_0;

			/* read data bytes from data regs */
			ret = I2C_Read(buffer, 4);
			break;

		case PS :
			/* PS */
			buffer[0] = LTR558_PS_DATA_0;

			/* read data bytes from data regs */
			ret = I2C_Read(buffer, 2);
			break;
	}

	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s |ltr558 read data error 0x%02X", __func__, buffer[0]);
		return ret;
	}


	switch (ltr558->mode) {
		case ALS :
			/* ALS Ch0 */
		 	ch1_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
				dev_dbg(&ltr558->i2c_client->dev, 
					"%s | als_ch1 value = 0x%04X\n", __func__, 
					ch1_val);
			
			if (ch1_val > ALS_MAX_MEASURE_VAL) {
				dev_err(&ltr558->i2c_client->dev,
				        "%s: ALS Value Error: 0x%X\n", __func__,
				        ch1_val);
			}
			ch1_val &= ALS_VALID_MEASURE_MASK;

			/* ALS Ch1 */
		 	ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
				dev_dbg(&ltr558->i2c_client->dev, 
					"%s | als_ch0 value = 0x%04X\n", __func__, 
					ch0_val);
			
			if (ch0_val > ALS_MAX_MEASURE_VAL) {
				dev_err(&ltr558->i2c_client->dev,
				        "%s: ALS Value Error: 0x%X\n", __func__,
				        ch0_val);
			}
			ch0_val &= ALS_VALID_MEASURE_MASK;
			ltr558->als_ch0_value = ch0_val;

			/* ALS Lux Conversion */
			value = lux_formula(ch0_val, ch1_val);
			break;

		case PS :
			/* PS */
			ps_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
			/*
				dev_dbg(&ltr558->i2c_client->dev, 
					"%s | ps value = 0x%04X\n", __func__, 
					ps_val);
			*/
			if (ps_val > PS_MAX_MEASURE_VAL) {
				dev_err(&ltr558->i2c_client->dev,
				        "%s: PS Value Error: 0x%X\n", __func__,
				        ps_val);
			}
			ps_val &= PS_VALID_MEASURE_MASK;
			value = ps_val;
			break;

	}
	return value;
}

/* Set ALS range */
static int set_als_range(uint16_t lt, uint16_t ht)
{
	int ret;
	char buffer[5];

	buffer[0] = LTR558_ALS_THRES_UP_0;
	buffer[1] = ht & 0xFF;
	buffer[2] = (ht >> 8) & 0xFF;
	buffer[3] = lt & 0xFF;
	buffer[4] = (lt >> 8) & 0xFF;

	ret = I2C_Write(buffer, 5);
	if (ret <0) {
		pr_alert("LTR558: set als range failed\n");
		return ret;
	}
	dev_info(&sensor_info->i2c_client->dev, "LTR558 Set als range:0x%04x"
	                                       " - 0x%04x\n",  lt, ht);
	return ret;
}

/* Set PS range */
static int set_ps_range(uint16_t lt, uint16_t ht)
{
	int ret;
	char buffer[5];

	buffer[0] = LTR558_PS_THRES_UP_0;
	buffer[1] = ht & 0xFF;
	buffer[2] = (ht >> 8) & 0x07;
	buffer[3] = lt & 0xFF;
	buffer[4] = (lt >> 8) & 0x07;

	ret = I2C_Write(buffer, 5);
	if (ret <0) {
		pr_alert("LTR558: set ps range failed\n");
		return ret;
	}
/*
	dev_dbg(&sensor_info->i2c_client->dev, "LTR558: Set ps range:0x%04x"
	                                       " - 0x%04x\n",  lt, ht);
*/
	return ret;
}

/* Report PS input event */
static void report_ps_input_event(struct ltr558_data *ltr558)
{
	int rc;
	uint16_t adc_value;
	int thresh_hi, thresh_lo;

	ltr558->mode = PS;
	adc_value = read_adc_value(ltr558);

	if (adc_value > ltr558->ps_highthresh) {
		thresh_lo = ltr558->ps_lowthresh;
		thresh_hi = PS_MAX_MEASURE_VAL;
		input_report_abs(ltr558->als_ps_input_dev, ABS_DISTANCE, 1);
		input_sync(ltr558->als_ps_input_dev);

	} else if ((ltr558->ps_on_flag) || (adc_value < ltr558->ps_lowthresh)) {
		thresh_lo = PS_MIN_MEASURE_VAL;
		thresh_hi = ltr558->ps_highthresh;
		input_report_abs(ltr558->als_ps_input_dev, ABS_DISTANCE, 0);
		input_sync(ltr558->als_ps_input_dev);
	} else {
		return;
	}

	/* Adjust measurement range using a crude filter to prevent interrupt
	 *  jitter. */
	if (thresh_lo < PS_MIN_MEASURE_VAL)
		thresh_lo = PS_MIN_MEASURE_VAL;
	if (thresh_hi > PS_MAX_MEASURE_VAL)
		thresh_hi = PS_MAX_MEASURE_VAL;
	rc = set_ps_range((uint16_t)thresh_lo, (uint16_t)thresh_hi);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s : PS Thresholds Write Fail...\n", __func__);
	}

	ltr558->ps_on_flag = 0;
}

/* Report ALS input event and select range */
static void report_als_input_event(struct ltr558_data *ltr558)
{
	int rc;
	uint16_t adc_value;
	int thresh_hi, thresh_lo;

	ltr558->mode = ALS;
	adc_value = read_adc_value(ltr558);

	if (adc_value > 10000)
		adc_value = 10000;
	input_report_abs(ltr558->als_ps_input_dev, ABS_MISC, adc_value);
	input_sync(ltr558->als_ps_input_dev);

	/* Adjust measurement range using a crude filter to prevent interrupt
	 *  jitter. */
	thresh_lo = (8 * ltr558->als_ch0_value) / 10;
	thresh_hi = (12 * ltr558->als_ch0_value) / 10;
	if (thresh_lo < ALS_MIN_MEASURE_VAL)
		thresh_lo = ALS_MIN_MEASURE_VAL;
	if (thresh_hi > ALS_MAX_MEASURE_VAL)
		thresh_hi = ALS_MAX_MEASURE_VAL;
	rc = set_als_range((uint16_t)thresh_lo, (uint16_t)thresh_hi);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s : ALS Thresholds Write Fail...\n", __func__);
	}
}

/* Work when interrupt */
static void ltr558_schedwork(struct work_struct *work)
{
	uint8_t status;
	uint8_t	interrupt_stat, newdata;
	struct ltr558_data *ltr558 = sensor_info;
	
	wake_lock(&ltr558->ltr558_wake_lock);
	status = i2c_smbus_read_byte_data(ltr558->i2c_client, LTR558_ALS_PS_STATUS);
	if (status < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s |ltr558 read status err\n", __func__);
		return;
	}

	interrupt_stat = status & 0x0a;
	newdata = status & 0x05;

	if (!interrupt_stat) {
		/* There was an interrupt with no work to do */
		int i;
		u8 buf[40];
		dev_info(&ltr558->i2c_client->dev,"%s Unexpected received"
			 " interrupt with no work to do status:0x%02x\n",
			 __func__, status);
		buf[0] = 0x80;
		I2C_Read(buf, sizeof(buf));
		for (i = 0; i < sizeof(buf); i++) {
			dev_info(&ltr558->i2c_client->dev, "%s reg:0x%02x"
				 " val:0x%02x\n", __func__, 0x80+i, buf[i]);
		}
	} else {
		if ((interrupt_stat & 0x02) && (newdata & 0x01)) {
			report_ps_input_event(ltr558);
		}
		if ((interrupt_stat & 0x08) && (newdata & 0x04)) {
			report_als_input_event(ltr558);
		}
	}
	wake_unlock(&ltr558->ltr558_wake_lock);
	enable_irq(ltr558->irq);
}


static DECLARE_WORK(irq_workqueue, ltr558_schedwork);

/* IRQ Handler */
static irqreturn_t ltr558_irq_handler(int irq, void *data)
{
	disable_irq_nosync(irq);
	schedule_work(&irq_workqueue);

	return IRQ_HANDLED;
}


/* PS Enable */
static int ps_enable(struct ltr558_data *ltr558)
{
	int rc = 0;
	
	mutex_lock(&ltr558->mutex);
	if (ltr558->ps_enable_flag) {
		dev_info(&ltr558->i2c_client->dev, "LTR558: ps already enabled\n");
		return 0;
	}

	rc = set_ps_range(ltr558->ps_lowthresh, ltr558->ps_highthresh);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558:PS Thresholds Write Fail...\n");
		return rc;
	}

	/* Allows this interrupt to wake the system */
	/*
	rc = enable_irq_wake(ltr558->irq);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: IRQ-%d WakeUp Enable Fail...\n",  ltr558->irq);
		return rc;
	}
	*/

	rc = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_PS_LED, 0x6B); //60kHZ,50%,50mA
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: PS LED Setup Fail...\n");
		return rc;
	}

	rc = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_PS_N_PULSES, ltr558->led_pulse_count);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: PS LED PulseFail...\n");
		return rc;
	}
	
	rc = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_PS_MEAS_RATE,  PS_MEAS_RATE);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: MeasRate Setup Fail...\n");
		return rc;
	}

	rc = _ltr558_set_bit(ltr558->i2c_client, SET_BIT, LTR558_PS_CONTR, PS_MODE);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: PS Enable Fail...\n");
		return rc;
	}
	ltr558->ps_on_flag = 1;
	mdelay(WAKEUP_DELAY);
	
	ltr558->ps_enable_flag = 1;
	mutex_unlock(&ltr558->mutex);
	
	return rc;
}

/* PS Disable */
static int ps_disable(struct ltr558_data *ltr558)
{
	int rc = 0;

	mutex_lock(&ltr558->mutex);
	ltr558->ps_on_flag = 0;
	if (ltr558->ps_enable_flag == 0) {
		dev_info(&ltr558->i2c_client->dev, "LTR558: already disabled\n");
		return 0;
	}

	/* Don't allow this interrupt to wake the system anymore */
	/*
	rc = disable_irq_wake(ltr558->irq);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: IRQ-%d WakeUp Disable Fail...\n",  ltr558->irq);
		return rc;
	}
	*/

	rc = _ltr558_set_bit(ltr558->i2c_client, CLR_BIT, LTR558_PS_CONTR, PS_MODE);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: PS Disable Fail...\n");
		return rc;
	}

	ltr558->ps_enable_flag = 0;
	mutex_unlock(&ltr558->mutex);
	
	return rc;
}

static int als_enable(struct ltr558_data *ltr558)
{
	int rc = 0;
	
	mutex_lock(&ltr558->mutex);
	/* if device not enabled, enable it */
	if (ltr558->als_enable_flag != 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: ALS already enabled...\n");
		return rc;
	}

	rc = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_ALS_MEAS_RATE,  ALS_MEAS_RATE);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: ALS MeasRate Setup Fail...\n");
		return rc;
	}

	/* Set minimummax thresholds where interrupt will *not* be generated */
	rc = set_als_range(0x0, 0x0);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: ALS Thresholds Write Fail...\n");
		return rc;
	}

	rc = _ltr558_set_bit(ltr558->i2c_client, SET_BIT, LTR558_ALS_CONTR, ALS_MODE);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: ALS Enable Fail...\n");
		return rc;
	}
	mdelay(WAKEUP_DELAY);

	ltr558->als_enable_flag = 1;
	mutex_unlock(&ltr558->mutex);
	
	return rc;
}

static int als_disable(struct ltr558_data *ltr558)
{
	int rc = 0;

	mutex_lock(&ltr558->mutex);
	if (ltr558->als_enable_flag != 1) {
		dev_err(&ltr558->i2c_client->dev, "LTR558: ALS already disabled...\n");
		return rc;
	}

	rc = _ltr558_set_bit(ltr558->i2c_client, CLR_BIT, LTR558_ALS_CONTR, ALS_MODE);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev,"LTR558: ALS Disable Fail...\n");
		return rc;
	}
	ltr558->als_enable_flag = 0;
	mutex_unlock(&ltr558->mutex);
	
	return rc;
}

static int ps_calibrate(struct ltr558_data *ltr558)
{
	int rc;
	int i;
	uint16_t value;
	uint32_t ps_sum = 0;
	uint32_t ps_max = 0;
	uint8_t  int_reg = 0;

	/*disable irq*/
	int_reg = i2c_smbus_read_byte_data(ltr558->i2c_client, LTR558_INTERRUPT);
	if (int_reg < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s: read fail\n", __func__);
	}
	rc = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_INTERRUPT, 0x0);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558:ioctl dis_irq fail\n");
	}
	
	/*enable ps to read data*/
	ps_enable(ltr558);
	for (i = 0; i < 20; i++)
	{
		ltr558->mode = PS;
		value = read_adc_value(ltr558);
		ps_sum += value;
		if (value > ps_max)
			ps_max = value;
		printk("LTR558: CAL adc value 0x%x\n", value);
		mdelay(100);
	}
	ps_disable(ltr558);

	/*enable irq*/
	rc = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_INTERRUPT, int_reg);
	if (rc < 0) {
		dev_err(&ltr558->i2c_client->dev, "LTR558:ioctl dis_irq fail\n");
	}

	ps_sum = ps_sum/20;
	if ((ps_max - ps_sum) > 2) {
		ltr558->ps_highthresh = ((((ps_max - ps_sum) * 200) + 50)/100) + ps_sum;
		ltr558->ps_lowthresh = ((((ps_max - ps_sum) * 170) + 50)/100) + ps_sum;
	} else {
		ltr558->ps_highthresh = 5 + ps_sum;
		ltr558->ps_lowthresh = 3 + ps_sum;
	}
	printk("LTR558: ps_lowthresh = 0x%x,ps_highthresh = 0x%x\n", ltr558->ps_lowthresh, ltr558->ps_highthresh);

	if (ltr558->ps_highthresh > 1000) {
		printk("LTR558: use default threshold\n");
		ltr558->ps_lowthresh = ltr558->default_ps_lowthresh;
		ltr558->ps_highthresh = ltr558->default_ps_highthresh;
	}
	return 0;
}

static int als_ps_open(struct inode *inode, struct file *file)
{
	struct ltr558_data *ltr558 = sensor_info;
	int rc = 0;

	ltr558->open_num += 1;
	printk("LTR558:  open number %d\n", ltr558->open_num);
	if (ltr558->open_num == 1)
		enable_irq(ltr558->irq);
	return rc;
}

static int als_ps_release(struct inode *inode, struct file *file)
{
	struct ltr558_data *ltr558 = sensor_info;

	  ltr558->open_num -= 1;   
	  printk("LTR558:  close number %d\n", ltr558->open_num); 
	  if (ltr558->open_num <= 0) {
	  	ltr558->open_num = 0;	  
		disable_irq(ltr558->irq);  
	  }
	  
	return 0;
}

static long als_ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	struct ltr558_data *ltr558 = sensor_info;

	//pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case LIGHT_IOCTL_ALS_ON:
			rc = als_enable(ltr558);
			break;
		case LIGHT_IOCTL_ALS_OFF:
			rc = als_disable(ltr558);
			break;
		case LIGHT_IOCTL_PROX_ON:
			rc = ps_enable(ltr558);
			break;
		case LIGHT_IOCTL_PROX_OFF:
			rc = ps_disable(ltr558);
			break;
		case LIGHT_IOCTL_PROX_CALIBRATE:
			rc = ps_calibrate(ltr558);
			break;
		default:
			pr_err("%s: ltr558 INVALID COMMAND %d\n", __func__, _IOC_NR(cmd));
			return -EINVAL;
		}
	return rc;
}

static const struct file_operations als_ps_fops = {
	.owner = THIS_MODULE,
	.open = als_ps_open,
	.release = als_ps_release,
	.unlocked_ioctl = als_ps_ioctl
};

static struct miscdevice als_ps_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "light_sensor",
	.fops = &als_ps_fops
};

static ssize_t ps_pulse_set(struct device *dev, struct device_attribute *attr,
                                                 const char *buf, size_t count)
{
	uint16_t value;
	struct ltr558_data *ltr558 = sensor_info;

	value = (unsigned int)simple_strtoul(buf, NULL, 10);
	value &= 0xff;
	i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_PS_N_PULSES, value);

	return count;
}

static ssize_t ps_pulse_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	struct ltr558_data *ltr558 = sensor_info;

	value = i2c_smbus_read_byte_data(ltr558->i2c_client, LTR558_PS_N_PULSES);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}

static DEVICE_ATTR(pulse, S_IRUGO | S_IWUSR, ps_pulse_show, ps_pulse_set);

static ssize_t ps_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct ltr558_data *ltr558 = sensor_info;

	ltr558->mode = PS;
	value = read_adc_value(ltr558);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}

static DEVICE_ATTR(ps_adc, S_IRUGO, ps_adc_show, NULL);


#define HOLE_REG_SPACE(a,b) buffer[0] = a; \
	I2C_Read(buffer, (b+1)-a); \
	for (i = 0; i < (b+1)-a; i++) { \
		buf += sprintf(buf, "0x%02x: 0x%02x\n", i+a, buffer[i]); \
	} \

static ssize_t dump_regs_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
	char *tmp_buf = buf;
	uint8_t buffer[64];
	int i;

	/* There are holes in the address space */
	HOLE_REG_SPACE(0x80, 0x9c);
	HOLE_REG_SPACE(0x9e, 0xa1);
	HOLE_REG_SPACE(0xa4, 0xa4);

	return strlen(tmp_buf);

}

static DEVICE_ATTR(dump_regs, S_IRUGO, dump_regs_show, NULL);

static ssize_t thresh_show(struct device *dev,
                           struct device_attribute *attr,
                           char *buf)
{
	int ret = 0;
	char *tmp_buf = buf;
	uint16_t min, max;
	uint8_t buffer[64];

	buffer[0] = LTR558_ALS_THRES_UP_0;

	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		pr_alert("%s LTR558:| 0x%02X",  __func__, buffer[0]);
		return 0;
	}

	max = (u16)buffer[0] | ((u16)buffer[1] << 8);
	min = (u16)buffer[2] | ((u16)buffer[3] << 8);
	buf += sprintf(buf, "als min:%d max:%d\n", min, max);

	buffer[0] = LTR558_PS_THRES_UP_0;

	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		pr_alert("%s |LTR558: 0x%02X", __func__, buffer[0]);
		return 0;
	}

	max = (u16)buffer[0] | (((u16)buffer[1] & 0x7) << 8);
	min = (u16)buffer[2] | (((u16)buffer[3] & 0x7) << 8);
	buf += sprintf(buf, "ps min:%d max:%d\n", min, max);

	return strlen(tmp_buf);

}

static DEVICE_ATTR(thresh, S_IRUGO, thresh_show, NULL);

static ssize_t ps_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t id = 0xff;
	int ret;
	struct ltr558_data *ltr558 = sensor_info;

	id = i2c_smbus_read_byte_data(ltr558->i2c_client, LTR558_PART_ID);
	if (id > 0) {
		ret = sprintf(buf, "0x%x\n", id & 0xff);
	} else {
		ret = sprintf(buf, "0x%x\n", 0xff);
	}

	return ret;
}
static DEVICE_ATTR(ps_name, S_IRUGO, ps_id_show, NULL);

static void sysfs_register_device(struct i2c_client *client) {
	int rc = 0;

	rc += device_create_file(&client->dev, &dev_attr_pulse);
	rc += device_create_file(&client->dev, &dev_attr_ps_adc);
	rc += device_create_file(&client->dev, &dev_attr_dump_regs);
	rc += device_create_file(&client->dev, &dev_attr_thresh);
	rc += device_create_file(&client->dev, &dev_attr_ps_name);
	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs files\n", __func__);
	}
}
static void sysfs_unregister_device(struct i2c_client *client) {
	device_remove_file(&client->dev, &dev_attr_pulse);
	device_remove_file(&client->dev, &dev_attr_ps_adc);
	device_remove_file(&client->dev, &dev_attr_dump_regs);
	device_remove_file(&client->dev, &dev_attr_thresh);
	device_remove_file(&client->dev, &dev_attr_ps_name);
}

static int ltr558_input_dev_setup(struct ltr558_data *ltr558)
{
	int ret;

	ltr558->als_ps_input_dev = input_allocate_device();
	if (!ltr558->als_ps_input_dev) {
		dev_err(&ltr558->i2c_client->dev, "%s: Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr558->als_ps_input_dev->name = "light_sensor";
	ltr558->als_ps_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, ltr558->als_ps_input_dev->evbit);
	input_set_abs_params(ltr558->als_ps_input_dev, ABS_MISC, ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, 0, 0);
	input_set_abs_params(ltr558->als_ps_input_dev, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);
	
	ret = input_register_device(ltr558->als_ps_input_dev);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s:  Register Input Device Fail...\n", __func__);
		goto err_register_input_device;
	}

	ret = misc_register(&als_ps_misc);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s: Register Misc Device Fail...\n", __func__);
		goto err_register_misc_device;
	}

	return ret;

err_register_misc_device:
	input_unregister_device(ltr558->als_ps_input_dev);
err_register_input_device:
	input_free_device(ltr558->als_ps_input_dev);

	return ret;
}

static void ltr558_input_dev_destroy(struct ltr558_data *ltr558)
{
	misc_deregister(&als_ps_misc);
	input_unregister_device(ltr558->als_ps_input_dev);
	input_free_device(ltr558->als_ps_input_dev);
}

static int ltr558_check_part_id(struct i2c_client *client)
{
	int ret;
	ret = i2c_smbus_read_byte_data(client, LTR558_PART_ID);
	if (ret != PARTID) {
		dev_err(&client->dev, "%s: part id miscompare :should be 0x%02X, read 0x%02X\n", __func__, PARTID, ret);
		return -ENODATA;
	}
	dev_info(&client->dev, "ltr558: chip id 0x%0x\n", ret);
	return 0;
}

static int ltr558_setup(struct ltr558_data *ltr558)
{
	int ret = 0;

	/* Reset the devices */
	ret = _ltr558_set_bit(ltr558->i2c_client, SET_BIT, LTR558_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s: ALS reset fail...\n", __func__);
		goto err_out1;
	}
	msleep(PON_DELAY);
	dev_dbg(&ltr558->i2c_client->dev, "%s: Reset ltr558 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (ltr558_check_part_id(ltr558->i2c_client) < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}

	/* Set count of measurements outside data range before interrupt is generated */
	ret = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_INTERRUPT_PRST, ALS_INT_PRST | PS_INT_PRST);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s:  Set Persist Fail...\n", __func__);
		goto err_out1;
	}

	/* Enable interrupts on the device and clear only when status is read */
	ret = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_INTERRUPT,  0x08 | INTERRUPT_POL |INTERRUPT_MODE);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s: Enabled interrupts failed...\n", __func__);
		goto err_out1;
	}

	/* Set ALS measurement gain */
	ret = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_ALS_CONTR, 0);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s: ALS set gain fail...\n", __func__);
		goto err_out1;
	}

	/* Set PS measurement gain */
	ret = i2c_smbus_write_byte_data(ltr558->i2c_client, LTR558_PS_CONTR, 0);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s: PS set gain fail...\n", __func__);
		goto err_out1;
	}

	ret = request_irq(ltr558->irq, ltr558_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, NULL);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s: Request IRQ (%d)  Fail (%d)\n", __func__, ltr558->irq,  ret);
		goto err_out1;
	}
	disable_irq(ltr558->irq);
	
	return ret;


err_out1:
	dev_err(&ltr558->i2c_client->dev, "%s Unable to setup device\n", __func__);
	return ret;
}


static int ltr558_suspend(struct device *dev)
{
	int ret = 0;
	struct ltr558_data *ltr558 = sensor_info;
	if (!ltr558)
		return 0;

	ltr558->is_suspended  = 0;
	/* Disable the devices for suspend if configured */
	if  (ltr558->als_enable_flag) {
		ret += als_disable(ltr558);
		if (!ret)
		ltr558->is_suspended |= ALS_SUSPENDED;
	}
	if ( ltr558->ps_enable_flag) {
		ret += ps_disable(ltr558);
		if (!ret)
		ltr558->is_suspended |= PS_SUSPENDED;
	}

	if (ret) {
		dev_err(&ltr558->i2c_client->dev, "%s Unable to complete suspend\n", __func__);
	} else {
		dev_info(&ltr558->i2c_client->dev, "%s Suspend completed 0x%02x\n", __func__, ltr558->is_suspended);
	}
	return 0;
}

static int ltr558_resume(struct device *dev)
{
	struct ltr558_data *ltr558 = sensor_info;
	int ret = 0;

	if (!ltr558)
		return 0;
	/* If ALS was enbled before suspend, enable during resume */
	if (ltr558->is_suspended & ALS_SUSPENDED) {
		ret += als_enable(ltr558);
	}

	/* If PS was enbled before suspend, enable during resume */
	if (ltr558->is_suspended & PS_SUSPENDED) {
		ret += ps_enable(ltr558);
	}

	if (ret) {
		dev_err(&ltr558->i2c_client->dev, "%s Unable to complete resume\n", __func__);
	} else {
		dev_info(&ltr558->i2c_client->dev, "%s Resume completed\n", __func__);
	}
	return 0;
}


static int ltr558_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct ltr558_data *ltr558;
	struct ltr558_platform_data *platdata;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA))
	{
		dev_err(&client->dev, "%s: LTR-558ALS functionality check failed.\n", __func__);
		ret = -EIO;
		goto err_out;
	}

	if (ltr558_check_part_id(client) < 0) {
		dev_err(&client->dev, "%s: Part ID Read Fail...\n", __func__);
		goto err_out;
	}
	
	ltr558 = kzalloc(sizeof(struct ltr558_data), GFP_KERNEL);
	if (!ltr558)
	{
		dev_err(&client->dev, "%s: Mem Alloc Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}

	/* Global pointer for this device */
	sensor_info = ltr558;


	/* Set initial defaults */
	ltr558->als_enable_flag = 0;
	ltr558->ps_enable_flag = 0;

	ltr558->i2c_client = client;
	ltr558->irq = gpio_to_irq(client->irq);

	i2c_set_clientdata(client, ltr558);

	/* Parse the platform data */
	platdata = client->dev.platform_data;
	if (!platdata) {
		dev_err(&ltr558->i2c_client->dev, "%s: Platform Data assign Fail...\n", __func__);
		ret = -EBUSY;
		goto err_mem_out;
	}

	ltr558->default_ps_lowthresh = platdata->pfd_ps_lowthresh;
	ltr558->default_ps_highthresh = platdata->pfd_ps_highthresh;
	ltr558->led_pulse_count = platdata->pfd_led_pulse;
	mutex_init(&ltr558->mutex);
	wake_lock_init(&ltr558->ltr558_wake_lock, WAKE_LOCK_SUSPEND, "ltr558-wake-lock");

	/* Setup the input subsystem for the ALS */
	ret = ltr558_input_dev_setup(ltr558);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev,"%s: input device Setup Fail...\n", __func__);
		goto err_mem_out;
	}


	/* Setup and configure both the ALS and PS on the ltr558 device */
	ret = ltr558_setup(ltr558);
	if (ret < 0) {
		dev_err(&ltr558->i2c_client->dev, "%s: Setup Fail...\n", __func__);
		goto err_ltr558_setup;
	}


	/* Register the sysfs files */
	sysfs_register_device(client);

	dev_dbg(&ltr558->i2c_client->dev, "%s: probe complete\n", __func__);
	return ret;

err_ltr558_setup:
	ltr558_input_dev_destroy(ltr558);	
err_mem_out:
	kfree(ltr558);
err_out:
	return ret;
}

static int ltr558_remove(struct i2c_client *client)
{
	int ret = 0;
	struct ltr558_data *ltr558 = sensor_info;
	if (!ltr558)
		return 0;

	sysfs_unregister_device(ltr558->i2c_client);
	free_irq(ltr558->irq, NULL);
	ltr558_input_dev_destroy(ltr558);
	kfree(ltr558);
	return ret;
}


static const struct i2c_device_id ltr558_id[] = {
	{ DEVICE_NAME, 0 },
	{}
};


static SIMPLE_DEV_PM_OPS(ltr558_pm_ops, ltr558_suspend, ltr558_resume);

static struct i2c_driver ltr558_driver = {
	.probe = ltr558_probe,
	.remove = ltr558_remove,
	.id_table = ltr558_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
		.pm = &ltr558_pm_ops,
	},
};

static struct ltr558_platform_data ltr558_plat = {
	.pfd_ps_lowthresh = 0x18, //should be change
	.pfd_ps_highthresh = 0x20, // should be change
	.pfd_led_pulse = 0xa,		// should be change
};

static struct i2c_board_info ltr558_i2c_info[] = {
	{
		I2C_BOARD_INFO(DEVICE_NAME, LTR558_I2C_ADDR),
		.irq = LTR558_IRQ_GPIO,
		.platform_data = &ltr558_plat,
	}
};

static int __init ltr558_init(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	
	adapter = i2c_get_adapter(LTR558_I2C_BUS);
	if (!adapter)
			return -ENODEV;
	
	client = i2c_new_device(adapter, ltr558_i2c_info);
	i2c_put_adapter(adapter);
	if (!client)
			return -ENODEV;
	return i2c_add_driver(&ltr558_driver);
}

static void __exit ltr558_exit(void)
{
	i2c_del_driver(&ltr558_driver);
	if (!sensor_info)
		return; 
	i2c_unregister_device(sensor_info->i2c_client);
}


module_init(ltr558_init)
module_exit(ltr558_exit)

MODULE_AUTHOR("ZTE Corp");
MODULE_DESCRIPTION("LTR-558ALS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
