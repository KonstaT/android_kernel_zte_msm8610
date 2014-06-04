/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

//maxiaoping 20130826
#if 1
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <mach/../../sysmon.h>

#define MAX_KEYPAD_BL_LEVEL	16
#define BLINK_LED_NUM   2

#define MSM_LED_DRIVER_NAME		"qcom,leds-gpio-color-led"
#define LED_TRIGGER_DEFAULT		"none"

struct BLINK_LED_data{
	int blink_flag;
	int blink_led_flag;  // 0: off, 1:0n
	int blink_on_time;  //ms
	int blink_off_time;  //ms
	struct timer_list timer;
	struct work_struct work_led_on;
	struct work_struct work_led_off;
	struct led_classdev led;
};

struct STATUS_LED_data {
	struct mutex data_lock;
	struct BLINK_LED_data blink_led[2];  /* red, green */
	int red_led;
	int green_led;
	int led_suspend_flag;
};

static struct of_device_id msm_led_of_match[] = {
	{.compatible = MSM_LED_DRIVER_NAME,},
	{},
};

struct STATUS_LED_data *STATUS_LED = NULL;

#ifdef CONFIG_MODEM_LED
static int send_info = 0;
enum {
	RED_ON,
	RED_OFF,
	RED_BLINK_ON,
	RED_BLINK_OFF,
	GREEN_ON,
	GREEN_OFF,
	GREEN_BLINK_ON,
	GREEN_BLINK_OFF,
};
static char* led_cmd[] = {
	[RED_ON]			= "leds:red:on",
	[RED_OFF]			= "leds:red:off",
	[RED_BLINK_ON]		= "leds:red:blink_on",
	[RED_BLINK_OFF]		= "leds:red:blink_off",
	[GREEN_ON]			= "leds:green:on",
	[GREEN_OFF]			= "leds:green:off",
	[GREEN_BLINK_ON]	= "leds:green:blink_on",
	[GREEN_BLINK_OFF]	= "leds:green:blink_off",
};
#endif

static void pmic_red_led_on(struct work_struct *work)
{	   
	gpio_direction_output(STATUS_LED->red_led, 1);   	   
}

static void pmic_red_led_off(struct work_struct *work)
{
	gpio_direction_output(STATUS_LED->red_led, 0);   	   
}

static void pmic_green_led_on(struct work_struct *work)
{
	gpio_direction_output(STATUS_LED->green_led, 1);      
}

static void pmic_green_led_off(struct work_struct *work)
{
	gpio_direction_output(STATUS_LED->green_led, 0);   	   
}

static void (*func[4])(struct work_struct *work) = {
	pmic_red_led_on,
	pmic_red_led_off,
	pmic_green_led_on,
	pmic_green_led_off,
};

static void msm_pmic_bl_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
#ifdef CONFIG_MODEM_LED
	if (!send_info) {
		char buf[50];
		int rc;
		
		snprintf(buf, sizeof(buf),"leds:info:%d:%d:%d:%d",
				STATUS_LED->red_led,STATUS_LED->green_led,
				STATUS_LED->blink_led[0].blink_on_time, 
				STATUS_LED->blink_led[0].blink_off_time);
		rc = sysmon_send_led_cmd(SYSMON_SS_MODEM, buf);
		if (!rc)
			send_info = 1;
		else {
			dev_info(led_cdev->dev, "led send info error in %s, %s\n", __func__, led_cdev->name);
			return;
		}
	}
#endif

	if (!strcmp(led_cdev->name, "red")) 
	{
		if(value == LED_OFF)
		{
		#ifdef CONFIG_MODEM_LED
			sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[RED_OFF]);
		#else
			gpio_direction_output(STATUS_LED->red_led, 0);
		#endif
		} else {
		#ifdef CONFIG_MODEM_LED
			sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[RED_ON]);
		#else
			gpio_direction_output(STATUS_LED->red_led, 1); 
		#endif
		}
	} 
	else //green
	{
		if(value == LED_OFF)
		{
		#ifdef CONFIG_MODEM_LED
			sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[GREEN_OFF]);
		#else
			gpio_direction_output(STATUS_LED->green_led, 0);  
		#endif
		} else {
		#ifdef CONFIG_MODEM_LED
			sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[GREEN_ON]);
		#else
			gpio_direction_output(STATUS_LED->green_led, 1);
		#endif
		}		
	}
}

static void pmic_leds_timer(unsigned long arg)
{
	struct BLINK_LED_data *b_led = (struct BLINK_LED_data *)arg;

	if(b_led->blink_flag)
	{
		if(b_led->blink_led_flag)
		{ 
			schedule_work(&b_led->work_led_off);
			b_led->blink_led_flag = 0;  
			mod_timer(&b_led->timer,jiffies + msecs_to_jiffies(b_led->blink_off_time));
		}else{
			schedule_work(&b_led->work_led_on);
			b_led->blink_led_flag = 1;
			mod_timer(&b_led->timer,jiffies + msecs_to_jiffies(b_led->blink_on_time));
		}
	}else {
		if(b_led->led.brightness)
			schedule_work(&b_led->work_led_on);
		else
			schedule_work(&b_led->work_led_off);
	}

}

static ssize_t led_blink_solid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = 0;

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else
		idx = 1;

	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);

	/* no lock needed for this */
	sprintf(buf, "%u\n", STATUS_LED->blink_led[idx].blink_flag);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t led_blink_solid_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	char *after;
	unsigned long state;
	ssize_t ret = -EINVAL;
	size_t count;

	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else
		idx = 1;

	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);

#ifdef CONFIG_MODEM_LED
	if (!send_info) {
		char buf[50];
		int rc;

		snprintf(buf, sizeof(buf),"leds:info:%d:%d:%d:%d",
				STATUS_LED->red_led,STATUS_LED->green_led,
				STATUS_LED->blink_led[idx].blink_on_time, 
				STATUS_LED->blink_led[idx].blink_off_time);
		rc = sysmon_send_led_cmd(SYSMON_SS_MODEM, buf);
		if (!rc)
			send_info = 1;
		else {
			dev_info(dev, "led send info error in %s\n", __func__);
			return ret;
		}
	}
#endif
	state = simple_strtoul(buf, &after, 10);

	count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		mutex_lock(&STATUS_LED->data_lock);
		if(0 == state)
		{
		#ifdef CONFIG_MODEM_LED
			if (idx == 0)
				sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[RED_BLINK_OFF]);
			else
				sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[GREEN_BLINK_OFF]);
		#endif
			STATUS_LED->blink_led[idx].blink_flag= 0;
		}
		else
		{
		#ifdef CONFIG_MODEM_LED
			if (idx == 0)
				sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[RED_BLINK_ON]);
			else
				sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[GREEN_BLINK_ON]);
		#else
			schedule_work(&STATUS_LED->blink_led[idx].work_led_on);
			mod_timer(&STATUS_LED->blink_led[idx].timer,jiffies + msecs_to_jiffies(STATUS_LED->blink_led[idx].blink_on_time));
		#endif
			STATUS_LED->blink_led[idx].blink_flag= 1;
			STATUS_LED->blink_led[idx].blink_led_flag = 1;

		}
		mutex_unlock(&STATUS_LED->data_lock);
	}

	return ret;
}

static DEVICE_ATTR(blink, 0644, led_blink_solid_show, led_blink_solid_store);

int led_parse_dt(struct platform_device *pdev)
{
	u32 temp_val;
	int rc;
	char *key;

	key = "qcom,red-led";
	rc = of_get_named_gpio(pdev->dev.of_node, key, 0);
	if (STATUS_LED->red_led < 0)
		goto parse_error;
	STATUS_LED->red_led = rc;

	key = "qcom,green-led";
	rc = of_get_named_gpio(pdev->dev.of_node, key, 0);
	if (STATUS_LED->green_led < 0)
		goto parse_error;
	STATUS_LED->green_led = rc;

	key = "qcom,blink-on-time";
	rc = of_property_read_u32(pdev->dev.of_node, key, &temp_val);
	if (rc && (rc != -EINVAL)) 
		goto parse_error;
	else if(rc) {
		dev_warn(&pdev->dev,"led have no property %s, use default\n", key);
		STATUS_LED->blink_led[0].blink_on_time = 500;
		STATUS_LED->blink_led[1].blink_on_time = 500;			
	} else {
		STATUS_LED->blink_led[0].blink_on_time = temp_val;
		STATUS_LED->blink_led[1].blink_on_time = temp_val;
	}

	key = "qcom,blink-off-time";
	rc = of_property_read_u32(pdev->dev.of_node, key, &temp_val);
	if (rc && (rc != -EINVAL)) 
		goto parse_error;
	else if(rc) {
		dev_warn(&pdev->dev,"led have no property %s, use default\n", key);
		STATUS_LED->blink_led[0].blink_off_time = 500;
		STATUS_LED->blink_led[1].blink_off_time = 500;			
	} else {
		STATUS_LED->blink_led[0].blink_off_time = temp_val;
		STATUS_LED->blink_led[1].blink_off_time = temp_val;
	}

	return 0;
parse_error:
	dev_err(&pdev->dev,"parse property %s failed, rc = %d\n", key, rc);
	return rc;
}

int led_gpio_config(struct platform_device *pdev)
{
	int rc;

	rc = gpio_request(STATUS_LED->red_led, "msm_red_led");
	if (rc) {
		dev_err(&pdev->dev, "Failed to request gpio %d,rc = %d\n", STATUS_LED->red_led, rc);
		return rc;
	}

	gpio_tlmm_config(GPIO_CFG(STATUS_LED->red_led, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_direction_output(STATUS_LED->red_led, 0);

	rc = gpio_request(STATUS_LED->green_led, "msm_green_led");
	if (rc) {
		dev_err(&pdev->dev,"Failed to request gpio %d,rc = %d\n", STATUS_LED->green_led, rc);
		gpio_free(STATUS_LED->red_led);
		return rc;
	}

	gpio_tlmm_config(GPIO_CFG(STATUS_LED->green_led, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_direction_output(STATUS_LED->green_led, 0);
	return 0;
}

void led_gpio_unconfig(void)
{
	gpio_free(STATUS_LED->green_led);
	gpio_free(STATUS_LED->red_led);
}

int __devinit msm_led_probe(struct platform_device *pdev)
{
	int rc = 0;
	int i, j;

	if (pdev->dev.of_node == NULL) {
		dev_info(&pdev->dev, "can not find device tree node\n");
		return -ENODEV;
	}

	STATUS_LED = kzalloc(sizeof(struct STATUS_LED_data), GFP_KERNEL);
	if (STATUS_LED == NULL) {
		dev_err(&pdev->dev,"no memory for device\n");
		return -ENOMEM;
	}

	rc = led_parse_dt(pdev);
	if (rc) {
		goto err_alloc_failed;
	}

	rc = led_gpio_config(pdev);
	if (rc) {
		dev_err(&pdev->dev,"config led gpio failed\n");
		goto err_alloc_failed;
	}

	STATUS_LED->blink_led[0].led.name = "red";
	STATUS_LED->blink_led[0].led.brightness_set = msm_pmic_bl_led_set;
	STATUS_LED->blink_led[0].led.brightness = LED_OFF;
	STATUS_LED->blink_led[0].blink_flag = 0;

	STATUS_LED->blink_led[1].led.name = "green";
	STATUS_LED->blink_led[1].led.brightness_set = msm_pmic_bl_led_set;
	STATUS_LED->blink_led[1].led.brightness = LED_OFF;
	STATUS_LED->blink_led[1].blink_flag = 0;

	mutex_init(&STATUS_LED->data_lock);

	for (i = 0; i < 2; i++) {	/* red, green */
		rc= led_classdev_register(&pdev->dev, &STATUS_LED->blink_led[i].led);
		if (rc) {
			dev_err(&pdev->dev,"STATUS_LED: led_classdev_register failed\n");
			goto err_led_classdev_register_failed;
		}
	}

	for (i = 0; i < 2; i++) {
		rc = device_create_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink);
		if (rc) {
			dev_err(&pdev->dev,"STATUS_LED: create dev_attr_blink failed\n");
			goto err_out_attr_blink;
		}
	}

	dev_set_drvdata(&pdev->dev, STATUS_LED);

	for (i = 0; i < 2; i++)
	{
		INIT_WORK(&STATUS_LED->blink_led[i].work_led_on, func[i*2]);
		INIT_WORK(&STATUS_LED->blink_led[i].work_led_off, func[i*2+1]);
		setup_timer(&STATUS_LED->blink_led[i].timer, pmic_leds_timer, (unsigned long)&STATUS_LED->blink_led[i]);
		msm_pmic_bl_led_set(&STATUS_LED->blink_led[i].led, LED_OFF);
	}

	printk(KERN_NOTICE "PM_DEBUG_MXP:Exit msm_led_probe.\r\n");		
	return 0;

err_out_attr_blink:
	for (j = 0; j < i; j++)
		device_remove_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink);
	i = 2;

err_led_classdev_register_failed:
	for (j = 0; j < i; j++)
		led_classdev_unregister(&STATUS_LED->blink_led[i].led);
	led_gpio_unconfig();
err_alloc_failed:
	kfree(STATUS_LED);

	return rc;

}


int __devexit msm_led_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 2; i++){
		device_remove_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink);
		led_classdev_unregister(&STATUS_LED->blink_led[i].led);
	}
	led_gpio_unconfig();
	kfree(STATUS_LED);
	return 0;
}

static int msm_led_suspend(struct device *dev)
{
//should always define CONFIG_MODEM_LED, this code only for test current,never used for normal purpose
#ifndef CONFIG_MODEM_LED 
	extern void led_set_sleep_time(int time);
	int i;
	for (i = 0; i < 2; i++)
		if ((STATUS_LED->blink_led[i].blink_flag) && (STATUS_LED->blink_led[i].blink_led_flag))
			led_set_sleep_time(STATUS_LED->blink_led[i].blink_on_time);
		else if ((STATUS_LED->blink_led[i].blink_flag) && !(STATUS_LED->blink_led[i].blink_led_flag))
			led_set_sleep_time(STATUS_LED->blink_led[i].blink_off_time);
#endif
	STATUS_LED->led_suspend_flag = 1;
	return 0;
}

static int msm_led_resume(struct device *dev)
{
	STATUS_LED->led_suspend_flag = 0;
	return 0;
}

static SIMPLE_DEV_PM_OPS(led_pm_ops, msm_led_suspend, msm_led_resume);

static struct platform_driver msm_led_driver = {
	.probe		= msm_led_probe,
	.remove		= __devexit_p(msm_led_remove),
	.driver		= {
		.name	= "zte,leds",
		.owner	= THIS_MODULE,
		.of_match_table = msm_led_of_match,
		.pm = &led_pm_ops,
	},
};

static int __init msm_led_init(void)
{
	return platform_driver_register(&msm_led_driver);
}

static void __exit msm_led_exit(void)
{
	platform_driver_unregister(&msm_led_driver);
}

late_initcall(msm_led_init);
module_exit(msm_led_exit);

MODULE_DESCRIPTION("QCOM MSM GPIO LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-msm-gpio-leds");
#else
#include <mach/pmic.h>

#define MAX_KEYPAD_BL_LEVEL	16

static void msm_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret;

	ret = pmic_set_led_intensity(LED_KEYPAD, value / MAX_KEYPAD_BL_LEVEL);
	if (ret)
		dev_err(led_cdev->dev, "can't set keypad backlight\n");
}

static struct led_classdev msm_kp_bl_led = {
	.name			= "keyboard-backlight",
	.brightness_set		= msm_keypad_bl_led_set,
	.brightness		= LED_OFF,
};

static int msm_pmic_led_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &msm_kp_bl_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led class driver\n");
		return rc;
	}
	msm_keypad_bl_led_set(&msm_kp_bl_led, LED_OFF);
	return rc;
}

static int __devexit msm_pmic_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&msm_kp_bl_led);

	return 0;
}

#ifdef CONFIG_PM
static int msm_pmic_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&msm_kp_bl_led);

	return 0;
}

static int msm_pmic_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&msm_kp_bl_led);

	return 0;
}
#else
#define msm_pmic_led_suspend NULL
#define msm_pmic_led_resume NULL
#endif

static struct platform_driver msm_pmic_led_driver = {
	.probe		= msm_pmic_led_probe,
	.remove		= __devexit_p(msm_pmic_led_remove),
	.suspend	= msm_pmic_led_suspend,
	.resume		= msm_pmic_led_resume,
	.driver		= {
		.name	= "pmic-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_pmic_led_init(void)
{
	return platform_driver_register(&msm_pmic_led_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&msm_pmic_led_driver);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("MSM PMIC LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-leds");
#endif
