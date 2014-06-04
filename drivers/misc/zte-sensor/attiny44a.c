/**
 * Copyright (C) 2011 ZTE
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/attiny44a.h>

enum ATTINY_STATE {
	ATTI_OFF = 0,
	ATTI_ON,
};
struct attiny_private {
	struct attiny_platform_data * platform_data;
	struct input_dev *idev;
	struct mutex mutex;
	enum ATTINY_STATE state;
};
static	struct attiny_private *attiny_private_data = NULL;

static irqreturn_t attiny_handler(int irq, void * data)
{
	struct attiny_private *attiny_private_data = (struct attiny_private *)data;
	if(attiny_private_data->platform_data->get_irq_level()) {
		//printk("handle 1\n");
		input_report_abs(attiny_private_data->idev, ABS_DISTANCE, 1);
	} else {
		//printk("handle 0\n");
		input_report_abs(attiny_private_data->idev, ABS_DISTANCE, 0);
	}
	input_sync(attiny_private_data->idev);
	return IRQ_HANDLED;
}

static ssize_t attiny_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int state;
	mutex_lock(&attiny_private_data->mutex);
	state = (attiny_private_data->state == ATTI_ON) ? 1 : 0;
	mutex_unlock(&attiny_private_data->mutex);

	return sprintf(buf, "%d\n", state);
}
static ssize_t attiny_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int state;
	state = simple_strtoul(buf, NULL, 10);

	mutex_lock(&attiny_private_data->mutex);
	if (state && (attiny_private_data->state == ATTI_OFF)) {
		attiny_private_data->platform_data->enable(1);
		attiny_private_data->state = ATTI_ON;
		enable_irq(attiny_private_data->platform_data->irq);
	} else if (!state && (attiny_private_data->state == ATTI_ON)){
		disable_irq(attiny_private_data->platform_data->irq);
		attiny_private_data->platform_data->enable(0);
		attiny_private_data->state = ATTI_OFF;
	} else {
		printk("attiny has already %s\n", (attiny_private_data->state == ATTI_ON) ? "on" : "off");
	}
	mutex_unlock(&attiny_private_data->mutex);
		
	return count;
}
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, attiny_enable_show, attiny_enable_store);

static int __devinit attiny_probe(struct platform_device *pdev)
{
	int result = 0;
	struct attiny_platform_data * attiny_data = NULL;
	struct input_dev *idev = NULL;

	attiny_data = (struct attiny_platform_data *)pdev->dev.platform_data;
	if (!attiny_data || (!attiny_data->enable) || (attiny_data->irq < 0)) {
		dev_err(&pdev->dev, "get platform data failed!\n");
		return -ENODATA;
	}
	
	attiny_private_data = kzalloc(sizeof(struct attiny_private), GFP_KERNEL);
	if (!attiny_private_data) {
		dev_err(&pdev->dev, "allocate memory failed!\n");
		return -ENOMEM;
	}
	attiny_private_data->platform_data = attiny_data;
	mutex_init(&attiny_private_data->mutex);
	attiny_private_data->state = ATTI_OFF;
	platform_set_drvdata(pdev, attiny_private_data);

	idev = input_allocate_device();
	if (!idev) {
		dev_err(&pdev->dev, "allocate input device failed!\n");
		result = -ENOMEM;
		goto err_allocate_input;
	}
	idev->name = ATTINY_NAME;
	idev->id.bustype = BUS_VIRTUAL;
	__set_bit(EV_ABS, idev->evbit);
	input_set_abs_params(idev, ABS_DISTANCE, 0, 1, 0, 0);
	result = input_register_device(idev);
	if (result) {
		dev_err(&pdev->dev, "register input device failed!\n");
		goto err_register_input;
	}
	attiny_private_data->idev = idev;


	result = request_irq(attiny_data->irq, attiny_handler, IRQ_TYPE_EDGE_BOTH, "attiny_irq", attiny_private_data);	
	if (result) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto err_request_irq;
	}
	disable_irq(attiny_data->irq);

	result = device_create_file(&idev->dev, &dev_attr_enable);
	if(result ) {
		dev_err(&pdev->dev, "create device file failed\n");
		goto err_create_file;
	}
	return result;

err_create_file:
	free_irq(attiny_data->irq, attiny_private_data);
err_request_irq:
	input_unregister_device(idev);
	goto err_allocate_input;
err_register_input:
	input_free_device(idev);
err_allocate_input:
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&attiny_private_data->mutex);
	kfree(attiny_private_data);
	return result;
	
}
static int __devexit attiny_remove(struct platform_device *pdev)
{
	struct attiny_private *attiny_private_data;

	attiny_private_data = platform_get_drvdata(pdev);
	
	device_remove_file(&attiny_private_data->idev->dev, &dev_attr_enable);
	free_irq(attiny_private_data->platform_data->irq, attiny_private_data);
	input_unregister_device(attiny_private_data->idev);
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&attiny_private_data->mutex);
	kfree(attiny_private_data);
	return 0;
}

#ifdef CONFIG_PM
static int attiny_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct attiny_private *attiny_private_data;
	attiny_private_data = platform_get_drvdata(pdev);
	if (attiny_private_data->state == ATTI_ON) {
		disable_irq(attiny_private_data->platform_data->irq);
		attiny_private_data->platform_data->enable(0);
	}
	return 0;
}
static int attiny_resume(struct platform_device *pdev)
{
	struct attiny_private *attiny_private_data;
	attiny_private_data = platform_get_drvdata(pdev);
	if (attiny_private_data->state == ATTI_ON) {
		attiny_private_data->platform_data->enable(1);
		enable_irq(attiny_private_data->platform_data->irq);
	}
	return 0;
}
#endif

static struct platform_driver attiny_driver = {
        .probe          = attiny_probe,
        .remove         = __devexit_p(attiny_remove),
#ifdef CONFIG_PM
	.suspend	= attiny_suspend,
	.resume		= attiny_resume,
#endif
        .driver         = {
                .name   = ATTINY_NAME,
                .owner  = THIS_MODULE,
        },
};

static int __init attiny_init(void) {
	return platform_driver_register(&attiny_driver);
}
static void __exit attiny_exit(void) {
	platform_driver_unregister(&attiny_driver);
}

module_init(attiny_init);
module_exit(attiny_exit);

MODULE_AUTHOR("wanghaifei <wang.haifei@zte.com.cn>");
MODULE_DESCRIPTION("attiny44a driver");
MODULE_LICENSE("GPL");

