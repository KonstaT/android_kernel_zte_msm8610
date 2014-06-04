/*! \file sx9500.c
 * \brief  SX9500 Driver
 *
 * Driver for the SX9500
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
//#define DEBUG
#define DRIVER_NAME "sx9500"

#define MAX_WRITE_ARRAY_SIZE 32
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input/smtc/misc/sx95xx.h> /* main struct, interrupt,init,pointers */
#include <linux/input/smtc/misc/sx95xx_i2c_reg.h>
#include <linux/input/smtc/misc/sx9500_platform_data.h>  /* platform data */
#include <linux/input/smtc/misc/smtc_sar_test.h>
#include <linux/device.h> 

/*! \struct sx9500
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx9500 {
    /* Updated whenever their respective flags in irqsrc occur */
    u8 capstate_Value;

    psx9500_platform_data_t hw; /* specific platform data settings */
    smtc_sar_test_t sar_data;
} sx9500_t, *psx9500_t;


/*! \fn static int write_register(psx95XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx95XX_t this, u8 address, u8 value)
{
    struct i2c_client *i2c = 0;
    char buffer[2];
    int returnValue = 0;
    buffer[0] = address;
    buffer[1] = value;
    returnValue = -ENOMEM;
    if (this && this->bus) {
        i2c = this->bus;

        returnValue = i2c_master_send(i2c,buffer,2);
        dev_dbg(&i2c->dev,"write_register Address: 0x%x Value: 0x%x Return: %d\n",
                (unsigned int)address,(unsigned int)value,returnValue);
    }
    return returnValue;
}

/*! \fn static int read_register(psx95XX_t this, u8 address, u8 *value)
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx95XX_t this, u8 address, u8 *value)
{
    struct i2c_client *i2c = 0;
    s32 returnValue = 0;
    if (this && value && this->bus) {
        i2c = this->bus;
        returnValue = i2c_smbus_read_byte_data(i2c,address);
        dev_dbg(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",(unsigned int)address,(unsigned int)returnValue);
        if (returnValue >= 0) {
            *value = returnValue;
            return 0;
        } else {
            return returnValue;
        }
    }
    return -ENOMEM;
}

/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx95XX_t this)
{
    s32 returnValue = 0;
    returnValue = write_register(this,SX950X_IRQSTAT_REG,0xFF);
    return returnValue;
}
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    psx95XX_t this = dev_get_drvdata(dev);

    dev_dbg(this->pdev, "Reading IRQSTAT_REG\n");
    read_register(this,SX950X_IRQSTAT_REG,&reg_value);
    return sprintf(buf, "%d\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    psx95XX_t this = dev_get_drvdata(dev);
    unsigned long val;
    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;
    if (val) {
        dev_info( this->pdev, "Performing manual_offset_calibration()\n");
        manual_offset_calibration(this);
    }
    return count;
}

static DEVICE_ATTR(calibrate, 0644, manual_offset_calibration_show,
                   manual_offset_calibration_store);
static struct attribute *sx9500_attributes[] = {
    &dev_attr_calibrate.attr,
    NULL,
};
static struct attribute_group sx9500_attr_group = {
    .attrs = sx9500_attributes,
};
/*********************************************************************/
/*! \fn static int read_regStat(psx95XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx95XX_t this)
{
    u8 data = 0;
    if (this) {
        if (read_register(this,SX950X_IRQSTAT_REG,&data) == 0)
            return (data & 0x00FF);
    }
    return 0;
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct
 */

//#define INITIAL_REG_PRINT
	

static void hw_init(psx95XX_t this)
{
    psx9500_t pDevice = 0;
    psx9500_platform_data_t pdata = 0;
	

#ifdef INITIAL_REG_PRINT
    u8 data = 0;
#endif
	

    int i=0 ,j=0;
    /* configure device */
    dev_dbg(this->pdev, "Going to Setup I2C Registers\n");

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        while ( i < pdata->i2c_reg_num) {
            /* Write all registers/values contained in i2c_reg */
            dev_dbg(this->pdev, "Going to Write Reg: 0x%x Value: 0x%x\n",
                    (unsigned int)pdata->pi2c_reg[i].reg,(unsigned int)pdata->pi2c_reg[i].val);
            if(pdata->pi2c_reg[i].reg==6) {
                write_register(this, pdata->pi2c_reg[i].reg,0);
                j=i;
            } else {
                write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
            }
            i++;
        }
        write_register(this, pdata->pi2c_reg[j].reg,pdata->pi2c_reg[j].val);
    } else {
        dev_err(this->pdev, "ERROR! platform data 0x%x\n",(unsigned int)pDevice->hw);
    }
#ifdef INITIAL_REG_PRINT
	
    printk("@@@@@zhaoyang in SX9500 prox  init_reg_value\n");
    for(i = 0; i <= 0x10; i++) {
        read_register(this,i,&data);
        printk("reg[0x%x] = 0x%x\n",(unsigned int)i ,(unsigned int)data);
    }
    msleep(20);
    write_register(this, 0x00, 0xff);
    msleep(5);	
	
#endif
}
/*********************************************************************/
/*! \fn static int initialize(psx95XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx95XX_t this)
{
    //int i;
    if (this) {
        /* prepare reset by disabling any irq handling */
        this->irq_disabled = 1;
        disable_irq(this->irq);
        /* perform a reset */

        write_register(this,SX950X_SOFTRESET_REG,SX950X_SOFTRESET);
        /* wait until the reset has finished by monitoring NIRQ */

        dev_dbg(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");
        /* just sleep for awhile instead of using a loop with reading irq status */
        msleep(30);
        dev_dbg(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n",this->get_nirq_low());
        hw_init(this);
        msleep(10); /* make sure everything is running */
        manual_offset_calibration(this);

        /* re-enable interrupt handling */
        enable_irq(this->irq);
        this->irq_disabled = 0;

        /* make sure no interrupts are pending since enabling irq will only
         * work on next falling edge */
        read_regStat(this);
        dev_dbg(this->pdev, "Exiting initialize(). NIRQ = %d\n",this->get_nirq_low());
        return 0;
    }
    return -ENOMEM;
}

static void touchProcess(psx95XX_t this)
{
    u8 i = 0;
    u8 prox_status=0;

    psx9500_t pDevice = 0;
	
    if (this && (pDevice = this->pDevice)) {
        dev_dbg(this->pdev, "Inside touchProcess()\n");
        read_register(this, SX950X_CPSSTAT_REG, &i);
        pDevice->capstate_Value =  i;

#ifdef RUNTIME_REG_PRINT			
        printk("@@@@@@@	@@@@pDevice->capstate_Value = 0x%x   @@@@@@@\n\n", (unsigned int)pDevice->capstate_Value);
#endif	
	
        if( (i & 0xf0) > 0)
            prox_status = 1;

        smtc_sar_state(&pDevice->sar_data,prox_status);
        dev_dbg(this->pdev, "Leaving touchProcess()\n");
    }
}

#ifdef CONFIG_OF
static int sx9500_parse_dt(struct device *dev, psx9500_platform_data_t pdata)
{
//todo: add device tree data parse
	return 0;
}
#else
static int sx9500_parse_dt(struct device *dev, psx9500_platform_data_t pdata)
{
	reuturn  -ENODEV;
}
#endif

/*! \fn static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    psx95XX_t this = 0;
    psx9500_t pDevice = 0;
    psx9500_platform_data_t pplatData = 0;

    int dummy_ret;

    dev_info(&client->dev, "sx9500_probe()\n");

	printk("@@@zhaoyang196673 Semtech sx9500 on board, NOTICE: NOT attiny44a !!!\n");
	
        if (client->dev.of_node) {
                printk("sx9500 use device tree\n");
                pplatData = devm_kzalloc(&client->dev, sizeof(*pplatData), GFP_KERNEL);
                if (!pplatData) {
                        dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENODEV;
                }
                dummy_ret = sx9500_parse_dt(&client->dev, pplatData);
                if (dummy_ret) {
                        dev_err(&client->dev,
                                "Unable to parse platfrom data err=%d\n", dummy_ret);
			return -ENODEV;
                }
        } else {
                printk("sx9500 not use device tree\n");
                pplatData = client->dev.platform_data;
        }


    if (!pplatData) {
        dev_err(&client->dev, "platform data is required!\n");
        return -EINVAL;
    }

    if (!i2c_check_functionality(client->adapter,
                                 I2C_FUNC_SMBUS_READ_WORD_DATA))
        return -EIO;

    this = kzalloc(sizeof(sx95XX_t), GFP_KERNEL); /* create memory for main struct */
    dev_dbg(&client->dev, "\t Initialized Main Memory: 0x%x\n",(unsigned int)this);

    if (this) {
        /* In case we need to reinitialize data
         * (e.q. if suspend reset device) */
        this->init = initialize;
        /* shortcut to read status of interrupt */
        this->refreshStatus = read_regStat;
        /* pointer to function from platform data to get pendown
         * (1->NIRQ=0, 0->NIRQ=1) */
        this->get_nirq_low = pplatData->get_is_nirq_low;
        /* save irq in case we need to reference it */
        this->irq = client->irq;
        /* do we need to create an irq timer after interrupt ? */
        this->useIrqTimer = 0;
        /* Setup function to call on corresponding reg irq source bit
        */
        if (MAX_NUM_STATUS_BITS>= 8) {
            this->statusFunc[0] = 0; /* TXEN_STAT */
            this->statusFunc[1] = 0; /* UNUSED */
            this->statusFunc[2] = 0; /* UNUSED */
            this->statusFunc[3] = 0; /* CONV_STAT */
            this->statusFunc[4] = 0; /* COMP_STAT */
            this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
            this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
            this->statusFunc[7] = 0; /* RESET_STAT */
        }

        /* setup i2c communication */
        this->bus = client;
        i2c_set_clientdata(client, this);

        /* record device struct */
        this->pdev = &client->dev;

        /* create memory for device specific struct */
        this->pDevice = pDevice = kzalloc(sizeof(sx9500_t), GFP_KERNEL);
        dev_dbg(&client->dev, "\t Initialized Device Specific Memory: 0x%x\n",(unsigned int)pDevice);

        if (pDevice) {
            /* for accessing items in user data (e.g. calibrate) */
            dummy_ret = sysfs_create_group(&client->dev.kobj, &sx9500_attr_group);
            if (pplatData->init_platform_hw)
                pplatData->init_platform_hw();

            pDevice->hw = pplatData;
            smtc_sar_init(&(pDevice->sar_data),pplatData->psar_platform_data,
                          this->pdev,BUS_I2C);
        }
        return sx95XX_init(this);
    }
    return 0;
}

/*! \fn static int sx9500_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx95XX_remove()
 */
static int sx9500_remove(struct i2c_client *client)
{
    psx9500_platform_data_t pplatData =0;
    psx9500_t pDevice = 0;
    psx95XX_t this = i2c_get_clientdata(client);
    if (this && (pDevice = this->pDevice)) {
        sysfs_remove_group(&client->dev.kobj, &sx9500_attr_group);
        pplatData = client->dev.platform_data;
        if (pplatData && pplatData->exit_platform_hw)
            pplatData->exit_platform_hw();
        smtc_sar_remove(&pDevice->sar_data);
        kfree(this->pDevice);
    }
    return sx95XX_remove(this);
}


static struct i2c_device_id sx9500_idtable[] = {
    { DRIVER_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sx9500_idtable);

static struct of_device_id sx9500_match_table[] = {
         { .compatible = "semtech,sx9500", },
         { }, 
};

static struct i2c_driver sx9500_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = DRIVER_NAME,
	.of_match_table = sx9500_match_table, 
    },
    .id_table = sx9500_idtable,
    .probe	  = sx9500_probe,
    .remove	  = __devexit_p(sx9500_remove),
};

static int __init sx9500_init(void)
{

    return i2c_add_driver(&sx9500_driver);
}
static void __exit sx9500_exit(void)
{
    i2c_del_driver(&sx9500_driver);
}

late_initcall(sx9500_init); 
module_exit(sx9500_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9500 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
