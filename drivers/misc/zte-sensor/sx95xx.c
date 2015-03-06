/*! \file sx86xx.c
 * \brief  Helper functions for Interrupt handling on SXxx products
 *
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

//#define DEBUG

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#define MAX_WRITE_ARRAY_SIZE 32
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <linux/input/smtc/misc/sx95xx.h> /* main struct, interrupt,init,pointers */
#include <linux/input/smtc/misc/sx95xx.h> /* main struct, interrupt,init,pointers */
#include <linux/input/smtc/misc/sx95xx_i2c_reg.h>
#include <linux/input/smtc/misc/sx9500_platform_data.h>  /* platform data */
#include <linux/input/smtc/misc/smtc_sar_test.h>


//#define RUNTIME_REG_PRINT
#ifdef RUNTIME_REG_PRINT
//static int read_registerEx(psx95XX_t this, unsigned char reg,
//                           unsigned char *data, int size)
//{
//    struct i2c_client *i2c = 0;
//    int ret = 0;
//    u8 tx[] = {
//        reg
//    };
//    if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE)) {
//        dev_dbg(this->pdev, "inside sx868x_i2c_readEx()\n");
//        dev_dbg(this->pdev,
//                "going to call i2c_master_send(0x%x,0x%x,1) Reg: 0x%x\n",
//                (unsigned int)i2c->addr,(unsigned int)reg,(unsigned int)reg);
//        ret = i2c_master_send(i2c,tx,1);
//        if (ret >= 0) {
//            dev_dbg(this->pdev, "going to call i2c_master_recv(0x%x,0x_x,%x)\n",
//                    (unsigned int)i2c->addr,(unsigned int)size);
//            ret = i2c_master_recv(i2c, data, size);
//        }
//    }
//    if (unlikely(ret < 0))
//        dev_err(this->pdev, "I2C read error\n");
//    dev_dbg(this->pdev, "leaving sx868x_i2c_readEx()\n");
//    return ret;
//}

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
/*! \brief Sends a write register range to the device
* \param this Pointer to main parent struct
* \param reg 8-bit register address (base address)
* \param data pointer to 8-bit register values
* \param size size of the data pointer
* \return Value from i2c_master_send
*/
//static int write_registerEx(psx95XX_t this, unsigned char reg,
//                            unsigned char *data, int size)
//{
//    struct i2c_client *i2c = 0;
//    u8 tx[MAX_WRITE_ARRAY_SIZE];
//    int ret = 0;

//    if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE)) {
//        dev_dbg(this->pdev, "inside sx868x_i2c_writeEx()\n");
//        tx[0] = reg;
//        dev_dbg(this->pdev, "going to call i2c_master_send(0x%x, 0x%x ",
//                (unsigned int)i2c->addr,tx[0]);
//        for (ret = 0; ret < size; ret++) {
//            tx[ret+1] = data[ret];
//            dev_dbg(this->pdev, "0x%x, ",tx[ret+1]);
//        }
//        dev_dbg(this->pdev, "\n");

//        ret = i2c_master_send(i2c, tx, size+1 );
//        if (ret < 0)
//            dev_err(this->pdev, "I2C write error\n");
//    }
//    dev_dbg(this->pdev, "leaving sx868x_i2c_write()\n");


//    return ret;
//}

#define SPEED_DOWN 5
//At lesat 3 ,otherwise too many printk to make system NO respond
#endif


#ifdef USE_THREADED_IRQ
static void sx95XX_process_interrupt(psx95XX_t this,u8 nirqlow)
{
    int status = 0;
    int counter = 0;

	
#ifdef RUNTIME_REG_PRINT
    static int ll = 0;
    int j,k;
    u8 data = 0;
#endif
		

    if (!this) {
        printk(KERN_ERR "sx95XX_worker_func, NULL sx95XX_t\n");
        return;
    }

    /* since we are not in an interrupt don't need to disable irq. */
    status = this->refreshStatus(this);
    counter = -1;
    dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);
    while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
        dev_dbg(this->pdev, "Looping Counter %d\n",counter);
        if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
            dev_dbg(this->pdev, "Function Pointer Found. Calling\n");
            this->statusFunc[counter](this);
        }
    }

    
#ifdef RUNTIME_REG_PRINT
    if(ll++ == SPEED_DOWN) {
        ll = 0;
        for(j = 2 ; j < 4; j++) {
            write_register(this, 0x20, j);
            printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@write reg[0x20] = 0x%x   @@@@@@@@@@@@@@@@@@@@@@@@@@\n", j);
            for(k = 1; k <= 0x0a; k++) {
                read_register(this,0x20 + k, &data);
                printk("reg[0x%x] = 0x%x\n",0x20 + k ,data);
            }
        }
        read_register(this, 1,  &data);
        printk("\nreg[0x%x] = 0x%x\n",0x01 ,data);
    }
#endif
    

    if (unlikely(this->useIrqTimer && nirqlow)) {
        /* In case we need to send a timer for example on a touchscreen
         * checking penup, perform this here
         */
        cancel_delayed_work(&this->dworker);
        schedule_delayed_work(&this->dworker,msecs_to_jiffies(this->irqTimeout));
        dev_info(this->pdev,"Schedule Irq timer");
    }
}

static void sx95XX_worker_func(struct work_struct *work)
{
    psx95XX_t this = 0;
    if (work) {
        this = container_of(work,sx95XX_t,dworker.work);
        if (!this) {
            printk(KERN_ERR "sx95XX_worker_func, NULL sx95XX_t\n");
            return;
        }
        if ((!this->get_nirq_low) || (!this->get_nirq_low())) {
            /* only run if nirq is high */
            sx95XX_process_interrupt(this,0);
        }
    } else {
        printk(KERN_ERR "sx95XX_worker_func, NULL work_struct\n");
    }
}

static irqreturn_t sx95XX_interrupt_thread(int irq, void *data)
{
    psx95XX_t this = 0;
    this = data;

    mutex_lock(&this->mutex);
    dev_dbg(this->pdev, "sx95XX_irq\n");
    if ((!this->get_nirq_low) || this->get_nirq_low()) {
        sx95XX_process_interrupt(this,1);
    } else
        dev_err(this->pdev, "sx95XX_irq - nirq read high\n");
    mutex_unlock(&this->mutex);
    return IRQ_HANDLED;
}
#else
static void sx95XX_schedule_work(psx95XX_t this, unsigned long delay)
{
    unsigned long flags;
    if (this) {
        dev_dbg(this->pdev, "sx95XX_schedule_work()\n");
        spin_lock_irqsave(&this->lock,flags);
        /* Stop any pending penup queues */
        cancel_delayed_work(&this->dworker);
        schedule_delayed_work(&this->dworker,delay);
        spin_unlock_irqrestore(&this->lock,flags);
    } else
        printk(KERN_ERR "sx95XX_schedule_work, NULL psx95XX_t\n");
}

static irqreturn_t sx95XX_irq(int irq, void *pvoid)
{
    psx95XX_t this = 0;
    if (pvoid) {
        this = (psx95XX_t)pvoid;
        dev_dbg(this->pdev, "sx95XX_irq\n");
        if ((!this->get_nirq_low) || this->get_nirq_low()) {
            dev_dbg(this->pdev, "sx95XX_irq - Schedule Work\n");
            sx95XX_schedule_work(this,0);
        } else
            dev_err(this->pdev, "sx95XX_irq - nirq read high\n");
    } else
        printk(KERN_ERR "sx95XX_irq, NULL pvoid\n");
    return IRQ_HANDLED;
}

static void sx95XX_worker_func(struct work_struct *work)
{
    psx95XX_t this = 0;
    int status = 0;
    int counter = 0;
    u8 nirqLow = 0;
    if (work) {
        this = container_of(work,sx95XX_t,dworker.work);

        if (!this) {
            printk(KERN_ERR "sx95XX_worker_func, NULL sx95XX_t\n");
            return;
        }
        if (unlikely(this->useIrqTimer)) {
            if ((!this->get_nirq_low) || this->get_nirq_low()) {
                nirqLow = 1;
            }
        }
        /* since we are not in an interrupt don't need to disable irq. */
        status = this->refreshStatus(this);
        counter = -1;
        dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);
        while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
            dev_dbg(this->pdev, "Looping Counter %d\n",counter);
            if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
                dev_dbg(this->pdev, "Function Pointer Found. Calling\n");
                this->statusFunc[counter](this);
            }
        }
        if (unlikely(this->useIrqTimer && nirqLow)) {
            /* Early models and if RATE=0 for newer models require a penup timer */
            /* Queue up the function again for checking on penup */
            sx95XX_schedule_work(this,msecs_to_jiffies(this->irqTimeout));
        }
    } else {
        printk(KERN_ERR "sx95XX_worker_func, NULL work_struct\n");
    }
}
#endif

void sx95XX_suspend(psx95XX_t this)
{	
    if (this){
        disable_irq(this->irq);
        
#ifndef CONFIG_OF
	gpio_request(ZTE_GPIO_SAR_SENSOR_TXEN, "sar_txen_pin");
	gpio_direction_output(ZTE_GPIO_SAR_SENSOR_TXEN, 0);
	gpio_free(ZTE_GPIO_SAR_SENSOR_TXEN);
#else
//todo:add device tree code
#endif
    }
}
void sx95XX_resume(psx95XX_t this)
{
    if (this) {
#ifndef CONFIG_OF
				gpio_request(ZTE_GPIO_SAR_SENSOR_TXEN, "sar_txen_pin");
				gpio_direction_output(ZTE_GPIO_SAR_SENSOR_TXEN, 1);
				gpio_free(ZTE_GPIO_SAR_SENSOR_TXEN);
				
#else
//todo:add device tree code
#endif
#ifdef USE_THREADED_IRQ
        mutex_lock(&this->mutex);
        /* Just in case need to reset any uncaught interrupts */
        sx95XX_process_interrupt(this,0);
        mutex_unlock(&this->mutex);
#else
        sx95XX_schedule_work(this,0);
#endif
        if (this->init)
            this->init(this);
        enable_irq(this->irq);
    }
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*TODO: Should actually call the device specific suspend/resume
 * As long as the kernel suspend/resume is setup, the device
 * specific ones will be called anyways
 */
extern suspend_state_t get_suspend_state(void);
void sx95XX_early_suspend(struct early_suspend *h)
{
    psx95XX_t this = 0;
    dev_dbg(this->pdev, "inside sx95XX_early_suspend()\n");
    this = container_of(h, sx95XX_t, early_suspend);
    sx95XX_suspend(this);
    dev_dbg(this->pdev, "exit sx95XX_early_suspend()\n");
}

void sx95XX_late_resume(struct early_suspend *h)
{
    psx95XX_t this = 0;
    dev_dbg(this->pdev, "inside sx95XX_late_resume()\n");
    this = container_of(h, sx95XX_t, early_suspend);
    sx95XX_resume(this);
    dev_dbg(this->pdev, "exit sx95XX_late_resume()\n");
}
#endif

int sx95XX_init(psx95XX_t this)
{
    int err = 0;
    if (this && this->pDevice) {
#ifdef USE_THREADED_IRQ
        /* initialize worker function */
        INIT_DELAYED_WORK(&this->dworker, sx95XX_worker_func);
        /* initialize mutex */
        mutex_init(&this->mutex);
        /* initailize interrupt reporting */
        this->irq_disabled = 0;
        err = request_threaded_irq(this->irq, NULL, sx95XX_interrupt_thread,
                                   IRQF_TRIGGER_FALLING, this->pdev->driver->name,
                                   this);
#else
        /* initialize spin lock */
        spin_lock_init(&this->lock);

        /* initialize worker function */
        INIT_DELAYED_WORK(&this->dworker, sx95XX_worker_func);

        /* initailize interrupt reporting */
        this->irq_disabled = 0;
        err = request_irq(this->irq, sx95XX_irq, IRQF_TRIGGER_FALLING,
                          this->pdev->driver->name, this);
#endif
        if (err) {
            dev_err(this->pdev, "irq %d busy?\n", this->irq);
            return err;
        }
#ifdef USE_THREADED_IRQ
        dev_info(this->pdev, "registered with threaded irq (%d)\n", this->irq);
#else
        dev_info(this->pdev, "registered with irq (%d)\n", this->irq);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
        this->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        this->early_suspend.suspend = sx95XX_early_suspend;
        this->early_suspend.resume = sx95XX_late_resume;
        register_early_suspend(&this->early_suspend);
        if (has_wake_lock(WAKE_LOCK_SUSPEND) == 0 &&
            get_suspend_state() == PM_SUSPEND_ON)
            sx95XX_early_suspend(&this->early_suspend);
#endif //CONFIG_HAS_EARLYSUSPEND
        /* call init function pointer (this should initialize all registers */
        if (this->init)
            return this->init(this);
        dev_err(this->pdev,"No init function!!!!\n");
    }
    return -ENOMEM;
}

int sx95XX_remove(psx95XX_t this)
{
    if (this) {
        cancel_delayed_work_sync(&this->dworker); /* Cancel the Worker Func */
        /*destroy_workqueue(this->workq); */
#ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&this->early_suspend);
#endif
        free_irq(this->irq, this);
        kfree(this);
        return 0;
    }
    return -ENOMEM;
}
