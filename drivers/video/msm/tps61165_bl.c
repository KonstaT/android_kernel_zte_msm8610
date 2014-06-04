/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


/*We use one wire program method to realize it.*/

/* modified history
 
         when             who   what  where
     12/04/27         maxiaoping  add file and code.
     12/06/08	     maxiaoping  modify lcd_gpio for P865E01 new hardware board.	
     12/07/13	     maxiaoping  disable debug logs for lcd backlight.
     13/03/15	     maxiaoping  modify delay function to avoid cpu freq varied leading udelay() loops_per_jiffy not updated problem.
*/

#include "msm_fb.h"
#include "mipi_dsi.h"
#include <mach/gpio.h>

static bool onewiremode = false;

int zte_backlight=0;

static int lcd_bkl_ctl=90;    // zhoufan 20130824
void myudelay(unsigned int usec);
static void select_1wire_mode(void);
static void send_bkl_address(void);
static void send_bkl_data(int level);
void mipi_zte_set_backlight( int level);

#define TPS61165_DEVICE_ADDRESS 0x72

void myudelay(unsigned int usec)
{
	udelay(usec);
}
//Aviod udelay problem cause tps61165 Es mode time window wrong.
static void select_1wire_mode(void)
{
	gpio_direction_output(lcd_bkl_ctl, 1);
	myudelay(300);//myudelay(275);//myudelay(360);//myudelay(150);
	gpio_direction_output(lcd_bkl_ctl, 0);
	myudelay(600);//myudelay(650);//myudelay(840);//myudelay(300);				
	gpio_direction_output(lcd_bkl_ctl, 1);
	myudelay(700);//myudelay(770);//myudelay(1000);//myudelay(700);				
	
}

static void send_bkl_address(void)
{
	unsigned int i,j;
	i = TPS61165_DEVICE_ADDRESS;//0x72
	gpio_direction_output(lcd_bkl_ctl, 1);
	myudelay(10);
	//printk("PM_DEBUG_MXP: send_bkl_address \n");
	for(j = 0; j < 8; j++)
	{
		if(i & 0x80)
		{
			gpio_direction_output(lcd_bkl_ctl, 0);
			myudelay(10);
			gpio_direction_output(lcd_bkl_ctl, 1);
			myudelay(40);//myudelay(180);
		}
		else
		{
			gpio_direction_output(lcd_bkl_ctl, 0);
			myudelay(40);//myudelay(180);
			gpio_direction_output(lcd_bkl_ctl, 1);
			myudelay(10);
		}
		i <<= 1;
	}
	gpio_direction_output(lcd_bkl_ctl, 0);
	myudelay(10);
	gpio_direction_output(lcd_bkl_ctl, 1);

}

static void send_bkl_data(int level)
{
	unsigned int i,j;

	i = level & 0x1F;
	//printk("PM_DEBUG_MXP: tps61165 backlight data = %d \n",i);
	gpio_direction_output(lcd_bkl_ctl, 1);
	myudelay(10);
	//printk("PM_DEBUG_MXP: send_bkl_data \n");
	for(j = 0; j < 8; j++)
	{
		if(i & 0x80)
		{
			gpio_direction_output(lcd_bkl_ctl, 0);
			myudelay(10);
			gpio_direction_output(lcd_bkl_ctl, 1);
			myudelay(40);//myudelay(180);
		}
		else
		{
			gpio_direction_output(lcd_bkl_ctl, 0);
			myudelay(40);//myudelay(180);
			gpio_direction_output(lcd_bkl_ctl, 1);
			myudelay(10);
		}
		i <<= 1;
	}
	gpio_direction_output(lcd_bkl_ctl, 0);
	myudelay(10);
	gpio_direction_output(lcd_bkl_ctl, 1);
}

int zte_get_backlight(void)
{
	return zte_backlight;
}
EXPORT_SYMBOL(zte_get_backlight);

void mipi_zte_set_backlight(int level)
{
	/*value range is 1--32*/
	int current_lel =level;
	unsigned long flags;

	//printk("PM_DEBUG_MXP: tps lcd_set_bl level=%d\n", current_lel );
	if(current_lel < 1)
	{
		current_lel = 0;
	}
		
	if(current_lel > 32)
	{
		current_lel = 32;
	}

	zte_backlight=current_lel;
	local_irq_save(flags);

	if(current_lel==0)
	{
		gpio_direction_output(lcd_bkl_ctl, 0);
		mdelay(3);
		onewiremode = FALSE;
    	}
    	else 
	{
		if(!onewiremode)	
		{
			//printk("PM_DEBUG_MXP: before select_1wire_mode\n");
			select_1wire_mode();
			onewiremode = TRUE;
		}
		send_bkl_address();
		if(current_lel >= 30)
		{
			current_lel = 26;
		}
		else
		{
			current_lel = (current_lel *3)/4;
		}
		//printk("PM_DEBUG_MXP: The new bl  level=%d.\n",current_lel);
		send_bkl_data(current_lel);
	}
    	local_irq_restore(flags);
}

EXPORT_SYMBOL_GPL(mipi_zte_set_backlight);
