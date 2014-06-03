/* Lite-On LTR-558ALS Android Driver
 *
 * Copyright (C) 2011 Lite-On Technology Corp (Singapore)
 * Copyright (C) 2012 ZTE modify
 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#ifndef __LTR558_H
#define __LTR558_H

/* LTR-558 Registers */
#define LTR558_ALS_CONTR			0x80
#define LTR558_PS_CONTR				0x81
#define LTR558_PS_LED				0x82
#define LTR558_PS_N_PULSES			0x83
#define LTR558_PS_MEAS_RATE			0x84
#define LTR558_ALS_MEAS_RATE			0x85
#define LTR558_PART_ID				0x86
#define LTR558_MANUFACTURER_ID			0x87
#define LTR558_ALS_DATA_CH1_0			0x88
#define LTR558_ALS_DATA_CH1_1			0x89
#define LTR558_ALS_DATA_CH0_0			0x8A
#define LTR558_ALS_DATA_CH0_1			0x8B
#define LTR558_ALS_PS_STATUS			0x8C
#define LTR558_PS_DATA_0			0x8D
#define LTR558_PS_DATA_1			0x8E
#define LTR558_INTERRUPT			0x8F
#define LTR558_PS_THRES_UP_0			0x90
#define LTR558_PS_THRES_UP_1			0x91
#define LTR558_PS_THRES_LOW_0			0x92
#define LTR558_PS_THRES_LOW_1			0x93
#define LTR558_ALS_THRES_UP_0			0x97
#define LTR558_ALS_THRES_UP_1			0x98
#define LTR558_ALS_THRES_LOW_0			0x99
#define LTR558_ALS_THRES_LOW_1			0x9A
#define LTR558_INTERRUPT_PRST			0x9E

#define SET_BIT 				1
#define CLR_BIT 				0

#define ALS 					0
#define PS 					1

/* Default Settings (Bitshift left: Setting << Bit Number) */
#define ALS_GAIN				(0 << 3)
#define ALS_SW_RESET				(1 << 2)
#define ALS_MODE				(1 << 1)
#define ALS_INT_TIME				(0 << 3)
#define ALS_MEAS_RATE				(2 << 0)
#define ALS_INT_PRST_MASK			(0xf << 0)
#define ALS_INT_PRST				(1 << 0)

#define ALS_INT_FLAG				(1 << 3)
#define ALS_NEWDATA				(1 << 2)
	
#define PS_SW_RESET				(1 << 2)
#define PS_GAIN					(0 << 2)
#define PS_MODE					(1 << 1)
#define PS_MEAS_RATE				(2 << 0)
#define PS_INT_PRST_MASK			(0xf << 4)
#define PS_INT_PRST				(1 << 4)

#define PS_INT_FLAG				(0 << 1)
#define PS_NEWDATA				(0 << 0)

#define INTERRUPT_MODE				(3 << 0)
#define INTERRUPT_POL				(0 << 2)

#define LED_PULSE_FREQ				(3 << 5)
#define LED_DUTY_CYC				(1 << 3)
#define LED_PEAK_CURR				(3 << 0)

/* Power On response time in ms */
#define PON_DELAY				600
#define WAKEUP_DELAY				10

#define ALS_MIN_MEASURE_VAL			0
#define ALS_MAX_MEASURE_VAL			0xFFFF	
#define ALS_VALID_MEASURE_MASK			ALS_MAX_MEASURE_VAL
#define PS_MIN_MEASURE_VAL			0x0
#define PS_MAX_MEASURE_VAL			2047
#define PS_VALID_MEASURE_MASK  			PS_MAX_MEASURE_VAL

#define ALS_SUSPENDED				0x01
#define PS_SUSPENDED				0x02

/*
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */

#define LIGHT_IOCTL_MAGIC       	 	0XCF
#define LIGHT_IOCTL_ALS_ON       		_IO(LIGHT_IOCTL_MAGIC, 1)
#define LIGHT_IOCTL_ALS_OFF      		_IO(LIGHT_IOCTL_MAGIC, 2)
#define LIGHT_IOCTL_PROX_ON			_IO(LIGHT_IOCTL_MAGIC, 7)
#define LIGHT_IOCTL_PROX_OFF			_IO(LIGHT_IOCTL_MAGIC, 8)
#define LIGHT_IOCTL_PROX_CALIBRATE		_IO(LIGHT_IOCTL_MAGIC, 11)

//should change against with hardware connect
#define LTR558_I2C_BUS 				1
#define LTR558_IRQ_GPIO				ZTE_GPIO_LIGHT_SENSOR_IRQ
#define LTR558_I2C_ADDR				0x23

struct ltr558_platform_data {
	uint16_t pfd_ps_lowthresh;
	uint16_t pfd_ps_highthresh;
	uint8_t pfd_led_pulse;
};

#endif
