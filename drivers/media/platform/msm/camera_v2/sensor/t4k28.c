/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#define T4K28_SENSOR_NAME "t4k28"
#define PLATFORM_DRIVER_NAME "msm_camera_t4k28"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


//#define FULLSIZE_OUTPUT //for fullsize preview and snapshot


#ifndef FULLSIZE_OUTPUT
//params used for  snapshot switch to preview
typedef enum
{  
      AE_enable,  
	AE_lock,  
	AE_disable,
} AE_status;

uint32_t set_pv_back_es = 0xff;
uint16_t set_pv_back_ag = 0x20;
uint16_t set_pv_back_dg = 0x10;
#endif

DEFINE_MSM_MUTEX(t4k28_mut);
static struct msm_sensor_ctrl_t t4k28_s_ctrl;

static struct msm_sensor_power_setting t4k28_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
	},
	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_camera_i2c_reg_conf t4k28_uxga_settings[] = {
	//FULL SIZE 1600x1200
	{0x3012,0x03},//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD
	{0x3015,0x04},//-/-/-/H_COUNT[12:8]  //
	{0x3016,0x04},//H_COUNT[7:0]
	{0x3017,0x03},//-/-/-/V_COUNT[12:8]
	{0x3018,0x00},//V_COUNT[7:0]
	{0x3019,0x00},//-/-/-/-/-/-/-/SCALE_M[8]
	{0x301A,0x10},//SCALE_M[7:0]
	{0x301B,0x00},//-/-/-/V_ANABIN/-/-/-/-
	{0x301C,0x01},//-/-/-/-/-/-/-/SCALING_MODE
	{0x3020,0x06},//-/-/-/-/-/HOUTPIX[10:8]
	{0x3021,0x40},//HOUTPIX[7:0]
	{0x3022,0x04},//-/-/-/-/-/VOUTPIX[10:8]
	{0x3023,0xB0},//VOUTPIX[7:0]
	{0x334E,0x00},//-/-/-/-/LSVCNT_MPY[11:8]
	{0x334F,0xA0},//LSVCNT_MPY[7:0]
	{0x3012,0x02},//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD
};

#ifndef FULLSIZE_OUTPUT
static struct msm_camera_i2c_reg_conf t4k28_svga_settings[] = {
   //SVGA 800X600
	{0x3012,0x03},//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD
	{0x3015,0x04},//-/-/-/H_COUNT[12:8]   //2012.12.26  //0x04
	{0x3016,0x04},//H_COUNT[7:0] //2012.12.26
	{0x3017,0x01},//-/-/-/V_COUNT[12:8]
	{0x3018,0x80},//V_COUNT[7:0]
	{0x3019,0x00},//-/-/-/-/-/-/-/SCALE_M[8]
	{0x301A,0x20},//SCALE_M[7:0]
	{0x301B,0x10},//-/-/-/V_ANABIN/-/-/-/-
	{0x301C,0x01},//-/-/-/-/-/-/-/SCALING_MODE
	{0x3020,0x03},//-/-/-/-/-/HOUTPIX[10:8]
	{0x3021,0x20},//HOUTPIX[7:0]
	{0x3022,0x02},//-/-/-/-/-/VOUTPIX[10:8]
	{0x3023,0x58},//VOUTPIX[7:0]
	{0x334E,0x01},//-/-/-/-/LSVCNT_MPY[11:8]
	{0x334F,0x40},//LSVCNT_MPY[7:0]
	{0x3012,0x02},//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD
};
#endif

static struct msm_camera_i2c_reg_conf t4k28_start_settings[] = {
};

static struct msm_camera_i2c_reg_conf t4k28_stop_settings[] = {
};

static struct msm_camera_i2c_reg_conf t4k28_recommend_settings[] = {

	{0x3000,0x08},
	{0x3001,0x40},
	{0x3002,0x00},
	{0x3003,0x00},
	{0x3004,0x00},
	{0x3005,0xB4},
	{0x3010,0x00},
	{0x3012,0x03},
	{0x3011,0x03},//0x00 yuxin modify for mirror&flip,2013.11.18
	{0x3014,0x03},
	{0x3015,0x04},
	{0x3016,0x04},
	{0x3017,0x03},
	{0x3018,0x00},
	{0x3019,0x00},
	{0x301A,0x10},
	{0x301B,0x00},
	{0x301C,0x01},
	{0x3020,0x06},
	{0x3021,0x40},
	{0x3022,0x04},
	{0x3023,0xB0},
	{0x3025,0x00},
	{0x3026,0x00},
	{0x3027,0x01},
	{0x302C,0x00},
	{0x302D,0x00},
	{0x302E,0x00},
	{0x302F,0x00},
	{0x3030,0x00},
	{0x3031,0x02},
	{0x3032,0x00},
	{0x3033,0x83},
	{0x3034,0x01},
	{0x3037,0x00},
	{0x303C,0x80},
	{0x303E,0x00},  //0x00
	{0x303F,0x00},
	{0x3040,0x80},
	{0x3044,0x02},
	{0x3045,0x04},
	{0x3046,0x00},
	{0x3047,0x80},
	{0x3048,0x04},
	{0x3049,0x01},
	{0x304A,0x04},
	{0x304B,0x0A},
	{0x304C,0x00},
	{0x304E,0x01},
	{0x3050,0x60},
	{0x3051,0x82},
	{0x3052,0x10},
	{0x3053,0x00},
	{0x3055,0x84},
	{0x3056,0x02},
	{0x3059,0x18},
	{0x305A,0x00},
	{0x3068,0xF0},
	{0x3069,0xF0},
	{0x306C,0x06},
	{0x306D,0x40},
	{0x306E,0x00},
	{0x306F,0x04},
	{0x3070,0x06},
	{0x3071,0x43},
	{0x3072,0x04},
	{0x3073,0xB0},
	{0x3074,0x00},
	{0x3075,0x04},
	{0x3076,0x04},
	{0x3077,0xB3},
	{0x307F,0x03},
	{0x3080,0x70},
	{0x3081,0x28},
	{0x3082,0x60},
	{0x3083,0x48},
	{0x3084,0x40},
	{0x3085,0x28},
	{0x3086,0xF8},
	{0x3087,0x38},
	{0x3088,0x03},
	{0x3089,0x02},
	{0x308A,0x58},
	{0x3091,0x00},
	{0x3092,0x10},
	{0x3093,0x6B},
	{0x3095,0x78},
	{0x3097,0x00},
	{0x3098,0x40},
	{0x309A,0x00},
	{0x309B,0x00},
	{0x309D,0x00},
	{0x309E,0x00},
	{0x309F,0x00},
	{0x30A0,0x02},//
	{0x30A1,0x00},//
	{0x30A2,0xA7},
	{0x30A3,0x20},
	{0x30A4,0xFF},
	{0x30A5,0x80},
	{0x30A6,0xFF},
	{0x30A7,0x00},
	{0x30A8,0x01},
	{0x30F1,0x00},
	{0x30F2,0x00},
	{0x30FE,0x80},
	{0x3100,0xD2},
	{0x3101,0xD3},
	{0x3102,0x45},
	{0x3103,0x80},
	{0x3104,0x31},
	{0x3105,0x02},
	{0x3106,0x23},
	{0x3107,0x20},
	{0x3108,0x7B},
	{0x3109,0x80},
	{0x310A,0x00},
	{0x310B,0x00},
	{0x3110,0x11},
	{0x3111,0x11},
	{0x3112,0x00},
	{0x3113,0x00},
	{0x3114,0x10},
	{0x3115,0x22},
	{0x3120,0x08},
	{0x3121,0x13},
	{0x3122,0x33},
	{0x3123,0x0E},
	{0x3124,0x26},
	{0x3125,0x00},
	{0x3126,0x0C},
	{0x3127,0x08},
	{0x3128,0x80},
	{0x3129,0x65},
	{0x312A,0x27},
	{0x312B,0x77},
	{0x312C,0x77},
	{0x312D,0x1A},
	{0x312E,0xB8},
	{0x312F,0x38},
	{0x3130,0x80},
	{0x3131,0x33},
	{0x3132,0x63},
	{0x3133,0x00},
	{0x3134,0xDD},
	{0x3135,0x07},
	{0x3136,0xB7},
	{0x3137,0x11},
	{0x3138,0x0B},
	{0x313B,0x0A},
	{0x313C,0x05},
	{0x313D,0x01},
	{0x313E,0x62},
	{0x313F,0x85},
	{0x3140,0x01},
	{0x3141,0x40},
	{0x3142,0x80},
	{0x3143,0x22},
	{0x3144,0x3E},
	{0x3145,0x32},
	{0x3146,0x2E},
	{0x3147,0x23},
	{0x3148,0x22},
	{0x3149,0x11},
	{0x314A,0x6B},
	{0x314B,0x30},
	{0x314C,0x69},
	{0x314D,0x80},
	{0x314E,0x31},
	{0x314F,0x32},
	{0x3150,0x32},
	{0x3151,0x03},
	{0x3152,0x0C},
	{0x3153,0xB3},
	{0x3154,0x20},
	{0x3155,0x13},
	{0x3156,0x66},
	{0x3157,0x02},
	{0x3158,0x03},
	{0x3159,0x01},
	{0x315A,0x16},
	{0x315B,0x10},
	{0x315C,0x00},
	{0x315D,0x44},
	{0x315E,0x1B},
	{0x315F,0x52},
	{0x3160,0x00},
	{0x3161,0x03},
	{0x3162,0x00},
	{0x3163,0xFF},
	{0x3164,0x00},
	{0x3165,0x01},
	{0x3166,0x00},
	{0x3167,0xFF},
	{0x3168,0x01},
	{0x3169,0x00},
	{0x3180,0x00},
	{0x3181,0x20},
	{0x3182,0x40},
	{0x3183,0x96},
	{0x3184,0x40},
	{0x3185,0x8F},
	{0x3186,0x31},
	{0x3187,0x06},
	{0x3188,0x0C},
	{0x3189,0x44},
	{0x318A,0x42},
	{0x318B,0x0B},
	{0x318C,0x11},
	{0x318D,0xAA},
	{0x318E,0x40},
	{0x318F,0x30},
	{0x3190,0x03},
	{0x3191,0x01},
	{0x3192,0x00},
	{0x3193,0x00},
	{0x3194,0x00},
	{0x3195,0x00},
	{0x3196,0x00},
	{0x3197,0xDE},
	{0x3198,0x00},
	{0x3199,0x00},
	{0x319A,0x00},
	{0x319B,0x00},
	{0x319C,0x16},
	{0x319D,0x0A},
	{0x31A0,0xBF},
	{0x31A1,0xFF},
	{0x31A2,0x11},
	{0x31B0,0x00},
	{0x31B1,0x42},
	{0x31B2,0x09},
	{0x31B3,0x51},
	{0x31B4,0x02},
	{0x31B5,0xEA},
	{0x31B6,0x09},
	{0x31B7,0x42},
	{0x31B8,0x00},
	{0x31B9,0x03},
	{0x31BA,0x3F},
	{0x31BB,0xFF},
	{0x3300,0xFF},
	{0x3301,0x35},
	{0x3303,0x40},
	{0x3304,0x00},
	{0x3305,0x00},
	{0x3306,0x30},
	{0x3307,0x00},
	{0x3308,0x87},
	{0x330A,0x60},
	{0x330B,0x56},
	{0x330D,0x79},
	{0x330E,0xFF},
	{0x330F,0xFF},
	{0x3310,0xFF},
	{0x3311,0x7F},
	{0x3312,0x0F},
	{0x3313,0x0F},
	{0x3314,0x02},
	{0x3315,0xC0},
	{0x3316,0x18},
	{0x3317,0x08},
	{0x3318,0xb0},//0x60
	{0x3319,0xf0},//0x90
	{0x331B,0x00},
	{0x331C,0x00},
	{0x331D,0x00},
	{0x331E,0x00},
	{0x3322,0x23},
	{0x3323,0x23},
	{0x3324,0x05},
	{0x3325,0x50},
	{0x3327,0x00},
	{0x3328,0x00},
	{0x3329,0x80},
	{0x332A,0x80},
	{0x332B,0x80},
	{0x332C,0x80},
	{0x332D,0x80},
	{0x332E,0x80},
	{0x332F,0x08},
	{0x3330,0x06},
	{0x3331,0x10},
	{0x3332,0x00},
	{0x3333,0x09},
	{0x3334,0x10},
	{0x3335,0x00},
	{0x3336,0x08},
	{0x3337,0x0d},
	{0x3338,0x03},
	{0x3339,0x03},
	{0x333A,0x02},
	{0x333B,0x74},
	{0x333C,0xa0},
	{0x333D,0x53},
	{0x333E,0x6c},
	{0x333F,0x97},
	{0x3340,0x52},
	{0x3341,0x3c},
	{0x3342,0x5c},
	{0x3343,0x2c},
	{0x3344,0x42},
	{0x3345,0x50},
	{0x3346,0x28},
	{0x3347,0x00},
	{0x3348,0x00},
	{0x3349,0x00},
	{0x334A,0x00},
	{0x334B,0x00},
	{0x334C,0x00},
	{0x334D,0x40},
	{0x334E,0x00},
	{0x334F,0xA0},
	{0x3350,0x03},
	{0x335F,0x00},
	{0x3360,0x00},
	{0x3400,0xA4},
	{0x3401,0x7F},
	{0x3402,0x00},
	{0x3403,0x00},
	{0x3404,0x3A},
	{0x3405,0xE3},
	{0x3406,0x22},
	{0x3407,0x25},
	{0x3408,0x17},
	{0x3409,0x5C},
	{0x340A,0x20},
	{0x340B,0x20},
	{0x340C,0x3B},
	{0x340D,0x2E},
	{0x340E,0x26},
	{0x340F,0x3F},
	{0x3410,0x34},
	{0x3411,0x2D},
	{0x3412,0x28},
	{0x3413,0x47},
	{0x3414,0x3E},
	{0x3415,0x6A},
	{0x3416,0x5A},
	{0x3417,0x50},
	{0x3418,0x48},
	{0x3419,0x42},
	{0x341B,0x10},
	{0x341C,0x40},
	{0x341D,0x70},
	{0x341E,0xc4},
	{0x341F,0x88},
	{0x3420,0x80},
	{0x3421,0xc4},
	{0x3422,0x00},
	{0x3423,0x0F},
	{0x3424,0x0F},
	{0x3425,0x0F},
	{0x3426,0x0F},
	{0x342B,0x10},
	{0x342C,0x20},
	{0x342D,0x80},
	{0x342E,0x50},
	{0x342F,0x60},
	{0x3430,0x30},
	{0x3431,0x1E},
	{0x3432,0x1E},
	{0x3433,0x0A},
	{0x3434,0x0A},
	{0x3435,0x15},
	{0x3436,0x15},
	{0x343F,0x10},
	{0x3440,0xF0},
	{0x3441,0x86},
	{0x3442,0xb0},
	{0x3443,0x60},
	{0x3444,0x08},
	{0x3446,0x03},
	{0x3447,0x00},
	{0x3448,0x00},
	{0x3449,0x00},
	{0x344A,0x80},//0x00,0xf0
	{0x344B,0x00},
	{0x344C,0x20},
	{0x344D,0xFF},
	{0x344E,0x0F},
	{0x344F,0x20},
	{0x3450,0x80},
	{0x3451,0x0F},
	{0x3452,0x55},
	{0x3453,0x49},
	{0x3454,0x6A},
	{0x3455,0x93},
	{0x345C,0x00},
	{0x345D,0x00},
	{0x345E,0x00},
	{0x3500,0xC1},
	{0x3501,0x01},
	{0x3502,0x40},
	{0x3503,0x1A},
	{0x3504,0x00},
	{0x3505,0x9C},
	{0x3506,0x04},
	{0x3507,0xD0},
	{0x3508,0x00},
	{0x3509,0xBD},
	{0x350A,0x00},
	{0x350B,0x20},
	{0x350C,0x00},
	{0x350D,0x15},
	{0x350E,0x15},
	{0x350F,0x51},
	{0x3510,0x50},
	{0x3511,0x90},
	{0x3512,0x10},
	{0x3513,0x00},
	{0x3514,0x00},
	{0x3515,0x10},
	{0x3516,0x10},
	{0x3517,0x00},
	{0x3518,0x00},
	{0x3519,0xFF},
	{0x351A,0xC0},
	{0x351B,0x08},
	{0x351C,0x69},
	{0x351D,0xba},
	{0x351E,0x16},
	{0x351F,0x80},
	{0x3520,0x26},
	{0x3521,0x02},
	{0x3522,0x08},
	{0x3523,0x0C},
	{0x3524,0x01},
	{0x3525,0x5A},
	{0x3526,0x3C},
	{0x3527,0xE0},
	{0x3528,0xbd},
	{0x3529,0x72},
	{0x352A,0xDE},
	{0x352B,0x22},
	{0x352C,0xD0},
	{0x352D,0x1e},
	{0x352E,0x21},
	{0x352F,0xEB},
	{0x3530,0x20},
	{0x3531,0x2A},
	{0x3532,0x34},
	{0x3533,0x13},
	{0x3534,0x21},
	{0x3535,0xE5},
	{0x3536,0x20},
	{0x3537,0x2A},
	{0x3538,0x34},
	{0x3539,0x20},
	{0x353A,0x05},
	{0x353B,0x19},
	{0x353C,0xEB},
	{0x353D,0x16},
	{0x353E,0xE2},
	{0x353F,0x18},
	{0x3540,0x40},
	{0x3541,0x18},
	{0x3542,0x42},
	{0x3543,0x21},
	{0x3544,0xE5},
	{0x3545,0x28},
	{0x3546,0xE8},
	{0x3547,0x0C},
	{0x3548,0x84},
	{0x3549,0x00},
	{0x354A,0x00},
	{0x354B,0x00},
	{0x354C,0x00},
	{0x354D,0x00},
	{0x354E,0x00},
	{0x354F,0x00},
	{0x3550,0x00},
	{0x3551,0x03},
	{0x3552,0x28},
	{0x3553,0x20},
	{0x3554,0x60},
	{0x3555,0xF0},
	{0x355D,0x80},
	{0x355E,0x27},
	{0x355F,0x0F},
	{0x3560,0x90},
	{0x3561,0x01},
	{0x3562,0x00},
	{0x3563,0x00},
	{0x3564,0x00},
	{0x3565,0x08},
	{0x3566,0x50},
	{0x3567,0x5F},
	{0x3568,0x00},
	{0x3569,0x00},
	{0x356A,0x00},
	{0x356B,0x00},
	{0x356C,0x16},
	{0x356D,0xEE},
	{0x356E,0x01},
	{0x356F,0xAA},
	{0x3570,0x01},
	{0x3571,0x00},
	{0x3572,0x01},
	{0x3573,0x41},
	{0x3574,0x01},
	{0x3575,0x00},
	{0x3576,0x02},
	{0x3577,0x66},
	{0x3578,0xBA},
	{0x3579,0x0C},
	{0x357A,0x88},
	{0x357B,0xFF},
	{0x357C,0xE0},
	{0x357D,0x00},
	{0x3900,0x00},
	{0x3901,0x07},
	{0x3902,0x00},
	{0x3903,0x00},
	{0x3904,0x00},
	{0x3905,0x00},
	{0x3906,0x00},
	{0x3907,0x00},
	{0x3908,0x00},
	{0x3909,0x00},
	{0x390A,0x00},
	{0x390B,0x00},
	{0x390C,0x00},
	{0x30F0,0x00},
   #ifdef FULLSIZE_OUTPUT
	{0x3047,0x80},//PLL_MULTI[7:0]
	{0x3048,0x04},//FLLONGON/FRMSPD[1:0]/FL600S[12:8]
	{0x304a,0x04},//FL600S[7:0]
	{0x351C,0x69},//FLLONGON/FRMSPD[1:0]/FL600S[12:8]
	{0x351D,0xba},//FL600S[7:0]
	{0x351B,0x08},//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]	/*还可以尝试一下0x98*/
   #else//BINNING MODE(800X600)
	{0x3047,0x68},//PLL_MULTI[7:0]   //0x58
	{0x3048,0x04},//FLLONGON/FRMSPD[1:0]/FL600S[12:8]
	{0x304a,0x04},//FL600S[7:0]
	{0x351C,0x66},//FLLONGON/FRMSPD[1:0]/FL600S[12:8]
	{0x351D,0xb0},//FL600S[7:0]
	{0x351B,0xa8},//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]	/*还可以尝试一下0xa8*/ //0x98
   #endif
	{0x3012,0x02},//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD
	{0x3010,0x01},//-/-/-/-/-/-/-/MODSEL ;
};

static struct v4l2_subdev_info t4k28_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt	= 1,
		.order	= 0,
	},
};



static struct msm_camera_i2c_reg_conf t4k28_reg_saturation[5][2] = {
	{//level 0
		{0x341E,0x80},
		{0x3421,0x80},
	},
	{//level 1
             {0x341E,0xa0},
		{0x3421,0xa0},
	},
	{//level 2 -default level
		{0x341E,0xc4},
		{0x3421,0xc4},
	},
	{//level 3
		{0x341E,0xe0},
		{0x3421,0xe0},
	},
	{//level 4
		{0x341E,0xff},
		{0x3421,0xff},
	},
	
};

static struct msm_camera_i2c_reg_conf t4k28_reg_contrast[5][1] = {
	{
		//Contrast -2
		{0x3441,0x49},

	},
	{
		//Contrast -1
		{0x3441,0x59},

	},
	{
		//Contrast (Default)
		{0x3441,0x86},
	},
	{
		//Contrast +1
		{0x3441,0x99},
	},
	{
		//Contrast +2
		{0x3441,0xa9},
	},
	
};

static struct msm_camera_i2c_reg_conf t4k28_reg_sharpness[5][5] = {
	{
		//Sharpness 0
		{0x342B,0x20},
		{0x342D,0x60},
		{0x342e,0x30},
		{0x342f,0x40},
		{0x3430,0x10},
	}, /* SHARPNESS LEVEL 0*/
	{
		//Sharpness 1
		{0x342B,0x18},
		{0x342D,0x70},
		{0x342e,0x40},
		{0x342f,0x50},
		{0x3430,0x20},
	}, /* SHARPNESS LEVEL 1*/
	{
		//Sharpness_Auto (Default)
		{0x342B,0x10},
		{0x342D,0x80},
		{0x342e,0x50},
		{0x342f,0x60},
		{0x3430,0x30},
	}, /* SHARPNESS LEVEL 2*/
	{
		//Sharpness 3
		{0x342B,0x08},
		{0x342D,0x90},
		{0x342e,0x60},
		{0x342f,0x70},
		{0x3430,0x40},
	}, /* SHARPNESS LEVEL 3*/
	{
		//Sharpness 4
		{0x342B,0x00},
		{0x342D,0xa0},
		{0x342e,0x70},
		{0x342f,0x80},
		{0x3430,0x50},
	}, /* SHARPNESS LEVEL 4*/
	
};

static struct msm_camera_i2c_reg_conf t4k28_reg_iso[7][2] = {
	{//auto
		{0x3504,0x00},//-/-/-/-/AGMAX[11:8]
		{0x3505,0x9C},//AGMAX[7:0],
       },
	{ //MSM_V4L2_ISO_DEBLUR ,not used
		/*不支持，参数设置为auto*/
		{0x3504,0x00},//-/-/-/-/AGMAX[11:8]
		{0x3505,0x9C},//AGMAX[7:0]
	},
	{//iso_100
		{0x3504,0x00},//-/-/-/-/AGMAX[11:8]
		{0x3505,0x1A},//AGMAX[7:0]
	},
	{//iso200
		{0x3504,0x00},//-/-/-/-/AGMAX[11:8]
		{0x3505,0x34},//AGMAX[7:0]
	},
	{ //iso400
		{0x3504,0x00},//-/-/-/-/AGMAX[11:8]
		{0x3505,0x68},//AGMAX[7:0]
	},
	{ //iso800
		{0x3504,0x00},//-/-/-/-/AGMAX[11:8]
		{0x3505,0xD0},//AGMAX[7:0]
	},
	{ //iso1600
		/*不支持，参数设置为ISO800*/
		{0x3504,0x00},//-/-/-/-/AGMAX[11:8]
		{0x3505,0xD0},//AGMAX[7:0]
       },
};

static struct msm_camera_i2c_reg_conf t4k28_reg_exposure_compensation[5][2] = {
	{
		//-1.7EV
		{0x3501,0x01},//-/-/-/-/-/-/ALCAIM[9:8]
		{0x3502,0x00},//ALCAIM[7:0]
	},
	{
		//-1.0EV
		{0x3501,0x01},//-/-/-/-/-/-/ALCAIM[9:8]
		{0x3502,0x20},//ALCAIM[7:0]
	},
	{
		//default
		{0x3501,0x01},//-/-/-/-/-/-/ALCAIM[9:8]
		{0x3502,0x40},//ALCAIM[7:0]
	},
	{
		//+1.0EV
		{0x3501,0x01},//-/-/-/-/-/-/ALCAIM[9:8]
		{0x3502,0x60},//ALCAIM[7:0]
	},
	{
		//+1.7EV
		{0x3501,0x01},//-/-/-/-/-/-/ALCAIM[9:8]
		{0x3502,0x80},//ALCAIM[7:0]
	},
};

static struct msm_camera_i2c_reg_conf t4k28_reg_antibanding[4][1] = {
	{//off.not used
            {0x351E, 0x16},
       },
	{//50Hz
            {0x351E, 0x16},
	   },
	{//60Hz
            {0x351E, 0x56},
       },
	{ //auto
	     {0x351E,0x86},//ACFDET/AC60M/FLMANU/ACDETDLY/MSKLINE[1:0]/ACDPWAIT[1:0]
	},
};

static struct msm_camera_i2c_reg_conf t4k28_reg_effect_normal[] = {
	/* normal: */
	{0x3402,0x00},
};

static struct msm_camera_i2c_reg_conf t4k28_reg_effect_black_white[] = {
	/* B&W: */
	{0x3402,0x06},
};

static struct msm_camera_i2c_reg_conf t4k28_reg_effect_negative[] = {
	/* Negative: */
	{0x3402,0x01},
};

static struct msm_camera_i2c_reg_conf t4k28_reg_effect_old_movie[] = {
	/* Sepia(antique): */
	{0x3402,0x05},
	{0x3454,0x6A},//SEPIAOFSU[7:0]
	{0x3455,0x93},//SEPIAOFSV[7:0]

};

//static struct msm_camera_i2c_reg_conf t4k28_reg_effect_solarize[] = {
	
//};

static struct msm_camera_i2c_reg_conf t4k28_reg_scene_auto[] = {
	//SCENE_MODE_AUTO  自动
		{0x3501,0x01},//-/-/-/-/-/-/ALCAIM[9:8]
		{0x3502,0x50},//ALCAIM[7:0]
		{0x351B,0x08},//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]   
		{0x3441,0x86},//LCONT_LEV[7:0]
		{0x3422,0x00},//Cbr_MGAIN[7:0]
		{0x341E,0xc4}, //saturation setting
		{0x3421,0xc4},
		{0x3500,0xc1},//ALCSW/AWBSW/ALCLOCK/-/ESLIMMODE/ROOMDET/-/ALCLIMMODE
		{0x3322,0x23},//PWBGAINGR[7:0]
		{0x3323,0x23},//PWBGAINGB[7:0]
		{0x3324,0x05},//PWBGAINR[7:0]
		{0x3325,0x50},//PWBGAINB[7:0]
		{0x350D,0x15},//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0]
		{0x350E,0x15},//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0]
		{0x350F,0x51},//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0]
		{0x3510,0x50},//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-
		
	};

static struct msm_camera_i2c_reg_conf t4k28_reg_scene_portrait[] = {
	//SCENE_MODE_PORTRAIT - Take people pictures.人物
		{0x3501,0x01},//-/-/-/-/-/-/ALCAIM[9:8]
		{0x3502,0x50},//ALCAIM[7:0]
		{0x351B,0x98},//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]   
		{0x3441,0x89},//LCONT_LEV[7:0]
		//{0x3422,0xa0},//Cbr_MGAIN[7:0]
		{0x3422,0x00},//Cbr_MGAIN[7:0]
		{0x341E,0xa0}, //saturation setting
		{0x3421,0xa0},
		{0x3500,0xc1},//ALCSW/AWBSW/ALCLOCK/-/ESLIMMODE/ROOMDET/-/ALCLIMMODE
		{0x3322,0x23},//PWBGAINGR[7:0]
		{0x3323,0x23},//PWBGAINGB[7:0]
		{0x3324,0x05},//PWBGAINR[7:0]
		{0x3325,0x50},//PWBGAINB[7:0]
		{0x350D,0x19},//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0]
		{0x350E,0x0A},//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0]
		{0x350F,0x81},//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0]
		{0x3510,0x90},//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-
       };

static struct msm_camera_i2c_reg_conf t4k28_reg_scene_landscape[] = {   
	//SCENE_MODE_LANDSCAPE，风景
		{0x3501,0x01},//-/-/-/-/-/-/ALCAIM[9:8]
		{0x3502,0x50},//ALCAIM[7:0]
		{0x351B,0x98},//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]   
		{0x3441,0x8c},//LCONT_LEV[7:0]
		//{0x3422,0xc8},//Cbr_MGAIN[7:0]
		{0x3422,0x00},//Cbr_MGAIN[7:0]
		{0x341E,0xc8}, //saturation setting
		{0x3421,0xc8},
		{0x3500,0xc1},//ALCSW/AWBSW/ALCLOCK/-/ESLIMMODE/ROOMDET/-/ALCLIMMODE
		{0x3322,0x23},//PWBGAINGR[7:0]
		{0x3323,0x23},//PWBGAINGB[7:0]
		{0x3324,0x05},//PWBGAINR[7:0]
		{0x3325,0x50},//PWBGAINB[7:0]
		{0x350D,0x55},//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0]
		{0x350E,0x55},//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0]
		{0x350F,0x55},//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0]
		{0x3510,0x54},//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-
      };

static struct msm_camera_i2c_reg_conf t4k28_reg_scene_night[] = {
	//SCENE_MODE_NIGHT    夜景
		{0x3501,0x01},//-/-/-/-/-/-/ALCAIM[9:8]
		{0x3502,0x50},//ALCAIM[7:0]
		{0x351B,0xB8},//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]
		{0x3441,0x89},//LCONT_LEV[7:0]
		//{0x3422,0xc0},//Cbr_MGAIN[7:0]
		{0x3422,0x00},//Cbr_MGAIN[7:0]
		{0x341E,0xc0}, //saturation setting
		{0x3421,0xc0},
		{0x3500,0xc1},//ALCSW/AWBSW/ALCLOCK/-/ESLIMMODE/ROOMDET/-/ALCLIMMODE
		{0x3322,0x23},//PWBGAINGR[7:0]
		{0x3323,0x23},//PWBGAINGB[7:0]
		{0x3324,0x05},//PWBGAINR[7:0]
		{0x3325,0x50},//PWBGAINB[7:0]
		{0x350D,0x55},//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0]
		{0x350E,0x55},//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0]
		{0x350F,0x95},//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0]
		{0x3510,0x54},//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-
      	};

static struct msm_camera_i2c_reg_conf t4k28_reg_wb_auto[] = {
	/* Auto: */
	//CAMERA_WB_AUTO                //1
	{0x3500,0xC1},//ALCSW/AWBSW/ALCLOCK/-/ESLIMMODE/ROOMDET/-/ALCLIMMODE
	{0x3322,0x23},//PWBGAINGR[7:0]
	{0x3323,0x23},//PWBGAINGB[7:0]
	{0x3324,0x05},//PWBGAINR[7:0]
	{0x3325,0x50},//PWBGAINB[7:0]
};

static struct msm_camera_i2c_reg_conf t4k28_reg_wb_sunny[] = {
	/* Sunny: */
	{0x3500,0x81},//ALCSW/AWBSW/ALCLOCK/-
	{0x3322,0x20},//PWBGAINGR[7:0]
	{0x3323,0x20},//PWBGAINGB[7:0]
	{0x3324,0x10},//PWBGAINR[7:0]
	{0x3325,0x40},//PWBGAINB[7:0]
};

static struct msm_camera_i2c_reg_conf t4k28_reg_wb_cloudy[] = {
	/* Cloudy: */
	{0x3500,0x81},//ALCSW/AWBSW/ALCLOCK/-
	{0x3322,0x30},//PWBGAINGR[7:0]
	{0x3323,0x30},//PWBGAINGB[7:0]
	{0x3324,0x30},//PWBGAINR[7:0]
	{0x3325,0x50},//PWBGAINB[7:0]
};

static struct msm_camera_i2c_reg_conf t4k28_reg_wb_office[] = {
	/* Office: */
	{0x3500,0x81},//ALCSW/AWBSW/ALCLOCK/-
	{0x3322,0x50},//PWBGAINGR[7:0]
	{0x3323,0x50},//PWBGAINGB[7:0]
	{0x3324,0x10},//PWBGAINR[7:0]
	{0x3325,0xc0},//PWBGAINB[7:0]
};

static struct msm_camera_i2c_reg_conf t4k28_reg_wb_home[] = {
	/* Home: */
	{0x3500,0x81},//ALCSW/AWBSW/ALCLOCK/-
	{0x3322,0x50},//PWBGAINGR[7:0]
	{0x3323,0x50},//PWBGAINGB[7:0]
	{0x3324,0x00},//PWBGAINR[7:0]
	{0x3325,0xf0},//PWBGAINB[7:0]
};


static const struct i2c_device_id t4k28_i2c_id[] = {
	{T4K28_SENSOR_NAME, (kernel_ulong_t)&t4k28_s_ctrl},
	{ }
};

static int32_t msm_t4k28_i2c_probe(struct i2c_client *client,
	   const struct i2c_device_id *id)
{
	   return msm_sensor_i2c_probe(client, id, &t4k28_s_ctrl);
}

static struct i2c_driver t4k28_i2c_driver = {
	.id_table = t4k28_i2c_id,
	.probe  = msm_t4k28_i2c_probe,
	.driver = {
		.name = T4K28_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client t4k28_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id t4k28_dt_match[] = {
	{.compatible = "qcom,t4k28", .data = &t4k28_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, t4k28_dt_match);

static struct platform_driver t4k28_platform_driver = {
	.driver = {
		.name = "qcom,t4k28",
		.owner = THIS_MODULE,
		.of_match_table = t4k28_dt_match,
	},
};

static void t4k28_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_i2c_reg_conf *table,
		int num)
{
	int i = 0;
	int rc = 0;
	for (i = 0; i < num; ++i) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
			s_ctrl->sensor_i2c_client, table->reg_addr,
			table->reg_data,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			msleep(100);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		table++;
	}
}


static int32_t t4k28_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(t4k28_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init t4k28_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&t4k28_platform_driver,
		t4k28_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&t4k28_i2c_driver);
}

static void __exit t4k28_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (t4k28_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&t4k28_s_ctrl);
		platform_driver_unregister(&t4k28_platform_driver);
	} else
		i2c_del_driver(&t4k28_i2c_driver);
	return;
}

static int32_t t4k28_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		printk("%s: %s: t4k28 read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	printk("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		printk("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

static void t4k28_set_stauration(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	t4k28_i2c_write_table(s_ctrl, &t4k28_reg_saturation[value][0],
		ARRAY_SIZE(t4k28_reg_saturation[value]));
}

static void t4k28_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	t4k28_i2c_write_table(s_ctrl, &t4k28_reg_contrast[value][0],
		ARRAY_SIZE(t4k28_reg_contrast[value]));
}

static void t4k28_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	pr_debug("%s %d", __func__, value);
	t4k28_i2c_write_table(s_ctrl, &t4k28_reg_sharpness[val][0],
		ARRAY_SIZE(t4k28_reg_sharpness[val]));
}


static void t4k28_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	t4k28_i2c_write_table(s_ctrl, &t4k28_reg_iso[value][0],
		ARRAY_SIZE(t4k28_reg_iso[value]));
}

static void t4k28_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = (value + 12) / 6;
	pr_debug("%s %d", __func__, val);
	t4k28_i2c_write_table(s_ctrl, &t4k28_reg_exposure_compensation[val][0],
		ARRAY_SIZE(t4k28_reg_exposure_compensation[val]));
}

static void t4k28_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_effect_normal[0],
			ARRAY_SIZE(t4k28_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_effect_black_white[0],
			ARRAY_SIZE(t4k28_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_effect_negative[0],
			ARRAY_SIZE(t4k28_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_effect_old_movie[0],
			ARRAY_SIZE(t4k28_reg_effect_old_movie));
		break;
	}
	default:
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_effect_normal[0],
			ARRAY_SIZE(t4k28_reg_effect_normal));
	}
}

static void t4k28_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	t4k28_i2c_write_table(s_ctrl, &t4k28_reg_antibanding[value][0],
		ARRAY_SIZE(t4k28_reg_antibanding[value]));
}

static void t4k28_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_scene_auto[0],
			ARRAY_SIZE(t4k28_reg_scene_auto));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_scene_night[0],
			ARRAY_SIZE(t4k28_reg_scene_night));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_scene_landscape[0],
			ARRAY_SIZE(t4k28_reg_scene_landscape));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_scene_portrait[0],
			ARRAY_SIZE(t4k28_reg_scene_portrait));
		break;
	}
	default:
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_scene_auto[0],
			ARRAY_SIZE(t4k28_reg_scene_auto));
	}
}

static void t4k28_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	pr_debug("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_wb_auto[0],
			ARRAY_SIZE(t4k28_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_wb_home[0],
			ARRAY_SIZE(t4k28_reg_wb_home));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_wb_sunny[0],
			ARRAY_SIZE(t4k28_reg_wb_sunny));
					break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_wb_office[0],
			ARRAY_SIZE(t4k28_reg_wb_office));
					break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_wb_cloudy[0],
			ARRAY_SIZE(t4k28_reg_wb_cloudy));
					break;
	}
	default:
		t4k28_i2c_write_table(s_ctrl, &t4k28_reg_wb_auto[0],
		ARRAY_SIZE(t4k28_reg_wb_auto));
	}
}

#ifndef FULLSIZE_OUTPUT

void T4K28YUV_write_shutter_and_gain(struct msm_sensor_ctrl_t *s_ctrl,
                                         uint16_t shutter, uint16_t again, uint16_t dgain)
{

	s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 
		                             0x3506, ((shutter >> 8) & 0xFF),MSM_CAMERA_I2C_BYTE_DATA);//MES[15:8]
       s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 
		                             0x3507, (shutter & 0xFF),MSM_CAMERA_I2C_BYTE_DATA);//MES[7:0]
       s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 
		                             0x350a, ((again >> 8) & 0xFF),MSM_CAMERA_I2C_BYTE_DATA);//ESLIMMODE/ROOMDET/-/-/MAG[11:8]
	s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 
		                             0x350b, (again & 0xFF),MSM_CAMERA_I2C_BYTE_DATA);//MAG[7:0]
       s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 
		                             0x350c, (dgain >> 2),MSM_CAMERA_I2C_BYTE_DATA);//MDG[7:0]
	return;
}   /* write_T4K28_shutter */

/*************************************************************************
* FUNCTION
*    T4K28YUV_set_AE_status
*
* DESCRIPTION
*    AE enable, manual AE or lock AE
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void T4K28YUV_set_AE_status(struct msm_sensor_ctrl_t *s_ctrl,
                            uint16_t AE_status)
{
    uint16_t temp_AE_reg = 0;

    if(AE_status == AE_enable) {
        //turn on AEC/AGC
       s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_read(s_ctrl->sensor_i2c_client, 0x3500, &temp_AE_reg,MSM_CAMERA_I2C_BYTE_DATA);

	s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 0x3500, (temp_AE_reg | 0x80),MSM_CAMERA_I2C_BYTE_DATA);
	
    } else if(AE_status == AE_lock) {
        //Lock AEC/AGC
       s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_read(s_ctrl->sensor_i2c_client, 0x3500, &temp_AE_reg,MSM_CAMERA_I2C_BYTE_DATA);

	s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 0x3500, (temp_AE_reg | 0x20),MSM_CAMERA_I2C_BYTE_DATA);
    } else {
	//turn off AEC/AGC
       s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_read(s_ctrl->sensor_i2c_client, 0x3500, &temp_AE_reg,MSM_CAMERA_I2C_BYTE_DATA);

	s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 0x3500, (temp_AE_reg & ~0xa0),MSM_CAMERA_I2C_BYTE_DATA);

    }
}
#endif

int32_t t4k28_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
		break;
	case CFG_SET_INIT_SETTING:
		CDBG("init setting");
		t4k28_i2c_write_table(s_ctrl,
				&t4k28_recommend_settings[0],
				ARRAY_SIZE(t4k28_recommend_settings));
		CDBG("init setting X");
		break;
	case CFG_SET_RESOLUTION: {
		int val = 0;
		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
	#ifndef FULLSIZE_OUTPUT

		switch(val)
		{
		    case 0://snapshot
		    	{
			       uint16_t MESH = 0;
				uint16_t MESL = 0;
				uint16_t MES = 0;
				uint16_t MAGH = 0;
				uint16_t MAGL = 0;
				uint16_t MAG = 0;
				uint16_t MDG = 0;
				uint16_t MDGH = 0;
				uint16_t MDGL = 0;
				uint16_t AG_1X = 0x1a;
				//lock AEC
				T4K28YUV_set_AE_status(s_ctrl, AE_lock);
				//read shutter and gain
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			              i2c_read(s_ctrl->sensor_i2c_client,
							0x355f, &MESH,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			              i2c_read(s_ctrl->sensor_i2c_client,
							0x3560, &MESL,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			              i2c_read(s_ctrl->sensor_i2c_client,
							0x3561, &MAGH,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			              i2c_read(s_ctrl->sensor_i2c_client,
							0x3562, &MAGL,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			              i2c_read(s_ctrl->sensor_i2c_client,
							0x3563, &MDGH,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			              i2c_read(s_ctrl->sensor_i2c_client,
							0x3564, &MDGL,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			              i2c_read(s_ctrl->sensor_i2c_client,
							0x3503, &AG_1X,MSM_CAMERA_I2C_BYTE_DATA);
				//calculate and record the preview shutter gain
				MES = (MESH << 8) | (MESL);
				MAG = (MAGH << 8) | (MAGL);
				MDG = ((MDGH & 0x03) << 8) | (MDGL & 0xFF);
				CDBG("shutter and gain ori : MES = %x, MAG = %x, MDG = %x\n", MES, MAG, MDG);
				set_pv_back_es = MES * MAG * 0x100 / AG_1X;
				set_pv_back_es = set_pv_back_es / 0x100;
				set_pv_back_ag = AG_1X;
				set_pv_back_dg = MDG;
				CDBG("shutter and gain calc : set_pv_back_es = %x, set_pv_back_ag = %x, set_pv_back_dg = %x\n", set_pv_back_es, set_pv_back_ag, set_pv_back_dg);
					
				t4k28_i2c_write_table(s_ctrl, &t4k28_uxga_settings[0],
				                        ARRAY_SIZE(t4k28_uxga_settings));
				//turn off AEC
				T4K28YUV_set_AE_status(s_ctrl, AE_disable);
				//set shutter and gain
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			                           i2c_write(s_ctrl->sensor_i2c_client,
							0x3506, MESH,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			                           i2c_write(s_ctrl->sensor_i2c_client,
							0x3507, MESL,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			                           i2c_write(s_ctrl->sensor_i2c_client,
							0x350a, MAGH,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			                           i2c_write(s_ctrl->sensor_i2c_client,
							0x350b, MAGL,MSM_CAMERA_I2C_BYTE_DATA);
				s_ctrl->sensor_i2c_client->i2c_func_tbl->
			                           i2c_write(s_ctrl->sensor_i2c_client,
							0x350c, (MDG>>2),MSM_CAMERA_I2C_BYTE_DATA);
                       break;
			}
			
		    case 1://preview
		      {
			  uint16_t AE_now_status;
			  s_ctrl->sensor_i2c_client->i2c_func_tbl->
			                  i2c_read(s_ctrl->sensor_i2c_client,
							0x3500, &AE_now_status,MSM_CAMERA_I2C_BYTE_DATA);
			  CDBG("read ae status : AE = %x,", AE_now_status);
			  //down preview window setting
			  t4k28_i2c_write_table(s_ctrl, &t4k28_svga_settings[0],
				ARRAY_SIZE(t4k28_svga_settings));
			  T4K28YUV_write_shutter_and_gain(s_ctrl, set_pv_back_es, 
			  	                      set_pv_back_ag, set_pv_back_dg);//es = 0x1200,ag = 0x1a,dg = 0x00
			  //turn on AEC
			  T4K28YUV_set_AE_status(s_ctrl, AE_enable);
				msleep(50);

                       break;
		      }
			
		}

	#else
		 t4k28_i2c_write_table(s_ctrl, &t4k28_uxga_settings[0],
				                        ARRAY_SIZE(t4k28_uxga_settings));
	#endif  //FULLSIZE_OUTPUT
		break;
	}
	case CFG_SET_STOP_STREAM:
		t4k28_i2c_write_table(s_ctrl,
			&t4k28_stop_settings[0],
			ARRAY_SIZE(t4k28_stop_settings));
		break;
	case CFG_SET_START_STREAM:
		t4k28_i2c_write_table(s_ctrl,
			&t4k28_start_settings[0],
			ARRAY_SIZE(t4k28_start_settings));
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
			(void *)
			sensor_slave_info.power_setting_array.power_setting,
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
			slave_index,
			power_setting_array->power_setting[slave_index].
			seq_type,
			power_setting_array->power_setting[slave_index].
			seq_val,
			power_setting_array->power_setting[slave_index].
			config_val,
			power_setting_array->power_setting[slave_index].
			delay);
		}
		kfree(power_setting_array->power_setting);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting, stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_SET_SATURATION: {
		int32_t sat_lev;
		if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);
		t4k28_set_stauration(s_ctrl, sat_lev);
		break;
	}
	case CFG_SET_CONTRAST: {
		int32_t con_lev;
		if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);
		t4k28_set_contrast(s_ctrl, con_lev);
		break;
	}
	case CFG_SET_SHARPNESS: {
		int32_t shp_lev;
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Sharpness Value is %d", __func__, shp_lev);
		t4k28_set_sharpness(s_ctrl, shp_lev);
		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: ISO Value is %d", __func__, iso_lev);
		t4k28_set_iso(s_ctrl, iso_lev);
		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
		t4k28_set_exposure_compensation(s_ctrl, ec_lev);
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Effect mode is %d", __func__, effect_mode);
		t4k28_set_effect(s_ctrl, effect_mode);
		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: anti-banding mode is %d", __func__,
			antibanding_mode);
		t4k28_set_antibanding(s_ctrl, antibanding_mode);
		break;
	}
	case CFG_SET_BESTSHOT_MODE: {
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: best shot mode is %d", __func__, bs_mode);
		t4k28_set_scene_mode(s_ctrl, bs_mode);
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: white balance is %d", __func__, wb_mode);
		t4k28_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t t4k28_sensor_func_tbl = {
	.sensor_config = t4k28_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = t4k28_sensor_match_id,
};

static struct msm_sensor_ctrl_t t4k28_s_ctrl = {
	.sensor_i2c_client = &t4k28_sensor_i2c_client,
	.power_setting_array.power_setting = t4k28_power_setting,
	.power_setting_array.size = ARRAY_SIZE(t4k28_power_setting),
	.msm_sensor_mutex = &t4k28_mut,
	.sensor_v4l2_subdev_info = t4k28_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(t4k28_subdev_info),
	.func_tbl = &t4k28_sensor_func_tbl,
};

module_init(t4k28_init_module);
module_exit(t4k28_exit_module);
MODULE_DESCRIPTION("t4k28 2MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
