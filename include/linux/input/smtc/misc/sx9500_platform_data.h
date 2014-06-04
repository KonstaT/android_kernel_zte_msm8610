/*
 * include/linux/input/sx9500_platform_data.h
 *
 * SX9500 Platform Data
 * 2 cap differential  
 *
 * Copyright 2012 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _SX9500_PLATFORM_DATA_H_
#define _SX9500_PLATFORM_DATA_H_
#define DIFFERENTIAL


#include <linux/input/smtc/misc/smtc_sar_test_platform_data.h>

struct smtc_reg_data {
  unsigned char reg;
  unsigned char val;
};
typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;

struct sx9500_platform_data {
  int i2c_reg_num;
  struct smtc_reg_data *pi2c_reg;

  psmtc_sar_test_platform_data_t psar_platform_data;

  int (*get_is_nirq_low)(void);
  
  int     (*init_platform_hw)(void);
  void    (*exit_platform_hw)(void);
 
};
typedef struct sx9500_platform_data sx9500_platform_data_t;
typedef struct sx9500_platform_data *psx9500_platform_data_t;

#endif
