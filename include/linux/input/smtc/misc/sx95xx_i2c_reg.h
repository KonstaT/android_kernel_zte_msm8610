/*
 * SX950X Cap Touch 
 * Currently Supports:
 *  SX9500
 *
 * Copyright 2011 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _SX950X_I2C_REG_H_
#define _SX950X_I2C_REG_H_

/*
 *  I2C Registers
 */
#define SX950X_IRQSTAT_REG    0x00
#define SX950X_CPSSTAT_REG    0x01
#define SX950X_NOP0_REG       0x03
#define SX950X_NCPS0_REG      0x06
#define SX950X_NCPS1_REG      0x07
#define SX950X_NCPS2_REG      0x08
#define SX950X_NCPS3_REG      0x09
#define SX950X_NCPS4_REG      0x0A
#define SX950X_NCPS5_REG      0x0B
#define SX950X_NCPS6_REG      0x0C
#define SX950X_NCPS7_REG      0x0D
#define SX950X_NCPS8_REG      0x0E
//#define SX950X_NCPS9_REG      0x0F
//#define SX950X_NCPS10_REG     0x10
#define SX950X_SOFTRESET_REG  0x7F

/*      IrqStat 0:Inactive 1:Active     */
#define SX950X_IRQSTAT_RESET_FLAG      0x80
#define SX950X_IRQSTAT_TOUCH_FLAG      0x40
#define SX950X_IRQSTAT_RELEASE_FLAG    0x20
#define SX950X_IRQSTAT_COMPDONE_FLAG   0x10
#define SX950X_IRQSTAT_CONV_FLAG       0x08
#define SX950X_IRQSTAT_TXENSTAT_FLAG   0x01


/* CpsStat  */
#define SX950X_CPSSTAT_TCHSTAT3_FLAG   0x80
#define SX950X_CPSSTAT_TCHSTAT2_FLAG   0x40
#define SX950X_CPSSTAT_TCHSTAT1_FLAG   0x20
#define SX950X_CPSSTAT_TCHSTAT0_FLAG   0x10



/*      SoftReset */
#define SX950X_SOFTRESET  0xDE

#endif /* _SX950X_I2C_REG_H_*/



