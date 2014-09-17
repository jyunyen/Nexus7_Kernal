/*
 * This file is part of the Dyna-Image CX5036 sensor driver for MTK platform.
 * CX5036 is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: cx5036.h
 *
 * Summary:
 *	CX5036 sensor dirver header file.
 *
 * Modification History:
 * Date     By       			Summary
 * -------- --------       --------------------------------------
 * 15/09/14 Templeton Tsai		 Original Creation
 * 
 */

/*
 * Definitions for cx5036 als/ps sensor chip.
 */
#ifndef __CX5036_H__
#define __CX5036_H__

#include <linux/ioctl.h>



#define CX5036_SUCCESS					0
#define CX5036_ERR_I2C					-1
#define CX5036_ERR_STATUS				-3
#define CX5036_ERR_SETUP_FAILURE			-4
#define CX5036_ERR_GETGSENSORDATA			-5
#define CX5036_ERR_IDENTIFICATION			-6


#define CX5036_NUM_CACHABLE_REGS	28

/* cx5036 control registers */
/*============================================================================*/
#define CX5036_REG_SYS_CON        0x00
#define CX5036_REG_SYS_CON_MEN_MASK	0x04
#define CX5036_REG_SYS_CON_SWRST_MASK	0x03
#define CX5036_REG_SYS_CON_PEN_MASK	0x02
#define CX5036_REG_SYS_CON_AEN_MASK	0x01

/* cx5036 interrupt flag */
/*============================================================================*/
#define CX5036_REG_SYS_INTSTATUS   0x01
#define CX5036_REG_SYS_INT_POR_MASK	0x40
#define CX5036_REG_SYS_INT_MGS_MASK	0x30
#define CX5036_REG_SYS_INT_ERRF_MASK	0x20//read only
#define CX5036_REG_SYS_INT_OBJ_MASK	0x10//read only
#define CX5036_REG_SYS_INT_PS_MASK	0x02
#define CX5036_REG_SYS_INT_AL_MASK	0x01


/* cx5036 interrupt control */
/*============================================================================*/
#define CX5036_REG_SYS_INTCTRL     0x02
#define CX5036_REG_SYS_INTCTRL_PIEN_MASK     0x01
#define CX5036_REG_SYS_INTCTRL_PSPEND_MASK   0x02
#define CX5036_REG_SYS_INTCTRL_PSMODE_MASK   0x03
#define CX5036_REG_SYS_INTCTRL_PSACC_MASK    0x04
#define CX5036_REG_SYS_INTCTRL_AIEN_MASK     0x05
#define CX5036_REG_SYS_INTCTRL_ALPEND_MASK   0x06
#define CX5036_REG_SYS_INTCTRL_MGSIEN_MASK   0x07

/* cx5036 waiting time registers */
#define CX5036_REG_WAITING_TIME     0x06
#define CX5036_REG_WAITING_TIME_WTIME_MASK 0x40     
#define CX5036_REG_WAITING_TIME_WUNIT_MASK 0x7F

/* cx5036 ALS control registers */
#define CX5036_REG_ALS_CON    0x07
#define CX5036_REG_ALS_ALSRC_CON    0x03
#define CX5036_REG_ALS_ALGAIN_CON    0x03

#define CX5036_REG_ALS_DATA_LOW    0x0C
#define CX5036_REG_ALS_DATA_HIGH   0x0D

#define CX5036_REG_PS_DATA_LOW     0x0E
#define CX5036_REG_PS_DATA_LOW_SHIFT     (0)
#define	AL3426_REG_PS_DATA_LOW_MASK	   0xFF
#define CX5036_REG_PS_DATA_HIGH    0x0F
#define CX5036_REG_PS_DATA_HIGH_SHIFT    (0)
#define	AL3426_REG_PS_DATA_HIGH_MASK	   0x03
/*----------------------------------------------------------------------------*/
#define CX5036_REG_ALS_CONF        0x10 /*ALS GAIN*/

#define CX5036_REG_ALS_PERSIS      0x14
//#define CX5036_REG_ALS_CAL         0x19

#define CX5036_REG_ALS_THDL_L      0x1A
#define CX5036_REG_ALS_THDL_L_SHIFT	(0)
#define CX5036_REG_ALS_THDL_L_MASK	0xFF

#define CX5036_REG_ALS_THDL_H      0x1B
#define CX5036_REG_ALS_THDL_H_SHIFT	(0)
#define CX5036_REG_ALS_THDL_H_MASK	0xFF

#define CX5036_REG_ALS_THDH_L      0x1C
#define CX5036_REG_ALS_THDH_L_SHIFT	(0)
#define CX5036_REG_ALS_THDH_L_MASK	0xFF

#define CX5036_REG_ALS_THDH_H      0x1D
#define CX5036_REG_ALS_THDH_H_SHIFT	(0)
#define CX5036_REG_ALS_THDH_H_MASK	0xFF


/*----------------------------------------------------------------------------*/
/* cx5036 PS CONFIG registers */
#define CX5036_REG_PS_CONF         0x20 /*PS GAIN*/
#define CX5036_REG_PS_CONF_SHIFT         (2) 
#define CX5036_REG_PS_CONF_MASK         0x0C 

#define CX5036_REG_PS_LEDD         0x21 /*PS LED DRIVER*/
#define CX5036_REG_PS_LEDD_SHIFT         (0) 
#define CX5036_REG_PS_LEDD_MASK         0x03

#define CX5036_REG_PS_IFORM        0x22 /* PS INT Mode*/

#define CX5036_REG_PS_MEAN         0x23
#define CX5036_REG_PS_MEAN_SHIFT         (0)
#define CX5036_REG_PS_MEAN_MASK         0x03

#define CX5036_REG_PS_SMARTINT     0x24 /* PS Smart INT for low power */
#define CX5036_REG_PS_INTEGR       0x25
#define CX5036_REG_PS_PERSIS       0x26
#define CX5036_REG_PS_PERSIS_SHIFT       (0)
#define CX5036_REG_PS_PERSIS_MASK       0x3F
#define CX5036_REG_PS_CAL_L        0x28
#define CX5036_REG_PS_CAL_H        0x29

#define CX5036_REG_PS_THDL_L       0x2A
#define CX5036_REG_PS_THDL_L_SHIFT	(0)
#define CX5036_REG_PS_THDL_L_MASK		0xFF

#define CX5036_REG_PS_THDL_H       0x2B
#define CX5036_REG_PS_THDL_H_SHIFT	(2)
#define CX5036_REG_PS_THDL_H_MASK		0x03

#define CX5036_REG_PS_THDH_L       0x2C
#define CX5036_REG_PS_THDH_L_SHIFT	(0)
#define CX5036_REG_PS_THDH_L_MASK		0xFF

#define CX5036_REG_PS_THDH_H       0x2D
#define CX5036_REG_PS_THDH_H_SHIFT	(2)
#define CX5036_REG_PS_THDH_H_MASK		0x03


/*============================================================================*/
//SYSTEM MODE (CX5036_REG_SYS_CONF)
#define	CX5036_SYS_DEV_DOWN        0x00
#define	CX5036_SYS_ALS_ENABLE      0x01
#define	CX5036_SYS_PS_ENABLE       0x02
#define	CX5036_SYS_ALS_PS_ENABLE   0x03
#define	CX5036_SYS_DEV_RESET       0x04
/*----------------------------------------------------------------------------*/
//INT FLAG BIT MASK
#define	CX5036_SYS_ALS_INT_TRI     0x01
#define	CX5036_SYS_PS_INT_TRI      0x02
#define	CX5036_SYS_PS_INT_OBJ      0x10
#define	CX5036_SYS_PS_INT_IROV     0x20
/*----------------------------------------------------------------------------*/
//INT CLEAN Mode
#define	CX5036_SYS_ICLEAN_AUTO     0x00
#define	CX5036_SYS_ICLEAN_MANUAL   0x01
/*----------------------------------------------------------------------------*/
//ALS CONFIG
#define CX5036_ALS_RANGE_0         0x00	/* Full range 32768 lux (0.5lux/count) */
#define CX5036_ALS_RANGE_1         0x01	/* Full range 8192 lux */
#define CX5036_ALS_RANGE_2         0x02	/* Full range 2048 lux */
#define CX5036_ALS_RANGE_3         0x03	/* Full range 512 lux */
#define CX5036_ALS_RANGE_MASK		0x30
#define CX5036_ALS_RANGE_SHIFT	(4)
#define CX5036_ALS_PERSIST_MASK	0x0F

/*----------------------------------------------------------------------------*/
//PS CONFIG
#define CX5036_PS_GAIN_1           0x00 /* PS resulation * 1 */
#define CX5036_PS_GAIN_2           0x01 /* PS resulation * 2 */
#define CX5036_PS_GAIN_4           0x02 /* PS resulation * 4 */
#define CX5036_PS_GAIN_8           0x03 /* PS resulation * 8 */
#define CX5036_PS_PERSIST_1            0x00
#define CX5036_PS_PERSIST_2            0x01
#define CX5036_PS_PERSIST_4            0x02
#define CX5036_PS_PERSIST_8            0x03
/*----------------------------------------------------------------------------*/
//PS LED Control
#define CX5036_PS_LED_P0        0x00	/* 0 puls */
#define CX5036_PS_LED_P1         0x01	/* 1 puls (default)*/
#define CX5036_PS_LED_P2         0x02	/* 2 puls  */
#define CX5036_PS_LED_P3         0x03	/* 3 puls  */
#define CX5036_PS_DEIVER_167         0x00	/* 16.7% */
#define CX5036_PS_DEIVER_333         0x01	/* 33.3% */
#define CX5036_PS_DEIVER_667         0x02	/* 66.7% */
#define CX5036_PS_DEIVER_1000         0x03	/* 100% (default)*/
/*----------------------------------------------------------------------------*/
//PS MEAN
#define CX5036_PS_MEAN_0         0x00	/* 5ms @2T*/
#define CX5036_PS_MEAN_1         0x01	/* 9.6ms @2T*/
#define CX5036_PS_MEAN_2         0x02	/* 14.1ms @2T*/
#define CX5036_PS_MEAN_3         0x03	/* 18.7ms @2T*/
/*----------------------------------------------------------------------------*/
#define DISABLE                     0x00
#define ENABLE                      0x01
/*============================================================================*/



/*----------------------------------------------------------------------------*/
//PS Engineering Registers
#define CX5036_REG_PS_DC_1         0x30 /*Only in Engineering chip, couldn't find in datasheet*/
#define CX5036_REG_PS_DC_1_SHIFT         (0) 
#define CX5036_REG_PS_DC_1_MASK         0xFF 
#define CX5036_REG_PS_DC_2         0x32 /*Only in Engineering chip, couldn't find in datasheet*/
#define CX5036_REG_PS_DC_2_SHIFT         (0) 
#define CX5036_REG_PS_DC_2_MASK         0xFF 

/*----------------------------------------------------------------------------*/



#endif

