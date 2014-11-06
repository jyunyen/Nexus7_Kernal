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


#define CX5036_NUM_CACHABLE_REGS	58
#define CX5036_ALS_GAIN_1   0x0
#define CX5036_ALS_GAIN_4   0x1
#define CX5036_ALS_GAIN_16  0x2
#define CX5036_ALS_GAIN_64  0x3


#define CX5036_SYS_MGS_ENABLE 0x8
#define CX5036_SYS_RST_ENABLE 0x4
#define CX5036_SYS_PS_ENABLE 0x2
#define CX5036_SYS_ALS_ENABLE 0x1
#define CX5036_SYS_DEV_DOWN 0x0



/* cx5036 control Register */
/*============================================================================*/
#define CX5036_REG_SYS_CON        0x00
#define CX5036_REG_SYS_CON_MASK	0x0F
#define CX5036_REG_SYS_CON_SHIFT	(0)
/* cx5036 interrupt flag */
/*============================================================================*/
#define CX5036_REG_SYS_INTSTATUS   0x01
#define CX5036_REG_SYS_INT_POR_MASK	0x80
#define CX5036_REG_SYS_INT_POR_SHIFT	(7)
#define CX5036_REG_SYS_INT_MGS_MASK	0x40
#define CX5036_REG_SYS_INT_MGS_SHIFT	(6)
#define CX5036_REG_SYS_INT_ERRF_MASK	0x20//read only
#define CX5036_REG_SYS_INT_OBJ_MASK	0x10//read only
#define CX5036_REG_SYS_INT_OBJ_SHIFT	(4)
#define CX5036_REG_SYS_INT_PS_MASK	0x02
#define CX5036_REG_SYS_INT_PS_SHIFT	(1)
#define CX5036_REG_SYS_INT_AL_MASK	0x01
#define CX5036_REG_SYS_INT_AL_SHIFT	(0)


/* cx5036 interrupt control */
/*============================================================================*/
#define CX5036_REG_SYS_INTCTRL     0x02
#define CX5036_REG_SYS_INTCTRL_PIEN_MASK     0x01
#define CX5036_REG_SYS_INTCTRL_PSPEND_MASK   0x02
#define CX5036_REG_SYS_INTCTRL_PSPEND_SHIFT   (5)
#define CX5036_REG_SYS_INTCTRL_PSMODE_MASK   0x03
#define CX5036_REG_SYS_INTCTRL_PSACC_MASK    0x04
#define CX5036_REG_SYS_INTCTRL_AIEN_MASK     0x05
#define CX5036_REG_SYS_INTCTRL_ALPEND_MASK   0x06
#define CX5036_REG_SYS_INTCTRL_MGSIEN_MASK   0x07

/* cx5036 waiting time Register */
#define CX5036_REG_WAITING_TIME     0x06
#define CX5036_REG_WAITING_TIME_WTIME_MASK 0x40     
#define CX5036_REG_WAITING_TIME_WTIME_SHIFT (0)
#define CX5036_REG_WAITING_TIME_WUNIT_MASK 0x7F

/* cx5036 ALS control Register */
#define CX5036_REG_ALS_CON    0x07
#define CX5036_REG_ALS_ALSRC_CON_MASK     0x30
#define CX5036_REG_ALS_ALSRC_CON_SHIFT    (4)
#define CX5036_REG_ALS_ALGAIN_CON_MASK    0x03
#define CX5036_REG_ALS_ALGAIN_CON_SHIFT   (0)

/* cx5036 ALS persistence Register */
#define CX5036_REG_ALS_PERS    0x08
#define CX5036_REG_ALS_PERS_MASK    0x3F


/* cx5036 ALS time Register */
#define CX5036_REG_ALS_TIME    0x0A
#define CX5036_REG_ALS_TIME_MASK    0xFF
#define CX5036_REG_ALS_TIME_SHIFT    (0)


/* cx5036 PS gain Register */
#define CX5036_REG_PS_GAIN    0x0C
#define CX5036_REG_PS_GAIN_MASK    0x01
#define CX5036_REG_PS_GAIN_SHIFT    (0)

/* cx5036 PS persistence Register */
#define CX5036_REG_PS_PERS    0x0D
#define CX5036_REG_PS_PERS_MASK    0x3F


/* cx5036 PS time Register */
#define CX5036_REG_PS_TIME    0x0F
#define CX5036_REG_PS_TIME_MASK    0x03
#define CX5036_REG_PS_TIME_SHIFT   (0)


/* cx5036 PS LED control Register */
#define CX5036_REG_PS_LED_CON    0x10
#define CX5036_REG_PS_LED_CON_PSLDR_MASK    0xC0
#define CX5036_REG_PS_LED_CON_PSLPUW_MASK    0x3F


/* cx5036 MGS control Register */
#define CX5036_REG_MGS_CON    0x11
#define CX5036_REG_MGS_GAIN_MASK    0xC0
#define CX5036_REG_MGS_GAIN_SHIFT    (0)


/* cx5036 MGS persistence Register */
#define CX5036_REG_MGS_PERS    0x12
#define CX5036_REG_MGS_PERS_MASK    0x3F
#define CX5036_REG_MGS_PERS_SHIFT   (0)


/* cx5036 MGS subsystem control Register */
#define CX5036_REG_MGS_SUBSYS_CON    0x13
#define CX5036_REG_MGS_SUBSYS_CON_AUTOPW_MASK      0x40
#define CX5036_REG_MGS_SUBSYS_CON_AUTOPW_SHIFT     (6)
#define CX5036_REG_MGS_SUBSYS_CON_AUTOCALB_MASK    0x04
#define CX5036_REG_MGS_SUBSYS_CON_AUTOCALB_SHIFT   (2)
#define CX5036_REG_MGS_SUBSYS_CON_MGSXYIMD_MASK    0x02
#define CX5036_REG_MGS_SUBSYS_CON_MGSXYIMD_SHIFT   (1)
#define CX5036_REG_MGS_SUBSYS_CON_MGSZIMD_MASK     0x01
#define CX5036_REG_MGS_SUBSYS_CON_MGSZIMD_SHIFT    (0)


/* cx5036 MGS XY Hold Register */
#define CX5036_REG_MGS_XY_HOLD    0x14
#define CX5036_REG_MGS_XY_HOLD_MASK      0xFF
#define CX5036_REG_MGS_XY_HOLD_SHIFT      (0)


/* cx5036 LED control Register */
#define CX5036_REG_LED_CON    0x15
#define CX5036_REG_MGSLDR_LED_CON_MASK      0xC0
#define CX5036_REG_MGSLDR_LED_CON_SHIFT     (6)
#define CX5036_REG_MGSLPUW_LED_CON_MASK      0x3F
#define CX5036_REG_MGSLPUW_LED_CON_SHIFT    (0)


/* cx5036 MGS Interrupt Status Register */
#define CX5036_REG_MGS_INT_STAT    0x1D
#define CX5036_REG_Z_MGS_INT_STAT_MASK      0x10
#define CX5036_REG_Y_MGS_INT_STAT_MASK      0x20
#define CX5036_REG_X_MGS_INT_STAT_MASK      0x40

/* cx5036 Error Flag Situation Register */
#define CX5036_REG_ERR    0x1E
#define CX5036_REG_ERRC_ERR_MASK      0x08
#define CX5036_REG_ERRB_ERR_MASK      0x04
#define CX5036_REG_ERRG_ERR_MASK      0x02
#define CX5036_REG_ERRR_ERR_MASK      0x01

/* cx5036 MGS Status Register */
#define CX5036_REG_MGS_STAT    0x1F
#define CX5036_REG_MGS_STAT_POW_MASK    0x40
#define CX5036_REG_MGS_STAT_Z_MASK      0x10
#define CX5036_REG_MGS_STAT_Y_MASK      0x0C
#define CX5036_REG_MGS_STAT_X_MASK      0x03


/* cx5036 MGS Data X Register */
#define CX5036_REG_MGS_DATA_X    0x20
#define CX5036_REG_MGS_DATA_X_MASK    0xFF


/* cx5036 MGS Data Y Register */
#define CX5036_REG_MGS_DATA_Y    0x21
#define CX5036_REG_MGS_DATA_Y_MASK    0xFF


/* cx5036 MGS Data Z Register */
#define CX5036_REG_MGS_DATA_Z    0x22
#define CX5036_REG_MGS_DATA_Z_MASK    0xFF




/* cx5036 PS Data High Register */
#define CX5036_REG_PS_DATA_LOW    0x26
#define CX5036_REG_PS_DATA_LOW_MASK    0xFF


/* cx5036 PS Data High Register */
#define CX5036_REG_PS_DATA_HIGH    0x27
#define CX5036_REG_PS_DATA_HIGH_MASK    0xFF


/* cx5036 Red Data Low Register */
#define CX5036_REG_RED_DATA_LOW    0x28
#define CX5036_REG_RED_DATA_LOW_MASK    0xFF


/* cx5036 Red Data High Register */
#define CX5036_REG_RED_DATA_HIGH    0x29
#define CX5036_REG_RED_DATA_HIGH_MASK    0xFF

/* cx5036 Green Data Low Register */
#define CX5036_REG_GREEN_DATA_LOW    0x2A
#define CX5036_REG_GREEN_DATA_LOW_MASK    0xFF


/* cx5036 Green Data High Register */
#define CX5036_REG_GREEN_DATA_HIGH    0x2B
#define CX5036_REG_GREEN_DATA_HIGH_MASK    0xFF

/* cx5036 Blue Data Low Register */
#define CX5036_REG_BLUE_DATA_LOW    0x2C
#define CX5036_REG_BLUE_DATA_LOW_MASK    0xFF

/* cx5036 Blue Data High Register */
#define CX5036_REG_BLUE_DATA_HIGH    0x2D
#define CX5036_REG_BLUE_DATA_HIGH_MASK    0xFF

/* cx5036 COMP Data Low Register */
#define CX5036_REG_COMP_DATA_LOW    0x2E
#define CX5036_REG_COMP_DATA_LOW_MASK    0xFF

/* cx5036 COMP Data High Register */
#define CX5036_REG_COMP_DATA_HIGH    0x2F
#define CX5036_REG_COMP_DATA_HIGH_MASK    0xFF


/* cx5036 L Data Low Register */
#define CX5036_REG_L_DATA_LOW    0x30
#define CX5036_REG_L_DATA_LOW_MASK    0xFF

/* cx5036 L Data High Register */
#define CX5036_REG_L_DATA_HIGH    0x31
#define CX5036_REG_L_DATA_HIGH_MASK    0xFF


/* cx5036 AL Low Threshold Low Register */
#define CX5036_REG_AL_ALTHL_LOW    0x32
#define CX5036_REG_AL_ALTHL_LOW_MASK    0xFF
#define CX5036_REG_AL_ALTHL_LOW_SHIFT    (0)

/* cx5036 AL Low Threshold High Register */
#define CX5036_REG_AL_ALTHL_HIGH    0x33
#define CX5036_REG_AL_ALTHL_HIGH_MASK    0xFF
#define CX5036_REG_AL_ALTHL_HIGH_SHIFT    (0)

/* cx5036 AL High Threshold Low Register */
#define CX5036_REG_AL_ALTHH_LOW    0x34
#define CX5036_REG_AL_ALTHH_LOW_MASK    0xFF
#define CX5036_REG_AL_ALTHH_LOW_SHIFT    (0)

/* cx5036 AL High Threshold High Register */
#define CX5036_REG_AL_ALTHH_HIGH    0x35
#define CX5036_REG_AL_ALTHH_HIGH_MASK    0xFF
#define CX5036_REG_AL_ALTHH_HIGH_SHIFT   (0)

/* cx5036 PS Low Threshold Low Register */
#define CX5036_REG_PS_PSTHL_LOW    0x36
#define CX5036_REG_PS_PSTHL_LOW_MASK    0xFF
#define CX5036_REG_PS_PSTHL_LOW_SHIFT    (0)

/* cx5036 PS Low Threshold High Register */
#define CX5036_REG_PS_PSTHL_HIGH    0x37
#define CX5036_REG_PS_PSTHL_HIGH_MASK    0xFF
#define CX5036_REG_PS_PSTHL_HIGH_SHIFT    (0)

/* cx5036 PS High Threshold Low Register */
#define CX5036_REG_PS_PSTHH_LOW    0x38
#define CX5036_REG_PS_PSTHH_LOW_MASK    0xFF
#define CX5036_REG_PS_PSTHH_LOW_SHIFT    (0)

/* cx5036 PS High Threshold High Register */
#define CX5036_REG_PS_PSTHH_HIGH    0x39
#define CX5036_REG_PS_PSTHH_HIGH_MASK    0xFF
#define CX5036_REG_PS_PSTHH_HIGH_SHIFT    (0)

/* cx5036 PS Calibrate L Register */
#define CX5036_REG_PS_CAL_L    0x3A
#define CX5036_REG_PS_CAL_L_MASK    0xFF
#define CX5036_REG_PS_CAL_L_SHIFT    (0)

/* cx5036 PS Calibrate H Register */
#define CX5036_REG_PS_CAL_H    0x3B
#define CX5036_REG_PS_CAL_H_MASK    0xFF
#define CX5036_REG_PS_CAL_H_SHIFT    (0)

/* cx5036 LuminanceCoef R Register */
#define CX5036_REG_LUM_COEFR    0x3C
#define CX5036_REG_LUM_COEFR_MASK    0xFF
#define CX5036_REG_LUM_COEFR_SHIFT    (0)

/* cx5036 LuminanceCoef G Register */
#define CX5036_REG_LUM_COEFG    0x3D
#define CX5036_REG_LUM_COEFG_MASK    0xFF
#define CX5036_REG_LUM_COEFG_SHIFT    (0)

/* cx5036 LuminanceCoef B Register */
#define CX5036_REG_LUM_COEFB    0x3E
#define CX5036_REG_LUM_COEFB_MASK    0xFF
#define CX5036_REG_LUM_COEFB_SHIFT    (0)

/* cx5036 MGS Z Calibration Register */
#define CX5036_REG_MGS_CAL_Z    0x3F
#define CX5036_REG_MGS_CAL_Z_MASK    0xFF
#define CX5036_REG_MGS_CAL_Z_SHIFT   (0)

/* cx5036 MGS Y Calibration Register */
#define CX5036_REG_MGS_CAL_Y    0x4E
#define CX5036_REG_MGS_CAL_Y_MASK    0xFF
#define CX5036_REG_MGS_CAL_Y_SHIFT    (0)

/* cx5036 MGS X Calibration Register */
#define CX5036_REG_MGS_CAL_X    0x4F
#define CX5036_REG_MGS_CAL_X_MASK    0xFF
#define CX5036_REG_MGS_CAL_X_SHIFT    (0)

/* cx5036 MGS X Low Threshold Register */
#define CX5036_REG_MGS_X_LTH    0x48
#define CX5036_REG_MGS_X_LTH_MASK    0xFF
#define CX5036_REG_MGS_X_LTH_SHIFT   (0)

/* cx5036 MGS X HIGH Threshold Register */
#define CX5036_REG_MGS_X_HTH    0x49
#define CX5036_REG_MGS_X_HTH_MASK    0xFF
#define CX5036_REG_MGS_X_HTH_SHIFT    (0)

/* cx5036 MGS Y Low Threshold Register */
#define CX5036_REG_MGS_Y_LTH    0x4A
#define CX5036_REG_MGS_Y_LTH_MASK    0xFF
#define CX5036_REG_MGS_Y_LTH_SHIFT    (0)

/* cx5036 MGS Y HIGH Threshold Register */
#define CX5036_REG_MGS_Y_HTH    0x4B
#define CX5036_REG_MGS_Y_HTH_MASK    0xFF
#define CX5036_REG_MGS_Y_HTH_SHIFT    (0)

/* cx5036 MGS Z Low Threshold Register */
#define CX5036_REG_MGS_Z_LTH    0x4C
#define CX5036_REG_MGS_Z_LTH_MASK    0xFF
#define CX5036_REG_MGS_Z_LTH_SHIFT    (0)

/* cx5036 MGS Y HIGH Threshold Register */
#define CX5036_REG_MGS_Z_HTH    0x4D
#define CX5036_REG_MGS_Z_HTH_MASK    0xFF
#define CX5036_REG_MGS_Z_HTH_SHIFT    (0)


#endif

