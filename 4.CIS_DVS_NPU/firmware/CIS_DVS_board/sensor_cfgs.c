/******************************************************************************
 * Copyright (C) 2017 - 2020 Xilinx, Inc.  All rights reserved.
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

/*****************************************************************************/
/**
 *
 * @file sensor_cfgs.c
 *
 * This file contains the IMX274 CSI2 Camera sensor configurations for
 *
 * IMX274
 * - 1280x720@60fps 	(quad lane)
 * - 1920x1080@30fps	(quad lane)
 * - 1920x1080@60fps	(quad lane)
 * - 3840x2160@30fps	(quad lane)
 * - 3840x2160@60fps	(quad lane)
 *
 * The structure names are sensor_<lane_count>L_<resolution>_<fps>fps_regs[]
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who    Date     Changes
 * ----- ------ -------- --------------------------------------------------
 * 1.00  pg    12/07/17 Initial release.
 * </pre>
 *
 ******************************************************************************/

#include "sensor_cfgs.h"
#include "xparameters.h"

// 2000fps
struct regval_list DVS_regs[] = {
	{0x3225, 0x12},
	{0x0166, 0x31},
	{0x300c, 0x00}, // STREAM_OUT_MODE_r(0 : MIPI, 1 : PARALLEL)
	{0x30a1, 0x00}, // [0] : OUTIF_PARA_ENABLE

	{0x3070, 0x04}, // PLL_P : Default 0x04, 20MHz 0x02
	{0x3071, 0x01}, // PLL_M_MSB : Default 0x01, 20MHz 0x00
	{0x3072, 0x2C}, // PLL_M_LSB : Default 0x2C, 20MHz 0xFA
	{0x3073, 0x00}, // PLL_S : Default 0x00, 20MHz 0x00

	{0x3079, 0x61}, // [7:4] PLL_DIV_G , [3:0] PLL_DIV_DBR
	{0x307a, 0x11}, // [7:4] PLL_DIV_OP, [3:0] PLL_DIV_OS
	{0x307b, 0x25}, // [7:4] PLL_DIV_VS, [3:0] PLL_DIV_VP
	{0x307e, 0x00}, // PLL_PD
	{0x30a0, 0x01}, // OUTIF_ENABLE
	{0x30a2, 0x11}, // DPHY_ENABLE
	{0x30a3, 0x1f}, // DPHY_ENABLE
	{0x30a5, 0x00}, // DPHY_PLL_PMS
	{0x30a6, 0x8D}, // DPHY_PLL_PMS
	{0x30a7, 0x00}, // DPHY_PLL_PMS
	{0x303b, 0x40}, // TEST PAD_CONTROL

	{0x3080, 0x3F}, // CLOCK_ENABLE ([0] : MULTICLK_ENABLE)
	{0x3081, 0x06}, // CLOCK_ENABLE ([3] : DTPDTAG_ENABLE)
	{0x3083, 0x6c}, // CLOCK_ENABLE

	//////////////////////////////////////////////////////////////////
	//////////////        OUTIF Setting        ///////////////////////
	//////////////////////////////////////////////////////////////////

	{0x3915, 0x24}, // [5:4] mipi_ck_mode, [3] : use_line, [2] : use_frm, [1] : use_2_lane, [0] : use_1_lane
					//		{0x3915, 0x04}, // clock disabled between packets
					//		{0x3915, 0x26}, // mipi 2lane占쏙옙占쏙옙 占쏙옙占쏙옙
					//		{0x3915, 0x25}, // mipi 1lane占쏙옙占쏙옙 占쏙옙占쏙옙

	{0x3901, 0x2a},
	{0x3902, 0x00}, // csi2_pkt_size_low
	{0x3903, 0x20}, // csi2_pkt_size_high
	{0x3908, 0x00}, // /DPHY ULPS disable(0x00) enable(0x01)
	{0x391A, 0x7F}, // frm_cnt_low
	{0x391B, 0x00}, // frm_cnt_high
	{0x3910, 0x81}, // [7] : use_timeout, [0] : use_big_endian
	{0x3911, 0x7C}, // to_scnt0 (event clock) (1us)
	{0x3912, 0xE7}, // to_scnt1_low
	{0x3913, 0x03}, // to_scnt1_high (1ms)
	{0x3914, 0x01}, // to_mcnt
	{0x3916, 0x40}, // pkt_gap_low
	{0x3917, 0x00}, // pkt_gap_high
	{0x3918, 0x00}, // frm_gap_low
	{0x3919, 0x95}, // frm_gap_high

	//		{0x3930, 0x60}, // D-PHY TX slew rate up
	{0x3931, 0x4c},
	//////////////////////////////////////////////////////////////////
	//////////////     Scan Rate Setting       ///////////////////////
	//////////////////////////////////////////////////////////////////
	// s329803 // DTAG_MIN_VALUE_BLOKING

	{0x3216, 0x04}, // DTAG_SELX_r
	{0x3217, 0x02}, // DTAG_SENSE_r
	{0x3218, 0x0C}, // DTAG_AY_r
	{0x3219, 0x05}, // DTAG_AY_RST_GAP_r
	{0x321A, 0x07}, // DTAG_APS_RST_r
	{0x321C, 0x04}, // DTAG_COL_MARGIN_r

	{0x320C, 0x1F}, // [6] : DTAG_FREE_RUN_MODE_r, [5] : DTAG_MASK_FIRST_FRAME_r, [1] : DTAG_GRST_MODE_r, [0] : DTAG_GH_MODE_r
	{0x320C, 0x5D},
	{0x321D, 0x00}, // DTAG_FRM_MAGRIN_r_MSB
	{0x321E, 0x03}, // DTAG_FRM_MAGRIN_r_LSB : 1LSB x 2^12 x Event Clock period
					// Scan Time             : # of Column x 92 x Evect Clock period
					// Margin                : 254 x Event Clock Period
					//		{0x321E, 0x0f},

	//////////////////////////////////////////////////////////////////
	//////////////        Analog Setting       ///////////////////////
	//////////////////////////////////////////////////////////////////
	{0x0157, 0x1F}, // CRGS Setting
	{0x0166, 0x0F}, // REG_DIV_BCM_BOT_UNIT_AMP
	{0x0167, 0x07}, // REG_DIV_BCM_BOT_UNIT_ON
	{0x0168, 0x0f}, // REG_DIV_BCM_BOT_UNIT_nOFF

	//////////////////////////////////////////////////////////////////
	//////////////        TimeStamp Setting    ///////////////////////
	//////////////////////////////////////////////////////////////////

	{0x32B1, 0x00}, // TSTAMP_SUB_UNIT_VAL_r_MSB
	{0x32B2, 0x7D}, // TSTAMP_SUB_UNIT_VAL_r_LSB
	{0x32B3, 0x03}, // TSTAMP_REF_UNIT_VAL_r_MSB
	{0x32B4, 0xE7}, // TSTAMP_REF_UNIT_VAL_r_LSB
	{0x311f, 0x00},
	{0x3040, 0x01},
	{0x4308, 0x01},
	{0x4300, 0x01},
	{0x4900, 0x01},

	//////////////////////////////////////////////////////////////////
	//////////////        CROP Setting         ///////////////////////
	//////////////////////////////////////////////////////////////////
	{0x3300, 0x01}, // CROP_BYPASS_r
	{0x3301, 0x00}, // CROP_Y_GROUP_START_r
	{0x3302, 0xFF}, // CROP_Y_START_MASK_r
	{0x3303, 0x59}, // CROP_Y_GROUP_END_r
	{0x3304, 0xFF}, // CROP_Y_END_MASK_r
	{0x3305, 0x00}, // CROP_START_X_r_MSB
	{0x3306, 0x00}, // CROP_START_X_r_LSB
	{0x3307, 0x03}, // CROP_END_X_r_MSB
	{0x3308, 0xBF}, // CROP_END_X_r_LSB
	{0x3309, 0x00}, // CROP_FRAME_SKIP_ENABLE_r
	{0x330A, 0x01}, // CROP_FRAME_REPEAT_NUM_r
	{0x330B, 0x00}, // CROP_VALID_FRAME_START_NUM_r
	{0x330C, 0x00}, // CROP_VALID_FRAME_END_NUM_r

	//////////////////////////////////////////////////////////////////
	//////////////       Slave Mode Setting    ///////////////////////
	//////////////////////////////////////////////////////////////////
	{0x3A00, 0x01}, // [0] : EXT_SYNC_IN_EN_r
	{0x3A01, 0x01}, // [0] : EXT_SYNC_RISING_SEL_r
	{0x3A02, 0x00}, // [0] : BURST_TRIGGER_MODE_EN_r
	{0x3A0C, 0x00}, // ECLK_FINE_COUNT_WIDTH_r_MSB
	{0x3A0D, 0x7C}, // ECLK_FINE_COUNT_WIDTH_r_LSB
	{0x3A0E, 0x03}, // ECLK_COARSE_COUNT_WIDTH_r_MSB
	{0x3A0F, 0x1F}, // ECLK_COARSE_COUNT_WIDTH_r_LSB (1us / LSB)
	{0x3A10, 0x00}, // FRAME_WIDTH_r
	{0x3283, 0x00}, // [0] : DTAG_MULTI_SENSOR_r
	{0x303B, 0x02}, // [1] : EXT_SYNC_IN_EN_r
	{0x32B6, 0x00}, // [1] : TSTAMP_EXT_RESET_r
	{0x4c00, 0x01}, //

	//////////////////////////////////////////////////////////////////
	//////////////          Streaming On       ///////////////////////
	//////////////////////////////////////////////////////////////////

	{0x3091, 0x6C}, // CCI_ADDRESS_CONTROL_2ND_r

	{0x0100, 0x01}};

// // 4000fps
// struct regval_list DVS_regs[] = {
// 		{0x3225, 0x12},
// 		{0x0166, 0x31},
// 		{0x300c, 0x00}, // STREAM_OUT_MODE_r(0 : MIPI, 1 : PARALLEL)
// 		{0x30a1, 0x00}, // [0] : OUTIF_PARA_ENABLE

// 		{0x3070, 0x04}, // PLL_P : Default 0x04, 20MHz 0x02
// 		{0x3071, 0x01}, // PLL_M_MSB : Default 0x01, 20MHz 0x00
// 		{0x3072, 0x2c}, // PLL_M_LSB : Default 0x2C, 20MHz 0xFA
// 		{0x3073, 0x00}, // PLL_S : Default 0x00, 20MHz 0x00

// 		{0x3079, 0x61}, // [7:4] PLL_DIV_G , [3:0] PLL_DIV_DBR
// 		{0x307a, 0x11}, // [7:4] PLL_DIV_OP, [3:0] PLL_DIV_OS
// 		{0x307b, 0x25}, // [7:4] PLL_DIV_VS, [3:0] PLL_DIV_VP
// 		{0x307e, 0x00}, // PLL_PD
// 		{0x30a0, 0x01}, // OUTIF_ENABLE
// 		{0x30a2, 0x11}, // DPHY_ENABLE
// 		{0x30a3, 0x1f}, // DPHY_ENABLE
// 		{0x30a5, 0x00}, // DPHY_PLL_PMS
// 		{0x30a6, 0x8D}, // DPHY_PLL_PMS
// 		{0x30a7, 0x00}, // DPHY_PLL_PMS
// 		{0x303b, 0x40}, // TEST PAD_CONTROL

// 		{0x3080, 0x3F}, // CLOCK_ENABLE ([0] : MULTICLK_ENABLE)
// 		{0x3081, 0x06}, // CLOCK_ENABLE ([3] : DTPDTAG_ENABLE)
// 		{0x3083, 0x6c}, // CLOCK_ENABLE

// 		//////////////////////////////////////////////////////////////////
// 		//////////////        OUTIF Setting        ///////////////////////
// 		//////////////////////////////////////////////////////////////////

// 		{0x3915, 0x24}, // [5:4] mipi_ck_mode, [3] : use_line, [2] : use_frm, [1] : use_2_lane, [0] : use_1_lane
// //		{0x3915, 0x04}, // clock disabled between packets
// //		{0x3915, 0x26}, // mipi 2lane占쏙옙占쏙옙 占쏙옙占쏙옙
// //		{0x3915, 0x25}, // mipi 1lane占쏙옙占쏙옙 占쏙옙占쏙옙

// 		{0x3901, 0x2a},
// 		{0x3902, 0x00}, // csi2_pkt_size_low
// 		{0x3903, 0x20}, // csi2_pkt_size_high
// 		{0x3908, 0x00}, // /DPHY ULPS disable(0x00) enable(0x01)
// 		{0x391A, 0x7F}, // frm_cnt_low
// 		{0x391B, 0x00}, // frm_cnt_high
// 		{0x3910, 0x81}, // [7] : use_timeout, [0] : use_big_endian
// 		{0x3911, 0x7C}, // to_scnt0 (event clock) (1us)
// 		{0x3912, 0xE7}, // to_scnt1_low
// 		{0x3913, 0x03}, // to_scnt1_high (1ms)
// 		{0x3914, 0x01}, // to_mcnt
// 		{0x3916, 0x40}, // pkt_gap_low
// 		{0x3917, 0x00}, // pkt_gap_high
// 		{0x3918, 0x40}, // frm_gap_low
// 		{0x3919, 0x00}, // frm_gap_high

// //		{0x3930, 0x60}, // D-PHY TX slew rate up
// 		{0x3931, 0x4c},
// 		//////////////////////////////////////////////////////////////////
// 		//////////////     Scan Rate Setting       ///////////////////////
// 		//////////////////////////////////////////////////////////////////
// 		//s329803 // DTAG_MIN_VALUE_BLOKING

// 		{0x3216, 0x02}, // DTAG_SELX_r
// 		{0x3217, 0x01}, // DTAG_SENSE_r
// 		{0x3218, 0x00}, // DTAG_AY_r
// 		{0x3219, 0x00}, // DTAG_AY_RST_GAP_r
// 		{0x321A, 0x00}, // DTAG_APS_RST_r
// 		{0x321B, 0x00}, //
// 		{0x321C, 0x02}, // DTAG_COL_MARGIN_r

// 		{0x320C, 0x1F}, // [6] : DTAG_FREE_RUN_MODE_r, [5] : DTAG_MASK_FIRST_FRAME_r, [1] : DTAG_GRST_MODE_r, [0] : DTAG_GH_MODE_r
// 		{0x320C, 0x5D},
// 		{0x321D, 0x00}, // DTAG_FRM_MAGRIN_r_MSB
// 		{0x321E, 0x01}, // DTAG_FRM_MAGRIN_r_LSB : 1LSB x 2^12 x Event Clock period
// 											// Scan Time             : # of Column x 92 x Evect Clock period
// 											// Margin                : 254 x Event Clock Period
// //		{0x321E, 0x0f},

// 		//////////////////////////////////////////////////////////////////
// 		//////////////        Analog Setting       ///////////////////////
// 		//////////////////////////////////////////////////////////////////
// 		{0x0157, 0x1F}, // CRGS Setting
// 		{0x0166, 0x0F}, // REG_DIV_BCM_BOT_UNIT_AMP
// 		{0x0167, 0x07}, // REG_DIV_BCM_BOT_UNIT_ON
// 		{0x0168, 0x0f}, // REG_DIV_BCM_BOT_UNIT_nOFF

// 		//////////////////////////////////////////////////////////////////
// 		//////////////        TimeStamp Setting    ///////////////////////
// 		//////////////////////////////////////////////////////////////////

// 		{0x32B1, 0x00}, // TSTAMP_SUB_UNIT_VAL_r_MSB
// 		{0x32B2, 0x7D}, // TSTAMP_SUB_UNIT_VAL_r_LSB
// 		{0x32B3, 0x03}, // TSTAMP_REF_UNIT_VAL_r_MSB
// 		{0x32B4, 0xE7}, // TSTAMP_REF_UNIT_VAL_r_LSB
// 		{0x311f, 0x00},
// 		{0x3040, 0x01},
// 		{0x4308, 0x01},
// 		{0x4300, 0x01},
// 		{0x4900, 0x01},

// 		//////////////////////////////////////////////////////////////////
// 		//////////////        CROP Setting         ///////////////////////
// 		//////////////////////////////////////////////////////////////////
// 		{0x3300, 0x01}, // CROP_BYPASS_r
// 		{0x3301, 0x00}, // CROP_Y_GROUP_START_r
// 		{0x3302, 0xFF}, // CROP_Y_START_MASK_r
// 		{0x3303, 0x59}, // CROP_Y_GROUP_END_r
// 		{0x3304, 0xFF}, // CROP_Y_END_MASK_r
// 		{0x3305, 0x00}, // CROP_START_X_r_MSB
// 		{0x3306, 0x00}, // CROP_START_X_r_LSB
// 		{0x3307, 0x03}, // CROP_END_X_r_MSB
// 		{0x3308, 0xBF}, // CROP_END_X_r_LSB
// 		{0x3309, 0x00}, // CROP_FRAME_SKIP_ENABLE_r
// 		{0x330A, 0x01}, // CROP_FRAME_REPEAT_NUM_r
// 		{0x330B, 0x00}, // CROP_VALID_FRAME_START_NUM_r
// 		{0x330C, 0x00}, // CROP_VALID_FRAME_END_NUM_r

// 		//////////////////////////////////////////////////////////////////
// 		//////////////       Slave Mode Setting    ///////////////////////
// 		//////////////////////////////////////////////////////////////////
// 		{0x3A00, 0x01}, // [0] : EXT_SYNC_IN_EN_r
// 		{0x3A01, 0x01}, // [0] : EXT_SYNC_RISING_SEL_r
// 		{0x3A02, 0x00}, // [0] : BURST_TRIGGER_MODE_EN_r
// 		{0x3A0C, 0x00}, // ECLK_FINE_COUNT_WIDTH_r_MSB
// 		{0x3A0D, 0x7C}, // ECLK_FINE_COUNT_WIDTH_r_LSB
// 		{0x3A0E, 0x03}, // ECLK_COARSE_COUNT_WIDTH_r_MSB
// 		{0x3A0F, 0x1F}, // ECLK_COARSE_COUNT_WIDTH_r_LSB (1us / LSB)
// 		{0x3A10, 0x00}, // FRAME_WIDTH_r
// 		{0x3283, 0x00}, // [0] : DTAG_MULTI_SENSOR_r
// 		{0x303B, 0x02}, // [1] : EXT_SYNC_IN_EN_r
// 		{0x32B6, 0x00}, // [1] : TSTAMP_EXT_RESET_r
// 		{0x4c00, 0x01}, //

// 		//////////////////////////////////////////////////////////////////
// 		//////////////          Streaming On       ///////////////////////
// 		//////////////////////////////////////////////////////////////////

// 		{0x3091, 0x6C}, // CCI_ADDRESS_CONTROL_2ND_r

// 		{0x0100, 0x01}
// };

struct regval_list imx274_config_1080p_60fps_regs[] = {
	{0x3000, 0x12},
	{0x3120, 0xF0},
	{0x3121, 0x00},
	{0x3122, 0x02},
	{0x3129, 0x9C},
	{0x312A, 0x02},
	{0x312D, 0x02},
	{0x310B, 0x00},
	{0x304C, 0x00},
	{0x304D, 0x03},
	{0x331C, 0x1A},
	{0x331D, 0x00},
	{0x3502, 0x02},
	{0x3529, 0x0E},
	{0x352A, 0x0E},
	{0x352B, 0x0E},
	{0x3538, 0x0E},
	{0x3539, 0x0E},
	{0x3553, 0x00},
	{0x357D, 0x05},
	{0x357F, 0x05},
	{0x3581, 0x04},
	{0x3583, 0x76},
	{0x3587, 0x01},
	{0x35BB, 0x0E},
	{0x35BC, 0x0E},
	{0x35BD, 0x0E},
	{0x35BE, 0x0E},
	{0x35BF, 0x0E},
	{0x366E, 0x00},
	{0x366F, 0x00},
	{0x3670, 0x00},
	{0x3671, 0x00},
	{0x3304, 0x32},
	{0x3305, 0x00},
	{0x3306, 0x32},
	{0x3307, 0x00},
	{0x3590, 0x32},
	{0x3591, 0x00},
	{0x3686, 0x32},
	{0x3687, 0x00},
	{0x3004, 0x02},
	{0x3005, 0x21},
	{0x3006, 0x00},
	{0x3007, 0x11},
	{0x300E, 0x01},
	{0x300F, 0x00},
	{0x301A, 0x00},
	{0x306B, 0x05},
	{0x30E2, 0x02},
	{0x30F6, 0x04},
	{0x30F7, 0x01},
	{0x30F8, 0x06},
	{0x30F9, 0x09},
	{0x30FA, 0x00},
	{0x30EE, 0x01},
	{0x3130, 0x4E},
	{0x3131, 0x04},
	{0x3132, 0x46},
	{0x3133, 0x04},
	{0x3342, 0x0A},
	{0x3343, 0x00},
	{0x3344, 0x1A},
	{0x3345, 0x00},
	{0x33A6, 0x01},
	{0x3528, 0x0E},
	{0x3554, 0x00},
	{0x3555, 0x01},
	{0x3556, 0x01},
	{0x3557, 0x01},
	{0x3558, 0x01},
	{0x3559, 0x00},
	{0x355A, 0x00},
	{0x35BA, 0x0E},
	{0x366A, 0x1B},
	{0x366B, 0x1A},
	{0x366C, 0x19},
	{0x366D, 0x17},
	{0x3A41, 0x08},
	{0x3012, 0x03},
	{0x3000, 0x00},
	{0x303E, 0x02},
	{0x30F4, 0x00},
	{0x3018, 0xA2},
};

struct regval_list imx274_config_1080p_30fps_regs[] = {
	{0x3000, 0x12},
	{0x3120, 0xF0},
	{0x3121, 0x00},
	{0x3122, 0x02},
	{0x3129, 0x9C},
	{0x312A, 0x02},
	{0x312D, 0x02},
	{0x310B, 0x00},
	{0x304C, 0x00},
	{0x304D, 0x03},
	{0x331C, 0x1A},
	{0x331D, 0x00},
	{0x3502, 0x02},
	{0x3529, 0x0E},
	{0x352A, 0x0E},
	{0x352B, 0x0E},
	{0x3538, 0x0E},
	{0x3539, 0x0E},
	{0x3553, 0x00},
	{0x357D, 0x05},
	{0x357F, 0x05},
	{0x3581, 0x04},
	{0x3583, 0x76},
	{0x3587, 0x01},
	{0x35BB, 0x0E},
	{0x35BC, 0x0E},
	{0x35BD, 0x0E},
	{0x35BE, 0x0E},
	{0x35BF, 0x0E},
	{0x366E, 0x00},
	{0x366F, 0x00},
	{0x3670, 0x00},
	{0x3671, 0x00},
	{0x3304, 0x32},
	{0x3305, 0x00},
	{0x3306, 0x32},
	{0x3307, 0x00},
	{0x3590, 0x32},
	{0x3591, 0x00},
	{0x3686, 0x32},
	{0x3687, 0x00},
	{0x3004, 0x02},
	{0x3005, 0x61},
	{0x3006, 0x00},
	{0x3007, 0x19},
	{0x300E, 0x02},
	{0x300F, 0x00},
	{0x301A, 0x00},
	{0x306B, 0x05},
	{0x30E2, 0x02},
	{0x30F6, 0x68},
	{0x30F7, 0x01},
	{0x30F8, 0xAC},
	{0x30F9, 0x08},
	{0x30FA, 0x00},
	{0x30EE, 0x01},
	{0x3130, 0x4E},
	{0x3131, 0x04},
	{0x3132, 0x46},
	{0x3133, 0x04},
	{0x3342, 0x0A},
	{0x3343, 0x00},
	{0x3344, 0x1B},
	{0x3345, 0x00},
	{0x33A6, 0x00},
	{0x3528, 0x0E},
	{0x3554, 0x00},
	{0x3555, 0x01},
	{0x3556, 0x01},
	{0x3557, 0x01},
	{0x3558, 0x01},
	{0x3559, 0x00},
	{0x355A, 0x00},
	{0x35BA, 0x0E},
	{0x366A, 0x1B},
	{0x366B, 0x19},
	{0x366C, 0x17},
	{0x366D, 0x17},
	{0x3A41, 0x08},
	{0x3012, 0x03},
	{0x3000, 0x00},
	{0x303E, 0x02},
	{0x30F4, 0x00},
	{0x3018, 0xA2},
};

struct regval_list imx274_config_720p_60fps_regs[] = {
	{0x3000, 0x12},
	{0x3120, 0xF0},
	{0x3121, 0x00},
	{0x3122, 0x02},
	{0x3129, 0x9C},
	{0x312A, 0x02},
	{0x312D, 0x02},
	{0x310B, 0x00},
	{0x304C, 0x00},
	{0x304D, 0x03},
	{0x331C, 0x1A},
	{0x331D, 0x00},
	{0x3502, 0x02},
	{0x3529, 0x0E},
	{0x352A, 0x0E},
	{0x352B, 0x0E},
	{0x3538, 0x0E},
	{0x3539, 0x0E},
	{0x3553, 0x00},
	{0x357D, 0x05},
	{0x357F, 0x05},
	{0x3581, 0x04},
	{0x3583, 0x76},
	{0x3587, 0x01},
	{0x35BB, 0x0E},
	{0x35BC, 0x0E},
	{0x35BD, 0x0E},
	{0x35BE, 0x0E},
	{0x35BF, 0x0E},
	{0x366E, 0x00},
	{0x366F, 0x00},
	{0x3670, 0x00},
	{0x3671, 0x00},
	{0x3304, 0x32},
	{0x3305, 0x00},
	{0x3306, 0x32},
	{0x3307, 0x00},
	{0x3590, 0x32},
	{0x3591, 0x00},
	{0x3686, 0x32},
	{0x3687, 0x00},
	{0x3004, 0x03},
	{0x3005, 0x31},
	{0x3006, 0x00},
	{0x3007, 0x09},
	{0x300E, 0x01},
	{0x300F, 0x00},
	{0x301A, 0x00},
	{0x306B, 0x05},
	{0x30E2, 0x03},
	{0x30F6, 0x04},
	{0x30F7, 0x01},
	{0x30F8, 0x06},
	{0x30F9, 0x09},
	{0x30FA, 0x00},
	{0x30EE, 0x01},
	{0x3130, 0xE2},
	{0x3131, 0x02},
	{0x3132, 0xDE},
	{0x3133, 0x02},
	{0x3342, 0x0A},
	{0x3343, 0x00},
	{0x3344, 0x1B},
	{0x3345, 0x00},
	{0x33A6, 0x01},
	{0x3528, 0x0E},
	{0x3554, 0x00},
	{0x3555, 0x01},
	{0x3556, 0x01},
	{0x3557, 0x01},
	{0x3558, 0x01},
	{0x3559, 0x00},
	{0x355A, 0x00},
	{0x35BA, 0x0E},
	{0x366A, 0x1B},
	{0x366B, 0x19},
	{0x366C, 0x17},
	{0x366D, 0x17},
	{0x3A41, 0x04},
	{0x3012, 0x05},
	{0x3000, 0x00},
	{0x303E, 0x02},
	{0x30F4, 0x00},
	{0x3018, 0xA2},
};

struct regval_list imx274_config_4K_30fps_regs[] = {
	{0x3000, 0x12},
	{0x3120, 0xF0},
	{0x3121, 0x00},
	{0x3122, 0x02},
	{0x3129, 0x9C},
	{0x312A, 0x02},
	{0x312D, 0x02},
	{0x310B, 0x00},
	{0x304C, 0x00},
	{0x304D, 0x03},
	{0x331C, 0x1A},
	{0x331D, 0x00},
	{0x3502, 0x02},
	{0x3529, 0x0E},
	{0x352A, 0x0E},
	{0x352B, 0x0E},
	{0x3538, 0x0E},
	{0x3539, 0x0E},
	{0x3553, 0x00},
	{0x357D, 0x05},
	{0x357F, 0x05},
	{0x3581, 0x04},
	{0x3583, 0x76},
	{0x3587, 0x01},
	{0x35BB, 0x0E},
	{0x35BC, 0x0E},
	{0x35BD, 0x0E},
	{0x35BE, 0x0E},
	{0x35BF, 0x0E},
	{0x366E, 0x00},
	{0x366F, 0x00},
	{0x3670, 0x00},
	{0x3671, 0x00},
	{0x3304, 0x32},
	{0x3305, 0x00},
	{0x3306, 0x32},
	{0x3307, 0x00},
	{0x3590, 0x32},
	{0x3591, 0x00},
	{0x3686, 0x32},
	{0x3687, 0x00},
	{0x3004, 0x01},
	{0x3005, 0x01},
	{0x3006, 0x00},
	{0x3007, 0x02},
	{0x300C, 0xff},
	{0x300D, 0x00},
	{0x300E, 0x00},
	{0x300F, 0x00},
	{0x3018, 0xA2},
	{0x301A, 0x00},
	{0x306B, 0x05},
	{0x30E2, 0x01},
	{0x30F6, 0xED},
	{0x30F7, 0x01},
	{0x30F8, 0x08},
	{0x30F9, 0x13},
	{0x30FA, 0x00},
	{0x30dd, 0x01},
	{0x30de, 0x0b},
	{0x30df, 0x00},
	{0x30e0, 0x06},
	{0x30e1, 0x00},
	{0x3037, 0x01},
	{0x3038, 0x0c},
	{0x3039, 0x00},
	{0x303a, 0x0c},
	{0x303b, 0x0f},
	{0x30EE, 0x01},
	{0x3130, 0x86},
	{0x3131, 0x08},
	{0x3132, 0x7E},
	{0x3133, 0x08},
	{0x3342, 0x0A},
	{0x3343, 0x00},
	{0x3344, 0x16},
	{0x3345, 0x00},
	{0x33A6, 0x01},
	{0x3528, 0x0E},
	{0x3554, 0x1F},
	{0x3555, 0x01},
	{0x3556, 0x01},
	{0x3557, 0x01},
	{0x3558, 0x01},
	{0x3559, 0x00},
	{0x355A, 0x00},
	{0x35BA, 0x0E},
	{0x366A, 0x1B},
	{0x366B, 0x1A},
	{0x366C, 0x19},
	{0x366D, 0x17},
	{0x3A41, 0x08},
	{0x3012, 0x04},
	{0x3000, 0x00},
	{0x303E, 0x02},
	{0x30F4, 0x00},
	{0x3018, 0xA2},
};

struct regval_list imx274_config_4K_60fps_regs[] = {
	{0x3000, 0x12},
	{0x3120, 0xF0},
	{0x3121, 0x00},
	{0x3122, 0x02},
	{0x3129, 0x9C},
	{0x312A, 0x02},
	{0x312D, 0x02},
	{0x310B, 0x00},
	{0x304C, 0x00},
	{0x304D, 0x03},
	{0x331C, 0x1A},
	{0x331D, 0x00},
	{0x3502, 0x02},
	{0x3529, 0x0E},
	{0x352A, 0x0E},
	{0x352B, 0x0E},
	{0x3538, 0x0E},
	{0x3539, 0x0E},
	{0x3553, 0x00},
	{0x357D, 0x05},
	{0x357F, 0x05},
	{0x3581, 0x04},
	{0x3583, 0x76},
	{0x3587, 0x01},
	{0x35BB, 0x0E},
	{0x35BC, 0x0E},
	{0x35BD, 0x0E},
	{0x35BE, 0x0E},
	{0x35BF, 0x0E},
	{0x366E, 0x00},
	{0x366F, 0x00},
	{0x3670, 0x00},
	{0x3671, 0x00},
	{0x3304, 0x32},
	{0x3305, 0x00},
	{0x3306, 0x32},
	{0x3307, 0x00},
	{0x3590, 0x32},
	{0x3591, 0x00},
	{0x3686, 0x32},
	{0x3687, 0x00},
	{0x3004, 0x01},
	{0x3005, 0x01},
	{0x3006, 0x00},
	{0x3007, 0xA2},
	{0x300C, 0xff},
	{0x300D, 0x00},
	{0x300E, 0x00},
	{0x300F, 0x00},
	{0x3018, 0xA2},
	{0x301A, 0x00},
	{0x306B, 0x05},
	{0x30E2, 0x01},
	{0x30F6, 0x07},
	{0x30F7, 0x01},
	{0x30F8, 0xC6},
	{0x30F9, 0x11},
	{0x30FA, 0x00},
	{0x30dd, 0x01},
	{0x30de, 0x07},
	{0x30df, 0x00},
	{0x30e0, 0x03},
	{0x30e1, 0x00},
	{0x3037, 0x01},
	{0x3038, 0x0c},
	{0x3039, 0x00},
	{0x303a, 0x0c},
	{0x303b, 0x0f},
	{0x30EE, 0x01},
	{0x3130, 0x78},
	{0x3131, 0x08},
	{0x3132, 0x70},
	{0x3133, 0x08},
	{0x3342, 0x0A},
	{0x3343, 0x00},
	{0x3344, 0x16},
	{0x3345, 0x00},
	{0x33A6, 0x01},
	{0x3528, 0x0E},
	{0x3554, 0x1F},
	{0x3555, 0x01},
	{0x3556, 0x01},
	{0x3557, 0x01},
	{0x3558, 0x01},
	{0x3559, 0x00},
	{0x355A, 0x00},
	{0x35BA, 0x0E},
	{0x366A, 0x1B},
	{0x366B, 0x1A},
	{0x366C, 0x19},
	{0x366D, 0x17},
	{0x3A41, 0x08},
	{0x3012, 0x04},
	{0x3000, 0x00},
	{0x303E, 0x02},
	{0x30F4, 0x00},
	{0x3018, 0xA2},
};

#define ARRAY_SIZE(x) sizeof((x)) / sizeof((x)[0])

const int length_DVS_regs = ARRAY_SIZE(DVS_regs);
const int length_imx274_config_720p_60fps_regs =
	ARRAY_SIZE(imx274_config_720p_60fps_regs);
const int length_imx274_config_1080p_60fps_regs =
	ARRAY_SIZE(imx274_config_1080p_60fps_regs);
const int length_imx274_config_1080p_30fps_regs =
	ARRAY_SIZE(imx274_config_1080p_30fps_regs);
const int length_imx274_config_4K_60fps_regs =
	ARRAY_SIZE(imx274_config_4K_60fps_regs);
const int length_imx274_config_4K_30fps_regs =
	ARRAY_SIZE(imx274_config_4K_30fps_regs);
