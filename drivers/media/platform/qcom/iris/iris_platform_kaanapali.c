// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "iris_core.h"
#include "iris_platform_common.h"

const char *const iris_kaanapali_clk_reset_table[] = {
	"bus0",
	"bus1",
	"core",
	"vcodec0_core",
};

const char *const iris_kaanapali_pmdomain_table[] = {
	"venus",
	"vcodec0",
	"vpp0",
	"vpp1",
	"apv",
};

const struct platform_clk_data iris_kaanapali_clk_table[] = {
	{IRIS_AXI_CLK,		"iface"			},
	{IRIS_CTRL_CLK,		"core"			},
	{IRIS_HW_CLK,		"vcodec0_core"		},
	{IRIS_AXI1_CLK,		"iface1"		},
	{IRIS_CTRL_FREERUN_CLK,	"core_freerun"		},
	{IRIS_HW_FREERUN_CLK,	"vcodec0_core_freerun"	},
	{IRIS_BSE_HW_CLK,	"vcodec_bse"		},
	{IRIS_VPP0_HW_CLK,	"vcodec_vpp0"		},
	{IRIS_VPP1_HW_CLK,	"vcodec_vpp1"		},
	{IRIS_APV_HW_CLK,	"vcodec_apv"		},
};

const char *const iris_kaanapali_opp_clk_table[] = {
	"vcodec0_core",
	"vcodec_apv",
	"vcodec_bse",
	"core",
	NULL,
};

struct tz_cp_config iris_kaanapali_tz_cp_config[] = {
	{
		.cp_start = VIDEO_REGION_VM0_SECURE_NP_ID,
		.cp_size = 0,
		.cp_nonpixel_start = 0x01000000,
		.cp_nonpixel_size = 0x24800000,
	},
	{
		.cp_start = VIDEO_REGION_VM0_NONSECURE_NP_ID,
		.cp_size = 0,
		.cp_nonpixel_start = 0x25800000,
		.cp_nonpixel_size = 0xda400000,
	},
};
