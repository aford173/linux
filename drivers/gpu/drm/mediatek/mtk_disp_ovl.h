/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2025 MediaTek Inc.
 */

#ifndef __MTK_DISP_OVL_H__
#define __MTK_DISP_OVL_H__

#include <drm/drm_fourcc.h>

extern const u32 mt8173_ovl_formats[];
extern const size_t mt8173_ovl_formats_len;
extern const u32 mt8195_ovl_formats[];
extern const size_t mt8195_ovl_formats_len;

bool mtk_ovl_is_10bit_rgb(unsigned int fmt);
unsigned int mtk_ovl_fmt_convert(unsigned int fmt, unsigned int blend_mode,
				 bool fmt_rgb565_is_0, bool color_convert,
				 u8 clrfmt_shift, u32 clrfmt_man, u32 byte_swap, u32 rgb_swap);

#endif
