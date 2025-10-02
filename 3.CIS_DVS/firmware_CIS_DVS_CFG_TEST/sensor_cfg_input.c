#include "sensor_cfg_input.h"
#include "sensor_cfgs.h"
#include "xuartps_hw.h"
#include "xil_printf.h"
#include <string.h>
#include <ctype.h>
#include <stdint.h>

/* ===== Local helpers / fallbacks ===== */
#ifndef ARRAY_LEN
#define ARRAY_LEN(a) (sizeof(a)/sizeof((a)[0]))
#endif

/* ===== Always-last register set (edit to your needs) =======================
 * These register addresses are FORCED to execute LAST, in this exact order.
 * Put 16-bit hex addresses here (datasheet values).
 * Example below is illustrative — replace with your real tail list.
 */
static const uint16_t kAlwaysLastRegs[] = {
	0x3091, 0x0100,
    /* 0xFFFF */            /* add more as needed */
};
static inline int is_tail_addr(uint16_t addr) {
    for (size_t i = 0; i < ARRAY_LEN(kAlwaysLastRegs); ++i)
        if (kAlwaysLastRegs[i] == addr) return 1;
    return 0;
}

/* ===== Small helpers ===== */
static inline int is_hex_char(char c) {
    return (c >= '0' && c <= '9') ||
           (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}
static inline int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}
static int ci_starts_with(const char *s, const char *pfx) {
    while (*pfx) {
        char a = *s ? (char)tolower((unsigned char)*s) : 0;
        char b = (char)tolower((unsigned char)*pfx);
        if (a != b) return 0;
        ++s; ++pfx;
    }
    return 1;
}
static int ci_equals(const char *a, const char *b) {
    while (*a && *b) {
        char ca = (char)tolower((unsigned char)*a++);
        char cb = (char)tolower((unsigned char)*b++);
        if (ca != cb) return 0;
    }
    return *a == '\0' && *b == '\0';
}

/* ===== Last-used configuration cache (RAM only) ===== */
static regval_list g_last_cfg[SENSOR_CFG_WORKBUF_CAP];
static size_t      g_last_cfg_len = 0;
static regval_list g_last_overrides[SENSOR_CFG_MAX_OVERRIDES];
static size_t      g_last_overrides_len = 0;

/* ===== Register metadata (extend as needed) =====
 * Includes reset/default values and experiment notes. Use 0xFF for "unknown".
 */
typedef struct reg_meta {
    uint16_t addr;        /* 16-bit register address (datasheet) */
    const char *name;     /* short name (e.g., "GAIN_MODE")      */
    const char *brief;    /* one-line role/description           */
    const char *notes;    /* access/range/timing free-form note  */

    uint8_t reset_val;    /* POR/datasheet reset value (0xFF = unknown) */
    uint8_t default_val;  /* project baseline default (0xFF = unknown)  */

    struct {              /* lab experiment ledger (optional) */
        uint8_t     val;  /* experimental value */
        const char *desc; /* short context/result note */
    } exp[4];
    uint8_t exp_len;      /* number of valid entries in exp[] (0~4) */
} reg_meta;

/* Example DB entries — fill with your real map */
static const reg_meta kRegDB[] = {
	// ex)
	/* {0x33ff, "AAA", "Frame timing/control", "RW", 0x12, 0x12, {{0}}, 0}, */
	/* {0x22aa, "BBB",  "Analog/digital gain selection", "RW", 0x31, 0x31, {{0x55,"low light"}}, 1}, */
	/*{
		.addr = 0x22aa, .name="BBB",
		.brief="Analog/digital gain selection",
		.notes="RW; latched at frame boundary",
		.reset_val=0x31, .default_val=0x31,
		.exp = {
		  { 0x55, "Low light (~5 lux): SNR↑, blur ok (2025-09-23)" },
		  { 0x21, "Bright scene: prevents highlight clipping" },
		},
		.exp_len = 2
	},*/

	{0x3210, "DTAG_GH_SET_r",
			""
			"memo: 0x1E",
			0xFF, 0xFF,
			{{0}}, 0},
	{0x3211, "DTAG_GL_SET_r",
			""
			"memo: 0x00",
			0xFF, 0xFF,
			{{0}}, 0},
	{0x3211, "DTAG_GR_r",
			""
			"memo: 0x07",
			0xFF, 0xFF,
			{{0}}, 0},
	{0x3211, "DTAG_GL_HLD_r",
			""
			"memo: 0x1D",
			0xFF, 0xFF,
			{{0}}, 0},
	{0x3214, " DTAG_DELAY_r(1)",
			""
			"memo: 0x00",
			0xFF, 0xFF,
			{{0}}, 0},
	{0x3215, " DTAG_DELAY_r(2)",
			""
			"memo: 0x00",
			0xFF, 0xFF,
			{{0}}, 0},


    {0x320C, "DTAG_GRST_MODE_r",
        "[6] FREE_RUN, [5] MASK_FIRST_FRAME, [1] GRST_MODE, [0] GH_MODE",
        "memo: 0x5D",
		0xFF, 0x5D,
		{{0}}, 0},

    {0x3216, "DTAG_SELX_r", "", "memo: 0x02", 0xFF, 0x02, {{0}}, 0},
    {0x3217, "DTAG_SENSE_r","", "memo: 0x01", 0xFF, 0x01, {{0}}, 0},
    {0x3218, "DTAG_AY_r",   "", "memo: 0x00", 0xFF, 0x00, {{0}}, 0},
    {0x3219, "DTAG_AY_RST_GAP_r","", "memo: 0x00", 0xFF, 0x00, {{0}}, 0},
    {0x321A, "DTAG_APS_RST_r","", "memo: 0x00", 0xFF, 0x00, {{0}}, 0},
    {0x321C, "DTAG_COL_MARGIN_r","", "memo: 0x02", 0xFF, 0x02, {{0}}, 0},
    {0x321D, "DTAG_FRM_MAGRIN_r_MSB","--", "memo: 0x00", 0xFF, 0x00, {{0}}, 0},
    {0x321E, "DTAG_FRM_MAGRIN_r_LSB","--", "memo: 0x02", 0xFF, 0x02, {{0}}, 0},
};

static const reg_meta* regmeta_lookup(uint16_t addr) {
    for (size_t i = 0; i < ARRAY_LEN(kRegDB); ++i)
        if (kRegDB[i].addr == addr) return &kRegDB[i];
    return NULL;
}

/* ===== Core utilities ===== */
static int find_addr_index(const regval_list *arr, size_t len, uint16_t addr) {
    for (size_t i = 0; i < len; ++i)
        if (arr[i].Address == addr) return (int)i;
    return -1;
}

/* Pretty-print one address with metadata + default/override/effective values */
static void print_reg_info_addr(uint16_t addr,
                                const regval_list *defaults, size_t defaults_len,
                                const regval_list *overrides, size_t overrides_len)
{
    const reg_meta *m = regmeta_lookup(addr);
    int idx_def = find_addr_index(defaults, defaults_len, addr);
    int idx_ovr = find_addr_index(overrides, overrides_len, addr);

    const char *nm   = m ? m->name  : "(unknown)";
    const char *brf  = m ? m->brief : "No metadata available";
    const char *note = m ? m->notes : "-";

    xil_printf("0x%04X  %-16s  %s\r\n", addr, nm, brf);
    xil_printf("    notes       : %s\r\n", note);

    if (m) {
        if (m->reset_val   != 0xFF) xil_printf("    reset_val   : 0x%02X\r\n", m->reset_val);
        if (m->default_val != 0xFF) xil_printf("    default_val : 0x%02X (DB)\r\n", m->default_val);
        if (m->exp_len) {
            for (uint8_t i = 0; i < m->exp_len; ++i)
                xil_printf("    exp[%u]      : 0x%02X  %s\r\n", i, m->exp[i].val, m->exp[i].desc ? m->exp[i].desc : "");
        }
    }

    if (idx_def >= 0) xil_printf("    defaults[]  : 0x%02X\r\n", defaults[idx_def].Data);
    else              xil_printf("    defaults[]  : (N/A)\r\n");
    if (idx_ovr >= 0) xil_printf("    overrides[] : 0x%02X\r\n", overrides[idx_ovr].Data);
    else              xil_printf("    overrides[] : (none)\r\n");

    if      (idx_ovr >= 0) xil_printf("    effective   : 0x%02X (override)\r\n", overrides[idx_ovr].Data);
    else if (idx_def >= 0) xil_printf("    effective   : 0x%02X (default)\r\n",  defaults[idx_def].Data);
    else                   xil_printf("    effective   : (undefined in current sets)\r\n");
}

/* UART line input (blocking; echoes; backspace supported). */
static size_t uart_readline(char *buf, size_t cap) {
    if (cap == 0) return 0;
    size_t n = 0;
    for (;;) {
        char c = (char)XUartPs_RecvByte(UART_BASEADDR);
        if (c == '\r' || c == '\n') {
            XUartPs_SendByte(UART_BASEADDR, '\r');
            XUartPs_SendByte(UART_BASEADDR, '\n');
            break;
        } else if (c == 0x08 || c == 0x7F) { /* backspace */
            if (n > 0) {
                XUartPs_SendByte(UART_BASEADDR, 0x08);
                XUartPs_SendByte(UART_BASEADDR, ' ');
                XUartPs_SendByte(UART_BASEADDR, 0x08);
                --n;
            }
        } else {
            XUartPs_SendByte(UART_BASEADDR, (uint8_t)c);
            if (n + 1 < cap) buf[n++] = c; /* keep room for '\0' */
        }
    }
    buf[n] = '\0';
    return n;
}

/* Parse "AAAA:DD 0166=31 0ABC 07, ..." into out[]. 0 on success, <0 on error. */
static int parse_pairs(const char *line,
                       regval_list *out, size_t out_cap, size_t *out_len) {
    size_t n = 0; const char *p = line;
    while (*p) {
        while (*p && !is_hex_char(*p) && *p != '0') ++p;
        if (!*p) break;

        if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;

        int cnt = 0; uint16_t addr = 0;
        while (is_hex_char(*p) && cnt < 4) {
            int nib = hex_nibble(*p++); if (nib < 0) return SENSOR_CFG_ERR_PARSE;
            addr = (uint16_t)((addr << 4) | (uint16_t)nib); ++cnt;
        }
        if (cnt == 0) break;
        if (cnt != 4) return SENSOR_CFG_ERR_PARSE;

        while (*p && !is_hex_char(*p) && *p != '0') ++p;
        if (!*p) return SENSOR_CFG_ERR_PARSE;

        if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;

        cnt = 0; uint16_t data = 0;
        while (is_hex_char(*p) && cnt < 2) {
            int nib = hex_nibble(*p++); if (nib < 0) return SENSOR_CFG_ERR_PARSE;
            data = (uint16_t)((data << 4) | (uint16_t)nib); ++cnt;
        }
        if (cnt != 2) return SENSOR_CFG_ERR_PARSE;

        if (n >= out_cap) return SENSOR_CFG_ERR_CAP;
        out[n].Address = addr;
        out[n].Data    = (uint8_t)data;
        ++n;

        while (*p && !is_hex_char(*p) && *p != '0') ++p;
    }
    *out_len = n;
    return SENSOR_CFG_OK;
}

/* Upsert into overrides list (editor list). */
static int upsert_pairs(regval_list *dst, size_t *dst_len, size_t dst_cap,
                        const regval_list *src, size_t src_len) {
    size_t n = *dst_len;
    for (size_t i = 0; i < src_len; ++i) {
        int idx = find_addr_index(dst, n, src[i].Address);
        if (idx >= 0) {
            dst[idx].Data = src[i].Data;
        } else {
            if (n >= dst_cap) return SENSOR_CFG_ERR_CAP;
            dst[n++] = src[i];
        }
    }
    *dst_len = n;
    return SENSOR_CFG_OK;
}

/* Delete helpers */
static int delete_by_address(regval_list *arr, size_t *len, uint16_t addr) {
    size_t n = *len, w = 0; int removed = 0;
    for (size_t i = 0; i < n; ++i) {
        if (arr[i].Address == addr) { ++removed; continue; }
        if (w != i) arr[w] = arr[i];
        ++w;
    }
    *len = w;
    return removed;
}
static int delete_by_index(regval_list *arr, size_t *len, size_t idx) {
    if (idx >= *len) return 0;
    for (size_t i = idx + 1; i < *len; ++i) arr[i-1] = arr[i];
    --(*len);
    return 1;
}

/* ===== Merge with forced tail ordering =====================================
 * Rules:
 *  1) Start from defaults order; apply overrides IN PLACE where address exists.
 *  2) Unknown override addresses are APPENDED after defaults.
 *  3) Any address listed in kAlwaysLastRegs[] is MOVED to the very END,
 *     in the exact order of kAlwaysLastRegs[], regardless of whether it
 *     came from defaults or from overrides.
 * Final length = defaults_len + (# of unknown overrides).
 */
static int build_regset_tail_ordered(const regval_list *def, size_t def_len,
                                     const regval_list *ovr, size_t ovr_len,
                                     regval_list *out, size_t out_cap, size_t *out_len)
{
    /* We'll use 'out' as a working buffer, then compact & append. */
    if (out_cap < def_len) {
        /* Not enough for even defaults; truncate and report. */
        memcpy(out, def, (out_cap) * sizeof(regval_list));
        *out_len = out_cap;
        return SENSOR_CFG_ERR_CAP;
    }
    memcpy(out, def, def_len * sizeof(regval_list));
    size_t n_def_work = def_len;

    /* Build list of unknown overrides to append later; keep unique. */
    regval_list extra[SENSOR_CFG_MAX_OVERRIDES];
    size_t extra_len = 0;

    /* Step 1: apply overrides in place if present in defaults; else add to 'extra'. */
    for (size_t i = 0; i < ovr_len; ++i) {
        int idx = find_addr_index(out, n_def_work, ovr[i].Address);
        if (idx >= 0) {
            out[idx].Data = ovr[i].Data;
        } else {
            /* upsert into extra[] to stay unique */
            int eidx = find_addr_index(extra, extra_len, ovr[i].Address);
            if (eidx >= 0) extra[eidx].Data = ovr[i].Data;
            else {
                if (extra_len >= ARRAY_LEN(extra)) {
                    *out_len = n_def_work; /* safe to report how far we got */
                    return SENSOR_CFG_ERR_CAP;
                }
                extra[extra_len++] = ovr[i];
            }
        }
    }

    /* Step 2: strip ALL tail addresses from defaults-working array into a small buffer. */
    regval_list tail_from_defaults[ARRAY_LEN(kAlwaysLastRegs)];
    size_t tail_def_len = 0;

    size_t w = 0;
    for (size_t i = 0; i < n_def_work; ++i) {
        if (is_tail_addr(out[i].Address)) {
            if (tail_def_len < ARRAY_LEN(tail_from_defaults))
                tail_from_defaults[tail_def_len++] = out[i];
            /* else: if tail list duplicated in defaults unexpectedly, just drop extras */
        } else {
            if (w != i) out[w] = out[i];
            ++w;
        }
    }
    n_def_work = w; /* now 'out[0..w)' contains defaults without any tail addresses */

    /* Step 3: split extra[] into non-tail and tail buckets. */
    regval_list extra_nontail[SENSOR_CFG_MAX_OVERRIDES];
    size_t extra_nontail_len = 0;
    regval_list extra_tail[SENSOR_CFG_MAX_OVERRIDES];
    size_t extra_tail_len = 0;

    for (size_t i = 0; i < extra_len; ++i) {
        if (is_tail_addr(extra[i].Address)) {
            extra_tail[extra_tail_len++] = extra[i];
        } else {
            extra_nontail[extra_nontail_len++] = extra[i];
        }
    }

    /* Step 4: capacity check for final size:
       final_len = n_def_work + tail_def_len + extra_nontail_len + unique(extra_tail vs tail_def)
                 = def_len + extra_len  (since we removed then re-add tail from defaults, and extra_tail add only unknown tails)
    */
    size_t final_needed = def_len + extra_len;
    if (final_needed > out_cap) {
        /* We can still fill up to out_cap items deterministically. */
        /* Copy back non-tail defaults (already in out[0..n_def_work)) — done. */
        /* Append as much extra_nontail as fits, then forced tail order until capacity. */
        size_t k = n_def_work;

        /* Append non-tail extras */
        for (size_t i = 0; i < extra_nontail_len && k < out_cap; ++i) {
            out[k++] = extra_nontail[i];
        }

        /* Build helper: look up value to place for each tail address (ovr > def). */
        for (size_t t = 0; t < ARRAY_LEN(kAlwaysLastRegs) && k < out_cap; ++t) {
            uint16_t addr = kAlwaysLastRegs[t];
            /* prefer extra override if present */
            int eidx = find_addr_index(extra_tail, extra_tail_len, addr);
            if (eidx >= 0) { out[k++] = extra_tail[eidx]; continue; }
            /* else use updated default tail if it existed */
            int didx = find_addr_index(tail_from_defaults, tail_def_len, addr);
            if (didx >= 0) { out[k++] = tail_from_defaults[didx]; continue; }
            /* else: tail addr not present anywhere -> skip */
        }

        *out_len = k;
        return SENSOR_CFG_ERR_CAP;
    }

    /* Step 5: build final array:
       - out[0..n_def_work): defaults without tails (already placed)
       - append all non-tail extras
       - append forced tail addresses in kAlwaysLastRegs order
    */
    size_t k = n_def_work;

    /* Append non-tail extras */
    for (size_t i = 0; i < extra_nontail_len; ++i) {
        out[k++] = extra_nontail[i];
    }

    /* Append tails in forced order; prefer extra-tail (override) over default-tail. */
    for (size_t t = 0; t < ARRAY_LEN(kAlwaysLastRegs); ++t) {
        uint16_t addr = kAlwaysLastRegs[t];
        int eidx = find_addr_index(extra_tail, extra_tail_len, addr);
        if (eidx >= 0) {
            out[k++] = extra_tail[eidx];
            continue;
        }
        int didx = find_addr_index(tail_from_defaults, tail_def_len, addr);
        if (didx >= 0) {
            out[k++] = tail_from_defaults[didx];
            continue;
        }
        /* else: nothing to place for this tail addr -> skip */
    }

    *out_len = k;
    return SENSOR_CFG_OK;
}

/* Pretty-print a regval list (helper for presets & overrides) */
static void print_reg_list(const regval_list *arr, size_t len) {
    for (size_t i = 0; i < len; ++i)
        xil_printf("  [%03d] 0x%04X <- 0x%02X\r\n", (int)i, arr[i].Address, arr[i].Data);
}

/* Pretty-print current overrides. */
static void print_overrides(const regval_list *arr, size_t len) {
    xil_printf("Overrides (%d item%s):\r\n", (int)len, (len==1?"":"s"));
    print_reg_list(arr, len);
}


/* ====== Presets ============================================================
 * 오버라이드 후보 집합
 * kPreset과 함께 kPresets도 함께 수정
 * - preset '<idx|name>' : 현재 overrides에 병합(upsert)
 * - preset set '<idx|name>' : 현재 overrides를 프리셋으로 교체
 * - preset show '<idx|name>' : 프리셋 내용 확인
 */
static const regval_list kPreset_1958[] = {
	{0x3210, 0x1E}, // DTAG_GH_SET_r
	{0x3211, 0x00}, // DTAG_GL_SET_r
	{0x3212, 0x07}, // DTAG_GR_r
	{0x3213, 0x1D}, // DTAG_GL_HLD_r
	{0x3214, 0x00}, // DTAG_DELAY_r(1)
	{0x3215, 0x00}, // DTAG_DELAY_r(2)

	{0x3216, 0x04}, // DTAG_SELX_r
	{0x3217, 0x02}, // DTAG_SENSE_r
	{0x3218, 0x0C}, // DTAG_AY_r
	{0x3219, 0x05}, // DTAG_AY_RST_GAP_r
	{0x321A, 0x07}, // DTAG_APS_RST_r
	{0x321C, 0x02}, // DTAG_COL_MARGIN_r

	{0x320C, 0x1F}, // [6] : DTAG_FREE_RUN_MODE_r, [5] : DTAG_MASK_FIRST_FRAME_r, [1] : DTAG_GRST_MODE_r, [0] : DTAG_GH_MODE_r
	//{0x320C, 0x5D},
	//{0x320C, 0x5F},
	{0x321D, 0x00}, // DTAG_FRM_MAGRIN_r_MSB
	//{0x321E, 0x03}, // DTAG_FRM_MAGRIN_r_LSB : 1LSB x 2^12 x Event Clock period
					// Scan Time             : # of Column x 92 x Evect Clock period
					// Margin                : 254 x Event Clock Period
					//		{0x321E, 0x0f},
	{0x321E, 0x02},
};
static const regval_list kPreset_3287[] = {
	{0x3210, 0x1E}, // DTAG_GH_SET_r
	{0x3211, 0x00}, // DTAG_GL_SET_r
	{0x3212, 0x07}, // DTAG_GR_r
	{0x3213, 0x1D}, // DTAG_GL_HLD_r
	{0x3214, 0x00}, // DTAG_DELAY_r(1)
	{0x3215, 0x00}, // DTAG_DELAY_r(2)

	{0x3216, 0x02}, // DTAG_SELX_r
	{0x3217, 0x01}, // DTAG_SENSE_r
	{0x3218, 0x00}, // DTAG_AY_r
	//{0x3218, 0x1C}, // DTAG_AY_r
	{0x3219, 0x00}, // DTAG_AY_RST_GAP_r
	{0x321A, 0x00}, // DTAG_APS_RST_r
	{0x321C, 0x02}, // DTAG_COL_MARGIN_r

	//{0x320C, 0x1F}, // [6] : DTAG_FREE_RUN_MODE_r, [5] : DTAG_MASK_FIRST_FRAME_r, [1] : DTAG_GRST_MODE_r, [0] : DTAG_GH_MODE_r
	//{0x320C, 0x5D}, 0b0101_1101
	//{0x320C, 0x5F},
	{0x320C, 0x1F},
	{0x321D, 0x00}, // DTAG_FRM_MAGRIN_r_MSB
	//{0x321E, 0x03}, // DTAG_FRM_MAGRIN_r_LSB : 1LSB x 2^12 x Event Clock period
					// Scan Time             : # of Column x 92 x Evect Clock period
					// Margin                : 254 x Event Clock Period
					//		{0x321E, 0x0f},
	{0x321E, 0x02},
};

typedef struct {
    const char     *name;
    const char     *desc;
    const regval_list *pairs;
    size_t          len;
} preset_def;

static const preset_def kPresets[] = {
    { "base1958",   "1958fps Base configs",
    		kPreset_1958,   ARRAY_LEN(kPreset_1958) },
    { "base3287",   "3287fps Base configs",
    		kPreset_3287, ARRAY_LEN(kPreset_3287) },
};

/* presets helper */
static void list_presets(void) {
    xil_printf("Available presets:\r\n");
    for (size_t i = 0; i < ARRAY_LEN(kPresets); ++i) {
        xil_printf("  [%d] %-12s (%u item%s) - %s\r\n",
                   (int)i, kPresets[i].name, (unsigned)kPresets[i].len,
                   (kPresets[i].len==1?"":"s"), kPresets[i].desc);
    }
}
static int find_preset_index_by_name(const char *name) {
    for (size_t i = 0; i < ARRAY_LEN(kPresets); ++i)
        if (ci_equals(kPresets[i].name, name)) return (int)i;
    return -1;
}
static void show_preset(int idx) {
    if (idx < 0 || (size_t)idx >= ARRAY_LEN(kPresets)) {
        xil_printf("ERR: preset index out of range.\r\n");
        return;
    }
    xil_printf("Preset [%d] %s (%u items): %s\r\n",
               idx, kPresets[idx].name, (unsigned)kPresets[idx].len, kPresets[idx].desc);
    print_reg_list(kPresets[idx].pairs, kPresets[idx].len);
}


/* ====== Base (default) configuration sets =================================
 * "defaults"로 사용될 전체 레지스터 집합
 * - base '<idx|name>'       : 해당 세팅을 현재 기본(defaults)으로 선택
 * - base show '<idx|name>'  : 세팅 내용 확인
 * - base current            : 현재 선택된 기본 세팅 정보 표시
 * - bases                   : 사용 가능한 세팅 목록 표시
 * sensor_cfgs 에서 가져옴
 */

typedef struct {
    const char        *name;
    const char        *desc;
    const regval_list *pairs;
    size_t             len;
} base_def;

enum { KBASES_MAX = 16 };
static base_def kBases[KBASES_MAX];
static size_t   kBases_cnt = 0;

static void bases_init_once(void) {
    static int inited = 0;
    if (inited) return;

    /* sensor_cfgs.h로부터 가져온 외부 기본 세팅을 런타임에 등록 */
    kBases[kBases_cnt++] = (base_def){ "default",  "default",
                                       DVS_regs,            length_DVS_regs };
    kBases[kBases_cnt++] = (base_def){ "1958fps",  "1958fps",
                                       DVS_regs_1958fps,    length_DVS_regs_1958fps };
    kBases[kBases_cnt++] = (base_def){ "3287fps",  "3287fps",
                                       DVS_regs_3287fps,    length_DVS_regs_3287fps };

    inited = 1;
}

static void list_bases(void) {
    bases_init_once();
    xil_printf("Available base (default) sets:\r\n");
    for (size_t i = 0; i < kBases_cnt; ++i) {
        xil_printf("  [%d] %-12s (%u item%s) - %s\r\n",
                   (int)i, kBases[i].name, (unsigned)kBases[i].len,
                   (kBases[i].len==1?"":"s"), kBases[i].desc);
    }
}
static int find_base_index_by_name(const char *name) {
    bases_init_once();
    for (size_t i = 0; i < kBases_cnt; ++i)
        if (ci_equals(kBases[i].name, name)) return (int)i;
    return -1;
}
static void show_base(int idx) {
    bases_init_once();
    if (idx < 0 || (size_t)idx >= kBases_cnt) {
        xil_printf("ERR: base index out of range.\r\n");
        return;
    }
    xil_printf("Base [%d] %s (%u items): %s\r\n",
               idx, kBases[idx].name, (unsigned)kBases[idx].len, kBases[idx].desc);
    print_reg_list(kBases[idx].pairs, kBases[idx].len);
}

/* Help text */
static void print_help(void) {
    xil_printf(
        "Commands:\r\n"
        "  add <pairs>     : Upsert pairs (e.g., add 3225:12 0166=31 0ABC 07)\r\n"
        "  del <addr|@idx> : Delete by hex address (4 digits) or index with @\r\n"
        "  info <what>     : Show register info/metadata\r\n"
        "                    - info 3225 0166      (one or more hex addresses)\r\n"
        "                    - info all            (all known entries in DB)\r\n"
        "                    - info defaults       (all addresses in defaults)\r\n"
        "                    - info overrides      (all addresses in overrides)\r\n"
        "  list            : Show current overrides\r\n"
        "  clear           : Remove all overrides\r\n"
        "  last            : USE last successful configuration (return immediately)\r\n"
        "  loadlast        : LOAD last overrides into editor (continue editing)\r\n"
        "  presets         : List available override presets\r\n"
        "  preset <id|name>            : MERGE preset into current overrides\r\n"
        "  preset set <id|name>        : REPLACE current overrides with preset\r\n"
        "  preset show <id|name>       : Show preset contents\r\n"
        "  bases           : List available BASE(default) sets\r\n"
        "  base <id|name>  : SELECT a BASE set as current defaults\r\n"
        "  base show <id|name> : Show a BASE set contents\r\n"
        "  base current    : Show currently selected BASE set\r\n"
        "  return|done|ok  : Build and return sensor_cfg (defaults in-place + append unknown + forced tail)\r\n"
        "  cancel|exit     : Abort without returning a config\r\n"
        "  help            : Show this help\r\n"
        "\r\n"
        "Tail order: specific registers are always executed last, in code-defined order.\r\n"
        "Pair formats (delimiters : , = space; 0x prefix allowed):\r\n"
        "  3225:12 0166=31 0ABC 07, 1234:FF\r\n"
        "Address = 4 hex digits; Value = 2 hex digits.\r\n"
    );
}

/* ===== Public API ===== */
int sensor_cfg_input(const regval_list *defaults, size_t defaults_len,
                     regval_list *out, size_t out_cap, size_t *out_len)
{
    regval_list overrides[SENSOR_CFG_MAX_OVERRIDES];
    size_t overrides_len = 0;

    /* 현재 활성 기본 세팅(초기값: 호출자가 넘긴 defaults) */
    const regval_list *active_def = defaults;
    size_t             active_def_len = defaults_len;
    const char        *active_def_name = "(caller-default)";

    xil_printf("\r\n=== sensor_cfg_input (UART) ===\r\n");
    if (g_last_cfg_len > 0) {
        xil_printf("Last configuration available: %u item(s). Type 'last' to use or 'loadlast' to edit.\r\n",
                   (unsigned)g_last_cfg_len);
    } else {
        xil_printf("No last configuration cached.\r\n");
    }

    /* 베이스 목록 초기화 및 안내 */
    bases_init_once();
    xil_printf("Active BASE(default): %s (%u item(s))\r\n", active_def_name, (unsigned)active_def_len);
    xil_printf("Type 'help' for commands. Enter raw pairs to implicitly add.\r\n");

    char line[256];
    for (;;) {
        xil_printf("\r\n> ");
        (void)uart_readline(line, sizeof(line));

        /* skip leading spaces */
        const char *p = line;
        while (*p && isspace((unsigned char)*p)) ++p;
        if (*p == '\0') continue;

        if (ci_starts_with(p, "help")) { print_help(); continue; }
        if (ci_starts_with(p, "list")) { print_overrides(overrides, overrides_len); continue; }
        if (ci_starts_with(p, "clear")){ overrides_len = 0; xil_printf("Overrides cleared.\r\n"); continue; }

        /* Use last merged configuration immediately */
        if (ci_starts_with(p, "last")) {
            if (g_last_cfg_len == 0) {
                xil_printf("No last configuration cached.\r\n");
                continue;
            }
            if (out_cap < g_last_cfg_len) {
                *out_len = 0;
                xil_printf("ERROR: output buffer too small for last configuration (%u needed).\r\n",
                           (unsigned)g_last_cfg_len);
                return SENSOR_CFG_ERR_CAP;
            }
            memcpy(out, g_last_cfg, g_last_cfg_len * sizeof(regval_list));
            *out_len = g_last_cfg_len;
            xil_printf("Returning LAST configuration (%u item(s)).\r\n", (unsigned)*out_len);
            return SENSOR_CFG_OK;
        }

        /* Load last overrides into editor (continue editing) */
        if (ci_starts_with(p, "loadlast")) {
            if (g_last_overrides_len == 0) {
                xil_printf("No last overrides cached.\r\n");
                continue;
            }
            size_t to_copy = g_last_overrides_len;
            if (to_copy > ARRAY_LEN(overrides)) {
                xil_printf("Last overrides exceed current capacity; truncating from %u to %u.\r\n",
                           (unsigned)to_copy, (unsigned)ARRAY_LEN(overrides));
                to_copy = ARRAY_LEN(overrides);
            }
            memcpy(overrides, g_last_overrides, to_copy * sizeof(regval_list));
            overrides_len = to_copy;
            xil_printf("Loaded last overrides (%u item(s)) into editor.\r\n", (unsigned)overrides_len);
            continue;
        }

        /* presets & preset command */
        if (ci_starts_with(p, "presets")) { list_presets(); continue; }

        if (ci_starts_with(p, "preset")) {
            /* skip 'preset' */
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            if (*p == '\0') {
                xil_printf("Usage: preset <id|name> | preset set <id|name> | preset show <id|name>\r\n");
                list_presets();
                continue;
            }

            int mode = 0; /* 0=merge, 1=set(replace), 2=show */
            if (ci_starts_with(p, "set")) { mode = 1; while (*p && !isspace((unsigned char)*p)) ++p; while (*p && isspace((unsigned char)*p)) ++p; }
            else if (ci_starts_with(p, "show")) { mode = 2; while (*p && !isspace((unsigned char)*p)) ++p; while (*p && isspace((unsigned char)*p)) ++p; }

            if (*p == '\0') {
                xil_printf("ERR: missing preset id or name.\r\n");
                continue;
            }

            /* read token (id or name) */
            char tok[64]; size_t ti = 0;
            while (*p && !isspace((unsigned char)*p) && ti + 1 < sizeof(tok)) tok[ti++] = *p++;
            tok[ti] = '\0';

            int idx = -1;
            int is_num = 1;
            for (size_t i = 0; i < ti; ++i) if (!isdigit((unsigned char)tok[i])) { is_num = 0; break; }
            if (is_num) idx = (int)strtol(tok, NULL, 10);
            else idx = find_preset_index_by_name(tok);

            if (idx < 0 || (size_t)idx >= ARRAY_LEN(kPresets)) {
                xil_printf("ERR: unknown preset '%s'. Try 'presets'.\r\n", tok);
                continue;
            }

            if (mode == 2) { /* show */
                xil_printf("Preset preview:\r\n");
                show_preset(idx);
                continue;
            }

            if (mode == 1) { /* set/replace */
                overrides_len = 0;
            }
            /* merge (upsert) the preset into overrides */
            int ur = upsert_pairs(overrides, &overrides_len, ARRAY_LEN(overrides),
                                  kPresets[idx].pairs, kPresets[idx].len);
            if (ur != SENSOR_CFG_OK) {
                xil_printf("ERROR: overrides capacity exceeded while applying preset '%s'.\r\n",
                           kPresets[idx].name);
                continue;
            }
            xil_printf("Applied preset '%s' (%u item(s)) with %s.\r\n",
                       kPresets[idx].name, (unsigned)kPresets[idx].len,
                       (mode==1?"REPLACE":"MERGE"));
            continue;
        }

        /* bases (default set) */
        if (ci_starts_with(p, "bases")) { list_bases(); continue; }

        if (ci_starts_with(p, "base")) {
            /* skip 'base' */
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            if (*p == '\0') {
                xil_printf("Usage: base <id|name> | base show <id|name> | base current\r\n");
                list_bases();
                continue;
            }

            if (ci_starts_with(p, "current")) {
                xil_printf("Current BASE: %s (%u item(s))\r\n", active_def_name, (unsigned)active_def_len);
                continue;
            }

            int show_only = 0;
            if (ci_starts_with(p, "show")) {
                show_only = 1;
                while (*p && !isspace((unsigned char)*p)) ++p;
                while (*p && isspace((unsigned char)*p)) ++p;
            }

            if (*p == '\0') {
                xil_printf("ERR: missing base id or name.\r\n");
                continue;
            }

            char tok[64]; size_t ti = 0;
            while (*p && !isspace((unsigned char)*p) && ti + 1 < sizeof(tok)) tok[ti++] = *p++;
            tok[ti] = '\0';

            int idx = -1;
            int is_num = 1;
            for (size_t i = 0; i < ti; ++i) if (!isdigit((unsigned char)tok[i])) { is_num = 0; break; }
            if (is_num) idx = (int)strtol(tok, NULL, 10);
            else idx = find_base_index_by_name(tok);

            if (idx < 0 || (size_t)idx >= kBases_cnt) {
                xil_printf("ERR: unknown base '%s'. Try 'bases'.\r\n", tok);
                continue;
            }

            if (show_only) {
                xil_printf("BASE preview:\r\n");
                show_base(idx);
                continue;
            }

            /* 선택된 BASE를 현재 defaults로 사용 */
            active_def = kBases[idx].pairs;
            active_def_len = kBases[idx].len;
            active_def_name = kBases[idx].name;
            xil_printf("Selected BASE '%s' (%u item(s)) as current defaults.\r\n",
                       active_def_name, (unsigned)active_def_len);
            continue;
        }

        if (ci_starts_with(p, "cancel") || ci_starts_with(p, "exit")) {
            xil_printf("Aborted by user.\r\n");
            if (out_len) *out_len = 0;
            return SENSOR_CFG_ABORTED;
        }

        if (ci_starts_with(p, "return") || ci_starts_with(p, "done") || ci_starts_with(p, "ok")) {
            /* Build and return merged configuration — defaults in-place, append unknown, force tail last */
            size_t merged_len = 0;
            int rc = build_regset_tail_ordered(active_def, active_def_len,
                                               overrides, overrides_len,
                                               out, out_cap, &merged_len);
            *out_len = merged_len;
            if (rc == SENSOR_CFG_OK) {
                xil_printf("Merged %d item(s) with tail-ordered execution on BASE '%s'. Returning sensor_cfg.\r\n",
                           (int)merged_len, active_def_name);

                /* Update caches */
                if (merged_len <= ARRAY_LEN(g_last_cfg)) {
                    memcpy(g_last_cfg, out, merged_len * sizeof(regval_list));
                    g_last_cfg_len = merged_len;
                } else {
                    xil_printf("WARN: last-cfg cache too small; not cached.\r\n");
                    g_last_cfg_len = 0;
                }
                if (overrides_len <= ARRAY_LEN(g_last_overrides)) {
                    memcpy(g_last_overrides, overrides, overrides_len * sizeof(regval_list));
                    g_last_overrides_len = overrides_len;
                } else {
                    xil_printf("WARN: last-overrides cache too small; not cached.\r\n");
                    g_last_overrides_len = 0;
                }
                return SENSOR_CFG_OK;
            } else if (rc == SENSOR_CFG_ERR_CAP) {
                xil_printf("ERROR: output capacity insufficient (built %d; need >= %d)\r\n",
                           (int)merged_len, (int)(active_def_len + overrides_len));
                return SENSOR_CFG_ERR_CAP;
            } else {
                xil_printf("ERROR: merge failed.\r\n");
                return rc;
            }
        }

        if (ci_starts_with(p, "add")) {
            /* skip command token */
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            regval_list tmp[128]; size_t tmp_len = 0;
            int pr = parse_pairs(p, tmp, ARRAY_LEN(tmp), &tmp_len);
            if (pr != SENSOR_CFG_OK) {
                xil_printf("Parse error. Example: add 3225:12 0166=31 0ABC 07\r\n");
                continue;
            }

            int ur = upsert_pairs(overrides, &overrides_len, ARRAY_LEN(overrides), tmp, tmp_len);
            if (ur != SENSOR_CFG_OK) {
                xil_printf("ERROR: overrides capacity exceeded (%d items)\r\n", (int)overrides_len);
                continue;
            }

            /* Optional hints: where will unknowns land? */
            for (size_t i = 0; i < tmp_len; ++i) {
                int in_def = (find_addr_index(active_def, active_def_len, tmp[i].Address) >= 0);
                if (!in_def) {
                    xil_printf("NOTE: 0x%04X not in defaults; will be appended%s.\r\n",
                               tmp[i].Address, is_tail_addr(tmp[i].Address) ? " at the very end (tail)" : "");
                }
            }

            xil_printf("Upserted %d pair(s). Now %d total.\r\n", (int)tmp_len, (int)overrides_len);
            continue;
        }

        if (ci_starts_with(p, "del")) {
            /* Syntax: del 0166 @3 ... */
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            int any = 0;
            while (*p) {
                while (*p && isspace((unsigned char)*p)) ++p;
                if (!*p) break;

                if (*p == '@') {
                    ++p;
                    int idx = 0, seen = 0;
                    while (isdigit((unsigned char)*p)) { idx = idx*10 + (*p - '0'); ++p; seen = 1; }
                    if (!seen) { xil_printf("ERR: '@' must be followed by index\r\n"); break; }
                    int rem = delete_by_index(overrides, &overrides_len, (size_t)idx);
                    xil_printf("del @%d -> %s\r\n", idx, rem ? "removed" : "no such index");
                    any = 1;
                } else {
                    if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;
                    int cnt = 0; uint16_t addr = 0;
                    while (is_hex_char(*p) && cnt < 4) {
                        int nib = hex_nibble(*p++); if (nib < 0) break;
                        addr = (uint16_t)((addr<<4)|(uint16_t)nib);
                        ++cnt;
                    }
                    if (cnt != 4) { xil_printf("ERR: need 4 hex digits for address\r\n"); break; }
                    int rem = delete_by_address(overrides, &overrides_len, addr);
                    xil_printf("del 0x%04X -> removed %d\r\n", addr, rem);
                    any = 1;
                }
                while (*p && !isalnum((unsigned char)*p) && *p!='@' && *p!='0') ++p;
            }
            if (!any) xil_printf("Usage: del <addr|@idx> [...]\r\n");
            continue;
        }

        if (ci_starts_with(p, "info")) {
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            if (*p == '\0') {
                xil_printf("Usage:\r\n");
                xil_printf("  info 3225 0166      (hex addresses)\r\n");
                xil_printf("  info all | defaults | overrides\r\n");
                continue;
            }

            if (ci_starts_with(p, "all")) {
                xil_printf("Register info (all known DB entries):\r\n");
                for (size_t i = 0; i < ARRAY_LEN(kRegDB); ++i)
                    print_reg_info_addr(kRegDB[i].addr, active_def, active_def_len, overrides, overrides_len);
                continue;
            }
            if (ci_starts_with(p, "defaults")) {
                xil_printf("Register info (defaults):\r\n");
                for (size_t i = 0; i < active_def_len; ++i)
                    print_reg_info_addr(active_def[i].Address, active_def, active_def_len, overrides, overrides_len);
                continue;
            }
            if (ci_starts_with(p, "overrides")) {
                xil_printf("Register info (overrides):\r\n");
                for (size_t i = 0; i < overrides_len; ++i)
                    print_reg_info_addr(overrides[i].Address, active_def, active_def_len, overrides, overrides_len);
                continue;
            }

            int shown = 0;
            while (*p) {
                while (*p && isspace((unsigned char)*p)) ++p;
                if (!*p) break;

                if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;

                int cnt = 0; uint16_t addr = 0;
                while (is_hex_char(*p) && cnt < 4) {
                    int nib = hex_nibble(*p++); if (nib < 0) break;
                    addr = (uint16_t)((addr<<4)|(uint16_t)nib);
                    ++cnt;
                }
                if (cnt != 4) { xil_printf("ERR: need 4 hex digits (e.g., 3225)\r\n"); break; }

                print_reg_info_addr(addr, active_def, active_def_len, overrides, overrides_len);
                shown = 1;

                while (*p && !isalnum((unsigned char)*p) && *p!='0') ++p;
            }
            if (!shown)
                xil_printf("Usage: info <addr...> | info all | info defaults | info overrides\r\n");
            continue;
        }

        /* If it looks like pairs, treat as implicit 'add' */
        if (is_hex_char(*p) || *p=='0') {
            regval_list tmp[128]; size_t tmp_len = 0;
            int pr = parse_pairs(p, tmp, ARRAY_LEN(tmp), &tmp_len);
            if (pr == SENSOR_CFG_OK && tmp_len > 0) {
                int ur = upsert_pairs(overrides, &overrides_len, ARRAY_LEN(overrides), tmp, tmp_len);
                if (ur != SENSOR_CFG_OK) {
                    xil_printf("ERROR: overrides capacity exceeded (%d items)\r\n", (int)overrides_len);
                    continue;
                }
                /* Optional hints */
                for (size_t i = 0; i < tmp_len; ++i) {
                    int in_def = (find_addr_index(active_def, active_def_len, tmp[i].Address) >= 0);
                    if (!in_def) {
                        xil_printf("NOTE: 0x%04X not in defaults; will be appended%s.\r\n",
                                   tmp[i].Address, is_tail_addr(tmp[i].Address) ? " at the very end (tail)" : "");
                    }
                }
                xil_printf("Upserted %d pair(s). Now %d total.\r\n", (int)tmp_len, (int)overrides_len);
                continue;
            }
        }

        xil_printf("Unknown input. Type 'help' for usage.\r\n");
    }
}



#ifdef legacy_250925

#include "sensor_cfg_input.h"
#include "xuartps_hw.h"
#include "xil_printf.h"
#include <string.h>
#include <ctype.h>

/* ====== Small helpers / macros ====== */
#ifndef ARRAY_LEN
#define ARRAY_LEN(a) (sizeof(a)/sizeof((a)[0]))
#endif

static inline int is_hex_char(char c) {
    return (c >= '0' && c <= '9') ||
           (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}
static inline int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}
static int ci_starts_with(const char *s, const char *pfx) {
    while (*pfx) {
        char a = *s ? (char)tolower((unsigned char)*s) : 0;
        char b = (char)tolower((unsigned char)*pfx);
        if (a != b) return 0;
        ++s; ++pfx;
    }
    return 1;
}

/* ---- Last-used configuration cache (RAM only) ------------------------- */
static regval_list g_last_cfg[SENSOR_CFG_WORKBUF_CAP];
static size_t      g_last_cfg_len = 0;
/* Keep last overrides too so user can reload for editing */
static regval_list g_last_overrides[SENSOR_CFG_MAX_OVERRIDES];
static size_t      g_last_overrides_len = 0;

/* ===== Register metadata ===== */
typedef struct reg_meta {
    uint16_t addr;
    const char *name;    /* short name */
    const char *brief;   /* what this register does */
    const char *notes;   /* access/reset/range or any extra notes */
    // TODO: default cfg values, experiment logs
} reg_meta;

static const reg_meta kRegDB[] = {
    /* examples
    {0x3225, "FRAME_CTRL", "Frame timing/control",          "Reset: 0x12, RW"},
    {0x0166, "GAIN_MODE",  "Analog/digital gain selection", "Reset: 0x31, RW"},
    {0x0ABC, "XXX", "desc", "notes"}, ... */

    {0x320C, "DTAG_GRST_MODE_r",
            "[6] : DTAG_FREE_RUN_MODE_r, [5] : DTAG_MASK_FIRST_FRAME_r, [1] : DTAG_GRST_MODE_r, [0] : DTAG_GH_MODE_r",
            "memo: 0x5D"},

    {0x3216, "DTAG_SELX_r",
            "",
            "memo: 0x02"},
    {0x3217, "DTAG_SENSE_r",
            "",
            "memo: 0x01"},
    {0x3218, "DTAG_AY_r",
            "",
            "memo: 0x00"},
    {0x3219, "DTAG_AY_RST_GAP_r",
            "",
            "memo: 0x00"},
    {0x321A, "DTAG_APS_RST_r",
            "",
            "memo: 0x00"},
    {0x321C, "DTAG_COL_MARGIN_r",
            "",
            "memo: 0x02"},

    {0x321D, "DTAG_FRM_MAGRIN_r_MSB",
            "--",
            "memo: 0x00"},
    {0x321E, "DTAG_FRM_MAGRIN_r_LSB",
            "--",
            "memo: 0x02"},
};

static const reg_meta* regmeta_lookup(uint16_t addr) {
    for (size_t i = 0; i < ARRAY_LEN(kRegDB); ++i)
        if (kRegDB[i].addr == addr) return &kRegDB[i];
    return NULL;
}

/* Find address index in arr[0..len). Returns -1 if not found. */
static int find_addr_index(const regval_list *arr, size_t len, uint16_t addr) {
    for (size_t i = 0; i < len; ++i){
        if (arr[i].Address == addr) return (int)i;
    }
    return -1;
}

/* Pretty-print one address with metadata + default/override/effective values */
static void print_reg_info_addr(uint16_t addr,
                                const regval_list *defaults, size_t defaults_len,
                                const regval_list *overrides, size_t overrides_len)
{
    const reg_meta *m = regmeta_lookup(addr);
    int idx_def = find_addr_index(defaults, defaults_len, addr);
    int idx_ovr = find_addr_index(overrides, overrides_len, addr);

    const char *nm   = m ? m->name  : "(unknown)";
    const char *brf  = m ? m->brief : "No metadata available";
    const char *note = m ? m->notes : "-";

    xil_printf("0x%04X  %-12s  %s\r\n", addr, nm, brf);
    xil_printf("    notes      : %s\r\n", note);
    if (idx_def >= 0) xil_printf("    default    : 0x%02X\r\n", defaults[idx_def].Data);
    else              xil_printf("    default    : (N/A)\r\n");
    if (idx_ovr >= 0) xil_printf("    override   : 0x%02X\r\n", overrides[idx_ovr].Data);
    else              xil_printf("    override   : (none)\r\n");

    if (idx_ovr >= 0) xil_printf("    effective  : 0x%02X (override)\r\n", overrides[idx_ovr].Data);
    else if (idx_def >= 0) xil_printf("    effective  : 0x%02X (default)\r\n", defaults[idx_def].Data);
    else xil_printf("    effective  : (undefined in current sets)\r\n");
}

/* UART line input (blocking; echoes; backspace supported). */
static size_t uart_readline(char *buf, size_t cap) {
    if (cap == 0) return 0;
    size_t n = 0;
    for (;;) {
        char c = (char)XUartPs_RecvByte(UART_BASEADDR);
        if (c == '\r' || c == '\n') {
            XUartPs_SendByte(UART_BASEADDR, '\r');
            XUartPs_SendByte(UART_BASEADDR, '\n');
            break;
        } else if (c == 0x08 || c == 0x7F) { /* backspace */
            if (n > 0) {
                XUartPs_SendByte(UART_BASEADDR, 0x08);
                XUartPs_SendByte(UART_BASEADDR, ' ');
                XUartPs_SendByte(UART_BASEADDR, 0x08);
                --n;
            }
        } else {
            XUartPs_SendByte(UART_BASEADDR, (uint8_t)c);
            if (n + 1 < cap) buf[n++] = c; /* keep room for '\0' */
        }
    }
    buf[n] = '\0';
    return n;
}

/* Parse "AAAA:DD 0166=31 0ABC 07, ..." into out[]. 0 on success, <0 on error. */
static int parse_pairs(const char *line,
                       regval_list *out, size_t out_cap, size_t *out_len) {
    size_t n = 0; const char *p = line;
    while (*p) {
        /* seek hex start */
        while (*p && !is_hex_char(*p) && *p != '0') ++p;
        if (!*p) break;

        /* optional 0x prefix for address */
        if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;

        /* address: exactly 4 hex digits */
        int cnt = 0; uint16_t addr = 0;
        while (is_hex_char(*p) && cnt < 4) {
            int nib = hex_nibble(*p++); if (nib < 0) return SENSOR_CFG_ERR_PARSE;
            addr = (uint16_t)((addr << 4) | (uint16_t)nib); ++cnt;
        }
        if (cnt == 0) break;
        if (cnt != 4) return SENSOR_CFG_ERR_PARSE;

        /* skip to value start */
        while (*p && !is_hex_char(*p) && *p != '0') ++p;
        if (!*p) return SENSOR_CFG_ERR_PARSE;

        /* optional 0x prefix for value */
        if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;

        /* value: exactly 2 hex digits */
        cnt = 0; uint16_t data = 0;
        while (is_hex_char(*p) && cnt < 2) {
            int nib = hex_nibble(*p++); if (nib < 0) return SENSOR_CFG_ERR_PARSE;
            data = (uint16_t)((data << 4) | (uint16_t)nib); ++cnt;
        }
        if (cnt != 2) return SENSOR_CFG_ERR_PARSE;

        if (n >= out_cap) return SENSOR_CFG_ERR_CAP;
        out[n].Address = addr;
        out[n].Data    = (uint8_t)data;
        ++n;

        while (*p && !is_hex_char(*p) && *p != '0') ++p; /* skip separators */
    }
    *out_len = n;
    return SENSOR_CFG_OK;
}

/* Upsert: update if address exists; append if new. */
static int upsert_pairs(regval_list *dst, size_t *dst_len, size_t dst_cap,
                        const regval_list *src, size_t src_len) {
    size_t n = *dst_len;
    for (size_t i = 0; i < src_len; ++i) {
        int idx = find_addr_index(dst, n, src[i].Address);
        if (idx >= 0) {
            dst[idx].Data = src[i].Data;
        } else {
            if (n >= dst_cap) return SENSOR_CFG_ERR_CAP;
            dst[n++] = src[i];
        }
    }
    *dst_len = n;
    return SENSOR_CFG_OK;
}

/* Delete by hex address; returns removed count. */
static int delete_by_address(regval_list *arr, size_t *len, uint16_t addr) {
    size_t n = *len, w = 0; int removed = 0;
    for (size_t i = 0; i < n; ++i) {
        if (arr[i].Address == addr) { ++removed; continue; }
        if (w != i) arr[w] = arr[i];
        ++w;
    }
    *len = w;
    return removed;
}

/* Delete by index; returns 1 if removed, 0 if invalid index. */
static int delete_by_index(regval_list *arr, size_t *len, size_t idx) {
    if (idx >= *len) return 0;
    for (size_t i = idx + 1; i < *len; ++i) arr[i-1] = arr[i];
    --(*len);
    return 1;
}

/* (Unused now) Validate overrides: check duplicates; values are already typed. */
static int validate_overrides(const regval_list *arr, size_t len) {
    int ok = 1;
    for (size_t i = 0; i < len; ++i)
        for (size_t j = i + 1; j < len; ++j)
            if (arr[i].Address == arr[j].Address) {
                xil_printf("WARN: duplicate address 0x%04X at [%d] and [%d]\r\n",
                           arr[i].Address, (int)i, (int)j);
                ok = 0;
            }
    if (ok) xil_printf("Validation: OK (no duplicates)\r\n");
    return ok ? SENSOR_CFG_OK : SENSOR_CFG_ERR_PARSE;
}

/* Merge defaults + overrides into out[]. */
static int build_regset(const regval_list *def, size_t def_len,
                        const regval_list *ovr, size_t ovr_len,
                        regval_list *out, size_t out_cap, size_t *out_len) {
    if (out_cap < def_len) {
        memcpy(out, def, out_cap * sizeof(regval_list));
        *out_len = out_cap;
        return SENSOR_CFG_ERR_CAP;
    }
    memcpy(out, def, def_len * sizeof(regval_list));
    size_t n = def_len;

    for (size_t i = 0; i < ovr_len; ++i) {
        int idx = find_addr_index(out, n, ovr[i].Address);
        if (idx >= 0) {
            out[idx].Data = ovr[i].Data;
        } else {
            if (n >= out_cap) { *out_len = n; return SENSOR_CFG_ERR_CAP; }
            out[n++] = ovr[i];
        }
    }
    *out_len = n;
    return SENSOR_CFG_OK;
}

/* Pretty-print current overrides. */
static void print_overrides(const regval_list *arr, size_t len) {
    xil_printf("Overrides (%d item%s):\r\n", (int)len, (len==1?"":"s"));
    for (size_t i = 0; i < len; ++i)
        xil_printf("  [%03d] 0x%04X <- 0x%02X\r\n",
                   (int)i, arr[i].Address, arr[i].Data);
}

/* Help text */
static void print_help(void) {
    xil_printf(
        "Commands:\r\n"
        "  add <pairs>     : Upsert pairs (e.g., add 3225:12 0166=31 0ABC 07)\r\n"
        "  del <addr|@idx> : Delete by hex address (4 digits) or index with @\r\n"
        "  info <what>     : Show register info/metadata\r\n"
        "                    - info 3225 0166      (one or more hex addresses)\r\n"
        "                    - info all            (all known entries in DB)\r\n"
        "                    - info defaults       (all addresses in defaults)\r\n"
        "                    - info overrides      (all addresses in overrides)\r\n"
        "  list            : Show current overrides\r\n"
        "  clear           : Remove all overrides\r\n"
        "  last            : USE last successful configuration (return immediately)\r\n"
        "  loadlast        : LOAD last overrides into editor (continue editing)\r\n"
        "  return|done|ok  : Build and return merged sensor_cfg\r\n"
        "  cancel|exit     : Abort without returning a config\r\n"
        "  help            : Show this help\r\n"
        "\r\n"
        "Pair formats (delimiters : , = space; 0x prefix allowed):\r\n"
        "  3225:12 0166=31 0ABC 07, 1234:FF\r\n"
        "Address = 4 hex digits; Value = 2 hex digits.\r\n"
    );
}

/* ===== Public API ===== */
int sensor_cfg_input(const regval_list *defaults, size_t defaults_len,
                     regval_list *out, size_t out_cap, size_t *out_len)
{
    regval_list overrides[SENSOR_CFG_MAX_OVERRIDES];
    size_t overrides_len = 0;

    xil_printf("\r\n=== sensor_cfg_input (UART) ===\r\n");
    if (g_last_cfg_len > 0) {
        xil_printf("Last configuration available: %u item(s). Type 'last' to use or 'loadlast' to edit.\r\n",
                   (unsigned)g_last_cfg_len);
    } else {
        xil_printf("No last configuration cached.\r\n");
    }
    xil_printf("Type 'help' for commands. Enter raw pairs to implicitly add.\r\n");

    char line[256];
    for (;;) {
        xil_printf("\r\n> ");
        (void)uart_readline(line, sizeof(line));

        /* skip leading spaces */
        const char *p = line;
        while (*p && isspace((unsigned char)*p)) ++p;
        if (*p == '\0') continue;

        if (ci_starts_with(p, "help")) { print_help(); continue; }
        if (ci_starts_with(p, "list")) { print_overrides(overrides, overrides_len); continue; }
        if (ci_starts_with(p, "clear")){ overrides_len = 0; xil_printf("Overrides cleared.\r\n"); continue; }

        /* Use last merged configuration immediately */
        if (ci_starts_with(p, "last")) {
            if (g_last_cfg_len == 0) {
                xil_printf("No last configuration cached.\r\n");
                continue;
            }
            if (out_cap < g_last_cfg_len) {
                *out_len = 0;
                xil_printf("ERROR: output buffer too small for last configuration (%u needed).\r\n",
                           (unsigned)g_last_cfg_len);
                return SENSOR_CFG_ERR_CAP;
            }
            memcpy(out, g_last_cfg, g_last_cfg_len * sizeof(regval_list));
            *out_len = g_last_cfg_len;
            xil_printf("Returning LAST configuration (%u item(s)).\r\n", (unsigned)*out_len);
            return SENSOR_CFG_OK;
        }

        /* Load last overrides into editor (continue editing) */
        if (ci_starts_with(p, "loadlast")) {
            if (g_last_overrides_len == 0) {
                xil_printf("No last overrides cached.\r\n");
                continue;
            }
            size_t to_copy = g_last_overrides_len;
            if (to_copy > ARRAY_LEN(overrides)) {
                xil_printf("Last overrides exceed current capacity; truncating from %u to %u.\r\n",
                           (unsigned)to_copy, (unsigned)ARRAY_LEN(overrides));
                to_copy = ARRAY_LEN(overrides);
            }
            memcpy(overrides, g_last_overrides, to_copy * sizeof(regval_list));
            overrides_len = to_copy;
            xil_printf("Loaded last overrides (%u item(s)) into editor.\r\n", (unsigned)overrides_len);
            continue;
        }

        if (ci_starts_with(p, "cancel") || ci_starts_with(p, "exit")) {
            xil_printf("Aborted by user.\r\n");
            if (out_len) *out_len = 0;
            return SENSOR_CFG_ABORTED;
        }
        if (ci_starts_with(p, "return") || ci_starts_with(p, "done") || ci_starts_with(p, "ok")) {
            /* Build and return merged configuration */
            size_t merged_len = 0;
            int rc = build_regset(defaults, defaults_len,
                                  overrides, overrides_len,
                                  out, out_cap, &merged_len);
            *out_len = merged_len;
            if (rc == SENSOR_CFG_OK) {
                xil_printf("Merged %d item(s). Returning sensor_cfg.\r\n", (int)merged_len);

                /* Update last-cfg cache */
                if (merged_len <= ARRAY_LEN(g_last_cfg)) {
                    memcpy(g_last_cfg, out, merged_len * sizeof(regval_list));
                    g_last_cfg_len = merged_len;
                } else {
                    xil_printf("WARN: last-cfg cache too small; not cached.\r\n");
                    g_last_cfg_len = 0;
                }
                /* Update last-overrides cache */
                if (overrides_len <= ARRAY_LEN(g_last_overrides)) {
                    memcpy(g_last_overrides, overrides, overrides_len * sizeof(regval_list));
                    g_last_overrides_len = overrides_len;
                } else {
                    xil_printf("WARN: last-overrides cache too small; not cached.\r\n");
                    g_last_overrides_len = 0;
                }

                return SENSOR_CFG_OK;
            } else {
                xil_printf("ERROR: output capacity insufficient (built %d)\r\n", (int)merged_len);
                return SENSOR_CFG_ERR_CAP;
            }
        }
        if (ci_starts_with(p, "add")) {
            /* skip command token */
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            regval_list tmp[128]; size_t tmp_len = 0;
            int pr = parse_pairs(p, tmp, ARRAY_LEN(tmp), &tmp_len);
            if (pr != SENSOR_CFG_OK) {
                xil_printf("Parse error. Example: add 3225:12 0166=31 0ABC 07\r\n");
                continue;
            }
            int ur = upsert_pairs(overrides, &overrides_len, ARRAY_LEN(overrides), tmp, tmp_len);
            if (ur != SENSOR_CFG_OK) {
                xil_printf("ERROR: overrides capacity exceeded (%d items)\r\n", (int)overrides_len);
                continue;
            }
            xil_printf("Upserted %d pair(s). Now %d total.\r\n", (int)tmp_len, (int)overrides_len);
            continue;
        }
        if (ci_starts_with(p, "del")) {
            /* Syntax: del 0166 @3 ... */
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            int any = 0;
            while (*p) {
                while (*p && isspace((unsigned char)*p)) ++p;
                if (!*p) break;

                if (*p == '@') {
                    ++p;
                    int idx = 0, seen = 0;
                    while (isdigit((unsigned char)*p)) { idx = idx*10 + (*p - '0'); ++p; seen = 1; }
                    if (!seen) { xil_printf("ERR: '@' must be followed by index\r\n"); break; }
                    int rem = delete_by_index(overrides, &overrides_len, (size_t)idx);
                    xil_printf("del @%d -> %s\r\n", idx, rem ? "removed" : "no such index");
                    any = 1;
                } else {
                    /* delete by hex address (allow 0x prefix) */
                    if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;
                    int cnt = 0; uint16_t addr = 0;
                    while (is_hex_char(*p) && cnt < 4) {
                        int nib = hex_nibble(*p++); if (nib < 0) break;
                        addr = (uint16_t)((addr<<4)|(uint16_t)nib);
                        ++cnt;
                    }
                    if (cnt != 4) { xil_printf("ERR: need 4 hex digits for address\r\n"); break; }
                    int rem = delete_by_address(overrides, &overrides_len, addr);
                    xil_printf("del 0x%04X -> removed %d\r\n", addr, rem);
                    any = 1;
                }
                while (*p && !isalnum((unsigned char)*p) && *p!='@' && *p!='0') ++p; /* next token */
            }
            if (!any) xil_printf("Usage: del <addr|@idx> [...]\r\n");
            continue;
        }
        if (ci_starts_with(p, "info")) {
            /* Skip command token */
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            if (*p == '\0') {
                xil_printf("Usage:\r\n");
                xil_printf("  info 3225 0166      (hex addresses)\r\n");
                xil_printf("  info all | defaults | overrides\r\n");
                continue;
            }

            /* Handle keywords: all / defaults / overrides */
            if (ci_starts_with(p, "all")) {
                xil_printf("Register info (all known DB entries):\r\n");
                for (size_t i = 0; i < ARRAY_LEN(kRegDB); ++i) {
                    print_reg_info_addr(kRegDB[i].addr, defaults, defaults_len, overrides, overrides_len);
                }
                continue;
            }
            if (ci_starts_with(p, "defaults")) {
                xil_printf("Register info (defaults):\r\n");
                for (size_t i = 0; i < defaults_len; ++i) {
                    print_reg_info_addr(defaults[i].Address, defaults, defaults_len, overrides, overrides_len);
                }
                continue;
            }
            if (ci_starts_with(p, "overrides")) {
                xil_printf("Register info (overrides):\r\n");
                for (size_t i = 0; i < overrides_len; ++i) {
                    print_reg_info_addr(overrides[i].Address, defaults, defaults_len, overrides, overrides_len);
                }
                continue;
            }

            /* Otherwise, parse one or more hex addresses from the rest of the line */
            int shown = 0;
            while (*p) {
                while (*p && isspace((unsigned char)*p)) ++p;
                if (!*p) break;

                /* optional 0x */
                if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;

                int cnt = 0; uint16_t addr = 0;
                while (is_hex_char(*p) && cnt < 4) {
                    int nib = hex_nibble(*p++); if (nib < 0) break;
                    addr = (uint16_t)((addr<<4)|(uint16_t)nib);
                    ++cnt;
                }
                if (cnt != 4) {
                    xil_printf("ERR: need 4 hex digits for address (e.g., 3225)\r\n");
                    break;
                }

                print_reg_info_addr(addr, defaults, defaults_len, overrides, overrides_len);
                shown = 1;

                /* Skip non-token chars to next token */
                while (*p && !isalnum((unsigned char)*p) && *p!='0') ++p;
            }
            if (!shown) {
                xil_printf("Usage: info <addr...> | info all | info defaults | info overrides\r\n");
            }
            continue;
        }

        /* If it looks like pairs, treat as implicit 'add' */
        if (is_hex_char(*p) || *p=='0') {
            regval_list tmp[128]; size_t tmp_len = 0;
            int pr = parse_pairs(p, tmp, ARRAY_LEN(tmp), &tmp_len);
            if (pr == SENSOR_CFG_OK && tmp_len > 0) {
                int ur = upsert_pairs(overrides, &overrides_len, ARRAY_LEN(overrides), tmp, tmp_len);
                if (ur != SENSOR_CFG_OK) {
                    xil_printf("ERROR: overrides capacity exceeded (%d items)\r\n", (int)overrides_len);
                    continue;
                }
                xil_printf("Upserted %d pair(s). Now %d total.\r\n", (int)tmp_len, (int)overrides_len);
                continue;
            }
        }

        xil_printf("Unknown input. Type 'help' for usage.\r\n");
    }
}

#endif


#ifdef legacy_250923

#include "sensor_cfg_input.h"
#include "xuartps_hw.h"
#include "xil_printf.h"
#include <string.h>
#include <ctype.h>

/* ====== Small helpers ====== */
static inline int is_hex_char(char c) {
    return (c >= '0' && c <= '9') ||
           (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}
static inline int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}
static int ci_starts_with(const char *s, const char *pfx) {
    while (*pfx) {
        char a = *s ? (char)tolower((unsigned char)*s) : 0;
        char b = (char)tolower((unsigned char)*pfx);
        if (a != b) return 0;
        ++s; ++pfx;
    }
    return 1;
}

/* ===== Register metadata ===== */
typedef struct reg_meta {
    uint16_t addr;
    const char *name;    /* short name */
    const char *brief;   /* what this register does */
    const char *notes;   /* access/reset/range or any extra notes */
    // TODO: defalut cfg values, experiment logs
} reg_meta;


static const reg_meta kRegDB[] = {
    /* examples
    {0x3225, "FRAME_CTRL", "Frame timing/control",          "Reset: 0x12, RW"},
    {0x0166, "GAIN_MODE",  "Analog/digital gain selection", "Reset: 0x31, RW"},
    {0x0ABC, "XXX", "desc", "notes"}, ... */

	{0x320C, "DTAG_GRST_MODE_r",
			"[6] : DTAG_FREE_RUN_MODE_r, [5] : DTAG_MASK_FIRST_FRAME_r, [1] : DTAG_GRST_MODE_r, [0] : DTAG_GH_MODE_r",
			"memo: 0x5D"},

	{0x3216, "DTAG_SELX_r",
			"",
			"memo: 0x02"},
	{0x3217, "DTAG_SENSE_r",
			"",
			"memo: 0x01"},
	{0x3218, "DTAG_AY_r",
			"",
			"memo: 0x00"},
	{0x3219, "DTAG_AY_RST_GAP_r",
			"",
			"memo: 0x00"},
	{0x321A, "DTAG_APS_RST_r",
			"",
			"memo: 0x00"},
	{0x321C, "DTAG_COL_MARGIN_r",
			"",
			"memo: 0x02"},

	{0x321D, "DTAG_FRM_MAGRIN_r_MSB",
			"--",
			"memo: 0x00"},
	{0x321E, "DTAG_FRM_MAGRIN_r_LSB",
			"--",
			"memo: 0x02"},
};

static const reg_meta* regmeta_lookup(uint16_t addr) {
    for (size_t i = 0; i < ARRAY_LEN(kRegDB); ++i)
        if (kRegDB[i].addr == addr) return &kRegDB[i];
    return NULL;
}

/* Find address index in arr[0..len). Returns -1 if not found. */
static int find_addr_index(const regval_list *arr, size_t len, uint16_t addr) {
    for (size_t i = 0; i < len; ++i){
        if (arr[i].Address == addr) return (int)i;
    }
    return -1;
}

/* Pretty-print one address with metadata + default/override/effective values */
static void print_reg_info_addr(uint16_t addr,
                                const regval_list *defaults, size_t defaults_len,
                                const regval_list *overrides, size_t overrides_len)
{
    const reg_meta *m = regmeta_lookup(addr);
    int idx_def = find_addr_index(defaults, defaults_len, addr);
    int idx_ovr = find_addr_index(overrides, overrides_len, addr);

    const char *nm   = m ? m->name  : "(unknown)";
    const char *brf  = m ? m->brief : "No metadata available";
    const char *note = m ? m->notes : "-";

    xil_printf("0x%04X  %-12s  %s\r\n", addr, nm, brf);
    xil_printf("    notes      : %s\r\n", note);
    if (idx_def >= 0) xil_printf("    default    : 0x%02X\r\n", defaults[idx_def].Data);
    else              xil_printf("    default    : (N/A)\r\n");
    if (idx_ovr >= 0) xil_printf("    override   : 0x%02X\r\n", overrides[idx_ovr].Data);
    else              xil_printf("    override   : (none)\r\n");

    if (idx_ovr >= 0) xil_printf("    effective  : 0x%02X (override)\r\n", overrides[idx_ovr].Data);
    else if (idx_def >= 0) xil_printf("    effective  : 0x%02X (default)\r\n", defaults[idx_def].Data);
    else xil_printf("    effective  : (undefined in current sets)\r\n");
}

/* UART line input (blocking; echoes; backspace supported). */
static size_t uart_readline(char *buf, size_t cap) {
    if (cap == 0) return 0;
    size_t n = 0;
    for (;;) {
        char c = (char)XUartPs_RecvByte(UART_BASEADDR);
        if (c == '\r' || c == '\n') {
            XUartPs_SendByte(UART_BASEADDR, '\r');
            XUartPs_SendByte(UART_BASEADDR, '\n');
            break;
        } else if (c == 0x08 || c == 0x7F) { /* backspace */
            if (n > 0) {
                XUartPs_SendByte(UART_BASEADDR, 0x08);
                XUartPs_SendByte(UART_BASEADDR, ' ');
                XUartPs_SendByte(UART_BASEADDR, 0x08);
                --n;
            }
        } else {
            XUartPs_SendByte(UART_BASEADDR, (uint8_t)c);
            if (n + 1 < cap) buf[n++] = c; /* keep room for '\0' */
        }
    }
    buf[n] = '\0';
    return n;
}


/* Parse "AAAA:DD 0166=31 0ABC 07, ..." into out[]. 0 on success, <0 on error. */
static int parse_pairs(const char *line,
                       regval_list *out, size_t out_cap, size_t *out_len) {
    size_t n = 0; const char *p = line;
    while (*p) {
        /* seek hex start */
        while (*p && !is_hex_char(*p) && *p != '0') ++p;
        if (!*p) break;

        /* optional 0x prefix for address */
        if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;

        /* address: exactly 4 hex digits */
        int cnt = 0; uint16_t addr = 0;
        while (is_hex_char(*p) && cnt < 4) {
            int nib = hex_nibble(*p++); if (nib < 0) return SENSOR_CFG_ERR_PARSE;
            addr = (uint16_t)((addr << 4) | (uint16_t)nib); ++cnt;
        }
        if (cnt == 0) break;
        if (cnt != 4) return SENSOR_CFG_ERR_PARSE;

        /* skip to value start */
        while (*p && !is_hex_char(*p) && *p != '0') ++p;
        if (!*p) return SENSOR_CFG_ERR_PARSE;

        /* optional 0x prefix for value */
        if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;

        /* value: exactly 2 hex digits */
        cnt = 0; uint16_t data = 0;
        while (is_hex_char(*p) && cnt < 2) {
            int nib = hex_nibble(*p++); if (nib < 0) return SENSOR_CFG_ERR_PARSE;
            data = (uint16_t)((data << 4) | (uint16_t)nib); ++cnt;
        }
        if (cnt != 2) return SENSOR_CFG_ERR_PARSE;

        if (n >= out_cap) return SENSOR_CFG_ERR_CAP;
        out[n].Address = addr;
        out[n].Data    = (uint8_t)data;
        ++n;

        while (*p && !is_hex_char(*p) && *p != '0') ++p; /* skip separators */
    }
    *out_len = n;
    return SENSOR_CFG_OK;
}

/* Upsert: update if address exists; append if new. */
static int upsert_pairs(regval_list *dst, size_t *dst_len, size_t dst_cap,
                        const regval_list *src, size_t src_len) {
    size_t n = *dst_len;
    for (size_t i = 0; i < src_len; ++i) {
        int idx = find_addr_index(dst, n, src[i].Address);
        if (idx >= 0) {
            dst[idx].Data = src[i].Data;
        } else {
            if (n >= dst_cap) return SENSOR_CFG_ERR_CAP;
            dst[n++] = src[i];
        }
    }
    *dst_len = n;
    return SENSOR_CFG_OK;
}

/* Delete by hex address; returns removed count. */
static int delete_by_address(regval_list *arr, size_t *len, uint16_t addr) {
    size_t n = *len, w = 0; int removed = 0;
    for (size_t i = 0; i < n; ++i) {
        if (arr[i].Address == addr) { ++removed; continue; }
        if (w != i) arr[w] = arr[i];
        ++w;
    }
    *len = w;
    return removed;
}

/* Delete by index; returns 1 if removed, 0 if invalid index. */
static int delete_by_index(regval_list *arr, size_t *len, size_t idx) {
    if (idx >= *len) return 0;
    for (size_t i = idx + 1; i < *len; ++i) arr[i-1] = arr[i];
    --(*len);
    return 1;
}

/* Validate overrides: check duplicates; values are already typed. */
static int validate_overrides(const regval_list *arr, size_t len) {
    int ok = 1;
    for (size_t i = 0; i < len; ++i)
        for (size_t j = i + 1; j < len; ++j)
            if (arr[i].Address == arr[j].Address) {
                xil_printf("WARN: duplicate address 0x%04X at [%d] and [%d]\r\n",
                           arr[i].Address, (int)i, (int)j);
                ok = 0;
            }
    if (ok) xil_printf("Validation: OK (no duplicates)\r\n");
    return ok ? SENSOR_CFG_OK : SENSOR_CFG_ERR_PARSE;
}

/* Merge defaults + overrides into out[]. */
static int build_regset(const regval_list *def, size_t def_len,
                        const regval_list *ovr, size_t ovr_len,
                        regval_list *out, size_t out_cap, size_t *out_len) {
    if (out_cap < def_len) {
        memcpy(out, def, out_cap * sizeof(regval_list));
        *out_len = out_cap;
        return SENSOR_CFG_ERR_CAP;
    }
    memcpy(out, def, def_len * sizeof(regval_list));
    size_t n = def_len;

    for (size_t i = 0; i < ovr_len; ++i) {
        int idx = find_addr_index(out, n, ovr[i].Address);
        if (idx >= 0) {
            out[idx].Data = ovr[i].Data;
        } else {
            if (n >= out_cap) { *out_len = n; return SENSOR_CFG_ERR_CAP; }
            out[n++] = ovr[i];
        }
    }
    *out_len = n;
    return SENSOR_CFG_OK;
}

/* Pretty-print current overrides. */
static void print_overrides(const regval_list *arr, size_t len) {
    xil_printf("Overrides (%d item%s):\r\n", (int)len, (len==1?"":"s"));
    for (size_t i = 0; i < len; ++i)
        xil_printf("  [%03d] 0x%04X <- 0x%02X\r\n",
                   (int)i, arr[i].Address, arr[i].Data);
}

/* Help text */
static void print_help(void) {
    xil_printf(
        "Commands:\r\n"
        "  add <pairs>     : Upsert pairs (e.g., add 3225:12 0166=31 0ABC 07)\r\n"
        "  del <addr|@idx> : Delete by hex address (4 digits) or index with @\r\n"
        "  info <what>     : Show register info/metadata\r\n"
        "                    - info 3225 0166      (one or more hex addresses)\r\n"
        "                    - info all            (all known entries in DB)\r\n"
        "                    - info defaults       (all addresses in defaults)\r\n"
        "                    - info overrides      (all addresses in overrides)\r\n"
        "  list            : Show current overrides\r\n"
        "  clear           : Remove all overrides\r\n"
        "  return|done|ok  : Build and return merged sensor_cfg\r\n"
        "  cancel|exit     : Abort without returning a config\r\n"
        "  help            : Show this help\r\n"
        "\r\n"
        "Pair formats (delimiters : , = space; 0x prefix allowed):\r\n"
        "  3225:12 0166=31 0ABC 07, 1234:FF\r\n"
        "Address = 4 hex digits; Value = 2 hex digits.\r\n"
    );
}

/* ===== Public API ===== */
int sensor_cfg_input(const regval_list *defaults, size_t defaults_len,
                     regval_list *out, size_t out_cap, size_t *out_len)
{
    regval_list overrides[SENSOR_CFG_MAX_OVERRIDES];
    size_t overrides_len = 0;

    xil_printf("\r\n=== sensor_cfg_input (UART) ===\r\n");
    xil_printf("Type 'help' for commands. Enter raw pairs to implicitly add.\r\n");

    char line[256];
    for (;;) {
        xil_printf("\r\n> ");
        (void)uart_readline(line, sizeof(line));

        /* skip leading spaces */
        const char *p = line;
        while (*p && isspace((unsigned char)*p)) ++p;
        if (*p == '\0') continue;

        if (ci_starts_with(p, "help")) { print_help(); continue; }
        if (ci_starts_with(p, "list")) { print_overrides(overrides, overrides_len); continue; }
        if (ci_starts_with(p, "clear")){ overrides_len = 0; xil_printf("Overrides cleared.\r\n"); continue; }
        if (ci_starts_with(p, "cancel") || ci_starts_with(p, "exit")) {
            xil_printf("Aborted by user.\r\n");
            if (out_len) *out_len = 0;
            return SENSOR_CFG_ABORTED;
        }
        if (ci_starts_with(p, "return") || ci_starts_with(p, "done") || ci_starts_with(p, "ok")) {
            /* Build and return merged configuration */
            size_t merged_len = 0;
            int rc = build_regset(defaults, defaults_len,
                                  overrides, overrides_len,
                                  out, out_cap, &merged_len);
            *out_len = merged_len;
            if (rc == SENSOR_CFG_OK) {
                xil_printf("Merged %d item(s). Returning sensor_cfg.\r\n", (int)merged_len);
                return SENSOR_CFG_OK;
            } else {
                xil_printf("ERROR: output capacity insufficient (built %d)\r\n", (int)merged_len);
                return SENSOR_CFG_ERR_CAP;
            }
        }
        if (ci_starts_with(p, "add")) {
            /* skip command token */
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            regval_list tmp[128]; size_t tmp_len = 0;
            int pr = parse_pairs(p, tmp, ARRAY_LEN(tmp), &tmp_len);
            if (pr != SENSOR_CFG_OK) {
                xil_printf("Parse error. Example: add 3225:12 0166=31 0ABC 07\r\n");
                continue;
            }
            int ur = upsert_pairs(overrides, &overrides_len, ARRAY_LEN(overrides), tmp, tmp_len);
            if (ur != SENSOR_CFG_OK) {
                xil_printf("ERROR: overrides capacity exceeded (%d items)\r\n", (int)overrides_len);
                continue;
            }
            xil_printf("Upserted %d pair(s). Now %d total.\r\n", (int)tmp_len, (int)overrides_len);
            continue;
        }
        if (ci_starts_with(p, "del")) {
            /* Syntax: del 0166 @3 ... */
            while (*p && !isspace((unsigned char)*p)) ++p;
            while (*p && isspace((unsigned char)*p)) ++p;

            int any = 0;
            while (*p) {
                while (*p && isspace((unsigned char)*p)) ++p;
                if (!*p) break;

                if (*p == '@') {
                    ++p;
                    int idx = 0, seen = 0;
                    while (isdigit((unsigned char)*p)) { idx = idx*10 + (*p - '0'); ++p; seen = 1; }
                    if (!seen) { xil_printf("ERR: '@' must be followed by index\r\n"); break; }
                    int rem = delete_by_index(overrides, &overrides_len, (size_t)idx);
                    xil_printf("del @%d -> %s\r\n", idx, rem ? "removed" : "no such index");
                    any = 1;
                } else {
                    /* delete by hex address (allow 0x prefix) */
                    if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;
                    int cnt = 0; uint16_t addr = 0;
                    while (is_hex_char(*p) && cnt < 4) {
                        int nib = hex_nibble(*p++); if (nib < 0) break;
                        addr = (uint16_t)((addr<<4)|(uint16_t)nib);
                        ++cnt;
                    }
                    if (cnt != 4) { xil_printf("ERR: need 4 hex digits for address\r\n"); break; }
                    int rem = delete_by_address(overrides, &overrides_len, addr);
                    xil_printf("del 0x%04X -> removed %d\r\n", addr, rem);
                    any = 1;
                }
                while (*p && !isalnum((unsigned char)*p) && *p!='@' && *p!='0') ++p; /* next token */
            }
            if (!any) xil_printf("Usage: del <addr|@idx> [...]\r\n");
            continue;
        }
        if (ci_starts_with(p, "info")) {
			/* Skip command token */
			while (*p && !isspace((unsigned char)*p)) ++p;
			while (*p && isspace((unsigned char)*p)) ++p;

			if (*p == '\0') {
				xil_printf("Usage:\r\n");
				xil_printf("  info 3225 0166      (hex addresses)\r\n");
				xil_printf("  info all | defaults | overrides\r\n");
				continue;
			}

			/* Handle keywords: all / defaults / overrides */
			if (ci_starts_with(p, "all")) {
				xil_printf("Register info (all known DB entries):\r\n");
				for (size_t i = 0; i < ARRAY_LEN(kRegDB); ++i) {
					print_reg_info_addr(kRegDB[i].addr, defaults, defaults_len, overrides, overrides_len);
				}
				continue;
			}
			if (ci_starts_with(p, "defaults")) {
				xil_printf("Register info (defaults):\r\n");
				for (size_t i = 0; i < defaults_len; ++i) {
					print_reg_info_addr(defaults[i].Address, defaults, defaults_len, overrides, overrides_len);
				}
				continue;
			}
			if (ci_starts_with(p, "overrides")) {
				xil_printf("Register info (overrides):\r\n");
				for (size_t i = 0; i < overrides_len; ++i) {
					print_reg_info_addr(overrides[i].Address, defaults, defaults_len, overrides, overrides_len);
				}
				continue;
			}

			/* Otherwise, parse one or more hex addresses from the rest of the line */
			int shown = 0;
			while (*p) {
				while (*p && isspace((unsigned char)*p)) ++p;
				if (!*p) break;

				/* optional 0x */
				if (p[0]=='0' && (p[1]=='x'||p[1]=='X')) p += 2;

				int cnt = 0; uint16_t addr = 0;
				while (is_hex_char(*p) && cnt < 4) {
					int nib = hex_nibble(*p++); if (nib < 0) break;
					addr = (uint16_t)((addr<<4)|(uint16_t)nib);
					++cnt;
				}
				if (cnt != 4) {
					xil_printf("ERR: need 4 hex digits for address (e.g., 3225)\r\n");
					break;
				}

				print_reg_info_addr(addr, defaults, defaults_len, overrides, overrides_len);
				shown = 1;

				/* Skip non-token chars to next token */
				while (*p && !isalnum((unsigned char)*p) && *p!='0') ++p;
			}
			if (!shown) {
				xil_printf("Usage: info <addr...> | info all | info defaults | info overrides\r\n");
			}
			continue;
		}

        /* If it looks like pairs, treat as implicit 'add' */
        if (is_hex_char(*p) || *p=='0') {
            regval_list tmp[128]; size_t tmp_len = 0;
            int pr = parse_pairs(p, tmp, ARRAY_LEN(tmp), &tmp_len);
            if (pr == SENSOR_CFG_OK && tmp_len > 0) {
                int ur = upsert_pairs(overrides, &overrides_len, ARRAY_LEN(overrides), tmp, tmp_len);
                if (ur != SENSOR_CFG_OK) {
                    xil_printf("ERROR: overrides capacity exceeded (%d items)\r\n", (int)overrides_len);
                    continue;
                }
                xil_printf("Upserted %d pair(s). Now %d total.\r\n", (int)tmp_len, (int)overrides_len);
                continue;
            }
        }

        xil_printf("Unknown input. Type 'help' for usage.\r\n");
    }
}
#endif
