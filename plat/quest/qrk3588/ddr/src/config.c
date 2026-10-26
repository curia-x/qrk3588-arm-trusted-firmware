/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* RK3588 DDR configuration tags, geometry and training reports. */

#include "qrk3588_ddr_internal.h"

/* 固定布局中的诊断字符串与参数表 */

#define STR_TRAIN_RESULT DDR_LAYOUT_CPTR(char, QRK3588_DDR_DEBUG_TRAIN_RESULT_NAME_VA)  /* "print_train_result" */
#define STR_VERSION DDR_LAYOUT_CPTR(char, QRK3588_DDR_VERSION_STRING_VA)  /* "ddr-v1.18-9fa84341ce" (21B) */
#define STR_ON DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_ON_VA)  /* "on" */
#define STR_OFF DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_OFF_VA)  /* "off" */
#define FMT_CH_HDR DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_CHANNEL_HEADER_VA)  /* "%s-%d: ch=%d\n" */
#define FMT_DLL_TFS DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_DLL_TFS_VA)  /* "DLL_LOCK_VAL:0x%x, tFS:%dfs, dll half %s\n" */
#define STR_RD_TRAIN DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_READ_TRAINING_VA)  /* "RD Training result:\n" */
#define FMT_CS_D DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_CS_VA)  /* "cs:%d\n" */
#define FMT_DQS DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_DQS_VA)  /* "DQS0:0x%x, DQS1:0x%x\n" */
#define FMT_PCT_S DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_LABEL_COLON_VA)  /* "%s:" */
#define STR_WR_TRAIN DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_WRITE_TRAINING_VA)  /* "WR Training result:\n" */
#define STR_CYCLE DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_CYCLE_VA)  /* "cycle:" */
#define FMT_FRAC DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_DECIMAL_FRACTION_VA)  /* "%d.%d " */
#define STR_NL DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_NEWLINE_VA)  /* "\n" */
#define STR_COMMA DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_COMMA_SPACE_VA)  /* ", " */

/* print_dram_geometry / print_dram_info 字符串 */
#define STR_BW DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_BUS_WIDTH_LABEL_VA)  /* "BW=" */
#define STR_COL DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_COLUMN_LABEL_VA)  /* " Col=" */
#define STR_BK DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_BANK_LABEL_VA)  /* " Bk=" */
#define STR_BG DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_BANK_GROUP_LABEL_VA)  /* " BG=" */
#define STR_CS0_ROW DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_CS0_ROW_LABEL_VA)  /* " CS0 Row=" */
#define STR_SLASH DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_SLASH_VA)  /* "/" */
#define STR_CS1_ROW DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_CS1_ROW_LABEL_VA)  /* " CS1 Row=" */
#define STR_CS2_ROW DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_CS2_ROW_LABEL_VA)  /* " CS2 Row=" */
#define STR_CS3_ROW DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_CS3_ROW_LABEL_VA)  /* " CS3 Row=" */
#define STR_CS DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_CS_COUNT_LABEL_VA)  /* " CS=" */
#define STR_DIE_BW DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_DIE_WIDTH_LABEL_VA)  /* " Die BW=" */
#define STR_SIZE DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_SIZE_LABEL_VA)  /* " Size=" */
#define STR_MB_NL DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_MB_NEWLINE_VA)  /* "MB\n" */
#define STR_CHANNEL DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_CHANNEL_OPEN_VA)  /* "channel[" */
#define STR_BRACKET DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_CHANNEL_CLOSE_VA)  /* "] " */

/* config_tag_parse slot 长度表 (11 字节, 索引 = tag-0x54410050) */
#define SLOT_TABLE        DDR_LAYOUT_CPTR(uint8_t, QRK3588_DDR_CONFIG_SLOT_TABLE_VA)

/* print_train_result 字节名表 (4 x u64 指针) */
#define BYTE_NAMES        DDR_LAYOUT_PTR(const char *const, QRK3588_DDR_PARAMETER_VA(0x868u))

/* 标签链表区 (与 debug.c TAG_AREA_BASE 一致) */
#define TAG_AREA          DDR_LAYOUT_PTR(volatile uint32_t, QRK3588_DDR_TAG_VA_BASE)
#define TAG_AREA_LIMIT    QRK3588_DDR_TAG_AREA_SIZE

/* 通道数组指针槽 (QRK3588_DDR_RUNTIME_VA(0x610), 数据区指针表: 值 QRK3588_DDR_EXTENDED_VA(0xa08)) */
#define CHAN_ARRAY_PP     DDR_LAYOUT_REF(volatile uint64_t, QRK3588_DDR_CONTEXT_SLOT_VA)

/*
 * ARM UDIV 语义: 除数为 0 时返回 0 (无陷阱).
 * C 的 '/' 是 UB, 必须显式守卫以保真.
 */
static uint32_t udiv32(uint32_t a, uint32_t b) {
    return b ? a / b : 0;
}


/* Rockchip 标准配置 hash */

/*
 * h = 0x47c6a7e6;
 * if (p == NULL || len == 0) return 0x47c6a7e6;
 * for (i = 0; i < len; i++) h ^= (h >> 2) + (h << 5) + s[i];
 * (U-Boot/Deny Rockchip 配置框架同款 hash)
 */
uint32_t config_hash(const void *p, uint32_t len) {
    p = (const void *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)p);
    const uint8_t *s = (const uint8_t *)p;
    uint32_t h = 0x47c6a7e6u;
    uint32_t i;

    if (p == 0 || len == 0)
        return 0x47c6a7e6u;               /* 返回初值 */

    for (i = 0; i < len; i++)
        h ^= (h >> 2) + (h << 5) + s[i];

    return h;
}


/* 配置 tag 合法性检查 */

/*
 * tag == 0 || tag == 0x54410001  -> return 0        (根/空 tag 合法)
 * Tags outside the accepted unsigned range are reported as bad magic.
 *                                     (打印 + 返回 1)
 * 否则 -> return 0
 * (有效范围 0x54410050..0x544100FF; 上游插入路径另行限制到 0x5441005A)
 */
int config_tag_validate(uint32_t tag) {
    if (tag == 0 || tag == 0x54410001u)
        return 0;
    if (tag - 0x54410050u > 0xafu)
        return report_bad_magic();
    return 0;
}


/* 配置标签解析/插入 */

/*
 * sdram_params_init uses this helper for the three 0x5441005x parameter tags.
 *              config_check_version / config_reparse.
 *
 * 状态机走查 (逐块对应 .L00bb8-.L00cec):
 *   buf == NULL                        -> -61 (-0x3d)
 *   config_tag_validate(tag) != 0      -> -22 (-0x16)
 *   tag == 0 || tag == 0x54410001      -> -1
 *   TAG_AREA[1] != 0x54410001 (首次)   -> 初始化根条目 {len=5,
 *      tag=0x54410001, 3 数据字清零}, e = TAG_AREA+5 (0x1fe014)
 *   否则 e = TAG_AREA (根), 循环:
 *     e[0] == 0                        -> 跳出 (空槽就地插入)
 *     report_tag_overflow(e) != 0      -> -22
 *     config_tag_validate(e[1]) != 0   -> -22
 *     tag == e[1] || e[1] == 0         -> 跳出 (命中/空 tag 就地覆盖)
 *     e += e[0]                        (next = e + len*4 字节)
 *   插入 (.L00c0c):
 *     idx = tag - 0x54410050 > 0xA     -> -22
 *     slot = SLOT_TABLE[idx] == 0      -> -22
 *     (e-0x1fe000) + slot*4 > 0x2000   -> -12
 *     e[1] = tag; e[0] = slot
 *     两次 copy primitive 的返回值均被丢弃
 *     e[slot] = 0; e[slot+1] = 0       (清下一空条目)
 *     return 0
 */
int config_tag_parse(uint32_t tag, void *buf) {
    volatile uint32_t *e;
    uint32_t idx, slot, hash;

    if (buf == 0)
        return -61;                          /* -0x3d */

    if (config_tag_validate(tag) != 0)
        return -22;                          /* -0x16 */

    if (tag == 0 || tag == 0x54410001u)
        return -1;                           /* */

    if (TAG_AREA[1] != 0x54410001u) {
        /* 首次调用: 初始化根条目, 插入点 = 0x1fe014 */
        TAG_AREA[1] = 0x54410001u;
        TAG_AREA[2] = 0;
        TAG_AREA[3] = 0;
        TAG_AREA[4] = 0;
        TAG_AREA[0] = 5;
        e = TAG_AREA + 5;
    } else {
        e = TAG_AREA;                        /* 从根条目遍历 */
        while (e[0] != 0) {
            if (report_tag_overflow((const void *)e) != 0)
                return -22;                  /* */
            if (config_tag_validate(e[1]) != 0)
                return -22;                  /* */
            if (tag == e[1] || e[1] == 0)
                break;                       /* 命中/空槽: 就地插入 */
            e += e[0];                       /* next = e + len*4 字节 */
        }
    }

    idx = tag - 0x54410050u;                 /* tag + 0xABBEFFB0 */
    if (idx > 0xa)
        return -22;                          /* */

    slot = SLOT_TABLE[idx];
    if (slot == 0)
        return -22;                          /* */

    if ((uintptr_t)e - (uintptr_t)TAG_AREA + (uint64_t)slot * 4 >
        TAG_AREA_LIMIT)
        return -12;                          /* */

    e[1] = tag;                              /* 先 tag */
    e[0] = slot;                             /* 后 len */

    /* quirk: 与 buf / hash 的两次 memcmp 返回值均被丢弃 */
    ddr_memcpy((void *)(e + 2), buf, (size_t)(slot * 4 - 12));
    hash = config_hash((const void *)e, slot * 4 - 4);
    {
        /* hash is staged as a 32-bit word; the copy return value is ignored */
        uint32_t hv = hash;
        ddr_memcpy((void *)((uint8_t *)e + slot * 4 - 4), &hv, 4);
    }

    e[slot] = 0;                             /* 下一空条目 len=0 */
    e[slot + 1] = 0;                         /* 下一空条目 tag=0 */

    return 0;
}


/* DRAM 几何布局打印 */

/* Print the decoded per-channel geometry and derived capacity. */
void print_dram_geometry(const void *geom, const void *info, uint32_t mode)
{
    const struct qrk3588_ddr_channel_geometry *g =
        (const struct qrk3588_ddr_channel_geometry *)geom;
    const uint32_t *inf = (const uint32_t *)info;
    uint32_t bg = (g->device_width_code == 0u) ? 1u : 0u;
    uint64_t size;

    dbg_puts(STR_BW);
    print_dec(8u << g->bus_width_code);

    dbg_puts(STR_COL);
    print_dec(g->column_bits);

    dbg_puts(STR_BK);
    print_dec(1u << g->bank_bits);

    if (inf[0] == 0u) {
        bg += 1u;
        dbg_puts(STR_BG);
        print_dec(1u << bg);
    }

    for (uint32_t cs = 0u; cs < g->rank_count && cs < 4u; ++cs) {
        const char *row_label;
        switch (cs) {
        case 0u: row_label = STR_CS0_ROW; break;
        case 1u: row_label = STR_CS1_ROW; break;
        case 2u: row_label = STR_CS2_ROW; break;
        default: row_label = STR_CS3_ROW; break;
        }
        dbg_puts(row_label);
        print_dec(g->cs_row_bits[cs]);
        if (g->cs_high16_row_bits[cs] != g->cs_row_bits[cs]) {
            dbg_puts(STR_SLASH);
            print_dec(g->cs_high16_row_bits[cs]);
        }
    }

    dbg_puts(STR_CS);
    print_dec(g->rank_count);

    if (inf[0] - 2u > 1u) {
        dbg_puts(STR_DIE_BW);
        print_dec(8u << g->device_width_code);
    }

    size = qrk3588_ddr_geometry_capacity_bytes(g, inf[0], mode);

    dbg_puts(STR_SIZE);
    print_dec((uint32_t)(size >> 20));
    dbg_puts(STR_MB_NL);
}


/* DRAM 信息打印 */

/*
 * flag == 1 时先 print_dram_type(info) (类型名 + ", " + 频率 + "MHz\n");
 * Then prints the channel label and decoded geometry.
 * (geometry 的 info 参数是本函数 info+4 —— 类型字所在地址).
 */
void print_dram_info(uint32_t chan, uint32_t flag, const void *geom,
                     const void *info, uint32_t mode) {
    if (flag == 1)
        print_dram_type(info);               /* */

    dbg_puts(STR_CHANNEL);
    print_dec(chan);
    dbg_puts(STR_BRACKET);

    print_dram_geometry(geom, (const uint8_t *)info + 4, mode);  /* */
}


/* 版本校验 */

/*
 * 栈缓冲 0x128 字节清零; quirk: 与 "ddr-v1.18-9fa84341ce" 的
 * 15-byte comparison/copy-side result is intentionally ignored before
 * the version tag is inserted. 随后插入版本 tag 0x5441005A (slot 76) 并返回
 * config_tag_parse 的结果.
 */
int config_check_version(void) {
    uint8_t buf[0x128];                      /* 栈 0x150, buf@fp+0x28 */

    ddr_memset(buf, 0, 0x128);

    /* quirk: 结果丢弃 (cbz/cbnz 均无) */
    ddr_memcpy(buf + 4, STR_VERSION, 0x15);

    return config_tag_parse(0x5441005au, buf);
}


/* 配置二次解析 */

/*
 * 栈缓冲 0x88 字节清零后建表:
 *   buf[0] = 1;
 *   hi  = dram_type & 0xffff0000;
 *   mid = dram_type & 0xf000, 为 0 时取 0x10000;
 *   low = dram_type & 0xfff;
 *   12 个 2 字条目: bit i of low 置位 -> {hi, mid}, 否则 {0, 0}
 * 返回 config_tag_parse(0x54410059, buf) (slot 36).
 */
int config_reparse(uint32_t dram_type) {
    uint32_t buf[0x88 / 4];
    uint32_t hi = dram_type & 0xffff0000u;
    uint32_t mid = dram_type & 0xf000u;
    uint32_t low = dram_type & 0xfffu;
    int i;

    ddr_memset(buf, 0, 0x88);

    if (mid == 0)
        mid = 0x10000u;                      /* csel */

    buf[0] = 1;                              /* */

    for (i = 0; i < 12; i++) {
        if (low & (1u << i)) {
            buf[1 + 2 * i] = hi;             /* stp w4, w3 */
            buf[2 + 2 * i] = mid;
        } else {
            buf[1 + 2 * i] = 0;              /* stp wzr, wzr */
            buf[2 + 2 * i] = 0;
        }
    }

    return config_tag_parse(0x54410059u, buf);
}


/* 训练结果打印 */

/*
 * Called for both channel result-reporting paths.
 *
 * 输出结构:
 *   "%s-%d: ch=%d\n"  ("print_train_result", 0x1d8, chan)   ; 472 = 固定常数
 *   "DLL_LOCK_VAL:0x%x, tFS:%dfs, dll half %s\n" (dll_lock, tFS, on/off)
 *   "RD Training result:\n"
 *   每 cs (cnt > cs):
 *     [cnt==2] "cs:%d\n"
 *     "DQS0:0x%x, DQS1:0x%x\n" (p[0x580+cs*0x1e8] 的两个 9 位域)
 *     4 x (字节名 "min/mid/max/range:") x 2 shift x 9 行 printhex + ", " + "\n"
 *   "WR Training result:\n"
 *   每 stage (cnt > stage):
 *     [cnt==2] "cs:%d\n"
 *     "cycle:" + 2 x (10 x "%d.%d " 3 位域) + ", " + "\n"
 *     4 x 字节名 x 2 shift x 10 行 printhex + ", " + "\n"
 */
void print_train_result(uint32_t chan, const void *results) {
    const uint32_t *res = (const uint32_t *)results;
    const uint32_t *p;
    uint32_t freq, cnt, cs, stage, dll_lock, flag, half, tFS;
    uint32_t shift, row, n, wa, wb;

    /* 通道结构链: arr=*(u64*)QRK3588_DDR_RUNTIME_VA(0x610); p=*(u64*)(arr+chan*32+8) */
    p = *(const uint32_t *const *)(CHAN_ARRAY_PP + (uint64_t)chan * 32 + 8);

    freq = res[0x1b0 / 4];                   /* res[0x1b0] */

    dll_lock = p[0xb4 / 4] & 0x7ffu;         /* DLL_LOCK_VAL */
    {
        uint32_t sel = p[0xb8 / 4] >> 30;    /* */
        if (sel != 0)
            flag = p[(0xc4 + (sel - 1) * 4) / 4] & 0x40000000u;  /* */
        else
            flag = p[0xb0 / 4] & 1u;         /* */
    }
    half = (flag != 0) ? 2u : 1u;            /* cset +1 */

    ddr_printf(FMT_CH_HDR, STR_TRAIN_RESULT, 0x1d8u, chan);   /* */

    tFS = udiv32(udiv32(udiv32(1000000000u, freq), dll_lock), half);  /* */
    ddr_printf(FMT_DLL_TFS, dll_lock, tFS, (half == 2) ? STR_ON : STR_OFF);

    ddr_printf(STR_RD_TRAIN);

    /* cnt: 同一计数驱动 RD cs 循环与 WR stage 循环 */
    cnt = *(const uint32_t *)((const uint8_t *)res + chan * 0x6c);

    /* ---- RD 矩阵: 每 cs 一轮 ---- */
    for (cs = 0; cnt > cs; cs++) {
        uint32_t offa_base, offb_base;

        if (cnt == 2)                        /* "cs:%d" 仅 cnt==2 */
            ddr_printf(FMT_CS_D, cs);

        {
            uint32_t v = p[(0x580 + cs * 0x1e8) / 4];   /* */
            ddr_printf(FMT_DQS, v & 0x1ffu, (v >> 16) & 0x1ffu);
        }

        if (cs != 0) {                       /* 矩阵基址按 cs 选择 */
            offa_base = 0x770;
            offb_base = 0x794;
        } else {
            offa_base = 0x190;
            offb_base = 0x614;
        }

        for (n = 0; n < 4; n++) {            /* min/mid/max/range */
            ddr_printf(FMT_PCT_S, BYTE_NAMES[n]);

            for (shift = 0; shift < 0x20; shift += 16) {
                for (row = 0; row < 9; row++) {
                    uint32_t offa, offb;

                    if (row == 8) {          /* 第 9 行: 基址-4 */
                        offa = offa_base - 4;
                        offb = offb_base - 4;
                    } else if (cs == 0) {    /* 行距 12 */
                        offa = offa_base + row * 12;
                        offb = offb_base + row * 12;
                    } else {                 /* 行距 4 */
                        offa = offa_base + row * 4;
                        offb = offb_base + row * 4;
                    }

                    /* 第一块: a=field(p[offb]), b=2*field(p[offa])-a */
                    {
                        uint32_t a = (p[offb / 4] >> shift) & 0x3ffu;
                        uint32_t b = (((p[offa / 4] >> shift) & 0x3ffu) << 1) - a;
                        printhex(a, b, (int)n);
                    }
                }
                ddr_printf(STR_COMMA);
            }
            ddr_printf(STR_NL);
        }
    }

    /* ---- WR 矩阵: 每 stage 一轮 ---- */
    ddr_printf(STR_WR_TRAIN);

    for (stage = 0; cnt > stage; stage++) {
        uint32_t base;

        if (cnt == 2)                        /* "cs:%d" 仅 cnt==2 */
            ddr_printf(FMT_CS_D, stage);

        ddr_printf(STR_CYCLE);

        /* cycle 字对: p[base]/p[base+4], 各 10 个 3 位域 -> "%d.%d " */
        base = (stage != 0) ? 0xdc8u : 0xdc0u;
        {
            int w, f;
            for (w = 0; w < 2; w++) {
                uint32_t v = p[(base + (uint32_t)w * 4) / 4];
                for (f = 0; f < 10; f++) {
                    uint32_t bits = (v >> (f * 3)) & 7u;
                    ddr_printf(FMT_FRAC, (bits >> 1) & 3u,
                               (bits & 1u) * 5u);
                }
                ddr_printf(STR_COMMA);
            }
        }
        ddr_printf(STR_NL);

        /* 第二矩阵块基址: stage 选择 (无 -4 修正) */
        if (stage != 0) {
            wa = 0x500;
            wb = 0x410;
        } else {
            wa = 0x490;
            wb = 0x1f0;
        }

        for (n = 0; n < 4; n++) {            /* min/mid/max/range */
            ddr_printf(FMT_PCT_S, BYTE_NAMES[n]);

            for (shift = 0; shift < 0x20; shift += 16) {
                for (row = 0; row < 10; row++) {
                    uint32_t offa, offb;

                    if (row != 9) {          /* 行偏移只加 A 侧 */
                        offa = wa + row * 12;
                        offb = wb;
                    } else {                 /* 第 10 行: 两侧 -8 */
                        offa = wa - 8;
                        offb = wb - 8;
                    }

                    /* 第二块: a=field(p[offa]), b=2*field(p[offb])-a */
                    {
                        uint32_t a = (p[offa / 4] >> shift) & 0x3ffu;
                        uint32_t b = (((p[offb / 4] >> shift) & 0x3ffu) << 1) - a;
                        printhex(a, b, (int)n);
                    }
                }
                ddr_printf(STR_COMMA);
            }
            ddr_printf(STR_NL);
        }
    }
}
