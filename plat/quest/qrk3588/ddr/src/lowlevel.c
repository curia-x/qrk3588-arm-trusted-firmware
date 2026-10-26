/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* RK3588 DDR low-level register, mode-word, copy and handshake helpers. */

#include "qrk3588_ddr_internal.h"

/* 任意地址 32/64 位 volatile 访问 (MMIO 与 SRAM 结构统一处理) */
#define RD32(p)   (*(volatile uint32_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)))
#define WR32(p,v) (*(volatile uint32_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)) = (uint32_t)(v))
#define RD64(p)   (*(volatile uint64_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)))

/* GPIO GRF 握手写对 (0x0000e4 / 0x000120) */

/*
 * io_grf_handshake_release:
 *   while (*(u32*)0xfe0500e0 != 0) ;      ; 自旋等忙标志清零
 *   *(u32*)0xfe0500e4 &= ~1;              ; 清请求位
 *   *(u32*)0xfe030040 |= 0xffff;          ; 置数据位
 */
void io_grf_handshake_release(void)
{
    if (qrk3588_ddr_poll32(0xfe0500e0u, UINT32_MAX, 0u, 0u) != 0)
        return;
    WR32(0xfe0500e4u, RD32(0xfe0500e4u) & ~1u);
    WR32(0xfe030040u, RD32(0xfe030040u) | 0xffffu);
}

/*
 * io_grf_handshake_assert:
 *   *(u32*)0xfe030040 &= 0xffff0000;      ; 清数据位
 *   *(u32*)0xfe0500e4 |= 1;               ; 置请求位
 */
void io_grf_handshake_assert(void)
{
    WR32(0xfe030040u, RD32(0xfe030040u) & 0xffff0000u);
    WR32(0xfe0500e4u, RD32(0xfe0500e4u) | 1u);
}

/* 模式字生成 (0x00014c) — 反转 + 复制 */

/*
 * mode 0: b = ~val & 0xff, 返回 b 在 4 个字节位复制
 * mode 1: h = ~val & 0xffff, 返回 h 在 2 个半字复制 (bfi)
 * 其他  : 返回 ~val (32 位)
 */
uint32_t ddr_encode_inverted_mode_word(uint32_t mode, uint32_t val)
{
    uint32_t v = ~val;

    if (mode == 0u) {
        uint32_t b = v & 0xffu;
        return (b << 24) | (b << 16) | (b << 8) | b;
    }
    if (mode == 1u) {
        uint32_t h = v & 0xffffu;
        return (h << 16) | h;
    }
    return v;
}

/* 字粒度拷贝 (0x001150) */

/*
 * n = len & ~3 (64 位 AND); i 从 0 步进 4, i != n 时逐字拷贝.
 * 循环条件是 != 而非 <，保持边界语义。
 */
void ddr_word_copy(void *dst, const void *src, uint64_t len)
{
    uint64_t n = len & ~3ull;
    uint64_t i;

    for (i = 0; i != n; i += 4u) {
        WR32((uint8_t *)dst + i, RD32((const uint8_t *)src + i));
    }
}

/* 3 位域打包写入 (0x00175c) */

/*
 * b = idx & 1; 移位组 {7,8,12} - 4*b (b=1 时为 {3,4,8});
 * w = val 复制到 3 个移位处 | 对应 3 个单 bit 使能左移 16;
 * 字索引 = 0x294 + idx + idx/2 (32 位运算);
 * base[索引] = w.
 */
void ddr_clock_triplet_write(volatile uint32_t *base, uint32_t idx, uint32_t val)
{
    uint32_t b = idx & 1u;
    uint32_t s0 = 7u - 4u * b;
    uint32_t s1 = 8u - 4u * b;
    uint32_t s2 = 12u - 4u * b;
    uint32_t widx = 0x294u + idx + (idx >> 1);
    uint32_t w;

    w  = (val << s0) | (val << s1) | (val << s2);
    w |= ((1u << s0) | (1u << s1) | (1u << s2)) << 16;
    base[widx] = w;
}

/* dram_type -> 配置表指针 (0x002060) */

/*
 * tbl = QRK3588_DDR_PARAMETER_VA(0x5d0) (数据区配置表);
 * type 7 (LPDDR4):  sel = tbl[0x18]
 * type 8 (LPDDR4X): sel = tbl[0x20]
 * type 9 (LPDDR5):  sel = tbl[0x1a]
 * Other types are reported as unsupported.
 * 返回 tbl + sel*4 (表内 dword 项指针).
 * 与 ddr_cfg_byte (默认项, tbl[0xa]) 同族.
 */
const void *dram_type_cfg_ptr(uint32_t dram_type)
{
    const volatile uint8_t *tbl = DDR_LAYOUT_CPTR(volatile uint8_t, QRK3588_DDR_TYPE_CONFIG_TABLE_VA);
    uint32_t sel;

    if (dram_type == 7u) {
        sel = tbl[0x18];
    } else if (dram_type == 8u) {
        sel = tbl[0x20];
    } else if (dram_type == 9u) {
        sel = tbl[0x1a];
    } else {
        return (const void *)(uintptr_t)report_dram_type_unsupported();
    }
    return (const void *)(uintptr_t)(tbl + sel * 4u);
}

/* 通道状态轮询 (0x0026f0) */

/*
 * arr = *(u64*)QRK3588_DDR_RUNTIME_VA(0x610) (通道数组基, 运行时 = QRK3588_DDR_EXTENDED_VA(0xa08));
 * p2 = arr[0], p3 = arr[1], p1 = arr[0x80/8] (三个 u64 槽);
 * off = ((*(u32*)(p2+0x10000+8)) & 3) << 20 + 0xd00;
 * w = (*(u32*)(p2+off) & 1) | (((*(u32*)(p3+4) >> 15) & 1) << 1);
 * *(u32*)(p1+0x218) = w.
 */
void ddr_publish_phy_channel_status(void)
{
    uint8_t *arr = (uint8_t *)(uintptr_t)RD64(QRK3588_DDR_CONTEXT_SLOT_VA);
    uint8_t *p2 = (uint8_t *)(uintptr_t)RD64(arr);
    uint8_t *p3 = (uint8_t *)(uintptr_t)RD64(arr + 8);
    uint8_t *p1 = (uint8_t *)(uintptr_t)RD64(arr + 0x80);
    uint32_t off = ((RD32(p2 + 0x10000u + 8u) & 3u) << 20) + 0xd00u;
    uint32_t w = (RD32(p2 + off) & 1u)
               | (((RD32(p3 + 4u) >> 15) & 1u) << 1);

    WR32(p1 + 0x218u, w);
}

/* PHY 间接寄存器配置块 (0x002dd8) */

/*
 * r = phy + 0x10000 (PHY 寄存器窗口);
 * r[0x80] = 0x00100030;              ; 预置命令
 * while (r[0x90] & 1) ;              ; 等状态 bit0 清
 * r[0x80] = (a << 4) | 1;            ; CTRL: 地址|写标志
 * r[0x84] = b << 8;                  ; DATA
 * r[0x80] |= 0x80000000;             ; 触发
 * while (r[0x80] & 0x80000000) ;     ; 等触发位自清
 * while (!(r[0x90] & 0x10000)) ;     ; 等状态 bit16 置位
 * return r[0x94] & 0xff;             ; 读回值
 * (与 ddr_phy_reg_write 同协议的配置块变体)
 */
uint32_t ddr_phy_config_block(volatile void *phy, uint32_t a, uint32_t b)
{
    volatile uint32_t *r = (volatile uint32_t *)((uint8_t *)phy + 0x10000u);

    r[0x80 / 4] = 0x00100030u;
    if (qrk3588_ddr_poll32((uintptr_t)&r[0x90 / 4], 1u, 0u, 0u) != 0)
        return UINT32_MAX;
    r[0x80 / 4] = (a << 4) | 1u;
    r[0x84 / 4] = b << 8;
    r[0x80 / 4] |= 0x80000000u;
    if (qrk3588_ddr_poll32((uintptr_t)&r[0x80 / 4], 0x80000000u, 0u, 0u) != 0)
        return UINT32_MAX;
    if (qrk3588_ddr_poll32((uintptr_t)&r[0x90 / 4], 0x10000u, 0x10000u, 0u) != 0)
        return UINT32_MAX;
    return r[0x94 / 4] & 0xffu;
}

/* PHY 触发对 (0x003114 / 0x003154) */

/*
 * 0x280 bit31 未置位: 置位并返回 1; 已置位: 返回 0;
 * 之后 0x208 |= 1, 再 0x208 ^= 0x10 (两次独立 RMW).
 */
uint32_t ddr_phy_trigger_begin(volatile void *phy)
{
    volatile uint32_t *r = (volatile uint32_t *)((uint8_t *)phy + 0x10000u);
    uint32_t ret;

    if (r[0x280 / 4] & 0x80000000u) {
        ret = 0u;
    } else {
        r[0x280 / 4] |= 0x80000000u;
        ret = 1u;
    }
    r[0x208 / 4] |= 1u;
    r[0x208 / 4] ^= 0x10u;
    return ret;
}

/*
 * flag != 0: 清 0x280 bit31;
 * 0x208 &= ~1, 再 0x208 ^= 0x10 (ddr_phy_trigger_begin 的对称清除变体).
 */
void ddr_phy_trigger_end(volatile void *phy, uint32_t flag)
{
    volatile uint32_t *r = (volatile uint32_t *)((uint8_t *)phy + 0x10000u);

    if (flag != 0u) {
        r[0x280 / 4] &= 0x7fffffffu;
    }
    r[0x208 / 4] &= ~1u;
    r[0x208 / 4] ^= 0x10u;
}

/* 频率档位 -> 配置块指针 (0x003228) */

/*
 * type 9 (LPDDR5): base = QRK3588_DDR_PARAMETER_VA(0x778), 5 档
 *   freq <= 533 (0x215) -> +0x00   (b.ls 含等号)
 *   freq <= 800 (0x320) -> +0x15
 *   freq <= 1600(0x640) -> +0x2a
 *   freq <= 2133(0x855) -> +0x3f
 *   其他             -> +0x54
 * 其他类型 (LPDDR4/4X): base = QRK3588_DDR_PARAMETER_VA(0x710), 仅 4 档
 *   freq <= 400 (0x190) -> +0x00
 *   freq <= 613 (0x265) -> +0x15
 *   freq <= 1066(0x42a) -> +0x2a
 *   其他             -> +0x3f
 */
const void *freq_cfg_ptr(uint32_t dram_type, uint32_t freq)
{
    const volatile uint8_t *base;

    if (dram_type == 9u) {
        base = (const volatile uint8_t *)qrk3588_lp5_freq_bins();
        if (freq <= 0x215u) {
            return (const void *)(uintptr_t)(base);
        }
        if (freq <= 0x320u) {
            return (const void *)(uintptr_t)(base + 0x15u);
        }
        if (freq <= 0x640u) {
            return (const void *)(uintptr_t)(base + 0x2au);
        }
        if (freq <= 0x855u) {
            return (const void *)(uintptr_t)(base + 0x3fu);
        }
        return (const void *)(uintptr_t)(base + 0x54u);
    }
    base = (const volatile uint8_t *)qrk3588_lp4_freq_bins();
    if (freq <= 0x190u) {
        return (const void *)(uintptr_t)(base);
    }
    if (freq <= 0x265u) {
        return (const void *)(uintptr_t)(base + 0x15u);
    }
    if (freq <= 0x42au) {
        return (const void *)(uintptr_t)(base + 0x2au);
    }
    return (const void *)(uintptr_t)(base + 0x3fu);
}

/* print_dram_info 通道封装 (0x006da4) */

/*
 * quirk: 第一参数 (x0) 完全未被使用 (调用方仍设置, 属死参数);
 * geom = channel state block, info = global frequency/info block,
 * Uses flag=(chan==0) and mode=0 when printing channel information.
 */
void print_dram_info_chan(const void *unused, uint32_t chan, const void *base)
{
    (void)unused;
    print_dram_info(chan, chan == 0u,
                    qrk3588_ddr_state_channel_const(base, chan),
                    qrk3588_ddr_state_frequency_ptr(base), 0u);
}

/* 默认配置项读取 (0x009258) */

/*
 * tbl = QRK3588_DDR_PARAMETER_VA(0x5d0); sel = tbl[0xa];
 * 返回 *(u32*)(tbl + sel*4). 与 dram_type_cfg_ptr 同表同族
 * (sel 来源不同: 此处用固定槽 0xa 的默认/当前项).
 */
uint32_t ddr_cfg_byte(void)
{
    const volatile uint8_t *tbl = DDR_LAYOUT_CPTR(volatile uint8_t, QRK3588_DDR_TYPE_CONFIG_TABLE_VA);
    uint32_t sel = tbl[0xa];

    return RD32(tbl + sel * 4u);
}

/* 双指针清零 (0x00bb24) */

/*
 * p1 = *(u64*)(s + 0x98), p2 = *(u64*)(s + 0xa0);
 * *(u32*)(p1 + 0x300) = 0; *(u32*)(p2 + 0x240) = 0;
 */
void ddr_clear_aux_status_words(const void *s)
{
    uint8_t *p1 = (uint8_t *)(uintptr_t)RD64((const uint8_t *)s + 0x98u);
    uint8_t *p2 = (uint8_t *)(uintptr_t)RD64((const uint8_t *)s + 0xa0u);

    WR32(p1 + 0x300u, 0u);
    WR32(p2 + 0x240u, 0u);
}

/* +0x20000 块握手对 (0x00cb2c / 0x00cb5c) */

/*
 * r = p + 0x20000;
 * r[0xe0] bit0 未置: 返回 0;
 * 已置: 清零该位, 自旋等 r[0xe4] bit0 清零, 返回 1.
 */
uint32_t ddr_phy_request_pause(volatile void *p)
{
    volatile uint32_t *r = (volatile uint32_t *)((uint8_t *)p + 0x20000u);

    if (!(r[0xe0 / 4] & 1u)) {
        return 0u;
    }
    r[0xe0 / 4] &= ~1u;
    if (qrk3588_ddr_poll32((uintptr_t)&r[0xe4 / 4], 1u, 0u, 0u) != 0)
        return 0u;
    return 1u;
}

/*
 * r = p + 0x20000; en == 1 时 r[0xe0] |= 1 (断言请求).
 */
void ddr_phy_request_restore(volatile void *p, uint32_t en)
{
    volatile uint32_t *r = (volatile uint32_t *)((uint8_t *)p + 0x20000u);

    if (en == 1u) {
        r[0xe0 / 4] |= 1u;
    }
}

/* Register/bitfield and utility operations */

/* Low-level geometry, descriptor, handshake and PLL helpers. */

#define RD32(p)   (*(volatile uint32_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)))
#define WR32(p,v) (*(volatile uint32_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)) = (uint32_t)(v))
#define RD64(p)   (*(volatile uint64_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)))
#define WR64(p,v) (*(volatile uint64_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)) = (uint64_t)(v))


/* Diagnostic strings use named DDR-layout addresses. */
#define STR_FOUT_INVALID  DDR_LAYOUT_CPTR(char, QRK3588_DDR_ERR_FOUT_INVALID_VA)  /* "fout_mhz invalid!!!" */
#define STR_FOUT_CANNOT   DDR_LAYOUT_CPTR(char, QRK3588_DDR_ERR_FOUT_NOT_FOUND_VA)  /* "cannot find fout_mhz!!!\n" */

/* 数据区配置表 */
#define FREQ_CFG_TBL      DDR_LAYOUT_ADDR(QRK3588_DDR_TYPE_CONFIG_TABLE_VA)  /* dram_type 配置表基址 */

/* ARM 语义辅助 */
static inline uint32_t shl32(uint32_t v, uint32_t n) { return v << (n & 31u); }
static inline uint32_t asr32(uint32_t v, uint32_t n)
{
    return (uint32_t)((int32_t)v >> (n & 31u));
}
/* ARM UDIV: 除零返回 0 */
static inline uint32_t adiv(uint32_t a, uint32_t b)
{
    return b ? a / b : 0u;
}

/* ============================================================
 * ddr_sram_workspace_reset_if_tagged
 * Clears the tagged SRAM workspace when the marker matches.
 * ============================================================ */
void ddr_sram_workspace_reset_if_tagged(void)
{
    if (RD32(0x1fe004u) == 0x54410001u)
        ddr_memset((void *)0x1fe000u, 0, 0x32cu);
}

/* Compute one CS capacity component, or the sum of all populated components. */
uint64_t ddr_geometry_capacity_component(
    const struct qrk3588_ddr_channel_geometry *geometry,
    uint32_t selector, uint32_t dram_type)
{
    uint32_t addend = dram_type != 0u ? 0u :
        (geometry->device_width_code == 0u ? 2u : 1u);
    uint32_t base = geometry->bus_width_code + geometry->column_bits + addend +
                    geometry->bank_bits;
    uint64_t component[4] = {0u, 0u, 0u, 0u};

    component[0] = 1ull << ((base + geometry->cs_row_bits[0]) & 63u);
    if (geometry->rank_count > 1u)
        component[1] = 1ull << ((base + geometry->cs_row_bits[1]) & 63u);
    if (geometry->rank_count == 4u) {
        component[2] = 1ull << ((base + geometry->cs_row_bits[2]) & 63u);
        component[3] = 1ull << ((base + geometry->cs_row_bits[3]) & 63u);
    }

    if (selector < 4u)
        return component[selector];
    return component[0] + component[1] + component[2] + component[3];
}

/* Pack channel geometry into the paired RK3588 geometry registers. */
void ddr_program_rank_geometry_fields(
    const struct qrk3588_ddr_channel_geometry *geometry,
    const uint32_t *timing,
    volatile uint32_t *ra, volatile uint32_t *rb, uint32_t cs)
{
    *ra &= ~0x2000u;
    *rb &= ~0x3000u;

    *ra |= shl32(timing[1] & 7u, 13);
    *rb |= shl32((timing[1] >> 3) & 3u, 12);

    uint32_t mn = timing[2] <= 2u ? timing[2] : 2u;
    *ra |= shl32(mn - 1u, 12);

    uint32_t csa = cs <= 1u ? cs : cs - 2u;
    uint32_t cs16 = shl32(csa, 4);
    uint32_t cs2 = shl32(csa, 1);

    uint32_t v = *ra;
    v = shl32(geometry->row_3_4, csa + 30u) | v | shl32(1u, csa + 28u);
    *ra = v;

    if (csa != 0u) {
        v |= shl32(geometry->rank_count - 1u, 27);
        *ra = v;
    } else {
        v &= ~0x800u;
        *ra = v;
        *rb &= ~0x4000u;
        *ra |= shl32(geometry->rank_count == 2u ? 1u : 0u, 11);
        *rb |= shl32(geometry->rank_count == 4u ? 1u : 0u, 14);
    }

    *ra |= shl32(geometry->column_bits - 9u, cs16 + 9u);
    *ra |= shl32(geometry->bank_bits != 3u ? 1u : 0u, cs16 + 8u);
    *ra |= shl32(asr32(2u, geometry->bus_width_code), cs16 + 2u);
    {
        uint32_t t = (*ra | shl32(asr32(2u, geometry->device_width_code), cs16))
                     & ~shl32(3u, cs16 + 6u);
        *ra = t;
    }

    *rb &= ~shl32(1u, cs2 + 5u);
    *ra |= shl32((geometry->cs_row_bits[0] - 0xdu) & 3u, cs16 + 6u);
    *rb |= shl32(((geometry->cs_row_bits[0] - 0xdu) >> 2) & 1u, cs2 + 5u);

    if (geometry->cs_row_bits[1] != 0u) {
        *ra &= ~shl32(3u, cs16 + 4u);
        *rb &= ~shl32(1u, cs2 + 4u);
        *ra |= shl32((geometry->cs_row_bits[1] - 0xdu) & 3u, cs16 + 4u);
        *rb |= shl32(((geometry->cs_row_bits[1] - 0xdu) >> 2) & 1u, cs2 + 4u);
    }

    if (csa == 0u && geometry->rank_count > 2u) {
        if (geometry->cs_row_bits[2] != geometry->cs_row_bits[0])
            *rb |= 0x8000u;
        if (geometry->cs_row_bits[3] != geometry->cs_row_bits[0])
            *rb |= 0x10000u;
    }

    *rb |= 0x30000000u | shl32(geometry->column_bits - 9u, cs2);
}

/* ============================================================
 * ddr_preset_to_dram_params
 *
 * cfg: 0x38 字配置 (cfg[0xa]=dram_type, cfg[0xb]=子类型,
 * cfg[0xe]=频率 MHz, cfg[0xc]=字节表指针(64位), cfg[8], cfg[1]=字数)。
 * out: generated parameter structure; word indices are 4-byte units.
 * fill: 填充值 (0 时用默认 0x80000000/0xc0000000)。
 *
 * 频率档位 (cfg[0xa]==0x15 且 cfg[0xb]==3):
 *   freq<=400→1, <=533→4, <=666→8, <=800→0xc, <=933→0x10,
 *   <=1066→0x14, 其他→0x15
 * 频率档位 (cfg[0xa]==0x18 且 cfg[0xb]==0):
 *   freq<=800→2, <=933→5, <=1066→8, <=1200→0xc, <=1333→0x10,
 *   <=1466→0x14, <=1600→0x17, 其他→0x18
 * ============================================================ */
void ddr_preset_to_dram_params(const uint32_t *cfg, uint32_t *out, uintptr_t fill)
{
    uint32_t tier = cfg[0xa];
    uint32_t freq = cfg[0xe];
    const volatile uint32_t *cfgv = (const volatile uint32_t *)cfg;

    if (tier == 0x15u) {
        if (cfg[0xb] == 3u) {
            if (freq <= 0x190u)        tier = 1u;
            else if (freq <= 0x215u)   tier = 4u;
            else if (freq <= 0x29au)   tier = 8u;
            else if (freq <= 0x320u)   tier = 0xcu;
            else if (freq <= 0x3a5u)   tier = 0x10u;
            else if (freq <= 0x42au)   tier = 0x14u;
            else                       tier = 0x15u;
        }
    } else if (tier == 0x18u) {
        if (cfg[0xb] == 0u) {
            if (freq <= 0x320u)        tier = 2u;
            else if (freq <= 0x3a5u)   tier = 5u;
            else if (freq <= 0x42au)   tier = 8u;
            else if (freq <= 0x4b0u)   tier = 0xcu;
            else if (freq <= 0x535u)   tier = 0x10u;
            else if (freq <= 0x5bau)   tier = 0x14u;
            else if (freq <= 0x640u)   tier = 0x17u;
            else                       tier = 0x18u;
        }
    }

    out[0] = tier;
    out[1] = cfg[1];

    /* out[2]..out[2+cfg[1]-1] 填充 (cfg[0xb]==9 用 0xc0000000 档) */
    uint32_t fill_c = (fill != 0u) ? (uint32_t)fill : 0xc0000000u;
    uint32_t fill_a = (fill != 0u) ? (uint32_t)fill : 0x80000000u;
    for (uint32_t *pw = &out[2]; pw != &out[2 + cfg[1]]; pw++)
        *pw = (cfgv[0xb] == 9u) ? fill_c : fill_a;

    const uint8_t *sub = *(const uint8_t *const *)(const void *)&cfg[0xc];

    out[4]   = shl32(8u, cfg[8] >> 4);      /* 8 << (cfg[8]>>4) */
    out[0xa] = cfg[0xb];
    out[0xb] = freq;
    out[0xc] = 1;
    out[0xd] = sub[0];
    out[0xe] = sub[1];
    out[0x14] = sub[3];

    if (cfg[0xb] != 0u) {
        if (freq <= 0x12cu) {
            out[0xf] = 1;  out[0x10] = 0;  out[0x14] = 0;
        } else {
            out[0xf] = 0;
            if (freq > 0x215u) {
                out[0x10] = 1;             /* out[0x14] 保持 sub[3] */
            } else {
                out[0x10] = 0;  out[0x14] = 0;
            }
        }
    } else {
        if (freq <= 0x271u) {
            out[0xf] = 1;  out[0x10] = 0;  out[0x14] = 0;
        } else {
            out[0xf] = 0;  out[0x10] = 1;  /* out[0x14] 保持 sub[3] */
        }
    }

    if (cfg[0xb] == 7u || cfg[0xb] == 8u) {
        out[0x11] = sub[7];
        out[0x12] = sub[8];
        out[0x15] = sub[4];
    } else if (cfg[0xb] == 9u) {
        out[0x11] = sub[7];
        out[0x12] = sub[8];
    } else if (cfg[0xb] == 0u) {
        out[0x11] = sub[7];
    }
    out[0x13] = sub[2];
}

/* ============================================================
 * ddr_copy_descriptor_block
 * The 16-byte prefix defines (length,type) in its first two words.  The
 * trailing two reserved words are normalized to zero so the driver never
 * copies indeterminate stack storage.  Payload bytes follow at +0x10.
 * ============================================================ */
uintptr_t ddr_copy_descriptor_block(void *a, const void *b, uint32_t w, uint32_t w3)
{
    const uint32_t prefix[4] = { w, w3, 0u, 0u };
    (void)ddr_memcpy(a, prefix, sizeof prefix);
    return ddr_memcpy((uint8_t *)a + sizeof prefix, b, w);
}

/* ddr_pctl_trigger_handshake */
void ddr_pctl_trigger_handshake(volatile void *p)
{
    uintptr_t a = (uintptr_t)p + 0x10000u;
    if (qrk3588_ddr_poll32(a + 0xb90u, 0x10000u, 0u, 0u) != 0)
        return;
    WR32(a + 0xb8cu, RD32(a + 0xb8cu) | 0x10000u);
    if (qrk3588_ddr_poll32(a + 0xb90u, 0x10000u, 0u, 0u) != 0)
        return;
}

/* ============================================================
 * fout_mhz_setup
 *
 * ctx 为训练上下文: ctx+idx*32+0x10 处为 PLL 寄存器基址指针,
 * context clock-table slot points to the clock register block (+0x594 is the gate register).
 *
 * tf = ddr_reference_clock_hz()/1e6; tf==fout 时仅写 PLL[0 或 +0x20]=0x80008000;
 * (fout-52)>6556 → dbg_puts("fout_mhz invalid!!!");
 * PLL 搜索: p(0-6) × m(1-4), w11=(m*6600)/(2*tf) 精化至 w11<=0x3f,
 *           失败 → dbg_puts("cannot find fout_mhz!!!\n")
 * 写 PLL 寄存器序列 + 1000×udelay(1) 轮询 bit15 + 恢复门控 0x594。
 * flag!=0 时写入 PLL+0x20 起的寄存器组 (第二组)。
 * ============================================================ */
void fout_mhz_setup(void *ctx, uint32_t idx, uint32_t flag, uint32_t fout_mhz)
{
    uint32_t tf = ddr_reference_clock_hz() / 1000000u;

    if (tf == fout_mhz) {
        volatile uint32_t *pll = *(volatile uint32_t **)
            ((uint8_t *)ctx + (uintptr_t)idx * 32u + 0x10u);
        WR32((uintptr_t)pll + (flag ? 0x20u : 0u), 0x80008000u);
        return;
    }

    if ((uint32_t)(fout_mhz - 0x34u) > 0x1994u) {
        dbg_puts(STR_FOUT_INVALID);
        return;
    }

    /* --- PLL 参数搜索 (w11=反馈分频, m=倍频, p=后分频) --- */
    uint32_t tf2 = shl32(tf, 1);
    uint32_t m = 1, p, w11 = 0;
    int found = 0;
    for (p = 0; p < 7u; p++) {
        uint32_t fout_sh = shl32(fout_mhz, p);
        if ((uint32_t)(fout_sh - 0xce4u) > 0xce4u)
            continue;
        for (m = 1; m < 5u; m++) {
            w11 = adiv(m * 6600u, tf2);
            uint32_t prod = tf2 * w11;
            /* 精化: fout_sh < prod/m 时递减 w11 (prod 同步减 tf2) */
            while (w11 > 0x3fu && fout_sh < adiv(prod, m)) {
                w11--;
                prod -= tf2;
            }
            if (w11 > 0x3fu) {
                found = 1;
                break;
            }
        }
        if (found)
            break;
    }
    if (!found) {
        dbg_puts(STR_FOUT_CANNOT);
        return;
    }

    uint32_t w8 = w11 | 0x3ff0000u;
    uint32_t w9 = m | 0x1ff0000u;
    uint32_t w4 = shl32(p, 6);

    /* 分频参数表: QRK3588_DDR_PARAMETER_VA(0x5d0) + tbl[0xa]*4 + 0xc */
    uint32_t sel = *(const uint8_t *)(FREQ_CFG_TBL + 0xau);
    uint32_t w5 = *(volatile uint32_t *)(FREQ_CFG_TBL + (sel & 0xffu) * 4u + 0xcu);

    volatile uint32_t *clk = (volatile uint32_t *)qrk3588_ddr_ctx_clock_table(ctx);
    uint32_t old594 = clk[0x594u / 4u];
    clk[0x594u / 4u] = 0xc0000040u;

    uint32_t w7 = (w5 >> 1) & 0xffu;
    uint32_t w0 = 0, w6 = 0, w10 = 0;

    if (w7 != 0u) {
        uint32_t w12 = (w5 >> 17) & 3u;
        uint32_t w5b = (w5 >> 9) & 0xffu;
        uint32_t w2 = w7 * 10u;
        uint32_t w13 = w5b * 1000u;
        uint32_t w3 = tf * 1000000u;
        w0 = w2 * 26u;
        w0 = w0 * w11;
        uint32_t w6a = adiv(w3, w13);
        w6a = adiv(w6a, m);
        uint32_t w5c = adiv(w6a, 25u);
        w0 = adiv(w0, w5c);
        w0 = adiv(w0, 10000u);
        w6 = (w12 == 0u) ? 3u : ((w12 != 1u) ? 1u : 0u);

        if (w5c > 0xffu) {
            w10 = 0xffu;
            if (w0 != 0u)
                goto L198c;
            w0 = 1u;
            goto L19e0;
        }
        {
            uint32_t w3d = adiv(w3, m);
            uint32_t w10a = w5c + 1u;
            uint32_t w12a = adiv(adiv(w3d, w10a), 25u);
            uint32_t w3e = adiv(adiv(w3d, w5c), 25u);
            w12a = w13 - w12a;
            w3e = w3e - w13;
            w10 = (w12a < w3e) ? w10a : w5c;
        }
        if (w0 == 0u) {
            w0 = 1u;
            goto L19cc;
        }
L198c:
        if (w0 > 0x3eu) {
            w0 = 0x3fu;
            goto L19cc;
        }
        {
            uint32_t w3f = w0 * 10000u;
            uint32_t w5f = w3f * w10;
            uint32_t w3g = (w3f + 10000u) * w10;
            w5f = adiv(adiv(w5f, w11), 26u);
            w3g = adiv(adiv(w3g, w11), 26u);
            w5f = w2 - w5f;
            w3g = w3g - w2;
            if (w5f > w3g)
                w0 = w0 + 1u;
        }
L19cc:
        if (w0 * w10 > 0x200u)
            w0 = adiv(0x200u, w10);
    }

L19e0:
    if (flag == 0u) {
        volatile uint32_t *pll = *(volatile uint32_t **)
            ((uint8_t *)ctx + (uintptr_t)idx * 32u + 0x10u);

        WR32((uintptr_t)pll, 0x80008000u);
        WR32((uintptr_t)pll + 0x04u, 0x20002000u);
        if (w7 == 0u) {
            /* 精简序列 */
            WR32((uintptr_t)pll, w8);
            WR32((uintptr_t)pll + 0x04u, w4 | w9);
            WR32((uintptr_t)pll + 0x08u, 0xffff0000u);
        } else {
            WR32((uintptr_t)pll, w8);
            WR32((uintptr_t)pll + 0x04u, w4 | w9);
            WR32((uintptr_t)pll + 0x08u, 0xffff0000u);
            w0 = w10 | shl32(w0, 8) | (shl32(w6, 14) | 0xffff0000u);
            WR32((uintptr_t)pll + 0x0cu, w0);
            WR32((uintptr_t)pll + 0x10u, 0x10001u);
        }
        goto poll;
    }

    /* flag != 0: 写 PLL+0x20 起的第二组 */
    {
        volatile uint32_t *pll = *(volatile uint32_t **)
            ((uint8_t *)ctx + (uintptr_t)idx * 32u + 0x10u);
        uint32_t w4f = w9 | w4;
        w10 |= 0xffff0000u;

        WR32((uintptr_t)pll + 0x20u, 0x80008000u);
        WR32((uintptr_t)pll + 0x24u, 0x20002000u);
        WR32((uintptr_t)pll + 0x20u, w8);
        WR32((uintptr_t)pll + 0x24u, w4f);
        WR32((uintptr_t)pll + 0x28u, 0xffff0000u);
        if (w7 != 0u) {
            w0 = shl32(w0, 8) | shl32(w6, 14) | w10;
            WR32((uintptr_t)pll + 0x2cu, w0);
            WR32((uintptr_t)pll + 0x30u, 0x10001u);
        }
    }

poll:
    {
        uint8_t *ent = (uint8_t *)ctx + (uintptr_t)idx * 32u;
        uint32_t cnt = 1000u;
        volatile uint32_t *pll = NULL;
        uint32_t off = (flag != 0u) ? 0x38u : 0x18u;

        WR32((uintptr_t)(*(volatile uint32_t **)(ent + 0x10u)) +
             (flag != 0u ? 0x24u : 0x04u), 0x20000000u);
        do {
            udelay(1);
            pll = *(volatile uint32_t **)(ent + 0x10u);
            if (RD32((uintptr_t)pll + off) & 0x8000u)
                break;
            cnt--;
        } while (cnt != 0u);
        if (cnt == 0u)
            qrk3588_ddr_set_error(QRK3588_DDR_ETIMEDOUT);
        WR32((uintptr_t)pll + (flag != 0u ? 0x20u : 0u), 0x80000000u);
    }

    /* Restore the clock gate state while retaining only bits 6 and 7. */
    {
        volatile uint32_t *clk2 = (volatile uint32_t *)qrk3588_ddr_ctx_clock_table(ctx);
        (void)clk2[0x594u / 4u];
        clk2[0x594u / 4u] = (old594 & 0xc0u) | 0xc00000u;
    }
}

/* Prepare the IOC block and the diagnostic state used during training. */
int ddr_training_prepare(void)
{
    /*
     * The platform console owns UART2.  Earlier native code treated the
     * IO-config region returned here as a private UART and rewrote LCR/DLL;
     * that conflicts with the bootloader console during an upgrade handoff.
     */
    (void)ddr_ioc_grf_init();

    WR64(QRK3588_DDR_DEBUG_UART_SLOT_VA, 0u);
    WR32(QRK3588_DDR_DEBUG_LOG_POS_VA, 0);
    WR32(QRK3588_DDR_DEBUG_LOG_OVERFLOW_VA, 0);

    return qrk3588_ddr_get_error();
}
