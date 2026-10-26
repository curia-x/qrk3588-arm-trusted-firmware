/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* RK3588 DDR platform support: IO-GRF, address map, reset and clocks. */

#include "qrk3588_ddr_internal.h"

#define RD32(p)   (*(volatile uint32_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)))
#define WR32(p,v) (*(volatile uint32_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)) = (uint32_t)(v))
#define RD64(p)   (*(volatile uint64_t *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)(p)))

/* NIU/IOCFG GRF 寄存器窗口 (0xfe050000 系, big VDDIO GRF) */
#define IOCFG_GRF_DATA0   0xfe050000u  /* 模式: 0=seq_a / 3=seq_b */
#define IOCFG_GRF_ADDR_LO 0xfe050004u
#define IOCFG_GRF_ADDR_HI 0xfe050008u
#define IOCFG_GRF_LEN     0xfe05000cu  /* len>>2 (字数) */
#define IOCFG_GRF_SEL     0xfe05002cu  /* B | C<<2 | D<<4 */
#define IOCFG_GRF_STATUS  0xfe050054u
#define IOCFG_GRF_TRIGGER 0xfe050058u  /* 写 7 触发 */
#define IOCFG_TBL_BASE    DDR_LAYOUT_ADDR(QRK3588_DDR_IO_CONFIG_TABLE_VA)  /* dispatch 相位表 (16 字) */

/* ARM 32 位变量移位语义: 移位量按 mod 32 */
static inline uint32_t shl32(uint32_t v, uint32_t n)
{
    return v << (n & 31u);
}

/* ============================================================
 * io_grf_range_clear
 *
 * 1) 清零 0xfe050060-0xfe05009c 与 0xfe0500a0-0xfe0500dc 两个
 *    交错窗口 (单循环 [p]=0, [p+4+0x3c]=0);
 * 2) n = (sel ? 1<<(sel+1) : 1) << shift; mod = min(n,16);
 *    cnt = max(n,4) (cnt uses the unclipped n rather than mod);
 * 3) i < cnt 循环: tbl0 非空时 [0x60+4i]=tbl0[i%mod];
 *    tbl1 非空时 [0xa0+4i]=tbl1[i%mod] 且 [0xf8+4i]=val。
 * ============================================================ */
void io_grf_range_clear(const uint32_t *tbl0, const uint32_t *tbl1,
                        uint32_t shift, uint32_t sel, uint32_t val)
{
    for (uintptr_t p = 0xfe050060u; p != 0xfe0500a0u; p += 4) {
        WR32(p, 0);
        WR32(p + 0x40u, 0);
    }

    uint32_t n = shl32(sel ? 1u << ((sel + 1u) & 31u) : 1u, shift);
    uint32_t mod = (n <= 16u) ? n : 16u;
    uint32_t cnt = (n >= 4u) ? n : 4u;

    for (uint32_t i = 0; i < cnt; i++) {
        if (tbl0 != NULL)
            WR32(0xfe050060u + 4u * i, tbl0[i % mod]);
        if (tbl1 != NULL) {
            WR32(0xfe0500a0u + 4u * i, tbl1[i % mod]);
            WR32(0xfe0500f8u + 4u * i, val);
        }
    }
}

/* ============================================================
 * io_config_program_mode0
 * 写 0xfe050000 系: [0]=0, [4/8]=addr 拆分, [0xc]=len>>2,
 * [0x2c]=B|C<<2|D<<4, [0x58]=7; 调 range_clear(tbl,tbl,B,C,-1);
 * 握手 io_grf_handshake_assert(); 自旋等 [0x54]!=0;
 * (v&7)==4 → 返回 0, 否则返回 [0x54] 新值。
 * (原码在握手前后各有一次读取 cntpct 但结果弃用)
 * ============================================================ */
uint32_t io_config_program_mode0(uint64_t addr, uint64_t len, const uint32_t *tbl,
                         uint32_t b, uint32_t c, uint32_t d)
{
    WR32(IOCFG_GRF_DATA0, 0);
    WR32(IOCFG_GRF_ADDR_LO, (uint32_t)addr);
    WR32(IOCFG_GRF_ADDR_HI, (uint32_t)(addr >> 32));
    WR32(IOCFG_GRF_LEN, (uint32_t)(len >> 2));
    WR32(IOCFG_GRF_SEL, b | (c << 2) | (d << 4));
    WR32(IOCFG_GRF_TRIGGER, 7);

    io_grf_range_clear(tbl, tbl, b, c, 0xffffffffu);

    io_grf_handshake_assert();
    if (qrk3588_ddr_poll_any32(IOCFG_GRF_STATUS, UINT32_MAX, 0u) == QRK3588_DDR_ETIMEDOUT)
        return (uint32_t)QRK3588_DDR_ETIMEDOUT;

    uint32_t v = RD32(IOCFG_GRF_STATUS);
    uint32_t r = ((v & 7u) == 4u) ? 0u : RD32(IOCFG_GRF_STATUS);
    io_grf_handshake_release();
    if (qrk3588_ddr_get_error() != 0)
        return (uint32_t)qrk3588_ddr_get_error();
    return r;
}

/* ============================================================
 * io_config_program_mode3
 * 同 seq_a, 但 [0xfe050000]=3 且 range_clear(NULL, tbl, B, C, e),
 * e 为第 7 参数 (mode_word)。
 * ============================================================ */
uint32_t io_config_program_mode3(uint64_t addr, uint64_t len, const uint32_t *tbl,
                         uint32_t b, uint32_t c, uint32_t d, uint32_t e)
{
    WR32(IOCFG_GRF_DATA0, 3);
    WR32(IOCFG_GRF_ADDR_LO, (uint32_t)addr);
    WR32(IOCFG_GRF_ADDR_HI, (uint32_t)(addr >> 32));
    WR32(IOCFG_GRF_LEN, (uint32_t)(len >> 2));
    WR32(IOCFG_GRF_SEL, b | (c << 2) | (d << 4));
    WR32(IOCFG_GRF_TRIGGER, 7);

    io_grf_range_clear(NULL, tbl, b, c, e);

    io_grf_handshake_assert();
    if (qrk3588_ddr_poll_any32(IOCFG_GRF_STATUS, UINT32_MAX, 0u) == QRK3588_DDR_ETIMEDOUT)
        return (uint32_t)QRK3588_DDR_ETIMEDOUT;

    uint32_t v = RD32(IOCFG_GRF_STATUS);
    uint32_t r = ((v & 7u) == 4u) ? 0u : RD32(IOCFG_GRF_STATUS);
    io_grf_handshake_release();
    if (qrk3588_ddr_get_error() != 0)
        return (uint32_t)qrk3588_ddr_get_error();
    return r;
}

/* ============================================================
 * io_config_dispatch
 *
 * 4 相位 (0-3) 循环, 每相位:
 *   按 mode(第 4 参) 与 odt(第 6 参) 填 QRK3588_DDR_RUNTIME_VA(0x638) 处 16 字;
 *   调 seq_a(a,b,tbl,2,3,0), 非 0 → 返回 -1;
 *   调 seq_b(a,b,tbl,2,3,0,mode_word), 非 0 → 返回 -1。
 * 全部相位成功返回 0。
 *
 * mode_word = ddr_encode_inverted_mode_word(mode, val) (val 为第 5 参, 原 x4)。
 * 相位表填充规则 (mode: 0=字节流, 1=半字流, 其他=符号扩展位型):
 *   phase 0/1: mode0→0x00ff00ff; mode1→0xffff; 其他→i 偶?全1:0;
 *              phase 0 时取反。
 *   phase 2/3, odt<=31: base = mode0 ? w28 : (mode1 ? local : 1<<odt);
 *              phase 2 时 base 取反; mode2 时 i 偶填 ~base 奇填 base。
 *   phase 2/3, odt>31:  mode0→0xffff; mode1→i 偶?全1:0;
 *              其他→((i^2)&2)?全1:0; phase 2 时取反。
 * 其中 msk=1<<(odt&31), nmsk=~msk:
 *   local = (msk&0xffff) | (nmsk<<16);
 *   w28   = ((msk&0xff)<<16 | nmsk<<24) | ((nmsk&0xff)<<8 | (msk&0xff))。
 * ============================================================ */
int io_config_dispatch(uint64_t a, uint32_t b, uint32_t sel2, uint32_t mode,
                       uint32_t val, uint32_t odt)
{
    (void)sel2;                     /* 原码中 x2 从未使用 (调用方恒传 1) */
    uint32_t mode_word = ddr_encode_inverted_mode_word(mode, val);

    uint32_t msk = 1u << (odt & 31u);   /* ARM LSL 移位量 mod 32 */
    uint32_t nmsk = ~msk;
    uint32_t local = (msk & 0xffffu) | shl32(nmsk, 16);
    uint32_t w28 = (shl32(msk & 0xffu, 16) | shl32(nmsk, 24))
                 | (shl32(nmsk & 0xffu, 8) | (msk & 0xffu));
    volatile uint32_t *tbl = (volatile uint32_t *)IOCFG_TBL_BASE;

    for (uint32_t phase = 0; phase < 4u; phase++) {
        if (phase <= 1u) {
            for (uint32_t i = 0; i < 16u; i++) {
                uint32_t v;
                if (mode == 0u) {
                    v = 0xff00ffu;
                } else if (mode == 1u) {
                    v = 0xffffu;
                } else {
                    v = (i & 1u) ? 0u : 0xffffffffu;
                }
                tbl[i] = (phase == 0u) ? ~v : v;
            }
        } else if (odt <= 0x1fu) {
            uint32_t base;
            if (mode == 0u) {
                base = w28;
            } else {
                base = (mode == 1u) ? local : msk;
            }
            if (phase == 2u)
                base = ~base;
            uint32_t inv = ~base;
            for (uint32_t i = 0; i < 16u; i++)
                tbl[i] = (mode == 2u) ? ((i & 1u) ? base : inv) : base;
        } else {
            for (uint32_t i = 0; i < 16u; i++) {
                uint32_t v;
                if (mode == 0u) {
                    v = 0xffffu;
                } else if (mode == 1u) {
                    v = (i & 1u) ? 0u : 0xffffffffu;
                } else {
                    v = ((i ^ 2u) & 2u) ? 0xffffffffu : 0u;
                }
                tbl[i] = (phase == 2u) ? ~v : v;
            }
        }

        if (io_config_program_mode0(a, b, (const uint32_t *)tbl, 2, 3, 0) != 0)
            return -1;
        if (io_config_program_mode3(a, b, (const uint32_t *)tbl, 2, 3, 0,
                            mode_word) != 0)
            return -1;
    }
    return 0;
}

/* ============================================================
 * ddr_ioc_grf_init
 *
 * cfg = ddr_cfg_byte();
 * cfg==0: [0xfd5f4000]=0x0ff00880, [0xfd5f800c]=0x0ff00aa0,
 *         返回 {0xfeb50000, 0x16e360};
 * 高 nibble top=cfg>>28 非 0: sub=(cfg>>24)&0xf, 按 top(1-9)/sub(0-2)
 *   写 0xfd5f4xxx/0xfd5f8xxx GRF (值 0x0ff00880/0x0ff00aa0/0x00ff0044/
 *   0x00ff000a/0x000f000a 等), 返回 IO 基址
 *   0xfeb40000-0xfebc0000 或 0xfd890000 (top==0), size=cfg&0xffffff;
 * top 为其他值: 返回 {0,0}。
 * ============================================================ */
ioc_region_t ddr_ioc_grf_init(void)
{
    ioc_region_t r;
    uint32_t cfg = ddr_cfg_byte();
    uint32_t size = cfg & 0xffffffu;

    if (cfg == 0u) {
        WR32(0xfd5f4000u, 0x0ff00880u);
        WR32(0xfd5f800cu, 0x0ff00aa0u);
        r.base = 0xfeb50000u;
        r.size = 0x16e360u;
        return r;
    }

    uint32_t top = cfg >> 28;
    uint32_t sub = (cfg >> 24) & 0xfu;

    if (top == 0u) {
        uint32_t mid = (cfg >> 24) & 0xffu;
        if (mid == 0u) {
            WR32(0xfd5f4008u, 0x00ff0044u);
        } else if (mid == 1u) {
            WR32(0xfd5f0008u, 0x00ff0044u);
        } else if (mid == 2u) {
            WR32(0xfd5f8084u, 0x000f000au);
            WR32(0xfd5f8080u, 0xf000a000u);
        }
        r.base = 0xfd890000u;
        r.size = size;
        return r;
    }

    switch (top) {
    case 1:
        if (sub == 0u)      WR32(0xfd5f804cu, 0xff00aa00u);
        else if (sub == 1u) WR32(0xfd5f802cu, 0xff00aa00u);
        else if (sub == 2u) {
            WR32(0xfd5f400cu, 0x0ff00880u);
            WR32(0xfd5f8018u, 0x0ff00aa0u);
        }
        r.base = 0xfeb40000u;
        break;
    case 2:
        if (sub == 0u) {
            WR32(0xfd5f4000u, 0x0ff00880u);
            WR32(0xfd5f800cu, 0x0ff00aa0u);
        } else if (sub == 1u) {
            WR32(0xfd5f8098u, 0x00ff00aau);
        } else if (sub == 2u) {
            WR32(0xfd5f8068u, 0x00ff0aa0u);
        }
        r.base = 0xfeb50000u;
        break;
    case 3:
        if (sub == 0u)      WR32(0xfd5f8030u, 0x00ff00aau);
        else if (sub == 1u) WR32(0xfd5f806cu, 0x00ff0aa0u);
        else if (sub == 2u) WR32(0xfd5f8084u, 0x00ff0aa0u);
        r.base = 0xfeb60000u;
        break;
    case 4:
        if (sub == 0u)      WR32(0xfd5f8038u, 0xff00aa00u);
        else if (sub == 1u) WR32(0xfd5f8078u, 0x00ff00aau);
        else if (sub == 2u) WR32(0xfd5f8028u, 0xff00aa00u);
        r.base = 0xfeb70000u;
        break;
    case 5:
        if (sub == 0u)      WR32(0xfd5f809cu, 0x00ff00aau);
        else if (sub == 1u) WR32(0xfd5f8074u, 0x00ff00aau);
        else if (sub == 2u) WR32(0xfd5f805cu, 0x00ff00aau);
        r.base = 0xfeb80000u;
        break;
    case 6:
        if (sub == 0u)      WR32(0xfd5f8044u, 0xff00aa00u);
        else if (sub == 1u) WR32(0xfd5f8020u, 0x00ff00aau);
        else if (sub == 2u) WR32(0xfd5f8038u, 0x00ff00aau);
        r.base = 0xfeb90000u;
        break;
    case 7:
        if (sub == 0u)      WR32(0xfd5f804cu, 0x00ff00aau);
        else if (sub == 1u) WR32(0xfd5f8070u, 0x00ff00aau);
        else if (sub == 2u) WR32(0xfd5f802cu, 0x00ff00aau);
        r.base = 0xfeba0000u;
        break;
    case 8:
        if (sub == 0u)      WR32(0xfd5f8088u, 0x00ff00aau);
        else if (sub == 1u) WR32(0xfd5f8060u, 0xff00aa00u);
        r.base = 0xfebb0000u;
        break;
    case 9:
        if (sub == 0u) {
            WR32(0xfd5f8054u, 0x000f000au);
            WR32(0xfd5f8050u, 0x00f00a00u);
        } else if (sub == 1u) {
            WR32(0xfd5f808cu, 0x00ff00aau);
        } else if (sub == 2u) {
            WR32(0xfd5f807cu, 0x00ff00aau);
        }
        r.base = 0xfebc0000u;
        break;
    default:
        r.base = 0u;
        r.size = 0u;
        return r;
    }

    r.size = size;
    return r;
}

/* Platform address-map/context initialization */

/*
 * Small, deterministic register helpers.
 *
 * Initialise the DDR/GRF/CRU base-address table in the context object.  Each
 * slot has a typed accessor in qrk3588_ddr_context.h.
 */
void ddr_context_base_table_init(uint64_t *ctx)
{
    static const uintptr_t channel_base0[QRK3588_DDR_CHANNEL_COUNT] = {
        UINT32_C(0xf7000000), UINT32_C(0xf8000000),
        UINT32_C(0xf9000000), UINT32_C(0xfa000000),
    };
    static const uintptr_t channel_base1[QRK3588_DDR_CHANNEL_COUNT] = {
        UINT32_C(0xfe0c0000), UINT32_C(0xfe0d0000),
        UINT32_C(0xfe0e0000), UINT32_C(0xfe0f0000),
    };
    static const uintptr_t channel_base2[QRK3588_DDR_CHANNEL_COUNT] = {
        UINT32_C(0xfd800000), UINT32_C(0xfd804000),
        UINT32_C(0xfd808000), UINT32_C(0xfd80c000),
    };
    static const uintptr_t channel_output[QRK3588_DDR_CHANNEL_COUNT] = {
        UINT32_C(0xfe000000), UINT32_C(0xfe002000),
        UINT32_C(0xfe004000), UINT32_C(0xfe006000),
    };

    for (uint32_t channel = 0u; channel < QRK3588_DDR_CHANNEL_COUNT; ++channel) {
        qrk3588_ddr_ctx_store_channel_base0(ctx, channel, channel_base0[channel]);
        qrk3588_ddr_ctx_store_channel_base1(ctx, channel, channel_base1[channel]);
        qrk3588_ddr_ctx_store_channel_base2(ctx, channel, channel_base2[channel]);
        qrk3588_ddr_ctx_store_channel_output(ctx, channel, channel_output[channel]);
    }

    qrk3588_ddr_ctx_store_ptr(ctx, QRK3588_DDR_CTX_GEOMETRY_REGS_OFFSET, UINT32_C(0xfd58a000));
    qrk3588_ddr_ctx_store_ptr(ctx, QRK3588_DDR_CTX_CLOCK_TABLE_OFFSET, UINT32_C(0xfd7c0000));
    qrk3588_ddr_ctx_store_ptr(ctx, QRK3588_DDR_CTX_ADDRESS_MAP_REGS_OFFSET, UINT32_C(0xfd59e000));
    qrk3588_ddr_ctx_store_ptr(ctx, QRK3588_DDR_CTX_GRF_REGS_OFFSET, UINT32_C(0xfd586000));
    qrk3588_ddr_ctx_store_ptr(ctx, QRK3588_DDR_CTX_GRF_MIRROR_REGS_OFFSET, UINT32_C(0xfd587000));
    qrk3588_ddr_ctx_store_ptr(ctx, QRK3588_DDR_CTX_RESET_REGS_OFFSET, UINT32_C(0xfd8d0000));
    qrk3588_ddr_ctx_store_ptr(ctx, QRK3588_DDR_CTX_TOPOLOGY_REGS_OFFSET, UINT32_C(0xfd588000));
    qrk3588_ddr_ctx_store_ptr(ctx, QRK3588_DDR_CTX_SHARED_BANK0_OFFSET, UINT32_C(0xfd59c000));
    qrk3588_ddr_ctx_store_ptr(ctx, QRK3588_DDR_CTX_SHARED_BANK1_OFFSET, UINT32_C(0xfd59d000));
}

/*
 * The blob can enter with an active controller and quiesces it before
 * reconfiguration. BL2 also enters directly from MaskROM, where no controller
 * is running and the reset-status handshake has no producer.
 */
bool ddr_controller_reset_required(const void *ctx)
{
    uintptr_t phy = qrk3588_ddr_ctx_channel_base0(ctx, 0u);

    return (RD32(phy + PHY_INIT_STATE_REG) & PHY_INIT_STATE_MASK) != 0u;
}

/* Reset an active DDR controller before rebuilding its configuration. */
void ddr_controller_reset_handshake(void *ctx)
{
    volatile uint32_t *ctrl = (volatile uint32_t *)
        (qrk3588_ddr_ctx_reset_regs(ctx) + 0x8000u);

    ctrl[0x110u / 4u] = 0xf000f000u;
    if (qrk3588_ddr_poll_any32((uintptr_t)&ctrl[0x118u / 4u], 0xf0000000u, 0u) != 0)
        return;
    if (qrk3588_ddr_poll_any32((uintptr_t)&ctrl[0x120u / 4u], 0xf0000000u, 0u) != 0)
        return;

    ctrl[0x160u / 4u] = 0x00030003u;
    ctrl[0x154u / 4u] = 0x00030003u;
    if (qrk3588_ddr_poll_any32((uintptr_t)&ctrl[0x184u / 4u], 3u, 0u) != 0)
        return;
    ctrl[0x154u / 4u] = 0x00030000u;
    if (qrk3588_ddr_poll32((uintptr_t)&ctrl[0x184u / 4u], 3u, 0u, 0u) != 0)
        return;
    ctrl[0x160u / 4u] = 0x00030000u;
    ctrl[0x110u / 4u] = 0xf0000000u;
}

/* DDR frequency divider encoding */

/*
 * DDR clock-divider encoding.
 *
 * The divider encoder takes a requested DDR clock in *mhz, clamps it
 * to the range implemented by the selected DRAM PHY profile, rounds the
 * stored clock down to that profile's divider, and returns the encoded
 * divider value.  Bit 6 selects the upper divider range.
 */
uint32_t ddr_freq_divider_encode(uint32_t *mhz, uint32_t dram_type)
{
    uint32_t value = *mhz;
    if (dram_type == 7u) {
        if (value < 100u)
            value = 100u;
        if (value > 420u)
            value = 420u;
        *mhz = (value / 4u) * 4u;
        if (value <= 300u)
            return (value - 100u) / 4u;
        return ((value - 220u) / 4u) | 0x40u;
    } else if (dram_type == 8u) {
        if (value < 150u)
            value = 150u;
        if (value > 629u)
            value = 629u;
        *mhz = (value / 6u) * 6u;
        if (value <= 449u)
            return (value - 150u) / 6u;
        return ((value - 329u) / 6u) | 0x40u;
    } else {
        if (value < 100u)
            value = 100u;
        if (value > 735u)
            value = 735u;
        *mhz = (value / 5u) * 5u;
        return (value - 100u) / 5u;
    }
}
