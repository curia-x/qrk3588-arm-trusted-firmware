/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* QRK3588 DDR driver internal interface. */

#ifndef QRK3588_DDR_INTERNAL_H
#define QRK3588_DDR_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <drivers/delay_timer.h>

#include "qrk3588_ddr_runtime.h"
#include "qrk3588_ddr_context.h"
#include "qrk3588_ddr_state.h"

/* 地址空间 */

#define DDR_CTRL0_BASE      0x2a380000u   /* DDR Controller Channel 0 */
#define DDR_CTRL1_BASE      0x2a390000u   /* DDR Controller Channel 1 */
#define DDR_PHY0_BASE       0x2a390100u   /* DDR PHY Channel 0 */
#define DDR_PHY1_BASE       0x2a490100u   /* DDR PHY Channel 1 */

/* DDR PHY 间接寄存器访问模型 */

#define PHY_OFFSET              0x10000u
#define PHY_CTRL_REG            0x80u   /* 寄存器地址 (w1<<4) | START(bit31) */
#define PHY_DATA_REG            0x84u   /* 写入数据 */
#define PHY_STATUS_REG          0x90u   /* bit0: PHY 空闲 */
#define PHY_COMPLETE_TRIGGER    0xC80u
#define PHY_COMPLETE_STATUS     0xC84u
#define PHY_UPDATE_CTRL_REG     0x50u
#define PHY_UPDATE_START_BIT    (1u << 24)
#define PHY_INIT_STATE_REG      (PHY_OFFSET + 0x14u)
#define PHY_INIT_STATE_MASK     0x7u

int     qrk3588_ddr_main(void *config);
uint32_t ddr_reference_clock_hz(void);
uint32_t ddr_phy_reg_write(uint32_t phy_base,       /* PHY 间接写 */
                           uint32_t reg_addr,
                           uint32_t data_hi, uint32_t data_lo);
void    ddr_phy_update_complete(uint32_t phy_base); /* PHY 更新触发 */
void    ddr_phy_update_toggle(uintptr_t ctrl_base);  /* PHY 更新开关 */
int     phy_update_seq(void *state, void *context, uint32_t channel,
                       uint32_t count, void *output_a, void *output_b);
int     ddr_config_write_seq(void *context, uint32_t channel, uint32_t mode,
                                    uint8_t *state, uint32_t arg4, uint32_t arg5,
                                    uint32_t operation_flags, uint32_t arg7,
                                    uint32_t control);
int     ddr_frequency_switch_program_sequence(void *context, uint8_t *config,
                                              uint32_t profile);
int     ddr_vref_training_scan(uint32_t selector, int32_t mode,
                               uint32_t direct, uint32_t channel,
                               const uint32_t *reference_code,
                               const void *limits, void *records);
int     ddr_eye_scan_report(void *config);          /* eye/Vref scan report */
int     ddr_config_check(const void *config);       /* 参数校验 */
void    ddr_memset(void *dst, int c, size_t n);
uintptr_t ddr_memcpy(void *dst, const void *src, size_t n); /* memcpy */

/* ---- Debug output ---- */
void    dbg_putc(char c);                           /* 单字符输出 */
char   *itoa_base(int value, char *buf, unsigned base); /* 带进制 itoa,
                                               返回写入末尾指针 */
void    dbg_uart_drain(void);                       /* 等发送排空 */
void    dbg_puts(const char *s);                    /* 字符串输出 (\n 前
                                               补 \r) */
void    print_dec(uint32_t v);                      /* 递归十进制打印 */
void    ddr_printf(const char *fmt, ...);           /* 格式化引擎 */
/* ---- DRAM information reporting ---- */
void    printhex(uint32_t a, uint32_t b, int mode); /* "0x%x " 打印 */
void    dram_type_name(uint32_t type);              /* 类型名打印 */
void    print_dram_type(const void *info);          /* 类型+频率 */
/* ---- Configuration/reporting helpers ---- */
int     report_bad_magic(void);
int     report_tag_overflow(const void *tag);
int     report_dram_type_unsupported(void);
int     dram_type_detect(void);

/* ---- Configuration parsing and training reports ---- */
uint32_t config_hash(const void *p, uint32_t len);  /* Rockchip configuration hash */
int     config_tag_validate(uint32_t tag);          /* configuration tag validation */
int     config_tag_parse(uint32_t tag, void *buf);  /* 标签链表解析/插入 */
void    print_dram_geometry(const void *geom,       /* 几何布局打印 */
                            const void *info,
                            uint32_t mode);
void    print_dram_info(uint32_t chan, uint32_t flag, /* 通道信息打印 */
                        const void *geom, const void *info,
                        uint32_t mode);
int     config_check_version(void);                 /* 版本 tag 插入 */
int     config_reparse(uint32_t dram_type);         /* 二次解析建表 */
void    print_train_result(uint32_t chan,           /* 训练结果报告 */
                           const void *results);

/* ---- PHY, clock and configuration helpers ---- */
void    io_grf_handshake_release(void);                      /* GRF 握手写 A */
void    io_grf_handshake_assert(void);                      /* 握手写 B 变体 */
uint32_t ddr_encode_inverted_mode_word(uint32_t mode, uint32_t val);  /* 模式字生成 */
void    ddr_word_copy(void *dst, const void *src,   /* 字粒度拷贝 */
                      uint64_t len);
void    ddr_clock_triplet_write(volatile uint32_t *base,        /* 3 位域打包写 */
                    uint32_t idx, uint32_t val);
const void *dram_type_cfg_ptr(uint32_t dram_type);  /* 配置表指针 */
void    ddr_publish_phy_channel_status(void);                          /* 通道状态轮询 */
uint32_t ddr_phy_config_block(volatile void *phy,   /* PHY 配置块 */
                              uint32_t a, uint32_t b);
uint32_t ddr_phy_trigger_begin(volatile void *phy);           /* PHY 触发置位 */
void    ddr_phy_trigger_end(volatile void *phy,             /* 触发清除变体 */
                    uint32_t flag);
const void *freq_cfg_ptr(uint32_t dram_type,        /* 频率档位指针 */
                         uint32_t freq);
void    print_dram_info_chan(const void *unused,    /* 通道封装
                                                  (unused first-parameter quirk) */
                             uint32_t chan,
                             const void *base);
uint32_t ddr_cfg_byte(void);                        /* 默认配置项 */
void    ddr_clock_regs_save_restore(void *context, void *state, uint32_t save);
void    ddr_sync_controller_b4_high11(void *context, void *state, uint32_t channel);
void    ddr_clear_aux_status_words(const void *s);                 /* 双指针清零 */
uint32_t ddr_phy_request_pause(volatile void *p);             /* +0x20000 握手 */
void    ddr_phy_request_restore(volatile void *p, uint32_t en); /* 条件断言 */
void    ddr_context_base_table_init(uint64_t *ctx);                 /* MMIO 基址表初始化 */
uint32_t ddr_freq_divider_encode(uint32_t *mhz,     /* DDR 频率限幅/分频编码 */
                                 uint32_t dram_type);
int     ddr_phy_lane_calibrate(void *lane_table,    /* PHY lane 校准握手 */
                               uint32_t lane, uint32_t select);
void    ddr_phy_state_capture_and_clear(void *ctx,  /* 保存并清理 PHY 通道状态 */
                                        uint32_t channel, void *snapshot);
void    ddr_phy_channel_init(void *ctx);            /* PHY/控制表通道预置 */
bool    ddr_controller_reset_required(const void *ctx);
void    ddr_controller_reset_handshake(void *ctx);  /* 控制器复位握手 */
void    ddr_phy_apply_byte_config(uint32_t phy_base, const uint8_t *cfg);
void    ddr_phy_timing_select(void *ctx, uint32_t channel, const void *state, uint32_t selector);
int     ddr_phy_command(const uint32_t *freq_mhz, void *ctx,
                        uint32_t channel, uint32_t mode,
                        uint32_t completion_token);
void    ddr_phy_frequency_register_image_build(void *ctx, uint32_t channel, uint8_t *state,
                                                 uint32_t selector, uint32_t profile,
                                                 uint32_t use_freq_cfg_flag);
void    ddr_phy_frequency_parameter_stage(void *ctx, uint32_t channel, uint8_t *state,
                                             uint32_t profile, uint32_t mode,
                                             uint32_t use_override);
void    ddr_phy_mode_config(void *ctx, uint32_t channel, void *state,
                            uint32_t short_path, uint32_t extra);
uint32_t ddr_phy_restore_margin(void *ctx, uint32_t channel, void *snapshot);
void ddr_debug_snapshot(uint32_t descriptor);
void ddr_phy_apply_profile_registers(uint32_t phy_base, uint32_t selector,
                         uint32_t dram_type);
void ddr_phy_channel_restore_and_program(void *ctx, uint32_t channel,
                                         void *snapshot);
uint32_t ddr_training_limits(uint32_t *out0, uint32_t *out1, uint32_t *out2,
                             uint32_t scaled, uint32_t channel);
uint32_t ddr_training_apply_limit(uint32_t limit, uint32_t selector,
                                  uint32_t direct, uint32_t channel);
int ddr_training_mirror_entry(uint32_t mode, uint32_t channel, uint32_t clear,
                              uint32_t channel_mask);
int ddr_training_mirror(uint32_t mode, void *state, uint32_t clear,
                        uint32_t channel_mask);
int ddr_training_mirror_invalid(void);
void ddr_capture_channel_training_state(void *context, uint32_t channel,
                                        uint8_t *snapshot,
                                        const uint8_t *rank_info);
void ddr_phy_timing_program(uint32_t phy_base, uint32_t channel,
                            const struct qrk3588_ddr_channel_geometry *geometry,
                            uint32_t dram_type, uint32_t setting);
void ddr_phy_reg_read(void *context, uint32_t channel,
                      const uint32_t *config, uint32_t command,
                      uint32_t frequency);
int phy_reg_rw_seq(void *context, uint32_t channel, uint32_t mode,
                   uint8_t *config);
void config_pctl_regs_init(void *context, uint32_t channel,
                           uint8_t *config, uint32_t profile);
void ddr_channel_topology_and_timing_init(void *context, uint8_t *config);
int grf_ddr_config(void *context, uint8_t *config,
                   const uint32_t hash_template[7], uint32_t profile);
int ddr_training_window(int32_t *low, int32_t *high, uint32_t read_set,
                        int32_t sample, uint32_t ten_bit, uint32_t channel);
int ddr_training_adjust(int32_t delta, uint32_t read_set, int32_t sample,
                        uint32_t ten_bit, uint32_t channel);
int ddr_training_phase_exec(void *context, uint32_t limits[3],
                            uint32_t selector, int32_t mode,
                            uint32_t direct, uint32_t channel,
                            uint32_t report);
void print_mode_registers(uint8_t *config, uint32_t verbose);
int ddr_build_frequency_params(uint8_t *config, uint8_t *dram_params,
                                   uint8_t *timing, uint32_t frequency,
                                   uint32_t dram_type, uint32_t preserve_geometry);
int train_channels_loop(void *context, uint32_t selector, int32_t mode,
                        uint32_t direct, uint32_t *result, uint32_t channel);
void ddr_channel_timing_pack(void *context, uint32_t channel, uint8_t *config);
void ddr_pctl_frequency_register_image_build(void *context, uint32_t channel, uint8_t *config, uint32_t profile);
void    ddr_phy_state_restore(void *ctx, uint32_t channel, void *snapshot);

/* ---- IO-GRF / NIU configuration ---- */
void    io_grf_range_clear(const uint32_t *tbl0,   /* 窗口清零+填充 */
                           const uint32_t *tbl1,
                           uint32_t shift, uint32_t sel, uint32_t val);
uint32_t io_config_program_mode0(uint64_t addr, uint64_t len, /* 配置序列 A */
                         const uint32_t *tbl, uint32_t b,
                         uint32_t c, uint32_t d);
uint32_t io_config_program_mode3(uint64_t addr, uint64_t len, /* 配置序列 B */
                         const uint32_t *tbl, uint32_t b,
                         uint32_t c, uint32_t d, uint32_t e);
int     io_config_dispatch(uint64_t a, uint32_t b, /* 4 相位聚合 */
                           uint32_t sel2, uint32_t mode,
                           uint32_t val, uint32_t odt);
/* IOC region descriptor. */
typedef struct { uintptr_t base; uint32_t size; } ioc_region_t;
ioc_region_t ddr_ioc_grf_init(void);               /* IOC GRF 初始化 */

/* ---- Register and PLL helpers ---- */
void    ddr_sram_workspace_reset_if_tagged(void);                         /* SRAM 魔数 memset */
uint64_t ddr_geometry_capacity_component(
                    const struct qrk3588_ddr_channel_geometry *geometry,
                    uint32_t selector, uint32_t dram_type);
void    ddr_program_rank_geometry_fields(
                    const struct qrk3588_ddr_channel_geometry *geometry,
                    const uint32_t *timing,
                    volatile uint32_t *ra,
                    volatile uint32_t *rb, uint32_t cs);
void    ddr_preset_to_dram_params(const uint32_t *cfg,           /* 频率档位映射 */
                    uint32_t *out, uintptr_t fill);
uintptr_t ddr_copy_descriptor_block(void *a, const void *b,  /* copy helper */
                    uint32_t w, uint32_t w3);
void    ddr_pctl_trigger_handshake(volatile void *p);             /* 握手轮询 */
void    fout_mhz_setup(void *ctx, uint32_t idx,    /* PLL 频率配置 */
                       uint32_t flag, uint32_t fout_mhz);
int     ddr_training_prepare(void);                         /* 训练总入口 */

/* High-level DDR orchestration. */

int  phy_dvfs_config(uint32_t frequency_mhz, uint32_t profile);
int  ddr_training_main(void);

/* Fixed-layout parameter interfaces */

void ddr_topology_region_normalize(void *context, void *aux, void *out, uint32_t flag);
void sdram_params_init(void);

void ddr_timing_base_calculate(uint8_t *dram, uint8_t *timing, uint32_t dram_type);
int ddr_timing_parameter_synthesize(uint8_t *dram, uint8_t *timing);

#endif /* QRK3588_DDR_H */
