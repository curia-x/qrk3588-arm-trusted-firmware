/*
 * Copyright (c) 2026, ninozhang001@gmail.com and Contributors.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* QRK3588 DDR diagnostics, UART logging and report formatting.
 *
 * Fixed strings and runtime scratch state are accessed through named layout
 * slots. Hardware polling uses the driver's bounded polling helpers.
 */

#include "qrk3588_ddr_internal.h"
#include <stdarg.h>

/* ============================================================
 * 运行时工作区地址 (SRAM, QRK3588_DDR_RUNTIME_VA(0x638) 之后;
 * layout pointer slots reference the shared runtime workspace)
 * ============================================================ */

#define DBG_UART_BASE   DDR_LAYOUT_REF(volatile uintptr_t, QRK3588_DDR_DEBUG_UART_SLOT_VA) /* UART 基址指针 */
#define DBG_LOG_POS     DDR_LAYOUT_REF(volatile uint32_t, QRK3588_DDR_DEBUG_LOG_POS_VA)  /* RAM 日志写位置 */
#define DBG_LOG_OVF     DDR_LAYOUT_REF(volatile uint32_t, QRK3588_DDR_DEBUG_LOG_OVERFLOW_VA)  /* 日志溢出标志 */
#define DBG_LOG_WINDOW  0x1fffu     /* 日志缓冲有效范围 */
#define DBG_LOG_WRAP    0xf00000u   /* 写地址回绕常数: pos - 0xf00000 */

/* DW UART register offsets. */
#define UART_THR_OFF    0x00u       /* 发送保持寄存器 */
#define UART_LSR_OFF    0x14u       /* 线路状态: bit6 TEMT */
#define UART_USR_OFF    0x7cu       /* 状态: bit1 忙检测 */
#define UART_LSR_TEMT   0x40u
#define UART_USR_TFNF   0x02u

/* 固定布局中的诊断字符串 */

#define STR_MAGIC_BAD DDR_LAYOUT_CPTR(char, QRK3588_DDR_ERR_MAGIC_UNSUPPORTED_VA)  /* "Magic is not support\n" */
#define STR_TAG_OVERFLOW DDR_LAYOUT_CPTR(char, QRK3588_DDR_ERR_TAG_OVERFLOW_VA)  /* "Tag is overflow\n" */
#define STR_TYPE_UNSUPPORTED DDR_LAYOUT_CPTR(char, QRK3588_DDR_ERR_DRAM_TYPE_UNSUPPORTED_VA) /* "unsupported dram type\n" */
#define STR_SPACE DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_SPACE_VA)  /* " " (printhex 对齐) */
#define STR_0X_FMT DDR_LAYOUT_CPTR(char, QRK3588_DDR_FMT_HEX_WORD_VA)  /* "0x%x " */
#define STR_COMMA DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_COMMA_SPACE_VA)  /* ", " */
#define STR_MHZ_NL DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_MHZ_NEWLINE_VA)  /* "MHz\n" */
/* DRAM 类型名映射: 0=DDR4, 2=DDR2, 3=DDR3, 5=LPDDR2,
 * 6=LPDDR3, 7=LPDDR4, 8=LPDDR4X, 9=LPDDR5; 1/4/其余=Unknown Device. */
#define STR_DDR4 DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_DDR4_VA)
#define STR_DDR2 DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_DDR2_VA)
#define STR_DDR3 DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_DDR3_VA)
#define STR_LPDDR2 DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_LPDDR2_VA)
#define STR_LPDDR3 DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_LPDDR3_VA)
#define STR_LPDDR4 DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_LPDDR4_VA)
#define STR_LPDDR4X DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_LPDDR4X_VA)
#define STR_LPDDR5 DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_LPDDR5_VA)
#define STR_UNKNOWN_DEV DDR_LAYOUT_CPTR(char, QRK3588_DDR_STR_UNKNOWN_DEVICE_VA)

/* 标签配置区 (report_tag_overflow 检查范围; config_tag_parse 同样
 * 直接访问 0x1fe000 区域) */
#define TAG_AREA_LIMIT      QRK3588_DDR_TAG_AREA_SIZE


/* 单字符调试输出 */

/*
 * 行为:
 *   w2 = c & 0xff
 *   x1 = *(QRK3588_DDR_WORKSPACE_VA(0xf80))                ; UART 基址指针
 *   if (x1 != 0):
 *     poll UART_USR bit1 until the transmitter is ready
 *     *(x1) = w2                                   ; THR 写
 *   ; RAM 日志通道 (无条件执行):
 *   pos = the shared diagnostic log position
 *   wrap address = pos - 0xf00000 (32-bit wrap semantics)
 *   *(uint8_t*)w3 = w2
 *   if (pos+1 <= 0x1fff) { *(QRK3588_DDR_WORKSPACE_VA(0xf78)) = pos+1; return; }
 *   *(QRK3588_DDR_WORKSPACE_VA(0xf78)) = 0                 ; 回绕
 *   *(QRK3588_DDR_WORKSPACE_VA(0xf7c)) = 0x2000            ; 溢出标志
 */
void dbg_putc(char c) {
    uint8_t ch = (uint8_t)c;

    /*
     * DDR setup must never block on the boot console.  In the production
     * path ddr_training_prepare() leaves this slot clear, so diagnostics go
     * only to the SRAM ring below.  A board may explicitly provide a
     * dedicated debug UART in this slot; that path is bounded and may drop a
     * character, but cannot stop DDR bring-up.
     */
    uintptr_t uart = DBG_UART_BASE;
    if (uart != 0u) {
        volatile uint32_t *usr = (volatile uint32_t *)(uart + UART_USR_OFF);

        if (qrk3588_ddr_try_poll32((uintptr_t)usr, UART_USR_TFNF,
                                   UART_USR_TFNF, 1000u) == 0)
            *(volatile uint32_t *)(uart + UART_THR_OFF) = ch;
    }

    uint32_t pos = DBG_LOG_POS;
    /* 写地址 = pos - 0xf00000 (32 位运算回绕到 0xff100000+pos) */
    if (qrk3588_ddr_active_context != 0)
        qrk3588_ddr_active_context->debug_log[pos & DBG_LOG_WINDOW] = ch;

    uint32_t next = pos + 1;
    if (next <= DBG_LOG_WINDOW) {
        DBG_LOG_POS = next;
    } else {
        DBG_LOG_POS = 0;            /* 回绕清零 */
        DBG_LOG_OVF = 0x2000;       /* 溢出标志 */
    }
}


/* 带进制整数转字符串 */

/*
 *
 * 行为要点:
 *   w3 = (base == 10); w5 = w3 & (value >> 31)   ; 仅十进制处理负号
 *   if (w5) value = -value                        ; cneg, 32 位回绕
 *   数字循环: rem = value % base; value /= base;
 *             rem>9 ? rem+0x57 : rem+0x30 -> 存 16B 栈缓冲 (低位在前)
 *             循环条件: value != 0 或尚未写过任何数字 (value==0 输出 "0")
 *   负号: *buf = '-'; buf++
 *   倒序拷贝: 从栈缓冲末位向前逐字节拷到 buf
 *   *end = 0; 返回 end (NUL 结尾处指针, ddr_printf 用它推进缓冲位置)
 *
 * quirk: 0x57 偏移给出小写 'a'-'w' (支持 base 至 36);
 *        INT_MIN 取模回绕与原 cneg 行为一致.
 */
char *itoa_base(int value, char *buf, unsigned base) {
    char tmp[16];                    /* 16 字节临时缓冲 */
    unsigned n = 0;
    unsigned v = (unsigned)value;
    int neg = (base == 10) && (value < 0);   /* 仅 base 10 打印负号 */
    if (neg)
        v = (unsigned)(-v);          /* 32 位取负，INT_MIN 保持无符号回绕语义 */

    do {
        unsigned rem = v % base;
        v = v / base;
        tmp[n++] = (char)(rem > 9 ? rem + 0x57u : rem + 0x30u);  /* */
    } while (v != 0);

    char *p = buf;
    if (neg)
        *p++ = '-';
    while (n > 0)
        *p++ = tmp[--n];             /* 倒序拷贝 (高位在前) */
    *p = '\0';
    return p;                        /* 返回写入末尾指针 */
}


/* UART 发送排空 */

/* Wait for UART transmit-empty when a debug UART is configured. */
void dbg_uart_drain(void) {
    uintptr_t uart = DBG_UART_BASE;
    if (uart != 0) {
        volatile uint32_t *lsr = (volatile uint32_t *)(uart + UART_LSR_OFF);
        (void)qrk3588_ddr_try_poll32((uintptr_t)lsr, UART_LSR_TEMT, UART_LSR_TEMT, 1000u);
    }
}


/* 字符串调试输出 */

/*
 * 循环: c = *s; if (!c) return;
 *       if (c == '\n') dbg_putc('\r');    ; 终端换行 (CR+LF)
 *       dbg_putc(*s++);
 */
void dbg_puts(const char *s) {
    s = (const char *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)s);
    while (*s) {
        if (*s == '\n')
            dbg_putc('\r');
        dbg_putc(*s++);
    }
}


/* 递归十进制打印 */

/*
 * if (v > 9) { q = v/10; r = v - q*10; print_dec(q); v_rem = r; }
 * else v_rem = v;
 * Emits the final decimal digit through dbg_putc().
 * (udiv/msub -> 无符号除法; 25 个调用点均传频率等非负值)
 */
void print_dec(uint32_t v) {
    if (v > 9)
        print_dec(v / 10);
    dbg_putc((char)('0' + (v % 10)));
}


/* 格式化打印引擎 */

/*
 * 行为约束: 栈上手工构造 va_list
 *   [x29+0x30] = 栈参数区基址; [x29+0x40] = FP 保存区顶 (x29+0xf0);
 *   gr_offs = -0x38 (-56, x1-x7 共 7 个 GP 寄存器);
 *   vr_offs = -0x80 (-128, q0-q7, 保存但无格式说明符消费 -> C 版
 *   va_arg 由编译器生成等价序列)
 * 输出: x1 = *(char**)QRK3588_DDR_RUNTIME_VA(0x5f8) (printf_buffer = QRK3588_DDR_EXTENDED_VA(0xe68));
 *   结束: buffer[cursor-buffer] = 0; dbg_puts(buffer).
 *
 * 说明符分派 (二分跳转树解码, quirks 见文件头):
 *   'd' 'i' 'o' 'u' -> itoa_base(arg, cursor, 10)
 *   'x' 'X'         -> itoa_base(arg, cursor, 16)
 *   'c'             -> cursor[0] = (char)arg
 *   's'             -> 逐字节拷贝 arg 指向的字符串
 *   '%'             -> cursor[0] = '%'
 *   其余            -> 不输出任何字符, fmt 跳 2
 */
void ddr_printf(const char *fmt, ...) {
    fmt = (const char *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)fmt);
    va_list ap;
    va_start(ap, fmt);

    char *buffer = (char *)(uintptr_t)DDR_LAYOUT_REF(uintptr_t, QRK3588_DDR_PRINTF_BUFFER_SLOT_VA);
    char *cur = buffer;   /* QRK3588_DDR_RUNTIME_VA(0x5f8) 处指针值 */

    while (*fmt) {
        char c = *fmt;

        if (c != '%') {
            *cur++ = c;
            fmt++;
            continue;
        }

        switch (fmt[1]) {
        case '%':
            *cur++ = '%';
            break;
        case 'd':
        case 'i':
        case 'o':
        case 'u':
            /* quirk: %o 与 %u 均按 base 10 处理; bit31 置位时
             * itoa_base prints '-' followed by the magnitude. */
            cur = itoa_base(va_arg(ap, int), cur, 10);
            break;
        case 'x':
        case 'X':
            /* quirk: %X 同样小写 (itoa_base 固定 +0x57) */
            cur = itoa_base(va_arg(ap, int), cur, 16);
            break;
        case 'c':
            *cur++ = (char)va_arg(ap, int);   /* 32 位槽低字节 */
            break;
        case 's': {
            const char *s = va_arg(ap, const char *);
            s = (const char *)(uintptr_t)qrk3588_ddr_resolve_addr((uintptr_t)s);
            while (*s)
                *cur++ = *s++;
            break;
        }
        default:
            /* 未知说明符: '%' 与该字符均不输出 */
            break;
        }
        fmt += 2;
    }

    *cur = '\0';
    va_end(ap);

    dbg_puts(buffer);
}


/* 十六进制值打印 */

/*
 * 值选择 (mode):
 *   0        -> v = a
 *   1        -> v = (a + b) >> 1
 *   2        -> v = b - a
 *   其余     -> v = b
 * 对齐: v <= 0xf 时先输出 " " (补齐两位十六进制宽度)
 * Prints the value in hexadecimal form.
 * (空格通过同一格式输出路径，结果等价)
 */
void printhex(uint32_t a, uint32_t b, int mode) {
    uint32_t v;

    if (mode == 0)
        v = a;
    else if (mode == 1)
        v = (a + b) >> 1;
    else if (mode == 2)
        v = b - a;
    else
        v = b;

    if (v <= 0xf)
        ddr_printf(STR_SPACE);      /* " " 对齐 */
    ddr_printf(STR_0X_FMT, v);      /* "0x%x " */
}


/* DRAM 类型名打印 */

/* RK3588 DRAM type-code to display-name mapping.
 * 0=DDR4, 2=DDR2, 3=DDR3, 5=LPDDR2, 6=LPDDR3, 7=LPDDR4,
 * 8=LPDDR4X, 9=LPDDR5; unsupported codes are reported as Unknown Device. */
void dram_type_name(uint32_t type) {
    switch (type & 0xff) {
    case 0:  dbg_puts(STR_DDR4);     break;
    case 2:  dbg_puts(STR_DDR2);     break;
    case 3:  dbg_puts(STR_DDR3);     break;
    case 5:  dbg_puts(STR_LPDDR2);   break;
    case 6:  dbg_puts(STR_LPDDR3);   break;
    case 7:  dbg_puts(STR_LPDDR4);   break;
    case 8:  dbg_puts(STR_LPDDR4X);  break;
    case 9:  dbg_puts(STR_LPDDR5);   break;
    default: dbg_puts(STR_UNKNOWN_DEV); break;   /* 1/4/>9 */
    }
}


/* DRAM 类型 + 频率打印 */

/* Print the DRAM type stored at info+4 and frequency stored at info+0,
 * followed by the MHz suffix. */
void print_dram_type(const void *info) {
    const uint8_t *p = (const uint8_t *)info;

    dram_type_name(p[4]);                        /* 类型 @+4 */
    dbg_puts(STR_COMMA);
    print_dec(*(const uint32_t *)p);             /* 频率 @+0 */
    dbg_puts(STR_MHZ_NL);
}


/* 配置报告族 */

/* 打印 "Magic is not support\n", 返回 1 (配置魔数错误) */
int report_bad_magic(void) {
    dbg_puts(STR_MAGIC_BAD);
    return 1;
}

/*
 * 标签范围检查 + 溢出报告
 *
 * 行为:
 *   offset = tag - TAG_AREA_BASE
 *   offset > TAG_AREA_LIMIT -> overflow
 *   length = *(uint32_t *)tag, expressed in 4-byte units
 *   (length * 4 + offset) > TAG_AREA_LIMIT -> overflow
 * Overflow is reported through the diagnostic UART and returned to the caller.
 */
int report_tag_overflow(const void *tag) {
    uintptr_t off = (uintptr_t)tag -
                    qrk3588_ddr_resolve_addr(QRK3588_DDR_TAG_VA_BASE);
    int ovf;

    if (off > TAG_AREA_LIMIT) {
        ovf = 1;
    } else {
        uint32_t len = *(const uint32_t *)tag;
        ovf = ((uint64_t)(len << 2) + off) > TAG_AREA_LIMIT;
    }

    if (ovf)
        dbg_puts(STR_TAG_OVERFLOW);
    return ovf;
}

/* Print "unsupported dram type\n" and return 0. */
int report_dram_type_unsupported(void) {
    dbg_puts(STR_TYPE_UNSUPPORTED);
    return 0;
}


/* 调用图辅助函数 (qrk3588_ddr_main 调用路径) */

/* Prepare the PMU and channel probe status used by cold DDR discovery. */
int dram_type_detect(void)
{
	*(volatile uint32_t *)0xfd5f8098u = 0xff005500u;
	*(volatile uint32_t *)0xfe0100f0u = 0;
	*(volatile uint32_t *)0xfe0100f4u = 0;
	*(volatile uint32_t *)0xfe0100f8u = 0;
	*(volatile uint32_t *)0xfe0100fcu = 0;

	return 0;
}


/* DDR debug snapshot capture */

/* Copy the SRAM debug trace into the caller-provided ring buffer. */
typedef struct {
    uint32_t signature;
    uint32_t write_offset;
    uint32_t used;
    uint8_t data[];
} debug_snapshot_t;

void ddr_debug_snapshot(uint32_t descriptor)
{
    uint32_t capacity = descriptor & 0xf000u;
    uintptr_t address = descriptor & 0xffff0000u;

    if (capacity == 0u)
        capacity = 0x10000u;
    if (address == 0u || (descriptor & 1u) == 0u)
        return;

    debug_snapshot_t *snapshot = (debug_snapshot_t *)address;
    capacity -= sizeof(*snapshot);
    if (snapshot->signature != 0x43474244u) { /* "DBGC" */
        snapshot->signature = 0x43474244u;
        snapshot->write_offset = 0u;
        snapshot->used = 0u;
    }

    if (snapshot->used > capacity)
        snapshot->used = capacity;
    if (snapshot->write_offset >= capacity)
        snapshot->write_offset = 0u;

    if (qrk3588_ddr_active_context == NULL)
        return;

    uint32_t count = DBG_LOG_OVF != 0u ? sizeof(qrk3588_ddr_active_context->debug_log)
                                      : DBG_LOG_POS;
    count &= ~7u;

    const uint8_t *source = qrk3588_ddr_active_context->debug_log;
    uint8_t *destination = snapshot->data + snapshot->write_offset;

    while (count != 0u) {
        while (((uintptr_t)destination & 7u) != 0u) {
            *destination++ = 0x20u;
            if (snapshot->used < capacity)
                ++snapshot->used;
            if (++snapshot->write_offset >= capacity) {
                snapshot->write_offset = 0u;
                destination = snapshot->data;
            }
        }

        if (capacity - snapshot->write_offset <= 7u) {
            snapshot->write_offset = 0u;
            destination = snapshot->data;
            continue;
        }

        *(uint64_t *)destination = *(volatile const uint64_t *)source;
        source += 8u;
        destination += 8u;
        count -= 8u;
        snapshot->write_offset += 8u;
        snapshot->used += 8u;
    }

    if (snapshot->used > capacity)
        snapshot->used = capacity;
}
