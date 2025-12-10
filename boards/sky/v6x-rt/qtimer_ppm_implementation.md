# QTIMER PPM Implementation Guide

## 概述

由于硬件 Rev 01 将 PPM 输入从 GPIO_EMC_B1_09 (GPT5_CAPTURE1) 改为 GPIO_EMC_B2_12 (QTIMER1_TIMER3)，需要实现基于 QTIMER 的 PPM 解码支持。

## 修改文件

### 1. 已完成的修改

- ✅ `/boards/sky/v6x-rt/src/board_config.h` - 添加 QTIMER PPM 配置
- ✅ `/boards/sky/v6x-rt/nuttx-config/include/board.h` - 添加 GPIO 定义

### 2. 需要修改的文件

- `/platforms/nuttx/src/px4/nxp/imxrt/hrt/hrt.c` - 核心 HRT 驱动

## QTIMER vs GPT 的区别

| 特性 | GPT (General Purpose Timer) | QTIMER (Quad Timer) |
|------|----------------------------|---------------------|
| 通道数 | 3个比较/捕获通道 | 4个独立定时器通道 |
| 输入捕获 | IC1, IC2 | CAPT (每个通道) |
| 计数器宽度 | 32-bit | 16-bit |
| 时钟源 | 可配置 | 可配置 |
| 中断 | 每通道独立 | 每定时器独立 |

## 实现步骤

### Step 1: 添加 QTIMER 寄存器定义

在 `hrt.c` 顶部添加 QTIMER 头文件：

```c
#ifdef HRT_PPM_QTIMER
#include "hardware/imxrt_tmr.h"  // QTIMER registers
#endif
```

### Step 2: 定义 QTIMER 基地址和寄存器宏

```c
#ifdef HRT_PPM_QTIMER
/* QTIMER1 for PPM input */
#define QTIMER_PPM_BASE          IMXRT_QTIMER1_BASE
#define QTIMER_PPM_CHANNEL       HRT_PPM_QTIMER_CHANNEL  // Channel 3

/* QTIMER channel register offsets (each channel is 0x20 bytes) */
#define QTIMER_CH_OFFSET         (0x20)
#define QTIMER_CH_BASE(ch)       (QTIMER_PPM_BASE + ((ch) * QTIMER_CH_OFFSET))

/* QTIMER registers for PPM channel */
#define rQTIMER_COMP1            (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_COMP1_OFFSET))
#define rQTIMER_COMP2            (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_COMP2_OFFSET))
#define rQTIMER_CAPT             (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_CAPT_OFFSET))
#define rQTIMER_LOAD             (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_LOAD_OFFSET))
#define rQTIMER_HOLD             (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_HOLD_OFFSET))
#define rQTIMER_CNTR             (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_CNTR_OFFSET))
#define rQTIMER_CTRL             (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_CTRL_OFFSET))
#define rQTIMER_SCTRL            (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_SCTRL_OFFSET))
#define rQTIMER_CMPLD1           (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_CMPLD1_OFFSET))
#define rQTIMER_CMPLD2           (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_CMPLD2_OFFSET))
#define rQTIMER_CSCTRL           (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_CSCTRL_OFFSET))
#define rQTIMER_FILT             (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_FILT_OFFSET))
#define rQTIMER_DMA              (*(volatile uint16_t *)(QTIMER_CH_BASE(QTIMER_PPM_CHANNEL) + IMXRT_TMR_DMA_OFFSET))
#define rQTIMER_ENBL             (*(volatile uint16_t *)(QTIMER_PPM_BASE + IMXRT_TMR_ENBL_OFFSET))

/* QTIMER Control Register bits */
#define QTIMER_CTRL_CM_MASK      (0x7 << 13)  /* Count Mode */
#define QTIMER_CTRL_CM_RISING    (0x1 << 13)  /* Count rising edges of primary source */
#define QTIMER_CTRL_PCS_MASK     (0xF << 9)   /* Primary Count Source */
#define QTIMER_CTRL_PCS_DIV1     (0x8 << 9)   /* IP bus clock divide by 1 */
#define QTIMER_CTRL_SCS_MASK     (0x3 << 7)   /* Secondary Count Source */
#define QTIMER_CTRL_SCS_INPUT    (0x0 << 7)   /* Input pin */
#define QTIMER_CTRL_ONCE         (1 << 6)     /* Count once */
#define QTIMER_CTRL_LENGTH       (1 << 5)     /* Count until compare, then re-init */
#define QTIMER_CTRL_DIR          (1 << 4)     /* Count direction (0=up, 1=down) */
#define QTIMER_CTRL_COINIT       (1 << 3)     /* Co-channel initialization */
#define QTIMER_CTRL_OUTMODE_MASK (0x7 << 0)   /* Output Mode */

/* QTIMER Status and Control Register bits */
#define QTIMER_SCTRL_TCFIE       (1 << 15)    /* Timer Compare Flag Interrupt Enable */
#define QTIMER_SCTRL_TCF         (1 << 14)    /* Timer Compare Flag */
#define QTIMER_SCTRL_TOFIE       (1 << 13)    /* Timer Overflow Flag Interrupt Enable */
#define QTIMER_SCTRL_TOF         (1 << 12)    /* Timer Overflow Flag */
#define QTIMER_SCTRL_IEF         (1 << 11)    /* Input Edge Flag */
#define QTIMER_SCTRL_IEFIE       (1 << 10)    /* Input Edge Flag Interrupt Enable */
#define QTIMER_SCTRL_IPS         (1 << 9)     /* Input Polarity Select */
#define QTIMER_SCTRL_INPUT       (1 << 8)     /* Input Signal */
#define QTIMER_SCTRL_CAPTURE_MODE_MASK (0x3 << 6)  /* Capture Mode */
#define QTIMER_SCTRL_CAPTURE_BOTH      (0x3 << 6)  /* Capture on both edges */
#define QTIMER_SCTRL_MSTR        (1 << 5)     /* Master Mode */
#define QTIMER_SCTRL_EEOF        (1 << 4)     /* External OFLAG Force */
#define QTIMER_SCTRL_VAL_MASK    (0xF << 0)   /* Capture Value */

/* QTIMER Interrupt Vector */
#define QTIMER1_IRQ              IMXRT_IRQ_QTIMER1

#endif /* HRT_PPM_QTIMER */
```

### Step 3: 初始化 QTIMER PPM

在 `hrt_tim_init()` 函数中添加 QTIMER 初始化：

```c
#ifdef HRT_PPM_QTIMER
static void qtimer_ppm_init(void)
{
	/* Enable QTIMER1 clock */
	imxrt_clockall_tim1();

	/* Disable timer during configuration */
	rQTIMER_ENBL &= ~(1 << QTIMER_PPM_CHANNEL);

	/* Configure GPIO pin for QTIMER input */
	px4_arch_configgpio(GPIO_PPM_IN);

	/* Configure QTIMER for input capture mode */
	rQTIMER_CTRL = 0;  /* Reset control register */

	/* Set count mode: count rising edges of primary source */
	/* Primary source: IP bus clock / 1 (150 MHz) */
	rQTIMER_CTRL = QTIMER_CTRL_CM_RISING |
	               QTIMER_CTRL_PCS_DIV1 |
	               QTIMER_CTRL_LENGTH;  /* Count until compare */

	/* Configure for input capture on both edges */
	rQTIMER_SCTRL = QTIMER_SCTRL_CAPTURE_BOTH |  /* Capture on both edges */
	                QTIMER_SCTRL_IEFIE;           /* Enable input edge interrupt */

	/* Set compare value to maximum for free-running */
	rQTIMER_COMP1 = 0xFFFF;
	rQTIMER_COMP2 = 0xFFFF;

	/* Load initial value */
	rQTIMER_LOAD = 0;

	/* Clear any pending flags */
	rQTIMER_SCTRL |= QTIMER_SCTRL_IEF | QTIMER_SCTRL_TCF | QTIMER_SCTRL_TOF;

	/* Enable QTIMER channel */
	rQTIMER_ENBL |= (1 << QTIMER_PPM_CHANNEL);

	/* Enable QTIMER interrupt in NVIC */
	up_enable_irq(QTIMER1_IRQ);

	spiinfo("QTIMER PPM initialized on QTIMER1 CH%d\\n", QTIMER_PPM_CHANNEL);
}
#endif
```

在 `hrt_tim_init()` 中调用：

```c
void hrt_tim_init(void)
{
	/* ... existing GPT initialization ... */

#ifdef HRT_PPM_QTIMER
	/* Initialize QTIMER for PPM input */
	qtimer_ppm_init();
#endif
}
```

### Step 4: QTIMER PPM 解码

修改 PPM 解码部分以支持 QTIMER：

```c
#ifdef HRT_PPM_QTIMER
static void qtimer_ppm_decode(void)
{
	uint16_t status = rQTIMER_SCTRL;

	/* Check for input edge flag */
	if (status & QTIMER_SCTRL_IEF) {
		/* Read captured value (16-bit counter) */
		uint16_t count = rQTIMER_HOLD;  /* Reading HOLD captures CAPT value */

		/* Clear input edge flag */
		rQTIMER_SCTRL |= QTIMER_SCTRL_IEF;

		/* QTIMER runs at 150 MHz, convert to microseconds */
		/* 1 tick = 1/150 MHz = 6.67 ns */
		/* To get microseconds: count / 150 */
		uint32_t count_us = ((uint32_t)count * 1000) / 150;  /* Convert to microseconds */

		/* Handle 16-bit counter overflow */
		static uint32_t last_count_us = 0;
		static uint32_t overflow_count = 0;

		if (count_us < last_count_us) {
			/* Counter wrapped around */
			overflow_count++;
		}

		uint32_t absolute_time_us = (overflow_count << 16) + count_us;
		uint32_t width = absolute_time_us - ppm.last_edge;

		/* Call existing PPM decode logic */
		ppm_decode_logic(width);  /* This is the existing PPM decode from hrt.c */

		ppm.last_edge = absolute_time_us;
		last_count_us = count_us;
	}

	/* Handle overflow */
	if (status & QTIMER_SCTRL_TOF) {
		rQTIMER_SCTRL |= QTIMER_SCTRL_TOF;  /* Clear overflow flag */
	}
}
#endif
```

### Step 5: 中断处理

修改中断处理器以支持 QTIMER：

```c
static int hrt_tim_isr(int irq, void *context, FAR void *arg)
{
	/* Handle HRT timer (GPT5) */
	uint32_t status = rSR;

	if (status & OFIE_HRT) {
		/* ... existing HRT interrupt handling ... */
	}

#ifdef HRT_PPM_QTIMER
	/* Handle QTIMER PPM input */
	if (irq == QTIMER1_IRQ) {
		qtimer_ppm_decode();
	}
#endif

	/* ... rest of interrupt handling ... */

	return OK;
}
```

### Step 6: 启用 QTIMER 时钟

确保在初始化时启用 QTIMER1 时钟。在 NuttX 中，这通常在板级初始化完成，但需要确认：

```c
/* In board init or hrt_tim_init() */
imxrt_clockall_tim1();  /* Enable QTIMER1 clock */
```

## 测试步骤

### 1. 编译固件

```bash
make sky_v6x-rt_default
```

### 2. 烧录并验证

```bash
make sky_v6x-rt_default upload
```

### 3. 检查 PPM 输入

连接 PPM 信号到 GPIO_EMC_B2_12，然后在 NSH 控制台：

```bash
# 查看 PPM 状态
listener input_rc

# 应该看到通道数据
```

### 4. 调试信息

如果需要调试，可以添加日志：

```c
spiinfo("QTIMER PPM: count=%u, width=%u us\\n", count, width);
```

## 注意事项

1. **时钟频率**: QTIMER 计数器是 16-bit，运行在 150 MHz 时会很快溢出（约437µs）。需要正确处理溢出。

2. **精度**: QTIMER 的 16-bit 计数器在高频率下精度足够用于 PPM 解码（PPM 脉冲宽度通常 > 200µs）。

3. **中断负载**: 每个 PPM 边沿都会触发中断，确保中断处理足够快。

4. **与 HRT 共存**: QTIMER 和 GPT 是独立的，不会互相干扰。

## 可能的问题和解决方案

### 问题 1: QTIMER 计数器溢出太快

**解决方案**: 使用更大的预分频器或者在软件中处理溢出

```c
/* Use prescaler to slow down counter */
rQTIMER_CTRL |= (0x7 << 9);  /* Divide by 128 */
```

### 问题 2: PPM 解码不准确

**解决方案**: 调整捕获模式或增加输入滤波

```c
/* Enable input filter */
rQTIMER_FILT = 0x05;  /* Filter 5 clock cycles */
```

### 问题 3: 中断未触发

**解决方案**: 检查：
- GPIO 引脚配置是否正确
- QTIMER 时钟是否启用
- NVIC 中断是否使能
- 引脚信号是否连接

## 下一步

1. ✅ 修改 board_config.h
2. ✅ 修改 board.h
3. ⏳ 修改 hrt.c（需要手动应用此文档中的修改）
4. ⏳ 编译测试
5. ⏳ 硬件验证

## 参考资料

- i.MX RT1170 Reference Manual Chapter 61: Quad Timer (QTIMER)
- PX4 HRT Documentation
- NuttX IMXRT QTIMER Driver
