/*
 * STM32 TIM
 */
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define M3_SYSTICK_BASE		0xe000e010
#define M3_SYSTICK_CTRL		0x00
#define M3_SYSTICK_LOAD		0x04
#define M3_SYSTICK_VAL		0x08
#define M3_SYSTICK_LOAD_RELOAD_MASK	0x00ffffff
#define M3_SYSTICK_LOAD_RELOAD_BITS	24
#define M3_SYSTICK_CTRL_EN		(1 << 0)

static u32 systick_read(unsigned offset)
{
	void __iomem *base = (void __iomem *)M3_SYSTICK_BASE;

	return readl_relaxed((u8 *)base + offset);
}

static void systick_write(unsigned offset, u32 value)
{
	void __iomem *base = (void __iomem *)M3_SYSTICK_BASE;

	writel_relaxed(value, (u8 *)base + offset);
}

static cycle_t m3_systick_read(struct clocksource *clksrc)
{
	u32 val;

	val = systick_read(M3_SYSTICK_VAL);
	val ^= M3_SYSTICK_LOAD_RELOAD_MASK;
	return val;
}

static struct clocksource m3_systick_clocksource = {
	.name = "cortex-m3-systick",
	.rating = 200,
	.read = m3_systick_read,
	.mask = CLOCKSOURCE_MASK(M3_SYSTICK_LOAD_RELOAD_BITS),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

static __init void m3_systick_init(struct device_node *node)
{
	void __iomem *base;
	struct clk *clk;
	unsigned long rate;
	u32 ctrl;
	int ret;

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk)) {
		pr_err("failed to get clock for AHB (%d)\n", ret);
		goto err_clk_get;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("failed to enable clock for AHB (%d)\n", ret);
		goto err_clk_enable;
	}
	rate = clk_get_rate(clk) / 8;

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("failed to map timer\n");
		goto err_iomap;
	}

	pr_info("systick @ 0x%p (%lu)\n", base, rate);

	systick_write(M3_SYSTICK_LOAD, M3_SYSTICK_LOAD_RELOAD_MASK);
	systick_write(M3_SYSTICK_VAL, 0);
	ctrl = systick_read(M3_SYSTICK_CTRL);
	ctrl |= M3_SYSTICK_CTRL_EN;
	systick_write(M3_SYSTICK_CTRL, ctrl);

	ret = clocksource_register_hz(&m3_systick_clocksource, rate);
	if (ret) {
		pr_err("registering clock source failed (%d)\n", ret);
		goto err_register;
	}

	return;
err_register:
err_iomap:
	clk_disable_unprepare(clk);
err_clk_enable:
	clk_put(clk);
err_clk_get:
	return;
}

#define STM32_TIM_CR1	0x00
#define STM32_TIM_DIER	0x0c
#define STM32_TIM_SR	0x10
#define STM32_TIM_EGR	0x14
#define STM32_TIM_PSC	0x28
#define STM32_TIM_ARR	0x2c

#define STM32_TIM_CR1_CEN	(1 << 0)
#define STM32_TIM_CR1_ARPE	(1 << 7)

#define STM32_TIM_DIER_UIE	(1 << 0)

#define STM32_TIM_SR_UIF	(1 << 0)

#define STM32_TIM_EGR_UG	(1 << 0)

#define to_stm_clockevent(_evt) container_of(_evt, struct stm32_clock_event_device, evtdev)

struct stm32_clock_event_device {
	struct clock_event_device evtdev;
	void __iomem *regs;
};

static void stm32f4_timer_set_mode(enum clock_event_mode mode,
	struct clock_event_device *clk)
{
	struct stm32_clock_event_device *stm_clk = to_stm_clockevent(clk);
	u32 cr1;

	pr_info("before TIMx CR1 (%pK)\n", stm_clk->regs);
	cr1 = readl_relaxed(stm_clk->regs + STM32_TIM_CR1);
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		cr1 |= STM32_TIM_CR1_CEN;
		break;
	default:
		cr1 &= ~STM32_TIM_CR1_CEN;
		break;
	}
	writel_relaxed(cr1, stm_clk->regs + STM32_TIM_CR1);
}

static struct stm32_clock_event_device stm32_clock_event_device = {
	.evtdev = {
		.name = "STM32 timer",
		.rating = 200,
		.features = CLOCK_EVT_FEAT_PERIODIC,
		.set_mode = stm32f4_timer_set_mode,
		.cpumask = cpu_all_mask,
	},
};

static irqreturn_t stm32_clock_event_handler(int irq, void *data)
{
	struct stm32_clock_event_device *evt = &stm32_clock_event_device;
	u32 sr;

	sr = readl_relaxed(evt->regs + STM32_TIM_SR);
	sr &= ~STM32_TIM_SR_UIF;
	writel_relaxed(sr, evt->regs + STM32_TIM_SR);

	if (evt->evtdev.event_handler)
		evt->evtdev.event_handler(&evt->evtdev);

	return IRQ_HANDLED;
}

static struct irqaction stm32_clock_event_irq = {
	.name = "stm32 clockevent",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = stm32_clock_event_handler,
};

static u32 rcc_read(unsigned offset)
{
	void __iomem *base = (void __iomem *)0x40023800;

	return readl_relaxed((u8 *)base + offset);
}

static void rcc_write(unsigned offset, u32 value)
{
	void __iomem *base = (void __iomem *)0x40023800;

	writel_relaxed(value, (u8 *)base + offset);
}

static __init void stm32f4_timer_init(struct device_node *node)
{
	struct clk *clk;
	void __iomem *base;
	unsigned long rate;
	u32 div, dier, apb1rstr;
	int ret, psc_shift;

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk)) {
		pr_err("failed to get clock for TIMx (%d)\n", ret);
		goto err_clk_get;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("failed to enable clock for TIMx (%d)\n", ret);
		goto err_clk_enable;
	}
	rate = clk_get_rate(clk) * 2;

	/* HACK */
	apb1rstr = rcc_read(0x20);
	apb1rstr |= 0x1;
	rcc_write(0x20, apb1rstr);
	apb1rstr = rcc_read(0x20);
	apb1rstr &= ~0x1;
	rcc_write(0x20, apb1rstr);

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("failed to map timer\n");
		goto err_iomap;
	}
	stm32_clock_event_device.regs = base;

	pr_info("timer @ 0x%p (%lu)\n", base, rate);

	writel_relaxed(STM32_TIM_CR1_ARPE, base + STM32_TIM_CR1);
	div = rate / HZ;
	psc_shift = ilog2(div) - 32 + 1;
	if (psc_shift < 0)
		psc_shift = 0;
	writel_relaxed(div >> psc_shift, base + STM32_TIM_ARR);
	writel_relaxed((1 << psc_shift) - 1, base + STM32_TIM_PSC);
	writel_relaxed(STM32_TIM_EGR_UG, base + STM32_TIM_EGR);

	ret = of_irq_get(node, 0);
	if (ret > 0) {
		stm32_clock_event_device.evtdev.irq = ret;
		setup_irq(ret, &stm32_clock_event_irq);
		dier = readl_relaxed(base + STM32_TIM_DIER);
		dier |= STM32_TIM_DIER_UIE;
		writel_relaxed(dier, base + STM32_TIM_DIER);
	} else
		pr_warn("irq not found for TIM (%d)\n", ret);

	clockevents_config_and_register(&stm32_clock_event_device.evtdev,
					DIV_ROUND_CLOSEST(rate, HZ),
					0xf, 0xfffffff0);

	return;
err_iomap:
	clk_disable_unprepare(clk);
err_clk_enable:
	clk_put(clk);
err_clk_get:
	return;
}
CLOCKSOURCE_OF_DECLARE(m3, "arm,cortex-m3-systick", m3_systick_init);
CLOCKSOURCE_OF_DECLARE(stm32, "st,stm32f4-timer", stm32f4_timer_init);
