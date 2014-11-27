/*
 * STM32F4xxx RCC
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <dt-bindings/clock/stm32f42xxx.h>

#define RCC_PLLCFGR	0x04
#define RCC_CFGR	0x08
#define RCC_APB1ENR	0x40

static struct clk *clk[CLK_NR_CLKS];

static struct clk_onecell_data clk_data = {
	.clks = clk,
	.clk_num = ARRAY_SIZE(clk),
};

static const char *stm32f42xxx_pll_parents[] = {
	"HSI", "HSE"
};

static const char *stm32f42xxx_sysclk_parents[] = {
	"HSI", "HSE", "PLLCLK"
};

static const struct clk_div_table stm32f42xxx_pll_p_dividers[] = {
	{ .val = 0x0, .div = 2 },
	{ .val = 0x1, .div = 4 },
	{ .val = 0x2, .div = 6 },
	{ .val = 0x3, .div = 8 },
	{ }
};

static const struct clk_div_table stm32f42xxx_ahb_dividers[] = {
	{ .val = 0x0, .div = 1 },
	{ .val = 0x1, .div = 1 },
	{ .val = 0x2, .div = 1 },
	{ .val = 0x3, .div = 1 },
	{ .val = 0x4, .div = 1 },
	{ .val = 0x5, .div = 1 },
	{ .val = 0x6, .div = 1 },
	{ .val = 0x7, .div = 1 },
	{ .val = 0x8, .div = 2 },
	{ .val = 0x9, .div = 4 },
	{ .val = 0xa, .div = 8 },
	{ .val = 0xb, .div = 16 },
	{ .val = 0xc, .div = 64 },
	{ .val = 0xd, .div = 128 },
	{ .val = 0xe, .div = 256 },
	{ .val = 0xf, .div = 512 },
	{ }
};

static const struct clk_div_table stm32f42xxx_apb_dividers[] = {
	{ .val = 0x0, .div = 1 },
	{ .val = 0x1, .div = 1 },
	{ .val = 0x2, .div = 1 },
	{ .val = 0x3, .div = 1 },
	{ .val = 0x4, .div = 2 },
	{ .val = 0x5, .div = 4 },
	{ .val = 0x6, .div = 8 },
	{ .val = 0x7, .div = 16 },
	{ }
};

static __init void stm32f42xxx_rcc_init(struct device_node *node)
{
	void __iomem *base;
	int i;

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		clk[i] = ERR_PTR(-ENOENT);

	base = of_iomap(node, 0);
	if (!base) {
		pr_warn("Failed to map address range for RCC node\n");
		return;
	}

	pr_info("rcc @ 0x%p\n", base);

	clk[CLK_HSI] = clk_register_fixed_rate(NULL, "HSI", NULL,
		CLK_IS_ROOT, 16000000);
	clk[CLK_HSE] = clk_register_fixed_rate(NULL, "HSE", NULL,
		CLK_IS_ROOT, 8000000); /* XXX F429-Discovery */

	clk[CLK_PLLSRC] = clk_register_mux(NULL, "PLLSRC",
		stm32f42xxx_pll_parents, 2, 0,
		base + RCC_PLLCFGR, 22, 1, CLK_MUX_READ_ONLY, NULL),
	clk[CLK_PLL] = clk_register_fractional_divider(NULL, "PLL", "PLLSRC",
		0, base + RCC_PLLCFGR, 6, 9, 0, 6, 0, NULL); /* m=N, n=M */
	clk[CLK_PLL48CK] = clk_register_divider(NULL, "PLL48CK", "PLL",
		0, base + RCC_PLLCFGR, 24, 4, CLK_DIVIDER_ONE_BASED, NULL);
	clk[CLK_PLLCLK] = clk_register_divider_table(NULL, "PLLCLK", "PLL",
		0, base + RCC_PLLCFGR, 16, 2, CLK_DIVIDER_ALLOW_ZERO,
		stm32f42xxx_pll_p_dividers, NULL);

	clk[CLK_SYSCLK] = clk_register_mux(NULL, "SYSCLK",
		stm32f42xxx_sysclk_parents, 3, 0,
		base + RCC_CFGR, /*0*/2, 2, CLK_MUX_READ_ONLY, NULL);

	clk[CLK_AHB] = clk_register_divider_table(NULL, "AHB", "SYSCLK",
		0, base + RCC_CFGR, 4, 4, CLK_DIVIDER_ALLOW_ZERO,
		stm32f42xxx_ahb_dividers, NULL);

	clk[CLK_APB1] = clk_register_divider_table(NULL, "APB1", "AHB",
		0, base + RCC_CFGR, 10, 3, CLK_DIVIDER_ALLOW_ZERO,
		stm32f42xxx_apb_dividers, NULL);
	clk[CLK_APB2] = clk_register_divider_table(NULL, "APB2", "AHB",
		0, base + RCC_CFGR, 13, 3, CLK_DIVIDER_ALLOW_ZERO,
		stm32f42xxx_apb_dividers, NULL);

	clk[CLK_TIM2] = clk_register_gate(NULL, "TIM2", "APB1",
		0, base + RCC_APB1ENR, 0, 0, NULL);
	clk[CLK_TIM3] = clk_register_gate(NULL, "TIM3", "APB1",
		0, base + RCC_APB1ENR, 1, 0, NULL);
	clk[CLK_TIM4] = clk_register_gate(NULL, "TIM4", "APB1",
		0, base + RCC_APB1ENR, 2, 0, NULL);
	clk[CLK_TIM5] = clk_register_gate(NULL, "TIM5", "APB1",
		0, base + RCC_APB1ENR, 3, 0, NULL);
	clk[CLK_USART3] = clk_register_gate(NULL, "USART3", "APB1",
		0, base + RCC_APB1ENR, 18, 0, NULL);

	of_clk_add_provider(node, of_clk_src_onecell_get, &clk_data);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (!IS_ERR(clk[i]))
			pr_info("clk %s @ %lu\n", __clk_get_name(clk[i]), __clk_get_rate(clk[i]));
}
CLK_OF_DECLARE(stm32f42xxxrcc, "st,stm32f429-rcc" /* XXX -clk */, stm32f42xxx_rcc_init);
