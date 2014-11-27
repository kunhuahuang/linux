/*
 * STM32 machine
 */

#include <asm/mach/arch.h>
#include <asm/v7m.h>
#include <linux/kernel.h>

static const char *const stm32_compat[] __initconst = {
	"st,stm32f429i-disco",
	NULL
};

DT_MACHINE_START(STM32DT, "STM32 (Device Tree Support)")
	.dt_compat = stm32_compat,
	.restart = armv7m_restart,
MACHINE_END
