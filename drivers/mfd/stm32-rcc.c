#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static const struct mfd_cell stm32f429_rcc_devs[] = {
	{
		.name = "stm32-rcc-clk",
		.of_compatible = "st,stm32f429-rcc-clk",
	},
};

static int stm32_rcc_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base;
	int ret;

	pr_info("MFD-RCC probe\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -EINVAL;
	}

	base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!base)
		return -ENOMEM;

	ret = mfd_add_devices(&pdev->dev, 0, stm32f429_rcc_devs,
		ARRAY_SIZE(stm32f429_rcc_devs), res, 0, NULL);
	if (ret)
		return ret;

	pr_info("MFD-RCC probe successful\n");

	return 0;
}

static const struct of_device_id stm32_rcc_of_match[] = {
	{ .compatible = "st,stm32f429-rcc" },
	{}
};
MODULE_DEVICE_TABLE(of, stm_rcc_of_match);

static struct platform_driver stm32_rcc_driver = {
	.driver = {
		.name = "stm32-rcc",
		.owner = THIS_MODULE,
		.of_match_table = stm32_rcc_of_match,
	},
	.probe = stm32_rcc_probe,
};

static __init int stm32_rcc_init(void)
{
	pr_info("MFD-RCC\n");

	return platform_driver_register(&stm32_rcc_driver);
}
core_initcall(stm32_rcc_init);

MODULE_LICENSE("GPL");
