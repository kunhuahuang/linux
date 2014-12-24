/*
 * STM32 RCC reset driver
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>

struct stm32_rcc_reset_data {
	struct reset_controller_dev rcdev;
	void __iomem *regs;
};

static int stm32_rcc_reset_assert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	return 0;
}

static int stm32_rcc_reset_deassert(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	return 0;
}

static struct reset_control_ops stm32_rcc_reset_ops = {
	.assert = stm32_rcc_reset_assert,
	.deassert = stm32_rcc_reset_deassert,
};

static int stm32_rcc_reset_probe(struct platform_device *pdev)
{
	struct stm32_rcc_reset_data *data;
	struct resource *res;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->regs = ioremap(res->start, resource_size(res));
	if (!data->regs) {
		kfree(data);
		return -ENOMEM;
	}

	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = 0;
	data->rcdev.ops = &stm32_rcc_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, data);

	pr_info("RCC reset\n");

	return reset_controller_register(&data->rcdev);
}

static int stm32_rcc_reset_remove(struct platform_device *pdev)
{
	struct stm32_rcc_reset_data *data = platform_get_drvdata(pdev);

	reset_controller_unregister(&data->rcdev);

	return 0;
}

static const struct of_device_id stm32_rcc_reset_of_matches[] = {
	{ .compatible = "st,stm32f429-rcc", },
	{ },
};
MODULE_DEVICE_TABLE(of, stm32_rcc_reset_of_matches);

static struct platform_driver stm32_rcc_reset_driver = {
	.driver = {
		.name		= "stm32-rcc-reset",
		.of_match_table	= stm32_rcc_reset_of_matches,
	},
	.probe	= stm32_rcc_reset_probe,
	.remove	= stm32_rcc_reset_remove,
};
module_platform_driver(stm32_rcc_reset_driver);

MODULE_AUTHOR("Andreas Faerber <afaerber@suse.de>");
MODULE_DESCRIPTION("STM32 Reset Controller Driver");
MODULE_LICENSE("GPL");
