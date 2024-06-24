// SPDX-License-Identifier: GPL-2.0-only

#include <asm-generic/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <dt-bindings/reset/am3517-ip-sw-reset.h>

struct am3517_reset {
	struct reset_controller_dev rcdev;
	struct reset_control *resets;
	void __iomem *base;
};

static int am3517_ctrl_sw_ip_reset_update_bits(struct am3517_reset *reset,
					       unsigned long id, unsigned char val)
{
	int ret = -EINVAL;
	int reg;

	switch (id) {
	case USB20OTGSS_SW_RST:
	case CPGMACSS_SW_RST:
	case VPFE_VBUSP_SW_RST:
	case HECC_SW_RST:
	case VPFE_PCLK_SW_RST:
		reg = readl(reset->base);
		if (val)
			reg |= (1 << (unsigned int) id);
		else
			reg &= ~(1 << (unsigned int) id);
		writel(reg, reset->base);
		/* OCP Barrier */
		reg = readl(reset->base);
	}
	return ret;
}
static int am3517_ctrl_sw_ip_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct am3517_reset *reset = container_of(rcdev, struct am3517_reset, rcdev);

	return am3517_ctrl_sw_ip_reset_update_bits(reset, id, 1);
}

static int am3517_ctrl_sw_ip_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct am3517_reset *reset = container_of(rcdev, struct am3517_reset, rcdev);

	return am3517_ctrl_sw_ip_reset_update_bits(reset, id, 0);
}

const struct reset_control_ops am3517_ctrl_sw_ip_reset_ops = {
	.assert = am3517_ctrl_sw_ip_reset_assert,
	.deassert = am3517_ctrl_sw_ip_reset_deassert,
};

static int am3517_ctrl_sw_ip_reset_probe(struct platform_device *pdev)
{
	struct am3517_reset *am3517_ctrl_sw_ip;
	struct device *dev = &pdev->dev;

	am3517_ctrl_sw_ip = devm_kzalloc(dev, sizeof(*am3517_ctrl_sw_ip), GFP_KERNEL);
	if (!am3517_ctrl_sw_ip)
		return -ENOMEM;

	am3517_ctrl_sw_ip->base = devm_platform_ioremap_resource(pdev, 0);

	if (IS_ERR(am3517_ctrl_sw_ip->base))
		return PTR_ERR(am3517_ctrl_sw_ip->base);

	am3517_ctrl_sw_ip->rcdev.owner = THIS_MODULE;
	am3517_ctrl_sw_ip->rcdev.nr_resets = 5;
	am3517_ctrl_sw_ip->rcdev.ops = &am3517_ctrl_sw_ip_reset_ops;
	am3517_ctrl_sw_ip->rcdev.of_node = dev->of_node;

	return devm_reset_controller_register(dev, &am3517_ctrl_sw_ip->rcdev);
}

static const struct of_device_id am3517_ctrl_sw_ip_reset_ids[] = {
	{ .compatible = "ti,am35xx-ip_sw_reset",  },
	{ /* sentinel */ },
};

static struct platform_driver am3517_ctrl_sw_ip_reset_driver = {
	.probe	= am3517_ctrl_sw_ip_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= am3517_ctrl_sw_ip_reset_ids,
	},
};
module_platform_driver(am3517_ctrl_sw_ip_reset_driver);

MODULE_AUTHOR("Adam Ford <aford173@gmail.com>");
MODULE_DESCRIPTION("TI AM35xx Control IP SW Reset");
MODULE_LICENSE("GPL");

