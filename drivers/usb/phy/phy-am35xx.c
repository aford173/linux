// SPDX-License-Identifier: GPL-2.0
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include "phy-generic.h"

/* AM35x */
/* USB 2.0 PHY Control */
#define CONF2_PHY_GPIOMODE      (1 << 23)
#define CONF2_OTGMODE           (3 << 14)
#define CONF2_NO_OVERRIDE       (0 << 14)
#define CONF2_FORCE_HOST        (1 << 14)
#define CONF2_FORCE_DEVICE      (2 << 14)
#define CONF2_SESENDEN          (1 << 13)
#define CONF2_VBDTCTEN          (1 << 12)
#define CONF2_REFFREQ_13MHZ     (6 << 8)
#define CONF2_REFFREQ           (0xf << 8)
#define CONF2_PHYCLKGD          (1 << 7)
#define CONF2_PHY_PLLON         (1 << 5)
#define CONF2_RESET             (1 << 4)
#define CONF2_PHYPWRDN          (1 << 3)
#define CONF2_OTGPWRDN          (1 << 2)
#define CONF2_DATPOL            (1 << 1)

struct am35_musb_phy {
	void __iomem *regs;
};

static int am35x_musb_phy_power(struct phy *phy, u8 on)
{
	struct am35_musb_phy *musb_phy = phy_get_drvdata(phy);
	void __iomem *base = musb_phy->regs;
	unsigned long timeout = jiffies + msecs_to_jiffies(100);
	u32 val = readl_relaxed(base);

	if (on) {
		/* Start the on-chip PHY and its PLL. */
		val &= ~(CONF2_RESET | CONF2_PHYPWRDN | CONF2_OTGPWRDN);
		val |= CONF2_PHY_PLLON;
		writel_relaxed(val, base);

		pr_info("Waiting for PHY clock good...\n");
		while (!(readl_relaxed(base) & CONF2_PHYCLKGD)) {
			cpu_relax();

			if (time_after(jiffies, timeout)) {
				pr_err("musb PHY clock good timed out\n");
				break;
			}
		}
	} else {
		/* Power down the on-chip PHY. */
		val &= ~CONF2_PHY_PLLON;
		val |=  CONF2_PHYPWRDN | CONF2_OTGPWRDN;
		writel_relaxed(val, base);
	}

	return 0;
}

static int am35xx_usbotg_phy_power_on(struct phy *phy)
{
	return am35x_musb_phy_power(phy, 1);
}

static int am35xx_usbotg_phy_power_off(struct phy *phy)
{
	return am35x_musb_phy_power(phy, 0);
}

static int am35xx_usbotg_phy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct am35_musb_phy *musbphy = phy_get_drvdata(phy);
	void __iomem *base = musbphy->regs;

	u32 val = readl_relaxed(base) & ~CONF2_OTGMODE;

	switch (mode) {
	case PHY_MODE_USB_HOST:		/* Force VBUS valid, ID = 0 */
		val |= CONF2_FORCE_HOST;
		break;
	case PHY_MODE_USB_DEVICE:	/* Force VBUS valid, ID = 1 */
		val |= CONF2_FORCE_DEVICE;
		break;
	case PHY_MODE_USB_OTG:		/* Don't override the VBUS/ID comparators */
		val |= CONF2_NO_OVERRIDE;
		break;
	default:
		return -EINVAL;
	}

	writel_relaxed(val, base);

	return 0;
}

static int am35xx_usbotg_phy_init(struct phy *phy)
{
	struct am35_musb_phy *musbphy = phy_get_drvdata(phy);
	void __iomem *base = musbphy->regs;
	u32 val = readl_relaxed(base);

	/* USB2.0 PHY reference clock is 13 MHz */
	val &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	val |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	writel_relaxed(val, base);
	return 0;
}

static const struct phy_ops am35xx_musb_phy_ops = {
	.init		= am35xx_usbotg_phy_init,
	.power_on	= am35xx_usbotg_phy_power_on,
	.power_off	= am35xx_usbotg_phy_power_off,
	.set_mode	= am35xx_usbotg_phy_set_mode,
};

static int am35xx_musb_phy_probe(struct platform_device *pdev)
{
	void __iomem *base;
	struct am35_musb_phy *am35xx_usbotg_phy;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	am35xx_usbotg_phy = devm_kzalloc(&pdev->dev, sizeof(*am35xx_usbotg_phy), GFP_KERNEL);
	if (!am35xx_usbotg_phy)
		return -ENOMEM;

	am35xx_usbotg_phy->regs = base;

	return 0;
}

static const struct of_device_id am35xx_musb_phy_ids[] = {
	{ .compatible = "ti,am35xx-musb-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, am35xx_musb_phy_ids);

static struct platform_driver am35xx_musb_phy_driver = {
	.probe	= am35xx_musb_phy_probe,
	.driver	= {
		.name	= "am35xx-musb-phy",
		.of_match_table = am35xx_musb_phy_ids,
	},
};

module_platform_driver(am35xx_musb_phy_driver);

MODULE_ALIAS("platform:am35xx-musb-phy");
MODULE_AUTHOR("Adam Ford <aford173@gmail.com>");
MODULE_DESCRIPTION("TI AM35xx MUSB PHY driver");
MODULE_LICENSE("GPL");

