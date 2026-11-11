// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726/HY300 "pck-600" power controller.
 *
 * The layout below was recovered from the vendor's decompiled power-domain
 * code (sunxi_pd_power()/sunxi_pd_power_on()/sunxi_pd_power_off() and the
 * per-domain table used by sunxi_pm_add_one_domain() in the stock misc
 * module) and matches the Arm PCK-600 / PPU register map used by the A523:
 *
 *   parent syscon @ 0x07060000, one 4 KiB block per domain:
 *
 *     0x000  PWPR   power mode request, PPU_PWR_STATUS in bits [3:0]
 *     0x008  PWSR   power mode status,  same field, polled after a request
 *     0x170  DCDR0  device control delay 0
 *     0x174  DCDR1  device control delay 1
 *     0xc00  logic power switch 0 delay
 *     0xc04  logic power switch 1 delay
 *     0xc10  off-to-on delay
 *
 * The vendor sequence is: read PWSR, bail out if it already matches, update
 * PWPR under the status mask (0x8 = on, 0x0 = off), issue a dsb, then poll
 * PWSR for up to 10 ms until it matches.  The domain index is the child's reg
 * value (pd_gpu@0 -> 0, pd_ve@1 -> 1), and the block offset is index * 0x1000.
 *
 * DCDR and delay values are inherited from the A523 descriptor of the same
 * block; the vendor table for this SoC was not recoverable, so its entries are
 * assumed identical.  Register access goes through the parent syscon regmap.
 *
 * Two additions are deliberately outside the register work:
 *  - CONFIG_SUN50IW15_PCK_PD_TOLERATE_TIMEOUT, a bring-up escape hatch for
 *    transitions that never acknowledge (see sun50iw15_pck_pd_set_power());
 *  - a per-domain debugfs dump of PWPR/PWSR, and the same values logged once
 *    at probe, so the first boot can be checked without a rebuild.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#define PPU_PWPR			0x000
#define PPU_PWSR			0x008
#define PPU_DCDR0			0x170
#define PPU_DCDR1			0x174
#define PPU_LPS0_DELAY			0xc00
#define PPU_LPS1_DELAY			0xc04
#define PPU_OFF2ON_DELAY		0xc10

/* Shared by PWPR and PWSR: 0x8 is "on", 0x0 is "off". */
#define PPU_PWR_STATUS			GENMASK(3, 0)
#define PPU_POWER_MODE_ON		0x8
#define PPU_POWER_MODE_OFF		0x0

#define PPU_REG_SIZE			0x1000

/* Delay programming, as used by the A523 PCK-600 block. */
#define PPU_DEVICE_CTRL0_DELAY		0xffffff
#define PPU_DEVICE_CTRL1_DELAY		0xffff
#define PPU_LPS0_DELAY_VALUE		0x8080808
#define PPU_LPS1_DELAY_VALUE		0x808
#define PPU_OFF2ON_DELAY_VALUE		0x8

#define PPU_POWER_TIMEOUT_US		10000

struct sun50iw15_pck_pd {
	struct generic_pm_domain genpd;
	struct device *dev;
	struct regmap *regmap;
	unsigned int offset;
	bool stuck_on;		/* bring-up fallback: off transition never acked */
};

struct sun50iw15_pck {
	struct device *dev;
	struct genpd_onecell_data genpd_data;
	struct sun50iw15_pck_pd *pds;
	unsigned int num_pds;
	struct dentry *debugfs;
};

#define to_sun50iw15_pd(gpd) container_of(gpd, struct sun50iw15_pck_pd, genpd)

/*
 * BRING-UP FALLBACK -- CONFIG_SUN50IW15_PCK_PD_TOLERATE_TIMEOUT.
 *
 * The DCDR and delay values are the A523 ones, so a value that is too short
 * for this silicon makes the 10 ms poll expire.  Strict gating would then fail
 * the GPU's runtime resume, i.e. no GPU at all, which is the worse failure
 * while bringing the board up, so the timeout is logged and treated as
 * success instead.
 *
 * Revert for production: set CONFIG_SUN50IW15_PCK_PD_TOLERATE_TIMEOUT=n.  That
 * returns every transition failure to genpd again; the stuck_on field and the
 * two dev_warn() calls below are the only fallback code, and can be deleted
 * together with the option.
 */
static int sun50iw15_pck_pd_set_power(struct sun50iw15_pck_pd *pd, bool on)
{
	unsigned int val = on ? PPU_POWER_MODE_ON : PPU_POWER_MODE_OFF;
	unsigned int reg;
	int ret;

	/* A failed off left the block powered; don't poll it every idle cycle. */
	if (!on && pd->stuck_on)
		return 0;

	regmap_read(pd->regmap, pd->offset + PPU_PWSR, &reg);
	if ((reg & PPU_PWR_STATUS) == val)
		return 0;

	regmap_update_bits(pd->regmap, pd->offset + PPU_PWPR,
			   PPU_PWR_STATUS, val);
	/* push the request out to the block before polling its status */
	regmap_read(pd->regmap, pd->offset + PPU_PWPR, &reg);

	ret = regmap_read_poll_timeout(pd->regmap, pd->offset + PPU_PWSR, reg,
				       (reg & PPU_PWR_STATUS) == val, 0,
				       PPU_POWER_TIMEOUT_US);
	if (!ret)
		return 0;

	if (IS_ENABLED(CONFIG_SUN50IW15_PCK_PD_TOLERATE_TIMEOUT)) {
		if (on) {
			dev_warn(pd->dev,
				 "domain %s not on after %u us, continuing (delays are A523 values, bring-up fallback)\n",
				 pd->genpd.name, PPU_POWER_TIMEOUT_US);
		} else {
			pd->stuck_on = true;
			dev_warn(pd->dev,
				 "domain %s stays on: off transition not acked in %u us (bring-up fallback)\n",
				 pd->genpd.name, PPU_POWER_TIMEOUT_US);
		}
		return 0;
	}

	dev_err(pd->dev, "domain %s failed to power %s: %d\n",
		pd->genpd.name, on ? "on" : "off", ret);

	return ret;
}

static int sun50iw15_pck_pd_power_on(struct generic_pm_domain *domain)
{
	return sun50iw15_pck_pd_set_power(to_sun50iw15_pd(domain), true);
}

static int sun50iw15_pck_pd_power_off(struct generic_pm_domain *domain)
{
	return sun50iw15_pck_pd_set_power(to_sun50iw15_pd(domain), false);
}

static int sun50iw15_pck_pd_regs_show(struct seq_file *s, void *unused)
{
	struct sun50iw15_pck_pd *pd = s->private;
	unsigned int pwpr, pwsr, state;

	regmap_read(pd->regmap, pd->offset + PPU_PWPR, &pwpr);
	regmap_read(pd->regmap, pd->offset + PPU_PWSR, &pwsr);

	state = pwsr & PPU_PWR_STATUS;
	seq_printf(s, "PWPR: 0x%08x\nPWSR: 0x%08x\nstate: %s\n",
		   pwpr, pwsr,
		   state == PPU_POWER_MODE_ON ? "on" :
		   state == PPU_POWER_MODE_OFF ? "off" : "transitioning");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sun50iw15_pck_pd_regs);

static void sun50iw15_pck_pd_setup(struct sun50iw15_pck_pd *pd)
{
	regmap_write(pd->regmap, pd->offset + PPU_DCDR0,
		     PPU_DEVICE_CTRL0_DELAY);
	regmap_write(pd->regmap, pd->offset + PPU_DCDR1,
		     PPU_DEVICE_CTRL1_DELAY);
	regmap_write(pd->regmap, pd->offset + PPU_LPS0_DELAY,
		     PPU_LPS0_DELAY_VALUE);
	regmap_write(pd->regmap, pd->offset + PPU_LPS1_DELAY,
		     PPU_LPS1_DELAY_VALUE);
	regmap_write(pd->regmap, pd->offset + PPU_OFF2ON_DELAY,
		     PPU_OFF2ON_DELAY_VALUE);
}

static int sun50iw15_pck_pd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct generic_pm_domain **domains;
	struct sun50iw15_pck *pck;
	struct regmap *regmap;
	struct clk *clk;
	struct reset_control *rst;
	unsigned int num, id;
	int ret;

	num = of_get_child_count(np);
	if (!num)
		return -ENODEV;

	regmap = syscon_node_to_regmap(np->parent);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "no parent syscon regmap\n");

	/* The block needs its clock, and the pck reset released, to sequence. */
	rst = devm_reset_control_get_optional_exclusive(dev, "pck_rst");
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst),
				     "failed to get pck_rst\n");

	clk = devm_clk_get_optional(dev, "pck");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get pck\n");

	reset_control_deassert(rst);

	ret = clk_prepare_enable(clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable pck\n");

	pck = devm_kzalloc(dev, sizeof(*pck), GFP_KERNEL);
	if (!pck)
		return -ENOMEM;

	pck->dev = dev;
	pck->num_pds = num;
	pck->pds = devm_kcalloc(dev, num, sizeof(*pck->pds), GFP_KERNEL);
	if (!pck->pds)
		return -ENOMEM;

	domains = devm_kcalloc(dev, num, sizeof(*domains), GFP_KERNEL);
	if (!domains)
		return -ENOMEM;

	pck->debugfs = debugfs_create_dir("sun50iw15-pck-pd", NULL);

	for_each_child_of_node(np, child) {
		struct sun50iw15_pck_pd *pd;
		unsigned int pwpr, pwsr;

		if (of_property_read_u32(child, "reg", &id) || id >= num) {
			dev_warn(dev, "%pOF: missing or out of range reg\n",
				 child);
			continue;
		}

		pd = &pck->pds[id];
		pd->dev = dev;
		pd->regmap = regmap;
		pd->offset = PPU_REG_SIZE * id;

		pd->genpd.name = devm_kasprintf(dev, GFP_KERNEL, "%pOFn", child);
		if (!pd->genpd.name) {
			of_node_put(child);
			return -ENOMEM;
		}
		pd->genpd.power_on = sun50iw15_pck_pd_power_on;
		pd->genpd.power_off = sun50iw15_pck_pd_power_off;

		sun50iw15_pck_pd_setup(pd);

		ret = pm_genpd_init(&pd->genpd, NULL, false);
		if (ret) {
			of_node_put(child);
			return ret;
		}
		domains[id] = &pd->genpd;

		regmap_read(regmap, pd->offset + PPU_PWPR, &pwpr);
		regmap_read(regmap, pd->offset + PPU_PWSR, &pwsr);
		dev_info(dev, "%s: PWPR=0x%08x PWSR=0x%08x (%s)\n",
			 pd->genpd.name, pwpr, pwsr,
			 (pwsr & PPU_PWR_STATUS) == PPU_POWER_MODE_ON ?
			 "on" : "off");

		if (!IS_ERR_OR_NULL(pck->debugfs))
			debugfs_create_file(pd->genpd.name, 0444, pck->debugfs,
					    pd, &sun50iw15_pck_pd_regs_fops);
	}

	pck->genpd_data.domains = domains;
	pck->genpd_data.num_domains = num;
	platform_set_drvdata(pdev, pck);

	ret = of_genpd_add_provider_onecell(np, &pck->genpd_data);
	if (ret)
		return ret;

	dev_info(dev, "registered %u gateable power domains, gating %s\n",
		 pck->num_pds,
		 IS_ENABLED(CONFIG_SUN50IW15_PCK_PD_TOLERATE_TIMEOUT) ?
		 "with timeout fallback (bring-up)" : "strict");

	return 0;
}

static int sun50iw15_pck_pd_remove(struct platform_device *pdev)
{
	struct sun50iw15_pck *pck = platform_get_drvdata(pdev);
	unsigned int i;

	debugfs_remove_recursive(pck->debugfs);

	of_genpd_del_provider(pdev->dev.of_node);
	for (i = 0; i < pck->genpd_data.num_domains; i++) {
		if (pck->genpd_data.domains[i])
			pm_genpd_remove(pck->genpd_data.domains[i]);
	}

	return 0;
}

static const struct of_device_id sun50iw15_pck_pd_ids[] = {
	{ .compatible = "allwinner,sun50iw15-pck-600" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun50iw15_pck_pd_ids);

static struct platform_driver sun50iw15_pck_pd_driver = {
	.probe = sun50iw15_pck_pd_probe,
	.remove = sun50iw15_pck_pd_remove,
	.driver = {
		.name = "sun50iw15-pck-pd",
		.of_match_table = sun50iw15_pck_pd_ids,
	},
};
builtin_platform_driver(sun50iw15_pck_pd_driver);

MODULE_DESCRIPTION("Allwinner sun50iw15 pck-600 power domains");
MODULE_LICENSE("GPL");
