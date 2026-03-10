// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 Code Construct
 *
 * Author: Jeremy Kerr <jk@codeconstruct.com.au>
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/bitfield.h>

#include "dw-i3c-master.h"

/* AST2600-specific global register set */
#define AST2600_I3CG_REG0(idx)	(((idx) * 4 * 4) + 0x10)
#define AST2600_I3CG_REG1(idx)	(((idx) * 4 * 4) + 0x14)

#define AST2600_I3CG_REG0_SDA_PULLUP_EN_MASK	GENMASK(29, 28)
#define AST2600_I3CG_REG0_SDA_PULLUP_EN_2K	(0x0 << 28)
#define AST2600_I3CG_REG0_SDA_PULLUP_EN_750	(0x2 << 28)
#define AST2600_I3CG_REG0_SDA_PULLUP_EN_545	(0x3 << 28)

#define AST2600_I3CG_REG1_I2C_MODE		BIT(0)
#define AST2600_I3CG_REG1_TEST_MODE		BIT(1)
#define AST2600_I3CG_REG1_ACT_MODE_MASK		GENMASK(3, 2)
#define AST2600_I3CG_REG1_ACT_MODE(x)		(((x) << 2) & AST2600_I3CG_REG1_ACT_MODE_MASK)
#define AST2600_I3CG_REG1_PENDING_INT_MASK	GENMASK(7, 4)
#define AST2600_I3CG_REG1_PENDING_INT(x)	(((x) << 4) & AST2600_I3CG_REG1_PENDING_INT_MASK)
#define AST2600_I3CG_REG1_SA_MASK		GENMASK(14, 8)
#define AST2600_I3CG_REG1_SA(x)			(((x) << 8) & AST2600_I3CG_REG1_SA_MASK)
#define AST2600_I3CG_REG1_SA_EN			BIT(15)
#define AST2600_I3CG_REG1_INST_ID_MASK		GENMASK(19, 16)
#define AST2600_I3CG_REG1_INST_ID(x)		(((x) << 16) & AST2600_I3CG_REG1_INST_ID_MASK)
#define SCL_SW_MODE_OE				BIT(20)
#define SCL_OUT_SW_MODE_VAL			BIT(21)
#define SCL_IN_SW_MODE_VAL			BIT(23)
#define SDA_SW_MODE_OE				BIT(24)
#define SDA_OUT_SW_MODE_VAL			BIT(25)
#define SDA_IN_SW_MODE_VAL			BIT(27)
#define SCL_IN_SW_MODE_EN			BIT(28)
#define SDA_IN_SW_MODE_EN			BIT(29)
#define SCL_OUT_SW_MODE_EN			BIT(30)
#define SDA_OUT_SW_MODE_EN			BIT(31)

#define AST2600_DEFAULT_SDA_PULLUP_OHMS		2000

/* dw-i3c registers */
#define IBI_QUEUE_STATUS			0x18
#define IBI_QUEUE_STATUS_DATA_LEN(x)	((x) & GENMASK(7, 0))

#define QUEUE_STATUS_LEVEL		0x4c
#define QUEUE_STATUS_IBI_STATUS_CNT(x) (((x) & GENMASK(28, 24)) >> 24)

#define PRESENT_STATE				0x54
#define   CM_TFR_STS				GENMASK(13, 8)
#define     CM_TFR_STS_MASTER_SERV_IBI		0xe
#define   SDA_LINE_SIGNAL_LEVEL			BIT(1)
#define   SCL_LINE_SIGNAL_LEVEL			BIT(0)

/* DAT */
#define DEV_ADDR_TABLE_IBI_PEC			BIT(11)

struct ast2600_i3c {
	struct dw_i3c_master dw;
	struct regmap *global_regs;
	unsigned int global_idx;
	unsigned int sda_pullup;
};

static struct ast2600_i3c *to_ast2600_i3c(struct dw_i3c_master *dw)
{
	return container_of(dw, struct ast2600_i3c, dw);
}

static int ast2600_i3c_pullup_to_reg(unsigned int ohms, u32 *regp)
{
	u32 reg;

	switch (ohms) {
	case 2000:
		reg = AST2600_I3CG_REG0_SDA_PULLUP_EN_2K;
		break;
	case 750:
		reg = AST2600_I3CG_REG0_SDA_PULLUP_EN_750;
		break;
	case 545:
		reg = AST2600_I3CG_REG0_SDA_PULLUP_EN_545;
		break;
	default:
		return -EINVAL;
	}

	if (regp)
		*regp = reg;

	return 0;
}

static int ast2600_i3c_init(struct dw_i3c_master *dw)
{
	struct ast2600_i3c *i3c = to_ast2600_i3c(dw);
	u32 reg = 0;
	int rc;

	/* reg0: set SDA pullup values */
	rc = ast2600_i3c_pullup_to_reg(i3c->sda_pullup, &reg);
	if (rc)
		return rc;

	rc = regmap_write(i3c->global_regs,
			  AST2600_I3CG_REG0(i3c->global_idx), reg);
	if (rc)
		return rc;

	/* reg1: set up the instance id, but leave everything else disabled,
	 * as it's all for client mode
	 */
	reg = AST2600_I3CG_REG1_INST_ID(i3c->global_idx);
	rc = regmap_write(i3c->global_regs,
			  AST2600_I3CG_REG1(i3c->global_idx), reg);

	return rc;
}

static void ast2600_i3c_set_dat_ibi(struct dw_i3c_master *i3c,
				    struct i3c_dev_desc *dev,
				    bool enable, u32 *dat)
{
	/*
	 * The ast2600 i3c controller will lock up on receiving 4n+1-byte IBIs
	 * if the PEC is disabled. We have no way to restrict the length of
	 * IBIs sent to the controller, so we need to unconditionally enable
	 * PEC checking, which means we drop a byte of payload data
	 */
	if (enable && dev->info.bcr & I3C_BCR_IBI_PAYLOAD) {
		dev_warn_once(&i3c->base.dev,
		      "Enabling PEC workaround. IBI payloads will be truncated\n");
		*dat |= DEV_ADDR_TABLE_IBI_PEC;
	}
}

static int aspeed_i3c_bus_recovery(struct dw_i3c_master *dw)
{
	struct ast2600_i3c *i3c = to_ast2600_i3c(dw);
	int i, ret = -1;

	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SCL_OUT_SW_MODE_VAL, SCL_OUT_SW_MODE_VAL);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SCL_SW_MODE_OE, SCL_SW_MODE_OE);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SCL_OUT_SW_MODE_EN, SCL_OUT_SW_MODE_EN);

	for (i = 0; i < 19; i++) {
		regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
				  SCL_OUT_SW_MODE_VAL, 0);
		regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
				  SCL_OUT_SW_MODE_VAL, SCL_OUT_SW_MODE_VAL);
		if (readl(dw->regs + PRESENT_STATE) & SDA_LINE_SIGNAL_LEVEL) {
			ret = 0;
			break;
		}
	}

	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SCL_OUT_SW_MODE_EN, 0);
	if (ret)
		dev_err(&dw->base.dev, "Failed to recover the bus\n");

	return ret;
}

static void ast2600_i3c_gen_target_reset_pattern(struct dw_i3c_master *dw)
{
	struct ast2600_i3c *i3c = to_ast2600_i3c(dw);
	int i;

	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_OUT_SW_MODE_VAL | SCL_OUT_SW_MODE_VAL,
			  SDA_OUT_SW_MODE_VAL | SCL_OUT_SW_MODE_VAL);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_SW_MODE_OE | SCL_SW_MODE_OE,
			  SDA_SW_MODE_OE | SCL_SW_MODE_OE);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_OUT_SW_MODE_EN | SCL_OUT_SW_MODE_EN,
			  SDA_OUT_SW_MODE_EN | SCL_OUT_SW_MODE_EN);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_IN_SW_MODE_VAL | SCL_IN_SW_MODE_VAL,
			  SDA_IN_SW_MODE_VAL | SCL_IN_SW_MODE_VAL);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_IN_SW_MODE_EN | SCL_IN_SW_MODE_EN,
			  SDA_IN_SW_MODE_EN | SCL_IN_SW_MODE_EN);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SCL_OUT_SW_MODE_VAL, 0);
	for (i = 0; i < 7; i++) {
		regmap_write_bits(i3c->global_regs,
				  AST2600_I3CG_REG1(i3c->global_idx),
				  SDA_OUT_SW_MODE_VAL, 0);
		regmap_write_bits(i3c->global_regs,
				  AST2600_I3CG_REG1(i3c->global_idx),
				  SDA_OUT_SW_MODE_VAL, SDA_OUT_SW_MODE_VAL);
	}
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SCL_OUT_SW_MODE_VAL, SCL_OUT_SW_MODE_VAL);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_OUT_SW_MODE_VAL, 0);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_OUT_SW_MODE_VAL, SDA_OUT_SW_MODE_VAL);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_OUT_SW_MODE_EN | SCL_OUT_SW_MODE_EN, 0);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_IN_SW_MODE_EN | SCL_IN_SW_MODE_EN, 0);
}

static void aspeed_i3c_drain_ibi_queue(struct dw_i3c_master *dw)
{
	/*
	 * Clear the IBI queue to avoid any stale IBI data when
	 * re-enabling the controller.
	 */
	u32 ibi_status = readl(dw->regs + IBI_QUEUE_STATUS);
	u8 length = IBI_QUEUE_STATUS_DATA_LEN(ibi_status);
	int i, nwords = (length + 3) >> 2;

	for (i = 0; i < nwords; i++)
		readl(dw->regs + IBI_QUEUE_STATUS);
}

static bool ast2600_i3c_fsm_exit_serv_ibi(struct dw_i3c_master *dw)
{
	u32 state;

	/*
	 * Clear the IBI queue to enable the hardware to generate SCL and
	 * begin detecting the T-bit low to stop reading IBI data.
	 */
	aspeed_i3c_drain_ibi_queue(dw);
	state = FIELD_GET(CM_TFR_STS, readl(dw->regs + PRESENT_STATE));
	if (state == CM_TFR_STS_MASTER_SERV_IBI)
		return false;

	return true;
}

static void ast2600_i3c_gen_tbits_in(struct dw_i3c_master *dw)
{
	struct ast2600_i3c *i3c = to_ast2600_i3c(dw);
	bool is_halted;
	u32 nibi, i;
	int ret;

	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_IN_SW_MODE_VAL, SDA_IN_SW_MODE_VAL);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_IN_SW_MODE_EN, SDA_IN_SW_MODE_EN);

	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_IN_SW_MODE_VAL, 0);
	ret = readx_poll_timeout_atomic(ast2600_i3c_fsm_exit_serv_ibi, dw,
					is_halted, is_halted, 0, 2000000);
	regmap_write_bits(i3c->global_regs, AST2600_I3CG_REG1(i3c->global_idx),
			  SDA_IN_SW_MODE_EN, 0);
	if (ret) {
 		dev_err(&dw->base.dev,
 			"Failed to exit the I3C fsm from %lx(MASTER_SERV_IBI): %d",
 			FIELD_GET(CM_TFR_STS, readl(dw->regs + PRESENT_STATE)),
 			ret);
	} else {
		/* Clear the dummy data generated in this recovery process */
		nibi = readl(dw->regs + QUEUE_STATUS_LEVEL);
		nibi = QUEUE_STATUS_IBI_STATUS_CNT(nibi);
		for (i = 0; i < nibi; i++)
			aspeed_i3c_drain_ibi_queue(dw);
	}
}

static const struct dw_i3c_platform_ops ast2600_i3c_ops = {
	.init = ast2600_i3c_init,
	.set_dat_ibi = ast2600_i3c_set_dat_ibi,
	.gen_target_reset_pattern = ast2600_i3c_gen_target_reset_pattern,
	.gen_tbits_in = ast2600_i3c_gen_tbits_in,
	.bus_recovery = aspeed_i3c_bus_recovery,
};

static int ast2600_i3c_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct of_phandle_args gspec;
	struct ast2600_i3c *i3c;
	int rc;

	i3c = devm_kzalloc(&pdev->dev, sizeof(*i3c), GFP_KERNEL);
	if (!i3c)
		return -ENOMEM;

	rc = of_parse_phandle_with_fixed_args(np, "aspeed,global-regs", 1, 0,
					      &gspec);
	if (rc)
		return -ENODEV;

	i3c->global_regs = syscon_node_to_regmap(gspec.np);
	of_node_put(gspec.np);

	if (IS_ERR(i3c->global_regs))
		return PTR_ERR(i3c->global_regs);

	i3c->global_idx = gspec.args[0];

	rc = of_property_read_u32(np, "sda-pullup-ohms", &i3c->sda_pullup);
	if (rc)
		i3c->sda_pullup = AST2600_DEFAULT_SDA_PULLUP_OHMS;

	rc = ast2600_i3c_pullup_to_reg(i3c->sda_pullup, NULL);
	if (rc)
		dev_err(&pdev->dev, "invalid sda-pullup value %d\n",
			i3c->sda_pullup);

	i3c->dw.platform_ops = &ast2600_i3c_ops;
	i3c->dw.base.i2c.dev.of_node = np;
	return dw_i3c_common_probe(&i3c->dw, pdev);
}

static void ast2600_i3c_remove(struct platform_device *pdev)
{
	struct dw_i3c_master *dw_i3c = platform_get_drvdata(pdev);

	dw_i3c_common_remove(dw_i3c);
}

static const struct of_device_id ast2600_i3c_master_of_match[] = {
	{ .compatible = "aspeed,ast2600-i3c", },
	{},
};
MODULE_DEVICE_TABLE(of, ast2600_i3c_master_of_match);

static struct platform_driver ast2600_i3c_driver = {
	.probe = ast2600_i3c_probe,
	.remove = ast2600_i3c_remove,
	.driver = {
		.name = "ast2600-i3c-master",
		.of_match_table = ast2600_i3c_master_of_match,
	},
};
module_platform_driver(ast2600_i3c_driver);

MODULE_AUTHOR("Jeremy Kerr <jk@codeconstruct.com.au>");
MODULE_DESCRIPTION("ASPEED AST2600 I3C driver");
MODULE_LICENSE("GPL");
