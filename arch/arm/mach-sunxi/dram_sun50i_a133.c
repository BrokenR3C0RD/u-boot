// SPDX-License-Identifier: GPL-2.0+
/*
 * sun50i A133 platform dram controller driver
 *
 * Controller and PHY appear to be quite similar to that of the H616;
 * however certain offsets, timings, and other details are different enough that
 * the original code does not work as expected. Some device flags and calibrations
 * are not yet implemented, and configuration aside from DDR4 have not been tested.
 *
 * (C) Copyright 2024 MasterR3C0RD <masterr3c0rd@epochal.quest>
 *
 * Uses code from H616 driver, which is
 * (C) Copyright 2020 Jernej Skrabec <jernej.skrabec@siol.net>
 *
 */

#define DEBUG

#include <asm/arch/clock.h>
#include <asm/arch/cpu.h>
#include <asm/arch/dram.h>
#include <asm/arch/prcm.h>
#include <asm/io.h>
#include <init.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <log.h>

#ifdef CONFIG_DRAM_SUN50I_PHY_ADDR_MAP_1
static const u8 phy_init[] = {
#ifdef CONFIG_SUNXI_DRAM_DDR3
	0x0c, 0x08, 0x19, 0x18, 0x10, 0x06, 0x0a, 0x03, 0x0e,
	0x00, 0x0b, 0x05, 0x09, 0x1a, 0x04, 0x13, 0x16, 0x11,
	0x01, 0x15, 0x0d, 0x07, 0x12, 0x17, 0x14, 0x02, 0x0f
#elif CONFIG_SUNXI_DRAM_DDR4
	0x19, 0x1a, 0x04, 0x12, 0x09, 0x06, 0x08, 0x0a, 0x16,
	0x17, 0x18, 0x0f, 0x0c, 0x13, 0x02, 0x05, 0x01, 0x11,
	0x0e, 0x00, 0x0b, 0x07, 0x03, 0x14, 0x15, 0x0d, 0x10
#elif CONFIG_SUNXI_DRAM_LPDDR3
	0x08, 0x03, 0x02, 0x00, 0x18, 0x19, 0x09, 0x01, 0x06,
	0x17, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x15, 0x16, 0x04, 0x05, 0x07, 0x1a
#elif CONFIG_SUNXI_DRAM_LPDDR4
	0x01, 0x05, 0x02, 0x00, 0x19, 0x03, 0x06, 0x07, 0x08,
	0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x04, 0x1a
#endif
};
#else
static const u8 phy_init[] = {
#ifdef CONFIG_SUNXI_DRAM_DDR3
	0x03, 0x19, 0x18, 0x02, 0x10, 0x15, 0x16, 0x07, 0x06,
	0x0e, 0x05, 0x08, 0x0d, 0x04, 0x17, 0x1a, 0x13, 0x11,
	0x12, 0x14, 0x00, 0x01, 0x0c, 0x0a, 0x09, 0x0b, 0x0f
#elif CONFIG_SUNXI_DRAM_DDR4
	0x13, 0x17, 0x0e, 0x01, 0x06, 0x12, 0x14, 0x07, 0x09,
	0x02, 0x0f, 0x00, 0x0d, 0x05, 0x16, 0x0c, 0x0a, 0x11,
	0x04, 0x03, 0x18, 0x15, 0x08, 0x10, 0x0b, 0x19, 0x1a
#elif CONFIG_SUNXI_DRAM_LPDDR3
	0x05, 0x06, 0x17, 0x02, 0x19, 0x18, 0x04, 0x07, 0x03,
	0x01, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x15, 0x16, 0x08, 0x09, 0x00, 0x1a
#elif CONFIG_SUNXI_DRAM_LPDDR4
	0x01, 0x03, 0x02, 0x19, 0x17, 0x00, 0x06, 0x07, 0x08,
	0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x15, 0x16, 0x04, 0x18, 0x05, 0x1a
#endif
};
#endif

static void mctl_clk_init(u32 clk)
{
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	/* Place all DRAM blocks into reset */
	clrbits_le32(&ccm->mbus_cfg, MBUS_ENABLE);
	clrbits_le32(&ccm->mbus_cfg, MBUS_RESET);
	clrbits_le32(&ccm->dram_gate_reset, BIT(GATE_SHIFT));
	clrbits_le32(&ccm->dram_gate_reset, BIT(RESET_SHIFT));
	clrbits_le32(&ccm->pll5_cfg, CCM_PLL5_CTRL_EN);
	clrbits_le32(&ccm->dram_clk_cfg, DRAM_MOD_RESET);
	udelay(5);

	/* Set up PLL5 clock, used for DRAM */
	clrsetbits_le32(&ccm->pll5_cfg, 0xff03,
			CCM_PLL5_CTRL_N((clk * 2) / 24) | CCM_PLL5_CTRL_EN);
	setbits_le32(&ccm->pll5_cfg, BIT(24));
	clrsetbits_le32(&ccm->pll5_cfg, 0x3,
			CCM_PLL5_LOCK_EN | CCM_PLL5_CTRL_EN | BIT(30));
	clrbits_le32(&ccm->pll5_cfg, 0x3 | BIT(30));
	mctl_await_completion(&ccm->pll5_cfg, CCM_PLL5_LOCK, CCM_PLL5_LOCK);

	/* Enable DRAM clock and gate*/
	clrbits_le32(&ccm->dram_clk_cfg, BIT(24) | BIT(25));
	clrsetbits_le32(&ccm->dram_clk_cfg, 0x1f, BIT(1) | BIT(0));
	setbits_le32(&ccm->dram_clk_cfg, DRAM_CLK_UPDATE);
	setbits_le32(&ccm->dram_gate_reset, BIT(RESET_SHIFT));
	setbits_le32(&ccm->dram_gate_reset, BIT(GATE_SHIFT));

	/* Re-enable MBUS and reset the DRAM module */
	setbits_le32(&ccm->mbus_cfg, MBUS_RESET);
	setbits_le32(&ccm->mbus_cfg, MBUS_ENABLE);
	setbits_le32(&ccm->dram_clk_cfg, DRAM_MOD_RESET);
	udelay(5);
}

static void mctl_set_odtmap(const struct dram_para *para,
			    const struct dram_config *config)
{
	struct sunxi_mctl_ctl_reg *mctl_ctl =
		(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	u32 val, temp1, temp2;

	/* Set ODT/rank mappings*/
	if (config->bus_full_width)
		writel_relaxed(0x0201, &mctl_ctl->odtmap);
	else
		writel_relaxed(0x0303, &mctl_ctl->odtmap);

	switch (para->type) {
	case SUNXI_DRAM_TYPE_DDR3:
		val = 0x06000400;
		break;
	case SUNXI_DRAM_TYPE_LPDDR3:
		/* TODO: What's the purpose of these values? */
		temp1 = para->clk * 7 / 2000;
		if (para->clk < 400)
			temp2 = 0x3;
		else
			temp2 = 0x4;

		val = 0x400 | (temp2 - temp1) << 16 | temp1 << 24;
		break;
	case SUNXI_DRAM_TYPE_DDR4:
		val = 0x400 | (para->mr4 << 10 & 0x70000) |
		      (((para->mr4 >> 12) & 1) + 6) << 24;

		break;
	case SUNXI_DRAM_TYPE_LPDDR4:
		val = 0x4000400;
		break;
	}

	writel_relaxed(val, &mctl_ctl->odtcfg);
	/* Documented as ODTCFG_SHADOW */
	writel_relaxed(val, &mctl_ctl->unk_0x2240);
	/* Offset's interesting; additional undocumented shadows? */
	writel_relaxed(val, &mctl_ctl->unk_0x3240);
	writel_relaxed(val, &mctl_ctl->unk_0x4240);
}

/*
 * This function produces address mapping parameters, used internally by the controller to
 * map address lines to HIF addresses. HIF addresses are word addresses, not byte addresses;
 * In other words, DDR address 0x400 maps to HIF address 0x100.
 *
 * This implementation sets up a reasonable mapping where HIF address ordering (LSB->MSB)
 * is as such:
 * - Bank Groups
 * - Columns
 * - Banks
 * - Rows
 * - Ranks
 *
 * TODO: Handle 1.5GB + 3GB configurations. Info about these is stored in upper bits of TPR13 after
 * autoscan in boot0, and then some extra logic happens in the address mapping
 */
#define INITIAL_HIF_OFFSET 3

static void mctl_set_addrmap(const struct dram_config *config)
{
	struct sunxi_mctl_ctl_reg *mctl_ctl =
		(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	u8 bankgrp_bits = config->bankgrps;
	u8 col_bits = config->cols;
	u8 bank_bits = config->banks;
	u8 row_bits = config->rows;
	u8 rank_bits = config->ranks;

	unsigned int offset;
	unsigned int hif_bits[6];
	int i;

	/*
	 * When the bus is half width, we need to adjust address mapping,
	 * as an extra column bit will be used to perform a full word
	 * access. Thus, COL[0] will not be part of the HIF, offsetting
	 * the column address mapping values by 1
	 */
	if (!config->bus_full_width)
		col_bits--;

	/* Match boot0's DRAM requirements */
	if (bankgrp_bits > 2)
		panic("invalid dram configuration (bankgrps_bits = %d)",
		      bankgrp_bits);
	if (col_bits < 8 || col_bits > 12)
		panic("invalid dram configuration (col_bits = %d)", col_bits);

	if (bank_bits < 2 || bank_bits > 3)
		panic("invalid dram configuration (bank_bits = %d)", bank_bits);

	if (row_bits < 14 || row_bits > 18)
		panic("invalid dram configuration (row_bits = %d)", row_bits);

	if (rank_bits > 1)
		panic("invalid dram configuration (rank_bits = %d)", rank_bits);

	/*
	 * Hardwired: COL[0:1] + HIF[0:1] (2 bits)
 	 * Required: COL[2] = HIF[2] (1 bit)
	 * Thus, we start allocating from HIF[3] onwards
	 */
	offset = INITIAL_HIF_OFFSET;
	debug("[*] offset = %d\n", offset);

	/*
	 * Bank groups:
	 * - BG0, if used, will be placed at HIF[3]
	 * - BG1, if used, will be placed at HIF[4]
         */
	switch (bankgrp_bits) {
	case 0:
		writel_relaxed(
			ADDRMAP8_BG0_B2(ADDRMAP_DISABLED_1F_B(2)) |
				ADDRMAP8_BG1_B3(ADDRMAP_DISABLED_1F_B(3)),
			&mctl_ctl->addrmap[8]);
		break;
	case 1:
		writel_relaxed(
			ADDRMAP8_BG0_B2(offset) |
				ADDRMAP8_BG1_B3(ADDRMAP_DISABLED_1F_B(3)),
			&mctl_ctl->addrmap[8]);
		break;
	case 2:
		writel_relaxed(ADDRMAP8_BG0_B2(offset) | ADDRMAP8_BG1_B3(offset + 1),
			       &mctl_ctl->addrmap[8]);
		break;
	default:
		panic("invalid dram configuration (bankgrp_bits = %d)",
		      bankgrp_bits);
	}

	offset += bankgrp_bits;
	debug("[*] offset = %d\n", offset);

	/*
	 * Columns:
	 * - COL[2] = HIF[2] (required)
	 * - COL[3] = HIF[offset] (always)
	 * ...
	 * - COL[5] = HIF[2 + offset] (always)
	 */
	writel_relaxed(ADDRMAP2_COL2_B2(2) | ADDRMAP2_COL3_B3(offset) |
			       ADDRMAP2_COL4_B4(offset + 1) |
			       ADDRMAP2_COL5_B5(offset + 2),
		       &mctl_ctl->addrmap[2]);

	/*
	 * Columns:
	 * - COL[6] = HIF[3 + offset] (always)
	 * - COL[7] = HIF[4 + offset] (always)
	 * - COL[8] = HIF[5 + offset] (always)
	 * - COL[9] = HIF[6 + offset]
	 * ...
	 * - COL[11] = HIF[8 + offset]
	 */
	for (i = 6; i < 12; i++) {
		if (i < col_bits)
			hif_bits[i - 6] = offset + (i - INITIAL_HIF_OFFSET);
		else
			hif_bits[i - 6] = ADDRMAP_DISABLED_1F_B(i);
	}

	writel_relaxed(ADDRMAP3_COL6_B6(hif_bits[0]) |
			       ADDRMAP3_COL7_B7(hif_bits[1]) |
			       ADDRMAP3_COL8_B8(hif_bits[2]) |
			       ADDRMAP3_COL9_B9(hif_bits[3]),
		       &mctl_ctl->addrmap[3]);

	writel_relaxed(ADDRMAP4_COL10_B10(hif_bits[4]) |
			       ADDRMAP4_COL11_B11(hif_bits[5]),
		       &mctl_ctl->addrmap[4]);

	offset = bankgrp_bits + col_bits;
	debug("[*] offset = %d\n", offset);

	/*
	 * Banks
	 * - Bank[0] = HIF[offset] (required)
	 * - Bank[1] = HIF[1 + offset] (required)
	 * - Bank[2] = HIF[2 + offset]
	 */
	if (bank_bits == 3)
		writel_relaxed(ADDRMAP1_BANK0_B2(offset) |
			       ADDRMAP1_BANK1_B3(offset + 1) |
			       ADDRMAP1_BANK2_B4(offset + 2),
		       &mctl_ctl->addrmap[1]);
	else
		writel_relaxed(ADDRMAP1_BANK0_B2(offset) |
			       ADDRMAP1_BANK1_B3(offset + 1) |
			       ADDRMAP1_BANK2_B4(ADDRMAP_DISABLED_1F_B(4)),
		       &mctl_ctl->addrmap[1]);

	offset += bank_bits;
	debug("[*] offset = %d\n", offset);

	/*
	 * Rows
	 * - Row[0] = HIF[offset] (always)
	 * - Row[1] = HIF[1 + offset] (always)
	 * - Row[10:2] = HIF[10 + offset:2 + offset] (always)
	 * - Row[11] = HIF[11 + offset] (always)
	 */
	writel_relaxed(ADDRMAP5_ROW0_B6(offset) | ADDRMAP5_ROW1_B7(offset + 1) |
			       ADDRMAP5_ROW2_10_B8(offset + 2) |
			       ADDRMAP5_ROW11_B17(offset + 11),
		       &mctl_ctl->addrmap[5]);

	/*
	 * Rows
	 * - Row[12] = HIF[offset] (always)
	 * - Row[13] = HIF[1 + offset] (always)
	 * - Row[14] = HIF[2 + offset] (always)
	 * ...
	 * - Row[17] = HIF[17 + offset] 
	 */
	for (i = 12; i < 18; i++) {
		if (i < row_bits) {
			hif_bits[i - 12] = offset + i;
		} else {
			/* ROW12 is base 18; the rest increment by 1 */
			hif_bits[i - 12] = ADDRMAP_DISABLED_0F(6 + i);
		}
	}

	writel_relaxed(ADDRMAP6_ROW12_B18(hif_bits[0]) |
			       ADDRMAP6_ROW13_B19(hif_bits[1]) |
			       ADDRMAP6_ROW14_B20(hif_bits[2]) |
			       ADDRMAP6_ROW15_B21(hif_bits[3]),
		       &mctl_ctl->addrmap[6]);

	writel_relaxed(ADDRMAP7_ROW16_B22(hif_bits[4]) |
			       ADDRMAP7_ROW17_B23(hif_bits[5]),
		       &mctl_ctl->addrmap[7]);

	offset += row_bits;
	debug("[*] offset = %d\n", offset);

	if (rank_bits == 1)
		writel_relaxed(ADDRMAP0_CS0_B6(offset), &mctl_ctl->addrmap[0]);
	else
		writel_relaxed(ADDRMAP0_CS0_B6(ADDRMAP_DISABLED_1F_B(6)),
			       &mctl_ctl->addrmap[0]);

	offset += rank_bits;
	debug("[*] FINAL voffset = %d\n", offset);
}

static void mctl_com_init(const struct dram_para *para,
			  const struct dram_config *config)
{
	struct sunxi_mctl_com_reg *mctl_com =
		(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg *mctl_ctl =
		(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	/* Might control power/reset of DDR-related blocks */
	clrsetbits_le32(&mctl_com->unk_0x008, BIT(24), BIT(25) | BIT(9));

	/* Unlock mctl_ctl registers */
	setbits_le32(&mctl_com->maer0, BIT(15));

	if (para->type == SUNXI_DRAM_TYPE_LPDDR4)
		setbits_le32(0x03102ea8, BIT(0));

	clrsetbits_le32(&mctl_ctl->sched[0], 0xff << 8, 0x30 << 8);
	if (!(para->tpr13 & BIT(28)))
		clrsetbits_le32(&mctl_ctl->sched[0], 0xf, BIT(0));

	writel_relaxed(0, &mctl_ctl->hwlpctl);

	/* Master settings */
	u32 mstr_value = MSTR_DEVICECONFIG_X32 |
			 MSTR_ACTIVE_RANKS(config->ranks);

	if (config->bus_full_width)
		mstr_value |= MSTR_BUSWIDTH_FULL;
	else
		mstr_value |= MSTR_BUSWIDTH_HALF;

	/*
	 * Geardown and 2T mode are always enabled here, but is controlled by a flag in boot0;
	 * it has not been a problem so far, but may be suspect if a particular board isn't booting.
	 */
	switch (para->type) {
	case SUNXI_DRAM_TYPE_DDR3:
		mstr_value |= MSTR_DEVICETYPE_DDR3 | MSTR_BURST_LENGTH(8) |
			      MSTR_2TMODE;
		break;
	case SUNXI_DRAM_TYPE_DDR4:
		mstr_value |= MSTR_DEVICETYPE_DDR4 | MSTR_BURST_LENGTH(8) |
			      MSTR_GEARDOWNMODE | MSTR_2TMODE;
		break;
	case SUNXI_DRAM_TYPE_LPDDR3:
		mstr_value |= MSTR_DEVICETYPE_LPDDR3 | MSTR_BURST_LENGTH(8);
		break;
	case SUNXI_DRAM_TYPE_LPDDR4:
		mstr_value |= MSTR_DEVICETYPE_LPDDR4 | MSTR_BURST_LENGTH(16);
		break;
	}

	writel_relaxed(mstr_value, &mctl_ctl->mstr);

	mctl_set_odtmap(para, config);
	mctl_set_addrmap(config);
	mctl_set_timing_params(para);

	dsb();
	writel(0, &mctl_ctl->pwrctl);

	/* Disable automatic controller updates + automatic controller update requests */
	setbits_le32(&mctl_ctl->dfiupd[0], BIT(31) | BIT(30));
	setbits_le32(&mctl_ctl->zqctl[0], BIT(31) | BIT(30));
	setbits_le32(&mctl_ctl->unk_0x2180, BIT(31) | BIT(30));
	setbits_le32(&mctl_ctl->unk_0x3180, BIT(31) | BIT(30));
	setbits_le32(&mctl_ctl->unk_0x4180, BIT(31) | BIT(30));

	/*
	 * Data bus inversion
	 * Controlled by a flag in boot0, enabled by default here.
	 */
	if (para->type == SUNXI_DRAM_TYPE_DDR4 ||
	    para->type == SUNXI_DRAM_TYPE_LPDDR4)
		setbits_le32(&mctl_ctl->dbictl, BIT(2));
}

static void mctl_drive_odt_config(const struct dram_para *para)
{
	u32 val;
	u64 base;
	u32 i;

	/* DX drive */
	for (i = 0; i < 4; i++) {
		base = SUNXI_DRAM_PHY0_BASE + 0x388 + 0x40 * i;
		val = (para->dx_dri >> (i * 8)) & 0x1f;

		writel_relaxed(val, base);
		if (para->type == SUNXI_DRAM_TYPE_LPDDR4) {
			if (para->tpr3 & 0x1f1f1f1f)
				val = (para->tpr3 >> (i * 8)) & 0x1f;
			else
				val = 4;
		}
		writel_relaxed(val, base + 4);
	}

	/* CA drive */
	for (i = 0; i < 2; i++) {
		base = SUNXI_DRAM_PHY0_BASE + 0x340 + 0x8 * i;
		val = (para->ca_dri >> (i * 8)) & 0x1f;

		writel_relaxed(val, base);
		writel_relaxed(val, base + 4);
	}

	/* DX ODT */
	for (i = 0; i < 4; i++) {
		base = SUNXI_DRAM_PHY0_BASE + 0x380 + 0x40 * i;
		val = (para->dx_odt >> (i * 8)) & 0x1f;

		if (para->type == SUNXI_DRAM_TYPE_DDR4 ||
		    para->type == SUNXI_DRAM_TYPE_LPDDR3)
			writel_relaxed(0, base);
		else
			writel_relaxed(val, base);

		if (para->type == SUNXI_DRAM_TYPE_LPDDR4)
			writel_relaxed(0, base + 4);
		else
			writel_relaxed(val, base + 4);
	}
	dsb();
}

static void mctl_phy_ca_bit_delay_compensation(const struct dram_para *para)
{
	u32 val, i;
	u32 *ptr;

	if (para->tpr10 & BIT(31)) {
		val = para->tpr2;
	} else {
		val = ((para->tpr10 << 1) & 0x1e) |
		      ((para->tpr10 << 5) & 0x1e00) |
		      ((para->tpr10 << 9) & 0x1e0000) |
		      ((para->tpr10 << 13) & 0x1e000000);

		if (para->tpr10 >> 29 != 0)
			val <<= 1;
	}

	ptr = (u32 *)(SUNXI_DRAM_PHY0_BASE + 0x780);
	for (i = 0; i < 32; i++)
		writel_relaxed((val >> 8) & 0x3f, &ptr[i]);

	writel_relaxed(val & 0x3f, SUNXI_DRAM_PHY0_BASE + 0x7dc);
	writel_relaxed(val & 0x3f, SUNXI_DRAM_PHY0_BASE + 0x7e0);

	switch (para->type) {
	case SUNXI_DRAM_TYPE_DDR3:
		writel_relaxed((val >> 16) & 0x3f,
			       SUNXI_DRAM_PHY0_BASE + 0x7b8);
		writel_relaxed((val >> 24) & 0x3f,
			       SUNXI_DRAM_PHY0_BASE + 0x784);
		break;
	case SUNXI_DRAM_TYPE_DDR4:
		writel_relaxed((val >> 16) & 0x3f,
			       SUNXI_DRAM_PHY0_BASE + 0x784);
		break;
	case SUNXI_DRAM_TYPE_LPDDR3:
		writel_relaxed((val >> 16) & 0x3f,
			       SUNXI_DRAM_PHY0_BASE + 0x788);
		writel_relaxed((val >> 24) & 0x3f,
			       SUNXI_DRAM_PHY0_BASE + 0x790);
		break;
	case SUNXI_DRAM_TYPE_LPDDR4:
		writel_relaxed((val >> 16) & 0x3f,
			       SUNXI_DRAM_PHY0_BASE + 0x790);
		writel_relaxed((val >> 24) & 0x3f,
			       SUNXI_DRAM_PHY0_BASE + 0x78c);
		break;
	}

	dsb();
}

static void mctl_phy_init(const struct dram_para *para,
			  const struct dram_config *config)
{
	struct sunxi_mctl_ctl_reg *mctl_ctl =
		(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;
	const struct sunxi_prcm_reg *prcm =
		(struct sunxi_prcm_reg *)SUNXI_PRCM_BASE;
	struct sunxi_mctl_com_reg *mctl_com =
		(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;

	u32 val, val2, i;
	u32 *ptr;

	/* Disable auto refresh. */
	setbits_le32(&mctl_ctl->rfshctl3, BIT(0));

	/* Set "phy_dbi_mode" to mark the DFI as implementing DBI functionality */
	writel_relaxed(0, &mctl_ctl->pwrctl);
	clrbits_le32(&mctl_ctl->dfimisc, 1);
	writel_relaxed(0x20, &mctl_ctl->pwrctl);

	/* PHY cold reset */
	clrsetbits_le32(&mctl_com->unk_0x008, BIT(24), BIT(9));
	udelay(1);
	setbits_le32(&mctl_com->unk_0x008, BIT(24));

	/* Not sure what this gates the power of. */
	clrbits_le32(&prcm->sys_pwroff_gating, BIT(4));

	if (para->type == SUNXI_DRAM_TYPE_LPDDR4)
		clrbits_le32(SUNXI_DRAM_PHY0_BASE + 0x4, BIT(7));

	/* Note: Similar enumeration of values is used during read training */
	if (config->bus_full_width)
		val = 0xf;
	else
		val = 0x3;

	clrsetbits_le32(SUNXI_DRAM_PHY0_BASE + 0x3c, 0xf, val);

	switch (para->type) {
	case SUNXI_DRAM_TYPE_DDR3:
		val = 13;
		val2 = 9;
		break;
	case SUNXI_DRAM_TYPE_DDR4:
		val = 13;
		val2 = 10;
		break;
	case SUNXI_DRAM_TYPE_LPDDR3:
		val = 14;
		val2 = 8;
		break;
	case SUNXI_DRAM_TYPE_LPDDR4:
		if (para->tpr13 & BIT(28))
			val = 22;
		else
			val = 20;

		val2 = 10;
		break;
	}

	writel_relaxed(val, SUNXI_DRAM_PHY0_BASE + 0x14);
	writel_relaxed(val, SUNXI_DRAM_PHY0_BASE + 0x35c);
	writel_relaxed(val, SUNXI_DRAM_PHY0_BASE + 0x368);
	writel_relaxed(val, SUNXI_DRAM_PHY0_BASE + 0x374);
	writel_relaxed(0, SUNXI_DRAM_PHY0_BASE + 0x18);
	writel_relaxed(0, SUNXI_DRAM_PHY0_BASE + 0x360);
	writel_relaxed(0, SUNXI_DRAM_PHY0_BASE + 0x36c);
	writel_relaxed(0, SUNXI_DRAM_PHY0_BASE + 0x378);
	writel_relaxed(val2, SUNXI_DRAM_PHY0_BASE + 0x1c);
	writel_relaxed(val2, SUNXI_DRAM_PHY0_BASE + 0x364);
	writel_relaxed(val2, SUNXI_DRAM_PHY0_BASE + 0x370);
	writel_relaxed(val2, SUNXI_DRAM_PHY0_BASE + 0x37c);

	/* Set up SDQ swizzle */
	ptr = (u32 *)(SUNXI_DRAM_PHY0_BASE + 0xc0);
	for (i = 0; i < ARRAY_SIZE(phy_init); i++)
		writel_relaxed(phy_init[i], &ptr[i]);

	/* Set VREF */
	val = 0;
	switch (para->type) {
	case SUNXI_DRAM_TYPE_DDR3:
		val = para->tpr6 & 0xff;
		if (val == 0)
			val = 0x80;
		break;
	case SUNXI_DRAM_TYPE_DDR4:
		val = (para->tpr6 >> 8) & 0xff;
		if (val == 0)
			val = 0x80;
		break;
	case SUNXI_DRAM_TYPE_LPDDR3:
		val = (para->tpr6 >> 16) & 0xff;
		if (val == 0)
			val = 0x80;
		break;
	case SUNXI_DRAM_TYPE_LPDDR4:
		val = (para->tpr6 >> 24) & 0xff;
		if (val == 0)
			val = 0x33;
		break;
	}
	writel_relaxed(val, SUNXI_DRAM_PHY0_BASE + 0x3dc);
	writel_relaxed(val, SUNXI_DRAM_PHY0_BASE + 0x45c);

	mctl_drive_odt_config(para);

	if (para->tpr10 & TPR10_CA_BIT_DELAY)
		mctl_phy_ca_bit_delay_compensation(para);

	switch (para->type) {
	case SUNXI_DRAM_TYPE_DDR3:
		val = 2;
		break;
	case SUNXI_DRAM_TYPE_LPDDR3:
		val = 3;
		break;
	case SUNXI_DRAM_TYPE_DDR4:
		val = 4;
		break;
	case SUNXI_DRAM_TYPE_LPDDR4:
		val = 5;
		break;
	}

	clrsetbits_le32(SUNXI_DRAM_PHY0_BASE + 0x4, 0x7, val | 8);

	if (para->clk <= 672)
		writel_relaxed(0xf, SUNXI_DRAM_PHY0_BASE + 0x20);

	if (para->clk > 500) {
		val = 0;
		val2 = 0;
	} else {
		val = 0x80;
		val2 = 0x20;
	}

	clrsetbits_le32(SUNXI_DRAM_PHY0_BASE + 0x144, 0x80, val);
	clrsetbits_le32(SUNXI_DRAM_PHY0_BASE + 0x14c, 0xe0, val2);

	dsb();
	clrbits_le32(&mctl_com->unk_0x008, BIT(9));
	udelay(1);
	clrbits_le32(SUNXI_DRAM_PHY0_BASE + 0x14c, BIT(3));

	mctl_await_completion((u32 *)(SUNXI_DRAM_PHY0_BASE + 0x180), BIT(2),
			      BIT(2));

	/* This delay is controlled by a tpr13 flag in boot0; doesn't hurt to always do it though. */
	udelay(1000);
	writel(0x37, SUNXI_DRAM_PHY0_BASE + 0x58);

	setbits_le32(&prcm->sys_pwroff_gating, BIT(4));
}

/* Helpers for updating mode registers */
static inline void mctl_mr_write(u32 mrctrl0, u32 mrctrl1)
{
	struct sunxi_mctl_ctl_reg *mctl_ctl =
		(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	writel(mrctrl1, &mctl_ctl->mrctrl1);
	writel(mrctrl0 | MRCTRL0_MR_WR | MRCTRL0_MR_RANKS_ALL,
	       &mctl_ctl->mrctrl0);
	mctl_await_completion(&mctl_ctl->mrctrl0, MRCTRL0_MR_WR, 0);
}

static inline void mctl_mr_write_lpddr4(u8 addr, u8 value)
{
	mctl_mr_write(0, MRCTRL1_MR_ADDR(addr) | MRCTRL1_MR_DATA(value));
}

static inline void mctl_mr_write_lpddr3(u8 addr, u8 value)
{
	/* Bit [7:6] are set by boot0, but undocumented */
	mctl_mr_write(BIT(6) | BIT(7),
		      MRCTRL1_MR_ADDR(addr) | MRCTRL1_MR_DATA(value));
}

static void mctl_dfi_init(const struct dram_para *para)
{
	struct sunxi_mctl_com_reg *mctl_com =
		(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg *mctl_ctl =
		(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	/* Unlock DFI registers? */
	setbits_le32(&mctl_com->maer0, BIT(8));

	/* Enable dfi_init_complete signal and trigger PHY init start request */
	writel_relaxed(0, &mctl_ctl->swctl);
	setbits_le32(&mctl_ctl->dfimisc, BIT(0));
	setbits_le32(&mctl_ctl->dfimisc, BIT(5));
	writel_relaxed(1, &mctl_ctl->swctl);
	mctl_await_completion(&mctl_ctl->swstat, BIT(0), BIT(0));

	/* Stop sending init request and wait for DFI initialization to complete. */
	writel_relaxed(0, &mctl_ctl->swctl);
	clrbits_le32(&mctl_ctl->dfimisc, BIT(5));
	writel_relaxed(1, &mctl_ctl->swctl);
	mctl_await_completion(&mctl_ctl->swstat, BIT(0), BIT(0));
	mctl_await_completion(&mctl_ctl->dfistat, BIT(0), BIT(0));

	/* Enter Software Exit from Self Refresh */
	writel_relaxed(0, &mctl_ctl->swctl);
	clrbits_le32(&mctl_ctl->pwrctl, BIT(5));
	writel_relaxed(1, &mctl_ctl->swctl);
	mctl_await_completion(&mctl_ctl->swstat, BIT(0), BIT(0));
	mctl_await_completion(&mctl_ctl->statr, 0x3, 1);

	udelay(200);

	/* Disable dfi_init_complete signal */
	writel_relaxed(0, &mctl_ctl->swctl);
	clrbits_le32(&mctl_ctl->dfimisc, BIT(0));
	writel_relaxed(1, &mctl_ctl->swctl);
	mctl_await_completion(&mctl_ctl->swstat, BIT(0), BIT(0));

	/* Write mode registers */
	switch (para->type) {
	case SUNXI_DRAM_TYPE_DDR3:
		mctl_mr_write(MRCTRL0_MR_ADDR(0), para->mr0);
		mctl_mr_write(MRCTRL0_MR_ADDR(1), para->mr1);
		mctl_mr_write(MRCTRL0_MR_ADDR(2), para->mr2);
		mctl_mr_write(MRCTRL0_MR_ADDR(3), para->mr3);
		break;
	case SUNXI_DRAM_TYPE_DDR4:
		mctl_mr_write(MRCTRL0_MR_ADDR(0), para->mr0);
		mctl_mr_write(MRCTRL0_MR_ADDR(1), para->mr1);
		mctl_mr_write(MRCTRL0_MR_ADDR(2), para->mr2);
		mctl_mr_write(MRCTRL0_MR_ADDR(3), para->mr3);
		mctl_mr_write(MRCTRL0_MR_ADDR(4), para->mr4);
		mctl_mr_write(MRCTRL0_MR_ADDR(5), para->mr5);

		mctl_mr_write(MRCTRL0_MR_ADDR(6), para->mr6 | BIT(7));
		mctl_mr_write(MRCTRL0_MR_ADDR(6), para->mr6 | BIT(7));
		mctl_mr_write(MRCTRL0_MR_ADDR(6), para->mr6 & (~BIT(7)));
		break;
	case SUNXI_DRAM_TYPE_LPDDR3:
		mctl_mr_write_lpddr3(1, para->mr1);
		mctl_mr_write_lpddr3(2, para->mr2);
		mctl_mr_write_lpddr3(3, para->mr3);
		mctl_mr_write_lpddr3(11, para->mr11);
		break;
	case SUNXI_DRAM_TYPE_LPDDR4:
		mctl_mr_write_lpddr4(0, para->mr0);
		mctl_mr_write_lpddr4(1, para->mr1);
		mctl_mr_write_lpddr4(2, para->mr2);
		mctl_mr_write_lpddr4(3, para->mr3);
		mctl_mr_write_lpddr4(4, para->mr4);
		mctl_mr_write_lpddr4(11, para->mr11);
		mctl_mr_write_lpddr4(12, para->mr12);
		mctl_mr_write_lpddr4(13, para->mr13);
		mctl_mr_write_lpddr4(14, para->mr14);
		mctl_mr_write_lpddr4(22, para->tpr1);
		break;
	}

	writel(0, SUNXI_DRAM_PHY0_BASE + 0x54);

	/* Re-enable controller refresh */
	writel(0, &mctl_ctl->swctl);
	clrbits_le32(&mctl_ctl->rfshctl3, BIT(0));
	writel(1, &mctl_ctl->swctl);
}

/* Slightly modified from H616 driver */
static bool mctl_phy_read_calibration(const struct dram_config *config)
{
	bool result = true;
	u32 val, tmp;

	clrsetbits_le32(SUNXI_DRAM_PHY0_BASE + 8, 0x30, 0x20);

	setbits_le32(SUNXI_DRAM_PHY0_BASE + 8, 1);

	if (config->bus_full_width)
		val = 0xf;
	else
		val = 3;

	while ((readl_relaxed(SUNXI_DRAM_PHY0_BASE + 0x184) & val) != val) {
		if (readl_relaxed(SUNXI_DRAM_PHY0_BASE + 0x184) & 0x20) {
			result = false;
			break;
		}
	}

	clrbits_le32(SUNXI_DRAM_PHY0_BASE + 8, 1);

	clrbits_le32(SUNXI_DRAM_PHY0_BASE + 8, 0x30);

	if (config->ranks == 1) {
		clrsetbits_le32(SUNXI_DRAM_PHY0_BASE + 8, 0x30, 0x10);

		setbits_le32(SUNXI_DRAM_PHY0_BASE + 8, 1);

		while ((readl_relaxed(SUNXI_DRAM_PHY0_BASE + 0x184) & val) !=
		       val) {
			if (readl_relaxed(SUNXI_DRAM_PHY0_BASE + 0x184) &
			    0x20) {
				result = false;
				break;
			}
		}

		clrbits_le32(SUNXI_DRAM_PHY0_BASE + 8, 1);
	}

	clrbits_le32(SUNXI_DRAM_PHY0_BASE + 8, 0x30);

	val = readl_relaxed(SUNXI_DRAM_PHY0_BASE + 0x274) & 7;
	tmp = readl_relaxed(SUNXI_DRAM_PHY0_BASE + 0x26c) & 7;
	if (val < tmp)
		val = tmp;
	tmp = readl_relaxed(SUNXI_DRAM_PHY0_BASE + 0x32c) & 7;
	if (val < tmp)
		val = tmp;
	tmp = readl_relaxed(SUNXI_DRAM_PHY0_BASE + 0x334) & 7;
	if (val < tmp)
		val = tmp;
	clrsetbits_le32(SUNXI_DRAM_PHY0_BASE + 0x38, 0x7, (val + 2) & 7);

	setbits_le32(SUNXI_DRAM_PHY0_BASE + 4, 0x20);

	return result;
}

static inline void mctl_phy_dx_delay1_inner(u32 *base, u32 val1, u32 val2)
{
	u32 *ptr = base;

	for (int i = 0; i < 9; i++) {
		writel_relaxed(val1, ptr);
		writel_relaxed(val1, ptr + 0x30);
		ptr += 2;
	}

	writel_relaxed(val2, ptr + 1);
	writel_relaxed(val2, ptr + 49);
	writel_relaxed(val2, ptr);
	writel_relaxed(val2, ptr + 48);
}

static inline void mctl_phy_dx_delay0_inner(u32 *base1, u32 *base2, u32 val1,
					    u32 val2)
{
	u32 *ptr = base1;
	for (int i = 0; i < 9; i++) {
		writel_relaxed(val1, ptr);
		writel_relaxed(val1, ptr + 0x30);
		ptr += 2;
	}

	writel_relaxed(val2, base2);
	writel_relaxed(val2, base2 + 48);
	writel_relaxed(val2, ptr);
	writel_relaxed(val2, base2 + 44);
}

/* This might be somewhat transferable to H616; whether or not people like the design is another question */
static void mctl_phy_dx_delay_compensation(const struct dram_para *para)
{
	if (para->tpr10 & TPR10_DX_BIT_DELAY1) {
		clrbits_le32(SUNXI_DRAM_PHY0_BASE + 0x60, 1);
		setbits_le32(SUNXI_DRAM_PHY0_BASE + 8, BIT(3));
		clrbits_le32(SUNXI_DRAM_PHY0_BASE + 0x190, BIT(4));

		if (para->type == SUNXI_DRAM_TYPE_DDR4)
			clrbits_le32(SUNXI_DRAM_PHY0_BASE + 0x4, BIT(7));

		mctl_phy_dx_delay1_inner((u32 *)(SUNXI_DRAM_PHY0_BASE + 0x484),
					 para->tpr11 & 0x3f,
					 para->para0 & 0x3f);
		mctl_phy_dx_delay1_inner((u32 *)(SUNXI_DRAM_PHY0_BASE + 0x4d8),
					 (para->tpr11 >> 8) & 0x3f,
					 (para->para0 >> 8) & 0x3f);
		mctl_phy_dx_delay1_inner((u32 *)(SUNXI_DRAM_PHY0_BASE + 0x604),
					 (para->tpr11 >> 16) & 0x3f,
					 (para->para0 >> 16) & 0x3f);
		mctl_phy_dx_delay1_inner((u32 *)(SUNXI_DRAM_PHY0_BASE + 0x658),
					 (para->tpr11 >> 24) & 0x3f,
					 (para->para0 >> 24) & 0x3f);

		setbits_le32(SUNXI_DRAM_PHY0_BASE + 0x60, 1);
	}

	if (para->tpr10 & TPR10_DX_BIT_DELAY0) {
		clrbits_le32(SUNXI_DRAM_PHY0_BASE + 0x54, BIT(7));
		clrbits_le32(SUNXI_DRAM_PHY0_BASE + 0x190, BIT(2));

		mctl_phy_dx_delay0_inner((u32 *)(SUNXI_DRAM_PHY0_BASE + 0x480),
					 (u32 *)(SUNXI_DRAM_PHY0_BASE + 0x528),
					 para->tpr12 & 0x3f,
					 para->tpr14 & 0x3f);

		mctl_phy_dx_delay0_inner((u32 *)(SUNXI_DRAM_PHY0_BASE + 0x4d4),
					 (u32 *)(SUNXI_DRAM_PHY0_BASE + 0x52c),
					 (para->tpr12 >> 8) & 0x3f,
					 (para->tpr14 >> 8) & 0x3f);

		mctl_phy_dx_delay0_inner((u32 *)(SUNXI_DRAM_PHY0_BASE + 0x600),
					 (u32 *)(SUNXI_DRAM_PHY0_BASE + 0x6a8),
					 (para->tpr12 >> 16) & 0x3f,
					 (para->tpr14 >> 16) & 0x3f);

		mctl_phy_dx_delay0_inner((u32 *)(SUNXI_DRAM_PHY0_BASE + 0x6ac),
					 (u32 *)(SUNXI_DRAM_PHY0_BASE + 0x528),
					 (para->tpr12 >> 24) & 0x3f,
					 (para->tpr14 >> 24) & 0x3f);

		setbits_le32(SUNXI_DRAM_PHY0_BASE + 0x54, BIT(7));
	}
}

static bool mctl_calibrate_phy(const struct dram_para *para,
			       const struct dram_config *config)
{
	struct sunxi_mctl_ctl_reg *mctl_ctl =
		(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	int i;

	/* TODO: Implement write levelling */
	if (para->tpr10 & TPR10_READ_CALIBRATION) {
		for (i = 0; i < 5; i++)
			if (mctl_phy_read_calibration(config))
				break;
		if (i == 5) {
			debug("read calibration failed\n");
			return false;
		}
	}

	/* TODO: Implement read training */
	/* TODO: Implement write training */

	mctl_phy_dx_delay_compensation(para);
	/* TODO: Implement DFS */
	clrbits_le32(SUNXI_DRAM_PHY0_BASE + 0x60, BIT(0));
	clrbits_le32(SUNXI_DRAM_PHY0_BASE + 0x54, 7);

	/* Q: Does self-refresh get disabled by a calibration? */
	writel_relaxed(0, &mctl_ctl->swctl);
	clrbits_le32(&mctl_ctl->rfshctl3, BIT(1));
	writel_relaxed(1, &mctl_ctl->swctl);
	mctl_await_completion(&mctl_ctl->swstat, BIT(0), BIT(0));

	return true;
}

static bool mctl_core_init(const struct dram_para *para,
			   const struct dram_config *config)
{
	mctl_clk_init(para->clk);
	mctl_com_init(para, config);
	mctl_phy_init(para, config);
	mctl_dfi_init(para);

	return mctl_calibrate_phy(para, config);
}

/* Heavily inspired from H616 driver. */
static void auto_detect_ranks(const struct dram_para *para,
			      struct dram_config *config)
{
	int i;
	bool found_config;

	config->cols = 9;
	config->rows = 14;
	config->banks = 2;
	config->bankgrps = 0;

	/* Test ranks */
	found_config = false;
	for (i = 1; i >= 0; i--) {
		config->ranks = i;
		config->bus_full_width = true;
		debug("Testing ranks = %d, 32-bit bus\n", i);
		if (mctl_core_init(para, config)) {
			found_config = true;
			break;
		}

		config->bus_full_width = false;
		debug("Testing ranks = %d, 16-bit bus\n", i);
		if (mctl_core_init(para, config)) {
			found_config = true;
			break;
		}
	}
}

static void mctl_auto_detect_dram_size(const struct dram_para *para,
				       struct dram_config *config)
{
	unsigned int shift;

	/* max config for bankgrps on DDR4, minimum for everything else */
	config->cols = 8;
	config->banks = 2;
	config->rows = 14;

	shift = 1 + config->bus_full_width;
	if (para->type == SUNXI_DRAM_TYPE_DDR4) {
		config->bankgrps = 2;
		mctl_core_init(para, config);

		if (mctl_mem_matches(1ULL << (shift + 4)))
			config->bankgrps = 1;
	} else {
		/* No bank groups in (LP)DDR3/LPDDR4 */
		config->bankgrps = 0;
	}

	/* reconfigure to make sure all active columns are accessible */
	config->cols = 12;
	mctl_core_init(para, config);
	dsb();

	/* detect column address bits */
	shift = 1 + config->bus_full_width + config->bankgrps;
	for (config->cols = 8; config->cols < 12; config->cols++) {
		if (mctl_mem_matches(1ULL << (config->cols + shift)))
			break;
	}

	/* reconfigure to make sure that all active banks are accessible */
	config->banks = 3;
	mctl_core_init(para, config);
	dsb();

	/* detect bank bits */
	shift += config->cols;
	for (config->banks = 2; config->banks < 3; config->banks++) {
		if (mctl_mem_matches(1ULL << (config->banks + shift)))
			break;
	}

	/* reconfigure to make sure that all active rows are accessible */
	config->rows = 18;
	mctl_core_init(para, config);
	dsb();

	/* detect row address bits */
	shift += config->banks;
	for (config->rows = 14; config->rows < 18; config->rows++) {
		if (mctl_mem_matches(1ULL << (config->rows + shift)))
			break;
	}
}

/* Modified from H616 driver to add banks and bank groups */
static unsigned long calculate_dram_size(const struct dram_config *config)
{
	/* Bootrom only uses x32 or x16 bus widths */
	u8 width = config->bus_full_width ? 4 : 2;

	return (1ULL << (config->cols + config->rows + config->banks +
			 config->bankgrps)) *
	       width * (1ULL << config->ranks);
}

static const struct dram_para para = {
	.clk = CONFIG_DRAM_CLK,
#ifdef CONFIG_SUNXI_DRAM_DDR3
	.type = SUNXI_DRAM_TYPE_DDR3,
#elif defined(CONFIG_SUNXI_DRAM_DDR4)
	.type = SUNXI_DRAM_TYPE_DDR4,
#elif defined(CONFIG_SUNXI_DRAM_LPDDR3)
	.type = SUNXI_DRAM_TYPE_LPDDR3,
#elif defined(CONFIG_SUNXI_DRAM_LPDDR4)
	.type = SUNXI_DRAM_TYPE_LPDDR4,
#endif
	.dx_odt = CONFIG_DRAM_SUN50I_DX_ODT,
	.dx_dri = CONFIG_DRAM_SUN50I_DX_DRI,
	.ca_dri = CONFIG_DRAM_SUN50I_CA_DRI,
	.para0 = CONFIG_DRAM_SUN50I_PARA0,
	.mr0 = CONFIG_DRAM_SUN50I_MR0,
	.mr1 = CONFIG_DRAM_SUN50I_MR1,
	.mr2 = CONFIG_DRAM_SUN50I_MR2,
	.mr3 = CONFIG_DRAM_SUN50I_MR3,
	.mr4 = CONFIG_DRAM_SUN50I_MR4,
	.mr5 = CONFIG_DRAM_SUN50I_MR5,
	.mr6 = CONFIG_DRAM_SUN50I_MR6,
	.mr11 = CONFIG_DRAM_SUN50I_MR11,
	.mr12 = CONFIG_DRAM_SUN50I_MR12,
	.mr13 = CONFIG_DRAM_SUN50I_MR13,
	.mr14 = CONFIG_DRAM_SUN50I_MR14,
	.mr16 = CONFIG_DRAM_SUN50I_MR16,
	.mr17 = CONFIG_DRAM_SUN50I_MR17,
	.tpr1 = CONFIG_DRAM_SUN50I_TPR1,
	.tpr2 = CONFIG_DRAM_SUN50I_TPR2,
	.tpr3 = CONFIG_DRAM_SUN50I_TPR3,
	.tpr6 = CONFIG_DRAM_SUN50I_TPR6,
	.tpr10 = CONFIG_DRAM_SUN50I_TPR10,
	.tpr11 = CONFIG_DRAM_SUN50I_TPR11,
	.tpr12 = CONFIG_DRAM_SUN50I_TPR12,
	.tpr13 = CONFIG_DRAM_SUN50I_TPR13,
	.tpr14 = CONFIG_DRAM_SUN50I_TPR14,
};

/* TODO: Remove, copied and modified slightly from aodzip repo as temporary sanity check */
static int libdram_dramc_simple_wr_test(unsigned long dram_size,
					uint32_t test_range)
{
	uint64_t *dram_memory = (uint64_t *)CFG_SYS_SDRAM_BASE;
	unsigned long step = dram_size / 16;
	int64_t error_value;

	for (unsigned i = 0; i < test_range; i++) {
		dram_memory[i] = i + 0x123456789ABCDEF;
		dram_memory[i + step] = i - 0xFEDCBA987654321;
	}

	dmb();

	for (unsigned i = 0; i < test_range; i++) {
		uint64_t *ptr;
		if (dram_memory[i] != i + 0x123456789ABCDEF) {
			ptr = &dram_memory[i];
			goto fail;
		}
		if (dram_memory[i + step] != i - 0xFEDCBA987654321) {
			ptr = &dram_memory[i + step];
			goto fail;
		}
		continue;
fail:
		error_value = (int64_t)readq(ptr);
		debug("DRAM simple test FAIL----- address %p = %lx\n", ptr,
		      error_value);

		if (error_value < 0)
			debug("Potentially aliased with %lx\n",
			      CFG_SYS_SDRAM_BASE +
				      (step + error_value + 0xFEDCBA987654321) *
					      8);
		else
			debug("Potentially aliased with %lx\n",
			      CFG_SYS_SDRAM_BASE +
				      (error_value - 0x123456789ABCDEF) * 8);
		return 1;
	}

	debug("DRAM simple test OK.\n");
	return 0;
}

unsigned long sunxi_dram_init(void)
{
	unsigned long size;

	struct dram_config config;

	/* Writing to undocumented SYS_CFG area, according to user manual. */
	setbits_le32(0x03000160, BIT(8));
	clrbits_le32(0x03000168, 0x3f);

	auto_detect_ranks(&para, &config);
	mctl_auto_detect_dram_size(&para, &config);

	if (!mctl_core_init(&para, &config))
		return 0;

	debug("cols = %d, rows = %d, banks = %d, bank groups = %d, ranks = %d, full width = %d\n",
	      config.cols, config.rows, config.banks, config.bankgrps,
	      config.ranks, config.bus_full_width);

	size = calculate_dram_size(&config);
	debug("expected size: %lu MB\n", size >> 20);

	/* TODO: This is just a sanity check for now. */
	if (libdram_dramc_simple_wr_test(size, 16384))
		return 0;

	return size;
}
