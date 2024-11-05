#include <asm/arch/cpu.h>
#include <asm/arch/dram.h>

void mctl_set_timing_params(const struct dram_para *para)
{
	struct sunxi_mctl_ctl_reg *const mctl_ctl =
		(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	bool tpr13_flag1 = para->tpr13 & BIT(28);
	bool tpr13_flag2 = para->tpr13 & BIT(3);
	bool tpr13_flag3 = para->tpr13 & BIT(5);

	u8 tccd		= 4;
	u8 tfaw		= ns_to_t(40);
	u8 trrd		= max(ns_to_t(10), 2);
	u8 trcd		= max(ns_to_t(18), 2);
	u8 trc		= ns_to_t(65);
	u8 txp		= max(ns_to_t(8), 2);

	u8 trp		= ns_to_t(21);
	u8 tras_min	= ns_to_t(42);
	u16 trefi_x32	= ns_to_t(3904) / 32;
	u16 trfc_min	= ns_to_t(180);
	u16 txsr	= ns_to_t(190);

	u8 tmrw		= max(ns_to_t(14), 5);
	u8 tmrd		= max(ns_to_t(14), 5);
	u8 tmod		= 12;
	u8 tcke		= max(ns_to_t(15), 2);
	u8 tcksrx	= max(ns_to_t(2), 2);
	u8 tcksre	= max(ns_to_t(5), 2);
	u8 tckesr	= max(ns_to_t(15), 2);
	u8 tras_max	= (trefi_x32 * 9) / 32;
	u8 txs_x32	= 4;
	u8 txsabort_x32 = 4;

	u8 wrlat        = 5;
	u8 wr2rd_s      = 8;
	u8 trrd_s       = 2;
	u8 tmrd_pda     = 8;

	u8 wr2pre       = 24;
	u8 rd2pre       = 4;
	u8 wr2rd        = 14 + max(ns_to_t(tpr13_flag1 ? 10 : 12), 4);
	u8 rd2wr        = 17 + ns_to_t(4) - ns_to_t(1);
	u8 tphy_wrlat	= 5;

	u8 rdlat	= 10;
	u8 trddata_en	= 17;

	if (tpr13_flag1) {
		rdlat = 11;
		trddata_en = 19;
	}

	writel_relaxed(tras_min | tras_max << 8 | tfaw << 16 | wr2pre << 24,
	       &mctl_ctl->dramtmg[0]);
	writel_relaxed(trc | rd2pre << 8 | txp << 16, &mctl_ctl->dramtmg[1]);
	writel_relaxed(wr2rd | rd2wr << 8 | rdlat << 16 | wrlat << 24,
	       &mctl_ctl->dramtmg[2]);
	writel_relaxed(tmod | tmrd << 12 | tmrw << 20, &mctl_ctl->dramtmg[3]);
	writel_relaxed(trp | trrd << 8 | tccd << 16 | trcd << 24,
	       &mctl_ctl->dramtmg[4]);
	writel_relaxed(tcke | tckesr << 8 | tcksre << 16 | tcksrx << 24,
	       &mctl_ctl->dramtmg[5]);
	writel_relaxed((txp + 2) | 0x20UL << 16 | 0x20UL << 24, &mctl_ctl->dramtmg[6]);
	writel_relaxed(txs_x32 | 0x10 << 8 | txsabort_x32 << 16 | txsabort_x32 << 24,
	       &mctl_ctl->dramtmg[8]);
	writel_relaxed(wr2rd_s | trrd_s << 8 | 0x2 << 16, &mctl_ctl->dramtmg[9]);
	writel_relaxed(0xe0c05, &mctl_ctl->dramtmg[10]);
	writel_relaxed(0x440c021c, &mctl_ctl->dramtmg[11]);
	writel_relaxed(tmrd_pda, &mctl_ctl->dramtmg[12]);
	writel_relaxed(0xa100002, &mctl_ctl->dramtmg[13]);
	writel_relaxed(txsr, &mctl_ctl->dramtmg[14]);

	clrsetbits_le32(&mctl_ctl->init[0], 0xc0000fff, 1008);

	if (tpr13_flag2)
		writel_relaxed(0x420000, &mctl_ctl->init[1]);
	else
		writel_relaxed(0x1f20000, &mctl_ctl->init[1]);

	clrsetbits_le32(&mctl_ctl->init[2], 0xff0f, 0xd05);
	writel_relaxed(0, &mctl_ctl->dfimisc);

	writel_relaxed(para->mr1 << 16 | para->mr2, &mctl_ctl->init[3]);
	writel_relaxed(para->mr3 << 16, &mctl_ctl->init[4]);
	writel_relaxed(para->mr11 << 16 | para->mr12, &mctl_ctl->init[6]);
	writel_relaxed(para->tpr1 << 16 | para->mr14, &mctl_ctl->init[7]);

	clrsetbits_le32(&mctl_ctl->rankctl, 0xff0, 0x660);
	if (!tpr13_flag3) {
		tphy_wrlat -= 1;
		trddata_en -= 1;
	}

	writel_relaxed(tphy_wrlat | trddata_en << 16 | 0x808000 | 0x2000000,
	       &mctl_ctl->dfitmg0);
	writel_relaxed(0x100202, &mctl_ctl->dfitmg1);

	writel_relaxed(trfc_min | trefi_x32 << 16, &mctl_ctl->rfshtmg);
}
