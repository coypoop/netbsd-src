/*
 * Copyright (c) 2010 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _BRCM_PHY_INT_H_
#define _BRCM_PHY_INT_H_

#include <bcm_types.h>
#include <brcmu_utils.h>
#include <brcmu_wifi.h>

#define	PHY_VERSION			{ 1, 82, 8, 0 }

#define LCNXN_BASEREV		16

struct phy_shim_info;

struct brcms_phy_srom_fem {
	/* TSSI positive slope, 1: positive, 0: negative */
	uint8_t tssipos;
	/* Ext PA gain-type: full-gain: 0, pa-lite: 1, no_pa: 2 */
	uint8_t extpagain;
	/* support 32 combinations of different Pdet dynamic ranges */
	uint8_t pdetrange;
	/* TR switch isolation */
	uint8_t triso;
	/* antswctrl lookup table configuration: 32 possible choices */
	uint8_t antswctrllut;
};

#define ISNPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_N)
#define ISLCNPHY(pi)	PHYTYPE_IS((pi)->pubpi.phy_type, PHY_TYPE_LCN)

#define PHY_GET_RFATTN(rfgain)	((rfgain) & 0x0f)
#define PHY_GET_PADMIX(rfgain)	(((rfgain) & 0x10) >> 4)
#define PHY_GET_RFGAINID(rfattn, padmix, width)	((rfattn) + ((padmix)*(width)))
#define PHY_SAT(x, n)		((x) > ((1<<((n)-1))-1) ? ((1<<((n)-1))-1) : \
				((x) < -(1<<((n)-1)) ? -(1<<((n)-1)) : (x)))
#define PHY_SHIFT_ROUND(x, n)	((x) >= 0 ? ((x)+(1<<((n)-1)))>>(n) : (x)>>(n))
#define PHY_HW_ROUND(x, s)		((x >> s) + ((x >> (s-1)) & (s != 0)))

#define CH_5G_GROUP	3
#define A_LOW_CHANS	0
#define A_MID_CHANS	1
#define A_HIGH_CHANS	2
#define CH_2G_GROUP	1
#define G_ALL_CHANS	0

#define FIRST_REF5_CHANNUM	149
#define LAST_REF5_CHANNUM	165
#define	FIRST_5G_CHAN		14
#define	LAST_5G_CHAN		50
#define	FIRST_MID_5G_CHAN	14
#define	LAST_MID_5G_CHAN	35
#define	FIRST_HIGH_5G_CHAN	36
#define	LAST_HIGH_5G_CHAN	41
#define	FIRST_LOW_5G_CHAN	42
#define	LAST_LOW_5G_CHAN	50

#define BASE_LOW_5G_CHAN	4900
#define BASE_MID_5G_CHAN	5100
#define BASE_HIGH_5G_CHAN	5500

#define CHAN5G_FREQ(chan)  (5000 + chan*5)
#define CHAN2G_FREQ(chan)  (2407 + chan*5)

#define TXP_FIRST_CCK		0
#define TXP_LAST_CCK		3
#define TXP_FIRST_OFDM		4
#define TXP_LAST_OFDM		11
#define TXP_FIRST_OFDM_20_CDD	12
#define TXP_LAST_OFDM_20_CDD	19
#define TXP_FIRST_MCS_20_SISO	20
#define TXP_LAST_MCS_20_SISO	27
#define TXP_FIRST_MCS_20_CDD	28
#define TXP_LAST_MCS_20_CDD	35
#define TXP_FIRST_MCS_20_STBC	36
#define TXP_LAST_MCS_20_STBC	43
#define TXP_FIRST_MCS_20_SDM	44
#define TXP_LAST_MCS_20_SDM	51
#define TXP_FIRST_OFDM_40_SISO	52
#define TXP_LAST_OFDM_40_SISO	59
#define TXP_FIRST_OFDM_40_CDD	60
#define TXP_LAST_OFDM_40_CDD	67
#define TXP_FIRST_MCS_40_SISO	68
#define TXP_LAST_MCS_40_SISO	75
#define TXP_FIRST_MCS_40_CDD	76
#define TXP_LAST_MCS_40_CDD	83
#define TXP_FIRST_MCS_40_STBC	84
#define TXP_LAST_MCS_40_STBC	91
#define TXP_FIRST_MCS_40_SDM	92
#define TXP_LAST_MCS_40_SDM	99
#define TXP_MCS_32	        100
#define TXP_NUM_RATES		101
#define ADJ_PWR_TBL_LEN		84

#define TXP_FIRST_SISO_MCS_20	20
#define TXP_LAST_SISO_MCS_20	27

#define PHY_CORE_NUM_1	1
#define PHY_CORE_NUM_2	2
#define PHY_CORE_NUM_3	3
#define PHY_CORE_NUM_4	4
#define PHY_CORE_MAX	PHY_CORE_NUM_4
#define PHY_CORE_0	0
#define PHY_CORE_1	1
#define PHY_CORE_2	2
#define PHY_CORE_3	3

#define MA_WINDOW_SZ		8

#define PHY_NOISE_SAMPLE_MON		1
#define PHY_NOISE_SAMPLE_EXTERNAL	2
#define PHY_NOISE_WINDOW_SZ	16
#define PHY_NOISE_GLITCH_INIT_MA 10
#define PHY_NOISE_GLITCH_INIT_MA_BADPlCP 10
#define PHY_NOISE_STATE_MON		0x1
#define PHY_NOISE_STATE_EXTERNAL	0x2
#define PHY_NOISE_SAMPLE_LOG_NUM_NPHY	10
#define PHY_NOISE_SAMPLE_LOG_NUM_UCODE	9

#define PHY_NOISE_OFFSETFACT_4322  (-103)
#define PHY_NOISE_MA_WINDOW_SZ	2

#define	PHY_RSSI_TABLE_SIZE	64
#define RSSI_ANT_MERGE_MAX	0
#define RSSI_ANT_MERGE_MIN	1
#define RSSI_ANT_MERGE_AVG	2

#define	PHY_TSSI_TABLE_SIZE	64
#define	APHY_TSSI_TABLE_SIZE	256
#define	TX_GAIN_TABLE_LENGTH	64
#define	DEFAULT_11A_TXP_IDX	24
#define NUM_TSSI_FRAMES        4
#define	NULL_TSSI		0x7f
#define	NULL_TSSI_W		0x7f7f

#define PHY_PAPD_EPS_TBL_SIZE_LCNPHY 64

#define LCNPHY_PERICAL_TEMPBASED_TXPWRCTRL 9

#define PHY_TXPWR_MIN		10
#define PHY_TXPWR_MIN_NPHY	8
#define RADIOPWR_OVERRIDE_DEF	(-1)

#define PWRTBL_NUM_COEFF	3

#define SPURAVOID_DISABLE	0
#define SPURAVOID_AUTO		1
#define SPURAVOID_FORCEON	2
#define SPURAVOID_FORCEON2	3

#define PHY_SW_TIMER_FAST		15
#define PHY_SW_TIMER_SLOW		60
#define PHY_SW_TIMER_GLACIAL	120

#define PHY_PERICAL_AUTO	0
#define PHY_PERICAL_FULL	1
#define PHY_PERICAL_PARTIAL	2

#define PHY_PERICAL_NODELAY	0
#define PHY_PERICAL_INIT_DELAY	5
#define PHY_PERICAL_ASSOC_DELAY	5
#define PHY_PERICAL_WDOG_DELAY	5

#define MPHASE_TXCAL_NUMCMDS	2

#define PHY_PERICAL_MPHASE_PENDING(pi) \
	(pi->mphase_cal_phase_id > MPHASE_CAL_STATE_IDLE)

enum {
	MPHASE_CAL_STATE_IDLE = 0,
	MPHASE_CAL_STATE_INIT = 1,
	MPHASE_CAL_STATE_TXPHASE0,
	MPHASE_CAL_STATE_TXPHASE1,
	MPHASE_CAL_STATE_TXPHASE2,
	MPHASE_CAL_STATE_TXPHASE3,
	MPHASE_CAL_STATE_TXPHASE4,
	MPHASE_CAL_STATE_TXPHASE5,
	MPHASE_CAL_STATE_PAPDCAL,
	MPHASE_CAL_STATE_RXCAL,
	MPHASE_CAL_STATE_RSSICAL,
	MPHASE_CAL_STATE_IDLETSSI
};

enum phy_cal_mode {
	CAL_FULL,
	CAL_RECAL,
	CAL_CURRECAL,
	CAL_DIGCAL,
	CAL_GCTRL,
	CAL_SOFT,
	CAL_DIGLO
};

#define RDR_NTIERS  1
#define RDR_TIER_SIZE 64
#define RDR_LIST_SIZE (512/3)
#define RDR_EPOCH_SIZE 40
#define RDR_NANTENNAS 2
#define RDR_NTIER_SIZE  RDR_LIST_SIZE
#define RDR_LP_BUFFER_SIZE 64
#define LP_LEN_HIS_SIZE 10

#define STATIC_NUM_RF 32
#define STATIC_NUM_BB 9

#define BB_MULT_MASK		0x0000ffff
#define BB_MULT_VALID_MASK	0x80000000

#define CORDIC_AG	39797
#define	CORDIC_NI	18
#define	FIXED(X)	((int32_t)((X) << 16))

#define	FLOAT(X) \
	(((X) >= 0) ? ((((X) >> 15) + 1) >> 1) : -((((-(X)) >> 15) + 1) >> 1))

#define PHY_CHAIN_TX_DISABLE_TEMP	115
#define PHY_HYSTERESIS_DELTATEMP	5

#define SCAN_INPROG_PHY(pi) \
	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_SCAN))

#define PLT_INPROG_PHY(pi)      (mboolisset(pi->measure_hold, PHY_HOLD_FOR_PLT))

#define ASSOC_INPROG_PHY(pi) \
	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_ASSOC))

#define SCAN_RM_IN_PROGRESS(pi) \
	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_SCAN | PHY_HOLD_FOR_RM))

#define PHY_MUTED(pi) \
	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_MUTE))

#define PUB_NOT_ASSOC(pi) \
	(mboolisset(pi->measure_hold, PHY_HOLD_FOR_NOT_ASSOC))

struct phy_table_info {
	uint table;
	int q;
	uint max;
};

struct phytbl_info {
	const void *tbl_ptr;
	uint32_t tbl_len;
	uint32_t tbl_id;
	uint32_t tbl_offset;
	uint32_t tbl_width;
};

struct interference_info {
	uint8_t curr_home_channel;
	uint16_t crsminpwrthld_40_stored;
	uint16_t crsminpwrthld_20L_stored;
	uint16_t crsminpwrthld_20U_stored;
	uint16_t init_gain_code_core1_stored;
	uint16_t init_gain_code_core2_stored;
	uint16_t init_gain_codeb_core1_stored;
	uint16_t init_gain_codeb_core2_stored;
	uint16_t init_gain_table_stored[4];

	uint16_t clip1_hi_gain_code_core1_stored;
	uint16_t clip1_hi_gain_code_core2_stored;
	uint16_t clip1_hi_gain_codeb_core1_stored;
	uint16_t clip1_hi_gain_codeb_core2_stored;
	uint16_t nb_clip_thresh_core1_stored;
	uint16_t nb_clip_thresh_core2_stored;
	uint16_t init_ofdmlna2gainchange_stored[4];
	uint16_t init_ccklna2gainchange_stored[4];
	uint16_t clip1_lo_gain_code_core1_stored;
	uint16_t clip1_lo_gain_code_core2_stored;
	uint16_t clip1_lo_gain_codeb_core1_stored;
	uint16_t clip1_lo_gain_codeb_core2_stored;
	uint16_t w1_clip_thresh_core1_stored;
	uint16_t w1_clip_thresh_core2_stored;
	uint16_t radio_2056_core1_rssi_gain_stored;
	uint16_t radio_2056_core2_rssi_gain_stored;
	uint16_t energy_drop_timeout_len_stored;

	uint16_t ed_crs40_assertthld0_stored;
	uint16_t ed_crs40_assertthld1_stored;
	uint16_t ed_crs40_deassertthld0_stored;
	uint16_t ed_crs40_deassertthld1_stored;
	uint16_t ed_crs20L_assertthld0_stored;
	uint16_t ed_crs20L_assertthld1_stored;
	uint16_t ed_crs20L_deassertthld0_stored;
	uint16_t ed_crs20L_deassertthld1_stored;
	uint16_t ed_crs20U_assertthld0_stored;
	uint16_t ed_crs20U_assertthld1_stored;
	uint16_t ed_crs20U_deassertthld0_stored;
	uint16_t ed_crs20U_deassertthld1_stored;

	uint16_t badplcp_ma;
	uint16_t badplcp_ma_previous;
	uint16_t badplcp_ma_total;
	uint16_t badplcp_ma_list[MA_WINDOW_SZ];
	int badplcp_ma_index;
	int16_t pre_badplcp_cnt;
	int16_t bphy_pre_badplcp_cnt;

	uint16_t init_gain_core1;
	uint16_t init_gain_core2;
	uint16_t init_gainb_core1;
	uint16_t init_gainb_core2;
	uint16_t init_gain_rfseq[4];

	uint16_t crsminpwr0;
	uint16_t crsminpwrl0;
	uint16_t crsminpwru0;

	int16_t crsminpwr_index;

	uint16_t radio_2057_core1_rssi_wb1a_gc_stored;
	uint16_t radio_2057_core2_rssi_wb1a_gc_stored;
	uint16_t radio_2057_core1_rssi_wb1g_gc_stored;
	uint16_t radio_2057_core2_rssi_wb1g_gc_stored;
	uint16_t radio_2057_core1_rssi_wb2_gc_stored;
	uint16_t radio_2057_core2_rssi_wb2_gc_stored;
	uint16_t radio_2057_core1_rssi_nb_gc_stored;
	uint16_t radio_2057_core2_rssi_nb_gc_stored;
};

struct aci_save_gphy {
	uint16_t rc_cal_ovr;
	uint16_t phycrsth1;
	uint16_t phycrsth2;
	uint16_t init_n1p1_gain;
	uint16_t p1_p2_gain;
	uint16_t n1_n2_gain;
	uint16_t n1_p1_gain;
	uint16_t div_search_gain;
	uint16_t div_p1_p2_gain;
	uint16_t div_search_gn_change;
	uint16_t table_7_2;
	uint16_t table_7_3;
	uint16_t cckshbits_gnref;
	uint16_t clip_thresh;
	uint16_t clip2_thresh;
	uint16_t clip3_thresh;
	uint16_t clip_p2_thresh;
	uint16_t clip_pwdn_thresh;
	uint16_t clip_n1p1_thresh;
	uint16_t clip_n1_pwdn_thresh;
	uint16_t bbconfig;
	uint16_t cthr_sthr_shdin;
	uint16_t energy;
	uint16_t clip_p1_p2_thresh;
	uint16_t threshold;
	uint16_t reg15;
	uint16_t reg16;
	uint16_t reg17;
	uint16_t div_srch_idx;
	uint16_t div_srch_p1_p2;
	uint16_t div_srch_gn_back;
	uint16_t ant_dwell;
	uint16_t ant_wr_settle;
};

struct lo_complex_abgphy_info {
	int8_t i;
	int8_t q;
};

struct nphy_iq_comp {
	int16_t a0;
	int16_t b0;
	int16_t a1;
	int16_t b1;
};

struct nphy_txpwrindex {
	int8_t index;
	int8_t index_internal;
	int8_t index_internal_save;
	uint16_t AfectrlOverride;
	uint16_t AfeCtrlDacGain;
	uint16_t rad_gain;
	uint8_t bbmult;
	uint16_t iqcomp_a;
	uint16_t iqcomp_b;
	uint16_t locomp;
};

struct txiqcal_cache {

	uint16_t txcal_coeffs_2G[8];
	uint16_t txcal_radio_regs_2G[8];
	struct nphy_iq_comp rxcal_coeffs_2G;

	uint16_t txcal_coeffs_5G[8];
	uint16_t txcal_radio_regs_5G[8];
	struct nphy_iq_comp rxcal_coeffs_5G;
};

struct nphy_pwrctrl {
	int8_t max_pwr_2g;
	int8_t idle_targ_2g;
	int16_t pwrdet_2g_a1;
	int16_t pwrdet_2g_b0;
	int16_t pwrdet_2g_b1;
	int8_t max_pwr_5gm;
	int8_t idle_targ_5gm;
	int8_t max_pwr_5gh;
	int8_t max_pwr_5gl;
	int16_t pwrdet_5gm_a1;
	int16_t pwrdet_5gm_b0;
	int16_t pwrdet_5gm_b1;
	int16_t pwrdet_5gl_a1;
	int16_t pwrdet_5gl_b0;
	int16_t pwrdet_5gl_b1;
	int16_t pwrdet_5gh_a1;
	int16_t pwrdet_5gh_b0;
	int16_t pwrdet_5gh_b1;
	int8_t idle_targ_5gl;
	int8_t idle_targ_5gh;
	int8_t idle_tssi_2g;
	int8_t idle_tssi_5g;
	int8_t idle_tssi;
	int16_t a1;
	int16_t b0;
	int16_t b1;
};

struct nphy_txgains {
	uint16_t txlpf[2];
	uint16_t txgm[2];
	uint16_t pga[2];
	uint16_t pad[2];
	uint16_t ipa[2];
};

#define PHY_NOISEVAR_BUFSIZE 10

struct nphy_noisevar_buf {
	int bufcount;
	int tone_id[PHY_NOISEVAR_BUFSIZE];
	uint32_t noise_vars[PHY_NOISEVAR_BUFSIZE];
	uint32_t min_noise_vars[PHY_NOISEVAR_BUFSIZE];
};

struct rssical_cache {
	uint16_t rssical_radio_regs_2G[2];
	uint16_t rssical_phyregs_2G[12];

	uint16_t rssical_radio_regs_5G[2];
	uint16_t rssical_phyregs_5G[12];
};

struct lcnphy_cal_results {

	uint16_t txiqlocal_a;
	uint16_t txiqlocal_b;
	uint16_t txiqlocal_didq;
	uint8_t txiqlocal_ei0;
	uint8_t txiqlocal_eq0;
	uint8_t txiqlocal_fi0;
	uint8_t txiqlocal_fq0;

	uint16_t txiqlocal_bestcoeffs[11];
	uint16_t txiqlocal_bestcoeffs_valid;

	uint32_t papd_eps_tbl[PHY_PAPD_EPS_TBL_SIZE_LCNPHY];
	uint16_t analog_gain_ref;
	uint16_t lut_begin;
	uint16_t lut_end;
	uint16_t lut_step;
	uint16_t rxcompdbm;
	uint16_t papdctrl;
	uint16_t sslpnCalibClkEnCtrl;

	uint16_t rxiqcal_coeff_a0;
	uint16_t rxiqcal_coeff_b0;
};

struct shared_phy {
	struct brcms_phy *phy_head;
	uint unit;
	struct phy_shim_info *physhim;
	uint corerev;
	uint32_t machwcap;
	bool up;
	bool clk;
	uint now;
	uint16_t vid;
	uint16_t did;
	uint chip;
	uint chiprev;
	uint chippkg;
	uint sromrev;
	uint boardtype;
	uint boardrev;
	uint32_t boardflags;
	uint32_t boardflags2;
	uint fast_timer;
	uint slow_timer;
	uint glacial_timer;
	uint8_t rx_antdiv;
	int8_t phy_noise_window[MA_WINDOW_SZ];
	uint phy_noise_index;
	uint8_t hw_phytxchain;
	uint8_t hw_phyrxchain;
	uint8_t phytxchain;
	uint8_t phyrxchain;
	uint8_t rssi_mode;
	bool _rifs_phy;
};

struct brcms_phy_pub {
	uint phy_type;
	uint phy_rev;
	uint8_t phy_corenum;
	uint16_t radioid;
	uint8_t radiorev;
	uint8_t radiover;

	uint coreflags;
	uint ana_rev;
	bool abgphy_encore;
};

struct phy_func_ptr {
	void (*init)(struct brcms_phy *);
	void (*calinit)(struct brcms_phy *);
	void (*chanset)(struct brcms_phy *, uint16_t chanspec);
	void (*txpwrrecalc)(struct brcms_phy *);
	int (*longtrn)(struct brcms_phy *, int);
	void (*txiqccget)(struct brcms_phy *, uint16_t *, uint16_t *);
	void (*txiqccset)(struct brcms_phy *, uint16_t, uint16_t);
	uint16_t (*txloccget)(struct brcms_phy *);
	void (*radioloftget)(struct brcms_phy *, uint8_t *, uint8_t *, uint8_t *, uint8_t *);
	void (*carrsuppr)(struct brcms_phy *);
	int32_t (*rxsigpwr)(struct brcms_phy *, int32_t);
	void (*detach)(struct brcms_phy *);
};

struct brcms_phy {
	struct brcms_phy_pub pubpi_ro;
	struct shared_phy *sh;
	struct phy_func_ptr pi_fptr;

	union {
		struct brcms_phy_lcnphy *pi_lcnphy;
	} u;
	bool user_txpwr_at_rfport;

	struct bcma_device *d11core;
	struct brcms_phy *next;
	struct brcms_phy_pub pubpi;

	bool do_initcal;
	bool phytest_on;
	bool ofdm_rateset_war;
	bool bf_preempt_4306;
	uint16_t radio_chanspec;
	uint8_t antsel_type;
	uint16_t bw;
	uint8_t txpwr_percent;
	bool phy_init_por;

	bool init_in_progress;
	bool initialized;
	bool sbtml_gm;
	uint refcnt;
	bool watchdog_override;
	uint8_t phynoise_state;
	uint phynoise_now;
	int phynoise_chan_watchdog;
	bool phynoise_polling;
	bool disable_percal;
	uint32_t measure_hold;

	int16_t txpa_2g[PWRTBL_NUM_COEFF];
	int16_t txpa_2g_low_temp[PWRTBL_NUM_COEFF];
	int16_t txpa_2g_high_temp[PWRTBL_NUM_COEFF];
	int16_t txpa_5g_low[PWRTBL_NUM_COEFF];
	int16_t txpa_5g_mid[PWRTBL_NUM_COEFF];
	int16_t txpa_5g_hi[PWRTBL_NUM_COEFF];

	uint8_t tx_srom_max_2g;
	uint8_t tx_srom_max_5g_low;
	uint8_t tx_srom_max_5g_mid;
	uint8_t tx_srom_max_5g_hi;
	uint8_t tx_srom_max_rate_2g[TXP_NUM_RATES];
	uint8_t tx_srom_max_rate_5g_low[TXP_NUM_RATES];
	uint8_t tx_srom_max_rate_5g_mid[TXP_NUM_RATES];
	uint8_t tx_srom_max_rate_5g_hi[TXP_NUM_RATES];
	uint8_t tx_user_target[TXP_NUM_RATES];
	int8_t tx_power_offset[TXP_NUM_RATES];
	uint8_t tx_power_target[TXP_NUM_RATES];

	struct brcms_phy_srom_fem srom_fem2g;
	struct brcms_phy_srom_fem srom_fem5g;

	uint8_t tx_power_max;
	uint8_t tx_power_max_rate_ind;
	bool hwpwrctrl;
	uint8_t nphy_txpwrctrl;
	int8_t nphy_txrx_chain;
	bool phy_5g_pwrgain;

	uint16_t phy_wreg;
	uint16_t phy_wreg_limit;

	int8_t n_preamble_override;
	uint8_t antswitch;
	uint8_t aa2g, aa5g;

	int8_t idle_tssi[CH_5G_GROUP];
	int8_t target_idle_tssi;
	int8_t txpwr_est_Pout;
	uint8_t tx_power_min;
	uint8_t txpwr_limit[TXP_NUM_RATES];
	uint8_t txpwr_env_limit[TXP_NUM_RATES];
	uint8_t adj_pwr_tbl_nphy[ADJ_PWR_TBL_LEN];

	bool channel_14_wide_filter;

	bool txpwroverride;
	bool txpwridx_override_aphy;
	int16_t radiopwr_override;
	uint16_t hwpwr_txcur;
	uint8_t saved_txpwr_idx;

	bool edcrs_threshold_lock;

	uint32_t tr_R_gain_val;
	uint32_t tr_T_gain_val;

	int16_t ofdm_analog_filt_bw_override;
	int16_t cck_analog_filt_bw_override;
	int16_t ofdm_rccal_override;
	int16_t cck_rccal_override;
	uint16_t extlna_type;

	uint interference_mode_crs_time;
	uint16_t crsglitch_prev;
	bool interference_mode_crs;

	uint32_t phy_tx_tone_freq;
	uint phy_lastcal;
	bool phy_forcecal;
	bool phy_fixed_noise;
	uint32_t xtalfreq;
	uint8_t pdiv;
	int8_t carrier_suppr_disable;

	bool phy_bphy_evm;
	bool phy_bphy_rfcs;
	int8_t phy_scraminit;
	uint8_t phy_gpiosel;

	int16_t phy_txcore_disable_temp;
	int16_t phy_txcore_enable_temp;
	int8_t phy_tempsense_offset;
	bool phy_txcore_heatedup;

	uint16_t radiopwr;
	uint16_t bb_atten;
	uint16_t txctl1;

	uint16_t mintxbias;
	uint16_t mintxmag;
	struct lo_complex_abgphy_info gphy_locomp_iq
			[STATIC_NUM_RF][STATIC_NUM_BB];
	int8_t stats_11b_txpower[STATIC_NUM_RF][STATIC_NUM_BB];
	uint16_t gain_table[TX_GAIN_TABLE_LENGTH];
	bool loopback_gain;
	int16_t max_lpback_gain_hdB;
	int16_t trsw_rx_gain_hdB;
	uint8_t power_vec[8];

	uint16_t rc_cal;
	int nrssi_table_delta;
	int nrssi_slope_scale;
	int nrssi_slope_offset;
	int min_rssi;
	int max_rssi;

	int8_t txpwridx;
	uint8_t min_txpower;

	uint8_t a_band_high_disable;

	uint16_t tx_vos;
	uint16_t global_tx_bb_dc_bias_loft;

	int rf_max;
	int bb_max;
	int rf_list_size;
	int bb_list_size;
	uint16_t *rf_attn_list;
	uint16_t *bb_attn_list;
	uint16_t padmix_mask;
	uint16_t padmix_reg;
	uint16_t *txmag_list;
	uint txmag_len;
	bool txmag_enable;

	int8_t *a_tssi_to_dbm;
	int8_t *m_tssi_to_dbm;
	int8_t *l_tssi_to_dbm;
	int8_t *h_tssi_to_dbm;
	uint8_t *hwtxpwr;

	uint16_t freqtrack_saved_regs[2];
	int cur_interference_mode;
	bool hwpwrctrl_capable;
	bool temppwrctrl_capable;

	uint phycal_nslope;
	uint phycal_noffset;
	uint phycal_mlo;
	uint phycal_txpower;

	uint8_t phy_aa2g;

	bool nphy_tableloaded;
	int8_t nphy_rssisel;
	uint32_t nphy_bb_mult_save;
	uint16_t nphy_txiqlocal_bestc[11];
	bool nphy_txiqlocal_coeffsvalid;
	struct nphy_txpwrindex nphy_txpwrindex[PHY_CORE_NUM_2];
	struct nphy_pwrctrl nphy_pwrctrl_info[PHY_CORE_NUM_2];
	uint16_t cck2gpo;
	uint32_t ofdm2gpo;
	uint32_t ofdm5gpo;
	uint32_t ofdm5glpo;
	uint32_t ofdm5ghpo;
	uint8_t bw402gpo;
	uint8_t bw405gpo;
	uint8_t bw405glpo;
	uint8_t bw405ghpo;
	uint8_t cdd2gpo;
	uint8_t cdd5gpo;
	uint8_t cdd5glpo;
	uint8_t cdd5ghpo;
	uint8_t stbc2gpo;
	uint8_t stbc5gpo;
	uint8_t stbc5glpo;
	uint8_t stbc5ghpo;
	uint8_t bwdup2gpo;
	uint8_t bwdup5gpo;
	uint8_t bwdup5glpo;
	uint8_t bwdup5ghpo;
	uint16_t mcs2gpo[8];
	uint16_t mcs5gpo[8];
	uint16_t mcs5glpo[8];
	uint16_t mcs5ghpo[8];
	uint32_t nphy_rxcalparams;

	uint8_t phy_spuravoid;
	bool phy_isspuravoid;

	uint8_t phy_pabias;
	uint8_t nphy_papd_skip;
	uint8_t nphy_tssi_slope;

	int16_t nphy_noise_win[PHY_CORE_MAX][PHY_NOISE_WINDOW_SZ];
	uint8_t nphy_noise_index;

	bool nphy_gain_boost;
	bool nphy_elna_gain_config;
	uint16_t old_bphy_test;
	uint16_t old_bphy_testcontrol;

	bool phyhang_avoid;

	bool rssical_nphy;
	uint8_t nphy_perical;
	uint nphy_perical_last;
	uint8_t cal_type_override;
	uint8_t mphase_cal_phase_id;
	uint8_t mphase_txcal_cmdidx;
	uint8_t mphase_txcal_numcmds;
	uint16_t mphase_txcal_bestcoeffs[11];
	uint16_t nphy_txiqlocal_chanspec;
	uint16_t nphy_iqcal_chanspec_2G;
	uint16_t nphy_iqcal_chanspec_5G;
	uint16_t nphy_rssical_chanspec_2G;
	uint16_t nphy_rssical_chanspec_5G;
	struct wlapi_timer *phycal_timer;
	bool use_int_tx_iqlo_cal_nphy;
	bool internal_tx_iqlo_cal_tapoff_intpa_nphy;
	int16_t nphy_lastcal_temp;

	struct txiqcal_cache calibration_cache;
	struct rssical_cache rssical_cache;

	uint8_t nphy_txpwr_idx[2];
	uint8_t nphy_papd_cal_type;
	uint nphy_papd_last_cal;
	uint16_t nphy_papd_tx_gain_at_last_cal[2];
	uint8_t nphy_papd_cal_gain_index[2];
	int16_t nphy_papd_epsilon_offset[2];
	bool nphy_papd_recal_enable;
	uint32_t nphy_papd_recal_counter;
	bool nphy_force_papd_cal;
	bool nphy_papdcomp;
	bool ipa2g_on;
	bool ipa5g_on;

	uint16_t classifier_state;
	uint16_t clip_state[2];
	uint nphy_deaf_count;
	uint8_t rxiq_samps;
	uint8_t rxiq_antsel;

	uint16_t rfctrlIntc1_save;
	uint16_t rfctrlIntc2_save;
	bool first_cal_after_assoc;
	uint16_t tx_rx_cal_radio_saveregs[22];
	uint16_t tx_rx_cal_phy_saveregs[15];

	uint8_t nphy_cal_orig_pwr_idx[2];
	uint8_t nphy_txcal_pwr_idx[2];
	uint8_t nphy_rxcal_pwr_idx[2];
	uint16_t nphy_cal_orig_tx_gain[2];
	struct nphy_txgains nphy_cal_target_gain;
	uint16_t nphy_txcal_bbmult;
	uint16_t nphy_gmval;

	uint16_t nphy_saved_bbconf;

	bool nphy_gband_spurwar_en;
	bool nphy_gband_spurwar2_en;
	bool nphy_aband_spurwar_en;
	uint16_t nphy_rccal_value;
	uint16_t nphy_crsminpwr[3];
	struct nphy_noisevar_buf nphy_saved_noisevars;
	bool nphy_anarxlpf_adjusted;
	bool nphy_crsminpwr_adjusted;
	bool nphy_noisevars_adjusted;

	bool nphy_rxcal_active;
	uint16_t radar_percal_mask;
	bool dfs_lp_buffer_nphy;

	uint16_t nphy_fineclockgatecontrol;

	int8_t rx2tx_biasentry;

	uint16_t crsminpwr0;
	uint16_t crsminpwrl0;
	uint16_t crsminpwru0;
	int16_t noise_crsminpwr_index;
	uint16_t init_gain_core1;
	uint16_t init_gain_core2;
	uint16_t init_gainb_core1;
	uint16_t init_gainb_core2;
	uint8_t aci_noise_curr_channel;
	uint16_t init_gain_rfseq[4];

	bool radio_is_on;

	bool nphy_sample_play_lpf_bw_ctl_ovr;

	uint16_t tbl_data_hi;
	uint16_t tbl_data_lo;
	uint16_t tbl_addr;

	uint tbl_save_id;
	uint tbl_save_offset;

	uint8_t txpwrctrl;
	int8_t txpwrindex[PHY_CORE_MAX];

	uint8_t phycal_tempdelta;
	uint32_t mcs20_po;
	uint32_t mcs40_po;
	struct wiphy *wiphy;
};

struct cint32_t {
	int32_t q;
	int32_t i;
};

struct radio_regs {
	uint16_t address;
	uint32_t init_a;
	uint32_t init_g;
	uint8_t do_init_a;
	uint8_t do_init_g;
};

struct radio_20xx_regs {
	uint16_t address;
	uint8_t init;
	uint8_t do_init;
};

struct lcnphy_radio_regs {
	uint16_t address;
	uint8_t init_a;
	uint8_t init_g;
	uint8_t do_init_a;
	uint8_t do_init_g;
};

uint16_t read_phy_reg(struct brcms_phy *pi, uint16_t addr);
void write_phy_reg(struct brcms_phy *pi, uint16_t addr, uint16_t val);
void and_phy_reg(struct brcms_phy *pi, uint16_t addr, uint16_t val);
void or_phy_reg(struct brcms_phy *pi, uint16_t addr, uint16_t val);
void mod_phy_reg(struct brcms_phy *pi, uint16_t addr, uint16_t mask, uint16_t val);

uint16_t read_radio_reg(struct brcms_phy *pi, uint16_t addr);
void or_radio_reg(struct brcms_phy *pi, uint16_t addr, uint16_t val);
void and_radio_reg(struct brcms_phy *pi, uint16_t addr, uint16_t val);
void mod_radio_reg(struct brcms_phy *pi, uint16_t addr, uint16_t mask, uint16_t val);
void xor_radio_reg(struct brcms_phy *pi, uint16_t addr, uint16_t mask);

void write_radio_reg(struct brcms_phy *pi, uint16_t addr, uint16_t val);

void wlc_phyreg_enter(struct brcms_phy_pub *pih);
void wlc_phyreg_exit(struct brcms_phy_pub *pih);
void wlc_radioreg_enter(struct brcms_phy_pub *pih);
void wlc_radioreg_exit(struct brcms_phy_pub *pih);

void wlc_phy_read_table(struct brcms_phy *pi,
			const struct phytbl_info *ptbl_info,
			uint16_t tblAddr, uint16_t tblDataHi, uint16_t tblDatalo);
void wlc_phy_write_table(struct brcms_phy *pi,
			 const struct phytbl_info *ptbl_info,
			 uint16_t tblAddr, uint16_t tblDataHi, uint16_t tblDatalo);
void wlc_phy_table_addr(struct brcms_phy *pi, uint tbl_id, uint tbl_offset,
			uint16_t tblAddr, uint16_t tblDataHi, uint16_t tblDataLo);
void wlc_phy_table_data_write(struct brcms_phy *pi, uint width, uint32_t val);

void write_phy_channel_reg(struct brcms_phy *pi, uint val);
void wlc_phy_txpower_update_shm(struct brcms_phy *pi);

uint8_t wlc_phy_nbits(int32_t value);
void wlc_phy_compute_dB(uint32_t *cmplx_pwr, int8_t *p_dB, uint8_t core);

uint wlc_phy_init_radio_regs_allbands(struct brcms_phy *pi,
				      struct radio_20xx_regs *radioregs);
uint wlc_phy_init_radio_regs(struct brcms_phy *pi,
			     const struct radio_regs *radioregs,
			     uint16_t core_offset);

void wlc_phy_txpower_ipa_upd(struct brcms_phy *pi);

void wlc_phy_do_dummy_tx(struct brcms_phy *pi, bool ofdm, bool pa_on);
void wlc_phy_papd_decode_epsilon(uint32_t epsilon, int32_t *eps_real, int32_t *eps_imag);

void wlc_phy_cal_perical_mphase_reset(struct brcms_phy *pi);
void wlc_phy_cal_perical_mphase_restart(struct brcms_phy *pi);

bool wlc_phy_attach_nphy(struct brcms_phy *pi);
bool wlc_phy_attach_lcnphy(struct brcms_phy *pi);

void wlc_phy_detach_lcnphy(struct brcms_phy *pi);

void wlc_phy_init_nphy(struct brcms_phy *pi);
void wlc_phy_init_lcnphy(struct brcms_phy *pi);

void wlc_phy_cal_init_nphy(struct brcms_phy *pi);
void wlc_phy_cal_init_lcnphy(struct brcms_phy *pi);

void wlc_phy_chanspec_set_nphy(struct brcms_phy *pi, uint16_t chanspec);
void wlc_phy_chanspec_set_lcnphy(struct brcms_phy *pi, uint16_t chanspec);
void wlc_phy_chanspec_set_fixup_lcnphy(struct brcms_phy *pi, uint16_t chanspec);
int wlc_phy_channel2freq(uint channel);
int wlc_phy_chanspec_freq2bandrange_lpssn(uint);
int wlc_phy_chanspec_bandrange_get(struct brcms_phy *, uint16_t chanspec);

void wlc_lcnphy_set_tx_pwr_ctrl(struct brcms_phy *pi, uint16_t mode);
int8_t wlc_lcnphy_get_current_tx_pwr_idx(struct brcms_phy *pi);

void wlc_phy_txpower_recalc_target_nphy(struct brcms_phy *pi);
void wlc_lcnphy_txpower_recalc_target(struct brcms_phy *pi);
void wlc_phy_txpower_recalc_target_lcnphy(struct brcms_phy *pi);

void wlc_lcnphy_set_tx_pwr_by_index(struct brcms_phy *pi, int index);
void wlc_lcnphy_tx_pu(struct brcms_phy *pi, bool bEnable);
void wlc_lcnphy_stop_tx_tone(struct brcms_phy *pi);
void wlc_lcnphy_start_tx_tone(struct brcms_phy *pi, int32_t f_kHz, uint16_t max_val,
			      bool iqcalmode);

void wlc_phy_txpower_sromlimit_get_nphy(struct brcms_phy *pi, uint chan,
					uint8_t *max_pwr, uint8_t rate_id);
void wlc_phy_ofdm_to_mcs_powers_nphy(uint8_t *power, uint8_t rate_mcs_start,
				     uint8_t rate_mcs_end, uint8_t rate_ofdm_start);
void wlc_phy_mcs_to_ofdm_powers_nphy(uint8_t *power, uint8_t rate_ofdm_start,
				     uint8_t rate_ofdm_end, uint8_t rate_mcs_start);

uint16_t wlc_lcnphy_tempsense(struct brcms_phy *pi, bool mode);
int16_t wlc_lcnphy_tempsense_new(struct brcms_phy *pi, bool mode);
int8_t wlc_lcnphy_tempsense_degree(struct brcms_phy *pi, bool mode);
int8_t wlc_lcnphy_vbatsense(struct brcms_phy *pi, bool mode);
void wlc_phy_carrier_suppress_lcnphy(struct brcms_phy *pi);
void wlc_lcnphy_crsuprs(struct brcms_phy *pi, int channel);
void wlc_lcnphy_epa_switch(struct brcms_phy *pi, bool mode);
void wlc_2064_vco_cal(struct brcms_phy *pi);

void wlc_phy_txpower_recalc_target(struct brcms_phy *pi);

#define LCNPHY_TBL_ID_PAPDCOMPDELTATBL	0x18
#define LCNPHY_TX_POWER_TABLE_SIZE	128
#define LCNPHY_MAX_TX_POWER_INDEX	(LCNPHY_TX_POWER_TABLE_SIZE - 1)
#define LCNPHY_TBL_ID_TXPWRCTL	0x07
#define LCNPHY_TX_PWR_CTRL_OFF	0
#define LCNPHY_TX_PWR_CTRL_SW		(0x1 << 15)
#define LCNPHY_TX_PWR_CTRL_HW         ((0x1 << 15) | \
					(0x1 << 14) | \
					(0x1 << 13))

#define LCNPHY_TX_PWR_CTRL_TEMPBASED	0xE001

void wlc_lcnphy_write_table(struct brcms_phy *pi,
			    const struct phytbl_info *pti);
void wlc_lcnphy_read_table(struct brcms_phy *pi, struct phytbl_info *pti);
void wlc_lcnphy_set_tx_iqcc(struct brcms_phy *pi, uint16_t a, uint16_t b);
void wlc_lcnphy_set_tx_locc(struct brcms_phy *pi, uint16_t didq);
void wlc_lcnphy_get_tx_iqcc(struct brcms_phy *pi, uint16_t *a, uint16_t *b);
uint16_t wlc_lcnphy_get_tx_locc(struct brcms_phy *pi);
void wlc_lcnphy_get_radio_loft(struct brcms_phy *pi, uint8_t *ei0, uint8_t *eq0, uint8_t *fi0,
			       uint8_t *fq0);
void wlc_lcnphy_calib_modes(struct brcms_phy *pi, uint mode);
void wlc_lcnphy_deaf_mode(struct brcms_phy *pi, bool mode);
bool wlc_phy_tpc_isenabled_lcnphy(struct brcms_phy *pi);
void wlc_lcnphy_tx_pwr_update_npt(struct brcms_phy *pi);
int32_t wlc_lcnphy_tssi2dbm(int32_t tssi, int32_t a1, int32_t b0, int32_t b1);
void wlc_lcnphy_get_tssi(struct brcms_phy *pi, int8_t *ofdm_pwr, int8_t *cck_pwr);
void wlc_lcnphy_tx_power_adjustment(struct brcms_phy_pub *ppi);

int32_t wlc_lcnphy_rx_signal_power(struct brcms_phy *pi, int32_t gain_index);

#define NPHY_MAX_HPVGA1_INDEX		10
#define NPHY_DEF_HPVGA1_INDEXLIMIT	7

struct phy_iq_est {
	int32_t iq_prod;
	uint32_t i_pwr;
	uint32_t q_pwr;
};

void wlc_phy_stay_in_carriersearch_nphy(struct brcms_phy *pi, bool enable);
void wlc_nphy_deaf_mode(struct brcms_phy *pi, bool mode);

#define wlc_phy_write_table_nphy(pi, pti) \
	wlc_phy_write_table(pi, pti, 0x72, 0x74, 0x73)

#define wlc_phy_read_table_nphy(pi, pti) \
	wlc_phy_read_table(pi, pti, 0x72, 0x74, 0x73)

#define wlc_nphy_table_addr(pi, id, off) \
	wlc_phy_table_addr((pi), (id), (off), 0x72, 0x74, 0x73)

#define wlc_nphy_table_data_write(pi, w, v) \
	wlc_phy_table_data_write((pi), (w), (v))

void wlc_phy_table_read_nphy(struct brcms_phy *pi, uint32_t, uint32_t l, uint32_t o, uint32_t w,
			     void *d);
void wlc_phy_table_write_nphy(struct brcms_phy *pi, uint32_t, uint32_t, uint32_t, uint32_t,
			      const void *);

#define	PHY_IPA(pi) \
	((pi->ipa2g_on && CHSPEC_IS2G(pi->radio_chanspec)) || \
	 (pi->ipa5g_on && CHSPEC_IS5G(pi->radio_chanspec)))

#define BRCMS_PHY_WAR_PR51571(pi) \
	if (NREV_LT((pi)->pubpi.phy_rev, 3)) \
		(void)bcma_read32(pi->d11core, D11REGOFFS(maccontrol))

void wlc_phy_cal_perical_nphy_run(struct brcms_phy *pi, uint8_t caltype);
void wlc_phy_aci_reset_nphy(struct brcms_phy *pi);
void wlc_phy_pa_override_nphy(struct brcms_phy *pi, bool en);

uint8_t wlc_phy_get_chan_freq_range_nphy(struct brcms_phy *pi, uint chan);
void wlc_phy_switch_radio_nphy(struct brcms_phy *pi, bool on);

void wlc_phy_stf_chain_upd_nphy(struct brcms_phy *pi);

void wlc_phy_force_rfseq_nphy(struct brcms_phy *pi, uint8_t cmd);
int16_t wlc_phy_tempsense_nphy(struct brcms_phy *pi);

uint16_t wlc_phy_classifier_nphy(struct brcms_phy *pi, uint16_t mask, uint16_t val);

void wlc_phy_rx_iq_est_nphy(struct brcms_phy *pi, struct phy_iq_est *est,
			    uint16_t num_samps, uint8_t wait_time, uint8_t wait_for_crs);

void wlc_phy_rx_iq_coeffs_nphy(struct brcms_phy *pi, uint8_t write,
			       struct nphy_iq_comp *comp);
void wlc_phy_aci_and_noise_reduction_nphy(struct brcms_phy *pi);

void wlc_phy_rxcore_setstate_nphy(struct brcms_phy_pub *pih, uint8_t rxcore_bitmask);
uint8_t wlc_phy_rxcore_getstate_nphy(struct brcms_phy_pub *pih);

void wlc_phy_txpwrctrl_enable_nphy(struct brcms_phy *pi, uint8_t ctrl_type);
void wlc_phy_txpwr_fixpower_nphy(struct brcms_phy *pi);
void wlc_phy_txpwr_apply_nphy(struct brcms_phy *pi);
void wlc_phy_txpwr_papd_cal_nphy(struct brcms_phy *pi);
uint16_t wlc_phy_txpwr_idx_get_nphy(struct brcms_phy *pi);

struct nphy_txgains wlc_phy_get_tx_gain_nphy(struct brcms_phy *pi);
int wlc_phy_cal_txiqlo_nphy(struct brcms_phy *pi,
			    struct nphy_txgains target_gain, bool full, bool m);
int wlc_phy_cal_rxiq_nphy(struct brcms_phy *pi, struct nphy_txgains target_gain,
			  uint8_t type, bool d);
void wlc_phy_txpwr_index_nphy(struct brcms_phy *pi, uint8_t core_mask,
			      int8_t txpwrindex, bool res);
void wlc_phy_rssisel_nphy(struct brcms_phy *pi, uint8_t core, uint8_t rssi_type);
int wlc_phy_poll_rssi_nphy(struct brcms_phy *pi, uint8_t rssi_type,
			   int32_t *rssi_buf, uint8_t nsamps);
void wlc_phy_rssi_cal_nphy(struct brcms_phy *pi);
int wlc_phy_aci_scan_nphy(struct brcms_phy *pi);
void wlc_phy_cal_txgainctrl_nphy(struct brcms_phy *pi, int32_t dBm_targetpower,
				 bool debug);
int wlc_phy_tx_tone_nphy(struct brcms_phy *pi, uint32_t f_kHz, uint16_t max_val, uint8_t mode,
			 uint8_t, bool);
void wlc_phy_stopplayback_nphy(struct brcms_phy *pi);
void wlc_phy_est_tonepwr_nphy(struct brcms_phy *pi, int32_t *qdBm_pwrbuf,
			      uint8_t num_samps);
void wlc_phy_radio205x_vcocal_nphy(struct brcms_phy *pi);

int wlc_phy_rssi_compute_nphy(struct brcms_phy *pi, struct d11rxhdr *rxh);

#define NPHY_TESTPATTERN_BPHY_EVM   0
#define NPHY_TESTPATTERN_BPHY_RFCS  1

void wlc_phy_nphy_tkip_rifs_war(struct brcms_phy *pi, uint8_t rifs);

void wlc_phy_get_pwrdet_offsets(struct brcms_phy *pi, int8_t *cckoffset,
				int8_t *ofdmoffset);
int8_t wlc_phy_upd_rssi_offset(struct brcms_phy *pi, int8_t rssi, uint16_t chanspec);

bool wlc_phy_n_txpower_ipa_ison(struct brcms_phy *pih);
#endif				/* _BRCM_PHY_INT_H_ */
