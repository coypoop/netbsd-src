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

#ifndef _BRCM_PHY_LCN_H_
#define _BRCM_PHY_LCN_H_

#include <types.h>

struct brcms_phy_lcnphy {
	int lcnphy_txrf_sp_9_override;
	uint8_t lcnphy_full_cal_channel;
	uint8_t lcnphy_cal_counter;
	uint16_t lcnphy_cal_temper;
	bool lcnphy_recal;

	uint8_t lcnphy_rc_cap;
	uint32_t lcnphy_mcs20_po;

	uint8_t lcnphy_tr_isolation_mid;
	uint8_t lcnphy_tr_isolation_low;
	uint8_t lcnphy_tr_isolation_hi;

	uint8_t lcnphy_bx_arch;
	uint8_t lcnphy_rx_power_offset;
	uint8_t lcnphy_rssi_vf;
	uint8_t lcnphy_rssi_vc;
	uint8_t lcnphy_rssi_gs;
	uint8_t lcnphy_tssi_val;
	uint8_t lcnphy_rssi_vf_lowtemp;
	uint8_t lcnphy_rssi_vc_lowtemp;
	uint8_t lcnphy_rssi_gs_lowtemp;

	uint8_t lcnphy_rssi_vf_hightemp;
	uint8_t lcnphy_rssi_vc_hightemp;
	uint8_t lcnphy_rssi_gs_hightemp;

	int16_t lcnphy_pa0b0;
	int16_t lcnphy_pa0b1;
	int16_t lcnphy_pa0b2;

	uint16_t lcnphy_rawtempsense;
	uint8_t lcnphy_measPower;
	uint8_t lcnphy_tempsense_slope;
	uint8_t lcnphy_freqoffset_corr;
	uint8_t lcnphy_tempsense_option;
	uint8_t lcnphy_tempcorrx;
	bool lcnphy_iqcal_swp_dis;
	bool lcnphy_hw_iqcal_en;
	uint lcnphy_bandedge_corr;
	bool lcnphy_spurmod;
	uint16_t lcnphy_tssi_tx_cnt;
	uint16_t lcnphy_tssi_idx;
	uint16_t lcnphy_tssi_npt;

	uint16_t lcnphy_target_tx_freq;
	int8_t lcnphy_tx_power_idx_override;
	uint16_t lcnphy_noise_samples;

	uint32_t lcnphy_papdRxGnIdx;
	uint32_t lcnphy_papd_rxGnCtrl_init;

	uint32_t lcnphy_gain_idx_14_lowword;
	uint32_t lcnphy_gain_idx_14_hiword;
	uint32_t lcnphy_gain_idx_27_lowword;
	uint32_t lcnphy_gain_idx_27_hiword;
	int16_t lcnphy_ofdmgainidxtableoffset;
	int16_t lcnphy_dsssgainidxtableoffset;
	uint32_t lcnphy_tr_R_gain_val;
	uint32_t lcnphy_tr_T_gain_val;
	int8_t lcnphy_input_pwr_offset_db;
	uint16_t lcnphy_Med_Low_Gain_db;
	uint16_t lcnphy_Very_Low_Gain_db;
	int8_t lcnphy_lastsensed_temperature;
	int8_t lcnphy_pkteng_rssi_slope;
	uint8_t lcnphy_saved_tx_user_target[TXP_NUM_RATES];
	uint8_t lcnphy_volt_winner;
	uint8_t lcnphy_volt_low;
	uint8_t lcnphy_54_48_36_24mbps_backoff;
	uint8_t lcnphy_11n_backoff;
	uint8_t lcnphy_lowerofdm;
	uint8_t lcnphy_cck;
	uint8_t lcnphy_psat_2pt3_detected;
	int32_t lcnphy_lowest_Re_div_Im;
	int8_t lcnphy_final_papd_cal_idx;
	uint16_t lcnphy_extstxctrl4;
	uint16_t lcnphy_extstxctrl0;
	uint16_t lcnphy_extstxctrl1;
	int16_t lcnphy_cck_dig_filt_type;
	int16_t lcnphy_ofdm_dig_filt_type;
	struct lcnphy_cal_results lcnphy_cal_results;

	uint8_t lcnphy_psat_pwr;
	uint8_t lcnphy_psat_indx;
	int32_t lcnphy_min_phase;
	uint8_t lcnphy_final_idx;
	uint8_t lcnphy_start_idx;
	uint8_t lcnphy_current_index;
	uint16_t lcnphy_logen_buf_1;
	uint16_t lcnphy_local_ovr_2;
	uint16_t lcnphy_local_oval_6;
	uint16_t lcnphy_local_oval_5;
	uint16_t lcnphy_logen_mixer_1;

	uint8_t lcnphy_aci_stat;
	uint lcnphy_aci_start_time;
	int8_t lcnphy_tx_power_offset[TXP_NUM_RATES];
};
#endif				/* _BRCM_PHY_LCN_H_ */
