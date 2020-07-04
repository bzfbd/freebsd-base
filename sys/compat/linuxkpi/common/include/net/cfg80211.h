/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 The FreeBSD Foundation
 *
 * This software was developed by Björn Zeeb under sponsorship from
 * the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	_NET_CFG80211_H
#define	_NET_CFG80211_H

#include <linux/nl80211.h>
#include <linux/ieee80211.h>
#include <linux/if_ether.h>

#define	IEEE80211_MAX_CHAINS	4		/* net80211: IEEE80211_MAX_CHAINS copied */

/*
 * Oh dear ... based on the filters in iwlwifi this is 6 long
 * as the filter follows with ether_type.
 * So I believe this is DSAP | SSAP | CTRL | ProtoID/OrgCode{3}.
 */
extern const uint8_t rfc1042_header[6];

enum ieee80211_channel_flags {
	IEEE80211_CHAN_DISABLED,
	IEEE80211_CHAN_INDOOR_ONLY,
	IEEE80211_CHAN_IR_CONCURRENT,
	IEEE80211_CHAN_RADAR,
	IEEE80211_CHAN_NO_IR,
	IEEE80211_CHAN_NO_HT40MINUS,
	IEEE80211_CHAN_NO_HT40PLUS,
	IEEE80211_CHAN_NO_80MHZ,
	IEEE80211_CHAN_NO_160MHZ,
};
#define	IEEE80211_CHAN_NO_HT40	(IEEE80211_CHAN_NO_HT40MINUS|IEEE80211_CHAN_NO_HT40PLUS)

/* XXX net80211 has an ieee80211_channel as well. */
struct ieee80211_channel {
	/* TODO FIXME */
	uint32_t				hw_value;	/* ic_ieee */
	uint32_t				center_freq;	/* ic_freq */
	enum ieee80211_channel_flags		flags;		/* ic_flags */
	enum nl80211_band			band;
	int8_t					max_power;	/* ic_maxpower */
	int     beacon_found, max_antenna_gain, max_reg_power;
};

struct cfg80211_bitrate_mask {
	/* TODO FIXME */
	/* This is so weird but nothing else works out...*/
	struct {
		uint64_t	legacy;		/* XXX? */
		uint8_t		ht_mcs[16];	/* XXX? */
		uint16_t	vht_mcs[16];	/* XXX? */
		uint8_t		gi;		/* NL80211_TXRATE_FORCE_LGI enum? */
	} control[NUM_NL80211_BANDS];
};

struct rate_info {
	/* TODO FIXME */
	int	bw, flags, he_dcm, he_gi, he_ru_alloc, legacy, mcs, nss;
};

struct ieee80211_rate {
	/* TODO FIXME */
	uint32_t		bitrate;
	uint32_t		hw_value;
	uint32_t		hw_value_short;
	uint32_t		flags;
};

/* XXX net80211 calls these IEEE80211_HTCAP_* */
#define	IEEE80211_HT_CAP_LDPC_CODING		0x0001	/* IEEE80211_HTCAP_LDPC */
#define	IEEE80211_HT_CAP_SUP_WIDTH_20_40	0x0002	/* IEEE80211_HTCAP_CHWIDTH40 */
#define	IEEE80211_HT_CAP_GRN_FLD		0x0010	/* IEEE80211_HTCAP_GREENFIELD */
#define	IEEE80211_HT_CAP_SGI_20			0x0020	/* IEEE80211_HTCAP_SHORTGI20 */
#define	IEEE80211_HT_CAP_SGI_40			0x0040	/* IEEE80211_HTCAP_SHORTGI40 */
#define	IEEE80211_HT_CAP_TX_STBC		0x0080	/* IEEE80211_HTCAP_TXSTBC */
#define	IEEE80211_HT_CAP_RX_STBC		0x0100	/* IEEE80211_HTCAP_RXSTBC */
#define	IEEE80211_HT_CAP_RX_STBC_SHIFT		8	/* IEEE80211_HTCAP_RXSTBC_S */
#define	IEEE80211_HT_CAP_MAX_AMSDU		0x0800	/* IEEE80211_HTCAP_MAXAMSDU */
#define	IEEE80211_HT_CAP_DSSSCCK40		0x1000	/* IEEE80211_HTCAP_DSSSCCK40 */

#define	IEEE80211_HT_MCS_TX_DEFINED		0x0001
#define	IEEE80211_HT_MCS_TX_RX_DIFF		0x0002
#define	IEEE80211_HT_MCS_TX_MAX_STREAMS_SHIFT	2
#define	IEEE80211_HT_MCS_RX_HIGHEST_MASK	0x3FF

struct ieee80211_sta_ht_cap {
		/* TODO FIXME */
	int	ampdu_density, ampdu_factor;
	int		ht_supported;
	uint32_t	cap;
	struct mcs {
		uint16_t	rx_mask[16];	/* XXX ? > 4 (rtw88) */
		int		rx_highest;
		uint32_t	tx_params;
	} mcs;
};

/* XXX net80211 calls these IEEE80211_VHTCAP_* */
#define	IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_3895	0x00000000	/* IEEE80211_VHTCAP_MAX_MPDU_LENGTH_3895 */
#define	IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_7991	0x00000001	/* IEEE80211_VHTCAP_MAX_MPDU_LENGTH_7991 */
#define	IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454	0x00000002	/* IEEE80211_VHTCAP_MAX_MPDU_LENGTH_11454 */
#define	IEEE80211_VHT_CAP_MAX_MPDU_MASK		0x00000003	/* IEEE80211_VHTCAP_MAX_MPDU_MASK */

#define	IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_160MHZ	1	/* IEEE80211_VHTCAP_SUPP_CHAN_WIDTH_160MHZ */

#define	IEEE80211_VHT_CAP_RXLDPC		0x00000010	/* IEEE80211_VHTCAP_RXLDPC */

#define	IEEE80211_VHT_CAP_SHORT_GI_80		0x00000020	/* IEEE80211_VHTCAP_SHORT_GI_80 */
#define	IEEE80211_VHT_CAP_SHORT_GI_160		0x00000040	/* IEEE80211_VHTCAP_SHORT_GI_160 */

#define	IEEE80211_VHT_CAP_TXSTBC		0x00000080	/* IEEE80211_VHTCAP_TXSTBC */

#define	IEEE80211_VHT_CAP_RXSTBC_1		0x00000100	/* IEEE80211_VHTCAP_RXSTBC_1 */
#define	IEEE80211_VHT_CAP_RXSTBC_MASK		0x00000700	/* IEEE80211_VHTCAP_RXSTBC_MASK */

#define	IEEE80211_VHT_CAP_SU_BEAMFORMER_CAPABLE	0x00000800	/* IEEE80211_VHTCAP_SU_BEAMFORMER_CAPABLE */

#define	IEEE80211_VHT_CAP_SU_BEAMFORMEE_CAPABLE	0x00001000	/* IEEE80211_VHTCAP_SU_BEAMFORMEE_CAPABLE */

#define	IEEE80211_VHT_CAP_BEAMFORMEE_STS_SHIFT		13	/* IEEE80211_VHTCAP_BEAMFORMEE_STS_SHIFT */

#define	IEEE80211_VHT_CAP_MU_BEAMFORMEE_CAPABLE	0x00100000	/* IEEE80211_VHTCAP_MU_BEAMFORMEE_CAPABLE */

#define	IEEE80211_VHT_CAP_RX_ANTENNA_PATTERN	0x10000000	/* IEEE80211_VHTCAP_RX_ANTENNA_PATTERN */
#define	IEEE80211_VHT_CAP_TX_ANTENNA_PATTERN	0x20000000	/* IEEE80211_VHTCAP_TX_ANTENNA_PATTERN */

#define	IEEE80211_VHT_CAP_MU_BEAMFORMER_CAPABLE	0x00080000	/* IEEE80211_VHTCAP_MU_BEAMFORMER_CAPABLE */

#define	IEEE80211_VHT_CAP_SOUNDING_DIMENSIONS_SHIFT	16	/* IEEE80211_VHTCAP_SOUNDING_DIMENSIONS_SHIFT */
#define	IEEE80211_VHT_CAP_SOUNDING_DIMENSIONS_MASK		\
	(7 << IEEE80211_VHTCAP_SOUNDING_DIMENSIONS_SHIFT)	/* IEEE80211_VHTCAP_SOUNDING_DIMENSIONS_MASK */

#define	IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT	23	/* IEEE80211_VHTCAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT */
#define	IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK	\
	(7 << IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT)	/* IEEE80211_VHTCAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK */

struct ieee80211_sta_vht_cap {
		/* TODO FIXME */
	int	cap, vht_supported;
	struct vht_mcs {
		int		rx_mcs_map;
		int		tx_mcs_map;
		int		tx_highest;
		int		rx_highest;
	} vht_mcs;
};

struct cfg80211_bss_ies {
		/* XXX TODO, type yb best guess. Fix if more info. */
	uint8_t				*data;
	int				len;
};

struct cfg80211_bss {
		/* XXX TODO */
	struct cfg80211_bss_ies		*ies;
};

struct cfg80211_chan_def {
		/* XXX TODO */
	struct ieee80211_channel	*chan;
	enum nl80211_chan_width		width;
	uint32_t			center_freq1;
	uint32_t			center_freq2;
};

struct cfg80211_ftm_responder_stats {
		/* XXX TODO */
	int	asap_num, failed_num, filled, non_asap_num, out_of_window_triggers_num, partial_num, reschedule_requests_num, success_num, total_duration_ms, unknown_triggers_num;
};

struct cfg80211_pmsr_capabilities {
		/* XXX TODO */
	int	max_peers, randomize_mac_addr, report_ap_tsf;
	struct {
		int	 asap, bandwidths, max_bursts_exponent, max_ftms_per_burst, non_asap, non_trigger_based, preambles, request_civicloc, request_lci, supported, trigger_based;
	} ftm;
};

struct cfg80211_pmsr_ftm_request {
		/* XXX TODO */
	int     asap, burst_period, ftmr_retries, ftms_per_burst, non_trigger_based, num_bursts_exp, request_civicloc, request_lci, trigger_based;
};

struct cfg80211_pmsr_request_peer {
		/* XXX TODO */
	struct cfg80211_chan_def		chandef;
	struct cfg80211_pmsr_ftm_request	ftm;
	uint8_t					addr[ETH_ALEN];
	int	report_ap_tsf;
};

struct cfg80211_pmsr_request {
		/* XXX TODO */
	int	cookie, n_peers, timeout;
	uint8_t					mac_addr[ETH_ALEN], mac_addr_mask[ETH_ALEN];
	struct cfg80211_pmsr_request_peer	peers[];
};

struct cfg80211_pmsr_ftm_result {
		/* XXX TODO */
	int	burst_index, busy_retry_time, failure_reason;
	int	num_ftmr_successes, rssi_avg, rssi_avg_valid, rssi_spread, rssi_spread_valid, rtt_avg, rtt_avg_valid, rtt_spread, rtt_spread_valid, rtt_variance, rtt_variance_valid;
	uint8_t					*lci;
	uint8_t					*civicloc;
	int					lci_len;
	int					civicloc_len;
};

struct cfg80211_pmsr_result {
		/* XXX TODO */
	int	ap_tsf, ap_tsf_valid, final, host_time, status, type;
	uint8_t					addr[ETH_ALEN];
	struct cfg80211_pmsr_ftm_result		ftm;
};

struct cfg80211_ssid {
	int	ssid_len;
	uint8_t	ssid[IEEE80211_MAX_SSID_LEN];
};

struct cfg80211_match_set {
	struct cfg80211_ssid	ssid;
	int			rssi_thold;
};

struct cfg80211_scan_request {
		/* XXX TODO */
	int	duration, duration_mandatory, flags;
	int		no_cck;
	int					ie_len;
	uint8_t					*ie;
	uint8_t					mac_addr[ETH_ALEN], mac_addr_mask[ETH_ALEN];
	int					n_ssids;
	int					n_channels;
	struct cfg80211_ssid			*ssids;
	struct ieee80211_channel		*channels[0];
};

struct cfg80211_sched_scan_plan {
		/* XXX TODO */
	int	interval, iterations;
};

struct cfg80211_sched_scan_request {
		/* XXX TODO */
	int	delay, flags;
	uint8_t					mac_addr[ETH_ALEN], mac_addr_mask[ETH_ALEN];
	int					n_match_sets;
	int					n_scan_plans;
	int					n_ssids;
	int					n_channels;
	struct cfg80211_match_set		*match_sets;
	struct cfg80211_sched_scan_plan		*scan_plans;
	struct cfg80211_ssid			*ssids;
	struct ieee80211_channel		*channels[0];
};

struct cfg80211_scan_info {
	int	aborted, scan_start_tsf;
	uint8_t					tsf_bssid[ETH_ALEN];
};

/* That the world needs so many different structs for this is amazing. */
struct mac_address {
	uint8_t	addr[ETH_ALEN];
};

struct ieee80211_reg_rule {
	/* TODO FIXME */
	int		flags;
	struct power_rule {
		int	max_antenna_gain;
		int	max_eirp;
	} power_rule;
	struct freq_range {
		int	start_freq_khz;
		int	end_freq_khz;
		int	max_bandwidth_khz;
	} freq_range;
};

struct ieee80211_regdomain {
	/* TODO FIXME */
	uint8_t					alpha2[4];	/* XXX */
	int					n_reg_rules;
	struct ieee80211_reg_rule		reg_rules[];
};

/* XXX-BZ this are insensible values probably ... */
#define	IEEE80211_HE_MAC_CAP0_HTC_HE			0x1
#define	IEEE80211_HE_MAC_CAP0_TWT_REQ			0x2

#define	IEEE80211_HE_MAC_CAP1_LINK_ADAPTATION		0x1
#define	IEEE80211_HE_MAC_CAP1_MULTI_TID_AGG_RX_QOS_8	0x2
#define	IEEE80211_HE_MAC_CAP1_TF_MAC_PAD_DUR_16US	0x4

#define	IEEE80211_HE_MAC_CAP2_32BIT_BA_BITMAP		0x1
#define	IEEE80211_HE_MAC_CAP2_ACK_EN			0x2
#define	IEEE80211_HE_MAC_CAP2_BSR			0x4
#define	IEEE80211_HE_MAC_CAP2_LINK_ADAPTATION		0x8

#define	IEEE80211_HE_MAC_CAP3_MAX_AMPDU_LEN_EXP_VHT_2	0x1
#define	IEEE80211_HE_MAC_CAP3_OMI_CONTROL		0x2

#define	IEEE80211_HE_MAC_CAP4_AMDSU_IN_AMPDU		0x1
#define	IEEE80211_HE_MAC_CAP4_BQR			0x2
#define	IEEE80211_HE_MAC_CAP4_MULTI_TID_AGG_TX_QOS_B39	0x4

#define	IEEE80211_HE_MAC_CAP5_HE_DYNAMIC_SM_PS		0x1
#define	IEEE80211_HE_MAC_CAP5_HT_VHT_TRIG_FRAME_RX	0x2
#define	IEEE80211_HE_MAC_CAP5_MULTI_TID_AGG_TX_QOS_B40	0x4
#define	IEEE80211_HE_MAC_CAP5_MULTI_TID_AGG_TX_QOS_B41	0x8
#define	IEEE80211_HE_MAC_CAP5_UL_2x996_TONE_RU		0x10

#define	IEEE80211_HE_MCS_NOT_SUPPORTED			0x0
#define	IEEE80211_HE_MCS_SUPPORT_0_7			0x1
#define	IEEE80211_HE_MCS_SUPPORT_0_9			0x2
#define	IEEE80211_HE_MCS_SUPPORT_0_11			0x4

#define	IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_160MHZ_IN_5G		0x1
#define	IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_80MHZ_IN_5G	0x2
#define	IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_40MHZ_IN_2G		0x4

#define	IEEE80211_HE_PHY_CAP1_DEVICE_CLASS_A		0x1
#define	IEEE80211_HE_PHY_CAP1_LDPC_CODING_IN_PAYLOAD	0x2
#define	IEEE80211_HE_PHY_CAP1_MIDAMBLE_RX_TX_MAX_NSTS	0x4
#define	IEEE80211_HE_PHY_CAP1_PREAMBLE_PUNC_RX_MASK	0x8

#define	IEEE80211_HE_PHY_CAP2_MIDAMBLE_RX_TX_MAX_NSTS	0x1
#define	IEEE80211_HE_PHY_CAP2_NDP_4x_LTF_AND_3_2US	0x2
#define	IEEE80211_HE_PHY_CAP2_STBC_RX_UNDER_80MHZ	0x4

#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_RX_MASK	0x1
#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_RX_NO_DCM	0x2
#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_TX_NO_DCM	0x4
#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_RX_NSS_1		0x8
#define	IEEE80211_HE_PHY_CAP3_DCM_MAX_TX_NSS_1		0x10

#define	IEEE80211_HE_PHY_CAP4_BEAMFORMEE_MAX_STS_UNDER_80MHZ_8	0x1
#define	IEEE80211_HE_PHY_CAP4_BEAMFORMEE_MAX_STS_ABOVE_80MHZ_8	0x2
#define	IEEE80211_HE_PHY_CAP4_SU_BEAMFORMEE			0x4

#define	IEEE80211_HE_PHY_CAP5_BEAMFORMEE_NUM_SND_DIM_ABOVE_80MHZ_2	0x1
#define	IEEE80211_HE_PHY_CAP5_BEAMFORMEE_NUM_SND_DIM_UNDER_80MHZ_2	0x2

#define	IEEE80211_HE_PHY_CAP6_PPE_THRESHOLD_PRESENT	0x1

#define	IEEE80211_HE_PHY_CAP7_HE_SU_MU_PPDU_4XLTF_AND_08_US_GI	0x1
#define	IEEE80211_HE_PHY_CAP7_MAX_NC_1				0x2
#define	IEEE80211_HE_PHY_CAP7_MAX_NC_MASK			0x4
#define	IEEE80211_HE_PHY_CAP7_POWER_BOOST_FACTOR_AR		0x8
#define	IEEE80211_HE_PHY_CAP7_STBC_RX_ABOVE_80MHZ		0x10

#define	IEEE80211_HE_PHY_CAP8_20MHZ_IN_160MHZ_HE_PPDU		0x1
#define	IEEE80211_HE_PHY_CAP8_20MHZ_IN_40MHZ_HE_PPDU_IN_2G	0x2
#define	IEEE80211_HE_PHY_CAP8_80MHZ_IN_160MHZ_HE_PPDU		0x4
#define	IEEE80211_HE_PHY_CAP8_DCM_MAX_RU_2x996			0x8
#define	IEEE80211_HE_PHY_CAP8_HE_ER_SU_PPDU_4XLTF_AND_08_US_GI	0x10

#define	IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_0US		0x1
#define	IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_16US		0x2
#define	IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_8US		0x4
#define	IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_MASK		0x8
#define	IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_RESERVED	0x10
#define	IEEE80211_HE_PHY_CAP9_NON_TRIGGERED_CQI_FEEDBACK	0x20
#define	IEEE80211_HE_PHY_CAP9_RX_FULL_BW_SU_USING_MU_WITH_COMP_SIGB	0x40
#define	IEEE80211_HE_PHY_CAP9_RX_FULL_BW_SU_USING_MU_WITH_NON_COMP_SIGB	0x80


struct ieee80211_he_cap_elem {
	u8 mac_cap_info[6];
	u8 phy_cap_info[10];
} __packed;

struct ieee80211_he_mcs_nss_supp {
	/* TODO FIXME */
	uint32_t	rx_mcs_80;
	uint32_t	tx_mcs_80;
	uint32_t	rx_mcs_160;
	uint32_t	tx_mcs_160;
	uint32_t	rx_mcs_80p80;
	uint32_t	tx_mcs_80p80;
};

#define	IEEE80211_STA_HE_CAP_PPE_THRES_MAX	32
struct ieee80211_sta_he_cap {
	/* TODO FIXME */
	int					has_he;
	struct ieee80211_he_cap_elem		he_cap_elem;
	struct ieee80211_he_mcs_nss_supp	he_mcs_nss_supp;
	uint8_t					ppe_thres[IEEE80211_STA_HE_CAP_PPE_THRES_MAX];
};


struct ieee80211_sband_iftype_data {
	/* TODO FIXME */
	enum nl80211_iftype			types_mask;
	struct ieee80211_sta_he_cap		he_cap;
};

struct ieee80211_supported_band {
	/* TODO FIXME */
	struct ieee80211_channel		*channels;
	struct ieee80211_rate			*bitrates;
	struct ieee80211_sband_iftype_data	*iftype_data;
	int					n_channels;
	int					n_bitrates;
	int					n_iftype_data;
	enum nl80211_band			band;
	struct ieee80211_sta_ht_cap		ht_cap;
	struct ieee80211_sta_vht_cap		vht_cap;
};

struct cfg80211_pkt_pattern {
	/* XXX TODO */
	uint8_t					*mask;
	uint8_t					*pattern;
	int					pattern_len;
	int					pkt_offset;
};

struct cfg80211_wowlan {
	/* XXX TODO */
	int	disconnect, gtk_rekey_failure, magic_pkt;
	int					n_patterns;
	struct cfg80211_sched_scan_request	*nd_config;
	struct cfg80211_pkt_pattern		*patterns;
};

struct cfg80211_gtk_rekey_data {
	/* XXX TODO */
};

struct ieee80211_iface_limit {
	/* TODO FIXME */
	int		max, types;
};

struct ieee80211_iface_combination {
	/* TODO FIXME */
	const struct ieee80211_iface_limit	*limits;
	int					n_limits;
	int		max_interfaces, num_different_channels;
	int		beacon_int_infra_match, beacon_int_min_gcd;
};

struct regulatory_request {
		/* XXX TODO */
	uint8_t					alpha2[2];
	int	initiator, dfs_region;
};

struct wiphy_iftype_ext_capab {
	/* TODO FIXME */
	enum nl80211_iftype			iftype;
	const uint8_t				*extended_capabilities;
	const uint8_t				*extended_capabilities_mask;
	uint8_t					extended_capabilities_len;

};

struct wiphy {
	/* XXX TODO */
	struct ieee80211_supported_band		*bands[NUM_NL80211_BANDS];
	const struct cfg80211_pmsr_capabilities	*pmsr_capa;
	const struct wiphy_iftype_ext_capab	*iftype_ext_capab;
	struct ieee80211_regdomain		*regd;
	char					fw_version[64];		/* XXX TODO */
	const struct ieee80211_iface_combination *iface_combinations;
	uint32_t				*cipher_suites;
	struct mac_address			*addresses;
	int					n_iface_combinations;
	int					n_cipher_suites;
	int					n_addresses;
	void(*reg_notifier)(struct wiphy *, struct regulatory_request *);
	int	available_antennas_rx, available_antennas_tx;
	int	features, flags, hw_version;
	int	 interface_modes, max_match_sets, max_remain_on_channel_duration, max_scan_ie_len, max_scan_ssids, max_sched_scan_ie_len, max_sched_scan_plan_interval, max_sched_scan_plan_iterations, max_sched_scan_plans, max_sched_scan_reqs, max_sched_scan_ssids;
	int	num_iftype_ext_capab;
	int	regulatory_flags;
	int	max_ap_assoc_sta, probe_resp_offload, rts_threshold, software_iftypes, wowlan;
};

struct wireless_dev {
		/* XXX TODO, like ic? */
	int		iftype;
};

static __inline int
reg_query_regdb_wmm(uint8_t *alpha2, uint32_t center_freq,
    struct ieee80211_reg_rule *rule)
{
	/* XXX TODO */
	return (ENXIO);
}

static __inline const u8 *
cfg80211_find_ie_match(uint32_t f, const u8 *ies, size_t ies_len,
    const u8 *match, int x, int y)
{
	/* XXX TODO */
	return (NULL);
}

static __inline const u8 *
cfg80211_find_ie(uint8_t  eid, uint8_t *variable, uint32_t frame_size)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
cfg80211_pmsr_complete(struct wireless_dev *wdev,
    struct cfg80211_pmsr_request *req, gfp_t gfp)
{
	/* XXX TODO */
	return;
}

static __inline void
cfg80211_pmsr_report(struct wireless_dev *wdev,
    struct cfg80211_pmsr_request *req,
    struct cfg80211_pmsr_result *result, gfp_t gfp)
{
	/* XXX TODO */
	return;
}

static __inline void
cfg80211_chandef_create(struct cfg80211_chan_def *chandef,
    struct ieee80211_channel *chan, enum nl80211_chan_flags flag)
{
	/* XXX TODO */
	return;
}

static __inline void
cfg80211_bss_iter(struct wiphy *wiphy, struct cfg80211_chan_def *chandef,
    void (*iterfunc)(struct wiphy *, struct cfg80211_bss *, void *), void *data)
{
	/* XXX TODO */
	return;
}

struct element {
	uint8_t		id;
	uint8_t		datalen;
	uint8_t		data[0];
};

static __inline const struct element *
cfg80211_find_elem(enum ieee80211_eid eid, uint8_t *data, size_t len)
{
	/* XXX TODO */
	return (NULL);
}

static __inline uint32_t
cfg80211_calculate_bitrate(struct rate_info *rate)
{
	/* XXX TODO */
	return (-1);
}

static __inline uint32_t
ieee80211_channel_to_frequency(uint32_t channel, enum nl80211_band band)
{

	/* XXX net80211: ieee80211_ieee2mhz() */

	switch (band) {
	case NL80211_BAND_2GHZ:
		/* XXX TODO */
		break;
	case NL80211_BAND_5GHZ:
		/* XXX TODO */
		break;
	default:
		/* XXX abort, retry, error, panic? */
		break;
	}

	return (0);
}

static __inline uint32_t
ieee80211_frequency_to_channel(uint32_t freq)
{
	/* XXX net80211: ieee80211_chan2ieee() */
	/* XXX TODO */
	return (0);
}

static __inline int
regulatory_set_wiphy_regd_sync_rtnl(struct wiphy *wiphy,
    struct ieee80211_regdomain *regd)
{
	/* XXX TODO */
	return (ENXIO);
}

static __inline void
regulatory_set_wiphy_regd(struct wiphy *wiphy,
    struct ieee80211_regdomain *regd)
{
	/* XXX TODO */
	return;
}

static __inline int
regulatory_hint(struct wiphy *wiphy, uint8_t *alpha2)
{
	/* XXX TODO */
	return (-ENXIO);
}

static __inline struct ieee80211_regdomain *
rtnl_dereference(struct ieee80211_regdomain *regd)
{
	/* XXX TODO */
	return (NULL);
}

static __inline struct ieee80211_reg_rule *
freq_reg_info(struct wiphy *wiphy, uint32_t center_freq)
{
	/* XXX TODO */
	return (NULL);
}

static __inline struct cfg80211_bss *
cfg80211_get_bss(struct wiphy *wiphy, struct ieee80211_channel *chan,
    uint8_t *bssid, void *p, int x, uint32_t f1, uint32_t f2)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
cfg80211_put_bss(struct wiphy *wiphy, struct cfg80211_bss *bss)
{
	/* XXX TODO */
	return;
}

static __inline void
wiphy_apply_custom_regulatory(struct wiphy *wiphy,
    struct ieee80211_regdomain *regd)
{
	/* XXX TODO */
	return;
}

static __inline char *
wiphy_name(struct wiphy *wiphy)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
wiphy_read_of_freq_limits(struct wiphy *wiphy)
{
	/* XXX TODO */
	return;
}

static __inline uint8_t *
cfg80211_find_vendor_ie(unsigned int oui, u8 oui_type,
    uint8_t *data, size_t len)
{
	/* XXX TODO */
	return (NULL);
}

#endif	/* _NET_CFG80211_H */

/* end */
