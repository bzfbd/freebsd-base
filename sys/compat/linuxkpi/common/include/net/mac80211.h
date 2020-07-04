/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 The FreeBSD Foundation
 * Copyright (c) 2020 Bjoern A. Zeeb
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

#ifndef	_NET_MAC80211_H
#define	_NET_MAC80211_H

#include <sys/types.h>

#include <asm/atomic64.h>
#include <linux/bitops.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <net/cfg80211.h>

#define	AMPDU_AGGREGATION		__LINE__ /* XXX TODO */
#define	AP_LINK_PS		__LINE__ /* XXX TODO */
#define	ASSOC_EVENT		__LINE__ /* XXX TODO */
#define	AUTH_EVENT		__LINE__ /* XXX TODO */
#define	BAR_RX_EVENT		__LINE__ /* XXX TODO */
#define	BA_FRAME_TIMEOUT		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_ARP_FILTER		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_ASSOC		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_BANDWIDTH		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_BEACON		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_BEACON_INFO		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_BSSID		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_CQM		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_ERP_CTS_PROT		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_FTM_RESPONDER		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_HT		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_IDLE		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_MU_GROUPS		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_P2P_PS		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_PS		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_QOS		__LINE__ /* XXX TODO */
#define	BSS_CHANGED_TXPOWER		__LINE__ /* XXX TODO */
#define	BUFF_MMPDU_TXQ		__LINE__ /* XXX TODO */
#define	CHANCTX_STA_CSA		__LINE__ /* XXX TODO */
#define	CHECKSUM_PARTIAL		__LINE__ /* XXX TODO */
#define	CHECKSUM_UNNECESSARY		__LINE__ /* XXX TODO */
#define	CONNECTION_MONITOR		__LINE__ /* XXX TODO */
#define	DEAUTH_NEED_MGD_TX_PREP		__LINE__ /* XXX TODO */
#define	DEAUTH_RX_EVENT		__LINE__ /* XXX TODO */
#define	DEAUTH_TX_EVENT		__LINE__ /* XXX TODO */
#define	DUMP_PREFIX_OFFSET		__LINE__ /* XXX TODO */
#define	FCS_LEN		__LINE__ /* XXX TODO */

/* ops.configure_filter() */
#define	FIF_ALLMULTI		__LINE__ /* XXX TODO */
#define	FIF_PROBE_REQ		__LINE__ /* XXX TODO */
#define	FIF_BCN_PRBRESP_PROMISC		__LINE__ /* XXX TODO */
#define	FIF_FCSFAIL		__LINE__ /* XXX TODO */
#define	FIF_OTHER_BSS		__LINE__ /* XXX TODO */

#define	HAS_RATE_CONTROL		__LINE__ /* XXX TODO */
#define	LINK_QUAL_AGG_FRAME_LIMIT_DEF		__LINE__ /* XXX TODO */
#define	LINK_QUAL_AGG_FRAME_LIMIT_GEN2_DEF		__LINE__ /* XXX TODO */
#define	MFP_CAPABLE		__LINE__ /* XXX TODO */
#define	MLME_DENIED		__LINE__ /* XXX TODO */
#define	MLME_EVENT		__LINE__ /* XXX TODO */
#define	MLME_TIMEOUT		__LINE__ /* XXX TODO */
#define	NEEDS_UNIQUE_STA_ADDR		__LINE__ /* XXX TODO */
#define	NETREG_DUMMY		__LINE__ /* XXX TODO */
#define	NEXTHDR_DEST		__LINE__ /* XXX TODO */
#define	NEXTHDR_HOP		__LINE__ /* XXX TODO */
#define	NEXTHDR_NONE		__LINE__ /* XXX TODO */
#define	NEXTHDR_ROUTING		__LINE__ /* XXX TODO */
#define	RATE_INFO_BW_160		__LINE__ /* XXX TODO */
#define	RATE_INFO_BW_20		__LINE__ /* XXX TODO */
#define	RATE_INFO_BW_40		__LINE__ /* XXX TODO */
#define	RATE_INFO_BW_80		__LINE__ /* XXX TODO */
#define	RATE_INFO_BW_HE_RU		__LINE__ /* XXX TODO */
#define	RATE_INFO_FLAGS_HE_MCS		__LINE__ /* XXX TODO */
#define	RATE_INFO_FLAGS_MCS		__LINE__ /* XXX TODO */
#define	RATE_INFO_FLAGS_SHORT_GI		__LINE__ /* XXX TODO */
#define	RATE_INFO_FLAGS_VHT_MCS		__LINE__ /* XXX TODO */
#define	REGULATORY_CUSTOM_REG		__LINE__ /* XXX TODO */
#define	REGULATORY_DISABLE_BEACON_HINTS		__LINE__ /* XXX TODO */
#define	REGULATORY_ENABLE_RELAX_NO_IR		__LINE__ /* XXX TODO */
#define	REGULATORY_WIPHY_SELF_MANAGED		__LINE__ /* XXX TODO */
#define	REPORTS_TX_ACK_STATUS		__LINE__ /* XXX TODO */
#define	RSSI_EVENT_HIGH		__LINE__ /* XXX TODO */
#define	RSSI_EVENT_LOW		__LINE__ /* XXX TODO */
#define	RS_NAME		__LINE__ /* XXX TODO */
#define	RX_ENC_HE		__LINE__ /* XXX TODO */
#define	RX_ENC_HT		__LINE__ /* XXX TODO */
#define	RX_ENC_VHT		__LINE__ /* XXX TODO */
#define	RX_FLAG_ALLOW_SAME_PN		__LINE__ /* XXX TODO */
#define	RX_FLAG_AMPDU_DETAILS		__LINE__ /* XXX TODO */
#define	RX_FLAG_AMPDU_EOF_BIT		__LINE__ /* XXX TODO */
#define	RX_FLAG_AMPDU_EOF_BIT_KNOWN		__LINE__ /* XXX TODO */
#define	RX_FLAG_DECRYPTED		__LINE__ /* XXX TODO */
#define	RX_FLAG_DUP_VALIDATED		__LINE__ /* XXX TODO */
#define	RX_FLAG_FAILED_FCS_CRC		__LINE__ /* XXX TODO */
#define	RX_FLAG_ICV_STRIPPED		__LINE__ /* XXX TODO */
#define	RX_FLAG_MACTIME_PLCP_START		__LINE__ /* XXX TODO */
#define	RX_FLAG_MACTIME_START		__LINE__ /* XXX TODO */
#define	RX_FLAG_MIC_STRIPPED		__LINE__ /* XXX TODO */
#define	RX_FLAG_MMIC_ERROR		__LINE__ /* XXX TODO */
#define	RX_FLAG_MMIC_STRIPPED		__LINE__ /* XXX TODO */
#define	RX_FLAG_NO_PSDU		__LINE__ /* XXX TODO */
#define	RX_FLAG_PN_VALIDATED		__LINE__ /* XXX TODO */
#define	RX_FLAG_RADIOTAP_HE		__LINE__ /* XXX TODO */
#define	RX_FLAG_RADIOTAP_HE_MU		__LINE__ /* XXX TODO */
#define	RX_FLAG_RADIOTAP_LSIG		__LINE__ /* XXX TODO */
#define	RX_FLAG_RADIOTAP_VENDOR_DATA		__LINE__ /* XXX TODO */
#define	RX_INCLUDES_FCS		__LINE__ /* XXX TODO */
#define	SIGNAL_DBM		__LINE__ /* XXX TODO */
#define	SINGLE_SCAN_ON_ALL_BANDS		__LINE__ /* XXX TODO */
#define	SMP_CACHE_BYTES		__LINE__ /* XXX TODO */
#define	SPECTRUM_MGMT		__LINE__ /* XXX TODO */
#define	STA_MMPDU_TXQ		__LINE__ /* XXX TODO */
#define	SUPPORTS_AMSDU_IN_AMPDU		__LINE__ /* XXX TODO */
#define	SUPPORTS_CLONED_SKBS		__LINE__ /* XXX TODO */
#define	SUPPORTS_DYNAMIC_PS		__LINE__ /* XXX TODO */
#define	SUPPORTS_MULTI_BSSID		__LINE__ /* XXX TODO */
#define	SUPPORTS_ONLY_HE_MULTI_BSSID		__LINE__ /* XXX TODO */
#define	SUPPORTS_PS		__LINE__ /* XXX TODO */
#define	SUPPORTS_REORDERING_BUFFER		__LINE__ /* XXX TODO */
#define	SUPPORTS_VHT_EXT_NSS_BW		__LINE__ /* XXX TODO */
#define	SUPPORT_FAST_XMIT		__LINE__ /* XXX TODO */
#define	SURVEY_INFO_TIME		__LINE__ /* XXX TODO */
#define	SURVEY_INFO_TIME_RX		__LINE__ /* XXX TODO */
#define	SURVEY_INFO_TIME_SCAN		__LINE__ /* XXX TODO */
#define	SURVEY_INFO_TIME_TX		__LINE__ /* XXX TODO */
#define	TDLS_WIDER_BW		__LINE__ /* XXX TODO */
#define	TIMING_BEACON_ONLY		__LINE__ /* XXX TODO */
#define	TX_AMPDU_SETUP_IN_HW		__LINE__ /* XXX TODO */
#define	TX_AMSDU		__LINE__ /* XXX TODO */
#define	TX_FRAG_LIST		__LINE__ /* XXX TODO */
#define	USES_RSS		__LINE__ /* XXX TODO */
#define	WANT_MONITOR_VIF		__LINE__ /* XXX TODO */
#define	WIPHY_FLAG_AP_UAPSD		__LINE__ /* XXX TODO */
#define	WIPHY_FLAG_HAS_CHANNEL_SWITCH		__LINE__ /* XXX TODO */
#define	WIPHY_FLAG_IBSS_RSN		__LINE__ /* XXX TODO */
#define	WIPHY_FLAG_PS_ON_BY_DEFAULT		__LINE__ /* XXX TODO */
#define	WIPHY_FLAG_SUPPORTS_TDLS		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_AES_CMAC		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_BIP_GMAC_128		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_BIP_GMAC_256		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_CCMP		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_CCMP_256		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_GCMP		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_GCMP_256		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_TKIP		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_WEP104		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_WEP40		__LINE__ /* XXX TODO */
#define	WLAN_OUI_MICROSOFT			(0x0050F2)
#define	WLAN_OUI_TYPE_MICROSOFT_TPC		(8)
#define	WLAN_OUI_TYPE_WFA_P2P			(9)
#define	WLAN_OUI_WFA		__LINE__ /* XXX TODO */
#define	WLAN_TDLS_CHANNEL_SWITCH_REQUEST		__LINE__ /* XXX TODO */
#define	WLAN_TDLS_CHANNEL_SWITCH_RESPONSE		__LINE__ /* XXX TODO */

#define	IEEE80211_CONF_CHANGE_CHANNEL		__LINE__ /* XXX TODO */
#define	IEEE80211_CONF_CHANGE_IDLE		__LINE__ /* XXX TODO */
#define	IEEE80211_CONF_CHANGE_PS		__LINE__ /* XXX TODO */
#define	IEEE80211_CONF_IDLE			__LINE__ /* XXX TODO */
#define	IEEE80211_CONF_PS			__LINE__ /* XXX TODO */
#define	IEEE80211_HT_MPDU_DENSITY_16		__LINE__ /* XXX TODO */
#define	IEEE80211_KEY_FLAG_GENERATE_IV		__LINE__ /* XXX TODO */
#define	IEEE80211_KEY_FLAG_SW_MGMT_TX		__LINE__ /* XXX TODO */
#define	IEEE80211_TX_CTL_REQ_TX_STATUS		__LINE__ /* XXX TODO */
#define	IEEE80211_TX_STAT_NOACK_TRANSMITTED		__LINE__ /* XXX TODO */
#define	IEEE80211_VHT_CAP_HTC_VHT		__LINE__ /* XXX TODO */

#define	BSS_CHANGED_ERP_SLOT		__LINE__ /* XXX TODO */
#define	WIPHY_FLAG_TDLS_EXTERNAL_SETUP		__LINE__ /* XXX TODO */
#define	WLAN_CIPHER_SUITE_BIP_CMAC_256		__LINE__ /* XXX TODO */
#define	REGULATORY_STRICT_REG		__LINE__ /* XXX TODO */


#define	CFG80211_TESTMODE_CMD(_x)	/* XXX TODO */

struct ieee80211_sta;

struct ieee80211_ampdu_params {
	/* TODO FIXME */
	struct ieee80211_sta			*sta;
	uint8_t					tid;
	uint16_t				ssn;
	int		action, amsdu, buf_size, timeout;
};

struct ieee80211_bar {
	/* TODO FIXME */
	int		control, start_seq_num;
	uint8_t		*ra;
};

struct ieee80211_p2p_noa_attr {
	/* TODO FIXME */
	int		oppps_ctwindow;
	int     desc, index;
};

#define	WLAN_MEMBERSHIP_LEN			(8)
#define	WLAN_USER_POSITION_LEN			(16)

struct ieee80211_bss_conf {
	/* TODO FIXME */
	uint8_t					bssid[ETH_ALEN];
	uint8_t					transmitter_bssid[ETH_ALEN];
	struct ieee80211_ftm_responder_params	*ftmr_params;
	struct ieee80211_p2p_noa_attr		p2p_noa_attr;
	struct cfg80211_chan_def		chandef;
	__be32					arp_addr_list[1];	/* XXX TODO */
	struct ieee80211_rate			*beacon_rate;
	struct {
		uint8_t membership[WLAN_MEMBERSHIP_LEN];
		uint8_t position[WLAN_USER_POSITION_LEN];
	}  mu_group;
	struct {
		int color;
	} he_bss_color;
	int		assoc, idle, txpower;
	int		ack_enabled, aid, arp_addr_cnt, basic_rates, beacon_int, bssid_index, bssid_indicator, cqm_rssi_hyst, cqm_rssi_thold, dtim_period, ema_ap, frame_time_rts_th, ftm_responder;
	int		he_support, ht_operation_mode, htc_trig_based_pkt_ext;
	int		multi_sta_back_32bit, nontransmitted;
	int		profile_periodicity, ps, qos, sync_device_ts, sync_dtim_count, sync_tsf;
	int		twt_requester, uora_exists, uora_ocw_range, use_cts_prot, use_short_preamble, use_short_slot;
	int		assoc_capability, enable_beacon, hidden_ssid, ibss_joined, mcast_rate, ssid, ssid_len;
};

struct ieee80211_chanctx_conf {
	/* TODO FIXME */
	int		rx_chains_dynamic, rx_chains_static, radar_enabled;
	void		*drv_priv;
	struct cfg80211_chan_def		def;
	struct cfg80211_chan_def		min_def;
};

struct ieee80211_channel_switch {
	/* TODO FIXME */
	int		block_tx, count, delay, device_timestamp, timestamp;
	struct cfg80211_chan_def		chandef;
};

struct ieee80211_cipher_scheme {
	/* TODO FIXME */
	int		cipher, hdr_len, iftype, key_idx_mask, key_idx_off, key_idx_shift, mic_len, pn_len, pn_off;
};

struct ieee80211_event {
	/* TODO FIXME */
	uint32_t		type;
	union {
		struct {
			int     ssn;
			struct ieee80211_sta	*sta;
			uint8_t		 	tid;
		} ba;
		struct {
			int     data, reason, ssn, status;
			struct ieee80211_sta	*sta;
			uint8_t			tid;
		} mlme;
	} u;
};

struct ieee80211_ftm_responder_params {
	/* TODO FIXME */
	uint8_t					*lci;
	uint8_t					*civicloc;
	int					lci_len;
	int					civicloc_len;
};

struct ieee80211_he_mu_edca_param_ac_rec {
	/* TODO FIXME */
	int		aifsn, ecw_min_max, mu_edca_timer;
};

struct ieee80211_hw {
	/* TODO FIXME */
	int		chanctx_data_size;
	int		max_listen_interval, max_rx_aggregation_subframes, max_tx_aggregation_subframes, max_tx_fragments;
	int		netdev_features, offchannel_tx_hw_queue, queues, radiotap_mcs_details;
	int		rate_control_algorithm, sta_data_size, txq_data_size, uapsd_max_sp_len, uapsd_queues, vif_data_size;
	int		extra_tx_headroom, weight_multiplier;
	int		max_rate_tries, max_rates, max_report_rates;
	struct ieee80211_cipher_scheme	*cipher_schemes;
	int				n_cipher_schemes;
	struct {
		uint16_t units_pos;	/* radiotap "spec" is .. inconsistent. */
		uint16_t accuracy;
	} radiotap_timestamp;
	uint16_t			radiotap_vht_details;
	unsigned long			flags;
	struct wiphy			*wiphy;
	void				*priv;
	struct {
		uint32_t		listen_interval;
		struct cfg80211_chan_def chandef;
		unsigned long		flags;		/* XXX? */
	} conf;
};

struct ieee80211_key_conf {
	/* TODO FIXME */
	int		cipher, flags, hw_key_idx, keyidx, iv_len;
	atomic64_t	tx_pn;
	uint8_t		keylen;
	uint8_t		key[0];
};

struct ieee80211_key_seq {
	/* TODO FIXME */
	struct {
		uint8_t		pn[IEEE80211_CCMP_PN_LEN];
	} aes_cmac;
	struct {
		uint8_t		pn[IEEE80211_CCMP_PN_LEN];
	} comp;
	struct {
		uint8_t		pn[IEEE80211_CCMP_PN_LEN];
	} ccmp;
	struct {
		uint32_t	iv32;
	} tkip;
};

struct ieee80211_mgmt {
	/* TODO FIXME */
	uint8_t		bssid[ETH_ALEN], da[ETH_ALEN], sa[ETH_ALEN];
	int		frame_control, seq_ctrl;
	union {
		struct {
			uint8_t variable[0];
		} probe_req;
		struct {
			uint8_t variable[0];
		} probe_resp;
		struct {
			uint8_t	variable[0];
		} beacon;
		struct {
			union {
				struct {
					uint8_t variable[0];
				} ftm;
			} u;
		} action;
	} u;
};

struct ieee80211_p2p_noa_desc {
	/* TODO FIXME */
};

struct ieee80211_rx_status {
	/* TODO FIXME */
	int		ampdu_reference, band, boottime_ns, bw;
	int		chain_signal[3];
	int chains, device_timestamp, enc_flags, encoding, flag, freq, he_dcm, he_gi, he_ru, mactime;
	int		signal, zero_length_psdu_type;
	uint8_t		nss;
	uint8_t		rate_idx;
};

struct ieee80211_scan_ies {
	/* TODO FIXME */
	uint8_t		*common_ies;
	uint8_t		*ies[NUM_NL80211_BANDS];
	int		common_ie_len;
	uint8_t		len[NUM_NL80211_BANDS];
};

struct ieee80211_scan_request {
	struct ieee80211_scan_ies	ies;
	struct cfg80211_scan_request	req;
};

struct ieee80211_txq {
	/* TODO FIXME */
	uint8_t				tid;
	int				ac;
	struct ieee80211_sta		*sta;
	struct ieee80211_vif		*vif;
	void				*drv_priv;
};

struct ieee80211_sta_rates {
	/* XXX TODO */
	/* XXX some _rcu thing */
	struct {
		int	idx;
		int	flags;
	} rate[1];		/* XXX what is the real number? */
};

#define	IEEE80211_NUM_TIDS			16	/* iwlwifi 8; so double? */
struct ieee80211_sta {
	/* TODO FIXME */
	int		aid, bandwidth, max_amsdu_len, max_amsdu_subframes, max_rc_amsdu_len, max_sp;
	int		mfp, rx_nss, smps_mode, tdls, tdls_initiator, uapsd_queues, wme, txpwr;
	void					*drv_priv;
	struct ieee80211_sta_ht_cap		ht_cap;
	struct ieee80211_sta_vht_cap		vht_cap;
	struct ieee80211_sta_he_cap		he_cap;
	struct ieee80211_txq			*txq[IEEE80211_NUM_TIDS + 1];	/* iwlwifi: 8 and adds +1 to tid_data */
	struct ieee80211_sta_rates		*rates;	/* some rcu thing? */
	uint32_t				max_tid_amsdu_len[IEEE80211_NUM_TIDS];
	uint32_t				supp_rates[NUM_NL80211_BANDS];
	uint8_t					addr[ETH_ALEN];
};

struct ieee80211_tdls_ch_sw_params {
	/* TODO FIXME */
	int		action_code, ch_sw_tm_ie, status, switch_time, switch_timeout, timestamp;
	struct ieee80211_sta			*sta;
	struct cfg80211_chan_def		*chandef;
	struct sk_buff				*tmpl_skb;
};

struct ieee80211_tx_control {
	/* TODO FIXME */
	struct ieee80211_sta			*sta;
};

struct ieee80211_tx_queue_params {
	/* TODO FIXME */
	int		acm, aifs, cw_max, cw_min, mu_edca;
	int txop, uapsd;
	struct ieee80211_he_mu_edca_param_ac_rec	mu_edca_param_rec;
};

struct ieee80211_tx_rate {
	/* TODO FIXME */
	int		flags, idx, count;
};

struct ieee80211_vif {
	/* TODO FIXME */
	int		csa_active, driver_flags, mu_mimo_owner, p2p, probe_req_reg, type;
	int		cab_queue, hw_queue;
	uint8_t				addr[ETH_ALEN];
	void				*drv_priv;
	struct ieee80211_chanctx_conf	*chanctx_conf;
	struct ieee80211_bss_conf	bss_conf;
	struct ieee80211_txq		*txq;
};

struct ieee80211_vif_chanctx_switch {
	struct ieee80211_chanctx_conf	*old_ctx, *new_ctx;
	struct ieee80211_vif		*vif;
};

struct ieee80211_tx_info {
	/* TODO FIXME */
	int		band;
	enum ieee80211_tx_info_flags		flags;
	void					*driver_data[8];		/* XXX TODO */
	struct {
		struct ieee80211_key_conf	*hw_key;
		struct ieee80211_vif		*vif;
		struct ieee80211_tx_rate	rates[8];			/* XXX TODO */
		enum ieee80211_tx_control_flags	flags;
		bool				use_rts;
	} control;
	struct {
		int	ampdu_ack_len, ampdu_len, antenna, tx_time;
		int     ack_signal;
		bool				is_valid_ack_signal;
		void				*status_driver_data[2];		/* XXX TODO */
		struct ieee80211_tx_rate	rates[8];			/* XXX TODO */
	} status;

};

#if 0 /* net80211 conflict */
struct ieee80211_tim_ie {
	/* TODO FIXME */
	int     bitmap_ctrl, dtim_count, dtim_period, virtual_map;
};
#endif

struct survey_info {
	/* TODO FIXME */
	int     filled, time, time_rx, time_scan, time_tx;
	int	channel, noise, time_busy;
};

struct station_info {
	/* TODO FIXME */
	int     filled, rx_beacon, rx_beacon_signal_avg, signal_avg;
	int	rx_duration, rxrate, tx_failed, tx_retries;
	struct rate_info			txrate;
};

enum ieee80211_iface_iter {
	IEEE80211_IFACE_ITER_NORMAL,
	IEEE80211_IFACE_ITER_RESUME_ALL,
};

enum ieee80211_hw_flags {
	IEEE80211_HW_AMPDU_AGGREGATION,
	IEEE80211_HW_AP_LINK_PS,
	IEEE80211_HW_BUFF_MMPDU_TXQ,
	IEEE80211_HW_CHANCTX_STA_CSA,
	IEEE80211_HW_CONNECTION_MONITOR,
	IEEE80211_HW_DEAUTH_NEED_MGD_TX_PREP,
	IEEE80211_HW_HAS_RATE_CONTROL,
	IEEE80211_HW_MFP_CAPABLE,
	IEEE80211_HW_NEEDS_UNIQUE_STA_ADDR,
	IEEE80211_HW_REPORTS_TX_ACK_STATUS,
	IEEE80211_HW_RX_INCLUDES_FCS,
	IEEE80211_HW_SIGNAL_DBM,
	IEEE80211_HW_SINGLE_SCAN_ON_ALL_BANDS,
	IEEE80211_HW_SPECTRUM_MGMT,
	IEEE80211_HW_STA_MMPDU_TXQ,
	IEEE80211_HW_SUPPORTS_AMSDU_IN_AMPDU,
	IEEE80211_HW_SUPPORTS_CLONED_SKBS,
	IEEE80211_HW_SUPPORTS_DYNAMIC_PS,
	IEEE80211_HW_SUPPORTS_MULTI_BSSID,
	IEEE80211_HW_SUPPORTS_ONLY_HE_MULTI_BSSID,
	IEEE80211_HW_SUPPORTS_PS,
	IEEE80211_HW_SUPPORTS_REORDERING_BUFFER,
	IEEE80211_HW_SUPPORTS_VHT_EXT_NSS_BW,
	IEEE80211_HW_SUPPORT_FAST_XMIT,
	IEEE80211_HW_TDLS_WIDER_BW,
	IEEE80211_HW_TIMING_BEACON_ONLY,
	IEEE80211_HW_TX_AMPDU_SETUP_IN_HW,
	IEEE80211_HW_TX_AMSDU,
	IEEE80211_HW_TX_FRAG_LIST,
	IEEE80211_HW_USES_RSS,
	IEEE80211_HW_WANT_MONITOR_VIF,
};

enum set_key_cmd {
	SET_KEY,
	DISABLE_KEY,
};

enum rx_enc_flags {
	RX_ENC_FLAG_SHORTPRE	=	BIT(0),
	RX_ENC_FLAG_SHORT_GI	=	BIT(1),
	RX_ENC_FLAG_HT_GF	=	BIT(2),
	RX_ENC_FLAG_LDPC	=	BIT(3),
	RX_ENC_FLAG_BF		=	BIT(4),
#define	RX_ENC_FLAG_STBC_SHIFT		6
};

enum sta_notify_cmd {
	STA_NOTIFY_AWAKE,
	STA_NOTIFY_SLEEP,
};

struct ieee80211_ops {
	/* TODO FIXME */
	void(*tx)(struct ieee80211_hw *, struct ieee80211_tx_control *, struct sk_buff *);
	void(*wake_tx_queue)(struct ieee80211_hw *, struct ieee80211_txq *);
	int(*ampdu_action)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_ampdu_params *);
	int(*get_antenna)(struct ieee80211_hw *, u32 *, u32 *);
	int(*start)(struct ieee80211_hw *);
	void(*reconfig_complete)(struct ieee80211_hw *, enum ieee80211_reconfig_type);
	void(*stop)(struct ieee80211_hw *);
	int(*add_interface)(struct ieee80211_hw *, struct ieee80211_vif *);
	void(*remove_interface)(struct ieee80211_hw *, struct ieee80211_vif *);
	int(*config)(struct ieee80211_hw *, u32);
	u64(*prepare_multicast)(struct ieee80211_hw *, struct netdev_hw_addr_list *);
	void(*configure_filter)(struct ieee80211_hw *, unsigned int, unsigned int *, u64);
	void(*config_iface_filter)(struct ieee80211_hw *, struct ieee80211_vif *, unsigned int, unsigned int);
	void(*bss_info_changed)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_bss_conf *, u32);
	int(*hw_scan)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_scan_request *);
	void(*cancel_hw_scan)(struct ieee80211_hw *, struct ieee80211_vif *);
	void(*sta_pre_rcu_remove)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *);
	int(*sta_state)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *, enum ieee80211_sta_state, enum ieee80211_sta_state);
	void(*sta_notify)(struct ieee80211_hw *, struct ieee80211_vif *, enum sta_notify_cmd, struct ieee80211_sta *);
	void(*allow_buffered_frames)(struct ieee80211_hw *, struct ieee80211_sta *, u16, int, enum ieee80211_frame_release_type, bool);
	void(*release_buffered_frames)(struct ieee80211_hw *, struct ieee80211_sta *, u16, int, enum ieee80211_frame_release_type, bool);
	int(*set_rts_threshold)(struct ieee80211_hw *, u32);
	void(*sta_rc_update)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *, u32);
	int(*conf_tx)(struct ieee80211_hw *, struct ieee80211_vif *, u16, const struct ieee80211_tx_queue_params *);
	void(*mgd_prepare_tx)(struct ieee80211_hw *, struct ieee80211_vif *, u16);
	void(*mgd_protect_tdls_discover)(struct ieee80211_hw *, struct ieee80211_vif *);
	void(*flush)(struct ieee80211_hw *, struct ieee80211_vif *, u32, bool);
	int(*sched_scan_start)(struct ieee80211_hw *, struct ieee80211_vif *, struct cfg80211_sched_scan_request *, struct ieee80211_scan_ies *);
	int(*sched_scan_stop)(struct ieee80211_hw *, struct ieee80211_vif *);
	int(*set_key)(struct ieee80211_hw *, enum set_key_cmd, struct ieee80211_vif *, struct ieee80211_sta *, struct ieee80211_key_conf *);
	void(*update_tkip_key)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_key_conf *, struct ieee80211_sta *, u32, u16 *);
	int(*remain_on_channel)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_channel *, int, enum ieee80211_roc_type);
	int(*cancel_remain_on_channel)(struct ieee80211_hw *, struct ieee80211_vif *);
	int(*add_chanctx)(struct ieee80211_hw *, struct ieee80211_chanctx_conf *);
	void(*remove_chanctx)(struct ieee80211_hw *, struct ieee80211_chanctx_conf *);
	void(*change_chanctx)(struct ieee80211_hw *, struct ieee80211_chanctx_conf *, u32);
	int(*assign_vif_chanctx)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_chanctx_conf *);
	void(*unassign_vif_chanctx)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_chanctx_conf *);
	int(*switch_vif_chanctx)(struct ieee80211_hw *, struct ieee80211_vif_chanctx_switch *, int, enum ieee80211_chanctx_switch_mode);
	int(*start_ap)(struct ieee80211_hw *, struct ieee80211_vif *);
	void(*stop_ap)(struct ieee80211_hw *, struct ieee80211_vif *);
	int(*join_ibss)(struct ieee80211_hw *, struct ieee80211_vif *);
	void(*leave_ibss)(struct ieee80211_hw *, struct ieee80211_vif *);
	int(*tx_last_beacon)(struct ieee80211_hw *);
	int(*set_tim)(struct ieee80211_hw *, struct ieee80211_sta *, bool);
	void(*channel_switch)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_channel_switch *);
	int(*pre_channel_switch)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_channel_switch *);
	int(*post_channel_switch)(struct ieee80211_hw *, struct ieee80211_vif *);
	void(*abort_channel_switch)(struct ieee80211_hw *, struct ieee80211_vif *);
	void(*channel_switch_rx_beacon)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_channel_switch *);
	int(*tdls_channel_switch)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *, u8, struct cfg80211_chan_def *, struct sk_buff *, u32);
	void(*tdls_cancel_channel_switch)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *);
	void(*tdls_recv_channel_switch)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_tdls_ch_sw_params *);
	void(*event_callback)(struct ieee80211_hw *, struct ieee80211_vif *, const struct ieee80211_event *);
	void(*sync_rx_queues)(struct ieee80211_hw *);
	int(*get_survey)(struct ieee80211_hw *, int, struct survey_info *);
	void(*sta_statistics)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *, struct station_info *);
	int(*get_ftm_responder_stats)(struct ieee80211_hw *, struct ieee80211_vif *, struct cfg80211_ftm_responder_stats *);
	int(*start_pmsr)(struct ieee80211_hw *, struct ieee80211_vif *, struct cfg80211_pmsr_request *);
	void(*abort_pmsr)(struct ieee80211_hw *, struct ieee80211_vif *, struct cfg80211_pmsr_request *);
	bool(*can_aggregate_in_amsdu)(struct ieee80211_hw *, struct sk_buff *, struct sk_buff *);
	int (*sta_add)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *);
	int (*sta_remove)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *);
	void (*sw_scan_start)(struct ieee80211_hw *, struct ieee80211_vif *, const u8 *);
	void (*sw_scan_complete)(struct ieee80211_hw *, struct ieee80211_vif *);
	int (*set_bitrate_mask)(struct ieee80211_hw *, struct ieee80211_vif *, const struct cfg80211_bitrate_mask *);
	int (*set_antenna)(struct ieee80211_hw *, u32, u32);
	void (*set_coverage_class)(struct ieee80211_hw *, s16);
	void (*set_default_unicast_key)(struct ieee80211_hw *, struct ieee80211_vif *, int);
	int (*sta_set_txpwr)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *);
	int (*set_frag_threshold)(struct ieee80211_hw *, u32);
	void (*offset_tsf)(struct ieee80211_hw *, struct ieee80211_vif *, s64);
	/* XXX TODO: get_et_sset_count, get_et_stats, get_et_strings */
	void (*sta_rate_tbl_update)(struct ieee80211_hw *, struct ieee80211_vif *, struct ieee80211_sta *);
};


static __inline bool
ieee80211_is_mgmt(uint16_t frame_control)
{
	/* XXX net80211: IEEE80211_IS_MGMT() */
	/* XXX TODO */

	return (false);
}

static __inline bool
ieee80211_is_ctl(uint16_t frame_control)
{
	/* XXX net80211: IEEE80211_IS_CTL() */
	/* XXX TODO */

	return (false);
}

static __inline bool
ieee80211_is_data(uint16_t frame_control)
{
	/* XXX net80211: IEEE80211_IS_DATA() */
	/* XXX TODO */

	return (false);
}

static __inline bool
ieee80211_is_data_qos(uint16_t frame_control)
{
	/* XXX net80211: IEEE80211_IS_QOSDATA() */
	/* XXX TODO */

	return (false);
}

/* MGMT */

static __inline bool
ieee80211_is_disassoc(uint16_t frame_control)
{
	/* XXX net80211: IEEE80211_FC0_SUBTYPE_DISASSOC */
	/* XXX TODO */

	return (false);
}


static __inline void
ieee80211_iterate_active_interfaces_atomic(struct ieee80211_hw *hw,
    enum ieee80211_iface_iter flags,
    void(*iterfunc)(void *, uint8_t *, struct ieee80211_vif *),
    void *arg)
{
	/* XXX TODO */
}

static __inline struct wireless_dev *
ieee80211_vif_to_wdev(struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return (NULL);
}

static __inline struct sk_buff *
ieee80211_beacon_get_template(struct ieee80211_hw *hw,
    struct ieee80211_vif *vif, void *p /* XXX TODO */)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
ieee80211_beacon_loss(struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_chswitch_done(struct ieee80211_vif *vif, bool t)
{
	/* XXX TODO */
	return;
}

static __inline bool
ieee80211_csa_is_complete(struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return (false);
}

static __inline void
ieee80211_csa_set_counter(struct ieee80211_vif *vif, uint8_t counter)
{
	/* XXX TODO */
	return;
}

static __inline int
ieee80211_csa_update_counter(struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return (-1);
}

static __inline void
ieee80211_csa_finish(struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return;
}

static __inline int
ieee80211_vif_type_p2p(struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return (-1);
}

static __inline void
ieee80211_rx_napi(struct ieee80211_hw *hw, void *pa /* XXX TODO */,
    struct sk_buff *skb, void *pb /* XXX TODO */)
{
	/* XXX TODO */
	return;
}

static __inline unsigned long
ieee80211_tu_to_usec(unsigned long tu)
{
	unsigned long usec;

	/* XXX TODO */
	usec = tu * 0 + -1;

	return (usec);
}


static __inline void
_ieee80211_hw_set(struct ieee80211_hw *hw, uint32_t flag)
{
	/* XXX TODO */
	return;
}

/* They pass in shortened flag names; how confusingly inconsistent. */
#define	ieee80211_hw_set(_hw, _flag)					\
	_ieee80211_hw_set((_hw), IEEE80211_HW_ ## _flag)

#define	IEEE80211_SKB_CB(_skb)						\
	((struct ieee80211_tx_info *)((_skb)->cb))

#define	IEEE80211_SKB_RXCB(_skb)					\
	((struct ieee80211_rx_status *)((_skb)->cb))


static __inline int
ieee80211_action_contains_tpc(struct sk_buff *skb)
{
	/* XXX TODO */
	return (0);
}

static __inline void
ieee80211_free_hw(struct ieee80211_hw *hw)
{

	/* XXX this probably should be a wiphy_free()! */
	if (hw->wiphy != NULL)
		kfree(hw->wiphy);
	if (hw->priv != NULL)
		kfree(hw->priv);
	kfree(hw);
	/* XXX TODO */
	return;
}

static __inline struct ieee80211_hw *
ieee80211_alloc_hw(size_t priv_len, const struct ieee80211_ops *ops)
{
	struct ieee80211_hw *hw;

	hw = (struct ieee80211_hw *)kzalloc(sizeof(*hw), 0);
	if (hw == NULL)
		return (NULL);

	/* Allocate an extra driver private area based on len given for that. */
	hw->priv = kzalloc(priv_len, 0);
	if (hw->priv == NULL) {
		ieee80211_free_hw(hw);
		return (NULL);
	}

	/* XXX Not sure if we'll need an "internal" wiphy_alloc() for that? */
	/* XXX check cfg80211 drivers, like fmac maybe? */
	hw->wiphy = kzalloc(sizeof(*hw->wiphy), 0);
	if (hw->wiphy == NULL) {
		kfree(hw->priv);
		ieee80211_free_hw(hw);
		return (NULL);
	}

	/* XXX TODO */

	return (hw);
}

static __inline void
ieee80211_connection_loss(struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return;
}

static __inline struct ieee80211_sta *
ieee80211_find_sta(struct ieee80211_vif *vif, const u8 *peer)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
ieee80211_get_tkip_p2k(struct ieee80211_key_conf *keyconf,
    struct sk_buff *skb_frag, u8 *key)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_get_tkip_rx_p1k(struct ieee80211_key_conf *keyconf,
    const u8 *addr, uint32_t iv32, u16 *p1k)
{
	/* XXX TODO */
	return;
}

static __inline bool
ieee80211_has_a4(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_has_order(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_has_retry(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_hw_check(struct ieee80211_hw *hw, enum ieee80211_hw_flags flag)
{
	/* XXX TODO */
	return (false);
}

static __inline size_t
ieee80211_ie_split(const u8 *ies, size_t len,
    const u8 *before_params, size_t before_params_len, int x)
{
	/* XXX TODO */
	return (-1);
}

static __inline bool
ieee80211_is_action(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_is_assoc_req(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_is_data_present(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_is_deauth(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_is_probe_req(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_is_reassoc_req(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline struct ieee80211_hw *
wiphy_to_ieee80211_hw(struct wiphy *wiphy)
{
	/* XXX TODO */
	return (NULL);
}

static __inline uint8_t *
ieee80211_get_DA(struct ieee80211_hdr *hdr)
{
	/* XXX TODO */
	return (NULL);
}

static __inline uint8_t *
ieee80211_get_SA(struct ieee80211_hdr *hdr)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
ieee80211_request_smps(struct ieee80211_vif *vif, enum ieee80211_smps_mode smps)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_iterate_active_interfaces(struct ieee80211_hw *hw,
    enum ieee80211_iface_iter flags,
    void(*iterfunc)(void *, uint8_t *, struct ieee80211_vif *),
    void *arg)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_tdls_oper_request(struct ieee80211_vif *vif, uint8_t *addr,
    enum nl80211_tdls_operation oper, enum ieee80211_reason_code code,
    gfp_t gfp)
{
	/* XXX TODO */
	return;
}

static __inline void
SET_IEEE80211_DEV(struct ieee80211_hw *hw, struct device *dev)
{

	/* XXX TODO */
	/* XXX ieee80211_hw has no dev member, ... where do we put the device? */
	return;
}

static __inline void
ieee80211_unregister_hw(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_stop_queues(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_wake_queues(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return;
}

static __inline void
wiphy_rfkill_set_hw_state(struct wiphy *wiphy, bool state)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_free_txskb(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_restart_hw(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_ready_on_channel(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_remain_on_channel_expired(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return;
}


static __inline bool
ieee80211_has_morefrags(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_has_protected(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline uint8_t
ieee80211_get_tid(struct ieee80211_hdr *hdr)
{
	/* XXX TODO */
	return (-1);
}

static __inline bool
ieee80211_is_probe_resp(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline struct ieee80211_sta *
ieee80211_find_sta_by_ifaddr(struct ieee80211_hw *hw, uint8_t *addr, void *p)
{
	/* XXX TODO */
	return (NULL);
}

static __inline bool
ieee80211_is_back_req(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_is_beacon(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline void
ieee80211_cqm_rssi_notify(struct ieee80211_vif *vif,
    enum nl80211_cqm_rssi_threshold_event crte, int sig, gfp_t gfp)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_mark_rx_ba_filtered_frames(struct ieee80211_sta *sta, uint8_t tid,
    uint32_t ssn, uint64_t bitmap, uint16_t received_mpdu)
{
	/* XXX TODO */
	return;
}

static __inline bool
ieee80211_is_qos_nullfunc(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_sn_less(uint16_t sn1, uint16_t sn2)
{
	/* XXX TODO */
	return (false);
}

static __inline uint16_t
ieee80211_sn_inc(uint16_t sn)
{
	/* XXX TODO */
	return (sn + 1);
}

static __inline uint16_t
ieee80211_sn_add(uint16_t sn, uint16_t a)
{
	/* XXX TODO */
	return (sn + a);
}

static __inline void
ieee80211_stop_rx_ba_session(struct ieee80211_vif *vif, uint32_t x, uint8_t *addr)
{
	/* XXX TODO */
	return;
}

static __inline uint8_t *
ieee80211_get_qos_ctl(struct ieee80211_hdr *hdr)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
ieee80211_rate_set_vht(struct ieee80211_tx_rate *r, uint32_t f1, uint32_t f2)
{
	/* XXX TODO */
	return;
}

static __inline int
ieee80211_register_hw(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return (ENXIO);
}

static __inline void
ieee80211_reserve_tid(struct ieee80211_sta *sta, uint8_t tid)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_unreserve_tid(struct ieee80211_sta *sta, uint8_t tid)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_rx_ba_timer_expired(struct ieee80211_vif *vif, uint8_t *addr,
    uint8_t tid)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_send_eosp_nullfunc(struct ieee80211_sta *sta, uint8_t tid)
{
	/* XXX TODO */
	return;
}

static __inline uint16_t
ieee80211_sn_sub(uint16_t sn, uint16_t n)
{
	/* XXX TODO */
	return (-1);
}

static __inline void
ieee80211_sta_block_awake(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
    bool disable)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_sta_ps_transition(struct ieee80211_sta *sta, bool sleeping)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_sta_pspoll(struct ieee80211_sta *sta)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_sta_uapsd_trigger(struct ieee80211_sta *sta, int ntids)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_start_tx_ba_cb_irqsafe(struct ieee80211_vif *vif, uint8_t *addr,
    uint8_t tid)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_tkip_add_iv(u8 *crypto_hdr, struct ieee80211_key_conf *keyconf,
    uint64_t pn)
{
	/* XXX TODO */
	return;
}

static __inline struct sk_buff *
ieee80211_tx_dequeue(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
ieee80211_update_mu_groups(struct ieee80211_vif *vif, uint8_t *ms, uint8_t *up)
{
	/* XXX TODO */
	return;
}

static __inline bool
ieee80211_is_bufferable_mmpdu(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline void
ieee80211_iterate_interfaces(struct ieee80211_hw *hw, int x,
   void (*iterfunc)(void *, u8 *, struct ieee80211_vif *), void *p)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_sta_set_buffered(struct ieee80211_sta *sta, uint8_t tid, bool t)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_tx_status(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_get_key_rx_seq(struct ieee80211_key_conf *keyconf, uint8_t tid,
    struct ieee80211_key_seq *seq)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_sched_scan_results(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_sta_eosp(struct ieee80211_sta *sta)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_stop_tx_ba_cb_irqsafe(struct ieee80211_vif *vif, uint8_t *addr,
    uint8_t tid)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_sched_scan_stopped(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_scan_completed(struct ieee80211_hw *hw,
    struct cfg80211_scan_info *info)
{
	/* XXX TODO */
	return;
}

static __inline struct sk_buff *
ieee80211_beacon_get(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return (NULL);
}

static __inline struct sk_buff *
ieee80211_pspoll_get(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return (NULL);
}

static __inline struct sk_buff *
ieee80211_proberesp_get(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return (NULL);
}

static __inline struct sk_buff *
ieee80211_nullfunc_get(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
    bool _t)
{
	/* XXX TODO */
	return (NULL);
}

static __inline struct sk_buff *
ieee80211_probereq_get(struct ieee80211_hw *hw, uint8_t *addr,
    uint8_t *ssid, size_t ssid_len, int _x)
{
	/* XXX TODO */
	return (NULL);
}

static __inline bool
ieee80211_has_fromds(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_has_tods(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_is_nullfunc(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline void
ieee80211_iter_keys(struct ieee80211_hw *hw, struct ieee80211_vif *vif, 
    void(*iter)(struct ieee80211_hw *, struct ieee80211_vif *,
        struct ieee80211_sta *, struct ieee80211_key_conf *, void *),
    void *p)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_iterate_stations_atomic(struct ieee80211_hw *hw,
   void (*iterfunc)(void *, struct ieee80211_sta *), void *p)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_queue_delayed_work(struct ieee80211_hw *hw, struct delayed_work *w,
    int delay)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_queue_work(struct ieee80211_hw *hw, struct work_struct *w)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_stop_queue(struct ieee80211_hw *hw, uint16_t q)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_wake_queue(struct ieee80211_hw *hw, uint16_t q)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_rx_irqsafe(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_tx_status_irqsafe(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

static __inline int
ieee80211_start_tx_ba_session(struct ieee80211_sta *sta, uint8_t tid, int x)
{
	/* XXX TODO */
	return (ENXIO);
}

static __inline void
ieee80211_tx_info_clear_status(struct ieee80211_tx_info *info)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_txq_get_depth(struct ieee80211_txq *txq, uint64_t *frame_cnt, uint64_t *byte_cnt)
{
	/* XXX TODO */
	return;
}

static __inline int
rate_lowest_index(struct ieee80211_supported_band *band,
    struct ieee80211_sta *sta)
{
	/* XXX TODO */
	return (-1);
}

static __inline void
wiphy_ext_feature_set(struct wiphy *wiphy, enum nl80211_ext_feature ef)
{
	/* XXX TODO */
	return;
}

static __inline void
SET_IEEE80211_PERM_ADDR	(struct ieee80211_hw *hw, uint8_t *addr)
{
	/* XXX TODO hw->xxx = addr; */
	return;
}

static __inline uint8_t *
ieee80211_bss_get_ie(struct cfg80211_bss *bss, uint32_t x)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
ieee80211_report_low_ack(struct ieee80211_sta *sta, int x)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_start_rx_ba_session_offl(struct ieee80211_vif *vif, uint8_t *addr,
    uint8_t tid)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_stop_rx_ba_session_offl(struct ieee80211_vif *vif, uint8_t *addr,
    uint8_t tid)
{
	/* XXX TODO */
	return;
}

static __inline struct sk_buff *
ieee80211_tx_dequeue_ni(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
ieee80211_tx_rate_update(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
    struct ieee80211_tx_info *info)
{
	/* XXX TODO */
	return;
}

static __inline bool
ieee80211_txq_may_transmit(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
	/* XXX TODO */
	return (false);
}

static __inline bool
ieee80211_is_auth(uint16_t fc)
{
	/* XXX TODO */
	return (false);
}

static __inline struct ieee80211_txq *
ieee80211_next_txq(struct ieee80211_hw *hw, uint32_t ac)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
ieee80211_radar_detected(struct ieee80211_hw *hw)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_rx_ni(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_sta_register_airtime(struct ieee80211_sta *sta,
    uint8_t tid, uint32_t duration, int x)
{
	/* XXX TODO */
	return;
}

static __inline bool
ieee80211_vif_is_mesh(struct ieee80211_vif *vif)
{
	/* XXX TODO */
	return (false);
}

static __inline void
ieee80211_return_txq(struct ieee80211_hw *hw,
    struct ieee80211_txq *txq, bool _t)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_txq_schedule_end(struct ieee80211_hw *hw, uint32_t ac)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_txq_schedule_start(struct ieee80211_hw *hw, uint32_t ac)
{
	/* XXX TODO */
	return;
}

static __inline void
ieee80211_iter_chan_contexts_atomic(struct ieee80211_hw *hw,
    void(*iterf)(struct ieee80211_hw *, struct ieee80211_chanctx_conf *, void *),
    void *p)
{
	/* XXX TODO */
	return;
}

static __inline struct ieee80211_channel *
ieee80211_get_channel(struct wiphy *wiphy, uint32_t freq)
{
	/* XXX TODO */
	return (NULL);
}

#endif	/* _NET_MAC80211_H */

/* end */
