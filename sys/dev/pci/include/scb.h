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

#ifndef _BRCM_SCB_H_
#define _BRCM_SCB_H_

#ifndef __NetBSD__
#include <linux/if_ether.h>
#endif

#include <brcmu_utils.h>
#include <defs.h>
#include <bcm_types.h>

#define AMPDU_TX_BA_MAX_WSIZE	64	/* max Tx ba window size (in pdu) */

#define AMPDU_MAX_SCB_TID	NUMPRIO

/* scb flags */
#define SCB_WMECAP		0x0040
#define SCB_HTCAP		0x10000	/* HT (MIMO) capable device */
#define SCB_IS40		0x80000	/* 40MHz capable */
#define SCB_STBCCAP		0x40000000	/* STBC Capable */

#define SCB_MAGIC	0xbeefcafe

/* structure to store per-tid state for the ampdu initiator */
struct scb_ampdu_tid_ini {
	uint8_t tid;		  /* initiator tid for easy lookup */
	/* tx retry count; indexed by seq modulo */
	uint8_t txretry[AMPDU_TX_BA_MAX_WSIZE];
	struct scb *scb;  /* backptr for easy lookup */
	uint8_t ba_wsize;	  /* negotiated ba window size (in pdu) */
};

struct scb_ampdu {
	struct scb *scb;	/* back pointer for easy reference */
	uint8_t mpdu_density;	/* mpdu density */
	uint8_t max_pdu;		/* max pdus allowed in ampdu */
	uint8_t release;		/* # of mpdus released at a time */
	uint16_t min_len;		/* min mpdu len to support the density */
	uint32_t max_rx_ampdu_bytes;	/* max ampdu rcv length; 8k, 16k, 32k, 64k */

	/*
	 * This could easily be a ini[] pointer and we keep this info in wl
	 * itself instead of having mac80211 hold it for us. Also could be made
	 * dynamic per tid instead of static.
	 */
	/* initiator info - per tid (NUMPRIO): */
	struct scb_ampdu_tid_ini ini[AMPDU_MAX_SCB_TID];
};

/* station control block - one per remote MAC address */
struct scb {
	uint32_t magic;
	uint32_t flags;	/* various bit flags as defined below */
	uint32_t flags2;	/* various bit flags2 as defined below */
	uint8_t state;	/* current state bitfield of auth/assoc process */
	uint8_t ea[ETH_ALEN];	/* station address */
	uint fragresid[NUMPRIO];/* #bytes unused in frag buffer per prio */

	uint16_t seqctl[NUMPRIO];	/* seqctl of last received frame (for dups) */
	/* seqctl of last received frame (for dups) for non-QoS data and
	 * management */
	uint16_t seqctl_nonqos;
	uint16_t seqnum[NUMPRIO];/* WME: driver maintained sw seqnum per priority */

	struct scb_ampdu scb_ampdu;	/* AMPDU state including per tid info */
};

#endif				/* _BRCM_SCB_H_ */
