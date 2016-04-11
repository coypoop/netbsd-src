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

/*
 * This is "two-way" interface, acting as the SHIM layer between driver
 * and PHY layer. The driver can optionally call this translation layer
 * to do some preprocessing, then reach PHY. On the PHY->driver direction,
 * all calls go through this layer since PHY doesn't have access to the
 * driver's brcms_hardware pointer.
 */
#ifndef __NetBSD__
#include <linux/slab.h>
#include <net/mac80211.h>
#endif

#include <main.h>
#include "mac80211_if.h"
#include "phy_shim.h"

/* PHY SHIM module specific state */
struct phy_shim_info {
	struct brcms_hardware *wlc_hw;	/* pointer to main wlc_hw structure */
	struct brcms_c_info *wlc;	/* pointer to main wlc structure */
	struct brcms_info *wl; /* pointer to os-specific private state */
};

struct phy_shim_info *wlc_phy_shim_attach(struct brcms_hardware *wlc_hw,
					  struct brcms_info *wl,
					  struct brcms_c_info *wlc) {
	struct phy_shim_info *physhim = NULL;

	physhim = kmem_zalloc(sizeof(struct phy_shim_info), KM_NOSLEEP);
	if (!physhim)
		return NULL;

	physhim->wlc_hw = wlc_hw;
	physhim->wlc = wlc;
	physhim->wl = wl;

	return physhim;
}

void wlc_phy_shim_detach(struct phy_shim_info *physhim)
{
	kmem_free(physhim);
}

struct wlapi_timer *wlapi_init_timer(struct phy_shim_info *physhim,
				     void (*fn)(struct brcms_phy *pi),
				     void *arg, const char *name)
{
	return (struct wlapi_timer *)
			brcms_init_timer(physhim->wl, (void (*)(void *))fn,
					 arg, name);
}

void wlapi_free_timer(struct wlapi_timer *t)
{
	brcms_free_timer((struct brcms_timer *)t);
}

void
wlapi_add_timer(struct wlapi_timer *t, uint ms, int periodic)
{
	brcms_add_timer((struct brcms_timer *)t, ms, periodic);
}

bool wlapi_del_timer(struct wlapi_timer *t)
{
	return brcms_del_timer((struct brcms_timer *)t);
}

void wlapi_intrson(struct phy_shim_info *physhim)
{
	brcms_intrson(physhim->wl);
}

uint32_t wlapi_intrsoff(struct phy_shim_info *physhim)
{
	return brcms_intrsoff(physhim->wl);
}

void wlapi_intrsrestore(struct phy_shim_info *physhim, uint32_t macintmask)
{
	brcms_intrsrestore(physhim->wl, macintmask);
}

void wlapi_bmac_write_shm(struct phy_shim_info *physhim, uint offset, uint16_t v)
{
	brcms_b_write_shm(physhim->wlc_hw, offset, v);
}

uint16_t wlapi_bmac_read_shm(struct phy_shim_info *physhim, uint offset)
{
	return brcms_b_read_shm(physhim->wlc_hw, offset);
}

void
wlapi_bmac_mhf(struct phy_shim_info *physhim, uint8_t idx, uint16_t mask,
	       uint16_t val, int bands)
{
	brcms_b_mhf(physhim->wlc_hw, idx, mask, val, bands);
}

void wlapi_bmac_corereset(struct phy_shim_info *physhim, uint32_t flags)
{
	brcms_b_corereset(physhim->wlc_hw, flags);
}

void wlapi_suspend_mac_and_wait(struct phy_shim_info *physhim)
{
	brcms_c_suspend_mac_and_wait(physhim->wlc);
}

void wlapi_switch_macfreq(struct phy_shim_info *physhim, uint8_t spurmode)
{
	brcms_b_switch_macfreq(physhim->wlc_hw, spurmode);
}

void wlapi_enable_mac(struct phy_shim_info *physhim)
{
	brcms_c_enable_mac(physhim->wlc);
}

void wlapi_bmac_mctrl(struct phy_shim_info *physhim, uint32_t mask, uint32_t val)
{
	brcms_b_mctrl(physhim->wlc_hw, mask, val);
}

void wlapi_bmac_phy_reset(struct phy_shim_info *physhim)
{
	brcms_b_phy_reset(physhim->wlc_hw);
}

void wlapi_bmac_bw_set(struct phy_shim_info *physhim, uint16_t bw)
{
	brcms_b_bw_set(physhim->wlc_hw, bw);
}

uint16_t wlapi_bmac_get_txant(struct phy_shim_info *physhim)
{
	return brcms_b_get_txant(physhim->wlc_hw);
}

void wlapi_bmac_phyclk_fgc(struct phy_shim_info *physhim, bool clk)
{
	brcms_b_phyclk_fgc(physhim->wlc_hw, clk);
}

void wlapi_bmac_macphyclk_set(struct phy_shim_info *physhim, bool clk)
{
	brcms_b_macphyclk_set(physhim->wlc_hw, clk);
}

void wlapi_bmac_core_phypll_ctl(struct phy_shim_info *physhim, bool on)
{
	brcms_b_core_phypll_ctl(physhim->wlc_hw, on);
}

void wlapi_bmac_core_phypll_reset(struct phy_shim_info *physhim)
{
	brcms_b_core_phypll_reset(physhim->wlc_hw);
}

void wlapi_bmac_ucode_wake_override_phyreg_set(struct phy_shim_info *physhim)
{
	brcms_c_ucode_wake_override_set(physhim->wlc_hw,
					BRCMS_WAKE_OVERRIDE_PHYREG);
}

void wlapi_bmac_ucode_wake_override_phyreg_clear(struct phy_shim_info *physhim)
{
	brcms_c_ucode_wake_override_clear(physhim->wlc_hw,
					  BRCMS_WAKE_OVERRIDE_PHYREG);
}

void
wlapi_bmac_write_template_ram(struct phy_shim_info *physhim, int offset,
			      int len, void *buf)
{
	brcms_b_write_template_ram(physhim->wlc_hw, offset, len, buf);
}

uint16_t wlapi_bmac_rate_shm_offset(struct phy_shim_info *physhim, uint8_t rate)
{
	return brcms_b_rate_shm_offset(physhim->wlc_hw, rate);
}

void wlapi_ucode_sample_init(struct phy_shim_info *physhim)
{
}

void
wlapi_copyfrom_objmem(struct phy_shim_info *physhim, uint offset, void *buf,
		      int len, uint32_t sel)
{
	brcms_b_copyfrom_objmem(physhim->wlc_hw, offset, buf, len, sel);
}

void
wlapi_copyto_objmem(struct phy_shim_info *physhim, uint offset, const void *buf,
		    int l, uint32_t sel)
{
	brcms_b_copyto_objmem(physhim->wlc_hw, offset, buf, l, sel);
}
