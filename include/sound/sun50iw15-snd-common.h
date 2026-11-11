/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * sun50iw15 sound common helpers
 *
 * Shared pieces of the vendor's snd_soc_sunxi_common module that other
 * sun50iw15 audio drivers can use: the RX sync domain/id reference counters
 * and the card jack constructor with the vendor's button key map.
 */

#ifndef _SUN50IW15_SND_COMMON_H
#define _SUN50IW15_SND_COMMON_H

#include <sound/jack.h>
#include <sound/soc.h>

/*
 * The vendor keeps two independent RX sync domains, each with up to eight
 * stream ids (one per DAI/substream).  Domain 0 is the I2S/TDM receive path,
 * domain 1 is OWA/HDMI receive.
 */
#define SUNXI_RX_SYNC_DOMAINS	2
#define SUNXI_RX_SYNC_IDS	8

typedef void (*sunxi_rx_sync_cb_t)(void *data, int on);

int sunxi_rx_sync_probe(unsigned int domain);
void sunxi_rx_sync_remove(unsigned int domain);

int sunxi_rx_sync_register_cb(unsigned int domain, unsigned int id,
			      sunxi_rx_sync_cb_t cb, void *data);
void sunxi_rx_sync_unregister_cb(unsigned int domain, unsigned int id);

/*
 * Domain wide callback, fired when the number of running streams in the
 * domain crosses zero.  This is the vendor's "+0x98" callback that enables
 * or disables the shared RX clock.
 */
int sunxi_rx_sync_register_domain_cb(unsigned int domain,
				     sunxi_rx_sync_cb_t cb, void *data);
void sunxi_rx_sync_unregister_domain_cb(unsigned int domain);

void sunxi_rx_sync_startup(unsigned int domain, unsigned int id);
void sunxi_rx_sync_shutdown(unsigned int domain, unsigned int id);
void sunxi_rx_sync_control(unsigned int domain, unsigned int id, int on);

/*
 * Create the card jack the way the vendor does: a plain snd_jack plus the
 * four headset buttons mapped to media / volume up / volume down / voice
 * command.  Buttons are only added when the requested mask has them, so a
 * jack that only reports insertion (our codec's GPIO) is unaffected.
 */
int snd_sunxi_card_jack_new(struct snd_soc_card *card, const char *id,
			    int mask, struct snd_soc_jack *jack);

#endif /* _SUN50IW15_SND_COMMON_H */
