// SPDX-License-Identifier: GPL-2.0-only
/*
 * sun50iw15 sound common helpers
 *
 * Reimplementation of the parts of the vendor snd_soc_sunxi_common.ko that
 * this tree can use, recovered from dec_mod/snd_soc_sunxi_common.c and the
 * module's own strings:
 *
 *   - sunxi_rx_sync_*: the reference counters that decide when the shared RX
 *     clock of a domain may be enabled.  The vendor keeps a per-domain user
 *     count (returned to probe callers as "previous users"), a per-domain
 *     count of started substreams, a per-domain count of control-enabled
 *     streams, and a 2 x 8 table of (callback, data) pairs.
 *   - snd_sunxi_card_jack_new(): the vendor's card jack constructor.  The
 *     vendor rolls its own struct instead of using snd_soc_card_jack_new(),
 *     but the object it hands to snd_jack_new() is the same, so this wraps
 *     the ASoC helper and adds the vendor's button key map.
 *
 * Not reproduced here, deliberately:
 *
 *   - the extcon and Type-C jack backends and the "sdbp" scan modes.  They
 *     are selected by jack-sdbp-method / jack-swpin-* / extcon properties,
 *     none of which exist on the HY300 device tree, and they need the vendor
 *     codec's private ops table to report anything.
 *   - the jack state machine itself (det IRQ work, det scan work).  Our
 *     codec drives detection through snd_soc_jack_add_gpios() with the
 *     codec's own set_jack callback instead.
 *   - snd_sunxi_pa_pin_*, snd_sunxi_regulator_*, snd_sunxi_extparam_* and the
 *     dump helpers.  PA pin power is implemented in sun50iw15-codec.c; the
 *     remaining three are used by the vendor's codec/HDMI modules, which
 *     this tree replaces.
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/spinlock.h>

#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-jack.h>
#include <sound/sun50iw15-snd-common.h>

struct sunxi_rx_sync_cb {
	sunxi_rx_sync_cb_t	cb;
	void			*data;
};

static DEFINE_SPINLOCK(sunxi_rx_sync_lock);
static int sunxi_rx_sync_users[SUNXI_RX_SYNC_DOMAINS];
static int sunxi_rx_sync_started[SUNXI_RX_SYNC_DOMAINS];
static int sunxi_rx_sync_running[SUNXI_RX_SYNC_DOMAINS];
static struct sunxi_rx_sync_cb sunxi_rx_sync_ids[SUNXI_RX_SYNC_DOMAINS][SUNXI_RX_SYNC_IDS];
static struct sunxi_rx_sync_cb sunxi_rx_sync_domain[SUNXI_RX_SYNC_DOMAINS];

int sunxi_rx_sync_probe(unsigned int domain)
{
	unsigned long flags;
	int users;

	if (domain >= SUNXI_RX_SYNC_DOMAINS) {
		pr_err("%s: invalid rx sync domain %u\n", __func__, domain);
		return -EINVAL;
	}

	spin_lock_irqsave(&sunxi_rx_sync_lock, flags);
	if (sunxi_rx_sync_users[domain] >= SUNXI_RX_SYNC_IDS) {
		pr_err("%s: domain %u already has %d users\n", __func__, domain,
		       sunxi_rx_sync_users[domain]);
		spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);
		return -EINVAL;
	}
	users = sunxi_rx_sync_users[domain]++;
	spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);

	/* The vendor returns the number of users before this one. */
	return users;
}
EXPORT_SYMBOL_GPL(sunxi_rx_sync_probe);

void sunxi_rx_sync_remove(unsigned int domain)
{
	unsigned long flags;

	if (domain >= SUNXI_RX_SYNC_DOMAINS) {
		pr_err("%s: invalid rx sync domain %u\n", __func__, domain);
		return;
	}

	spin_lock_irqsave(&sunxi_rx_sync_lock, flags);
	if (sunxi_rx_sync_users[domain] > 0)
		sunxi_rx_sync_users[domain]--;
	spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);
}
EXPORT_SYMBOL_GPL(sunxi_rx_sync_remove);

int sunxi_rx_sync_register_cb(unsigned int domain, unsigned int id,
			      sunxi_rx_sync_cb_t cb, void *data)
{
	unsigned long flags;

	if (domain >= SUNXI_RX_SYNC_DOMAINS || id >= SUNXI_RX_SYNC_IDS) {
		pr_err("%s: invalid domain %u id %u\n", __func__, domain, id);
		return -EINVAL;
	}
	if (!cb || !data)
		return -EINVAL;

	spin_lock_irqsave(&sunxi_rx_sync_lock, flags);
	sunxi_rx_sync_ids[domain][id].cb = cb;
	sunxi_rx_sync_ids[domain][id].data = data;
	spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_rx_sync_register_cb);

void sunxi_rx_sync_unregister_cb(unsigned int domain, unsigned int id)
{
	unsigned long flags;

	if (domain >= SUNXI_RX_SYNC_DOMAINS || id >= SUNXI_RX_SYNC_IDS) {
		pr_err("%s: invalid domain %u id %u\n", __func__, domain, id);
		return;
	}

	spin_lock_irqsave(&sunxi_rx_sync_lock, flags);
	sunxi_rx_sync_ids[domain][id].cb = NULL;
	sunxi_rx_sync_ids[domain][id].data = NULL;
	spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);
}
EXPORT_SYMBOL_GPL(sunxi_rx_sync_unregister_cb);

int sunxi_rx_sync_register_domain_cb(unsigned int domain,
				     sunxi_rx_sync_cb_t cb, void *data)
{
	unsigned long flags;

	if (domain >= SUNXI_RX_SYNC_DOMAINS) {
		pr_err("%s: invalid rx sync domain %u\n", __func__, domain);
		return -EINVAL;
	}
	if (!cb || !data)
		return -EINVAL;

	spin_lock_irqsave(&sunxi_rx_sync_lock, flags);
	sunxi_rx_sync_domain[domain].cb = cb;
	sunxi_rx_sync_domain[domain].data = data;
	spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_rx_sync_register_domain_cb);

void sunxi_rx_sync_unregister_domain_cb(unsigned int domain)
{
	unsigned long flags;

	if (domain >= SUNXI_RX_SYNC_DOMAINS)
		return;

	spin_lock_irqsave(&sunxi_rx_sync_lock, flags);
	sunxi_rx_sync_domain[domain].cb = NULL;
	sunxi_rx_sync_domain[domain].data = NULL;
	spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);
}
EXPORT_SYMBOL_GPL(sunxi_rx_sync_unregister_domain_cb);

void sunxi_rx_sync_startup(unsigned int domain, unsigned int id)
{
	unsigned long flags;

	if (domain >= SUNXI_RX_SYNC_DOMAINS || id >= SUNXI_RX_SYNC_IDS) {
		pr_err("%s: invalid domain %u id %u\n", __func__, domain, id);
		return;
	}

	spin_lock_irqsave(&sunxi_rx_sync_lock, flags);
	sunxi_rx_sync_started[domain]++;
	spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);
}
EXPORT_SYMBOL_GPL(sunxi_rx_sync_startup);

void sunxi_rx_sync_shutdown(unsigned int domain, unsigned int id)
{
	unsigned long flags;

	if (domain >= SUNXI_RX_SYNC_DOMAINS || id >= SUNXI_RX_SYNC_IDS) {
		pr_err("%s: invalid domain %u id %u\n", __func__, domain, id);
		return;
	}

	spin_lock_irqsave(&sunxi_rx_sync_lock, flags);
	if (sunxi_rx_sync_started[domain] > 0)
		sunxi_rx_sync_started[domain]--;
	spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);
}
EXPORT_SYMBOL_GPL(sunxi_rx_sync_shutdown);

/*
 * Called when a stream in the domain is enabled or disabled at the control
 * level.  The per-id callback for this stream fires with the new state; the
 * domain callback fires once when the first stream of the domain starts and
 * once when the last one stops, which is the point the shared RX clock can be
 * turned on or off.  The vendor takes rx_sync_lock around the callbacks, so
 * they must not sleep - the same contract applies here.
 */
void sunxi_rx_sync_control(unsigned int domain, unsigned int id, int on)
{
	struct sunxi_rx_sync_cb id_cb, domain_cb;
	bool fire_domain = false;
	int state;
	unsigned long flags;

	if (domain >= SUNXI_RX_SYNC_DOMAINS || id >= SUNXI_RX_SYNC_IDS) {
		pr_err("%s: invalid domain %u id %u\n", __func__, domain, id);
		return;
	}

	spin_lock_irqsave(&sunxi_rx_sync_lock, flags);

	if (on) {
		sunxi_rx_sync_running[domain]++;
		if (sunxi_rx_sync_running[domain] == 1)
			fire_domain = true;
		if (sunxi_rx_sync_running[domain] > sunxi_rx_sync_started[domain])
			pr_warn("%s: domain %u running %d > started %d\n",
				__func__, domain, sunxi_rx_sync_running[domain],
				sunxi_rx_sync_started[domain]);
		state = 1;
	} else {
		sunxi_rx_sync_running[domain]--;
		if (sunxi_rx_sync_running[domain] < 0) {
			pr_err("%s: domain %u unbalanced off\n", __func__, domain);
			sunxi_rx_sync_running[domain] = 0;
		}
		if (sunxi_rx_sync_running[domain] == 0)
			fire_domain = true;
		state = 0;
	}

	id_cb = sunxi_rx_sync_ids[domain][id];
	if (fire_domain)
		domain_cb = sunxi_rx_sync_domain[domain];

	spin_unlock_irqrestore(&sunxi_rx_sync_lock, flags);

	if (id_cb.cb)
		id_cb.cb(id_cb.data, state);
	if (fire_domain && domain_cb.cb)
		domain_cb.cb(domain_cb.data, state);
}
EXPORT_SYMBOL_GPL(sunxi_rx_sync_control);

int snd_sunxi_card_jack_new(struct snd_soc_card *card, const char *id,
			    int mask, struct snd_soc_jack *jack)
{
	int ret;

	if (!card || !jack)
		return -EINVAL;

	/* Pins are registered later by the codec's set_jack callback. */
	ret = snd_soc_card_jack_new(card, id, mask, jack, NULL, 0);
	if (ret)
		return ret;

	/*
	 * Vendor key map from snd_sunxi_jack_codec_register():
	 * BTN_0 media, BTN_1 volume up, BTN_2 volume down, BTN_3 voice.
	 * snd_jack_set_key() fails for a type the jack does not claim, so a
	 * headphone-only jack simply ends up with no keys.
	 */
	if (mask & SND_JACK_BTN_0)
		snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_MEDIA);
	if (mask & SND_JACK_BTN_1)
		snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOLUMEUP);
	if (mask & SND_JACK_BTN_2)
		snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEDOWN);
	if (mask & SND_JACK_BTN_3)
		snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOICECOMMAND);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_sunxi_card_jack_new);

MODULE_DESCRIPTION("Allwinner sun50iw15 sound common helpers");
MODULE_LICENSE("GPL");
