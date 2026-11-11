// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/*
 * mcu_comm_dev - HiDTVPro MCU communication driver.
 *
 * Rewritten from the vendor mcu_comm_dev.ko decompile.  The vendor module
 * carries the whole MCU protocol (transmit/receive/package rings of 140-byte
 * entries) and talks to the arisc core over the msgbox rpmsg channel with two
 * exports supplied by vmlinux:
 *
 *   sunxi_arisc_rpm_register_message_callback(cb)
 *   sunxi_arisc_rpm_send(channel, len, data)
 *
 * Our kernel has no msgbox/rpmsg stack, so this module also carries the arisc
 * transport and exports both symbols, which lets the stock mcu_comm_dev.ko
 * resolve against it as well.
 *
 * Wire format (vendor): 8-byte header {0xa5, seq, channel, len, 0,0,0,0}
 * followed by the payload.  The receive path strips the header and hands
 * (channel, len, payload) to the registered callback.
 *
 * Userspace sees /dev/mcu_comm with 132-byte messages: byte 0 is the length,
 * data starts at offset 4.  Ioctl 0xC0845F01 sends one message, 0xC0845F02
 * receives one.
 *
 * Msgbox hardware, from h726_clean/drivers/arisc_msgbox.c:
 *   - each amp has a 0x100-byte block at amp_base[amp]
 *   - RX pending / ack   : +0x20 / +0x24, bit (channel << 1)
 *   - TX pending / ack   : +0x30 / +0x34, bit ((channel << 1) | 1)
 *   - per-channel FIFO   : +0x60 (word count, full at 8), +0x70 (data word)
 *   - channel re-arm     : +0x80 (written 0 on resume)
 * The vendor moves messages in 8-word (32-byte) transactions: the sender sets
 * the TX bit, the IRQ fills the FIFO with the next 8 words and clears it, and
 * the receiver drains the FIFO and acknowledges through +0x24.
 */
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define MCU_MSG_SIZE		132
#define MCU_MAX_PAYLOAD		128
#define MCU_TRANS_COUNT		20
#define MCU_RECV_COUNT		20
#define MCU_PACK_COUNT		16
#define MCU_RX_STASH		8

#define MCU_CMD_SEND		0xC0845F01	/* -1065066751 */
#define MCU_CMD_RECV		0xC0845F02	/* -1065066750 */

#define ARISC_HDR_MAGIC		0xa5
#define ARISC_HDR_LEN		8
#define MSGBOX_XFER_WORDS	8
#define MSGBOX_XFER_BYTES	(MSGBOX_XFER_WORDS * 4)

#define MSGBOX_AMP_STRIDE	0x100
#define MSGBOX_RX_PEND		0x20
#define MSGBOX_RX_ACK		0x24
#define MSGBOX_TX_PEND		0x30
#define MSGBOX_TX_ACK		0x34
#define MSGBOX_FIFO_CNT		0x60
#define MSGBOX_FIFO_DATA	0x70
#define MSGBOX_CH_RESET		0x80

#define TX_TIMEOUT_MS		1500

struct mcu_entry {
	u8 buf[MCU_MSG_SIZE];
	struct list_head node;
};

struct mcu_stash {
	u8 len;
	u8 data[MCU_MAX_PAYLOAD];
};

struct mcu_tx {
	u8 buf[ARISC_HDR_LEN + MCU_MAX_PAYLOAD];
	size_t len;
	size_t pos;
	bool active;
	struct completion done;
};

struct mcu_dev {
	struct device *dev;
	struct mutex lock;		/* rings, stash and tx state */
	wait_queue_head_t readq;

	/* rings */
	struct mcu_entry trans[MCU_TRANS_COUNT];
	struct mcu_entry recv[MCU_RECV_COUNT];
	struct mcu_entry pack[MCU_PACK_COUNT];
	struct list_head trans_free;
	struct list_head recv_free, recv_new;
	struct list_head pack_free, pack_new;

	struct mcu_stash stash[MCU_RX_STASH];
	unsigned int stash_head, stash_tail;
	spinlock_t stash_lock;

	struct workqueue_struct *wq;
	struct work_struct rx_work;

	int users;

	/* arisc transport */
	struct clk *clk;
	struct reset_control *rst;
	void __iomem *amp_base[2];
	u32 amp_local;
	u32 rx_ch;
	u32 tx_ch;
	int irq[2];
	int nirq;
	u8 tx_seq;
	struct mcu_tx tx;
	spinlock_t tx_lock;
	void (*rx_cb)(int channel, u32 len, void *data);
};

static struct mcu_dev *mcu;
static dev_t mcu_devt;
static bool mcu_devt_valid;
static struct class *mcu_class;

/* ------------------------------------------------------------------ */
/* msgbox register accessors                                           */
/* ------------------------------------------------------------------ */

static void __iomem *amp_reg(struct mcu_dev *m, u32 amp)
{
	if (amp >= ARRAY_SIZE(m->amp_base) || !m->amp_base[amp])
		return NULL;
	return m->amp_base[amp] + amp * MSGBOX_AMP_STRIDE;
}

static void __iomem *chan_fifo(struct mcu_dev *m, u32 ch)
{
	void __iomem *base = amp_reg(m, m->amp_local);

	return base ? base + ch * 4 : NULL;
}

/*
 * Hand the next 8 words of the queued message to the hardware and drop the TX
 * request bit, as the vendor IRQ does.  Returns true when the whole message
 * has been written.
 */
static bool mcu_tx_pump(struct mcu_dev *m)
{
	void __iomem *base = amp_reg(m, m->amp_local);
	void __iomem *fifo = chan_fifo(m, m->tx_ch);
	unsigned long flags;
	u32 bit = BIT((m->tx_ch << 1) | 1);
	bool done = false;
	int i;

	if (!base || !fifo)
		return true;

	spin_lock_irqsave(&m->tx_lock, flags);
	if (m->tx.active) {
		for (i = 0; i < MSGBOX_XFER_WORDS; i++) {
			u32 word = 0;
			size_t n = min_t(size_t, 4, m->tx.len - m->tx.pos);

			if (n)
				memcpy(&word, m->tx.buf + m->tx.pos, n);
			writel(word, fifo + MSGBOX_FIFO_DATA);
			m->tx.pos += n;
		}

		writel(readl(base + MSGBOX_TX_PEND) & ~bit,
		       base + MSGBOX_TX_PEND);

		if (m->tx.pos >= m->tx.len) {
			m->tx.active = false;
			done = true;
		} else {
			/* ask for the next transaction */
			writel(readl(base + MSGBOX_TX_PEND) | bit,
			       base + MSGBOX_TX_PEND);
		}
	}
	spin_unlock_irqrestore(&m->tx_lock, flags);

	if (done)
		complete(&m->tx.done);
	return done;
}

/* ------------------------------------------------------------------ */
/* receive path                                                        */
/* ------------------------------------------------------------------ */

static void mcu_msgbox_rx(struct mcu_dev *m)
{
	void __iomem *base = amp_reg(m, m->amp_local);
	void __iomem *fifo = chan_fifo(m, m->rx_ch);
	u32 bit = BIT(m->rx_ch << 1);
	struct mcu_stash msg;
	unsigned long flags;
	int i;

	if (!base || !fifo)
		return;

	if (!(readl(base + MSGBOX_RX_PEND) & bit))
		return;
	/* the vendor waits for the ack bit before touching the FIFO */
	if (!(readl(base + MSGBOX_RX_ACK) & bit))
		return;

	memset(&msg, 0, sizeof(msg));
	for (i = 0; i < MSGBOX_XFER_WORDS; i++) {
		u32 word = readl(fifo + MSGBOX_FIFO_DATA);

		memcpy(&msg.data[i * 4], &word, 4);
	}

	/* release the transaction back to the remote core */
	writel(bit, base + MSGBOX_RX_ACK);

	if (msg.data[0] != ARISC_HDR_MAGIC)
		return;

	msg.len = msg.data[3];
	if (msg.len > MCU_MAX_PAYLOAD)
		return;

	spin_lock_irqsave(&m->stash_lock, flags);
	if (((m->stash_head + 1) % MCU_RX_STASH) != m->stash_tail) {
		struct mcu_stash *slot = &m->stash[m->stash_head];

		slot->len = msg.len;
		memcpy(slot->data, msg.data + ARISC_HDR_LEN, msg.len);
		m->stash_head = (m->stash_head + 1) % MCU_RX_STASH;
	}
	spin_unlock_irqrestore(&m->stash_lock, flags);

	queue_work(m->wq, &m->rx_work);
}

static irqreturn_t mcu_msgbox_isr(int irq, void *data)
{
	struct mcu_dev *m = data;

	mcu_msgbox_rx(m);
	mcu_tx_pump(m);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* arisc exports                                                       */
/* ------------------------------------------------------------------ */

int sunxi_arisc_rpm_register_message_callback(void (*cb)(int, u32, void *))
{
	if (!mcu)
		return -ENODEV;

	mcu->rx_cb = cb;
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_arisc_rpm_register_message_callback);

/*
 * Messages longer than one msgbox transaction are pushed one 8-word chunk at a
 * time; each chunk re-raises the TX request bit and waits for the IRQ to have
 * drained it.  The vendor layer gets the same effect from rpmsg, which we do
 * not carry, so the per-chunk rpmsg framing is absent here.
 */
u32 sunxi_arisc_rpm_send(u8 channel, size_t len, void *data)
{
	struct mcu_dev *m = mcu;
	void __iomem *base;
	unsigned long flags;
	size_t total = ARISC_HDR_LEN + len;
	size_t sent = 0;
	u32 bit;
	int ret = 0;

	if (!m || !m->amp_base[m->amp_local])
		return (u32)-ENODEV;
	if (!data || len > MCU_MAX_PAYLOAD)
		return (u32)-EINVAL;

	base = amp_reg(m, m->amp_local);
	bit = BIT((m->tx_ch << 1) | 1);

	spin_lock_irqsave(&m->tx_lock, flags);
	m->tx.buf[0] = ARISC_HDR_MAGIC;
	m->tx.buf[1] = m->tx_seq++;
	m->tx.buf[2] = channel;
	m->tx.buf[3] = len;
	memcpy(m->tx.buf + ARISC_HDR_LEN, data, len);
	m->tx.len = total;
	m->tx.pos = 0;
	spin_unlock_irqrestore(&m->tx_lock, flags);

	while (sent < total) {
		reinit_completion(&m->tx.done);

		spin_lock_irqsave(&m->tx_lock, flags);
		m->tx.active = true;
		spin_unlock_irqrestore(&m->tx_lock, flags);

		writel(readl(base + MSGBOX_TX_PEND) | bit,
		       base + MSGBOX_TX_PEND);

		if (!wait_for_completion_timeout(&m->tx.done,
						 msecs_to_jiffies(TX_TIMEOUT_MS))) {
			spin_lock_irqsave(&m->tx_lock, flags);
			m->tx.active = false;
			spin_unlock_irqrestore(&m->tx_lock, flags);
			ret = -ETIMEDOUT;
			break;
		}

		sent += MSGBOX_XFER_BYTES;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sunxi_arisc_rpm_send);

/* ------------------------------------------------------------------ */
/* MCU protocol rings                                                  */
/* ------------------------------------------------------------------ */

static void mcu_rings_init(struct mcu_dev *m)
{
	int i;

	INIT_LIST_HEAD(&m->trans_free);
	INIT_LIST_HEAD(&m->recv_free);
	INIT_LIST_HEAD(&m->recv_new);
	INIT_LIST_HEAD(&m->pack_free);
	INIT_LIST_HEAD(&m->pack_new);

	for (i = 0; i < MCU_TRANS_COUNT; i++)
		list_add_tail(&m->trans[i].node, &m->trans_free);
	for (i = 0; i < MCU_RECV_COUNT; i++)
		list_add_tail(&m->recv[i].node, &m->recv_free);
	for (i = 0; i < MCU_PACK_COUNT; i++)
		list_add_tail(&m->pack[i].node, &m->pack_free);
}

/* caller holds m->lock */
static struct mcu_entry *mcu_get(struct list_head *ring)
{
	struct mcu_entry *e;

	if (list_empty(ring))
		return NULL;

	e = list_first_entry(ring, struct mcu_entry, node);
	list_del(&e->node);
	return e;
}

/* caller holds m->lock */
static void mcu_put(struct mcu_entry *e, struct list_head *ring)
{
	list_add_tail(&e->node, ring);
}

/* vendor rpm_message_cb(): {total=1, index=1, len, payload} */
static void mcu_rpm_cb(int channel, u32 len, void *data)
{
	struct mcu_entry *p;

	mutex_lock(&mcu->lock);
	p = mcu_get(&mcu->pack_free);
	if (p) {
		p->buf[0] = 1;
		p->buf[1] = 1;
		p->buf[2] = min_t(u32, len, MCU_MAX_PAYLOAD);
		memcpy(p->buf + 3, data, p->buf[2]);
		mcu_put(p, &mcu->pack_new);
	}
	mutex_unlock(&mcu->lock);
}

/* vendor RecvPackageAction(): reassemble {total, index, len} packets */
static void mcu_rx_work(struct work_struct *work)
{
	struct mcu_dev *m = container_of(work, struct mcu_dev, rx_work);
	struct mcu_entry *cur = NULL;
	struct mcu_stash item;
	unsigned long flags;
	struct mcu_entry *p;
	bool have;

	for (;;) {
		spin_lock_irqsave(&m->stash_lock, flags);
		have = m->stash_tail != m->stash_head;
		if (have) {
			item = m->stash[m->stash_tail];
			m->stash_tail = (m->stash_tail + 1) % MCU_RX_STASH;
		}
		spin_unlock_irqrestore(&m->stash_lock, flags);

		if (!have)
			break;
		if (m->rx_cb)
			m->rx_cb(0, item.len, item.data);
	}

	mutex_lock(&m->lock);
	while ((p = mcu_get(&m->pack_new))) {
		u8 total = p->buf[0];
		u8 index = p->buf[1];
		u8 len = p->buf[2];
		u32 used;

		if (index == 1) {
			if (cur)
				mcu_put(cur, &m->recv_free);
			cur = mcu_get(&m->recv_free);
			if (!cur) {
				mcu_put(p, &m->pack_free);
				continue;
			}
			memset(cur->buf, 0, sizeof(cur->buf));
		}

		if (cur && len <= MCU_MAX_PAYLOAD) {
			used = *(u32 *)cur->buf;
			if (used + len <= MCU_MAX_PAYLOAD) {
				memcpy(cur->buf + 4 + used, p->buf + 3, len);
				*(u32 *)cur->buf = used + len;
			}
		}

		if (cur && index == total) {
			mcu_put(cur, &m->recv_new);
			cur = NULL;
		}

		mcu_put(p, &m->pack_free);
	}
	mutex_unlock(&m->lock);

	wake_up_interruptible(&m->readq);
}

static bool mcu_recv_ready(struct mcu_dev *m)
{
	bool ready;

	mutex_lock(&m->lock);
	ready = !list_empty(&m->recv_new);
	mutex_unlock(&m->lock);
	return ready;
}

/* ------------------------------------------------------------------ */
/* char device                                                         */
/* ------------------------------------------------------------------ */

static int mcu_comm_open(struct inode *inode, struct file *file)
{
	mutex_lock(&mcu->lock);
	if (!mcu->users++) {
		mcu_rings_init(mcu);
		sunxi_arisc_rpm_register_message_callback(mcu_rpm_cb);
	}
	mutex_unlock(&mcu->lock);

	file->private_data = mcu;
	return nonseekable_open(inode, file);
}

static int mcu_comm_release(struct inode *inode, struct file *file)
{
	mutex_lock(&mcu->lock);
	if (!--mcu->users) {
		mcu->rx_cb = NULL;
		cancel_work_sync(&mcu->rx_work);
	}
	mutex_unlock(&mcu->lock);
	return 0;
}

static ssize_t mcu_comm_read(struct file *file, char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct mcu_dev *m = file->private_data;
	size_t done = 0;

	while (!mcu_recv_ready(m)) {
		int ret;

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(m->readq, mcu_recv_ready(m));
		if (ret)
			return ret;
	}

	while (done < count) {
		struct mcu_entry *e;
		size_t len;

		mutex_lock(&m->lock);
		e = mcu_get(&m->recv_new);
		mutex_unlock(&m->lock);
		if (!e)
			break;

		len = *(u32 *)e->buf;
		if (len > count - done)
			len = count - done;
		if (copy_to_user(buf + done, e->buf + 4, len)) {
			mutex_lock(&m->lock);
			mcu_put(e, &m->recv_free);
			mutex_unlock(&m->lock);
			return done ? done : -EFAULT;
		}
		done += len;

		mutex_lock(&m->lock);
		mcu_put(e, &m->recv_free);
		mutex_unlock(&m->lock);
	}

	return done;
}

static long mcu_comm_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct mcu_dev *m = file->private_data;
	struct mcu_entry *e;
	u8 msg[MCU_MSG_SIZE];

	switch (cmd) {
	case MCU_CMD_SEND:
		mutex_lock(&m->lock);
		e = mcu_get(&m->trans_free);
		mutex_unlock(&m->lock);
		if (!e)
			return -ENOMEM;

		if (copy_from_user(e->buf, (void __user *)arg, sizeof(e->buf))) {
			mutex_lock(&m->lock);
			mcu_put(e, &m->trans_free);
			mutex_unlock(&m->lock);
			return -EFAULT;
		}

		if (sunxi_arisc_rpm_send(0, e->buf[0], e->buf + 4) == (u32)-ETIMEDOUT) {
			mutex_lock(&m->lock);
			mcu_put(e, &m->trans_free);
			mutex_unlock(&m->lock);
			return -ETIMEDOUT;
		}

		mutex_lock(&m->lock);
		mcu_put(e, &m->trans_free);
		mutex_unlock(&m->lock);
		return 0;

	case MCU_CMD_RECV:
		while (!mcu_recv_ready(m)) {
			int ret;

			if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;
			ret = wait_event_interruptible(m->readq,
						       mcu_recv_ready(m));
			if (ret)
				return ret;
		}

		mutex_lock(&m->lock);
		e = mcu_get(&m->recv_new);
		mutex_unlock(&m->lock);
		if (!e)
			return -ENODATA;

		memcpy(msg, e->buf, sizeof(msg));

		mutex_lock(&m->lock);
		mcu_put(e, &m->recv_free);
		mutex_unlock(&m->lock);

		if (copy_to_user((void __user *)arg, msg, sizeof(msg)))
			return -EFAULT;
		return 0;
	default:
		return -ENOTTY;
	}
}

static __poll_t mcu_comm_poll(struct file *file, poll_table *wait)
{
	struct mcu_dev *m = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &m->readq, wait);
	if (mcu_recv_ready(m))
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

static const struct file_operations mcu_comm_fops = {
	.owner		= THIS_MODULE,
	.open		= mcu_comm_open,
	.release	= mcu_comm_release,
	.read		= mcu_comm_read,
	.unlocked_ioctl	= mcu_comm_ioctl,
	.poll		= mcu_comm_poll,
	.llseek		= no_llseek,
};

/* ------------------------------------------------------------------ */
/* probe / remove / PM                                                 */
/* ------------------------------------------------------------------ */

static int mcu_comm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcu_dev *m;
	u32 val;
	int i, ret;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->dev = dev;
	mutex_init(&m->lock);
	spin_lock_init(&m->stash_lock);
	spin_lock_init(&m->tx_lock);
	init_waitqueue_head(&m->readq);
	init_completion(&m->tx.done);
	INIT_WORK(&m->rx_work, mcu_rx_work);

	/* vendor: of_clk_get(np, 0) + clk_prepare_enable() */
	m->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(m->clk)) {
		dev_err(dev, "error get clk\n");
		return PTR_ERR(m->clk);
	}
	ret = clk_prepare_enable(m->clk);
	if (ret) {
		dev_err(dev, "clk_prepare_enable failed: %d\n", ret);
		return ret;
	}

	/* vendor: devm_reset_control_get(dev, NULL) + reset_control_deassert() */
	m->rst = devm_reset_control_get(dev, NULL);
	if (IS_ERR(m->rst)) {
		dev_err(dev, "error get reset control\n");
		clk_disable_unprepare(m->clk);
		return PTR_ERR(m->rst);
	}
	reset_control_deassert(m->rst);

	/* up to two amp register blocks in our DTS (reg 0 and 1) */
	for (i = 0; i < ARRAY_SIZE(m->amp_base); i++) {
		void __iomem *base = devm_platform_ioremap_resource(pdev, i);

		if (IS_ERR(base)) {
			if (i == 0) {
				ret = PTR_ERR(base);
				goto err_clk;
			}
			break;
		}
		m->amp_base[i] = base;
	}

	if (of_property_read_u32(dev->of_node, "msgbox_amp_local", &val))
		val = 0;
	if (val >= ARRAY_SIZE(m->amp_base) || !m->amp_base[val]) {
		dev_err(dev, "amp cur config error\n");
		ret = -EINVAL;
		goto err_clk;
	}
	m->amp_local = val;

	if (of_property_read_u32(dev->of_node, "rpmsg_read_channel-0", &val))
		val = 1;
	m->rx_ch = val;
	if (of_property_read_u32(dev->of_node, "rpmsg_write_channel-0", &val))
		val = 1;
	m->tx_ch = val;
	if (m->rx_ch > 3 || m->tx_ch > 3) {
		dev_err(dev, "rpmsg param config error\n");
		ret = -EINVAL;
		goto err_clk;
	}

	m->nirq = platform_irq_count(pdev);
	if (m->nirq > ARRAY_SIZE(m->irq))
		m->nirq = ARRAY_SIZE(m->irq);
	for (i = 0; i < m->nirq; i++) {
		int irq = platform_get_irq(pdev, i);

		if (irq < 0) {
			ret = irq;
			goto err_clk;
		}
		m->irq[i] = irq;
		ret = devm_request_threaded_irq(dev, irq, NULL, mcu_msgbox_isr,
						IRQF_ONESHOT, dev_name(dev), m);
		if (ret) {
			dev_err(dev, "error request irq\n");
			goto err_clk;
		}
	}

	m->wq = alloc_workqueue("McuCommWQ", WQ_MEM_RECLAIM, 1);
	if (!m->wq) {
		ret = -ENOMEM;
		goto err_clk;
	}

	mcu = m;
	platform_set_drvdata(pdev, m);
	dev_info(dev, "msgbox transport ready, amp %u, rx ch %u, tx ch %u\n",
		 m->amp_local, m->rx_ch, m->tx_ch);
	return 0;

err_clk:
	clk_disable_unprepare(m->clk);
	return ret;
}

static int mcu_comm_remove(struct platform_device *pdev)
{
	struct mcu_dev *m = platform_get_drvdata(pdev);

	if (m->wq) {
		flush_workqueue(m->wq);
		destroy_workqueue(m->wq);
	}
	reset_control_assert(m->rst);
	clk_disable_unprepare(m->clk);
	mcu = NULL;
	return 0;
}

/* vendor sunxi_msgbox_suspend() is a no-op */
static int __maybe_unused mcu_comm_suspend(struct device *dev)
{
	return 0;
}

/*
 * vendor sunxi_msgbox_resume(): clear the per-channel control register and
 * re-arm the receive request bit for every endpoint.
 */
static int __maybe_unused mcu_comm_resume(struct device *dev)
{
	struct mcu_dev *m = dev_get_drvdata(dev);
	void __iomem *base = amp_reg(m, m->amp_local);
	void __iomem *fifo = chan_fifo(m, m->rx_ch);
	u32 bit = BIT(m->rx_ch << 1);

	if (!base || !fifo)
		return 0;

	writel(0, fifo + MSGBOX_CH_RESET);
	wmb();
	writel(readl(base + MSGBOX_RX_PEND) | bit, base + MSGBOX_RX_PEND);

	dev_info(dev, "sunxi apm msgbox driver resume\n");
	return 0;
}

static const struct dev_pm_ops mcu_comm_pm_ops = {
	.suspend	= mcu_comm_suspend,
	.resume		= mcu_comm_resume,
};

static const struct of_device_id mcu_comm_of_match[] = {
	{ .compatible = "sunxi,msgbox-amp" },
	{ .compatible = "allwinner,sunxi-msgbox-amp" },
	{ }
};
MODULE_DEVICE_TABLE(of, mcu_comm_of_match);

static struct platform_driver mcu_comm_driver = {
	.probe	= mcu_comm_probe,
	.remove	= mcu_comm_remove,
	.driver	= {
		.name		= "mcu_comm_dev",
		.of_match_table	= mcu_comm_of_match,
		.pm		= &mcu_comm_pm_ops,
	},
};

static int __init mcu_comm_init(void)
{
	struct device *dev;
	int ret;

	ret = alloc_chrdev_region(&mcu_devt, 0, 1, "mcu_comm");
	if (ret)
		return ret;
	mcu_devt_valid = true;

	mcu_class = class_create(THIS_MODULE, "mcu_comm");
	if (IS_ERR(mcu_class)) {
		unregister_chrdev_region(mcu_devt, 1);
		mcu_devt_valid = false;
		return PTR_ERR(mcu_class);
	}

	dev = device_create(mcu_class, NULL, mcu_devt, NULL, "mcu_comm");
	if (IS_ERR(dev)) {
		class_destroy(mcu_class);
		unregister_chrdev_region(mcu_devt, 1);
		mcu_devt_valid = false;
		return PTR_ERR(dev);
	}

	return platform_driver_register(&mcu_comm_driver);
}
module_init(mcu_comm_init);

static void __exit mcu_comm_exit(void)
{
	platform_driver_unregister(&mcu_comm_driver);
	if (!IS_ERR_OR_NULL(mcu_class)) {
		device_destroy(mcu_class, mcu_devt);
		class_destroy(mcu_class);
	}
	if (mcu_devt_valid)
		unregister_chrdev_region(mcu_devt, 1);
}
module_exit(mcu_comm_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("HiDTVPro MCU communicaton driver");
MODULE_AUTHOR("Jun He (tonyhe@trident.com.cn)");
