// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner boot time log: /proc/bootevent
 *
 * Replacement for the vendor bootevent.ko from the vendor_boot ramdisk.  The
 * vendor module keeps a 960 entry ring of boot milestones and prints them as
 * milliseconds since boot:
 *
 *   ----------------------------------------
 *   1 BOOTEVENT (unit:msec)
 *   ----------------------------------------
 *       21.123456 : ON
 *     1234.567890 :   87-kworker         : some message
 *   ----------------------------------------
 *
 * Writing "0" or "1" to the file switches collection off or on, anything else
 * is appended to the log verbatim (the vendor module does not strip the
 * trailing newline either).
 *
 * The vendor module carries bootevent_initcall()/bootevent_probe() entry
 * points that nothing registers, so its ring stays empty unless userspace
 * writes to it.  This version hooks the in-tree initcall_start/initcall_finish
 * tracepoints instead, and exports sunxi_bootevent_log() so that drivers in
 * this tree can record their own milestones.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/math64.h>
#include <linux/tracepoint.h>
#include <trace/events/initcall.h>

#define SUNXI_BOOTEVENT_MAX	960
#define SUNXI_BOOTEVENT_MSG	128

struct sunxi_bootevent_entry {
	u64 ts;
	int pid;
	char comm[TASK_COMM_LEN];
	char msg[SUNXI_BOOTEVENT_MSG];
};

static struct sunxi_bootevent_entry *bootevent_log;
static unsigned int bootevent_count;
static bool bootevent_enabled;
static u64 bootevent_on_ts;
static u64 bootevent_off_ts;
static DEFINE_MUTEX(bootevent_lock);

static DEFINE_PER_CPU(initcall_t, bootevent_cur_fn);
static DEFINE_PER_CPU(u64, bootevent_cur_ts);

/**
 * sunxi_bootevent_log() - append one milestone to the boot log
 * @fmt: printf style format
 *
 * Does nothing while collection is switched off.
 */
void sunxi_bootevent_log(const char *fmt, ...)
{
	struct sunxi_bootevent_entry *e;
	va_list args;

	if (!READ_ONCE(bootevent_enabled))
		return;

	mutex_lock(&bootevent_lock);

	if (bootevent_count >= SUNXI_BOOTEVENT_MAX) {
		mutex_unlock(&bootevent_lock);
		return;
	}

	e = &bootevent_log[bootevent_count];
	e->ts = sched_clock();
	e->pid = current->pid;
	memcpy(e->comm, current->comm, TASK_COMM_LEN);

	va_start(args, fmt);
	vsnprintf(e->msg, sizeof(e->msg), fmt, args);
	va_end(args);

	bootevent_count++;

	mutex_unlock(&bootevent_lock);
}
EXPORT_SYMBOL_GPL(sunxi_bootevent_log);

static void bootevent_switch(bool on)
{
	mutex_lock(&bootevent_lock);

	if (bootevent_enabled != on) {
		pr_info("[BOOTEVENT] %s\n", on ? "on" : "off");
		bootevent_enabled = on;
		if (on)
			bootevent_on_ts = sched_clock();
		else
			bootevent_off_ts = sched_clock();
	}

	mutex_unlock(&bootevent_lock);
}

static void bootevent_initcall_start_cb(void *data, initcall_t fn)
{
	this_cpu_write(bootevent_cur_fn, fn);
	this_cpu_write(bootevent_cur_ts, sched_clock());
}

static void bootevent_initcall_finish_cb(void *data, initcall_t fn, int ret)
{
	u64 start = this_cpu_read(bootevent_cur_ts);
	u64 ns;
	u32 rem;

	/* nested initcalls are not expected, but do not log a bogus duration */
	if (this_cpu_read(bootevent_cur_fn) != fn)
		return;

	ns = sched_clock() - start;

	sunxi_bootevent_log("initcall: %ps %5llu.%06ums", (void *)fn,
			    div_u64_rem(ns, 1000000, &rem), rem);
}

static void bootevent_print_ts(struct seq_file *m, u64 ns, const char *state)
{
	u32 rem;

	seq_printf(m, "%10llu.%06u : %s\n", div_u64_rem(ns, 1000000, &rem),
		   rem, state);
}

static int bootevent_show(struct seq_file *m, void *v)
{
	unsigned int i;

	seq_puts(m, "----------------------------------------\n");
	seq_printf(m, "%d BOOTEVENT (unit:msec)\n", bootevent_enabled ? 1 : 0);
	seq_puts(m, "----------------------------------------\n");

	if (bootevent_on_ts)
		bootevent_print_ts(m, bootevent_on_ts, "ON");

	mutex_lock(&bootevent_lock);
	for (i = 0; i < bootevent_count; i++) {
		const struct sunxi_bootevent_entry *e = &bootevent_log[i];
		u32 rem;
		u64 ms = div_u64_rem(e->ts, 1000000, &rem);

		seq_printf(m, "%10llu.%06u :%5d-%-16s: %s\n",
			   ms, rem, e->pid, e->comm, e->msg);
	}
	mutex_unlock(&bootevent_lock);

	if (bootevent_off_ts)
		bootevent_print_ts(m, bootevent_off_ts, "OFF");

	seq_puts(m, "----------------------------------------\n");

	return 0;
}

static int bootevent_open(struct inode *inode, struct file *file)
{
	return single_open(file, bootevent_show, NULL);
}

static ssize_t bootevent_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	char tmp[256];

	if (count >= sizeof(tmp))
		count = sizeof(tmp) - 1;

	if (copy_from_user(tmp, buf, count))
		return -EFAULT;

	tmp[count] = '\0';

	/* the vendor module only toggles on a bare single character */
	if (count == 1 && tmp[0] == '1') {
		bootevent_switch(true);
		return 1;
	}

	if (count == 1 && tmp[0] == '0') {
		bootevent_switch(false);
		return 1;
	}

	sunxi_bootevent_log("%s", tmp);

	return count;
}

static const struct proc_ops bootevent_fops = {
	.proc_open	= bootevent_open,
	.proc_read	= seq_read,
	.proc_write	= bootevent_write,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int __init bootevent_init(void)
{
	int ret;

	bootevent_log = kvcalloc(SUNXI_BOOTEVENT_MAX, sizeof(*bootevent_log),
				 GFP_KERNEL);
	if (!bootevent_log)
		return -ENOMEM;

	bootevent_switch(true);

	ret = register_trace_initcall_start(bootevent_initcall_start_cb, NULL);
	if (ret)
		pr_warn("[BOOTEVENT] cannot hook initcall start: %d\n", ret);

	ret = register_trace_initcall_finish(bootevent_initcall_finish_cb, NULL);
	if (ret)
		pr_warn("[BOOTEVENT] cannot hook initcall finish: %d\n", ret);

	if (!proc_create("bootevent", 0664, NULL, &bootevent_fops))
		return -ENOMEM;

	sunxi_bootevent_log("bootevent: ready");

	return 0;
}

static void __exit bootevent_exit(void)
{
	unregister_trace_initcall_finish(bootevent_initcall_finish_cb, NULL);
	unregister_trace_initcall_start(bootevent_initcall_start_cb, NULL);
	proc_remove(NULL);
	kvfree(bootevent_log);
	bootevent_log = NULL;
}

module_init(bootevent_init);
module_exit(bootevent_exit);

MODULE_DESCRIPTION("Allwinner boot time log (/proc/bootevent)");
MODULE_LICENSE("GPL");
