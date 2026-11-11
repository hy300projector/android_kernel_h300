/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Allwinner H726 (sun50iw15p1) tvpanel driver - DSI/DPHY/LVDS/TCON.
 *
 * The structures in the "vendor ABI" section below are bit-exact copies of
 * the types recovered from the vendor module's .BTF (tvpanel.ko, firmware
 * build 5.15.167-g192ae9e8004a-dirty).  They are kept exact because the
 * panel driver modules (panel_lvds_gen, panel_dsi_gen, z20hd720m) implement
 * ->get_extra_para()/->prepare() etc. against them, so a differently-laid-out
 * struct would silently corrupt para data across the module boundary.
 */
#ifndef __TVPANEL_H
#define __TVPANEL_H

#include <linux/atomic.h>
#include <linux/backlight.h>
#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/types.h>
#include <video/display_timing.h>

struct tvpanel_connector;
struct tvpanel_panel;

#define TVPANEL_MAX_DSI		2
/* dsi_set_reg_base() in the vendor binary maps the D-PHY one page above the
 * DSI controller: dphy = base + 0x1000.  Matches the DT reg size of 0x1fff.
 */
#define TVPANEL_DPHY_OFFSET	0x1000

/* Values of disp_panel_para.lcd_dsi_if seen in dsi_open(): 1 means the
 * controller runs in video mode and dsi_open() does nothing.
 */
#define TVPANEL_DSI_IF_VIDEO	1

/* disp_panel_para.lcd_dsi_clk_mode */
#define TVPANEL_DSI_CLK_CONTINUOUS	0
#define TVPANEL_DSI_CLK_NON_CONTINUOUS	1

/*
 * connector_type.  The vendor's tvpanel_connector_bind() call sites pass the
 * literal 1 for the LVDS connector and 2 for DSI, so the enum starts at 1.
 */
#define TVPANEL_CONN_LVDS	1
#define TVPANEL_CONN_DSI	2

/* ------------------------------------------------------------------ */
/* vendor ABI - do not reorder                                         */
/* ------------------------------------------------------------------ */

/*
 * 168 bytes, 42 u32 members.  The vendor type uses enums here; they are all
 * 4-byte ints in the vendor ABI, so u32 keeps the layout identical without
 * needing the enum definitions (which the module .BTF does not carry).
 */
struct disp_panel_para {
	u32 lcd_if;			/*   0 */
	u32 lvds_if;			/*   4 */
	u32 lvds_bit_depth;		/*   8 */
	u32 lvds_mode;			/*  12 */
	u32 lcd_lvds_de_invert;		/*  16 */
	u32 lcd_lvds_hs_invert;		/*  20 */
	u32 lcd_lvds_vs_invert;		/*  24 */
	u32 lcd_lvds_clk_invert;	/*  28 */
	u32 lcd_lvds_channel;		/*  32 */
	u32 lcd_lvds_diff_amplifier_lv;	/*  36 */
	u32 lcd_lvds_common_amplifier_lv; /* 40 */
	u32 lvds0_lane_np_inv;		/*  44 */
	u32 lvds1_lane_np_inv;		/*  48 */
	u32 lvds_even_odd_dir;		/*  52 */
	u32 lvds_data_lane_num;		/*  56 */
	u32 lcd_dsi_dual_dsi;		/*  60 */
	u32 lcd_dsi_if;			/*  64 */
	u32 lcd_dsi_lane;		/*  68 */
	u32 lcd_dsi_format;		/*  72 */
	u32 lcd_dsi_eotp;		/*  76 */
	u32 lcd_dsi_clk_mode;		/*  80 */
	u32 hs_trail_set;		/*  84 */
	u32 lcd_dclk_freq;		/*  88 */
	u32 lcd_x;			/*  92 */
	u32 lcd_y;			/*  96 */
	u32 lcd_vt;			/* 100 */
	u32 lcd_ht;			/* 104 */
	u32 lcd_vbp;			/* 108 */
	u32 lcd_vfp;			/* 112 */
	u32 lcd_hbp;			/* 116 */
	u32 lcd_hfp;			/* 120 */
	u32 lcd_vspw;			/* 124 */
	u32 lcd_hspw;			/* 128 */
	u32 ssc_en;			/* 132 */
	u32 ssc_percent;		/* 136 */
	u32 ssc_mode;			/* 140 */
	u32 ssc_freq;			/* 144 */
	u32 var_refresh_rate_en;	/* 148 */
	u32 power_on_delay0;		/* 152 */
	u32 power_on_delay1;		/* 156 */
	u32 power_down_delay0;		/* 160 */
	u32 power_down_delay1;		/* 164 */
};
static_assert(sizeof(struct disp_panel_para) == 168);
static_assert(offsetof(struct disp_panel_para, lcd_dsi_if) == 64);
static_assert(offsetof(struct disp_panel_para, lcd_dsi_clk_mode) == 80);
static_assert(offsetof(struct disp_panel_para, lcd_x) == 92);

/* 32 bytes, 8 members */
struct tvpanel_panel_funcs {
	int (*unprepare)(struct tvpanel_panel *panel);
	int (*prepare)(struct tvpanel_panel *panel);
	int (*enable)(struct tvpanel_panel *panel);
	int (*disable)(struct tvpanel_panel *panel);
	int (*get_native_timing)(struct tvpanel_panel *panel,
				 struct display_timing **timing);
	int (*get_timings)(struct tvpanel_panel *panel, unsigned int num,
			   struct display_timings *timings);
	/* struct drm_dsc_config * in the vendor build */
	int (*get_dsc_para)(struct tvpanel_panel *panel, void *dsc);
	int (*get_extra_para)(struct tvpanel_panel *panel,
			      struct disp_panel_para *para);
};
static_assert(sizeof(struct tvpanel_panel_funcs) == 32);

/* 28 bytes, 7 members */
struct tvpanel_panel {
	struct device *dev;
	const struct tvpanel_panel_funcs *funcs;
	struct backlight_device *backlight;
	struct list_head list;
	int connector_type;
	bool prepared;
	bool enabled;
};
static_assert(sizeof(struct tvpanel_panel) == 28);

/*
 * Timing container the vendor passes to display_timing_to_panel_para().
 *
 * Its word layout is NOT mainline 'struct display_timing' order: the vendor
 * struct is pixelclock, hactive, hfront_porch, hback_porch, hsync_len,
 * vactive, vfront_porch, vback_porch, vsync_len.  This is derived from the
 * member indices in both conversion functions in the vendor binary, e.g.
 *   lcd_hfp <- a1[7], lcd_hbp <- a1[10], lcd_hspw <- a1[13], lcd_y <- a1[16]
 * which with 3-u32 timing_entry members puts hfp/hbp/hsync before vactive.
 * Both structures are 112 bytes, so the compiler cannot catch a mix-up:
 * use tvpanel_timing_to_para(), never a bare cast.
 */
struct tvpanel_timing {
	struct timing_entry pixelclock;
	struct timing_entry hactive;
	struct timing_entry hfront_porch;
	struct timing_entry hback_porch;
	struct timing_entry hsync_len;
	struct timing_entry vactive;
	struct timing_entry vfront_porch;
	struct timing_entry vback_porch;
	struct timing_entry vsync_len;
	u32 flags;
};
static_assert(sizeof(struct timing_entry) == 12);
static_assert(sizeof(struct tvpanel_timing) == 112);
static_assert(offsetof(struct tvpanel_timing, vactive) == 60);

/* 36 bytes, 9 members */
struct tvpanel_connector_funcs {
	int (*detect)(struct tvpanel_connector *conn);
	int (*get_timing)(struct tvpanel_connector *conn,
			  struct display_timing *timing);
	int (*get_edid)(struct tvpanel_connector *conn);
	int (*enable)(struct tvpanel_connector *conn);
	int (*disable)(struct tvpanel_connector *conn);
	int (*prepare)(struct tvpanel_connector *conn);
	int (*unprepare)(struct tvpanel_connector *conn);
	int (*sw_enable)(struct tvpanel_connector *conn);
	int (*adjust_refresh_rate)(struct tvpanel_connector *conn,
				   unsigned int rate, unsigned int a,
				   unsigned int b);
};
static_assert(sizeof(struct tvpanel_connector_funcs) == 36);

/* 156 bytes, 11 members */
struct tvpanel_connector {
	struct tvpanel_device *tpdev;
	struct device *dev;
	struct tvpanel_panel *panel;
	struct list_head head;
	const struct tvpanel_connector_funcs *funcs;
	int connector_type;
	int connector_id;
	struct display_timing timing;	/* 112 bytes */
	unsigned int htotal;
	unsigned int vtotal;
	bool is_enabled;
};
static_assert(sizeof(struct tvpanel_connector) == 156);
static_assert(offsetof(struct tvpanel_connector, timing) == 32);
static_assert(offsetof(struct tvpanel_connector, is_enabled) == 152);

/* 24 bytes, 4 members */
struct tvpanel_device {
	struct device *dev;
	struct list_head connector_list;
	int num_connector;
	struct list_head display_list;
};
static_assert(sizeof(struct tvpanel_device) == 24);

/* ------------------------------------------------------------------ */
/* driver-private structures                                           */
/* ------------------------------------------------------------------ */

/* 12 bytes: the three D-PHY registers the DSI core caches per connector */
struct combodphy_config {
	u32 dphy_tx_time0;
	u32 dphy_ana0;
	u32 dphy_ana4;
};
static_assert(sizeof(struct combodphy_config) == 12);

/* 16 bytes, matches the vendor struct dsi_data */
struct tvpanel_dsi_data {
	int id;
	struct combodphy_config phy_config;
};
static_assert(sizeof(struct tvpanel_dsi_data) == 16);

struct tvpanel_dsi {
	struct tvpanel_connector conn;	/* must stay first */
	bool bound;
	bool enabled;
	struct device *dev;
	struct device *tcon_top;
	struct device *tcon;
	unsigned int dsi_id;
	int irq;
	unsigned int irq_count;
	unsigned int err_count;
	struct disp_panel_para para;
	atomic_t clk_enabled;
	struct clk *clks[5];
	struct reset_control *rsts[5];
	unsigned int clk_freq[5];
	const struct tvpanel_dsi_data *dsi_data;
	void __iomem *res_base;
};

/* ------------------------------------------------------------------ */
/* LVDS instance and register layer (tvpanel_lvds.c)                   */
/* ------------------------------------------------------------------ */

/* the two sub-blocks that follow the core window (vendor lvds_set_regbase) */
#define TVPANEL_LVDS_BUF_OFFSET		0x80000
#define TVPANEL_LVDS_WM_TOP_OFFSET	0xc0000

/* device-tree match data: the LVDS instance id, as in the vendor module */
struct tvpanel_lvds_data {
	int id;
};

struct tvpanel_lvds {
	struct tvpanel_connector conn;	/* must stay first, as for tvpanel_dsi */
	bool bound;
	bool enabled;
	struct device *dev;
	struct device *tcon_top;
	struct device *tcon;
	unsigned int lvds_id;
	int irq;
	struct disp_panel_para para;
	atomic_t clk_enabled;
	struct clk *clks[4];
	struct reset_control *rsts[4];
	unsigned int clk_freq[4];
	const struct tvpanel_lvds_data *lvds_data;
	void __iomem *base;
};

int lvds_set_regbase(void __iomem *base);
int lvds_cfg(const struct disp_panel_para *para);
int lvds_test_pattern(unsigned int enable, unsigned int pattern);
int lvds_combphy_open(unsigned int id);
int lvds_combphy_close(unsigned int id);
int sunxi_tvpanel_lvds_add(struct device *dev);

/* the vendor's LVDS connector export set */
int sunxi_lvds_connector_prepare(struct tvpanel_connector *conn);
int sunxi_lvds_connector_unprepare(struct tvpanel_connector *conn);
int sunxi_lvds_connector_enable(struct tvpanel_connector *conn);
int sunxi_lvds_connector_disable(struct tvpanel_connector *conn);
int sunxi_lvds_connector_vrr(struct tvpanel_connector *conn,
			     unsigned int rate, unsigned int htotal,
			     unsigned int vtotal);
int sunxi_lvds_get_timing(struct tvpanel_connector *conn,
			  struct display_timing *dt);
int sunxi_lvds_connector_sw_enable(struct tvpanel_connector *conn);

extern struct platform_driver tvpanel_lvds_driver;
extern const struct tvpanel_connector_funcs tvpanel_lvds_conn_funcs;

/* display-top / TCON / panel lookups used by the LVDS driver */
struct device *tvpanel_get_tcon_top(void);
struct device *tvpanel_get_tcon(void);
struct tvpanel_panel *tvpanel_get_panel_dev(struct device *dev);
int tvdisp_top_panel_sys_enable(unsigned int enable,
				const struct disp_panel_para *para);
int tvdisp_lvds_phy_setting(unsigned int enable,
			    const struct disp_panel_para *para);

/* ------------------------------------------------------------------ */
/* register layer (tvpanel_dsi.c)                                      */
/* ------------------------------------------------------------------ */

int tvpanel_dsi_set_reg_base(unsigned int id, void __iomem *base,
			     void __iomem *res_base);
void __iomem *tvpanel_dsi_get_reg_base(unsigned int id);
void __iomem *tvpanel_dsi_get_dphy_base(unsigned int id);
void tvpanel_dsi_res_select(void);
void tvpanel_dsi_irq_enable(unsigned int id, unsigned int irq);
void tvpanel_dsi_irq_disable(unsigned int id, unsigned int irq);
int tvpanel_dsi_irq_query(unsigned int id, unsigned int irq);
int tvpanel_dsi_inst_busy(unsigned int id);
int tvpanel_dsi_open(unsigned int id, const struct disp_panel_para *para);
int tvpanel_dsi_close(unsigned int id);
int tvpanel_dsi_clk_enable(unsigned int id, const struct disp_panel_para *para);
int tvpanel_dsi_dphy_open(unsigned int id, const struct disp_panel_para *para,
			  const struct combodphy_config *cfg);
int tvpanel_dsi_dphy_close(unsigned int id);
int tvpanel_dsi_io_open(unsigned int id, const struct disp_panel_para *para,
			const struct combodphy_config *cfg);
int tvpanel_dsi_io_close(unsigned int id);

/* full mode programming: timings, packets, lanes, D-PHY */
int tvpanel_dsi_cfg(unsigned int id, const struct disp_panel_para *para,
		    const struct combodphy_config *cfg);

/* packet header helpers (shared by the cfg and command layers) */
u8 tvpanel_dsi_ecc(u32 header);
u16 tvpanel_dsi_crc(u32 nbytes, u16 init);

/* ------------------------------------------------------------------ */
/* command channel (tvpanel_dsi_cmd.c)                                 */
/* ------------------------------------------------------------------ */

/* sequencer program selector, 0..10 as in the vendor dsi_start() */
int tvpanel_dsi_start(unsigned int id, unsigned int mode);
int tvpanel_dsi_dcs_wr(unsigned int id, const u8 *buf, unsigned int len);
int tvpanel_dsi_dcs_rd(unsigned int id, u8 *buf, unsigned int len);
int tvpanel_dsi_set_max_ret_size(unsigned int id, u16 size);
int tvpanel_dsi_read_mode_en(unsigned int id, bool enable);
int tvpanel_dsi_open_panel(unsigned int id);
int tvpanel_dsi_close_panel(unsigned int id);
int tvpanel_dsi_tri_start(unsigned int id);

/* ------------------------------------------------------------------ */
/* panel framework (tvpanel_core.c)                                    */
/* ------------------------------------------------------------------ */

/*
 * Panel-side binding surface.  The vendor exports all four in __ksymtab_gpl and
 * the panel modules are the only importers, so the export class is kept
 * identical: a prebuilt panel module must be able to resolve them.  (The GPL
 * marker is the symbol's section, `__ksymtab_gpl`, not a name prefix, so nm
 * cannot be used to tell the classes apart.)
 */
int tvpanel_panel_panel_init(struct tvpanel_panel *panel, struct device *dev,
			     const struct tvpanel_panel_funcs *funcs,
			     int connector_type);
int tvpanel_panel_of_backlight(struct tvpanel_panel *panel);

int tvpanel_panel_add(struct tvpanel_panel *panel);
int tvpanel_panel_remove(struct tvpanel_panel *panel);
struct tvpanel_panel *of_tvpanel_find_panel(struct device_node *np);
int tvpanel_panel_prepare(struct tvpanel_panel *panel);
int tvpanel_panel_unprepare(struct tvpanel_panel *panel);
int tvpanel_panel_enable(struct tvpanel_panel *panel);
int tvpanel_panel_disable(struct tvpanel_panel *panel);

int display_timing_to_panel_para(const struct tvpanel_timing *timing,
				 struct disp_panel_para *para);
int panel_para_to_display_timing(const struct disp_panel_para *para,
				 struct tvpanel_timing *timing);
void tvpanel_timing_from_display_timing(const struct display_timing *dt,
					struct tvpanel_timing *t);
void tvpanel_timing_to_display_timing(const struct tvpanel_timing *t,
				      struct display_timing *dt);

struct tvpanel_connector *get_tvpanel_connector_by_type(int type, int id);
struct tvpanel_connector *get_tvpanel_connector_by_device(int type, int id);
struct tvpanel_connector *get_first_tvpanel_connector(struct tvpanel_device *tpdev);

struct tvpanel_device *tvpanel_get_device(void);

/* ------------------------------------------------------------------ */
/* connector layer (tvpanel_connector.c)                               */
/* ------------------------------------------------------------------ */

int tvpanel_connector_bind(struct tvpanel_connector *conn,
			   struct tvpanel_device *tpdev, struct device *dev,
			   int connector_id,
			   const struct tvpanel_connector_funcs *funcs,
			   const struct display_timing *timing,
			   int connector_type);
int tvpanel_conn_enable(int enable, struct device *dev, const char *buf,
			unsigned int count);
int tvpanel_panel_get_native_timing(struct tvpanel_panel *panel,
				    struct display_timing **timing);
int tvpanel_panel_sw_enable(struct tvpanel_panel *panel);
int tvpanel_pin_set_state(struct device *dev, const char *name);

extern const struct tvpanel_connector_funcs tvpanel_dsi_conn_funcs;

/*
 * Display top / TCON glue.  These are DEFINED in the vendor tvpanel.ko (the
 * symbol table shows them as T, not undefined), so the from-source module has
 * to provide them too; they live in the top/tcon driver and are declared here
 * as the interface the connector layer calls.
 */
int sunxi_tcon_module_enable(struct device *tcon, unsigned int enable);
int sunxi_tcon_clk_enable(struct device *tcon, unsigned int enable);
int sunxi_tcon_config(struct device *tcon, const struct disp_panel_para *para);
int sunxi_tvpanel_top_clk_enable(struct device *top);
int sunxi_tvpanel_top_clk_disable(struct device *top);
int sunxi_tvpanel_top_dsi_enable(struct device *top, unsigned int enable);
int sunxi_tvpanel_top_tcon_tv_enable(struct device *top, unsigned int enable);
int sunxi_tvpanel_top_lvds_enable(struct device *top, unsigned int enable,
				  const struct disp_panel_para *para);
int sunxi_tvdisp_top_set_working_pll_index(struct device *top, unsigned int idx);
int sunxi_tvdisp_top_get_working_pll_index(struct device *top);
int sunxi_tvdisp_top_get_pixel_clk(struct device *top, int pll_index, u64 *clk);
int sunxi_tvdisp_top_pll_setting(struct device *top,
				 const struct disp_panel_para *para, unsigned int rate,
				 int pll_index, int enable);
int sunxi_tvdisp_top_tcon_colorbar_enable(struct device *top,
					  unsigned int enable);
int sunxi_tvdisp_top_afbd_enable(struct device *top, unsigned int enable);
int sunxi_tvdisp_top_svp_enable(struct device *top, unsigned int enable);
int sunxi_tvdisp_top_ksc_enable(struct device *top, unsigned int enable);
int tcon_colorbar_style(u8 style);

/* sub-driver registration, driven by the single module_init in tvpanel_core.c */
int tvpanel_sub_drivers_register(void);
void tvpanel_sub_drivers_unregister(void);

#endif /* __TVPANEL_H */
