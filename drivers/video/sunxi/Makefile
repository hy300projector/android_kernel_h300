# SPDX-License-Identifier: GPL-2.0-only
# Each block follows its own CONFIG symbol so it can be built in or as a
# module, matching the stock module split (tvpanel.ko, z20hd720m.ko,
# vs-display.ko, vs-osd.ko, panel_{dsi,lvds}_gen.ko).
obj-$(CONFIG_SUNXI_TVPANEL)		+= tvpanel/
obj-$(CONFIG_SUNXI_PANEL_LVDS_GEN)	+= panel_lvds_gen/
obj-$(CONFIG_SUNXI_PANEL_DSI_GEN)	+= panel_dsi_gen/
obj-$(CONFIG_SUNXI_PANEL_Z20HD720M)	+= z20hd720m/
obj-$(CONFIG_SUNXI_VS_DISPLAY)		+= vs-display/
obj-$(CONFIG_SUNXI_VS_OSD)		+= vs-osd/
