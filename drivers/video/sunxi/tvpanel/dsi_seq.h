/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Precompiled DSI instruction words written into dsi_inst_jump_sel/
 * dsi_inst_jump_sel1 by dsi_open(). Values taken verbatim from the vendor
 * module; the DSI instruction set is not documented, so these are kept
 * bit-exact rather than reconstructed.
 */
#ifndef __TVPANEL_DSI_SEQ_H
#define __TVPANEL_DSI_SEQ_H

/* 0x63f07006  opcode=0x06  len=112   lo16=0x7006 hi16=0x63f0 */
#define DSI_INST_JUMP_SEL_DEFAULT 0x63f07006u

/* 0x52e07306  opcode=0x06  len=115   lo16=0x7306 hi16=0x52e0 */
#define DSI_INST_JUMP_SEL_NONCONT 0x52e07306u

/* 0x0d68000f  opcode=0x0f  len=0   lo16=0x000f hi16=0x0d68 */
#define DSI_INST_JUMP_SEL1_NONCONT 0x0d68000fu

/*
 * dsi_inst_jump_sel words for the command path (vendor dsi_open_panel(),
 * dsi_tri_start() and the DCS packet builders).
 */

/* keep the panel sequence running; read mode off */
#define DSI_INST_JUMP_SEL_PANEL_ON 0x63f07602u

/* send whatever is staged in dsi_cmd_tx (dsi_dcs_wr, set_max_ret_size) */
#define DSI_INST_JUMP_SEL_DCS_WR 0x000f0004u

/* issue a DCS read with read mode enabled (dsi_dcs_rd) */
#define DSI_INST_JUMP_SEL_DCS_RD 0x100700f4u

#endif /* __TVPANEL_DSI_SEQ_H */
