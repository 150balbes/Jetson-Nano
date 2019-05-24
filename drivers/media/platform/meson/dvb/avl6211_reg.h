/*
 * Driver for the Availink AVL6211+AV2011 DVB-S/S2 demod+tuner
 *
 * Copyright (C) 2014 Sasa Savic <sasa.savic.sr@gmail.com>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
 
#ifndef __AVL6211_REG_H_
#define __AVL6211_REG_H_


#define core_reset_b_reg            				0x600000
#define gpio_data_in_to_reg         				0x6C0004
#define gpio_data_reg_out           				0x6C0008
#define gpio_reg_enb                				0x6C000C

#define pll_clkr_map_addr           				0x6C40C0
#define pll_clkf_map_addr          					0x6C4100
#define pll_od_map_addr             				0x6C4080
#define pll_od2_map_addr            				0x6C4140
#define pll_od3_map_addr            				0x6C4180
#define pll_bwadj_map_addr          				0x6C41C0
#define pll_softvalue_en_map_addr   				0x6C4200
#define reset_register_addr         				0x6C4000

	
#define rx_aagc_gain                                0x0040004C
#define rc_rfagc_tri_enb                            0x006C002C
#define rc_mpeg_bus_tri_enb                         0x006C0028


#define raptor_status_addr						   (0x00000860 + 0x0)
#define rx_state_addr                              (0x00000690 + 0x0)
#define rx_cmd_addr                                (0x00000400 + 0x0)
#define i2cm_cmd_addr                              (0x00000404 + 0x0)
#define i2cm_rsp_addr                              (0x00000418 + 0x0)
#define error_msg_addr                             (0x0000042c + 0x0)
#define rx_config_addr                             (0x0000043c + 0x0)
#define core_ready_word_addr                       (0x00000434 + 0x0)

#define rs_cust_chip_id_addr                        0x006C0034

#define	rp_uint_BER_addr                           (raptor_status_addr + 0x0)
#define	rc_rfagc_pol_addr                          (rx_config_addr + 0x0)
#define	rc_equalizer_addr                          (rx_config_addr + 0x8)
#define	rs_code_rate_addr                          (rx_state_addr + 0x8)
#define	rs_modulation_addr                         (rx_state_addr + 0xc)
#define	rc_format_addr                             (rx_config_addr + 0x10)
#define	rc_mpeg_mode_addr                          (rx_config_addr + 0x20)
#define	rc_outpin_sel_addr                         (rx_config_addr + 0x24)
#define	rs_int_SNR_dB_addr                         (rx_state_addr + 0x40)
#define	rc_aagc_ref_addr                           (rx_config_addr + 0xaa)
#define	rc_mpeg_posedge_addr                       (rx_config_addr + 0xbc)
#define	rc_mpeg_serial_addr                        (rx_config_addr + 0xbe)
#define	rs_fec_lock_addr                           (rx_state_addr + 0x164)
#define	rc_specinv_addr                            (rx_config_addr + 0x34)
#define	rc_int_sym_rate_MHz_addr                   (rx_config_addr + 0x54)
#define	rc_dvbs_ber_addr                           (rx_config_addr + 0x98)
#define	rc_int_dmd_clk_MHz_addr                    (rx_config_addr + 0x162)
#define	rc_int_fec_clk_MHz_addr                    (rx_config_addr + 0x164)
#define	rc_int_mpeg_clk_MHz_addr                   (rx_config_addr + 0x166)
#define	rc_int_carrier_freq_half_range_MHz_addr    (rx_config_addr + 0x16c)
#define	rc_fec_bypass_coderate_addr                (rx_config_addr + 0x194)
#define	rc_i2cm_speed_kHz_addr                     (rx_config_addr + 0x1ae)
#define	rc_tuner_slave_addr_addr                   (rx_config_addr + 0x1b6)
#define	rc_tuner_max_LPF_100kHz_addr               (rx_config_addr + 0x1b8)
#define	rc_tuner_LPF_margin_100kHz_addr            (rx_config_addr + 0x1ba)
#define	rc_tuner_use_internal_control_addr         (rx_config_addr + 0x1bc)

#define	rc_decode_mode_addr                        (rx_config_addr + 0x202)
#define	rc_iq_mode_addr                            (rx_config_addr + 0x204)
#define	rc_lock_mode_addr                          (rx_config_addr + 0x20a)
#define	rc_blind_scan_tuner_spectrum_inversion_addr (rx_config_addr + 0x220)



#define diseqc_tx_cntrl_addr						0x00700000
#define diseqc_tone_frac_n_addr						0x00700004
#define diseqc_tone_frac_d_addr						0x00700008
#define diseqc_tx_st_addr							0x0070000c
#define diseqc_rx_msg_tim_addr						0x00700014
#define diseqc_rx_cntrl_addr						0x0070001c
#define diseqc_srst_addr							0x00700020
#define diseqc_samp_frac_n_addr						0x00700028
#define diseqc_samp_frac_d_addr						0x0070002c
#define diseqc_tx_fifo_map_addr						0x00700080

#endif
