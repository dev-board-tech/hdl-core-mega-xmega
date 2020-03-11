/*
 * This IP is the MEGA/XMEGA ATMEGA32A4 implementation.
 * 
 * Copyright (C) 2020  Iulian Gheorghiu (morgoth@devboard.tech)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

`timescale 1ns / 1ps

`include "mega-def.v"

//`define USE_PLL
`define USE_TIMER_0
`define USE_TIMER_1
`define USE_TIMER_3
`define USE_TIMER_4
`define USE_SPI_1
//`define USE_UART_1
`define USE_EEPROM


/* ATMEGA32U4 is a "MEGA_ENHANCED_128K" family */
`define CORE_TYPE				`MEGA_ENHANCED_128K
`define ROM_ADDR_WIDTH			15
`define BUS_ADDR_DATA_LEN		12
`define RAM_ADDR_WIDTH			12
`define EEP_ADDR_WIDTH			10
`define RESERVED_RAM_FOR_IO		'h100

`define VECTOR_INT_TABLE_SIZE	42
`define WATCHDOG_CNT_WIDTH		0//27

/* DATA BUS DEMULTIPLEXER */
module io_bus_dmux # (
		parameter NR_OF_BUSSES_IN = 1
		)(
		input [(NR_OF_BUSSES_IN * 8) - 1 : 0]bus_in,
		output reg[7:0]bus_out
		);
reg [NR_OF_BUSSES_IN - 1 : 0]tmp_busses_bits;
integer cnt_add_busses;
integer cnt_add_bits;
		always @ *
		begin
			for(cnt_add_bits = 0; cnt_add_bits < 8; cnt_add_bits = cnt_add_bits + 1)
			begin: DMUX_IO_DATA_BITS
				for(cnt_add_busses = 0; cnt_add_busses < NR_OF_BUSSES_IN; cnt_add_busses = cnt_add_busses + 1)
				begin: DMUX_IO_DATA_BUSES
					tmp_busses_bits[cnt_add_busses] = bus_in[(cnt_add_busses * 8) + cnt_add_bits];
				end
				bus_out[cnt_add_bits] = |tmp_busses_bits;
			end
		end
endmodule
/* !DATA BUS DEMULTIPLEXER */

/* TIMMERS PRESCALLERS MODULE */
module tim_013_prescaller (
	input rst,
	input clk,
	output clk8,
	output clk64,
	output clk256,
	output clk1024
);
reg [9:0]cnt;

always @(posedge clk)
begin
	if(rst)
	begin
		cnt <= 10'h000;
	end
	else
	begin
		cnt <= cnt + 10'd1;
	end
end

assign clk8 = cnt[2];
assign clk64 = cnt[5];
assign clk256 = cnt[7];
assign clk1024 = cnt[9];

endmodule
/* !TIMMERS PRESCALLERS MODULE */

module atmega32u4_arduboy # (
	parameter PLATFORM = "XILINX",
	parameter REGS_REGISTERED = "FALSE",
	parameter ROM_PATH = "",
	parameter USE_HALT = "FALSE",
	parameter USE_PIO_B = "TRUE",
	parameter USE_PIO_C = "TRUE",
	parameter USE_PIO_D = "TRUE",
	parameter USE_PIO_E = "TRUE",
	parameter USE_PIO_F = "TRUE",
	parameter USE_PLL = "TRUE",
	parameter USE_PLL_HI_FREQ = "FALSE",
	parameter USE_TIMER_0 = "TRUE",
	parameter USE_TIMER_1 = "TRUE",
	parameter USE_TIMER_3 = "TRUE",
	parameter USE_TIMER_4 = "TRUE",
	parameter USE_SPI_1 = "TRUE",
	parameter USE_UART_1 = "TRUE",
	parameter USE_EEPROM = "TRUE"
)(
	input rst,
	input clk,
	input clk_pll,
	input halt,
	output halt_ack,
    input [5:0] buttons,
    output [2:0] RGB,
    output Buzzer1, Buzzer2, OledDC, OledCS, OledRST, spi_scl, spi_mosi,
	/* For IO's that are not included in original ATmega32u4 device */
	output [5:0]io_addr,
	output [7:0]io_out,
	output io_write,
	input [7:0]io_ext_in,
	output io_read,
	
	input [24:0]deb_addr,
	input deb_wr,
	input [7:0]deb_data_in,
	input deb_rd,
	output [7:0]deb_data_out,
	input deb_en,
	
	output eep_content_modifyed,

	output [4:0]debug
	);

wire core_clk = clk;
wire wdt_rst;

/* CORE WIRES */
wire [`ROM_ADDR_WIDTH-1:0]pgm_addr;
wire [15:0]pgm_data;
wire [`BUS_ADDR_DATA_LEN-1:0]data_addr;
wire [7:0]core_data_out;
wire data_write;
wire [7:0]core_data_in;
wire data_read;
/* !CORE WIRES */


/* IO WIRES */
wire [7:0]pb_in;
wire [7:0]pc_in;
wire [7:0]pd_in;
wire [7:0]pe_in;
wire [7:0]pf_in;
wire [7:0]pb_out;
wire [7:0]pc_out;
wire [7:0]pd_out;
wire [7:0]pe_out;
wire [7:0]pf_out;
/* !IO WIRES */

/* IO PIN FUNCTION CHANGE REQUEST */
wire [7:0]piob_out_io_connect;
wire [7:0]pioc_out_io_connect;
wire [7:0]piod_out_io_connect;
wire [7:0]pioe_out_io_connect;
wire [7:0]piof_out_io_connect;
wire tim0_oca_io_connect;
wire tim0_ocb_io_connect;
wire tim1_oca_io_connect;
wire tim1_ocb_io_connect;
wire tim1_occ_io_connect;
wire tim3_oca_io_connect;
wire tim3_ocb_io_connect;
wire tim3_occ_io_connect;
wire tim4_ocap_io_connect;
wire tim4_ocan_io_connect;
wire tim4_ocbp_io_connect;
wire tim4_ocbn_io_connect;
wire tim4_occp_io_connect;
wire tim4_occn_io_connect;
wire tim4_ocdp_io_connect;
wire tim4_ocdn_io_connect;
wire uart_tx_io_connect;
wire spi_io_connect;
wire io_conn_slave;
/* !IO PIN FUNCTION CHANGE REQUEST */
wire pll_enabled;
/* IO ALTERNATIVE FUNCTION */
wire tim0_oca;
wire tim0_ocb;
wire tim1_oca;
wire tim1_ocb;
wire tim1_occ;
wire tim3_oca;
wire tim3_ocb;
wire tim3_occ;
wire tim4_oca;
wire tim4_ocb;
wire tim4_occ;
wire tim4_ocd;
//wire spi_scl;
wire spi_miso;
//wire spi_mosi;
wire uart_tx;
wire uart_rx;
wire usb_ck_out;
wire tim_ck_out;
/* !IO ALTERNATIVE FUNCTION */

assign pf_in[6] = buttons[0]; // BUTTON RIGHT
assign pf_in[5] = buttons[1]; // BUTTON LEFT
assign pf_in[4] = buttons[2]; // BUTTON DOWN
assign pf_in[7] = buttons[3]; // BUTTON UP
assign pe_in[6] = buttons[4]; // BUTTON A
assign pb_in[4] = buttons[5]; // BUTTON B

// RGB LED is common anode (ie. HIGH = OFF)
assign RGB[2] = tim1_ocb_io_connect ? tim1_ocb : pb_out[6];
assign RGB[1] = tim0_oca_io_connect ? ~tim0_oca : pb_out[7];
assign RGB[0] = tim1_oca_io_connect ? tim1_oca : pb_out[5];

assign Buzzer1 = tim4_ocan_io_connect ? ~tim4_oca : pc_out[6];
assign Buzzer2 = tim4_ocap_io_connect ? tim4_oca : pc_out[7];
assign OledDC = pd_out[4];
assign OledCS = pd_out[6];
assign OledRST = pd_out[7];

/* Switch pins functionality */
/*assign pb_in[0] = pb[0];
assign pb_in[1] = pb[1];
assign pb_in[2] = pb[2];
assign pb_in[3] = pb[3];
assign pb_in[4] = pb[4];
assign pb_in[5] = pb[5];
assign pb_in[6] = pb[6];
assign pb_in[7] = pb[7];
//assign spi_scl = (spi_io_connect & io_conn_slave) ? pb[1] : 1'bz; // Add in case of SPI slave usage, this will become input.
//assign spi_mosi = (spi_io_connect & io_conn_slave) ? pb[2] : 1'bz; // Add in case of SPI slave usage, this will become input.
assign spi_miso = (spi_io_connect & ~io_conn_slave)  ? pb[3] : 1'bz;
assign pb[0] = piob_out_io_connect[0] ? pb_out[0] : 1'bz;
assign pb[1] = (spi_io_connect & ~io_conn_slave) ? spi_scl : (piob_out_io_connect[1] ? pb_out[1] : 1'bz);
assign pb[2] = (spi_io_connect & ~io_conn_slave) ? spi_mosi : (piob_out_io_connect[2] ? pb_out[2] : 1'bz);
assign pb[3] = (spi_io_connect & io_conn_slave) ? spi_miso : (piob_out_io_connect[3] ? pb_out[3] : 1'bz);
assign pb[4] = (piob_out_io_connect[4] ? pb_out[4] : 1'bz);
assign pb[5] = tim1_oca_io_connect ? tim1_oca : (piob_out_io_connect[5] ? pb_out[5] : 1'bz);
assign pb[6] = tim1_ocb_io_connect ? tim1_ocb : (piob_out_io_connect[6] ? pb_out[6] : 1'bz);
assign pb[7] = tim0_oca_io_connect ? tim0_oca : (tim1_occ_io_connect ? tim1_occ : (piob_out_io_connect[7] ? pb_out[7] : 1'bz));

assign pc_in[0] = pc[0];
assign pc_in[1] = pc[1];
assign pc_in[2] = pc[2];
assign pc_in[3] = pc[3];
assign pc_in[4] = pc[4];
assign pc_in[5] = pc[5];
assign pc_in[6] = pc[6];
assign pc_in[7] = pc[7];
assign pc[0] = pioc_out_io_connect[0] ? pc_out[0] : 1'bz;
assign pc[1] = pioc_out_io_connect[1] ? pc_out[1] : 1'bz;
assign pc[2] = pioc_out_io_connect[2] ? pc_out[2] : 1'bz;
assign pc[3] = pioc_out_io_connect[3] ? pc_out[3] : 1'bz;
assign pc[4] = pioc_out_io_connect[4] ? pc_out[4] : 1'bz;
assign pc[5] = pioc_out_io_connect[5] ? pc_out[5] : 1'bz;
assign pc[6] = tim4_ocan_io_connect ? ~tim4_oca : (tim3_oca_io_connect ? tim3_oca : (pioc_out_io_connect[6] ? pc_out[6] : 1'bz));
assign pc[7] = tim4_ocap_io_connect ? tim4_oca : (pioc_out_io_connect[7] ? pc_out[7] : 1'bz);

assign pd_in[0] = pd[0];
assign pd_in[1] = pd[1];
assign pd_in[2] = pd[2];
assign pd_in[3] = pd[3];
assign pd_in[4] = pd[4];
assign pd_in[5] = pd[5];
assign pd_in[6] = pd[6];
assign pd_in[7] = pd[7];
assign uart_rx = pd[2];
assign pd[0] = tim0_ocb_io_connect ? tim0_ocb : (piod_out_io_connect[0] ? pd_out[0] : 1'bz);
assign pd[1] = piod_out_io_connect[1] ? pd_out[1] : 1'bz;
assign pd[2] = piod_out_io_connect[2] ? pd_out[2] : 1'bz;
assign pd[3] = uart_tx_io_connect ? uart_tx : (piod_out_io_connect[3] ? pd_out[3] : 1'bz);
assign pd[4] = piod_out_io_connect[4] ? pd_out[4] : 1'bz;
assign pd[5] = piod_out_io_connect[5] ? pd_out[5] : 1'bz;
assign pd[6] = piod_out_io_connect[6] ? pd_out[6] : 1'bz;
assign pd[7] = piod_out_io_connect[7] ? pd_out[7] : 1'bz;

assign pe_in[0] = pe[0];
assign pe_in[1] = pe[1];
assign pe_in[2] = pe[2];
assign pe_in[3] = pe[3];
assign pe_in[4] = pe[4];
assign pe_in[5] = pe[5];
assign pe_in[6] = pe[6];
assign pe_in[7] = pe[7];
assign pe[0] = pioe_out_io_connect[0] ? pe_out[0] : 1'bz;
assign pe[1] = pioe_out_io_connect[1] ? pe_out[1] : 1'bz;
assign pe[2] = pioe_out_io_connect[2] ? pe_out[2] : 1'bz;
assign pe[3] = pioe_out_io_connect[3] ? pe_out[3] : 1'bz;
assign pe[4] = pioe_out_io_connect[4] ? pe_out[4] : 1'bz;
assign pe[5] = pioe_out_io_connect[5] ? pe_out[5] : 1'bz;
assign pe[6] = pioe_out_io_connect[6] ? pe_out[6] : 1'bz;
assign pe[7] = pioe_out_io_connect[7] ? pe_out[7] : 1'bz;

assign pf_in[0] = pf[0];
assign pf_in[1] = pf[1];
assign pf_in[2] = pf[2];
assign pf_in[3] = pf[3];
assign pf_in[4] = pf[4];
assign pf_in[5] = pf[5];
assign pf_in[6] = pf[6];
assign pf_in[7] = pf[7];
assign pf[0] = piof_out_io_connect[0] ? pf_out[0] : 1'bz;
assign pf[1] = piof_out_io_connect[1] ? pf_out[1] : 1'bz;
assign pf[2] = piof_out_io_connect[2] ? pf_out[2] : 1'bz;
assign pf[3] = piof_out_io_connect[3] ? pf_out[3] : 1'bz;
assign pf[4] = piof_out_io_connect[4] ? pf_out[4] : 1'bz;
assign pf[5] = piof_out_io_connect[5] ? pf_out[5] : 1'bz;
assign pf[6] = piof_out_io_connect[6] ? pf_out[6] : 1'bz;
assign pf[7] = piof_out_io_connect[7] ? pf_out[7] : 1'bz;*/
/* !Switch pins functionality */


/* Interrupt wires */
wire ram_sel = |data_addr[`BUS_ADDR_DATA_LEN-1:8];
wire int_int0 = 0;
wire int_int1 = 0;
wire int_int2 = 0;
wire int_int3 = 0;
wire int_reserved0 = 0;
wire int_reserved1 = 0;
wire int_int6 = 0;
wire int_reserved3 = 0;
wire int_pcint0 = 0;
wire int_usb_general = 0;
wire int_usb_endpoint = 0;
wire int_wdt = 0;
wire int_reserved4 = 0;
wire int_reserved5 = 0;
wire int_reserved6 = 0;
wire int_timer1_capt = 0;
wire int_timer1_compa;
wire int_timer1_compb;
wire int_timer1_compc;
wire int_timer1_ovf;
wire int_timer0_compa;
wire int_timer0_compb;
wire int_timer0_ovf;
wire int_spi_stc;
wire int_usart1_rx;
wire int_usart1_udre;
wire int_usart1_tx;
wire int_analog_comp = 0;
wire int_adc = 0;
wire int_ee_ready;
wire int_timer3_capt = 0;
wire int_timer3_compa;
wire int_timer3_compb;
wire int_timer3_compc;
wire int_timer3_ovf;
wire int_twi = 0;
wire int_spm_ready = 0;
wire int_timer4_compa;
wire int_timer4_compb;
wire int_timer4_compd;
wire int_timer4_ovf;
wire int_timer4_fpf = 0;
/* !Interrupt wires */

/* Interrupt reset wires */
wire int_int0_rst;
wire int_int1_rst;
wire int_int2_rst;
wire int_int3_rst;
wire int_reserved0_rst;
wire int_reserved1_rst;
wire int_int6_rst;
wire int_reserved3_rst;
wire int_pcint0_rst;
wire int_usb_general_rst;
wire int_usb_endpoint_rst;
wire int_wdt_rst;
wire int_reserved4_rst;
wire int_reserved5_rst;
wire int_reserved6_rst;
wire int_timer1_capt_rst;
wire int_timer1_compa_rst;
wire int_timer1_compb_rst;
wire int_timer1_compc_rst;
wire int_timer1_ovf_rst;
wire int_timer0_compa_rst;
wire int_timer0_compb_rst;
wire int_timer0_ovf_rst;
wire int_spi_stc_rst;
wire int_usart1_rx_rst;
wire int_usart1_udre_rst;
wire int_usart1_tx_rst;
wire int_analog_comp_rst;
wire int_adc_rst;
wire int_ee_ready_rst;
wire int_timer3_capt_rst;
wire int_timer3_compa_rst;
wire int_timer3_compb_rst;
wire int_timer3_compc_rst;
wire int_timer3_ovf_rst;
wire int_twi_rst;
wire int_spm_ready_rst;
wire int_timer4_compa_rst;
wire int_timer4_compb_rst;
wire int_timer4_compd_rst;
wire int_timer4_ovf_rst;
wire int_timer4_fpf_rst;
/* !Interrupt reset wires */

/* Busses MUX/DMUX module for debug interface. */
wire[16:0]ext_pgm_addr;
wire[15:0]ext_pgm_data_in;
wire ext_pgm_data_wr;
wire[15:0]ext_pgm_data_out;
wire ext_pgm_data_rd;
wire ext_pgm_data_en = 0;
	
wire[15:0]ext_ram_addr;
wire[7:0]ext_ram_data_in;
wire ext_ram_data_wr;
wire[7:0]ext_ram_data_out;
wire ext_ram_data_rd;
wire ext_ram_data_en = 0;
	
generate
if (0)
begin
mega_debug_mem_sel mega_debug (

	.rst(rst),
	.clk(clk),

	.deb_addr(deb_addr),
	.deb_wr(deb_wr),
	.deb_data_in(deb_data_in),
	.deb_rd(deb_rd),
	.deb_data_out(deb_data_out),
	.deb_en(deb_en),

	.ext_pgm_addr(ext_pgm_addr),
	.ext_pgm_data_in(ext_pgm_data_in),
	.ext_pgm_data_wr(ext_pgm_data_wr),
	.ext_pgm_data_out(ext_pgm_data_out),
	.ext_pgm_data_rd(ext_pgm_data_rd),
	.ext_pgm_data_en(ext_pgm_data_en),
	
	.ext_ram_addr(ext_ram_addr),
	.ext_ram_data_in(ext_ram_data_in),
	.ext_ram_data_wr(ext_ram_data_wr),
	.ext_ram_data_out(ext_ram_data_out),
	.ext_ram_data_rd(ext_ram_data_rd),
	.ext_ram_data_en(ext_ram_data_en)
);
end
endgenerate
/* !Busses MUX/DMUX module for debug interface. */

/* PORTB */
wire [7:0]dat_pb_d_out;
generate
if (USE_PIO_B == "TRUE")
begin: PORTB
atmega_pio # (
	.PLATFORM(PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.PORT_WIDTH(8),
	.USE_CLEAR_SET("FALSE"),
	.PORT_OUT_ADDR('h25),
	.PORT_CLEAR_ADDR('h00),
	.PORT_SET_ADDR('h01),
	.DDR_ADDR('h24),
	.PIN_ADDR('h23),
	.PINMASK(8'b11110000),
	.PULLUP_MASK(8'b00000000),
	.PULLDN_MASK(8'b00000000),
	.INVERSE_MASK(8'b00010000),
	.OUT_ENABLED_MASK(8'b11101111)
)pio_b(
	.rst(rst),
	.clk(clk),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_pb_d_out),

	.io_in(pb_in),
	.io_out(pb_out),
	.pio_out_io_connect(piob_out_io_connect)
	);
end
else
begin
assign dat_pb_d_out = 0;
end
endgenerate
/* !PORTB */

/* PORTC */
wire [7:0]dat_pc_d_out;
generate
if (USE_PIO_C == "TRUE")
begin: PORTC
atmega_pio # (
	.PLATFORM(PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.PORT_WIDTH(8),
	.USE_CLEAR_SET("FALSE"),
	.PORT_OUT_ADDR('h28),
	.PORT_CLEAR_ADDR('h00),
	.PORT_SET_ADDR('h01),
	.DDR_ADDR('h27),
	.PIN_ADDR('h26),
	.PINMASK(8'b11000000),
	.PULLUP_MASK(8'b00000000),
	.PULLDN_MASK(8'b00000000),
	.INVERSE_MASK(8'b00000000),
	.OUT_ENABLED_MASK(8'b11000000)
)pio_c(
	.rst(rst),
	.clk(clk),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_pc_d_out),

	.io_in(pc_in),
	.io_out(pc_out),
	.pio_out_io_connect(pioc_out_io_connect)
	);
end
else
begin
assign dat_pc_d_out = 0;
end
endgenerate
/* !PORTC */

/* PORTD */
wire [7:0]dat_pd_d_out;
generate
if (USE_PIO_D == "TRUE")
begin: PORTD
atmega_pio # (
	.PLATFORM(PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.PORT_WIDTH(8),
	.USE_CLEAR_SET("FALSE"),
	.PORT_OUT_ADDR('h2b),
	.PORT_CLEAR_ADDR('h00),
	.PORT_SET_ADDR('h01),
	.DDR_ADDR('h2a),
	.PIN_ADDR('h29),
	.PINMASK(8'b11010000),
	.PULLUP_MASK(8'b00000000),
	.PULLDN_MASK(8'b00000000),
	.INVERSE_MASK(8'b00000000),
	.OUT_ENABLED_MASK(8'b11111111)
)pio_d(
	.rst(rst),
	.clk(clk),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_pd_d_out),

	.io_in(pd_in),
	.io_out(pd_out),
	.pio_out_io_connect(piod_out_io_connect)
	);
end
else
begin
assign dat_pd_d_out = 0;
end
endgenerate
/* !PORTD */

/* PORTE */
wire [7:0]dat_pe_d_out;
generate
if (USE_PIO_E == "TRUE")
begin: PORTE
atmega_pio # (
	.PLATFORM(PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.PORT_WIDTH(8),
	.USE_CLEAR_SET("FALSE"),
	.PORT_OUT_ADDR('h2e),
	.PORT_CLEAR_ADDR('h00),
	.PORT_SET_ADDR('h01),
	.DDR_ADDR('h2d),
	.PIN_ADDR('h2c),
	.PINMASK(8'b01000000),
	.PULLUP_MASK(8'b00000000),
	.PULLDN_MASK(8'b00000000),
	.INVERSE_MASK(8'b01000000),
	.OUT_ENABLED_MASK(8'b00000000)
)pio_e(
	.rst(rst),
	.clk(clk),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_pe_d_out),

	.io_in(pe_in),
	.io_out(pe_out),
	.pio_out_io_connect(pioe_out_io_connect)
	);
end
else
begin
assign dat_pe_d_out = 0;
end
endgenerate
/* !PORTE */

/* PORTF */
wire [7:0]dat_pf_d_out;
generate
if (USE_PIO_F == "TRUE")
begin: PORTF
atmega_pio # (
	.PLATFORM(PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.PORT_WIDTH(8),
	.USE_CLEAR_SET("FALSE"),
	.PORT_OUT_ADDR('h31),
	.PORT_CLEAR_ADDR('h00),
	.PORT_SET_ADDR('h01),
	.DDR_ADDR('h30),
	.PIN_ADDR('h2f),
	.PINMASK(8'b11110011),
	.PULLUP_MASK(8'b00000000),
	.PULLDN_MASK(8'b00000000),
	.INVERSE_MASK(8'b11110000),
	.OUT_ENABLED_MASK(8'b00000000)
)pio_f(
	.rst(rst),
	.clk(clk),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_pf_d_out),

	.io_in(pf_in),
	.io_out(pf_out),
	.pio_out_io_connect(piof_out_io_connect)
	);
end
else
begin
assign dat_pf_d_out = 0;
end
endgenerate
/* !PORTF */

/* SPI */
wire [7:0]dat_spi_d_out;
generate
if (USE_SPI_1 == "TRUE")
begin: SPI_DISPLAY
atmega_spi_m # (
	.PLATFORM(PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.SPCR_ADDR('h4c),
	.SPSR_ADDR('h4d),
	.SPDR_ADDR('h4e),
	.DINAMIC_BAUDRATE("FALSE"),
	.BAUDRATE_CNT_LEN(0),
	.BAUDRATE_DIVIDER(0),
	.USE_TX("TRUE"),
	.USE_RX("FALSE")
)spi(
	.rst(rst),
	.halt(halt_ack),
	.clk(clk),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_spi_d_out),
	.int(int_spi_stc),
	.int_rst(int_spi_stc_rst),
	.io_connect(spi_io_connect),
	.io_conn_slave(io_conn_slave),

	.scl(spi_scl),
	.miso(spi_miso),
	.mosi(spi_mosi)
	);
end
else
begin
assign dat_spi_d_out = 0;
assign int_spi_stc = 1'b0;
assign spi_io_connect = 1'b0;
end
endgenerate
/* !SPI */
/* UART */
wire [7:0]dat_uart0_d_out;
generate
if (USE_UART_1 == "TRUE")
begin: UART1
atmega_uart # (
	.PLATFORM(PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.UDR_ADDR('hce),
	.UCSRA_ADDR('hc8),
	.UCSRB_ADDR('hc9),
	.UCSRC_ADDR('hca),
	.UBRRL_ADDR('hcc),
	.UBRRH_ADDR('hcd),
	.USE_TX("TRUE"),
	.USE_RX("TRUE")
	)uart(
	.rst(rst),
	.clk(clk),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_uart0_d_out),
	.rxc_int(int_usart1_rx),
	.rxc_int_rst(int_usart1_rx_rst),
	.txc_int(int_usart1_tx),
	.txc_int_rst(int_usart1_tx_rst),
	.udre_int(int_usart1_udre),
	.udre_int_rst(int_usart1_udre_rst),

	.rx(uart_rx),
	.tx(uart_tx),
	.tx_connect(uart_tx_io_connect)
	);
end
else
begin
assign dat_uart0_d_out = 1'b0;
assign int_usart1_rx = 1'b0;
assign int_usart1_tx = 1'b0;
assign int_usart1_udre = 1'b0;
assign uart_tx_io_connect = 1'b0;
end
endgenerate
/* UART */

/* TIMER PRESCALLER */
wire clk8;
wire clk64;
wire clk256;
wire clk1024;
tim_013_prescaller tim_013_prescaller_inst(
	.rst(rst),
	.clk(clk),
	.clk8(clk8),
	.clk64(clk64),
	.clk256(clk256),
	.clk1024(clk1024)
);
/* !TIMER PRESCALLER */

/* TIMER 0 */
wire [7:0]dat_tim0_d_out;
generate
if (USE_TIMER_0 == "TRUE")
begin:TIMER0
atmega_tim_8bit # (
	.PLATFORM(PLATFORM),
	.USE_OCRB("TRUE"),
	.BUS_ADDR_DATA_LEN(8),
	.GTCCR_ADDR('h43),
	.TCCRA_ADDR('h44),
	.TCCRB_ADDR('h45),
	.TCNT_ADDR('h46),
	.OCRA_ADDR('h47),
	.OCRB_ADDR('h48),
	.TIMSK_ADDR('h6E),
	.TIFR_ADDR('h35)
)tim_0(
	.rst(rst),
	.halt(halt_ack),
	.clk(clk),
	.clk8(clk8),
	.clk64(clk64),
	.clk256(clk256),
	.clk1024(clk1024),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_tim0_d_out),
	.tov_int(int_timer0_ovf),
	.tov_int_rst(int_timer0_ovf_rst),
	.ocra_int(int_timer0_compa),
	.ocra_int_rst(int_timer0_compa_rst),
	.ocrb_int(int_timer0_compb),
	.ocrb_int_rst(int_timer0_compb_rst),
	
	.t(),
	.oca(tim0_oca),
	.ocb(tim0_ocb),
	.oca_io_connect(tim0_oca_io_connect),
	.ocb_io_connect(tim0_ocb_io_connect)
	);
end
else
begin
assign dat_tim0_d_out = 0;
assign int_timer0_ovf = 1'b0;
assign int_timer0_compa = 1'b0;
assign tim0_oca_io_connect = 1'b0;
assign tim0_ocb_io_connect = 1'b0;
end
endgenerate
/* !TIMER 0 */

/* TIMER 1 */
wire [7:0]dat_tim1_d_out;
generate
if (USE_TIMER_1 == "TRUE")
begin: TIMER1
atmega_tim_16bit # (
	.PLATFORM(PLATFORM),
	.USE_OCRB("TRUE"),
	.USE_OCRC("TRUE"),
	.USE_OCRD("FALSE"),
	.BUS_ADDR_DATA_LEN(8),
	.GTCCR_ADDR('h43),
	.TCCRA_ADDR('h80),
	.TCCRB_ADDR('h81),
	.TCCRC_ADDR('h82),
	.TCNTL_ADDR('h84),
	.TCNTH_ADDR('h85),
	.ICRL_ADDR('h86),
	.ICRH_ADDR('h87),
	.OCRAL_ADDR('h88),
	.OCRAH_ADDR('h89),
	.OCRBL_ADDR('h8A),
	.OCRBH_ADDR('h8B),
	.OCRCL_ADDR('h8C),
	.OCRCH_ADDR('h8D),
	.TIMSK_ADDR('h6F),
	.TIFR_ADDR('h36)
)tim_1(
	.rst(rst),
	.halt(halt_ack),
	.clk(clk),
	.clk8(clk8),
	.clk64(clk64),
	.clk256(clk256),
	.clk1024(clk1024),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_tim1_d_out),
	.tov_int(int_timer1_ovf),
	.tov_int_rst(int_timer1_ovf_rst),
	.ocra_int(int_timer1_compa),
	.ocra_int_rst(int_timer1_compa_rst),
	.ocrb_int(int_timer1_compb),
	.ocrb_int_rst(int_timer1_compb_rst),
	.ocrc_int(int_timer1_compc),
	.ocrc_int_rst(int_timer1_compc_rst),
	.ocrd_int(),
	.ocrd_int_rst(),
	
	.t(),
	.oca(tim1_oca),
	.ocb(tim1_ocb),
	.occ(tim1_occ),
	.ocd(),
	.oca_io_connect(tim1_oca_io_connect),
	.ocb_io_connect(tim1_ocb_io_connect),
	.occ_io_connect(tim1_occ_io_connect),
	.ocd_io_connect()
	);
end
else
begin
assign dat_tim1_d_out = 0;
end
endgenerate
/* !TIMER 1 */

/* TIMER 3 */
wire [7:0]dat_tim3_d_out;
generate
if (USE_TIMER_3 == "TRUE")
begin: TIMER3
atmega_tim_16bit # (
	.PLATFORM(PLATFORM),
	.USE_OCRB("FALSE"),
	.USE_OCRC("FALSE"),
	.USE_OCRD("FALSE"),
	.BUS_ADDR_DATA_LEN(8),
	.GTCCR_ADDR('h43),
	.TCCRA_ADDR('h90),
	.TCCRB_ADDR('h91),
	.TCCRC_ADDR('h92),
	.TCNTL_ADDR('h94),
	.TCNTH_ADDR('h95),
	.ICRL_ADDR('h96),
	.ICRH_ADDR('h97),
	.OCRAL_ADDR('h98),
	.OCRAH_ADDR('h99),
	.OCRBL_ADDR('h9A),
	.OCRBH_ADDR('h9B),
	.OCRCL_ADDR('h9C),
	.OCRCH_ADDR('h9D),
	.TIMSK_ADDR('h71),
	.TIFR_ADDR('h38)
)tim_3(
	.rst(rst),
	.halt(halt_ack),
	.clk(clk),
	.clk8(clk8),
	.clk64(clk64),
	.clk256(clk256),
	.clk1024(clk1024),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_tim3_d_out),
	.tov_int(int_timer3_ovf),
	.tov_int_rst(int_timer3_ovf_rst),
	.ocra_int(int_timer3_compa),
	.ocra_int_rst(int_timer3_compa_rst),
	.ocrb_int(int_timer3_compb),
	.ocrb_int_rst(int_timer3_compb_rst),
	.ocrc_int(int_timer3_compc),
	.ocrc_int_rst(int_timer3_compc_rst),
	.ocrd_int(),
	.ocrd_int_rst(),
	
	.t(),
	.oca(tim3_oca),
	.ocb(tim3_ocb),
	.occ(tim3_occ),
	.ocd(),
	.oca_io_connect(tim3_oca_io_connect),
	.ocb_io_connect(tim3_ocb_io_connect),
	.occ_io_connect(tim3_occ_io_connect),
	.ocd_io_connect()
	);
end
else
begin
assign dat_tim3_d_out = 0;
end
endgenerate
/* !TIMER 3 */

/* PLL */
wire [7:0]dat_pll_d_out;
generate
if(USE_PLL == "TRUE")
begin: PLL
atmega_pll # (
	.PLATFORM(PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.PLLCSR_ADDR('h49),
	.PLLFRQ_ADDR('h52),
	.USE_PLL(USE_PLL_HI_FREQ)
)pll(
	.rst(rst),
	.clk(clk),
	.clk_pll(clk_pll),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_pll_d_out),
	.pll_enabled(pll_enabled),

	.usb_ck_out(usb_ck_out),
	.tim_ck_out(tim_ck_out)
	);
end
else
begin
	assign dat_pll_d_out = 0;
end
endgenerate
/* !PLL */

/* TIMER 4 */
wire [7:0]dat_tim4_d_out;
generate
if (USE_TIMER_4 == "TRUE")
begin: TIMER4
atmega_tim_10bit # (
	.PLATFORM(PLATFORM),
	.USE_OCRA("TRUE"),
	.USE_OCRB("TRUE"),
	.USE_OCRD("TRUE"),
	.BUS_ADDR_DATA_LEN(8),
	.TCCRA_ADDR('hc0),
	.TCCRB_ADDR('hc1),
	.TCCRC_ADDR('hc2),
	.TCCRD_ADDR('hc3),
	.TCCRE_ADDR('hc4),
	.TCNTL_ADDR('hbe),
	.TCH_ADDR('hbf),
	.OCRA_ADDR('hcf),
	.OCRB_ADDR('hd0),
	.OCRC_ADDR('hd1),
	.OCRD_ADDR('hd2),
	.TIMSK_ADDR('h72),
	.TIFR_ADDR('h39)
)tim_4(
	.rst(rst),
	.halt(halt_ack),
	.clk(clk),
	.clk_pll(tim_ck_out),
	.pll_enabled(pll_enabled),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_tim4_d_out),
	.tov_int(int_timer4_ovf),
	.tov_int_rst(int_timer4_ovf_rst),
	.ocra_int(int_timer4_compa),
	.ocra_int_rst(int_timer4_compa_rst),
	.ocrb_int(int_timer4_compb),
	.ocrb_int_rst(int_timer4_compb_rst),
	.ocrc_int(),
	.ocrc_int_rst(),
	.ocrd_int(int_timer4_compd),
	.ocrd_int_rst(int_timer4_compd_rst),
	
	.t(),
	.oca(tim4_oca),
	.ocb(tim4_ocb),
	.occ(tim4_occ),
	.ocd(tim4_ocd),
	.ocap_io_connect(tim4_ocap_io_connect),
	.ocan_io_connect(tim4_ocan_io_connect),
	.ocbp_io_connect(tim4_ocbp_io_connect),
	.ocbn_io_connect(tim4_ocbn_io_connect),
	.occp_io_connect(tim4_occp_io_connect),
	.occn_io_connect(tim4_occn_io_connect),
	.ocdp_io_connect(tim4_ocdp_io_connect),
	.ocdn_io_connect(tim4_ocdn_io_connect)
	);
end
else
begin
assign dat_tim4_d_out = 0;
end
endgenerate
/* !TIMER 4 */

/* EEPROM */
wire [7:0]dat_eeprom_d_out;
generate
if (USE_EEPROM == "TRUE")
begin: EEPROM
atmega_eep # (
	.PLATFORM(PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.EEARH_ADDR('h42),
	.EEARL_ADDR('h41),
	.EEDR_ADDR('h40),
	.EECR_ADDR('h3F),
	.EEP_SIZE(1024)
)eep(
	.rst(rst),
	.clk(clk),
	.addr_dat(data_addr[7:0]),
	.wr_dat(data_write & ~ram_sel),
	.rd_dat(data_read & ~ram_sel),
	.bus_dat_in(core_data_out),
	.bus_dat_out(dat_eeprom_d_out),
	.int(int_ee_ready),
	.int_rst(int_ee_ready_rst),
	/*.ext_eep_addr(0),
	.ext_eep_data_in(0),
	.ext_eep_data_wr(1'b0),
	.ext_eep_data_out(ext_eep_data_out),
	.ext_eep_data_rd(1'b0),
	.ext_eep_data_en(1'b0),*/
	.content_modifyed(eep_content_modifyed),
	.debug()
	);
end
else
begin
assign dat_eeprom_d_out = 0;
end
endgenerate
/* !EEPROM */

/* ROM APP */
wire [15:0]pgm_data_app;
mega_ram  #(
	.PLATFORM(PLATFORM),
	.MEM_MODE("SRAM"), // "BLOCK","SRAM"
	.ADDR_BUS_WIDTH(`ROM_ADDR_WIDTH),
	.DATA_BUS_WIDTH(16),
	.RAM_PATH(ROM_PATH)
)rom_app(
	.clk(core_clk),
	.cs(ext_pgm_data_en ? ~ext_pgm_addr[14] : ~pgm_addr[14]),
	.re(ext_pgm_data_en ? ext_pgm_data_rd : 1'b1),
	.we(ext_pgm_data_en ? ext_pgm_data_wr : 1'b0),
	.a(ext_pgm_data_en ? ext_pgm_addr : pgm_addr),
	.d_in(ext_pgm_data_in),
	.d_out(pgm_data_app)
);
wire [15:0]pgm_data_boot;
mega_ram  #(
	.PLATFORM(PLATFORM),
	.MEM_MODE("SRAM"), // "BLOCK","SRAM"
	.ADDR_BUS_WIDTH(`ROM_ADDR_WIDTH),
	.DATA_BUS_WIDTH(16),
	.RAM_PATH(ROM_PATH)
)rom_boot(
	.clk(core_clk),
	.cs(ext_pgm_data_en ? ext_pgm_addr[14] : pgm_addr[14]),
	.re(ext_pgm_data_en ? ext_pgm_data_rd : 1'b1),
	.we(ext_pgm_data_en ? ext_pgm_data_wr : 1'b0),
	.a(ext_pgm_data_en ? ext_pgm_addr : pgm_addr),
	.d_in(ext_pgm_data_in),
	.d_out(pgm_data_boot)
);
assign pgm_data = pgm_data_app | pgm_data_boot;
assign ext_pgm_data_out = pgm_data;
/*
mega_rom  #(
.ADDR_ROM_BUS_WIDTH(`ROM_ADDR_WIDTH),
.ROM_PATH(ROM_PATH)
)rom(
	.clk(core_clk),
	.a(pgm_addr),
	.d(pgm_data)
);*/
/* !ROM APP */

/* RAM */
wire [7:0]ram_bus_out;
mega_ram  #(
	.PLATFORM(PLATFORM),
	.MEM_MODE("SRAM"), // "BLOK","SRAM"
	.ADDR_BUS_WIDTH(`RAM_ADDR_WIDTH),
	.DATA_BUS_WIDTH(8),
	.RAM_PATH("")
)ram(
	.clk(core_clk),
	.cs(ram_sel),
	.re(data_read),
	.we(data_write),
	.a(data_addr[`RAM_ADDR_WIDTH-1:0]/* - `RESERVED_RAM_FOR_IO*/),
	.d_in(core_data_out),
	.d_out(ram_bus_out)
);
/* !RAM */

/* DATA BUS IN DEMULTIPLEXER */
io_bus_dmux #(
	.NR_OF_BUSSES_IN(14)
	)
	ram_bus_dmux_inst(
	.bus_in({
	ram_bus_out,
	dat_pb_d_out,
	dat_pc_d_out,
	dat_pd_d_out,
	dat_pe_d_out,
	dat_pf_d_out,
	dat_spi_d_out,
	dat_tim0_d_out,
	dat_tim1_d_out,
	dat_tim3_d_out,
	dat_tim4_d_out,
	dat_pll_d_out,
	dat_eeprom_d_out,
	dat_uart0_d_out
	}),
	.bus_out(core_data_in)
	);
/* !DATA BUS IN DEMULTIPLEXER */

/* ATMEGA CORE */
wire [`BUS_ADDR_DATA_LEN-1:0]data_addr_core;
wire [7:0]core_data_out_core;
wire data_write_core;
wire data_read_core;
assign data_write = ext_ram_data_en ? ext_ram_data_wr : data_write_core;
assign data_read = ext_ram_data_en ? ext_ram_data_rd : data_read_core;
assign data_addr = ext_ram_data_en ? ext_ram_addr : data_addr_core;
assign core_data_out = ext_ram_data_en ? ext_ram_data_in : core_data_out_core;
assign ext_ram_data_out = core_data_in;


mega # (
	.PLATFORM(PLATFORM),
	.CORE_TYPE(`CORE_TYPE),
	.ROM_ADDR_WIDTH(`ROM_ADDR_WIDTH),
	.RAM_ADDR_WIDTH(`BUS_ADDR_DATA_LEN),
	.WATCHDOG_CNT_WIDTH(`WATCHDOG_CNT_WIDTH),/* If is 0 the watchdog is disabled */
	.VECTOR_INT_TABLE_SIZE(`VECTOR_INT_TABLE_SIZE),/* If is 0 the interrupt module is disabled */
	.USE_HALT(USE_HALT),
	.REGS_REGISTERED(REGS_REGISTERED)
	)atmega32u4_inst(
	.rst(rst),
	.sys_rst_out(wdt_rst),
	// Core clock.
	.clk(core_clk),
	// Watchdog clock input that can be different from the core clock.
	.clk_wdt(core_clk),
	// Used to halt the core.
	.halt(halt),
	.halt_ack(halt_ack),
	// FLASH space data interface.
	.pgm_addr(pgm_addr),
	.pgm_data(pgm_data),
	// RAM space data interface.
	.data_addr(data_addr_core),
	.data_out(core_data_out_core),
	.data_write(data_write_core),
	.data_in(core_data_in),
	.data_read(data_read_core),
	// Interrupt lines from all IO's.
	.int_sig({
	int_timer4_fpf, int_timer4_ovf, int_timer4_compd, int_timer4_compb, int_timer4_compa,
	int_spm_ready,
	int_twi,
	int_timer3_ovf, int_timer3_compc, int_timer3_compb, int_timer3_compa, int_timer3_capt,
	int_ee_ready,
	int_adc,
	int_analog_comp,
	int_usart1_tx, int_usart1_udre, int_usart1_rx,
	int_spi_stc,
	int_timer0_ovf, int_timer0_compb, int_timer0_compa,
	int_timer1_ovf, int_timer1_compc, int_timer1_compb, int_timer1_compa, int_timer1_capt, 
	int_reserved6, int_reserved5, int_reserved4,
	int_wdt,
	int_usb_endpoint, int_usb_general,
	int_pcint0,
	int_reserved3,
	int_int6,
	int_reserved1, int_reserved0,
	int_int3, int_int2, int_int1, int_int0}
	),
	// Interrupt reset lines going to all IO's.
	.int_rst({
	int_timer4_fpf_rst, int_timer4_ovf_rst, int_timer4_compd_rst, int_timer4_compb_rst, int_timer4_compa_rst,
	int_spm_ready_rst,
	int_twi_rst,
	int_timer3_ovf_rst, int_timer3_compc_rst, int_timer3_compb_rst, int_timer3_compa_rst, int_timer3_capt_rst,
	int_ee_ready_rst,
	int_adc_rst,
	int_analog_comp_rst,
	int_usart1_tx_rst, int_usart1_udre_rst, int_usart1_rx_rst,
	int_spi_stc_rst,
	int_timer0_ovf_rst, int_timer0_compb_rst, int_timer0_compa_rst,
	int_timer1_ovf_rst, int_timer1_compc_rst, int_timer1_compb_rst, int_timer1_compa_rst, int_timer1_capt_rst, 
	int_reserved6_rst, int_reserved5_rst, int_reserved4_rst,
	int_wdt_rst,
	int_usb_endpoint_rst, int_usb_general_rst,
	int_pcint0_rst,
	int_reserved3_rst,
	int_int6_rst,
	int_reserved1_rst, int_reserved0_rst,
	int_int3_rst, int_int2_rst, int_int1_rst, int_int0_rst}
	)
);
/* !ATMEGA CORE */

endmodule
