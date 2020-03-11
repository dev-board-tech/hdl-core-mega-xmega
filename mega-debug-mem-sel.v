/*
 * This IP is the ATMEGA debug memory selector implementation.
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


module mega_debug_mem_sel # (
	parameter BUS_ADDR_DATA_LEN = 8,
	parameter TEXT_ORIGIN = 'h000000,
	parameter TEXT_LENGTH = 'h020000,
	parameter RAM_ORIGIN = 'h800060,
	parameter RAM_LENGTH = 'h010000,
	parameter REG_ORIGIN = 'h800000,
	parameter REG_LENGTH = 'h000020,
	parameter EEP_ORIGIN = 'h810000,
	parameter EEP_LENGTH = 'h010000,
	parameter IO_ORIGIN = 'h800020,
	parameter IO_LENGTH = 'h000060
)(
	input rst,
	input clk,

	input [BUS_ADDR_DATA_LEN-1:0]addr_dat,
	input wr_dat,
	input rd_dat,
	input [7:0]bus_dat_in,
	output reg [7:0]bus_dat_out,

	input [24:0]deb_addr,
	input deb_wr,
	input [7:0]deb_data_in,
	input deb_rd,
	output reg[7:0]deb_data_out,
	input deb_en,

	output reg[16:0]ext_pgm_addr,
	output reg[15:0]ext_pgm_data_in,
	output reg ext_pgm_data_wr,
	input [15:0]ext_pgm_data_out,
	output reg ext_pgm_data_rd,
	output reg ext_pgm_data_en,
	
	output reg[15:0]ext_ram_addr,
	output reg[7:0]ext_ram_data_in,
	output reg ext_ram_data_wr,
	input [7:0]ext_ram_data_out,
	output reg ext_ram_data_rd,
	output reg ext_ram_data_en
	);


wire text_select = deb_addr < TEXT_LENGTH;
wire ram_select = (deb_addr >= RAM_ORIGIN) & (deb_addr < RAM_ORIGIN + RAM_LENGTH);
reg [7:0]TMP;

always @ *
begin
	deb_data_out = 8'h00;
	
	ext_pgm_addr = deb_addr[16:1];
	ext_pgm_data_in = {TMP, deb_data_in};
	ext_pgm_data_wr = deb_wr;
	ext_pgm_data_rd = deb_rd;
	ext_pgm_data_en = 1'b0;
	
	ext_ram_addr = deb_addr[15:0];
	ext_ram_data_in = deb_data_in;
	ext_ram_data_wr = deb_wr;
	ext_ram_data_rd = deb_rd;
	ext_ram_data_en = 1'b0;
	
	case({ram_select, text_select})
		2'b01:
		begin
			if(deb_addr[0])
				deb_data_out = ext_pgm_data_out[15:8];
			else
				deb_data_out = ext_pgm_data_out[7:0];
			ext_pgm_data_en = deb_en;
		end
		2'b10:
		begin
			deb_data_out = ext_ram_data_out;
			ext_ram_data_en = deb_en;
		end
	endcase
end

always @ (posedge clk)
begin
	if(&{text_select, deb_en, ~deb_addr[0], deb_wr})
		TMP = deb_data_in;
end

endmodule
