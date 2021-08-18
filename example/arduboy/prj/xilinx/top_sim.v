/*
 * This IP is the MEGA/XMEGA TOP simulation.
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
 

module top_sim(

    );

reg clk = 1;
reg rst = 1;
//always	#(5)	clk	<=	~clk;
always	#(41.66)	clk	<=	~clk;
reg [7:0]sw = 8'hFF;
wire [7:0]sw_ = sw;

wire [7:0]ja; // ja[7]=DC, ja[6]=Buzzer1, ja[5]=OledRst, ja[4]=Buzzer2, ja[3]=OledCS
wire [7:0]LED; // LED[0] = Green, LED[1] = RED, LED[2] = BLUE, 
wire [7:0]SW = 8'h00;// SW[0] = Button B
wire btnc = 1;// Button A
wire btnd = 1;// Button Down
wire btnl = 1;// Button Left
wire btnr = 1;// Button Right
wire btnu = 1; // Button Up


initial begin
	wait(clk);
	wait(~clk);
	wait(clk);
	wait(~clk);
	rst = 0;
	wait(~clk);
	wait(clk);
	wait(~clk);
	wait(clk);
	wait(~clk);
	rst = 1;
	#10000;
	//sw = 2;
	#10;
	//sw = 0;
	#50000;
	$finish;
end

arduboy_top # (
	.SIMULATE("TRUE")
)top_inst(
	.rst(rst),
	.clk(clk),

	.ja(ja), // ja[7]=DC, ja[6]=Buzzer1, ja[5]=OledRst, ja[4]=Buzzer2, ja[3]=OledCS
	.LED(LED), // LED[0] = Green, LED[1] = RED, LED[2] = BLUE, 
	.SW(SW),// SW[0] = Button B
	.btnc(btnc),// Button A
	.btnd(btnd),// Button Down
	.btnl(btnl),// Button Left
	.btnr(btnr),// Button Right
	.btnu(btnu) // Button Up
);
endmodule
