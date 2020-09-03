/*
 * This IP is the MEGA/XMEGA TOP implementation.
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

`define REV							"1.1"

`define PLATFORM					"XILINX"
`define FLASH_ROM_FILE_NAME			"Breakout-v.Division"

`define USE_HDMI_OUTPUT				"FALSE"

`define USE_EXTERNAL_SSD1306		"FALSE"
`define USE_SSD1306_REGISTERED_RAM	"TRUE"

`include "mega-def.v"

module arduboy_top # (
	parameter SIMULATE = "FALSE"
	)(
	input rst,
	input clk,
	inout [7:0]ja, // ja[7]=Buzzer2, ja[6]=Buzzer1, ja[5]=OledRst, ja[4]=OledCS, ja[3]=OledDC, ja[2]=OledSCL, ja[1]=OledMOSI
	output [1:0]jb,
	inout [7:0]LED, // LED[0] = Green, LED[1] = RED, LED[2] = BLUE, 
	inout [7:0]SW,// SW[0] = Button B
	inout btnc,// Button A
	inout btnd,// Button Down
	inout btnl,// Button Left
	inout btnr,// Button Right
	inout btnu, // Button Up
	inout UART_TXD,
	inout UART_RXD/*,

	output hdmi_tx_cec,
	output hdmi_tx_clk_n,
	output hdmi_tx_clk_p,
	input hdmi_tx_hpd,
	output hdmi_tx_rscl,
	inout hdmi_tx_rsda,
	output [2:0]hdmi_tx_n,
	output [2:0]hdmi_tx_p*/
);


reg rst_reg;
wire pll_locked;
wire pll_hdmi_locked;
wire sys_rst = ~rst_reg | ~pll_locked | ~pll_hdmi_locked;
wire sys_clk;// = clk;
wire pll_clk;// = clk;
wire hdmi_clk;// = clk;
wire clkfb;
wire clkfb_hdmi;
wire ntsc_clk;


PLLE2_BASE #(
	.BANDWIDTH("OPTIMIZED"),	// OPTIMIZED, HIGH, LOW
	.CLKFBOUT_MULT(16),			// Multiply value for all CLKOUT, (2-64)
	.CLKFBOUT_PHASE(0.0),		// Phase offset in degrees of CLKFB, (-360.000-360.000).
	.CLKIN1_PERIOD(10.0),		// Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz).
	// CLKOUT0_DIVIDE - CLKOUT5_DIVIDE: Divide amount for each CLKOUT (1-128)
	.CLKOUT0_DIVIDE(100), // Core clock 16Mhz.
	.CLKOUT1_DIVIDE(8), // PLL clock ~192Mhz.
	.CLKOUT2_DIVIDE(1), // HDMI clock.
	.CLKOUT3_DIVIDE(1),
	.CLKOUT4_DIVIDE(1),
	.CLKOUT5_DIVIDE(1),
	// CLKOUT0_DUTY_CYCLE - CLKOUT5_DUTY_CYCLE: Duty cycle for each CLKOUT (0.001-0.999).
	.CLKOUT0_DUTY_CYCLE(0.5),
	.CLKOUT1_DUTY_CYCLE(0.5),
	.CLKOUT2_DUTY_CYCLE(0.5),
	.CLKOUT3_DUTY_CYCLE(0.5),
	.CLKOUT4_DUTY_CYCLE(0.5),
	.CLKOUT5_DUTY_CYCLE(0.5),
	// CLKOUT0_PHASE - CLKOUT5_PHASE: Phase offset for each CLKOUT (-360.000-360.000).
	.CLKOUT0_PHASE(0.0),
	.CLKOUT1_PHASE(0.0),
	.CLKOUT2_PHASE(0.0),
	.CLKOUT3_PHASE(0.0),
	.CLKOUT4_PHASE(0.0),
	.CLKOUT5_PHASE(0.0),
	.DIVCLK_DIVIDE(1),			// Master division value, (1-56)
	.REF_JITTER1(0.0),			// Reference input jitter in UI, (0.000-0.999).
	.STARTUP_WAIT("TRUE")		// Delay DONE until PLL Locks, ("TRUE"/"FALSE")
)
PLLE2_CORE_BASE_inst (
	// Clock Outputs: 1-bit (each) output: User configurable clock outputs
	.CLKOUT0(sys_clk),			// 1-bit output: CLKOUT0
	.CLKOUT1(pll_clk),			// 1-bit output: CLKOUT1
	.CLKOUT2(),			// 1-bit output: CLKOUT2
	.CLKOUT3(),					// 1-bit output: CLKOUT3
	.CLKOUT4(),					// 1-bit output: CLKOUT4
	.CLKOUT5(),					// 1-bit output: CLKOUT5
	// Feedback Clocks: 1-bit (each) output: Clock feedback ports
	.CLKFBOUT(clkfb),			// 1-bit output: Feedback clock
	.LOCKED(pll_locked),		// 1-bit output: LOCK
	.CLKIN1(clk),				// 1-bit input: Input clock
	// Control Ports: 1-bit (each) input: PLL control ports
	.PWRDWN(),					// 1-bit input: Power-down
	.RST(~rst),					// 1-bit input: Reset
	// Feedback Clocks: 1-bit (each) input: Clock feedback ports
	.CLKFBIN(clkfb)				// 1-bit input: Feedback clock
);

PLLE2_BASE #(
	.BANDWIDTH("OPTIMIZED"),	// OPTIMIZED, HIGH, LOW
	.CLKFBOUT_MULT(15),			// Multiply value for all CLKOUT, (2-64)
	.CLKFBOUT_PHASE(0.0),		// Phase offset in degrees of CLKFB, (-360.000-360.000).
	.CLKIN1_PERIOD(10.0),		// Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz).
	// CLKOUT0_DIVIDE - CLKOUT5_DIVIDE: Divide amount for each CLKOUT (1-128)
	.CLKOUT0_DIVIDE(4), // HDMI clock 375Mhz.
	.CLKOUT1_DIVIDE(32), // NTSC clock 50Mhz.
	.CLKOUT2_DIVIDE(1),
	.CLKOUT3_DIVIDE(1),
	.CLKOUT4_DIVIDE(1),
	.CLKOUT5_DIVIDE(1),
	// CLKOUT0_DUTY_CYCLE - CLKOUT5_DUTY_CYCLE: Duty cycle for each CLKOUT (0.001-0.999).
	.CLKOUT0_DUTY_CYCLE(0.5),
	.CLKOUT1_DUTY_CYCLE(0.5),
	.CLKOUT2_DUTY_CYCLE(0.5),
	.CLKOUT3_DUTY_CYCLE(0.5),
	.CLKOUT4_DUTY_CYCLE(0.5),
	.CLKOUT5_DUTY_CYCLE(0.5),
	// CLKOUT0_PHASE - CLKOUT5_PHASE: Phase offset for each CLKOUT (-360.000-360.000).
	.CLKOUT0_PHASE(0.0),
	.CLKOUT1_PHASE(0.0),
	.CLKOUT2_PHASE(0.0),
	.CLKOUT3_PHASE(0.0),
	.CLKOUT4_PHASE(0.0),
	.CLKOUT5_PHASE(0.0),
	.DIVCLK_DIVIDE(1),			// Master division value, (1-56)
	.REF_JITTER1(0.0),			// Reference input jitter in UI, (0.000-0.999).
	.STARTUP_WAIT("TRUE")		// Delay DONE until PLL Locks, ("TRUE"/"FALSE")
)
PLLE2_HDMI_BASE_inst (
	// Clock Outputs: 1-bit (each) output: User configurable clock outputs
	.CLKOUT0(hdmi_clk),			// 1-bit output: CLKOUT0
	.CLKOUT1(ntsc_clk),			// 1-bit output: CLKOUT1
	.CLKOUT2(),			// 1-bit output: CLKOUT2
	.CLKOUT3(),					// 1-bit output: CLKOUT3
	.CLKOUT4(),					// 1-bit output: CLKOUT4
	.CLKOUT5(),					// 1-bit output: CLKOUT5
	// Feedback Clocks: 1-bit (each) output: Clock feedback ports
	.CLKFBOUT(clkfb_hdmi),			// 1-bit output: Feedback clock
	.LOCKED(pll_hdmi_locked),		// 1-bit output: LOCK
	.CLKIN1(clk),				// 1-bit input: Input clock
	// Control Ports: 1-bit (each) input: PLL control ports
	.PWRDWN(),					// 1-bit input: Power-down
	.RST(~rst),					// 1-bit input: Reset
	// Feedback Clocks: 1-bit (each) input: Clock feedback ports
	.CLKFBIN(clkfb_hdmi)				// 1-bit input: Feedback clock
);

always @ (posedge sys_clk) 
begin
	if(~rst)
		rst_reg <= 1'b1;
	else
		rst_reg <= rst;
end


wire pb_dummy_0 = 0;
wire pb_dummy_3 = 0;
wire [5:0]pc_dummy_5_0 = 0;
wire pd_dummy_5 = 0;
wire [1:0]pd_dummy_1_0 = 0;
wire [1:0]pe_dummy_1_0 = 0;
wire pe_dummy_2 = 0;
wire [2:0]pe_dummy_5_3 = 0;
wire pe_dummy_7 = 0;
wire [3:0]pf_dummy_3_0 = 0;
	
wire eep_content_modifyed;
/* !Interface for debug unit. */

/* Debug wires */
wire [4:0]debug;
/* Debug wires */

wire lcd_clk;
wire lcd_h;
wire lcd_v;
wire lcd_de;
wire [7:0]lcd_r;
wire [7:0]lcd_g;
wire [7:0]lcd_b;
wire [12:0]lcd_h_cnt;
wire [12:0]lcd_v_cnt;
wire [31:0]ssd1306_rgb_data;

generate
if (`USE_EXTERNAL_SSD1306 != "TRUE" && SIMULATE != "TRUE")
begin
if(`USE_HDMI_OUTPUT == "TRUE")
begin
vga_simple #(
	//.MASTER("TRUE"),
	.DEBUG(""),//"PATERN_RASTER"
	.DISPLAY_CFG("1920_1080_30_DISPLAY_74_25_Mhz"),
	.VRAM_BUFFERED_OUTPUT(`USE_SSD1306_REGISTERED_RAM),
	
	.ADDRESS(0),
	.BUS_VRAM_ADDR_LEN(24),
	.PIXEL_SIZE_CONF(1),
	
	.H_RES_CONF(800),
	.H_BACK_PORCH_CONF(46),
	.H_FRONT_PORCH_CONF(210),
	.H_PULSE_WIDTH_CONF(2),
	.V_RES_CONF(480),
	.V_BACK_PORCH_CONF(23),
	.V_FRONT_PORCH_CONF(22),
	.V_PULSE_WIDTH_CONF(2),
	.HSYNK_INVERTED_CONF(1'b1),
	.VSYNK_INVERTED_CONF(1'b1),
	.DATA_ENABLE_INVERTED_CONF(1'b0),
	
	.COLOR_INVERTED("FALSE"),

	.DEDICATED_VRAM_SIZE(0)
)vga_inst(
	.rst_i(sys_rst),

	.lcd_clk_i(lcd_clk),
	.lcd_h_synk_o(lcd_h),
	.lcd_v_synk_o(lcd_v),
	.lcd_r_o(lcd_r),
	.lcd_g_o(lcd_g),
	.lcd_b_o(lcd_b),
	.lcd_de_o(lcd_de),
	
	.vram_addr_o(),
	.h_pos_o(lcd_h_cnt),
	.v_pos_o(lcd_v_cnt),
	.video_data_i(ssd1306_rgb_data)

);

hdmi_out #(
	.PLATFORM("XILINX_ARTIX_7")
	)hdmi_out_inst(
	.rst(sys_rst),
	.clk(hdmi_clk),
	.hdmi_tx_cec(hdmi_tx_cec),
	.hdmi_tx_clk_n(hdmi_tx_clk_n),
	.hdmi_tx_clk_p(hdmi_tx_clk_p),
	.hdmi_tx_hpd(hdmi_tx_hpd),
	.hdmi_tx_rscl(hdmi_tx_rscl),
	.hdmi_tx_rsda(hdmi_tx_rsda),
	.hdmi_tx_n(hdmi_tx_n),
	.hdmi_tx_p(hdmi_tx_p),
	
	.lcd_clk_out(lcd_clk),
	.lcd_h_synk(lcd_h),
	.lcd_v_synk(lcd_v),
	.lcd_r(lcd_r),
	.lcd_g(lcd_g),
	.lcd_b(lcd_b),
	.lcd_de(lcd_de)
	);
end
else /* (`USE_HDMI_OUTPUT != "TRUE") */
begin

wire pixel_is_visible;
wire [1:0]ntsc_out;

localparam [3:0]  SIGNAL_LEVEL_SYNC         = 4'b0000,
                    SIGNAL_LEVEL_BLANK        = 4'b0001,
                    SIGNAL_LEVEL_DARK_GREY    = 4'b0011,
                    SIGNAL_LEVEL_LIGHT_GREY   = 4'b0111,
                    SIGNAL_LEVEL_WHITE        = 4'b1111;

interlaced_ntsc # (
	.PIXEL_NUANCE_DEPTH(1)
	)interlaced_ntsc_inst(
    .rst_i(sys_rst),
    .clk_i(ntsc_clk),
    .pixel_data_i(pixel_is_visible ? (ssd1306_rgb_data[0] ? SIGNAL_LEVEL_WHITE : SIGNAL_LEVEL_BLANK) : SIGNAL_LEVEL_BLANK),
    .h_sync_out_o(), // single clock tick indicating pixel_y will incrememt on next clock (for debugging)
    .v_sync_out_o(), // single clock tick indicating pixel_y will reset to 0 or 1 on next clock, depending on the field (for debugging)
    .pixel_y_o(lcd_v_cnt),    // which line
    .pixel_x_o(lcd_h_cnt),
    .pixel_is_visible_o(pixel_is_visible),
    .ntsc_out_o(ntsc_out)
);

assign jb = ntsc_out;
end

ssd1306 # (
	.X_OLED_SIZE(128),
	.Y_OLED_SIZE(64),
	.X_PARENT_SIZE((`USE_HDMI_OUTPUT != "TRUE") ? 560 : 1920),
	.Y_PARENT_SIZE((`USE_HDMI_OUTPUT != "TRUE") ? 400 : 1080),
	.PIXEL_INACTIVE_COLOR(32'h10101010),
	.PIXEL_ACTIVE_COLOR(32'hE0E0E0E0),
	.INACTIVE_DISPLAY_COLOR(32'h10101010),
	.VRAM_BUFFERED_OUTPUT(`USE_SSD1306_REGISTERED_RAM),
	.FULL_COLOR_OUTPUT("FALSE")
	)ssd1306_inst(
	.rst_i(~ja[5]),
	.clk_i(sys_clk),
	
	.edge_color_i(32'h00808080),
	.raster_x_i(lcd_h_cnt),
	.raster_y_i(lcd_v_cnt),
	.raster_clk_i((`USE_HDMI_OUTPUT != "TRUE") ? ntsc_clk : lcd_clk),
	.raster_d_o(ssd1306_rgb_data),
	
	.ss_i(ja[4]),
	.scl_i(ja[2]),
	.mosi_i(ja[1]),
	.dc_i(ja[3])
    );
end// !(USE_EXTERNAL_SSD1306 != "TRUE" && SIMULATE != "TRUE")
endgenerate
wire ld0;
wire ld1;
wire ld2;

wire [5:0]io_addr;
wire [7:0]io_out;
wire io_write;
wire [7:0]io_in;
wire io_read;

wire nmi_sig = 0;

wire uSD_CD = 0;
wire MISO = 0;

atmega32u4_arduboy # (
	.PLATFORM(`PLATFORM),
	//.BOOT_ADDR(16'hF800),

	.CORE_TYPE(`MEGA_ENHANCED_128K),
	.ROM_ADDR_WIDTH(15), // 14 = 16K Words / 32K Bytes; 15 = 32K Words / 64K Bytes; 16 = 64K Words / 128K Bytes Not supported yet.
	.BOOT_ADDR_WIDTH(10), // 1024 Words / 2048 Bytes, how big the first stage boot-loader ROM to be.
	.BUS_ADDR_DATA_LEN(16), // Max 64K Bytes.
	.RAM_TYPE("SRAM"),  // "BLOCK","SRAM"// If "SRAM" is choosen, will be a 32KB block of RAM.
	.RAM_ADDR_WIDTH(15), // 32KB, if you use "SRAM" this value need to be 15.
	.EEP_ADDR_WIDTH(10), // 1K Bytes.
	.RESERVED_RAM_FOR_IO(12'h100), // Lowest 256 Bytes of RAM addresses are reserved for IO's.
	.VECTOR_INT_TABLE_SIZE(43),// 42 of original ATmega32U4 + NMI
	.WATCHDOG_CNT_WIDTH(0),//27 // We do not use watchdog, is not a critical design and most of arduboy games does not use him.

	.ARDU_FPGA_ICE40UP5K_GAME("FALSE"),
	.REGS_REGISTERED("FALSE"),
	.ROM_PATH(`FLASH_ROM_FILE_NAME),
	.USE_PIO_B("TRUE"),
	.USE_PIO_C("TRUE"),
	.USE_PIO_D("TRUE"),
	.USE_PIO_E("TRUE"),
	.USE_PIO_F("TRUE"),
	.USE_PLL("TRUE"),
	.USE_PLL_HI_FREQ("FALSE"),
	.USE_TIMER_0("TRUE"),
	.USE_TIMER_1("FALSE"),
	.USE_TIMER_3("TRUE"),
	.USE_TIMER_4("TRUE"),
	.USE_SPI_1("TRUE"),
	.USE_UART_1("TRUE"),
	.USE_TWI_1("TRUE"),
	.USE_EEPROM("TRUE"),
	.USE_RNG_AS_ADC("TRUE")
) atmega32u4_arduboy_inst (
	.core_rst(sys_rst),
	.dev_rst(sys_rst),
	.clk(sys_clk),
	.clk_pll(pll_clk),
	.nmi_sig(nmi_sig),
	.nmi_ack(nmi_ack),

    .buttons({~SW[0], ~btnc, ~btnl, ~btnr, ~btnd, ~btnu}),
    .RGB({ld2, ld1, ld0}),
    .Buzzer1(ja[6]),
    .Buzzer2(ja[7]),
    .OledDC(ja[3]),
    .OledCS(ja[4]),
    .OledRST(ja[5]),
    .spi_scl(ja[2]),
    .spi_mosi(ja[1]),
	.spi_miso(MISO),
	.uSD_CS(uSD_SS),
	.uSD_CD(uSD_CD),
	.ADC_CS(ADC_SS),
	.VS_RST(),
	.VS_xCS(),
	.VS_xDCS(),
	.VS_DREQ(),
	.uart_tx(),
	.uart_rx(),
	.twi_scl(),
	.twi_sda(),

	.io_addr(io_addr),
	.io_out(io_out),
	.io_write(io_write),
	.io_in(io_in),
	.io_read(io_read),
	.io_sel(ram_sel),
	//.io_rst(io_rst),
	.nmi_rst()
);

assign LED[2:0] = {ld2, ld1, ld0};
assign halt = SW[7];

assign io_in = 0;


endmodule
