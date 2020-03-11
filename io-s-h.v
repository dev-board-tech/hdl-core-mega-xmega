/*
 * This is the simplifyed IO header file definition.
 * 
 * Copyright (C) 2017  Iulian Gheorghiu
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
/*
--------------------------------------------------------------------------
RTC - Real time counter interface
--------------------------------------------------------------------------
*/

`define RTC_CNT							'h00  
`define RTC_PERIOD						'h04 
`define RTC_STATUS						'h08 

/*
--------------------------------------------------------------------------
LCD - LCD display Interface
--------------------------------------------------------------------------
*/

`define LCD_CTRL					0
`define LCD_H_RES_LOW				1
`define LCD_H_RES_HIGH				2
`define LCD_H_PULSE_WIDTH			3
`define LCD_H_BACK_PORCH			4
`define LCD_H_FRONT_PORCH			5
`define LCD_V_RES_LOW				6
`define LCD_V_RES_HIGH				7
`define LCD_V_PULSE_WIDTH			8
`define LCD_V_BACK_PORCH			9
`define LCD_V_FRONT_PORCH			10
`define LCD_PIXEL_SIZE				11
`define LCD_BASE_ADDR_BYTE0			12
`define LCD_BASE_ADDR_BYTE1			13
`define LCD_BASE_ADDR_BYTE2			14
`define LCD_BASE_ADDR_BYTE3			15

`define EN_bp						1
`define HSYNK_INVERTED_bp			2
`define VSYNK_INVERTED_bp			4
`define DATA_ENABLE_INVERTED_bp		8

/*
--------------------------------------------------------------------------
GFX_ACCEL - GFX_ACCEL LCD display 2D accelerator interface
--------------------------------------------------------------------------
*/

`define GFX_ACCEL_CMD				16
`define GFX_ACCEL_CLIP_X_MIN_L		17
`define GFX_ACCEL_CLIP_X_MIN_H		18
`define GFX_ACCEL_CLIP_X_MAX_L		19
`define GFX_ACCEL_CLIP_X_MAX_H		20
`define GFX_ACCEL_CLIP_Y_MIN_L		21
`define GFX_ACCEL_CLIP_Y_MIN_H		22
`define GFX_ACCEL_CLIP_Y_MAX_L		23
`define GFX_ACCEL_CLIP_Y_MAX_H		24
`define GFX_ACCEL_COLOR_BYTE_0		25
`define GFX_ACCEL_COLOR_BYTE_1		26
`define GFX_ACCEL_COLOR_BYTE_2		27
`define GFX_ACCEL_COLOR_BYTE_3		28

`define GFX_ACCEL_CMD_IDLE			0
`define GFX_ACCEL_CMD_VRAM_ACCESS	1
`define GFX_ACCEL_CMD_PIXEL_LOAD	2
`define GFX_ACCEL_CMD_PIXEL			3
`define GFX_ACCEL_CMD_CTRL_ACCESS	4
`define GFX_ACCEL_CMD_FILL_RECT		5
`define GFX_ACCEL_CMD_OFF			254
`define GFX_ACCEL_CMD_ON			254
