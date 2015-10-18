//////////////////////////////////////////////////////////////////////
////                                                              ////
////  uart16550_jtag.v                                            ////
////                                                              ////
////                                                              ////
////  This file is part of the "UART 16550 compatible" project    ////
////  http://www.opencores.org/cores/uart16550/                   ////
////                                                              ////
////  Documentation related to this project:                      ////
////  - http://www.opencores.org/cores/uart16550/                 ////
////                                                              ////
////  Projects compatibility:                                     ////
////  - WISHBONE                                                  ////
////  JTAG Atlantic Protocol                                      ////
////  16550D uart (mostly supported)                              ////
////                                                              ////
////  Overview (main Features):                                   ////
////  UART core top level.                                        ////
////                                                              ////
////  Known problems (limits):                                    ////
////  Note that transmitter and receiver instances are inside     ////
////  the uart_regs.v file.                                       ////
////                                                              ////
////  To Do:                                                      ////
////  Nothing so far.                                             ////
////                                                              ////
////  Author(s):                                                  ////
////      - gorban@opencores.org                                  ////
////      - Jacob Gorban                                          ////
////      - Igor Mohor (igorm@opencores.org)                      ////
////                                                              ////
////  Created:        2001/05/12                                  ////
////  Last Updated:   2013/10/10                                  ////
////                  (See log for the revision history)          ////
////                                                              ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000, 2001 Authors                             ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
//
// Revision History
//
// Revision 1.50 1013/10/10 Elmar Melcher (elmar@dsc.ufcg.edu.br)
// Implementation of JTAG Atlantic Protocal
//  * Communication via JTAG-atlantic interface: uart-jtag bridge
//
// Revision 1.18  2002/07/22 23:02:23  gorban
// Bug Fixes:
//  * Possible loss of sync and bad reception of stop bit on slow baud rates fixed.
//   Problem reported by Kenny.Tung.
//  * Bad (or lack of ) loopback handling fixed. Reported by Cherry Withers.
//
// Improvements:
//  * Made FIFO's as general inferrable memory where possible.
//  So on FPGA they should be inferred as RAM (Distributed RAM on Xilinx).
//  This saves about 1/3 of the Slice count and reduces P&R and synthesis times.
//
//  * Added optional baudrate output (baud_o).
//  This is identical to BAUDOUT* signal on 16550 chip.
//  It outputs 16xbit_clock_rate - the divided clock.
//  It's disabled by default. Define UART_HAS_BAUDRATE_OUTPUT to use.
//
// Revision 1.17  2001/12/19 08:40:03  mohor
// Warnings fixed (unused signals removed).
//
// Revision 1.16  2001/12/06 14:51:04  gorban
// Bug in LSR[0] is fixed.
// All WISHBONE signals are now sampled, so another wait-state is introduced on all transfers.
//
// Revision 1.15  2001/12/03 21:44:29  gorban
// Updated specification documentation.
// Added full 32-bit data bus interface, now as default.
// Address is 5-bit wide in 32-bit data bus mode.
// Added wb_sel_i input to the core. It's used in the 32-bit mode.
// Added debug interface with two 32-bit read-only registers in 32-bit mode.
// Bits 5 and 6 of LSR are now only cleared on TX FIFO write.
// My small test bench is modified to work with 32-bit mode.
//
// Revision 1.14  2001/11/07 17:51:52  gorban
// Heavily rewritten interrupt and LSR subsystems.
// Many bugs hopefully squashed.
//
// Revision 1.13  2001/10/20 09:58:40  gorban
// Small synopsis fixes
//
// Revision 1.12  2001/08/25 15:46:19  gorban
// Modified port names again
//
// Revision 1.11  2001/08/24 21:01:12  mohor
// Things connected to parity changed.
// Clock devider changed.
//
// Revision 1.10  2001/08/23 16:05:05  mohor
// Stop bit bug fixed.
// Parity bug fixed.
// WISHBONE read cycle bug fixed,
// OE indicator (Overrun Error) bug fixed.
// PE indicator (Parity Error) bug fixed.
// Register read bug fixed.
//
// Revision 1.4  2001/05/31 20:08:01  gorban
// FIFO changes and other corrections.
//
// Revision 1.3  2001/05/21 19:12:02  gorban
// Corrected some Linter messages.
//
// Revision 1.2  2001/05/17 18:34:18  gorban
// First 'stable' release. Should be sythesizable now. Also added new header.
//
// Revision 1.0  2001-05-17 21:27:12+02  jacob
// Initial revision
//

// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

`include "uart_defines.v"

module uart_receiver_jtag (clk, wb_rst_i, lcr, rf_pop, srx_pad_i, enable, 
		      counter_t, rf_count, rf_data_out, rf_error_bit, rf_overrun, rx_reset, lsr_mask, rstate, rf_push_pulse,
                      jtag_t_dat, jtag_t_dav, jtag_t_ena);

   input				clk;
   input				wb_rst_i;
   input [7:0] 				lcr;
   input				rf_pop;
   input				srx_pad_i;
   input				enable;
   input				rx_reset;
   input 				lsr_mask;

   output [9:0] 			counter_t;
   output [`UART_FIFO_COUNTER_W-1:0] 	rf_count;
   output [`UART_FIFO_REC_WIDTH-1:0] 	rf_data_out;
   output				rf_overrun;
   output                               rf_push_pulse;
   output				rf_error_bit;
   output [3:0] 			rstate;

   //JTAG Atlantic
   input  [`UART_FIFO_WIDTH-1:0]        jtag_t_dat;
   output                               jtag_t_dav;
   input                                jtag_t_ena;

   // RX FIFO signals
   wire [`UART_FIFO_REC_WIDTH-1:0] 	rf_data_out;
   wire [`UART_FIFO_WIDTH-1:0]          rf_data_in;
   wire 				rf_pop;
   reg  				rf_push_pulse;
   wire 				rf_overrun;
   wire [`UART_FIFO_COUNTER_W-1:0] 	rf_count;
   wire 				rf_error_bit; // an error (parity or framing) is inside the fifo

   wire rfifo_empty, rfifo_full;
   reg  rfifo_rd;
   reg  jtag_t_dav;

   // RX FIFO instance
   uart_rfifo #(`UART_FIFO_REC_WIDTH) fifo_rx(
					      .clk(		clk		), 
					      .wb_rst_i(	wb_rst_i	),
					      .data_in(	{rf_data_in, 3'b000} ),
					      .data_out(	rf_data_out	),
					      .push( rf_push_pulse ),
					      .pop(		rf_pop		),
					      .overrun(	rf_overrun	),
					      .count(		rf_count	),
					      .error_bit(	rf_error_bit	),
					      .fifo_reset(	rx_reset	),
					      .reset_status(lsr_mask)
					      );

   // 0123456789abcdefghijklmnopqrst

  	scfifo # (
		.lpm_hunt ("RAM_BLOCK_TYPE=AUTO"),
		.lpm_numwords (64),
		.lpm_showahead ("OFF"),
		.lpm_type ("scfifo"),
		.lpm_width (8),
		.lpm_widthu (6),
		.overflow_checking ("OFF"),
		.underflow_checking ("OFF"),
		.use_eab ("ON")
	) rfifo (
		.clock (clk),
		.data (jtag_t_dat),
		.empty (rfifo_empty),
		.full (rfifo_full),
		.q (rf_data_in),
		.rdreq (rfifo_rd),
		.usedw (),
		.wrreq (jtag_t_ena & ~rfifo_full)
	);

 	always @ (posedge clk or posedge wb_rst_i)
		if (wb_rst_i) begin
                   jtag_t_dav <= 1;
		end else begin
                   jtag_t_dav <= ~rfifo_full;
		end

        reg [12:0] count; 
 	always @ (posedge clk or posedge wb_rst_i)
		if (wb_rst_i) begin
                   rfifo_rd <= 0;
                   rf_push_pulse <= 0;
                   count <= -1;
		end else begin
                   if(rfifo_rd) begin
                      rfifo_rd <= 0;
                      rf_push_pulse <= 1;
                   end
                   if(rf_push_pulse) rf_push_pulse <= 0;
                   else begin
                      if(count>0) count <= count-1;
                      if(rf_count < `UART_FIFO_DEPTH-2 && rfifo_empty==0 && count==0) begin
                         rfifo_rd <= 1;
                         count <= 4340; // char rate at 115 kbaud
                      end
                   end
		end


endmodule
