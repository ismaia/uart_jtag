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

module uart_transmitter_jtag (clk, wb_rst_i, lcr, tf_push, wb_dat_i, enable,	stx_pad_o, tstate, tf_count, tx_reset, lsr_mask, // name changed by Elmar
                               tf_data_out, tf_jtag_can_write, tf_pop);

input 					clk;
input 					wb_rst_i;
input [7:0] 				lcr;
input 					tf_push;
input [7:0] 				wb_dat_i;
input 					enable;
input 					tx_reset;
input 					lsr_mask; //reset of fifo
output 					stx_pad_o;
output [2:0] 				tstate;
output [`UART_FIFO_COUNTER_W-1:0] 	tf_count;

output  [`UART_FIFO_WIDTH-1:0]          tf_data_out;
input                                   tf_jtag_can_write;  // indicates that tf_data_out can be send to jtag
output                                  tf_pop;

reg [2:0] 				tstate;
reg 					tf_pop;

// TX FIFO instance
//
// Transmitter FIFO signals
wire [`UART_FIFO_WIDTH-1:0] 		tf_data_in;
wire 					tf_push;
wire 					tf_overrun;
wire [`UART_FIFO_COUNTER_W-1:0] 	tf_count;


assign 		tf_data_in = wb_dat_i;

uart_tfifo fifo_tx(	// error bit signal is not used in transmitter FIFO
	.clk(		clk		), 
	.wb_rst_i(	wb_rst_i	),
	.data_in(	tf_data_in	),
	.data_out(	tf_data_out	),
	.push(		tf_push		),
	.pop(		tf_pop		),
	.overrun(	tf_overrun	),
	.count(		tf_count	),
	.fifo_reset(	tx_reset	),
	.reset_status(lsr_mask)
);

// TRANSMITTER FINITE STATE MACHINE

parameter s_idle        = 3'd0;
parameter s_send_start  = 3'd1;
parameter s_send_byte   = 3'd2;
parameter s_send_parity = 3'd3;
parameter s_send_stop   = 3'd4;
parameter s_pop_byte    = 3'd5;

always @(posedge clk or posedge wb_rst_i)
begin
  if (wb_rst_i)
  begin
	tstate       <=  s_idle;
	tf_pop      <=  1'b0;
  end
  else
  if (enable)
  begin
	case (tstate)
	s_idle	 :	if ( tf_count==0 || tf_jtag_can_write==0)
			begin
				tf_pop <=  1'b0;
				tstate <=  s_idle;
			end
			else
			begin
				tf_pop <=  1'b0;
				tstate  <=  s_pop_byte;
			end
	s_pop_byte :	begin
				tf_pop <=  1'b1;
				tstate <=  s_send_start;
			end
	s_send_start :	begin
				tf_pop <=  1'b0;
				tstate <=  s_idle;
			end
	default : // should never get here
			tstate <=  s_idle;
	endcase
  end // end if enable
  else
    tf_pop <=  1'b0;  // tf_pop must be 1 cycle width
end // transmitter logic

assign stx_pad_o = 1;

endmodule
