//========================================================================== //
// Copyright (c) 2016, Stephen Henry
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//========================================================================== //

`include "dcache_blocking_pkg.vh"

module dcache_blocking_tb (/*AUTOARG*/
  // Outputs
  fetch_accept, dcache__mem_wrbk_r, dcache__mem_valid_r, dcache__mem_sop_r,
  dcache__mem_eop_r, dcache__mem_dat_valid_r, dcache__mem_dat_r,
  dcache__mem_addr_r, dcache__busy_r, commit_valid_r, commit_load_r,
  commit_data_r,
  // Inputs
  rst, mem__dcache_valid_w, mem__dcache_sop_w, mem__dcache_eop_w,
  mem__dcache_data_w, fetch_valid, fetch_op, fetch_data, fetch_addr,
  commit_accept, clk
  );
  import dcache_blocking_pkg::*;
`include "libtb_tb_top_inc.vh"
  /*AUTOINPUT*/
  // Beginning of automatic inputs (from unused autoinst inputs)
  input logic           clk;                    // To u_uut of dcache_blocking.v
  input logic           commit_accept;          // To u_uut of dcache_blocking.v
  input logic [dcache_blocking_pkg::ADDR_W-1:0] fetch_addr;// To u_uut of dcache_blocking.v
  input logic [dcache_blocking_pkg::DATA_W-1:0] fetch_data;// To u_uut of dcache_blocking.v
  input logic [dcache_blocking_pkg::OP_W-1:0] fetch_op;// To u_uut of dcache_blocking.v
  input logic           fetch_valid;            // To u_uut of dcache_blocking.v
  input dcache_blocking_pkg::ram_dat_dat_t mem__dcache_data_w;// To u_uut of dcache_blocking.v
  input logic           mem__dcache_eop_w;      // To u_uut of dcache_blocking.v
  input logic           mem__dcache_sop_w;      // To u_uut of dcache_blocking.v
  input logic           mem__dcache_valid_w;    // To u_uut of dcache_blocking.v
  input logic           rst;                    // To u_uut of dcache_blocking.v
  // End of automatics
  /*AUTOOUTPUT*/
  // Beginning of automatic outputs (from unused autoinst outputs)
  output dcache_blocking_pkg::data_t commit_data_r;// From u_uut of dcache_blocking.v
  output logic          commit_load_r;          // From u_uut of dcache_blocking.v
  output logic          commit_valid_r;         // From u_uut of dcache_blocking.v
  output logic          dcache__busy_r;         // From u_uut of dcache_blocking.v
  output dcache_blocking_pkg::addr_t dcache__mem_addr_r;// From u_uut of dcache_blocking.v
  output dcache_blocking_pkg::ram_dat_dat_t dcache__mem_dat_r;// From u_uut of dcache_blocking.v
  output logic          dcache__mem_dat_valid_r;// From u_uut of dcache_blocking.v
  output logic          dcache__mem_eop_r;      // From u_uut of dcache_blocking.v
  output logic          dcache__mem_sop_r;      // From u_uut of dcache_blocking.v
  output logic          dcache__mem_valid_r;    // From u_uut of dcache_blocking.v
  output logic          dcache__mem_wrbk_r;     // From u_uut of dcache_blocking.v
  output logic          fetch_accept;           // From u_uut of dcache_blocking.v
  // End of automatics
  dcache_blocking #(/*AUTOINSTPARAM*/)
  u_uut (/*AUTOINST*/
         // Outputs
         .dcache__busy_r                (dcache__busy_r),
         .commit_data_r                 (commit_data_r),
         .commit_load_r                 (commit_load_r),
         .commit_valid_r                (commit_valid_r),
         .dcache__mem_addr_r            (dcache__mem_addr_r),
         .dcache__mem_dat_r             (dcache__mem_dat_r),
         .dcache__mem_dat_valid_r       (dcache__mem_dat_valid_r),
         .dcache__mem_eop_r             (dcache__mem_eop_r),
         .dcache__mem_sop_r             (dcache__mem_sop_r),
         .dcache__mem_valid_r           (dcache__mem_valid_r),
         .dcache__mem_wrbk_r            (dcache__mem_wrbk_r),
         .fetch_accept                  (fetch_accept),
         // Inputs
         .clk                           (clk),
         .commit_accept                 (commit_accept),
         .fetch_addr                    (fetch_addr[dcache_blocking_pkg::ADDR_W-1:0]),
         .fetch_data                    (fetch_data[dcache_blocking_pkg::DATA_W-1:0]),
         .fetch_op                      (fetch_op[dcache_blocking_pkg::OP_W-1:0]),
         .fetch_valid                   (fetch_valid),
         .mem__dcache_data_w            (mem__dcache_data_w),
         .mem__dcache_eop_w             (mem__dcache_eop_w),
         .mem__dcache_sop_w             (mem__dcache_sop_w),
         .mem__dcache_valid_w           (mem__dcache_valid_w),
         .rst                           (rst));
endmodule // dcache_blocking_tb

// Local Variables:
// verilog-typedef-regexp:"_t$"
// End:
