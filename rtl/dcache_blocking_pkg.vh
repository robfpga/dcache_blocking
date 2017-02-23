`ifndef DCACHE_BLOCKING_PKG_VH
`define DCACHE_BLOCKING_PKG_VH

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

package dcache_blocking_pkg;

  // Cache parameterizations
  //

//  typedef logic [ADDR_W-1:0] addr_t;

  localparam int DATA_W  = 32;
  typedef logic [DATA_W-1:0] data_t;

  //
  localparam int CACHE_SIZE_B  = (128 * 1024);

  //
  localparam int CACHE_LINE_B  = (256 / 8);
  localparam int CACHE_LINE_ADDR_W  = $clog2(CACHE_LINE_B);
  typedef logic [CACHE_LINE_ADDR_W-1:0] cache_line_addr_t;

  //
  localparam int CACHE_WAYS_N  = 4;

  //
  localparam int RAM_DAT_B  = 8;

  //
  typedef enum logic [1:0] {
      NOP   = 2'b00
    , LOAD  = 2'b01
    , STORE = 2'b10
    , INV   = 2'b11
  } op_t;
  localparam int OP_W = $bits(op_t);

  typedef struct packed {
    data_t   data;
  } resp_t;
  localparam int RESP_W = $bits(resp_t);

  localparam int ADDR_W  = 32;

  // Derived
  //
  localparam int RAM_TAG_N  = (CACHE_SIZE_B / CACHE_LINE_B) / CACHE_WAYS_N;

  //
  localparam int RAM_TAG_ADDR_W  = $clog2(RAM_TAG_N);
  typedef logic [RAM_TAG_ADDR_W-1:0] cache_line_off_t;

  //
  localparam int WAY_SIZE_B  = (CACHE_SIZE_B / CACHE_WAYS_N);

  //
  localparam int RAM_DAT_W  = RAM_DAT_B * 8;

  //
  localparam int RAM_DAT_WORDS_N = RAM_DAT_W / DATA_W;

  //
  localparam int RAM_DAT_OFF_W  = $clog2(RAM_DAT_B);
  typedef logic [RAM_DAT_OFF_W-1:0] ram_dat_off_t;

  typedef logic [RAM_DAT_W-1:0] ram_dat_dat_t;
  typedef ram_dat_dat_t [CACHE_WAYS_N-1:0] ram_dat_dat_n_t;

  //
  localparam int RAM_DAT_LINE_N  = CACHE_LINE_B / RAM_DAT_B;
  typedef logic [RAM_DAT_LINE_N-1:0] ram_dat_bnk_n_t;

  localparam int RAM_DAT_LINE_W  = $clog2(RAM_DAT_LINE_N);
  typedef logic [RAM_DAT_LINE_W-1:0] ram_dat_line_t;

  //
  localparam int RAM_DAT_N  = CACHE_SIZE_B / CACHE_WAYS_N / RAM_DAT_LINE_N;

  //
  localparam int RAM_DAT_ADDR_W  = $clog2(RAM_DAT_N);

  //
  localparam int CACHE_LINE_TAG_W  = ADDR_W - CACHE_LINE_ADDR_W - RAM_TAG_ADDR_W;
  typedef logic [CACHE_LINE_TAG_W-1:0] cache_line_tag_t;


  // ======================================================================== //
  //                                                                          //
  // Common Data Types                                                        //
  //                                                                          //
  // ======================================================================== //

  localparam int ADDR_BNK_PAD_W  = ADDR_W - RAM_DAT_LINE_W - RAM_DAT_OFF_W;

  //
  typedef union packed {

    // Physical Format
    struct packed {
      logic [ADDR_BNK_PAD_W - 1:0] __pad;
      ram_dat_line_t        b;
      ram_dat_off_t         o;
    } p;

    // Logical Format
    struct packed {
      cache_line_tag_t      t;
      cache_line_off_t      o;
      cache_line_addr_t     l;
    } l;

    // Unpacked
    logic [ADDR_W-1:0]      a;
  } addr_t;

  //
  typedef struct            packed {
    op_t     op;
    addr_t   addr;
    data_t   data;
  } cmd_t;
  localparam int CMD_W = $bits(cmd_t);

  //
  typedef logic [CACHE_WAYS_N-1:0] ways_t;

  //
  localparam int MEM_DATA_W  = RAM_DAT_W;
  typedef logic [MEM_DATA_W-1:0] mem_data_t;

endpackage // dcache_blocking_pkg

`endif
