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

module dcache
(
   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   //
     input                                        clk
   , input                                        rst

   //======================================================================== //
   //                                                                         //
   // Command Interface                                                       //
   //                                                                         //
   //======================================================================== //

   //
   , output logic                                 dcache__mem_valid_r
   , output logic                                 dcache__mem_wrbk_r
   , output dcache_blocking_pkg::addr_t           dcache__mem_addr_r
   , output logic                                 dcache__mem_dat_valid_r
   , output logic                                 dcache__mem_sop_r
   , output logic                                 dcache__mem_eop_r
   , output dcache_blocking_pkg::ram_dat_dat_t    dcache__mem_dat_r

   //
   , input                                        mem__dcache_valid_w
   , input                                        mem__dcache_sop_w
   , input                                        mem__dcache_eop_w
   , input dcache_blocking_pkg::ram_dat_dat_t     mem__dcache_data_w

   //======================================================================== //
   //                                                                         //
   // Pipe Interface                                                          //
   //                                                                         //
   //======================================================================== //

   //
   , input                                        s0_valid_r
   , input dcache_blocking_pkg::cmd_t             s0_cmd_r
   //
   , input                                        s1_valid_r
   , input dcache_blocking_pkg::cmd_t             s1_cmd_r
   //
   , input                                        s2_valid_w
   , input                                        s2_valid_r
   , input dcache_blocking_pkg::cmd_t             s2_cmd_r
   //
   , input                                        s3_replay_r
   , input                                        s3_replay_inv_r

   //======================================================================== //
   //                                                                         //
   // Response Interface                                                      //
   //                                                                         //
   //======================================================================== //

   //
   , output logic                                 dcache__resp_valid_r
   , output dcache_blocking_pkg::resp_t           dcache__resp_w
   //
   , output logic                                 dcache__resp_replay_r
   , output logic                                 dcache__resp_replay_inv_r

   //======================================================================== //
   //                                                                         //
   // Status                                                                  //
   //                                                                         //
   //======================================================================== //

   , output logic                                 dcache__busy_r
);
  import dcache_blocking_pkg::*;

  //
  typedef enum logic [8:0] { DC_RESET_INIT      = 9'b1_000_00000,
                             DC_RESET_INV       = 9'b1_100_00001,
                             //
                             DC_IDLE            = 9'b0_000_00000,
                             //
                             DC_LKUP_EMIT_CMD   = 9'b1_001_01000,
                             DC_LKUP_WAIT_RESP  = 9'b1_001_01001,
                             //
                             DC_WRBK_EMIT_CMD   = 9'b1_010_01100,
                             DC_WRBK_EMIT_DATA  = 9'b1_010_01101,
                             //
                             DC_INV_LKUP_TAG_0  = 9'b1_100_10000,
                             DC_INV_LKUP_TAG_1  = 9'b1_100_10001,
                             DC_INV_CHK_TAG     = 9'b1_100_10010,
                             DC_INV_NXT         = 9'b1_100_10011
                            } fsm_dc_t;

  //
  localparam int DC_FSM_LKUP_B  = 5;
  localparam int DC_FSM_WRBK_B  = 6;
  localparam int DC_FSM_RMW_B   = 7;
  localparam int DC_FSM_BUSY_B  = 8;

  //
  typedef struct packed {
    // Command bus
    logic           valid;
    logic           wrbk;
    addr_t          addr;

    // Data bus
    logic           dat_valid;
    logic           sop;
    logic           eop;
    ram_dat_dat_t   dat;
  } mem_cmd_t;
  localparam int MEM_CMD_W  = $bits(mem_cmd_t);

  //
  typedef struct packed {
    logic           valid;
    logic           sop;
    logic           eop;
    ram_dat_dat_t   dat;
  } mem_resp_t;
  localparam int MEM_RESP_W  = $bits(mem_resp_t);

  //
  typedef struct packed {
    logic        valid;
    logic        dirty;
    cache_line_tag_t   tag;
  } tag_state_t;
  localparam int TAG_STATE_W = $bits(tag_state_t);

  typedef tag_state_t [CACHE_WAYS_N-1:0]     tag_state_n_t;

  //
  typedef struct packed {
    //
    tag_state_t          wrbk_tag;
    fsm_dc_t             wrbk_blink;

    ways_t               way_sel_d;
    ways_t               bnk_sel_d;
    cache_line_off_t     cnt;

    //
    addr_t               miss_addr;
  } dc_fsm_state_t;
  localparam int DC_FSM_STATE_W  = $bits(dc_fsm_state_t);

  //
  typedef union packed {
    ram_dat_dat_t r;
    data_t [RAM_DAT_WORDS_N-1:0] d;
  } dat_dat_t;

  //
  typedef struct packed {
    cache_line_off_t o;

    //
    ways_t w;

    //
    dat_dat_t d;

    //
    ram_dat_bnk_n_t b;

    // On WRITEBACK, tag state is updated to indicate the cache line is
    // dirty. As a power-saving measure, no attempt is made to rewrite tag
    // updates to lines already marked dirty as in this case such a write is
    // unnecessary.
    //
    logic        t_wen;

    //
    tag_state_t t;
  } wrbk_t;

  // ======================================================================== //
  //                                                                          //
  // Wires                                                                    //
  //                                                                          //
  // ======================================================================== //

  //
  ways_t                                     tag_cmd_en;
  ways_t                                     tag_cmd_wen;
  cache_line_off_t                           tag_cmd_addr;
  tag_state_t                                tag_cmd_wdata;
  tag_state_n_t                              tag_cmd_rdata;
  //
  ways_t            [CACHE_WAYS_N-1:0]       dat_cmd_en;
  ways_t            [CACHE_WAYS_N-1:0]       dat_cmd_wen;
  cache_line_off_t                           dat_cmd_addr;
  ram_dat_dat_t                              dat_cmd_wdata;
  ram_dat_dat_n_t   [CACHE_WAYS_N-1:0]       dat_cmd_rdata;
  //
  fsm_dc_t                                   dc_fsm_w;
  fsm_dc_t                                   dc_fsm_r;
  logic                                      dc_fsm_en;
  // S0
  //
  logic                                      s0_cmd_nop;
  // S1
  //
  logic                                      s1_cmd_nop;
  logic                                      s1_cmd_inv;
  logic                                      s1_cmd_store;
  logic                                      dcache__resp_replay_inv_w;
  logic                                      dcache__resp_replay_w;
  logic                                      dcache__resp_valid_w;
  logic                                      s1_miss;
  logic                                      s1_read_valid_w;
  logic                                      s1_read_valid_r;
  ways_t                                     s1_evict_way_sel_w;
  ways_t                                     s1_evict_way_sel_r;
  ways_t                                     s1_wrbk_bnk_sel_w;
  ways_t                                     s1_wrbk_bnk_sel_r;
  ways_t                                     s1_dat_sel;
  ways_t                                     s1_tag_sel;
  tag_state_t  [CACHE_WAYS_N-1:0]            s1_tag_bypass;
  ram_dat_dat_t [CACHE_WAYS_N-1:0]           s1_dat_bypass;
  logic [1:0]                                s1_dat_bnk_sel;
  // S2
  //
  logic                                      s2_cmd_nop;
  dat_dat_t                                  s2_dat_r;
  dat_dat_t                                  s2_dat_w;
  logic                                      s2_dat_en;
  ways_t                                     s2_hit_w;
  ways_t                                     s2_hit_r;
  ways_t                                     s2_tag_valid_w;
  ways_t                                     s2_tag_valid_r;
  tag_state_t                                s2_tag_w;
  tag_state_t                                s2_tag_r;
  logic                                      s2_tag_en;
  ways_t                                     s2_wrbk_bnk_sel_r;
  //
  dat_dat_t                                  s2_dat_bypass;
  logic                                      s2_dat_bypass_wrbk;
  // S3
  //
  logic                                      s3_replay_inv;
  //
  mem_cmd_t                                  dcache__mem_r;
  mem_cmd_t                                  dcache__mem_w;
  //
  mem_resp_t                                 mem__dcache_r;
  mem_resp_t                                 mem__dcache_w;
  //
  dc_fsm_state_t                             dc_fsm_state_r;
  dc_fsm_state_t                             dc_fsm_state_w;
  logic                                      dc_fsm_state_en;
  logic                                      dc_fsm_state_fsm_en;
  //
  cache_line_off_t                           dc_fsm_state_cnt_inc;
  ways_t                                     dc_fsm_state_bnk_sel_s1;
  ways_t                                     dc_fsm_state_way_sel_s1;
  //
  ways_t                                     s2_evict_way_r;
  ways_t                                     s2_evict_way_w;
  //
  //
  wrbk_t                                     wrbk_r;
  wrbk_t                                     wrbk_w;
  logic                                      wrbk_en;
  //
  logic                                      wrbk_valid_r;
  logic                                      wrbk_valid_w;
  //
  logic                                      dcache__busy_w;
  //
  logic                                      tag_sel_inv;
  logic                                      tag_sel_inv_cmd;
  logic                                      tag_sel_init;
  logic                                      tag_sel_wrbk;
  //

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : status_PROC

      //
      dcache__busy_w = (~rst) & (dc_fsm_w [DC_FSM_BUSY_B] | wrbk_valid_w);

    end // block: status_PROC


`define FFS_SUFFIX WaysN
`define FFS_W CACHE_WAYS_N
`include "ffs.vh"
`undef FFS_W
`undef FFS_SUFFIX

`define ENCODE_SUFFIX WaysN
`define ENCODE_W CACHE_WAYS_N
`include "encode.vh"
`undef ENCODE_W
`undef ENCODE_SUFFIX

`define ROTATE_SUFFIX WaysN
`define ROTATE_W CACHE_WAYS_N
`include "rotate.vh"
`undef ROTATE_W
`undef ROTATE_SUFFIX

  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : dc_fsm_PROC

      //
      dc_fsm_w                  = dc_fsm_r;

      //
      dcache__mem_w             = '0;
      dcache__mem_w.dat         = s2_dat_r;

      //
      s1_wrbk_bnk_sel_w         = '0;

      //
      dc_fsm_state_w            = dc_fsm_state_r;
      dc_fsm_state_fsm_en       = '0;

      //
      dc_fsm_state_bnk_sel_s1   = (dc_fsm_state_r.bnk_sel_d << 1);

      //
      dc_fsm_state_way_sel_s1   = RotateWaysN(dc_fsm_state_r.way_sel_d, 1);

      //
      dc_fsm_state_cnt_inc      = dc_fsm_state_r.cnt + 'b1;

      //
      case (dc_fsm_r)

        // ------------------------------------------------------------------ //
        //
        // Idle State
        //
        // ------------------------------------------------------------------ //

        DC_IDLE: begin

          priority casez ({s3_replay_inv, dcache__resp_replay_r})

            2'b1_?: begin
              // Issued Op: invalidation command.

              //
              dc_fsm_state_fsm_en       = 1'b1;
              dc_fsm_state_w.cnt        = '0;
              dc_fsm_state_w.way_sel_d  = 'b1;

              //
              dc_fsm_w                  = DC_INV_LKUP_TAG_0;
            end // case: 2'b0_1

            2'b0_1: begin
              // Replay invoke op: writeback or lookup operation.

              //
              dc_fsm_state_fsm_en           = 1'b1;
              dc_fsm_state_w.bnk_sel_d      = 'b1;
              dc_fsm_state_w.way_sel_d      = s2_evict_way_r;
              //
              dc_fsm_state_w.miss_addr      = '0;
              dc_fsm_state_w.miss_addr.l.t  = s2_cmd_r.addr.l.t;
              dc_fsm_state_w.miss_addr.l.o  = s2_cmd_r.addr.l.o;
              //
              dc_fsm_state_w.wrbk_tag       = s2_tag_r;
              dc_fsm_state_w.wrbk_blink     = DC_LKUP_EMIT_CMD;

              //
              dc_fsm_w                      =   s2_tag_r.dirty
                                              ? DC_WRBK_EMIT_CMD
                                              : DC_LKUP_EMIT_CMD
                                            ;
            end // case: 2'b1_?

            default: begin
            end
          endcase // casez ({dcach__resp_replay_r})
        end


        // ------------------------------------------------------------------ //
        //
        // Write-Back
        //
        // ------------------------------------------------------------------ //

        // Writeback (WRBK) procedure to flush a dirty line in the cache to
        // backing-store.
        //
        DC_WRBK_EMIT_CMD: begin
          addr_t a;

          //
          dcache__mem_w.valid       = 1'b1;
          dcache__mem_w.wrbk        = 1'b1;

          a                         = '0;
          a.l.t                     = dc_fsm_state_r.wrbk_tag.tag;
          a.l.o                     = dc_fsm_state_r.miss_addr.l.o;
          dcache__mem_w.addr        = a;

          // Update state
          dc_fsm_state_fsm_en       = 1'b1;
          dc_fsm_state_w.bnk_sel_d  = dc_fsm_state_bnk_sel_s1;

          //
          s1_wrbk_bnk_sel_w         = dc_fsm_state_r.bnk_sel_d;

          dc_fsm_w                  = DC_WRBK_EMIT_DATA;
        end

        // WRBK data state read from DAT RAM and emit corresponding bus
        // operation.
        //
        DC_WRBK_EMIT_DATA: begin

          // Update state
          dc_fsm_state_fsm_en       = 1'b1;
          dc_fsm_state_w.bnk_sel_d  = dc_fsm_state_bnk_sel_s1;

          //
          s1_wrbk_bnk_sel_w         = dc_fsm_state_r.bnk_sel_d;

          // Emit Command
          dcache__mem_w.dat_valid   = (|s2_wrbk_bnk_sel_r);
          dcache__mem_w.sop         = s2_wrbk_bnk_sel_r [0];
          dcache__mem_w.eop         = s2_wrbk_bnk_sel_r [RAM_DAT_LINE_N - 1];

          //
          if (dcache__mem_w.dat_valid & dcache__mem_w.eop)
            dc_fsm_w = dc_fsm_state_r.wrbk_blink;

        end

        // ------------------------------------------------------------------ //
        //
        // Invalidation (Programmatic)
        //
        // ------------------------------------------------------------------ //

        // Initial TAG lookup
        //
        DC_INV_LKUP_TAG_0:
          dc_fsm_w = DC_INV_LKUP_TAG_1;

        // TAG lookup waitstate until tag state latched in s2_tag_r
        //
        DC_INV_LKUP_TAG_1:
          dc_fsm_w = DC_INV_CHK_TAG;

        // TAG comparison function; check TAG to identify whether current
        // LINE/WAY is dirty and, if so, branch to WRBK procedure.
        //
        DC_INV_CHK_TAG: begin
          dc_fsm_w  = DC_INV_NXT;

          if (s2_tag_r.dirty) begin
            // Current Line/Way is dirty and must be written-back.

            dc_fsm_state_fsm_en           = 'b1;
            dc_fsm_state_w.bnk_sel_d      = 'b1;
            dc_fsm_state_w.wrbk_tag       = s2_tag_r;
            dc_fsm_state_w.miss_addr.l.o  = dc_fsm_state_r.cnt;

            // FSM calls the wrbk procedure link a subroutine. Upon completion
            // of the WRBK, the FSM jumped to the LINK address, which is in this
            // case the final state of the invalidation routine.
            //
            dc_fsm_state_w.wrbk_blink     = DC_INV_NXT;

            //
            dc_fsm_w                      = DC_WRBK_EMIT_CMD;
          end

        end // case: DC_INV_CHK_TAG

        // Completion of TAG invalidation sequence. Increment WAY pointer; point
        // to next line if WAYS in current line have been exhausted.
        //
        DC_INV_NXT:begin

          //
          dc_fsm_state_fsm_en       = 'b1;
          dc_fsm_state_w.way_sel_d  = dc_fsm_state_way_sel_s1;

          //
          if (dc_fsm_state_w.way_sel_d[0])
            // Line exhausted, move to new line.
            dc_fsm_state_w.cnt  = dc_fsm_state_cnt_inc;

          //
          dc_fsm_w  = (dc_fsm_state_r.cnt == '1) ? DC_IDLE : DC_INV_LKUP_TAG_0;

        end // case: DC_INV_NXT

        // ------------------------------------------------------------------ //
        //
        // Lookup State(s)
        //
        // ------------------------------------------------------------------ //

        DC_LKUP_EMIT_CMD: begin

          //
          dc_fsm_state_fsm_en       = 1'b1;
          dc_fsm_state_w.bnk_sel_d  = 'b1;

          //
          dcache__mem_w.valid       = 1'b1;
          dcache__mem_w.addr        = dc_fsm_state_r.miss_addr;

          //
          dc_fsm_w                  = DC_LKUP_WAIT_RESP;
        end


        DC_LKUP_WAIT_RESP: begin

          //
          dc_fsm_state_fsm_en       = mem__dcache_r.valid;
          dc_fsm_state_w.bnk_sel_d  = dc_fsm_state_bnk_sel_s1;

          //
          if (mem__dcache_r.valid && mem__dcache_r.eop)
            dc_fsm_w  = DC_IDLE;
        end


        // ------------------------------------------------------------------ //
        //
        // Reset Invalidation State(s)
        //
        // ------------------------------------------------------------------ //

        // Initial power-on reset sequence to initialize TAG RAM state to known
        // value (in this case, an known invalidate state).
        //
        DC_RESET_INIT: begin
          //
          dc_fsm_state_fsm_en  = 'b1;
          dc_fsm_state_w.cnt   = '0;

          //
          dc_fsm_w             = DC_RESET_INV;
        end

        DC_RESET_INV: begin
          dc_fsm_state_fsm_en  = 'b1;
          dc_fsm_state_w.cnt   = dc_fsm_state_cnt_inc;

          //
          if (dc_fsm_state_r.cnt == '1)
            dc_fsm_w = DC_IDLE;
        end

        default: begin
          dc_fsm_w  = 'x;
        end

      endcase // case (dc_fsm_r)

      //
      dc_fsm_en  =     rst
                     | dc_fsm_r [DC_FSM_BUSY_B]
                     | dcache__resp_replay_r
                     | s3_replay_inv
                   ;

      //
      dc_fsm_state_en  = (rst | dc_fsm_state_fsm_en);

    end // block: dc_fsm_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : tag_PROC

      //
      tag_sel_inv      = (dc_fsm_r == DC_RESET_INV);

      //
      tag_sel_inv_cmd  =    (dc_fsm_r == DC_INV_LKUP_TAG_0)
                         || (dc_fsm_r == DC_INV_CHK_TAG)
                       ;

      //
      tag_sel_init     = (dc_fsm_r == DC_LKUP_WAIT_RESP);

      //
      tag_sel_wrbk     = wrbk_valid_r;

      //
      priority case (1'b1)
`define V(b) {CACHE_WAYS_N{(b)}}

        tag_sel_inv: begin
          tag_cmd_en     = '1;
          tag_cmd_wen    = '1;
          tag_cmd_addr   =  dc_fsm_state_r.cnt;
          tag_cmd_wdata  = '0;
        end

        tag_sel_inv_cmd: begin
          tag_cmd_en     = dc_fsm_state_r.way_sel_d;
          if (dc_fsm_r == DC_INV_CHK_TAG)
             tag_cmd_wen    = '1;
          tag_cmd_addr   = dc_fsm_state_r.cnt;
          tag_cmd_wdata  = '0;
        end

        tag_sel_init: begin
          tag_state_t t  = '0;
          //
          t.valid        = 1'b1;
          t.tag          = dc_fsm_state_r.miss_addr.l.t;

          //
          tag_cmd_en     =    `V(dc_fsm_w == DC_IDLE)
                            & dc_fsm_state_r.way_sel_d;
          tag_cmd_wen    = '1;
          tag_cmd_addr   = dc_fsm_state_r.miss_addr.l.o;
          tag_cmd_wdata  = t;
        end

        tag_sel_wrbk: begin
          // Pending writeback
          tag_cmd_en     = wrbk_r.w;
          tag_cmd_wen    = '1;
          tag_cmd_addr   = wrbk_r.o;
          tag_cmd_wdata  = wrbk_r.t;
        end // case: tag_sel_wrbk

        (~dcache__busy_r): begin
          // Nominal case
          tag_cmd_en     = `V(s0_valid_r & ~s0_cmd_nop);
          tag_cmd_wen    = '0;
          tag_cmd_addr   = s0_cmd_r.addr.l.o;
          tag_cmd_wdata  = '0;
        end

        default: begin
          tag_cmd_en     = '0;
          tag_cmd_wen    = '0;
          tag_cmd_addr   = '0;
          tag_cmd_wdata  = '0;
        end
`undef V
      endcase // priority case (1'b1)

    end // block: tag_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : dat_PROC

      logic [1:0] w = EncodeWaysN(dc_fsm_state_r.way_sel_d);

      //
      dat_cmd_en       = '0;
      dat_cmd_wen      = '0;
      dat_cmd_addr     = '0;
      dat_cmd_wdata    = wrbk_r.d;

      //
      priority case (1'b1)

        dc_fsm_r [DC_FSM_LKUP_B]: begin

          //
`define V(b) {CACHE_WAYS_N{(b)}}
          dat_cmd_en [w]    =   `V(mem__dcache_r.valid)
                              & dc_fsm_state_r.bnk_sel_d
                            ;
          dat_cmd_wen [w]   = '1;
          dat_cmd_addr      = s0_cmd_r.addr.l.o;
          dat_cmd_wdata     = mem__dcache_r.dat;
`undef V
        end // case: dc_fsm_r [DC_FSM_LKUP_B]

        dc_fsm_r [DC_FSM_WRBK_B]: begin

          dat_cmd_en [w]    = dc_fsm_state_r.bnk_sel_d;
          dat_cmd_addr      = dc_fsm_state_r.miss_addr.l.o;

        end // case: dc_fsm_r [DC_FSM_WRBK_B]

        wrbk_valid_r: begin

          logic [1:0] w = EncodeWaysN(wrbk_r.w);
          dat_cmd_en [w]    = wrbk_r.b;
          dat_cmd_wen [w]   = '1;
          dat_cmd_addr      = wrbk_r.o;

        end // case: wrbk_valid_r

        default: begin
`define V(b) {RAM_DAT_LINE_N{(b)}}
          //
          for (int w = 0; w < CACHE_WAYS_N; w++) begin

            dat_cmd_en [w]     =    `V(s0_valid_r & ~s0_cmd_nop)
                                  & (1 << s0_cmd_r.addr.p.b)
                               ;
            dat_cmd_wen [w]    = '0;
            dat_cmd_addr       =  s0_cmd_r.addr.l.o;
          end
`undef V
        end // case: default

      endcase // priority case (1'b1)

    end // block: dat_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : s0_PROC

      //
      s0_cmd_nop       = s0_valid_r & (s0_cmd_r.op == NOP);

      //
      s1_read_valid_w  = (~rst) & (|dat_cmd_en) &(~|dat_cmd_wen);

    end // block: s0_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : s1_hit_PROC

      //
      for (int w = 0; w < CACHE_WAYS_N; w++) begin

        //
        s2_tag_valid_w [w]   = tag_cmd_rdata [w].valid;

        //
        s2_hit_w [w]         =    tag_cmd_rdata [w].valid
                               & (tag_cmd_rdata [w].tag == s1_cmd_r.addr.l.t)
                             ;
      end

      //
      s1_miss    = (s2_hit_w == '0);

    end // block: s1_hit_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : s1_evict_PROC

      // Maintain a one-hot vector that is rotated on each cycle. This
      // implements a pseudo-random replacement strategy on EVICTION of a fully
      // utilized, dirty set.
      //
      s1_evict_way_sel_w  =   rst
                            ? 'b1
                            : {   s1_evict_way_sel_r [CACHE_WAYS_N-2:0]
                                , s1_evict_way_sel_r [CACHE_WAYS_N-1]
                              }
                            ;

      //
      s2_evict_way_w    =   (s2_tag_valid_w != '1)
                          ? FFSWaysN(~s2_tag_valid_w)
                          : s1_evict_way_sel_r
                        ;

    end // block: s1_evict_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : s1_mux_PROC

      //
      s2_dat_en  = s1_read_valid_r;

      //
      s1_dat_sel = dcache__busy_r ? dc_fsm_state_r.way_sel_d : s2_hit_w;

`define V(x) {RAM_DAT_W{(x)}}

      // DATA RAM selection/hit MUX
      //
      s2_dat_w   = '0;
      for (int w = 0; w < CACHE_WAYS_N; w++)
        s2_dat_w |= (`V(s1_dat_sel [w]) & s1_dat_bypass [w]);
`undef V

      //
      casez ({dcache__busy_r, s1_cmd_store})
        2'b0_0:  s1_tag_sel  = s2_hit_w;
        2'b0_1:  s1_tag_sel  = s2_evict_way_w;
        default: s1_tag_sel  = dc_fsm_state_r.way_sel_d;
      endcase // casez ({dcache__busy_r, s1_cmd_store})

`define V(x) {TAG_STATE_W{(x)}}

      // TAG RAM selection/hit MUX
      //
      s2_tag_w  = '0;
      for (int w = 0; w < CACHE_WAYS_N; w++)
        s2_tag_w |= (`V(s1_tag_sel [w]) & s1_tag_bypass [w]);
`undef V

    end // block: s1_mux_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : s1_cntrl_PROC

      //
      s1_cmd_nop             = s1_valid_r & (s1_cmd_r.op == NOP);
      s1_cmd_inv             = s1_valid_r & (s1_cmd_r.op == INV);
      s1_cmd_store           = s1_valid_r & (s1_cmd_r.op == STORE);

      // REPLAY pipeline from S2 when cache miss detected in S1. Qualify REPLAY
      // on s2_valid_w instead of s1_valid_r because otherwise if a REPLAY is
      // asserted at S2, the s1_valid_r will not be killed until the following
      // cycle, resulting in the assertion of the replay for two
      // cycles. Although functionality remains unchanged, the double assertion
      // is an unnecesary and unwelcome performance penalty.
      //
      dcache__resp_replay_w  = s2_valid_w
                               & (~s1_cmd_nop)
                               & (~s1_cmd_inv)
                               & ( s2_hit_w == '0)
                               & (~rst)
                             ;

      //
      dcache__resp_replay_inv_w  = (~rst) & s1_valid_r & s1_cmd_inv;

      //
      dcache__resp_valid_w       = (~rst) & (|s2_hit_w);

    end // block: s1_cntrl_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : s2_PROC

      //
      s2_cmd_nop                      = s2_valid_r & (s2_cmd_r.op == NOP);

      //
      wrbk_w                          = '0;
      wrbk_w.w                        = s2_hit_r;
      wrbk_w.b                        = (1 << s2_cmd_r.addr.p.b);
      wrbk_w.t                        = '0;
      wrbk_w.t.valid                  = 'b1;
      wrbk_w.t.dirty                  = 'b1;
      wrbk_w.t.tag                    = s2_cmd_r.addr.l.t;
      wrbk_w.o                        = s2_cmd_r.addr.l.o;
      wrbk_w.t_wen                    = (~s2_tag_r.dirty);

      // Simplified read/modify write.
      //
      // In this simple example, accesses to the CACHE are always WORD (4B)
      // aligned and WORD length. In a production implementation, the write-back
      // may work with non-WORD length accesses, and potentially misaligned
      // accesses that straddle multiple banks (although probably not
      // lines). The code that would fullfill this function, would be located
      // below.
      //
      wrbk_w.d                        = s2_dat_r;
      wrbk_w.d.d [s2_cmd_r.addr.p.b]  = s2_cmd_r.data;

      // This is perhaps overly pessimistic as this stall condition need only
      // take effect when there is a collision to the same slice of the line. At
      // present, the stall is raised on every and any write back from S2.
      //
      wrbk_valid_w                    =    (~rst)
                                         &   s2_valid_r
                                         & ( s2_cmd_r.op == STORE)
                                         & (~dcache__resp_replay_r)
                                      ;
      //
      wrbk_en                         = wrbk_valid_w;

    end // block: s2_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : s3_PROC

      // Programmatically invoked cache invalidation is implemented as a
      // post-commit operation. In this case, the pipeline is replayed post
      // commit of the invoking invalidation instruction and subsequent issue
      // held-up/stalled until completion of the invalidation sequence.
      //
      s3_replay_inv  = s3_replay_r & s3_replay_inv_r;

    end // block: s3_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : resp_PROC

      //
      dcache__resp_w          = '0;

      //
      for (int b = 0; b < RAM_DAT_B; b++) begin
        int shift_bits     = (b * 8);
        ram_dat_dat_t shift_word = s2_dat_bypass >> shift_bits;

        if (s2_cmd_r.addr.p.o == ram_dat_off_t'(b))
          dcache__resp_w.data |= data_t'(shift_word);
      end

    end // block: resp_PROC


  // ------------------------------------------------------------------------ //
  // Tag writeback forwarding network. Tag state may be updated by a downstream
  // command pending writeback. The following code derives the most recent
  // tag architectural state for each WAY in the cache.
  //
  always_comb
    begin: bypass_tag_PROC

      // STORE/INV instructions are the only permissible commands that can alter
      // TAG state. Consequently, forwarding is predicated upon a hit on TAG
      // line, stage validity and whether the command at that stage is a
      // store. By definition, writebacks only occur to TAG memory on STORES
      // therefore the presence of a valid writeback is sufficient
      // qualification.
      //
      logic bypass_s2       =    s2_valid_r
                              & (s2_cmd_r.addr.l.o == s1_cmd_r.addr.l.o)
                              & (s2_cmd_r.op == STORE)
                            ;

      logic bypass_wrbk     =    wrbk_valid_r
                              & (wrbk_r.o == s1_cmd_r.addr.l.o)
                            ;

      for (int w = 0; w < CACHE_WAYS_N; w++) begin
        //
        logic way_bypass_s2    = bypass_s2 & wrbk_w.w[w];
        logic way_bypass_wrbk  = bypass_wrbk & wrbk_r.w[w];

        //
        casez ({way_bypass_s2, way_bypass_wrbk})
          2'b1_?:  s1_tag_bypass [w] = wrbk_w.t;
          2'b0_1:  s1_tag_bypass [w] = wrbk_r.t;
          default: s1_tag_bypass [w] = tag_cmd_rdata [w];
        endcase

      end // for (int w = 0; w < CACHE_WAYS_N; w++)

      // Latch match state on MISS or a STORE op. On a STORE, tag state is
      // required for any potential writeback operation. On a MISS, the TAG
      // state is required for any potential eviction operation.
      //
      s2_tag_en  =   (s1_valid_r & (s1_miss | (s1_cmd_r.op == STORE)))
                   | (dc_fsm_r == DC_INV_LKUP_TAG_1)
                 ;

    end // block: bypass_tag_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : bypass_dat_PROC

      // On eviction, the DCACHE controller FSM overrides any bank selection
      // state that may reside in the pipeline. When the FSM is inactive, bank
      // selection is made from the value of the address in the pipeline ucode.
      //
      s1_dat_bnk_sel      =   dcache__busy_r
                            ? EncodeWaysN(s1_wrbk_bnk_sel_r)
                            : s1_cmd_r.addr.p.b
                          ;

      //
      s2_dat_bypass_wrbk  =    wrbk_valid_r
                            & (wrbk_r.o == s2_cmd_r.addr.l.o)
                            & (wrbk_r.t.tag == s2_cmd_r.addr.l.t)
                          ;

      //
      casez ({s2_dat_bypass_wrbk})
        1'b1:    s2_dat_bypass  = wrbk_r.d;
        default: s2_dat_bypass  = s2_dat_r;
      endcase // casez ({s2_dat_bypass_wrbk})


      // DAT RAM bypass code. Data becomes available from the DAT RAM in stage
      // S1. This forwarding with S2 and WRBK is performed here to ensure that
      // downstream modifications to the same word are seen. In the absence of
      // such forwarding logic, it becomes necessary to insert stall cycles from
      // the period when DATA has been read and when it has been committed
      // (written back to DAT RAM).
      //
      for (int w= 0; w < CACHE_WAYS_N; w++) begin

        //
        logic s1_dat_bypass_s2    =    s2_valid_r
                                    & (s2_cmd_r.addr.l.o == s1_cmd_r.addr.l.o)
                                    & (s2_cmd_r.addr.l.t == s1_cmd_r.addr.l.t)
                                    & (s2_cmd_r.op == STORE)
                                    &  wrbk_w.w [w]
                                  ;

        //
        logic s1_dat_bypass_wrbk  =    wrbk_valid_r
                                    & (wrbk_r.o == s1_cmd_r.addr.l.o)
                                    & (wrbk_r.t.tag == s1_cmd_r.addr.l.t)
                                    &  wrbk_r.w [w]
                                  ;

        //
        casez ({s1_dat_bypass_s2, s1_dat_bypass_wrbk})
          2'b1_?:  s1_dat_bypass [w]  = wrbk_w.d;
          2'b0_1:  s1_dat_bypass [w]  = wrbk_r.d;
          default: s1_dat_bypass [w]  = dat_cmd_rdata [w][s1_dat_bnk_sel];
        endcase

      end

    end // block: bypass_dat_PROC


  // ======================================================================== //
  //                                                                          //
  // Flops                                                                    //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    begin : dc_fsm_reg_PROC
      if (dc_fsm_en)
        dc_fsm_r <= rst ? DC_RESET_INIT : dc_fsm_w;
    end // block: dc_fsm_reg_PROC

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (dc_fsm_state_en)
      dc_fsm_state_r <= dc_fsm_state_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (s2_dat_en)
      s2_dat_r <= s2_dat_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (wrbk_en)
      wrbk_r <= wrbk_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (s2_tag_en)
      s2_tag_r <= s2_tag_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk) begin : reg_PROC
    //
    s1_read_valid_r           <= s1_read_valid_w;
    s1_evict_way_sel_r        <= s1_evict_way_sel_w;
    s1_wrbk_bnk_sel_r         <= s1_wrbk_bnk_sel_w;
    //
    s2_tag_valid_r            <= s2_tag_valid_w;
    s2_hit_r                  <= s2_hit_w;
    s2_evict_way_r            <= s2_evict_way_w;
    s2_wrbk_bnk_sel_r         <= s1_wrbk_bnk_sel_r;
    //
    wrbk_valid_r              <= wrbk_valid_w;
    //
    dcache__resp_replay_r     <= dcache__resp_replay_w;
    dcache__resp_replay_inv_r <= dcache__resp_replay_inv_w;
    dcache__mem_r             <= dcache__mem_w;
    dcache__busy_r            <= dcache__busy_w;
    dcache__resp_valid_r      <= dcache__resp_valid_w;
    //
    mem__dcache_r             <= mem__dcache_w;
  end // block: reg_PROC


  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  generate for (genvar w = 0; w < CACHE_WAYS_N; w++) begin : tag_GEN

    //
    spram #(   .MEM_N    (RAM_TAG_N)
             , .DATA_W   (TAG_STATE_W)
             , .ADDR_W   (RAM_TAG_ADDR_W)
           ) u_ram_tag
    (
      //
        .clk               (clk                )
      , .rst               (rst                )
      //
      , .cmd_en            (tag_cmd_en [w]     )
      , .cmd_wen           (tag_cmd_wen [w]    )
      , .cmd_addr          (tag_cmd_addr       )
      , .cmd_wdata         (tag_cmd_wdata      )
      //
      , .resp_rdata        (tag_cmd_rdata [w]  )
      , .resp_serr         ()
      , .resp_derr         ()
    );

  end endgenerate // block: tag_GEN

  // ------------------------------------------------------------------------ //
  //
  generate for (genvar w = 0; w < CACHE_WAYS_N; w++) begin : ram_dat_GEN

      //
      ram_dat_way u_ram_dat
      (
          //
          .clk               (clk                )
        , .rst               (rst                )
        //
        , .cmd_en            (dat_cmd_en [w]     )
        , .cmd_wen           (dat_cmd_wen [w]    )
        , .cmd_addr          (dat_cmd_addr       )
        , .cmd_wdata         (dat_cmd_wdata      )
        //
        , .resp_rdata        (dat_cmd_rdata [w]  )
     );

  end endgenerate // block: ram_dat_GEN


  // ======================================================================== //
  //                                                                          //
  // Misc.                                                                    //
  //                                                                          //
  // ======================================================================== //

  //
  always_comb dcache__mem_valid_r  = dcache__mem_r.valid;
  always_comb dcache__mem_wrbk_r = dcache__mem_r.wrbk;
  always_comb dcache__mem_addr_r  = dcache__mem_r.addr;
  always_comb dcache__mem_dat_valid_r = dcache__mem_r.dat_valid;
  always_comb dcache__mem_sop_r = dcache__mem_r.sop;
  always_comb dcache__mem_eop_r = dcache__mem_r.eop;
  always_comb dcache__mem_dat_r  = dcache__mem_r.dat;

  //
  always_comb
    begin : mem__dcache_w_PROC
      mem__dcache_w.valid  = mem__dcache_valid_w;
      mem__dcache_w.sop    = mem__dcache_sop_w;
      mem__dcache_w.eop    = mem__dcache_eop_w;
      mem__dcache_w.dat    = mem__dcache_data_w;
    end // block: mem__dcache_w_PROC

endmodule

// Local Variables:
// verilog-typedef-regexp:"_t$"
// End:
