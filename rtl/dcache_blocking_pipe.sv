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

module dcache_blocking_pipe
(
   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   //
     input                                   clk
   , input                                   rst

   //======================================================================== //
   //                                                                         //
   // Fetch Interface                                                         //
   //                                                                         //
   //======================================================================== //

   //
   , input                                    fetch_valid
   , input  [dcache_blocking_pkg::OP_W-1:0]   fetch_op
   , input  [dcache_blocking_pkg::ADDR_W-1:0] fetch_addr
   , input  [dcache_blocking_pkg::DATA_W-1:0] fetch_data
   //
   , output logic                             fetch_accept

   //======================================================================== //
   //                                                                         //
   // Commit Interface                                                        //
   //                                                                         //
   //======================================================================== //

   //
   , input                                    commit_accept
   //
   , output logic                             commit_valid_r
   , output logic                             commit_load_r
   , output dcache_blocking_pkg::data_t       commit_data_r

   //======================================================================== //
   //                                                                         //
   // Pipe Interface                                                          //
   //                                                                         //
   //======================================================================== //

   //
   , output logic                             s0_valid_r
   , output dcache_blocking_pkg::cmd_t        s0_cmd_r
   //
   , output logic                             s1_valid_r
   , output dcache_blocking_pkg::cmd_t        s1_cmd_r
   //
   , output logic                             s2_valid_w
   , output logic                             s2_valid_r
   , output dcache_blocking_pkg::cmd_t        s2_cmd_r
   //
   , output logic                             s3_replay_r
   , output logic                             s3_replay_inv_r

   //======================================================================== //
   //                                                                         //
   // DCache Interface                                                        //
   //                                                                         //
   //======================================================================== //

   , input logic                             dcache__busy_r
   //
   , input logic                             dcache__resp_valid_r
   , input dcache_blocking_pkg::resp_t       dcache__resp_w
   //
   , input logic                             dcache__resp_replay_r
   , input logic                             dcache__resp_replay_inv_r
);

  import dcache_blocking_pkg::*;

  //
  typedef struct packed {
    logic           valid;
    cmd_t           cmd;
  } ucode_t;

  //
  typedef ucode_t [2:0] pipe_t;
  typedef logic [2:0] stages_t;

  // ======================================================================== //
  //                                                                          //
  // Wires                                                                    //
  //                                                                          //
  // ======================================================================== //

  //
  cmd_t             fetch_fifo_push_data;
  cmd_t             issue_cmd;
  //
  logic             fetch_fifo_full_r;
  logic             fetch_fifo_empty_r;
  //
  logic             fetch_fifo_push;
  logic             fetch_fifo_pop;
  //
  logic             fetch_fifo_commit;
  logic             fetch_fifo_replay;
  //
  pipe_t            pipe_r;
  pipe_t            pipe_w;
  stages_t          pipe_en;
  //
  logic             commit_valid_w;
  logic             commit_load_w;
  data_t            commit_data_w;
  // Common
  //
  logic             sx_kill_common;
  // S0
  //
  logic             s0_kill;
  logic             s0_stall;
  // S1
  //
  logic             s1_kill;
  // S2
  //
  logic             s2_kill;
  // S3
  //
  logic             s3_replay_w;
  logic             s3_replay_r;
  logic             s3_replay_inv_w;
  logic             s3_replay_inv_r;

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : pipe_PROC

      //
      fetch_fifo_push_data       = '0;
      fetch_fifo_push_data.op    = fetch_op;
      fetch_fifo_push_data.addr  = fetch_addr;
      fetch_fifo_push_data.data  = fetch_data;

      //
      fetch_accept               = (~fetch_fifo_full_r);

      //
      fetch_fifo_push            = fetch_valid & fetch_accept;

    end // block: pipe_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : fetch_PROC

      //
      fetch_fifo_pop     = (~fetch_fifo_empty_r) & (~s0_stall);

      //
      fetch_fifo_commit  = (pipe_r [2].valid & (~s2_kill));

      //
      fetch_fifo_replay  =   s3_replay_r
                           | (pipe_r [2].valid & dcache__resp_replay_r)
                         ;

    end // block: fetch_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : commit_PROC

      //
      commit_valid_w     = (~rst) & fetch_fifo_commit;

      //
      commit_load_w      = (pipe_r [2].cmd.op == LOAD);

      //
      commit_data_w      = dcache__resp_w;

    end // block: dcache_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : stall_PROC

      //
      s0_stall          = pipe_r [0].valid & dcache__busy_r;

    end // block: stall_PROC


  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : ucode_w_PROC

      //
      sx_kill_common    = (rst | fetch_fifo_replay | s3_replay_r);
      s0_kill           = sx_kill_common;
      s1_kill           = sx_kill_common;
      s2_kill           = sx_kill_common;

      //
      pipe_w [0]        = '0;
      pipe_w [0].valid  = (~s0_kill) & fetch_fifo_pop;
      pipe_w [0].cmd    = issue_cmd;

      //
      pipe_w [1]        = pipe_r [0];
      pipe_w [1].valid  = (~s1_kill) & pipe_r [0].valid & (~s0_stall);

      //
      pipe_w [2]        = pipe_r [1];
      pipe_w [2].valid  = (~s2_kill) & pipe_r [1].valid;

      //
      s3_replay_w       = (~s2_kill) & dcache__resp_replay_inv_r;
      s3_replay_inv_w   = (~s2_kill) & dcache__resp_replay_inv_r;

      //
      pipe_en [0]       = s0_kill | (~s0_stall);
      pipe_en [1]       = 'b1;
      pipe_en [2]       = 'b1;

    end // block: ucode_PROC


  // ======================================================================== //
  //                                                                          //
  // Flops                                                                    //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    for (int i = 0; i < 3; i++)
      if (pipe_en [i])
        pipe_r [i]  <= pipe_w [i];

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    commit_valid_r <= commit_valid_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    commit_load_r <= commit_load_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    commit_data_r <= commit_data_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    s3_replay_r <= s3_replay_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    s3_replay_inv_r <= s3_replay_inv_w;

  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  fifo #(.W(CMD_W), .N(), .HAS_REPLAY(1)) u_fetch_fifo
  (
    //
      .clk               (clk                )
    , .rst               (rst                )
    //
    , .push              (fetch_fifo_push    )
    , .push_data         (fetch_fifo_push_data)
    //
    , .pop               (fetch_fifo_pop     )
    , .pop_data          (issue_cmd          )
    //
    , .flush             (1'b0               )
    , .commit            (fetch_fifo_commit  )
    , .replay            (fetch_fifo_replay  )
    //
    , .empty_r           (fetch_fifo_empty_r )
    , .full_r            (fetch_fifo_full_r  )
  );

  // ======================================================================== //
  //                                                                          //
  // Misc.                                                                    //
  //                                                                          //
  // ======================================================================== //

  //
  always_comb s0_valid_r = pipe_r [0].valid;
  always_comb s0_cmd_r = pipe_r [0].cmd;
  always_comb s1_valid_r = pipe_r [1].valid;
  always_comb s1_cmd_r = pipe_r [1].cmd;
  always_comb s2_valid_w = pipe_w [2].valid;
  always_comb s2_valid_r = pipe_r [2].valid;
  always_comb s2_cmd_r = pipe_r [2].cmd;

endmodule

// Local Variables:
// verilog-typedef-regexp:"_t$"
// End:
