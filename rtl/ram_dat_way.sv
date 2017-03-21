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

module ram_dat_way
(
   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   //
     input                                             clk

   //======================================================================== //
   //                                                                         //
   // RAM Interface                                                           //
   //                                                                         //
   //======================================================================== //

   //
   , input     dcache_blocking_pkg::ways_t             cmd_en
   , input     dcache_blocking_pkg::ways_t             cmd_wen
   , input     dcache_blocking_pkg::cache_line_off_t   cmd_addr
   , input     dcache_blocking_pkg::ram_dat_dat_t      cmd_wdata
   //
   , output    dcache_blocking_pkg::ram_dat_dat_n_t    resp_rdata
);

  import dcache_blocking_pkg::*;

  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //


  // ------------------------------------------------------------------------ //
  //
  generate for (genvar g = 0; g < RAM_DAT_LINE_N; g++)

    // TODO
    spsram #(   .N       (256     )
//    spsram #(   .N       (RAM_DAT_N     )
              , .W       (RAM_DAT_W     )
            ) u_ram_dat
    (
      //
        .clk               (clk                )
      //
      , .en                (cmd_en [g]         )
      , .wen               (cmd_wen [g]        )
      , .addr              (cmd_addr           )
      , .din               (cmd_wdata          )
      //
      , .dout              (resp_rdata [g]     )
    );

  endgenerate

endmodule
