//========================================================================== //
// Copyright (c) 2016-17, Stephen Henry
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

#include "dcache_blocking_tb.h"

struct test_0 : public DCacheBlockingTb {
  test_0() {}

  bool run_test() {
      LIBTB_REPORT_INFO("Test 0 START: Invalidate");
      issue_op(CacheCommand{STORE, 0x10000000, 0x11111111});
      issue_op(CacheCommand{STORE, 0x20000000, 0x22222222});
      issue_op(CacheCommand{STORE, 0x30000000, 0x33333333});
      cache_invalidate();
      issue_op(CacheCommand{LOAD, 0x10000000});
      issue_op(CacheCommand{LOAD, 0x20000000});
      issue_op(CacheCommand{LOAD, 0x30000000});
      wait_until_complete();
      LIBTB_REPORT_INFO("Test 0 END: Invalidate");

      return true;
  }
};

int sc_main(int argc, char **argv) {
    using namespace libtb;
    test_0 t;
    LibTbContext::init(argc, argv);
    return LibTbContext::start();
}
