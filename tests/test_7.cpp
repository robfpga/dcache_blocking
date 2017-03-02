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

struct test_7 : public DCacheBlockingTb {
  test_7() {}

  bool run_test() {
    LIBTB_REPORT_INFO("Test7 START: Performance");
    cache_invalidate();

    const uint32_t line_shift = libtb::log2ceil(CACHE_LINE_B);
    const uint32_t lines_total = (CACHE_SIZE_B / CACHE_LINE_B);

    int i;
    AddrT A;

    // Warm up cache.
    mem_.clear_count();
    for (i = 0; i < lines_total; i++) {
      A = i << line_shift;

      CacheCommand c;
      c.c = LOAD;
      c.a = A;
      issue_op(c);
    }
    wait_until_complete();
    if (mem_.count_op_wrbk() != 0) LIBTB_REPORT_ERROR("Unexpected writeback");
    if (mem_.count_op_lkup() != lines_total) {
      std::stringstream ss;

      ss << "Unexpected lookup operation count. "
         << " Expected: " << lines_total << " Actual: " << mem_.count_op_lkup();
      LIBTB_REPORT_ERROR(ss.str());
    }

    // All consequent cache operations should hit in cache.
    mem_.clear_count();
    i = 1000;
    while (i--) {
      CacheCommand c;
      c.c = LOAD;
      c.a = random_address(A);
      issue_op(c);
    }
    wait_until_complete();
    if (mem_.count_op_total() != 0) {
      std::stringstream ss;
      ss << "Expected zero mem ops to warm cache. Count="
         << mem_.count_op_total();
      LIBTB_REPORT_ERROR(ss.str());
    }
    wait_until_complete();
    cache_invalidate();
    LIBTB_REPORT_INFO("Test7 END: Performance");

    return false;
  }
};

int sc_main(int argc, char **argv) {
  using namespace libtb;
  test_7 t;
  LibTbContext::init(argc, argv);
  return LibTbContext::start();
}
