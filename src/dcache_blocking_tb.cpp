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
//
#include <iomanip>
#include <sstream>

DCacheBlockingTb::DCacheBlockingTb(sc_core::sc_module_name mn)
    : libtb::TopLevel(mn), uut_("uut") {
  bind_rtl();
  SC_METHOD(m_commit);
  sensitive << e_reset_done();
  dont_initialize();
}

void DCacheBlockingTb::bind_rtl() {
  //
  uut_.clk(libtb::Ctxt()->clk());
  uut_.rst(libtb::Ctxt()->rst());
  //
  uut_.fetch_valid(fetch_valid_);
  uut_.fetch_op(fetch_op_);
  uut_.fetch_addr(fetch_addr_);
  uut_.fetch_data(fetch_data_);
  uut_.fetch_accept(fetch_accept_);
  //
  uut_.commit_valid_r(commit_valid_r_);
  uut_.commit_load_r(commit_load_r_);
  uut_.commit_data_r(commit_data_r_);
  uut_.commit_accept(commit_accept_);
//
#define VERILATOR_MANGLING_BUG
#ifdef VERILATOR_MANGLING_BUG
  uut_.dcache___05Fmem_valid_r(dcache__mem_valid_r_);
  uut_.dcache___05Fmem_wrbk_r(dcache__mem_wrbk_r_);
  uut_.dcache___05Fmem_addr_r(dcache__mem_addr_r_);
  uut_.dcache___05Fmem_dat_valid_r(dcache__mem_dat_valid_r_);
  uut_.dcache___05Fmem_sop_r(dcache__mem_sop_r_);
  uut_.dcache___05Fmem_eop_r(dcache__mem_eop_r_);
  uut_.dcache___05Fmem_dat_r(dcache__mem_dat_r_);
  //
  uut_.dcache___05Fbusy_r(dcache__busy_r_);
  //
  uut_.mem___05Fdcache_valid_w(mem__dcache_valid_w_);
  uut_.mem___05Fdcache_sop_w(mem__dcache_sop_w_);
  uut_.mem___05Fdcache_eop_w(mem__dcache_eop_w_);
  uut_.mem___05Fdcache_data_w(mem__dcache_data_w_);
#else
  //
  uut_.dcache__mem_valid_r(dcache__mem_valid_r_);
  uut_.dcache__mem_wrbk_r(dcache__mem_wrbk_r_);
  uut_.dcache__mem_addr_r(dcache__mem_addr_r_);
  uut_.dcache__mem_dat_valid_r(dcache__mem_dat_valid_r_);
  uut_.dcache__mem_sop_r(dcache__mem_sop_r_);
  uut_.dcache__mem_eop_r(dcache__mem_eop_r_);
  uut_.dcache__mem_dat_r(dcache__mem_dat_r_);
  //
  uut_.dcache__busy_r(dcache__busy_r_);
  //
  uut_.mem__dcache_valid_w(mem__dcache_valid_w_);
  uut_.mem__dcache_sop_w(mem__dcache_sop_w_);
  uut_.mem__dcache_eop_w(mem__dcache_eop_w_);
  uut_.mem__dcache_data_w(mem__dcache_data_w_);
#endif
  //
  mem_.clk(libtb::Ctxt()->clk());
  mem_.dcache__mem_valid_r(dcache__mem_valid_r_);
  mem_.dcache__mem_wrbk_r(dcache__mem_wrbk_r_);
  mem_.dcache__mem_addr_r(dcache__mem_addr_r_);
  mem_.dcache__mem_dat_valid_r(dcache__mem_dat_valid_r_);
  mem_.dcache__mem_sop_r(dcache__mem_sop_r_);
  mem_.dcache__mem_eop_r(dcache__mem_eop_r_);
  mem_.dcache__mem_dat_r(dcache__mem_dat_r_);
  //
  mem_.mem__dcache_valid_w(mem__dcache_valid_w_);
  mem_.mem__dcache_sop_w(mem__dcache_sop_w_);
  mem_.mem__dcache_eop_w(mem__dcache_eop_w_);
  mem_.mem__dcache_data_w(mem__dcache_data_w_);
}

void DCacheBlockingTb::wait_until_complete() {
  while (predicted_commits_.size()) wait(1, SC_US);
}

void DCacheBlockingTb::wait_until_busy() {
  t_wait_sync();
  while (!dcache__busy_r_) t_wait_sync();
  t_wait_posedge_clk();
}

void DCacheBlockingTb::wait_until_not_busy() {
  t_wait_sync();
  while (dcache__busy_r_) t_wait_sync();
  t_wait_posedge_clk();
}

void DCacheBlockingTb::cache_invalidate() {
  LIBTB_REPORT_DEBUG("Invalidating Cache");
  CacheCommand c;
  c.c = INV;
  issue_op(c);
  wait_until_busy();
  wait_until_not_busy();
}

void DCacheBlockingTb::issue_op(CacheCommand const& c) {
  update_prediction_queue(c);

  // Emit command
  fetch_valid_ = true;
  fetch_op_ = c.c;
  fetch_addr_ = c.a;
  fetch_data_ = c.d;
  t_wait_sync();
  while (!fetch_accept_) t_wait_sync();
  t_wait_posedge_clk();
  fetch_idle();
}

void DCacheBlockingTb::update_prediction_queue(CacheCommand const& c) {
  Prediction p;
  switch (c.c) {
    case LOAD: {
      p.is_load = true;
      p.a = c.a;
      p.d = cache_.read(p.a);
    } break;

    case STORE: {
      p.is_store = true;
      p.a = c.a;
      p.d = c.d;
      cache_.write(p.a, p.d);
    } break;

    case NOP:
    case INV:
      break;

    default:
      LIBTB_REPORT_FATAL("Unrecognized Op");
  }
  predicted_commits_.push_back(p);
}

void DCacheBlockingTb::wait_init() {
  t_wait_reset_done();
  wait_until_not_busy();
}

void DCacheBlockingTb::fetch_idle() {
  fetch_valid_ = false;
  fetch_op_ = 0;
  fetch_addr_ = 0;
  fetch_data_ = 0;
}

void DCacheBlockingTb::m_commit() {
  if (commit_valid_r_) {
    if (predicted_commits_.size() == 0) {
      LIBTB_REPORT_INFO("Unexpected commit");
      return;
    }

    Prediction& p = predicted_commits_.front();
    if (p.is_load) {
      // Predicting a LOAD retirement. Verify that a LOAD op has been
      // committed.
      if (!commit_load_r_) LIBTB_REPORT_ERROR("Expected load operation");

      // Validate actual versus expected.
      const uint32_t actual = commit_data_r_;
      if (actual != p.d) {
        std::stringstream ss;

        ss << "Mismatch Actual: " << std::hex << std::setw(8) << actual
           << " Expected: " << std::hex << std::setw(8) << p.d;
        LIBTB_REPORT_ERROR(ss.str());
      }
    }
    if (p.is_store) {
      if (commit_load_r_) LIBTB_REPORT_ERROR("Expected store operation");
    }
    predicted_commits_.pop_front();
  }
  next_trigger(e_tb_sample());
}
