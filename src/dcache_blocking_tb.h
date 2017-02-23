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

#pragma once

#include <libtb.h>
#include <deque>
//
#include "Vdcache_blocking_tb.h"
#include "dcache_blocking_tb_model.h"

struct DCacheBlockingTb : public libtb::TopLevel {
  SC_HAS_PROCESS(DCacheBlockingTb);
  DCacheBlockingTb(sc_core::sc_module_name mn = "t");

 protected:
  //
  void m_commit();
  //
  void bind_rtl();
  //
  void wait_until_complete();
  void wait_until_busy();
  void wait_until_not_busy();
  void cache_invalidate();
  void issue_op(CacheCommand const& c);
  void update_prediction_queue(CacheCommand const& c);
  void wait_init();
  void fetch_idle();

  //
  sc_signal<bool> fetch_valid_;
  sc_signal<uint32_t> fetch_op_;
  sc_signal<uint32_t> fetch_addr_;
  sc_signal<uint32_t> fetch_data_;
  sc_signal<bool> fetch_accept_;
  //
  sc_signal<bool> commit_valid_r_;
  sc_signal<bool> commit_load_r_;
  sc_signal<uint32_t> commit_data_r_;
  sc_signal<bool> commit_accept_;
  //
  sc_signal<bool> dcache__busy_r_;
  //
  sc_signal<bool> dcache__mem_valid_r_;
  sc_signal<bool> dcache__mem_wrbk_r_;
  sc_signal<uint32_t> dcache__mem_addr_r_;
  sc_signal<bool> dcache__mem_dat_valid_r_;
  sc_signal<bool> dcache__mem_sop_r_;
  sc_signal<bool> dcache__mem_eop_r_;
  sc_signal<vluint64_t> dcache__mem_dat_r_;
  //
  sc_signal<bool> mem__dcache_valid_w_;
  sc_signal<bool> mem__dcache_sop_w_;
  sc_signal<bool> mem__dcache_eop_w_;
  sc_signal<vluint64_t> mem__dcache_data_w_;

  Vdcache_blocking_tb uut_;

  //
  std::deque<Prediction> predicted_commits_;
  MemBehavioral mem_;
  CacheBehavioral cache_{mem_};
};
