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

#include "dcache_blocking_tb_model.h"
//
#include <sstream>

MemBehavioral::MemBehavioral(sc_module_name mn)
    : libtb::LibTbModule(mn), model_(0x100000, PageInit::RANDOMIZED) {
  clear_count();

  SC_THREAD(thread_cmd);
  SC_THREAD(thread_resp);
}

void MemBehavioral::read(uint32_t a, uint32_t& d) {
  model_.read_n(a, &d, sizeof(uint32_t));
}

void MemBehavioral::write(uint32_t a, uint32_t d) {
  model_.write_n(a, &d, sizeof(uint32_t));
}

void MemBehavioral::clear_count() {
  n_wrbk_ = 0;
  n_lkup_ = 0;
}
int MemBehavioral::count_op_wrbk() const { return n_wrbk_; }
int MemBehavioral::count_op_lkup() const { return n_lkup_; }
int MemBehavioral::count_op_total() const {
  return count_op_wrbk() + count_op_lkup();
}

void MemBehavioral::thread_cmd() {
  while (true) {
    t_wait_sync();
    if (!dcache__mem_valid_r) {
      wait_posedge();
      continue;
    }

    Cmd c;
    c.wrbk_ = dcache__mem_wrbk_r;
    c.addr_ = dcache__mem_addr_r;

    if (!c.wrbk_) {
      inc_lkup();

      for (int i = 0; i < c.dat_.size(); i++) {
        const uint32_t addr = c.addr_ + (i * RAM_DAT_B);
        model_.read_n(addr, &c.dat_[i], RAM_DAT_B);

        std::stringstream ss;
        ss << "READ A=" << std::hex << addr << " "
           << "D=" << c.dat_[i];
        LIBTB_REPORT_DEBUG(ss.str());
        wait_posedge();
      }
    }

    if (c.wrbk_) {
      inc_wrbk();

      for (int i = 0; true;) {
        if (dcache__mem_dat_valid_r) {
          const uint32_t addr = c.addr_ + (i * RAM_DAT_B);
          const vluint64_t data = dcache__mem_dat_r;

          std::stringstream ss;
          ss << "Write A=" << std::hex << addr << " "
             << "D=" << data;
          LIBTB_REPORT_DEBUG(ss.str());
          model_.write_n(addr, &data, RAM_DAT_B);
          if (dcache__mem_eop_r) break;
          i++;
        }
        wait_posedge();
        t_wait_sync();
      }
    }

    // Schedule response/completion notification.
    fifo_.write(c);
  }
}

void MemBehavioral::thread_resp() {
  bool first{true};

  while (true) {
    const Cmd c = fifo_.read();

    if (first) wait_posedge(2);

    // Writeback completion notifications not implemented.
    if (c.wrbk_) continue;

    mem__dcache_valid_w = true;
    for (int i = 0; i < c.dat_.size(); i++) {
      mem__dcache_sop_w = (i == 0);
      mem__dcache_eop_w = (i == c.dat_.size() - 1);
      mem__dcache_data_w = c.dat_[i];
      wait_posedge();
    }
    mem__dcache_valid_w = false;
    first = false;
  }
}

//
void MemBehavioral::wait_posedge(unsigned n) {
  while (n--) wait(clk.posedge_event());
}

void MemBehavioral::inc_wrbk() {
  LIBTB_REPORT_DEBUG("Received WRBK request");
  n_wrbk_++;
}

void MemBehavioral::inc_lkup() {
  LIBTB_REPORT_DEBUG("Received LKUP request");
  n_lkup_++;
}

CacheBehavioral::CacheBehavioral(MemBehavioral& mem) : mem_(mem) {}

DataT CacheBehavioral::read(AddrT a) {
  if (!is_cached(a)) mem_.read(a, cache_[a]);

  return cache_[a];
}

void CacheBehavioral::write(AddrT a, DataT d) { cache_[a] = d; }

bool CacheBehavioral::is_cached(AddrT a) {
  return (cache_.find(a) != cache_.end());
}
