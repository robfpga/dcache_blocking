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

#pragma once

#include "dcache_blocking_tb_common.h"

enum CacheOp : uint32_t { NOP = 0, LOAD = 1, STORE = 2, INV = 3 };

struct CacheCommand {
  CacheOp c{NOP};
  uint32_t a{};
  uint32_t d{};
};

struct MemBehavioral : public sc_module {
  //
  sc_in<bool> clk;
  //
  sc_in<bool> dcache__mem_valid_r;
  sc_in<bool> dcache__mem_wrbk_r;
  sc_in<uint32_t> dcache__mem_addr_r;
  sc_in<bool> dcache__mem_dat_valid_r;
  sc_in<bool> dcache__mem_sop_r;
  sc_in<bool> dcache__mem_eop_r;
  sc_in<vluint64_t> dcache__mem_dat_r;

  //
  sc_out<bool> mem__dcache_valid_w;
  sc_out<bool> mem__dcache_sop_w;
  sc_out<bool> mem__dcache_eop_w;
  sc_out<vluint64_t> mem__dcache_data_w;

  SC_HAS_PROCESS(MemBehavioral);
  MemBehavioral(sc_module_name mn = "mem");

  void read(uint32_t a, uint32_t& d);

  void write(uint32_t a, uint32_t d);

  void clear_count();
  int count_op_wrbk() const;
  int count_op_lkup() const;
  int count_op_total() const;

 private:
  //
  struct Cmd {
    bool wrbk_{false};
    uint32_t addr_;
    std::array<vluint64_t, 4> dat_;
    friend ostream& operator<<(ostream& os, const Cmd& c) { return os; }
  };

  //
  struct Resp {
    bool sop_{false};
    bool eop_{false};
    std::array<vluint64_t, 4> dat_;
  };

  void thread_cmd();
  void thread_resp();

  void wait_posedge(unsigned n = 1);
  void sync_sample();
  void inc_wrbk();
  void inc_lkup();

  //
  unsigned n_wrbk_{}, n_lkup_{};
  sc_fifo<Cmd> fifo_;
  MemoryModel model_;
};

struct CacheBehavioral {
  CacheBehavioral(MemBehavioral& mem);
  DataT read(AddrT a);
  void write(AddrT a, DataT d);

 private:
  bool is_cached(AddrT a);
  MemBehavioral& mem_;
  std::unordered_map<AddrT, DataT> cache_;
};
