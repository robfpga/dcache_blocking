# Blocking Data Cache

[![Build Status](https://travis-ci.org/stephenry/dcache_blocking.svg?branch=master)](https://travis-ci.org/stephenry/dcache_blocking)

## Introduction

Caching is a broad micro-architectural technique used to lower latency to main
memory and thereby increase the effective bandwidth available to an
agent. Caches are effective at capturing the inherent locality between access
patterns and are fundamental to high performance micro-architectures. Cache
design is a huge topic and therefore shall not be discussed in depth.

Presented is a realistic, although non production, implementation of a 4-way
set-associative data cache typical to that which would be found in a CPU
implementing a standard 5-staged MIPS pipeline.

The cache implements standard progammatic Invalidation, Lookup and Writeback
functions. For simplicitly, all access to the cache are aligned and alignment or
sign-extension logic is not implemented. Line locking, uncached accesses and bus
faults are not implemented; although these may be readily integrated on top of
the existing micro-architecture.

Memory macros are single cycle although the micro-architecture can be readily
extended (through forwarding) to support multi-cycle RAMS (although this is
atypical of such a design).

A small representative CPU pipeline is presented alongside the datapath of the
cache. This is used to implemented stall and replay functionality, and to retain
pipeline micro-code.

## System Requirements
* cmake >= 3.2
* systemc >= 2.3.1
* verilator >= 3.9
* clang >= 3.9

## Build Steps
~~~~
git clone https://github.com/stephenry/dcache_blocking.git
cd dcache_blocking
git submodule update --init --recursive
mkdir build
cd build
cmake ../
make
~~~~

## Parameterization

## Memory Requirements

All macros are single ported, single cycle. By consequence of this design
decision, collisions to the same memory bank may take place on write-back. This
is detected and resolved by asserting a single cycle stall at the issue stage.

The micro-architecture may be extended by implementing additional forwarding
logic or by introducing dual-ported SRAM (at the cost of some added area).

## Micro-Architecture

### Invalidation

On reset, TAG state is initialized such that each location is explicitly
invalid. Consequently, the DCACHE is blocked and commands may not proceed until
the sequence has completed. This sequence occurs once on reset and
programmatically on request. Therefore, it is not relevent from an overall
performance standpoint.

A secondary aspect is programmatic invalidation. In typical eviction cases, the
eviction process is carried out pre-commit in response to a missing LOAD/STORE
instruction. In programmatic invalidation, the invoking instruction must commit
before the invalidation process takes place. This is called a post-commit
action.

The invalidation process is complicate by the requirement to evict lines that
are dirty. Correctness is not maintained if the cache is simply cleared as in
during reset. The micro-architecture is optimized to query the DIRTY status of
each line and writeback only those lines that are dirty.

### Core Pipeline

The code datapath consists of three stages, broadly corresponding to the XA
(execution), CA (commit) and WA (writeback) stages of the core pipeline.

#### Execution Stage

Addresses to the DC are computed by the Address Generation Unit (AGU) found in
the execution stage. Load/Store/Invalidate instructions are stalled in the XA
stage until pending writebacks have retired.

Based upon the computed address, the relevant TAG line and DAT lines are
queried. Queries become available in the later stage.

#### Commit Stage

Tag matching occurs in CA and upon a hit the relevant DAT line MUXed in the
response. Alternatively, a miss is detected. Due to timing constraints, we are
limited by what we can accomplish using the Queried data (data from such macros
typically arrive late in the cycle and therefore cannot be forwarded).

State is forwarded to WA.

#### Writeback Stage

At the writeback stage, LOADED state is either forwarded upstream to dependent
instruction in the datapath; combined with data to be stored to be written
back. Alternatively, on a miss event, a pipeline replay is executed and a line
lookup takes place. The pipeline remains blocked at XA until line replacement
has completed.

### Line lookup

Line lookup takes place when attempting to load-from or write-to a line not
present in the cache. By virtue of the set associative micro-architecture, a
line has up to N locations where it may reside. On installation, a line may be
written to a free way. Alternatively, it may overwrite a clean line already
installed in the cache. In the absence of clean lines, the replacement policy
randomly selects a line to evict and writeback to main memory. The line
replacement policy will preferentially replace clean lines in the cache as these
operations are quicker to complete.

## Error handling

## Performance

## Verification Methodology

Line lookup and placement are issued through a standard bus interface to a
SystemC behavioral model of memory. This memory is initialized on startup
against a randomized address space.

Commands are randomly generated and are issued to the data cache. Based upon the
current state of memory the expected response is known. The verification
environment checks expected memory state against actual and terminates
simulation on a mismatch.

### Tests

* __test_0 Basic Invalidation__ A synthetic sequence is constructed to write to
  regions of memory that alias to the same line in the data cache. A
  programmatic invalidation takes place and the memory regions read back.
* __test_2 Lookup Requests__ A synthetic sequence is constructed to fetch 4
  lines each aliasing to the same line. The sequence is repeated a large number
  of times. The total number of memory accesses is recorded and checked to equal
  the number of ways.
* __test_3 Elementary Loads__ A randomized sequence of LOADS is generated. The
  verification environment validates that the read result equals that of the
  predicted memory model.
* __test_4 Directed Stores__ A synthetic sequence back-to-back load/stores
  aliasing to the same line is constructed.
* __test_5 Randomized Stores__ A randomized sequence of stores is executed. A
  randomized sequence of loads is executed. The results loaded from the dcache
  is checked to verify that the correct data is read back.
* __test_6 Invalidation__ Basic test to verify behavior of programmatic
  invalidation.
* __test_7 Performance__ The Cache is completely filled by a region of
  memory. The region memory is specifically constructed to consume all lines and
  all ways of the cache. After an initial warm-up period, a randomized series of
  load/store instructions is emitted to the same region. The expected number of
  memory/bus operations is expected to be zero (as this data should already be
  present in the cache).
