##========================================================================== //
## Copyright (c) 2016, Stephen Henry
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## * Redistributions of source code must retain the above copyright notice, this
##   list of conditions and the following disclaimer.
##
## * Redistributions in binary form must reproduce the above copyright notice,
##   this list of conditions and the following disclaimer in the documentation
##   and/or other materials provided with the distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
##========================================================================== //

CMAKE_MINIMUM_REQUIRED(VERSION 3.4)
PROJECT(dcache_blocking)

SET(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_SOURCE_DIR}/contrib/libtb/cmake"
  )
SET(CMAKE_CXX_STANDARD 14)

SET(TB_TOP "dcache_blocking_tb")


SET(VERILATED_OUTPUT "${CMAKE_BINARY_DIR}/verilated")
SET(CONTRIB_BASE "${CMAKE_SOURCE_DIR}/contrib/")
SET(LIBTB_BASE "${CONTRIB_BASE}libtb/")
SET(LIBV_BASE "${CONTRIB_BASE}libv/")
SET(LIB_BASE "${CMAKE_SOURCE_DIR}/lib/")
SET(RTL_BASE "${CMAKE_SOURCE_DIR}/rtl/")

FIND_PACKAGE(SystemC REQUIRED)
FIND_PACKAGE(Verilator REQUIRED)

ADD_SUBDIRECTORY(contrib)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)
