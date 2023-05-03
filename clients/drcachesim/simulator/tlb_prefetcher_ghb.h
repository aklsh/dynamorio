/* **********************************************************
 * Copyright (c) 2017-2020 Google, Inc.  All rights reserved.
 * **********************************************************/

/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Google, Inc. nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL VMWARE, INC. OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

/* prefetcher: represents a hardware prefetching implementation.
 */

#ifndef _TLB_PREFETCHER_GHB_H_
#define _TLB_PREFETCHER_GHB_H_ 1

#include "tlb.h"
#include "trace_entry.h"
#include "memref.h"
#include <unordered_map>

#define DELTA_HISTORY_LENGTH 4
#define PAGE_SIZE 4096
#define PAGE_SIZE_BITS 12

class tlb_t;

// boost library hashing algorithm
struct delta_hash {
    std::size_t operator()(const std::array<int_least64_t, DELTA_HISTORY_LENGTH>& a) const {
        std::size_t h = 0;
        for (auto e : a) {
            h ^= std::hash<int>{}(e)  + 0x9e3779b9 + (h << 6) + (h >> 2);
        }
        return h;
    }
};

class ghb_entry_t {
public:
    addr_t pc;
    addr_t prev_page;
    std::unordered_map<std::array<int_least64_t, DELTA_HISTORY_LENGTH> , int_least64_t, delta_hash> delta_lookup_;
    std::array<int_least64_t, DELTA_HISTORY_LENGTH> deltas_;
    uint8_t cpu_;
    uint8_t lru_counter_;

    ghb_entry_t(uint8_t cpu);
    int_least64_t
    update_state(addr_t curr_page, bool same_cpu);
};

class tlb_prefetcher_ghb_t {
private:
    std::vector<std::vector<ghb_entry_t*>> ghb_;
    uint8_t max_entries_cpu_;

public:
    tlb_prefetcher_ghb_t(uint8_t num_cpus);
    void
    prefetch(tlb_t *tlb, const memref_t &memref, uint8_t cpu, int_least64_t access_num);
    void
    print_results(std::string prefix, uint8_t cpu_id);
};

#endif /* _TLB_PREFETCHER_GHB_H_ */