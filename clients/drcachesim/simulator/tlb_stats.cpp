/* **********************************************************
 * Copyright (c) 2015-2020 Google, Inc.  All rights reserved.
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

#include <iostream>
#include <iomanip>
#include "tlb_stats.h"

tlb_stats_t::tlb_stats_t(int block_size, int id, std::string type)
    : caching_device_stats_t("", block_size)
    , id_(id)
    , type_(type)
    , num_prefetches_requested_(0)
    , num_prefetches_requested_at_reset_(0)
    , num_prefetches_used_(0)
    , num_prefetches_used_at_reset_(0)

{
    stats_map_.emplace(metric_name_t::PREFETCH_REQUESTS, num_prefetches_requested_);
    stats_map_.emplace(metric_name_t::PREFETCH_USED, num_prefetches_used_);
    stats_map_.emplace(metric_name_t::PREFETCH_REQUESTS_AT_RESET, num_prefetches_requested_at_reset_);
    stats_map_.emplace(metric_name_t::PREFETCH_USED_AT_RESET, num_prefetches_used_at_reset_);
}

void
tlb_stats_t::access(const memref_t &memref, bool hit,
                    caching_device_block_t *cache_block)
{
    // We assume we're single-threaded.
    // We're only computing miss rate so we just inc counters here.
    if (hit) {
        num_hits_++;
        if (((tlb_entry_t *)cache_block)->prefetched_ == true){
            num_prefetches_used_++;
            // count each prefetched page only once, as after 1st hit it'll already be in TLB
            ((tlb_entry_t *)cache_block)->prefetched_ = false;
        }
    }
    else {
        if (memref.data.type != TRACE_TYPE_HARDWARE_PREFETCH){
            num_misses_++;
            if (dump_misses_)
                dump_miss(memref);

            check_compulsory_miss(memref.data.addr);
        }
    }

    if (memref.data.type == TRACE_TYPE_HARDWARE_PREFETCH)
        num_prefetches_requested_++;
}

void
tlb_stats_t::print_counts(std::string prefix)
{
    std::cerr << prefix << std::setw(18) << std::left << "Hits:" << std::setw(20)
              << std::right << num_hits_ << std::endl;
    std::cerr << prefix << std::setw(18) << std::left << "Prefetches:" << std::setw(20)
              << std::right << num_prefetches_requested_ << std::endl;
    std::cerr << prefix << std::setw(18) << std::left << "Used Prefetches:" << std::setw(20)
              << std::right << num_prefetches_used_ << std::endl;
    std::cerr << prefix << std::setw(18) << std::left << "Misses:" << std::setw(20)
              << std::right << num_misses_ << std::endl;
    std::cerr << prefix << std::setw(18) << std::left << "Compulsory misses:" << std::setw(20)
              << std::right << num_compulsory_misses_ << std::endl;
    if (is_coherent_) {
        std::cerr << prefix << std::setw(21) << std::left
                  << "Parent invalidations:" << std::setw(17) << std::right
                  << num_inclusive_invalidates_ << std::endl;
        std::cerr << prefix << std::setw(20) << std::left
                  << "Write invalidations:" << std::setw(18) << std::right
                  << num_coherence_invalidates_ << std::endl;
    } else {
        std::cerr << prefix << std::setw(18) << std::left
                  << "Invalidations:" << std::setw(20) << std::right
                  << num_inclusive_invalidates_ << std::endl;
    }
}

void
tlb_stats_t::reset()
{
    num_hits_at_reset_ = num_hits_;
    num_misses_at_reset_ = num_misses_;
    num_child_hits_at_reset_ = num_child_hits_;
    num_prefetches_requested_at_reset_ = num_prefetches_requested_;
    num_prefetches_used_at_reset_ = num_prefetches_used_;
    num_hits_ = 0;
    num_prefetches_requested_ = 0;
    num_prefetches_used_ = 0;
    num_misses_ = 0;
    num_compulsory_misses_ = 0;
    num_child_hits_ = 0;
    num_inclusive_invalidates_ = 0;
    num_coherence_invalidates_ = 0;
}