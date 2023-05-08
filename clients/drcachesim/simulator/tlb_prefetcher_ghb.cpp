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

#include "tlb.h"
#include "../common/memref.h"
#include "../common/utils.h"

#include <algorithm>
#include <utility>
#include <iostream>

ghb_entry_t::ghb_entry_t(uint8_t cpu)
            : cpu_(cpu)
            , pc(0)
            , prev_page(0)
            , prev_time(0)
{
    delta_lookup_.clear();
    deltas_.fill(0);
    num_access_lookup_.clear();
    num_accesses_.fill(0);
}

std::tuple<int_least64_t, int_least64_t>
ghb_entry_t::update_state(addr_t curr_page, bool same_cpu, int_least64_t curr_time)
{
    // std::cerr << "In ghb_entry_t::update_state" << std::endl;
    int_least64_t next_delta, curr_delta, next_interval, curr_interval;
    bool found_delta = true;
    bool found_interval = true;


    // Get next predicted delta
    if(delta_lookup_.find(deltas_) == delta_lookup_.end()){
        found_delta = false;
        next_delta = 0;
    } else {
        next_delta = delta_lookup_[deltas_];
    }

    // Get next predicted interval
    if(num_access_lookup_.find(num_accesses_) == num_access_lookup_.end()){
        found_interval = false;
        next_interval = 0;
    } else {
        next_interval = num_access_lookup_[num_accesses_];
    }

    // update state only if same cpu
    if (same_cpu) {
        // Calculate current delta to update state 
        curr_delta = curr_page-prev_page;
        curr_interval = curr_time-prev_time;
        prev_page = curr_page;

        // Update delta lookup for current shift register state.
        if (found_delta==false){
            delta_lookup_.insert(std::pair<std::array<int_least64_t, DELTA_HISTORY_LENGTH>, int_least64_t>(deltas_, curr_delta));
        }
        
        // Update access time lookup for current shift register state.
        if (found_interval==false){
            num_access_lookup_.insert(std::pair<std::array<int_least64_t, DELTA_HISTORY_LENGTH>, int_least64_t>(num_accesses_, curr_interval));
        }

        // Delta shift register
        for(int i=1;i<DELTA_HISTORY_LENGTH;i++)
            deltas_[i-1] = deltas_[i];
        deltas_[DELTA_HISTORY_LENGTH-1] = curr_delta;

        // Interval shift register
        for(int i=1;i<DELTA_HISTORY_LENGTH;i++)
            num_accesses_[i-1] = num_accesses_[i];
        num_accesses_[DELTA_HISTORY_LENGTH-1] = curr_interval;
    }
    
    // std::cerr << "In ghb_entry_t::update_state leaving" << std::endl;
    return {next_delta, next_interval};
}

tlb_prefetcher_ghb_t::tlb_prefetcher_ghb_t(uint8_t num_cpus)
{
    ghb_.reserve(num_cpus);
    ghb_.resize(num_cpus);
    std::cerr << "Created TLB GHB Prefetcher for " << (int32_t)ghb_.size() << " CPUs" << std::endl;
}

void
tlb_prefetcher_ghb_t::prefetch(tlb_t *tlb, const memref_t &memref_in, uint8_t cpu, int_least64_t access_num)
{
    memref_t memref = memref_in;
    addr_t curr_pc = memref.data.pc;
    addr_t curr_page = memref.data.addr >> PAGE_SIZE_BITS;
    ghb_entry_t *curr_entry = nullptr;
    std::tuple<int_least64_t, int_least64_t> predictor_out;
    int_least64_t delta_pred, interval_pred;
    bool request_status = false;
    
    // check current cpu's buffer
    for (auto entry: ghb_[cpu]) {
        if (entry->pc == curr_pc) {
            curr_entry = entry;
            predictor_out = curr_entry->update_state(curr_page, true, access_num);
            delta_pred = std::get<0>(predictor_out);
            interval_pred = std::get<1>(predictor_out);
            if (delta_pred != 0){ // 0 => same page as now, no need to prefetch
                memref.data.addr += (delta_pred<<PAGE_SIZE_BITS);
                memref.data.type = TRACE_TYPE_HARDWARE_PREFETCH;
                // request_status = tlb->request_status(memref);
                tlb->prefetch_backlog_.push_back(std::pair<memref_t, int_least64_t>(memref, interval_pred));
                // tlb->get_parent()->request(memref);
            }
            break;
        }
    }
    // check other cpus' buffers if not found
    for (int i=1;i<ghb_.size();i++){
        if (i!=cpu){
            for (auto entry: ghb_[i]) {
                if (entry->pc == curr_pc) {
                    curr_entry = entry;
                    predictor_out = curr_entry->update_state(curr_page, true, access_num);
                    delta_pred = std::get<0>(predictor_out);
                    interval_pred = std::get<1>(predictor_out);
                    if (delta_pred != 0){ // 0 => same page as now, no need to prefetch
                        memref.data.addr += (delta_pred<<PAGE_SIZE_BITS);
                        memref.data.type = TRACE_TYPE_HARDWARE_PREFETCH;
                        // request_status = tlb->request_status(memref);
                        tlb->prefetch_backlog_.push_back(std::pair<memref_t, int_least64_t>(memref, interval_pred));
                        // tlb->get_parent()->request(memref);
                    }
                    break;
                }
            }
            // if (curr_entry != nullptr) // found in some buffer
            //     break;
        }
    }
    if (curr_entry == nullptr) {
        // create new ghb entry
        // std::cerr << "Found new PC " << curr_pc << " in CPU " << (int) cpu << std::endl;
        curr_entry = new ghb_entry_t(cpu);
        curr_entry->pc = curr_pc;
        predictor_out = curr_entry->update_state(curr_page, true, access_num);
        ghb_[cpu].push_back(curr_entry);
    }
}

void
tlb_prefetcher_ghb_t::print_results(std::string prefix, uint8_t cpu_id)
{
    std::cerr << prefix << "Entries in GHB: " << ghb_[cpu_id].size() << std::endl;

/*     for(auto entry: ghb_[cpu_id]) {
        std::cerr << prefix << "PC: " << std::hex << "0x" << entry->pc << std::dec << std::endl;
        std::cerr << prefix << "Page deltas: " << std::endl;
        for (auto it_deltas: entry->deltas_){
            std::cerr << prefix << prefix << "\t" << it_deltas << "\t";
        }
        std::cerr << std::endl;
        std::cerr << prefix << "Delta Lookup table: " << std:: endl;
        for(auto delta_map_entry: entry->delta_lookup_){
            for(auto delta_in_reg: delta_map_entry.first){
                std::cerr << prefix << prefix << (int_least64_t) delta_in_reg << " ";
            }
            std::cerr << prefix << "\t: " << delta_map_entry.second << std::endl;
        }
    } */
}