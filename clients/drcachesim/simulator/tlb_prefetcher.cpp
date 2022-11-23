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
#include <algorithm>
#include <utility>
#include <iostream>

tlb_prefetcher_t::tlb_prefetcher_t(int page_size_bits)
    : page_size_bits_(page_size_bits)
{
    pc_refs_.clear();
}

tlb_prefetcher_t::~tlb_prefetcher_t()
{
    for (auto &it : pc_refs_)
        delete it.second;
}

void
tlb_prefetcher_t::prefetch(tlb_t *tlb, const memref_t &memref_in)
{
    // We implement a simple next-line prefetcher.
    memref_t memref = memref_in;
    memref.data.addr += 64;
    memref.data.type = TRACE_TYPE_HARDWARE_PREFETCH;
    tlb->request(memref);
}

void
tlb_prefetcher_t::pc_update(const memref_t &memref_in)
{
    addr_t ip = memref_in.data.pc;
    addr_t page = memref_in.data.addr >> page_size_bits_;

    bool found = false;
    for (auto it : pc_refs_){
        std::vector<addr_t>* v = it.second;
        if(it.first == ip){
            found = true;
            if (std::find(v->begin(), v->end(), page) == v->end()){
                v->push_back(page);
            }
            break;
        }
    }
    if (found == false){
        std::vector<addr_t>* v = new std::vector<addr_t>;
        v->push_back(page);
        pc_refs_.insert(std::pair<addr_t, std::vector<addr_t>*>(ip, v));
    }
}

void
tlb_prefetcher_t::print_results(std::string prefix)
{
    for (auto &it : pc_refs_){
        std::cerr << prefix << "PC: " << std::hex << "0x" << it.first << std::dec << "(" << it.second->size() << ")" << std::endl;
        std::vector<addr_t>* v = it.second;
        for (auto it_v : *v){
            std::cerr << prefix << prefix << "0x" << std::hex << it_v <<std::dec << std::endl;
        }
    }
}
