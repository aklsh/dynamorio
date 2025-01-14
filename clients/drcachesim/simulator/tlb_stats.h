/* **********************************************************
 * Copyright (c) 2015-2017 Google, Inc.  All rights reserved.
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

/* tlb_stats: represents a TLB.
 */

#ifndef _TLB_STATS_H_
#define _TLB_STATS_H_ 1

#include <string>
#include "caching_device_stats.h"
#include "tlb_entry.h"

class tlb_stats_t : public caching_device_stats_t {
protected:
    int id_;
    std::string type_;
    int_least64_t num_prefetches_requested_;
    int_least64_t num_prefetches_requested_at_reset_;
    int_least64_t num_prefetches_used_;
    int_least64_t num_prefetches_used_at_reset_;

public:
    explicit tlb_stats_t(int block_size, int id = -1, std::string type = "");
    // XXX: support page privilege and MMU-related exceptions

    // It might be necessary to report stats of exceptions
    // triggered by address translation, e.g., address unaligned exception.
    void
    access(const memref_t &memref, bool hit, caching_device_block_t *cache_block) override;
    void
    print_counts(std::string prefix) override;
    void
    reset() override;
};

#endif /* _TLB_STATS_H_ */
