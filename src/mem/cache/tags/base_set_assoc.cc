/*
 * Copyright (c) 2012-2014 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definitions of a conventional tag store.
 */

#include "mem/cache/tags/base_set_assoc.hh"

#include <string>

#include "base/intmath.hh"

#include "create-subitmap.hh"

BaseSetAssoc::BaseSetAssoc(const Params *p)
    :BaseTags(p), allocAssoc(p->assoc), blks(p->size / p->block_size),
     firstLvlCache(p->first_lvl_cache), il1Cache(p->il1_cache),
     dl1Cache(p->dl1_cache), faultyCache(p->faulty_cache),
     subBlocks(p->sub_blocks),
     useFtaPredictor(p->fta_predictor), useMaepScheme(p->maep_scheme),
     sequentialAccess(p->sequential_access),
     replacementPolicy(p->replacement_policy)
{
    // Check parameters
    if (blkSize < 4 || !isPowerOf2(blkSize)) {
        fatal("Block size must be at least 4 and a power of 2");
    }

    if ((subBlocks < 1) || (subBlocks > 8)) {
        fatal("Subblocks out of valid range");
    }
}

void
BaseSetAssoc::tagsInit()
{
    //get the fault map
    if (faultyCache) {
        updatesublkbitmap(size/1024, allocAssoc);
    }

    // Initialize all blocks
    for (unsigned blk_index = 0; blk_index < numBlocks; blk_index++) {
        // Locate next cache block
        CacheBlk* blk = &blks[blk_index];

        // Link block to indexing policy
        indexingPolicy->setEntry(blk, blk_index);

        // Associate a data chunk to the block
        blk->data = &dataBlks[blkSize*blk_index];

        // Associate a replacement data entry to the block
        blk->replacementData = replacementPolicy->instantiateEntry();

        //In case of DL1 or IL1, update the fault map
        if ((faultyCache) &&
            (((name() == "system.cpu.icache.tags") && (il1Cache)) ||
            ((name() == "system.cpu.dcache.tags") && (dl1Cache)))) {
            for (unsigned sblk=0; sblk<subBlocks; sblk++) {
                blk->faulty_flag[sblk] =
                    sbitmap[blk_index/allocAssoc]
                           [(sblk + ((blk_index%allocAssoc)*subBlocks))];
            }
        }
        else {
            for (unsigned sblk=0; sblk<subBlocks; sblk++) {
                blk->faulty_flag[sblk] = 0;
            }
        }
    }
#if 0
    if (faultyCache) {
        printf("%s: %u %u %u %u\n",
            name().c_str(), firstLvlCache,
            dl1Cache, il1Cache, subBlocks);

        for (unsigned blk_index = 0; blk_index < numBlocks; blk_index++) {
           if (blk_index%allocAssoc == 0)
              printf("\n");
           CacheBlk* blk_t = &blks[blk_index];
           for (unsigned sblk=0; sblk<subBlocks; sblk++) {
               printf("%u ",blk_t->faulty_flag[sblk]);
           }
           printf("  ");
        }
    }
#endif
}

void
BaseSetAssoc::invalidate(CacheBlk *blk)
{
    // Invalidate replacement data
    if (blk->false_hit == 0) {
        replacementPolicy->invalidate(blk->replacementData);
    }

    BaseTags::invalidate(blk);
    // Decrease the number of tags in use
    stats.tagsInUse--;
}

BaseSetAssoc *
BaseSetAssocParams::create()
{
    // There must be a indexing policy
    fatal_if(!indexing_policy, "An indexing policy is required");

    return new BaseSetAssoc(this);
}
