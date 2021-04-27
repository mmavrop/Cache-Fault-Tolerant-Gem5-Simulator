/*
 * Copyright (c) 2012-2014,2017 ARM Limited
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
 * Declaration of a base set associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
#define __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/packet.hh"
#include "params/BaseSetAssoc.hh"

/**
 * A basic cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The BaseSetAssoc placement policy divides the cache into s sets of w
 * cache lines (ways).
 */
class BaseSetAssoc : public BaseTags
{
  protected:
    /** The allocatable associativity of the cache (alloc mask). */
    unsigned allocAssoc;

    /** The cache blocks. */
    std::vector<CacheBlk> blks;

    /** Extra fields */
    bool firstLvlCache;
    bool il1Cache;
    bool dl1Cache;
    bool faultyCache;
    unsigned subBlocks;

    bool useFtaPredictor;
    bool useMaepScheme;
    /** Whether tags and data are accessed sequentially. */
    const bool sequentialAccess;

    /** Replacement policy */
    BaseReplacementPolicy *replacementPolicy;

    uint8_t ftaPredictorSblkRemaping(uint8_t norm_sblk_index,
                            uint8_t blk_remap) {
        uint8_t remaped_sblk;

        if (subBlocks == 2){
                if (blk_remap == 0) {
                remaped_sblk = norm_sblk_index;
                    }
            else if (blk_remap == 1) {
                remaped_sblk = !norm_sblk_index;
            }
            else {
                assert(0);
                }
            }
        else if (subBlocks == 4) {
                if (norm_sblk_index  == 0){
                        if ((blk_remap == 1) || (blk_remap == 3) ||
                                        (blk_remap == 6)) {
                     remaped_sblk = 1 ^ norm_sblk_index;
                }
                        else if ((blk_remap == 4) || (blk_remap == 5)) {
                     remaped_sblk = 2 ^ norm_sblk_index;
                }
                        else {
                     remaped_sblk = 0 ^ norm_sblk_index;
                }
                }
                else if (norm_sblk_index == 1) {
                        if ((blk_remap == 1) || (blk_remap == 4) ||
                      (blk_remap == 6) || (blk_remap == 7)) {
                    remaped_sblk = 2 ^ norm_sblk_index;
                }
                        else if ((blk_remap == 2) || (blk_remap == 3)) {
                    remaped_sblk = 3 ^ norm_sblk_index;
                }
                        else if (blk_remap == 5) {
                    remaped_sblk = 1 ^ norm_sblk_index;
                }
                        else {
                    remaped_sblk = 0 ^ norm_sblk_index;
                }
                }
                else if (norm_sblk_index == 2) {
                        if ((blk_remap == 2) || (blk_remap == 5) ||
                                        (blk_remap == 7)) {
                    remaped_sblk = 3 ^ norm_sblk_index;
                }
                        else if ((blk_remap == 3) || (blk_remap == 4) ||
                                        (blk_remap == 6)) {
                    remaped_sblk = 2 ^ norm_sblk_index;
                }
                        else {
                    remaped_sblk = 0 ^ norm_sblk_index;
                }
                }
                else if (norm_sblk_index == 3) {
                        if (blk_remap == 1) {
                    remaped_sblk = 3 ^ norm_sblk_index;
                }
                        else if (blk_remap == 4) {
                    remaped_sblk = 2 ^ norm_sblk_index;
                }
                        else if ((blk_remap == 6) || (blk_remap == 7)) {
                    remaped_sblk = 1 ^ norm_sblk_index;
                }
                        else {
                    remaped_sblk = 0 ^ norm_sblk_index;
                }
                }
            else {
                assert(0);
            }
        }
        else if (subBlocks == 8) {
          if (norm_sblk_index == 0){
            if ((blk_remap == 1) || (blk_remap == 3)) {
                remaped_sblk = 2;
            }
            else if (blk_remap == 4) {
                remaped_sblk = 4;
            }
            else if ((blk_remap == 6) || (blk_remap == 7)) {
                remaped_sblk = 1;
            }
            else remaped_sblk = 0;
          }
          else if (norm_sblk_index == 1) {
            if ((blk_remap == 1) || (blk_remap == 3) ||
                (blk_remap == 6)) {
                remaped_sblk = 3;
            }
            else if ((blk_remap == 5) || (blk_remap == 7)) {
                remaped_sblk = 2;
            }
            else if (blk_remap == 4) {
                remaped_sblk = 5;
            }
            else remaped_sblk = 1;
          }
          else if (norm_sblk_index == 2) {
            if ((blk_remap == 1) || (blk_remap == 4)) {
                remaped_sblk = 6;
            }
            else if ((blk_remap == 2) || (blk_remap == 3) ||
                     (blk_remap == 5)) {
                remaped_sblk = 4;
            }
            else if (blk_remap == 6) {
                remaped_sblk = 5;
            }
            else if (blk_remap == 7) {
                remaped_sblk = 3;
            }
            else remaped_sblk = 2;
          }
          else if (norm_sblk_index == 3) {
            if ((blk_remap == 1) || (blk_remap == 4) ||
                (blk_remap == 6)) {
                 remaped_sblk = 7;
            }
            else if ((blk_remap == 2) || (blk_remap == 3)) {
                remaped_sblk = 5;
            }
            else if (blk_remap == 7) {
                remaped_sblk = 4;
            }
            else remaped_sblk = 3;
          }
          else if (norm_sblk_index == 4) {
            if ((blk_remap == 3) || (blk_remap == 4) ||
                (blk_remap == 6)) {
                remaped_sblk = 0;
            }
            else if (blk_remap == 2) {
                remaped_sblk = 2;
            }
            else if (blk_remap == 5) {
                remaped_sblk = 1;
            }
            else if (blk_remap == 7) {
                remaped_sblk = 5;
            }
            else remaped_sblk = 4;
          }
          else if (norm_sblk_index == 5) {
            if ((blk_remap == 2) || (blk_remap == 5)) {
                remaped_sblk = 3;
            }
            else if ((blk_remap == 3) || (blk_remap == 4)) {
                remaped_sblk = 1;
            }
            else if (blk_remap == 6) {
                remaped_sblk = 2;
            }
            else if (blk_remap == 7) {
                remaped_sblk = 6;
            }
            else remaped_sblk = 5;
          }
          else if (norm_sblk_index == 6) {
            if (blk_remap == 1) {
                remaped_sblk = 0;
            }
            else if (blk_remap == 4) {
                remaped_sblk = 2;
            }
            else if (blk_remap == 5) {
                remaped_sblk = 5;
            }
            else if (blk_remap == 6) {
                remaped_sblk = 4;
            }
            else if (blk_remap == 7) {
                remaped_sblk = 7;
            }
            else remaped_sblk = 6;
          }
          else if (norm_sblk_index == 7) {
            if (blk_remap == 1) {
                remaped_sblk = 1;
            }
            else if (blk_remap == 4) {
                remaped_sblk = 3;
            }
            else if (blk_remap == 6) {
                remaped_sblk = 6;
            }
            else if (blk_remap == 7) {
                remaped_sblk = 0;
            }
            else remaped_sblk = 7;
          }
        }
        else assert(0);

        return remaped_sblk;
    }

  public:
    /** Convenience typedef. */
     typedef BaseSetAssocParams Params;

    /**
     * Construct and initialize this tag store.
     */
    BaseSetAssoc(const Params *p);

    /**
     * Destructor
     */
    virtual ~BaseSetAssoc() {};

    /**
     * Initialize blocks as CacheBlk instances.
     */
    void tagsInit() override;

    /**
     * This function updates the tags when a block is invalidated. It also
     * updates the replacement data.
     *
     * @param blk The block to invalidate.
     */
    void invalidate(CacheBlk *blk) override;

    /**
     * Access block and update replacement data. May not succeed, in which case
     * nullptr is returned. This has all the implications of a cache access and
     * should only be used as such. Returns the tag lookup latency as a side
     * effect.
     *
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param lat The latency of the tag lookup.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat,
                          unsigned &fh_flag) override
    {
        CacheBlk *blk = findBlock(addr, is_secure);

        // Get the requested subblock
        uint8_t subblk = indexingPolicy->extractSubBlock(addr);

        //Subblock disabling scheme.

        if (blk != nullptr) {
            //In case of MAEP scheme then apply the remapping
            if (useMaepScheme) {
                subblk = blk->remap ^ subblk;
            }
            else if (useFtaPredictor) {
                subblk = ftaPredictorSblkRemaping(subblk, blk->remap);
            }

            //Invalidate the block and put it in the bottom of lru list
            if (blk->faulty_flag[subblk] == 1) {
                replacementPolicy->touch(blk->replacementData);
                fh_flag =1;
                stats.falseHits++;
                assert(subBlocks != 1);
            }
        }

        // Access all tags in parallel, hence one in each way.  The data side
        // either accesses all blocks in parallel, or one block sequentially on
        // a hit.  Sequential access with a miss doesn't access data.
        stats.tagAccesses += allocAssoc;
        if (sequentialAccess) {
            if (blk != nullptr) {
                stats.dataAccesses += 1;
            }
        } else {
            stats.dataAccesses += allocAssoc;
        }

        // If a cache hit
        if (blk != nullptr) {
            // Update the utilization counters
            blk->util_cntrs[subblk] = 1;

            // Update number of references to accessed block
            blk->refCount++;

            // Update replacement data of accessed block
            replacementPolicy->touch(blk->replacementData);
        }

        // The tag lookup latency is the same for a hit or a miss
        lat = lookupLatency;

        return blk;
    }

    /**
     * Find replacement victim based on address. The list of evicted blocks
     * only contains the victim.
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param size Size, in bits, of new block to allocate.
     * @param evict_blks Cache blocks to be evicted.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(Addr addr, const bool is_secure,
                         const std::size_t size,
                         std::vector<CacheBlk*>& evict_blks,
                         uint64_t util_prediction,
                         Addr full_addr) const override
    {
        // Get possible entries to be victimized
        const std::vector<ReplaceableEntry*> entries =
            indexingPolicy->getPossibleEntries(addr);

        // Get the requested subblock
        uint8_t subblk = indexingPolicy->extractSubBlock(full_addr);

        CacheBlk* victim;
        // Choose replacement victim from replacement candidates
        if ((!useMaepScheme) && (!useFtaPredictor)) {
            victim = static_cast<CacheBlk*>(replacementPolicy->getVictim(
                                entries));
        }
        else {
            victim = static_cast<CacheBlk*>(
                     replacementPolicy->getVictimWithPred(
                     entries,util_prediction, subblk));
        }

        // Faulty block can not be accepted at block disabling scheme
        // Fatal error
        if ((victim != nullptr) && (subBlocks == 1)) {
            assert(victim->faulty_flag[0]==0);
        }

        // There is only one eviction for this replacement
        evict_blks.push_back(victim);

        return victim;
    }

    /**
     * Insert the new block into the cache and update replacement data.
     *
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
    void insertBlock(const PacketPtr pkt, CacheBlk *blk) override
    {
        // Insert block
        BaseTags::insertBlock(pkt, blk);

        // Increment tag counter
        stats.tagsInUse++;

        // Get the requested subblock
        const Addr addr = (pkt->req->hasPaddr() ? pkt->req->getPaddr() : 0);
        unsigned subblk = indexingPolicy->extractSubBlock(addr);
        // Update the utilization counters
        assert(blk->util_cntrs[subblk] == 0);
        blk->util_cntrs[subblk] = 1;

        // Update replacement policy
        replacementPolicy->reset(blk->replacementData);
    }

    /**
     * Limit the allocation for the cache ways.
     * @param ways The maximum number of ways available for replacement.
     */
    virtual void setWayAllocationMax(int ways) override
    {
        fatal_if(ways < 1, "Allocation limit must be greater than zero");
        allocAssoc = ways;
    }

    /**
     * Get the way allocation mask limit.
     * @return The maximum number of ways available for replacement.
     */
    virtual int getWayAllocationMax() const override
    {
        return allocAssoc;
    }

    /**
     * Regenerate the block address from the tag and indexing location.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override
    {
        return indexingPolicy->regenerateAddr(blk->tag, blk);
    }

    void forEachBlk(std::function<void(CacheBlk &)> visitor) override {
        for (CacheBlk& blk : blks) {
            visitor(blk);
        }
    }

    bool anyBlk(std::function<bool(CacheBlk &)> visitor) override {
        for (CacheBlk& blk : blks) {
            if (visitor(blk)) {
                return true;
            }
        }
        return false;
    }
};

#endif //__MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
