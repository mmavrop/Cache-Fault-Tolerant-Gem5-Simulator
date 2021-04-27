/*
 * Copyright (c) 2013,2016,2018-2019 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Ron Dreslinski
 */

/**
 * @file
 * Definitions of BaseTags.
 */

#include "mem/cache/tags/base.hh"

#include <cassert>

#include "base/types.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/request.hh"
#include "sim/core.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

BaseTags::BaseTags(const Params *p)
    : ClockedObject(p), blkSize(p->block_size), blkMask(blkSize - 1),
      size(p->size), lookupLatency(p->tag_latency),
      system(p->system), indexingPolicy(p->indexing_policy),
      subBlocks(p->sub_blocks),
      warmupBound((p->warmup_percentage/100.0) * (p->size / p->block_size)),
      warmedUp(false), numBlocks(p->size / p->block_size),
      dataBlks(new uint8_t[p->size]), // Allocate data storage in one big chunk
      stats(*this)
{
    registerExitCallback(new BaseTagsCallback(this));
}

ReplaceableEntry*
BaseTags::findBlockBySetAndWay(int set, int way) const
{
    return indexingPolicy->getEntry(set, way);
}

CacheBlk*
BaseTags::findBlock(Addr addr, bool is_secure) const
{
    // Extract block tag
    Addr tag = extractTag(addr);

    // Find possible entries that may contain the given address
    const std::vector<ReplaceableEntry*> entries =
        indexingPolicy->getPossibleEntries(addr);

    // Search for block
    for (const auto& location : entries) {
        CacheBlk* blk = static_cast<CacheBlk*>(location);
        if ((blk->tag == tag) && blk->isValid() &&
            (blk->isSecure() == is_secure)) {
            return blk;
        }
    }

    // Did not find block
    return nullptr;
}

void
BaseTags::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    assert(!blk->isValid());

    // Previous block, if existed, has been removed, and now we have
    // to insert the new one

    // Deal with what we are bringing in
    MasterID master_id = pkt->req->masterId();
    assert(master_id < system->maxMasters());
    stats.occupancies[master_id]++;

    // Insert block with tag, src master id and task id
    blk->insert(extractTag(pkt->getAddr()), pkt->isSecure(), master_id,
                pkt->req->taskId());

    // Check if cache warm up is done
    if (!warmedUp && stats.tagsInUse.value() >= warmupBound) {
        warmedUp = true;
        stats.warmupCycle = curTick();
    }

    // We only need to write into one tag and one data block.
    stats.tagAccesses += 1;
    stats.dataAccesses += 1;
}

Addr
BaseTags::extractTag(const Addr addr) const
{
    return indexingPolicy->extractTag(addr);
}

// Update the utilization caracteristics
void
BaseTags::updatePredictorStats(CacheBlk *blk)
{
    uint8_t left_part = 0;
    uint8_t right_part = 0;

    if (subBlocks == 8) {
        left_part  = blk->util_cntrs[0] + blk->util_cntrs[1] +
                     blk->util_cntrs[2] + blk->util_cntrs[3];
        right_part = blk->util_cntrs[4] + blk->util_cntrs[5] +
                     blk->util_cntrs[6] + blk->util_cntrs[7];
    }
    else if (subBlocks == 4) {
        left_part  = blk->util_cntrs[0] + blk->util_cntrs[1];
        right_part = blk->util_cntrs[2] + blk->util_cntrs[3];
    }
    else if (subBlocks == 2) {
        left_part  = blk->util_cntrs[0];
        right_part = blk->util_cntrs[1];
    }

    if (blk->prediction == 3){
        if ((left_part != 0) && (right_part != 0)) {
            stats.pred_oracle_accuracy++;
        }
        else {
            stats.pred_failures++;
        }
    }
    else { // prediction 0,1,2
        if (((left_part == 0) && (right_part != 0)) ||
            ((left_part != 0) && (right_part == 0)) ||
            ((left_part == 0) && (right_part == 0)))
        {
            stats.pred_oracle_accuracy++;
        }
        else {
            stats.pred_failures++;
        }

        if (blk->prediction == 0) {
            stats.pred_zero_val++;
        }
    }
}

void
BaseTags::updateMaepEraseFullutil(void)
{
    stats.maep_erase_total_util_history++;
}

void
BaseTags::updateMaepPredStats(CacheBlk *blk)
{
    bool maep_pred_failure = false;
    for (uint8_t iteration=0; iteration<subBlocks; iteration++) {
        if ((((blk->util_history & ((uint64_t)0xff << (8*iteration))) == 0x0)
                && (blk->util_cntrs[iteration] != 0)) ||
            (((blk->util_history & ((uint64_t)0xff << (8*iteration))) != 0x0)
                && (blk->util_cntrs[iteration] == 0))){
            maep_pred_failure = true;
        }
    }

    if (blk->util_history != 0) {
        if (maep_pred_failure) {
            stats.maep_failures++;
        }
        else {
            stats.maep_oracle_accuracy++;
        }
    }
    else {
        stats.maep_zero_val++;
    }
}

// Update the utilization caracteristics
void
BaseTags::updateUtilStats(CacheBlk *blk)
{
   unsigned int lowerHalfUtil = 0;
   unsigned int upperHalfUtil = 0;
   unsigned int subIndx;
   // Utilization statistics
   if (blk != nullptr) {
       if (subBlocks > 1) {
           for (subIndx = 0; subIndx < subBlocks; subIndx++) {
               if (blk->util_cntrs[subIndx] != 0) {
                   if (subIndx < subBlocks/2) lowerHalfUtil = 1;
                   else                       upperHalfUtil = 1;
               }
           }
       }
       else {
           for (subIndx = 0; subIndx < subBlocks; subIndx++) {
               if (blk->util_cntrs[subIndx] != 0) {
                   lowerHalfUtil = 1;
                   upperHalfUtil = 1;
               }
           }
       }

       if (lowerHalfUtil + upperHalfUtil == 2) {
           stats.totalUtil += 1;
       }
       else if (lowerHalfUtil + upperHalfUtil == 1) {
           stats.halfUtil += 1;
       }
       else if (lowerHalfUtil + upperHalfUtil == 0) {
           stats.zeroUtil += 1;
       }
       else {
           assert(0);
       }
   }
}

void
BaseTags::cleanupRefsVisitor(CacheBlk &blk)
{
    if (blk.isValid()) {
        stats.totalRefs += blk.refCount;
        ++stats.sampledRefs;
    }
}

void
BaseTags::cleanupRefs()
{
    forEachBlk([this](CacheBlk &blk) { cleanupRefsVisitor(blk); });
}

void
BaseTags::computeStatsVisitor(CacheBlk &blk)
{
    if (blk.isValid()) {
        assert(blk.task_id < ContextSwitchTaskId::NumTaskId);
        stats.occupanciesTaskId[blk.task_id]++;
        assert(blk.tickInserted <= curTick());
        Tick age = curTick() - blk.tickInserted;

        int age_index;
        if (age / SimClock::Int::us < 10) { // <10us
            age_index = 0;
        } else if (age / SimClock::Int::us < 100) { // <100us
            age_index = 1;
        } else if (age / SimClock::Int::ms < 1) { // <1ms
            age_index = 2;
        } else if (age / SimClock::Int::ms < 10) { // <10ms
            age_index = 3;
        } else
            age_index = 4; // >10ms

        stats.ageTaskId[blk.task_id][age_index]++;
    }
}

void
BaseTags::computeStats()
{
    for (unsigned i = 0; i < ContextSwitchTaskId::NumTaskId; ++i) {
        stats.occupanciesTaskId[i] = 0;
        for (unsigned j = 0; j < 5; ++j) {
            stats.ageTaskId[i][j] = 0;
        }
    }

    forEachBlk([this](CacheBlk &blk) { computeStatsVisitor(blk); });
}

std::string
BaseTags::print()
{
    std::string str;

    auto print_blk = [&str](CacheBlk &blk) {
        if (blk.isValid())
            str += csprintf("\tBlock: %s\n", blk.print());
    };
    forEachBlk(print_blk);

    if (str.empty())
        str = "no valid tags\n";

    return str;
}

BaseTags::BaseTagStats::BaseTagStats(BaseTags &_tags)
    : Stats::Group(&_tags),
    tags(_tags),

    tagsInUse(this, "tagsinuse",
              "Cycle average of tags in use"),
    totalRefs(this, "total_refs",
              "Total number of references to valid blocks."),
    sampledRefs(this, "sampled_refs",
                "Sample count of references to valid blocks."),
    avgRefs(this, "avg_refs",
            "Average number of references to valid blocks."),
    warmupCycle(this, "warmup_cycle",
                "Cycle when the warmup percentage was hit."),
    occupancies(this, "occ_blocks",
                "Average occupied blocks per requestor"),
    avgOccs(this, "occ_percent",
            "Average percentage of cache occupancy"),
    occupanciesTaskId(this, "occ_task_id_blocks",
                      "Occupied blocks per task id"),
    ageTaskId(this, "age_task_id_blocks", "Occupied blocks per task id"),
    percentOccsTaskId(this, "occ_task_id_percent",
                      "Percentage of cache occupancy per task id"),
    tagAccesses(this, "tag_accesses", "Number of tag accesses"),
    dataAccesses(this, "data_accesses", "Number of data accesses"),


    falseHits(this, "false_hits", "False hits"),

    pred_oracle_accuracy(this, "pred_oracle_accuracy",
                         "Predictor oracle accuracy"),
    pred_failures(this, "pred_failures", "Predictor failures"),
    pred_zero_val(this, "pred_zero_val", "predictor does not have answer"),

    maep_oracle_accuracy(this, "maep_oracle_accuracy",
                         "MAEP oracle accuracy"),
    maep_failures(this, "maep_failures", "MAEP failures"),
    maep_zero_val(this, "maep_zero_val", "MAEP does not have answer"),
    maep_erase_total_util_history(this, "maep_erase_total_util_history",
                                  "MAEP erase util history"),

    halfUtil(this, "half_utilization", "Blocks with 50\% Utilization"),
    totalUtil(this, "total_utilization", "Blocks with 100\% Utilization"),
    zeroUtil(this, "zero_utilization", "Blocks with 0\% Utilization")
{
}

void
BaseTags::BaseTagStats::regStats()
{
    using namespace Stats;

    Stats::Group::regStats();

    System *system = tags.system;

    avgRefs = totalRefs / sampledRefs;

    occupancies
        .init(system->maxMasters())
        .flags(nozero | nonan)
        ;
    for (int i = 0; i < system->maxMasters(); i++) {
        occupancies.subname(i, system->getMasterName(i));
    }

    avgOccs.flags(nozero | total);
    for (int i = 0; i < system->maxMasters(); i++) {
        avgOccs.subname(i, system->getMasterName(i));
    }

    avgOccs = occupancies / Stats::constant(tags.numBlocks);

    occupanciesTaskId
        .init(ContextSwitchTaskId::NumTaskId)
        .flags(nozero | nonan)
        ;

    ageTaskId
        .init(ContextSwitchTaskId::NumTaskId, 5)
        .flags(nozero | nonan)
        ;

    percentOccsTaskId.flags(nozero);

    percentOccsTaskId = occupanciesTaskId / Stats::constant(tags.numBlocks);
}

void
BaseTags::BaseTagStats::preDumpStats()
{
    Stats::Group::preDumpStats();

    tags.computeStats();
}
