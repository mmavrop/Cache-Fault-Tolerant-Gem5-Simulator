/**
 * Copyright (c) 2018 Inria
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
 * Authors: Daniel Carvalho
 */

#include "mem/cache/replacement_policies/lru_rp.hh"

#include <cassert>
#include <cmath>
#include <memory>

#include "mem/cache/cache_blk.hh"
#include "params/LRURP.hh"

LRURP::LRURP(const Params *p)
    : BaseReplacementPolicy(p), assoc(p->assoc), subBlocks(p->sub_blocks),
         useFtaPredictor(p->fta_predictor), useMaepScheme(p->maep_scheme)
{
}

void
LRURP::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
const
{
    // Reset last touch timestamp
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->lastTouchTick = Tick(0);
}

void
LRURP::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Update last touch timestamp
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->lastTouchTick = curTick();
}

void
LRURP::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set last touch timestamp
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->lastTouchTick = curTick();
}

ReplaceableEntry*
LRURP::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    unsigned int setFaultyCondition = 0;
    unsigned int wayFaultyCondition = 0;

    // Visit all candidates to find victim
    ReplaceableEntry* victim = nullptr; // = candidates[0];
    for (const auto& candidate : candidates) {

        wayFaultyCondition = 0;

        /*
         * block condition should be checked
         * a faulty block could not be a valid candidate
         */
        CacheBlk* blk = static_cast<CacheBlk*>(candidate);
        for (unsigned int subblk = 0; subblk < subBlocks; subblk++) {
            if (blk->faulty_flag[subblk] == 1) {
                wayFaultyCondition++;
            }
        }

        if (wayFaultyCondition == subBlocks) {
            setFaultyCondition++;
        }

        if ((victim == nullptr) && (wayFaultyCondition < subBlocks)) {
            victim = candidate;
        }
        else if ((victim == nullptr) && (wayFaultyCondition == subBlocks)){
            continue;
        }

        // Update victim entry if necessary
        if ((std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->lastTouchTick <
                std::static_pointer_cast<LRUReplData>(
                    victim->replacementData)->lastTouchTick)
             && (wayFaultyCondition < subBlocks)){
            victim = candidate;
        }
    }

    //In case of full-faulty set then return nullptr
    if (setFaultyCondition == assoc) {
        victim = nullptr;
    }

    return victim;
}

ReplaceableEntry*
LRURP::getVictimWithPred(const ReplacementCandidates& candidates,
                               uint64_t util_prediction,
                               uint8_t sblk) const
{
  // There must be at least one replacement candidate
  assert(candidates.size() > 0);

  unsigned int wayFaultyCondition = 0;
  unsigned int totalFaultySubblocks = 0;

  ReplaceableEntry* victim = nullptr;

  uint64_t timestamp_mru = 0;
  uint64_t timestamp_min = curTick();
  uint64_t blkTimeStamp = 0;
  uint8_t limit = assoc * subBlocks;
  uint8_t sum_missmatch[8] = {0};
  uint64_t match = 0;
  uint64_t access_pattern = 0;
  uint64_t access_pattern1 = 0;
  uint64_t error_pattern = 0;
  uint8_t min_missmatch = 0;
  bool total_match = false;

  if (useMaepScheme) {
    access_pattern = util_prediction;

    // Visit all candidates to find victim
    for (const auto& candidate : candidates) {

        wayFaultyCondition = 0;
        /*
         * block condition should be checked
         * a faulty block could not be a valid candidate
         */
        CacheBlk* blk = static_cast<CacheBlk*>(candidate);

       blkTimeStamp = std::static_pointer_cast<LRUReplData>(
                      candidate->replacementData)->lastTouchTick;
// printf("set: %u, way: %u : fault map: ", blk->getSet(), blk->getWay());
// for (uint8_t i=0; i<subBlocks; i++){
//    printf(" %u ", blk->faulty_flag[i]);
// }
//printf(" \n waycond: %u, set cond: %u \n",
//        wayFaultyCondition, totalFaultySubblocks);
//printf(" timestamp: %lu \n ", blkTimeStamp);

        //MAEPER
        min_missmatch = 255;
        error_pattern = 0;
        access_pattern1 = 0;

        for (uint8_t i=0; i<subBlocks; i++){
            if (blk->faulty_flag[i] == 1) {
                wayFaultyCondition ++;
                totalFaultySubblocks ++;
            }
        }

        error_pattern = (uint64_t) blk->faulty_flag[0] +
                         ((uint64_t) blk->faulty_flag[1] << 8) +
                         ((uint64_t) blk->faulty_flag[2] << 16) +
                         ((uint64_t) blk->faulty_flag[3] << 24) +
                         ((uint64_t) blk->faulty_flag[4] << 32) +
                         ((uint64_t) blk->faulty_flag[5] << 40) +
                         ((uint64_t) blk->faulty_flag[6] << 48) +
                         ((uint64_t) blk->faulty_flag[7] << 56);

        for (uint8_t cond=0; cond<8; cond++){
            if (cond == 0) {
                match = error_pattern & access_pattern;
            }
            else if (cond == 1) {
                access_pattern1 =
                        ((access_pattern << 8) & 0x1100110011001100) |
                        ((access_pattern >> 8) & 0x0011001100110011) ;
                match = error_pattern & access_pattern1;
            }
            else if (cond == 2) {
                access_pattern1 =
                        ((access_pattern << 16) & 0x1111000011110000) |
                         ((access_pattern >> 16) & 0x0000111100001111) ;
                match = error_pattern & access_pattern1;
            }
            else if (cond == 3) {
                access_pattern1 =
                        ((access_pattern << 8) & 0x1100110011001100) |
                        ((access_pattern >> 8) & 0x0011001100110011) ;
                access_pattern1 =
                        ((access_pattern1 << 16) & 0x1111000011110000) |
                        ((access_pattern1 >> 16) & 0x0000111100001111) ;
                match = error_pattern & access_pattern1;
            }
            else if (cond == 4) {
                access_pattern1 =
                        ((access_pattern << 32) & 0x1111111100000000) |
                        ((access_pattern >> 32) & 0x0000000011111111) ;
                match = error_pattern & access_pattern1;
            }
            else if (cond == 5) {
                access_pattern1 =
                        ((access_pattern << 8) & 0x1100110011001100) |
                        ((access_pattern >> 8) & 0x0011001100110011) ;
                access_pattern1 =
                        ((access_pattern1 << 32) & 0x1111111100000000) |
                        ((access_pattern1 >> 32) & 0x0000000011111111) ;
                match = error_pattern & access_pattern1;
            }
            else if (cond == 6) {
                access_pattern1 =
                        ((access_pattern << 16) & 0x1111000011110000) |
                        ((access_pattern >> 16) & 0x0000111100001111) ;
                access_pattern1 =
                        ((access_pattern1 << 32) & 0x1111111100000000) |
                        ((access_pattern1 >> 32) & 0x0000000011111111) ;
                match = error_pattern & access_pattern1;
            }
            else if (cond == 7) {
                access_pattern1 =
                        ((access_pattern << 8) & 0x1100110011001100) |
                        ((access_pattern >> 8) & 0x0011001100110011) ;
                access_pattern1 =
                        ((access_pattern1 << 16) & 0x1111000011110000) |
                        ((access_pattern1 >> 16) & 0x0000111100001111) ;
                access_pattern1 =
                        ((access_pattern1 << 32) & 0x1111111100000000) |
                        ((access_pattern1 >> 32) & 0x0000000011111111) ;
                match = error_pattern & access_pattern1;
            }
//    printf("set: %u way: %u cond: %u\n", blk->getSet(), blk->getWay(), cond);
//    printf("er_pat: %lx, ac_pat1: %lx, ac_pat: %lx, match: %lx\n",
//                    error_pattern, access_pattern1, access_pattern, match);
//    printf("cand: %lu tmstamp_min: %lu\n",
//            (std::static_pointer_cast<LRUReplData>(
//            candidate->replacementData)->lastTouchTick),
//            timestamp_min);

            // Update victim entry if necessary
            if ((std::static_pointer_cast<LRUReplData>(
                candidate->replacementData)->lastTouchTick < timestamp_min) &&
                (match == 0) && (error_pattern != 0x0101010101010101 )) {
                    timestamp_min = std::static_pointer_cast<LRUReplData>(
                                    candidate->replacementData)->lastTouchTick;
                    blk->remap = cond;
                    victim = candidate;
                    total_match = true;
            }
            else {
                sum_missmatch[cond]=0;
                if (match & 0x01)
                        sum_missmatch[cond]++;
                if (match & 0x0100)
                        sum_missmatch[cond]++;
                if (match & 0x010000)
                        sum_missmatch[cond]++;
                if (match & 0x01000000)
                        sum_missmatch[cond]++;
                if (match & 0x0100000000)
                        sum_missmatch[cond]++;
                if (match & 0x010000000000)
                        sum_missmatch[cond]++;
                if (match & 0x01000000000000)
                        sum_missmatch[cond]++;
                if (match & 0x0100000000000000)
                        sum_missmatch[cond]++;

                if (sum_missmatch[cond] < min_missmatch) {
                     //printf("min_cond: %u\n", cond);
                    min_missmatch = sum_missmatch[cond];
                    blk->min_cond = cond;
                    blk->min_missmatch = sum_missmatch[cond];
                }
            }
        }
    }
//----------------------
    if (totalFaultySubblocks == limit){
        return nullptr;
    }

    if (!total_match) { //didnt found total match in lru
//            printf("Didnt found any way and remaping where"
//           	   " there is exact match\n"
//  		    "Find the best solution min missmatch and lru \n");
        assert(timestamp_min == curTick());
        min_missmatch = 8;

        for (const auto& candidate : candidates) {

            CacheBlk* blk = static_cast<CacheBlk*>(candidate);
            blkTimeStamp = std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->lastTouchTick;
            wayFaultyCondition = 0;

            for (uint8_t i=0; i<subBlocks; i++){
                if (blk->faulty_flag[i] == 1) {
                    wayFaultyCondition ++;
                }
            }

            if (wayFaultyCondition < subBlocks) {
                if (blk->min_missmatch < min_missmatch) {
                    blk->remap = blk->min_cond;
                    victim = candidate;
                    min_missmatch = blk->min_missmatch;
                    timestamp_min = blkTimeStamp;
                }
                else if (blk->min_missmatch == min_missmatch) {
                    if (blkTimeStamp < timestamp_min) {
                        blk->remap = blk->min_cond;
                        victim = candidate;
                        timestamp_min = blkTimeStamp;
                    }
                }
            }
        }
    }
//printf("victim->remap: %u, way: %u\n",
//            static_cast<CacheBlk*>(victim)->remap,
//            static_cast<CacheBlk*>(victim)->getWay() );
    return victim;
  } //useMaepScheme end
  else if (useFtaPredictor) {

     uint8_t sub0_condition = 0;
     uint8_t sub1_condition = 0;

     // Visit all candidates to find mru
     for (const auto& candidate : candidates) {
        if (std::static_pointer_cast<LRUReplData>(
                candidate->replacementData)->lastTouchTick > timestamp_mru) {
            timestamp_mru = std::static_pointer_cast<LRUReplData>(
                                candidate->replacementData)->lastTouchTick;
        }
     }

     // Visit all candidates to find victim
     for (const auto& candidate : candidates) {

       wayFaultyCondition = 0;
       /*
        * block condition should be checked
        * a faulty block could not be a valid candidate
        */
       CacheBlk* blk = static_cast<CacheBlk*>(candidate);
       blkTimeStamp = std::static_pointer_cast<LRUReplData>(
                      candidate->replacementData)->lastTouchTick;
       for (uint8_t i=0; i<subBlocks; i++){
          if (blk->faulty_flag[i] == 1) {
               wayFaultyCondition ++;
               totalFaultySubblocks ++;
           }
       }

       //In case of total faulty blocks continue
       if (wayFaultyCondition == subBlocks) {
           continue;
       }

       if (subBlocks == 2) {
       // printf("candidate timestamp: %lu, min_timestamp: %lu\n",
       //             std::static_pointer_cast<LRUReplData>(
       //                 candidate->replacementData)->lastTouchTick,
       //              timestamp_min);
           if (blkTimeStamp < timestamp_min) {
                timestamp_min = blkTimeStamp;
                blk->remap = (blk->faulty_flag[sblk] == 0) ? 0 : 1;
                victim = candidate;
                total_match = true;
           }
       }
       else if (subBlocks == 4) {
         sub0_condition = blk->faulty_flag[0] + blk->faulty_flag[1];
         sub1_condition = blk->faulty_flag[2] + blk->faulty_flag[3];
       // printf("sub0: %u, sub1:%u\n", sub0_condition, sub1_condition);
       // printf("candidate timestamp: %lu, min_timestamp: %lu\n",
       //             std::static_pointer_cast<LRUReplData>(
       //                 candidate->replacementData)->lastTouchTick,
       //              timestamp_min);

         if (blkTimeStamp < timestamp_min) {
            //printf("Oportunity with way cond: %u\n", wayFaultyCondition);
            // If there is healthy space for the half block then this
            // block it whould be a condicate
            if (wayFaultyCondition <= 2 ) {
                timestamp_min = blkTimeStamp;
                victim = candidate;
                total_match = true;
              //printf("The way condition indicates a valid candidate!\n");
              //printf("New timestamp_min: %lu\n", timestamp_min);
            }

            uint8_t sblk_2 = uint8_t(sblk / 2);
            //printf("Req subblk: %u, ~sub: %u\n", sblk, sblk_2);
            if ((blk->faulty_flag[2*sblk_2] == 0) &&
                (blk->faulty_flag[2*sblk_2+1] == 0))
            {
                blk->remap = 0;
            }
            else if ((blk->faulty_flag[2*!sblk_2] == 0) &&
                     (blk->faulty_flag[2*!sblk_2+1] == 0))
            {
                blk->remap = 4;
            }
            else if ((sub0_condition == 1) && (sub1_condition == 1))
            {
              if (sblk_2 == 0)
              {
                  if ((blk->faulty_flag[0] == 0) &&
                      (blk->faulty_flag[2] == 0)) {
                      blk->remap = 2;
                  }
                  else if ((blk->faulty_flag[0] == 0) &&
                           (blk->faulty_flag[3] == 0)) {
                       blk->remap = 7;
                  }
                  else if ((blk->faulty_flag[1] == 0) &&
                           (blk->faulty_flag[2] == 0)) {
                        blk->remap = 3;
                  }
                  else if ((blk->faulty_flag[1] == 0) &&
                           (blk->faulty_flag[3] == 0)) {
                        blk->remap = 6;
                  }
              }
              else if (sblk_2 == 1){
                  if ((blk->faulty_flag[0] == 0) &&
                      (blk->faulty_flag[2] == 0)) {
                      blk->remap = 6;
                  }
                  else if ((blk->faulty_flag[0] == 0) &&
                           (blk->faulty_flag[3] == 0)) {
                      blk->remap = 3;
                  }
                  else if ((blk->faulty_flag[1] == 0) &&
                           (blk->faulty_flag[2] == 0)) {
                      blk->remap = 7;
                  }
                  else if ((blk->faulty_flag[1] == 0) &&
                           (blk->faulty_flag[3] == 0)) {
                       blk->remap = 2;
                  }
              }
            }
          }
        }
        else if (subBlocks == 8) {

            if ((util_prediction == 1) || (util_prediction == 2) ||
                (util_prediction == 0))
            {
                if ((sblk == 0) || (sblk == 1) ||
                    (sblk == 2) || (sblk == 3)) {
                  access_pattern = 15;
                }
                else {
                  access_pattern = 240;
                }
            }
            else //if (our_pred == 3)	access_pattern = 255;
            {
              access_pattern = 255;
              assert(util_prediction == 3);
            }
            //printf("sblk: %u, prediction: %lu, accss_pattern: %lu\n",
            //             sblk, util_prediction, access_pattern);

            min_missmatch = 255;
            error_pattern = 0;
            for (uint8_t i=0;i<subBlocks;i++){
              if (blk->faulty_flag[i] == 1) {
                error_pattern = error_pattern + (unsigned long int) pow(2,i);
              }
            }
            //printf("error pattern: %lu\n", error_pattern);

            for (uint8_t cond=0; cond<8; cond++){
               if (cond == 0) {
                 match = error_pattern & access_pattern;
               }
               else if (cond == 1) {
                 if (access_pattern == 15) access_pattern1 = 204;
                 else if (access_pattern == 240) access_pattern1 = 51;
                 else access_pattern1 = access_pattern;
                 match = error_pattern & access_pattern1;
               }
               else if (cond == 2) {
                 if (access_pattern == 15) access_pattern1 = 51;
                 else if (access_pattern == 240) access_pattern1 = 204;
                 else access_pattern1 = access_pattern;
                 match = error_pattern & access_pattern1;
               }
               else if (cond == 3) {
                 if (access_pattern == 15) access_pattern1 = 60;
                 else if (access_pattern == 240) access_pattern1 = 195;
                 else access_pattern1 = access_pattern;
                 match = error_pattern & access_pattern1;
               }
               else if (cond == 4) {
                 access_pattern1 = ((access_pattern << 4) & 240) |
                                   ((access_pattern >> 4) & 15) ;
                 match = error_pattern & access_pattern1;
               }
               else if (cond == 5) {
                 if (access_pattern == 15) access_pattern1 = 85;
                 else if (access_pattern == 240) access_pattern1 = 170;
                 else access_pattern1 = access_pattern;
                 match = error_pattern & access_pattern1;
               }
               else if (cond == 6) {
                 if (access_pattern == 15) access_pattern1 = 170;
                 else if (access_pattern == 240) access_pattern1 = 85;
                 else access_pattern1 = access_pattern;
                 match = error_pattern & access_pattern1;
               }
               else if (cond == 7) {
                 if (access_pattern == 15) access_pattern1 = 30;
                 else if (access_pattern == 240) access_pattern1 = 225;
                 else access_pattern1 = access_pattern;
                 match = error_pattern & access_pattern1;
               }

// printf("candidate tstamp: %lu, min_timestamp: %lu, match: %lu, mru: %lu\n",
//         std::static_pointer_cast<LRUReplData>(
//         candidate->replacementData)->lastTouchTick,
//         timestamp_min, match, timestamp_mru);

               if ((blkTimeStamp < timestamp_min) &&
                  ((blkTimeStamp != timestamp_mru) || (timestamp_mru == 0)) &&
                   (match == 0) && (error_pattern < 255 )) {
                 timestamp_min = std::static_pointer_cast<LRUReplData>(
                                 candidate->replacementData)->lastTouchTick;
                 blk->remap = cond;
                 victim = candidate;
                 total_match = true;
                 //printf("Total match: remap: %u\n", blk->remap);
               }
               else {
                 sum_missmatch[cond]=0;
                 if (match & 1) sum_missmatch[cond]++;
                 if (match & 2) sum_missmatch[cond]++;
                 if (match & 4) sum_missmatch[cond]++;
                 if (match & 8) sum_missmatch[cond]++;
                 if (match & 16) sum_missmatch[cond]++;
                 if (match & 32) sum_missmatch[cond]++;
                 if (match & 64) sum_missmatch[cond]++;
                 if (match & 128) sum_missmatch[cond]++;

                 if (sum_missmatch[cond] < min_missmatch) {
                   min_missmatch = sum_missmatch[cond];
                   blk->min_cond = cond;
                   blk->min_missmatch = sum_missmatch[cond];
                   //printf("blk min massmatch: %u, cond: %u\n",
                   //        blk->min_missmatch, blk->min_cond);
                 }
               }
            }
        }
    }
//----------------------
    if (totalFaultySubblocks == limit){
        return nullptr;
    }

    if (!total_match) { //didnt found total match in lru
        //printf("Didnt found any way and remaping where"
        //       " there is exact match\n"
        //"Find the best solution min missmatch and lru \n");
        assert(timestamp_min == curTick());

        uint64_t timestamp_min0 = timestamp_min;
        uint64_t timestamp_min1 = timestamp_min;
        uint8_t requestedCleared = 0;
                uint8_t min_missmatch1 = 255;
        min_missmatch = 255;

        for (const auto& candidate : candidates)
        {
            CacheBlk* blk = static_cast<CacheBlk*>(candidate);
            blkTimeStamp = std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->lastTouchTick;
            wayFaultyCondition = 0;

            for (uint8_t i=0; i<subBlocks; i++){
                if (blk->faulty_flag[i] == 1) {
                    wayFaultyCondition ++;
                }
            }
            //printf("WayFaultyCond: %u\n", wayFaultyCondition);
            if (subBlocks == 4) {
               // printf("blkTimeStamp: %lu, min_timestamp: %lu\n",
               //      blkTimeStamp, timestamp_min);
               if ((blkTimeStamp < timestamp_min) &&
                     (wayFaultyCondition < subBlocks )){
                    assert(wayFaultyCondition == 3);

                    victim = candidate;
                    timestamp_min = blkTimeStamp;
                //printf("New cand: way:%u, new min tstamp:%lu\n",
                //        blk->getWay(), timestamp_min);
                    uint8_t sblk_2 = uint8_t(sblk / 2);

                    if ((blk->faulty_flag[0] == 1) &&
                        (blk->faulty_flag[1] == 1)){
                        if (sblk_2 == 0) {
                           blk->remap = 4;
                        }
                        else {
                            blk->remap = 0;
                        }
                    }
                    else if ((blk->faulty_flag[2] == 1) &&
                             (blk->faulty_flag[3] == 1)){
                        if (sblk_2 == 0) {
                            blk->remap = 0;
                        }
                        else {
                            blk->remap = 4;
                        }
                    }
                    //printf("remap: %u\n", blk->remap);
                }
            }
            else if (subBlocks == 8) {
                // printf("way condition: %u, sbblocks: %u\n",
                //       wayFaultyCondition, subBlocks);
                if (util_prediction != 3)
                {
                  if ((wayFaultyCondition < subBlocks) &&
                      ((blkTimeStamp != timestamp_mru) ||
                       (timestamp_mru == 0))) {
                      //printf("blk min missmatch: %u, min_missmatch: %u\n",
                      //      blk->min_missmatch, min_missmatch );
                      if (blk->min_missmatch  < min_missmatch) {
                          blk->remap = blk->min_cond;
                          victim = candidate;
                          min_missmatch = blk->min_missmatch;
                          timestamp_min = blkTimeStamp;
                          //printf("1) New cand: remap: %u\n", blk->remap);
                      }
                      else if (blk->min_missmatch == min_missmatch) {
                          if (blkTimeStamp < timestamp_min) {
                              blk->remap = blk->min_cond;
                              victim = candidate;
                              timestamp_min = blkTimeStamp;
                            //printf("2)New cand:remap:%u\n", blk->remap);
                          }
                      }
                  }
                }
                else {
                  if (sblk == 0) access_pattern = 127;
                  else if (sblk == 1) access_pattern = 254;
                  else if (sblk == 2) access_pattern = 252;
                  else if (sblk == 3) access_pattern = 248;
                  else if (sblk == 4) access_pattern = 240;
                  else if (sblk == 5) access_pattern = 224;
                  else if (sblk == 6) access_pattern = 192;
                  else if (sblk == 7) access_pattern = 128;

                  error_pattern = 0;
                  for (uint8_t i=0; i<subBlocks; i++){
                    if (blk->faulty_flag[i] == 1) {
                      error_pattern = error_pattern +
                                         (unsigned long int) pow(2,i);
                    }
                  }

                  for (uint8_t cond=0; cond<8; cond++){
                    if (cond == 0) {
                      match = error_pattern & access_pattern;
                    }
                    else if (cond == 1) {
                      if (access_pattern == 254) access_pattern1 = 251;
                      else if (access_pattern == 252) access_pattern1 = 243;
                      else if (access_pattern == 248) access_pattern1 = 179;
                      else if (access_pattern == 127) access_pattern1 = 253;
                      else if (access_pattern == 240) access_pattern1 = 51;
                      else if (access_pattern == 224) access_pattern1 = 35;
                      else if (access_pattern == 192) access_pattern1 = 3;
                      else if (access_pattern == 128) access_pattern1 = 2;
                      match = error_pattern & access_pattern1;
                    }
                    else if (cond == 2) {
                      if (access_pattern == 254) access_pattern1 = 254;
                      else if (access_pattern == 252) access_pattern1 = 252;
                      else if (access_pattern == 248) access_pattern1 = 236;
                      else if (access_pattern == 127) access_pattern1 = 127;
                      else if (access_pattern == 240) access_pattern1 = 204;
                      else if (access_pattern == 224) access_pattern1 = 200;
                      else if (access_pattern == 192) access_pattern1 = 192;
                      else if (access_pattern == 128) access_pattern1 = 128;
                      match = error_pattern & access_pattern1;
                    }
                    else if (cond == 3) {
                      if (access_pattern == 254) access_pattern1 = 251;
                      else if (access_pattern == 252) access_pattern1 = 243;
                      else if (access_pattern == 248) access_pattern1 = 227;
                      else if (access_pattern == 127) access_pattern1 = 127;
                      else if (access_pattern == 240) access_pattern1 = 195;
                      else if (access_pattern == 224) access_pattern1 = 194;
                      else if (access_pattern == 192) access_pattern1 = 192;
                      else if (access_pattern == 128) access_pattern1 = 128;
                      match = error_pattern & access_pattern1;
                    }
                    else if (cond == 4) {
                      access_pattern1 = ((access_pattern << 4) & 240) |
                                         ((access_pattern >> 4) & 15) ;
                      match = error_pattern & access_pattern1;
                    }
                    else if (cond == 5) {
                      if (access_pattern == 254) access_pattern1 = 254;
                      else if (access_pattern == 252) access_pattern1 = 250;
                      else if (access_pattern == 248) access_pattern1 = 234;
                      else if (access_pattern == 127) access_pattern1 = 127;
                      else if (access_pattern == 240) access_pattern1 = 170;
                      else if (access_pattern == 224) access_pattern1 = 168;
                      else if (access_pattern == 192) access_pattern1 = 160;
                      else if (access_pattern == 128) access_pattern1 = 128;
                      match = error_pattern & access_pattern1;
                    }
                    else if (cond == 6) {
                      if (access_pattern == 254) access_pattern1 = 253;
                      else if (access_pattern == 252) access_pattern1 = 245;
                      else if (access_pattern == 248) access_pattern1 = 213;
                      else if (access_pattern == 127) access_pattern1 = 191;
                      else if (access_pattern == 240) access_pattern1 = 85;
                      else if (access_pattern == 224) access_pattern1 = 84;
                      else if (access_pattern == 192) access_pattern1 = 80;
                      else if (access_pattern == 128) access_pattern1 = 64;
                      match = error_pattern & access_pattern1;
                    }
                    else if (cond == 7) {
                      if (access_pattern == 254) access_pattern1 = 253;
                      else if (access_pattern == 252) access_pattern1 = 249;
                      else if (access_pattern == 248) access_pattern1 = 241;
                      else if (access_pattern == 127) access_pattern1 = 254;
                      else if (access_pattern == 240) access_pattern1 = 225;
                      else if (access_pattern == 224) access_pattern1 = 193;
                      else if (access_pattern == 192) access_pattern1 = 129;
                      else if (access_pattern == 128) access_pattern1 = 1;
                      match = error_pattern & access_pattern1;
                    }

                    if ((blkTimeStamp < timestamp_min) &&
                        ((blkTimeStamp != timestamp_mru) ||
                         (timestamp_mru == 0)) &&
                        (match == 0) && (error_pattern < 255 )) {
                      timestamp_min = blkTimeStamp;
                      blk->remap = cond;
                      victim = candidate;
                      min_missmatch1 = 0;
                      min_missmatch = 0;
                      // printf("Total match: remap: %u\n", blk->remap);
                    }
                    else {
                      uint8_t sblk_poss = 0;
                      if (sblk == 0){
                        if ((cond == 1) || (cond == 3)) sblk_poss = 2;
                        else if (cond == 4) sblk_poss= 4;
                        else if ((cond == 6) || (cond == 7)) sblk_poss = 1;
                        else sblk_poss = 0;
                      }
                      else if (sblk == 1) {
                        if ((cond == 1) || (cond == 3) || (cond == 6))
                           sblk_poss = 3;
                        else if ((cond == 5) || (cond == 7)) sblk_poss = 2;
                        else if (cond == 4) sblk_poss = 5;
                        else sblk_poss = 1;
                      }
                      else if (sblk == 2) {
                        if ((cond == 1) || (cond == 4)) sblk_poss = 6;
                        else if ((cond == 2) || (cond == 3) || (cond == 5))
                           sblk_poss = 4;
                        else if (cond == 6) sblk_poss = 5;
                        else if (cond == 7) sblk_poss = 3;
                        else sblk_poss = 2;
                      }
                      else if (sblk == 3) {
                        if ((cond == 1) || (cond == 4) || (cond == 6))
                           sblk_poss = 7;
                        else if ((cond == 2) || (cond == 3)) sblk_poss = 5;
                        else if (cond == 7) sblk_poss = 4;
                        else sblk_poss = 3;
                      }
                      else if (sblk == 4) {
                        if ((cond == 3) || (cond == 4) || (cond == 6))
                           sblk_poss = 0;
                        else if (cond == 2) sblk_poss = 2;
                        else if (cond == 5) sblk_poss = 1;
                        else if (cond == 7) sblk_poss = 5;
                        else sblk_poss = 4;
                      }
                      else if (sblk == 5) {
                        if ((cond == 2) || (cond == 5)) sblk_poss = 3;
                        else if ((cond == 3) || (cond == 4)) sblk_poss = 1;
                        else if (cond == 6) sblk_poss = 2;
                        else if (cond == 7) sblk_poss = 6;
                        else sblk_poss = 5;
                      }
                      else if (sblk == 6) {
                        if (cond == 1) sblk_poss = 0;
                        else if (cond == 4) sblk_poss = 2;
                        else if (cond == 5) sblk_poss = 5;
                        else if (cond == 6) sblk_poss = 4;
                        else if (cond == 7) sblk_poss = 7;
                        else sblk_poss = 6;
                      }
                      else if (sblk == 7) {
                        if (cond == 1) sblk_poss = 1;
                        else if (cond == 4) sblk_poss = 3;
                        else if (cond == 6) sblk_poss = 6;
                        else if (cond == 7) sblk_poss = 0;
                        else sblk_poss = 7;
                      }

                      sum_missmatch[cond]=0;
                      if (match & 1) sum_missmatch[cond]++;
                      if (match & 2) sum_missmatch[cond]++;
                      if (match & 4) sum_missmatch[cond]++;
                      if (match & 8) sum_missmatch[cond]++;
                      if (match & 16) sum_missmatch[cond]++;
                      if (match & 32) sum_missmatch[cond]++;
                      if (match & 64) sum_missmatch[cond]++;
                      if (match & 128) sum_missmatch[cond]++;

                      if (blk->faulty_flag[sblk_poss] == 0)
                      {
                         if ((sum_missmatch[cond] < min_missmatch) &&
                                (blkTimeStamp < timestamp_min0)) {
                             timestamp_min0 = blkTimeStamp;
                             blk->remap = cond;
                             victim = candidate;
                             min_missmatch1 = sum_missmatch[cond];
                             requestedCleared = 1;
                             // printf("blk min massmatch: %u, cond: %u\n",
                             //        blk->min_missmatch, blk->min_cond);
                          }
                      }

                      if (requestedCleared == 0)
                      {
                          if ((sum_missmatch[cond] < min_missmatch1) &&
                                (blkTimeStamp < timestamp_min1)) {
                              timestamp_min1 = blkTimeStamp;
                              blk->remap = cond;
                              victim = candidate;
                              min_missmatch1 = sum_missmatch[cond];
                              //requestedCleared = 1;
                          //printf("faulty req subblok blk min massmatch: %u,
                          // cond: %u\n", blk->min_missmatch, blk->min_cond);
                          }
                      }
                    }
                  }
               }
            }
        }
    }

    // In case of 2 and 4 subblocks check the prediction
    if (subBlocks != 8)
    {
        assert ((subBlocks == 2) || (subBlocks == 4));

        if (util_prediction == 3) {
            //printf("util_pred: %lu, need to repl full healthy block\n",
            //                 util_prediction);
            timestamp_min = curTick()+1;
            CacheBlk* blk_cand = NULL;
            uint8_t blkFCondition[8] = {0};

            for (const auto& candidate : candidates)
            {
                blk_cand = static_cast<CacheBlk*>(candidate);
                uint8_t way = blk_cand->getWay();

                blkTimeStamp = std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->lastTouchTick;

                for (uint8_t subblock = 0; subblock < subBlocks; subblock++) {
                    blkFCondition[way] = blkFCondition[way] +
                           blk_cand->faulty_flag[subblock];
                }
             //printf("%u: Cand tstamp: %lu, tstamp_min: %lu, wayCond: %u\n",
             //        way, blkTimeStamp, timestamp_min, blkFCondition[way]);
                if ((blkTimeStamp < timestamp_min) &&
                    ((blkTimeStamp != timestamp_mru) ||
                       (timestamp_mru == 0)) &&
                    (blkFCondition[way] == 0)) {
                    victim = candidate;
                    blk_cand->remap = 0;
                    timestamp_min = blkTimeStamp;
                  //  printf("New candidate: remap: %u, timestamp_min: %lu\n",
                  //              blk_cand->remap, timestamp_min);
                }
            }
                }
    }

//  printf("victim->remap: %u, way: %u\n",
//            static_cast<CacheBlk*>(victim)->remap,
//            static_cast<CacheBlk*>(victim)->getWay() );
    return victim;

  }

  assert(0);
  return nullptr;
}

std::shared_ptr<ReplacementData>
LRURP::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new LRUReplData());
}

LRURP*
LRURPParams::create()
{
    return new LRURP(this);
}
