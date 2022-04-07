#include <cmath>
#include <cstdio>

#include <mem/cache/predictor/fta_predictor.hh>
extern "C" {

ftaPredictor predictor;

ftaPredictor* create_fta(void)
{
        predictor_init();
        return &predictor;
}

void predictor_init ()
{
        int i;
        for (i = 0 ; i < pred_entries ; i++ )
        {
                predictor.offset[i] = 0 ;
                predictor.valid[i] = 0 ;
                predictor.pc[i] = 0 ;
                predictor.sb1[i]  = 0;
                predictor.sb2[i]  = 0;
                predictor.cc[i] = init_conf_value ;
                predictor.timestamp[i] = 0;

                predictor.pc_match_during_predict[i] = 0;
                predictor.pc_match_during_update[i] = 0;
                predictor.pc_correct[i] = 0;
        }
        predictor.cur_entries = 0;
        predictor.total_pc = 0;
        predictor.total_updates = 0;
        predictor.total_pc_match_during_update = 0;
        predictor.total_pc_match_during_predict = 0;
        predictor.total_pc_correct = 0;
        predictor.total_repl = 0;
}


int predictor_predict ( Addr pc, Addr offset, Tick tstamp )
{
        if (pc == 0) return 0;
        int pred = 0;
        predictor.total_pc ++ ;

  //      printf("--- Predict pc: 0x%lx \t offset: 0x%lx \n",pc,offset);

#ifdef PRED_ADDR
        offset = offset & PRED_ADDR_MASK;

        if (PRED_ADDR_MASK == 0xff000000) {
            offset = offset >> 24;
        }
        else if (PRED_ADDR_MASK == 0xfc000000) {
            offset = offset >> 26;
        }
        else if (PRED_ADDR_MASK == 0xffc00000) {
            offset = offset >> 22;
        }
        else if (PRED_ADDR_MASK == 0xfff00000) {
            offset = offset >> 20;
        }
        else if (PRED_ADDR_MASK == 0xffff0000) {
            offset = offset >> 16;
        }
        else if (PRED_ADDR_MASK == 0x3fc0) {
            offset = offset >> 6;
        }
        else if (PRED_ADDR_MASK == 0x3fffc0) {
            offset = offset >> 6;
        }
        else if (PRED_ADDR_MASK == 0xfc0) {
            offset = offset >> 6;
        }
        else if (PRED_ADDR_MASK == 0xffc0) {
            offset = offset >> 6;
        }
        else if (PRED_ADDR_MASK == 0x3ffc0) {
            offset = offset >> 6;
        }
#endif

#ifdef PRED_PC
        pc = pc & PRED_PC_MASK;

        if (PRED_PC_MASK == 0xff000000) {
            pc = pc >> 24;
        }
        else if (PRED_PC_MASK == 0xfc000000) {
            pc = pc >> 26;
        }
        else if (PRED_PC_MASK == 0xffff0000) {
            pc = pc >> 16;
        }
#endif

//printf("PREDICT: pc:0x%lx, offset: 0x%lx \n\n", pc, offset);

//printf("\n--------- Predictor entries --------\n");
//for ( i=0; i < 64 ; i++ )
//{
//    printf("%d) PC: %llx \t OFFSET: %llx \t SB2: %d \t SB1: %d \n",
//           i, predictor.pc[i], predictor.offset[i],
//           predictor.sb2[i], predictor.sb1[i]);
//    printf("%d) VALID: %d \t OFFSET: %d \t SB2: %d \t SB1: %d \n",
//           i, predictor.valid[i],predictor.offset[i],
//           predictor.sb2[i], predictor.sb1[i]);
//}
//printf("\n");
#ifdef NO_TAG_PRED
#ifdef PRED_PC
offset = pc;
#endif

#ifdef CONFIDENCE_CNT
    #ifdef sec_impl
     #ifndef not_allways
        if (predictor.cc[offset] >= 2){
            return 3;
        }
        else if (predictor.cc[offset] < 2){
            return 1;
        }
        assert(0);
     #endif
    #endif
#endif
//printf("offset: %lx\n", offset);
if (predictor.valid[offset] == 1)
{
        //printf("offset = %lu, %d, %d \n",
        //        offset,predictor.sb1[offset], predictor.sb2[offset] );
        if (predictor.sb1[offset] == 1) pred = pred + (int) pow(2,0);
        if (predictor.sb2[offset] == 1) pred = pred + (int) pow(2,1);

        #ifdef CONFIDENCE_CNT
                #ifdef init_impl
                        if (predictor.cc[offset] >= safe_limit){
                                return pred;
                        }
                        else if (predictor.cc[offset] < safe_limit){
                                return 0;
                        }
                #endif
                #ifdef sec_impl
                        #ifdef not_allways
                                if (predictor.cc[offset] >= 2){
                                        return pred;
                                }
                                else if (predictor.cc[offset] < 2){
                                        return 0;
                                }
                        #else
                                assert(0);
                        #endif
                #endif
        #else
                //printf("Return : %d\n", pred);
                                return pred;
        #endif
}
#else
        int i=0;
        for ( i=0; i < predictor.cur_entries; i++ )
        {

        if (predictor.valid[i] == 1){
//--------In case of a hit, update the entry---------------
        #ifdef nooffset
             if (predictor.pc[i] == pc)
        #else
             #ifdef onlyoffset
                 if (predictor.offset[i] == offset)
             #else
                 if ((predictor.pc[i] == pc) &&
                     (predictor.offset[i] == offset))
             #endif
        #endif
                 {

//printf("FOUND AT POSITION %d\n",i);
                    predictor.total_pc_match_during_predict++;
                    predictor.pc_match_during_predict[i] ++;
                    #ifdef pred_LRU
                        predictor.timestamp[i]=tstamp;	//FIFO - LRU
                    #endif

                    if (predictor.sb1[i] == 1) pred = pred + (int) pow(2,0);
                    if (predictor.sb2[i] == 1) pred = pred + (int) pow(2,1);

// what to return assuming 4 subblocks
// 0  cannot do prediction
// 1  sb4 0 sb3 0 sb2 0 sb1 1
// 2  sb4 0 sb3 0 sb2 1 sb1 0
// 3  sb4 0 sb3 0 sb2 1 sb1 1
// 4  sb4 0 sb3 1 sb2 0 sb1 0
// 5  sb4 0 sb3 1 sb2 0 sb1 1
// 6  sb4 0 sb3 1 sb2 1 sb1 0
// 7  sb4 0 sb3 1 sb2 1 sb1 1
// 8  sb4 1 sb3 0 sb2 0 sb1 0
// 9  sb4 1 sb3 0 sb2 0 sb1 1
// 10 sb4 1 sb3 0 sb2 1 sb1 0
// 11 sb4 1 sb3 0 sb2 1 sb1 1
// 12 sb4 1 sb3 1 sb2 0 sb1 0
// 13 sb4 1 sb3 1 sb2 0 sb1 1
// 14 sb4 1 sb3 1 sb2 1 sb1 0
// 15 sb4 1 sb3 1 sb2 1 sb1 1
#ifdef CONFIDENCE_CNT
        #ifdef init_impl
            if (predictor.cc[i] >= safe_limit){
        #endif
        #ifdef sec_impl
            if (predictor.cc[i] >= 2){
        #endif
#endif
        return pred;
        #ifdef CONFIDENCE_CNT
        }
                #ifdef init_impl
                        else if (predictor.cc[i] < safe_limit) 	return 0;
                #endif
                #ifdef sec_impl
                        else if (predictor.cc[i] < 2) 	return 0;
                #endif

        #endif
        }
}
}
#endif

return 0;

}


void predictor_update ( Addr pc, Addr offset,
                uint8_t sc1, uint8_t sc2, Tick tstamp )
{
        if (pc == 0) return;

        int bit_counter1 = 0,  bit_counter2 = 0;
        predictor.total_updates ++;
//      printf(">>>> Predictor Update: 0x%lx \t offset= 0x%lx \n", pc, offset);

#ifdef PRED_ADDR
        offset = offset & PRED_ADDR_MASK;

        if (PRED_ADDR_MASK == 0xff000000) {
            offset = offset >> 24;
        }
        else if (PRED_ADDR_MASK == 0xfc000000) {
            offset = offset >> 26;
        }
        else if (PRED_ADDR_MASK == 0xffc00000) {
            offset = offset >> 22;
        }
        else if (PRED_ADDR_MASK == 0xfff00000) {
            offset = offset >> 20;
        }
        else if (PRED_ADDR_MASK == 0xffff0000) {
            offset = offset >> 16;
        }
        else if (PRED_ADDR_MASK == 0x3fc0) {
            offset = offset >> 6;
        }
        else if (PRED_ADDR_MASK == 0x3fffc0) {
            offset = offset >> 6;
        }
        else if (PRED_ADDR_MASK == 0xfc0) {
            offset = offset >> 6;
        }
        else if (PRED_ADDR_MASK == 0xffc0) {
            offset = offset >> 6;
        }
        else if (PRED_ADDR_MASK == 0x3ffc0) {
            offset = offset >> 6;
        }
#endif

#ifdef PRED_PC
        pc = pc & PRED_PC_MASK;

        if (PRED_PC_MASK == 0xff000000) {
            pc = pc >> 24;
        }
        else if (PRED_PC_MASK == 0xfc000000) {
            pc = pc >> 26;
        }
        else if (PRED_PC_MASK == 0xffff0000) {
            pc = pc >> 16;
        }
#endif
//        printf("Predictor Update: 0x%lx \t offset: 0x%lx \n\n", pc, offset);
        if (sc1>0) bit_counter1=1;
        if (sc2>0) bit_counter2=1;

#ifdef NO_TAG_PRED
#ifdef PRED_PC
offset = pc;
#endif

#ifdef CONFIDENCE_CNT
        #ifdef sec_impl
        #ifndef not_allways
        assert(bit_counter1==0 || bit_counter1==1);
        assert(bit_counter2==0 || bit_counter2==1);

        //Update the valid flag
        predictor.valid[offset]=1;

        int suma= bit_counter1 + bit_counter2;
        
        if (predictor.cc[offset] == 3){
                if (suma==2) predictor.cc[offset] = 3;
                else if ((suma==1) || (suma==0)) {
                     predictor.cc[offset] = 2;
                }
                else assert(0);
        }
        else if (predictor.cc[offset] == 2){
                if (suma==2) predictor.cc[offset] = 3;
                else if ((suma==1) || (suma==0)) {
                      predictor.cc[offset] = 0;
                }
                else assert(0);
        }
        else if (predictor.cc[offset] == 1){
                if (suma==2) predictor.cc[offset] = 3;
                else if ((suma==1) || (suma==0)) {
                    predictor.cc[offset] = 0;
                }
                else assert(0);
        }
        else if (predictor.cc[offset] == 0){
                if (suma==2) predictor.cc[offset] = 1;
                else if ((suma==1) || (suma==0)) {
                    predictor.cc[offset] = 0;
                }
                else assert(0);
        }
        else {
            assert(0);
        }
        return;

        #endif
        #endif
#endif

int suma= bit_counter1 + bit_counter2;
int suma2 = predictor.sb1[offset] + predictor.sb2[offset];

if (predictor.valid[offset] == 1){

    if (suma == suma2)
    {
    #ifdef CONFIDENCE_CNT
      #ifdef init_impl
        if (predictor.cc[offset] < upper_limit) {
            predictor.cc[offset] = predictor.cc[offset] + 1;
        }
      #endif
      #ifdef sec_impl
         #ifdef not_allways
             if (predictor.cc[offset] == 2) predictor.cc[offset] = 3;
             else if (predictor.cc[offset] == 1) predictor.cc[offset] = 3;
             else if (predictor.cc[offset] == 0) predictor.cc[offset] = 1;
             else if (predictor.cc[offset] == 3) predictor.cc[offset] = 3;
         #else
            assert(0);
         #endif
      #endif
    #endif
    }
    else{
    //paw kai enimerwnw to entry pou mou deixnei to offset
    #ifndef CONFIDENCE_CNT
        predictor.valid[offset]=1;
        predictor.sb1[offset]=bit_counter1;
        predictor.sb2[offset]=bit_counter2;
    #else
      #ifdef init_impl
          if (predictor.cc[offset]>0){
              predictor.cc[offset] = predictor.cc[offset] - 1;
          }
       #endif
       #ifdef sec_impl
         #ifdef not_allways
            if (predictor.cc[offset] == 3) predictor.cc[offset] = 2;
            else if (predictor.cc[offset] == 2) predictor.cc[offset] = 0;
            else if (predictor.cc[offset] == 1) predictor.cc[offset] = 0;
            else if (predictor.cc[offset] == 0) predictor.cc[offset] = 0;
         #else
            assert(0);
         #endif
       #endif

       if (predictor.cc[offset] == 0) {
           predictor.valid[offset]=1;
           predictor.sb1[offset]=bit_counter1;
           predictor.sb2[offset]=bit_counter2;
       }
    #endif
    }
}
else {
    predictor.valid[offset]=1;
    predictor.sb1[offset]=bit_counter1;
    predictor.sb2[offset]=bit_counter2;
    predictor.cc[offset]=init_conf_value;
}

#else  //TAGLESSD:274

    int min,mini, i=0;
    int flag = 0;
    for ( i=0; i<predictor.cur_entries; i++ )
    {
    //--Update the info in case of hit--
    #ifdef nooffset
        if (predictor.pc[i] == pc)
    #else
        #ifdef onlyoffset
            if (predictor.offset[i] == offset)
        #else
            if ((predictor.pc[i] == pc) && (predictor.offset[i] == offset))
        #endif
    #endif
            {
                predictor.pc_match_during_update[i] ++ ;
                predictor.total_pc_match_during_update ++ ;

                flag = 1;
    #ifdef pred_LRU
                predictor.timestamp[i]=tstamp;	//FIFO - LRU
    #endif
    //Update the accuracy in case of correct prediction 
               if ((predictor.sb1[i] == bit_counter1) &&
                               (predictor.sb2[i] == bit_counter2))
               {
                   predictor.pc_correct[i]++;
                   predictor.total_pc_correct++;

                   #ifdef CONFIDENCE_CNT
                     #ifdef init_impl
                         if (predictor.cc[i] < upper_limit) {
                         predictor.cc[i] = predictor.cc[i] + 1;
                     }
                     #endif
                     #ifdef sec_impl
                         if (predictor.cc[i] == 2) predictor.cc[i] = 3;
                         else if (predictor.cc[i] == 1) predictor.cc[i] = 3;
                         else if (predictor.cc[i] == 0) predictor.cc[i] = 1;
                         else if (predictor.cc[i] == 3) predictor.cc[i] = 3;
                     #endif
                   #endif
               }
               else
               {
                #ifndef CONFIDENCE_CNT
                    predictor.sb1[i]=bit_counter1;
                    predictor.sb2[i]=bit_counter2;
                #else
                   #ifdef init_impl
                     if (predictor.cc[i]>0){
                         predictor.cc[i] = predictor.cc[i] - 1;
                     }
                   #endif
                   #ifdef sec_impl
                     if (predictor.cc[i] == 3) predictor.cc[i] = 2;
                     else if (predictor.cc[i] == 2) predictor.cc[i] = 0;
                     else if (predictor.cc[i] == 1) predictor.cc[i] = 0;
                     else if (predictor.cc[i] == 0) predictor.cc[i] = 0;
                   #endif
                   if (predictor.cc[i] == 0) {
                     predictor.sb1[i]=bit_counter1;
                     predictor.sb2[i]=bit_counter2;
                   }
                #endif
               }
            }
    }	
    //---PC not found, check if the table is full----
    if (flag==0) {
         //There is empty entry
         if (predictor.cur_entries<pred_entries){
             predictor.pc[predictor.cur_entries]=pc;
             predictor.offset[predictor.cur_entries]=offset;
             predictor.valid[predictor.cur_entries]=1;
             predictor.sb1[predictor.cur_entries]= bit_counter1;
             predictor.sb2[predictor.cur_entries]= bit_counter2;
             predictor.cc[predictor.cur_entries]= init_conf_value;
             predictor.timestamp[predictor.cur_entries]=tstamp;
             predictor.pc_match_during_update[predictor.cur_entries]=0;
             predictor.pc_correct[predictor.cur_entries]=0;
             predictor.cur_entries++;
         }
         else
         {//A replacement should be done, table is full 
             predictor.total_repl++;
             min = predictor.timestamp[0];
             mini = 0;
             for (i=0; i<pred_entries; i++ ){
                     if (predictor.timestamp[i]<min) {
                             min=predictor.timestamp[i];
                             mini=i;
                     }
             }

             predictor.pc[mini]=pc;
             predictor.offset[mini]=offset;
             predictor.valid[mini]=1;
             predictor.sb1[mini]=bit_counter1;
             predictor.sb2[mini]=bit_counter2;
             predictor.cc[mini]=init_conf_value;
             predictor.pc_match_during_update[mini]=0;
             predictor.pc_correct[mini]=0;
             predictor.timestamp[mini]=tstamp;
       }
    }
#endif
}

//==========end_predictor=========
}
