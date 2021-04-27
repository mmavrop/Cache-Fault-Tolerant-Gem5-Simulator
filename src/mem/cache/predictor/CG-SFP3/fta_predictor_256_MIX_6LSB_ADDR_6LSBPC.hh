#include "base/types.hh"

extern "C" {


#define pred_ordering    	//ifdef order doesn't matter
#define pred_entries 256
//#define pred_mask_bits 26
//#define pred_mask_bits_ms 4
// #define pred_high_order  // pred_low_order 	//number of pc masked bits
//replacement policy of the predictor - if notdef policy is FIFO/
#define pred_LRU  // pred_FIFO
//#define LOWMSB
//#define nooffset
//#define onlyoffset
//#define more_addr 6
//#define NO_TAG_PRED
#define init_impl
// #define sec_impl
// #define not_allways
#define init_conf_value 0

//#define LSB6 0x3f
//#define LSB8 0xff
//#define MSB6 0xfc000000
//#define LSB8 0xff000000
#define PRED_ADDR
#define PRED_ADDR_MASK 0x3f
#define PRED_PC_MASK 0x3f
#define PRED_PC

typedef struct{

//----------------TRUE HARWARE VARIABLES------------------//
        unsigned long long int offset[pred_entries];
        unsigned long long int pc[pred_entries];
        int sb1[pred_entries], sb2[pred_entries];
        int valid[pred_entries];
        int cc[pred_entries];
        unsigned long long int timestamp[pred_entries];
//----------------H/W VARIABLES FOR STATS------------------//
        //times the pc was accessed during tenure in the predictor
        int pc_match_during_update[pred_entries];
        //times the pc was accessed during tenure in the predictor
        int pc_match_during_predict[pred_entries];
        //right predictions of the pc during tenure
        int pc_correct[pred_entries];
        int cur_entries;
        int total_pc, total_updates, total_pc_match_during_update;
        int total_pc_match_during_predict, total_pc_correct, total_repl;
} ftaPredictor;

ftaPredictor* create_fta(void);
void predictor_init () ;
int predictor_predict ( Addr pc, Addr offset, Tick tstamp )  ;
void predictor_update ( Addr pc, Addr offset,
                uint8_t sc1, uint8_t sc2, Tick tstamp ) ;
}
