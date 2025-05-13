
#ifndef LIMMSSWARM_H_
#define LIMMSSWARM_H_

//#define DISABLE_MOTION

#define MAXN 20 // 20
#define PERCENT_MSG_SENT 50
#define MAIN_LOOP_HZ 60
#define MAX_NB_MSGS_PROCESSED_PER_TICK 100

#define ENABLE_AVG_AVG_LAMBDA
//#define ENABLE_INIT_REMOVE_SUM_S
#define ENABLE_PRE_DIFFUSION

//#define ENABLE_INITIAL_WAIT_FOR_NEIGHBOR
#define ENABLE_PHOTO_START
#define LIGHT_THRESHOLD 40  // You can tweak this parameter to change the sensitivity to the light changes

//#define ENABLE_SINGLE_DIFF
//#define ENABLE_DOUBLE_DIFF
#define ENABLE_TRIPLE_DIFF
//#define ENABLE_HEXA_DIFF

//#define ENABLE_COLOR_FROM_S0
//#define ENABLE_COLOR_FROM_S
//#define ENABLE_COLOR_FROM_SIGNS
//#define ENABLE_COLOR_FROM_LAMBDA
//#define ENABLE_COLOR_FROM_AVGLAMBDA
#define ENABLE_COLOR_SIGNS_AND_AVGLAMBDA
//#define ENABLE_COLOR_SIGNS_AND_AVGLAMBDA_ALL_DIFF
//#define ENABLE_COLOR_NB_NEIGHBOURS

//#define ENABLE_DEBUG_MSG
#define SHOW_AVGLAMBDA_IN_LED1
//#define ENABLE_SHOW_BEHAVIOR_COLOR

#define DIFFUSION_WINDOW_SIZE 40
//#define DIFFUSION_WINDOW_SIZE 20
//#define DIFFUSION_WINDOW_SIZE 3

#if defined(ENABLE_SINGLE_DIFF)
#define NUMBER_DIFF 1
#elif defined(ENABLE_DOUBLE_DIFF)
#define NUMBER_DIFF 2
#elif defined(ENABLE_TRIPLE_DIFF)
#define NUMBER_DIFF 3
#elif defined(ENABLE_HEXA_DIFF)
#define NUMBER_DIFF 6
#endif

typedef float fp_t;
#define ABS(x) fabsf(x)
#define LOG(x) logf(x)
#define EXP(x) expf(x)

#include "pogobase.h"

extern fp_t initial_s_max_val;

#ifdef SIMULATOR
#define printf0(fmt, ...) if (pogobot_helper_getid() == 0) { printf(fmt, ##__VA_ARGS__ ); }
#else
//#define POGOBOT_PRINTF_ID  65535 //38191
//#define printf0(fmt, ...) if (pogobot_helper_getid() == POGOBOT_PRINTF_ID) { printf(fmt, ##__VA_ARGS__ ); }
#define printf0(fmt, ...) printf(fmt, ##__VA_ARGS__ );
#endif



// Behaviors
typedef enum {
    INIT_BEHAVIOR,
    INIT_RANDOM_WALK,
    RANDOM_WALK,
    PRE_DIFFUSION,
    DIFFUSION,
    AVG_LAMBDA,
#ifdef ENABLE_AVG_AVG_LAMBDA
    AVG_AVG_LAMBDA,
#endif
    WAITING_TIME,
    MISC_BEHAVIOR,
} behavior_t;


// declare variables

typedef enum {
    DATA_NULL = 5,
#ifdef ENABLE_INITIAL_WAIT_FOR_NEIGHBOR
    DATA_HEARTBEAT,
#endif
    DATA_PRE_S,
    DATA_S,
    DATA_LAMBDA,
    DATA_AVG_LAMBDA,
} data_type_t;

#pragma pack(1)                                      // These two lines are needed to ensure 
typedef struct __attribute__((__packed__)) {        //   that all variable follow the same order as defined in the code
    data_type_t data_type;
    fp_t val[NUMBER_DIFF];
} message_data_t;

typedef struct {
    uint16_t id;
    uint64_t timestamp;

    data_type_t data_type;
    fp_t val[NUMBER_DIFF];
} neighbor_t;



typedef enum {
    NORMAL_DIFFUSION_TYPE = 0,
    PRE_DIFFUSION_TYPE,
} diffusion_type_t;


typedef struct {
    diffusion_type_t type;

    int8_t next_diff_to_compute;

    fp_t t;
    fp_t s[NUMBER_DIFF];
    fp_t s0[NUMBER_DIFF];

#if defined(ENABLE_INIT_REMOVE_SUM_S)
    fp_t sum_s0[NUMBER_DIFF];
#endif

    fp_t lambda;
    fp_t lambda_[NUMBER_DIFF];
    fp_t avg_lambda;
    fp_t sum_lambda;

    fp_t sum_t[NUMBER_DIFF];
    fp_t sum_t2[NUMBER_DIFF];
    fp_t sum_logs[NUMBER_DIFF];
    fp_t sum_tlogs[NUMBER_DIFF];
    fp_t ls_nb_points[NUMBER_DIFF];

    fp_t hist_logs[NUMBER_DIFF][DIFFUSION_WINDOW_SIZE];
    fp_t hist_t[NUMBER_DIFF][DIFFUSION_WINDOW_SIZE];
    fp_t best_mse[NUMBER_DIFF];
    fp_t hist_mse[NUMBER_DIFF][DIFFUSION_WINDOW_SIZE];

    uint16_t current_avg_it;
    bool diffusion_valid;
    bool stopped_diffusion[NUMBER_DIFF];

    uint16_t current_diffusion_it;
    uint64_t time_last_diff_it;
} diffusion_session_t;

typedef struct {
    uint32_t µs_iteration; // Set in ``setup()``

    neighbor_t neighbors[MAXN];
    uint8_t nb_neighbors;

    uint32_t neighbors_age_max;
    bool enable_message_sending;
    message_data_t data_to_send;

    diffusion_session_t diff1;
    diffusion_session_t* curr_diff;

    uint32_t time_last_it;
    uint32_t time_last_coll_avg_lambda_it;
#ifdef ENABLE_AVG_AVG_LAMBDA
    uint32_t time_last_coll_avg_avg_lambda_it;
#endif

    uint64_t behavior_start_µs;
    uint64_t µs_start_it;
    behavior_t current_behavior;
    uint16_t current_it;

    // Photo start values;
#ifdef ENABLE_PHOTO_START
    int16_t last_data_b;
    int16_t last_data_fl;
    int16_t last_data_fr;
#endif

} USERDATA;

// Call this macro in the same file (.h or .c) as the declaration of USERDATA
DECLARE_USERDATA(USERDATA);


void compute_next_s(void);
fp_t compute_MSE(uint8_t i);
void compute_lambda_v_leastsquaresMSE(void);

void set_behavior(behavior_t behavior);
void behav_random_walk(void);
void init_diffusion(diffusion_session_t* diff, fp_t* s, diffusion_type_t type);
void setup_diff(diffusion_session_t* diff);
void behav_diffusion(void);
void end_diffusion(void);
void init_coll_avg_lambda(void);
void behav_coll_avg_lambda(void);
void end_coll_avg_lambda(void);
void init_coll_avg_avg_lambda(void);
void behav_coll_avg_avg_lambda(void);

void iteration(void);
void end_iteration(void);

void purge_old_neighbors(void);
void clear_all_neighbors(void);
bool send_message(void);
void process_message(message_t* mr);

void setup(void);
void loop(void);

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
