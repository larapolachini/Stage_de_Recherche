
#ifndef SSR_H_
#define SSR_H_

/* --- navigation FSMs -------------------------------------------------- */
typedef enum { PHASE_RUN, PHASE_TUMBLE }          PhaseState;
typedef enum { RT_PHASE_RUN, RT_PHASE_TUMBLE }    RTPhaseState;
/* ---------------------------------------------------------------------- */


////////////////// BASE MACROS ////////////////// {{{1

//#define DISABLE_MOTION

#define MAXN 20
#define PERCENT_MSG_SENT 35
#define MAIN_LOOP_HZ 30 //120
#define MAX_NB_MSGS_PROCESSED_PER_TICK MAXN
//#define DISABLE_PRINTF_ON_ROBOTS

#define ENABLE_FINAL_LAMBDA
//#define ENABLE_INIT_REMOVE_SUM_S
#define ENABLE_PRE_DIFFUSION
//#define ENABLE_HANDSHAKES
//#define ENABLE_TAU_INCREASE
#define ENABLE_SYNC_TIME
//#define ENABLE_ALWAYS_MOVING
#define ENABLE_STOP_DIFFUSION_AFTER_THRESHOLD

#define ENABLE_PHOTO_START
#define LIGHT_THRESHOLD 40  // You can tweak this parameter to change the sensitivity to the light changes

//#define ENABLE_SINGLE_DIFF
//#define ENABLE_DOUBLE_DIFF
#define ENABLE_TRIPLE_DIFF
//#define ENABLE_HEXA_DIFF

//#define ENABLE_COLOR_FROM_S0
//#define ENABLE_COLOR_FROM_S
//#define ENABLE_COLOR_FROM_SIGNS
#define ENABLE_COLOR_SIGNS_AND_CONSENSUSLAMBDA
//#define ENABLE_COLOR_NB_NEIGHBOURS

//#define SIMULATE_FAKE_COMM
//#define ENABLE_DEBUG_MSG
//#define SHOW_CONSENSUSLAMBDA_IN_LED1
//#define ENABLE_SHOW_BEHAVIOR_COLOR

//#define DIFFUSION_WINDOW_SIZE 60
#define DIFFUSION_WINDOW_SIZE 40
//#define DIFFUSION_WINDOW_SIZE 30
//#define DIFFUSION_WINDOW_SIZE 25
//#define DIFFUSION_WINDOW_SIZE 20 // Original value
//#define DIFFUSION_WINDOW_SIZE 10
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


////////////////// DATA TYPES ////////////////// {{{1
#include "pogo-utils/fixp.h"

#define MAIN_DATA_TYPE_FLOAT
//#define MAIN_DATA_TYPE_DOUBLE
//#define MAIN_DATA_TYPE_Q8_24
//#define MAIN_DATA_TYPE_Q16_16

#if defined(MAIN_DATA_TYPE_FLOAT)
typedef float fp_t;
#define ABS(x) fabsf(x)
#define LOG(x) logf(x)
#define EXP(x) expf(x)
#define ADD(x, y)  (x + y)
#define SUB(x, y)  (x - y)
#define MUL(x, y)  (x * y)
#define DIV(x, y)  (x / y)
#define FROM_FLOAT(x)  ((float)x)
#define FROM_INT(x)  ((float)x)
#define FROM_Q6_10(x)   ((float)q6_10_to_float(x))
#define TO_FLOAT(x)   ((float)x)
#define TO_Q16_16(x)   (q16_16_from_float(x))
#define TO_Q6_10(x)   (q6_10_from_float(x))
#define NOT_A_NUMBER    (0.f/0.f)
#define IS_NUMBER_VALID(nb)  (!isnan(nb) && !isinf(nb))

#elif defined(MAIN_DATA_TYPE_DOUBLE)
typedef double fp_t;
#define ABS(x) fabs(x)
#define LOG(x) log(x)
#define EXP(x) exp(x)
#define ADD(x, y)  (x + y)
#define SUB(x, y)  (x - y)
#define MUL(x, y)  (x * y)
#define DIV(x, y)  (x / y)
#define FROM_FLOAT(x)  ((double)x)
#define FROM_INT(x)  ((double)x)
#define FROM_Q6_10(x)   ((double)q6_10_to_double(x))
#define TO_FLOAT(x)   ((float)x)
#define TO_Q16_16(x)   (q16_16_from_float(x))
#define TO_Q6_10(x)   (q6_10_from_float(x))
#define NOT_A_NUMBER    (0./0.)
#define IS_NUMBER_VALID(nb)  (!isnan(nb) && !isinf(nb))

#elif defined(MAIN_DATA_TYPE_Q8_24)
typedef q8_24_t fp_t;
#define ABS(x)      q8_24_abs(x)
#define LOG(x)      q8_24_log(x)
#define EXP(x)      q8_24_exp(x)
#define ADD(x, y)   q8_24_add(x, y)
#define SUB(x, y)   q8_24_sub(x, y)
#define MUL(x, y)   q8_24_mul(x, y)
#define DIV(x, y)   q8_24_div(x, y)
#define FROM_FLOAT(x)   q8_24_from_float(x)
#define FROM_INT(x)     q8_24_from_int(x)
#define FROM_Q6_10(x)   (q8_24_from_q6_10(x))
#define TO_FLOAT(x)     q8_24_to_float(x)
#define TO_Q16_16(x)    (q16_16_from_q8_24(x))
#define TO_Q6_10(x)   (q6_10_from_q8_24(x))
#define NOT_A_NUMBER    (Q8_24_MAX)
#define IS_NUMBER_VALID(nb)  (!q8_24_is_saturated(nb))

#elif defined(MAIN_DATA_TYPE_Q16_16)
typedef q16_16_t fp_t;
#define ABS(x)      q16_16_abs(x)
#define LOG(x)      q16_16_log(x)
#define EXP(x)      q16_16_exp(x)
#define ADD(x, y)   q16_16_add(x, y)
#define SUB(x, y)   q16_16_sub(x, y)
#define MUL(x, y)   q16_16_mul(x, y)
#define DIV(x, y)   q16_16_div(x, y)
#define FROM_FLOAT(x)   q16_16_from_float(x)
#define FROM_INT(x)     q16_16_from_int(x)
#define FROM_Q6_10(x)   (q16_16_from_q6_10(x))
#define TO_FLOAT(x)     q16_16_to_float(x)
#define TO_Q16_16(x)    (x)
#define TO_Q6_10(x)   (q6_10_from_q16_16(x))
#define NOT_A_NUMBER    (Q16_16_MAX)
#define IS_NUMBER_VALID(nb)  (!q16_16_is_saturated(nb))

#endif


////////////////// BASE STRUCTS AND GLOBALS ////////////////// {{{1

#include "pogobase.h"

extern fp_t initial_s_max_val;

#ifdef SIMULATOR
//#define printf0(fmt, ...) if (pogobot_helper_getid() == 0) { printf(fmt, ##__VA_ARGS__ ); }
//#define printf_fixp0(fmt, ...) if (pogobot_helper_getid() == 0) { printf_fixp(fmt, ##__VA_ARGS__ ); }
#define printf0(fmt, ...) if (pogobot_helper_getid() == 0) { printf_fixp(fmt, ##__VA_ARGS__ ); }
#else
////#define POGOBOT_PRINTF_ID  65535 //38191
////#define printf0(fmt, ...) if (pogobot_helper_getid() == POGOBOT_PRINTF_ID) { printf(fmt, ##__VA_ARGS__ ); }
//#define printf0(fmt, ...) printf(fmt, ##__VA_ARGS__ );
//#define printf_fixp0(fmt, ...) printf_fixp(fmt, ##__VA_ARGS__ );
#ifdef DISABLE_PRINTF_ON_ROBOTS
#define printf0(fmt, ...) do { } while(0)
#define printf(fmt, ...)  do { } while(0)
#else
#define printf0(fmt, ...) printf_fixp(fmt, ##__VA_ARGS__ );
#endif
#endif

#ifdef ENABLE_DEBUG_MSG
#define printf_debug(fmt, ...)      do { printf(fmt, ##__VA_ARGS__ ); } while(0)
#define printf0_debug(fmt, ...)     do { printf0(fmt, ##__VA_ARGS__ ); } while(0)
#else
#define printf_debug(fmt, ...)      do { } while(0)
#define printf0_debug(fmt, ...)     do { } while(0)
#endif



// Behaviors
typedef enum {
    INIT_BEHAVIOR,
    INIT_RUN_TUMBLE,
    RUN_TUMBLE,
#ifdef ENABLE_HANDSHAKES
    HANDSHAKE,
#endif
    PRE_DIFFUSION,
    DIFFUSION,
    CONSENSUS_LAMBDA,
#ifdef ENABLE_FINAL_LAMBDA
    FINAL_LAMBDA,
#endif
    WAITING_TIME,
    MISC_BEHAVIOR,
} behavior_t;


// declare variables

typedef enum __attribute__((__packed__)) {
    DATA_NULL = 5,
#ifdef ENABLE_HANDSHAKES
    DATA_HANDSHAKE,
#endif
    DATA_PRE_S,
    DATA_S,
    DATA_LAMBDA,
    DATA_CONSENSUS_LAMBDA,
} data_type_t;

#pragma pack(1)                                      // These two lines are needed to ensure 
typedef struct __attribute__((__packed__)) {        //   that all variable follow the same order as defined in the code
    data_type_t data_type;
    uint16_t sender_id;
    q6_10_t val[NUMBER_DIFF];
#ifdef ENABLE_SYNC_TIME
    uint32_t time;
#endif
} message_data_t;

#ifdef ENABLE_HANDSHAKES
#pragma pack(1)                                    // These two lines are needed to ensure 
typedef struct __attribute__((__packed__)) {       //   that all variable follow the same order as defined in the code
    data_type_t data_type;
    uint16_t sender_id;
    uint16_t peers[MAXN];
    uint8_t nb_peers;
#ifdef ENABLE_SYNC_TIME
    uint32_t time;
#endif
} handshake_message_data_t;
#endif

typedef struct {
    uint16_t id;
    uint32_t timestamp;

    data_type_t data_type;
    fp_t val[NUMBER_DIFF];
#ifdef ENABLE_SYNC_TIME
    uint32_t time;
#endif
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
    uint8_t ls_nb_points[NUMBER_DIFF];

    fp_t hist_logs[NUMBER_DIFF][DIFFUSION_WINDOW_SIZE];
    fp_t hist_t[NUMBER_DIFF][DIFFUSION_WINDOW_SIZE];
    fp_t best_mse[NUMBER_DIFF];
    fp_t hist_mse[NUMBER_DIFF][DIFFUSION_WINDOW_SIZE];

    uint16_t current_avg_it;
    bool diffusion_valid;
    bool stopped_diffusion[NUMBER_DIFF];

    uint16_t current_diffusion_it;
    uint32_t time_last_diff_it;

#ifdef ENABLE_TAU_INCREASE
    fp_t tau;
#endif
} diffusion_session_t;

typedef struct {
    neighbor_t neighbors[MAXN];
    uint8_t nb_neighbors;

#ifdef ENABLE_HANDSHAKES
    uint16_t known_neighbors_uid[MAXN];
    uint8_t nb_known_neighbors;
    uint8_t current_peer_index;
    handshake_message_data_t hmsg_to_send;
#endif

    uint32_t neighbors_age_max;
    bool enable_message_sending;
    message_data_t data_to_send;

    diffusion_session_t diff1;
    diffusion_session_t* curr_diff;

    uint32_t time_last_it;
    uint32_t time_last_coll_avg_lambda_it;
#ifdef ENABLE_FINAL_LAMBDA
    uint32_t time_last_coll_final_lambda_it;
#endif

    uint32_t behavior_start_ms;
    uint32_t ms_start_it;
    behavior_t current_behavior;
    uint16_t current_it;

    // Dispersion
    uint32_t cycle_dispersion;
    uint32_t last_pogoticks_dispersion;
    uint32_t tumble_time;
    uint32_t run_time;
    uint8_t direction;
    float prob;
    uint8_t flag_dispersion;
    float d_min;
    float d_max;
    float frustration;

    /* --- locomotion & run-and-tumble state ------------------------ */
    time_reference_t timer_it;     /* stopwatch used in setup()           */
    PhaseState       phase;        /* high-level nav phase                */
    uint32_t         phase_start_time;
    uint32_t         phase_duration;
    uint8_t          tumble_direction;

    uint8_t          motor_dir_left;   /* factory-calibrated directions    */
    uint8_t          motor_dir_right;

    RTPhaseState     rt_phase;         /* sub-FSM for run-and-tumble       */
    uint32_t         rt_phase_start;
    uint32_t         rt_phase_duration;
    uint8_t          rt_tumble_dir;

    uint8_t          data_foo[8];      /* demo array still used in code    */

    // Photo start values;
#ifdef ENABLE_PHOTO_START
    int16_t last_data_b;
    int16_t last_data_fl;
    int16_t last_data_fr;
#endif

} USERDATA;

DECLARE_USERDATA(USERDATA);


void compute_next_s(void);
fp_t compute_MSE(uint8_t i);
void compute_lambda_v_leastsquaresMSE(void);

void set_behavior(behavior_t behavior);
void behav_run_tumble(void);
void init_diffusion(diffusion_session_t* diff, fp_t* s, diffusion_type_t type);
void setup_diff(diffusion_session_t* diff);
void behav_diffusion(void);
void end_diffusion(void);
void init_coll_avg_lambda(void);
void behav_coll_avg_lambda(void);
void end_coll_avg_lambda(void);
void init_coll_final_lambda(void);
void behav_coll_final_lambda(void);
#ifdef ENABLE_HANDSHAKES
void behav_handshake(void);
void clear_known_neighbors(void);
bool is_neighbor_known(uint16_t uid);
#endif

void init_transitions(void);

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
