/**
 * POGOBOT
 *
 * Copyright © 2023 Sorbonne Université ISIR
 * This file is licensed under the Expat License, sometimes known as the MIT License.
 * Please refer to file LICENCE for details.
 *
 * Alessia Loi 2023-12 hanabi code.
 *  - Start-up phase for simultaneous start of the robots when the lights turn off: at start, the upper led is red
 *    and stays on until the environmental light is lower than LIGHT_THRESHOLD; then, the upper led switches to white
 *    for a period of BOOT_TIME seconds; after this BOOT_TIME period, hanabi starts
 *  - A robot has a state, consisting of an age and a color. The age is the number of color changes since initialization.
 *    Age value goes from 0 to max 65535 (16 bits unsigned). The robot broadcasts continuously its state to the neighbors.
 *  - A robot receives messages from neighbors, and elaborates the first MAX_NB_OF_MSG received. One message contains the sender state.
 *    If the receiver detects that the (neighbor) sender age is higher, then it copies its age and color.
 *  - A robot changes autonomously its state with a probability of 1/den_p_change_led_color.
 *  - A stagnation prevention mechanism tests if the age stays the same in a period of timeout_age_s seconds. This could happen
 *    if the maximum age value is exceeded, or if a robot doesn't receive messages from aged neighbors and doesn't change its state autonoumosly.
 *
 * Leo Cazenille 2025-01.
 *  - adapted the hanabi code to be used in the pogosim simulator.

 * This algorithm shows the rapidity of the diffusion of information among the swarm and by which path it propagates.
 * One robot changes autonomously its color and increments its age, this causes a cascade of changes affecting the robots with lower age.
 * If the probability to change the color autonomously (den_p_change_led_color) is too high, it is possible to get 2 distant robots autonomous changes
 * and observe partitions with groups of same age.
**/

//#include "pogobot.h"
#include "pogosim.h"
#include "time.h"

#define CODENAME "HANABI"

#define INFRARED_POWER 2 // 1,2,3

#define FQCY 60             // control update frequency. 30Hz | 60 Hz | 90 Hz | etc.
#define MAX_NB_OF_MSG 3     // max. number of messages per step which this robot can record // 3
#define PERCENT_MSG_SENT 50 // Percent of messages sent per tick

#define SEND_MODE_ALLDIRECTION true // true: all direction at once; false: 4x one-direction
#define MSG_MODE_FULL_HEADER true // true: full header; false: short header

#define DEBUG_LEVEL 0 // 0: nothing; 1: debug; 2: synchronzation; 3: communication

#define BOOT_TIME 5 // waiting time before the start of the experience in seconds
#define LIGHT_THRESHOLD 10

// Uncomment to have moving robots
//#define MOVING_ROBOTS


// ********************************************************************************
// * Initialization: Pogobot message structure
// ********************************************************************************

typedef struct RawMessage {
    uint16_t sender_id;
    uint16_t msg_id;
    uint16_t age;
    uint8_t rgb_colors_index;
} RawMessage;

#define MSG_SIZE sizeof(RawMessage) // number of bytes

typedef union message_template {
    uint8_t msg_array[MSG_SIZE];
    RawMessage msg_values;
} message;


// ********************************************************************************
// * Initialization: Pogobot led colors structure
// ********************************************************************************

typedef struct {
    char name[16];
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color;

rgb_color const red =         {.name = "red",        .r = 25, .g = 0,  .b = 0};
rgb_color const green =       {.name = "green",      .r = 0,  .g = 25, .b = 0};
rgb_color const blue =        {.name = "blue",       .r = 0,  .g = 0,  .b = 25};
rgb_color const magenta =     {.name = "magenta",    .r = 25, .g = 0,  .b = 25};
rgb_color const yellow =      {.name = "yellow",     .r = 12, .g = 6,  .b = 0};
rgb_color const cyan =        {.name = "cyan",       .r = 0,  .g = 25, .b = 25};
rgb_color const orange =      {.name = "orange",     .r = 25, .g = 6,  .b = 0};
rgb_color const purple =      {.name = "purple",     .r = 6,  .g = 0,  .b = 25};
rgb_color const light_pink =  {.name = "light_pink", .r = 12, .g = 3,  .b = 12};
rgb_color const mint_green =  {.name = "mint_green", .r = 6,  .g = 25, .b = 6};
rgb_color const white =       {.name = "white",      .r = 25, .g = 25, .b = 25};

void process_message(message_t* mr);
bool send_message(void);


// ********************************************************************************
// * main
// ********************************************************************************

rgb_color const rgb_colors[] = {red, green, blue, magenta, yellow, cyan, orange, purple, light_pink, mint_green}; // 10 hanabi colors
uint8_t const nb_rgb_colors = sizeof(rgb_colors) / sizeof(rgb_colors[0]);
uint16_t const den_p_change_led_color = 20000; // probability to change led color independently (1/den_p_change_led_color) (p=3000 for 15 robots; p=20000 for 72 robots)


typedef struct {
    // XXX remove and put into lib
    uint64_t start_of_experiment_ms;
    // XXX remove and put into lib
    bool started;

    uint16_t my_pogobot_id;
    uint16_t age; // number of times this robot has changed color from the start of the algorithm, from 0 to max 65535 (16 bits unsigned)
    //uint8_t led1_status;

    uint8_t rgb_colors_index; // index of the rgb_colors array
} USERDATA;
REGISTER_USERDATA(USERDATA)


void process_message(message_t* mr) {
    // XXX remove and put into lib
    // Elaborate robot messages only (avoid controllers messages). NB: this condition works only with long headers
    if (MSG_MODE_FULL_HEADER && mr->header._packet_type != ir_t_user) {
        printf("[I'm Pogobot %d] [RECV] This message is discarded because it didn't come from a Pogobot\n", mydata->my_pogobot_id);
        return;
    }

    // Read the received message and stock it in the msg_from_neighbor structure
    message msg_from_neighbor;
    for (uint16_t i = 0; i != MSG_SIZE; i++)
        msg_from_neighbor.msg_array[i] = mr->payload[i];

    if (msg_from_neighbor.msg_values.age > mydata->age && mydata->rgb_colors_index != msg_from_neighbor.msg_values.rgb_colors_index &&
            mydata->rgb_colors_index < nb_rgb_colors && (msg_from_neighbor.msg_values.age - mydata->age) < 10000) {
        mydata->age = msg_from_neighbor.msg_values.age; // update to the greatest age in the neighborhood
        // pogobot_stopwatch_reset(&timeout_age_watch); // reset of the timer, for age timeout
        if (DEBUG_LEVEL == 1)
            printf("[I'm Pogobot %d] [HANABI] Copying state from neighbor: passing from old color %s to new color %s, copying age to %d  *____*\n",
                    mydata->my_pogobot_id, rgb_colors[mydata->rgb_colors_index].name, rgb_colors[msg_from_neighbor.msg_values.rgb_colors_index].name, mydata->age);
        mydata->rgb_colors_index = msg_from_neighbor.msg_values.rgb_colors_index;
        pogobot_led_setColor(rgb_colors[mydata->rgb_colors_index].r, rgb_colors[mydata->rgb_colors_index].g, rgb_colors[mydata->rgb_colors_index].b);

    } else {
        if (DEBUG_LEVEL == 1)
            printf("[I'm Pogobot %d] [HANABI] NOT Changing color, not incrementing age %d  ç____ç\n", mydata->my_pogobot_id, mydata->age);
    }
}


// ********************************************************************************
// * Send message
// * Each robot transmits 2 information to the neighbors: a color and an iterator.
// * The iterator (age) corresponds to how many times the sender has changed color.
// ********************************************************************************
bool send_message(void) {
    uint16_t msg_id;
    uint8_t data[MSG_SIZE]; // message to send, containing uint8_t data
    message msg_from_neighbor;

    msg_id = nb_msgs_sent;

    // Composing a new message to send
    msg_from_neighbor.msg_values.sender_id = mydata->my_pogobot_id;
    msg_from_neighbor.msg_values.msg_id = msg_id;
    msg_from_neighbor.msg_values.age = mydata->age;
    msg_from_neighbor.msg_values.rgb_colors_index = mydata->rgb_colors_index; // value from 0 to nb_rgb_colors-1

    // Convert the message into an uint8_t pointer
    for ( uint16_t i = 0; i != MSG_SIZE; i++ )
        data[i] = msg_from_neighbor.msg_array[i];

    if (SEND_MODE_ALLDIRECTION == true) {
        if (MSG_MODE_FULL_HEADER == true)
            pogobot_infrared_sendLongMessage_omniGen((uint8_t *)(data), MSG_SIZE);
        else
            pogobot_infrared_sendShortMessage_omni((uint8_t *)(data), MSG_SIZE);
    } else {
        for (uint16_t i  = 0; i < 4 ; i++) // i is one of the 4 possible directions
        {
            if (MSG_MODE_FULL_HEADER == true)
                pogobot_infrared_sendLongMessage_uniSpe(i, (uint8_t *)(data), MSG_SIZE);
            else
                pogobot_infrared_sendShortMessage_uni(i, (uint8_t *)(data), MSG_SIZE);
        }
    }

    if (DEBUG_LEVEL == 3)
        printf("[I'm Pogobot %d] [SEND] Sent message msg_id=%d.\n", mydata->my_pogobot_id, msg_id);

    return true;
}


void user_init(void) {
    srand(pogobot_helper_getRandSeed()); // initialize the random number generator
    pogobot_infrared_set_power(INFRARED_POWER); // set the power level used to send all the next messages

    // Set mydata variables to 0
    uint8_t* mydata_ptr = (uint8_t *) mydata;
    for (size_t i = 0; i < sizeof(*mydata); i++) {
        mydata_ptr[i] = 0;
    }

//    // Init timer
//    pogobot_stopwatch_reset(&mydata->timer_it);

    // Set main loop frequency, message sending frequency, message processing frequency
    main_loop_hz = FQCY;
    max_nb_processed_msg_per_tick = MAX_NB_OF_MSG;
    percent_msgs_sent_per_ticks = PERCENT_MSG_SENT;
    // Specify functions to send/transmit messages
    msg_rx_fn = process_message;
    msg_tx_fn = send_message;

    // Set led index to show error codes
    error_codes_led_idx = 3; // Default value, negative values to disable

    mydata->my_pogobot_id = pogobot_helper_getid();

    if (DEBUG_LEVEL) {
        printf("\n");
        printf("[INFO] =-=-=-=-=-=-=-=-=-=-=-=-=-=\n");
        printf("[INFO] =-=- POGOBOT::METADATA -=-=\n");
        printf("[INFO] =-=-=-=-=-=-=-=-=-=-=-=-=-=\n");
        printf("[INFO] CODENAME : %s\n",CODENAME);
        printf("[INFO] INFRARED_POWER         %d\n",INFRARED_POWER);
        printf("[INFO] FQCY                   %d\n",FQCY);
        //printf("[INFO] MAX_NB_OF_MSG          %d\n",MAX_NB_OF_MSG);
        printf("[INFO] SEND_MODE_ALLDIRECTION %d\n",SEND_MODE_ALLDIRECTION);
        printf("[INFO] MSG_MODE_FULL_HEADER   %d\n",MSG_MODE_FULL_HEADER);
        printf("[INFO] DEBUG_LEVEL            %d\n",DEBUG_LEVEL);
        printf("[INFO] =-=-=-=-=-=-=-=-=-=-=-=-=-=\n");
    }

    if (DEBUG_LEVEL == 1 && den_p_change_led_color > RAND_MAX)
        printf("Warning: the maximal value allowed for den_p_change_led_color is %d\n", RAND_MAX);
}


void user_step(void) {
    // XXX remove and put into lib
    // ********************************************************************************
    // * Start-up phase for simultaneous start of the robots when the lights turn off
    // ********************************************************************************
    if (!mydata->started) {
        pogobot_led_setColor(red.r, red.g, red.b); // set initial led color to red

        int16_t photo0 = pogobot_photosensors_read(0);
        int16_t photo1 = pogobot_photosensors_read(1);
        int16_t photo2 = pogobot_photosensors_read(2);

        if (photo0 < LIGHT_THRESHOLD || photo1 < LIGHT_THRESHOLD || photo2 < LIGHT_THRESHOLD) {
            mydata->started = true;
            mydata->start_of_experiment_ms = current_time_milliseconds();
        } else {
            return; // Quit function if experiment has not started
        }
    }

    // XXX remove and put into lib
    // Experiment has started. Wait for some time
    if (current_time_milliseconds() - mydata->start_of_experiment_ms < BOOT_TIME * 1000) {
        pogobot_led_setColor(white.r, white.g, white.b); // set boot led color to white
        return; // Wait
    }

    // ********************************************************************************
    // * Main loop
    // ********************************************************************************
    // A robot changes autonomously its state with a probability of 1/den_p_change_led_color
    if (rand() % den_p_change_led_color <= 1) {
        uint8_t rgb_colors_choice = rand()%nb_rgb_colors;  // random index to choose the next led color
        if (rgb_colors_choice != mydata->rgb_colors_index) {
            mydata->age++;
            // pogobot_stopwatch_reset(&timeout_age_watch); // reset of the timer, for age timeout

            if (DEBUG_LEVEL == 1)
                printf("[I'm Pogobot %d] [HANABI] Changing color from old color %s to new color %s because I'm lucky, age %d  *____*\n",
                        mydata->my_pogobot_id, rgb_colors[mydata->rgb_colors_index].name, rgb_colors[rgb_colors_choice].name, mydata->age);

            mydata->rgb_colors_index = rgb_colors_choice;
            pogobot_led_setColor(rgb_colors[mydata->rgb_colors_index].r, rgb_colors[mydata->rgb_colors_index].g, rgb_colors[mydata->rgb_colors_index].b);
        } else {
            if (DEBUG_LEVEL == 1)
                printf("[I'm Pogobot %d] [HANABI] NOT Changing color because old color %s = new color %s, age %d  ç____ç\n",
                        mydata->my_pogobot_id, rgb_colors[mydata->rgb_colors_index].name, rgb_colors[rgb_colors_choice].name, mydata->age);
        }
    }

    // ********************************************************************************
    // * Motility
    // ********************************************************************************

#ifdef MOVING_ROBOTS
    if ((uint32_t)(current_time_milliseconds() / 10000) % 2 == 0) {
        pogobot_led_setColors(blue.r, blue.g, blue.b, 1);
        pogobot_motor_set(motorL, motorFull);
        pogobot_motor_set(motorR, motorStop);
    } else {
        pogobot_led_setColors(red.r, red.g, red.b, 1);
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorFull);
    }
#else
    // * Robots do not move in this experience.
    // ... NOTHING TO DO ...
#endif

}

int main(void) {
    pogobot_init();

    pogobot_start(user_init, user_step);
    return 0;
}

