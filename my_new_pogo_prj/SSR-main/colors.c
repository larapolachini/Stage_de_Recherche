/* TODO
 */

#include "colors.h"
#include "math.h"


#define NRAINBOWCOLORS (sizeof(colors) / sizeof((colors)[0]))

// Double rainbow colors (all the way !)
rgb8_t const colors[] = {
    RGB(3,0,0),  //0 - red
    RGB(3,3,0),  //1 - yellow
    RGB(0,3,0),  //2 - green
    RGB(0,3,3),  //3 - cyan
    RGB(0,0,3),  //4 - blue

    RGB(2,1,0),  //5 - orange
    RGB(1,1,1),  //6 - grey

    RGB(2,0,0),  //7 - red
    RGB(2,1,0),  //8 - yellow
    RGB(0,2,0),  //9 - green
    RGB(0,2,1),  //10 - green
    RGB(0,0,1),  //11 - blue
};


#define NCLASSES (sizeof(class_centroids) / sizeof((class_centroids)[0]))

//// Config final_triplediff-MSE-HANDSHAKE-diff54250-burnin10000-msg03-ag25-fop85-it248-tau50-expe5000-50runs_expeIC_danda.yaml
//fp_t class_centroids[10] = {
//    0.0001f,
//    0.01f,
//    0.40343048418760297f,
//    0.6576639905929565f,
//
//    -1000.0,
//    -1000.0,
//    -1000.0,
//    -1000.0,
//    -1000.0,
//    -1000.0,
//};

// Config conf/ssr.yaml bb3a71855408175f189b0c5bf44efedbcd193e56
fp_t class_centroids[10] = {
    0.0001f,
    0.01f,
    0.6482396937684235f,
    0.991076648235321f,

    -1000.0,
    -1000.0,
    -1000.0,
    -1000.0,
    -1000.0,
    -1000.0,
};

// Cf dict 'colors_dict' in scripts/plots.py.
// To generate Pogobot colors:
//   colors_k = {k: np.around(np.array(matplotlib.colors.to_rgb(v)) * 3.).astype(int) for k,v in plots.colors_dict.items()}
rgb8_t const class_colors[] = {
    RGB(0,3,0), // green (= error!)
    RGB(2,0,2), // purple - annulus
    RGB(2,0,2), // purple - annulus
    RGB(0,2,2), // cyan - disk

    RGB(3,1,0), // orange - square
    RGB(1,2,1), // dark green - arrow2
    RGB(3,0,0), // red - star
    RGB(2,1,1), // brown - 8
    RGB(1,1,1), // grey - 6
    RGB(2,2,0), // yellow - triangle
};



void set_color_from_lambda(fp_t lambda, uint8_t nb_led) {
    fp_t tmp_lambda = ABS(lambda);

    // Find closest centroid
    fp_t min_d = 100000.0f;
    uint8_t min_idx = 0;
    for(uint8_t i = 0; i < NCLASSES; i++) {
        fp_t const d = ABS(tmp_lambda - class_centroids[i]);
        if(d < min_d) {
            min_d = d;
            min_idx = i;
        }
    }
    // Set color depending on closest centroid
    set_color(&class_colors[min_idx], nb_led);
}


void set_color_from_s(fp_t s) {
    fp_t tmp_s = s;
    fp_t const max_s = initial_s_max_val;
    fp_t const min_s = -initial_s_max_val;

    if(tmp_s < min_s) {
        tmp_s = min_s;
    } else if(tmp_s > max_s) {
        tmp_s = max_s;
    }
    fp_t color_idx = (NRAINBOWCOLORS - 1) * ((fp_t)(tmp_s - min_s) / (fp_t)(max_s - min_s));
    if(color_idx < 0)
        color_idx = 0;
    set_color(&colors[(uint8_t)color_idx], 0);
}

void set_color_from_signs(fp_t s) {
#ifdef ENABLE_INIT_REMOVE_SUM_S
    if(current_it == 0) {
        if(s <= 0) {
            pogobot_led_setColors(3,0,3, 0);
        } else if(s > 0) {
            pogobot_led_setColors(0,3,0, 0);
        }
    } else {
        if(s <= 0) {
            pogobot_led_setColors(0,0,2, 0);
        } else if(s > 0) {
            pogobot_led_setColors(3,0,0, 0);
        }
    }
#elif defined(ENABLE_PRE_DIFFUSION)
    if(mydata->curr_diff->type == PRE_DIFFUSION_TYPE) {
        if(s <= 0) {
            pogobot_led_setColors(1,1,1, 0);
        } else if(s > 0) {
            pogobot_led_setColors(1,1,1, 0);
        }
    } else {
        if(s <= 0) {
            pogobot_led_setColors(0,0,2, 0);
        } else if(s > 0) {
            pogobot_led_setColors(3,0,0, 0);
        }
    }
#else
    if(s <= 0) {
        pogobot_led_setColors(0,0,2, 0);
    } else if(s > 0) {
        pogobot_led_setColors(3,0,0, 0);
    } /* else if(s == 0) {
        set_color(colors[(int)(NRAINBOWCOLORS/2)-1]);
    } */
#endif
}

void set_color_from_nb_neighbours(void) {
    uint8_t nb_neighbours = mydata->nb_neighbors;
    if(nb_neighbours == 0) {
//        printf("set_color_from_nb_neighbours: nb_neighbours: %d\n", nb_neighbours);
        return;
    } else if (nb_neighbours >= NRAINBOWCOLORS) {
        nb_neighbours = NRAINBOWCOLORS - 1;
    }
    //fp_t color_idx = (NRAINBOWCOLORS - 1) * ((fp_t)(nb_neighbours - 0) / (fp_t)(MAXN - 0));
    fp_t color_idx = (NRAINBOWCOLORS - 1) * ((fp_t)(nb_neighbours - 0) / (fp_t)(NRAINBOWCOLORS - 0));
    set_color(&colors[(uint8_t)color_idx], 0);
//    printf("set_color_from_nb_neighbours: nb_neighbours: %d\n", nb_neighbours);
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
