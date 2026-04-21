#include "ArenaSetup.h"
#include <iostream>
using namespace std;

void setupObstacles(TestMode mode, int& N_obs,
                    double x_obs[], double y_obs[], double size_obs[],
                    double scale) {
    N_obs = 0;
    switch (mode) {
        case TEST_ID_DANCE:
            break;

        case TEST_OFFENSE:
        case TEST_DEFENSE:
            N_obs = 8;
            x_obs[1] = 160*scale; y_obs[1] = 60*scale;  size_obs[1] = 40*scale;
            x_obs[2] = 320*scale; y_obs[2] = 60*scale;  size_obs[2] = 40*scale;
            x_obs[3] = 480*scale; y_obs[3] = 60*scale;  size_obs[3] = 40*scale;
            x_obs[4] = 160*scale; y_obs[4] = 420*scale; size_obs[4] = 40*scale;
            x_obs[5] = 320*scale; y_obs[5] = 420*scale; size_obs[5] = 40*scale;
            x_obs[6] = 480*scale; y_obs[6] = 420*scale; size_obs[6] = 40*scale;
            x_obs[7] = 250*scale; y_obs[7] = 240*scale; size_obs[7] = 50*scale;
            x_obs[8] = 390*scale; y_obs[8] = 240*scale; size_obs[8] = 50*scale;
            break;

        case TEST_NAVIGATION:
        case TEST_FULL_GAME:
            N_obs = 12;
            x_obs[1]  = 100*scale; y_obs[1]  = 100*scale; size_obs[1]  = 40*scale;
            x_obs[2]  = 540*scale; y_obs[2]  = 100*scale; size_obs[2]  = 40*scale;
            x_obs[3]  = 100*scale; y_obs[3]  = 380*scale; size_obs[3]  = 40*scale;
            x_obs[4]  = 540*scale; y_obs[4]  = 380*scale; size_obs[4]  = 40*scale;
            x_obs[5]  = 200*scale; y_obs[5]  = 180*scale; size_obs[5]  = 50*scale;
            x_obs[6]  = 320*scale; y_obs[6]  = 140*scale; size_obs[6]  = 50*scale;
            x_obs[7]  = 440*scale; y_obs[7]  = 180*scale; size_obs[7]  = 50*scale;
            x_obs[8]  = 200*scale; y_obs[8]  = 300*scale; size_obs[8]  = 50*scale;
            x_obs[9]  = 320*scale; y_obs[9]  = 340*scale; size_obs[9]  = 50*scale;
            x_obs[10] = 440*scale; y_obs[10] = 300*scale; size_obs[10] = 50*scale;
            x_obs[11] = 150*scale; y_obs[11] = 240*scale; size_obs[11] = 35*scale;
            x_obs[12] = 490*scale; y_obs[12] = 240*scale; size_obs[12] = 35*scale;
            break;
    }
}

void addBoundaryObstacles(int& N_obs,
                          double x_obs[], double y_obs[], double size_obs[],
                          double width, double height, double margin) {
    int start_idx = N_obs + 1;

    N_obs++; x_obs[N_obs] = width/2.0;          y_obs[N_obs] = margin/2.0;          size_obs[N_obs] = width;   // top
    N_obs++; x_obs[N_obs] = width/2.0;          y_obs[N_obs] = height - margin/2.0; size_obs[N_obs] = width;   // bottom
    N_obs++; x_obs[N_obs] = margin/2.0;          y_obs[N_obs] = height/2.0;          size_obs[N_obs] = height;  // left
    N_obs++; x_obs[N_obs] = width - margin/2.0; y_obs[N_obs] = height/2.0;          size_obs[N_obs] = height;  // right

    cout << "Added " << (N_obs - start_idx + 1) << " boundary walls (margin=" << margin << "px)" << endl;
}