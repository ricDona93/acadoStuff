//
// Created by Riccardo Donà on 10/28/21.
//
#include <stdio.h>
#include <acado_gnuplot.hpp>
#include "lib_models.h"
#include "matplotlibcpp.hpp"
#include <vector>

namespace plt = matplotlibcpp;

int main(){

    // create data structure for velocity
    short int steps = 10;
    const double vMin=5., vStep=5, nF=3.2;

    std::vector<double> vv(steps), tvST(steps), tvSTJ(steps);

    ACADO::DMatrix tST(steps,2);
    ACADO::DMatrix tSTJ(steps,2);

    double vel, tST_tmp, tSTJ_tmp;
    for (uint8_t i = 0; i < steps; ++i)
    {
        vel = vMin + vStep*i;
        vv.at(i) = vel;

        tST(i, 0) = vel;
        tST_tmp = calcTimeST_delta(vel, nF);
        tST(i, 1) = tST_tmp;
        tvST.at(i) = tST_tmp;

        tSTJ(i, 0) = vel;
        tSTJ_tmp = calcTimeST_jerk(vel, nF);
        tSTJ(i, 1) = tSTJ_tmp;
        tvSTJ.at(i) = tSTJ_tmp;
    }

    plt::figure_size(1200, 780);
    plt::named_plot("Single-Track", vv, tvST);
    plt::named_plot("Single-Track Jerk", vv, tvSTJ,"r--");
    plt::xlabel("Velocity (m/s)");
    plt::ylabel("Time (s)");
    plt::title("OCP minimum time lane-change");
    plt::legend();
    plt::save("./ocp_laneChange.png");
//    plt::show();

    return EXIT_SUCCESS;
}
