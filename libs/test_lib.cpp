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
    double vel;

    std::vector<double> vv(steps), tvST(steps), tvSTJ(steps), tvSTJR(steps);

    for (uint8_t i = 0; i < steps; ++i)
    {
        vel = vMin + vStep*i;
        vv.at(i) = vel;

        // single track
        tvST.at(i) = calcTimeST_delta(vel, nF);

        // single track jerk
        tvSTJ.at(i) = calcTimeST_jerk(vel, nF);

        // single track jerk relax
        tvSTJR.at(i) = calcTimeST_jerk_relax(vel, nF);
    }

    plt::figure_size(1200, 780);
    plt::named_plot("Single-Track", vv, tvST,"b-o");
    plt::named_plot("Single-Track Jerk", vv, tvSTJ,"r-o");
    plt::named_plot("Single-Track Jerk Relax", vv, tvSTJR,"g-o");
    plt::xlabel("Velocity (m/s)");
    plt::ylabel("Time (s)");
    plt::title("OCP minimum time lane-change");
    plt::legend();
    plt::save("./ocp_laneChange.png");
//    plt::show();

    return EXIT_SUCCESS;
}
