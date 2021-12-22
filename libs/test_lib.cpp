//
// Created by Riccardo Donà on 10/28/21.
//
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include <string.h>

#include <acado_gnuplot.hpp>

#include "lib_models.h"
#include "matplotlibcpp.hpp"

namespace plt = matplotlibcpp;

int main(){

    // create data structure for velocity
    short int steps = 10;
    const double vMin=5., vStep=5, nF=2.5;
    double vel;

    std::vector<double> vv(steps), tvST(steps), tvSTJ(steps), tvSTJR(steps), tvSTPC(steps), tvKI(steps), tvBR(steps);
    std::vector<std::string> models_list;

    models_list.push_back("single_track");
    models_list.push_back("single_track_jerk");
    models_list.push_back("single_track_jerk_relax");
    models_list.push_back("single_track_jerk_pacejka");
    models_list.push_back("kinematic");
    models_list.push_back("braking");

    std::vector<std::vector<double>> results_to_file(models_list.size());


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

        // simple Pacejka model
        tvSTPC.at(i) = calcTimeST_jerk_pac(vel, nF);

        // kinematic model
        tvKI.at(i) = calcTime_kine(vel, nF);

        // braking model
        tvBR.at(i) = calcTime_braking(vel, nF);

    }

    results_to_file[0] = tvST;
    results_to_file[1] = tvSTJ;
    results_to_file[2] = tvSTJR;
    results_to_file[3] = tvSTPC;
    results_to_file[4] = tvKI;
    results_to_file[5] = tvBR;

    // save to CSV
    std::ofstream logFile;
    logFile.open ("lane_change_T_results.csv");
//    logFile << (std::chrono::system_clock::now()) << std::endl;
    for (uint8_t i = 0; i <= models_list.size(); ++i){
        if(i==0){
            logFile << "vel";
        }
        else {
            logFile << ", " << models_list[i-1] ;
        }
    }
    logFile << "\n";

    for (uint8_t j = 0; j < steps; ++j){
        for (uint8_t i = 0; i <= models_list.size(); ++i){
            if(i==0){
                logFile << vv[j];
            } else {
                logFile << ", " << results_to_file[i-1][j];
            }
        }
        logFile << "\n";
    }
    logFile.close();

    plt::figure_size(1200, 780);
    plt::named_plot("Single-Track", vv, tvST,"b-o");
    plt::named_plot("Single-Track Jerk", vv, tvSTJ,"r-o");
    plt::named_plot("Single-Track Jerk Relax", vv, tvSTJR,"m-o");
    plt::named_plot("Single-Track Jerk Pacejka", vv, tvSTPC,"k-o");
    plt::named_plot("Braking model", vv, tvBR, "y-*");
    plt::named_plot("Kinematic model", vv, tvKI, "g-o");
    plt::xlabel("Velocity (m/s)");
    plt::ylabel("Time (s)");
    plt::title("OCP minimum time lane-change");
    plt::legend();
    plt::save("./ocp_laneChange.png");

    return EXIT_SUCCESS;
}
