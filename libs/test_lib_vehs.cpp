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
    short int steps = 15;
    const double vMin=5., vStep=2.5, nF=2.5;
    double vel;

    vehST abe;
    abe.lf = 1.1;
    abe.lr = 1.6;
    abe.Kr = 60000;
    abe.Kf = 55000;
    abe.m = 1500;
    abe.I = 2500;

    vehST lee;
    lee.lf = 1.37;
    lee.lr = 1.58;
    lee.Kf = 57930;
    lee.Kr = 42040;
    lee.m = 2200;
    lee.I = 5000;

    vehST reina;
    reina.lf = 1.22;
    reina.lr = 1.04;
    reina.Kf = 58620;
    reina.Kr = 71360;
    reina.m = 1400;
    reina.I = 1776;

    vehST lundahl;
    lundahl.lf = 1.03;
    lundahl.lr = 1.55;
    lundahl.Kf = 46100;
    lundahl.Kr = 30700;
    lundahl.m = 1425;
    lundahl.I = 2500;
    lundahl.lx = 0.4;

    vehST soudbakksh;
    soudbakksh.lf = 0.959;
    soudbakksh.lr = 1.727;
    soudbakksh.Kf = 50758;
    soudbakksh.Kr = 33678;
    soudbakksh.m  = 1451;
    soudbakksh.I  = 2765;

    std::vector<double> vv(steps), tvABE(steps), tvLEE(steps), tvREINA(steps), tvLUN(steps), tvSOUD(steps), tvKI(steps), tvBR(steps);
    std::vector<std::string> models_list;

    models_list.push_back("Abe");
    models_list.push_back("Lee");
    models_list.push_back("Reina");
    models_list.push_back("Lundahl");
    models_list.push_back("Soudbakksh");
    models_list.push_back("K-LC");
    models_list.push_back("Braking");

    std::vector<std::vector<double>> results_to_file(models_list.size());


    for (uint8_t i = 0; i < steps; ++i)
    {
        vel = vMin + vStep*i;
        vv.at(i) = vel;

        // abe
        tvABE.at(i) = calcTimeST_jerk_relax(abe, vel, nF);

        // lee
        tvLEE.at(i) = calcTimeST_jerk_relax(lee, vel, nF);

        // reina
        tvREINA.at(i) = calcTimeST_jerk_relax(reina, vel, nF);

        // jan
        tvLUN.at(i) = calcTimeST_jerk_relax(lundahl, vel, nF);

        tvSOUD.at(i) = calcTimeST_jerk_relax(soudbakksh, vel, nF);

        // kinematic model
        tvKI.at(i) = calcTime_kine(vel, nF);

        // braking model
        tvBR.at(i) = calcTime_braking(vel, nF);

    }

    results_to_file[0] = tvABE;
    results_to_file[1] = tvLEE;
    results_to_file[2] = tvREINA;
    results_to_file[3] = tvLUN;
    results_to_file[4] = tvSOUD;
    results_to_file[5] = tvKI;
    results_to_file[6] = tvBR;

    //TODO save xcoord

    // save to CSV
    std::ofstream logFile;
    logFile.open ("results_lane_change_T_vehs.csv");
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

    plt::figure_size(800, 600);
    plt::named_plot("STJR Abe", vv, tvABE,"b-o");
    plt::named_plot("STJR Lee", vv, tvLEE,"r-o");
    plt::named_plot("STJR Baffet", vv, tvREINA,"m-o");
    plt::named_plot("STJR Lundahl", vv, tvLUN,"k-o");
    plt::named_plot("STJR Soudbakksh", vv, tvSOUD,"k-o");
    plt::named_plot("Braking model", vv, tvBR, "y-*");
    plt::named_plot("Kinematic model", vv, tvKI, "g-o");
    plt::xlabel("Velocity (m/s)");
    plt::ylabel("Time (s)");
    plt::title("OCP minimum time lane-change");
    plt::legend();
    plt::save("./ocp_laneChange_vehs.png");

    return EXIT_SUCCESS;
}
