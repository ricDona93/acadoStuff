//
// Created by Riccardo Donà on 10/28/21.
//
#include <stdio.h>
#include <acado_gnuplot.hpp>
#include "lib_models.h"

int main(){

    // create data structure for velocity
    short int steps = 6;
    const double vMin=5., vStep=5, nF=3.2;

    ACADO::DMatrix tST(steps,2);
    ACADO::DMatrix tSTJ(steps,2);

    double vel = 0.;
    for (uint8_t i = 0; i < steps; ++i)
    {
        vel = vMin + vStep*i;
        tST(i, 0) = vel;
        tST(i, 1) = calcTimeST_delta(vel, nF);
        tSTJ(i, 0) = vel;
        tSTJ(i, 1) = calcTimeST_jerk(vel, nF);
    }

    ACADO::VariablesGrid outputST = tST;
    ACADO::VariablesGrid outputSTJ = tSTJ;

    ACADO::GnuplotWindow window;
    window.addSubplot(outputST, "Single Track");
    window.addSubplot(outputSTJ, "Single Track Jerk");
    window.plot( );


    return EXIT_SUCCESS;
}
