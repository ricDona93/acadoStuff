//
// Created by Riccardo Donà on 10/28/21.
//

#ifndef TESTACADO_LIB_MODELS_H
#define TESTACADO_LIB_MODELS_H

// global variables
extern const double KKT_th;
extern const int maxNumIteration;

extern const double betaMax;
extern const double deltaMax;
extern const double ayMax;
extern const double uyMax;

struct vehST{
    double lf;
    double lr;
    double Kr;
    double Kf;
    double  m;
    double  I;
    double lx = 0.5;
};


double calcTimeST_delta(vehST veh, double V, double nF);

double calcTimeST_pac(vehST veh, double V, double nF);

double calcTimeST_jerk(vehST veh, double V, double nF);

double calcTimeST_jerk_relax(vehST veh, double V, double nF);

double calcTimeST_jerk_pac(vehST veh, double V, double nF);

double calcTime_braking(double V, double nF);

double calcTime_kine(double V, double nF);

#endif //TESTACADO_LIB_MODELS_H
