//
// Created by Riccardo Donà on 10/28/21.
//

#ifndef TESTACADO_LIB_MODELS_H
#define TESTACADO_LIB_MODELS_H

// global variables
extern const double KKT_th;

extern const double betaMax;
extern const double deltaMax;
extern const double ayMax;
extern const double jMax;

extern const double lf;
extern const double lr ;
extern const double Kr;
extern const double Kf;
extern const double  m;
extern const double  I;


double calcTimeST_delta(double V, double nF);

double calcTimeST_pac(double V, double nF);

double calcTimeST_jerk(double V, double nF);

double calcTimeST_jerk_relax(double V, double nF);


#endif //TESTACADO_LIB_MODELS_H
