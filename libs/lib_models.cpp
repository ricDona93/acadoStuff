//
// Created by Riccardo Donà on 10/28/21.
//
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include "lib_models.h"

// global variables
const double KKT_th = 1e-14;

const double betaMax = 8e-2;
const double deltaMax = 0.4;
const double ayMax = 8;
const double jMax  = 1.0;

const double lf = 1.1;
const double lr = 1.6;
const double Kr = 60000;
const double Kf = 55000;
const double  m = 1500;
const double  I = 2500;
const double lx = 0.2;

USING_NAMESPACE_ACADO

double calcTimeST_delta(double V, double nF){

    // OCP variables
    DifferentialState       n, psi, Omega, beta;
    IntermediateState       ay;
    Control                 delta;
    Parameter               T          ;   // the time horizon T
    DifferentialEquation    f( 0.0, T );

    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;
    ocp.minimizeLagrangeTerm(delta*delta);

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == -2*(lf*lf*Kf+lr*lr*Kr)/(I*V)*Omega -2*(lf*Kf-lr*Kr)/I*beta + 2*lf*Kf/I*delta;
    f << dot(beta)  == -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta;

    // lateral acceleration for saturation
    ay = -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta + Omega*V;


    // DEFINE AN OPTIMAL CONTROL PROBLEM
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, n ==  0.0 );
    ocp.subjectTo( AT_START, psi ==  0.01 );
    ocp.subjectTo( AT_START, beta ==  0.0 );
    ocp.subjectTo( AT_START, Omega == 0.01 );

    ocp.subjectTo( AT_END  , n == nF );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );
    ocp.subjectTo( AT_END  , beta ==  0.0 );

    ocp.subjectTo( -betaMax <= beta <=  betaMax   );
    ocp.subjectTo( -ayMax <= ay <=  ayMax  );
    ocp.subjectTo( -deltaMax <= delta <= deltaMax );

    // SOLVE OCP
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( KKT_TOLERANCE, KKT_th);
    algorithm.solve();

    VariablesGrid params;
    algorithm.getParameters( params );

    double tF = double(params(0,0));

    printf("\n------------- Minimum time for OCP ------------\n");
    printf("Case selected: single track\n");
    printf("Reference velocity: %f (m/s), lateral offset: %f (m)\n", V, nF);
    printf("T  =  %.3e (s)", tF);
    printf("\n-----------------------------------------------\n");

    clearAllStaticCounters();

    return tF;

}

double calcTimeST_jerk(double V, double nF){

    // OCP variables
    DifferentialState       n, psi, Omega, beta, delta;
    IntermediateState       ay;
    Control                 u;
    Parameter               T;
    DifferentialEquation    f( 0.0, T);

    OCP ocp( 0.0, T);
    ocp.minimizeMayerTerm(T);
    ocp.minimizeLagrangeTerm(u*u);


    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == -2*(lf*lf*Kf+lr*lr*Kr)/(I*V)*Omega -2*(lf*Kf-lr*Kr)/I*beta + 2*lf*Kf/I*delta;
    f << dot(beta)  == -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta;
    f << dot(delta) == u;

    // lateral acceleration for saturation
    ay = -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta + Omega*V;


    // DEFINE AN OPTIMAL CONTROL PROBLEM
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, n ==  0.0 );
    ocp.subjectTo( AT_START, psi ==  0.01 );
    ocp.subjectTo( AT_START, beta ==  0.0 );
    ocp.subjectTo( AT_START, Omega == 0.01 );
    ocp.subjectTo( AT_START, delta == 0.0 );

    ocp.subjectTo( AT_END  , n == nF );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );
    ocp.subjectTo( AT_END  , beta ==  0.0 );
    ocp.subjectTo( AT_END, delta == 0.0 );

    ocp.subjectTo( -betaMax <= beta <=  betaMax   );
    ocp.subjectTo( -ayMax <= ay <=  ayMax  );
    ocp.subjectTo( -deltaMax <= delta <= deltaMax );
    ocp.subjectTo( -jMax <= u <= jMax );


    // SOLVE OCP
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( KKT_TOLERANCE        , KKT_th);
    algorithm.solve()                   ;

    VariablesGrid params;
    algorithm.getParameters( params );

    double tF = double(params(0,0));

    printf("\n------------- Minimum time for OCP ------------\n");
    printf("Case selected: single track jerk\n");
    printf("Reference velocity: %.3e (m/s), lateral offset: %.3e (m)\n", V, nF);
    printf("T  =  %.3e (s)", tF );
    printf("\n-----------------------------------------------\n");

    clearAllStaticCounters();

    return tF;

}

double calcTimeST_jerk_relax(double V, double nF){

    // OCP variables
    DifferentialState       n, psi, Omega, beta, aF, aR, delta;
    IntermediateState       ay;
    Control                 u;
    Parameter               T;
    DifferentialEquation    f( 0.0, T);

    OCP ocp( 0.0, T);
    ocp.minimizeMayerTerm(T);
    ocp.minimizeLagrangeTerm(u*u);


    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == (2*(-(Kf*lf*aF) + Kr*lr*aR))/(I);
    f << dot(beta)  == -(2*(Kf*aF+Kr*aR)/(m*V))-Omega;
    f << dot(aR)    == -V*aR/lx + (V*beta - lr*Omega)/lx;
    f << dot(aF)    == -V*aF/lx + (V*beta - V*delta + lf*Omega)/lx;
    f << dot(delta) == u;

    // lateral acceleration for saturation
    ay = -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta + Omega*V;


    // DEFINE AN OPTIMAL CONTROL PROBLEM
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, n ==  0.0 );
    ocp.subjectTo( AT_START, psi ==  0.01 );
    ocp.subjectTo( AT_START, beta ==  0.0 );
    ocp.subjectTo( AT_START, Omega == 0.01 );
    ocp.subjectTo( AT_START, aR == 0.0 );
    ocp.subjectTo( AT_START, aF == 0.0 );
    ocp.subjectTo( AT_START, delta == 0.0 );

    ocp.subjectTo( AT_END  , n == nF );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );
    ocp.subjectTo( AT_END  , beta ==  0.0 );
    ocp.subjectTo( AT_END  , delta ==  0.0 );
    ocp.subjectTo( AT_END, -0.01 <= aR <= 0.01 );
    ocp.subjectTo( AT_END, -0.01 <= aF <= 0.01 );

    ocp.subjectTo( -betaMax <= beta <=  betaMax   );
    ocp.subjectTo( -ayMax <= ay <=  ayMax  );
    ocp.subjectTo( -deltaMax <= delta <= deltaMax );
    ocp.subjectTo( -jMax <= u <= jMax );


    // SOLVE OCP
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( KKT_TOLERANCE        , KKT_th);
    algorithm.solve()                   ;

    VariablesGrid params;
    algorithm.getParameters( params );

    double tF = double(params(0,0));

    printf("\n------------- Minimum time for OCP ------------\n");
    printf("Case selected: single track relax jerk\n");
    printf("Reference velocity: %.3e (m/s), lateral offset: %.3e (m)\n", V, nF);
    printf("T  =  %.3e (s)", tF );
    printf("\n-----------------------------------------------\n");

    clearAllStaticCounters();

    return tF;

}