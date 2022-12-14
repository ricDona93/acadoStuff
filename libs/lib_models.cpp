//
// Created by Riccardo Donà on 10/28/21.
//
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include "lib_models.h"

// global variables
const double KKT_th = 1e-14;
const int maxNumIteration = 250;

const double betaMax = 1e-1;
const double betaEnd =  20e-3;
const double deltaEnd = 1e-3;
const double deltaMax = 0.5;
const double ayMax = 8;
const double jyMax = 50;
const double uyMax  = .5;

const double poleSteer = 5.;
const double By = 15;
const double Cy = 1.2;

// braking model
const double axMax = -8;
const double jxMax = -40;

USING_NAMESPACE_ACADO

double calcTimeST_delta(vehST veh_params, double V, double nF){

    const double lf = veh_params.lf;
    const double lr = veh_params.lr;
    const double Kr = veh_params.Kr;
    const double Kf = veh_params.Kf;
    const double  m = veh_params.m;
    const double  I = veh_params.I;
    const double lx = veh_params.lx;

    // OCP variables
    DifferentialState       n, psi, Omega, beta;
    IntermediateState       ay;
    Control                 delta;
    Parameter               T          ;   // the time horizon T
    DifferentialEquation    f( 0.0, T );

    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;
    ocp.minimizeLagrangeTerm(0.5*delta*delta);

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
    algorithm.set( MAX_NUM_ITERATIONS, maxNumIteration);
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

double calcTimeSTV_delta(vehST veh_params, double V, double nF){

    const double lf = veh_params.lf;
    const double lr = veh_params.lr;
    const double Kr = veh_params.Kr;
    const double Kf = veh_params.Kf;
    const double  m = veh_params.m;
    const double  I = veh_params.I;
    const double lx = veh_params.lx;

    // OCP variables
    DifferentialState       s, n, psi, Omega, beta, u;
    IntermediateState       ay;
    Control                 delta, gas;
    Parameter               T, S          ;   // the time horizon T
    DifferentialEquation    f( 0.0, T );

    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;
    ocp.minimizeLagrangeTerm(0.5*delta*delta);

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(s)     == u*cos(psi);
    f << dot(u)     == gas;
    f << dot(n)     == u*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == -2*(lf*lf*Kf+lr*lr*Kr)/(I*u)*Omega -2*(lf*Kf-lr*Kr)/I*beta + 2*lf*Kf/I*delta;
    f << dot(beta)  == -(1+2*(lf*Kf-lr*Kr)/(m*u*u))*Omega -2*(Kf+Kr)/(m*u)*beta   + 2*Kf/(m*u)*delta;
    S << T*u(AT_END);

    // lateral acceleration for saturation
    ay = -(1+2*(lf*Kf-lr*Kr)/(m*u*u))*Omega -2*(Kf+Kr)/(m*u)*beta   + 2*Kf/(m*u)*delta + Omega*u;


    // DEFINE AN OPTIMAL CONTROL PROBLEM
    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, n ==  0.0 );
    ocp.subjectTo( AT_START, s ==  0.0 );
    ocp.subjectTo( AT_START, psi ==  0.01 );
    ocp.subjectTo( AT_START, beta ==  0.0 );
    ocp.subjectTo( AT_START, Omega == 0.01 );
    ocp.subjectTo( AT_START, u == V );

    ocp.subjectTo( AT_END  , n == nF );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );
    ocp.subjectTo( AT_END  , beta ==  0.0 );

    ocp.subjectTo( -betaMax <= beta <=  betaMax   );
    ocp.subjectTo( -ayMax <= ay <=  ayMax  );
    ocp.subjectTo( -deltaMax <= delta <= deltaMax );

    // SOLVE OCP
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( MAX_NUM_ITERATIONS, maxNumIteration);
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

double calcTimeDT_delta(vehST veh_params, double V, double nF){

    const double lf = veh_params.lf;
    const double lr = veh_params.lr;
    const double Kr = veh_params.Kr;
    const double Kf = veh_params.Kf;
    const double  m = veh_params.m;
    const double  I = veh_params.I;
    const double lx = veh_params.lx;

    // OCP variables
    DifferentialState       n, psi, Omega, beta, ;
    IntermediateState       ay;
    Control                 delta;
    Parameter               T          ;   // the time horizon T
    DifferentialEquation    f( 0.0, T );

    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;
    ocp.minimizeLagrangeTerm(0.5*delta*delta);

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
    algorithm.set( MAX_NUM_ITERATIONS, maxNumIteration);
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

double calcTimeST_jerk(vehST veh_params, double V, double nF){

    const double lf = veh_params.lf;
    const double lr = veh_params.lr;
    const double Kr = veh_params.Kr;
    const double Kf = veh_params.Kf;
    const double  m = veh_params.m;
    const double  I = veh_params.I;
    const double lx = veh_params.lx;

    // OCP variables
    DifferentialState       n, psi, Omega, beta, delta;
    IntermediateState       ay;
    Control                 u;
    Parameter               T;
    DifferentialEquation    f( 0.0, T);

    OCP ocp( 0.0, T);
    ocp.minimizeMayerTerm(T);
    ocp.minimizeLagrangeTerm(0.05*u*u);


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
    ocp.subjectTo(-uyMax <= u <= uyMax );


    // SOLVE OCP
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( MAX_NUM_ITERATIONS, maxNumIteration);
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

double calcTimeST_jerk_relax(vehST veh_params, double V, double nF){

    const double lf = veh_params.lf;
    const double lr = veh_params.lr;
    const double Kr = veh_params.Kr;
    const double Kf = veh_params.Kf;
    const double  m = veh_params.m;
    const double  I = veh_params.I;
    const double lx = veh_params.lx;

    // OCP variables
    DifferentialState       n, psi, Omega, beta, aF, aR, delta;
    IntermediateState       ay;
    Control                 u;
    Parameter               T;
    DifferentialEquation    f( 0.0, T);

    OCP ocp( 0.0, T);
    ocp.minimizeMayerTerm(T);
    ocp.minimizeLagrangeTerm(0.5*u*u);


    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == (2*(-(Kf*lf*aF) + Kr*lr*aR))/(I);
    f << dot(beta)  == -(2*(Kf*aF+Kr*aR)/(m*V))-Omega;
    f << dot(aR)    == -V*aR/lx + (V*beta - lr*Omega)/lx;
    f << dot(aF)    == -V*aF/lx + (V*beta - V*delta + lf*Omega)/lx;
//    f << dot(delta) == u;
    f << dot(delta) == (u - delta)*poleSteer;

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
    ocp.subjectTo( AT_END  , -betaEnd <= beta <=  betaEnd );
    ocp.subjectTo( AT_END  , -deltaEnd <= delta <=  deltaEnd );
    ocp.subjectTo( AT_END, -betaEnd <= aR <= betaEnd );
    ocp.subjectTo( AT_END, -betaEnd <= aF <= betaEnd );

    ocp.subjectTo( -betaMax <= beta <=  betaMax   );
    ocp.subjectTo( -ayMax <= ay <=  ayMax  );
    ocp.subjectTo( -deltaMax <= delta <= deltaMax );
    ocp.subjectTo(-uyMax <= u <= uyMax );


    // SOLVE OCP
    OptimizationAlgorithm algorithm(ocp);
//    algorithm.set( DISCRETIZATION_TYPE  , MULTIPLE_SHOOTING );
//    algorithm.set( INTEGRATOR_TOLERANCE , 1e-10          );
    algorithm.set( MAX_NUM_ITERATIONS, maxNumIteration);
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

double calcTimeST_jerk_pac(vehST veh_params, double V, double nF){

    const double lf = veh_params.lf;
    const double lr = veh_params.lr;
    const double Kr = veh_params.Kr;
    const double Kf = veh_params.Kf;
    const double  m = veh_params.m;
    const double  I = veh_params.I;
    const double lx = veh_params.lx;

    // OCP variables
    DifferentialState       n, psi, Omega, beta, delta;
    IntermediateState       ay;
    Control                 u;
    Parameter               T;
    DifferentialEquation    f( 0.0, T);

    OCP ocp( 0.0, T);
    ocp.minimizeMayerTerm(T);
    ocp.minimizeLagrangeTerm(0.05*u*u);


    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == (2*(-(Kf*lf*sin(Cy*atan(By*(beta - delta + (lf*Omega)/V)))) + Kr*lr*sin(Cy*atan(By*(beta - (lr*Omega)/V)))))/(Cy*By*I);
    f << dot(beta)  == (-2*(Kf*sin(Cy*atan(By*(beta - delta + (lf*Omega)/V))) + Kr*sin(Cy*atan(By*(beta - (lr*Omega)/V)))))/(Cy*By*m*V) - Omega;
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
    ocp.subjectTo( AT_END  , delta ==  0.0 );

    ocp.subjectTo( -deltaMax <= delta <= deltaMax );
    ocp.subjectTo(-uyMax <= u <= uyMax );


    // SOLVE OCP
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( KKT_TOLERANCE        , KKT_th);
    algorithm.set( MAX_NUM_ITERATIONS, maxNumIteration);
    algorithm.solve()                   ;

    VariablesGrid params;
    algorithm.getParameters( params );

    double tF = double(params(0,0));

    printf("\n------------- Minimum time for OCP ------------\n");
    printf("Case selected: single track Pacejka jerk\n");
    printf("Reference velocity: %.3e (m/s), lateral offset: %.3e (m)\n", V, nF);
    printf("T  =  %.3e (s)", tF );
    printf("\n-----------------------------------------------\n");

    clearAllStaticCounters();

    return tF;

}


double calcTime_braking(double V, double nF){

    double t_axMax = axMax / jxMax;
    double v_axMax = V + axMax * t_axMax + 0.5 * jxMax * pow(t_axMax, 2);
    double tF;
    if(v_axMax <= 0){
        // stopping without reaching full deceleration
        tF = (- axMax - sqrt(pow(axMax,2) - 2 * V * jxMax) ) / jxMax;
    }
    else{
        // stopping wih full deceleration
        tF =  t_axMax - v_axMax / axMax;
    }

    //tF = - V / axMax;


    printf("\n------------- Minimum time for OCP ------------\n");
    printf("Case selected: braking\n");
    printf("Reference velocity: %.3e (m/s), lateral offset: %.3e (m)\n", V, nF);
    printf("T  =  %.3e (s)", tF );
    printf("\n-----------------------------------------------\n");

    return tF;
}

double calcTime_kine(double V, double nF){

    // steady steering time
    double tY = -0.5 * (3*ayMax - sqrt(pow(ayMax,3) + 4 * pow(jyMax, 2) * nF) / sqrt(ayMax)) / jyMax;

    double tF = 4 * (ayMax / jyMax) + 2 * tY;

    printf("\n------------- Minimum time for OCP ------------\n");
    printf("Case selected: kinematic\n");
    printf("Reference velocity: %.3e (m/s), lateral offset: %.3e (m)\n", V, nF);
    printf("T  =  %.3e (s)", tF );
    printf("\n-----------------------------------------------\n");

    return tF;
}