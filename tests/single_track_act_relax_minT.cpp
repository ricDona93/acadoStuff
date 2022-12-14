#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( )
{
    USING_NAMESPACE_ACADO

    const double V = 30.0;       // reference velocity
    const double betaMax = 1e-1;
    const double betaEnd =  20e-3;
    const double deltaEnd = 1e-3;
    const double deltaMax = 0.5;
    const double ayMax = 8;
    const double jyMax = 50;
    const double uyMax  = 2.0;
    const double nF = 2.5;

//    const double lf = 1.1;
//    const double lr = 1.6;
//    const double Kr = 60000;
//    const double Kf = 55000;
//    const double  m = 1500;
//    const double  I = 2500;
    const double lx = 0.5;

    double lf, lr, Kf, Kr, m, I;
    lf = 1.37;
    lr = 1.58;
    Kf = 57930;
    Kr = 42040;
    m = 2200;
    I = 5000;

    lf = 1.22;
    lr = 1.04;
    Kf = 58620;
    Kr = 71360;
    m = 1400;
    I = 1776;

    // OCP variables
    DifferentialState       n, psi, Omega, beta, aF, aR, delta;
    IntermediateState       ay;
    Control                 u;
    Parameter               T;
    DifferentialEquation    f( 0.0, T);

    VariablesGrid OCP_states, OCP_control;

    OCP ocp( 0.0, T);
    ocp.minimizeMayerTerm(T);
//    ocp.minimizeMayerTerm(10*beta);
    ocp.minimizeLagrangeTerm(0.5*u*u);


    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == (2*(-(Kf*lf*aF) + Kr*lr*aR))/(I);
    f << dot(beta)  == -(2*(Kf*aF+Kr*aR)/(m*V))-Omega;
    f << dot(aR)    == -V*aR/lx + (V*beta - lr*Omega)/lx;
    f << dot(aF)    == -V*aF/lx + (V*beta - V*delta + lf*Omega)/lx;
    f << dot(delta) == (u-delta)*5;


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
    ocp.subjectTo( AT_END  , -betaEnd <= beta <=  betaEnd );
    ocp.subjectTo( AT_END  , -deltaEnd <= delta <= deltaEnd );
    ocp.subjectTo( AT_END  , -betaEnd <= aF <=  betaEnd );
    ocp.subjectTo( AT_END  , -betaEnd <= aR <=  betaEnd );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
    ocp.subjectTo( AT_END  , delta ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );


    ocp.subjectTo( -betaMax <= beta <=  betaMax   );
//    ocp.subjectTo( -ayMax/V <= Omega <=  ayMax/V   );
    ocp.subjectTo( -ayMax <= ay <=  ayMax  );
    ocp.subjectTo( -deltaMax <= delta <= deltaMax );
    ocp.subjectTo(- uyMax <= u <= uyMax );

    GnuplotWindow window;
    window.addSubplot( n,   "n [m]" );
    window.addSubplot( psi,   "psi [rad]" , "Time (s)");
    window.addSubplot( beta,   "beta [rad/s]", "Time (s)" );
    window.addSubplot( Omega,   "Omega [rad/s]", "Time (s)" );
//    window.addSubplot( aF,   "Front slip [rad]", "Time (s)" );
//    window.addSubplot( aR,   "Rear slip [rad]", "Time (s)" );
    window.addSubplot( ay,   "ay [m/s2]", "Time (s)" );
    window.addSubplot( delta, "steering angle [rad]", "Time (s)" );
//    window.addSubplot( u, "steering rate [rad/s]", "Time (s)" );
    window.plot( );

    OptimizationAlgorithm algorithm(ocp);   // construct optimization algorithm,
    algorithm << window                 ;   // flush the plot window,


    // SOLVE OCP
    algorithm.set( MAX_NUM_ITERATIONS, 200);
//    algorithm.set( INTEGRATOR_TOLERANCE , 1e-10          );
//    algorithm.set( DISCRETIZATION_TYPE  , MULTIPLE_SHOOTING );
    algorithm.set( KKT_TOLERANCE        , 1e-14);
    algorithm.solve()                   ;

    VariablesGrid params;
    algorithm.getParameters( params );
    algorithm.getDifferentialStates(OCP_states);
    algorithm.getControls(OCP_control);

//    OCP_states.print("results_states_jerk_relax.txt");
//    OCP_control.print("results_control_jerk_relax.txt");
//    params.print("results_params_jerk_relax.txt");

    OCP_states.print("results_states_jerk_relax_reina.txt");
    OCP_control.print("results_control_jerk_relax_reina.txt");
    params.print("results_params_jerk_relax_reina.txt");

    printf("\n------------- Minimum time for OCP ------------\n");
    printf("T  =  %.3e (s)", params(0,0) );
    printf("\n-----------------------------------------------\n");

    return 0;
}

