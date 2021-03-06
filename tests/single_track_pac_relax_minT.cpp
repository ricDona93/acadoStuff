#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( )
{
    USING_NAMESPACE_ACADO

    DifferentialState       n, psi, Omega, beta, aF, aR, delta;
    IntermediateState       ay;
    Control                 u;
    Parameter               T          ;   // the time horizon T
    DifferentialEquation    f( 0.0, T );

    const double KKT_th = 1e-14;
    const double V = 20.0;
    const double lat_offs =3.5;

    const double betaMax = 8e-2;
    const double deltaMax = 0.4;
    const double ayMax = 8;
    const double jMax  = 1.0;

    const double By = 15;
    const double Cy = 1.2;

    const double lf = 1.1;
    const double lr = 1.6;
    const double Kr = 60000;
    const double Kf = 55000;
    const double  m = 1500;
    const double  I = 2500;
    const double lx = 0.4;

    //-------------------------------------
    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;   // the time T should be optimized
    ocp.minimizeLagrangeTerm(0.05*u*u);

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == (2*(-(Kf*lf*sin(Cy*atan(By*aF))) + Kr*lr*sin(Cy*atan(By*aR))))/(Cy*By*I);
//    f << dot(Omega) == (2*(-(Kf*lf*aF) + Kr*lr*aR))/(I);
    f << dot(beta)  == -(2*(Kf*sin(Cy*atan(By*aF))+Kr*sin(Cy*atan(By*aR)))/(Cy*By*m*V))-Omega;
//    f << dot(beta)  == -(2*(Kf*aF+Kr*aR)/(m*V))-Omega;
    f << dot(aR)    == -V*aR/lx + (V*beta - lr*Omega)/lx;
    f << dot(aF)    == -V*aF/lx + (V*beta - V*delta + lf*Omega)/lx;
    f << dot(delta) == u;

    ay = -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta + Omega*V;


    // DEFINE AN OPTIMAL CONTROL PROBLEM
    // ----------------------------------

    ocp.subjectTo( f );   // minimize T s.t. the model,
    ocp.subjectTo( AT_START, n ==  0.0 );
    ocp.subjectTo( AT_START, psi ==  0.01 );
    ocp.subjectTo( AT_START, beta ==  0.0 );
    ocp.subjectTo( AT_START, Omega == 0.01 );
    ocp.subjectTo( AT_START, aR == 0.0 );
    ocp.subjectTo( AT_START, aF == 0.0 );
    ocp.subjectTo( AT_START, delta == 0.0 );

    ocp.subjectTo( AT_END  , n == lat_offs );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );
    ocp.subjectTo( AT_END  , delta ==  0.0 );
    ocp.subjectTo( AT_END  , beta ==  0.0 );
    ocp.subjectTo( AT_END, -0.01 <= aR <= 0.01 );
//    ocp.subjectTo( AT_END, -0.05 <= aF <= 0.05 );
//    ocp.subjectTo( AT_END, aR == 0.0 );
//    ocp.subjectTo( AT_END, aF == 0.0 );


//    ocp.subjectTo( 0 <= T <=  4   );
    ocp.subjectTo( -betaMax <= beta <=  betaMax   );
    ocp.subjectTo( -ayMax <= ay <=  ayMax );
    ocp.subjectTo( -deltaMax <= delta <= deltaMax );
    ocp.subjectTo( -jMax <= u <= jMax );

    GnuplotWindow window;
    window.addSubplot( n,   "n [m]" );
    window.addSubplot( psi,   "psi [rad]" );
    window.addSubplot( beta,   "beta [rad]" );
    window.addSubplot( Omega,   "Omega [rad/s]" );
    window.addSubplot( delta, "steering angle [rad]" );
    window.addSubplot( u, "steering angle rate [rad/s]" );
    window.addSubplot( ay, "lateral acceleration [m/s^2]" );
    window.addSubplot( aF, "front slip [rad]" );
    window.addSubplot( aR, "rear slip [rad]" );
    window.plot( );

    OptimizationAlgorithm algorithm(ocp);   // construct optimization algorithm,
    algorithm << window                 ;   // flush the plot window,


//    algorithm.set( INTEGRATOR_TYPE      , INT_RK45        );
//    algorithm.set( INTEGRATOR_TOLERANCE , 1e-10          );
//    algorithm.set( DISCRETIZATION_TYPE  , MULTIPLE_SHOOTING );
    algorithm.set( KKT_TOLERANCE        , KKT_th            );
    algorithm.set( MAX_NUM_ITERATIONS, 1000);

    algorithm.solve()                   ;   // and solve the problem.

    VariablesGrid params;
    algorithm.getParameters( params );

    printf("\n------------- Minimum time for OCP ------------\n");
    printf("T  =  %.3e (s)", params(0,0) );
    printf("\n-----------------------------------------------\n");

    return 0;
}

