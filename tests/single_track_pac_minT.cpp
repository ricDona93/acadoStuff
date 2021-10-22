#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( )
{
    USING_NAMESPACE_ACADO

    DifferentialState       n, psi, Omega, beta;
    Control                 delta;
    Parameter               T          ;   // the time horizon T
    DifferentialEquation    f( 0.0, T );


    const double V = 10.0;       // reference velocity
    const double ayMax = 8;
    const double lat_offs = 3.2;

    const double lf = 1.1;
    const double lr = 1.6;
    const double Kr = 60000;
    const double Kf = 55000;
    const double By = 10;
    const double Cy = 1.5;
    const double  m = 1500;
    const double  I = 2500;


    //-------------------------------------
    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;   // the time T should be optimized

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == (2*Cy*(-(Kf*lf*sin(Cy*atan(By*(beta - delta + (lf*Omega)/V)))) + Kr*lr*sin(Cy*atan(By*(beta - (lr*Omega)/V)))))/(By*I);
    f << dot(beta)  == (-2*Cy*(Kf*sin(Cy*atan(By*(beta - delta + (lf*Omega)/V))) + Kr*sin(Cy*atan(By*(beta - (lr*Omega)/V)))))/(By*m*V) - Omega;

    // DEFINE AN OPTIMAL CONTROL PROBLEM
    // ----------------------------------

    ocp.subjectTo( f );   // minimize T s.t. the model,
    ocp.subjectTo( AT_START, n ==  0.0 );
    ocp.subjectTo( AT_START, psi ==  0.01 );
    ocp.subjectTo( AT_START, beta ==  0.0 );
    ocp.subjectTo( AT_START, Omega == 0.01 );

    ocp.subjectTo( AT_END  , n == lat_offs );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
//    ocp.subjectTo( AT_END  , beta ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );

    ocp.subjectTo( -0.2 <= beta <=  0.2   );
    ocp.subjectTo( -ayMax/V <= Omega <=  ayMax/V  );
    ocp.subjectTo( -0.2 <= delta <= 0.2 );


    GnuplotWindow window;
    window.addSubplot( n,   "n [m]" );
    window.addSubplot( psi,   "psi [rad]" );
    window.addSubplot( beta,   "beta [rad]" );
    window.addSubplot( Omega,   "Omega [rad/s]" );
    window.addSubplot( delta, "steering angle [rad]" );
    window.plot( );

    OptimizationAlgorithm algorithm(ocp);   // construct optimization algorithm,
    algorithm << window                 ;   // flush the plot window,


//    algorithm.set( INTEGRATOR_TYPE      , INT_RK45        );
//    algorithm.set( INTEGRATOR_TOLERANCE , 1e-10          );
//    algorithm.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    algorithm.set( KKT_TOLERANCE        , 1e-13            );


    algorithm.solve()                   ;   // and solve the problem.

    return 0;
}