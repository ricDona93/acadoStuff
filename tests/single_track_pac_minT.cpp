#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( )
{
    USING_NAMESPACE_ACADO

    DifferentialState       n, psi, Omega, beta, delta;
    IntermediateState       ay;
    Control                 u;
    Parameter               T          ;   // the time horizon T
    DifferentialEquation    f( 0.0, T );


    const double V = 30.0;       // reference velocity
    const double betaMax = 8e-2;
    const double deltaMax = 0.4;
    const double jMax  = 1.0;
    const double lat_offs = 3.2;

    const double lf = 1.1;
    const double lr = 1.6;
    const double Kr = 60000;
    const double Kf = 55000;
    const double By = 20;
    const double Cy = 1.2;
    const double  m = 1500;
    const double  I = 2500;


    //-------------------------------------
    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;   // the time T should be optimized
    ocp.minimizeLagrangeTerm(delta*delta);

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == (2*(-(Kf*lf*sin(Cy*atan(By*(beta - delta + (lf*Omega)/V)))) + Kr*lr*sin(Cy*atan(By*(beta - (lr*Omega)/V)))))/(Cy*By*I);
    f << dot(beta)  == (-2*(Kf*sin(Cy*atan(By*(beta - delta + (lf*Omega)/V))) + Kr*sin(Cy*atan(By*(beta - (lr*Omega)/V)))))/(Cy*By*m*V) - Omega;
    f << dot(delta) == u;

    ay = -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta + Omega*V;

    // DEFINE AN OPTIMAL CONTROL PROBLEM
    // ----------------------------------

    ocp.subjectTo( f );   // minimize T s.t. the model,
    ocp.subjectTo( AT_START, n ==  0.0 );
    ocp.subjectTo( AT_START, psi ==  0.01 );
    ocp.subjectTo( AT_START, beta ==  0.0 );
    ocp.subjectTo( AT_START, delta ==  0.0 );
    ocp.subjectTo( AT_START, Omega == 0.01 );

    ocp.subjectTo( AT_END  , n == lat_offs );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
    ocp.subjectTo( AT_END  , beta ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );

    ocp.subjectTo( -betaMax <= beta <=  betaMax   );
    ocp.subjectTo( -deltaMax <= delta <= deltaMax );
    ocp.subjectTo( -jMax <= u <= jMax );

    GnuplotWindow window;
    window.addSubplot( n,   "n [m]" );
    window.addSubplot( psi,   "psi [rad]" );
    window.addSubplot( beta,   "beta [rad]" );
    window.addSubplot( ay,   "ay [m/s^2]" );
    window.addSubplot( Omega,   "Omega [rad/s]" );
    window.addSubplot( delta, "steering angle [rad]" );
    window.plot( );

    OptimizationAlgorithm algorithm(ocp);   // construct optimization algorithm,
    algorithm << window                 ;   // flush the plot window,

    algorithm.set( MAX_NUM_ITERATIONS, 1000);

//    algorithm.set( INTEGRATOR_TYPE      , INT_RK45        );
//    algorithm.set( INTEGRATOR_TOLERANCE , 1e-10          );
    algorithm.set( DISCRETIZATION_TYPE  , MULTIPLE_SHOOTING );
    algorithm.set( KKT_TOLERANCE        , 1e-14            );

    algorithm.solve()                   ;   // and solve the problem.

    VariablesGrid params;
    algorithm.getParameters( params );

    printf("\n------------- Minimum time for OCP ------------\n");
    printf("T  =  %.3e (s)", params(0,0) );
    printf("\n-----------------------------------------------\n");

    return 0;
}

