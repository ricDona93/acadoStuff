#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( )
{
    USING_NAMESPACE_ACADO
    // DEFINE THE VARIABLES:
    // ----------------------------------------------------------
    /*
     *     DifferentialState        s,v,m      ;   // the differential states
    Control                  u          ;   // the control input u
    Parameter                T          ;   // the time horizon T
    DifferentialEquation     f( 0.0, T );   // the differential equation

    //-------------------------------------
    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;   // the time T should be optimized

    f << dot(s) == v                    ;   // an implementation
    f << dot(v) == (u-0.2*v*v)/m        ;   // of the model equations
    f << dot(m) == -0.01*u*u            ;   // for the rocket.

    ocp.subjectTo( f                   );   // minimize T s.t. the model,
    ocp.subjectTo( AT_START, s ==  0.0 );   // the initial values for s,
    ocp.subjectTo( AT_START, v ==  0.0 );   // v,
    ocp.subjectTo( AT_START, m ==  1.0 );   // and m,

    ocp.subjectTo( AT_END  , s == 10.0 );   // the terminal constraints for s
    ocp.subjectTo( AT_END  , v ==  0.0 );   // and v,

    ocp.subjectTo( -0.1 <= v <=  1.7   );   // as well as the bounds on v
    ocp.subjectTo( -1.1 <= u <=  1.1   );   // the control input u,
    ocp.subjectTo(  5.0 <= T <= 15.0   );   // and the time horizon T.
    //-------------------------------------

    GnuplotWindow window                ;   // visualize the results in a
    window.addSubplot( s, "DISTANCE s" );   // Gnuplot window.
    window.addSubplot( v, "VELOCITY v" );
    window.addSubplot( m, "MASS     m" );
    window.addSubplot( u, "CONTROL  u" );

    OptimizationAlgorithm algorithm(ocp);   // construct optimization algorithm,
    algorithm << window                 ;   // flush the plot window,
    algorithm.solve()                   ;   // and solve the problem.

    return 0                            ;
     *
     * */

    DifferentialState       n, psi, Omega, beta;
    Control                 delta;
//    DifferentialState       delta, D_delta;
//    Control                 T_act;      // control steering angle
    Parameter               T          ;   // the time horizon T
    DifferentialEquation    f( 0.0, T );


    const double V = 20.0;       // reference velocity
    const double lat_offs = 3.;

    const double lf = 1.1;
    const double lr = 1.6;
    const double Kr = 60000;
    const double Kf = 55000;
    const double  m = 1500;
    const double  I = 2500;

    // steering actuator parameters
    const double A_act_21 = -0.1924;
    const double A_act_22 = -133.8;
    const double B_act_1  = -3.985;
    const double B_act_2  = 655;

    //-------------------------------------
    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;   // the time T should be optimized

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == -2*(lf*lf*Kf+lr*lr*Kr)/(I*V)*Omega -2*(lf*Kf-lr*Kr)/I*beta + 2*lf*Kf/I*delta;
    f << dot(beta)  == -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta;
//    f << dot(delta) == D_delta + B_act_1*T_act;
//    f << dot(D_delta) == A_act_21*delta + A_act_22*D_delta + B_act_2*T_act;

    // DEFINE AN OPTIMAL CONTROL PROBLEM
    // ----------------------------------

    ocp.subjectTo( f );   // minimize T s.t. the model,
    ocp.subjectTo( AT_START, n ==  0.0 );
    ocp.subjectTo( AT_START, psi ==  0.01 );
    ocp.subjectTo( AT_START, Omega == 0.1 );
//    ocp.subjectTo( AT_START, delta ==  0.0 );
//    ocp.subjectTo( AT_START, D_delta ==  0.0 );

    ocp.subjectTo( AT_END  , n == lat_offs );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
    ocp.subjectTo( AT_END  , beta ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );
//    ocp.subjectTo( AT_END  , delta ==  0.0 );
//    ocp.subjectTo( AT_END  , D_delta ==  0.0 );

    ocp.subjectTo( -0.3 <= beta <=  0.3   );
    ocp.subjectTo( -10 <= Omega <=  10   );
    ocp.subjectTo( -0.5 <= delta <= 0.5 );
//    ocp.subjectTo( -5.0 <= D_delta <= 5.0 );
//    ocp.subjectTo( -0.5 <= T_act <= 0.5 );

    GnuplotWindow window;
    window.addSubplot( n,   "n [m]" );
    window.addSubplot( psi,   "psi [rad]" );
    window.addSubplot( beta,   "beta [rad]" );
    window.addSubplot( Omega,   "Omega [rad/s]" );
    window.addSubplot( delta, "steering angle [rad]" );
    window.plot( );

    OptimizationAlgorithm algorithm(ocp);   // construct optimization algorithm,
    algorithm << window                 ;   // flush the plot window,


    algorithm.set( INTEGRATOR_TYPE      , INT_RK45        );
    algorithm.set( INTEGRATOR_TOLERANCE , 1e-8          );
    algorithm.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    algorithm.set( KKT_TOLERANCE        , 1e-9            );


    algorithm.solve()                   ;   // and solve the problem.

    return 0;
}