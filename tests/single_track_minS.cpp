#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( )
{
    USING_NAMESPACE_ACADO

    //TODO find how to print KKT

    DifferentialState       n, s, V, psi, Omega, beta;
    IntermediateState       ay;
    Control                 delta, gas;
    Parameter               T          ;   // the time horizon T
    DifferentialEquation    f( 0.0, T );


    const double V0 = 30.0;       // reference velocity
    const double betaMax = 8e-2;
    const double deltaMax = 0.5;
    const double ayMax = 10;
    const double axMax = 10;
    const double lat_offs = 2.5;

    const double lf = 1.1;
    const double lr = 1.6;
    const double Kr = 60000;
    const double Kf = 55000;
    const double  m = 1500;
    const double  I = 2500;

    //-------------------------------------
    OCP ocp( 0.0, T )                   ;   // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T )          ;
    ocp.minimizeLagrangeTerm(0.5*delta*delta);

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    f << dot(s)     == V*cos(psi);
    f << dot(V)     == gas;
    f << dot(n)     == V*(psi+beta);
    f << dot(psi)   == Omega;
    f << dot(Omega) == -2*(lf*lf*Kf+lr*lr*Kr)/(I*V)*Omega -2*(lf*Kf-lr*Kr)/I*beta + 2*lf*Kf/I*delta;
    f << dot(beta)  == -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta;

    ay = -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta + Omega*V;
    
    // DEFINE AN OPTIMAL CONTROL PROBLEM
    // ----------------------------------

    ocp.subjectTo( f );   // minimize T s.t. the model,
    ocp.subjectTo( AT_START, n ==  0.0 );
    ocp.subjectTo( AT_START, s ==  0.0 );
    ocp.subjectTo( AT_START, V ==  V0 );
    ocp.subjectTo( AT_START, psi ==  0.01 );
    ocp.subjectTo( AT_START, beta ==  0.0 );
    ocp.subjectTo( AT_START, Omega == 0.01 );

    ocp.subjectTo( AT_END  , n == lat_offs );
    ocp.subjectTo( AT_END  , psi ==  0.0 );
    ocp.subjectTo( AT_END  , Omega ==  0.0 );
    ocp.subjectTo( AT_END  , beta ==  0.0 );
    ocp.subjectTo( AT_END  , s ==  100.0 );

    ocp.subjectTo( -betaMax <= beta <=  betaMax   );
    ocp.subjectTo( -ayMax <= ay <=  ayMax  );
    ocp.subjectTo( -deltaMax <= delta <= deltaMax );
    ocp.subjectTo( 0 <= V <= V0*1.5 );
    ocp.subjectTo( -axMax <= gas <= axMax/3 );

    GnuplotWindow window;
    window.addSubplot( n,   "n [m]" );
    window.addSubplot( psi,   "psi [rad]" );
    window.addSubplot( beta,   "beta [rad]" );
    window.addSubplot( s,   "s [m]" );
    window.addSubplot( V,   "V [m/s]" );
    window.addSubplot( ay, "Lateral acceleration [m/s^2]" );
    window.addSubplot( Omega,   "Omega [rad/s]" );
    window.addSubplot( delta, "steering angle [rad]" );
    window.addSubplot( gas, "gas [m/s2]" );
    window.plot( );



    OptimizationAlgorithm algorithm(ocp);   // construct optimization algorithm,
    algorithm << window                 ;   // flush the plot window,


//    algorithm.set( INTEGRATOR_TYPE      , INT_RK45        );
//    algorithm.set( INTEGRATOR_TOLERANCE , 1e-10          );
//    algorithm.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    algorithm.set( KKT_TOLERANCE        , 1e-14            );

    algorithm.solve()                   ;   // and solve the problem.

    VariablesGrid params;
    algorithm.getParameters( params );

    double tF = double(params(0,0));

    printf("\n------------- Minimum time for OCP ------------\n");
    printf("T  =  %.3e (s)", tF );
    printf("\n-----------------------------------------------\n");

    return 0;
}