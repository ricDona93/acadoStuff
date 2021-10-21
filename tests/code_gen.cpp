#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>

int main( )
{
    USING_NAMESPACE_ACADO
    // DEFINE THE VARIABLES:
    // ----------------------------------------------------------
    DifferentialState   Omega;
    DifferentialState   beta;

    Control             delta;      // control steering angle

    const double  V = 10.0;       // reference velocity

    const double lf = 1.1;
    const double lr = 1.6;
    const double Kr = 60000;
    const double Kf = 55000;
    const double  m = 1500;
    const double  I = 2500;

    const double  T = 0.1; // horizon step length
    const int     N = 10;   // prediction horizon steps

    const double t_sim_step = T;
    const double t_sim_total = 1;

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    DifferentialEquation f;
    f << dot(Omega) == -2*(lf*lf*Kf+lr*lr*Kr)/(I*V)*Omega -2*(lf*Kf-lr*Kr)/I*beta + 2*lf*Kf/I*delta;
    f << dot(beta)  == -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta;

    // DEFINE AN OPTIMAL CONTROL PROBLEM
    // ----------------------------------
    Function h, hN;
    h << Omega;
    h << beta;
    h << delta;
    hN << Omega << beta;

    DMatrix W(3,3); // LSQ coefficient matrix
    W(0,0) = 0.5;
    W(1,1) = 1;
    W(2,2) = 1;

    // Provide defined weighting matrices:
    DMatrix WN = eye<double>( hN.getDim() );
    WN *= 0.001;

    OCP ocp(0.0, 1.0, N);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    ocp.subjectTo( -0.5 <= delta <= 0.5 );

    // Export the code:
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
    mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
    mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
    mpc.set( NUM_INTEGRATOR_STEPS,        10              );

    mpc.set( QP_SOLVER,                   QP_QPOASES3);
// 	mpc.set( HOTSTART_QP,                 YES             );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
    mpc.set( GENERATE_TEST_FILE,          YES             );
    mpc.set( GENERATE_MAKE_FILE,          YES             );
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );

// 	mpc.set( USE_SINGLE_PRECISION,        YES             );

    if (mpc.exportCode( "out" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}
