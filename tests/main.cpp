#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( )
{
    USING_NAMESPACE_ACADO
    // DEFINE THE VARIABLES:
    // ----------------------------------------------------------
    DifferentialState   x;          // position in x-direction
    DifferentialState   y;          // position in y-direction
    DifferentialState   theta;      // robot orientation

    Control             V;          // control commands
    Control             Omega;      // control commands

    const double    T = 0.25; // horizon step length
    const int       N = 20; // prediction horizon steps

    const double t_sim_step = T;
    const double t_sim_total = 15;

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    DiscretizedDifferentialEquation f(T);
    f << dot( x     )  == x+ V*cos(theta)*T;
    f << dot( y     )  == y+ V*sin(theta)*T;
    f << dot( theta )  == theta+Omega*T;

    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
    OutputFcn identity;
    DynamicSystem dynamicSystem( f,identity );
    Process process( dynamicSystem, INT_RK45);

    // DEFINE AN OPTIMAL CONTROL PROBLEM
    // ----------------------------------
    Function h;
    h << x;
    h << y;
    h << theta;
    h << V;
    h << Omega;

    DMatrix Q(5,5); // LSQ coefficient matrix
    Q(0,0) = 0.2;
    Q(1,1) = 2;
    Q(2,2) = 0.2;
    Q(3,3) = 2;
    Q(4,4) = 2.5;

    // reference 
    DVector ref(5);
    ref.setAll( 0.0 );
    ref(1) = -1.0;

    // define the prediction horizon 
    const double tStart = 0.0;
    const double tEnd   = N*T;

    // optimal control problem
    OCP ocp( tStart, tEnd, N );
    ocp.minimizeLSQ( Q, h, ref );
    ocp.subjectTo( f );
    ocp.subjectTo( -0.5 <= V <= 0.5 );
    ocp.subjectTo( -0.8 <= Omega <= 0.8 );

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
    RealTimeAlgorithm alg( ocp,t_sim_step );

    StaticReferenceTrajectory zeroReference;
    Controller controller( alg,zeroReference );

    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
    SimulationEnvironment sim( 0.0,t_sim_total,process,controller );

    DVector x0(3);
    x0.setZero();
    x0(0) = 2;
    x0(1) = 0;
    x0(2) = 3.14;

    sim.init( x0 );
    sim.run( );

    // ... AND PLOT THE RESULTS
    // ------------------------
    VariablesGrid diffStates;
    sim.getProcessDifferentialStates( diffStates );

    VariablesGrid feedbackControl;
    sim.getFeedbackControl( feedbackControl );

    diffStates.print();

    GnuplotWindow window;
    window.addSubplot( diffStates(0),   "x [m]" );
    window.addSubplot( diffStates(1),   "y [m]" );
    window.addSubplot( diffStates(2),   "theta [rad]" );
    window.addSubplot( feedbackControl(0), "lin vel [m/s]" );
    window.addSubplot( feedbackControl(1), "ang vel [rad/s]" );
    window.plot( );
    
    return 0;
}