#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main(){
    USING_NAMESPACE_ACADO
    // DEFINE THE VARIABLES:
    // ----------------------------------------------------------
    DifferentialState   beta;
    DifferentialState   Omega;
    DifferentialState   delta;
    DifferentialState   D_delta;

    Control             T_act;      // control steering angle

    // vehicle actuator parameters
    const double A_veh_11 = -30.86;
    const double A_veh_12 = -1.358;
    const double A_veh_21 = 382.4;
    const double A_veh_22 = 12.1;
    const double B_veh_1  = 0.1136;
    const double B_veh_2  = 0.1857;

    // steering actuator parameters
    const double A_act_21 = -0.1924;
    const double A_act_22 = -133.8;
    const double B_act_1  = -3.985;
    const double B_act_2  = 655;

    const double    T = 0.05; // horizon step length
    const int       N = 20;   // prediction horizon steps

    const double t_sim_step = T;
    const double t_sim_total = 1;


    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    DifferentialEquation f;
    f << dot(beta)  == A_veh_11*beta + A_veh_12*Omega + B_veh_1*delta;
    f << dot(Omega) == A_veh_21*beta + A_veh_22*Omega + B_veh_2*delta;
    f << dot(delta) == D_delta + B_act_1*T_act;
    f << dot(D_delta) == A_act_21*delta + A_act_22*D_delta + B_act_2*T_act;

    // DEFINE AN OPTIMAL CONTROL PROBLEM
    // ----------------------------------
    Function h;
    h << beta;
    h << Omega;
    h << delta;
    h << D_delta;
    h << T_act;

    DMatrix Q(5,5); // LSQ coefficient matrix
    Q(0,0) = 5;
    Q(1,1) = 1;
    Q(2,2) = 1;
    Q(3,3) = 0.01;
    Q(4,4) = 0.001;

    // reference
    DVector ref(5);
    ref.setAll( 0.0 );

    // define the prediction horizon
    const double tStart = 0.0;
    const double tEnd   = N*T;

    // optimal control problem
    OCP ocp( tStart, tEnd, N );
    ocp.minimizeLSQ( Q, h, ref );
    ocp.subjectTo( f );
    ocp.subjectTo( -0.5 <= T_act <= 0.5 );

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
    RealTimeAlgorithm alg( ocp,t_sim_step );

    StaticReferenceTrajectory zeroReference;
    Controller controller( alg,zeroReference );

    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
    OutputFcn identity;
    DynamicSystem dynamicSystem( f,identity );

    Process process( dynamicSystem,INT_RK45 );

    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
    SimulationEnvironment sim( 0.0,t_sim_total,process,controller );

    DVector x0(4);
    x0.setZero();
    x0(0) = 0.0;
    x0(1) = 0.5;
    x0(2) = -0.01;
    x0(3) = 0.0;

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
    window.addSubplot( diffStates(0),   "beta [rad]" );
    window.addSubplot( diffStates(1),   "Omega [rad/s]" );
    window.addSubplot( diffStates(2),   "steering angle [rad]" );
    window.addSubplot( diffStates(3),   "steering velocity [rad/s]" );
    window.addSubplot( feedbackControl(0), "steering torque [Nm]" );
    window.plot( );

    return 0;
}