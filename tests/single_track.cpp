#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main(){
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

        const double    T = 0.1; // horizon step length
        const int       N = 10;   // prediction horizon steps

        const double t_sim_step = T;
        const double t_sim_total = 1;


        // DEFINE THE MODEL EQUATIONS:
        // ----------------------------------------------------------
        DifferentialEquation f;
        f << dot(Omega) == -2*(lf*lf*Kf+lr*lr*Kr)/(I*V)*Omega -2*(lf*Kf-lr*Kr)/I*beta + 2*lf*Kf/I*delta;
        f << dot(beta)  == -(1+2*(lf*Kf-lr*Kr)/(m*V*V))*Omega -2*(Kf+Kr)/(m*V)*beta   + 2*Kf/(m*V)*delta;

        // DEFINE AN OPTIMAL CONTROL PROBLEM
        // ----------------------------------
        Function h;
        h << Omega;
        h << beta;
        h << delta;

        DMatrix Q(3,3); // LSQ coefficient matrix
        Q(0,0) = 0.5;
        Q(1,1) = 1;
        Q(2,2) = 1;

        // reference
        DVector ref(3);
        ref.setAll( 0.0 );

        // define the prediction horizon
        const double tStart = 0.0;
        const double tEnd   = N*T;

        // optimal control problem
        OCP ocp( tStart, tEnd, N );
        ocp.minimizeLSQ( Q, h, ref );
        ocp.subjectTo( f );
        ocp.subjectTo( -0.5 <= delta <= 0.5 );

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

        DVector x0(2);
        x0.setZero();
        x0(0) = 1.0;
        x0(1) = 0.;

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
        window.addSubplot( diffStates(0),   "Omega [rad/s]" );
        window.addSubplot( diffStates(1),   "beta [rad]" );
        window.addSubplot( feedbackControl(0), "steering angle [rad]" );
        window.plot( );

    return 0;
}