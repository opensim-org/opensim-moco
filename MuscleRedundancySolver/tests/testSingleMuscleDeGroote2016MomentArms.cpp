#include <OpenSim/OpenSim.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>
#include <MuscleRedundancySolver.h>
#include <GlobalStaticOptimizationSolver.h>
#include <DeGroote2016Muscle.h>
#include <mesh.h>

#include "testing.h"

using namespace OpenSim;

/// Use De Groote's 2016 muscle model (outside of OpenSim) to solve a
/// trajectory optimization problem for a muscle's optimal length trajectory.
/// Then use an inverse solver to recover the original activation trajectory.
/// We do this for both GlobalStaticOptimizationSolver and
/// MuscleRedundancySolver. This is similar to
/// testSingleMuscleDeGroote2016MomentArms except this time we use a
/// rotational degree of freedom and we seek to make sure moment arms are
/// used properly in MuscleRedundancySolver.

// TODO test passive swing (force excitation/activation to be 0), and ensure
// the recovered activity is nearly zero.

/// Lift a muscle against gravity from a fixed starting state to a fixed end
/// position and velocity, in minimum time. Unlike in
/// testSingleMuscleDeGroote2016, in which the single degree of freedom is
/// translational, this DOF is rotational. This allows us to test the moment
/// arms calculations.
///
///                            |< --- d --->|
///                    ----------------------
///                            .\         /
///                            . \ d     .
///                            .  \     / muscle
///                            . q \   .
///                            .    \ /
///                                  O mass
///
/// The problem is to lift the mass from q = 0 to q = 45 degrees. The length
/// of the pendulum is d.
/// Here's a sketch of the problem we solve, with activation and fiber dynamics.
/// @verbatim
///   minimize   t_f
///   subject to qdot = u             kinematics
///              udot = -g/(d sin(q)) + r f_t / (m d^2) dynamics; r: moment arm
///              adot = f_a(e, a)     activation dynamics
///              lmdot = vmdot        fiber dynamics
///              (a f_l(lm) f_v(vm) + f_p(lm)) cos(alpha) = f_t(lt) equilibrium
///              q(0) = 0
///              u(0) = 0
///              a(0) = 0
///              vm(0) = 0
///              q(t_f) = pi/4
///              u(t_f) = 0
/// @endverbatim
/// This class can also solve the problem without muscle dynamics.
class DeGroote2016MuscleLiftMinTime
        : public mesh::OptimalControlProblemNamed<adouble> {
public:
    using T = adouble;
    const double g = 9.81;
    const double d = 0.2;
    const double mass = 0.5;
    const double max_isometric_force = 30;
    // I've chosen these two lengths to sum close to d*sqrt(2), so that,
    // initially, the muscle has no passive force. The optimal fiber length
    // is actually a little longer so that the initial norm_fiber_length is
    // less than 1 (for no particular reason other than diversity in tests).
    const double optimal_fiber_length = 0.23;
    const double tendon_slack_length = 0.07;
    const double pennation_angle_at_optimal = 0.15;
    // optimal fiber lengths per second:
    const double max_contraction_velocity = 10;

    DeGroote2016MuscleLiftMinTime(bool muscleDynamics) :
            mesh::OptimalControlProblemNamed<T>("hanging_muscle_min_time"),
            m_muscleDynamics(muscleDynamics) {
        this->set_time(0, {0.01, 1.0});
        this->add_state("angle", {-SimTK::Pi/2, SimTK::Pi/2}, 0, SimTK::Pi/4);
        this->add_state("speed", {-5, 10}, 0, 0);
        if (muscleDynamics) {
            this->add_state("activation", {0, 1}, 0);
            this->add_state("norm_fiber_length", {0.2, 1.8});
            this->add_control("excitation", {0, 1});
            this->add_control("norm_fiber_velocity", {-1, 1}, 0);
            this->add_path_constraint("fiber_equilibrium", 0);
        } else {
            this->add_control("activation", {0, 1});
        }
        m_muscle = DeGroote2016Muscle<T>(
                max_isometric_force, optimal_fiber_length, tendon_slack_length,
                pennation_angle_at_optimal, max_contraction_velocity);
    }
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // Unpack variables.
        // -----------------
        const T& angle = states[0];
        const T& speed = states[1];
        const T& activation = m_muscleDynamics ? states[2] : controls[0];

        // Multibody kinematics.
        // ---------------------
        derivatives[0] = speed;

        // Multibody dynamics.
        // -------------------
        // T = I alpha ->
        // -mgd sin(q) - r f_t = md^2 qdd

        // Muscle-tendon length.
        // Law of cosines: lMT = sqrt(d^2 + d^2 - 2d*d cos(pi/2 - q))
        const T intAngle = SimTK::Pi / 2 - angle;
        const T musTenLen = d * sqrt(2 * (1 - cos(intAngle)));

        // Moment arm.
        // r = d(lMT)/dq = -d^2 sin(pi/2-q) / sqrt(2d^2 (1 - cos(pi/2-q)))
        const T momArm = -d * sin(intAngle) / sqrt(2 * (1 - cos(intAngle)));

        // Tendon force.
        T tendonForce;
        if (m_muscleDynamics) {
            const T& normFibLen = states[3];
            m_muscle.calcTendonForce(musTenLen, normFibLen, tendonForce);
        } else {
            // Muscle-tendon velocity.
            // d(lMT)/dt = d(lMT)/dq * dq/dt
            const T musTenVel = momArm * speed;
            tendonForce = m_muscle.calcRigidTendonFiberForceAlongTendon(
                    activation, musTenLen, musTenVel);
        }

        derivatives[1] = -g / d * sin(angle)                      // gravity
                       - momArm * tendonForce / mass / pow(d, 2); // muscle

        if (m_muscleDynamics) {
            // Activation dynamics.
            // --------------------
            const T& excitation = controls[0];
            m_muscle.calcActivationDynamics(excitation, activation,
                                            derivatives[2]);

            // Fiber dynamics.
            // ---------------
            const T& normFibVel = controls[1];
            derivatives[3] = max_contraction_velocity * normFibVel;
        }
    }
    void path_constraints(unsigned /*i_mesh*/,
                          const T& /*time*/,
                          const mesh::VectorX<T>& states,
                          const mesh::VectorX<T>& controls,
                          Eigen::Ref<mesh::VectorX<T>> constraints)
    const override {
        if (m_muscleDynamics) {
            // Unpack variables.
            const T& angle = states[0];
            const T& activation = states[2];
            const T& normFibLen = states[3];
            const T& normFibVel = controls[1];

            // Muscle-tendon length.
            // Law of cosines: lMT = sqrt(d^2 + d^2 - 2d*d cos(pi/2 - q))
            // Duplicated from dynamics().
            const T intAngle = SimTK::Pi / 2 - angle;
            const T musTenLen = d * sqrt(2 * (1 - cos(intAngle)));

            m_muscle.calcEquilibriumResidual(
                    activation, musTenLen, normFibLen, normFibVel,
                    constraints[0]);
        }
    }
    void endpoint_cost(const T& final_time,
                       const mesh::VectorX<T>& /*final_states*/,
                       T& cost) const override {
        cost = final_time;
    }
private:
    DeGroote2016Muscle<T> m_muscle;
    bool m_muscleDynamics;
};

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryGlobalStaticOptimizationSolver() {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<DeGroote2016MuscleLiftMinTime>(false);
    ocp->print_description();
    const int N = 100;
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    // Create an initial guess.
    using Eigen::RowVectorXd;
    mesh::OptimalControlIterate guess;
    guess.time.setLinSpaced(N, 0, 0.26);
    ocp->set_state_guess(guess, "angle",
                         RowVectorXd::LinSpaced(N, 0, SimTK::Pi/4));
    ocp->set_state_guess(guess, "speed", RowVectorXd::LinSpaced(N, 6, 0));
    ocp->set_control_guess(guess, "activation",
                           RowVectorXd::LinSpaced(N, 1.0, 0));

    mesh::OptimalControlSolution ocp_solution = dircol.solve(guess);
    std::string trajectoryFile =
            "testSingleMuscleDeGroote2016MomentArms_GSO_trajectory.csv";
    ocp_solution.write(trajectoryFile);

    // Save the trajectory with a header so that OpenSim can read it.
    // --------------------------------------------------------------
    // CSVFileAdapter expects an "endheader" line in the file.
    auto fRead = std::ifstream(trajectoryFile);
    std::string trajFileWithHeader = trajectoryFile;
    trajFileWithHeader.replace(trajectoryFile.rfind(".csv"), 4,
                               "_with_header.csv");
    auto fWrite = std::ofstream(trajFileWithHeader);
    fWrite << "endheader" << std::endl;
    std::string line;
    while (std::getline(fRead, line)) fWrite << line << std::endl;
    fRead.close(); fWrite.close();

    // Create a table containing only the angle and speed of the pendulum.
    TimeSeriesTable ocpSolution = CSVFileAdapter::read(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"joint/flexion/value",
                                "joint/flexion/speed"});
    const auto& position = ocpSolution.getDependentColumn("angle");
    const auto& speed = ocpSolution.getDependentColumn("speed");
    for (size_t iRow = 0; iRow < ocpSolution.getNumRows(); ++iRow) {
        SimTK::RowVector row(2);
        row[0] = position[iRow];
        row[1] = speed[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }

    // Compute actual inverse dynamics moment, for debugging.
    // ------------------------------------------------------
    // TimeSeriesTable actualInvDyn;
    // actualInvDyn.setColumnLabels({"inverse_dynamics"});
    // DeGroote2016Muscle<double> muscle(ocp->max_isometric_force,
    //                                   ocp->optimal_fiber_length,
    //                                   ocp->tendon_slack_length,
    //                                   ocp->pennation_angle_at_optimal,
    //                                   ocp->max_contraction_velocity);
    // for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
    //     const auto& angle = ocp_solution.states(0, iTime);
    //     const auto& speed = ocp_solution.states(1, iTime);
    //     const auto& activation = ocp_solution.controls(0, iTime);
    //     const auto intAngle = SimTK::Pi / 2 - angle;
    //     const auto musTenLen = ocp->d * sqrt(2 * (1 - cos(intAngle)));
    //     const auto momArm = -ocp->d * sin(intAngle) /
    //             sqrt(2 * (1 - cos(intAngle)));
    //     const auto musTenVel = momArm * speed;
    //     const auto tendonForce = muscle.calcRigidTendonFiberForceAlongTendon(
    //             activation, musTenLen, musTenVel);
    //     actualInvDyn.appendRow(ocp_solution.time(iTime),
    //                            SimTK::RowVector(1, -momArm * tendonForce));
    // }
    // CSVFileAdapter::write(actualInvDyn,
    //     "DEBUG_testLiftingMassMomentArms_GSO_actualInvDyn.csv");

    return {ocpSolution, kinematics};
}

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryMuscleRedundancySolver() {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<DeGroote2016MuscleLiftMinTime>(true);
    ocp->print_description();
    const int N = 100;
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    // Create an initial guess.
    using Eigen::RowVectorXd;
    mesh::OptimalControlIterate guess;
    guess.time.setLinSpaced(N, 0, 0.3);
    ocp->set_state_guess(guess, "angle",
                         RowVectorXd::LinSpaced(N, 0, SimTK::Pi/4));
    ocp->set_state_guess(guess, "speed", RowVectorXd::Constant(N, 2.0));
    ocp->set_state_guess(guess, "activation",
                         RowVectorXd::LinSpaced(N, 0.8, 0));
    ocp->set_state_guess(guess, "norm_fiber_length",
                         RowVectorXd::LinSpaced(N, 0.9, 0.4));
    ocp->set_control_guess(guess, "excitation",
                           RowVectorXd::LinSpaced(N, 1.0, 0));
    ocp->set_control_guess(guess, "norm_fiber_velocity",
                           RowVectorXd::LinSpaced(N, -0.3, 0));

    mesh::OptimalControlSolution ocp_solution = dircol.solve(guess);

    std::string trajectoryFile =
            "testSingleMuscleDeGroote2016MomentArms_MRS_trajectory.csv";
    ocp_solution.write(trajectoryFile);

    // Save the trajectory with a header so that OpenSim can read it.
    // --------------------------------------------------------------
    // CSVFileAdapter expects an "endheader" line in the file.
    auto fRead = std::ifstream(trajectoryFile);
    std::string trajFileWithHeader = trajectoryFile;
    trajFileWithHeader.replace(trajectoryFile.rfind(".csv"), 4,
                               "_with_header.csv");
    auto fWrite = std::ofstream(trajFileWithHeader);
    fWrite << "endheader" << std::endl;
    std::string line;
    while (std::getline(fRead, line)) fWrite << line << std::endl;
    fRead.close();
    fWrite.close();

    // Create a table containing only the angle and speed of the pendulum.
    TimeSeriesTable ocpSolution = CSVFileAdapter::read(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"joint/flexion/value",
                                "joint/flexion/speed"});
    const auto& position = ocpSolution.getDependentColumn("angle");
    const auto& speed = ocpSolution.getDependentColumn("speed");
    for (size_t iRow = 0; iRow < ocpSolution.getNumRows(); ++iRow) {
        SimTK::RowVector row(2);
        row[0] = position[iRow];
        row[1] = speed[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }

    // Compute actual inverse dynamics moment, for debugging.
    // ------------------------------------------------------
    // TimeSeriesTable actualInvDyn;
    // actualInvDyn.setColumnLabels({"inverse_dynamics"});
    // DeGroote2016Muscle<double> muscle(ocp->max_isometric_force,
    //                                   ocp->optimal_fiber_length,
    //                                   ocp->tendon_slack_length,
    //                                   ocp->pennation_angle_at_optimal,
    //                                   ocp->max_contraction_velocity);
    // for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
    //     const auto& angle = ocp_solution.states(0, iTime);
    //     const auto& normFibLen = ocp_solution.states(3, iTime);
    //     const auto intAngle = SimTK::Pi / 2 - angle;
    //     const auto musTenLen = ocp->d * sqrt(2 * (1 - cos(intAngle)));
    //     const auto momArm = -ocp->d * sin(intAngle) /
    //             sqrt(2 * (1 - cos(intAngle)));
    //     double tendonForce;
    //     muscle.calcTendonForce(musTenLen, normFibLen, tendonForce);
    //     actualInvDyn.appendRow(ocp_solution.time(iTime),
    //                            SimTK::RowVector(1, -momArm * tendonForce));
    // }
    // CSVFileAdapter::write(actualInvDyn,
    //     "DEBUG_testLiftingMassMomentArms_MRS_actualInvDyn.csv");

    return {ocpSolution, kinematics};
}

OpenSim::Model buildLiftingMassModel() {
    using SimTK::Vec3;

    DeGroote2016MuscleLiftMinTime ocp(true);
    Model model;
    //model.setUseVisualizer(true);
    model.setName("hanging_muscle");

    // First, we need to build a similar OpenSim model.
    model.set_gravity(Vec3(0, -ocp.g, 0));
    auto* body = new Body("body", ocp.mass, Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // The body is translated down from this frame by d.
    auto* frame = new PhysicalOffsetFrame("offset", *body, Vec3(0, ocp.d, 0));
    model.addComponent(frame);

    // Allows translation along x.
    auto* joint = new PinJoint();
    joint->setName("joint");
    joint->connectSocket_parent_frame(model.getGround());
    joint->connectSocket_child_frame(*frame);
    auto& coord = joint->updCoordinate(PinJoint::Coord::RotationZ);
    coord.setName("flexion");
    model.addComponent(joint);

    auto* actu = new Millard2012EquilibriumMuscle();
    actu->setName("actuator");
    actu->set_max_isometric_force(ocp.max_isometric_force);
    actu->set_optimal_fiber_length(ocp.optimal_fiber_length);
    actu->set_tendon_slack_length(ocp.tendon_slack_length);
    actu->set_pennation_angle_at_optimal(ocp.pennation_angle_at_optimal);
    // Attached the ground to the right of the pendulum.
    actu->addNewPathPoint("origin", model.updGround(), Vec3(ocp.d, 0, 0));
    // Attached to the mass at the end of the pendulum.
    actu->addNewPathPoint("insertion", *body, Vec3(0));
    model.addComponent(actu);

    //SimTK::State s = model.initSystem();
    //model.updMatterSubsystem().setShowDefaultGeometry(true);
    //model.updVisualizer().updSimbodyVisualizer().setBackgroundType
    //        (SimTK::Visualizer::GroundAndSky);
    //model.getVisualizer().show(s);
    //std::cin.get();
    //Manager manager(model);
    //manager.integrate(s, 1.0);
    return model;
}

// Reproduce the trajectory (generated without muscle dynamics) using static
// optimization.
void testLiftingMassGlobalStaticOptimizationSolver(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Build a similar OpenSim model.
    // ------------------------------
    Model model = buildLiftingMassModel();
    model.finalizeFromProperties();

    // Create the MuscleRedundancySolver.
    // ----------------------------------
    GlobalStaticOptimizationSolver mrs;
    mrs.setModel(model);
    mrs.setKinematicsData(kinematics);
    double reserveOptimalForce = 0.001;
    mrs.set_create_reserve_actuators(reserveOptimalForce);
    GlobalStaticOptimizationSolver::Solution solution = mrs.solve();
    solution.write("testSingleMuscleDeGroote2016MomentArms_GSO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    rootMeanSquare(solution.activation, "/hanging_muscle/actuator", ocpSolution, "activation", 0.03);
    auto reserveForceRMS = reserveOptimalForce *
         solution.other_controls.getDependentColumnAtIndex(0).normRMS();
    SimTK_TEST(reserveForceRMS < 0.03);
}

// Reproduce the trajectory (generated with muscle dynamics) using the
// MuscleRedundancySolver.
void testLiftingMassMuscleRedundancySolver(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Build a similar OpenSim model.
    // ------------------------------
    Model model = buildLiftingMassModel();
    model.finalizeFromProperties();

    // Create the MuscleRedundancySolver.
    // ----------------------------------
    MuscleRedundancySolver mrs;
    mrs.setModel(model);
    mrs.setKinematicsData(kinematics);
    // Without filtering, the moments have high frequency content,
    // probably related to unfiltered generalized coordinates and getting
    // accelerations from a spline fit.
    mrs.set_create_reserve_actuators(0.001);
    MuscleRedundancySolver::Solution solution = mrs.solve();
    solution.write("testSingleMuscleDeGroote2016MomentArms_MRS");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    compare(solution.activation, "/hanging_muscle/actuator", ocpSolution, "activation", 0.05);
    compare(solution.norm_fiber_length, "/hanging_muscle/actuator", ocpSolution, "norm_fiber_length",
            0.005);

    // We use a weaker check for the controls; they don't match as well.
    rootMeanSquare(solution.excitation, "/hanging_muscle/actuator", ocpSolution, "excitation", 0.08);
    rootMeanSquare(solution.norm_fiber_velocity, "/hanging_muscle/actuator", ocpSolution,
                  "norm_fiber_velocity", 0.04);
}

int main() {
    SimTK_START_TEST("testSingleMuscleDeGroote2016MomentArms");
        {
            auto gsoData = solveForTrajectoryGlobalStaticOptimizationSolver();
            SimTK_SUBTEST1(testLiftingMassGlobalStaticOptimizationSolver,
                           gsoData);
        }
        {
            auto mrsData = solveForTrajectoryMuscleRedundancySolver();
            SimTK_SUBTEST1(testLiftingMassMuscleRedundancySolver, mrsData);
        }
    SimTK_END_TEST();
}
