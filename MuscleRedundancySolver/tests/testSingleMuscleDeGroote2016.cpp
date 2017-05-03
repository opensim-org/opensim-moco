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
/// MuscleRedundancySolver.

// TODO try improving solution time by first solving a static optimization
// problem.

// TODO test passive swing (force excitation/activation to be 0), and ensure
// the recovered activity is nearly zero.


/// Lift a muscle against gravity from a fixed starting state to a fixed end
/// position and velocity, in minimum time.
/// Here's a sketch of the problem we solve, without muscle dynamics:
/// @verbatim
///   minimize   t_f
///   subject to qdot = u             kinematics
///              udot = (mg - f_t)/m  dynamics
///              f_t = (a f_l(lm) f_v(vm) + f_p(lm)) cos(alpha)
///              q(0) = 0.2
///              u(0) = 0
///              q(t_f) = 0.15
///              u(t_f) = 0
/// @endverbatim
/// where lm and vm are determined from the muscle-tendon length and velocity
/// with the assumption of a rigid tendon.
class DeGroote2016MuscleLiftMinTimeStatic
        : public mesh::OptimalControlProblemNamed<adouble> {
public:
    using T = adouble;
    const double g = 9.81;
    const double mass = 0.5;
    // TODO move to a common place.
    const double max_isometric_force = 30;
    const double optimal_fiber_length = 0.10;
    const double tendon_slack_length = 0.05;
    const double pennation_angle_at_optimal = 0.1;
    // optimal fiber lengths per second:
    const double max_contraction_velocity = 10;

    DeGroote2016MuscleLiftMinTimeStatic() :
            mesh::OptimalControlProblemNamed<T>("hanging_muscle_min_time") {
        this->set_time(0, {0.01, 1.0});
        // TODO these functions should return indices for these variables.
        this->add_state("position", {0, 0.3}, 0.15, 0.10);
        this->add_state("speed", {-10, 10}, 0, 0);
        this->add_control("activation", {0, 1});
        // TODO move this to a constructor parameter.
        m_muscle = DeGroote2016Muscle<T>(
                max_isometric_force, optimal_fiber_length, tendon_slack_length,
                pennation_angle_at_optimal, max_contraction_velocity);
    }
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // Unpack variables.
        const T& position = states[0];
        const T& speed = states[1];
        const T& activation = controls[0];

        // Multibody kinematics.
        derivatives[0] = speed;

        // Multibody dynamics.
        const T tendonForce =
                m_muscle.calcRigidTendonFiberForceAlongTendon(activation,
                                                              position, speed);
        derivatives[1] = g - tendonForce / mass;
    }
    void endpoint_cost(const T& final_time,
                       const mesh::VectorX<T>& /*final_states*/,
                       T& cost) const override {
        cost = final_time;
    }
private:
    DeGroote2016Muscle<T> m_muscle;
};

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryGlobalStaticOptimizationSolver() {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<DeGroote2016MuscleLiftMinTimeStatic>();
    ocp->print_description();
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", 100);
    mesh::OptimalControlSolution ocp_solution = dircol.solve();
    std::string trajectoryFile =
            "testSingleMuscleDeGroote2016_GSO_trajectory.csv";
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

    // Create a table containing only the position and speed of the mass.
    TimeSeriesTable ocpSolution = CSVFileAdapter::read(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"joint/height/value",
                                "joint/height/speed"});
    const auto& position = ocpSolution.getDependentColumn("position");
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
	//     const auto& position = ocp_solution.states(0, iTime);
	//     const auto& speed = ocp_solution.states(1, iTime);
	//     const auto& activation = ocp_solution.controls(0, iTime);
	//     const auto normTendonForce =
	//             muscle.calcRigidTendonNormFiberForceAlongTendon(activation,
	//                                                             position,
	//                                                             speed);
	//     const auto tendonForce = muscle.get_max_isometric_force()
	//                            * normTendonForce;
	//     actualInvDyn.appendRow(ocp_solution.time(iTime),
	//                            SimTK::RowVector(1, -tendonForce));
	// }
	// CSVFileAdapter::write(actualInvDyn,
	//                       "DEBUG_testLiftingMass_GSO_actualInvDyn.csv");

    // TODO add regression test!!!

    return {ocpSolution, kinematics};
}

/// Lift a muscle against gravity from a fixed starting state to a fixed end
/// position and velocity, in minimum time.
/// Here's a sketch of the problem we solve, with activation and fiber dynamics.
/// @verbatim
///   minimize   t_f
///   subject to qdot = u             kinematics
///              udot = (mg - f_t)/m  dynamics
///              adot = f_a(e, a)     activation dynamics
///              lmdot = vmdot        fiber dynamics
///              (a f_l(lm) f_v(vm) + f_p(lm)) cos(alpha) = f_t(lt) equilibrium
///              q(0) = 0.2
///              u(0) = 0
///              a(0) = 0
///              vm(0) = 0
///              q(t_f) = 0.15
///              u(t_f) = 0
/// @endverbatim
/// Making the initial fiber velocity 0 helps avoid a sharp spike in fiber
/// velocity at the beginning of the motion.
class DeGroote2016MuscleLiftMinTimeDynamic
        : public mesh::OptimalControlProblemNamed<adouble> {
public:
    using T = adouble;
    const double g = 9.81;
    const double mass = 0.5;
    const double max_isometric_force = 30;
    const double optimal_fiber_length = 0.10;
    const double tendon_slack_length = 0.05;
    const double pennation_angle_at_optimal = 0.1;
    // optimal fiber lengths per second:
    const double max_contraction_velocity = 10;

    DeGroote2016MuscleLiftMinTimeDynamic() :
            mesh::OptimalControlProblemNamed<T>("hanging_muscle_min_time") {
        this->set_time(0, {0.01, 1.0});
        // TODO these functions should return indices for these variables.
        this->add_state("position", {0, 0.3}, 0.15, 0.10);
        this->add_state("speed", {-10, 10}, 0, 0);
        this->add_state("activation", {0, 1}, 0);
        this->add_state("norm_fiber_length", {0.2, 1.8});
        this->add_control("excitation", {0, 1});
        this->add_control("norm_fiber_velocity", {-1, 1}, 0);
        this->add_path_constraint("fiber_equilibrium", 0);
        m_muscle = DeGroote2016Muscle<T>(
                max_isometric_force, optimal_fiber_length, tendon_slack_length,
                pennation_angle_at_optimal, max_contraction_velocity);
    }
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // Unpack variables.
        const T& position = states[0];
        const T& speed = states[1];
        const T& activation = states[2];
        const T& normFibLen = states[3];
        const T& excitation = controls[0];
        const T& normFibVel = controls[1];

        // Multibody kinematics.
        derivatives[0] = speed;

        // Multibody dynamics.
        T tendonForce;
        m_muscle.calcTendonForce(position, normFibLen, tendonForce);
        derivatives[1] = g - tendonForce / mass;

        // Activation dynamics.
        m_muscle.calcActivationDynamics(excitation, activation, derivatives[2]);

        // Fiber dynamics.
        derivatives[3] = max_contraction_velocity * normFibVel;
    }
    void path_constraints(unsigned /*i_mesh*/,
                          const T& /*time*/,
                          const mesh::VectorX<T>& states,
                          const mesh::VectorX<T>& controls,
                          Eigen::Ref<mesh::VectorX<T>> constraints)
    const override {
        // Unpack variables.
        // -----------------
        const T& position = states[0];
        const T& activation = states[2];
        const T& normFibLen = states[3];
        const T& normFibVel = controls[1];
        m_muscle.calcEquilibriumResidual(
                activation, position, normFibLen, normFibVel, constraints[0]);
    }
    void endpoint_cost(const T& final_time,
                       const mesh::VectorX<T>& /*final_states*/,
                       T& cost) const override {
        cost = final_time;
    }
private:
    DeGroote2016Muscle<T> m_muscle;
};

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryMuscleRedundancySolver() {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<DeGroote2016MuscleLiftMinTimeDynamic>();

    ocp->print_description();
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", 100);
    mesh::OptimalControlSolution ocp_solution = dircol.solve();
    std::string trajectoryFile =
            "testSingleMuscleDeGroote2016_MRS_trajectory.csv";
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

    // Create a table containing only the position and speed of the mass.
    TimeSeriesTable ocpSolution = CSVFileAdapter::read(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"joint/height/value",
                                "joint/height/speed"});
    const auto& position = ocpSolution.getDependentColumn("position");
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
    //     const auto& musTenLength = ocp_solution.states(0, iTime);
    //     const auto& normFiberLength = ocp_solution.states(3, iTime);
    //     double tendonForce;
    //     muscle.calcTendonForce(musTenLength, normFiberLength, tendonForce);
    //     actualInvDyn.appendRow(ocp_solution.time(iTime),
    //                            SimTK::RowVector(1, -tendonForce));
    // }
    // CSVFileAdapter::write(actualInvDyn,
    //                       "DEBUG_testLiftingMass_MRS_actualInvDyn.csv");

    // TODO add regression test!!!

    return {ocpSolution, kinematics};
}

OpenSim::Model buildLiftingMassModel() {
    DeGroote2016MuscleLiftMinTimeDynamic ocp;
    Model model;
    model.setName("hanging_muscle");

    // First, we need to build a similar OpenSim model.
    model.set_gravity(SimTK::Vec3(ocp.g, 0, 0));
    auto* body = new Body("body", ocp.mass,
                          SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint();
    joint->setName("joint");
    joint->connectSocket_parent_frame(model.getGround());
    joint->connectSocket_child_frame(*body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("height");
    model.addComponent(joint);

    auto* actu = new Millard2012EquilibriumMuscle();
    actu->setName("actuator");
    actu->set_max_isometric_force(ocp.max_isometric_force);
    actu->set_optimal_fiber_length(ocp.optimal_fiber_length);
    actu->set_tendon_slack_length(ocp.tendon_slack_length);
    actu->set_pennation_angle_at_optimal(ocp.pennation_angle_at_optimal);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(actu);
    return model;
}

void testLiftingMassGlobalStaticOptimizationSolver(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Build a similar OpenSim model.
    // ------------------------------
    Model model = buildLiftingMassModel();
    model.finalizeFromProperties();

    // Create the GlobalStaticOptimizationSolver.
    // ------------------------------------------
    GlobalStaticOptimizationSolver gso;
    gso.setModel(model);
    gso.setKinematicsData(kinematics);
    // Without filtering, the moments have high frequency content,
    // probably related to unfiltered generalized coordinates and getting
    // accelerations from a spline fit.
    // GSO requires a higher cutoff than MRS because the lack of activation
    // dynamics results in an abrupt change in force when the muscle turns off
    // instantaneously. I tried 80-300 Hz; higher than 100 Hz gives noisy
    // activation.
    gso.set_lowpass_cutoff_frequency_for_joint_moments(100);
    // TODO is the filtering necessary if we have reserve actuators?
    gso.set_create_reserve_actuators(0.001);
    GlobalStaticOptimizationSolver::Solution solution = gso.solve();
    solution.write("testSingleMuscleDeGroote2016_GSO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    // TODO the reserve actuators are used far more than they should be,
    // at the start of the motion and in the middle. This results from
    // filtering the higher-frequency parts of the inverse dynamics moments.

    // The rationale for the tolerances: as tight as they could be for the
    // test to pass.
    rootMeanSquare(solution.activation, ocpSolution, "activation", 0.05);
}

// Reproduce the trajectory using the MuscleRedundancy, without specifying an
// initial guess.
void testLiftingMassMuscleRedundancySolver(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        bool useStaticOptiGuess) {
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
    mrs.set_lowpass_cutoff_frequency_for_joint_moments(80);
    // TODO is the filtering necessary if we have reserve actuators?
    mrs.set_create_reserve_actuators(0.001);
    if (useStaticOptiGuess) {
        mrs.set_initial_guess("static_optimization");
    } else {
        mrs.set_initial_guess("bounds");
    }
    MuscleRedundancySolver::Solution solution = mrs.solve();
    solution.write("testSingleMuscleDeGroote2016_MRS");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    // TODO the reserve actuators are used far more than they should be,
    // at the start of the motion. This might be a result of the
    // filtering of inverse dynamics moments, combined with the spike
    // in fiber velocity from the initial trajectory optimization.

    // The states match better than the controls.
    // The rationale for the tolerances: as tight as they could be for the
    // test to pass.
    compare(solution.activation, ocpSolution, "activation", 0.04);
    compare(solution.norm_fiber_length, ocpSolution, "norm_fiber_length",
            0.01);

    // We use a weaker check for the controls; they don't match as well.
    rootMeanSquare(solution.excitation, ocpSolution, "excitation", 0.08);
    rootMeanSquare(solution.norm_fiber_velocity, ocpSolution,
                   "norm_fiber_velocity", 0.04);
}

int main() {
    SimTK_START_TEST("testSingleMuscleDeGroote2016");
        {
            auto gsoData = solveForTrajectoryGlobalStaticOptimizationSolver();
            SimTK_SUBTEST1(testLiftingMassGlobalStaticOptimizationSolver,
                           gsoData);
        }
        {
            auto mrsData = solveForTrajectoryMuscleRedundancySolver();
            // Without using static optimization to obtain an initial guess.
            SimTK_SUBTEST2(testLiftingMassMuscleRedundancySolver, mrsData,
                           false);
            // Use static optimization to obtain an initial guess; this
            // should take under 1 second, compare to 10 seconds for using
            // the more naive guess above.
            SimTK_SUBTEST2(testLiftingMassMuscleRedundancySolver, mrsData,
                           true);
        }
    SimTK_END_TEST();
}


// This is no longer used...it's just here for comparison and checking
// performance.
//class DeGroote2016MuscleTrajectoryOptimizationOrig
//        : public mesh::OptimalControlProblemNamed<adouble> {
//public:
//    using T = adouble;
//    const double g = 9.81;
//    const double mass = 0.5;
//    const double max_isometric_force = 30;
//    const double optimal_fiber_length = 0.10;
//    const double tendon_slack_length = 0.05;
//    const double pennation_angle_at_optimal = 0.1;
//    // optimal fiber lengths per second:
//    const double max_contraction_velocity = 10;
//
//    // Tendon force-length curve.
//    constexpr static const double kT = 35;
//    constexpr static const double c1 = 0.200;
//    constexpr static const double c2 = 0.995;
//    constexpr static const double c3 = 0.250;
//
//    DeGroote2016MuscleTrajectoryOptimizationOrig() :
//            mesh::OptimalControlProblemNamed<T>("hanging_muscle_min_time") {
//        // The motion occurs in 1 second.
//        this->set_time(0, {0.01, 1.0});
//        // TODO these functions should return indices for these variables.
//        this->add_state("position", {0, 0.3}, 0.15, 0.10);
//        this->add_state("speed", {-10, 10}, 0, 0);
//        this->add_state("activation", {0, 1}, 0);
//        this->add_state("norm_fiber_length", {0.2, 1.8});
//        this->add_control("excitation", {0, 1});
//        this->add_control("norm_fiber_velocity", {-1, 1});
//        this->add_path_constraint("fiber_equilibrium", 0);
//    }
//    void dynamics(const mesh::VectorX<T>& states,
//                  const mesh::VectorX<T>& controls,
//                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
//        // Unpack variables.
//        const T& position = states[0];
//        const T& speed = states[1];
//        const T& activation = states[2];
//        const T& normFibLen = states[3];
//        const T& excitation = controls[0];
//        const T& normFibVel = controls[1];
//
//        // Multibody kinematics.
//        derivatives[0] = speed;
//
//        // Multibody dynamics.
//        // TODO computing tendon force is why we'd want a single "continuous"
//        // function rather than separate dynamics and path constraints
//        // functions.
//        const T fibLen = normFibLen * optimal_fiber_length;
//        // TODO cache this somewhere; this is constant.
//        const double fibWidth = optimal_fiber_length
//                * sin(pennation_angle_at_optimal);
//        // Tendon length.
//        const T& musTenLen = position;
//        // lT = lMT - sqrt(lM^2 - w^2)
//        const T tenLen = musTenLen
//                - sqrt(fibLen*fibLen - fibWidth*fibWidth);
//        const T normTenLen = tenLen / tendon_slack_length;
//        const T normTenForce = c1 * exp(kT * (normTenLen - c2)) - c3;
//        const T tenForce = max_isometric_force * normTenForce;
//        derivatives[1] = g - tenForce / mass;
//
//        // Activation dynamics.
//        static const double actTimeConst   = 0.015;
//        static const double deactTimeConst = 0.060;
//        static const double tanhSteepness  = 0.1;
//        //     f = 0.5 tanh(b(e - a))
//        //     z = 0.5 + 1.5a
//        // da/dt = [(f + 0.5)/(tau_a * z) + (-f + 0.5)*z/tau_d] * (e - a)
//        const T timeConstFactor = 0.5 + 1.5 * activation;
//        const T tempAct = 1.0 / (actTimeConst * timeConstFactor);
//        const T tempDeact = timeConstFactor / deactTimeConst;
//        const T f = 0.5 * tanh(tanhSteepness * (excitation - activation));
//        const T timeConst = tempAct * (f + 0.5) + tempDeact * (-f + 0.5);
//        derivatives[2] = timeConst * (excitation - activation);
//
//        // Fiber dynamics.
//        derivatives[3] = max_contraction_velocity * normFibVel;
//    }
//    void path_constraints(unsigned /*i_mesh*/,
//                          const T& /*time*/,
//                          const mesh::VectorX<T>& states,
//                          const mesh::VectorX<T>& controls,
//                          Eigen::Ref<mesh::VectorX<T>> constraints)
//    const override {
//        // Fiber-tendon equilibrium.
//        // =========================
//        // Parameters.
//        // -----------
//        // Active force-length curve.
//        static const double b11 =  0.815;
//        static const double b21 =  1.055;
//        static const double b31 =  0.162;
//        static const double b41 =  0.063;
//        static const double b12 =  0.433;
//        static const double b22 =  0.717;
//        static const double b32 = -0.030;
//        static const double b42 =  0.200;
//        static const double b13 =  0.100;
//        static const double b23 =  1.000;
//        static const double b33 =  0.354;
//        static const double b43 =  0.000;
//
//        // Passive force-length curve.
//        static const double kPE = 4.0;
//        static const double e0  = 0.6;
//
//        // Muscle force-velocity.
//        static const double d1 = -0.318;
//        static const double d2 = -8.149;
//        static const double d3 = -0.374;
//        static const double d4 =  0.886;
//
//        auto gaussian = [](const T& x, const double& b1, const double& b2,
//                           const double& b3, const double& b4) -> T {
//            return b1 * exp((-0.5 * pow(x - b2, 2)) / (b3 + b4 * x));
//        };
//
//        // Unpack variables.
//        // -----------------
//        const T& position = states[0];
//        const T& activation = states[2];
//        const T& normFibLen = states[3];
//        const T& normFibVel = controls[1];
//
//        // Intermediate quantities.
//        // ------------------------
//        const T fibLen = normFibLen * optimal_fiber_length;
//        // TODO cache this somewhere; this is constant.
//        const double fibWidth = optimal_fiber_length
//                * sin(pennation_angle_at_optimal);
//        // Tendon length.
//        const T& musTenLen = position;
//        // lT = lMT - sqrt(lM^2 - w^2)
//        const T tenLen = musTenLen
//                - sqrt(fibLen*fibLen - fibWidth*fibWidth);
//        const T normTenLen = tenLen / tendon_slack_length;
//        const T cosPenn = (musTenLen - tenLen) / fibLen;
//
//        // Curves/multipliers.
//        // -------------------
//        // Tendon force-length curve.
//        const T normTenForce = c1 * exp(kT * (normTenLen - c2)) - c3;
//
//        // Active force-length curve.
//        // Sum of 3 gaussians.
//        const T activeForceLenMult =
//                gaussian(normFibLen, b11, b21, b31, b41) +
//                        gaussian(normFibLen, b12, b22, b32, b42) +
//                        gaussian(normFibLen, b13, b23, b33, b43);
//
//        // Passive force-length curve.
//        const T passiveFibForce = (exp(kPE * (normFibLen - 1)/ e0) - 1) /
//                (exp(kPE) - 1);
//
//        // Force-velocity curve.
//        const T tempV = d2 * normFibVel + d3;
//        const T tempLogArg = tempV + sqrt(pow(tempV, 2) + 1);
//        const T forceVelMult = d1 * log(tempLogArg) + d4;
//
//        // Equilibrium constraint.
//        // -----------------------
//        const T normFibForce =
//                activation * activeForceLenMult * forceVelMult
//                        + passiveFibForce;
//        const T normFibForceAlongTen = normFibForce * cosPenn;
//
//        constraints[0] = normFibForceAlongTen - normTenForce;
//    }
//    void endpoint_cost(const T& final_time,
//                       const mesh::VectorX<T>& /*final_states*/,
//                       T& cost) const override {
//        cost = final_time;
//    }
//    //void integral_cost(const T& /*time*/,
//    //                   const mesh::VectorX<T>& /*states*/,
//    //                   const mesh::VectorX<T>& controls,
//    //                   T& integrand) const override {
//    //    integrand = controls[0] * controls[0];
//    //}
//};
