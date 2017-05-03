#include <OpenSim/OpenSim.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>
#include <MuscleRedundancySolver.h>
#include <GlobalStaticOptimizationSolver.h>
#include <DeGroote2016Muscle.h>
#include <mesh.h>
#include "testing.h"

using namespace OpenSim;

/// Use DeGroote's 2016 muscle model (outside of OpenSim) to solve a
/// trajectory optimization problem for the optimal length trajectories of two
/// muscles in parallel lifting a mass against gravity.
/// Then use an inverse solver to recover the original activation trajectory.
/// We do this for both GlobalStaticOptimizationSolver and
/// MuscleRedundancySolver.

/// Lift two muscles in parallel against gravity from a fixed starting state 
/// to a fixed end position and velocity, in minimum time.
/// Here's a sketch of the problem we solve, without muscle dynamics:
/// @verbatim
///   minimize   t_f
///   subject to qdot = u             kinematics
///              udot = (mg - f1_t - f2_t)/m  dynamics
///              fi_t = (a f_l(li_m) f_v(vi_m) + f_p(li_m)) cos(alpha_i), i = 1,2
///              q(0) = 0.2
///              u(0) = 0
///              q(t_f) = 0.15
///              u(t_f) = 0
/// @endverbatim
/// where l1_m, 12_m, v1_m and v2_m are determined from the muscle-tendon
/// lengths and velocities with the assumption of rigid tendons.
class TwoDeGrooteMusclesLiftMinTimeStatic 
		: public mesh::OptimalControlProblemNamed<adouble> {
public:
	using T = adouble;
	const double g = 9.81;
	const double mass = 0.5;
	// TODO move to a common place.
	const double max_isometric_force_1 = 20;
	const double max_isometric_force_2 = 10;
	const double optimal_fiber_length_1 = 0.075;
	const double optimal_fiber_length_2 = 0.125;
	const double tendon_slack_length = 0.05;
	const double pennation_angle_at_optimal = 0.1;
	// optimal fiber lengths per second:
	const double max_contraction_velocity = 10;

	TwoDeGrooteMusclesLiftMinTimeStatic() : 
			mesh::OptimalControlProblemNamed<T>("two_hanging_muscles_min_time") {
		this->set_time(0, {0.01, 1.0});
		// TODO these functions should return indices for these variables.
		this->add_state("position", { 0, 0.3 }, 0.15, 0.10);
		this->add_state("speed", { -10, 10 }, 0, 0);
		this->add_control("activation_1", { 0, 1 });
		this->add_control("activation_2", { 0, 1 });
		// TODO move these to a constructor parameter.
		m_muscle_1 = DeGroote2016Muscle<T>(
				max_isometric_force_1, optimal_fiber_length_1, tendon_slack_length,
				pennation_angle_at_optimal, max_contraction_velocity);
		m_muscle_2 = DeGroote2016Muscle<T>(
				max_isometric_force_2, optimal_fiber_length_2, tendon_slack_length,
				pennation_angle_at_optimal, max_contraction_velocity);
	}

	void dynamics(const mesh::VectorX<T>& states,
				  const mesh::VectorX<T>& controls,
				  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
		// Unpack variables.
		const T& position = states[0];
		const T& speed = states[1];
		const T& activation_1 = controls[0];
		const T& activation_2 = controls[1];

		// Multibody kinematics.
		derivatives[0] = speed;

		// Multibody dynamics.
		const T tendonForce_1 =
				m_muscle_1.calcRigidTendonFiberForceAlongTendon(activation_1,
															   position, speed);
		const T tendonForce_2 =
				m_muscle_2.calcRigidTendonFiberForceAlongTendon(activation_2,
															   position, speed);
		derivatives[1] = g - tendonForce_1 / mass - tendonForce_2 / mass;
	}
	void endpoint_cost(const T& final_time,
					   const mesh::VectorX<T>& /*final_states*/,
					   T& cost) const override {
		cost = final_time;
	}
private:
	DeGroote2016Muscle<T> m_muscle_1, m_muscle_2;
};

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryGlobalStaticOptimizationSolver() {
	// Solve a trajectory optimization problem.
	// ----------------------------------------
	auto ocp = std::make_shared<TwoDeGrooteMusclesLiftMinTimeStatic>();
	mesh::DirectCollocationSolver<adouble> dircol (ocp, "trapezoidal",
												   "ipopt", 100);
	mesh::OptimalControlSolution ocp_solution = dircol.solve();
	std::string trajectoryFile = 
			"testTwoParallelMusclesDeGroote2016_GSO_trajectory.csv";
	ocp_solution.write(trajectoryFile);

	// Save the trajectory with a header so that OpenSim can read it.
	//---------------------------------------------------------------
	// CVSFileAdapter expects an "endheader" line in the file.
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
	TimeSeriesTable actualInvDyn;
	actualInvDyn.setColumnLabels({"inverse_dynamics"});
	DeGroote2016Muscle<double> muscle_1(ocp->max_isometric_force_1,
										ocp->optimal_fiber_length_1,
										ocp->tendon_slack_length,
										ocp->pennation_angle_at_optimal,
										ocp->max_contraction_velocity);
	DeGroote2016Muscle<double> muscle_2(ocp->max_isometric_force_2,
									    ocp->optimal_fiber_length_2,
										ocp->tendon_slack_length,
										ocp->pennation_angle_at_optimal,
										ocp->max_contraction_velocity);
	for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
		const auto& position = ocp_solution.states(0, iTime);
		const auto& speed = ocp_solution.states(1, iTime);
		const auto& activation_1 = ocp_solution.controls(0, iTime);
		const auto& activation_2 = ocp_solution.controls(1, iTime);
		const auto normTendonForce_1 = 
				muscle_1.calcRigidTendonFiberForceAlongTendon(activation_1,
															  position,
															  speed);
		const auto tendonForce_1 = muscle_1.get_max_isometric_force()
			* normTendonForce_1;
		const auto normTendonForce_2 = 
				muscle_2.calcRigidTendonFiberForceAlongTendon(activation_2,
														      position,
															  speed);
		const auto tendonForce_2 = muscle_2.get_max_isometric_force()
								 * normTendonForce_2;
		actualInvDyn.appendRow(ocp_solution.time(iTime),
							   SimTK::RowVector(1, -tendonForce_1-tendonForce_2));

	}
	CSVFileAdapter::write(actualInvDyn,
						  "DEBUG_testLiftingMassTwoMuscles_GSO_actualInvDyn.csv");

	return {ocpSolution, kinematics};
}

OpenSim::Model buildLiftingMassModel() {
	TwoDeGrooteMusclesLiftMinTimeStatic ocp;
	Model model;
	model.setName("two_hanging_muscles");

	// First, we need to build a similar OpenSim model.
	model.set_gravity(SimTK::Vec3(ocp.g, 0.0, 0.0));
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

	auto* actu_1 = new Millard2012EquilibriumMuscle();
	actu_1->setName("muscle_1");
	actu_1->set_max_isometric_force(ocp.max_isometric_force_1);
	actu_1->set_optimal_fiber_length(ocp.optimal_fiber_length_1);
	actu_1->set_tendon_slack_length(ocp.tendon_slack_length);
	actu_1->set_pennation_angle_at_optimal(ocp.pennation_angle_at_optimal);
	actu_1->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
	actu_1->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
	model.addComponent(actu_1);

	auto* actu_2 = new Millard2012EquilibriumMuscle();
	actu_2->setName("muscle_2");
	actu_2->set_max_isometric_force(ocp.max_isometric_force_2);
	actu_2->set_optimal_fiber_length(ocp.optimal_fiber_length_2);
	actu_2->set_tendon_slack_length(ocp.tendon_slack_length);
	actu_2->set_pennation_angle_at_optimal(ocp.pennation_angle_at_optimal);
	actu_2->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
	actu_2->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
	model.addComponent(actu_2);

	return model;
}

void testLiftingMassGlobalStaticOptimizationSolver(
		const std::pair<TimeSeriesTable, TimeSeriesTable>& data) {
	const auto& ocpSolution = data.first;
	const auto& kinematics = data.second;

	// Build a similar OpenSim model.
	//-------------------------------
	Model model = buildLiftingMassModel();
	model.finalizeFromProperties();

	// Create the GlobalStaticOptimizationSolver.
	//-------------------------------------------
	GlobalStaticOptimizationSolver gso;
	gso.setModel(model);
	gso.setKinematicsData(kinematics);

	// Filter joint moments
	gso.set_lowpass_cutoff_frequency_for_joint_moments(100);

	gso.set_create_reserve_actuators(0.001);
	GlobalStaticOptimizationSolver::Solution solution = gso.solve();
	solution.write("testTwoParallelMusclesDeGroote2016_GSO");

	// Compare the solution to the intial trajectory optimization solution.
	//---------------------------------------------------------------------

	// The rationale for the tolerances: as tight as they could be for the 
	// test to pass
	rootMeanSquare_temp(solution.activation,"/two_hanging_muscles/muscle_1",
						ocpSolution, "activation_1", 0.05);
	rootMeanSquare_temp(solution.activation, "/two_hanging_muscles/muscle_1",
				   ocpSolution, "activation_2", 0.05);
}

int main() {
	SimTK_START_TEST("testTwoParallelMusclesDeGroote2016");
	{
		auto gsoData = solveForTrajectoryGlobalStaticOptimizationSolver();
		SimTK_SUBTEST1(testLiftingMassGlobalStaticOptimizationSolver,
			gsoData);
	}
	SimTK_END_TEST();
}