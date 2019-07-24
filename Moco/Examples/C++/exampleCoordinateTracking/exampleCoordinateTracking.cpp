/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleCoordinateTrackingSymmetry.cpp                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <Moco/osimMoco.h>

using namespace OpenSim;

// Set a coordinate tracking problem. Here the model is driven by muscles whose
// muscle-tendon lengths, velocities, and moment arms are derived from
// polynomial approximations of joint coordinates. The cost function combines
// a tracking term (coordinate values and speeds) and an effort term (squared
// controls). Periodicity is imposed over half a gait cycle (symmetric walking
// pattern).
void coordinateTracking_Muscles() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking_Muscles");
    // Set model
    ModelProcessor modelprocessor = ModelProcessor(
            "gait10dof18musc.osim");
    track.setModel(modelprocessor);
    // Set experimental coordinate values to track
    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
            TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    // Set guess
    track.set_apply_tracked_states_to_guess(true);
    // Set time bounds
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    // Initialize study
    MocoStudy moco = track.initialize();
    // Set solver settings
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100000);
    solver.set_parallel(6);
    // Update problem
    MocoProblem& problem = moco.updProblem();
    // Add symmetry goal
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    // Coordinate values
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/value"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/value"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/value",
            "/jointset/hip_r/hip_flexion_r/value"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/value",
            "/jointset/hip_l/hip_flexion_l/value"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/value",
            "/jointset/knee_r/knee_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/value",
            "/jointset/knee_l/knee_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/value",
            "/jointset/ankle_r/ankle_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/value",
            "/jointset/ankle_l/ankle_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/value"});
    // Coordinate speeds
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tx/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/speed",
            "/jointset/hip_r/hip_flexion_r/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/speed",
            "/jointset/hip_l/hip_flexion_l/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/speed",
            "/jointset/knee_r/knee_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/speed",
            "/jointset/knee_l/knee_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/speed",
            "/jointset/ankle_r/ankle_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/speed",
            "/jointset/ankle_l/ankle_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/speed"});
    // Coodinate actuators
    symmetryGoal->addControlPair({"/lumbarAct"});
    // Muscle activations
    symmetryGoal->addStatePair({"/hamstrings_l/activation",
            "/hamstrings_r/activation"});
    symmetryGoal->addStatePair({"/hamstrings_r/activation",
            "/hamstrings_l/activation"});
    symmetryGoal->addStatePair({"/bifemsh_l/activation",
            "/bifemsh_r/activation"});
    symmetryGoal->addStatePair({"/bifemsh_r/activation",
            "/bifemsh_l/activation"});
    symmetryGoal->addStatePair({"/glut_max_l/activation",
            "/glut_max_r/activation"});
    symmetryGoal->addStatePair({"/glut_max_r/activation",
            "/glut_max_l/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_l/activation",
            "/iliopsoas_r/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_r/activation",
            "/iliopsoas_l/activation"});
    symmetryGoal->addStatePair({"/rect_fem_l/activation",
            "/rect_fem_r/activation"});
    symmetryGoal->addStatePair({"/rect_fem_r/activation",
            "/rect_fem_l/activation"});
    symmetryGoal->addStatePair({"/vasti_l/activation",
            "/vasti_r/activation"});
    symmetryGoal->addStatePair({"/vasti_r/activation",
            "/vasti_l/activation"});
    symmetryGoal->addStatePair({"/gastroc_l/activation",
            "/gastroc_r/activation"});
    symmetryGoal->addStatePair({"/gastroc_r/activation",
            "/gastroc_l/activation"});
    symmetryGoal->addStatePair({"/soleus_l/activation",
            "/soleus_r/activation"});
    symmetryGoal->addStatePair({"/soleus_r/activation",
            "/soleus_l/activation"});
    symmetryGoal->addStatePair({"/tib_ant_l/activation",
            "/tib_ant_r/activation"});
    symmetryGoal->addStatePair({"/tib_ant_r/activation",
            "/tib_ant_l/activation"});
    // Add effort cots
    auto* effortGoal = problem.addGoal<MocoControlGoal>("effortGoal", 10);
    // Adjust bounds
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75,1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0,20*SimTK::Pi/180});
    // Solve problem
    MocoSolution solution = moco.solve();

    // Extract ground reaction forces
    std::vector<std::string> contactSpheres_r;
    std::vector<std::string> contactSpheres_l;
    // What users would provide
    contactSpheres_r.push_back("contactSphereHeel_r");
    contactSpheres_r.push_back("contactSphereFront_r");
    contactSpheres_l.push_back("contactSphereHeel_l");
    contactSpheres_l.push_back("contactSphereFront_l");
    // Extract forces
    TimeSeriesTableVec3 externalForcesTable{};
    StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
    SimTK::Vector optTime = solution.getTime();
    auto model = modelprocessor.process();
    model.initSystem();
    int count = 0;
    for (const auto& state : optStates) {
        model.realizeVelocity(state);

        SimTK::Vec3 forces_r(0);
        SimTK::Vec3 torques_r(0);
        for (const auto& contactSphere_r : contactSpheres_r) {
            Array<double> forcesContactSphere_r = model.getComponent<
                    SmoothSphereHalfSpaceForce>(
                            contactSphere_r).getRecordValues(state);
            forces_r += SimTK::Vec3(forcesContactSphere_r[0],
                forcesContactSphere_r[1], forcesContactSphere_r[2]);
            torques_r += SimTK::Vec3(forcesContactSphere_r[3],
                forcesContactSphere_r[4], forcesContactSphere_r[5]);
        }
        SimTK::Vec3 forces_l(0);
        SimTK::Vec3 torques_l(0);
        for (const auto& contactSphere_l : contactSpheres_l) {
            Array<double> forcesContactSphere_l = model.getComponent<
                    SmoothSphereHalfSpaceForce>(
                            contactSphere_l).getRecordValues(state);
            forces_l += SimTK::Vec3(forcesContactSphere_l[0],
                forcesContactSphere_l[1], forcesContactSphere_l[2]);
            torques_l += SimTK::Vec3(forcesContactSphere_l[3],
                forcesContactSphere_l[4], forcesContactSphere_l[5]);
        }
        // Combine in a row
        SimTK::RowVector_<SimTK::Vec3> row(6);
        row(0) = forces_r;
        row(1) = SimTK::Vec3(0);
        row(2) = forces_l;
        row(3) = SimTK::Vec3(0);
        row(4) = torques_r;
        row(5) = torques_l;
        // Append to a table
        externalForcesTable.appendRow(optTime[count],row);
        ++count;
    }
    // Write file
    // Create labels for output file
    std::vector<std::string> labels;
    labels.push_back("ground_force_v");
    labels.push_back("ground_force_p");
    labels.push_back("1_ground_force_v");
    labels.push_back("1_ground_force_p");
    labels.push_back("ground_torque_");
    labels.push_back("1_ground_torque_");
    std::vector<std::string> suffixes;
    suffixes.push_back("x");
    suffixes.push_back("y");
    suffixes.push_back("z");
    externalForcesTable.setColumnLabels(labels);
    TimeSeriesTable externalForcesTableFlat =
            externalForcesTable.flatten(suffixes);
    DataAdapter::InputTables tables = {{"table", &externalForcesTableFlat}};
    FileAdapter::writeFile(tables,
            "coordinateTracking_Muscles_GRF.sto");
}

// Set a coordinate tracking problem. Here the model is driven by muscles whose
// muscle-tendon lengths, velocities, and moment arms are derived from
// polynomial approximations of joint coordinates. The cost function combines
// a tracking term (coordinate values and speeds) and an effort term (squared
// controls). Periodicity is imposed over half a gait cycle (symmetric walking
// pattern).
void coordinateTracking_CoordinateActuators() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking_CoordinateActuators");
    // Set model
    ModelProcessor modelprocessor = ModelProcessor(
            "gait10dof18musc_CoordinateActuators.osim");
    track.setModel(modelprocessor);
    // Set experimental coordinate values to track
    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
            TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    // Set guess
    track.set_apply_tracked_states_to_guess(true);
    // Set time bounds
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    // Initialize study
    MocoStudy moco = track.initialize();
    // Set solver settings
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100000);
    solver.set_parallel(6);
    // Update problem
    MocoProblem& problem = moco.updProblem();
    // Add symmetry goal
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    // Coordinate values
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/value"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/value"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/value",
            "/jointset/hip_r/hip_flexion_r/value"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/value",
            "/jointset/hip_l/hip_flexion_l/value"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/value",
            "/jointset/knee_r/knee_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/value",
            "/jointset/knee_l/knee_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/value",
            "/jointset/ankle_r/ankle_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/value",
            "/jointset/ankle_l/ankle_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/value"});
    // Coordinate speeds
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tx/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/speed",
            "/jointset/hip_r/hip_flexion_r/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/speed",
            "/jointset/hip_l/hip_flexion_l/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/speed",
            "/jointset/knee_r/knee_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/speed",
            "/jointset/knee_l/knee_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/speed",
            "/jointset/ankle_r/ankle_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/speed",
            "/jointset/ankle_l/ankle_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/speed"});
    // Coodinate actuators
    symmetryGoal->addControlPair({"/lumbarAct"});
    symmetryGoal->addControlPair({"/hipAct_l", "/hipAct_r"});
    symmetryGoal->addControlPair({"/hipAct_r", "/hipAct_l"});
    symmetryGoal->addControlPair({"/kneeAct_l", "/kneeAct_r"});
    symmetryGoal->addControlPair({"/kneeAct_r", "/kneeAct_l"});
    symmetryGoal->addControlPair({"/ankleAct_l", "/ankleAct_r"});
    symmetryGoal->addControlPair({"/ankleAct_r", "/ankleAct_l"});
    // Add effort cots
    auto* effortGoal = problem.addGoal<MocoControlGoal>("effortGoal", 10);
    // Adjust bounds
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75,1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0,20*SimTK::Pi/180});
    // Solve problem
    MocoSolution solution = moco.solve();

    // Extract ground reaction forces
    std::vector<std::string> contactSpheres_r;
    std::vector<std::string> contactSpheres_l;
    // What users would provide
    contactSpheres_r.push_back("contactSphereHeel_r");
    contactSpheres_r.push_back("contactSphereFront_r");
    contactSpheres_l.push_back("contactSphereHeel_l");
    contactSpheres_l.push_back("contactSphereFront_l");
    // Extract forces
    TimeSeriesTableVec3 externalForcesTable{};
    StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
    SimTK::Vector optTime = solution.getTime();
    auto model = modelprocessor.process();
    model.initSystem();
    int count = 0;
    for (const auto& state : optStates) {
        model.realizeVelocity(state);

        SimTK::Vec3 forces_r(0);
        SimTK::Vec3 torques_r(0);
        for (const auto& contactSphere_r : contactSpheres_r) {
            Array<double> forcesContactSphere_r = model.getComponent<
                    SmoothSphereHalfSpaceForce>(
                            contactSphere_r).getRecordValues(state);
            forces_r += SimTK::Vec3(forcesContactSphere_r[0],
                forcesContactSphere_r[1], forcesContactSphere_r[2]);
            torques_r += SimTK::Vec3(forcesContactSphere_r[3],
                forcesContactSphere_r[4], forcesContactSphere_r[5]);
        }
        SimTK::Vec3 forces_l(0);
        SimTK::Vec3 torques_l(0);
        for (const auto& contactSphere_l : contactSpheres_l) {
            Array<double> forcesContactSphere_l = model.getComponent<
                    SmoothSphereHalfSpaceForce>(
                            contactSphere_l).getRecordValues(state);
            forces_l += SimTK::Vec3(forcesContactSphere_l[0],
                forcesContactSphere_l[1], forcesContactSphere_l[2]);
            torques_l += SimTK::Vec3(forcesContactSphere_l[3],
                forcesContactSphere_l[4], forcesContactSphere_l[5]);
        }
        // Combine in a row
        SimTK::RowVector_<SimTK::Vec3> row(6);
        row(0) = forces_r;
        row(1) = SimTK::Vec3(0);
        row(2) = forces_l;
        row(3) = SimTK::Vec3(0);
        row(4) = torques_r;
        row(5) = torques_l;
        // Append to a table
        externalForcesTable.appendRow(optTime[count],row);
        ++count;
    }
    // Write file
    // Create labels for output file
    std::vector<std::string> labels;
    labels.push_back("ground_force_v");
    labels.push_back("ground_force_p");
    labels.push_back("1_ground_force_v");
    labels.push_back("1_ground_force_p");
    labels.push_back("ground_torque_");
    labels.push_back("1_ground_torque_");
    std::vector<std::string> suffixes;
    suffixes.push_back("x");
    suffixes.push_back("y");
    suffixes.push_back("z");
    externalForcesTable.setColumnLabels(labels);
    TimeSeriesTable externalForcesTableFlat =
            externalForcesTable.flatten(suffixes);
    DataAdapter::InputTables tables = {{"table", &externalForcesTableFlat}};
    FileAdapter::writeFile(tables,
            "coordinateTracking_CoordinateActuators_GRF.sto");
}

int main() {
   coordinateTracking_Muscles();
   coordinateTracking_CoordinateActuators();
}
