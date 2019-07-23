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
void testCoordinateTracking() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking");
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

    //// Extract ground reaction forces
    //// Get optimal states
    //StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
    //// Get optimal time vector
    //SimTK::Vector optTime = solution.getTime();
    //// Get model
    //auto model = modelprocessor.process();
    //// Create labels for output file
    //std::vector<std::string> labels;
    //labels.push_back("time");
    //labels.push_back("ground_force_vx"); // right
    //labels.push_back("ground_force_vy");
    //labels.push_back("ground_force_vz");
    //labels.push_back("ground_force_px");
    //labels.push_back("ground_force_py");
    //labels.push_back("ground_force_pz");
    //labels.push_back("1_ground_force_vx"); // left
    //labels.push_back("1_ground_force_vy");
    //labels.push_back("1_ground_force_vz");
    //labels.push_back("1_ground_force_px");
    //labels.push_back("1_ground_force_py");
    //labels.push_back("1_ground_force_pz");
    //labels.push_back("ground_torque_x"); // right
    //labels.push_back("ground_torque_y");
    //labels.push_back("ground_torque_z");
    //labels.push_back("1_ground_torque_x"); // left
    //labels.push_back("1_ground_torque_y");
    //labels.push_back("1_ground_torque_z");
    //TimeSeriesTable externalForcesTable{};
    //externalForcesTable.setColumnLabels(labels);
    //// Helper Vec3
    //SimTK::Vec3 nullP(0);
    //// Extract forces
    //int count = 0;
    //for (const auto& state : optStates) {
    //    Array<double> forcesContactSphereHeel_r = model.getComponent<
    //            SmoothSphereHalfSpaceForce>(
    //            "contactSphereHeel_r").getRecordValues(state);
    //    Array<double> forcesContactSphereHeel_l = model.getComponent<
    //            SmoothSphereHalfSpaceForce>(
    //            "contactSphereHeel_l").getRecordValues(state);
    //    Array<double> forcesContactSphereFront_r = model.getComponent<
    //            SmoothSphereHalfSpaceForce>(
    //            "contactSphereFront_r").getRecordValues(state);
    //    Array<double> forcesContactSphereFront_l = model.getComponent<
    //            SmoothSphereHalfSpaceForce>(
    //            "contactSphereFront_l").getRecordValues(state);
    //    // Combine forces and torques from contact spheres from same foot
    //    // Forces
    //    SimTK::Vec3 forces_r = SimTK::Vec3(forcesContactSphereHeel_r[0],
    //            forcesContactSphereHeel_r[1], forcesContactSphereHeel_r[2]) +
    //            SimTK::Vec3(forcesContactSphereFront_r[0],
    //            forcesContactSphereFront_r[1], forcesContactSphereFront_r[2]);
    //    SimTK::Vec3 forces_l = SimTK::Vec3(forcesContactSphereHeel_l[0],
    //            forcesContactSphereHeel_l[1], forcesContactSphereHeel_l[2]) +
    //            SimTK::Vec3(forcesContactSphereFront_l[0],
    //            forcesContactSphereFront_l[1], forcesContactSphereFront_l[2]);
    //    // Torques
    //    SimTK::Vec3 torques_r = SimTK::Vec3(forcesContactSphereHeel_r[3],
    //            forcesContactSphereHeel_r[4], forcesContactSphereHeel_r[5]) +
    //            SimTK::Vec3(forcesContactSphereFront_r[3],
    //            forcesContactSphereFront_r[4], forcesContactSphereFront_r[5]);
    //    SimTK::Vec3 torques_l = SimTK::Vec3(forcesContactSphereHeel_l[3],
    //            forcesContactSphereHeel_l[4], forcesContactSphereHeel_l[5]) +
    //            SimTK::Vec3(forcesContactSphereFront_l[3],
    //            forcesContactSphereFront_l[4], forcesContactSphereFront_l[5]);
    //    // Create row
    //    SimTK::RowVector row{18,0.0};
    //    for (int i = 0; i < 2; ++i) {
    //        row(i) = forces_r[i];
    //        row(i+3) = nullP[i];
    //        row(i+6) = forces_l[i];
    //        row(i+9) = nullP[i];
    //        row(i+12) = torques_r[i];
    //        row(i+15) = torques_l[i];
    //    }
    //    // Append row
    //    externalForcesTable.appendRow(optTime[count],row);
    //    ++count;
    //}
    //// Write file
    //writeTableToFile(externalForcesTable,"test_GRF.sto");
}

// Set a coordinate tracking problem. Here the model is driven by muscles whose
// muscle-tendon lengths, velocities, and moment arms are derived from
// polynomial approximations of joint coordinates. The cost function combines
// a tracking term (coordinate values and speeds) and an effort term (squared
// controls). Periodicity is imposed over half a gait cycle (symmetric walking
// pattern).
void testCoordinateTracking_CoordinateActuators() {
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
    // Get optimal states
    StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
    // Get optimal time vector
    SimTK::Vector optTime = solution.getTime();
    // Get model
    auto model = modelprocessor.process();
    // Create labels for output file
    std::vector<std::string> labels;
    labels.push_back("time");
    labels.push_back("ground_force_vx"); // right
    labels.push_back("ground_force_vy");
    labels.push_back("ground_force_vz");
    labels.push_back("ground_force_px");
    labels.push_back("ground_force_py");
    labels.push_back("ground_force_pz");
    labels.push_back("1_ground_force_vx"); // left
    labels.push_back("1_ground_force_vy");
    labels.push_back("1_ground_force_vz");
    labels.push_back("1_ground_force_px");
    labels.push_back("1_ground_force_py");
    labels.push_back("1_ground_force_pz");
    labels.push_back("ground_torque_x"); // right
    labels.push_back("ground_torque_y");
    labels.push_back("ground_torque_z");
    labels.push_back("1_ground_torque_x"); // left
    labels.push_back("1_ground_torque_y");
    labels.push_back("1_ground_torque_z");
    TimeSeriesTable externalForcesTable{};
    externalForcesTable.setColumnLabels(labels);
    // Helper Vec3
    SimTK::Vec3 nullP(0);
    // Extract forces
    int count = 0;
    for (const auto& state : optStates) {
        Array<double> forcesContactSphereHeel_r = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereHeel_r").getRecordValues(state);
        Array<double> forcesContactSphereHeel_l = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereHeel_l").getRecordValues(state);
        Array<double> forcesContactSphereFront_r = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereFront_r").getRecordValues(state);
        Array<double> forcesContactSphereFront_l = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereFront_l").getRecordValues(state);
        // Combine forces and torques from contact spheres from same foot
        // Forces
        SimTK::Vec3 forces_r = SimTK::Vec3(forcesContactSphereHeel_r[0],
                forcesContactSphereHeel_r[1], forcesContactSphereHeel_r[2]) +
                SimTK::Vec3(forcesContactSphereFront_r[0],
                forcesContactSphereFront_r[1], forcesContactSphereFront_r[2]);
        SimTK::Vec3 forces_l = SimTK::Vec3(forcesContactSphereHeel_l[0],
                forcesContactSphereHeel_l[1], forcesContactSphereHeel_l[2]) +
                SimTK::Vec3(forcesContactSphereFront_l[0],
                forcesContactSphereFront_l[1], forcesContactSphereFront_l[2]);
        // Torques
        SimTK::Vec3 torques_r = SimTK::Vec3(forcesContactSphereHeel_r[3],
                forcesContactSphereHeel_r[4], forcesContactSphereHeel_r[5]) +
                SimTK::Vec3(forcesContactSphereFront_r[3],
                forcesContactSphereFront_r[4], forcesContactSphereFront_r[5]);
        SimTK::Vec3 torques_l = SimTK::Vec3(forcesContactSphereHeel_l[3],
                forcesContactSphereHeel_l[4], forcesContactSphereHeel_l[5]) +
                SimTK::Vec3(forcesContactSphereFront_l[3],
                forcesContactSphereFront_l[4], forcesContactSphereFront_l[5]);
        // Create row
        SimTK::RowVector row{18,0.0};
        for (int i = 0; i < 2; ++i) {
            row(i) = forces_r[i];
            row(i+3) = nullP[i];
            row(i+6) = forces_l[i];
            row(i+9) = nullP[i];
            row(i+12) = torques_r[i];
            row(i+15) = torques_l[i];
        }
        // Append row
        externalForcesTable.appendRow(optTime[count],row);
        ++count;
    }
    // Write file
    writeTableToFile(externalForcesTable,"test_GRF.sto");
}

// Set a coordinate tracking problem. Here the model is driven by muscles whose
// muscle-tendon lengths, velocities, and moment arms are derived from
// polynomial approximations of joint coordinates. The cost function combines
// a tracking term (coordinate values and speeds), an effort term (squared
// controls), and a term encouraging symmetry of the coordinate values over
// half a gait cycle.
//void testCoordinateTracking_MusclePolynomials_withPassiveForces() {
//    // Create a MocoTrack
//    MocoTrack track;
//    track.setName("coordinateTracking_MusclePolynomials_withPassiveForces");
//    // Set model
//    ModelProcessor modelprocessor = ModelProcessor(
//            "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
//    track.setModel(modelprocessor);
//    // Set experimental coordinate values to track
//    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
//            TabOpLowPassFilter(6));
//    track.set_states_global_tracking_weight(10.0);
//    track.set_allow_unused_references(true);
//    track.set_track_reference_position_derivatives(true);
//    // Set guess
//    track.set_apply_tracked_states_to_guess(true);
//    // Set time bounds
//    track.set_initial_time(0.0);
//    track.set_final_time(0.47008941);
//    // Initialize study
//    MocoStudy moco = track.initialize();
//    // Set solver settings
//    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(1e-4);
//    solver.set_optim_constraint_tolerance(1e-4);
//    solver.set_optim_max_iterations(100000);
//    solver.set_parallel(6);
//    // Update problem
//    MocoProblem& problem = moco.updProblem();
//    // Add symmetry goal
//    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");
//    symmetryCost->set_weight(10);
//    // Add effort cots
//    auto* effortCost = problem.addCost<MocoControlCost>("effortCost");
//    effortCost->set_weight(10);
//    // Adjust bounds
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
//            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
//            {0.75,1.25});
//    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/lumbar/lumbar/value",
//            {0,20*SimTK::Pi/180});
//    // Solve problem
//    MocoSolution solution = moco.solve();
//
//    // Extract ground reaction forces
//    // Get optimal states
//    StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
//    // Get optimal time vector
//    SimTK::Vector optTime = solution.getTime();
//    // Get model
//    auto model = modelprocessor.process();
//    // Create labels for output file
//    std::vector<std::string> labels;
//    labels.push_back("time");
//    labels.push_back("ground_force_vx"); // right
//    labels.push_back("ground_force_vy");
//    labels.push_back("ground_force_vz");
//    labels.push_back("ground_force_px");
//    labels.push_back("ground_force_py");
//    labels.push_back("ground_force_pz");
//    labels.push_back("1_ground_force_vx"); // left
//    labels.push_back("1_ground_force_vy");
//    labels.push_back("1_ground_force_vz");
//    labels.push_back("1_ground_force_px");
//    labels.push_back("1_ground_force_py");
//    labels.push_back("1_ground_force_pz");
//    labels.push_back("ground_torque_x"); // right
//    labels.push_back("ground_torque_y");
//    labels.push_back("ground_torque_z");
//    labels.push_back("1_ground_torque_x"); // left
//    labels.push_back("1_ground_torque_y");
//    labels.push_back("1_ground_torque_z");
//    TimeSeriesTable externalForcesTable{};
//    externalForcesTable.setColumnLabels(labels);
//    // Helper Vec3
//    SimTK::Vec3 nullP(0);
//    // Extract forces
//    int count = 0;
//    for (const auto& state : optStates) {
//        Array<double> forcesContactSphereHeel_r = model.getComponent<
//                SmoothSphereHalfSpaceForce>(
//                "contactSphereHeel_r").getRecordValues(state);
//        Array<double> forcesContactSphereHeel_l = model.getComponent<
//                SmoothSphereHalfSpaceForce>(
//                "contactSphereHeel_l").getRecordValues(state);
//        Array<double> forcesContactSphereFront_r = model.getComponent<
//                SmoothSphereHalfSpaceForce>(
//                "contactSphereFront_r").getRecordValues(state);
//        Array<double> forcesContactSphereFront_l = model.getComponent<
//                SmoothSphereHalfSpaceForce>(
//                "contactSphereFront_l").getRecordValues(state);
//        // Combine forces and torques from contact spheres from same foot
//        // Forces
//        SimTK::Vec3 forces_r = SimTK::Vec3(forcesContactSphereHeel_r[0],
//                forcesContactSphereHeel_r[1], forcesContactSphereHeel_r[2]) +
//                SimTK::Vec3(forcesContactSphereFront_r[0],
//                forcesContactSphereFront_r[1], forcesContactSphereFront_r[2]);
//        SimTK::Vec3 forces_l = SimTK::Vec3(forcesContactSphereHeel_l[0],
//                forcesContactSphereHeel_l[1], forcesContactSphereHeel_l[2]) +
//                SimTK::Vec3(forcesContactSphereFront_l[0],
//                forcesContactSphereFront_l[1], forcesContactSphereFront_l[2]);
//        // Torques
//        SimTK::Vec3 torques_r = SimTK::Vec3(forcesContactSphereHeel_r[3],
//                forcesContactSphereHeel_r[4], forcesContactSphereHeel_r[5]) +
//                SimTK::Vec3(forcesContactSphereFront_r[3],
//                forcesContactSphereFront_r[4], forcesContactSphereFront_r[5]);
//        SimTK::Vec3 torques_l = SimTK::Vec3(forcesContactSphereHeel_l[3],
//                forcesContactSphereHeel_l[4], forcesContactSphereHeel_l[5]) +
//                SimTK::Vec3(forcesContactSphereFront_l[3],
//                forcesContactSphereFront_l[4], forcesContactSphereFront_l[5]);
//        // Create row
//        SimTK::RowVector row{18,0.0};
//        for (int i = 0; i < 2; ++i) {
//            row(i) = forces_r[i];
//            row(i+3) = nullP[i];
//            row(i+6) = forces_l[i];
//            row(i+9) = nullP[i];
//            row(i+12) = torques_r[i];
//            row(i+15) = torques_l[i];
//        }
//        // Append row
//        externalForcesTable.appendRow(optTime[count],row);
//        ++count;
//    }
//    // Write file
//    writeTableToFile(externalForcesTable,"test_GRF.sto");
//}


//void testCoordinateTracking_CoordinateActuators() {
//    // Create a MocoTrack
//    MocoTrack track;
//    track.setName("coordinateTracking_CoordinateActuators");
//    // Set model
//    ModelProcessor modelprocessor = ModelProcessor(
//            "gait10dof18musc_CoordinateActuators.osim");
//    track.setModel(modelprocessor);
//    // Set experimental coordinate values to track
//    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
//            TabOpLowPassFilter(6));
//    track.set_states_global_tracking_weight(10.0);
//    track.set_allow_unused_references(true);
//    track.set_track_reference_position_derivatives(true);
//    // Set guess
//    track.set_apply_tracked_states_to_guess(true);
//    // Set time bounds
//    track.set_initial_time(0.0);
//    track.set_final_time(0.47008941);
//    // Initialize study
//    MocoStudy moco = track.initialize();
//    // Set solver settings
//    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(1e-4);
//    solver.set_optim_constraint_tolerance(1e-4);
//    solver.set_optim_max_iterations(100000);
//    solver.set_parallel(8);
//    // Update problem
//    MocoProblem& problem = moco.updProblem();
//    // Add symmetry cost
//    auto* symmetryCost = problem.addCost<MocoSymmetryCost2>("symmetryCost");
//    symmetryCost->set_weight(10);
//    // Add effort cots
//    auto* effortCost = problem.addCost<MocoControlCost>("effortCost");
//    effortCost->set_weight(10);
//    // Adjust bounds
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
//            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
//            {0.75,1.25});
//    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/lumbar/lumbar/value",
//            {0,20*SimTK::Pi/180});
//    // Solve problem
//    MocoSolution solution = moco.solve();
//
//    // Extract ground reaction forces
//    // Get optimal states
//    StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
//    // Get optimal time vector
//    SimTK::Vector optTime = solution.getTime();
//    // Get model
//    auto model = modelprocessor.process();
//    model.initSystem();
//    // Create labels for output file
//    std::vector<std::string> labels;
//    labels.push_back("ground_force_vx"); // right
//    labels.push_back("ground_force_vy");
//    labels.push_back("ground_force_vz");
//    labels.push_back("ground_force_px");
//    labels.push_back("ground_force_py");
//    labels.push_back("ground_force_pz");
//    labels.push_back("1_ground_force_vx"); // left
//    labels.push_back("1_ground_force_vy");
//    labels.push_back("1_ground_force_vz");
//    labels.push_back("1_ground_force_px");
//    labels.push_back("1_ground_force_py");
//    labels.push_back("1_ground_force_pz");
//    labels.push_back("ground_torque_x"); // right
//    labels.push_back("ground_torque_y");
//    labels.push_back("ground_torque_z");
//    labels.push_back("1_ground_torque_x"); // left
//    labels.push_back("1_ground_torque_y");
//    labels.push_back("1_ground_torque_z");
//    //labels.push_back("ground_force_vx"); // right
//    //labels.push_back("ground_force_vy");
//    //labels.push_back("ground_force_vz");
//    //labels.push_back("ground_force_vx_b");
//    //labels.push_back("ground_force_vy_b");
//    //labels.push_back("ground_force_vz_b");
//    //labels.push_back("1_ground_force_vx"); // left
//    //labels.push_back("1_ground_force_vy");
//    //labels.push_back("1_ground_force_vz");
//    //labels.push_back("1_ground_force_vx_b");
//    //labels.push_back("1_ground_force_vy_b");
//    //labels.push_back("1_ground_force_vz_b");
//    //labels.push_back("ground_torque_x"); // right
//    //labels.push_back("ground_torque_y");
//    //labels.push_back("ground_torque_z");
//    //labels.push_back("ground_torque_x_b"); // right
//    //labels.push_back("ground_torque_y_b");
//    //labels.push_back("ground_torque_z_b");
//    //labels.push_back("1_ground_torque_x"); // left
//    //labels.push_back("1_ground_torque_y");
//    //labels.push_back("1_ground_torque_z");
//    //labels.push_back("1_ground_torque_x_b"); // left
//    //labels.push_back("1_ground_torque_y_b");
//    //labels.push_back("1_ground_torque_z_b");
//    TimeSeriesTable externalForcesTable{};
//    externalForcesTable.setColumnLabels(labels);
//    TimeSeriesTable externalForcesTable2{};
//    externalForcesTable2.setColumnLabels(labels);
//    // Helper Vec3
//    SimTK::Vec3 nullP(0);
//    // Extract forces
//    int count = 0;
//    for (const auto& state : optStates) {
//        model.realizeVelocity(state);
//        Array<double> forcesContactSphereFront_l = model.getComponent<
//                SmoothSphereHalfSpaceForce>(
//                "contactSphereFront_l").getRecordValues(state);
//        Array<double> forcesContactSphereHeel_r = model.getComponent<
//                SmoothSphereHalfSpaceForce>(
//                "contactSphereHeel_r").getRecordValues(state);
//        Array<double> forcesContactSphereHeel_l = model.getComponent<
//                SmoothSphereHalfSpaceForce>(
//                "contactSphereHeel_l").getRecordValues(state);
//        Array<double> forcesContactSphereFront_r = model.getComponent<
//                SmoothSphereHalfSpaceForce>(
//                "contactSphereFront_r").getRecordValues(state);
//
//        // Combine forces and torques from contact spheres from same foot
//        // Forces
//        SimTK::Vec3 forces_r = SimTK::Vec3(forcesContactSphereHeel_r[0],
//                forcesContactSphereHeel_r[1], forcesContactSphereHeel_r[2]) +
//                SimTK::Vec3(forcesContactSphereFront_r[0],
//                forcesContactSphereFront_r[1], forcesContactSphereFront_r[2]);
//        SimTK::Vec3 forces_l = SimTK::Vec3(forcesContactSphereHeel_l[0],
//                forcesContactSphereHeel_l[1], forcesContactSphereHeel_l[2]) +
//                SimTK::Vec3(forcesContactSphereFront_l[0],
//                forcesContactSphereFront_l[1], forcesContactSphereFront_l[2]);
//        // Torques
//        SimTK::Vec3 torques_r = SimTK::Vec3(forcesContactSphereHeel_r[3],
//                forcesContactSphereHeel_r[4], forcesContactSphereHeel_r[5]) +
//                SimTK::Vec3(forcesContactSphereFront_r[3],
//                forcesContactSphereFront_r[4], forcesContactSphereFront_r[5]);
//        SimTK::Vec3 torques_l = SimTK::Vec3(forcesContactSphereHeel_l[3],
//                forcesContactSphereHeel_l[4], forcesContactSphereHeel_l[5]) +
//                SimTK::Vec3(forcesContactSphereFront_l[3],
//                forcesContactSphereFront_l[4], forcesContactSphereFront_l[5]);
//
//        //SimTK::Vec3 locCSHInB_r = model.getComponent<SmoothSphereHalfSpaceForce>("contactSphereHeel_r").getProperty_contact_sphere_location().getValue();
//        //SimTK::Vec3 locCSHInG_r = model.getBodySet().get("calcn_r").findStationLocationInGround(state,locCSHInB_r);
//        ////std::cout << locCSHInB_r << std::endl;
//        ////std::cout << locCSHInG_r << std::endl;
//        //SimTK::Vec3 locCSFInB_r = model.getComponent<SmoothSphereHalfSpaceForce>("contactSphereFront_r").getProperty_contact_sphere_location().getValue();
//        //SimTK::Vec3 locCSFInG_r = model.getBodySet().get("calcn_r").findStationLocationInGround(state,locCSFInB_r);
//        ////std::cout << locCSFInB_r << std::endl;
//        ////std::cout << locCSFInG_r << std::endl;
//        //SimTK::Real dCSH = SimTK::square(std::pow(locCSHInG_r[0],2)+std::pow(locCSHInG_r[2],2));
//        //SimTK::Real dCSF = SimTK::square(std::pow(locCSFInG_r[0],2)+std::pow(locCSFInG_r[2],2));
//        //SimTK::Real M_r = dCSH*forcesContactSphereHeel_r[1] + dCSF*forcesContactSphereFront_r[1];
//        //std::cout << torques_r << std::endl;
//        //std::cout << M_r << std::endl;
//
//
//        // Express as three forces and three torques
//        // Create row
//        SimTK::Vec3 pApp_r(0);
//        SimTK::Vec3 pApp_l(0);
//        SimTK::RowVector row{18,0.0};
//        for (int i = 0; i < 3; ++i) {
//            row(i) = forces_r[i];
//            row(i+3) = pApp_r[i];
//            row(i+6) = forces_l[i];
//            row(i+9) = pApp_l[i];
//            row(i+12) = torques_r[i];
//            row(i+15) = torques_l[i];
//        }
//        externalForcesTable.appendRow(optTime[count],row);
//
//        // Express as three forces, COP, and one torque
//        SimTK::Vec3 pApp2_r(0);
//        pApp2_r[0] = torques_r[2]/forces_r[1];
//        pApp2_r[2] = -torques_r[0]/forces_r[1];
//        SimTK::Vec3 torques2_r(0);
//        torques2_r[1] = torques_r[1] + torques_r[0]/forces_r[1]*forces_r[0] +
//                torques_r[2]/forces_r[1]*forces_r[2];
//        SimTK::Vec3 pApp2_l(0);
//        pApp2_l[0] = torques_l[2]/forces_l[1];
//        pApp2_l[2] = -torques_l[0]/forces_l[1];
//        SimTK::Vec3 torques2_l(0);
//        torques2_l[1] = torques_l[1] + torques_l[0]/forces_l[1]*forces_l[0] +
//                torques_l[2]/forces_l[1]*forces_l[2];
//
//        // Create row
//        SimTK::RowVector row2{18,0.0};
//        for (int i = 0; i < 3; ++i) {
//            row2(i) = forces_r[i];
//            row2(i+3) = pApp2_r[i];
//            row2(i+6) = forces_l[i];
//            row2(i+9) = pApp2_l[i];
//            row2(i+12) = torques2_r[i];
//            row2(i+15) = torques2_l[i];
//        }
//        externalForcesTable2.appendRow(optTime[count],row2);
//        ++count;
//    }
//    // Write file
//    writeTableToFile(externalForcesTable,"test_GRF_A.mot");
//    writeTableToFile(externalForcesTable2,"test_GRF_B.mot");
//}

int main() {
    //try {
    //    testCoordinateTracking();
    //}
    //    catch(const std::exception& e){
    //        std::cout << e.what() << std::endl;
    //    }
   testCoordinateTracking_CoordinateActuators();
}
