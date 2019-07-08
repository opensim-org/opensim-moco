/* -------------------------------------------------------------------------- *
 * OpenSim Moco: examplePredictive.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

/// This model is torque-actuated.
std::unique_ptr<Model> createGait2D() {
    auto model = make_unique<Model>("gait_2D_torque.osim");

    return model;
}

//int main() {
//
//    MocoStudy moco;
//    moco.setName("gait2D_TrackingTorque");
//
//    // Define the optimal control problem.
//    // ===================================
//    MocoProblem& problem = moco.updProblem();
//
//    // Model (dynamics).
//    // -----------------
//    problem.setModel(createGait2D());
//
//    // Bounds.
//    // -------
//
//    // States: joint positions and velocities
//    // Ground pelvis
//    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_rz/value", {-10, 10});
//    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_rz/speed", {-10, 10});
//    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_tx/value", {-10, 10});
//    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_tx/speed", {-10, 10});
//    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_ty/value", {-10, 10});
//    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_ty/speed", {-10, 10});
//    // Hip left
//    problem.setStateInfo("/jointset/hip_l/hip_q_l/value", {-10, 10});
//    problem.setStateInfo("/jointset/hip_l/hip_q_l/speed", {-10, 10});
//    // Hip right
//    problem.setStateInfo("/jointset/hip_r/hip_q_r/value", {-10, 10});
//    problem.setStateInfo("/jointset/hip_r/hip_q_r/speed", {-10, 10});
//    // Knee left
//    problem.setStateInfo("/jointset/knee_l/knee_q_l/value", {-10, 10});
//    problem.setStateInfo("/jointset/knee_l/knee_q_l/speed", {-10, 10});
//    // Knee right
//    problem.setStateInfo("/jointset/knee_r/knee_q_r/value", {-10, 10});
//    problem.setStateInfo("/jointset/knee_r/knee_q_r/speed", {-10, 10});
//    // Ankle left
//    problem.setStateInfo("/jointset/ankle_l/ankle_q_l/value", {-10, 10});
//    problem.setStateInfo("/jointset/ankle_l/ankle_q_l/speed", {-10, 10});
//    // Ankle right
//    problem.setStateInfo("/jointset/ankle_r/ankle_q_r/value", {-10, 10});
//    problem.setStateInfo("/jointset/ankle_r/ankle_q_r/speed", {-10, 10});
//    // Lumbar
//    problem.setStateInfo("/jointset/lumbar/lumbar_q/value", {-10, 10});
//    problem.setStateInfo("/jointset/lumbar/lumbar_q/speed", {-10, 10});
//
//    // Controls: torque actuators
//    problem.setControlInfo("/groundPelvisAct_rz", {-150, 150});
//    problem.setControlInfo("/groundPelvisAct_tx", {-150, 150});
//    problem.setControlInfo("/groundPelvisAct_ty", {-150, 150});
//    problem.setControlInfo("/hipAct_l", {-150, 150});
//    problem.setControlInfo("/hipAct_r", {-150, 150});
//    problem.setControlInfo("/kneeAct_l", {-150, 150});
//    problem.setControlInfo("/kneeAct_r", {-150, 150});
//    problem.setControlInfo("/ankleAct_l", {-150, 150});
//    problem.setControlInfo("/ankleAct_r", {-150, 150});
//    problem.setControlInfo("/lumbarAct", {-150, 150});
//
//    // Static parameter: final time
//    double finalTime_lb = 0.47008941;
//    double finalTime_ub = finalTime_lb;
//    problem.setTimeBounds(0, finalTime_ub);
//
//    auto* trackingCost = problem.addCost<MocoStateTrackingCost>("trackingCost");
//    trackingCost->setReference(TableProcessor("IK_reference.sto"));
//
//    //controlCost->set_weight(1);
//
//    //MocoStateTrackingCost tracking;
//    //TableProcessor ref = TableProcessor("IK_reference.sto");
//    //tracking.setReference(ref);
//
//    auto& solver = moco.initCasADiSolver();
//    //solver.set_dynamics_mode("implicit");
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(4);
//    solver.set_optim_constraint_tolerance(4);
//
//    MocoSolution solution = moco.solve();
//    moco.visualize(solution);
//
//    return EXIT_SUCCESS;
//
//}


/*Use MocoTrack*/
//int main() {
void testTracking(){

    MocoTrack track;
    track.setName("gait2D_TrackingTorque");

    ModelProcessor modelprocessor = ModelProcessor("gait_2D_torque.osim");

    // Set model
    track.setModel(modelprocessor);
    track.setStatesReference(TableProcessor("IK_reference.sto"));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    //track.set_apply_tracked_states_to_guess(true);

    TimeSeriesTable IGTable = TimeSeriesTable("IK_reference_guess.sto");
    //std::cout << IGTable.getRow(0) << std::endl;
    MocoTrajectory IG;
    IG.setStatesTrajectory(IGTable,true,true);
    IG.write("test_MocoTrajectory_IG.sto");
    //track.set_guess_file(IG); // STO or mocoTrajectory?

    //MocoTrajectory IG;
    //IG.setStatesTrajectory(IGTable,true,true);
    ////IG.write("IK_reference_guess_write.sto");
    //track.set_guess_file(IG);
    //track.set_apply_tracked_states_to_guess(true);

    //std::cout << IGTable.getColumnLabel(0) << std::endl;
    //std::cout << IGTable.getRow(0) << std::endl;

    /*TimeSeriesTable ref;
    ref.setColumnLabels({"/jointset/groundPelvis/groundPelvis_q_rz/value",
        "/jointset/groundPelvis/groundPelvis_q_tx/value",
        "/jointset/groundPelvis/groundPelvis_q_ty/value",
        "/jointset/hip_l/hip_q_l/value", "/jointset/hip_r/hip_q_r/value",
        "/jointset/knee_l/knee_q_l/value", "/jointset/knee_r/knee_q_r/value",
        "/jointset/ankle_l/ankle_q_l/value",
        "/jointset/ankle_r/ankle_q_r/value",
        "/jointset/lumbar/lumbar_q/value",
        "/jointset/groundPelvis/groundPelvis_q_rz/speed",
        "/jointset/groundPelvis/groundPelvis_q_tx/speed",
        "/jointset/groundPelvis/groundPelvis_q_ty/speed",
        "/jointset/hip_l/hip_q_l/speed", "/jointset/hip_r/hip_q_r/speed",
        "/jointset/knee_l/knee_q_l/speed", "/jointset/knee_r/knee_q_r/speed",
        "/jointset/ankle_l/ankle_q_l/speed",
        "/jointset/ankle_r/ankle_q_r/speed",
        "/jointset/lumbar/lumbar_q/speed"});
    ref.appendRow(0.0,{-15.12042338, 0, 0.88365423, 0.94361719, 38.0573941,
        -16.23119179, -10.56684797, 5.78097847, 13.83724009, 9.61940188, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0});
    ref.appendRow(0.00959366,{-15.08237687, 0.01477811, 0.88286864, 0.8499583,
        37.57263532, -17.72505032, -9.96145335, 4.58050531, 13.38888806,
        9.71629935, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});*/

    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    MocoStudy moco = track.initialize();
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_dynamics_mode("implicit");
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(4);
    solver.set_optim_constraint_tolerance(4);

    MocoSolution solution = moco.solve();
    moco.visualize(solution);
}

int main()  {
    try { testTracking(); }
    catch (const OpenSim::Exception& e) {
        e.print(std::cerr);
        return 1;
    }
    return 0;
}
