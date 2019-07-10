///* -------------------------------------------------------------------------- *
// * OpenSim Moco: examplePredictive.cpp                                        *
// * -------------------------------------------------------------------------- *
// * Copyright (c) 2017 Stanford University and the Authors                     *
// *                                                                            *
// * Author(s): Antoine Falisse                                                 *
// *                                                                            *
// * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
// * not use this file except in compliance with the License. You may obtain a  *
// * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
// *                                                                            *
// * Unless required by applicable law or agreed to in writing, software        *
// * distributed under the License is distributed on an "AS IS" BASIS,          *
// * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
// * See the License for the specific language governing permissions and        *
// * limitations under the License.                                             *
// * -------------------------------------------------------------------------- */
//
//#include <Moco/osimMoco.h>
//
//using namespace OpenSim;
//
//
//int main() {
//
//    MocoTrack track;
//    track.setName("gait2D_TrackingMuscle");
//
//    ModelProcessor modelprocessor = ModelProcessor("gait_2D_muscle.osim");
//
//    // Set model
//    track.setModel(modelprocessor);
//    track.setStatesReference(TableProcessor("IK_reference.sto"));
//    track.set_states_global_tracking_weight(1000.0);
//    track.set_allow_unused_references(true);
//    track.set_track_reference_position_derivatives(true);
//    track.set_initial_time(0.0);
//    track.set_final_time(0.47008941);
//
//    MocoStudy moco = track.initialize();
//    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
//    solver.set_dynamics_mode("implicit");
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(4);
//    solver.set_optim_constraint_tolerance(4);
//    solver
//
//
//    MocoSolution solution = moco.solve();
//    /*moco.visualize(solution);*/
//
//}
//
//
////    // Bounds.
////    // -------
////
////    // States: joint positions and velocities
////    // Ground pelvis
////
////    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_rz/value", {-10, 10});
////    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_rz/speed", {-10, 10});
////    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_tx/value", {0, 10},{0});
////    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_tx/speed", {-10, 10});
////    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_ty/value", {-10, 10});
////    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_ty/speed", {-10, 10});
////    // Hip left
////    problem.setStateInfo("/jointset/hip_l/hip_q_l/value", {-10, 10});
////    problem.setStateInfo("/jointset/hip_l/hip_q_l/speed", {-10, 10});
////    // Hip right
////    problem.setStateInfo("/jointset/hip_r/hip_q_r/value", {-10, 10});
////    problem.setStateInfo("/jointset/hip_r/hip_q_r/speed", {-10, 10});
////    // Knee left
////    problem.setStateInfo("/jointset/knee_l/knee_q_l/value", {-10, 10});
////    problem.setStateInfo("/jointset/knee_l/knee_q_l/speed", {-10, 10});
////    // Knee right
////    problem.setStateInfo("/jointset/knee_r/knee_q_r/value", {-10, 10});
////    problem.setStateInfo("/jointset/knee_r/knee_q_r/speed", {-10, 10});
////    // Ankle left
////    problem.setStateInfo("/jointset/ankle_l/ankle_q_l/value", {-10, 10});
////    problem.setStateInfo("/jointset/ankle_l/ankle_q_l/speed", {-10, 10});
////    // Ankle right
////    problem.setStateInfo("/jointset/ankle_r/ankle_q_r/value", {-10, 10});
////    problem.setStateInfo("/jointset/ankle_r/ankle_q_r/speed", {-10, 10});
////    // Lumbar
////    problem.setStateInfo("/jointset/lumbar/lumbar_q/value", {-10, 10});
////    problem.setStateInfo("/jointset/lumbar/lumbar_q/speed", {-10, 10});
////
////    // States: muscle activations
////    // Right
////    problem.setStateInfo("/forceset/hamstrings_r/activation", {0, 1});
////    problem.setStateInfo("/forceset/bifemsh_r/activation", {0, 1});
////    problem.setStateInfo("/forceset/glut_max_r/activation", {0, 1});
////    problem.setStateInfo("/forceset/iliopsoas_r/activation", {0, 1});
////    problem.setStateInfo("/forceset/rect_fem_r/activation", {0, 1});
////    problem.setStateInfo("/forceset/vasti_r/activation", {0, 1});
////    problem.setStateInfo("/forceset/gastroc_r/activation", {0, 1});
////    problem.setStateInfo("/forceset/soleus_r/activation", {0, 1});
////    problem.setStateInfo("/forceset/tib_ant_r/activation", {0, 1});
////    // Left
////    problem.setStateInfo("/forceset/hamstrings_l/activation", {0, 1});
////    problem.setStateInfo("/forceset/bifemsh_l/activation", {0, 1});
////    problem.setStateInfo("/forceset/glut_max_l/activation", {0, 1});
////    problem.setStateInfo("/forceset/iliopsoas_l/activation", {0, 1});
////    problem.setStateInfo("/forceset/rect_fem_l/activation", {0, 1});
////    problem.setStateInfo("/forceset/vasti_l/activation", {0, 1});
////    problem.setStateInfo("/forceset/gastroc_l/activation", {0, 1});
////    problem.setStateInfo("/forceset/soleus_l/activation", {0, 1});
////    problem.setStateInfo("/forceset/tib_ant_l/activation", {0, 1});
////
////    // Controls: torque actuators
////    problem.setControlInfo("/groundPelvisAct_rz", {-150, 150});
////    problem.setControlInfo("/groundPelvisAct_tx", {-150, 150});
////    problem.setControlInfo("/groundPelvisAct_ty", {-150, 150});
////    problem.setControlInfo("/lumbarAct", {-150, 150});
////    // Controls: muscle excitations
////    // Right
////    problem.setControlInfo("/forceset/hamstrings_r", {0, 1});
////    problem.setControlInfo("/forceset/bifemsh_r", {0, 1});
////    problem.setControlInfo("/forceset/glut_max_r", {0, 1});
////    problem.setControlInfo("/forceset/iliopsoas_r", {0, 1});
////    problem.setControlInfo("/forceset/rect_fem_r", {0, 1});
////    problem.setControlInfo("/forceset/vasti_r", {0, 1});
////    problem.setControlInfo("/forceset/gastroc_r", {0, 1});
////    problem.setControlInfo("/forceset/soleus_r", {0, 1});
////    problem.setControlInfo("/forceset/tib_ant_r", {0, 1});
////    // Left
////    problem.setControlInfo("/forceset/hamstrings_l", {0, 1});
////    problem.setControlInfo("/forceset/bifemsh_l", {0, 1});
////    problem.setControlInfo("/forceset/glut_max_l", {0, 1});
////    problem.setControlInfo("/forceset/iliopsoas_l", {0, 1});
////    problem.setControlInfo("/forceset/rect_fem_l", {0, 1});
////    problem.setControlInfo("/forceset/vasti_l", {0, 1});
////    problem.setControlInfo("/forceset/gastroc_l", {0, 1});
////    problem.setControlInfo("/forceset/soleus_l", {0, 1});
////    problem.setControlInfo("/forceset/tib_ant_l", {0, 1});
////
////    // Static parameter: final time
////    double finalTime = 1.0;
////    problem.setTimeBounds(0, finalTime);
////
////    ////// Cost.
////    ////// -----
////    //// Minimize torque actuators squared
////    //auto* controlCost = problem.addCost<MocoControlCost>("controlCost");
////    //controlCost->set_weight(1);
////
////    //// Impose average speed
////    //auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
////    //speedCost->set_weight(1);
////    //speedCost->set_desired_speed(1.2);
////
////    MocoStateTrackingCost tracking;
////
////
////    // Configure the solver.
////    // =====================
////    auto& solver = moco.initCasADiSolver();
////    solver.set_num_mesh_points(50);
////    solver.set_verbosity(2);
////    solver.set_optim_solver("ipopt");
////
////    moco.print("gait2D_PredictiveMuscle.omoco");
////
////    // Solve the problem.
////    // ==================
////    MocoSolution solution = moco.solve();
////    solution.write("gait2D_PredictiveMuscle_solution.sto");
////
////    moco.visualize(solution);
////
////    return EXIT_SUCCESS;
////
////return 0;
////}
