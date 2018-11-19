/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxLeg39.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>

using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Inertia;

/// Convenience function to apply an CoordinateActuator to the model.
void addCoordinateActuator(Model& model, std::string coordName,
    double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new CoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    actu->setMinControl(-1);
    actu->setMaxControl(1);
    model.addComponent(actu);
}

void minimizeControlEffortLeg39Welded() {
    MucoTool muco;
    muco.setName("minimize_control_effort_leg39welded");
    MucoProblem& mp = muco.updProblem();
    Model model("Leg39.osim");

    // Remove ground_pelvis joint and replace with weld
    auto& ground_pelvis = model.updJointSet().get("ground_pelvis");
    model.updJointSet().remove(&ground_pelvis);
    const auto& pelvis = model.getBodySet().get("pelvis");
    auto* gpWeld = new WeldJoint("ground_pelvis", model.getGround(),
        Vec3(0, 1.2, 0), Vec3(0), pelvis, Vec3(0), Vec3(0));
    model.addJoint(gpWeld);

    // Remove muscles and add coordinate actuators
    addCoordinateActuator(model, "hip_angle_r", 100);
    addCoordinateActuator(model, "knee_angle_r", 100);
    addCoordinateActuator(model, "ankle_angle_r", 100);
    addCoordinateActuator(model, "tib_tx_r", 100);
    addCoordinateActuator(model, "tib_ty_r", 100);
    addCoordinateActuator(model, "pat_angle_r", 100);
    addCoordinateActuator(model, "pat_tx_r", 100);
    addCoordinateActuator(model, "pat_ty_r", 100);
    removeMuscles(model);

    // Finalize model and print.
    model.finalizeConnections();
    model.print("Leg39_welded.osim");
    model.printSubcomponentInfo();
    mp.setModel(model);

    // Set bounds.
    mp.setTimeBounds(0, 1);
    mp.setStateInfo("/jointset/knee_r/knee_angle_r/value", {-10, 10}, -0.6, 0);
    mp.setStateInfo("/jointset/hip_r/hip_angle_r/value", {-10, 10},
        -SimTK::Pi / 4.0, SimTK::Pi / 4.0);
    mp.setStateInfo("/jointset/ankle_r/ankle_angle_r/value", {-1, 1}, -0.1,
        0.1);

    MucoControlCost effort;
    effort.setName("control_effort");
    mp.addCost(effort);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-6);
    ms.set_optim_hessian_approximation("exact");
    ms.set_multiplier_weight(1.0);
    ms.setGuessFile("minimize_control_effort_leg39welded_solution.sto");

    MucoSolution solution = muco.solve();
    //solution.write("sandboxJointReaction_minimizeJointReaction.sto");
    muco.visualize(solution);
}

void stateTrackingLeg39Welded() {
    MucoTool muco;
    muco.setName("state_tracking_leg39welded");
    MucoProblem& mp = muco.updProblem();
    Model model("Leg39.osim");

    // Remove ground_pelvis joint and replace with weld
    auto& ground_pelvis = model.updJointSet().get("ground_pelvis");
    model.updJointSet().remove(&ground_pelvis);
    const auto& pelvis = model.getBodySet().get("pelvis");
    auto* ground_pelvis_weld = new WeldJoint("ground_pelvis", model.getGround(),
        Vec3(0, 1.2, 0), Vec3(0), pelvis, Vec3(0), Vec3(0));
    model.addJoint(ground_pelvis_weld);

    // Remove subtalar joint and replace with weld
    auto& subtalar_r = model.getJointSet().get("subtalar_r");
    model.updJointSet().remove(&subtalar_r);
    const auto& talus_r = model.getBodySet().get("talus_r");
    const auto& calcn_r = model.getBodySet().get("calcn_r");
    //const auto& talus_r_offset = subtalar_r.getConnectee("talus_r_offset");


    auto* subtalar_r_weld = new WeldJoint("subtalar_r", talus_r, calcn_r);
    model.addJoint(subtalar_r_weld);

    // Remove mtp joint and replace with weld
    auto& mtp_r = model.updJointSet().get("mtp_r");
    model.updJointSet().remove(&mtp_r);
    const auto& toes_r = model.getBodySet().get("toes_r");
    auto* mtp_r_weld = new WeldJoint("mtp_r", calcn_r, toes_r);
    model.addJoint(mtp_r_weld);

    // Remove muscles and add coordinate actuators
    addCoordinateActuator(model, "hip_angle_r", 50);
    addCoordinateActuator(model, "knee_angle_r", 50);
    addCoordinateActuator(model, "ankle_angle_r", 10);
    addCoordinateActuator(model, "tib_tx_r", 50);
    addCoordinateActuator(model, "tib_ty_r", 250);
    addCoordinateActuator(model, "pat_angle_r", 10);
    addCoordinateActuator(model, "pat_tx_r", 50);
    addCoordinateActuator(model, "pat_ty_r", 50);
    removeMuscles(model);

    // Finalize model and print.
    model.finalizeConnections();
    model.print("Leg39_welded.osim");
    model.printSubcomponentInfo();
    mp.setModel(model);

    auto markersRef = TRCFileAdapter::read("leg39_swing.trc");
    auto time = markersRef.getIndependentColumn();
    mp.setTimeBounds(0, time.back());

    MucoStateTrackingCost stateTracking;
    stateTracking.setName("state_tracking");
    auto statesRef = STOFileAdapter::read("Leg39_swing_IK_results.sto");
    stateTracking.setReference(statesRef);
    stateTracking.setWeight("/jointset/knee_r/tib_tx_r/value", 0);
    stateTracking.setWeight("/jointset/knee_r/tib_ty_r/value", 0);
    stateTracking.setWeight("/jointset/tib_pat_r/pat_angle_r/value", 0);
    stateTracking.setWeight("/jointset/tib_pat_r/pat_tx_r/value", 0);
    stateTracking.setWeight("/jointset/tib_pat_r/pat_ty_r/value", 0);
    stateTracking.setAllowUnusedReferences(true);
    mp.addCost(stateTracking);

    //MucoControlCost controlEffort;
    //controlEffort.setName("control_effort");
    //controlEffort.set_weight(0.1);
    //mp.addCost(controlEffort);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-4);
    ms.set_optim_hessian_approximation("exact");

    MucoIterate guess = ms.createGuess();
    model.initSystem();
    model.getSimbodyEngine().convertDegreesToRadians(statesRef);
    STOFileAdapter::write(statesRef, "Leg39_swing_IK_results_radians.sto");
    guess.setStatesTrajectory(statesRef, true, true);
    ms.setGuess(guess);

    MucoSolution solution = muco.solve();
    muco.visualize(solution);
}

//void markerTrackingLeg39Welded() {
//    MucoTool muco;
//    muco.setName("minimize_control_effort_leg39welded");
//    MucoProblem& mp = muco.updProblem();
//    Model model("Leg39.osim");
//
//    // Remove ground_pelvis joint and replace with weld
//    auto& ground_pelvis = model.updJointSet().get("ground_pelvis");
//    model.updJointSet().remove(&ground_pelvis);
//    const auto& pelvis = model.getBodySet().get("pelvis");
//    auto* gpWeld = new WeldJoint("ground_pelvis", model.getGround(),
//        Vec3(0, 1.2, 0), Vec3(0), pelvis, Vec3(0), Vec3(0));
//    model.addJoint(gpWeld);
//
//    // Remove muscles and add coordinate actuators
//    addCoordinateActuator(model, "hip_angle_r", 100);
//    addCoordinateActuator(model, "knee_angle_r", 100);
//    addCoordinateActuator(model, "ankle_angle_r", 100);
//    addCoordinateActuator(model, "tib_tx_r", 100);
//    addCoordinateActuator(model, "tib_ty_r", 100);
//    addCoordinateActuator(model, "pat_angle_r", 100);
//    addCoordinateActuator(model, "pat_tx_r", 100);
//    addCoordinateActuator(model, "pat_ty_r", 100);
//    removeMuscles(model);
//
//    // Finalize model and print.
//    model.finalizeConnections();
//    model.print("Leg39_welded.osim");
//    model.printSubcomponentInfo();
//    mp.setModel(model);
//
//    auto markersRef = TRCFileAdapter::read("leg39_swing.trc");
//    auto time = markersRef.getIndependentColumn();
//    mp.setTimeBounds(0, time.back());
//    //mp.setStateInfo("/jointset/ground_pelvis/pelvis_ty/value", {-1.15, -1.10});
//    //mp.setStateInfo("/jointset/ground_pelvis/pelvis_tx/value", {-0.05, 0});
//
//    MucoStateTrackingCost stateTracking;
//    auto statesRef = STOFileAdapter::read("Leg39_swing_IK_results.sto");
//    stateTracking.setReference(statesRef);
//    stateTracking.setWeight("/jointset/knee_r/tib_tx_r/value", 0);
//    stateTracking.setWeight("/jointset/knee_r/tib_ty_r/value", 0);
//    stateTracking.setWeight("/jointset/tib_pat_r/pat_angle_r/value", 0);
//    stateTracking.setWeight("/jointset/tib_pat_r/pat_tx_r/value", 0);
//    stateTracking.setWeight("/jointset/tib_pat_r/pat_ty_r/value", 0);
//    stateTracking.setWeight("/jointset/mtp_r/mtp_angle_r/value", 0);
//    stateTracking.setWeight("/jointset/subtalar_r/subtalar_angle_r/value", 0);
//    stateTracking.setAllowUnusedReferences(true);
//
//    mp.addCost(stateTracking);
//
//    //MucoMarkerTrackingCost markerTracking;
//    //markerTracking.setName("marker_tracking");
//    //InverseKinematicsTool iktool("leg39_swing_IK_Setup.xml");
//    //IKTaskSet& tasks = iktool.getIKTaskSet();
//    //Set<MarkerWeight> markerWeights;
//    //tasks.createMarkerWeightSet(markerWeights);
//    //markerTracking.setMarkersReference(MarkersReference(markersRef, 
//    //    &markerWeights));
//    //markerTracking.setAllowUnusedReferences(true);
//    //mp.addCost(markerTracking);
//
//    MucoTropterSolver& ms = muco.initSolver();
//    ms.set_num_mesh_points(25);
//    ms.set_verbosity(2);
//    ms.set_optim_solver("ipopt");
//    ms.set_optim_convergence_tolerance(1e-4);
//    ms.set_optim_hessian_approximation("exact");
//
//    MucoIterate guess = ms.createGuess();
//    model.initSystem();
//    model.getSimbodyEngine().convertDegreesToRadians(statesRef);
//    STOFileAdapter::write(statesRef, "Leg39_swing_IK_results_radians.sto");
//    guess.setStatesTrajectory(statesRef, true, true);
//    ms.setGuess(guess);
//
//    MucoSolution solution = muco.solve();
//    //solution.write("sandboxJointReaction_minimizeJointReaction.sto");
//    muco.visualize(solution);
//}

void main() {

    //minimizeControlEffortLeg39Welded();
    stateTrackingLeg39Welded();
}