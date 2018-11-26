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

Model createWeldedLeg39Model(const std::string& actuatorType) {
    Model model("Leg39.osim");
    model.finalizeConnections(); // need this here to access offset frames

    // Remove ground_pelvis joint and replace with weld
    auto& ground_pelvis = model.updJointSet().get("ground_pelvis");
    model.updJointSet().remove(&ground_pelvis);
    const auto& pelvis = model.getBodySet().get("pelvis");
    auto* ground_pelvis_weld = new WeldJoint("ground_pelvis", model.getGround(),
        Vec3(0, 1.2, 0), Vec3(0), pelvis, Vec3(0), Vec3(0));
    model.addJoint(ground_pelvis_weld);

    // Remove subtalar joint and replace with weld
    auto& subtalar_r = model.updJointSet().get("subtalar_r");
    PhysicalOffsetFrame* talus_r_offset_subtalar_r
        = PhysicalOffsetFrame().safeDownCast(subtalar_r.getParentFrame().clone());
    PhysicalOffsetFrame* calcn_r_offset_subtalar_r
        = PhysicalOffsetFrame().safeDownCast(subtalar_r.getChildFrame().clone());
    model.updJointSet().remove(&subtalar_r);
    auto* subtalar_r_weld = new WeldJoint("subtalar_r",
        model.getBodySet().get("talus_r"),
        talus_r_offset_subtalar_r->get_translation(),
        talus_r_offset_subtalar_r->get_orientation(),
        model.getBodySet().get("calcn_r"),
        calcn_r_offset_subtalar_r->get_translation(),
        calcn_r_offset_subtalar_r->get_orientation());
    model.addJoint(subtalar_r_weld);

    // Remove mtp joint and replace with weld
    auto& mtp_r = model.updJointSet().get("mtp_r");
    PhysicalOffsetFrame* calcn_r_offset_mtp_r
        = PhysicalOffsetFrame().safeDownCast(mtp_r.getParentFrame().clone());
    PhysicalOffsetFrame* toes_r_offset_mtp_r
        = PhysicalOffsetFrame().safeDownCast(mtp_r.getChildFrame().clone());
    model.updJointSet().remove(&mtp_r);
    auto* mtp_r_weld = new WeldJoint("mtp_r",
        model.getBodySet().get("calcn_r"),
        calcn_r_offset_mtp_r->get_translation(),
        calcn_r_offset_mtp_r->get_orientation(),
        model.getBodySet().get("toes_r"),
        toes_r_offset_mtp_r->get_translation(),
        toes_r_offset_mtp_r->get_orientation());
    model.addJoint(mtp_r_weld);

    if (actuatorType == "torques") { 
        // Remove muscles and add coordinate actuators
        addCoordinateActuator(model, "hip_angle_r", 50);
        addCoordinateActuator(model, "knee_angle_r", 50);
        addCoordinateActuator(model, "ankle_angle_r", 50);
        //addCoordinateActuator(model, "tib_tx_r", 35);
        //addCoordinateActuator(model, "tib_ty_r", 150);
        //addCoordinateActuator(model, "pat_angle_r", 5);
        //addCoordinateActuator(model, "pat_tx_r", 5);
        //addCoordinateActuator(model, "pat_ty_r", 5);
        removeMuscles(model);
    } else if (actuatorType == "path_actuators") {
        replaceMusclesWithPathActuators(model);
    } else {
        OPENSIM_THROW(Exception, "Invalid actuator type");
    }

    // Finalize model and print.
    model.finalizeFromProperties();
    model.finalizeConnections();
    model.print("WeldedLeg39_" + actuatorType + ".osim");

    return model;
}

Model createLeg39Model(const std::string& actuatorType) {
    Model model("Leg39.osim");
    model.finalizeConnections(); // need this here to access offset frames

    // Remove subtalar joint and replace with weld
    auto& subtalar_r = model.updJointSet().get("subtalar_r");
    PhysicalOffsetFrame* talus_r_offset_subtalar_r
        = PhysicalOffsetFrame().safeDownCast(subtalar_r.getParentFrame().clone());
    PhysicalOffsetFrame* calcn_r_offset_subtalar_r
        = PhysicalOffsetFrame().safeDownCast(subtalar_r.getChildFrame().clone());
    model.updJointSet().remove(&subtalar_r);
    auto* subtalar_r_weld = new WeldJoint("subtalar_r",
        model.getBodySet().get("talus_r"),
        talus_r_offset_subtalar_r->get_translation(),
        talus_r_offset_subtalar_r->get_orientation(),
        model.getBodySet().get("calcn_r"),
        calcn_r_offset_subtalar_r->get_translation(),
        calcn_r_offset_subtalar_r->get_orientation());
    model.addJoint(subtalar_r_weld);

    // Remove mtp joint and replace with weld
    auto& mtp_r = model.updJointSet().get("mtp_r");
    PhysicalOffsetFrame* calcn_r_offset_mtp_r
        = PhysicalOffsetFrame().safeDownCast(mtp_r.getParentFrame().clone());
    PhysicalOffsetFrame* toes_r_offset_mtp_r
        = PhysicalOffsetFrame().safeDownCast(mtp_r.getChildFrame().clone());
    model.updJointSet().remove(&mtp_r);
    auto* mtp_r_weld = new WeldJoint("mtp_r",
        model.getBodySet().get("calcn_r"),
        calcn_r_offset_mtp_r->get_translation(),
        calcn_r_offset_mtp_r->get_orientation(),
        model.getBodySet().get("toes_r"),
        toes_r_offset_mtp_r->get_translation(),
        toes_r_offset_mtp_r->get_orientation());
    model.addJoint(mtp_r_weld);

    addCoordinateActuator(model, "pelvis_tx", 1000);
    addCoordinateActuator(model, "pelvis_ty", 1000);
    addCoordinateActuator(model, "pelvis_tz", 1000);
    addCoordinateActuator(model, "pelvis_tilt", 50);
    if (actuatorType == "torques") {
        // Remove muscles and add coordinate actuators
        addCoordinateActuator(model, "hip_angle_r", 20);
        addCoordinateActuator(model, "knee_angle_r", 20);
        addCoordinateActuator(model, "ankle_angle_r", 5);
        addCoordinateActuator(model, "tib_tx_r", 35);
        addCoordinateActuator(model, "tib_ty_r", 150);
        addCoordinateActuator(model, "pat_angle_r", 5);
        addCoordinateActuator(model, "pat_tx_r", 5);
        addCoordinateActuator(model, "pat_ty_r", 5);
        removeMuscles(model);
    } else if (actuatorType == "path_actuators") {
        replaceMusclesWithPathActuators(model);
    } else {
        OPENSIM_THROW(Exception, "Invalid actuator type");
    }

    // Finalize model and print.
    model.finalizeFromProperties();
    model.finalizeConnections();
    model.print("Leg39_" + actuatorType + ".osim");

    return model;
}

void minimizeControlEffortWeldedLeg39(const std::string& actuatorType) {
    MucoTool muco;
    muco.setName("sandboxLeg39_welded_" + actuatorType + 
        "_minimize_control_effort");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createWeldedLeg39Model(actuatorType));

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

    mp.setMultiplierBounds({-100, 100});

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(20);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-1);
    ms.set_optim_hessian_approximation("exact");
    //ms.set_multiplier_weight(0);
    ms.setGuess("bounds");

    MucoSolution solution = muco.solve();
    muco.visualize(solution);
}

void stateTrackingWeldedLeg39(const std::string& actuatorType) {
    MucoTool muco;
    muco.setName("sandboxLeg39_welded_" + actuatorType +
        "_state_tracking");
    MucoProblem& mp = muco.updProblem();
    Model model = createWeldedLeg39Model(actuatorType);
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

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-6);
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

void markerTrackingLeg39(const std::string& actuatorType) {
    MucoTool muco;
    muco.setName("sandboxLeg39_" + actuatorType +
        "_marker_tracking");
    MucoProblem& mp = muco.updProblem();
    Model model = createLeg39Model(actuatorType);
    mp.setModel(model);

    auto markersRef = TRCFileAdapter::read("leg39_swing.trc");
    auto time = markersRef.getIndependentColumn();
    mp.setTimeBounds(0, time.back());

    MucoMarkerTrackingCost markerTracking;
    markerTracking.setName("marker_tracking");
    InverseKinematicsTool iktool("leg39_swing_IK_Setup.xml");
    IKTaskSet& tasks = iktool.getIKTaskSet();
    Set<MarkerWeight> markerWeights;
    tasks.createMarkerWeightSet(markerWeights);
    markerTracking.setMarkersReference(MarkersReference(markersRef, 
        &markerWeights));
    markerTracking.setAllowUnusedReferences(true);
    mp.addCost(markerTracking);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(75);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.set_optim_hessian_approximation("exact");

    //MucoIterate guess = ms.createGuess();
    //guess.setStatesTrajectory(
    //    STOFileAdapter::read("Leg39_swing_IK_results_radians.sto"), true, true);
    ms.setGuessFile("sandboxLeg39_torques_marker_tracking_solution.sto");

    MucoSolution solution = muco.solve();
    muco.visualize(solution);
}

void main() {

    //minimizeControlEffortWeldedLeg39("torques");
    stateTrackingWeldedLeg39("torques");
    //markerTrackingLeg39("torques");
    //minimizeControlEffortWeldedLeg39("path_actuators");

}
