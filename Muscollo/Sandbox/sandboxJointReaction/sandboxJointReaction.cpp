/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxJointReaction.cpp                                 *
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

/// This model is torque-actuated.
Model createInvertedPendulumModel() {
    Model model;
    model.setName("inverted_pendulum");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create one link with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* b0 = new OpenSim::Body("b0", 1, Vec3(0), Inertia(1));
    model.addBody(b0);

    // Connect the body to ground with a pin joint. Assume each body is 1 m 
    // long.
    auto* j0 = new PinJoint("j0", model.getGround(), Vec3(0), Vec3(0),
        *b0, Vec3(-1, 0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    model.addJoint(j0);

    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    model.addComponent(tau0);

    // Add display geometry.
    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    SimTK::Transform transform(SimTK::Vec3(-0.5, 0, 0));
    auto* b0Center = new PhysicalOffsetFrame("b0_center", "b0", transform);
    b0->addComponent(b0Center);
    b0Center->attachGeometry(bodyGeometry.clone());

    return model;
}

void minimizeReactionLoadsInvertedPendulum() {
    MucoTool muco;
    muco.setName("minimize_inverted_pendulum_reaction_loads");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createInvertedPendulumModel());

    mp.setTimeBounds(0, 1);
    mp.setStateInfo("j0/q0/value", {-10, 10}, 0, SimTK::Pi);
    mp.setStateInfo("j0/q0/speed", {-50, 50}, 0, 0);
    mp.setControlInfo("tau0", {-100, 100});

    MucoJointReactionNormCost reactionNormCost;
    reactionNormCost.setJointPath("j0");
    mp.addCost(reactionNormCost);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    //ms.set_optim_ipopt_print_level(5);
    ms.set_optim_hessian_approximation("exact");
    ms.setGuess("bounds");

    MucoSolution solution = muco.solve();
    solution.write("sandboxJointReaction_minimizePendulumReactionLoads.sto");
    muco.visualize(solution);
}

void minimizeControlEffortInvertedPendulum() {
    MucoTool muco;
    muco.setName("minimize_inverted_pendulum_control_effort");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createInvertedPendulumModel());

    mp.setTimeBounds(0, 1);
    mp.setStateInfo("j0/q0/value", {-10, 10}, 0, SimTK::Pi);
    mp.setStateInfo("j0/q0/speed", {-50, 50}, 0, 0);
    mp.setControlInfo("tau0", {-100, 100});

    MucoControlCost effort;
    mp.addCost(effort);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    //ms.set_optim_ipopt_print_level(5);
    ms.set_optim_hessian_approximation("exact");
    ms.setGuess("bounds");

    MucoSolution solution = muco.solve();
    solution.write("sandboxJointReaction_minimizeControlEffort.sto");
    muco.visualize(solution);
}

/// This model is torque-actuated.
Model createPendulumPathActuatorModel() {
    Model model;
    model.setName("pendulum_path_actuator");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create one link with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* b0 = new OpenSim::Body("b0", 1, Vec3(0), Inertia(1));
    model.addBody(b0);

    // Connect the body to ground with a pin joint. Assume each body is 1 m 
    // long.
    auto* j0 = new PinJoint("j0", model.getGround(), Vec3(0, 1, 0), Vec3(0),
        *b0, Vec3(-1, 0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    model.addJoint(j0);

    auto* actu = new PathActuator();
    actu->setName("actuator");
    actu->setOptimalForce(1);
    actu->addNewPathPoint("point_on_ground", model.getGround(), Vec3(0.9, 1, 0));
    actu->addNewPathPoint("point_on_actuator", *b0, Vec3(0));
    model.addComponent(actu);

    // Add display geometry.
    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    SimTK::Transform transform(SimTK::Vec3(-0.5, 0, 0));
    auto* b0Center = new PhysicalOffsetFrame("b0_center", "b0", transform);
    b0->addComponent(b0Center);
    b0Center->attachGeometry(bodyGeometry.clone());
    model.print("pendulum_path_actuator.osim");

    return model;
}


void minimizeControlEffortPendulumPathActuator() {
    MucoTool muco;
    muco.setName("minimize_pendulum_path_actuator_control_effort");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createPendulumPathActuatorModel());

    mp.setTimeBounds(0, 1);
    mp.setStateInfo("j0/q0/value", {-10, 10}, -SimTK::Pi / 2.0, 
        -SimTK::Pi / 4.0);
    mp.setStateInfo("j0/q0/speed", {-50, 50}, 0, 0);
    mp.setControlInfo("actuator", {-1000, 1000});

    MucoControlCost effort;
    mp.addCost(effort);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    //ms.set_optim_ipopt_print_level(5);
    ms.set_optim_hessian_approximation("exact");
    ms.setGuess("bounds");

    MucoSolution solution = muco.solve();
    //solution.write("sandboxJointReaction_minimizeControlEffort.sto");
    muco.visualize(solution);
}

void minimizeJointReactionPendulumPathActuator() {
    MucoTool muco;
    muco.setName("minimize_pendulum_path_actuator_joint_reaction");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createPendulumPathActuatorModel());

    mp.setTimeBounds(0, 1);
    mp.setStateInfo("j0/q0/value", {-10, 10}, -SimTK::Pi / 2.0,
        -SimTK::Pi / 4.0);
    mp.setStateInfo("j0/q0/speed", {-50, 50}, 0, 0);
    mp.setControlInfo("actuator", {-1000, 1000});

    MucoJointReactionNormCost reaction;
    reaction.setJointPath("j0");
    mp.addCost(reaction);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    //ms.set_optim_ipopt_print_level(5);
    ms.set_optim_hessian_approximation("exact");
    ms.setGuess("bounds");

    MucoSolution solution = muco.solve();
    //solution.write("sandboxJointReaction_minimizeJointReaction.sto");
    muco.visualize(solution);
}

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

void minimizeControlEffortLeg39() {
    MucoTool muco;
    muco.setName("minimize_control_effort_leg39");
    MucoProblem& mp = muco.updProblem();
    Model model("Leg39.osim");
    addCoordinateActuator(model, "pelvis_tilt", 100);
    addCoordinateActuator(model, "pelvis_tx", 1000);
    addCoordinateActuator(model, "pelvis_ty", 1000);
    addCoordinateActuator(model, "pelvis_tz", 1000);
    addCoordinateActuator(model, "hip_angle_r", 100);
    addCoordinateActuator(model, "knee_angle_r", 100);
    addCoordinateActuator(model, "ankle_angle_r", 100);
    addCoordinateActuator(model, "mtp_angle_r", 50);
    addCoordinateActuator(model, "subtalar_angle_r", 50);

    removeMuscles(model);
    
    model.finalizeFromProperties();
    model.print("Leg39_pathact.osim");
    mp.setModel(model);

    auto markersRef = TRCFileAdapter::read("leg39_swing.trc");
    auto time = markersRef.getIndependentColumn();
    mp.setTimeBounds(0, time.back());

    //MucoControlCost effort;
    //effort.setName("control_effort");
    //effort.set_weight(0.1);
    //mp.addCost(effort);

    MucoStateTrackingCost stateTracking;
    auto statesRef = STOFileAdapter::read("Leg39_swing_IK_results.sto");
    stateTracking.setReference(statesRef);
    //stateTracking.set_weight(10);
    stateTracking.setWeight("knee_r/knee_angle_r/value", 10);
    stateTracking.setWeight("knee_r/tib_tx_r/value", 0);
    stateTracking.setWeight("knee_r/tib_ty_r/value", 0);
    stateTracking.setWeight("tib_pat_r/pat_angle_r/value", 0);
    stateTracking.setWeight("tib_pat_r/pat_tx_r/value", 0);
    stateTracking.setWeight("tib_pat_r/pat_ty_r/value", 0);

    mp.addCost(stateTracking);

    //MucoMarkerTrackingCost markerTracking;
    //markerTracking.setName("marker_tracking");
    //InverseKinematicsTool iktool("leg39_swing_IK_Setup.xml");
    //IKTaskSet& tasks = iktool.getIKTaskSet();
    //Set<MarkerWeight> markerWeights;
    //tasks.createMarkerWeightSet(markerWeights);
    //markerTracking.setMarkersReference(MarkersReference(markersRef, 
    //    &markerWeights));
    //markerTracking.setAllowUnusedReferences(true);
    //mp.addCost(markerTracking);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(10);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    //ms.set_optim_ipopt_print_level(5);
    ms.set_optim_hessian_approximation("exact");
    //ms.set_multiplier_weight(1.0);

    MucoIterate guess = ms.createGuess();
    model.initSystem();
    model.getSimbodyEngine().convertDegreesToRadians(statesRef);
    STOFileAdapter::write(statesRef, "Leg39_swing_IK_results_radians.sto");
    guess.setStatesTrajectory(statesRef, true, true);
    ms.setGuess(guess);



    MucoSolution solution = muco.solve();
    
    //solution.write("sandboxJointReaction_minimizeJointReaction.sto");
    muco.visualize(solution);
}

void main() {

    //minimizeControlEffortInvertedPendulum();
    //minimizeReactionLoadsInvertedPendulum();
    //minimizeControlEffortPendulumPathActuator();
    //minimizeJointReactionPendulumPathActuator();
    minimizeControlEffortLeg39();

}
