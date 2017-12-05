/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxMuscle.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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
#include <OpenSim/OpenSim.h>
#include <Muscollo/osimMuscollo.h>

using namespace OpenSim;

/// Same problem as in testSingleMuscleDeGrooteFregly2016.
Model createHangingMuscleModel() {
    Model model;
    model.setName("hanging_muscle");
    TimeSeriesTable states;
    // +X is down.
    model.set_gravity(SimTK::Vec3(9.81, 0, 0));
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("height");
    model.addComponent(joint);

    auto* actu = new Millard2012EquilibriumMuscle();
    actu->setName("muscle");
    actu->set_max_isometric_force(30);
    actu->set_optimal_fiber_length(0.10);
    actu->set_tendon_slack_length(0.05);
    actu->set_pennation_angle_at_optimal(0.1);
    actu->set_max_contraction_velocity(10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(actu);

    /*
    auto* contr = new PrescribedController();
    contr->setName("controller");
    contr->addActuator(*actu);
    contr->prescribeControlForActuator("muscle", new Constant(0.8));
    model.addComponent(contr);

    auto* rep = new ConsoleReporter();
    rep->setName("reporter");
    rep->set_report_time_interval(0.1);
    rep->addToReport(coord.getOutput("value"), "height");
    rep->addToReport(actu->getOutput("actuation"), "applied_force");
    model.addComponent(rep);

    auto* statesRep = new StatesTrajectoryReporter();
    statesRep->setName("states_reporter");
    // This small interval is helpful for obtaining accurate estimates of
    // generalized accelerations, which are needed for inverse dynamics.
    statesRep->set_report_time_interval(0.001);
    model.addComponent(statesRep);
     */

    return model;
}

int main() {

    MucoTool muco;

    Model model = createHangingMuscleModel();

    MucoProblem& mp = muco.updProblem();
    mp.setModel(model);
    // TODO no exception is thrown if time bounds are not provided.
    mp.setTimeBounds(0, {0.01, 1.0});
    mp.setStateInfo("joint/height/value", {0, 0.3}, 0.15, 0.10);
    mp.setStateInfo("joint/height/speed", {-10, 10}, 0, 0);
    mp.setStateInfo("muscle/activation", {0, 1}, 0);
    mp.setStateInfo("muscle/fiber_length", {0.02, 0.3});
    mp.setControlInfo("muscle", {0, 1});

    mp.addCost(MucoFinalTimeCost());

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_optim_hessian_approximation("exact");

    MucoSolution solution = muco.solve();

    return EXIT_SUCCESS;
}

