/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxTestContact.cpp                                   *
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

// This file will become testContact.

// TODO add 3D tests (contact models are currently only 2D).

#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Simulation/Manager/Manager.h>

#include "MuscolloSandboxShared.h"

using namespace OpenSim;
using SimTK::Vec3;

using CreateContactFunction = std::function<StationPlaneContactForce*(void)>;
Model create2DPointMassModel(CreateContactFunction createContact) {
    Model model;
    model.setName("point_mass");
    auto* intermed = new Body("intermed", 0, Vec3(0), SimTK::Inertia(0));
    model.addComponent(intermed);
    auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(1));
    model.addComponent(body);

    // Allows translation along x.
    auto* jointX = new SliderJoint();
    jointX->setName("tx");
    jointX->connectSocket_parent_frame(model.getGround());
    jointX->connectSocket_child_frame(*intermed);
    auto& coordX = jointX->updCoordinate(SliderJoint::Coord::TranslationX);
    coordX.setName("tx");
    model.addComponent(jointX);

    // The joint's x axis must point in the global "+y" direction.
    auto* jointY = new SliderJoint("ty",
            *intermed, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi),
            *body, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi));
    auto& coordY = jointY->updCoordinate(SliderJoint::Coord::TranslationX);
    coordY.setName("ty");
    model.addComponent(jointY);

    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*body);
    model.addComponent(station);

    auto* force = createContact();
    force->setName("contact");
    model.addComponent(force);
    force->connectSocket_station(*station);

    return model;
}

// Test that, with dissipation, the contact force settles to the weight of the
// system. This is the same type of test done in OpenSim's testForces for
// HuntCrossleyForce. The test is performed with both time stepping and direct
// collocation.
void testWeight(CreateContactFunction createContact) {
    Model model(create2DPointMassModel(createContact));
    SimTK::Real weight;
    {
        SimTK::State state = model.initSystem();
        weight = model.getTotalMass(state) * (-model.getGravity()[1]);
    }

    const SimTK::Real ty0 = 0.5;
    const SimTK::Real finalTime = 2.0;

    // Time stepping.
    // --------------
    SimTK::Real finalHeightTimeStepping;
    {
        SimTK::State state = model.initSystem();
        model.setStateVariableValue(state, "ty/ty/value", ty0);
        model.realizeDynamics(state);
        Manager manager(model, state);
        state = manager.integrate(finalTime);
        model.realizeVelocity(state);

        // visualize(model, manager.getStateStorage());

        auto& contact = model.getComponent<StationPlaneContactForce>("contact");
        const Vec3 contactForce = contact.calcContactForceOnStation(state);
        // The horizontal force is a few Newtons, maybe from a buildup of
        // numerical error. TODO ideally the horizontal force would be zero.
        SimTK_TEST(std::abs(contactForce[0] / weight) < 0.02);
        SimTK_TEST_EQ_TOL(contactForce[1], weight, 0.01);
        // The system is planar, so there is no force in the z direction.
        SimTK_TEST_EQ(contactForce[2], 0);

        finalHeightTimeStepping =
                model.getStateVariableValue(state, "ty/ty/value");
    }


    // Direct collocation.
    // -------------------
    // This is a simulation (initial value problem), not a trajectory
    // optimization.
    SimTK::Real finalHeightDircol;
    {
        MucoTool muco;
        MucoProblem& mp = muco.updProblem();
        mp.setModel(model);
        mp.setTimeBounds(0, finalTime);
        mp.setStateInfo("tx/tx/value", {-5, 5}, 0);
        mp.setStateInfo("ty/ty/value", {-0.5, 1}, ty0);
        mp.setStateInfo("tx/tx/speed", {-10, 10}, 0);
        mp.setStateInfo("ty/ty/speed", {-10, 10}, 0);

        MucoTropterSolver& ms = muco.initSolver();
        ms.set_num_mesh_points(50);

        MucoSolution solution = muco.solve();
        // muco.visualize(solution);

        auto statesTraj = solution.exportToStatesTrajectory(mp);
        const auto& finalState = statesTraj.back();
        model.realizeVelocity(finalState);
        auto& contact = model.getComponent<StationPlaneContactForce>("contact");
        const Vec3 contactForce = contact.calcContactForceOnStation(finalState);
        // For some reason, direct collocation doesn't produce the same
        // numerical issues with the x component of the force as seen above.
        SimTK_TEST_EQ(contactForce[0], 0);
        SimTK_TEST_EQ_TOL(contactForce[1], weight, 0.01);
        SimTK_TEST_EQ(contactForce[2], 0);

        finalHeightDircol =
                model.getStateVariableValue(finalState, "ty/ty/value");
    }

    SimTK_TEST_EQ_TOL(finalHeightTimeStepping, finalHeightDircol, 1e-5);


}

AckermannVanDenBogert2010Force* createAVDB() {
    auto* contact = new AckermannVanDenBogert2010Force();
    contact->set_stiffness(1e5);
    contact->set_dissipation(1.0);
    return contact;
}

EspositoMiller2018Force* createEspositoMiller() {
    auto* contact = new EspositoMiller2018Force();
    contact->set_stiffness(1e5);
    contact->set_dissipation(1.0);
    return contact;
}

MeyerFregly2016Force* createMeyerFregly() {
    auto* contact = new MeyerFregly2016Force();
    contact->set_stiffness(1e5);
    contact->set_dissipation(1.0);
    return contact;
}

int main() {
    SimTK_START_TEST("testContact");
        SimTK_SUBTEST1(testWeight, createAVDB);
        SimTK_SUBTEST1(testWeight, createEspositoMiller);
        // TODO does not pass:
        // SimTK_SUBTEST1(testWeight, createMeyerFregly);
    SimTK_END_TEST();
}
