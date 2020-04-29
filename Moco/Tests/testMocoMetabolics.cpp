/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoMetabolics.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

#define CATCH_CONFIG_MAIN
#include "Testing.h"

using namespace OpenSim;

TEST_CASE("Bhargava2004Metabolics basics") {

    Model model;
    model.setName("muscle");
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("x");
    model.addComponent(joint);
    auto* musclePtr = new DeGrooteFregly2016Muscle();
    musclePtr->set_ignore_tendon_compliance(false);
    musclePtr->set_fiber_damping(0.1);
    musclePtr->setName("muscle");
    musclePtr->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    musclePtr->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(musclePtr);
    auto& muscle = model.getComponent<DeGrooteFregly2016Muscle>("muscle");
    // Add non-smooth metabolics
    auto metabolicsPtr_nonsmooth = new Bhargava2004Metabolics();
    metabolicsPtr_nonsmooth->setName("metabolics_nonsmooth");
    metabolicsPtr_nonsmooth->set_use_smoothing(false);
    metabolicsPtr_nonsmooth->set_include_negative_mechanical_work(false);
    metabolicsPtr_nonsmooth->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_nonsmooth);
    auto& metabolics_nonsmooth =
        model.getComponent<Bhargava2004Metabolics>("metabolics_nonsmooth");
    // Add smooth metabolics
    auto metabolicsPtr_smooth = new Bhargava2004Metabolics();
    metabolicsPtr_smooth->setName("metabolics_smooth");
    metabolicsPtr_smooth->set_use_smoothing(true);
    // We set a high value for the velocity smoothing parameter so that
    // the tanh transition is very steep and the smooth model best approximates
    // the non-smooth model. In pratice we use a lower value (default is 10).
    metabolicsPtr_smooth->set_velocity_smoothing(1e6);
    metabolicsPtr_smooth->set_include_negative_mechanical_work(false);
    metabolicsPtr_smooth->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_smooth);
    auto& metabolics_smooth =
        model.getComponent<Bhargava2004Metabolics>("metabolics_smooth");
    model.finalizeConnections();

    SECTION("Verify computed values") {
        auto state = model.initSystem();
        double excitation = 0.8;
        double activation = 1.0;
        double fiberLength = muscle.get_optimal_fiber_length() + 0.05;
        double tendonLength = muscle.get_tendon_slack_length();
        const double Vmax = muscle.get_optimal_fiber_length() *
                            muscle.get_max_contraction_velocity();
        muscle.setActivation(state, activation);
        coord.setValue(state, fiberLength + tendonLength);

        SECTION("Smooth vs non-smooth metabolics models with concentratic "
                "and eccentric contractions") {

            for (double speed = -0.1; speed <= 0.1; speed += 0.01) {

                coord.setSpeedValue(state, speed * Vmax);

                model.realizeVelocity(state);
                muscle.computeInitialFiberEquilibrium(state);
                SimTK::Vector& controls(model.updControls(state));
                muscle.setControls(SimTK::Vector(1, excitation), controls);
                model.setControls(state, controls);

                model.realizeDynamics(state);

                CHECK(metabolics_nonsmooth.getTotalShorteningRate(state) ==
                    Approx(metabolics_smooth.getTotalShorteningRate(state)).margin(1e-6));
                CHECK(metabolics_nonsmooth.getTotalMechanicalWorkRate(state) ==
                    Approx(metabolics_smooth.getTotalMechanicalWorkRate(state)).margin(1e-6));
                CHECK(metabolics_nonsmooth.getTotalMetabolicRate(state) ==
                    Approx(metabolics_smooth.getTotalMetabolicRate(state)).margin(1e-6));
            }
        }

        SECTION("mechanicalWorkRate=0 if fiberVelocity=0") {

            double speed = 0;
            coord.setSpeedValue(state, speed);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            muscle.setControls(SimTK::Vector(1, excitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            CHECK(metabolics_nonsmooth.getTotalMechanicalWorkRate(state) ==
                    Approx(0).margin(1e-6));
            CHECK(metabolics_smooth.getTotalMechanicalWorkRate(state) ==
                    Approx(0).margin(1e-6));
        }

        SECTION("activationHeatRate=0 and maintenanceHeatRate=0 if "
            "muscleExcitation=0") {

            double nullExcitation = 0.0;
            muscle.setActivation(state, activation);
            double speed = -0.4;
            coord.setSpeedValue(state, speed);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            muscle.setControls(SimTK::Vector(1, nullExcitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            CHECK(metabolics_nonsmooth.getTotalActivationRate(state) ==
                    Approx(0.0));
            CHECK(metabolics_smooth.getTotalActivationRate(state) ==
                    Approx(0.0));
            CHECK(metabolics_nonsmooth.getTotalMaintenanceRate(state) ==
                    Approx(0.0));
            CHECK(metabolics_smooth.getTotalMaintenanceRate(state) ==
                    Approx(0.0));
        }
    }
}
