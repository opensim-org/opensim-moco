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
    auto metabolicsPtr_ns = new Bhargava2004Metabolics();
    metabolicsPtr_ns->setName("metabolics_ns");
    metabolicsPtr_ns->set_use_smoothing(false);
    metabolicsPtr_ns->set_include_negative_mechanical_work(false);
    metabolicsPtr_ns->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_ns);
    auto& metabolics_ns =
        model.getComponent<Bhargava2004Metabolics>("metabolics_ns");
    // Add non-smooth metabolics with force_dependent_shortening_prop_constant.
    auto metabolicsPtr_fd_ns = new Bhargava2004Metabolics();
    metabolicsPtr_fd_ns->setName("metabolics_fd_ns");
    metabolicsPtr_fd_ns->set_use_smoothing(false);
    metabolicsPtr_fd_ns->
            set_use_force_dependent_shortening_prop_constant(true);
    metabolicsPtr_fd_ns->set_include_negative_mechanical_work(false);
    metabolicsPtr_fd_ns->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_fd_ns);
    auto& metabolics_fd_ns =
        model.getComponent<Bhargava2004Metabolics>("metabolics_fd_ns");
    // Add non-smooth metabolics with negative mechanical work.
    auto metabolicsPtr_nw_ns = new Bhargava2004Metabolics();
    metabolicsPtr_nw_ns->setName("metabolics_nw_ns");
    metabolicsPtr_nw_ns->set_use_smoothing(false);
    metabolicsPtr_nw_ns->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_nw_ns);
    auto& metabolics_nw_ns =
        model.getComponent<Bhargava2004Metabolics>("metabolics_nw_ns");
    // Add smooth metabolics
    auto metabolicsPtr_s = new Bhargava2004Metabolics();
    metabolicsPtr_s->setName("metabolics_s");
    metabolicsPtr_s->set_use_smoothing(true);
    // We set a high value for the velocity smoothing parameter so that
    // the tanh transition is very steep and the smooth model best approximates
    // the non-smooth model. In pratice we use a lower value (default is 10).
    metabolicsPtr_s->set_velocity_smoothing(1e6);
    metabolicsPtr_s->set_include_negative_mechanical_work(false);
    metabolicsPtr_s->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_s);
    auto& metabolics_s =
        model.getComponent<Bhargava2004Metabolics>("metabolics_s");
    // Add smooth metabolics with force_dependent_shortening_prop_constant.
    auto metabolicsPtr_fd_s = new Bhargava2004Metabolics();
    metabolicsPtr_fd_s->setName("metabolics_fd_s");
    metabolicsPtr_fd_s->set_use_smoothing(true);
    // We set a high value for the velocity smoothing parameter so that
    // the tanh transition is very steep and the smooth model best approximates
    // the non-smooth model. In pratice we use a lower value (default is 10).
    metabolicsPtr_fd_s->
            set_use_force_dependent_shortening_prop_constant(true);
    metabolicsPtr_fd_s->set_velocity_smoothing(1e6);
    metabolicsPtr_fd_s->set_include_negative_mechanical_work(false);
    metabolicsPtr_fd_s->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_fd_s);
    auto& metabolics_fd_s =
        model.getComponent<Bhargava2004Metabolics>("metabolics_fd_s");
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

        SECTION("Smooth vs non-smooth metabolics models with concentric "
                "and eccentric contractions") {

            for (double speed = -0.1; speed <= 0.1; speed += 0.01) {

                coord.setSpeedValue(state, speed * Vmax);

                model.realizeVelocity(state);
                muscle.computeInitialFiberEquilibrium(state);
                SimTK::Vector& controls(model.updControls(state));
                muscle.setControls(SimTK::Vector(1, excitation), controls);
                model.setControls(state, controls);

                model.realizeDynamics(state);
                // Metabolics not using
                // force_dependent_shortening_prop_constant.
                CHECK(metabolics_ns.getTotalShorteningRate(state) ==
                        Approx(metabolics_s.getTotalShorteningRate(state)).
                                margin(1e-6));
                CHECK(metabolics_ns.getTotalMechanicalWorkRate(state) ==
                        Approx(metabolics_s.getTotalMechanicalWorkRate(state)).
                                margin(1e-6));
                CHECK(metabolics_ns.getTotalMetabolicRate(state) ==
                        Approx(metabolics_s.getTotalMetabolicRate(state)).
                                margin(1e-6));
                // Metabolics using force_dependent_shortening_prop_constant.
                CHECK(metabolics_fd_ns.getTotalShorteningRate(state) ==
                        Approx(metabolics_fd_s.getTotalShorteningRate(state)).
                                margin(1e-6));
                CHECK(metabolics_fd_ns.getTotalMechanicalWorkRate(state) ==
                        Approx(metabolics_fd_s.
                                getTotalMechanicalWorkRate(state)).
                                        margin(1e-6));
                CHECK(metabolics_fd_ns.getTotalMetabolicRate(state) ==
                        Approx(metabolics_fd_s.getTotalMetabolicRate(state)).
                                margin(1e-6));
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
            CHECK(metabolics_ns.getTotalMechanicalWorkRate(state) ==
                    Approx(0).margin(1e-6));
            CHECK(metabolics_s.getTotalMechanicalWorkRate(state) ==
                    Approx(0).margin(1e-6));
        }

        SECTION("activationHeatRate=0 and maintenanceHeatRate=0 if "
            "muscleExcitation=0") {

            double speed = -0.4;
            coord.setSpeedValue(state, speed);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            double nullExcitation = 0.0;
            muscle.setControls(SimTK::Vector(1, nullExcitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            CHECK(metabolics_ns.getTotalActivationRate(state) ==
                    Approx(0.0));
            CHECK(metabolics_s.getTotalActivationRate(state) ==
                    Approx(0.0));
            CHECK(metabolics_ns.getTotalMaintenanceRate(state) ==
                    Approx(0.0));
            CHECK(metabolics_s.getTotalMaintenanceRate(state) ==
                    Approx(0.0));
        }

        SECTION("mechanicalWorkRate=-fiberForceActive * fiberVelocity with "
            "concentric contractions") {

            double speed = -0.1;
            coord.setSpeedValue(state, speed);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            muscle.setControls(SimTK::Vector(1, excitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            double mechanicalWorkRate =
                - metabolics_s.get_muscle_effort_scaling_factor()
                * muscle.getActiveFiberForce(state)
                * muscle.getFiberVelocity(state);
            CHECK(mechanicalWorkRate ==
                    metabolics_ns.getTotalMechanicalWorkRate(state));
            CHECK(mechanicalWorkRate ==
                    metabolics_s.getTotalMechanicalWorkRate(state));
        }

        SECTION("mechanicalWorkRate=0 with eccentric contractions (negative "
            "mechanical work not allowed)") {

            double speed = 0.1;
            coord.setSpeedValue(state, speed);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            muscle.setControls(SimTK::Vector(1, excitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            double mechanicalWorkRate = 0;
            CHECK(mechanicalWorkRate ==
                    metabolics_ns.getTotalMechanicalWorkRate(state));
            CHECK(mechanicalWorkRate ==
                    metabolics_s.getTotalMechanicalWorkRate(state));
        }

        SECTION("") {

            double speed = -0.1;
            coord.setSpeedValue(state, speed);
            double activation = 0.025;
            muscle.setActivation(state, activation);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            excitation = 0.02;
            muscle.setControls(SimTK::Vector(1, excitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            double activationHeatRate = metabolics_nw_ns.getTotalActivationRate(state);
            double maintenanceHeatRate = metabolics_nw_ns.getTotalMaintenanceRate(state);
            double shorteningHeatRate = metabolics_nw_ns.getTotalShorteningRate(state);
            double mechanicalWorkRate = metabolics_nw_ns.getTotalMechanicalWorkRate(state);
            double Edot_W_beforeClamp = activationHeatRate
                + maintenanceHeatRate + shorteningHeatRate
                + mechanicalWorkRate;
            std::cout << activationHeatRate << std::endl;
            std::cout << maintenanceHeatRate << std::endl;
            std::cout << shorteningHeatRate << std::endl;
            std::cout << mechanicalWorkRate << std::endl;
            std::cout << muscle.getFiberVelocity(state) << std::endl;
            std::cout << muscle.getNormalizedFiberLength(state) << std::endl;
            std::cout << muscle.getActiveFiberForce(state) << std::endl;
            std::cout << muscle.getFiberForce(state) << std::endl;
            std::cout << muscle.getPassiveFiberForce(state) << std::endl;


        }

    }
}
