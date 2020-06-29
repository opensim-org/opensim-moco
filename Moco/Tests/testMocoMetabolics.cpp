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
    musclePtr->set_fiber_damping(0.01);
    musclePtr->setName("muscle");
    musclePtr->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    musclePtr->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(musclePtr);
    auto& muscle = model.getComponent<DeGrooteFregly2016Muscle>("muscle");

    // Add non-smooth metabolics
    auto metabolicsPtr_nonSmooth = new Bhargava2004Metabolics();
    metabolicsPtr_nonSmooth->setName("metabolics_nonSmooth");
    metabolicsPtr_nonSmooth->set_use_smoothing(false);
    metabolicsPtr_nonSmooth->set_include_negative_mechanical_work(false);
    metabolicsPtr_nonSmooth->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_nonSmooth);
    auto& metabolics_nonSmooth =
            model.getComponent<Bhargava2004Metabolics>("metabolics_nonSmooth");
    //// Add non-smooth metabolics with force_dependent_shortening_prop_constant.
    //auto metabolicsPtr_forceDep_nonSmooth = new Bhargava2004Metabolics();
    //metabolicsPtr_forceDep_nonSmooth->setName("metabolics_forceDep_nonSmooth");
    //metabolicsPtr_forceDep_nonSmooth->set_use_smoothing(false);
    //metabolicsPtr_forceDep_nonSmooth->
    //        set_use_force_dependent_shortening_prop_constant(true);
    //metabolicsPtr_forceDep_nonSmooth->
    //        set_include_negative_mechanical_work(false);
    //metabolicsPtr_forceDep_nonSmooth->addMuscle("muscle",  muscle);
    //model.addComponent(metabolicsPtr_forceDep_nonSmooth);
    //auto& metabolics_forceDep_nonSmooth =
    //        model.getComponent<Bhargava2004Metabolics>(
    //                "metabolics_forceDep_nonSmooth");
    // Add non-smooth metabolics with negative mechanical work.
    auto metabolicsPtr_negativeWork_nonSmooth = new Bhargava2004Metabolics();
    metabolicsPtr_negativeWork_nonSmooth->setName(
            "metabolics_negativeWork_nonSmooth");
    metabolicsPtr_negativeWork_nonSmooth->set_use_smoothing(false);
    metabolicsPtr_negativeWork_nonSmooth->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_negativeWork_nonSmooth);
    auto& metabolics_negativeWork_nonSmooth =
            model.getComponent<Bhargava2004Metabolics>(
                    "metabolics_negativeWork_nonSmooth");

    // Add smooth metabolics
    auto metabolicsPtr_smooth = new Bhargava2004Metabolics();
    metabolicsPtr_smooth->setName("metabolics_smooth");
    metabolicsPtr_smooth->set_use_huber_loss(true);
    // We set a high value for the velocity and heat rate smoothing parameters
    // so that the tanh transitions are very steep and the smooth models best
    // approximate the non-smooth models. In pratice we use lower values
    // (default is 10).
    metabolicsPtr_smooth->set_velocity_smoothing(1e6);
    metabolicsPtr_smooth->set_heat_rate_smoothing(1e6);
    metabolicsPtr_smooth->set_include_negative_mechanical_work(false);
    metabolicsPtr_smooth->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_smooth);
    auto& metabolics_smooth =
            model.getComponent<Bhargava2004Metabolics>("metabolics_smooth");
    //// Add smooth metabolics with force_dependent_shortening_prop_constant.
    //auto metabolicsPtr_forceDep_smooth = new Bhargava2004Metabolics();
    //metabolicsPtr_forceDep_smooth->setName("metabolics_forceDep_smooth");
    //metabolicsPtr_forceDep_smooth->set_use_huber_loss(true);
    //// We set a high value for the velocity smoothing parameter so that
    //// the tanh transition is very steep and the smooth model best approximates
    //// the non-smooth model. In pratice we use a lower value (default is 10).
    //metabolicsPtr_forceDep_smooth->
    //        set_use_force_dependent_shortening_prop_constant(true);
    //metabolicsPtr_forceDep_smooth->set_velocity_smoothing(1e6);
    //metabolicsPtr_forceDep_smooth->set_include_negative_mechanical_work(false);
    //metabolicsPtr_forceDep_smooth->addMuscle("muscle",  muscle);
    //model.addComponent(metabolicsPtr_forceDep_smooth);
    //auto& metabolics_forceDep_smooth =
    //        model.getComponent<Bhargava2004Metabolics>(
    //                "metabolics_forceDep_smooth");
    // Add smooth metabolics with negative mechanical work.
    auto metabolicsPtr_negativeWork_smooth = new Bhargava2004Metabolics();
    metabolicsPtr_negativeWork_smooth->setName(
            "metabolics_negativeWork_smooth");
    metabolicsPtr_negativeWork_smooth->set_use_huber_loss(true);
    metabolicsPtr_negativeWork_smooth->set_velocity_smoothing(1e6);
    metabolicsPtr_negativeWork_smooth->set_heat_rate_smoothing(1e6);
    metabolicsPtr_negativeWork_smooth->set_power_smoothing(1e6);
    metabolicsPtr_negativeWork_smooth->addMuscle("muscle",  muscle);
    model.addComponent(metabolicsPtr_negativeWork_smooth);
    auto& metabolics_negativeWork_smooth =
            model.getComponent<Bhargava2004Metabolics>(
                    "metabolics_negativeWork_smooth");
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

        SECTION("Smooth = non-smooth metabolics models with concentric "
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
                CHECK(metabolics_nonSmooth.getTotalShorteningRate(state) ==
                        Approx(metabolics_smooth.
                                getTotalShorteningRate(state)).margin(1e-4));
                CHECK(metabolics_nonSmooth.getTotalMechanicalWorkRate(state) ==
                        Approx(metabolics_smooth.
                                getTotalMechanicalWorkRate(state)).
                                        margin(1e-4));
                CHECK(metabolics_nonSmooth.getTotalMetabolicRate(state) ==
                        Approx(metabolics_smooth.getTotalMetabolicRate(state)).
                                margin(1e-4));
                //// Metabolics using force_dependent_shortening_prop_constant.
                //CHECK(metabolics_forceDep_nonSmooth.
                //        getTotalShorteningRate(state) == Approx(
                //                metabolics_forceDep_smooth.
                //                        getTotalShorteningRate(state)).
                //                                margin(1e-4));
                //CHECK(metabolics_forceDep_nonSmooth.
                //        getTotalMechanicalWorkRate(state) == Approx(
                //                metabolics_forceDep_smooth.
                //                        getTotalMechanicalWorkRate(state)).
                //                                margin(1e-4));
                //CHECK(metabolics_forceDep_nonSmooth.
                //        getTotalMetabolicRate(state) == Approx(
                //                metabolics_forceDep_smooth.
                //                        getTotalMetabolicRate(state)).
                //                                margin(1e-4));
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
            // Non-smooth metabolics model.
            CHECK(metabolics_nonSmooth.getTotalMechanicalWorkRate(state) ==
                    Approx(0).margin(1e-4));
            // Smooth metabolics model.
            CHECK(metabolics_smooth.getTotalMechanicalWorkRate(state) ==
                    Approx(0).margin(1e-4));
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
            // Non-smooth metabolics model.
            CHECK(metabolics_nonSmooth.getTotalActivationRate(state) ==
                    Approx(0.0));
            CHECK(metabolics_nonSmooth.getTotalMaintenanceRate(state) ==
                    Approx(0.0));
            // Smooth metabolics model.
            CHECK(metabolics_smooth.getTotalActivationRate(state) ==
                    Approx(0.0));
            CHECK(metabolics_smooth.getTotalMaintenanceRate(state) ==
                    Approx(0.0));
        }

        SECTION("mechanicalWorkRate=-fiberForceActive * fiberVelocity with "
            "concentric and eccentric contractions when negative mechanical "
            "work rate is allowed") {

            // Concentric contraction
            double speed = -0.1;
            coord.setSpeedValue(state, speed);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            muscle.setControls(SimTK::Vector(1, excitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            // Non-smooth metabolics model.
            double mechanicalWorkRate_nonSmooth =
                    - metabolics_negativeWork_nonSmooth.
                            get_muscle_effort_scaling_factor() *
                    muscle.getActiveFiberForce(state) *
                    muscle.getFiberVelocity(state);
            CHECK(mechanicalWorkRate_nonSmooth ==
                    metabolics_negativeWork_nonSmooth.
                            getTotalMechanicalWorkRate(state));
            // Smooth metabolics model.
            double mechanicalWorkRate_smooth =
                    - metabolics_negativeWork_smooth.
                            get_muscle_effort_scaling_factor() *
                    muscle.getActiveFiberForce(state) *
                    muscle.getFiberVelocity(state);
            CHECK(mechanicalWorkRate_smooth ==
                    metabolics_negativeWork_smooth.
                            getTotalMechanicalWorkRate(state));

            // Eccentric contraction
            speed = 0.1;
            coord.setSpeedValue(state, speed);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            muscle.setControls(SimTK::Vector(1, excitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            // Non-smooth metabolics model.
            mechanicalWorkRate_nonSmooth =
                    - metabolics_negativeWork_nonSmooth.
                            get_muscle_effort_scaling_factor() *
                    muscle.getActiveFiberForce(state) *
                    muscle.getFiberVelocity(state);
            CHECK(mechanicalWorkRate_nonSmooth ==
                    metabolics_negativeWork_nonSmooth.
                            getTotalMechanicalWorkRate(state));
            // Smooth metabolics model.
            mechanicalWorkRate_smooth =
                    - metabolics_negativeWork_smooth.
                            get_muscle_effort_scaling_factor() *
                    muscle.getActiveFiberForce(state) *
                    muscle.getFiberVelocity(state);
            CHECK(mechanicalWorkRate_smooth ==
                    metabolics_negativeWork_smooth.
                            getTotalMechanicalWorkRate(state));
        }

        SECTION("mechanicalWorkRate=0 with eccentric contractions when "
            "negative mechanical work rate is not allowed)") {

            double speed = 0.1;
            coord.setSpeedValue(state, speed);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            muscle.setControls(SimTK::Vector(1, excitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            double mechanicalWorkRate = 0.0;
            // Non-smooth metabolics model.
            CHECK(mechanicalWorkRate ==
                    metabolics_nonSmooth.getTotalMechanicalWorkRate(state));
            // Smooth metabolics model.
            CHECK(mechanicalWorkRate ==
                    metabolics_smooth.getTotalMechanicalWorkRate(state));
        }

        SECTION("Total power is clamped so that it is always non-negative") {

            // Eccentric contraction to have negative mechanical work rate.
            double speed = 0.02;
            coord.setSpeedValue(state, speed);
            // Low activation to have |mechanical work| > |heat rates| and
            // total rate < 0.
            double activation = 0.001;
            muscle.setActivation(state, activation);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            excitation = 0.02;
            muscle.setControls(SimTK::Vector(1, excitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            // Non-smooth metabolics model.
            double activationHeatRate_nonSmooth =
                    metabolics_negativeWork_nonSmooth.
                            getTotalActivationRate(state);
            double maintenanceHeatRate_nonSmooth =
                    metabolics_negativeWork_nonSmooth.
                            getTotalMaintenanceRate(state);
            double shorteningHeatRate_nonSmooth =
                    metabolics_negativeWork_nonSmooth.
                            getTotalShorteningRate(state);
            double mechanicalWorkRate_nonSmooth =
                    metabolics_negativeWork_nonSmooth.
                            getTotalMechanicalWorkRate(state);
            double Edot_clamped_nonSmooth =
                    activationHeatRate_nonSmooth +
                    maintenanceHeatRate_nonSmooth +
                    shorteningHeatRate_nonSmooth +
                    mechanicalWorkRate_nonSmooth;
            // Smooth metabolics model.
            double activationHeatRate_smooth =
                    metabolics_negativeWork_smooth.
                            getTotalActivationRate(state);
            double maintenanceHeatRate_smooth =
                    metabolics_negativeWork_smooth.
                            getTotalMaintenanceRate(state);
            double shorteningHeatRate_smooth =
                    metabolics_negativeWork_smooth.
                            getTotalShorteningRate(state);
            double mechanicalWorkRate_smooth =
                    metabolics_negativeWork_smooth.
                            getTotalMechanicalWorkRate(state);
            double Edot_clamped_smooth =
                    activationHeatRate_smooth + maintenanceHeatRate_smooth +
                    shorteningHeatRate_smooth + mechanicalWorkRate_smooth;
            double Edot_clamped = 0.0;
            CHECK(Edot_clamped == Edot_clamped_smooth);
            CHECK(Edot_clamped == Edot_clamped_nonSmooth);
        }

        SECTION("The total heat rate (i.e., activationHeatRate + "
            " maintenanceHeatRate + shorteningHeatRate) for a given muscle "
            " cannot fall below 1.0 W/kg") {

            // Eccentric contraction to have null mechanical work rate
            // (negative mechanical work rate is not allowed).
            double speed = 0.02;
            coord.setSpeedValue(state, speed);
            // Low activation to have low heat rates.
            double activation = 0.001;
            muscle.setActivation(state, activation);

            model.realizeVelocity(state);
            muscle.computeInitialFiberEquilibrium(state);
            SimTK::Vector& controls(model.updControls(state));
            excitation = 0.02;
            muscle.setControls(SimTK::Vector(1, excitation), controls);
            model.setControls(state, controls);

            model.realizeDynamics(state);
            // Non-smooth metabolics model.
            double activationHeatRate_nonSmooth =
                    metabolics_nonSmooth.getTotalActivationRate(state);
            double maintenanceHeatRate_nonSmooth =
                    metabolics_nonSmooth.getTotalMaintenanceRate(state);
            double shorteningHeatRate_nonSmooth =
                    metabolics_nonSmooth.getTotalShorteningRate(state);
            double totalHeatRate_nonSmooth = activationHeatRate_nonSmooth +
                    maintenanceHeatRate_nonSmooth +
                    shorteningHeatRate_nonSmooth;
            // Check that the total heat rate before clamping (i.e., activation
            // heat rate + maintenance heat rate + shortening heat rate) is
            // below 1.0 W/kg so that it will be clamped to 1.0 W/kg when
            // enforce_minimum_heat_rate_per_muscle() is true (default).
            CHECK(totalHeatRate_nonSmooth <
                    metabolics_nonSmooth.get_muscle_parameters(0).
                            getMuscleMass());
            double mechanicalWorkRate_nonSmooth =
                    metabolics_nonSmooth.getTotalMechanicalWorkRate(state);
            double totalMetabolicRate_nonSmooth =
                    metabolics_nonSmooth.getTotalMetabolicRate(state);
            double basalRate_nonSmooth =
                    metabolics_nonSmooth.get_basal_coefficient() *
                    pow(model.getMatterSubsystem().calcSystemMass(state),
                            metabolics_nonSmooth.get_basal_exponent());
            // Check that the total heat rate after clamping (i.e., total
            // metabolic rate - basal rate - mechanical work rate) is
            // equal to the muscle mass (i.e., clamped to 1.0 W/kg).
            CHECK(totalMetabolicRate_nonSmooth - basalRate_nonSmooth
                    - mechanicalWorkRate_nonSmooth - metabolics_nonSmooth.
                            get_muscle_parameters(0).getMuscleMass() ==
                                    Approx(0.0).margin(1e-4));
            // Smooth metabolics model.
            double activationHeatRate_smooth =
                    metabolics_smooth.getTotalActivationRate(state);
            double maintenanceHeatRate_smooth =
                    metabolics_smooth.getTotalMaintenanceRate(state);
            double shorteningHeatRate_smooth =
                    metabolics_smooth.getTotalShorteningRate(state);
            double totalHeatRate_smooth = activationHeatRate_smooth +
                    maintenanceHeatRate_smooth + shorteningHeatRate_smooth;
            // Check that the total heat rate before clamping (i.e., activation
            // heat rate + maintenance heat rate + shortening heat rate) is
            // below 1.0 W/kg so that it will be clamped to 1.0 W/kg when
            // enforce_minimum_heat_rate_per_muscle() is true (default).
            CHECK(totalHeatRate_smooth <
                    metabolics_smooth.get_muscle_parameters(0).
                            getMuscleMass());
            double mechanicalWorkRate_smooth =
                    metabolics_smooth.getTotalMechanicalWorkRate(state);
            double totalMetabolicRate_smooth =
                    metabolics_smooth.getTotalMetabolicRate(state);
            double basalRate_smooth =
                    metabolics_smooth.get_basal_coefficient() *
                    pow(model.getMatterSubsystem().calcSystemMass(state),
                            metabolics_smooth.get_basal_exponent());
            // Check that the total heat rate after clamping (i.e., total
            // metabolic rate - basal rate - mechanical work rate) is
            // equal to the muscle mass (i.e., clamped to 1.0 W/kg).
            CHECK(totalMetabolicRate_smooth - basalRate_smooth
                    - mechanicalWorkRate_smooth - metabolics_smooth.
                            get_muscle_parameters(0).getMuscleMass() ==
                                    Approx(0.0).margin(1e-4));
        }

    }
}
