/* -------------------------------------------------------------------------- *
 * OpenSim Moco: SmoothBhargava2004Metabolics.cpp                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
 * Contributors: Tim Dorn, Thomas Uchida, Christopher Dembia                  *
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

#include "SmoothBhargava2004Metabolics.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
//  SmoothBhargava2004Metabolics_MuscleParameters
//=============================================================================

SmoothBhargava2004Metabolics_MuscleParameters::
SmoothBhargava2004Metabolics_MuscleParameters() {
    constructProperties();
}

void SmoothBhargava2004Metabolics_MuscleParameters::
setMuscleMass()
{
    if (get_use_provided_muscle_mass())
        muscleMass = get_provided_muscle_mass();
    else {
        muscleMass = (
                getMuscle().getMaxIsometricForce() / get_specific_tension())
                * get_density() * getMuscle().getOptimalFiberLength();
        }
}

void SmoothBhargava2004Metabolics_MuscleParameters::
constructProperties()
{
    // Specific tension of mammalian muscle (Pascals (N/m^2)).
    constructProperty_specific_tension(0.25e6);
    // Density of mammalian muscle (kg/m^3).
    constructProperty_density(1059.7);
    constructProperty_ratio_slow_twitch_fibers(0.5);

    constructProperty_use_provided_muscle_mass(false);
    constructProperty_provided_muscle_mass(SimTK::NaN);

    // Defaults from Bhargava et al (2004).
    constructProperty_activation_constant_slow_twitch(40.0);
    constructProperty_activation_constant_fast_twitch(133.0);
    constructProperty_maintenance_constant_slow_twitch(74.0);
    constructProperty_maintenance_constant_fast_twitch(111.0);
}

//=============================================================================
//  SmoothBhargava2004Metabolics
//=============================================================================
SmoothBhargava2004Metabolics::SmoothBhargava2004Metabolics()
{
    constructProperties();
}

SmoothBhargava2004Metabolics::SmoothBhargava2004Metabolics(
        const bool activation_rate_on,
        const bool maintenance_rate_on,
        const bool shortening_rate_on,
        const bool basal_rate_on,
        const bool work_rate_on)
{
    constructProperties();

    set_activation_rate_on(activation_rate_on);
    set_maintenance_rate_on(maintenance_rate_on);
    set_shortening_rate_on(shortening_rate_on);
    set_basal_rate_on(basal_rate_on);
    set_mechanical_work_rate_on(work_rate_on);
}

void SmoothBhargava2004Metabolics::addMuscle(const std::string& name,
        const Muscle& muscle, double ratio_slow_twitch_fibers,
        double specific_tension, double muscle_mass) {
    append_muscle_parameters(SmoothBhargava2004Metabolics_MuscleParameters());
    auto& mp = upd_muscle_parameters(
            getProperty_muscle_parameters().size() - 1);
    mp.setName(name);
    mp.set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    mp.set_specific_tension(specific_tension);
    if (SimTK::isNaN(muscle_mass)) {
        mp.set_use_provided_muscle_mass(false);
    }
    else {
        mp.set_use_provided_muscle_mass(true);
        mp.set_provided_muscle_mass(muscle_mass);
    }
    mp.connectSocket_muscle(muscle);
}

void SmoothBhargava2004Metabolics::addMuscle(const std::string& name,
        const Muscle& muscle, double ratio_slow_twitch_fibers,
        double specific_tension, double activation_constant_slow_twitch,
        double activation_constant_fast_twitch,
        double maintenance_constant_slow_twitch,
        double maintenance_constant_fast_twitch,
        double muscle_mass) {
    append_muscle_parameters(SmoothBhargava2004Metabolics_MuscleParameters());
    auto& mp = upd_muscle_parameters(
            getProperty_muscle_parameters().size() - 1);
    mp.setName(name);
    mp.set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    mp.set_specific_tension(specific_tension);
    mp.set_activation_constant_slow_twitch(activation_constant_slow_twitch);
    mp.set_activation_constant_fast_twitch(activation_constant_fast_twitch);
    mp.set_maintenance_constant_slow_twitch(maintenance_constant_slow_twitch);
    mp.set_maintenance_constant_fast_twitch(maintenance_constant_fast_twitch);

    if (SimTK::isNaN(muscle_mass)) {
        mp.set_use_provided_muscle_mass(false);
    }
    else {
        mp.set_use_provided_muscle_mass(true);
        mp.set_provided_muscle_mass(muscle_mass);
    }
    mp.connectSocket_muscle(muscle);
}

void SmoothBhargava2004Metabolics::constructProperties()
{
    constructProperty_muscle_parameters();

    constructProperty_activation_rate_on(true);
    constructProperty_maintenance_rate_on(true);
    constructProperty_shortening_rate_on(true);
    constructProperty_basal_rate_on(true);
    constructProperty_mechanical_work_rate_on(true);
    constructProperty_enforce_minimum_heat_rate_per_muscle(true);

    const int curvePoints = 5;
    const double curveX[] = {0.0, 0.5, 1.0, 1.5, 2.0};
    const double curveY[] = {0.5, 0.5, 1.0, 0.0, 0.0};
    PiecewiseLinearFunction fiberLengthDepCurveDefault(curvePoints, curveX,
            curveY, "defaultCurve");
    constructProperty_normalized_fiber_length_dependence_on_maintenance_rate(
            fiberLengthDepCurveDefault);

    constructProperty_use_fiber_length_dependence_on_maintenance_rate(true);
    constructProperty_use_force_dependent_shortening_prop_constant(false);
    constructProperty_basal_coefficient(1.2);
    constructProperty_basal_exponent(1.0);
    constructProperty_muscle_effort_scaling_factor(1.0);
    constructProperty_include_negative_mechanical_work(true);
    constructProperty_forbid_negative_total_power(true);

    constructProperty_use_smoothing(false);
    constructProperty_velocity_smoothing(10);
    constructProperty_power_smoothing(10);
    constructProperty_heat_rate_smoothing(10);
}

double SmoothBhargava2004Metabolics::getTotalMetabolicRate(
        const SimTK::State& s) const {

    // BASAL METABOLIC RATE (W) (based on whole body mass, not muscle mass).
    // ---------------------------------------------------------------------
    double Bdot = 0;
    if (get_basal_rate_on()) {

        Bdot = get_basal_coefficient()
            * pow(getModel().getMatterSubsystem().calcSystemMass(s),
                    get_basal_exponent());

        if (SimTK::isNaN(Bdot))
            std::cout << "WARNING::" << getName()
                    << ": Bdot = NaN!" << std::endl;
    }

    return getMetabolicRate(s).sum() + Bdot;
}

double SmoothBhargava2004Metabolics::getMuscleMetabolicRate(
        const SimTK::State& s, const std::string& channel) const {
    return getMetabolicRate(s).get(m_muscleIndices.at(channel));
}

void SmoothBhargava2004Metabolics::extendRealizeTopology(SimTK::State& state)
const {
    Super::extendRealizeTopology(state);
    m_muscleParameters.clear();
    m_muscleIndices.clear();
    int nM = getProperty_muscle_parameters().size();
    for (int i=0; i < nM; ++i) {
        const auto& muscle_parameters = get_muscle_parameters(i);
        const auto& muscle = muscle_parameters.getMuscle();
        if (muscle.get_appliesForce()) {
            m_muscleParameters.emplace_back(&muscle_parameters);
            m_muscleIndices[muscle.getAbsolutePathString()] = i;
            ++i;
        }
    }
}

void SmoothBhargava2004Metabolics::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    addCacheVariable<SimTK::Vector>("metabolic_rate",
            SimTK::Vector((int)m_muscleParameters.size(), 0.0),
                    SimTK::Stage::Velocity);
}

const SimTK::Vector& SmoothBhargava2004Metabolics::getMetabolicRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "metabolic_rate")) {
        calcMetabolicRate(
                s, updCacheVariableValue<SimTK::Vector>(s, "metabolic_rate"));
        markCacheVariableValid(s, "metabolic_rate");
    }
    return getCacheVariableValue<SimTK::Vector>(s, "metabolic_rate");
}

void SmoothBhargava2004Metabolics::calcMetabolicRate(
        const SimTK::State& s, SimTK::Vector& ratesForMuscles) const {
    ratesForMuscles.resize((int)m_muscleParameters.size());
    double Adot, Mdot, Sdot, Wdot;
    Adot = Mdot = Sdot = Wdot = 0;

    int i = 0;
    for (const auto& muscleParameter : m_muscleParameters) {
        const auto& muscle = muscleParameter->getMuscle();

        const double max_isometric_force = muscle.getMaxIsometricForce();
        const double activation = get_muscle_effort_scaling_factor()
            * muscle.getActivation(s);
        const double excitation = get_muscle_effort_scaling_factor()
            * muscle.getControl(s);
        const double fiber_force_passive =  muscle.getPassiveFiberForce(s);
        const double fiber_force_active = get_muscle_effort_scaling_factor()
            * muscle.getActiveFiberForce(s);
        const double fiber_force_total = fiber_force_active
            + fiber_force_passive;
        const double fiber_length_normalized =
            muscle.getNormalizedFiberLength(s);
        const double fiber_velocity = muscle.getFiberVelocity(s);
        const double slow_twitch_excitation =
            muscleParameter->get_ratio_slow_twitch_fibers()
            * sin(SimTK::Pi/2 * excitation);
        const double fast_twitch_excitation =
            (1 - muscleParameter->get_ratio_slow_twitch_fibers())
            * (1 - cos(SimTK::Pi/2 * excitation));
        double alpha, fiber_length_dependence;

        // Get the unnormalized total active force, F_iso that 'would' be
        // developed at the current activation and fiber length under isometric
        // conditions (i.e. Vm=0).
        const double F_iso = activation
            * muscle.getActiveForceLengthMultiplier(s) * max_isometric_force;

        // ACTIVATION HEAT RATE (W).
        // -------------------------
        if (get_forbid_negative_total_power() || get_activation_rate_on())
        {
            // This value is set to 1.0, as used by Anderson & Pandy (1999),
            // however, in Bhargava et al., (2004) they assume a function here.
            // We will ignore this function and use 1.0 for now.
            const double decay_function_value = 1.0;
            Adot = muscleParameter->getMuscleMass() * decay_function_value
                * ( (muscleParameter->get_activation_constant_slow_twitch()
                            * slow_twitch_excitation)
                    + (muscleParameter->get_activation_constant_fast_twitch()
                            * fast_twitch_excitation) );
        }

        // MAINTENANCE HEAT RATE (W).
        // --------------------------
        if (get_forbid_negative_total_power() || get_maintenance_rate_on())
        {
            // TODO: discuss this addition, based on my stuff but not validated
            if (get_use_fiber_length_dependence_on_maintenance_rate()) {
                SimTK::Vector tmp(1, fiber_length_normalized);
                fiber_length_dependence =
                get_normalized_fiber_length_dependence_on_maintenance_rate().
                    calcValue(tmp);
            }
            else {
                fiber_length_dependence = fiber_length_normalized;
            }
            Mdot = muscleParameter->getMuscleMass() * fiber_length_dependence
                * ( (muscleParameter->get_maintenance_constant_slow_twitch()
                            * slow_twitch_excitation)
                    + (muscleParameter->get_maintenance_constant_fast_twitch()
                            * fast_twitch_excitation) );
        }

        // SHORTENING HEAT RATE (W).
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening.
        // --------------------------------------------------------------------
        if (get_forbid_negative_total_power() || get_shortening_rate_on())
        {
            if (get_use_force_dependent_shortening_prop_constant())
            {
                if (get_use_smoothing()) {
                    // Smooth approximation between concentric and eccentric
                    // contractions.
                    const double bv = get_velocity_smoothing();
                    // fiber_velocity_ecc=1 if eccentric contraction.
                    const double fiber_velocity_ecc =
                        0.5 + 0.5 * tanh(bv * fiber_velocity);
                    alpha = (0.16 * F_iso) + (0.18 * fiber_force_total);
                    alpha = alpha + (-alpha + 0.157 * fiber_force_total)
                            * fiber_velocity_ecc;
                }
                else {
                    if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                        alpha = (0.16 * F_iso) + (0.18 * fiber_force_total);
                    else                        // eccentric contraction, Vm>0
                        alpha = 0.157 * fiber_force_total;
                }
            }
            else
            {
                if (get_use_smoothing()) {
                    // Smooth approximation between concentric and eccentric
                    // contractions.
                    const double bv = get_velocity_smoothing();
                    // fiber_velocity_ecc=1 if eccentric contraction.
                    const double fiber_velocity_ecc =
                        0.5 + 0.5 * tanh(bv * fiber_velocity);
                    alpha = 0.25 * fiber_force_total;
                    alpha = alpha + -alpha * fiber_velocity_ecc;
                }
                else {
                    if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                        alpha = 0.25 * fiber_force_total;
                    else                        // eccentric contraction, Vm>0
                        alpha = 0.0;
                }
            }
            Sdot = -alpha * fiber_velocity;
        }

        // MECHANICAL WORK RATE for the contractile element of the muscle (W).
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening.
        // -------------------------------------------------------------------
        if (get_forbid_negative_total_power() || get_mechanical_work_rate_on())
        {
            if (get_use_smoothing()) {
                // Smooth approximation between concentric and eccentric
                // contractions. TODO for some reasons, cannot have bv and
                // fiber_velocity_conc in else
                const double bv = get_velocity_smoothing();
                const double fiber_velocity_conc = 1 - (
                        0.5 + 0.5 * tanh(bv * fiber_velocity));
                if (get_include_negative_mechanical_work())
                    Wdot = -fiber_force_active*fiber_velocity;
                else
                    // fiber_velocity_conc=1 if concentric contraction.
                    Wdot = -fiber_force_active*fiber_velocity
                            *fiber_velocity_conc;
            }
            else {
                if (get_include_negative_mechanical_work() ||
                            fiber_velocity <= 0)
                    Wdot = -fiber_force_active*fiber_velocity;
                else
                    Wdot = 0;
            }
        }

        // NAN CHECKING
        // ------------------------------------------
        if (SimTK::isNaN(Adot))
            std::cout << "WARNING::" << getName() << ": Adot (" <<
                    muscleParameter->getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(Mdot))
            std::cout << "WARNING::" << getName() << ": Mdot (" <<
                    muscleParameter->getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(Sdot))
            std::cout << "WARNING::" << getName() << ": Sdot (" <<
                    muscleParameter->getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(Wdot))
            std::cout << "WARNING::" << getName() << ": Wdot (" <<
                    muscleParameter->getName() << ") = NaN!" << std::endl;

        // If necessary, increase the shortening heat rate so that the total
        // power is non-negative.
        if (get_forbid_negative_total_power()) {
            const double Edot_W_beforeClamp = Adot + Mdot + Sdot + Wdot;
            if (get_use_smoothing()) {
                // Variables for smooth approximations between negative and
                // positive total power.
                const double bp = get_power_smoothing();
                // Edot_W_beforeClamp_neg=1 if Edot_W_beforeClamp is negative.
                const double Edot_W_beforeClamp_neg = 0.5 + (
                        0.5 * tanh(bp * -Edot_W_beforeClamp));
                Sdot -= Edot_W_beforeClamp * Edot_W_beforeClamp_neg;
            }
            else {
                if (Edot_W_beforeClamp < 0)
                    Sdot -= Edot_W_beforeClamp;
            }
        }

        // This check is adapted from Umberger(2003), page 104: the total heat
        // rate (i.e., Adot + Mdot + Sdot) for a given muscle cannot fall below
        // 1.0 W/kg.
        // --------------------------------------------------------------------
        double totalHeatRate = Adot + Mdot + Sdot;
        if (get_use_smoothing()) {
            if(get_enforce_minimum_heat_rate_per_muscle()
                    && get_activation_rate_on()
                    && get_maintenance_rate_on()
                    && get_shortening_rate_on())
            {
                // Variables for smooth approximations between total heat rate
                // for a given muscle below or above 1.0 W/kg.
                const double bhr = get_heat_rate_smoothing();
                // totalHeatRate_bmm= 1 if totalHeatRate is below the muscle
                // mass.
                const double totalHeatRate_bmm = 0.5 + 0.5 * tanh(bhr * (1.0 *
                        muscleParameter->getMuscleMass() - totalHeatRate));
                totalHeatRate = totalHeatRate + (-totalHeatRate + 1.0 *
                        muscleParameter->getMuscleMass()) * totalHeatRate_bmm;
            }
        }
        else {
            if(get_enforce_minimum_heat_rate_per_muscle()
                    && totalHeatRate < 1.0 * muscleParameter->getMuscleMass()
                    && get_activation_rate_on()
                    && get_maintenance_rate_on()
                    && get_shortening_rate_on())
            {
                totalHeatRate = 1.0 * muscleParameter->getMuscleMass();
            }
        }

        // TOTAL METABOLIC ENERGY RATE (W).
        // --------------------------------
        double Edot = 0;

        if (get_activation_rate_on() && get_maintenance_rate_on()
                && get_shortening_rate_on())
        {
            Edot += totalHeatRate; // May have been clamped to 1.0 W/kg.
        }
        else {
            if (get_activation_rate_on())
                Edot += Adot;
            if (get_maintenance_rate_on())
                Edot += Mdot;
            if (get_shortening_rate_on())
                Edot += Sdot;
        }
        if (get_mechanical_work_rate_on())
            Edot += Wdot;

        ratesForMuscles[i] = Edot;
        ++i;
    }
}

int SmoothBhargava2004Metabolics::getNumMetabolicMuscles() const
{
    return getProperty_muscle_parameters().size();
}
