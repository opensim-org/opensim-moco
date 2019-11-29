/* -------------------------------------------------------------------------- *
 * OpenSim Moco: SmoothBhargava2004Metabolics.cpp                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "SmoothBhargava2004Metabolics.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
//  SmoothBhargava2004Metabolics_MetabolicMuscleParameters
//=============================================================================

SmoothBhargava2004Metabolics_MetabolicMuscleParameters::
SmoothBhargava2004Metabolics_MetabolicMuscleParameters() {
    constructProperties();
}

SmoothBhargava2004Metabolics_MetabolicMuscleParameters::
SmoothBhargava2004Metabolics_MetabolicMuscleParameters(
        const std::string& muscleName, double ratio_slow_twitch_fibers,
        double muscle_mass) {
    constructProperties();
    setName(muscleName);
    set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);

    if (SimTK::isNaN(muscle_mass)) {
        set_use_provided_muscle_mass(false);
    }
    else {
        set_use_provided_muscle_mass(true);
        set_provided_muscle_mass(muscle_mass);
    }
}

SmoothBhargava2004Metabolics_MetabolicMuscleParameters::
SmoothBhargava2004Metabolics_MetabolicMuscleParameters(
        const std::string& muscleName,
        double ratio_slow_twitch_fibers,
        double activation_constant_slow_twitch,
        double activation_constant_fast_twitch,
        double maintenance_constant_slow_twitch,
        double maintenance_constant_fast_twitch,
        double muscle_mass) {
    constructProperties();
    setName(muscleName);
    set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    set_activation_constant_slow_twitch(activation_constant_slow_twitch);
    set_activation_constant_fast_twitch(activation_constant_fast_twitch);
    set_maintenance_constant_slow_twitch(maintenance_constant_slow_twitch);
    set_maintenance_constant_fast_twitch(maintenance_constant_fast_twitch);

    if (SimTK::isNaN(muscle_mass)) {
        set_use_provided_muscle_mass(false);
    }
    else {
        set_use_provided_muscle_mass(true);
        set_provided_muscle_mass(muscle_mass);
    }
}

void SmoothBhargava2004Metabolics_MetabolicMuscleParameters::
setMuscleMass()
{
    if (get_use_provided_muscle_mass())
        m_muscleMass = get_provided_muscle_mass();
    else {
        m_muscleMass = (
                m_muscle->getMaxIsometricForce() / get_specific_tension())
                * get_density() * m_muscle->getOptimalFiberLength();
        }
}

void SmoothBhargava2004Metabolics_MetabolicMuscleParameters::
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

void SmoothBhargava2004Metabolics::constructProperties()
{
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

    constructProperty_use_force_dependent_shortening_prop_constant(false);
    constructProperty_basal_coefficient(1.2);
    constructProperty_basal_exponent(1.0);
    constructProperty_muscle_effort_scaling_factor(1.0);
    constructProperty_include_negative_mechanical_work(true);
    constructProperty_forbid_negative_total_power(true);
    constructProperty_SmoothBhargava2004Metabolics_MetabolicMuscleParameterSet
       (SmoothBhargava2004Metabolics_MetabolicMuscleParameterSet());
}

double SmoothBhargava2004Metabolics::getTotalMetabolicRate(
        const SimTK::State& s) const {

    // BASAL METABOLIC RATE (W) (based on whole body mass, not muscle mass)
    // ------------------------------------------------------------------
    double Bdot = 0;
    if (get_basal_rate_on()) {
        Bdot = get_basal_coefficient()
            * pow(_model->getMatterSubsystem().calcSystemMass(s),
                    get_basal_exponent());
        if (SimTK::isNaN(Bdot))
            std::cout << "WARNING::" << getName();
            std::cout << ": Bdot = NaN!" << std::endl;
    }

    return getMetabolicRate(s).sum() + Bdot;
}

double SmoothBhargava2004Metabolics::getMuscleMetabolicRate(
        const SimTK::State& s, const std::string& channel) const {
    return getMetabolicRate(s).get(m_muscleIndices.at(channel));
}

void SmoothBhargava2004Metabolics::extendConnectToModel(Model& model) {
    // TODO: Should this be in extendFinalizeFromProperties()?
    m_muscles.clear();
    m_muscleIndices.clear();
    int i = 0;
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (muscle.get_appliesForce()) {
            m_muscleIndices[muscle.getAbsolutePathString()] = i;
            ++i;
        }
    }
}

void SmoothBhargava2004Metabolics::extendAddToSystem(
        SimTK::MultibodySystem&) const {
    addCacheVariable<SimTK::Vector>("metabolic_rate",
            SimTK::Vector((int)m_muscles.size(), 0.0), SimTK::Stage::Velocity);
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
    ratesForMuscles.resize((int)m_muscles.size());

    double Adot, Mdot, Sdot, Wdot;
    Adot = Mdot = Sdot = Wdot = 0;

    int i = 0;
    for (const auto& entry : m_muscles) {

        SmoothBhargava2004Metabolics_MetabolicMuscleParameters& mm =
            get_SmoothBhargava2004Metabolics_MetabolicMuscleParameterSet()[i];

        const double max_isometric_force = entry->getMaxIsometricForce();
        const double activation = get_muscle_effort_scaling_factor()
                                  * entry->getActivation(s);
        const double excitation = get_muscle_effort_scaling_factor()
                                  * entry->getControl(s);
        const double fiber_force_passive = entry->getPassiveFiberForce(s);
        const double fiber_force_active = get_muscle_effort_scaling_factor()
                                          * entry->getActiveFiberForce(s);
        const double fiber_force_total = fiber_force_active
                                         + fiber_force_passive;
        const double fiber_length_normalized =
            entry->getNormalizedFiberLength(s);
        const double fiber_velocity = entry->getFiberVelocity(s);

        const double slow_twitch_excitation = mm.get_ratio_slow_twitch_fibers()
                                              * sin(SimTK::Pi/2 * excitation);
        const double fast_twitch_excitation =
            (1 - mm.get_ratio_slow_twitch_fibers())
            * (1 - cos(SimTK::Pi/2 * excitation));
        double alpha, fiber_length_dependence;

        // Get the unnormalized total active force, F_iso that 'would' be
        // developed at the current activation and fiber length under isometric
        // conditions (i.e. Vm=0)
        const double F_iso = activation
                             * entry->getActiveForceLengthMultiplier(s)
                             * max_isometric_force;

        // ACTIVATION HEAT RATE (W)
        // ------------------------
        if (get_forbid_negative_total_power() || get_activation_rate_on())
        {
            // This value is set to 1.0, as used by Anderson & Pandy (1999),
            // however, in Bhargava et al., (2004) they assume a function here.
            // We will ignore this function and use 1.0 for now.
            const double decay_function_value = 1.0;
            Adot = mm.getMuscleMass() * decay_function_value
                * ( (mm.get_activation_constant_slow_twitch()
                            * slow_twitch_excitation)
                        + (mm.get_activation_constant_fast_twitch()
                                * fast_twitch_excitation) );
        }

        // MAINTENANCE HEAT RATE (W)
        // -------------------------
        if (get_forbid_negative_total_power() || get_maintenance_rate_on())
        {
            SimTK::Vector tmp(1, fiber_length_normalized);
            fiber_length_dependence =
                get_normalized_fiber_length_dependence_on_maintenance_rate().
                    calcValue(tmp);
            Mdot = mm.getMuscleMass() * fiber_length_dependence
                * ( (mm.get_maintenance_constant_slow_twitch()
                        * slow_twitch_excitation)
                    + (mm.get_maintenance_constant_fast_twitch()
                            * fast_twitch_excitation) );
        }

        // SHORTENING HEAT RATE (W)
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening
        // --------------------------------------------------------------------
        if (get_forbid_negative_total_power() || get_shortening_rate_on())
        {
            if (get_use_force_dependent_shortening_prop_constant())
            {
                if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                    alpha = (0.16 * F_iso) + (0.18 * fiber_force_total);
                else                        // eccentric contraction, Vm>0
                    alpha = 0.157 * fiber_force_total;
            }
            else
            {
                if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                    alpha = 0.25 * fiber_force_total;
                else                        // eccentric contraction, Vm>0
                    alpha = 0.0;
            }
            Sdot = -alpha * fiber_velocity;
        }

        // MECHANICAL WORK RATE for the contractile element of the muscle (W).
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening.
        // -------------------------------------------------------------------
        if (get_forbid_negative_total_power() || get_mechanical_work_rate_on())
        {
            if (get_include_negative_mechanical_work() || fiber_velocity <= 0)
                Wdot = -fiber_force_active*fiber_velocity;
            else
                Wdot = 0;
        }

        // If necessary, increase the shortening heat rate so that the total
        // power is non-negative.
        if (get_forbid_negative_total_power()) {
            const double Edot_W_beforeClamp = Adot + Mdot + Sdot + Wdot;
            if (Edot_W_beforeClamp < 0)
                Sdot -= Edot_W_beforeClamp;
        }

        // This check is adapted from Umberger(2003), page 104: the total heat
        // rate (i.e., Adot + Mdot + Sdot) for a given muscle cannot fall below
        // 1.0 W/kg.
        // --------------------------------------------------------------------
        double totalHeatRate = Adot + Mdot + Sdot;

        if(get_enforce_minimum_heat_rate_per_muscle()
                    && totalHeatRate < 1.0 * mm.getMuscleMass()
                    && get_activation_rate_on()
                    && get_maintenance_rate_on()
                    && get_shortening_rate_on())
        {
                totalHeatRate = 1.0 * mm.getMuscleMass();
        }

        // TOTAL METABOLIC ENERGY RATE (W)
        // -------------------------------
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
