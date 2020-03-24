/* -------------------------------------------------------------------------- *
 * OpenSim Moco: Bhargava2004Metabolics.cpp                                   *
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

#include "Bhargava2004Metabolics.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
//  Bhargava2004Metabolics_MuscleParameters
//=============================================================================

Bhargava2004Metabolics_MuscleParameters::
Bhargava2004Metabolics_MuscleParameters() {
    constructProperties();
}

void Bhargava2004Metabolics_MuscleParameters::
setMuscleMass()
{
    if (SimTK::isNaN(get_provided_muscle_mass())) {
        muscleMass =
            (getMuscle().getMaxIsometricForce() / get_specific_tension())
            * get_density() * getMuscle().getOptimalFiberLength();
    } else {
        muscleMass = get_provided_muscle_mass();
    }
}

void Bhargava2004Metabolics_MuscleParameters::
constructProperties()
{
    // Specific tension of mammalian muscle (Pascals (N/m^2)).
    constructProperty_specific_tension(0.25e6);
    // Density of mammalian muscle (kg/m^3).
    constructProperty_density(1059.7);

    constructProperty_ratio_slow_twitch_fibers(0.5);

    constructProperty_provided_muscle_mass(SimTK::NaN);

    // Defaults (W/kg) from Bhargava et al (2004).
    constructProperty_activation_constant_slow_twitch(40.0);
    constructProperty_activation_constant_fast_twitch(133.0);
    constructProperty_maintenance_constant_slow_twitch(74.0);
    constructProperty_maintenance_constant_fast_twitch(111.0);
}

//=============================================================================
//  Bhargava2004Metabolics
//=============================================================================
Bhargava2004Metabolics::Bhargava2004Metabolics()
{
    constructProperties();
}

void Bhargava2004Metabolics::addMuscle(const std::string& name,
        const Muscle& muscle, double muscle_mass) {
    append_muscle_parameters(Bhargava2004Metabolics_MuscleParameters());
    auto& mp = upd_muscle_parameters(
            getProperty_muscle_parameters().size() - 1);
    mp.setName(name);
    if (!SimTK::isNaN(muscle_mass))
        mp.set_provided_muscle_mass(muscle_mass);
    mp.connectSocket_muscle(muscle);
    mp.setMuscleMass();
}

void Bhargava2004Metabolics::addMuscle(const std::string& name,
        const Muscle& muscle, double ratio_slow_twitch_fibers,
        double specific_tension, double muscle_mass) {
    append_muscle_parameters(Bhargava2004Metabolics_MuscleParameters());
    auto& mp = upd_muscle_parameters(
            getProperty_muscle_parameters().size() - 1);
    mp.setName(name);
    mp.set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    mp.set_specific_tension(specific_tension);
    if (!SimTK::isNaN(muscle_mass))
        mp.set_provided_muscle_mass(muscle_mass);
    mp.connectSocket_muscle(muscle);
    mp.setMuscleMass();
}

void Bhargava2004Metabolics::addMuscle(const std::string& name,
        const Muscle& muscle, double ratio_slow_twitch_fibers,
        double specific_tension, double activation_constant_slow_twitch,
        double activation_constant_fast_twitch,
        double maintenance_constant_slow_twitch,
        double maintenance_constant_fast_twitch,
        double muscle_mass) {
    append_muscle_parameters(Bhargava2004Metabolics_MuscleParameters());
    auto& mp = upd_muscle_parameters(
            getProperty_muscle_parameters().size() - 1);
    mp.setName(name);
    mp.set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    mp.set_specific_tension(specific_tension);
    mp.set_activation_constant_slow_twitch(activation_constant_slow_twitch);
    mp.set_activation_constant_fast_twitch(activation_constant_fast_twitch);
    mp.set_maintenance_constant_slow_twitch(maintenance_constant_slow_twitch);
    mp.set_maintenance_constant_fast_twitch(maintenance_constant_fast_twitch);
    if (!SimTK::isNaN(muscle_mass))
        mp.set_provided_muscle_mass(muscle_mass);
    mp.connectSocket_muscle(muscle);
    mp.setMuscleMass();
}

void Bhargava2004Metabolics::constructProperties()
{
    constructProperty_muscle_parameters();

    constructProperty_enforce_minimum_heat_rate_per_muscle(true);

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

void Bhargava2004Metabolics::extendFinalizeFromProperties() {
    if (get_use_smoothing()) {
        m_conditional = [](const double& cond, const double& left,
                                const double& right, const double& smoothing) {
            const double smoothed_binary = 0.5 + 0.5 * tanh(smoothing * cond);
            return left + (-left + right) * smoothed_binary;
        };
    } else {
        m_conditional = [](const double& cond, const double& left,
                                const double& right, const double&) {
            if (cond <= 0) {
                return left;
            } else {
                return right;
            }
        };
    }
}

double Bhargava2004Metabolics::getTotalMetabolicRate(
        const SimTK::State& s) const {
    // BASAL METABOLIC RATE (W) (based on whole body mass, not muscle mass).
    // ---------------------------------------------------------------------
    double Bdot = get_basal_coefficient()
            * pow(getModel().getMatterSubsystem().calcSystemMass(s),
                    get_basal_exponent());
    return getMetabolicRate(s).sum() + Bdot;
}

double Bhargava2004Metabolics::getTotalActivationRate(
        const SimTK::State& s) const {
    return getActivationRate(s).sum();
}

double Bhargava2004Metabolics::getTotalMaintenanceRate(
        const SimTK::State& s) const {
    return getMaintenanceRate(s).sum();
}

double Bhargava2004Metabolics::getTotalShorteningRate(
        const SimTK::State& s) const {
    return getShorteningRate(s).sum();
}

double Bhargava2004Metabolics::getTotalMechanicalWorkRate(
        const SimTK::State& s) const {
    return getMechanicalWorkRate(s).sum();
}

double Bhargava2004Metabolics::getMuscleMetabolicRate(
        const SimTK::State& s, const std::string& channel) const {
    return getMetabolicRate(s).get(m_muscleIndices.at(channel));
}

void Bhargava2004Metabolics::extendRealizeTopology(SimTK::State& state)
const {
    Super::extendRealizeTopology(state);
    m_muscleIndices.clear();
    for (int i=0; i < getProperty_muscle_parameters().size(); ++i) {
        const auto& muscle = get_muscle_parameters(i).getMuscle();
        if (muscle.get_appliesForce()) {
            m_muscleIndices[muscle.getAbsolutePathString()] = i;
            ++i;
        }
    }
}

void Bhargava2004Metabolics::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    SimTK::Vector rates = SimTK::Vector((int)m_muscleIndices.size(), 0.0);
    addCacheVariable<SimTK::Vector>("metabolic_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("activation_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("maintenance_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("shortening_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("mechanical_work_rate", rates,
            SimTK::Stage::Dynamics);
}

void Bhargava2004Metabolics::calcMetabolicRateForCache(
    const SimTK::State& s) const {
    calcMetabolicRate(s,
            updCacheVariableValue<SimTK::Vector>(s, "metabolic_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "activation_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "maintenance_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "shortening_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "mechanical_work_rate")
            );
    markCacheVariableValid(s, "metabolic_rate");
    markCacheVariableValid(s, "activation_rate");
    markCacheVariableValid(s, "maintenance_rate");
    markCacheVariableValid(s, "shortening_rate");
    markCacheVariableValid(s, "mechanical_work_rate");
}

const SimTK::Vector& Bhargava2004Metabolics::getMetabolicRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "metabolic_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "metabolic_rate");
}

const SimTK::Vector& Bhargava2004Metabolics::getActivationRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "activation_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "activation_rate");
}

const SimTK::Vector& Bhargava2004Metabolics::getMaintenanceRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "maintenance_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "maintenance_rate");
}

const SimTK::Vector& Bhargava2004Metabolics::getShorteningRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "shortening_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "shortening_rate");
}

const SimTK::Vector& Bhargava2004Metabolics::getMechanicalWorkRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "mechanical_work_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "mechanical_work_rate");
}

void Bhargava2004Metabolics::calcMetabolicRate(
        const SimTK::State& s, SimTK::Vector& totalRatesForMuscles,
        SimTK::Vector& activationRatesForMuscles,
        SimTK::Vector& maintenanceRatesForMuscles,
        SimTK::Vector& shorteningRatesForMuscles,
        SimTK::Vector& mechanicalWorkRatesForMuscles) const {
    totalRatesForMuscles.resize((int)m_muscleIndices.size());
    activationRatesForMuscles.resize((int)m_muscleIndices.size());
    maintenanceRatesForMuscles.resize((int)m_muscleIndices.size());
    shorteningRatesForMuscles.resize((int)m_muscleIndices.size());
    mechanicalWorkRatesForMuscles.resize((int)m_muscleIndices.size());
    double activationHeatRate, maintenanceHeatRate, shorteningHeatRate;
    double mechanicalWorkRate;
    activationHeatRate = maintenanceHeatRate = shorteningHeatRate =
        mechanicalWorkRate = 0;

    int i = 0;
    for (const auto& muscleIndex : m_muscleIndices) {

        const auto& muscleParameter =
            get_muscle_parameters(muscleIndex.second);
        const auto& muscle = muscleParameter.getMuscle();

        const double maximalIsometricForce = muscle.getMaxIsometricForce();
        const double activation =
            get_muscle_effort_scaling_factor() * muscle.getActivation(s);
        const double excitation =
            get_muscle_effort_scaling_factor() * muscle.getControl(s);
        const double fiberForcePassive =  muscle.getPassiveFiberForce(s);
        const double fiberForceActive =
            get_muscle_effort_scaling_factor() * muscle.getActiveFiberForce(s);
        const double fiberForceTotal =
            fiberForceActive + fiberForcePassive;
        const double fiberLengthNormalized =
            muscle.getNormalizedFiberLength(s);
        const double fiberVelocity = muscle.getFiberVelocity(s);
        const double slowTwitchExcitation =
            muscleParameter.get_ratio_slow_twitch_fibers()
            * sin(SimTK::Pi/2 * excitation);
        const double fastTwitchExcitation =
            (1 - muscleParameter.get_ratio_slow_twitch_fibers())
            * (1 - cos(SimTK::Pi/2 * excitation));

        // Get the unnormalized total active force, isometricTotalActiveForce
        // that 'would' be developed at the current activation and fiber length
        // under isometric conditions (i.e., fiberVelocity=0).
        const double isometricTotalActiveForce =
            activation * muscle.getActiveForceLengthMultiplier(s)
            * maximalIsometricForce;

        // ACTIVATION HEAT RATE (W).
        // -------------------------
        // This value is set to 1.0, as used by Anderson & Pandy (1999),
        // however, in Bhargava et al., (2004) they assume a function here.
        // We will ignore this function and use 1.0 for now.
        const double decay_function_value = 1.0;
        activationHeatRate =
            muscleParameter.getMuscleMass() * decay_function_value
            * ( (muscleParameter.get_activation_constant_slow_twitch()
                        * slowTwitchExcitation)
                + (muscleParameter.get_activation_constant_fast_twitch()
                        * fastTwitchExcitation) );

        // MAINTENANCE HEAT RATE (W).
        // --------------------------
        const int curvePoints = 5;
        const double curveX[] = {0.0, 0.5, 1.0, 1.5, 10.0};
        const double curveY[] = {0.5, 0.5, 1.0, 0.0, 0.0};
        PiecewiseLinearFunction fiberLengthDepCurveDefault(curvePoints, curveX,
                curveY, "defaultCurve");
        const double fiber_length_dependence =
            fiberLengthDepCurveDefault.calcValue(
                    SimTK::Vector(1, fiberLengthNormalized));
        maintenanceHeatRate =
            muscleParameter.getMuscleMass() * fiber_length_dependence
                * ( (muscleParameter.get_maintenance_constant_slow_twitch()
                            * slowTwitchExcitation)
                + (muscleParameter.get_maintenance_constant_fast_twitch()
                            * fastTwitchExcitation) );

        // SHORTENING HEAT RATE (W).
        // --> note that we define fiberVelocity<0 as shortening and
        //     fiberVelocity>0 as lengthening.
        // ---------------------------------------------------------
        double alpha;
        if (get_use_force_dependent_shortening_prop_constant())
        {
            alpha = m_conditional(fiberVelocity,
                    (0.16 * isometricTotalActiveForce)
                    + (0.18 * fiberForceTotal),
                    0.157 * fiberForceTotal,
                    get_velocity_smoothing());
        } else {
            // This simpler value of alpha comes from Frank Anderson's 1999
            // dissertation "A Dynamic Optimization Solution for a Complete
            // Cycle of Normal Gait".
            alpha = m_conditional(fiberVelocity,
                    0.25 * fiberForceTotal,
                    0,
                    get_velocity_smoothing());
        }
        shorteningHeatRate = -alpha * fiberVelocity;

        // MECHANICAL WORK RATE for the contractile element of the muscle (W).
        // --> note that we define fiberVelocity<0 as shortening and
        //     fiberVelocity>0 as lengthening.
        // -------------------------------------------------------------------
        if (get_include_negative_mechanical_work())
        {
            mechanicalWorkRate = -fiberForceActive * fiberVelocity;
        } else {
            mechanicalWorkRate = m_conditional(fiberVelocity,
                    -fiberForceActive * fiberVelocity,
                    0,
                    get_velocity_smoothing());
        }

        // NAN CHECKING
        // ------------------------------------------
        if (SimTK::isNaN(activationHeatRate))
            std::cout << "WARNING::" << getName() << ": activationHeatRate ("
                    << muscleParameter.getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(maintenanceHeatRate))
            std::cout << "WARNING::" << getName() << ": maintenanceHeatRate ("
                    << muscleParameter.getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(shorteningHeatRate))
            std::cout << "WARNING::" << getName() << ": shorteningHeatRate ("
                    << muscleParameter.getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(mechanicalWorkRate))
            std::cout << "WARNING::" << getName() << ": mechanicalWorkRate ("
                    <<  muscleParameter.getName() << ") = NaN!" << std::endl;

        // If necessary, increase the shortening heat rate so that the total
        // power is non-negative.
        if (get_forbid_negative_total_power()) {
            const double Edot_W_beforeClamp = activationHeatRate
                + maintenanceHeatRate + shorteningHeatRate
                + mechanicalWorkRate;
            if (get_use_smoothing()) {
                // Variables for smooth approximations between negative and
                // positive total power.
                const double bp = get_power_smoothing();
                // Edot_W_beforeClamp_neg=1 if Edot_W_beforeClamp is negative.
                const double Edot_W_beforeClamp_neg = tanhSmoothing(
                        -Edot_W_beforeClamp, 0, bp);

                shorteningHeatRate -=
                    Edot_W_beforeClamp * Edot_W_beforeClamp_neg;
            } else {
                if (Edot_W_beforeClamp < 0)
                    shorteningHeatRate -= Edot_W_beforeClamp;
            }
        }

        // This check is adapted from Umberger(2003), page 104: the total heat
        // rate (i.e., activationHeatRate + maintenanceHeatRate
        // + shorteningHeatRate) for a given muscle cannot fall below 1.0 W/kg.
        // If the total heat rate falls below 1.0 W/kg, the sum of the reported
        // individual heat rates and work rate does not equal the reported
        // metabolic rate.
        // --------------------------------------------------------------------
        double totalHeatRate = activationHeatRate + maintenanceHeatRate
            + shorteningHeatRate;
        if (get_use_smoothing()) {
            if (get_enforce_minimum_heat_rate_per_muscle())
            {
                // Variables for smooth approximations between total heat rate
                // for a given muscle below or above 1.0 W/kg.
                const double bhr = get_heat_rate_smoothing();
                // totalHeatRate_bmm=1 if totalHeatRate is below muscle mass.
                const double totalHeatRate_bmm = tanhSmoothing(-totalHeatRate,
                        -1.0 *  muscleParameter.getMuscleMass(), bhr);

                totalHeatRate = totalHeatRate + (-totalHeatRate + 1.0 *
                        muscleParameter.getMuscleMass()) * totalHeatRate_bmm;
            }
        } else {
            if (get_enforce_minimum_heat_rate_per_muscle()
                    && totalHeatRate < 1.0 * muscleParameter.getMuscleMass())
            {
                totalHeatRate = 1.0 * muscleParameter.getMuscleMass();
            }
        }

        // TOTAL METABOLIC ENERGY RATE (W).
        // --------------------------------
        double Edot = totalHeatRate + mechanicalWorkRate;

        totalRatesForMuscles[i] = Edot;
        activationRatesForMuscles[i] = activationHeatRate;
        maintenanceRatesForMuscles[i] = maintenanceHeatRate;
        shorteningRatesForMuscles[i] = shorteningHeatRate;
        mechanicalWorkRatesForMuscles[i] = mechanicalWorkRate;

        ++i;
    }
}

int Bhargava2004Metabolics::getNumMetabolicMuscles() const
{
    return getProperty_muscle_parameters().size();
}

double Bhargava2004Metabolics::tanhSmoothing(const double x,
        double smoothing_threshold, double smoothing_constant) const
{
    return 0.5 + 0.5 * (tanh(smoothing_constant * (x - smoothing_threshold)));
}
