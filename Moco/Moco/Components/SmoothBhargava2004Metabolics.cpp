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
//  SmoothBhargava2004Metabolics_MetabolicMusclePrameters
//=============================================================================

SmoothBhargava2004Metabolics_MetabolicMusclePrameters::
SmoothBhargava2004Metabolics_MetabolicMusclePrameters() {
    constructProperties();
}

SmoothBhargava2004Metabolics_MetabolicMusclePrameters::
SmoothBhargava2004Metabolics_MetabolicMusclePrameters(
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

SmoothBhargava2004Metabolics_MetabolicMusclePrameters::
SmoothBhargava2004Metabolics_MetabolicMusclePrameters(
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

void SmoothBhargava2004Metabolics_MetabolicMusclePrameters::
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

void SmoothBhargava2004Metabolics_MetabolicMusclePrameters::
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

double SmoothBhargava2004Metabolics::getTotalMetabolicRate(
        const SimTK::State& s) const {
    return getMetabolicRate(s).sum();
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
            // This "maxPower" is Fmax * Vmax.
            // Must convert max contraction velocity from
            // optimal_fiber_lengths/second to meters/second.
            const double maxPower = muscle.get_max_isometric_force() *
                                    muscle.get_max_contraction_velocity() *
                                    muscle.get_optimal_fiber_length();
            m_muscles.emplace_back(&muscle, maxPower);
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
    int i = 0;
    for (const auto& entry : m_muscles) {
        const double activation = entry.first->getActivation(s);
        const double& maxPower = entry.second;
        const double normFiberShortVel =
                -entry.first->getNormalizedFiberVelocity(s);

        const double numerator =
                0.054 * normFiberShortVel * (0.506 + normFiberShortVel * 2.46);
        const double denominator =
                1 +
                normFiberShortVel *
                        (-1.13 + normFiberShortVel *
                                         (12.8 + normFiberShortVel * -1.64));

        ratesForMuscles[i] = activation * maxPower * numerator / denominator;
        ++i;
    }
}
