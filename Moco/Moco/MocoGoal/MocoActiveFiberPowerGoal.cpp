/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoActiveFiberPowerGoal.h                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoActiveFiberPowerGoal.h"

using namespace OpenSim;

MocoActiveFiberPowerGoal::MocoActiveFiberPowerGoal() { constructProperties(); }

void MocoActiveFiberPowerGoal::constructProperties() {
    constructProperty_exponent(1);
    constructProperty_divide_by_displacement(false);
    constructProperty_muscle_density(1059.7);
    constructProperty_specific_tension(600000.0);
    constructProperty_ignore_negative_fiber_power(false);
    constructProperty_positive_fiber_power_smoothing(1e-3);
}

void MocoActiveFiberPowerGoal::initializeOnModelImpl(const Model& model) const {
    
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        m_muscleRefs.emplace_back(&muscle);
        // Physiological cross-sectional area.
        const double PCSA = 
                muscle.get_max_isometric_force() / get_specific_tension();
        // Muscle mass.
        const double mass = 
            PCSA * get_muscle_density() * muscle.get_optimal_fiber_length();

        m_muscleMasses.push_back(mass);
    }

    OPENSIM_THROW_IF_FRMOBJ(
            get_exponent() < 1, Exception, "Exponent must be 1 or greater.");
    int exponent = get_exponent();

    m_epsilonSquared = SimTK::square(get_positive_fiber_power_smoothing()); 

    // The pow() function gives slightly different results than x * x. On Mac,
    // using x * x requires fewer solver iterations.
    if (exponent == 1) {
        m_power_function = [](const double& x) { return x; };
    } else if (exponent == 2) {
        m_power_function = [](const double& x) { return x * x; };
    } else {
        m_power_function = [exponent](const double& x) {
            return pow(std::abs(x), exponent);
        };
    }

    setNumIntegralsAndOutputs(1, 1);
}

void MocoActiveFiberPowerGoal::calcIntegrandImpl(
        const SimTK::State& state, double& integrand) const {
    integrand = 0;
    for (int i = 0; i < (int)m_muscleRefs.size(); ++i) {
        const auto& activeFiberPower = 
            m_muscleRefs[i]->getFiberActivePower(state);
        //const auto& mass = m_muscleMasses[i];

        double normActiveFiberPower = activeFiberPower; // / mass;
        if (get_ignore_negative_fiber_power()) {
            normActiveFiberPower = 0.5 * (normActiveFiberPower + 
                sqrt(SimTK::square(normActiveFiberPower) + m_epsilonSquared));
        }

        integrand += m_power_function(normActiveFiberPower);
    }
}

void MocoActiveFiberPowerGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
    if (get_divide_by_displacement()) {
        const SimTK::Vec3 comInitial =
                getModel().calcMassCenterPosition(input.initial_state);
        const SimTK::Vec3 comFinal =
                getModel().calcMassCenterPosition(input.final_state);
        // TODO: Use distance squared for convexity.
        const SimTK::Real displacement = (comFinal - comInitial).norm();
        cost[0] /= displacement;
    }
}