/* -------------------------------------------------------------------------- *
 * OpenSim Moco: SynergyController.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Chris Dembia                                                    *
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

#include "SynergyController.h"

#include "../MocoUtilities.h"

using namespace OpenSim;

SynergyController::SynergyController() {
    constructProperties();
}

void SynergyController::constructProperties() {
    constructProperty_synergy_weights();
}

void SynergyController::setSynergyWeights(
        std::string actuatorControlName, SimTK::Vector weights) {
    append_synergy_weights(SynergyWeightsVector(
            std::move(actuatorControlName), std::move(weights)));
}

void SynergyController::extendFinalizeFromProperties() {
    // TODO support non-scalar actuators.
    int numActuatorControls = getActuatorSet().getSize();
    OPENSIM_THROW_IF_FRMOBJ(
            numActuatorControls != getProperty_synergy_weights().size(),
            Exception,
            format("Number of Actuators in this Controller (%i) must "
                   "match the number of synergy weights vectors (%i).",
                    numActuatorControls,
            getProperty_synergy_weights().size()));

    if (numActuatorControls == 0) return;

    int numSynergies = get_synergy_weights(0).get_weights().size();
    m_synergyWeights.resize(numActuatorControls, numSynergies);

    std::unordered_map<std::string, int> map;
    for (int isw = 0; isw < getProperty_synergy_weights().size(); ++isw) {
        const auto& synWeights = get_synergy_weights(isw);
        map[synWeights.getName()] = isw;
    }
    for (int iact = 0; iact < numActuatorControls; ++iact) {
        const auto& actuator = getActuatorSet().get(iact);
        const auto actPath = actuator.getAbsolutePathString();

        OPENSIM_THROW_IF_FRMOBJ(!map.count(actPath), Exception,
                format("No synergy weights provided for Actuator '%s'.",
                        actPath));
        const auto& weights = get_synergy_weights(map[actPath]).get_weights();
        OPENSIM_THROW_IF_FRMOBJ(weights.size() != numSynergies,
                Exception,
                format("Length of synergy weights vectors is "
                "inconsistent (%i vs %i).", numSynergies, weights.size()));
        m_synergyWeights.updRow(iact) = weights.transpose();
    }
}

void SynergyController::computeControls(
        const SimTK::State& s, SimTK::Vector& controls) const {
    if (getActuatorSet().getSize() == 0) return;

    const auto synergyControls =
            getInputValue<SimTK::Vector>(s, "synergy_controls");
    SimTK::Vector actuatorControls = m_synergyWeights * synergyControls;
    for (int iact = 0; iact < getActuatorSet().getSize(); ++iact) {
        const auto& actuator = getActuatorSet().get(iact);
        actuator.addInControls(createVector({actuatorControls[iact]}),
                // actuatorControls.block()(TODO, 0, actuator.numControls(), 1),
                controls);
    }
}
