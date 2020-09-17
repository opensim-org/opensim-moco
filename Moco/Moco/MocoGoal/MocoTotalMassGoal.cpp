/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTotalMassGoal.h                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

#include "MocoTotalMassGoal.h"

using namespace OpenSim;

MocoTotalMassGoal::MocoTotalMassGoal() {
    constructProperties();
}

void MocoTotalMassGoal::constructProperties() {
    constructProperty_total_mass(1.0);
}

void MocoTotalMassGoal::initializeOnModelImpl(
        const Model& model) const {
    setRequirements(0, 1, SimTK::Stage::Instance);
}

void MocoTotalMassGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& goal) const {
    const auto& s = input.initial_state;
    getModel().getMultibodySystem().realize(s, SimTK::Stage::Instance);
    goal[0] = get_total_mass() - getModel().getTotalMass(s);
}
