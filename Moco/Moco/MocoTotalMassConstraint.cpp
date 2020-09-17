/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTotalMassConstraint.h                                    *
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

#include "MocoTotalMassConstraint.h"
#include "MocoProblemInfo.h"

using namespace OpenSim;

MocoTotalMassConstraint::MocoTotalMassConstraint() {
    constructProperties();
}

void MocoTotalMassConstraint::constructProperties() {
    constructProperty_total_mass(1.0);
}

void MocoTotalMassConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo& problemInfo) const {

    setNumEquations(1);

    // TODO: setConstraintInfo() is not really intended for use here.
    MocoConstraintInfo info;
    std::vector<MocoBounds> bounds;
    bounds.emplace_back(0, 0);
    info.setBounds(bounds);
    const_cast<MocoTotalMassConstraint*>(this)->setConstraintInfo(info);
}

void MocoTotalMassConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state, SimTK::Vector& errors) const {
    getModel().getMultibodySystem().realize(state, SimTK::Stage::Instance);
    errors[0] = get_total_mass() - getModel().getTotalMass(state);
}