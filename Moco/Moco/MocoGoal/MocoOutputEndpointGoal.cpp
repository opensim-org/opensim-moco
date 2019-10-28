/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoOutputEndpointGoal.cpp                                   *
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

#include "MocoOutputEndpointGoal.h"

using namespace OpenSim;

void MocoOutputEndpointGoal::constructProperties() {
    constructProperty_output_path("");
    constructProperty_endpoint("final");
}

void MocoOutputEndpointGoal::initializeOnModelImpl(const Model& output) const {
    std::string componentPath;
    std::string outputName;
    std::string channelName;
    std::string alias;
    AbstractInput::parseConnecteePath(
            get_output_path(), componentPath, outputName, channelName, alias);
    const auto& component = getModel().getComponent(componentPath);
    m_output.reset(&component.getOutput(outputName));
    if (get_endpoint() == "initial") {
        m_initial = true;
    } else if (get_endpoint() == "final"){
        m_initial = false;
    } else {
        OPENSIM_THROW_FRMOBJ(Exception,
                format("Expected 'endpoint' to be 'initial' or 'final', but "
                       "got '%s'.", get_endpoint()));
    }

    // TODO check type.
    if (dynamic_cast<const Output<double>*>(m_output.get())) {
        m_scalarize = [](const AbstractOutput& o,
                              const SimTK::State& s) -> double {
            return static_cast<const Output<double>&>(o).getValue(s);
        };
    } else if (dynamic_cast<const Output<SimTK::Vec3>*>(m_output.get())) {
        m_scalarize = [](const AbstractOutput& o,
                              const SimTK::State& s) -> double {
            double value = static_cast<const Output<SimTK::Vec3>&>(o)
                  .getValue(s)
                  .normSqr();
            return static_cast<const Output<SimTK::Vec3>&>(o)
                    .getValue(s)
                    .normSqr();
        };
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, format("Unexpected Output type '%s'.",
                m_output->getTypeName()));
    }
    setNumIntegralsAndOutputs(0, 1); // TODO, m_output->getDependsOnStage());

}

void MocoOutputEndpointGoal::calcGoalImpl(
        const MocoGoal::GoalInput& input, SimTK::Vector& cost) const {
    const SimTK::State* state;
    if (m_initial) {
        state = &input.initial_state;
    } else {
        state = &input.final_state;
    }
    getModel().getSystem().realize(*state, m_output->getDependsOnStage());

    cost[0] = m_scalarize(m_output.getRef(), *state);
}



