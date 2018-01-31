/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoParameter.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MucoParameter.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MucoParameter::MucoParameter() {
    constructProperties();
    if (getName().empty()) setName("parameter");
}

MucoParameter::MucoParameter(const std::string& name,
    const std::string& componentPath,
    const std::string& propertyName,
    const MucoBounds& bounds) : MucoParameter() {
    setName(name);
    set_bounds(bounds.getAsArray());
    append_component_paths(componentPath);
    set_property_name(propertyName);
}

MucoParameter::MucoParameter(const std::string& name,
    const std::vector<std::string>& componentPaths,
    const std::string& propertyName,
    const MucoBounds& bounds,
    const unsigned& propertyElt) : MucoParameter() {
    setName(name);
    set_bounds(bounds.getAsArray());
    set_component_paths(componentPaths.get_allocator());
    set_property_name(propertyName);
}

void MucoParameter::constructProperties() {
    constructProperty_bounds();
    constructProperty_property_name("");
    constructProperty_component_paths();
}

void MucoParameter::initialize(const Model& model) const {
    
    OPENSIM_THROW_IF(getProperty_component_paths().empty(), Exception,
        "A model component name must be provided.");
    OPENSIM_THROW_IF(get_property_name().empty(), Exception,
        "A component property name must be provided.");

    for (int i = 0; i < (int)getProperty_component_paths().size(); ++i) {
        // Get model component.
        auto& component = model.getComponent(get_component_paths(i));
        // Get component property.
        // TODO: get rid of need for const_cast
        auto& property = dynamic_cast<Property<double>&>(
            const_cast<AbstractProperty&>(
                component.getPropertyByName(get_property_name())));

        m_property_refs.emplace_back(&property);
    }
}

void MucoParameter::applyParameterToModel(const double& value) const {
    for (auto& propRef : m_property_refs) {
        propRef->setValue(value);
    }
}