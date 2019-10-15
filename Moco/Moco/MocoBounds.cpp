/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoBounds.cpp                                               *
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

#include "MocoBounds.h"

#include "MocoUtilities.h"

#include <OpenSim/Common/Constant.h>

using namespace OpenSim;

MocoBounds::MocoBounds() {
    constructProperties();
}

MocoBounds::MocoBounds(double value) : MocoBounds() {
    OPENSIM_THROW_IF(SimTK::isNaN(value), Exception, "NaN value detected. "
        "Please provide a non-NaN value for the bounds.");
    append_bounds(value);
}

MocoBounds::MocoBounds(double lower, double upper) : MocoBounds() {
    OPENSIM_THROW_IF(SimTK::isNaN(lower) || SimTK::isNaN(upper), Exception, 
        "NaN value detected. Please provide a non-NaN values for the bounds.");
    OPENSIM_THROW_IF(lower > upper, Exception,
        format("Expected lower <= upper, but lower=%g and upper=%g.",
                lower, upper));
    append_bounds(lower);
    append_bounds(upper);
}

MocoBounds::MocoBounds(const Property<double>& p) : MocoBounds() {
    assert(p.size() <= 2);
    if (p.size() >= 1) {
        OPENSIM_THROW_IF(SimTK::isNaN(p[0]), Exception, "NaN value detected. "
            "Please provide a non-NaN value for the bounds.");
        append_bounds(p[0]);
        if (p.size() == 2) {
            OPENSIM_THROW_IF(SimTK::isNaN(p[1]), Exception, "NaN value "
                "detected. Please provide a non-NaN value for the bounds.");
            append_bounds(p[1]);
        }
    }
}

void MocoBounds::constructProperties() {
    constructProperty_bounds();
}

void MocoBounds::printDescription(std::ostream& stream) const {
    if (isEquality()) {
        stream << getLower();
    }
    else {
        stream << "[" << getLower() << ", " << getUpper() << "]";
    }
    stream.flush();
}

MocoFunctionBounds::MocoFunctionBounds() {
    constructProperties();
}

MocoFunctionBounds::MocoFunctionBounds(const Function& lower)
        : MocoFunctionBounds() {
    set_lower_bound(lower);
}

MocoFunctionBounds::MocoFunctionBounds(const Function& lower,
        const Function& upper) : MocoFunctionBounds(lower) {
    set_upper_bound(upper);
}


void MocoFunctionBounds::printDescription(std::ostream& stream) const {
    // TODO: Print min and max of the functions.
    if (isEquality()) {
        stream << get_lower_bound().getConcreteClassName();
    }
    else {
        const auto& lower = get_lower_bound().getConcreteClassName();
        const auto& upper = get_upper_bound().getConcreteClassName();
        stream << "[" << lower << ", " << upper << "]";
    }
    stream.flush();
}

void MocoFunctionBounds::constructProperties() {
    constructProperty_lower_bound(Constant(0));
    constructProperty_upper_bound();
    constructProperty_equality_with_lower(false);
}
