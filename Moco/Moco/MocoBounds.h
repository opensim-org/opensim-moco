#ifndef MOCO_MOCOBOUNDS_H
#define MOCO_MOCOBOUNDS_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoBounds.h                                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "osimMocoDLL.h"

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Function.h>

namespace OpenSim {

class MocoPhase;
class MocoVariableInfo;
class MocoParameter;

/// Small struct to handle bounds.
class OSIMMOCO_API MocoBounds : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoBounds, Object);
public:
    /// The bounds are unset (NaN).
    MocoBounds();
    /// The lower and upper bound are equal (the variable is constrained to this
    /// single value).
    MocoBounds(double value);
    /// The variable is constrained to be within [lower, upper].
    MocoBounds(double lower, double upper);
    /// Create bounds that are (-inf, inf), so the variable is unconstrained.
    static MocoBounds unconstrained() {
        return MocoBounds(-SimTK::Infinity, SimTK::Infinity);
    }
    /// True if the lower and upper bounds are both not NaN.
    bool isSet() const {
        return !getProperty_bounds().empty();
    }
    /// True if the lower and upper bounds are the same, resulting in an
    /// equality constraint.
    bool isEquality() const {
        return isSet() && getLower() == getUpper();
    }
    /// Returns true if the provided value is within these bounds.
    bool isWithinBounds(const double& value) const {
        return getLower() <= value && value <= getUpper();
    }
    double getLower() const {
        if (!isSet()) {
            return SimTK::NTraits<double>::getNaN();
        } else {
            return get_bounds(0);
        }
    }
    double getUpper() const {
        if (!isSet()) {
            return SimTK::NTraits<double>::getNaN();
        } else if (getProperty_bounds().size() == 1) {
            return get_bounds(0);
        } else {
            return get_bounds(1);
        }
    }
    /// The returned array has either 0, 1, or 2 elements.
    /// - 0 elements: bounds are not set.
    /// - 1 element: equality constraint
    /// - 2 elements: range (inequality constraint).
    Array<double> getAsArray() const {
        Array<double> vec;
        if (isSet()) {
            vec.append(getLower());
            if (getLower() != getUpper()) vec.append(getUpper());
        }
        return vec;
    }

    void printDescription(std::ostream& stream) const;

protected:
    /// Used internally to create Bounds from a list property.
    /// The list property must have either 0, 1 or 2 elements.
    MocoBounds(const Property<double>& p);

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
        "1 value: required value. "
        "2 values: lower, upper bounds on value.");

    friend MocoPhase;
    friend MocoVariableInfo;
    friend MocoParameter;

private:
    void constructProperties();

};
/// Used for specifying the bounds on a variable at the start of a phase.
class OSIMMOCO_API MocoInitialBounds : public MocoBounds {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoInitialBounds, MocoBounds);
    using MocoBounds::MocoBounds;
    friend MocoPhase;
    friend MocoVariableInfo;
};
/// Used for specifying the bounds on a variable at the end of a phase.
class OSIMMOCO_API MocoFinalBounds : public MocoBounds {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoFinalBounds, MocoBounds);
    using MocoBounds::MocoBounds;
    friend MocoPhase;
    friend MocoVariableInfo;
};

class OSIMMOCO_API MocoFunctionBounds : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFunctionBounds, Object);
public:
    MocoFunctionBounds();
    MocoFunctionBounds(const Function& bounds);
    MocoFunctionBounds(const Function& lower, const Function& upper);
    /// True if the lower and upper bounds are the same, resulting in an
    /// equality constraint.
    bool isEquality() const {
        return get_equality_with_lower();
    }
    /// Returns true if the provided value is within these bounds.
    bool isWithinBounds(const double& time, const double& value) const {
        return calcLower(time) <= value && value <= calcUpper(time);
    }
    double calcLower(const double& time) const {
        SimTK::Vector timeVec(1, &time, true);
        return get_lower_bound().calcValue(timeVec);
    }
    double calcUpper(const double& time) const {
        if (get_equality_with_lower()) {
            return calcLower(time);
        } else {
            OPENSIM_THROW_IF_FRMOBJ(getProperty_upper_bound().empty(),
                    Exception, "No upper bound provided.");
            SimTK::Vector timeVec(1, &time, true);
            return get_upper_bound().calcValue(timeVec);
        }
    }

    void printDescription(std::ostream& stream) const;

private:
    void constructProperties();

    OpenSim_DECLARE_PROPERTY(
            lower_bound, Function, "Lower bound as a function of time.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(
            upper_bound, Function, "Upper bound as a function of time.");
    OpenSim_DECLARE_PROPERTY(equality_with_lower, bool,
            "The variable must be equal to the lower bound; "
            "upper must be unspecified (default: false).");
};

} // namespace OpenSim

#endif // MOCO_MOCOBOUNDS_H
