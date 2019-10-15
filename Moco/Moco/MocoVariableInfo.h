#ifndef MOCO_MOCOVARIABLEINFO_H
#define MOCO_MOCOVARIABLEINFO_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoProblem.h                                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

namespace OpenSim {

/// Bounds on continuous variables (states, controls, multipliers, etc). For 
/// states, the name should correspond to a path of a state variable in the 
/// model. For controls, the name should correspond to a path of an actuator
/// in the model, or, for controls associated with actuators that have more than 
/// one control, the path of an actuator in the model appended by the control 
/// index (e.g. "/actuator_0").
class OSIMMOCO_API MocoVariableInfo : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoVariableInfo, Object);

public:
    MocoVariableInfo();
    MocoVariableInfo(const std::string& name, const MocoBounds&,
            const MocoInitialBounds&, const MocoFinalBounds&);
    MocoVariableInfo(const std::string& name, const MocoFunctionBounds&);

    /// TODO
    bool getUseFunctionBounds() const { return get_use_function_bounds(); }

    double calcLowerFunctionBound(const double& time) const {
        OPENSIM_THROW_IF_FRMOBJ(!getUseFunctionBounds(), Exception,
                "This info is not set to use function bounds.");
        return get_function_bounds().calcLower(time);
    }

    double calcUpperFunctionBound(const double& time) const {
        OPENSIM_THROW_IF_FRMOBJ(!getUseFunctionBounds(), Exception,
                "This info is not set to use function bounds.");
        return get_function_bounds().calcUpper(time);
    }

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MocoBounds getPhaseBounds() const {
        OPENSIM_THROW_IF_FRMOBJ(getUseFunctionBounds(), Exception,
                "This info is set to use function bounds.");
        return MocoBounds(getProperty_phase_bounds());
    }
    /// @copydoc getPhaseBounds()
    MocoInitialBounds getInitialBounds() const {
        OPENSIM_THROW_IF_FRMOBJ(getUseFunctionBounds(), Exception,
                "This info is set to use function bounds.");
        return MocoInitialBounds(getProperty_initial_bounds());
    }
    /// @copydoc getPhaseBounds()
    MocoFinalBounds getFinalBounds() const {
        OPENSIM_THROW_IF_FRMOBJ(getUseFunctionBounds(), Exception,
                "This info is set to use function bounds.");
        return MocoFinalBounds(getProperty_final_bounds());
    }

    void setPhaseBounds(const MocoBounds& bounds) {
        set_phase_bounds(bounds.getAsArray());
    }
    void setInitialBounds(const MocoInitialBounds& bounds) {
        set_initial_bounds(bounds.getAsArray());
    }
    void setFinalBounds(const MocoFinalBounds& bounds) {
        set_final_bounds(bounds.getAsArray());
    }

    /// Throws an exception if initial and final bounds are not within phase
    /// bounds. This does not check the function bounds.
    // TODO Move to finalizeFromProperties() and cache MocoBounds.
    void validate() const;

    /// Print the bounds on this variable.
    void printDescription(std::ostream& stream = std::cout) const;

protected:
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(phase_bounds, double, 2,
            "1 value: required value over all time. "
            "2 values: lower, upper bounds on value over all time.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(initial_bounds, double, 2,
            "1 value: required initial value. "
            "2 values: lower, upper bounds on initial value.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(final_bounds, double, 2,
            "1 value: required final value. "
            "2 values: lower, upper bounds on final value.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(function_bounds, MocoFunctionBounds,
            "TODO");
    OpenSim_DECLARE_PROPERTY(use_function_bounds, bool, "TODO");

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // MOCO_MOCOVARIABLEINFO_H
