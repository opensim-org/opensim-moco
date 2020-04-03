#ifndef MOCO_MOCONONNEGATIVEMUSCLEFORCECONSTRAINT_H
#define MOCO_MOCONONNEGATIVEMUSCLEFORCECONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoNonnegativeMuscleForceConstraint.h                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

#include "MocoConstraint.h"
#include "osimMocoDLL.h"

namespace OpenSim {

class OSIMMOCO_API MocoNonnegativeMuscleForceConstraint
        : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoNonnegativeMuscleForceConstraint, MocoPathConstraint);
public:

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;

private:
};

} // namespace OpenSim

#endif // MOCO_MOCONONNEGATIVEMUSCLEFORCECONSTRAINT_H
