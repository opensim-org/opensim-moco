#ifndef MOCO_MOCOTOTALMASSCONSTRAINT_H
#define MOCO_MOCOTOTALMASSCONSTRAINT_H
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

#include "MocoConstraint.h"
#include "osimMocoDLL.h"

namespace OpenSim {

class MocoProblemInfo;

// TODO
class OSIMMOCO_API MocoTotalMassConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoTotalMassConstraint, MocoPathConstraint);

public:
    MocoTotalMassConstraint();

    void setTotalMass(double mass) { set_total_mass(mass); }
    double getTotalMass() const { return get_total_mass(); }

    protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;

    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;

private:
    OpenSim_DECLARE_PROPERTY(total_mass, double, "TODO");

    void constructProperties();
};

} // namespace OpenSim

#endif // MOCO_MOCOCONTROLBOUNDCONSTRAINT_H