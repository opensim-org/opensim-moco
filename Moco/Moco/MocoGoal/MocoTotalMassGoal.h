#ifndef MOCO_MOCOTOTALMASSGOAL_H
#define MOCO_MOCOTOTALMASSGOAL_H
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

#include "MocoGoal.h"

namespace OpenSim {

// TODO
/// This is an endpoint constraint goal by default.
/// @ingroup mocogoal
class OSIMMOCO_API MocoTotalMassGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoTotalMassGoal, MocoGoal);

public:
    MocoTotalMassGoal();
    MocoTotalMassGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    
    void setTotalMass(double mass) { set_total_mass(mass); }
    double getTotalMass() const { return get_total_mass(); }

protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    // TODO
    void initializeOnModelImpl(const Model&) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& goal) const override;

private:
    OpenSim_DECLARE_PROPERTY(total_mass, double, "TODO");

    void constructProperties();
};

} // namespace OpenSim

#endif // MOCO_MOCOTOTALMASSGOAL_H