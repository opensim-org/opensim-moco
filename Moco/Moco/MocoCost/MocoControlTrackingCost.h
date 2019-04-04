#ifndef MOCO_MOCOCONTROLTRACKINGCOST_H
#define MOCO_MOCOCONTROLTRACKINGCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlTrackingCost.h                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoCost.h"
#include "../MocoWeightSet.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/GCVSplineSet.h>

namespace OpenSim {

class OSIMMOCO_API MocoControlTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoControlTrackingCost, MocoCost);

public:
    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
        "Set of weight objects to weight the tracking of individual "
        "control variables in the cost.");

    MocoControlTrackingCost() {}
    MocoControlTrackingCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {
        constructProperties();
    }

    void setReference(const TimeSeriesTable& ref) {
        m_table = ref;
    }
    /// Set the weight for an individual control variable. If a weight is
    /// already set for the requested control, then the provided weight
    /// replaces the previous weight. An exception is thrown if a weight
    /// for an unknown state is provided.
    void setWeight(const std::string& controlName, const double& weight) {
        if (get_control_weights().contains(controlName)) {
            upd_control_weights().get(controlName).setWeight(weight);
        } else {
            upd_control_weights().cloneAndAppend({controlName, weight});
        }
    }
    /// Provide a MocoWeightSet to weight the control variables in the cost.
    /// Replaces the weight set if it already exists.
    void setWeightSet(const MocoWeightSet& weightSet) {
        for (int w = 0; w < weightSet.getSize(); ++w) {
            const auto& weight = weightSet[w];
            setWeight(weight.getName(), weight.getWeight());
        }
    }

protected:
    void initializeOnModelImpl(const Model& model) const override;
    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override;

private:
    TimeSeriesTable m_table;
    mutable GCVSplineSet m_refsplines;
    mutable std::vector<int> m_controlIndices;
    mutable std::vector<double> m_control_weights;

    void constructProperties();
};

} // namespace OpenSim

#endif // MOCO_MOCOCONTROLTRACKINGCOST_H