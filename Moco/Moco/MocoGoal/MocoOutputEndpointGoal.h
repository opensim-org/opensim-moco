#ifndef MOCO_MOCOOUTPUTENDPOINTGOAL_H
#define MOCO_MOCOOUTPUTENDPOINTGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoOutputEndpointGoal.h                                     *
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

#include "MocoGoal.h"

namespace OpenSim {

/// TODO
/// @mocogoal
class OSIMMOCO_API MocoOutputEndpointGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputEndpointGoal, MocoGoal);

public:
    MocoOutputEndpointGoal() {
        constructProperties();
    }
    MocoOutputEndpointGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoOutputEndpointGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// TODO
    void setOutputPath(std::string path) { set_output_path(std::move(path)); }
    const std::string& getOutputPath() const { return get_output_path(); }

    void setEndpoint(std::string endpoint) {
        set_endpoint(std::move(endpoint));
    }
    const std::string& getEndpoint() const { return get_endpoint(); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;

private:
    OpenSim_DECLARE_PROPERTY(output_path, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(endpoint, std::string, "TODO");
    void constructProperties();

    mutable SimTK::ReferencePtr<const AbstractOutput> m_output;
    mutable bool m_initial;
    mutable std::function<double(const AbstractOutput&, const SimTK::State&)>
            m_scalarize;
};

} // namespace OpenSim

#endif // MOCO_MOCOOUTPUTENDPOINTGOAL_H
