/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlConstraint.h                                      *
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

#include "MocoControlConstraint.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void MocoControlConstraint::initializeOnModelImpl(const Model& model) const {
    // Convert data table to splines.
    // auto controlsFiltered = filterLowpass(m_table, 6, true);
    auto allSplines = GCVSplineSet(m_table);

    // Get all expected control names.
    std::vector<std::string> controlNames;
    const auto modelPath = model.getAbsolutePath();
    for (const auto& actu : model.getComponentList<Actuator>()) {
        std::string actuPath =
            actu.getAbsolutePath().formRelativePath(modelPath).toString();
        if (actu.numControls() == 1) {
            controlNames.push_back(actuPath);
        } else {
            for (int i = 0; i < actu.numControls(); ++i) {
                controlNames.push_back(actuPath + "_" + std::to_string(i));
            }
        }
    }

    // TODO this assumes controls are in the same order as actuators.
    // The loop that processes weights (two down) assumes that controls are 
    // in the same order as actuators. However, the control indices are 
    // allocated in the order in which addToSystem() is invoked (not 
    // necessarily the order used by getComponentList()). So until we can be 
    // absolutely sure that the controls are in the same order as actuators, 
    // we run the following check: in order, set an actuator's control 
    // signal(s) to NaN and ensure the i-th control is NaN.
    {
        const SimTK::State state = model.getWorkingState();
        int i = 0;
        auto modelControls = model.updControls(state);
        for (const auto& actu : model.getComponentList<Actuator>()) {
            int nc = actu.numControls();
            SimTK::Vector origControls(nc);
            SimTK::Vector nan(nc, SimTK::NaN);
            actu.getControls(modelControls, origControls);
            actu.setControls(nan, modelControls);
            for (int j = 0; j < nc; ++j) {
                OPENSIM_THROW_IF_FRMOBJ(!SimTK::isNaN(modelControls[i]),
                    Exception, "Internal error: actuators are not in the "
                    "expected order. Submit a bug report.");
                ++i;
            }
            actu.setControls(origControls, modelControls);
        }
    }

    for (int iref = 0; iref < allSplines.getSize(); ++iref) {
        const auto& refName = allSplines[iref].getName();

        int i = 0;
        for (const auto& actu : model.getComponentList<Actuator>()) {
            std::string actuPath =
                actu.getAbsolutePath().formRelativePath(modelPath).toString();
            if (actu.numControls() == 1) {
                if (refName == actuPath) {
                    m_refsplines.cloneAndAppend(allSplines[iref]);
                    m_controlIndices.push_back(i);
                }

                ++i;
            } else {
                for (int j = 0; j < actu.numControls(); ++j) {
                    std::string controlName = actuPath + "_" +
                        std::to_string(j);
                    if (refName == controlName) {
                        m_refsplines.cloneAndAppend(allSplines[iref]);
                        m_controlIndices.push_back(i);
                    }

                    ++i;
                }
            }
        }
    }

    setNumEquations(m_refsplines.getSize());
}

void MocoControlConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state, SimTK::Vector& errors) const {

    const auto& time = state.getTime();
    SimTK::Vector timeVec(1, time);
    getModel().realizeVelocity(state);

    const auto& controls = getModel().getControls(state);
    // TODO cache the reference coordinate values at the mesh points, 
    // rather than evaluating the spline.
    for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
        const auto& refValue = m_refsplines[iref].calcValue(timeVec);
        errors[iref] = 
            pow(controls[m_controlIndices[iref]] - refValue, 2);
    }
}