/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxWholeBodyTracking.cpp                                 *
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
 
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <Moco/osimMoco.h>

using namespace OpenSim;

int main() {

    MocoTrack track;
    ModelProcessor modelProcessor =
            ModelProcessor("subject_walk_armless_contact_80musc.osim") |
            //ModOpAddExternalLoads("grf_walk.xml") |
            ModOpReplaceJointsWithWelds(
                    {"subtalar_r", "mtp_r", "subtalar_l", "mtp_l"}) |
            ModOpRemoveMuscles() |
            ModOpAddReserves(250, 1, true);

    //Model model = modelProcessor.process();

    //model.setUseVisualizer(true);
    //auto state = model.initSystem();
    //model.getVisualizer().show(state);

    track.setModel(modelProcessor);
    track.setStatesReference(TableProcessor("coordinates.mot") |
            TabOpLowPassFilter(6) |
            TabOpUseAbsoluteStateNames());
    track.set_allow_unused_references(true);
    track.set_markers_global_tracking_weight(10);

    track.set_initial_time(0.83);
    track.set_final_time(2.0);
    track.set_mesh_interval(0.05);

    MocoStudy study = track.initialize();
    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_optim_max_iterations(25);
    
    MocoSolution solution = study.solve();
    study.visualize(solution);
    solution.write("sandbox_tracking_contact_solution.sto");

    return EXIT_SUCCESS;
}
