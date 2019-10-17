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
    track.setModel(
            ModelProcessor("subject_walk_armless_80musc.osim") |
            ModOpAddExternalLoads("grf_walk.xml") |
            ModOpRemoveMuscles() |
            ModOpAddReserves(250));
    track.setStatesReference(TableProcessor("coordinates.mot"));

    // There is marker data in the 'marker_trajectories.trc' associated with
    // model markers that no longer exists (i.e. markers on the arms). Set this
    // flag to avoid an exception from being thrown.
    track.set_allow_unused_references(true);

    // Increase the global marker tracking weight, which is the weight
    // associated with the internal MocoMarkerTrackingGoal term.
    track.set_markers_global_tracking_weight(10);


    return EXIT_SUCCESS;

}
