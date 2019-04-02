#ifndef MOCO_MOCOTRACK_H
#define MOCO_MOCOTRACK_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTrack.h                                                  *
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

#include "osimMocoDLL.h"

namespace OpenSim {

//class MocoTrack : Object {
//    OpenSim_DECLARE_CONCRETE_OBJECT(MocoTrack, Object);
//
//public:
//    OpenSim_DECLARE_PROPERTY(states_file, std::string, "TODO");
//    OpenSim_DECLARE_PROPERTY(states_weights, MocoWeightSet, "TODO");
//    OpenSim_DECLARE_PROPERTY(markers_file, std::string, "TODO");
//    OpenSim_DECLARE_PROPERTY(lowpass_cutoff_frequency_for_kinematics, double,
//        "The frequency (Hz) at which to filter the kinematics. "
//        "(default is -1, which means no filtering; for walking, "
//        "consider 6 Hz).");
//
//    OpenSim_DECLARE_PROPERTY(ik_setup_file, std::string, "TODO");
//    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string, "TODO");
//    OpenSim_DECLARE_PROPERTY(external_load_weights, MocoWeightSet, "TODO");
//    OpenSim_DECLARE_PROPERTY(start_time, double, "TODO");
//    OpenSim_DECLARE_PROPERTY(end_time, double, "TODO");
//    OpenSim_DECLARE_PROPERTY(guess_type, std::string,
//        "Choices: 'bounds', 'from_data', or 'from_file' (default: 'bounds')");
//    OpenSim_DECLARE_PROPERTY(guess_file, std::string,
//        "This overrides guesses set automatically from states and/or "
//        "force data.");
//    OpenSim_DECLARE_PROPERTY(minimize_controls, double, "TODO");
//    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet, "TODO");
//
//
//
//    MocoTrack() { constructProperties(); }
//
//    MocoTool initialize();
//    void solve() const;
//
//
//private:
//    Model m_model;
//    double m_start_time;
//    double m_end_time;
//    TimeSeriesTable m_states_from_file;
//    TimeSeriesTable m_states_from_markers;
//    TimeSeriesTable m_forces;
//    int m_min_data_length;
//
//    void configureStateTracking(MocoProblem& problem, Model& model);
//    void configureMarkerTracking(MocoProblem& problem, Model& model);
//    void configureForceTracking(MocoProblem& problem, Model& model);
//    void updateCoordinatesLabelsAndUnits(const Model& model,
//        TimeSeriesTable& states);
//    void updateTimes(double dataStartTime, double dataEndTime,
//        std::string dataType);
//    void applyStatesToGuess(const TimeSeriesTable& states, const Model& model,
//        MocoIterate& guess);
//    void applyControlsToGuess(const TimeSeriesTable& table, MocoIterate& guess);
//};



} // namespace OpenSim

#endif // MOCO_MOCOINVERSE_H
