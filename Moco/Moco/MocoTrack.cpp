/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTrack.cpp                                                *
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

#include "MocoTrack.h"

using namespace OpenSim;

//void MocoTrack::constructProperties() {
//
//    constructProperty_states_file("");
//    constructProperty_state_weights(MocoWeightSet());
//    constructProperty_markers_file("");
//    constructProperty_lowpass_cutoff_frequency_for_kinematics(-1);
//
//    constructProperty_ik_setup_file("");
//    constructProperty_external_loads_file("");
//    constructProperty_external_load_weights(MocoWeightSet());
//    constructProperty_start_time(-1);
//    constructProperty_end_time(-1);
//    constructProperty_guess_type("bounds");
//    constructProperty_guess_file("");
//    constructProperty_minimize_controls(-1);
//    constructProperty_control_weights(MocoWeightSet());
//}
//
//MocoTool MocoTrack::initialize() {
//
//    MocoTool moco;
//    moco.setName("MocoTrack");
//    auto& problem = moco.updProblem();
//
//    // Modeling.
//    // ---------
//    Model model(m_model);
//    model.finalizeFromProperties();
//    model.initSystem();
//
//    // Costs.
//    // ------
//    m_start_time = get_start_time();
//    m_end_time = get_end_time();
//
//    // State tracking cost.
//    if (!get_states_file().empty()) {
//        configureStateTracking(problem, model);
//    }
//
//    // Marker tracking cost.
//    if (!get_markers_file().empty()) {
//        configureMarkerTracking(problem, model);
//    }
//
//    // GRF tracking cost.
//    if (!get_external_loads_file().empty()) {
//        configureForceTracking(problem, model);
//    }
//
//    // Set the model again, in case it was changed while configuring costs.
//    problem.setModelCopy(model);
//
//    // Control minimization.
//    // ---------------------
//    if (get_minimize_controls() != -1) {
//        auto* effort = problem.addCost<MocoControlCost>("control_effort");
//        assert(get_minimize_controls() > 0);
//        effort->set_weight(get_minimize_controls());
//
//        const auto& control_weights = get_control_weights();
//        for (int c = 0; c < control_weights.getSize(); ++c) {
//            const auto& control_weight = control_weights.get(c);
//            effort->setWeight(control_weight.getName(),
//                control_weight.getWeight());
//        }
//    } else {
//        OPENSIM_THROW_IF(get_control_weights().getSize() != 0, Exception,
//            "Control weight set provided, but control minimization was not "
//            "specified by the user.");
//    }
//
//    // Set the time range.
//    // -------------------
//    // Set the time bounds based on the time range in the states file.
//    // Pad the beginning and end time points to allow room for finite 
//    // difference calculations.
//    // TODO: filter? Handle padding with filtering?
//    problem.setTimeBounds(m_start_time + 1e-3, m_end_time - 1e-3);
//
//    // Configure solver.
//    // -----------------
//    auto& solver = moco.initCasADiSolver();
//    solver.set_num_mesh_points(25);
//    solver.set_dynamics_mode("explicit");
//    solver.set_optim_convergence_tolerance(1e-4);
//    solver.set_optim_constraint_tolerance(1e-5);
//    solver.set_enforce_constraint_derivatives(true);
//    solver.set_transcription_scheme("hermite-simpson");
//    solver.set_optim_finite_difference_scheme("forward");
//
//    // Set the problem guess.
//    // ----------------------
//    checkPropertyInSet(*this, getProperty_guess_type(),
//            {"bounds", "from_data", "from_file"});
//    OPENSIM_THROW_IF(get_guess_type() == "from_file" &&
//            get_guess_file().empty(), Exception,
//            "Guess type set to 'from_file' but no guess file was provided.");
//
//    if (get_guess_type() == "from_file") {
//        solver.setGuessFile(get_guess_file());
//    } else if (get_guess_type() == "from_data") {
//        auto guess = solver.createGuess("bounds");
//        if (m_states_from_file.getNumRows()) {
//            applyStatesToGuess(m_states_from_file, model, guess);
//        }
//        else if (m_states_from_markers.getNumRows()) {
//            applyStatesToGuess(m_states_from_markers, model, guess);
//        }
//        if (m_forces.getNumRows()) {
//            applyControlsToGuess(m_forces, guess);
//        }
//        solver.setGuess(guess);
//    }
//    else {
//        solver.setGuess("bounds");
//    }
//
//    return moco
//}
//
//void MocoTrack::solve() const {
//    MocoTool moco = initialize();
//
//    // Solve!
//    // ------
//    MocoSolution solution = moco.solve().unseal();
//    solution.write("sandboxMocoTrack_solution.sto");
//    moco.visualize(solution);
//}
//
//void MocoTrack::configureStateTracking(MocoProblem& problem, Model& model) {
//
//    // Read in the states reference data, filter, and spline.
//    auto statesRaw = STOFileAdapter::read(get_states_file());
//    if (get_lowpass_cutoff_frequency_for_kinematics() != -1) {
//        states = filterLowpass(statesRaw,
//            get_lowpass_cutoff_frequency_for_kinematics(), true);
//    } else {
//        states = statesRaw;
//    }
//    auto stateSplines = GCVSplineSet(states, states.getColumnLabels());
//
//    // Check that there are no redundant columns in the reference data.
//    auto labelsSorted = state.getColumnLabels();
//    std::sort(labelsSorted.begin(), labelsSorted.end());
//    auto it = std::adjacent_find(labelsSorted.begin(), labelsSorted.end());
//    OPENSIM_THROW_IF(it != labelsSorted.end(), Exception, 
//            "Multiple reference data provided for the same state variable.");
//
//    // Loop through all coordinates and compare labels in the reference data
//    // to coordinate variable names. 
//    auto time = states.getIndependentColumn();
//    auto labels = states.getColumnLabels();
//    int numRefStates = (int)states.getNumColumns();
//    MocoWeightSet weights;
//    MocoWeightSet user_weights = get_state_weights();
//    for (const auto& coord : model.getComponentList<Coordinate>()) {
//        std::string coordPath = coord.getAbsolutePathString();
//        std::string valueName = coordPath + "/value";
//        std::string speedName = coordPath + "/speed";
//        std::string coordName = coord.getName();
//        bool trackingValue = false;
//        bool trackingSpeed = false;
//        int valueIdx;
//        for (int i = 0; i < numRefStates; ++i) {
//            if (labels[i] == valueName) {
//                trackingValue = true;
//                valueIdx = i;
//            } else if (labels[i] == speedName) {
//                trackingSpeed = true;
//            } else if (labels[i] == coordName) {
//                // Accept reference data labeled with coordinate names only, 
//                // but set only for position data. Also, update the column label
//                // to the complete state variable path.
//                states.setColumnLabel(i, valueName);
//                trackingValue = true;
//                valueIdx = i;
//            }
//        }
//
//        // If a coordinate value was provided to track in the reference data, 
//        // but no corresponding speed, append the derivative of the coordinate
//        // value to the tracking reference.
//        // TODO: make this a user option.
//        if (trackingValue && !trackingSpeed) {
//            auto value = states.getDependentColumnAtIndex(valueIdx);
//            auto* valueSpline = stateSplines.getGCVSpline(valueIdx);
//            SimTK::Vector speed((int)time.size());
//            for (int j = 0; j < time.size(); ++j) {
//                speed[j] = valueSpline->calcDerivative({0},
//                    SimTK::Vector(1, time[j]));
//            }
//            states.appendColumn(speedName, speed);
//        }
//
//        // Handle weights.
//        bool valueWeightProvided = false;
//        bool speedWeightProvided = false;
//        for (int w = 0; w < user_weights.getSize(); ++w) {
//            const auto& user_weight = user_weights.get(w);
//            if (user_weight.getName() == valueName) {
//                weights.cloneAndAppend(user_weight);
//                valueWeightProvided = true;
//            } else if (user_weight.getName() == speedName) {
//                weights.cloneAndAppend(user_weight);
//                speedWeightProvided = true;
//            }
//        }
//
//        // Don't track states that are already constrained.
//        double weight = coord.isConstrained(model.getWorkingState()) ? 0 : 1;
//        if (!valueWeightProvided) {
//            weights.cloneAndAppend({valueName, weight});
//        }
//        if (!speedWeightProvided) {
//            weights.cloneAndAppend({speedName, weight});
//        }
//
//    }
//
//    auto* stateTracking =
//        problem.addCost<MocoStateTrackingCost>("state_tracking");
//    stateTracking->setReference(states);
//    stateTracking->setWeightSet(weights);
//    stateTracking->setAllowUnusedReferences(true);
//
//    updateTimes(states.getIndependentColumn().front(),
//        states.getIndependentColumn().back(), "state");
//
//    updateStateLabelsAndUnits(model, states);
//    STOFileAdapter::write(states, "states_new_labels.mot");
//    m_states_from_file = states;
//    if (m_min_data_length == -1 ||
//        m_min_data_length > states.getNumRows()) {
//        m_min_data_length = (int)states.getNumRows();
//    }
//}
//
//void MocoTrack::configureMarkerTracking(MocoProblem& problem, Model& model) {
//
//    MarkersReference markersRefFromFile;
//    markersRefFromFile.loadMarkersFile(get_markers_file());
//
//    auto markersTableRaw = markersRefFromFile.getMarkerTable();
//    auto markersTable = filterLowpass(markersTableRaw.flatten(), 6, true);
//
//    MarkersReference markersRef(markersTable.pack<SimTK::Vec3>());
//
//    // If the user provided an IK setup file, get the marker weights
//    // from it and append it to the MarkersReference.
//    TimeSeriesTable states;
//    if (!get_ik_setup_file().empty()) {
//        InverseKinematicsTool iktool(get_ik_setup_file());
//        iktool.setModel(model);
//        auto& iktasks = iktool.getIKTaskSet();
//        Set<MarkerWeight> markerWeights;
//        iktasks.createMarkerWeightSet(markerWeights);
//
//        markersRef.setMarkerWeightSet(markerWeights);
//
//        iktool.run();
//        states = STOFileAdapter::read(
//            iktool.getOutputMotionFileName());
//    }
//
//    auto* markerTracking =
//        problem.addCost<MocoMarkerTrackingCost>("marking_tracking");
//    markerTracking->setMarkersReference(markersRef);
//    markerTracking->setAllowUnusedReferences(true);
//
//    updateTimes(markersRef.getMarkerTable().getIndependentColumn().front(),
//        markersRef.getMarkerTable().getIndependentColumn().back(),
//        "marker");
//
//    updateStateLabelsAndUnits(model, states);
//    m_states_from_markers = states;
//    if (m_min_data_length == -1 ||
//        m_min_data_length > states.getnumrows()) {
//        m_min_data_length = (int)states.getnumrows();
//    }
//}
//
//void MocoTrack::configureForceTracking(MocoProblem& problem, Model& model) {
//    ExternalLoads extLoads(get_external_loads_file(), true);
//
//    auto forces = STOFileAdapter::read(extLoads.getDataFileName());
//    std::vector<std::string> dirs = {"x", "y", "z"};
//    int f = 0;
//    for (int i = 0; i < extLoads.getSize(); ++i) {
//        const auto& extForce = extLoads.get(i);
//        for (int j = 0; j < dirs.size(); ++j) {
//            // Torques
//            std::string torqueName =
//                extForce.getName() + "_" + std::to_string(j);
//            forces.setColumnLabel(forces.getColumnIndex(
//                extForce.getTorqueIdentifier() + dirs[j]), torqueName);
//            problem.setControlInfo("/" + extForce.getName(), j,
//            {-250, 250});
//
//            // Forces
//            std::string forceName =
//                extForce.getName() + "_" + std::to_string(j + 3);
//            forces.setColumnLabel(forces.getColumnIndex(
//                extForce.getForceIdentifier() + dirs[j]), forceName);
//            problem.setControlInfo("/" + extForce.getName(), j + 3,
//            {-1000, 1000});
//
//            // Points
//            std::string pointName =
//                extForce.getName() + "_" + std::to_string(j + 6);
//            forces.setColumnLabel(forces.getColumnIndex(
//                extForce.getPointIdentifier() + dirs[j]), pointName);
//            problem.setControlInfo("/" + extForce.getName(), j + 6,
//            {-10, 10});
//        }
//
//        FreePointBodyActuator* fpbAct = new FreePointBodyActuator();
//        fpbAct->setName(extForce.getName());
//        fpbAct->setBodyName("/bodyset/" +
//            extForce.getAppliedToBodyName());
//        fpbAct->setPointForceIsGlobal(
//            extForce.getPointExpressedInBodyName() == "ground");
//        fpbAct->setSpatialForceIsGlobal(
//            extForce.getForceExpressedInBodyName() == "ground");
//        model.addComponent(fpbAct);
//        ++f;
//    }
//
//    auto* grfTracking =
//        problem.addCost<ControlTrackingCost>("grf_tracking", 1);
//    grfTracking->setReference(forces);
//    grfTracking->setWeightSet(get_external_load_weights());
//    STOFileAdapter::write(forces, "forces_new_labels.mot");
//
//    updateTimes(forces.getIndependentColumn().front(),
//        forces.getIndependentColumn().back(), "marker");
//
//    m_forces = forces;
//    if (m_min_data_length == -1 ||
//        m_min_data_length > forces.getNumRows()) {
//        m_min_data_length = (int)forces.getNumRows();
//    }
//}
//
//void MocoTrack::updateStatesLabelsAndUnits(const Model& model,
//    TimeSeriesTable& states) {
//    for (const auto& coord : model.getComponentList<Coordinate>()) {
//        std::string path = coord.getAbsolutePathString();
//        for (int i = 0; i < states.getNumColumns(); ++i) {
//            if (path.find(states.getColumnLabel(i))
//                != std::string::npos) {
//                states.setColumnLabel(i, path + "/value");
//            }
//        }
//    }
//
//    if (states.hasTableMetaDataKey("inDegrees") &&
//        states.getTableMetaDataAsString("inDegrees") == "yes") {
//        model.getSimbodyEngine().convertDegreesToRadians(states);
//    }
//}
//
//void MocoTrack::updateTimes(double dataStartTime, double dataEndTime,
//        std::string dataType) {
//
//    if (m_start_time == -1) {
//        m_start_time = dataStartTime;
//    }
//    else if (m_start_time < dataStartTime) {
//        OPENSIM_THROW_IF(get_start_time() != -1, Exception,
//            format("Start time provided inconsisent with %s data",
//                dataType));
//        m_start_time = dataStartTime;
//    }
//    if (m_end_time == -1) {
//        m_end_time = dataEndTime;
//    }
//    else if (m_end_time > dataEndTime){
//        OPENSIM_THROW_IF(get_end_time() != -1, Exception,
//            format("End time provided inconsisent with %s data", dataType));
//        m_end_time = dataEndTime;
//    }
//}
//
//void MocoTrack::applyStatesToGuess(const TimeSeriesTable& states, 
//        const Model& model, MocoIterate& guess) {
//
//    guess.resampleWithNumTimes(m_min_data_length);
//    auto time = guess.getTime();
//    GCVSplineSet stateSplines(states);
//
//    SimTK::Vector currTime(1);
//    SimTK::Vector value(m_min_data_length);
//    SimTK::Vector speed(m_min_data_length);
//    for (const auto& coord : model.getComponentList<Coordinate>()) {
//        auto path = coord.getAbsolutePathString();
//        for (int i = 0; i < states.getNumColumns(); ++i) {
//            auto label = states.getColumnLabel(i);
//            if (path.find(label) != std::string::npos) {
//
//                for (int j = 0; j < m_min_data_length; ++i) {
//                    currTime[0] = time[j];
//                    auto* spline = stateSplines.getGCVSpline(i);
//
//                    value[j] = spline->calcValue(currTime);
//                    speed[j] = spline->calcDerivative({0}, currTime);
//                }
//
//                guess.setState(path + "/value", value);
//                guess.setState(path + "/speed", speed);
//            }
//        }
//    }
//}
//
//void MocoTrack::applyControlsToGuess(const TimeSeriesTable& table, 
//        MocoIterate& guess) {
//
//    guess.resampleWithNumTimes(m_min_data_length);
//    auto time = guess.getTime();
//    for (const auto& label : table.getColumnLabels()) {
//        auto col = table.getDependentColumn(label);
//        auto colTime = createVectorLinspace(m_min_data_length, time[0],
//            time[time.size() - 1]);
//        guess.setControl("/" + label, interpolate(colTime, col, time));
//    }
//}