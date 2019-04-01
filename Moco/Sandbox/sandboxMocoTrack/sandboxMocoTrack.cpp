/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxMocoTrack.cpp                                         *
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

#include <algorithm>
#include <Moco/osimMoco.h>

#include <OpenSim/OpenSim.h>

namespace OpenSim {

class ControlTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(ControlTrackingCost, MocoCost);
public:
    ControlTrackingCost() {}
    ControlTrackingCost(std::string name, double weight)
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
        }
        else {
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
    void initializeOnModelImpl(const Model& model) const override {
        // Convert data table to splines.
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

            double refWeight = 1.0;
            if (get_control_weights().contains(refName)) {
                refWeight = get_control_weights().get(refName).getWeight();
            }
            m_control_weights.push_back(refWeight);

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
    }

    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override {

        const auto& time = state.getTime();
        SimTK::Vector timeVec(1, time);
        
        const auto& controls = getModel().getControls(state);
        integrand = 0;
        // TODO cache the reference coordinate values at the mesh points, 
        // rather than evaluating the spline.
        for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
            const auto& refValue = m_refsplines[iref].calcValue(timeVec);
            integrand += m_control_weights[iref] * 
                         pow(controls[m_controlIndices[iref]] - refValue, 2);
        }
    }

private:
    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
        "Set of weight objects to weight the tracking of individual "
        "control variables in the cost.");

    void constructProperties() {
        constructProperty_control_weights(MocoWeightSet());
    }

    TimeSeriesTable m_table;
    mutable GCVSplineSet m_refsplines;
    mutable std::vector<int> m_controlIndices;
    mutable std::vector<double> m_control_weights;

};


class TransformTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(TransformTrackingCost, MocoCost);
public:
    TransformTrackingCost() {}
    TransformTrackingCost(std::string name, double weight)
        : MocoCost(std::move(name), weight) {}

    void setStatesTrajectory(const StatesTrajectory& statesTraj) {
        m_statesTraj = statesTraj;
    }
    void setComponentPaths(const std::vector<std::string>& compPaths) {
        m_compPaths = compPaths;
    }
    void setTrackedComponents(const std::string& trackedComps) {
        m_trackedComps = trackedComps;
    }

protected:
    void initializeOnModelImpl(const Model& model) const override {

        OPENSIM_THROW_IF(!(m_trackedComps == "all" ||
                           m_trackedComps == "rotation" || 
                           m_trackedComps == "position"), Exception,
            format("Tracked component %s not recognized.", m_trackedComps));

        auto table = getTransformTrajectories(model, m_statesTraj, 
                m_compPaths);
        m_refsplines = GCVSplineSet(table);

        for (int i = 0; i < m_compPaths.size(); ++i) {
            const auto& compPath = m_compPaths[i];
            const auto& frame = model.getComponent<Frame>(compPath);
            m_model_frames.emplace_back(&frame);
            m_refindices.push_back(i);
        }

    }
    void calcIntegralCostImpl(const SimTK::State& state,
        double& integrand) const override {
        const auto& time = state.getTime();
        getModel().realizePosition(state);
        SimTK::Vector timeVec(1, time);

        for (int f = 0; f < (int)m_model_frames.size(); ++f) {
            const auto& transform = 
                m_model_frames[f]->getTransformInGround(state);

            SimTK::Vector error(4, 0.0);
            int refIdx = m_refindices[f];

            // Rotation errors.
            if (m_trackedComps == "all" || m_trackedComps == "rotation") {
                const auto& R_GD = transform.R(); 
                const SimTK::Quaternion e(
                    m_refsplines[7*refIdx].calcValue(timeVec),
                    m_refsplines[7*refIdx + 1].calcValue(timeVec),
                    m_refsplines[7*refIdx + 2].calcValue(timeVec),
                    m_refsplines[7*refIdx + 3].calcValue(timeVec));
                const SimTK::Rotation R_GM(e);
                const SimTK::Rotation R_DM = ~R_GD*R_GM;
                const SimTK::Vec4 aa_DM = R_DM.convertRotationToAngleAxis();
                error[0] = aa_DM[0];
            }

            // Position errors.
            if (m_trackedComps == "all" || m_trackedComps == "position") {
                const auto& p = transform.p();
                for (int i = 0; i < 3; ++i) {
                    error[i+1] = (p(i) - 
                        m_refsplines[7*refIdx + 4+i].calcValue(timeVec)) / 
                        p.norm();
                }
            }

            // Add this components transform error to the cost.
            integrand += error.normSqr();
        }
    }


private:
    StatesTrajectory m_statesTraj;
    std::vector<std::string> m_compPaths;
    std::string m_trackedComps = "all";
    mutable std::vector<int> m_refindices;
    mutable GCVSplineSet m_refsplines;
    mutable std::vector<SimTK::ReferencePtr<const Frame>> m_model_frames;

    TimeSeriesTable getTransformTrajectories(Model model,
            const StatesTrajectory& statesTraj, 
            std::vector<std::string> compPaths) const {

        // Create independent time vector and construct table.
        std::vector<double> indCol;
        for (const auto& state : statesTraj) {
            indCol.push_back((double)state.getTime());
        }
        TimeSeriesTable table(indCol);

        // Append columns.
        model.initSystem();
        SimTK::Matrix mat((int)indCol.size(), 7*(int)compPaths.size());
        std::vector<std::string> colLabels;
        for (int row = 0; row < statesTraj.getSize(); ++row) {
            auto state = statesTraj.get(row);
            model.getSystem().prescribe(state);
            model.realizePosition(state);

            int col = 0;
            for (const auto& compPath : compPaths) {
                SimTK::Transform transform =
                    model.getComponent(compPath)
                    .getOutputValue<SimTK::Transform>(state, "transform");

                // Rotations.
                const auto& R = transform.R();
                auto e = R.convertRotationToQuaternion();
                for (int i = 0; i < 4; ++i) {
                    mat.updElt(row, col++) = e[i];
                    if (!row) {
                        colLabels.push_back(format("%s/transform_e%i",
                            compPath, i + 1));
                    }
                }
                // Position vector.
                const auto& p = transform.p();
                for (int i = 0; i < 3; ++i) {
                    mat.updElt(row, col++) = p(i);
                    if (!row) {
                        colLabels.push_back(format("%s/transform_p%i", compPath,
                            i + 1));
                    }
                }
            }
        }

        table.updMatrix() = mat;
        table.setColumnLabels(colLabels);

        return table;
    }
};



class MocoTrack : Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoTrack, Object);

public:
    OpenSim_DECLARE_PROPERTY(coordinates_file, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(coordinate_weights, MocoWeightSet, "TODO");
    OpenSim_DECLARE_PROPERTY(markers_file, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(ik_setup_file, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(external_load_weights, MocoWeightSet, "TODO");
    OpenSim_DECLARE_PROPERTY(start_time, double, "TODO");
    OpenSim_DECLARE_PROPERTY(end_time, double, "TODO");
    OpenSim_DECLARE_PROPERTY(guess_type, std::string, 
        "Choices: 'bounds', 'from_data', or 'from_file' (default: 'bounds')");
    OpenSim_DECLARE_PROPERTY(guess_file, std::string, 
        "This overrides guesses set automatically from coordinates and/or "
        "force data.");
    OpenSim_DECLARE_PROPERTY(minimize_controls, double, "TODO");
    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet, "TODO"); 

    MocoTrack() {
        constructProperty_coordinates_file("");
        constructProperty_coordinate_weights(MocoWeightSet());
        constructProperty_markers_file("");
        constructProperty_ik_setup_file("");
        constructProperty_external_loads_file("");
        constructProperty_external_load_weights(MocoWeightSet());
        constructProperty_start_time(-1);
        constructProperty_end_time(-1);
        constructProperty_guess_type("bounds");
        constructProperty_guess_file("");
        constructProperty_minimize_controls(-1);
        constructProperty_control_weights(MocoWeightSet());
    }

    void setModel(Model model) { m_model = std::move(model); }

    void setCoordinatesFile(std::string fileName) {
        set_coordinates_file(std::move(fileName));
    }
    void setCoordinatesWeightSet(MocoWeightSet weights) {
        set_coordinate_weights(std::move(weights));
    }
    void setMarkersFile(std::string fileName) {
        set_markers_file(std::move(fileName));
    }
    void setIKSetupFile(std::string fileName) {
        set_ik_setup_file(std::move(fileName));
    }
    void setExternalLoadsFile(std::string fileName) {
        set_external_loads_file(std::move(fileName));
    }
    void setExternalLoadWeights(MocoWeightSet weights) {
        set_external_load_weights(std::move(weights));
    }
    void setStartTime(double startTime) {
        set_start_time(startTime);
    }
    void setEndTime(double endTime) {
        set_end_time(endTime);
    }
    void setGuessType(std::string type) {
        set_guess_type(std::move(type));
    }
    void setGuessFile(std::string fileName) {
        set_guess_file(std::move(fileName));
    }
    void setMinimizeControls(double weight) {
        set_minimize_controls(weight);
    }
    void setControlWeights(MocoWeightSet weights) {
        set_control_weights(std::move(weights));
    }

    MocoTool initialize() {

        MocoTool moco;
        moco.setName("MocoTrack");
        auto& problem = moco.updProblem();

        // Modeling.
        // ---------
        Model model(m_model);
        model.finalizeFromProperties();
        model.initSystem();

        // Costs.
        // ------
        m_start_time = get_start_time();
        m_end_time = get_end_time();

        // State tracking cost.
        if (!get_coordinates_file().empty()) {
            configureStateTracking(problem, model);
        }

        // Marker tracking cost.
        if (!get_markers_file().empty()) {
            configureMarkerTracking(problem, model);
        }

        // GRF tracking cost.
        if (!get_external_loads_file().empty()) {
            configureForceTracking(problem, model);
        }

        // Set the model again, in case it was changed while configuring costs.
        problem.setModelCopy(model);
        
        // Control minimization.
        // ---------------------
        if (get_minimize_controls() != -1) {
            auto* effort = problem.addCost<MocoControlCost>("control_effort");
            assert(get_minimize_controls() > 0);
            effort->set_weight(get_minimize_controls());

            const auto& control_weights = get_control_weights();
            for (int c = 0; c < control_weights.getSize(); ++c) {
                const auto& control_weight = control_weights.get(c);
                effort->setWeight(control_weight.getName(), 
                    control_weight.getWeight());
            }
        } else {
            OPENSIM_THROW_IF(get_control_weights().getSize() != 0, Exception,
                "Control weight set provided, but control minimization was not "
                "specified by the user.");
        }
                    
        // Set the time range.
        // -------------------
        // Set the time bounds based on the time range in the coordinates file.
        // Pad the beginning and end time points to allow room for finite 
        // difference calculations.
        // TODO: filter? Handle padding with filtering?
        problem.setTimeBounds(m_start_time + 1e-3, m_end_time - 1e-3);

        // Configure solver.
        // -----------------
        auto& solver = moco.initCasADiSolver();
        solver.set_num_mesh_points(25);
        solver.set_dynamics_mode("explicit");
        solver.set_optim_convergence_tolerance(1e-4);
        solver.set_optim_constraint_tolerance(1e-5);
        solver.set_enforce_constraint_derivatives(true);
        solver.set_transcription_scheme("hermite-simpson");
        solver.set_optim_finite_difference_scheme("forward");

        // Set the problem guess.
        // ----------------------
        checkPropertyInSet(*this, getProperty_guess_type(),
            {"bounds", "from_data", "from_file"});
        OPENSIM_THROW_IF(get_guess_type() == "from_file" && 
                         get_guess_file().empty(), Exception, 
            "Guess type set to 'from_file' but no guess file was provided.");

        if (get_guess_type() == "from_file") {
            solver.setGuessFile(get_guess_file());
        } else if (get_guess_type() == "from_data") {
            auto guess = solver.createGuess("bounds");
            if (m_coordinates_from_file.getNumRows()) {
                applyStatesToGuess(m_coordinates_from_file, model, guess);
            } else if (m_coordinates_from_markers.getNumRows()) {
                applyStatesToGuess(m_coordinates_from_markers, model, guess);
            }
            if (m_forces.getNumRows()) {
                applyControlsToGuess(m_forces, guess);
            }
            solver.setGuess(guess);
        } else {
            solver.setGuess("bounds");
        }

        return moco;
    }

    void solve() {
        MocoTool moco = initialize();

        // Solve!
        // ------
        MocoSolution solution = moco.solve().unseal();
        solution.write("sandboxMocoTrack_solution.sto");
        moco.visualize(solution);
    }

private:
    Model m_model;
    double m_start_time;
    double m_end_time;
    TimeSeriesTable m_coordinates_from_file;
    TimeSeriesTable m_coordinates_from_markers;
    TimeSeriesTable m_forces;
    int m_min_data_length;

    void configureStateTracking(MocoProblem& problem, Model& model) {
        
        auto coordinatesRaw = STOFileAdapter::read(get_coordinates_file());
        auto coordinates = filterLowpass(coordinatesRaw, 6, true);

        auto time = coordinates.getIndependentColumn();
        auto coordinateSplines = GCVSplineSet(coordinates,
                coordinates.getColumnLabels());
        auto labels = coordinates.getColumnLabels();

        int numCols = coordinates.getNumColumns();

        const auto& s = model.getWorkingState();
        MocoWeightSet weights;
        MocoWeightSet user_weights = get_coordinate_weights();
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            std::string path = coord.getAbsolutePathString();
            for (int col = 0; col < numCols; ++col) {
                if (path.find(labels[col]) != std::string::npos ||
                        labels[col] == (path + "/value")) {

                    std::cout << "labels[i]: " << labels[col] << std::endl;

                    coordinates.setColumnLabel(col, path + "/value");

                    bool speedStateInReference = false;
                    if (std::find(labels.begin(), labels.end(), 
                            path + "/speed") != labels.end()) {
                        speedStateInReference = true;
                    }
                            
                    // If speed not in reference, use derivative of value for
                    // the tracking reference instead.
                    if (!speedStateInReference && !coord.isConstrained(s)) {
                        std::cout << "path: " << path << std::endl;
                        std::cout << "index: " << col << std::endl;
                        auto value = coordinates.getDependentColumnAtIndex(col);
                        auto* valueSpline = coordinateSplines.getGCVSpline(col);
                        SimTK::Vector speed((int)time.size());
                        for (int j = 0; j < time.size(); ++j) {
                            speed[j] = valueSpline->calcDerivative({0}, 
                                SimTK::Vector(1, time[j]));
                        }
                        coordinates.appendColumn(path + "/speed", speed);
                    }
                }
                
                bool valueWeightProvided = false;
                bool speedWeightProvided = false;
                for (int w = 0; w < user_weights.getSize(); ++w) {
                    const auto& user_weight = user_weights.get(w);
                    if (user_weight.getName() == (path + "/value")) {
                        weights.cloneAndAppend(user_weight);
                        valueWeightProvided = true;
                    } else if(user_weight.getName() == (path + "/speed")) {
                        weights.cloneAndAppend(user_weight);
                        speedWeightProvided = true;
                    } else {
                        OPENSIM_THROW(Exception, format("Provided coordinate "
                            "weight with name %s does not match any states in "
                            "the current model."));
                    }
                }

                // Don't track coordinates that are already constrained.
                double weight = coord.isConstrained(s) ? 0 : 1;
                if (!valueWeightProvided) {
                    weights.cloneAndAppend({path + "/value", weight});
                }
                if (!speedWeightProvided) {
                    weights.cloneAndAppend({path + "/speed", weight});
                }

            }
        }

        auto* stateTracking =
                problem.addCost<MocoStateTrackingCost>("state_tracking");
        stateTracking->setReference(coordinates);
        stateTracking->setWeightSet(weights);
        stateTracking->setAllowUnusedReferences(true);

        updateTimes(coordinates.getIndependentColumn().front(),
                    coordinates.getIndependentColumn().back(), "coordinate");

        updateCoordinatesLabelsAndUnits(model, coordinates);
        STOFileAdapter::write(coordinates, "coordinates_new_labels.mot");
        m_coordinates_from_file = coordinates;
        if (m_min_data_length == -1 ||
            m_min_data_length > coordinates.getNumRows()) {
            m_min_data_length = (int)coordinates.getNumRows();
        }
    }

    void configureMarkerTracking(MocoProblem& problem, Model& model) {
        MarkersReference markersRefFromFile;
        markersRefFromFile.loadMarkersFile(get_markers_file());

        auto markersTableRaw = markersRefFromFile.getMarkerTable();
        auto markersTable = filterLowpass(markersTableRaw.flatten(), 6, true);

        MarkersReference markersRef(markersTable.pack<SimTK::Vec3>());

        // If the user provided an IK setup file, get the marker weights
        // from it and append it to the MarkersReference.
        TimeSeriesTable coordinates;
        if (!get_ik_setup_file().empty()) {
            InverseKinematicsTool iktool(get_ik_setup_file());
            iktool.setModel(model);
            auto& iktasks = iktool.getIKTaskSet();
            Set<MarkerWeight> markerWeights;
            iktasks.createMarkerWeightSet(markerWeights);

            markersRef.setMarkerWeightSet(markerWeights);

            // iktool.run();
            //coordinates = STOFileAdapter::read(
            //    iktool.getOutputMotionFileName());
        }

        auto* markerTracking =
            problem.addCost<MocoMarkerTrackingCost>("marking_tracking");
        markerTracking->setMarkersReference(markersRef);
        markerTracking->setAllowUnusedReferences(true);

        updateTimes(markersRef.getMarkerTable().getIndependentColumn().front(), 
                    markersRef.getMarkerTable().getIndependentColumn().back(), 
                    "marker");

        //updateCoordinatesLabelsAndUnits(model, coordinates);
        //m_coordinates_from_markers = coordinates;
        //if (m_min_data_length == -1 ||
        //    m_min_data_length > coordinates.getNumRows()) {
        //    m_min_data_length = (int)coordinates.getNumRows();
        //}
    }

    void configureForceTracking(MocoProblem& problem, Model& model) {
        ExternalLoads extLoads(get_external_loads_file(), true);

        auto forces = STOFileAdapter::read(extLoads.getDataFileName());
        std::vector<std::string> dirs = {"x", "y", "z"};
        int f = 0;
        for (int i = 0; i < extLoads.getSize(); ++i) {
            const auto& extForce = extLoads.get(i);
            for (int j = 0; j < dirs.size(); ++j) {
                // Torques
                std::string torqueName =
                    extForce.getName() + "_" + std::to_string(j);
                forces.setColumnLabel(forces.getColumnIndex(
                        extForce.getTorqueIdentifier() + dirs[j]), torqueName);
                problem.setControlInfo("/" + extForce.getName(), j,
                        {-250, 250});

                // Forces
                std::string forceName =
                        extForce.getName() + "_" + std::to_string(j + 3);
                forces.setColumnLabel(forces.getColumnIndex(
                        extForce.getForceIdentifier() + dirs[j]), forceName);
                problem.setControlInfo("/" + extForce.getName(), j + 3,
                        {-1000, 1000});

                // Points
                std::string pointName =
                        extForce.getName() + "_" + std::to_string(j + 6);
                forces.setColumnLabel(forces.getColumnIndex(
                        extForce.getPointIdentifier() + dirs[j]), pointName);
                problem.setControlInfo("/" + extForce.getName(), j + 6,
                        {-10, 10});
            }

            FreePointBodyActuator* fpbAct = new FreePointBodyActuator();
            fpbAct->setName(extForce.getName());
            fpbAct->setBodyName("/bodyset/" +
                extForce.getAppliedToBodyName());
            fpbAct->setPointForceIsGlobal(
                extForce.getPointExpressedInBodyName() == "ground");
            fpbAct->setSpatialForceIsGlobal(
                extForce.getForceExpressedInBodyName() == "ground");
            model.addComponent(fpbAct);
            ++f;
        }

        auto* grfTracking =
            problem.addCost<ControlTrackingCost>("grf_tracking", 1);
        grfTracking->setReference(forces);
        grfTracking->setWeightSet(get_external_load_weights());
        STOFileAdapter::write(forces, "forces_new_labels.mot");

        updateTimes(forces.getIndependentColumn().front(),
                    forces.getIndependentColumn().back(), "marker");

        m_forces = forces;
        if (m_min_data_length == -1 ||
            m_min_data_length > forces.getNumRows()) {
            m_min_data_length = (int)forces.getNumRows();
        }
    }

    void updateCoordinatesLabelsAndUnits(const Model& model,
            TimeSeriesTable& coordinates) {
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            std::string path = coord.getAbsolutePathString();
            for (int i = 0; i < coordinates.getNumColumns(); ++i) {
                if (path.find(coordinates.getColumnLabel(i))
                    != std::string::npos) {
                    coordinates.setColumnLabel(i, path + "/value");
                }
            }
        }

        if (coordinates.hasTableMetaDataKey("inDegrees") &&
            coordinates.getTableMetaDataAsString("inDegrees") == "yes") {
            model.getSimbodyEngine().convertDegreesToRadians(coordinates);
        }
    }

    void updateTimes(double dataStartTime, double dataEndTime, 
            std::string dataType) {
        
        if (m_start_time == -1) {
            m_start_time = dataStartTime;
        } else if (m_start_time < dataStartTime) {
            OPENSIM_THROW_IF(get_start_time() != -1, Exception,
                format("Start time provided inconsisent with %s data", 
                    dataType));
            m_start_time = dataStartTime;
        }
        if (m_end_time == -1) {
            m_end_time = dataEndTime;
        } else if (m_end_time > dataEndTime){
            OPENSIM_THROW_IF(get_end_time() != -1, Exception,
                format("End time provided inconsisent with %s data", dataType));
            m_end_time = dataEndTime;
        }
    }

    void applyStatesToGuess(const TimeSeriesTable& states, const Model& model,
            MocoIterate& guess) {
        guess.resampleWithNumTimes(m_min_data_length);
        auto time = guess.getTime();
        GCVSplineSet stateSplines(states);

        SimTK::Vector currTime(1);
        SimTK::Vector value(m_min_data_length);
        SimTK::Vector speed(m_min_data_length);
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            auto path = coord.getAbsolutePathString();
            for (int i = 0; i < states.getNumColumns(); ++i) {
                auto label = states.getColumnLabel(i);
                if (path.find(label) != std::string::npos) {

                    for (int j = 0; j < m_min_data_length; ++i) {
                        currTime[0] = time[j];
                        auto* spline = stateSplines.getGCVSpline(i);

                        value[j] = spline->calcValue(currTime);
                        speed[j] = spline->calcDerivative({0}, currTime);
                    }

                    guess.setState(path + "/value", value);
                    guess.setState(path + "/speed", speed);
                }
            }
        }
    }

    void applyControlsToGuess(const TimeSeriesTable& table, MocoIterate& guess)
    {
        guess.resampleWithNumTimes(m_min_data_length);
        auto time = guess.getTime();
        for (const auto& label : table.getColumnLabels()) {
            auto col = table.getDependentColumn(label);
            auto colTime = createVectorLinspace(m_min_data_length, time[0],
                time[time.size() - 1]);
            guess.setControl("/" + label, interpolate(colTime, col, time));
        }
    }

}; // class MocoTrack

} // namespace OpenSim

using namespace OpenSim;

void addCoordinateActuator(Model& model, std::string coordName, 
        double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new CoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    actu->setMinControl(-1);
    actu->setMaxControl(1);
    model.addComponent(actu);
}

void addActivationCoordinateActuator(Model& model, std::string coordName,
    double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new ActivationCoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    actu->setMinControl(-1);
    actu->setMaxControl(1);
    actu->set_default_activation(0.0);
    actu->set_activation_time_constant(0.005);
    model.addComponent(actu);
}

int main() {

    Model model("subject_walk_rra_adjusted.osim");
    model.updForceSet().clearAndDestroy();
    model.initSystem();
    
    // weld joints w/ locked coordinates
    replaceJointWithWeldJoint(model, "subtalar_l");
    replaceJointWithWeldJoint(model, "subtalar_r");
    replaceJointWithWeldJoint(model, "mtp_l");
    replaceJointWithWeldJoint(model, "mtp_r");
    replaceJointWithWeldJoint(model, "radius_hand_l");
    replaceJointWithWeldJoint(model, "radius_hand_r");
    // lower body
    addCoordinateActuator(model, "pelvis_tilt", 10);
    addCoordinateActuator(model, "pelvis_list", 10);
    addCoordinateActuator(model, "pelvis_rotation", 10);
    addCoordinateActuator(model, "pelvis_tx", 10);
    addCoordinateActuator(model, "pelvis_ty", 10);
    addCoordinateActuator(model, "pelvis_tz", 10);
    addCoordinateActuator(model, "hip_adduction_l", 100);
    addCoordinateActuator(model, "hip_adduction_r", 100);
    addCoordinateActuator(model, "hip_flexion_l", 100);
    addCoordinateActuator(model, "hip_flexion_r", 100);
    addCoordinateActuator(model, "hip_rotation_l", 25);
    addCoordinateActuator(model, "hip_rotation_r", 25);
    addCoordinateActuator(model, "knee_angle_l", 100);
    addCoordinateActuator(model, "knee_angle_r", 100);
    addCoordinateActuator(model, "ankle_angle_l", 250);
    addCoordinateActuator(model, "ankle_angle_r", 250);
    // upper body
    addCoordinateActuator(model, "elbow_flex_l", 5);
    addCoordinateActuator(model, "elbow_flex_r", 5);
    addCoordinateActuator(model, "pro_sup_l", 0.1);
    addCoordinateActuator(model, "pro_sup_r", 0.1);
    addCoordinateActuator(model, "arm_add_l", 5);
    addCoordinateActuator(model, "arm_add_r", 5);
    addCoordinateActuator(model, "arm_rot_l", 5);
    addCoordinateActuator(model, "arm_rot_r", 5);
    addCoordinateActuator(model, "arm_flex_l", 10);
    addCoordinateActuator(model, "arm_flex_r", 10);
    addCoordinateActuator(model, "lumbar_bending", 50);
    addCoordinateActuator(model, "lumbar_extension", 50);
    addCoordinateActuator(model, "lumbar_rotation", 50);  

    model.print("subject_walk_rra_adjusted_updated.osim");

    // Baseline tracking problem.
    // --------------------------
    MocoTrack track;
    track.setModel(model);
    track.setCoordinatesFile("coordinates_rra_adjusted.sto");
    track.setMarkersFile("motion_capture_walk.trc");
    track.setIKSetupFile("ik_setup_walk_feet_only.xml");
    track.setExternalLoadsFile("grf_walk.xml");
    track.setGuessType("from_file");
    track.setGuessFile("sandboxMocoTrack_solution_trackingFeetMarkers.sto");
    track.setStartTime(0.75);
    track.setEndTime(1.79);

    // Control weights.
    MocoWeightSet controlWeights;
    // Penalize pelvis residuals.
    controlWeights.cloneAndAppend({"tau_pelvis_tilt", 10});
    controlWeights.cloneAndAppend({"tau_pelvis_list", 10});
    controlWeights.cloneAndAppend({"tau_pelvis_rotation", 10});
    controlWeights.cloneAndAppend({"tau_pelvis_tx", 10});
    controlWeights.cloneAndAppend({"tau_pelvis_ty", 10});
    controlWeights.cloneAndAppend({"tau_pelvis_tz", 10});

    // Don't penalize GRF controls.
    for (int i = 0; i < 9; ++i) {
        controlWeights.cloneAndAppend({"Left_GRF_" + std::to_string(i), 0});
        controlWeights.cloneAndAppend({"Right_GRF_" + std::to_string(i), 0});
    }
    track.setMinimizeControls(0.01);
    track.setControlWeights(controlWeights);

    MocoTool moco = track.initialize();
    MocoSolution solution = moco.solve().unseal();
    MocoProblem problem = moco.getProblem();
    //solution.write("sandboxMocoTrack_solution_trackingFeetMarkers.sto");
    //moco.visualize(solution);

    // Medial thrust gait.
    // -------------------
    MocoSolution solutionPrev("sandboxMocoTrack_solution_trackingFeetMarkers.sto");

    MocoTrack trackMTG;
    track.setModel(model);
    track.setCoordinatesFile("sandboxMocoTrack_solution_trackingFeetMarkers.sto");
    // Track 
    MocoWeightSet coordinateWeights;
    for (const auto& coordName : solution.getStateNames()) {
        if (coordName.find("pelvis") != std::string::npos) {
            if (coordName == "pelvis_tx" || coordName == "pelvis_tz") {
                coordinateWeights.cloneAndAppend({coordName, 1000});
            } else {
                // From Fregly et al. 2007: unlock superior/inferior translation
                // and all pelvis rotations.
                coordinateWeights.cloneAndAppend({coordName, 0});
            }
        } else if (coordName.find("lumbar") != std::string::npos) {
            // From Fregly et al. 2007: unlock all back rotations.
            coordinateWeights.cloneAndAppend({coordName, 0});
        } else if (coordName.find("hip") != std::string::npos ||
                   coordName.find("knee") != std::string::npos ||
                   coordName.find("ankle") != std::string::npos) {
            // From Fregly et al. 2007: unlock all hip, knee, and ankle 
            // rotations.
            coordinateWeights.cloneAndAppend({coordName, 0});
        } else {
            coordinateWeights.cloneAndAppend({coordName, 1000});
        }
    }
    track.setCoordinatesWeightSet(coordinateWeights);
    track.setExternalLoadsFile("grf_walk.xml");
    // Set the tracking weights for the GRFs. Keep the force and torque weights
    // high, but loosen the COP weights.
    MocoWeightSet extLoadWeights;
    for (int i = 0; i < 9; ++i) {
        double w;
        if (i <= 5) { 
            w = 1000;
        } else {
            w = 0.5; // w3 from Fregly et al. 2007
        }
        extLoadWeights.cloneAndAppend({"Left_GRF_" + std::to_string(i), w});
        extLoadWeights.cloneAndAppend({"Right_GRF_" + std::to_string(i), w});
    }
    track.setExternalLoadWeights(extLoadWeights);
    track.setGuessType("from_file");
    track.setGuessFile("sandboxMocoTrack_solution_trackingFeetMarkers.sto");
    track.setStartTime(0.75);
    track.setEndTime(1.79);
    MocoTool mocoMTG = track.initialize();
    MocoProblem problemMTG = mocoMTG.updProblem();

    // Control tracking cost.
    auto controlNames = solution.getControlNames();
    // Construct controls reference.
    auto time = solution.getTime();
    std::vector<double> timeVec;
    for (int i = 0; i < time.size(); ++i) {
        timeVec.push_back(time[i]);
    }
    TimeSeriesTable controlsRef(timeVec);
    // Control weights.
    MocoWeightSet controlTrackingWeights;
    for (const auto& controlName : controlNames) {
        if (controlName.find("pelvis") != std::string::npos) {
            // w6 from Fregly et al. 2007
            controlTrackingWeights.cloneAndAppend({controlName, 10});
            controlsRef.appendColumn(controlName, 
                    solution.getControl(controlName));
        } else if (controlName.find("hip") != std::string::npos ||
                   controlName.find("knee") != std::string::npos ||
                   controlName.find("ankle") != std::string::npos) {
            // w2 from Fregly et al. 2007
            controlTrackingWeights.cloneAndAppend({controlName, 0.5});
            controlsRef.appendColumn(controlName, 
                    solution.getControl(controlName));
        }
    }
    auto* controlTracking =
            problemMTG.addCost<ControlTrackingCost>("control_tracking", 1);
    controlTracking->setReference(controlsRef);
    controlTracking->setWeightSet(controlTrackingWeights);

    // Feet transform tracking cost.
    auto statesTraj = solutionPrev.exportToStatesTrajectory(problem);
    // w4 from Fregly et al. 2007
    auto* footTransformTracking =
            problemMTG.addCost<TransformTrackingCost>("foot_tracking", 10);
    footTransformTracking->setStatesTrajectory(statesTraj);
    footTransformTracking->setComponentPaths({"/bodyset/calcn_r", 
            "/bodyset/calcn_l"});
    footTransformTracking->setTrackedComponents("all");

    // Torso transform tracking cost.
    // w5 from Fregly et al. 2007
    auto* torsoTransformTracking =
        problemMTG.addCost<TransformTrackingCost>("torso_tracking", 10);
    torsoTransformTracking->setStatesTrajectory(statesTraj);
    torsoTransformTracking->setComponentPaths({"/bodyset/torso"});
    torsoTransformTracking->setTrackedComponents("rotation");

    // Knee adduction cost.
    // w1 from Fregly et al. 2007
    auto* kneeAdductionCost_l = 
        problemMTG.addCost<MocoJointReactionCost>("knee_adduction_cost_l", 1.5);
    kneeAdductionCost_l->setJointPath("/jointset/walker_knee_l");
    kneeAdductionCost_l->setReactionComponent(2);
    auto* kneeAdductionCost_r =
        problemMTG.addCost<MocoJointReactionCost>("knee_adduction_cost_r", 1.5);
    kneeAdductionCost_r->setJointPath("/jointset/walker_knee_r");
    kneeAdductionCost_r->setReactionComponent(2);

    return EXIT_SUCCESS;
}