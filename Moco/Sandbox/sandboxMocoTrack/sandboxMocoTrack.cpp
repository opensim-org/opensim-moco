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

#include <Moco/osimMoco.h>

#include <OpenSim/OpenSim.h>

namespace OpenSim {

class ControlTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(ControlTrackingCost, MocoCost);
public:
    ControlTrackingCost() {}
    ControlTrackingCost(std::string name, double weight)
        : MocoCost(std::move(name), weight) {}

    void setReference(const TimeSeriesTable& ref) {
        m_table = ref;
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
            integrand += pow(controls[m_controlIndices[iref]] - refValue, 2);
        }
    }

private:
    TimeSeriesTable m_table;
    mutable GCVSplineSet m_refsplines;
    mutable std::vector<int> m_controlIndices;

};

class MocoTrack : Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoTrack, Object);

public:
    OpenSim_DECLARE_PROPERTY(create_coordinate_actuators, double,
        "Create a CoordinateActuator for each unconstrained coordinate in the "
        "model, and add each to the model. Each actuator will have the "
        "specified `optimal_force`, which should be set low to discourage the "
        "use of the reserve actuators. (default is -1, which means no reserves "
        "are created)");
    OpenSim_DECLARE_PROPERTY(kinematics_file, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(markers_file, std::string, "TODO");
    // TODO check that time ranges in setup files are consistent with user
    // specified time range.
    OpenSim_DECLARE_PROPERTY(ik_setup_file, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(id_setup_file, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(remove_force_set, bool, "TODO");
    OpenSim_DECLARE_PROPERTY(start_time, double, "TODO");
    OpenSim_DECLARE_PROPERTY(end_time, double, "TODO");

    MocoTrack() {
        constructProperty_create_coordinate_actuators(-1);
        constructProperty_kinematics_file("");
        constructProperty_markers_file("");
        constructProperty_ik_setup_file("");
        constructProperty_external_loads_file("");
        constructProperty_id_setup_file("");
        constructProperty_remove_force_set(false);
        constructProperty_start_time(-1);
        constructProperty_end_time(-1);
    }

    void setModel(Model model) { m_model = std::move(model); }

    void setKinematicsFile(std::string fileName) {
        set_kinematics_file(std::move(fileName));
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
    void setIDSetupFile(std::string fileName) {
        set_id_setup_file(std::move(fileName));
    }
    void setRemoveModelForceSet(bool tf) {
        set_remove_force_set(tf);
    }
    void setStartTime(double startTime) {
        set_start_time(startTime);
    }
    void setEndTime(double endTime) {
        set_end_time(endTime);
    }

    void solve() {

        MocoTool moco;
        auto& problem = moco.updProblem();

        // Modeling.
        // ---------
        Model model(m_model);
        model.finalizeFromProperties();

        if (get_remove_force_set()) { model.updForceSet().clearAndDestroy(); }
        model.initSystem();

        if (get_create_coordinate_actuators() != -1) {
            // model.initSystem() is called within this utility.
            double optForce = get_create_coordinate_actuators();
            addCoordinateActuatorsToModel(model, 1);
            const auto coordActs = model.getComponentList<CoordinateActuator>();
            for (const auto& coordAct : coordActs) {
                problem.setControlInfo("/" + coordAct.getName(), 
                        {-optForce, optForce});
            }
        }

        // Costs.
        // ------
        m_start_time = get_start_time();
        m_end_time = get_end_time();

        // State tracking cost.
        if (!get_kinematics_file().empty()) {
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

        auto* effort = problem.addCost<MocoControlCost>("effort", 0.1);
        if (!get_external_loads_file().empty()) {
            for (int i = 0; i < 9; ++i) {
                effort->setWeight("Left_GRF_" + std::to_string(i), 0);
                effort->setWeight("Right_GRF_" + std::to_string(i), 0);

            }
        }

        problem.setModelCopy(model);
            
        // Set the time range.
        // -------------------
        // Set the time bounds based on the time range in the kinematics file.
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
        solver.set_optim_constraint_tolerance(1e-4);
        solver.set_enforce_constraint_derivatives(true);
        solver.set_transcription_scheme("hermite-simpson");
        solver.set_optim_finite_difference_scheme("forward");

        // Set the problem guess.
        // ----------------------
        auto guess = solver.createGuess("bounds");
        //std::cout << "guess num times: " << guess.getNumTimes() << std::endl;

        // Set position states in guess, if available.
        //if (m_kinematics_from_file.getNumRows()) {
        //    applyStatesToGuess(m_kinematics_from_file, model, guess);
        //} else if (m_kinematics_from_markers.getNumRows()) {
        //    applyStatesToGuess(m_kinematics_from_markers, model, guess);
        //}

        // Set ground reaction force controls in guess, if available.
        //if (m_forces.getNumRows()) {
        //    applyControlsToGuess(m_forces, guess);
        //}

        //// Set actuator controls in guess, if available.
        //if (m_controls.getNumRows()) {
        //    applyControlsToGuess(m_controls, guess);
        //}

        solver.setGuess(guess);

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
    TimeSeriesTable m_kinematics_from_file;
    TimeSeriesTable m_kinematics_from_markers;
    TimeSeriesTable m_forces;
    TimeSeriesTable m_controls;
    int m_min_data_length = -1;

    void configureStateTracking(MocoProblem& problem, Model& model) {
        auto* stateTracking = 
            problem.addCost<MocoStateTrackingCost>("state_tracking");
        auto kinematics = STOFileAdapter::read(get_kinematics_file());

        // Don't track coordinates that are already constrained.
        // TODO: it would be better to just remove the columns for constrained
        // coordinates.
        const auto& s = model.getWorkingState();
        MocoWeightSet weights;
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            std::string path = coord.getAbsolutePathString();
            double weight = coord.isConstrained(s) ? 0 : 1;
            weights.cloneAndAppend({path + "/value", weight});
        }

        stateTracking->setReference(kinematics);
        updateKinematicsLabelsAndUnits(model, kinematics);

        stateTracking->setWeightSet(weights);
        stateTracking->setAllowUnusedReferences(true);

        updateTimes(kinematics.getIndependentColumn().front(),
                kinematics.getIndependentColumn().back(), "kinematic");

        m_kinematics_from_file = kinematics;
        if (m_min_data_length == -1 || 
                m_min_data_length > kinematics.getNumRows()) {
            m_min_data_length = (int)kinematics.getNumRows();
        }
    }

    void configureMarkerTracking(MocoProblem& problem, Model& model) {
        MarkersReference markersRef;
        markersRef.loadMarkersFile(get_markers_file());

        auto* markerTracking =
            problem.addCost<MocoMarkerTrackingCost>("marking_tracking");
        markerTracking->setMarkersReference(markersRef);
        markerTracking->setAllowUnusedReferences(true);

        // If the user provided an IK setup file, get the marker weights
        // from it and append it to the MarkersReference.
        TimeSeriesTable kinematics;
        if (!get_ik_setup_file().empty()) {
            InverseKinematicsTool iktool(get_ik_setup_file());
            auto& iktasks = iktool.getIKTaskSet();
            Set<MarkerWeight> markerWeights;
            iktasks.createMarkerWeightSet(markerWeights);

            markersRef.setMarkerWeightSet(markerWeights);

            iktool.run();
            kinematics = STOFileAdapter::read(
                iktool.getOutputMotionFileName());
        }

        updateTimes(markersRef.getMarkerTable().getIndependentColumn().front(), 
            markersRef.getMarkerTable().getIndependentColumn().back(), 
            "marker");

        updateKinematicsLabelsAndUnits(model, kinematics);
        m_kinematics_from_markers = kinematics;
        if (m_min_data_length == -1 ||
                m_min_data_length > kinematics.getNumRows()) {
            m_min_data_length = (int)kinematics.getNumRows();
        }
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

                // Force application points
                std::string pointName =
                        extForce.getName() + "_" + std::to_string(j + 3);
                forces.setColumnLabel(forces.getColumnIndex(
                        extForce.getPointIdentifier() + dirs[j]), pointName);
                problem.setControlInfo("/" + extForce.getName(), j + 6,
                        {-500, 500});
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


        // TODO: getting initial guess from ID moments is trickier
        // Inverse dynamics for guess
        //if (!get_id_setup_file().empty()) {
        //    InverseDynamicsTool idtool(get_id_setup_file());
        //    idtool.run();

        //    m_controls = STOFileAdapter::read(
        //        idtool.getOutputGenForceFileName());
        //}

        //TimeSeriesTable kinematics;
        //if (m_kinematics_from_file.getNumRows()) {
        //    kinematics = m_kinematics_from_file;
        //} else if (m_kinematics_from_markers.getNumRows()) {
        //    kinematics = m_kinematics_from_markers;
        //}

        //if (kinematics.getNumRows()) {
        //    InverseDynamicsTool idtool;
        //    idtool.setExternalLoadsFileName(get_external_loads_file());
        //    idtool.setCoordinatesFileName(get_kinematics_file());
        //    //auto coordVals = convertTableToStorage(kinematics);
        //    //idtool.setCoordinateValues(coordVals);
        //    idtool.setModel(model);
        //    idtool.setOutputGenForceFileName("inverse_dynamics.sto");
        //    idtool.run();
        //}

        auto* grfTracking = 
                problem.addCost<ControlTrackingCost>("grf_tracking", 1);
        grfTracking->setReference(forces);

        updateTimes(forces.getIndependentColumn().front(),
                forces.getIndependentColumn().back(), "force");

        STOFileAdapter::write(forces, "grf_walk_new_labels.mot");

        m_forces = forces;
        if (m_min_data_length == -1 ||
                m_min_data_length > forces.getNumRows()) {
            m_min_data_length = (int)forces.getNumRows();
        }
    }

    void updateKinematicsLabelsAndUnits(const Model& model, 
            TimeSeriesTable& kinematics) {
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            std::string path = coord.getAbsolutePathString();
            for (int i = 0; i < kinematics.getNumColumns(); ++i) {
                if (path.find(kinematics.getColumnLabel(i))
                    != std::string::npos) {
                    kinematics.setColumnLabel(i, path + "/value");
                }
            }
        }

        if (kinematics.hasTableMetaDataKey("inDegrees") &&
            kinematics.getTableMetaDataAsString("inDegrees") == "yes") {
            model.getSimbodyEngine().convertDegreesToRadians(kinematics);
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

int main() {

    try {
        Model model("subject_scaled_walk.osim");
        replaceJointWithWeldJoint(model, "back");
        replaceJointWithWeldJoint(model, "subtalar_l");
        replaceJointWithWeldJoint(model, "subtalar_r");
        replaceJointWithWeldJoint(model, "mtp_l");
        replaceJointWithWeldJoint(model, "mtp_r");

        MocoTrack track;
        track.setModel(model);
        track.setKinematicsFile("coordinates.mot");
        //track.setMarkersFile("motion_capture_walk.trc");
        //track.setIKSetupFile("ik_setup_walk.xml");
        track.set_create_coordinate_actuators(1000);
        track.setRemoveModelForceSet(true);
        //track.setExternalLoadsFile("grf_walk.xml");
        //track.setIDSetupFile("id_setup_walk.xml");
        track.setStartTime(0.45);
        track.setEndTime(1.8);

        track.solve();
    }
    catch (const std::exception& e) { std::cerr << e.what() << std::endl; }


    return EXIT_SUCCESS;
}