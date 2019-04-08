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
#include <OpenSim/Common/LogManager.h>

#include <OpenSim/OpenSim.h>

namespace OpenSim {

//class ControlTrackingCost : public MocoCost {
//    OpenSim_DECLARE_CONCRETE_OBJECT(ControlTrackingCost, MocoCost);
//public:
//    ControlTrackingCost() {}
//    ControlTrackingCost(std::string name, double weight)
//            : MocoCost(std::move(name), weight) {
//        constructProperties();
//    }
//
//    void setReference(const TimeSeriesTable& ref) {
//        m_table = ref;
//    }
//    /// Set the weight for an individual control variable. If a weight is
//    /// already set for the requested control, then the provided weight
//    /// replaces the previous weight. An exception is thrown if a weight
//    /// for an unknown state is provided.
//    void setWeight(const std::string& controlName, const double& weight) {
//        if (get_control_weights().contains(controlName)) {
//            upd_control_weights().get(controlName).setWeight(weight);
//        }
//        else {
//            upd_control_weights().cloneAndAppend({controlName, weight});
//        }
//    }
//    /// Provide a MocoWeightSet to weight the control variables in the cost.
//    /// Replaces the weight set if it already exists.
//    void setWeightSet(const MocoWeightSet& weightSet) {
//        for (int w = 0; w < weightSet.getSize(); ++w) {
//            const auto& weight = weightSet[w];
//            setWeight(weight.getName(), weight.getWeight());
//        }
//    }
//
//protected:
//    void initializeOnModelImpl(const Model& model) const override {
//        // Convert data table to splines.
//        auto controlsFiltered = filterLowpass(m_table, 6, true);
//
//        auto allSplines = GCVSplineSet(controlsFiltered);
//
//        // Get all expected control names.
//        std::vector<std::string> controlNames;
//        const auto modelPath = model.getAbsolutePath();
//        for (const auto& actu : model.getComponentList<Actuator>()) {
//            std::string actuPath =
//                actu.getAbsolutePath().formRelativePath(modelPath).toString();
//            if (actu.numControls() == 1) {
//                controlNames.push_back(actuPath);
//            } else {
//                for (int i = 0; i < actu.numControls(); ++i) {
//                    controlNames.push_back(actuPath + "_" + std::to_string(i));
//                }
//            }
//        }
//
//        // TODO this assumes controls are in the same order as actuators.
//        // The loop that processes weights (two down) assumes that controls are 
//        // in the same order as actuators. However, the control indices are 
//        // allocated in the order in which addToSystem() is invoked (not 
//        // necessarily the order used by getComponentList()). So until we can be 
//        // absolutely sure that the controls are in the same order as actuators, 
//        // we run the following check: in order, set an actuator's control 
//        // signal(s) to NaN and ensure the i-th control is NaN.
//        {
//            const SimTK::State state = model.getWorkingState();
//            int i = 0;
//            auto modelControls = model.updControls(state);
//            for (const auto& actu : model.getComponentList<Actuator>()) {
//                int nc = actu.numControls();
//                SimTK::Vector origControls(nc);
//                SimTK::Vector nan(nc, SimTK::NaN);
//                actu.getControls(modelControls, origControls);
//                actu.setControls(nan, modelControls);
//                for (int j = 0; j < nc; ++j) {
//                    OPENSIM_THROW_IF_FRMOBJ(!SimTK::isNaN(modelControls[i]),
//                        Exception, "Internal error: actuators are not in the "
//                        "expected order. Submit a bug report.");
//                    ++i;
//                }
//                actu.setControls(origControls, modelControls);
//            }
//        }
//
//        for (int iref = 0; iref < allSplines.getSize(); ++iref) {
//            const auto& refName = allSplines[iref].getName();
//
//            double refWeight = 1.0;
//            if (get_control_weights().contains(refName)) {
//                refWeight = get_control_weights().get(refName).getWeight();
//            }
//            m_control_weights.push_back(refWeight);
//
//            int i = 0;
//            for (const auto& actu : model.getComponentList<Actuator>()) {
//                std::string actuPath =
//                    actu.getAbsolutePath().formRelativePath(modelPath).toString();
//                if (actu.numControls() == 1) {
//                    if (refName == actuPath) {
//                        m_refsplines.cloneAndAppend(allSplines[iref]);
//                        m_controlIndices.push_back(i);
//                    }
//
//                    ++i;
//                } else {
//                    for (int j = 0; j < actu.numControls(); ++j) {
//                        std::string controlName = actuPath + "_" + 
//                                std::to_string(j);
//                        if (refName == controlName) {
//                            m_refsplines.cloneAndAppend(allSplines[iref]);
//                            m_controlIndices.push_back(i);
//                        }
//
//                        ++i;
//                    }
//                }
//            }
//        }
//    }
//
//    void calcIntegralCostImpl(const SimTK::State& state,
//            double& integrand) const override {
//
//        const auto& time = state.getTime();
//        SimTK::Vector timeVec(1, time);
//        
//        const auto& controls = getModel().getControls(state);
//        integrand = 0;
//        // TODO cache the reference coordinate values at the mesh points, 
//        // rather than evaluating the spline.
//        for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
//            const auto& refValue = m_refsplines[iref].calcValue(timeVec);
//            integrand += m_control_weights[iref] * 
//                         pow(controls[m_controlIndices[iref]] - refValue, 2);
//        }
//    }
//
//private:
//    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
//        "Set of weight objects to weight the tracking of individual "
//        "control variables in the cost.");
//
//    void constructProperties() {
//        constructProperty_control_weights(MocoWeightSet());
//    }
//
//    TimeSeriesTable m_table;
//    mutable GCVSplineSet m_refsplines;
//    mutable std::vector<int> m_controlIndices;
//    mutable std::vector<double> m_control_weights;
//
//};


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

class COPTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(COPTrackingCost, MocoCost);
public:
    COPTrackingCost() {}
    COPTrackingCost(std::string name, double weight)
        : MocoCost(std::move(name), weight) {}

    void setReference(const TimeSeriesTable& ref) {
        m_ref = ref;
    }
    void setExternalForceNames(const std::vector<std::string>& names) {
        m_external_force_names = names;
    }
protected:
    void initializeOnModelImpl(const Model& model) const override {

        auto colLabels = m_ref.getColumnLabels();
        m_refsplines = GCVSplineSet(m_ref, colLabels);
        std::vector<std::string> suffixes = {"x", "y", "z"};

        for (const auto& extForceName : m_external_force_names) {
            const auto& extForce = 
                    model.getComponent<ExternalForce>(extForceName);
            m_model_ext_forces.emplace_back(&extForce);

            // Find the reference data column labels that match the COP names
            // and save their indices. 
            for (int i = 0; i < colLabels.size(); ++i) {
                for (const auto& suffix : suffixes) {
                    if (colLabels[i] == 
                            (extForce.getPointIdentifier() + suffix)) {
                        m_refindices.push_back(i);
                    }
                }
            }
        }
    }

    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override {
        const auto& time = state.getTime();
        // Need to realize to velocity to get controls.
        getModel().realizeDynamics(state);
        SimTK::Vector timeVec(1, time);

        for (int iforce = 0; iforce < m_model_ext_forces.size(); ++iforce) {
            const auto& extforce = m_model_ext_forces[iforce];

            int point_x_idx = m_refindices[3*iforce];
            int point_y_idx = m_refindices[3*iforce + 1];
            int point_z_idx = m_refindices[3*iforce + 2];

            SimTK::Vec3 copRef(
                    m_refsplines[point_x_idx].calcValue(timeVec),
                    m_refsplines[point_y_idx].calcValue(timeVec),
                    m_refsplines[point_z_idx].calcValue(timeVec));

            SimTK::Vec3 copModel = extforce->getPointAtTime(time);
            
            // Convert points to body frame.
            // TODO: this assumes that the data is in the same frame as the
            // COP in the model actuator.
            if (extforce->getPointExpressedInBodyName() == "ground") {

                const auto& bodyName = extforce->getAppliedToBodyName();
                const auto& body = 
                        getModel().getComponent<Body>("/bodyset/" + bodyName);

                copModel = getModel().getGround().
                    findStationLocationInAnotherFrame(state, copModel,
                        body);
            }

            integrand += (copModel - copRef).normSqr() / copRef.norm();

        }
    }

private:
    TimeSeriesTable m_ref;
    mutable std::vector<int> m_refindices;
    mutable GCVSplineSet m_refsplines;
    std::vector<std::string> m_external_force_names;
    mutable std::vector<SimTK::ReferencePtr<const ExternalForce>> 
    m_model_ext_forces;

};

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

void transformReactionToBodyFrame(const MocoTool& moco, 
        const MocoIterate& iterate, 
        TimeSeriesTable_<SimTK::SpatialVec>& reactionTable) {
    auto model = moco.getProblem().createRep().getModelBase();
    model.initSystem();
    const auto& ground = model.getGround();
    auto statesTraj = iterate.exportToStatesTrajectory(moco.getProblem());
    assert(statesTraj.getSize() == reactionTable.getNumRows());

    for (int irow = 0; irow < reactionTable.getNumRows(); ++irow) {
        auto& row = reactionTable.updRowAtIndex(irow);
        for (int ielt = 0; ielt < row.size(); ++ielt) {

            const auto& state = statesTraj.get(irow);
            const auto& label = reactionTable.getColumnLabel(ielt);
            std::string frameName;
            if (label.find("walker_knee_l") != std::string::npos) {
                frameName = "/bodyset/tibia_l";
            } else if (label.find("walker_knee_r") != std::string::npos) {
                frameName = "/bodyset/tibia_r";
            }
            const auto& frame = model.getComponent<Frame>(frameName);

            const auto& elt = row.getElt(0, ielt);
            model.realizeAcceleration(state);
            SimTK::Vec3 moment = ground.expressVectorInAnotherFrame(state,
                elt[0], frame);
            SimTK::Vec3 force = ground.expressVectorInAnotherFrame(state,
                elt[1], frame);

            SimTK::SpatialVec newElt(moment, force);
            row.updElt(0, ielt) = newElt;
        }
    }
}

void transformExternalForceToBodyFrame(const MocoTool& moco,
        const MocoIterate& iterate,
        const std::string& extLoadsFile) {

    auto model = moco.getProblem().createRep().getModelBase();
    model.initSystem();

    const auto& ground = model.getGround();

    auto statesTraj = iterate.exportToStatesTrajectory(moco.getProblem());
    std::vector<double> time;
    for (const auto& state : statesTraj) {
        time.push_back(state.getTime());
    }
    TimeSeriesTableVec3 table(time);
    for (const auto& extForce : model.getComponentList<ExternalForce>()) {
        const auto& bodyName = extForce.getAppliedToBodyName();
        const auto& body = model.getComponent<Body>("/bodyset/" + bodyName);

        SimTK::Vector_<SimTK::Vec3> moment(statesTraj.getSize());
        SimTK::Vector_<SimTK::Vec3> force(statesTraj.getSize());
        SimTK::Vector_<SimTK::Vec3> point(statesTraj.getSize());
        for (int istate = 0; istate < time.size(); ++istate) {
            const auto& state = statesTraj.get(istate);
            model.realizeAcceleration(state);

            auto time = state.getTime();
            SimTK::Vec3 momentInGround = extForce.getTorqueAtTime(time);
            SimTK::Vec3 forceInGround = extForce.getForceAtTime(time);
            SimTK::Vec3 pointInGround = extForce.getPointAtTime(time);

            moment[istate] = ground.expressVectorInAnotherFrame(state,
                momentInGround, body);
            force[istate] = ground.expressVectorInAnotherFrame(state,
                forceInGround, body);
            point[istate] = ground.expressVectorInAnotherFrame(state,
                pointInGround, body);
        }

        table.appendColumn(extForce.getTorqueIdentifier(), moment);
        table.appendColumn(extForce.getForceIdentifier(), force);
        table.appendColumn(extForce.getPointIdentifier(), point);
    }

    TimeSeriesTable tableFlat = table.flatten({"x", "y", "z"});
    STOFileAdapter::write(tableFlat, "forces_transformed_to_body.sto");
}

Model createModel() {

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
    addCoordinateActuator(model, "hip_rotation_l", 20);
    addCoordinateActuator(model, "hip_rotation_r", 20);
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
    addCoordinateActuator(model, "lumbar_bending", 100);
    addCoordinateActuator(model, "lumbar_extension", 100);
    addCoordinateActuator(model, "lumbar_rotation", 100);

    model.print("subject_walk_rra_adjusted_updated.osim");

    return model;
}

void smoothSolutionControls(std::string statesFile, 
        const std::string& guessFile) {

    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cerr.rdbuf(LogManager::cerr.rdbuf());

    Model model = createModel();

    MocoTrack track;
    track.setName("smoothed");
    track.setModel(model);
    track.set_states_tracking_file(statesFile);
    track.set_external_loads_file("grf_walk.xml");
    track.set_external_loads_mode("applied");
    track.set_guess_type("from_file");
    track.set_guess_file(guessFile);
    track.set_initial_time(0.825);
    track.set_final_time(1.635);

    track.set_minimize_controls(0.1);

    MocoTool moco = track.initialize();

    auto& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_convergence_tolerance(1e-3);

    MocoProblem problem = moco.getProblem();

    MocoSolution solution = moco.solve().unseal();
    solution.write(statesFile.replace(statesFile.end()-4, statesFile.end(),
        "_smoothed.sto"));
    moco.visualize(solution);

    TimeSeriesTable_<SimTK::SpatialVec> reactionTable =
        moco.analyze<SimTK::SpatialVec>(solution,
        {"/jointset/walker_knee_l/reaction_on_child",
            "/jointset/walker_knee_r/reaction_on_child"});

    transformReactionToBodyFrame(moco, solution, reactionTable);
    TimeSeriesTable reactionTableFlat = reactionTable.flatten();
    STOFileAdapter::write(reactionTableFlat, 
        statesFile.replace(statesFile.end() - 4, statesFile.end(),
            "_reactions.sto"));
}

int main() {

    Model model = createModel();

    // Baseline tracking problem.
    // --------------------------
    MocoTrack track;
    track.setName("baseline");
    track.setModel(model);
    track.set_states_tracking_file("coordinates_rra_adjusted.sto");
    MocoWeightSet stateWeights;
    MocoSolution prevSol("sandboxMocoTrack_solution.sto");
    for (const auto& stateName : prevSol.getStateNames()) {
        if (stateName.find("pelvis") != std::string::npos) {
            stateWeights.cloneAndAppend({stateName, 1});
        } else if (stateName.find("lumbar") != std::string::npos) {
            // From Fregly et al. 2007: unlock all back rotations.
            stateWeights.cloneAndAppend({stateName, 1});
        } else if (stateName.find("hip") != std::string::npos ||
                   stateName.find("knee") != std::string::npos ||
                   stateName.find("ankle") != std::string::npos) {
            // From Fregly et al. 2007: unlock all hip, knee, and ankle 
            // rotations.
            stateWeights.cloneAndAppend({stateName, 1});
        } else {
            stateWeights.cloneAndAppend({stateName, 1});
        }
    }
    track.set_state_weights(stateWeights);
    track.set_track_state_reference_derivatives(true);
    //track.set_markers_tracking_file("motion_capture_walk.trc");
    //track.set_ik_setup_file("ik_setup_walk_feet_only.xml");
    track.set_external_loads_file("grf_walk.xml");
    track.set_external_loads_mode("applied");
    track.set_initial_time(0.81);
    track.set_final_time(1.65);

    track.set_minimize_controls(0.05);

    MocoTool moco = track.initialize();
    auto& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-2);
    solver.set_optim_convergence_tolerance(1e-2);

    MocoProblem problem = moco.getProblem();
    //MocoSolution solution = moco.solve().unseal();
    //solution.write("sandboxMocoTrack_solution_applied_extloads.sto");
    //moco.visualize(solution);

    //TimeSeriesTable_<SimTK::SpatialVec> reactionTable =
    //        moco.analyze<SimTK::SpatialVec>(solution,
    //            {"/jointset/walker_knee_l/reaction_on_child",
    //             "/jointset/walker_knee_r/reaction_on_child"});

    //transformReactionToBodyFrame(moco, solution, reactionTable);
    //TimeSeriesTable reactionTableFlat = reactionTable.flatten();    
    //STOFileAdapter::write(reactionTableFlat, "knee_reactions.sto");
    MocoSolution solution("sandboxMocoTrack_solution_baseline.sto");

    // Smooth baseline solution.
    // -------------------------
    //smoothSolutionControls("sandboxMocoTrack_solution_baseline.sto",
    //    "sandboxMocoTrack_solution_baseline.sto");

    //transformExternalForceToBodyFrame(moco, solutionPrev, "grf_walk.xml");

    // Medial thrust gait.
    // -------------------
    MocoTrack trackMTG;
    trackMTG.setModel(model);
    trackMTG.set_states_tracking_file("sandboxMocoTrack_solution_baseline.sto");
    // Track 
    MocoWeightSet coordinateWeights;
    for (const auto& stateName : solution.getStateNames()) {
        if (stateName.find("pelvis") != std::string::npos) {
            if (stateName == "pelvis_tx" || stateName == "pelvis_tz") {
                coordinateWeights.cloneAndAppend({stateName, 1000});
            } else {
                // From Fregly et al. 2007: unlock superior/inferior translation
                // and all pelvis rotations.
                coordinateWeights.cloneAndAppend({stateName, 0.0001});
            }
        } else if (stateName.find("lumbar") != std::string::npos) {
            // From Fregly et al. 2007: unlock all back rotations.
            coordinateWeights.cloneAndAppend({stateName, 0.0001});
        } else if (stateName.find("hip") != std::string::npos ||
                   stateName.find("knee") != std::string::npos ||
                   stateName.find("ankle") != std::string::npos) {
            // From Fregly et al. 2007: unlock all hip, knee, and ankle 
            // rotations.
            coordinateWeights.cloneAndAppend({stateName, 0.0001});
        } else {
            coordinateWeights.cloneAndAppend({stateName, 250});
        }
    }
    trackMTG.set_state_weights(coordinateWeights);
    trackMTG.set_external_loads_file("grf_walk.xml");
    trackMTG.set_external_loads_mode("applied");

    // Keep the low weighted control minimization term to help smooth controls.
    trackMTG.set_minimize_controls(0.01);
    trackMTG.set_guess_type("from_file");
    trackMTG.set_guess_file("sandboxMocoTrack_solution_MTG.sto");
    trackMTG.set_initial_time(0.82);
    trackMTG.set_final_time(1.64);
    MocoTool mocoMTG = trackMTG.initialize();
    auto& problemMTG = mocoMTG.updProblem();

    // COP tracking (in the body frame).
    // w3 from Fregly et al. 2007
    //auto* copTracking = problemMTG.addCost<COPTrackingCost>("cop_tracking", 
    //        10);
    //copTracking->setReference(TimeSeriesTable("forces_new_labels.mot"));
    //copTracking->setFreePointBodyActuatorNames({"Left_GRF", "Right_GRF"});

    //// Control tracking cost.
    //// Construct controls reference.
    //auto time = solutionPrev.getTime();
    //std::vector<double> timeVec;
    //for (int i = 0; i < time.size(); ++i) {
    //    timeVec.push_back(time[i]);
    //}
    //TimeSeriesTable controlsRef(timeVec);
    //// Control weights.
    //MocoWeightSet controlTrackingWeights;
    //for (const auto& controlName : controlNames) {
    //    if (controlName.find("pelvis") != std::string::npos) {
    //        // w6 from Fregly et al. 2007
    //        controlTrackingWeights.cloneAndAppend({controlName, 25});
    //        controlsRef.appendColumn(controlName, 
    //                solutionPrev.getControl(controlName));
    //    } else if (controlName.find("hip") != std::string::npos ||
    //               controlName.find("knee") != std::string::npos ||
    //               controlName.find("ankle") != std::string::npos) {
    //        // w2 from Fregly et al. 2007
    //        controlTrackingWeights.cloneAndAppend({controlName, 1});
    //        controlsRef.appendColumn(controlName, 
    //                solutionPrev.getControl(controlName));
    //    }
    //}
    //auto* controlTracking =
    //        problemMTG.addCost<ControlTrackingCost>("control_tracking", 1);
    //controlTracking->setReference(controlsRef);
    //controlTracking->setWeightSet(controlTrackingWeights);

    // Feet transform tracking cost.
    auto statesTraj = solution.exportToStatesTrajectory(problem);
    // w4 from Fregly et al. 2007
    auto* footTransformTracking =
            problemMTG.addCost<TransformTrackingCost>("foot_tracking", 500);
    footTransformTracking->setStatesTrajectory(statesTraj);
    footTransformTracking->setComponentPaths({"/bodyset/calcn_r", 
            "/bodyset/calcn_l"});
    footTransformTracking->setTrackedComponents("all");

    // Torso transform tracking cost.
    // w5 from Fregly et al. 2007
    auto* torsoTransformTracking =
        problemMTG.addCost<TransformTrackingCost>("torso_tracking", 500);
    torsoTransformTracking->setStatesTrajectory(statesTraj);
    torsoTransformTracking->setComponentPaths({"/bodyset/torso"});
    torsoTransformTracking->setTrackedComponents("rotation");

    // Knee adduction cost.
    // w1 from Fregly et al. 2007
    auto* kneeAdductionCost_l = 
        problemMTG.addCost<MocoJointReactionCost>("knee_adduction_cost_l", 250);
    kneeAdductionCost_l->setJointPath("/jointset/walker_knee_l");
    kneeAdductionCost_l->setExpressedInFramePath("/bodyset/tibia_l");
    kneeAdductionCost_l->setReactionComponent(0);
    auto* kneeAdductionCost_r =
        problemMTG.addCost<MocoJointReactionCost>("knee_adduction_cost_r", 250);
    kneeAdductionCost_r->setJointPath("/jointset/walker_knee_r");
    kneeAdductionCost_r->setExpressedInFramePath("/bodyset/tibia_r");
    kneeAdductionCost_r->setReactionComponent(0);

    auto& solverMTG = mocoMTG.updSolver<MocoCasADiSolver>();
    solverMTG.set_optim_constraint_tolerance(1e-2);
    solverMTG.set_optim_convergence_tolerance(1e-2);

    //MocoSolution solutionMTG = mocoMTG.solve().unseal();
    //solutionMTG.write("sandboxMocoTrack_solution_MTG.sto");
    //mocoMTG.visualize(solutionMTG);

    //TimeSeriesTable_<SimTK::SpatialVec> reactionTableMTG = 
    //        mocoMTG.analyze<SimTK::SpatialVec>(solutionMTG,
    //            {"/jointset/walker_knee_l/reaction_on_child",
    //             "/jointset/walker_knee_r/reaction_on_child"});

    //transformReactionToBodyFrame(mocoMTG, solutionMTG, reactionTableMTG);
    //TimeSeriesTable reactionTableFlatMTG = reactionTableMTG.flatten();

    //STOFileAdapter::write(reactionTableFlatMTG, "knee_reactions_MTG.sto");

    //MocoSolution solutionMTG("sandboxMocoTrack_solution_MTG.sto");

    smoothSolutionControls("sandboxMocoTrack_solution_MTG.sto",
            "sandboxMocoTrack_solution_MTG.sto");

    //// Track medial thrust gait result to smooth controls.
    //// ---------------------------------------------------
    //MocoTrack trackMTG2;
    //trackMTG2.setModel(model);
    //trackMTG2.set_states_tracking_file("sandboxMocoTrack_solution_MTG.sto");
    //trackMTG2.set_external_loads_file("grf_walk.xml");
    //trackMTG2.set_guess_type("from_file");
    //trackMTG2.set_guess_file("sandboxMocoTrack_solution_MTG2.sto");
    //trackMTG2.set_initial_time(0.82);
    //trackMTG2.set_final_time(1.64);

    //// Control weights.
    //MocoWeightSet controlWeightsMTG2;
    //// Penalize pelvis residuals.
    //// Don't penalize GRF controls.
    //for (int i = 0; i < 6; ++i) {
    //    controlWeightsMTG2.cloneAndAppend({"Left_GRF_" + std::to_string(i), 0});
    //    controlWeightsMTG2.cloneAndAppend({"Right_GRF_" + std::to_string(i), 0});
    //}
    //trackMTG2.set_minimize_controls(0.01);
    //trackMTG2.set_control_weights(controlWeightsMTG2);
    //// Set the tracking weights for the GRFs. Keep all weights high, since we
    //// want good tracking in the ground frame.
    //MocoWeightSet extLoadWeightsMTG2;
    //for (int i = 0; i < 6; ++i) {
    //    extLoadWeightsMTG2.cloneAndAppend({"Left_GRF_" + std::to_string(i), 10});
    //    extLoadWeightsMTG2.cloneAndAppend({"Right_GRF_" + std::to_string(i), 10});
    //}
    //trackMTG2.set_external_load_weights(extLoadWeightsMTG2);
    //trackMTG2.set_external_loads_mode("tracked");

    //MocoTool mocoMTG2 = trackMTG2.initialize();

    //auto& solverMTG2 = mocoMTG2.updSolver<MocoCasADiSolver>();
    //solverMTG2.set_optim_constraint_tolerance(1e-3);
    //solverMTG2.set_optim_convergence_tolerance(1e-3);

    //MocoProblem problemMTG2 = mocoMTG2.getProblem();
    ////MocoSolution solutionMTG2 = mocoMTG2.solve().unseal();
    ////solutionMTG2.write("sandboxMocoTrack_solution_MTG2.sto");
    ////moco.visualize(solutionMTG2);

    ////TimeSeriesTable_<SimTK::SpatialVec> reactionTableMTG2 =
    ////    mocoMTG2.analyze<SimTK::SpatialVec>(solutionMTG2,
    ////    {"/jointset/walker_knee_l/reaction_on_child",
    ////        "/jointset/walker_knee_r/reaction_on_child"});

    ////transformReactionToBodyFrame(mocoMTG2, solutionMTG2, reactionTableMTG2);
    ////TimeSeriesTable reactionTableFlatMTG2 = reactionTableMTG2.flatten();

    ////STOFileAdapter::write(reactionTableFlatMTG2, "knee_reactions_MTG2.sto");

    ////MocoSolution solutionMTG("sandboxMocoTrack_solution_MTG.sto");


    return EXIT_SUCCESS;
}
