/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleCoordinateTrackingSymmetry.cpp                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

using namespace OpenSim;
// This class defines a MocoCost that imposes symmetry of the walking cycle.
class MocoSymmetryCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoSymmetryCost, MocoCost);
public:
    MocoSymmetryCost() = default;
    MocoSymmetryCost(std::string name) : MocoCost(std::move(name)) {}
protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    bool getDefaultEndpointConstraintImpl() const override { return true; }
    int getNumOutputsImpl() const override { return 38; }
    int getNumIntegralsImpl() const override { return 0; }
    void calcCostImpl(const CostInput& input, SimTK::Vector& values)
            const override {
        // Initial states: coordinates - positions (all but pelvis tx)
        SimTK::Real position_pelvis_tilt_IS =
                m_coord_pelvis_tilt->getValue(input.initial_state);
        SimTK::Real position_pelvis_ty_IS =
                m_coord_pelvis_ty->getValue(input.initial_state);
        SimTK::Real position_hip_flexion_l_IS =
                m_coord_hip_flexion_l->getValue(input.initial_state);
        SimTK::Real position_hip_flexion_r_IS =
                m_coord_hip_flexion_r->getValue(input.initial_state);
        SimTK::Real position_knee_angle_l_IS =
                m_coord_knee_angle_l->getValue(input.initial_state);
        SimTK::Real position_knee_angle_r_IS =
                m_coord_knee_angle_r->getValue(input.initial_state);
        SimTK::Real position_ankle_angle_l_IS =
                m_coord_ankle_angle_l->getValue(input.initial_state);
        SimTK::Real position_ankle_angle_r_IS =
                m_coord_ankle_angle_r->getValue(input.initial_state);
        SimTK::Real position_lumbar_IS =
                m_coord_lumbar->getValue(input.initial_state);
        // Final states: coordinates - positions (all but pelvis tx)
        SimTK::Real position_pelvis_tilt_FS =
                m_coord_pelvis_tilt->getValue(input.final_state);
        SimTK::Real position_pelvis_ty_FS =
                m_coord_pelvis_ty->getValue(input.final_state);
        SimTK::Real position_hip_flexion_l_FS =
                m_coord_hip_flexion_l->getValue(input.final_state);
        SimTK::Real position_hip_flexion_r_FS =
                m_coord_hip_flexion_r->getValue(input.final_state);
        SimTK::Real position_knee_angle_l_FS =
                m_coord_knee_angle_l->getValue(input.final_state);
        SimTK::Real position_knee_angle_r_FS =
                m_coord_knee_angle_r->getValue(input.final_state);
        SimTK::Real position_ankle_angle_l_FS =
                m_coord_ankle_angle_l->getValue(input.final_state);
        SimTK::Real position_ankle_angle_r_FS =
                m_coord_ankle_angle_r->getValue(input.final_state);
        SimTK::Real position_lumbar_FS =
                m_coord_lumbar->getValue(input.final_state);

        // Initial states: coordinates - speeds (all)
        SimTK::Real speed_pelvis_tilt_IS =
            m_coord_pelvis_tilt->getSpeedValue(input.initial_state);
        SimTK::Real speed_pelvis_tx_IS =
            m_coord_pelvis_tx->getSpeedValue(input.initial_state);
        SimTK::Real speed_pelvis_ty_IS =
            m_coord_pelvis_ty->getSpeedValue(input.initial_state);
        SimTK::Real speed_hip_flexion_l_IS =
            m_coord_hip_flexion_l->getSpeedValue(input.initial_state);
        SimTK::Real speed_hip_flexion_r_IS =
            m_coord_hip_flexion_r->getSpeedValue(input.initial_state);
        SimTK::Real speed_knee_angle_l_IS =
            m_coord_knee_angle_l->getSpeedValue(input.initial_state);
        SimTK::Real speed_knee_angle_r_IS =
            m_coord_knee_angle_r->getSpeedValue(input.initial_state);
        SimTK::Real speed_ankle_angle_l_IS =
            m_coord_ankle_angle_l->getSpeedValue(input.initial_state);
        SimTK::Real speed_ankle_angle_r_IS =
            m_coord_ankle_angle_r->getSpeedValue(input.initial_state);
        SimTK::Real speed_lumbar_IS =
            m_coord_lumbar->getSpeedValue(input.initial_state);
        // Final states: coordinates - speeds (all)
        SimTK::Real speed_pelvis_tilt_FS =
            m_coord_pelvis_tilt->getSpeedValue(input.final_state);
        SimTK::Real speed_pelvis_tx_FS =
            m_coord_pelvis_tx->getSpeedValue(input.final_state);
        SimTK::Real speed_pelvis_ty_FS =
            m_coord_pelvis_ty->getSpeedValue(input.final_state);
        SimTK::Real speed_hip_flexion_l_FS =
            m_coord_hip_flexion_l->getSpeedValue(input.final_state);
        SimTK::Real speed_hip_flexion_r_FS =
            m_coord_hip_flexion_r->getSpeedValue(input.final_state);
        SimTK::Real speed_knee_angle_l_FS =
            m_coord_knee_angle_l->getSpeedValue(input.final_state);
        SimTK::Real speed_knee_angle_r_FS =
            m_coord_knee_angle_r->getSpeedValue(input.final_state);
        SimTK::Real speed_ankle_angle_l_FS =
            m_coord_ankle_angle_l->getSpeedValue(input.final_state);
        SimTK::Real speed_ankle_angle_r_FS =
            m_coord_ankle_angle_r->getSpeedValue(input.final_state);
        SimTK::Real speed_lumbar_FS =
            m_coord_lumbar->getSpeedValue(input.final_state);

        // Initial states: muscles and coordinate actuator (all)
        // Coordinate actuator
        SimTK::Real control_lumbar_IS =
            m_coordAct_lumbar->getControl(input.initial_state);
        // Muscles
        // Right
        SimTK::Real control_hamstrings_r_IS =
            m_hamstrings_r->getControl(input.initial_state);
        SimTK::Real control_bifemsh_r_IS =
            m_bifemsh_r->getControl(input.initial_state);
        SimTK::Real control_glut_max_r_IS =
            m_glut_max_r->getControl(input.initial_state);
        SimTK::Real control_iliopsoas_r_IS =
            m_iliopsoas_r->getControl(input.initial_state);
        SimTK::Real control_rect_fem_r_IS =
            m_rect_fem_r->getControl(input.initial_state);
        SimTK::Real control_vasti_r_IS =
            m_vasti_r->getControl(input.initial_state);
        SimTK::Real control_gastroc_r_IS =
            m_gastroc_r->getControl(input.initial_state);
        SimTK::Real control_soleus_r_IS =
            m_soleus_r->getControl(input.initial_state);
        SimTK::Real control_tib_ant_r_IS =
            m_tib_ant_r->getControl(input.initial_state);
        // Left
        SimTK::Real control_hamstrings_l_IS =
            m_hamstrings_l->getControl(input.initial_state);
        SimTK::Real control_bifemsh_l_IS =
            m_bifemsh_l->getControl(input.initial_state);
        SimTK::Real control_glut_max_l_IS =
            m_glut_max_l->getControl(input.initial_state);
        SimTK::Real control_iliopsoas_l_IS =
            m_iliopsoas_l->getControl(input.initial_state);
        SimTK::Real control_rect_fem_l_IS =
            m_rect_fem_l->getControl(input.initial_state);
        SimTK::Real control_vasti_l_IS =
            m_vasti_l->getControl(input.initial_state);
        SimTK::Real control_gastroc_l_IS =
            m_gastroc_l->getControl(input.initial_state);
        SimTK::Real control_soleus_l_IS =
            m_soleus_l->getControl(input.initial_state);
        SimTK::Real control_tib_ant_l_IS =
            m_tib_ant_l->getControl(input.initial_state);

        // Final states: muscles and coordinate actuator (all)
        // Coordinate actuator
        SimTK::Real control_lumbar_FS =
            m_coordAct_lumbar->getControl(input.final_state);
        // Muscles
        // Right
        SimTK::Real control_hamstrings_r_FS =
            m_hamstrings_r->getControl(input.final_state);
        SimTK::Real control_bifemsh_r_FS =
            m_bifemsh_r->getControl(input.final_state);
        SimTK::Real control_glut_max_r_FS =
            m_glut_max_r->getControl(input.final_state);
        SimTK::Real control_iliopsoas_r_FS =
            m_iliopsoas_r->getControl(input.final_state);
        SimTK::Real control_rect_fem_r_FS =
            m_rect_fem_r->getControl(input.final_state);
        SimTK::Real control_vasti_r_FS =
            m_vasti_r->getControl(input.final_state);
        SimTK::Real control_gastroc_r_FS =
            m_gastroc_r->getControl(input.final_state);
        SimTK::Real control_soleus_r_FS =
            m_soleus_r->getControl(input.final_state);
        SimTK::Real control_tib_ant_r_FS =
            m_tib_ant_r->getControl(input.final_state);
        // Left
        SimTK::Real control_hamstrings_l_FS =
            m_hamstrings_l->getControl(input.final_state);
        SimTK::Real control_bifemsh_l_FS =
            m_bifemsh_l->getControl(input.final_state);
        SimTK::Real control_glut_max_l_FS =
            m_glut_max_l->getControl(input.final_state);
        SimTK::Real control_iliopsoas_l_FS =
            m_iliopsoas_l->getControl(input.final_state);
        SimTK::Real control_rect_fem_l_FS =
            m_rect_fem_l->getControl(input.final_state);
        SimTK::Real control_vasti_l_FS =
            m_vasti_l->getControl(input.final_state);
        SimTK::Real control_gastroc_l_FS =
            m_gastroc_l->getControl(input.final_state);
        SimTK::Real control_soleus_l_FS =
            m_soleus_l->getControl(input.final_state);
        SimTK::Real control_tib_ant_l_FS =
            m_tib_ant_l->getControl(input.final_state);

        // Constraints
        // Positions
        values[0] = position_pelvis_tilt_IS - position_pelvis_tilt_FS;
        values[1] = position_pelvis_ty_IS - position_pelvis_ty_FS;
        values[2] = position_hip_flexion_l_IS - position_hip_flexion_r_FS;
        values[3] = position_hip_flexion_r_IS - position_hip_flexion_l_FS;
        values[4] = position_knee_angle_l_IS - position_knee_angle_r_FS;
        values[5] = position_knee_angle_r_IS - position_knee_angle_l_FS;
        values[6] = position_ankle_angle_l_IS - position_ankle_angle_r_FS;
        values[7] = position_ankle_angle_r_IS - position_ankle_angle_l_FS;
        values[8] = position_lumbar_IS - position_lumbar_FS;

        // Speeds
        values[9] = speed_pelvis_tilt_IS - speed_pelvis_tilt_FS;
        values[10] = speed_pelvis_tx_IS - speed_pelvis_tx_FS;
        values[11] = speed_pelvis_ty_IS - speed_pelvis_ty_FS;
        values[12] = speed_hip_flexion_l_IS - speed_hip_flexion_r_FS;
        values[13] = speed_hip_flexion_r_IS - speed_hip_flexion_l_FS;
        values[14] = speed_knee_angle_l_IS - speed_knee_angle_r_FS;
        values[15] = speed_knee_angle_r_IS - speed_knee_angle_l_FS;
        values[16] = speed_ankle_angle_l_IS - speed_ankle_angle_r_FS;
        values[17] = speed_ankle_angle_r_IS - speed_ankle_angle_l_FS;
        values[18] = speed_lumbar_IS - speed_lumbar_FS;

        // Excitations
        values[19] = control_lumbar_IS - control_lumbar_FS;
        values[20] = control_hamstrings_r_IS - control_hamstrings_l_FS;
        values[21] = control_hamstrings_l_IS - control_hamstrings_r_FS;
        values[22] = control_bifemsh_r_IS - control_bifemsh_l_FS;
        values[23] = control_bifemsh_l_IS - control_bifemsh_r_FS;
        values[24] = control_glut_max_r_IS - control_glut_max_l_FS;
        values[25] = control_glut_max_l_IS - control_glut_max_r_FS;
        values[26] = control_iliopsoas_r_IS - control_iliopsoas_l_FS;
        values[27] = control_iliopsoas_l_IS - control_iliopsoas_r_FS;
        values[28] = control_rect_fem_r_IS - control_rect_fem_l_FS;
        values[29] = control_rect_fem_l_IS - control_rect_fem_r_FS;
        values[30] = control_vasti_r_IS - control_vasti_l_FS;
        values[31] = control_vasti_l_IS - control_vasti_r_FS;
        values[32] = control_gastroc_r_IS - control_gastroc_l_FS;
        values[33] = control_gastroc_l_IS - control_gastroc_r_FS;
        values[34] = control_soleus_r_IS - control_soleus_l_FS;
        values[35] = control_soleus_l_IS - control_soleus_r_FS;
        values[36] = control_tib_ant_r_IS - control_tib_ant_l_FS;
        values[37] = control_tib_ant_l_IS - control_tib_ant_r_FS;

    }
    void initializeOnModelImpl(const Model& model) const {
        // Coordinates
        m_coord_pelvis_tilt.reset(
                &model.getCoordinateSet().get("pelvis_tilt"));
        m_coord_pelvis_tx.reset(
                &model.getCoordinateSet().get("pelvis_tx"));
        m_coord_pelvis_ty.reset(
                &model.getCoordinateSet().get("pelvis_ty"));
        m_coord_hip_flexion_l.reset(
                &model.getCoordinateSet().get("hip_flexion_l"));
        m_coord_hip_flexion_r.reset(
                &model.getCoordinateSet().get("hip_flexion_r"));
        m_coord_knee_angle_l.reset(
                &model.getCoordinateSet().get("knee_angle_l"));
        m_coord_knee_angle_r.reset(
                &model.getCoordinateSet().get("knee_angle_r"));
        m_coord_ankle_angle_l.reset(
                &model.getCoordinateSet().get("ankle_angle_l"));
        m_coord_ankle_angle_r.reset(
                &model.getCoordinateSet().get("ankle_angle_r"));
        m_coord_lumbar.reset(
                &model.getCoordinateSet().get("lumbar"));
        // Coordinate actuators
        m_coordAct_lumbar.reset(
                &model.getComponent<CoordinateActuator>("lumbarAct"));
        // Muscles
        // Right
        m_hamstrings_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "hamstrings_r"));
        m_bifemsh_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "bifemsh_r"));
        m_glut_max_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "glut_max_r"));
        m_iliopsoas_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "iliopsoas_r"));
        m_rect_fem_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "rect_fem_r"));
        m_vasti_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "vasti_r"));
        m_gastroc_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "gastroc_r"));
        m_soleus_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "soleus_r"));
        m_tib_ant_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "tib_ant_r"));
        // Left
        m_hamstrings_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "hamstrings_l"));
        m_bifemsh_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "bifemsh_l"));
        m_glut_max_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "glut_max_l"));
        m_iliopsoas_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "iliopsoas_l"));
        m_rect_fem_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "rect_fem_l"));
        m_vasti_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "vasti_l"));
        m_gastroc_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "gastroc_l"));
        m_soleus_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "soleus_l"));
        m_tib_ant_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
                "tib_ant_l"));
    }
private:
    // Coordinates
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_pelvis_tilt;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_pelvis_tx;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_pelvis_ty;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_hip_flexion_l;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_hip_flexion_r;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_knee_angle_l;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_knee_angle_r;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_ankle_angle_l;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_ankle_angle_r;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_lumbar;
    // Coordinate actuators
    mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_lumbar;
    // Muscles
    // Right
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_hamstrings_r;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_bifemsh_r;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_glut_max_r;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_iliopsoas_r;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_rect_fem_r;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_vasti_r;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_gastroc_r;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_soleus_r;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_tib_ant_r;
    // Left
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_hamstrings_l;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_bifemsh_l;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_glut_max_l;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_iliopsoas_l;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_rect_fem_l;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_vasti_l;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_gastroc_l;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_soleus_l;
    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_tib_ant_l;
};

// Set a coordinate tracking problem. Here the model is driven by muscles whose
// muscle-tendon lengths, velocities, and moment arms are derived from
// polynomial approximations of joint coordinates. The cost function combines
// a tracking term (coordinate values and speeds), an effort term (squared
// controls), and a term encouraging symmetry of the coordinate values over
// half a gait cycle.
void testCoordinateTracking_MusclePolynomials_withPassiveForces() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking_MusclePolynomials_withPassiveForces");
    // Set model
    ModelProcessor modelprocessor = ModelProcessor(
            "gait10dof18musc_MusclePolynomials_withPassiveForces.osim.osim");
    track.setModel(modelprocessor);
    // Set experimental coordinate values to track
    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
            TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    // Set guess
    track.set_apply_tracked_states_to_guess(true);
    // Set time bounds
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    // Initialize study
    MocoStudy moco = track.initialize();
    // Set solver settings
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100000);
    solver.set_parallel(8);
    // Update problem
    MocoProblem& problem = moco.updProblem();
    // Add symmetry cost
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");
    symmetryCost->set_weight(10);
    // Add effort cots
    auto* effortCost = problem.addCost<MocoControlCost>("effortCost");
    effortCost->set_weight(10);
    // Adjust bounds
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75,1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0,20*SimTK::Pi/180});
    // Solve problem
    MocoSolution solution = moco.solve();
}

void testCoordinateTracking_MusclePolynomials_withoutPassiveForces() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking_MusclePolynomials_withoutPassiveForces");
    // Set model
    ModelProcessor modelprocessor = ModelProcessor(
            "gait10dof18musc_MusclePolynomials_withoutPassiveForces.osim");
    track.setModel(modelprocessor);
    // Set experimental coordinate values to track
    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
            TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    // Set guess
    track.set_apply_tracked_states_to_guess(true);
    // Set time bounds
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    // Initialize study
    MocoStudy moco = track.initialize();
    // Set solver settings
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100000);
    solver.set_parallel(8);
    // Update problem
    MocoProblem& problem = moco.updProblem();
    // Add symmetry cost
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");
    symmetryCost->set_weight(10);
    // Add effort cots
    auto* effortCost = problem.addCost<MocoControlCost>("effortCost");
    effortCost->set_weight(10);
    // Adjust bounds
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75,1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0,20*SimTK::Pi/180});
    // Solve problem
    MocoSolution solution = moco.solve();
}

int main() {
   //testCoordinateTracking_MusclePolynomials_withPassiveForces();
   testCoordinateTracking_MusclePolynomials_withoutPassiveForces();
}
