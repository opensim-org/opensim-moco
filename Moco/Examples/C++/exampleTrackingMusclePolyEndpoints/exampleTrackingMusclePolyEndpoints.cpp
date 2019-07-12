/* -------------------------------------------------------------------------- *
 * OpenSim Moco: examplePredictive.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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
// This class defines a MocoCost that encourages symmetry of the walking cycle
// by minimizing the difference between initial and final states/controls.
class MocoSymmetryCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoSymmetryCost, MocoCost);
public:
    MocoSymmetryCost() = default;
    MocoSymmetryCost(std::string name) : MocoCost(std::move(name)) {}
    MocoSymmetryCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {}
protected:
    void calcCostImpl(const CostInput& input, SimTK::Real& cost)
        const override {
        // Initial states: coordinates - positions (all but pelvis tx)
        SimTK::Real position_groundPelvis_q_rz_IS =
            m_coord_groundPelvis_q_rz->getValue(input.initial_state);
        SimTK::Real position_groundPelvis_q_ty_IS =
            m_coord_groundPelvis_q_ty->getValue(input.initial_state);
        SimTK::Real position_hip_q_l_IS =
            m_coord_hip_q_l->getValue(input.initial_state);
        SimTK::Real position_hip_q_r_IS =
            m_coord_hip_q_r->getValue(input.initial_state);
        SimTK::Real position_knee_q_l_IS =
            m_coord_knee_q_l->getValue(input.initial_state);
        SimTK::Real position_knee_q_r_IS =
            m_coord_knee_q_r->getValue(input.initial_state);
        SimTK::Real position_ankle_q_l_IS =
            m_coord_ankle_q_l->getValue(input.initial_state);
        SimTK::Real position_ankle_q_r_IS =
            m_coord_ankle_q_r->getValue(input.initial_state);
        SimTK::Real position_lumbar_q_IS =
            m_coord_lumbar_q->getValue(input.initial_state);
        // Final states: coordinates - positions (all but pelvis tx)
        SimTK::Real position_groundPelvis_q_rz_FS =
            m_coord_groundPelvis_q_rz->getValue(input.final_state);
        SimTK::Real position_groundPelvis_q_ty_FS =
            m_coord_groundPelvis_q_ty->getValue(input.final_state);
        SimTK::Real position_hip_q_l_FS =
            m_coord_hip_q_l->getValue(input.final_state);
        SimTK::Real position_hip_q_r_FS =
            m_coord_hip_q_r->getValue(input.final_state);
        SimTK::Real position_knee_q_l_FS =
            m_coord_knee_q_l->getValue(input.final_state);
        SimTK::Real position_knee_q_r_FS =
            m_coord_knee_q_r->getValue(input.final_state);
        SimTK::Real position_ankle_q_l_FS =
            m_coord_ankle_q_l->getValue(input.final_state);
        SimTK::Real position_ankle_q_r_FS =
            m_coord_ankle_q_r->getValue(input.final_state);
        SimTK::Real position_lumbar_q_FS =
            m_coord_lumbar_q->getValue(input.final_state);

        //// Initial states: coordinates - speeds (all)
        //SimTK::Real speed_groundPelvis_q_rz_IS =
        //    m_coord_groundPelvis_q_rz->getSpeedValue(input.initial_state);
        //SimTK::Real speed_groundPelvis_q_tx_IS =
        //    m_coord_groundPelvis_q_tx->getSpeedValue(input.initial_state);
        //SimTK::Real speed_groundPelvis_q_ty_IS =
        //    m_coord_groundPelvis_q_ty->getSpeedValue(input.initial_state);
        //SimTK::Real speed_hip_q_l_IS =
        //    m_coord_hip_q_l->getSpeedValue(input.initial_state);
        //SimTK::Real speed_hip_q_r_IS =
        //    m_coord_hip_q_r->getSpeedValue(input.initial_state);
        //SimTK::Real speed_knee_q_l_IS =
        //    m_coord_knee_q_l->getSpeedValue(input.initial_state);
        //SimTK::Real speed_knee_q_r_IS =
        //    m_coord_knee_q_r->getSpeedValue(input.initial_state);
        //SimTK::Real speed_ankle_q_l_IS =
        //    m_coord_ankle_q_l->getSpeedValue(input.initial_state);
        //SimTK::Real speed_ankle_q_r_IS =
        //    m_coord_ankle_q_r->getSpeedValue(input.initial_state);
        //SimTK::Real speed_lumbar_q_IS =
        //    m_coord_lumbar_q->getSpeedValue(input.initial_state);
        //// Final states: coordinates - speeds (all)
        //SimTK::Real speed_groundPelvis_q_rz_FS =
        //    m_coord_groundPelvis_q_rz->getSpeedValue(input.final_state);
        //SimTK::Real speed_groundPelvis_q_tx_FS =
        //    m_coord_groundPelvis_q_tx->getSpeedValue(input.final_state);
        //SimTK::Real speed_groundPelvis_q_ty_FS =
        //    m_coord_groundPelvis_q_ty->getSpeedValue(input.final_state);
        //SimTK::Real speed_hip_q_l_FS =
        //    m_coord_hip_q_l->getSpeedValue(input.final_state);
        //SimTK::Real speed_hip_q_r_FS =
        //    m_coord_hip_q_r->getSpeedValue(input.final_state);
        //SimTK::Real speed_knee_q_l_FS =
        //    m_coord_knee_q_l->getSpeedValue(input.final_state);
        //SimTK::Real speed_knee_q_r_FS =
        //    m_coord_knee_q_r->getSpeedValue(input.final_state);
        //SimTK::Real speed_ankle_q_l_FS =
        //    m_coord_ankle_q_l->getSpeedValue(input.final_state);
        //SimTK::Real speed_ankle_q_r_FS =
        //    m_coord_ankle_q_r->getSpeedValue(input.final_state);
        //SimTK::Real speed_lumbar_q_FS =
        //    m_coord_lumbar_q->getSpeedValue(input.final_state);

        //// Initial states: muscles and coordinate actuator (all)
        //// Coordinate actuator
        //SimTK::Real control_lumbar_IS =
        //    m_coordAct_lumbar->getControl(input.initial_state);
        //// Muscles
        //// Right
        //SimTK::Real control_hamstrings_r_IS =
        //    m_hamstrings_r->getControl(input.initial_state);
        //SimTK::Real control_bifemsh_r_IS =
        //    m_bifemsh_r->getControl(input.initial_state);
        //SimTK::Real control_glut_max_r_IS =
        //    m_glut_max_r->getControl(input.initial_state);
        //SimTK::Real control_iliopsoas_r_IS =
        //    m_iliopsoas_r->getControl(input.initial_state);
        //SimTK::Real control_rect_fem_r_IS =
        //    m_rect_fem_r->getControl(input.initial_state);
        //SimTK::Real control_vasti_r_IS =
        //    m_vasti_r->getControl(input.initial_state);
        //SimTK::Real control_gastroc_r_IS =
        //    m_gastroc_r->getControl(input.initial_state);
        //SimTK::Real control_soleus_r_IS =
        //    m_soleus_r->getControl(input.initial_state);
        //SimTK::Real control_tib_ant_r_IS =
        //    m_tib_ant_r->getControl(input.initial_state);
        //// Left
        //SimTK::Real control_hamstrings_l_IS =
        //    m_hamstrings_l->getControl(input.initial_state);
        //SimTK::Real control_bifemsh_l_IS =
        //    m_bifemsh_l->getControl(input.initial_state);
        //SimTK::Real control_glut_max_l_IS =
        //    m_glut_max_l->getControl(input.initial_state);
        //SimTK::Real control_iliopsoas_l_IS =
        //    m_iliopsoas_l->getControl(input.initial_state);
        //SimTK::Real control_rect_fem_l_IS =
        //    m_rect_fem_l->getControl(input.initial_state);
        //SimTK::Real control_vasti_l_IS =
        //    m_vasti_l->getControl(input.initial_state);
        //SimTK::Real control_gastroc_l_IS =
        //    m_gastroc_l->getControl(input.initial_state);
        //SimTK::Real control_soleus_l_IS =
        //    m_soleus_l->getControl(input.initial_state);
        //SimTK::Real control_tib_ant_l_IS =
        //    m_tib_ant_l->getControl(input.initial_state);

        //// Final states: muscles and coordinate actuator (all)
        //// Coordinate actuator
        //SimTK::Real control_lumbar_FS =
        //    m_coordAct_lumbar->getControl(input.final_state);
        //// Muscles
        //// Right
        //SimTK::Real control_hamstrings_r_FS =
        //    m_hamstrings_r->getControl(input.final_state);
        //SimTK::Real control_bifemsh_r_FS =
        //    m_bifemsh_r->getControl(input.final_state);
        //SimTK::Real control_glut_max_r_FS =
        //    m_glut_max_r->getControl(input.final_state);
        //SimTK::Real control_iliopsoas_r_FS =
        //    m_iliopsoas_r->getControl(input.final_state);
        //SimTK::Real control_rect_fem_r_FS =
        //    m_rect_fem_r->getControl(input.final_state);
        //SimTK::Real control_vasti_r_FS =
        //    m_vasti_r->getControl(input.final_state);
        //SimTK::Real control_gastroc_r_FS =
        //    m_gastroc_r->getControl(input.final_state);
        //SimTK::Real control_soleus_r_FS =
        //    m_soleus_r->getControl(input.final_state);
        //SimTK::Real control_tib_ant_r_FS =
        //    m_tib_ant_r->getControl(input.final_state);
        //// Left
        //SimTK::Real control_hamstrings_l_FS =
        //    m_hamstrings_l->getControl(input.final_state);
        //SimTK::Real control_bifemsh_l_FS =
        //    m_bifemsh_l->getControl(input.final_state);
        //SimTK::Real control_glut_max_l_FS =
        //    m_glut_max_l->getControl(input.final_state);
        //SimTK::Real control_iliopsoas_l_FS =
        //    m_iliopsoas_l->getControl(input.final_state);
        //SimTK::Real control_rect_fem_l_FS =
        //    m_rect_fem_l->getControl(input.final_state);
        //SimTK::Real control_vasti_l_FS =
        //    m_vasti_l->getControl(input.final_state);
        //SimTK::Real control_gastroc_l_FS =
        //    m_gastroc_l->getControl(input.final_state);
        //SimTK::Real control_soleus_l_FS =
        //    m_soleus_l->getControl(input.final_state);
        //SimTK::Real control_tib_ant_l_FS =
        //    m_tib_ant_l->getControl(input.final_state);

        // Cost
        // Squared differences between initial and final coordinate values
        // For the hips, knees, and ankles, the difference is between the right
        // and left legs since the simulation is for half a gait cycle.
        cost = SimTK::square(position_groundPelvis_q_rz_IS -
                position_groundPelvis_q_rz_FS) +
            SimTK::square(position_groundPelvis_q_ty_IS -
                position_groundPelvis_q_ty_FS) +
            SimTK::square(position_hip_q_l_IS - position_hip_q_r_FS) +
            SimTK::square(position_hip_q_r_IS - position_hip_q_l_FS) +
            SimTK::square(position_knee_q_l_IS - position_knee_q_r_FS) +
            SimTK::square(position_knee_q_r_IS - position_knee_q_l_FS) +
            SimTK::square(position_ankle_q_l_IS - position_ankle_q_r_FS) +
            SimTK::square(position_ankle_q_r_IS - position_ankle_q_l_FS) +
            SimTK::square(position_lumbar_q_IS - position_lumbar_q_FS);
        //// Squared differences between initial and final coordinate speeds
        //// For the hips, knees, and ankles, the difference is between the right
        //// and left legs since the simulation is for half a gait cycle.
        //cost += SimTK::square(speed_groundPelvis_q_rz_IS -
        //        speed_groundPelvis_q_rz_FS) +
        //    SimTK::square(speed_groundPelvis_q_tx_IS -
        //        speed_groundPelvis_q_tx_FS) +
        //    SimTK::square(speed_groundPelvis_q_ty_IS -
        //        speed_groundPelvis_q_ty_FS) +
        //    SimTK::square(speed_hip_q_l_IS - speed_hip_q_r_FS) +
        //    SimTK::square(speed_hip_q_r_IS - speed_hip_q_l_FS) +
        //    SimTK::square(speed_knee_q_l_IS - speed_knee_q_r_FS) +
        //    SimTK::square(speed_knee_q_r_IS - speed_knee_q_l_FS) +
        //    SimTK::square(speed_ankle_q_l_IS - speed_ankle_q_r_FS) +
        //    SimTK::square(speed_ankle_q_r_IS - speed_ankle_q_l_FS) +
        //    SimTK::square(speed_lumbar_q_IS - speed_lumbar_q_FS);
        //// Squared differences between initial and final actuator (coordinate
        //// and muscles) controls. The difference is between the right and left
        //// legs since the simulation is for half a gait cycle.
        //cost += SimTK::square(control_lumbar_IS - control_lumbar_FS) +
        //    SimTK::square(control_hamstrings_r_IS - control_hamstrings_l_FS) +
        //    SimTK::square(control_hamstrings_l_IS - control_hamstrings_r_FS) +
        //    SimTK::square(control_bifemsh_r_IS - control_bifemsh_l_FS) +
        //    SimTK::square(control_bifemsh_l_IS - control_bifemsh_r_FS) +
        //    SimTK::square(control_glut_max_r_IS - control_glut_max_l_FS) +
        //    SimTK::square(control_glut_max_l_IS - control_glut_max_r_FS) +
        //    SimTK::square(control_iliopsoas_r_IS - control_iliopsoas_l_FS) +
        //    SimTK::square(control_iliopsoas_l_IS - control_iliopsoas_r_FS) +
        //    SimTK::square(control_rect_fem_r_IS - control_rect_fem_l_FS) +
        //    SimTK::square(control_rect_fem_l_IS - control_rect_fem_r_FS) +
        //    SimTK::square(control_vasti_r_IS - control_vasti_l_FS) +
        //    SimTK::square(control_vasti_l_IS - control_vasti_r_FS) +
        //    SimTK::square(control_gastroc_r_IS - control_gastroc_l_FS) +
        //    SimTK::square(control_gastroc_l_IS - control_gastroc_r_FS) +
        //    SimTK::square(control_soleus_r_IS - control_soleus_l_FS) +
        //    SimTK::square(control_soleus_l_IS - control_soleus_r_FS) +
        //    SimTK::square(control_tib_ant_r_IS - control_tib_ant_l_FS) +
        //    SimTK::square(control_tib_ant_l_IS - control_tib_ant_r_FS);

    }
    void initializeOnModelImpl(const Model& model) const {
        // Coordinates
        m_coord_groundPelvis_q_rz.reset(
                &model.getCoordinateSet().get("groundPelvis_q_rz"));
        m_coord_groundPelvis_q_tx.reset(
                &model.getCoordinateSet().get("groundPelvis_q_tx"));
        m_coord_groundPelvis_q_ty.reset(
                &model.getCoordinateSet().get("groundPelvis_q_ty"));
        m_coord_hip_q_l.reset(&model.getCoordinateSet().get("hip_q_l"));
        m_coord_hip_q_r.reset(&model.getCoordinateSet().get("hip_q_r"));
        m_coord_knee_q_l.reset(&model.getCoordinateSet().get("knee_q_l"));
        m_coord_knee_q_r.reset(&model.getCoordinateSet().get("knee_q_r"));
        m_coord_ankle_q_l.reset(&model.getCoordinateSet().get("ankle_q_l"));
        m_coord_ankle_q_r.reset(&model.getCoordinateSet().get("ankle_q_r"));
        m_coord_lumbar_q.reset(&model.getCoordinateSet().get("lumbar_q"));
        //// Coordinate actuators
        //m_coordAct_lumbar.reset(
        //        &model.getComponent<CoordinateActuator>("lumbarAct"));
        //// Muscles
        //// Right
        //m_hamstrings_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "hamstrings_r"));
        //m_bifemsh_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "bifemsh_r"));
        //m_glut_max_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "glut_max_r"));
        //m_iliopsoas_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "iliopsoas_r"));
        //m_rect_fem_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "rect_fem_r"));
        //m_vasti_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "vasti_r"));
        //m_gastroc_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "gastroc_r"));
        //m_soleus_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "soleus_r"));
        //m_tib_ant_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "tib_ant_r"));
        //// Left
        //m_hamstrings_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "hamstrings_l"));
        //m_bifemsh_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "bifemsh_l"));
        //m_glut_max_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "glut_max_l"));
        //m_iliopsoas_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "iliopsoas_l"));
        //m_rect_fem_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "rect_fem_l"));
        //m_vasti_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "vasti_l"));
        //m_gastroc_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "gastroc_l"));
        //m_soleus_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "soleus_l"));
        //m_tib_ant_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
        //        "tib_ant_l"));

    }
private:
    // Coordinates
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_groundPelvis_q_rz;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_groundPelvis_q_tx;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_groundPelvis_q_ty;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_hip_q_l;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_hip_q_r;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_knee_q_l;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_knee_q_r;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_ankle_q_l;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_ankle_q_r;
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_lumbar_q;
    //// Coordinate actuators
    //mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_lumbar;
    //// Muscles
    //// Right
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_hamstrings_r;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_bifemsh_r;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_glut_max_r;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_iliopsoas_r;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_rect_fem_r;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_vasti_r;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_gastroc_r;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_soleus_r;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_tib_ant_r;
    //// Left
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_hamstrings_l;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_bifemsh_l;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_glut_max_l;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_iliopsoas_l;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_rect_fem_l;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_vasti_l;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_gastroc_l;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_soleus_l;
    //mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_tib_ant_l;

};



void testTracking(){

    MocoTrack track;
    track.setName("gait2D_TrackingMusclePolyEndPoints");

    ModelProcessor modelprocessor = ModelProcessor(
        "gait_2D_contact_muscle_poly_bounds_noPelvisRes.osim");

    // Set model
    track.setModel(modelprocessor);
    track.setStatesReference(TableProcessor("IK_reference_radian.sto") |
        TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_apply_tracked_states_to_guess(true);

    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    MocoStudy moco = track.initialize();
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100000);
    solver.set_parallel(8);

    MocoProblem& problem = moco.updProblem();
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");
    symmetryCost->set_weight(10);

    //moco.visualize(solver.getGuess());

    MocoSolution solution = moco.solve();
    moco.visualize(solution);
}

int main()  {
    try { testTracking(); }
    catch (const OpenSim::Exception& e) {
        e.print(std::cerr);
        return 1;
    }
    return 0;
}
