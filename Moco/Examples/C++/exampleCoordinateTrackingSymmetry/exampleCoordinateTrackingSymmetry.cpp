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

class MocoSymmetryCost2 : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoSymmetryCost2, MocoCost);
public:
    MocoSymmetryCost2() = default;
    MocoSymmetryCost2(std::string name) : MocoCost(std::move(name)) {}
protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    bool getDefaultEndpointConstraintImpl() const override { return true; }
    int getNumOutputsImpl() const override { return 26; }
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
        SimTK::Real control_hip_l_IS =
            m_coordAct_hip_l->getControl(input.initial_state);
        SimTK::Real control_hip_r_IS =
            m_coordAct_hip_r->getControl(input.initial_state);
        SimTK::Real control_knee_l_IS =
            m_coordAct_knee_l->getControl(input.initial_state);
        SimTK::Real control_knee_r_IS =
            m_coordAct_knee_r->getControl(input.initial_state);
        SimTK::Real control_ankle_l_IS =
            m_coordAct_ankle_l->getControl(input.initial_state);
        SimTK::Real control_ankle_r_IS =
            m_coordAct_ankle_r->getControl(input.initial_state);
        SimTK::Real control_lumbar_IS =
            m_coordAct_lumbar->getControl(input.initial_state);
        // Final states: muscles and coordinate actuator (all)
        // Coordinate actuator
        SimTK::Real control_hip_l_FS =
            m_coordAct_hip_l->getControl(input.final_state);
        SimTK::Real control_hip_r_FS =
            m_coordAct_hip_r->getControl(input.final_state);
        SimTK::Real control_knee_l_FS =
            m_coordAct_knee_l->getControl(input.final_state);
        SimTK::Real control_knee_r_FS =
            m_coordAct_knee_r->getControl(input.final_state);
        SimTK::Real control_ankle_l_FS =
            m_coordAct_ankle_l->getControl(input.final_state);
        SimTK::Real control_ankle_r_FS =
            m_coordAct_ankle_r->getControl(input.final_state);
        SimTK::Real control_lumbar_FS =
            m_coordAct_lumbar->getControl(input.final_state);

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

        // Coordinate actuators
        values[19] = control_hip_l_IS - control_hip_r_FS;
        values[20] = control_hip_r_IS - control_hip_l_FS;
        values[21] = control_knee_l_IS - control_knee_r_FS;
        values[22] = control_knee_r_IS - control_knee_l_FS;
        values[23] = control_ankle_l_IS - control_ankle_r_FS;
        values[24] = control_ankle_r_IS - control_ankle_l_FS;
        values[25] = control_lumbar_IS - control_lumbar_FS;

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
        m_coordAct_hip_l.reset(
                &model.getComponent<CoordinateActuator>("hipAct_l"));
        m_coordAct_hip_r.reset(
                &model.getComponent<CoordinateActuator>("hipAct_r"));
        m_coordAct_knee_l.reset(
                &model.getComponent<CoordinateActuator>("kneeAct_l"));
        m_coordAct_knee_r.reset(
                &model.getComponent<CoordinateActuator>("kneeAct_r"));
        m_coordAct_ankle_l.reset(
                &model.getComponent<CoordinateActuator>("ankleAct_l"));
        m_coordAct_ankle_r.reset(
                &model.getComponent<CoordinateActuator>("ankleAct_r"));
        m_coordAct_lumbar.reset(
                &model.getComponent<CoordinateActuator>("lumbarAct"));
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
    mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_hip_l;
    mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_hip_r;
    mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_knee_l;
    mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_knee_r;
    mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_ankle_l;
    mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_ankle_r;
    mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_lumbar;
};

// Set a coordinate tracking problem. Here the model is driven by muscles whose
// muscle-tendon lengths, velocities, and moment arms are derived from
// polynomial approximations of joint coordinates. The cost function combines
// a tracking term (coordinate values and speeds), an effort term (squared
// controls), and a term encouraging symmetry of the coordinate values over
// half a gait cycle.
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
    solver.set_parallel(6);
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

    // Extract ground reaction forces
    // Get optimal states
    StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
    // Get optimal time vector
    SimTK::Vector optTime = solution.getTime();
    // Get model
    auto model = modelprocessor.process();
    // Create labels for output file
    std::vector<std::string> labels;
    labels.push_back("time");
    labels.push_back("ground_force_vx"); // right
    labels.push_back("ground_force_vy");
    labels.push_back("ground_force_vz");
    labels.push_back("ground_force_px");
    labels.push_back("ground_force_py");
    labels.push_back("ground_force_pz");
    labels.push_back("1_ground_force_vx"); // left
    labels.push_back("1_ground_force_vy");
    labels.push_back("1_ground_force_vz");
    labels.push_back("1_ground_force_px");
    labels.push_back("1_ground_force_py");
    labels.push_back("1_ground_force_pz");
    labels.push_back("ground_torque_x"); // right
    labels.push_back("ground_torque_y");
    labels.push_back("ground_torque_z");
    labels.push_back("1_ground_torque_x"); // left
    labels.push_back("1_ground_torque_y");
    labels.push_back("1_ground_torque_z");
    TimeSeriesTable externalForcesTable{};
    externalForcesTable.setColumnLabels(labels);
    // Helper Vec3
    SimTK::Vec3 nullP(0);
    // Extract forces
    int count = 0;
    for (const auto& state : optStates) {
        Array<double> forcesContactSphereHeel_r = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereHeel_r").getRecordValues(state);
        Array<double> forcesContactSphereHeel_l = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereHeel_l").getRecordValues(state);
        Array<double> forcesContactSphereFront_r = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereFront_r").getRecordValues(state);
        Array<double> forcesContactSphereFront_l = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereFront_l").getRecordValues(state);
        // Combine forces and torques from contact spheres from same foot
        // Forces
        SimTK::Vec3 forces_r = SimTK::Vec3(forcesContactSphereHeel_r[0],
                forcesContactSphereHeel_r[1], forcesContactSphereHeel_r[2]) +
                SimTK::Vec3(forcesContactSphereFront_r[0],
                forcesContactSphereFront_r[1], forcesContactSphereFront_r[2]);
        SimTK::Vec3 forces_l = SimTK::Vec3(forcesContactSphereHeel_l[0],
                forcesContactSphereHeel_l[1], forcesContactSphereHeel_l[2]) +
                SimTK::Vec3(forcesContactSphereFront_l[0],
                forcesContactSphereFront_l[1], forcesContactSphereFront_l[2]);
        // Torques
        SimTK::Vec3 torques_r = SimTK::Vec3(forcesContactSphereHeel_r[3],
                forcesContactSphereHeel_r[4], forcesContactSphereHeel_r[5]) +
                SimTK::Vec3(forcesContactSphereFront_r[3],
                forcesContactSphereFront_r[4], forcesContactSphereFront_r[5]);
        SimTK::Vec3 torques_l = SimTK::Vec3(forcesContactSphereHeel_l[3],
                forcesContactSphereHeel_l[4], forcesContactSphereHeel_l[5]) +
                SimTK::Vec3(forcesContactSphereFront_l[3],
                forcesContactSphereFront_l[4], forcesContactSphereFront_l[5]);
        // Create row
        SimTK::RowVector row{18,0.0};
        for (int i = 0; i < 2; ++i) {
            row(i) = forces_r[i];
            row(i+3) = nullP[i];
            row(i+6) = forces_l[i];
            row(i+9) = nullP[i];
            row(i+12) = torques_r[i];
            row(i+15) = torques_l[i];
        }
        // Append row
        externalForcesTable.appendRow(optTime[count],row);
        ++count;
    }
    // Write file
    writeTableToFile(externalForcesTable,"test_GRF.sto");
}

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
            "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
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
    solver.set_parallel(6);
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

    // Extract ground reaction forces
    // Get optimal states
    StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
    // Get optimal time vector
    SimTK::Vector optTime = solution.getTime();
    // Get model
    auto model = modelprocessor.process();
    // Create labels for output file
    std::vector<std::string> labels;
    labels.push_back("time");
    labels.push_back("ground_force_vx"); // right
    labels.push_back("ground_force_vy");
    labels.push_back("ground_force_vz");
    labels.push_back("ground_force_px");
    labels.push_back("ground_force_py");
    labels.push_back("ground_force_pz");
    labels.push_back("1_ground_force_vx"); // left
    labels.push_back("1_ground_force_vy");
    labels.push_back("1_ground_force_vz");
    labels.push_back("1_ground_force_px");
    labels.push_back("1_ground_force_py");
    labels.push_back("1_ground_force_pz");
    labels.push_back("ground_torque_x"); // right
    labels.push_back("ground_torque_y");
    labels.push_back("ground_torque_z");
    labels.push_back("1_ground_torque_x"); // left
    labels.push_back("1_ground_torque_y");
    labels.push_back("1_ground_torque_z");
    TimeSeriesTable externalForcesTable{};
    externalForcesTable.setColumnLabels(labels);
    // Helper Vec3
    SimTK::Vec3 nullP(0);
    // Extract forces
    int count = 0;
    for (const auto& state : optStates) {
        Array<double> forcesContactSphereHeel_r = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereHeel_r").getRecordValues(state);
        Array<double> forcesContactSphereHeel_l = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereHeel_l").getRecordValues(state);
        Array<double> forcesContactSphereFront_r = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereFront_r").getRecordValues(state);
        Array<double> forcesContactSphereFront_l = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereFront_l").getRecordValues(state);
        // Combine forces and torques from contact spheres from same foot
        // Forces
        SimTK::Vec3 forces_r = SimTK::Vec3(forcesContactSphereHeel_r[0],
                forcesContactSphereHeel_r[1], forcesContactSphereHeel_r[2]) +
                SimTK::Vec3(forcesContactSphereFront_r[0],
                forcesContactSphereFront_r[1], forcesContactSphereFront_r[2]);
        SimTK::Vec3 forces_l = SimTK::Vec3(forcesContactSphereHeel_l[0],
                forcesContactSphereHeel_l[1], forcesContactSphereHeel_l[2]) +
                SimTK::Vec3(forcesContactSphereFront_l[0],
                forcesContactSphereFront_l[1], forcesContactSphereFront_l[2]);
        // Torques
        SimTK::Vec3 torques_r = SimTK::Vec3(forcesContactSphereHeel_r[3],
                forcesContactSphereHeel_r[4], forcesContactSphereHeel_r[5]) +
                SimTK::Vec3(forcesContactSphereFront_r[3],
                forcesContactSphereFront_r[4], forcesContactSphereFront_r[5]);
        SimTK::Vec3 torques_l = SimTK::Vec3(forcesContactSphereHeel_l[3],
                forcesContactSphereHeel_l[4], forcesContactSphereHeel_l[5]) +
                SimTK::Vec3(forcesContactSphereFront_l[3],
                forcesContactSphereFront_l[4], forcesContactSphereFront_l[5]);
        // Create row
        SimTK::RowVector row{18,0.0};
        for (int i = 0; i < 2; ++i) {
            row(i) = forces_r[i];
            row(i+3) = nullP[i];
            row(i+6) = forces_l[i];
            row(i+9) = nullP[i];
            row(i+12) = torques_r[i];
            row(i+15) = torques_l[i];
        }
        // Append row
        externalForcesTable.appendRow(optTime[count],row);
        ++count;
    }
    // Write file
    writeTableToFile(externalForcesTable,"test_GRF.sto");
}


void testCoordinateTracking_CoordinateActuators() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking_CoordinateActuators");
    // Set model
    ModelProcessor modelprocessor = ModelProcessor(
            "gait10dof18musc_CoordinateActuators.osim");
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
    auto* symmetryCost = problem.addCost<MocoSymmetryCost2>("symmetryCost");
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

    // Extract ground reaction forces
    // Get optimal states
    StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
    // Get optimal time vector
    SimTK::Vector optTime = solution.getTime();
    // Get model
    auto model = modelprocessor.process();
    model.initSystem();
    // Create labels for output file
    std::vector<std::string> labels;
    labels.push_back("ground_force_vx"); // right
    labels.push_back("ground_force_vy");
    labels.push_back("ground_force_vz");
    labels.push_back("ground_force_px");
    labels.push_back("ground_force_py");
    labels.push_back("ground_force_pz");
    labels.push_back("1_ground_force_vx"); // left
    labels.push_back("1_ground_force_vy");
    labels.push_back("1_ground_force_vz");
    labels.push_back("1_ground_force_px");
    labels.push_back("1_ground_force_py");
    labels.push_back("1_ground_force_pz");
    labels.push_back("ground_torque_x"); // right
    labels.push_back("ground_torque_y");
    labels.push_back("ground_torque_z");
    labels.push_back("1_ground_torque_x"); // left
    labels.push_back("1_ground_torque_y");
    labels.push_back("1_ground_torque_z");
    //labels.push_back("ground_force_vx"); // right
    //labels.push_back("ground_force_vy");
    //labels.push_back("ground_force_vz");
    //labels.push_back("ground_force_vx_b");
    //labels.push_back("ground_force_vy_b");
    //labels.push_back("ground_force_vz_b");
    //labels.push_back("1_ground_force_vx"); // left
    //labels.push_back("1_ground_force_vy");
    //labels.push_back("1_ground_force_vz");
    //labels.push_back("1_ground_force_vx_b");
    //labels.push_back("1_ground_force_vy_b");
    //labels.push_back("1_ground_force_vz_b");
    //labels.push_back("ground_torque_x"); // right
    //labels.push_back("ground_torque_y");
    //labels.push_back("ground_torque_z");
    //labels.push_back("ground_torque_x_b"); // right
    //labels.push_back("ground_torque_y_b");
    //labels.push_back("ground_torque_z_b");
    //labels.push_back("1_ground_torque_x"); // left
    //labels.push_back("1_ground_torque_y");
    //labels.push_back("1_ground_torque_z");
    //labels.push_back("1_ground_torque_x_b"); // left
    //labels.push_back("1_ground_torque_y_b");
    //labels.push_back("1_ground_torque_z_b");
    TimeSeriesTable externalForcesTable{};
    externalForcesTable.setColumnLabels(labels);
    TimeSeriesTable externalForcesTable2{};
    externalForcesTable2.setColumnLabels(labels);
    // Helper Vec3
    SimTK::Vec3 nullP(0);
    // Extract forces
    int count = 0;
    for (const auto& state : optStates) {
        model.realizeVelocity(state);
        Array<double> forcesContactSphereFront_l = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereFront_l").getRecordValues(state);
        Array<double> forcesContactSphereHeel_r = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereHeel_r").getRecordValues(state);
        Array<double> forcesContactSphereHeel_l = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereHeel_l").getRecordValues(state);
        Array<double> forcesContactSphereFront_r = model.getComponent<
                SmoothSphereHalfSpaceForce>(
                "contactSphereFront_r").getRecordValues(state);

        // Combine forces and torques from contact spheres from same foot
        // Forces
        SimTK::Vec3 forces_r = SimTK::Vec3(forcesContactSphereHeel_r[0],
                forcesContactSphereHeel_r[1], forcesContactSphereHeel_r[2]) +
                SimTK::Vec3(forcesContactSphereFront_r[0],
                forcesContactSphereFront_r[1], forcesContactSphereFront_r[2]);
        SimTK::Vec3 forces_l = SimTK::Vec3(forcesContactSphereHeel_l[0],
                forcesContactSphereHeel_l[1], forcesContactSphereHeel_l[2]) +
                SimTK::Vec3(forcesContactSphereFront_l[0],
                forcesContactSphereFront_l[1], forcesContactSphereFront_l[2]);
        // Torques
        SimTK::Vec3 torques_r = SimTK::Vec3(forcesContactSphereHeel_r[3],
                forcesContactSphereHeel_r[4], forcesContactSphereHeel_r[5]) +
                SimTK::Vec3(forcesContactSphereFront_r[3],
                forcesContactSphereFront_r[4], forcesContactSphereFront_r[5]);
        SimTK::Vec3 torques_l = SimTK::Vec3(forcesContactSphereHeel_l[3],
                forcesContactSphereHeel_l[4], forcesContactSphereHeel_l[5]) +
                SimTK::Vec3(forcesContactSphereFront_l[3],
                forcesContactSphereFront_l[4], forcesContactSphereFront_l[5]);

        //SimTK::Vec3 locCSHInB_r = model.getComponent<SmoothSphereHalfSpaceForce>("contactSphereHeel_r").getProperty_contact_sphere_location().getValue();
        //SimTK::Vec3 locCSHInG_r = model.getBodySet().get("calcn_r").findStationLocationInGround(state,locCSHInB_r);
        ////std::cout << locCSHInB_r << std::endl;
        ////std::cout << locCSHInG_r << std::endl;
        //SimTK::Vec3 locCSFInB_r = model.getComponent<SmoothSphereHalfSpaceForce>("contactSphereFront_r").getProperty_contact_sphere_location().getValue();
        //SimTK::Vec3 locCSFInG_r = model.getBodySet().get("calcn_r").findStationLocationInGround(state,locCSFInB_r);
        ////std::cout << locCSFInB_r << std::endl;
        ////std::cout << locCSFInG_r << std::endl;
        //SimTK::Real dCSH = SimTK::square(std::pow(locCSHInG_r[0],2)+std::pow(locCSHInG_r[2],2));
        //SimTK::Real dCSF = SimTK::square(std::pow(locCSFInG_r[0],2)+std::pow(locCSFInG_r[2],2));
        //SimTK::Real M_r = dCSH*forcesContactSphereHeel_r[1] + dCSF*forcesContactSphereFront_r[1];
        //std::cout << torques_r << std::endl;
        //std::cout << M_r << std::endl;


        // Express as three forces and three torques
        // Create row
        SimTK::Vec3 pApp_r(0);
        SimTK::Vec3 pApp_l(0);
        SimTK::RowVector row{18,0.0};
        for (int i = 0; i < 3; ++i) {
            row(i) = forces_r[i];
            row(i+3) = pApp_r[i];
            row(i+6) = forces_l[i];
            row(i+9) = pApp_l[i];
            row(i+12) = torques_r[i];
            row(i+15) = torques_l[i];
        }
        externalForcesTable.appendRow(optTime[count],row);

        // Express as three forces, COP, and one torque
        SimTK::Vec3 pApp2_r(0);
        pApp2_r[0] = torques_r[2]/forces_r[1];
        pApp2_r[2] = -torques_r[0]/forces_r[1];
        SimTK::Vec3 torques2_r(0);
        torques2_r[1] = torques_r[1] + torques_r[0]/forces_r[1]*forces_r[0] +
                torques_r[2]/forces_r[1]*forces_r[2];
        SimTK::Vec3 pApp2_l(0);
        pApp2_l[0] = torques_l[2]/forces_l[1];
        pApp2_l[2] = -torques_l[0]/forces_l[1];
        SimTK::Vec3 torques2_l(0);
        torques2_l[1] = torques_l[1] + torques_l[0]/forces_l[1]*forces_l[0] +
                torques_l[2]/forces_l[1]*forces_l[2];

        // Create row
        SimTK::RowVector row2{18,0.0};
        for (int i = 0; i < 3; ++i) {
            row2(i) = forces_r[i];
            row2(i+3) = pApp2_r[i];
            row2(i+6) = forces_l[i];
            row2(i+9) = pApp2_l[i];
            row2(i+12) = torques2_r[i];
            row2(i+15) = torques2_l[i];
        }
        externalForcesTable2.appendRow(optTime[count],row2);
        ++count;
    }
    // Write file
    writeTableToFile(externalForcesTable,"test_GRF_A.mot");
    writeTableToFile(externalForcesTable2,"test_GRF_B.mot");
}

int main() {
   //testCoordinateTracking_MusclePolynomials_withPassiveForces();
   //testCoordinateTracking_MusclePolynomials_withoutPassiveForces();
   testCoordinateTracking_CoordinateActuators();
}
