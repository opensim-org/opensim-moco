/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleCoordinateTracking.cpp                                *
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

// This class defines a MocoCost that computes the average speed defined as the
// distance travelled by the pelvis in the forward direction divided by the
// final time.
class MocoAverageSpeedCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoAverageSpeedCost, MocoCost);
public:
    OpenSim_DECLARE_PROPERTY(desired_speed, double,
            "The desired forward speed defined as the distance travelled by "
            "the pelvis in the forward direction divided by the final time.");
    MocoAverageSpeedCost() {
        constructProperties();
    }
    MocoAverageSpeedCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }
protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    bool getDefaultEndpointConstraintImpl() const override { return true; }
    int getNumOutputsImpl() const override { return 1; }
    int getNumIntegralsImpl() const override { return 0; }
    void calcCostImpl(const CostInput& input, SimTK::Vector& values)
        const override {
        // get final time
        SimTK::Real time = input.final_state.getTime();
        // get initial and final pelvis forward coordinate values
        SimTK::Real position_IS =  m_coord->getValue(input.initial_state);
        SimTK::Real position_FS =  m_coord->getValue(input.final_state);
        SimTK::Real distanceTravelled = position_FS - position_IS;
        values[0] = get_desired_speed() - (distanceTravelled / time);
    }
    void initializeOnModelImpl(const Model& model) const {
        m_coord.reset(&model.getCoordinateSet().get("pelvis_tx"));
    }
private:
    void constructProperties() {
        constructProperty_desired_speed(0.0);
    }
    mutable SimTK::ReferencePtr<const Coordinate> m_coord;
};

// This class defines a MocoCost that computes the integral of the squared
// controls divided by the distance travelled by the pelvis in the forward
// direction.
class MocoControlOverDistanceCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoControlOverDistanceCost, MocoCost);
public:
    MocoControlOverDistanceCost() = default;
    MocoControlOverDistanceCost(std::string name)
            : MocoCost(std::move(name)) {}
    MocoControlOverDistanceCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {}
    int getNumOutputsImpl() const override { return 1; }
    int getNumIntegralsImpl() const override { return 1; }
protected:
    void calcCostImpl(const CostInput& input, SimTK::Vector& cost)
        const override {
        // get initial and final pelvis forward coordinate values
        SimTK::Real position_IS =  m_coord->getValue(input.initial_state);
        SimTK::Real position_FS =  m_coord->getValue(input.final_state);
        SimTK::Real distanceTravelled = position_FS - position_IS;
        // normalize integral by distance travelled
        cost[0] = input.integral / distanceTravelled ;
    }
    void calcIntegrandImpl(
        const SimTK::State& state, double& integrand) const {
        // integrand is squared controls
        const auto& controls = getModel().getControls(state);
        integrand = 0;
        for (int i = 0; i < getModel().getNumControls(); ++i)
            integrand += SimTK::square(controls[i]);
    }
    void initializeOnModelImpl(const Model& model) const {
        m_coord.reset(&model.getCoordinateSet().get("pelvis_tx"));
    }
private:
    mutable SimTK::ReferencePtr<const Coordinate> m_coord;
};

// This class defines a MocoCost that computes the integral of the squared
// muscle activations + squared coordinate actuator controls divided by the
// distance travelled by the pelvis in the forward direction.
class MocoActivationOverDistanceCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoActivationOverDistanceCost, MocoCost);
public:
    MocoActivationOverDistanceCost() = default;
    MocoActivationOverDistanceCost(std::string name)
            : MocoCost(std::move(name)) {}
    MocoActivationOverDistanceCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {}
    int getNumOutputsImpl() const override { return 1; }
    int getNumIntegralsImpl() const override { return 1; }
protected:
    void calcCostImpl(const CostInput& input, SimTK::Vector& cost)
        const override {
        // get initial and final pelvis forward coordinate values
        SimTK::Real position_IS =  m_coord->getValue(input.initial_state);
        SimTK::Real position_FS =  m_coord->getValue(input.final_state);
        SimTK::Real distanceTravelled = position_FS - position_IS;
        // normalize integral by distance travelled
        cost[0] = input.integral / distanceTravelled ;
    }
    void calcIntegrandImpl(
        const SimTK::State& state, double& integrand) const {
        // Coordinate actuator
        SimTK::Real control_lumbar =  m_coordAct_lumbar->getControl(state);
        // Muscles
        // Right
        SimTK::Real state_hamstrings_r =
                m_hamstrings_r->getStateVariableValue(state, "activation");
        SimTK::Real state_bifemsh_r =
                m_bifemsh_r->getStateVariableValue(state, "activation");
        SimTK::Real state_glut_max_r =
                m_glut_max_r->getStateVariableValue(state, "activation");
        SimTK::Real state_iliopsoas_r =
                m_iliopsoas_r->getStateVariableValue(state, "activation");
        SimTK::Real state_rect_fem_r =
                m_rect_fem_r->getStateVariableValue(state, "activation");
        SimTK::Real state_vasti_r =
                m_vasti_r->getStateVariableValue(state, "activation");
        SimTK::Real state_gastroc_r =
                m_gastroc_r->getStateVariableValue(state, "activation");
        SimTK::Real state_soleus_r =
                m_soleus_r->getStateVariableValue(state, "activation");
        SimTK::Real state_tib_ant_r =
                m_tib_ant_r->getStateVariableValue(state, "activation");
        // Left
        SimTK::Real state_hamstrings_l =
                m_hamstrings_l->getStateVariableValue(state, "activation");
        SimTK::Real state_bifemsh_l =
                m_bifemsh_l->getStateVariableValue(state, "activation");
        SimTK::Real state_glut_max_l =
                m_glut_max_l->getStateVariableValue(state, "activation");
        SimTK::Real state_iliopsoas_l =
                m_iliopsoas_l->getStateVariableValue(state, "activation");
        SimTK::Real state_rect_fem_l =
                m_rect_fem_l->getStateVariableValue(state, "activation");
        SimTK::Real state_vasti_l =
                m_vasti_l->getStateVariableValue(state, "activation");
        SimTK::Real state_gastroc_l =
                m_gastroc_l->getStateVariableValue(state, "activation");
        SimTK::Real state_soleus_l =
                m_soleus_l->getStateVariableValue(state, "activation");
        SimTK::Real state_tib_ant_l =
                m_tib_ant_l->getStateVariableValue(state, "activation");
        // integrand is squared coordinate actuator control + squared muscle
        // states (i.e., muscle activations)
        integrand = 0;
        integrand += SimTK::square(control_lumbar);
        integrand += SimTK::square(state_hamstrings_r);
        integrand += SimTK::square(state_bifemsh_r);
        integrand += SimTK::square(state_glut_max_r);
        integrand += SimTK::square(state_iliopsoas_r);
        integrand += SimTK::square(state_rect_fem_r);
        integrand += SimTK::square(state_vasti_r);
        integrand += SimTK::square(state_gastroc_r);
        integrand += SimTK::square(state_soleus_r);
        integrand += SimTK::square(state_tib_ant_r);
        integrand += SimTK::square(state_hamstrings_l);
        integrand += SimTK::square(state_bifemsh_l);
        integrand += SimTK::square(state_glut_max_l);
        integrand += SimTK::square(state_iliopsoas_l);
        integrand += SimTK::square(state_rect_fem_l);
        integrand += SimTK::square(state_vasti_l);
        integrand += SimTK::square(state_gastroc_l);
        integrand += SimTK::square(state_soleus_l);
        integrand += SimTK::square(state_tib_ant_l);
    }
    void initializeOnModelImpl(const Model& model) const {
        // Coordinates
        m_coord.reset(&model.getCoordinateSet().get("pelvis_tx"));
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
    mutable SimTK::ReferencePtr<const Coordinate> m_coord;
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

// This class defines a MocoCost that computes the integral of the squared
// muscle activations + squared coordinate actuator controls divided by the
// distance travelled by the pelvis in the forward direction.
class MocoAccelerationOverDistanceCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoAccelerationOverDistanceCost, MocoCost);
public:
    MocoAccelerationOverDistanceCost() = default;
    MocoAccelerationOverDistanceCost(std::string name)
            : MocoCost(std::move(name)) {}
    MocoAccelerationOverDistanceCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {}
    int getNumOutputsImpl() const override { return 1; }
    int getNumIntegralsImpl() const override { return 1; }
protected:
    void calcCostImpl(const CostInput& input, SimTK::Vector& cost)
        const override {
        // get initial and final pelvis forward coordinate values
        SimTK::Real position_IS =  m_coord_pelvis_tx->getValue(
                input.initial_state);
        SimTK::Real position_FS =  m_coord_pelvis_tx->getValue(
                input.final_state);
        SimTK::Real distanceTravelled = position_FS - position_IS;
        // normalize integral by distance travelled
        cost[0] = input.integral / distanceTravelled ;
    }
    void calcIntegrandImpl(
        const SimTK::State& state, double& integrand) const {

        getModel().realizeAcceleration(state);
        const auto& uDots = state.getUDot();
        integrand = 0;
        for (int i = 0; i < getModel().getNumCoordinates(); ++i)
            integrand += SimTK::square(uDots[i]);
    }
    void initializeOnModelImpl(const Model& model) const {
        // Coordinates
        m_coord_pelvis_tx.reset(
                &model.getCoordinateSet().get("pelvis_tx"));
    }
private:
    // Coordinates
    mutable SimTK::ReferencePtr<const Coordinate> m_coord_pelvis_tx;
};

void testPredictive_withPassiveForces(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_withPassiveForce");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor(
        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
    problem.setModelProcessor(modelprocessor);

    problem.setTimeBounds(0, {0.4,0.6});

    //// Cost.
    //// =====
    // Impose symmetric walking pattern
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");

    // Impose prescribed average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_desired_speed(1.2);

    // Minimize squared control normalized by the distance travelled
    auto* controlCost =
        problem.addCost<MocoControlOverDistanceCost>("controlCost");
    controlCost->set_weight(1);

    // Adjust bounds
    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
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

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);
    solver.set_parallel(8);
    // Set Guess
    solver.setGuessFile(
        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");

    MocoSolution solution = moco.solve();
}

void testPredictive_withoutPassiveForces(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_withoutPassiveForce");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor(
        "gait10dof18musc_MusclePolynomials_withoutPassiveForces.osim");
    problem.setModelProcessor(modelprocessor);

    problem.setTimeBounds(0, {0.4,0.6});

    //// Cost.
    //// =====
    // Impose symmetric walking pattern
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");

    // Impose prescribed average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_desired_speed(1.2);

    // Minimize squared control normalized by the distance travelled
    auto* controlCost =
        problem.addCost<MocoControlOverDistanceCost>("controlCost");
    controlCost->set_weight(1);

    // Adjust bounds
    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
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

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);
    solver.set_parallel(8);
    // Set Guess
    solver.setGuessFile(
        "coordinateTracking_MusclePolynomials_withoutPassiveForces_solution.sto");

    MocoSolution solution = moco.solve();
}

void testPredictive_withPassiveForces_activationSquared(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_withPassiveForce_activationSquared");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor(
        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
    problem.setModelProcessor(modelprocessor);

    problem.setTimeBounds(0, {0.4,0.6});

    //// Cost.
    //// =====
    // Impose symmetric walking pattern
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");

    // Impose prescribed average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_desired_speed(1.2);

    // Minimize squared control normalized by the distance travelled
    auto* activationCost =
        problem.addCost<MocoActivationOverDistanceCost>("activationCost");
    activationCost->set_weight(1);

    // Adjust bounds
    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
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

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);
    solver.set_parallel(8);
    // Set Guess
    solver.setGuessFile(
        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");

    MocoSolution solution = moco.solve();
}

void testPredictive_withPassiveForces_accelerationSquared(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_withPassiveForce_accelerationSquared");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor(
        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
    problem.setModelProcessor(modelprocessor);

    problem.setTimeBounds(0, {0.4,0.6});

    //// Cost.
    //// =====
    // Impose symmetric walking pattern
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");

    // Impose prescribed average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_desired_speed(1.2);

    // Minimize squared control normalized by the distance travelled
    auto* controlCost =
        problem.addCost<MocoControlOverDistanceCost>("controlCost");
    controlCost->set_weight(1);

    // Minimize squared control normalized by the distance travelled
    auto* accelerationCost =
        problem.addCost<MocoAccelerationOverDistanceCost>("accelerationCost");
    accelerationCost->set_weight(1);

    // Adjust bounds
    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
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

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);
    solver.set_parallel(7);
    // Set Guess
    solver.setGuessFile(
        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");

    MocoSolution solution = moco.solve();
}

void testPredictive_withPassiveForces_Implicit(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_withPassiveForce_Implicit");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor(
        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
    problem.setModelProcessor(modelprocessor);

    problem.setTimeBounds(0, {0.4,0.6});

    //// Cost.
    //// =====
    // Impose symmetric walking pattern
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");

    // Impose prescribed average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_desired_speed(1.2);

    // Minimize squared control normalized by the distance travelled
    auto* controlCost =
        problem.addCost<MocoControlOverDistanceCost>("controlCost");
    controlCost->set_weight(1);

    // Adjust bounds
    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
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

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_dynamics_mode("implicit");
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);
    solver.set_parallel(7);
    // Set Guess
    MocoTrajectory guess = solver.createGuess();
    TableProcessor guessTable(
        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");
    guess.setStatesTrajectory(guessTable.process(),true,true);
    solver.setGuess(guess);

    MocoSolution solution = moco.solve();
}

void testPredictive_withPassiveForces_Implicit_Acceleration(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_withPassiveForce_Implicit_Acceleration");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor(
        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
    problem.setModelProcessor(modelprocessor);

    problem.setTimeBounds(0, {0.4,0.6});

    //// Cost.
    //// =====
    // Impose symmetric walking pattern
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");

    // Impose prescribed average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_desired_speed(1.2);

    // Minimize squared control normalized by the distance travelled
    auto* controlCost =
        problem.addCost<MocoControlOverDistanceCost>("controlCost");
    controlCost->set_weight(1);

    // Minimize squared accelerations normalized by the distance travelled
    auto* accelerationCost =
        problem.addCost<MocoAccelerationOverDistanceCost>("accelerationCost");
    accelerationCost->set_weight(100);

    // Adjust bounds
    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
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

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_dynamics_mode("implicit");
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);
    solver.set_parallel(6);
    // Set Guess
    MocoTrajectory guess = solver.createGuess();
    TableProcessor guessTable(
        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");
    guess.setStatesTrajectory(guessTable.process(),true,true);
    solver.setGuess(guess);

    MocoSolution solution = moco.solve();
}

void testPredictive_withPassiveForces_JRL(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_withPassiveForce");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor(
        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
    problem.setModelProcessor(modelprocessor);

    problem.setTimeBounds(0, {0.4,0.6});

    //// Cost.
    //// =====
    // Impose symmetric walking pattern
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");

    // Impose prescribed average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_desired_speed(1.2);

    // Minimize squared control normalized by the distance travelled
    auto* controlCost =
        problem.addCost<MocoControlOverDistanceCost>("controlCost");
    controlCost->set_weight(1);

    // Minimize joint reaction loads
    auto* loadCost_knee_r = problem.addCost<MocoJointReactionCost>();
    loadCost_knee_r->setName("tibiofemoral_compressive_force_r");
    loadCost_knee_r->setJointPath("/jointset/knee_r");
    loadCost_knee_r->setLoadsFrame("child");
    loadCost_knee_r->setExpressedInFramePath("/bodyset/tibia_r");
    loadCost_knee_r->setReactionMeasures({"force-y"});
    loadCost_knee_r->set_weight(1);
    auto* loadCost_knee_l = problem.addCost<MocoJointReactionCost>();
    loadCost_knee_l->setName("tibiofemoral_compressive_force_l");
    loadCost_knee_l->setJointPath("/jointset/knee_l");
    loadCost_knee_l->setLoadsFrame("child");
    loadCost_knee_l->setExpressedInFramePath("/bodyset/tibia_l");
    loadCost_knee_l->setReactionMeasures({"force-y"});
    loadCost_knee_l->set_weight(1);
    // Adjust bounds
    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
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

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);
    solver.set_parallel(8);
    // Set Guess
    solver.setGuessFile(
        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");

    MocoSolution solution = moco.solve();
}

void testPredictive_withPassiveForces_Implicit_Acceleration_Scaling(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_withPassiveForce_Implicit_Acceleration");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor(
        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
    problem.setModelProcessor(modelprocessor);

    problem.setTimeBounds(0, {0.4,0.6});

    //// Cost.
    //// =====
    // Impose symmetric walking pattern
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");

    // Impose prescribed average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_desired_speed(1.2);

    // Minimize squared control normalized by the distance travelled
    auto* controlCost =
        problem.addCost<MocoControlOverDistanceCost>("controlCost");
    controlCost->set_weight(1);

    // Minimize squared accelerations normalized by the distance travelled
    auto* accelerationCost =
        problem.addCost<MocoAccelerationOverDistanceCost>("accelerationCost");
    accelerationCost->set_weight(0.001);

    // Adjust bounds
    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
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

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_dynamics_mode("implicit");
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);
    solver.set_scale_variables_using_bounds(true);
    solver.set_parallel(6);
    // Set Guess
    MocoTrajectory guess = solver.createGuess();
    TableProcessor guessTable(
        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");
    guess.setStatesTrajectory(guessTable.process(),true,true);
    solver.setGuess(guess);

    MocoSolution solution = moco.solve();
}

int main() {
    //testPredictive_withPassiveForces();
    //testPredictive_withoutPassiveForces();
    //testPredictive_withPassiveForces_activationSquared();
    //testPredictive_withPassiveForces_accelerationSquared();
    //testPredictive_withPassiveForces_Implicit();
    //testPredictive_withPassiveForces_JRL();
    //testPredictive_withPassiveForces_Implicit_Acceleration();
    testPredictive_withPassiveForces_Implicit_Acceleration_Scaling();
}
