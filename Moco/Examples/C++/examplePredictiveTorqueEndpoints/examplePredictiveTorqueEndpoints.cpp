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

        //// Initial states: coordinate actuators (all)
        //SimTK::Real control_hip_l_IS =
        //    m_coordAct_hip_l->getControl(input.initial_state);
        //SimTK::Real control_hip_r_IS =
        //    m_coordAct_hip_r->getControl(input.initial_state);
        //SimTK::Real control_knee_l_IS =
        //    m_coordAct_knee_l->getControl(input.initial_state);
        //SimTK::Real control_knee_r_IS =
        //    m_coordAct_knee_r->getControl(input.initial_state);
        //SimTK::Real control_ankle_l_IS =
        //    m_coordAct_ankle_l->getControl(input.initial_state);
        //SimTK::Real control_ankle_r_IS =
        //    m_coordAct_ankle_r->getControl(input.initial_state);
        //SimTK::Real control_lumbar_IS =
        //    m_coordAct_lumbar->getControl(input.initial_state);

        //// Final states: coordinate actuators (all)
        //SimTK::Real control_hip_l_FS =
        //    m_coordAct_hip_l->getControl(input.final_state);
        //SimTK::Real control_hip_r_FS =
        //    m_coordAct_hip_r->getControl(input.final_state);
        //SimTK::Real control_knee_l_FS =
        //    m_coordAct_knee_l->getControl(input.final_state);
        //SimTK::Real control_knee_r_FS =
        //    m_coordAct_knee_r->getControl(input.final_state);
        //SimTK::Real control_ankle_l_FS =
        //    m_coordAct_ankle_l->getControl(input.final_state);
        //SimTK::Real control_ankle_r_FS =
        //    m_coordAct_ankle_r->getControl(input.final_state);
        //SimTK::Real control_lumbar_FS =
        //    m_coordAct_lumbar->getControl(input.final_state);

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
        //// Squared differences between initial and final coordinate actuator
        //// controls. The difference is between the right and left legs since
        //// the simulation is for half a gait cycle.
        //cost += SimTK::square(control_hip_l_IS - control_hip_r_FS) +
        //    SimTK::square(control_hip_r_IS - control_hip_l_FS) +
        //    SimTK::square(control_knee_l_IS - control_knee_r_FS) +
        //    SimTK::square(control_knee_r_IS - control_knee_l_FS) +
        //    SimTK::square(control_ankle_l_IS - control_ankle_r_FS) +
        //    SimTK::square(control_ankle_r_IS - control_ankle_l_FS);

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
        //m_coordAct_hip_l.reset(
        //    &model.getComponent<CoordinateActuator>("hipAct_l"));
        //m_coordAct_hip_r.reset(
        //    &model.getComponent<CoordinateActuator>("hipAct_r"));
        //m_coordAct_knee_l.reset(
        //    &model.getComponent<CoordinateActuator>("kneeAct_l"));
        //m_coordAct_knee_r.reset(
        //    &model.getComponent<CoordinateActuator>("kneeAct_r"));
        //m_coordAct_ankle_l.reset(
        //    &model.getComponent<CoordinateActuator>("ankleAct_l"));
        //m_coordAct_ankle_r.reset(
        //    &model.getComponent<CoordinateActuator>("ankleAct_r"));
        //m_coordAct_lumbar.reset(
        //    &model.getComponent<CoordinateActuator>("lumbarAct"));

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
    //mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_hip_l;
    //mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_hip_r;
    //mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_knee_l;
    //mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_knee_r;
    //mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_ankle_l;
    //mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_ankle_r;
    //mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_lumbar;
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
    MocoAverageSpeedCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {
        constructProperties();
    }
protected:
    void calcCostImpl(const CostInput& input, SimTK::Real& cost)
        const override {
        // get final time
        SimTK::Real time = input.final_state.getTime();
        // get initial and final pelvis forward coordinate values
        SimTK::Real position_IS =  m_coord->getValue(input.initial_state);
        SimTK::Real position_FS =  m_coord->getValue(input.final_state);
        SimTK::Real distanceTravelled = position_FS - position_IS;
        cost = SimTK::square(get_desired_speed() - (distanceTravelled / time));
    }
    void initializeOnModelImpl(const Model& model) const {
        m_coord.reset(&model.getCoordinateSet().get("groundPelvis_q_tx"));
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
protected:
    void calcCostImpl(const CostInput& input, SimTK::Real& cost)
        const override {
        // get initial and final pelvis forward coordinate values
        SimTK::Real position_IS =  m_coord->getValue(input.initial_state);
        SimTK::Real position_FS =  m_coord->getValue(input.final_state);
        SimTK::Real distanceTravelled = position_FS - position_IS;
        // normalize integral by distance travelled
        cost = input.integral / distanceTravelled ;
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
        m_coord.reset(&model.getCoordinateSet().get("groundPelvis_q_tx"));
    }
private:
    mutable SimTK::ReferencePtr<const Coordinate> m_coord;
};



void examplePredictiveTorqueEndpoints(){

    MocoStudy moco;
    moco.setName("examplePredictiveTorqueEndpoints");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor(
        "gait_2D_contact_torque_bounds_noPelvisRes.osim");
    problem.setModelProcessor(modelprocessor);

    problem.setTimeBounds(0, {0.4,0.6});

    //// Cost.
    //// =====
    // Minimize deviation from symmetric walking pattern
    auto* symmetryCost = problem.addCost<MocoSymmetryCost>("symmetryCost");
    symmetryCost->set_weight(1000);

    // Minimize deviation for prescribed average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_weight(1000);
    speedCost->set_desired_speed(1.2);

    // Minimize squared control normalized by the distance travelled
    auto* controlCost =
        problem.addCost<MocoControlOverDistanceCost>("controlCost");
    controlCost->set_weight(1);

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);
    solver.set_parallel(4);
    // Set Guess
    solver.setGuessFile("exampleTrackingTorqueEndpoints_solution.sto");

    MocoSolution solution = moco.solve();
    solution.write("examplePredictiveTorqueEndpoints_solution.sto");
    moco.visualize(solution);
}

int main()  {
    try { examplePredictiveTorqueEndpoints(); }
    catch (const OpenSim::Exception& e) {
        e.print(std::cerr);
        return 1;
    }
    return 0;
}
