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
// MocoGoal imposing the average gait speed through endpoint constraints. The
// average gait speed is defined as the distance travelled by the pelvis in the
// forward direction divided by the final time.
class MocoAverageSpeedGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoAverageSpeedGoal, MocoGoal);
public:
    OpenSim_DECLARE_PROPERTY(desired_speed, double,
            "The desired gait speed defined as the distance travelled by "
            "the pelvis in the forward direction divided by the final time.");
    MocoAverageSpeedGoal() {
        constructProperties();
    }
    MocoAverageSpeedGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void calcGoalImpl(const GoalInput& input, SimTK::Vector& values)
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
        setNumIntegralsAndOutputs(0, 1);
    }
private:
    void constructProperties() {
        constructProperty_desired_speed(0);
    }
    mutable SimTK::ReferencePtr<const Coordinate> m_coord;
};

// MocoGoal minimizing the integral of the squared controls divided by the
// distance travelled by the pelvis in the forward direction.
class MocoControlOverDistanceGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoControlOverDistanceGoal, MocoGoal);
public:
    MocoControlOverDistanceGoal() = default;
    MocoControlOverDistanceGoal(std::string name)
            : MocoGoal(std::move(name)) {}
    MocoControlOverDistanceGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {}
protected:
    void calcGoalImpl(const GoalInput& input, SimTK::Vector& goal)
        const override {
        // get initial and final pelvis forward coordinate values
        SimTK::Real position_IS =  m_coord->getValue(input.initial_state);
        SimTK::Real position_FS =  m_coord->getValue(input.final_state);
        SimTK::Real distanceTravelled = position_FS - position_IS;
        // normalize integral by distance travelled
        goal[0] = input.integral / distanceTravelled ;
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
        setNumIntegralsAndOutputs(1, 1);
    }
private:
    mutable SimTK::ReferencePtr<const Coordinate> m_coord;
};

// This class defines a MocoGoal that computes the integral of the squared
// muscle activations + squared coordinate actuator controls divided by the
// distance travelled by the pelvis in the forward direction.
//class MocoActivationOverDistanceGoal : public MocoGoal {
//OpenSim_DECLARE_CONCRETE_OBJECT(MocoActivationOverDistanceGoal, MocoGoal);
//public:
//    MocoActivationOverDistanceGoal() = default;
//    MocoActivationOverDistanceGoal(std::string name)
//            : MocoGoal(std::move(name)) {}
//    MocoActivationOverDistanceGoal(std::string name, double weight)
//            : MocoGoal(std::move(name), weight) {}
//protected:
//    void calcGoalImpl(const GoalInput& input, SimTK::Vector& Goal)
//        const override {
//        // get initial and final pelvis forward coordinate values
//        SimTK::Real position_IS =  m_coord->getValue(input.initial_state);
//        SimTK::Real position_FS =  m_coord->getValue(input.final_state);
//        SimTK::Real distanceTravelled = position_FS - position_IS;
//        // normalize integral by distance travelled
//        Goal[0] = input.integral / distanceTravelled ;
//    }
//    void calcIntegrandImpl(
//        const SimTK::State& state, double& integrand) const {
//        // Coordinate actuator
//        SimTK::Real control_lumbar =  m_coordAct_lumbar->getControl(state);
//        // Muscles
//        // Right
//        SimTK::Real state_hamstrings_r =
//                m_hamstrings_r->getStateVariableValue(state, "activation");
//        SimTK::Real state_bifemsh_r =
//                m_bifemsh_r->getStateVariableValue(state, "activation");
//        SimTK::Real state_glut_max_r =
//                m_glut_max_r->getStateVariableValue(state, "activation");
//        SimTK::Real state_iliopsoas_r =
//                m_iliopsoas_r->getStateVariableValue(state, "activation");
//        SimTK::Real state_rect_fem_r =
//                m_rect_fem_r->getStateVariableValue(state, "activation");
//        SimTK::Real state_vasti_r =
//                m_vasti_r->getStateVariableValue(state, "activation");
//        SimTK::Real state_gastroc_r =
//                m_gastroc_r->getStateVariableValue(state, "activation");
//        SimTK::Real state_soleus_r =
//                m_soleus_r->getStateVariableValue(state, "activation");
//        SimTK::Real state_tib_ant_r =
//                m_tib_ant_r->getStateVariableValue(state, "activation");
//        // Left
//        SimTK::Real state_hamstrings_l =
//                m_hamstrings_l->getStateVariableValue(state, "activation");
//        SimTK::Real state_bifemsh_l =
//                m_bifemsh_l->getStateVariableValue(state, "activation");
//        SimTK::Real state_glut_max_l =
//                m_glut_max_l->getStateVariableValue(state, "activation");
//        SimTK::Real state_iliopsoas_l =
//                m_iliopsoas_l->getStateVariableValue(state, "activation");
//        SimTK::Real state_rect_fem_l =
//                m_rect_fem_l->getStateVariableValue(state, "activation");
//        SimTK::Real state_vasti_l =
//                m_vasti_l->getStateVariableValue(state, "activation");
//        SimTK::Real state_gastroc_l =
//                m_gastroc_l->getStateVariableValue(state, "activation");
//        SimTK::Real state_soleus_l =
//                m_soleus_l->getStateVariableValue(state, "activation");
//        SimTK::Real state_tib_ant_l =
//                m_tib_ant_l->getStateVariableValue(state, "activation");
//        // integrand is squared coordinate actuator control + squared muscle
//        // states (i.e., muscle activations)
//        integrand = 0;
//        integrand += SimTK::square(control_lumbar);
//        integrand += SimTK::square(state_hamstrings_r);
//        integrand += SimTK::square(state_bifemsh_r);
//        integrand += SimTK::square(state_glut_max_r);
//        integrand += SimTK::square(state_iliopsoas_r);
//        integrand += SimTK::square(state_rect_fem_r);
//        integrand += SimTK::square(state_vasti_r);
//        integrand += SimTK::square(state_gastroc_r);
//        integrand += SimTK::square(state_soleus_r);
//        integrand += SimTK::square(state_tib_ant_r);
//        integrand += SimTK::square(state_hamstrings_l);
//        integrand += SimTK::square(state_bifemsh_l);
//        integrand += SimTK::square(state_glut_max_l);
//        integrand += SimTK::square(state_iliopsoas_l);
//        integrand += SimTK::square(state_rect_fem_l);
//        integrand += SimTK::square(state_vasti_l);
//        integrand += SimTK::square(state_gastroc_l);
//        integrand += SimTK::square(state_soleus_l);
//        integrand += SimTK::square(state_tib_ant_l);
//    }
//    void initializeOnModelImpl(const Model& model) const {
//        // Coordinates
//        m_coord.reset(&model.getCoordinateSet().get("pelvis_tx"));
//        // Coordinate actuators
//        m_coordAct_lumbar.reset(
//                &model.getComponent<CoordinateActuator>("lumbarAct"));
//        // Muscles
//        // Right
//        m_hamstrings_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "hamstrings_r"));
//        m_bifemsh_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "bifemsh_r"));
//        m_glut_max_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "glut_max_r"));
//        m_iliopsoas_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "iliopsoas_r"));
//        m_rect_fem_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "rect_fem_r"));
//        m_vasti_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "vasti_r"));
//        m_gastroc_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "gastroc_r"));
//        m_soleus_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "soleus_r"));
//        m_tib_ant_r.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "tib_ant_r"));
//        // Left
//        m_hamstrings_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "hamstrings_l"));
//        m_bifemsh_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "bifemsh_l"));
//        m_glut_max_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "glut_max_l"));
//        m_iliopsoas_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "iliopsoas_l"));
//        m_rect_fem_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "rect_fem_l"));
//        m_vasti_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "vasti_l"));
//        m_gastroc_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "gastroc_l"));
//        m_soleus_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "soleus_l"));
//        m_tib_ant_l.reset(&model.getComponent<DeGrooteFregly2016Muscle>(
//                "tib_ant_l"));
//    }
//private:
//    // Coordinates
//    mutable SimTK::ReferencePtr<const Coordinate> m_coord;
//    // Coordinate actuators
//    mutable SimTK::ReferencePtr<const CoordinateActuator> m_coordAct_lumbar;
//    // Muscles
//    // Right
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_hamstrings_r;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_bifemsh_r;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_glut_max_r;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_iliopsoas_r;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_rect_fem_r;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_vasti_r;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_gastroc_r;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_soleus_r;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_tib_ant_r;
//    // Left
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_hamstrings_l;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_bifemsh_l;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_glut_max_l;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_iliopsoas_l;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_rect_fem_l;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_vasti_l;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_gastroc_l;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_soleus_l;
//    mutable SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_tib_ant_l;
////};
//
//// This class defines a MocoGoal that computes the integral of the squared
//// muscle activations + squared coordinate actuator controls divided by the
//// distance travelled by the pelvis in the forward direction.
//class MocoAccelerationOverDistanceGoal : public MocoGoal {
//OpenSim_DECLARE_CONCRETE_OBJECT(MocoAccelerationOverDistanceGoal, MocoGoal);
//public:
//    MocoAccelerationOverDistanceGoal() = default;
//    MocoAccelerationOverDistanceGoal(std::string name)
//            : MocoGoal(std::move(name)) {}
//    MocoAccelerationOverDistanceGoal(std::string name, double weight)
//            : MocoGoal(std::move(name), weight) {}
//    int getNumOutputsImpl() const override { return 1; }
//    int getNumIntegralsImpl() const override { return 1; }
//protected:
//    void calcGoalImpl(const GoalInput& input, SimTK::Vector& Goal)
//        const override {
//        // get initial and final pelvis forward coordinate values
//        SimTK::Real position_IS =  m_coord_pelvis_tx->getValue(
//                input.initial_state);
//        SimTK::Real position_FS =  m_coord_pelvis_tx->getValue(
//                input.final_state);
//        SimTK::Real distanceTravelled = position_FS - position_IS;
//        // normalize integral by distance travelled
//        Goal[0] = input.integral / distanceTravelled ;
//    }
//    void calcIntegrandImpl(
//        const SimTK::State& state, double& integrand) const {
//
//        getModel().realizeAcceleration(state);
//        const auto& uDots = state.getUDot();
//        integrand = 0;
//        for (int i = 0; i < getModel().getNumCoordinates(); ++i)
//            integrand += SimTK::square(uDots[i]);
//    }
//    void initializeOnModelImpl(const Model& model) const {
//        // Coordinates
//        m_coord_pelvis_tx.reset(
//                &model.getCoordinateSet().get("pelvis_tx"));
//    }
//private:
//    // Coordinates
//    mutable SimTK::ReferencePtr<const Coordinate> m_coord_pelvis_tx;
//};
//
//void testPredictive_withPassiveForces(){
//
//    MoGoaludy moco;
//    moco.setName("2DGaitPrediction_withPassiveForce");
//
//    // Define the optimal control problem.
//    // ===================================
//    MocoProblem& problem = moco.updProblem();
//    ModelProcessor modelprocessor = ModelProcessor(
//        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
//    problem.setModelProcessor(modelprocessor);
//
//    problem.setTimeBounds(0, {0.4,0.6});
//
//    //// Goal.
//    //// =====
//    // Impose symmetric walking pattern
//    auto* symmetryGoal = problem.addGoal<MocoSymmetryGoal>("symmetryGoal");
//
//    // Impose prescribed average speed
//    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speedGoal");
//    speedGoal->set_desired_speed(1.2);
//
//    // Minimize squared control normalized by the distance travelled
//    auto* controlGoal =
//        problem.addGoal<MocoControlOverDistanceGoal>("controlGoal");
//    controlGoal->set_weight(1);
//
//    // Adjust bounds
//    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
//            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
//            {0.75,1.25});
//    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/lumbar/lumbar/value",
//            {0,20*SimTK::Pi/180});
//
//    // Configure the solver.
//    // =====================
//    auto& solver = moco.initCasADiSolver();
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(1e-4);
//    solver.set_optim_constraint_tolerance(1e-4);
//    solver.set_optim_max_iterations(10000);
//    solver.set_parallel(8);
//    // Set Guess
//    solver.setGuessFile(
//        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");
//
//    MocoSolution solution = moco.solve();
//}

void example2DPrediction_Polynomial(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_Polynomial");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor("gait10dof18musc.osim");
    problem.setModelProcessor(modelprocessor);

    // Goal.
    // =====
    // Symmetry
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    // Coordinate values
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/value"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/value"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/value",
            "/jointset/hip_r/hip_flexion_r/value"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/value",
            "/jointset/hip_l/hip_flexion_l/value"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/value",
            "/jointset/knee_r/knee_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/value",
            "/jointset/knee_l/knee_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/value",
            "/jointset/ankle_r/ankle_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/value",
            "/jointset/ankle_l/ankle_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/value"});
    // Coordinate speeds
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tx/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/speed",
            "/jointset/hip_r/hip_flexion_r/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/speed",
            "/jointset/hip_l/hip_flexion_l/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/speed",
            "/jointset/knee_r/knee_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/speed",
            "/jointset/knee_l/knee_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/speed",
            "/jointset/ankle_r/ankle_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/speed",
            "/jointset/ankle_l/ankle_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/speed"});
    // Coordinate actuators
    symmetryGoal->addControlPair({"/lumbarAct"});
    // Muscle activations
    symmetryGoal->addStatePair({"/hamstrings_l/activation",
            "/hamstrings_r/activation"});
    symmetryGoal->addStatePair({"/hamstrings_r/activation",
            "/hamstrings_l/activation"});
    symmetryGoal->addStatePair({"/bifemsh_l/activation",
            "/bifemsh_r/activation"});
    symmetryGoal->addStatePair({"/bifemsh_r/activation",
            "/bifemsh_l/activation"});
    symmetryGoal->addStatePair({"/glut_max_l/activation",
            "/glut_max_r/activation"});
    symmetryGoal->addStatePair({"/glut_max_r/activation",
            "/glut_max_l/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_l/activation",
            "/iliopsoas_r/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_r/activation",
            "/iliopsoas_l/activation"});
    symmetryGoal->addStatePair({"/rect_fem_l/activation",
            "/rect_fem_r/activation"});
    symmetryGoal->addStatePair({"/rect_fem_r/activation",
            "/rect_fem_l/activation"});
    symmetryGoal->addStatePair({"/vasti_l/activation",
            "/vasti_r/activation"});
    symmetryGoal->addStatePair({"/vasti_r/activation",
            "/vasti_l/activation"});
    symmetryGoal->addStatePair({"/gastroc_l/activation",
            "/gastroc_r/activation"});
    symmetryGoal->addStatePair({"/gastroc_r/activation",
            "/gastroc_l/activation"});
    symmetryGoal->addStatePair({"/soleus_l/activation",
            "/soleus_r/activation"});
    symmetryGoal->addStatePair({"/soleus_r/activation",
            "/soleus_l/activation"});
    symmetryGoal->addStatePair({"/tib_ant_l/activation",
            "/tib_ant_r/activation"});
    symmetryGoal->addStatePair({"/tib_ant_r/activation",
            "/tib_ant_l/activation"});

    // Prescribed average gait speed
    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speedGoal");
    speedGoal->set_desired_speed(1.2);

    // Minimize squared controls normalized by distance travelled
    auto* controlGoal =
        problem.addGoal<MocoControlOverDistanceGoal>("controlGoal");
    controlGoal->setWeight(10);

    // Adjust bounds
    problem.setTimeBounds(0, {0.4,0.6});

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
    solver.setGuessFile("coordinateTracking_Muscles_solution.sto");
    // Solve problem
    MocoSolution solution = moco.solve();

    // Extract ground reaction forces
    std::vector<std::string> contactSpheres_r;
    std::vector<std::string> contactSpheres_l;
    // What users would provide
    contactSpheres_r.push_back("contactSphereHeel_r");
    contactSpheres_r.push_back("contactSphereFront_r");
    contactSpheres_l.push_back("contactSphereHeel_l");
    contactSpheres_l.push_back("contactSphereFront_l");


    TimeSeriesTable externalForcesTableFlat =
        moco.getSmoothSphereHalfSpaceForce(solution,contactSpheres_r,contactSpheres_l);

    //// Extract ground reaction forces
    //std::vector<std::string> contactSpheres_r;
    //std::vector<std::string> contactSpheres_l;
    //// What users would provide
    //contactSpheres_r.push_back("contactSphereHeel_r");
    //contactSpheres_r.push_back("contactSphereFront_r");
    //contactSpheres_l.push_back("contactSphereHeel_l");
    //contactSpheres_l.push_back("contactSphereFront_l");
    //// Extract forces
    //auto model = modelprocessor.process();
    //model.initSystem();
    //problem.setModelCopy(model);
    //TimeSeriesTableVec3 externalForcesTable{};
    //StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
    //SimTK::Vector optTime = solution.getTime();
    //int count = 0;
    //for (const auto& state : optStates) {
    //    model.realizeVelocity(state);

    //    SimTK::Vec3 forces_r(0);
    //    SimTK::Vec3 torques_r(0);
    //    for (const auto& contactSphere_r : contactSpheres_r) {
    //        Array<double> forcesContactSphere_r = model.getComponent<
    //                SmoothSphereHalfSpaceForce>(
    //                        contactSphere_r).getRecordValues(state);
    //        forces_r += SimTK::Vec3(forcesContactSphere_r[0],
    //            forcesContactSphere_r[1], forcesContactSphere_r[2]);
    //        torques_r += SimTK::Vec3(forcesContactSphere_r[3],
    //            forcesContactSphere_r[4], forcesContactSphere_r[5]);
    //    }
    //    SimTK::Vec3 forces_l(0);
    //    SimTK::Vec3 torques_l(0);
    //    for (const auto& contactSphere_l : contactSpheres_l) {
    //        Array<double> forcesContactSphere_l = model.getComponent<
    //                SmoothSphereHalfSpaceForce>(
    //                        contactSphere_l).getRecordValues(state);
    //        forces_l += SimTK::Vec3(forcesContactSphere_l[0],
    //            forcesContactSphere_l[1], forcesContactSphere_l[2]);
    //        torques_l += SimTK::Vec3(forcesContactSphere_l[3],
    //            forcesContactSphere_l[4], forcesContactSphere_l[5]);
    //    }
    //    // Combine in a row
    //    SimTK::RowVector_<SimTK::Vec3> row(6);
    //    row(0) = forces_r;
    //    row(1) = SimTK::Vec3(0);
    //    row(2) = forces_l;
    //    row(3) = SimTK::Vec3(0);
    //    row(4) = torques_r;
    //    row(5) = torques_l;
    //    // Append to a table
    //    externalForcesTable.appendRow(optTime[count],row);
    //    ++count;
    //}
    //// Write file
    //// Create labels for output file
    //std::vector<std::string> labels;
    //labels.push_back("ground_force_v");
    //labels.push_back("ground_force_p");
    //labels.push_back("1_ground_force_v");
    //labels.push_back("1_ground_force_p");
    //labels.push_back("ground_torque_");
    //labels.push_back("1_ground_torque_");
    //std::vector<std::string> suffixes;
    //suffixes.push_back("x");
    //suffixes.push_back("y");
    //suffixes.push_back("z");
    //externalForcesTable.setColumnLabels(labels);
    //TimeSeriesTable externalForcesTableFlat =
    //        externalForcesTable.flatten(suffixes);
    DataAdapter::InputTables tables = {{"table", &externalForcesTableFlat}};
    FileAdapter::writeFile(tables,
            "2DGaitPrediction_Polynomial_GRF.sto");
}

void example2DPrediction_GeometryPath(){

    MocoStudy moco;
    moco.setName("2DGaitPrediction_GeometryPath");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();
    ModelProcessor modelprocessor = ModelProcessor("gait10dof18musc.osim") |
        ModOpUsePathLengthApproximation(false);
    problem.setModelProcessor(modelprocessor);

    // Goal.
    // =====
    // Symmetry
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    // Coordinate values
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/value"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/value"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/value",
            "/jointset/hip_r/hip_flexion_r/value"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/value",
            "/jointset/hip_l/hip_flexion_l/value"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/value",
            "/jointset/knee_r/knee_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/value",
            "/jointset/knee_l/knee_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/value",
            "/jointset/ankle_r/ankle_angle_r/value"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/value",
            "/jointset/ankle_l/ankle_angle_l/value"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/value"});
    // Coordinate speeds
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tilt/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tx/speed"});
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_ty/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/speed",
            "/jointset/hip_r/hip_flexion_r/speed"});
    symmetryGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/speed",
            "/jointset/hip_l/hip_flexion_l/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_l/knee_angle_l/speed",
            "/jointset/knee_r/knee_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/knee_r/knee_angle_r/speed",
            "/jointset/knee_l/knee_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_l/ankle_angle_l/speed",
            "/jointset/ankle_r/ankle_angle_r/speed"});
    symmetryGoal->addStatePair({"/jointset/ankle_r/ankle_angle_r/speed",
            "/jointset/ankle_l/ankle_angle_l/speed"});
    symmetryGoal->addStatePair({"/jointset/lumbar/lumbar/speed"});
    // Coordinate actuators
    symmetryGoal->addControlPair({"/lumbarAct"});
    // Muscle activations
    symmetryGoal->addStatePair({"/hamstrings_l/activation",
            "/hamstrings_r/activation"});
    symmetryGoal->addStatePair({"/hamstrings_r/activation",
            "/hamstrings_l/activation"});
    symmetryGoal->addStatePair({"/bifemsh_l/activation",
            "/bifemsh_r/activation"});
    symmetryGoal->addStatePair({"/bifemsh_r/activation",
            "/bifemsh_l/activation"});
    symmetryGoal->addStatePair({"/glut_max_l/activation",
            "/glut_max_r/activation"});
    symmetryGoal->addStatePair({"/glut_max_r/activation",
            "/glut_max_l/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_l/activation",
            "/iliopsoas_r/activation"});
    symmetryGoal->addStatePair({"/iliopsoas_r/activation",
            "/iliopsoas_l/activation"});
    symmetryGoal->addStatePair({"/rect_fem_l/activation",
            "/rect_fem_r/activation"});
    symmetryGoal->addStatePair({"/rect_fem_r/activation",
            "/rect_fem_l/activation"});
    symmetryGoal->addStatePair({"/vasti_l/activation",
            "/vasti_r/activation"});
    symmetryGoal->addStatePair({"/vasti_r/activation",
            "/vasti_l/activation"});
    symmetryGoal->addStatePair({"/gastroc_l/activation",
            "/gastroc_r/activation"});
    symmetryGoal->addStatePair({"/gastroc_r/activation",
            "/gastroc_l/activation"});
    symmetryGoal->addStatePair({"/soleus_l/activation",
            "/soleus_r/activation"});
    symmetryGoal->addStatePair({"/soleus_r/activation",
            "/soleus_l/activation"});
    symmetryGoal->addStatePair({"/tib_ant_l/activation",
            "/tib_ant_r/activation"});
    symmetryGoal->addStatePair({"/tib_ant_r/activation",
            "/tib_ant_l/activation"});

    // Prescribed average gait speed
    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speedGoal");
    speedGoal->set_desired_speed(1.2);

    // Minimize squared controls normalized by distance travelled
    auto* controlGoal =
        problem.addGoal<MocoControlOverDistanceGoal>("controlGoal");
    controlGoal->setWeight(10);

    // Adjust bounds
    problem.setTimeBounds(0, {0.4,0.6});

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
    solver.setGuessFile("coordinateTracking_Muscles_solution.sto");
    // Solve problem
    MocoSolution solution = moco.solve();

    // Extract ground reaction forces
    std::vector<std::string> contactSpheres_r;
    std::vector<std::string> contactSpheres_l;
    // What users would provide
    contactSpheres_r.push_back("contactSphereHeel_r");
    contactSpheres_r.push_back("contactSphereFront_r");
    contactSpheres_l.push_back("contactSphereHeel_l");
    contactSpheres_l.push_back("contactSphereFront_l");
    // Extract forces
    auto model = modelprocessor.process();
    model.initSystem();
    problem.setModelCopy(model);
    TimeSeriesTableVec3 externalForcesTable{};
    StatesTrajectory optStates = solution.exportToStatesTrajectory(problem);
    SimTK::Vector optTime = solution.getTime();
    int count = 0;
    for (const auto& state : optStates) {
        model.realizeVelocity(state);

        SimTK::Vec3 forces_r(0);
        SimTK::Vec3 torques_r(0);
        for (const auto& contactSphere_r : contactSpheres_r) {
            Array<double> forcesContactSphere_r = model.getComponent<
                    SmoothSphereHalfSpaceForce>(
                            contactSphere_r).getRecordValues(state);
            forces_r += SimTK::Vec3(forcesContactSphere_r[0],
                forcesContactSphere_r[1], forcesContactSphere_r[2]);
            torques_r += SimTK::Vec3(forcesContactSphere_r[3],
                forcesContactSphere_r[4], forcesContactSphere_r[5]);
        }
        SimTK::Vec3 forces_l(0);
        SimTK::Vec3 torques_l(0);
        for (const auto& contactSphere_l : contactSpheres_l) {
            Array<double> forcesContactSphere_l = model.getComponent<
                    SmoothSphereHalfSpaceForce>(
                            contactSphere_l).getRecordValues(state);
            forces_l += SimTK::Vec3(forcesContactSphere_l[0],
                forcesContactSphere_l[1], forcesContactSphere_l[2]);
            torques_l += SimTK::Vec3(forcesContactSphere_l[3],
                forcesContactSphere_l[4], forcesContactSphere_l[5]);
        }
        // Combine in a row
        SimTK::RowVector_<SimTK::Vec3> row(6);
        row(0) = forces_r;
        row(1) = SimTK::Vec3(0);
        row(2) = forces_l;
        row(3) = SimTK::Vec3(0);
        row(4) = torques_r;
        row(5) = torques_l;
        // Append to a table
        externalForcesTable.appendRow(optTime[count],row);
        ++count;
    }
    // Write file
    // Create labels for output file
    std::vector<std::string> labels;
    labels.push_back("ground_force_v");
    labels.push_back("ground_force_p");
    labels.push_back("1_ground_force_v");
    labels.push_back("1_ground_force_p");
    labels.push_back("ground_torque_");
    labels.push_back("1_ground_torque_");
    std::vector<std::string> suffixes;
    suffixes.push_back("x");
    suffixes.push_back("y");
    suffixes.push_back("z");
    externalForcesTable.setColumnLabels(labels);
    TimeSeriesTable externalForcesTableFlat =
            externalForcesTable.flatten(suffixes);
    DataAdapter::InputTables tables = {{"table", &externalForcesTableFlat}};
    FileAdapter::writeFile(tables,
            "2DGaitPrediction_GeometryPath_GRF.sto");
}




//void testPredictive_withPassiveForces_activationSquared(){
//
//    MoGoaludy moco;
//    moco.setName("2DGaitPrediction_withPassiveForce_activationSquared");
//
//    // Define the optimal control problem.
//    // ===================================
//    MocoProblem& problem = moco.updProblem();
//    ModelProcessor modelprocessor = ModelProcessor(
//        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
//    problem.setModelProcessor(modelprocessor);
//
//    problem.setTimeBounds(0, {0.4,0.6});
//
//    //// Goal.
//    //// =====
//    // Impose symmetric walking pattern
//    auto* symmetryGoal = problem.addGoal<MocoSymmetryGoal>("symmetryGoal");
//
//    // Impose prescribed average speed
//    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speedGoal");
//    speedGoal->set_desired_speed(1.2);
//
//    // Minimize squared control normalized by the distance travelled
//    auto* activationGoal =
//        problem.addGoal<MocoActivationOverDistanceGoal>("activationGoal");
//    activationGoal->set_weight(1);
//
//    // Adjust bounds
//    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
//            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
//            {0.75,1.25});
//    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/lumbar/lumbar/value",
//            {0,20*SimTK::Pi/180});
//
//    // Configure the solver.
//    // =====================
//    auto& solver = moco.initCasADiSolver();
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(1e-4);
//    solver.set_optim_constraint_tolerance(1e-4);
//    solver.set_optim_max_iterations(10000);
//    solver.set_parallel(8);
//    // Set Guess
//    solver.setGuessFile(
//        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");
//
//    MocoSolution solution = moco.solve();
//}
//
//void testPredictive_withPassiveForces_accelerationSquared(){
//
//    MoGoaludy moco;
//    moco.setName("2DGaitPrediction_withPassiveForce_accelerationSquared");
//
//    // Define the optimal control problem.
//    // ===================================
//    MocoProblem& problem = moco.updProblem();
//    ModelProcessor modelprocessor = ModelProcessor(
//        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
//    problem.setModelProcessor(modelprocessor);
//
//    problem.setTimeBounds(0, {0.4,0.6});
//
//    //// Goal.
//    //// =====
//    // Impose symmetric walking pattern
//    auto* symmetryGoal = problem.addGoal<MocoSymmetryGoal>("symmetryGoal");
//
//    // Impose prescribed average speed
//    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speedGoal");
//    speedGoal->set_desired_speed(1.2);
//
//    // Minimize squared control normalized by the distance travelled
//    auto* controlGoal =
//        problem.addGoal<MocoControlOverDistanceGoal>("controlGoal");
//    controlGoal->set_weight(1);
//
//    // Minimize squared control normalized by the distance travelled
//    auto* accelerationGoal =
//        problem.addGoal<MocoAccelerationOverDistanceGoal>("accelerationGoal");
//    accelerationGoal->set_weight(1);
//
//    // Adjust bounds
//    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
//            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
//            {0.75,1.25});
//    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/lumbar/lumbar/value",
//            {0,20*SimTK::Pi/180});
//
//    // Configure the solver.
//    // =====================
//    auto& solver = moco.initCasADiSolver();
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(1e-4);
//    solver.set_optim_constraint_tolerance(1e-4);
//    solver.set_optim_max_iterations(10000);
//    solver.set_parallel(7);
//    // Set Guess
//    solver.setGuessFile(
//        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");
//
//    MocoSolution solution = moco.solve();
//}
//
//void testPredictive_withPassiveForces_Implicit(){
//
//    MoGoaludy moco;
//    moco.setName("2DGaitPrediction_withPassiveForce_Implicit");
//
//    // Define the optimal control problem.
//    // ===================================
//    MocoProblem& problem = moco.updProblem();
//    ModelProcessor modelprocessor = ModelProcessor(
//        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
//    problem.setModelProcessor(modelprocessor);
//
//    problem.setTimeBounds(0, {0.4,0.6});
//
//    //// Goal.
//    //// =====
//    // Impose symmetric walking pattern
//    auto* symmetryGoal = problem.addGoal<MocoSymmetryGoal>("symmetryGoal");
//
//    // Impose prescribed average speed
//    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speedGoal");
//    speedGoal->set_desired_speed(1.2);
//
//    // Minimize squared control normalized by the distance travelled
//    auto* controlGoal =
//        problem.addGoal<MocoControlOverDistanceGoal>("controlGoal");
//    controlGoal->set_weight(1);
//
//    // Adjust bounds
//    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
//            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
//            {0.75,1.25});
//    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/lumbar/lumbar/value",
//            {0,20*SimTK::Pi/180});
//
//    // Configure the solver.
//    // =====================
//    auto& solver = moco.initCasADiSolver();
//    solver.set_dynamics_mode("implicit");
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(1e-4);
//    solver.set_optim_constraint_tolerance(1e-4);
//    solver.set_optim_max_iterations(10000);
//    solver.set_parallel(7);
//    // Set Guess
//    MocoTrajectory guess = solver.createGuess();
//    TableProcessor guessTable(
//        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");
//    guess.setStatesTrajectory(guessTable.process(),true,true);
//    solver.setGuess(guess);
//
//    MocoSolution solution = moco.solve();
//}
//
//void testPredictive_withPassiveForces_Implicit_Acceleration(){
//
//    MoGoaludy moco;
//    moco.setName("2DGaitPrediction_withPassiveForce_Implicit_Acceleration");
//
//    // Define the optimal control problem.
//    // ===================================
//    MocoProblem& problem = moco.updProblem();
//    ModelProcessor modelprocessor = ModelProcessor(
//        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
//    problem.setModelProcessor(modelprocessor);
//
//    problem.setTimeBounds(0, {0.4,0.6});
//
//    //// Goal.
//    //// =====
//    // Impose symmetric walking pattern
//    auto* symmetryGoal = problem.addGoal<MocoSymmetryGoal>("symmetryGoal");
//
//    // Impose prescribed average speed
//    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speedGoal");
//    speedGoal->set_desired_speed(1.2);
//
//    // Minimize squared control normalized by the distance travelled
//    auto* controlGoal =
//        problem.addGoal<MocoControlOverDistanceGoal>("controlGoal");
//    controlGoal->set_weight(1);
//
//    // Minimize squared accelerations normalized by the distance travelled
//    auto* accelerationGoal =
//        problem.addGoal<MocoAccelerationOverDistanceGoal>("accelerationGoal");
//    accelerationGoal->set_weight(0.001);
//
//    // Adjust bounds
//    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
//            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
//            {0.75,1.25});
//    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/lumbar/lumbar/value",
//            {0,20*SimTK::Pi/180});
//
//    // Configure the solver.
//    // =====================
//    auto& solver = moco.initCasADiSolver();
//    solver.set_dynamics_mode("implicit");
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(1e-4);
//    solver.set_optim_constraint_tolerance(1e-4);
//    solver.set_optim_max_iterations(10000);
//    solver.set_parallel(6);
//    // Set Guess
//    MocoTrajectory guess = solver.createGuess();
//    TableProcessor guessTable(
//        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");
//    guess.setStatesTrajectory(guessTable.process(),true,true);
//    solver.setGuess(guess);
//
//    MocoSolution solution = moco.solve();
//    }
//}
//
//void testPredictive_withPassiveForces_JRL(){
//
//    MoGoaludy moco;
//    moco.setName("2DGaitPrediction_withPassiveForce");
//
//    // Define the optimal control problem.
//    // ===================================
//    MocoProblem& problem = moco.updProblem();
//    ModelProcessor modelprocessor = ModelProcessor(
//        "gait10dof18musc_MusclePolynomials_withPassiveForces.osim");
//    problem.setModelProcessor(modelprocessor);
//
//    problem.setTimeBounds(0, {0.4,0.6});
//
//    //// Goal.
//    //// =====
//    // Impose symmetric walking pattern
//    auto* symmetryGoal = problem.addGoal<MocoSymmetryGoal>("symmetryGoal");
//
//    // Impose prescribed average speed
//    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speedGoal");
//    speedGoal->set_desired_speed(1.2);
//
//    // Minimize squared control normalized by the distance travelled
//    auto* controlGoal =
//        problem.addGoal<MocoControlOverDistanceGoal>("controlGoal");
//    controlGoal->set_weight(1);
//
//    // Minimize joint reaction loads
//    auto* loadGoal_knee_r = problem.addGoal<MocoJointReactionGoal>();
//    loadGoal_knee_r->setName("tibiofemoral_compressive_force_r");
//    loadGoal_knee_r->setJointPath("/jointset/knee_r");
//    loadGoal_knee_r->setLoadsFrame("child");
//    loadGoal_knee_r->setExpressedInFramePath("/bodyset/tibia_r");
//    loadGoal_knee_r->setReactionMeasures({"force-y"});
//    loadGoal_knee_r->set_weight(1);
//    auto* loadGoal_knee_l = problem.addGoal<MocoJointReactionGoal>();
//    loadGoal_knee_l->setName("tibiofemoral_compressive_force_l");
//    loadGoal_knee_l->setJointPath("/jointset/knee_l");
//    loadGoal_knee_l->setLoadsFrame("child");
//    loadGoal_knee_l->setExpressedInFramePath("/bodyset/tibia_l");
//    loadGoal_knee_l->setReactionMeasures({"force-y"});
//    loadGoal_knee_l->set_weight(1);
//    // Adjust bounds
//    //problem.setStateInfo("/lumbarAct/activation",{-1,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
//            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
//    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
//            {0.75,1.25});
//    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
//            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
//            {-50*SimTK::Pi/180,0});
//    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
//            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
//    problem.setStateInfo("/jointset/lumbar/lumbar/value",
//            {0,20*SimTK::Pi/180});
//
//    // Configure the solver.
//    // =====================
//    auto& solver = moco.initCasADiSolver();
//    solver.set_num_mesh_points(50);
//    solver.set_verbosity(2);
//    solver.set_optim_solver("ipopt");
//    solver.set_optim_convergence_tolerance(1e-4);
//    solver.set_optim_constraint_tolerance(1e-4);
//    solver.set_optim_max_iterations(10000);
//    solver.set_parallel(8);
//    // Set Guess
//    solver.setGuessFile(
//        "coordinateTracking_MusclePolynomials_withPassiveForces_solution.sto");
//
//    MocoSolution solution = moco.solve();
//}

int main() {
    example2DPrediction_Polynomial();
    //example2DPrediction_GeometryPath();
}
