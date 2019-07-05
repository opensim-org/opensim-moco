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

#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/LinearFunction.h>
#include <Moco/osimMoco.h>
#include <Moco/Components/SmoothSphereHalfSpaceForce.h>

using namespace OpenSim;

// This class defines a MocoCost that computes the average speed defined as the
// distance travelled by the pelvis in the forward direction divided by the
// final time
class MocoAverageSpeedCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoAverageSpeedCost, MocoCost);
public:
    OpenSim_DECLARE_PROPERTY(desired_speed, double,
            "The desired forward speed defined as the distance travelled by"
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
    void calcEndpointCostImpl(const SimTK::State& finalState,
            SimTK::Real& cost) const override {
        // get final time
        SimTK::Real time = finalState.getTime();
        // get final pelvis forward position
        SimTK::Real position =  m_coord->getValue(finalState);
        cost = SimTK::square(get_desired_speed() - (position / time));
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

/// This model is torque-actuated.
std::unique_ptr<Model> createGait2D() {
    auto model = make_unique<Model>("gait_2D_muscle.osim");

    return model;
}

int main() {

    MocoStudy moco;
    moco.setName("gait2D_PredictiveMuscle");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = moco.updProblem();

    // Model (dynamics).
    // -----------------
    problem.setModel(createGait2D());

    // Bounds.
    // -------

    // States: joint positions and velocities
    // Ground pelvis
    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_rz/value", {-10, 10});
    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_rz/speed", {-10, 10});
    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_tx/value", {0, 10},{0});
    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_tx/speed", {-10, 10});
    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_ty/value", {-10, 10});
    problem.setStateInfo("/jointset/groundPelvis/groundPelvis_q_ty/speed", {-10, 10});
    // Hip left
    problem.setStateInfo("/jointset/hip_l/hip_q_l/value", {-10, 10});
    problem.setStateInfo("/jointset/hip_l/hip_q_l/speed", {-10, 10});
    // Hip right
    problem.setStateInfo("/jointset/hip_r/hip_q_r/value", {-10, 10});
    problem.setStateInfo("/jointset/hip_r/hip_q_r/speed", {-10, 10});
    // Knee left
    problem.setStateInfo("/jointset/knee_l/knee_q_l/value", {-10, 10});
    problem.setStateInfo("/jointset/knee_l/knee_q_l/speed", {-10, 10});
    // Knee right
    problem.setStateInfo("/jointset/knee_r/knee_q_r/value", {-10, 10});
    problem.setStateInfo("/jointset/knee_r/knee_q_r/speed", {-10, 10});
    // Ankle left
    problem.setStateInfo("/jointset/ankle_l/ankle_q_l/value", {-10, 10});
    problem.setStateInfo("/jointset/ankle_l/ankle_q_l/speed", {-10, 10});
    // Ankle right
    problem.setStateInfo("/jointset/ankle_r/ankle_q_r/value", {-10, 10});
    problem.setStateInfo("/jointset/ankle_r/ankle_q_r/speed", {-10, 10});
    // Lumbar
    problem.setStateInfo("/jointset/lumbar/lumbar_q/value", {-10, 10});
    problem.setStateInfo("/jointset/lumbar/lumbar_q/speed", {-10, 10});

    // States: muscle activations
    // Right
    problem.setStateInfo("/forceset/hamstrings_r/activation", {0, 1});
    problem.setStateInfo("/forceset/bifemsh_r/activation", {0, 1});
    problem.setStateInfo("/forceset/glut_max_r/activation", {0, 1});
    problem.setStateInfo("/forceset/iliopsoas_r/activation", {0, 1});
    problem.setStateInfo("/forceset/rect_fem_r/activation", {0, 1});
    problem.setStateInfo("/forceset/vasti_r/activation", {0, 1});
    problem.setStateInfo("/forceset/gastroc_r/activation", {0, 1});
    problem.setStateInfo("/forceset/soleus_r/activation", {0, 1});
    problem.setStateInfo("/forceset/tib_ant_r/activation", {0, 1});
    // Left
    problem.setStateInfo("/forceset/hamstrings_l/activation", {0, 1});
    problem.setStateInfo("/forceset/bifemsh_l/activation", {0, 1});
    problem.setStateInfo("/forceset/glut_max_l/activation", {0, 1});
    problem.setStateInfo("/forceset/iliopsoas_l/activation", {0, 1});
    problem.setStateInfo("/forceset/rect_fem_l/activation", {0, 1});
    problem.setStateInfo("/forceset/vasti_l/activation", {0, 1});
    problem.setStateInfo("/forceset/gastroc_l/activation", {0, 1});
    problem.setStateInfo("/forceset/soleus_l/activation", {0, 1});
    problem.setStateInfo("/forceset/tib_ant_l/activation", {0, 1});

    // Controls: torque actuators
    problem.setControlInfo("/groundPelvisAct_rz", {-150, 150});
    problem.setControlInfo("/groundPelvisAct_tx", {-150, 150});
    problem.setControlInfo("/groundPelvisAct_ty", {-150, 150});
    problem.setControlInfo("/lumbarAct", {-150, 150});
    // Controls: muscle excitations
    // Right
    problem.setControlInfo("/forceset/hamstrings_r", {0, 1});
    problem.setControlInfo("/forceset/bifemsh_r", {0, 1});
    problem.setControlInfo("/forceset/glut_max_r", {0, 1});
    problem.setControlInfo("/forceset/iliopsoas_r", {0, 1});
    problem.setControlInfo("/forceset/rect_fem_r", {0, 1});
    problem.setControlInfo("/forceset/vasti_r", {0, 1});
    problem.setControlInfo("/forceset/gastroc_r", {0, 1});
    problem.setControlInfo("/forceset/soleus_r", {0, 1});
    problem.setControlInfo("/forceset/tib_ant_r", {0, 1});
    // Left
    problem.setControlInfo("/forceset/hamstrings_l", {0, 1});
    problem.setControlInfo("/forceset/bifemsh_l", {0, 1});
    problem.setControlInfo("/forceset/glut_max_l", {0, 1});
    problem.setControlInfo("/forceset/iliopsoas_l", {0, 1});
    problem.setControlInfo("/forceset/rect_fem_l", {0, 1});
    problem.setControlInfo("/forceset/vasti_l", {0, 1});
    problem.setControlInfo("/forceset/gastroc_l", {0, 1});
    problem.setControlInfo("/forceset/soleus_l", {0, 1});
    problem.setControlInfo("/forceset/tib_ant_l", {0, 1});

    // Static parameter: final time
    double finalTime = 1.0;
    problem.setTimeBounds(0, finalTime);

    //// Cost.
    //// -----
    // Minimize torque actuators squared
    auto* controlCost = problem.addCost<MocoControlCost>("controlCost");
    controlCost->set_weight(1);

    // Impose average speed
    auto* speedCost = problem.addCost<MocoAverageSpeedCost>("speedCost");
    speedCost->set_weight(1);
    speedCost->set_desired_speed(1.2);

    // Impose symmetry
    // TODO

    // Impose 1/d * squared controls

    // Configure the solver.
    // =====================
    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");

    moco.print("gait2D_PredictiveMuscle.omoco");

    // Solve the problem.
    // ==================
    MocoSolution solution = moco.solve();
    solution.write("gait2D_PredictiveMuscle_solution.sto");

    moco.visualize(solution);

    return EXIT_SUCCESS;

return 0;
}
