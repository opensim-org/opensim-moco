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

class MocoTrack : Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoTrack, Object);

public:
    OpenSim_DECLARE_PROPERTY(create_reserve_actuators, double,
        "Create a reserve actuator (CoordinateActuator) for each "
        "unconstrained coordinate in the model, and add each to the model. "
        "Each actuator will have the specified `optimal_force`, which should "
        "be set low to discourage the use of the reserve actuators. (default "
        "is -1, which means no reserves are created)");
    OpenSim_DECLARE_PROPERTY(kinematics_file, std::string,
        "TODO");
    //OpenSim_DECLARE_PROPERTY(markers_file, std::string,
    //    "TODO");
    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string,
        "TODO");
    OpenSim_DECLARE_PROPERTY(remove_force_set, bool,
        "TODO");
    MocoTrack() {
        constructProperty_create_reserve_actuators(-1);
        constructProperty_kinematics_file("");
        //constructProperty_markers_file("");
        constructProperty_external_loads_file("");
        constructProperty_remove_force_set(false);
    }

    void setModel(Model model) { m_model = std::move(model); }

    void setKinematicsFile(std::string fileName) {
        set_kinematics_file(std::move(fileName));
    }

    //void setMarkersFile(std::string fileName) {
    //    m_markersFileName = std::move(fileName);
    //}

    void setExternalLoadsFile(std::string fileName) {
        set_external_loads_file(std::move(fileName));
    }

    void setRemoveModelForceSet(bool tf) {
        set_remove_force_set(tf);
    }

    void solve() const {

        MocoTool moco;
        auto& problem = moco.updProblem();

        // Modeling.
        // ---------
        Model model(m_model);
        model.finalizeFromProperties();

        if (get_remove_force_set()) { model.updForceSet().clearAndDestroy(); }
        model.initSystem();

        if (get_create_reserve_actuators() != -1) {
            // model.initSystem() is called within this utility.
            addCoordinateActuatorsToModel(model, 
                    get_create_reserve_actuators());
            const auto coordActs = model.getComponentList<CoordinateActuator>();
            for (const auto& coordAct : coordActs) {
                problem.setControlInfo("/" + coordAct.getName(), {-1, 1});
            }
        }

        problem.setModelCopy(model);

        // Costs.
        // ------
        // TODO: option for problem.addCost<MocoMarkerTrackingCost>();
        auto* stateTracking = 
                problem.addCost<MocoStateTrackingCost>("state_tracking");
        // Don't track coordinates that are already constrained.
        const auto& s = model.getWorkingState();
        auto kinematics = STOFileAdapter::read(get_kinematics_file());

        MocoWeightSet weights;
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            std::string path = coord.getAbsolutePathString();
            for (int i = 0; i < kinematics.getNumColumns(); ++i) {
                if (path.find(kinematics.getColumnLabel(i)) 
                        != std::string::npos) 
                {
                   kinematics.setColumnLabel(i, path + "/value");
                }

                double weight = coord.isConstrained(s) ? 0 : 1;
                weights.cloneAndAppend({path + "/value", weight});
            }
        }

        stateTracking->setReference(kinematics);
        stateTracking->setWeightSet(weights);
        stateTracking->setAllowUnusedReferences(true);

        // Set the time range.
        // -------------------
        // Set the time bounds based on the time range in the kinematics file.
        // Pad the beginning and end time points to allow room for finite 
        // difference calculations.
        // TODO: filter? Handle padding with filtering?
        problem.setTimeBounds(
            kinematics.getIndependentColumn().front() + 1e-3,
            kinematics.getIndependentColumn().back() - 1e-3);

        // Configure solver.
        // -----------------
        auto& solver = moco.initCasADiSolver();
        solver.set_num_mesh_points(25);
        solver.set_dynamics_mode("explicit");
        solver.set_optim_convergence_tolerance(1e-2);
        solver.set_optim_constraint_tolerance(1e-2);
        solver.set_enforce_constraint_derivatives(true);
        solver.set_transcription_scheme("hermite-simpson");
        solver.set_optim_finite_difference_scheme("forward");

        // Set the problem guess.
        // ----------------------
        solver.setGuess("bounds");

        // Solve!
        // ------
        MocoSolution solution = moco.solve();
        solution.write("sandboxMocoTrack_solution.sto");
        moco.visualize(solution);
    }

private:
    Model m_model;

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
        track.setKinematicsFile("ik_output_walk.mot");
        track.set_create_reserve_actuators(1000);
        track.setRemoveModelForceSet(true);

        //track.setExternalLoadsFile("grf_walk.xml");

        track.solve();
    }
    catch (const std::exception& e) { std::cerr << e.what() << std::endl; }


    return EXIT_SUCCESS;
}