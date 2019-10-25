/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxWholeBodyTracking.cpp                                 *
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
 
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <Moco/osimMoco.h>

using namespace OpenSim;

void createExternalLoadsTable(Model model, MocoTrajectory solution, 
        std::string problemName) {

    std::vector<std::string> forceNamesRightFoot{"contactSphereHeel_r",
            "contactLateralRearfoot_r", "contactLateralMidfoot_r",
            "contactLateralToe_r", "contactMedialToe_r",
            "contactMedialMidfoot_r"};
    std::vector<std::string> forceNamesLeftFoot{"contactSphereHeel_l",
            "contactLateralRearfoot_l", "contactLateralMidfoot_l",
            "contactLateralToe_l", "contactMedialToe_l",
            "contactMedialMidfoot_l"};
    TimeSeriesTable externalLoads = createExternalLoadsTableForGait(
            model, solution, forceNamesRightFoot, forceNamesLeftFoot);

    writeTableToFile(externalLoads, problemName + "_grfs.sto");
}

void runTrackingProblem() {
    ModelProcessor modelProcessor =
            ModelProcessor("subject_walk_armless_contact_torque_driven.osim") |
            ModOpReplaceJointsWithWelds(
                    {"subtalar_r", "mtp_r", "subtalar_l", "mtp_l"});

    Model model = modelProcessor.process();
    model.print("subject_walk_armless_contact_torque_driven_welded.osim");

    MocoTrack track;
    std::string problemName = "sandbox_tracking_contact";
    track.setName(problemName);
    track.setModel(modelProcessor);
    track.setStatesReference(TableProcessor("coordinates.mot") |
                             TabOpLowPassFilter(6) |
                             TabOpUseAbsoluteStateNames());
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_states_global_tracking_weight(10);
    track.set_control_effort_weight(0.1);

    track.set_initial_time(0.81);
    track.set_final_time(1.96);
    track.set_mesh_interval(0.01);

    MocoStudy study = track.initialize();
    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.setGuessFile(problemName + "_solution.sto");

    MocoSolution solution = study.solve().unseal();
    study.visualize(solution);
    solution.write("sandbox_tracking_contact_solution.sto");

    createExternalLoadsTable(model, solution, problemName);
}

void runSymmetricTrackingProblem() {
    ModelProcessor modelProcessor =
            ModelProcessor("subject_walk_armless_contact_torque_driven_temp.osim") |
            ModOpReplaceJointsWithWelds(
                    {"subtalar_r", "mtp_r", "subtalar_l", "mtp_l"}) |
            ModOpRemoveMuscles();

    MocoTrack track;
    std::string problemName = "sandbox_symmetric_tracking_contact";
    track.setName(problemName);
    track.setModel(modelProcessor);
    track.setStatesReference(TableProcessor("coordinates.mot") |
                             TabOpLowPassFilter(6) |
                             TabOpUseAbsoluteStateNames());
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_states_global_tracking_weight(10);
    track.set_control_effort_weight(0.1);

    track.set_initial_time(0.81);
    track.set_final_time(1.385);
    track.set_mesh_interval(0.01);

    MocoStudy study = track.initialize();
    auto& problem = study.updProblem();

    Model model = modelProcessor.process();
    model.initSystem();
    auto* symmetryGoal =
            problem.addGoal<MocoPeriodicityGoal>("symmetry_goal");
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        if (endsWith(coord.getName(), "_r")) {
            symmetryGoal->addStatePair({
                coord.getStateVariableNames()[0],
                std::regex_replace(coord.getStateVariableNames()[0],
                    std::regex("_r/"), "_l/")
            });
            symmetryGoal->addStatePair({
                coord.getStateVariableNames()[1],
                std::regex_replace(coord.getStateVariableNames()[1],
                    std::regex("_r/"), "_l/")
            });
        }
        if (endsWith(coord.getName(), "_l")) {
            symmetryGoal->addStatePair({
                coord.getStateVariableNames()[0],
                std::regex_replace(coord.getStateVariableNames()[0],
                    std::regex("_l/"), "_r/")
            });
            symmetryGoal->addStatePair({
                coord.getStateVariableNames()[1],
                std::regex_replace(coord.getStateVariableNames()[1],
                    std::regex("_l/"), "_r/")
            });
        }
        if (!endsWith(coord.getName(), "_l") &&
            !endsWith(coord.getName(), "_r") &&
            !endsWith(coord.getName(), "_tx") && 
            !endsWith(coord.getName(), "_rotation") && 
            !endsWith(coord.getName(), "_list") &&
            !endsWith(coord.getName(), "_tz")) {
            symmetryGoal->addStatePair({
                coord.getStateVariableNames()[0],
                coord.getStateVariableNames()[0]
            });
            symmetryGoal->addStatePair({
                coord.getStateVariableNames()[1],
                coord.getStateVariableNames()[1]
            });
        }
    }
    symmetryGoal->addStatePair({"/jointset/ground_pelvis/pelvis_tx/speed"});

    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.setGuessFile("sandbox_tracking_contact_solution.sto");

    MocoSolution solution = study.solve().unseal();
    study.visualize(solution);
    solution.write(problemName + "_solution.sto");

    //auto solution_full = createPeriodicTrajectory(solution);
    //study.visualize(solution_full);
    //solution_full.write(problemName + "_solution.sto");

    createExternalLoadsTable(model, solution, problemName);
}

void runInverseProblem() {
    ModelProcessor modelProcessor =
            ModelProcessor("subject_walk_armless_80musc.osim") |
            ModOpReplaceJointsWithWelds(
                    {"subtalar_r", "mtp_r", "subtalar_l", "mtp_l"}) |
            ModOpAddExternalLoads("grf_walk.xml") |
            ModOpReplaceMusclesWithDeGrooteFregly2016() |
            ModOpIgnorePassiveFiberForcesDGF() | 
            ModOpIgnoreTendonCompliance() |
            //ModOpUseImplicitTendonComplianceDynamicsDGF() |
            ModOpAddReserves(1, 1000, true);

    modelProcessor.process().print("subject_walk_armless_80musc_welded.osim");

    MocoInverse inverse;
    std::string problemName = "sandbox_inverse_muscles";
    inverse.setName(problemName);
    inverse.setModel(modelProcessor);
    inverse.setKinematics(TableProcessor("coordinates.mot") |
                             TabOpLowPassFilter(6) |
                             TabOpUseAbsoluteStateNames());
    inverse.set_kinematics_allow_extra_columns(true);

    inverse.set_initial_time(0.81);
    inverse.set_final_time(1.385);
    inverse.set_mesh_interval(0.01);

    MocoSolution solution = inverse.solve().getMocoSolution();
    solution.write(problemName + "_solution.sto");
}

void addSymmetryConstraints(Model model, MocoProblem& problem) {
    model.initSystem();
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetry_goal");
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        // Initial left leg coordinate equals final right leg coordinate and
        // vice versa.
        if (endsWith(coord.getName(), "_r")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    std::regex_replace(coord.getStateVariableNames()[0],
                            std::regex("_r/"), "_l/")});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_r/"), "_l/")});
        }
        if (endsWith(coord.getName(), "_l")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    std::regex_replace(coord.getStateVariableNames()[0],
                            std::regex("_l/"), "_r/")});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_l/"), "_r/")});
        }
        // Coordinates with equal initial and final values.
        if (!endsWith(coord.getName(), "_l") &&
                !endsWith(coord.getName(), "_r") &&
                !endsWith(coord.getName(), "_tx") &&
                !endsWith(coord.getName(), "_rotation") &&
                !endsWith(coord.getName(), "_list") &&
                !endsWith(coord.getName(), "_tz")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    coord.getStateVariableNames()[0]});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    coord.getStateVariableNames()[1]});
        }
        // Coordinates with negated initial and final values. Note that this
        // forces these coordinates to cross zero, even if the tracked data does
        // not (e.g., pelvis z-direction translation).
        if (endsWith(coord.getName(), "lumbar_bending") ||
                endsWith(coord.getName(), "_rotation") ||
                endsWith(coord.getName(), "_tz") ||
                endsWith(coord.getName(), "_list")) {
            symmetryGoal->addNegatedStatePair({coord.getStateVariableNames()[0],
                    coord.getStateVariableNames()[0]});
            symmetryGoal->addNegatedStatePair({coord.getStateVariableNames()[1],
                    coord.getStateVariableNames()[1]});
        }
    }
    // Only the pelvis x-direction speed is symmetric.
    symmetryGoal->addStatePair({"/jointset/ground_pelvis/pelvis_tx/speed"});
    // Symmetric muscle activations.
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (endsWith(muscle.getName(), "_r")) {
            symmetryGoal->addStatePair({muscle.getStateVariableNames()[0],
                    std::regex_replace(muscle.getStateVariableNames()[0],
                            std::regex("_r"), "_l")});
        }
        if (endsWith(muscle.getName(), "_l")) {
            symmetryGoal->addStatePair({muscle.getStateVariableNames()[0],
                    std::regex_replace(muscle.getStateVariableNames()[0],
                            std::regex("_l"), "_r")});
        }
    }
}

void runTrackingProblemWithMuscles(const double& trackingWeight = 1.0, 
        const double& controlEffortWeight = 1.0, 
        std::string guessType = "inverse") {
    // Model
    // -----
    ModelProcessor modelProcessor =
            ModelProcessor("subject_walk_armless_contact_80musc.osim") |
            ModOpReplaceJointsWithWelds(
                    {"subtalar_r", "mtp_r", "subtalar_l", "mtp_l"}) |
            ModOpIgnorePassiveFiberForcesDGF() |
            ModOpIgnoreTendonCompliance();

    // Construct the base tracking problem
    // -----------------------------------
    MocoTrack track;
    std::string problemName = "sandbox_tracking_contact_muscles";
    track.setName(problemName);
    track.setModel(modelProcessor);
    track.setStatesReference(TableProcessor("coordinates.mot") |
                             TabOpLowPassFilter(6) |
                             TabOpUseAbsoluteStateNames());
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_states_global_tracking_weight(trackingWeight);
    track.set_control_effort_weight(controlEffortWeight);
    track.set_initial_time(0.81);
    track.set_final_time(1.385);
    track.set_mesh_interval(0.02);

    // Customize the base tracking problem
    // -----------------------------------
    MocoStudy study = track.initialize();
    auto& problem = study.updProblem();
    // Ensure that the pelvis starts and ends at the same x-positions from the 
    // measurements, even if the tracking weight is low. Since we are tracking
    // data, we used this instead of an effort-over-distance cost function.
    problem.setStateInfo("/jointset/ground_pelvis/pelvis_tx/value", {},
        0.446, 1.156);

    // Symmetry constraints
    // --------------------
    Model model = modelProcessor.process();
    addSymmetryConstraints(model, problem);

    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_convergence_tolerance(1e-2);

    MocoTrajectory guess;
    if (guessType == "inverse") {
        MocoTrajectory inverseSolution("sandbox_inverse_muscles_solution.sto");

        guess = solver.createGuess();
        auto simtkTime = inverseSolution.getTime();
        std::vector<double> time(simtkTime.getContiguousScalarData(),
                simtkTime.getContiguousScalarData() + simtkTime.size());

        TimeSeriesTable statesTraj(time, inverseSolution.getStatesTrajectory(),
                inverseSolution.getStateNames());
        guess.insertStatesTrajectory(statesTraj, true);

        TimeSeriesTable controlsTraj(time,
                inverseSolution.getControlsTrajectory(),
                inverseSolution.getControlNames());
        guess.insertStatesTrajectory(statesTraj, true);

    } else if (guessType == "previous") {
        guess = MocoTrajectory(format("%s_solution.sto", problemName));
    }
    solver.setGuess(guess);

    // Solve and print solution.
    // -------------------------
    MocoSolution solution = study.solve().unseal();
    solution.write(format("%s_solution.sto", problemName));
    solution.write(format("%s_solution_track%f_control%f.sto", problemName,
            trackingWeight, controlEffortWeight));

    // Create a full gait cycle trajectory from the periodic solution.
    auto fullTraj = createPeriodicTrajectory(solution);
    fullTraj.write(format("%s_solution_fullcycle.sto", problemName));
    fullTraj.write(format("%s_solution_fullcycle_track%f_control%f.sto", 
            problemName, trackingWeight, controlEffortWeight));

    // Compute ground reaction forces generated by contact sphere from the full
    // gait cycle trajectory.
    createExternalLoadsTable(model, fullTraj, problemName);

    // Visualize
    // study.visualize(fullTraj);
}

void runPredictiveProblem(const MocoTrajectory& trackingSolution) {

    MocoStudy study;
    std::string problemName = "sandbox_prediction_contact_muscles";
    study.setName(problemName);

    // Define the problem
    // ------------------
    MocoProblem& problem = study.updProblem();
    ModelProcessor modelProcessor =
            ModelProcessor("subject_walk_armless_contact_80musc.osim") |
            ModOpReplaceJointsWithWelds(
                    {"subtalar_r", "mtp_r", "subtalar_l", "mtp_l"}) |
            ModOpIgnorePassiveFiberForcesDGF() | ModOpIgnoreTendonCompliance();
    problem.setModelProcessor(modelProcessor);

    // Bounds.
    // =======
    problem.setTimeBounds(0.81, {1.185, 1.585});

    // Costs
    // -----
    // Prescribed average gait speed.
    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speed");
    speedGoal->set_desired_average_speed(1.2);
    // Effort over distance.
    auto* effortGoal = problem.addGoal<MocoControlGoal>("effort", 10);
    effortGoal->setExponent(3);
    effortGoal->setDivideByDisplacement(true);

    // Symmetry constraints
    // --------------------
    Model model = modelProcessor.process();
    addSymmetryConstraints(model, problem);

    // Configure the solver
    // --------------------
    auto& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(25);
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_convergence_tolerance(1e-3);
    solver.setGuess(trackingSolution);

    // Solve and print solution.
    // -------------------------
    MocoSolution solution = study.solve().unseal();
    solution.write(format("%s_solution.sto", problemName));

    // Create a full gait cycle trajectory from the periodic solution.
    auto fullTraj = createPeriodicTrajectory(solution);
    fullTraj.write(format("%s_solution_fullcycle.sto", problemName));

    // Compute ground reaction forces generated by contact sphere from the full
    // gait cycle trajectory.
    createExternalLoadsTable(model, fullTraj, problemName);

    // Visualize.
    study.visualize(fullTraj);
}

int main() {

    //runTrackingProblem();
    //runSymmetricTrackingProblem();
    //runInverseProblem();
    
    //runTrackingProblemWithMuscles(1.0, 0.01, "inverse");
    //runTrackingProblemWithMuscles(1.0, 0.1, "previous");
    //runTrackingProblemWithMuscles(1.0, 1.0, "previous");
    //runTrackingProblemWithMuscles(1.0, 10, "previous");
    //runTrackingProblemWithMuscles(1.0, 100, "previous");
    //runTrackingProblemWithMuscles(1.0, 1000, "previous");

    runPredictiveProblem(MocoTrajectory(
        "sandbox_tracking_contact_muscles_solution_track1_control1000.sto"));

    return EXIT_SUCCESS;
}
