/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxResidualReduction.cpp                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

#include <OpenSim/Common/LogManager.h>
#include <OpenSim/OpenSim.h>

using namespace OpenSim;

TimeSeriesTable configureStateTracking(MocoProblem& problem, Model& model,
        const TableProcessor& statesReference, double trackingWeight,
        MocoWeightSet stateWeights = {}, bool costEnabled = true) {

    // Read in the states reference data and spline.
    TimeSeriesTable states = statesReference.process(&model);
    auto stateSplines = GCVSplineSet(states, states.getColumnLabels());

    // Loop through all coordinates and compare labels in the reference data
    // to coordinate variable names.
    auto time = states.getIndependentColumn();
    auto labels = states.getColumnLabels();
    int numRefStates = (int)states.getNumColumns();
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        std::string coordPath = coord.getAbsolutePathString();
        std::string valueName = coordPath + "/value";
        std::string speedName = coordPath + "/speed";
        bool trackingValue = false;
        bool trackingSpeed = false;
        int valueIdx = -1;
        for (int i = 0; i < numRefStates; ++i) {
            if (labels[i] == valueName) {
                trackingValue = true;
                valueIdx = i;
            } else if (labels[i] == speedName) {
                trackingSpeed = true;
            }
        }

        // If a coordinate value was provided to track in the reference data,
        // but no corresponding speed, append the derivative of the coordinate
        // value to the tracking reference.
        if (trackingValue && !trackingSpeed) {
            auto value = states.getDependentColumnAtIndex(valueIdx);
            auto* valueSpline = stateSplines.getGCVSpline(valueIdx);
            SimTK::Vector speed((int)time.size());
            for (int j = 0; j < (int)time.size(); ++j) {
                speed[j] = valueSpline->calcDerivative(
                        {0}, SimTK::Vector(1, time[j]));
            }
            states.appendColumn(speedName, speed);
            trackingSpeed = true;
        }
    }

    // Add state tracking cost to the MocoProblem.
    auto* stateTracking = problem.addGoal<MocoStateTrackingGoal>(
            "state_tracking", trackingWeight);
    stateTracking->setReference(states);
    stateTracking->setWeightSet(stateWeights);
    stateTracking->setAllowUnusedReferences(true);
    stateTracking->setScaleWeightsWithRange(false);
    stateTracking->setEnabled(costEnabled);

    // Write tracked states to file in case any label updates or filtering
    // occured.
    writeTableToFile(states, "residual_reduction_tracked_states.sto");

    // Return tracked states to possibly include in the guess.
    return states;
}

void applyStatesToGuess(const TimeSeriesTable& states, MocoTrajectory& guess) {

    guess.resampleWithNumTimes((int)states.getNumRows());
    std::vector<std::string> names = guess.getStateNames();
    for (int i = 0; i < (int)states.getNumColumns(); ++i) {
        const auto& label = states.getColumnLabel(i);
        const auto& col = states.getDependentColumnAtIndex(i);

        if (std::find(names.begin(), names.end(), label) != names.end()) {
            guess.setState(label, col);
        }
    }
}

class MassConservationConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MassConservationConstraint, MocoPathConstraint);

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override {
        const auto state = model.getWorkingState();
        model.realizeTime(state);
        m_originalMass = model.getTotalMass(state);
        setNumEquations(1);
    }
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override {
        errors[0] = m_originalMass - getModel().getTotalMass(state);
    }

private:
    mutable double m_originalMass;
};

class StateDeviationConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            StateDeviationConstraint, MocoPathConstraint);

public:
    OpenSim_DECLARE_PROPERTY(reference, TableProcessor, "TODO");
    OpenSim_DECLARE_PROPERTY(rms_error, double, "TODO");
    OpenSim_DECLARE_PROPERTY(individual_error, double, "TODO");
    OpenSim_DECLARE_PROPERTY(state_deviation_weights, MocoWeightSet, "TODO");
    OpenSim_DECLARE_PROPERTY(allow_unused_references, bool,
            "Flag to determine whether or not references contained in the "
            "reference_file are allowed to be ignored by the constraint.");

    StateDeviationConstraint::StateDeviationConstraint() {
        constructProperties();
    }

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override {

        TimeSeriesTable tableToUse = get_reference().process("", &model);

        auto allSplines = GCVSplineSet(tableToUse);

        // Check that there are no redundant columns in the reference data.
        checkRedundantLabels(tableToUse.getColumnLabels());

        // Throw exception if a weight is specified for a nonexistent state.
        auto allSysYIndices = createSystemYIndexMap(model);

        // TODO add regex pattern option
        for (int i = 0; i < get_state_deviation_weights().getSize(); ++i) {
            const auto& weightName =
                    get_state_deviation_weights().get(i).getName();
            if (allSysYIndices.count(weightName) == 0) {
                OPENSIM_THROW_FRMOBJ(
                        Exception, "Weight provided with name '" + weightName +
                                           "' but this is "
                                           "not a recognized state.");
            }
        }

        // Populate member variables needed to compute cost. Unless the property
        // allow_unused_references is set to true, an exception is thrown for
        // names in the references that don't correspond to a state variable.
        const auto coordSet = model.getCoordinateSet();
        for (int iref = 0; iref < allSplines.getSize(); ++iref) {
            const auto& refName = allSplines[iref].getName();
            if (allSysYIndices.count(refName) == 0) {
                if (get_allow_unused_references()) { continue; }
                OPENSIM_THROW_FRMOBJ(Exception,
                        "State reference '" + refName + "' unrecognized.");
            }

            m_sysYIndices.push_back(allSysYIndices[refName]);
            double stateDevWeight = 1.0;
            if (get_state_deviation_weights().contains(refName)) {
                stateDevWeight =
                        get_state_deviation_weights().get(refName).getWeight();
            }
            m_state_dev_weights.push_back(stateDevWeight);
            m_refsplines.cloneAndAppend(allSplines[iref]);
            m_state_names.push_back(refName);
        }

        setNumEquations(m_refsplines.getSize() + 1);

        MocoConstraintInfo info;
        std::vector<MocoBounds> bounds;
        for (int i = 0; i < m_refsplines.getSize(); ++i) {
            bounds.emplace_back(0, SimTK::square(get_individual_error()));
        }
        bounds.emplace_back(0, SimTK::square(get_rms_error()));
        info.setBounds(bounds);
        const_cast<StateDeviationConstraint*>(this)->setConstraintInfo(info);
    }

    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override {
        const auto& time = state.getTime();

        SimTK::Vector timeVec(1, time);

        // TODO cache the reference coordinate values at the mesh points, rather
        // than evaluating the spline.
        double totalSquaredError = 0;
        double error = 0;
        double squaredError = 0;
        for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
            const auto& modelValue = state.getY()[m_sysYIndices[iref]];
            const auto& refValue = m_refsplines[iref].calcValue(timeVec);
            error = modelValue - refValue;
            errors[iref] = m_state_dev_weights[iref] * (error * error);
            totalSquaredError += errors[iref];
        }

        errors[m_refsplines.getSize()] =
                totalSquaredError / m_refsplines.getSize();
    }

private:
    void constructProperties() {
        constructProperty_reference(TableProcessor());
        constructProperty_rms_error(0);
        constructProperty_individual_error(0);
        constructProperty_allow_unused_references(false);
        constructProperty_state_deviation_weights(MocoWeightSet());
    }

    mutable GCVSplineSet m_refsplines;
    /// The indices in Y corresponding to the provided reference coordinates.
    mutable std::vector<int> m_sysYIndices;
    mutable std::vector<double> m_state_dev_weights;
    mutable std::vector<std::string> m_state_names;
};

class GeneralizedAccelerationGoal : public MocoGoal {

public:
    GeneralizedAccelerationGoal();

protected:
    void initializeOnModelImpl(const Model&) const override {
        setNumIntegralsAndOutputs(1, 1);
    }
    void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const override {
        getModel().realizeAcceleration(state);
        integrand = state.getUDot().normSqr();
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& goal) const override {
        goal[0] = input.integral /
            calcSystemDisplacement(input.initial_state, input.final_state);
    }
};

int main() {

    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cerr.rdbuf(LogManager::cerr.rdbuf());

    //bool halfGaitCycle = true;
    bool symmetryEnabled = true;

    bool stateDevEnabled = false;
    double stateDevRMS = 0.08;
    double individualStateDev = 1;

    bool stateTrackingEnabled = true;
    bool costsEnabled = true;
    double costsScale = 1;

    bool usePrevSolAsGuess = true;
    std::string dynamicsMode = "explicit";

    MocoStudy study;
    study.setName("residual_reduction");
    MocoProblem& problem = study.updProblem();

    auto modelProcessor =
            ModelProcessor(
                    "subject_walk_contact_bounded_nopatella_80musc.osim") |
            ModOpAddReserves(150, 1) |
            // ModOpAddExternalLoads("grf_walk.xml") |
            ModOpReplaceJointsWithWelds({"subtalar_r", "mtp_r", "subtalar_l",
                    "mtp_l", "radius_hand_r", "radius_hand_l"});
    Model model = modelProcessor.process();
    model.initSystem();
    model.print("updated_model.osim");

    problem.setModelCopy(model);
    //double finalTime = halfGaitCycle ? 1.385 : 1.96;
    problem.setTimeBounds(0.81, 1.385);
    const auto coordinates = TableProcessor("coordinates.mot") |
                             TabOpLowPassFilter(6.0) |
                             TabOpUseAbsoluteStateNames();

    // Symmetry constraints
    // ====================
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    model.initSystem();
    // Symmetric coordinate values (except for pelvis_tx) and speeds.
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        if (endsWith(coord.getName(), "_r")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    std::regex_replace(coord.getStateVariableNames()[0],
                            std::regex("_r/"), "_l/")});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_r/"), "_l/")});
        } else if (endsWith(coord.getName(), "_l")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    std::regex_replace(coord.getStateVariableNames()[0],
                            std::regex("_l/"), "_r/")});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_l/"), "_r/")});
        } else if (endsWith(coord.getName(), "_bending") ||
                   endsWith(coord.getName(), "_rotation") ||
                   endsWith(coord.getName(), "_tz") ||
                   endsWith(coord.getName(), "_list")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    coord.getStateVariableNames()[0]});
            symmetryGoal->addNegatedStatePair({coord.getStateVariableNames()[1],
                    coord.getStateVariableNames()[1]});
        } else if (!endsWith(coord.getName(), "_tx")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    coord.getStateVariableNames()[0]});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    coord.getStateVariableNames()[1]});
        }
    }
    symmetryGoal->addStatePair({"/jointset/ground_pelvis/pelvis_tx/speed"});
    symmetryGoal->setEnabled(symmetryEnabled);

    // State infos
    // ===========
    problem.setStateInfo("/jointset/back/lumbar_extension/value", {}, -0.12);
    problem.setStateInfo("/jointset/back/lumbar_bending/value", {}, 0);
    problem.setStateInfo("/jointset/back/lumbar_rotation/value", {}, 0.04);
    //problem.setStateInfo("/jointset/ground_pelvis/pelvis_tx/value", {}, 0.446);
    //problem.setStateInfo("/jointset/ground_pelvis/pelvis_tilt/value", {}, 0);
    //problem.setStateInfo("/jointset/ground_pelvis/pelvis_list/value", {}, 0);
    //problem.setStateInfo("/jointset/ground_pelvis/pelvis_rotation/value", {}, 0);

    // State tracking cost
    // ===================
    MocoWeightSet stateWeights;
    //stateWeights.cloneAndAppend({"/jointset/ground_pelvis/pelvis_tx/value", 1});
    stateWeights.cloneAndAppend({"/jointset/ground_pelvis/pelvis_ty/value", 0});
    stateWeights.cloneAndAppend({"/jointset/ground_pelvis/pelvis_tz/value", 0});
    stateWeights.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_list/value", 0});
    stateWeights.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_tilt/value", 0});
    stateWeights.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_rotation/value", 0});
    stateWeights.cloneAndAppend({"/jointset/hip_r/hip_rotation_r/value", 0});
    stateWeights.cloneAndAppend({"/jointset/hip_r/hip_adduction_r/value", 0});
    stateWeights.cloneAndAppend({"/jointset/hip_l/hip_rotation_l/value", 0});
    stateWeights.cloneAndAppend({"/jointset/hip_l/hip_adduction_l/value", 0});
    const auto states = configureStateTracking(problem, model, coordinates,
            10.0 / model.getCoordinateSet().getSize(), stateWeights,
            stateTrackingEnabled);

    // State deviation constraint
    // ==========================
    MocoWeightSet stateDevWeights;
    stateDevWeights.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_tx/value", 0});
    stateDevWeights.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_ty/value", 0});
    stateDevWeights.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_tz/value", 0});
    stateDevWeights.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_list/value", 0});
    stateDevWeights.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_tilt/value", 0});
    stateDevWeights.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_rotation/value", 0});
    if (stateDevEnabled) {
        auto* stateDev = problem.addPathConstraint<StateDeviationConstraint>();
        stateDev->setName("kinematic_deviation");
        stateDev->set_reference(coordinates);
        stateDev->set_rms_error(stateDevRMS);
        stateDev->set_individual_error(individualStateDev);
        stateDev->set_allow_unused_references(true);
        stateDev->set_state_deviation_weights(stateDevWeights);
    }

    // Effort cost 
    // ===========
    // Heavily penalized pelvis residual actuators
    auto* effort = problem.addGoal<MocoControlGoal>();
    effort->setName("effort");
    effort->setWeight(1.0 / model.getCoordinateSet().getSize());
    const double residualWeight = 1e5;
    for (const auto actu : model.getComponentList<CoordinateActuator>()) {
        const auto actuName = actu.getName();
        if (actuName.find("pelvis") != std::string::npos) {
            const auto actuPath = "/forceset/" + actuName;
            std::cout << actuPath << " residual weight: " << residualWeight
                      << std::endl;
            effort->setWeightForControl(actuPath, residualWeight);
        }
    }
    effort->setEnabled(costsEnabled);

    // TODO
    // Optimize mass parameters
    // ========================
    // for (const auto body : model.getComponentList<Body>()) {
    //    std::string paramName = body.getName() + "_mass";
    //    std::string bodyPath = "/bodyset/" + body.getName();
    //    double lower = 0.9 * body.getMass();
    //    double upper = 1.1 * body.getMass();
    //    problem.addParameter(
    //        MocoParameter(paramName, bodyPath, "mass", {lower, upper}));
    //}
    // auto* massCon = problem.addPathConstraint<MassConservationConstraint>();
    // massCon->setName("mass_conservation");

    // Contact tracking
    // ================
    const std::vector<std::string> forceNamesRightFoot = {
            "forceset/contactSphereHeel_r", "forceset/contactLateralRearfoot_r",
            "forceset/contactLateralMidfoot_r", "forceset/contactLateralToe_r",
            "forceset/contactMedialToe_r", "forceset/contactMedialMidfoot_r"};
    const std::vector<std::string> forceNamesLeftFoot = {
            "forceset/contactSphereHeel_l", "forceset/contactLateralRearfoot_l",
            "forceset/contactLateralMidfoot_l", "forceset/contactLateralToe_l",
            "forceset/contactMedialToe_l", "forceset/contactMedialMidfoot_l"};
    auto* contactTracking = problem.addGoal<MocoContactTrackingGoal>(
            "contact", 0.001);
    contactTracking->setExternalLoadsFile("grf_walk.xml");
    contactTracking->addContactGroup(forceNamesRightFoot, "Right_GRF");
    contactTracking->addContactGroup(forceNamesLeftFoot, "Left_GRF");
    //contactTracking->setProjection("plane");
    //contactTracking->setProjectionVector(SimTK::Vec3(0, 0, 1));
    contactTracking->setEnabled(false);

    // Average speed constraint
    // ========================
    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>("speed");
    speedGoal->set_desired_average_speed(1.235);

    // Configure solver
    // ================
    MocoCasADiSolver& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(25);
    solver.set_multibody_dynamics_mode(dynamicsMode);
    solver.set_optim_convergence_tolerance(1e-3);
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_minimize_implicit_multibody_accelerations(true);
    solver.set_implicit_multibody_accelerations_weight(costsScale * 1e-3);
    // solver.set_transcription_scheme("trapezoidal");
    // solver.set_optim_max_iterations(50);

    auto guess = solver.createGuess("bounds");
    applyStatesToGuess(states, guess);
    if (usePrevSolAsGuess) {
        MocoTrajectory prevSolution("residual_reduction_solution.sto");
        if (dynamicsMode == "implicit" && !prevSolution.getNumDerivatives()) {
            prevSolution.generateAccelerationsFromValues();
        }
        solver.setGuess(prevSolution);   
    } else {
        solver.setGuess(guess);
    }

    // Solve!
    // ======
    MocoSolution solution = study.solve().unseal();
    solution.write("residual_reduction_solution.sto");
    //MocoTrajectory solution("residual_reduction_solution.sto");

    MocoTrajectory fullTraj = createPeriodicTrajectory(solution);
    fullTraj.write("residual_reduction_solution_fulltraj.sto");

    TimeSeriesTable extLoads = createExternalLoadsTableForGait(model, fullTraj, 
        forceNamesRightFoot, forceNamesLeftFoot);
    writeTableToFile(extLoads, "residual_reduction_solution_fulltraj_grfs.sto");

    auto solStatesTable = solution.exportToStatesTable();
    for (const auto label : solStatesTable.getColumnLabels()) {
        if (label.find("/speed") != std::string::npos) {
            solStatesTable.removeColumn(label);
        }
    }
    writeTableToFile(solStatesTable, "coordinates_rra_halfcycle.sto");

    auto posErr = guess.compareContinuousVariablesRMSPattern(solution, "states",
        ".*/value\b");
    std::cout << "Residual reduction position RMSE: " << posErr << std::endl;

    study.visualize(fullTraj);

    // Inverse problem validation
    // ==========================
//    auto modelProcessorInverse =
//            ModelProcessor(
//                    "subject_walk_contact_bounded_nopatella_80musc.osim") |
//            ModOpAddReserves(250, 1) |
//            ModOpAddExternalLoads("grf_walk.xml") |
//            ModOpReplaceJointsWithWelds({"subtalar_r", "mtp_r", "subtalar_l",
//                    "mtp_l", "radius_hand_r", "radius_hand_l"});
//    Model modelInverse = modelProcessorInverse.process();
//
//    auto forceSet = model.getForceSet();
//    int numContacts = 0;
//    for (int i = 0; i < forceSet.getSize(); ++i) {
//        auto forceName = forceSet.get(i).getName(); 
//;       if (forceName.find("contact") != std::string::npos) {
//            numContacts++; 
//        }
//    }
//        
//    int contactsRemoved = 0;
//    while (contactsRemoved < numContacts) {
//        for (int i = 0; i < forceSet.getSize(); ++i) {
//            auto forceName = forceSet.get(i).getName();
//            if (forceName.find("contact") != std::string::npos)  
//                std::cout << "  --> removed " << forceSet.get(i).getName() 
//                          << std::endl;
//                forceSet.remove(i);
//                contactsRemoved++;
//                break;
//        }
//    }
//
//    modelInverse.initSystem();
//    modelInverse.print("updated_model_inverse.osim");
//
//    MocoInverse inverse;
//    inverse.setKinematics(coordinates);
//    inverse.setModel(modelInverse);
//    inverse.set_constraint_tolerance(1e-4);
//    inverse.set_convergence_tolerance(1e-4);
//    inverse.set_kinematics_allow_extra_columns(true);
//
//    auto inverseSolution = inverse.solve();
//    auto inverseMocoSolution = inverseSolution.getMocoSolution();
//
//    inverseMocoSolution.write("residual_reduction_inverse_solution.sto");
}