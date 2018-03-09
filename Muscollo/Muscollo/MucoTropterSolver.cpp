/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoTropterSolver.cpp                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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
#include "MucoTropterSolver.h"
#include "MucoProblem.h"
#include "MuscolloUtilities.h"

#include <OpenSim/Simulation/Manager/Manager.h>

#include <tropter/tropter.h>

using namespace OpenSim;

using tropter::VectorX;

/// The map provides the index of each state variable in
/// SimTK::State::getY() from its each state variable path string.
std::vector<std::string> createStateVariableNamesInSystemOrder(
        const Model& model) {
    std::vector<std::string> svNamesInSysOrder;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                svNamesInSysOrder.push_back(svNames[isv]);
                s.updY()[iy] = 0;
                break;
            }
        }
    }
    SimTK_ASSERT2_ALWAYS((size_t)svNames.size() == svNamesInSysOrder.size(),
            "Expected to get %i state names but found %i.", svNames.size(),
            svNamesInSysOrder.size());
    return svNamesInSysOrder;
}

template <typename MucoIterateType, typename tropIterateType>
MucoIterateType convert(const tropIterateType& tropSol) {
    const auto& tropTime = tropSol.time;
    SimTK::Vector time((int)tropTime.size(), tropTime.data());
    const auto& state_names = tropSol.state_names;
    const auto& control_names = tropSol.control_names;
    const auto& parameter_names = tropSol.parameter_names;

    int numTimes = (int)time.size();
    int numStates = (int)state_names.size();
    int numControls = (int)control_names.size();
    int numParameters = (int)parameter_names.size();
    SimTK::Matrix states(numTimes, numStates);
    for (int itime = 0; itime < numTimes; ++itime) {
        for (int istate = 0; istate < numStates; ++istate) {
            states(itime, istate) = tropSol.states(istate, itime);
        }
    }
    SimTK::Matrix controls(numTimes, numControls);
    for (int itime = 0; itime < numTimes; ++itime) {
        for (int icontrol = 0; icontrol < numControls; ++icontrol) {
            controls(itime, icontrol) = tropSol.controls(icontrol, itime);
        }
    }
    SimTK::RowVector parameters(numParameters, tropSol.parameters.data());
    return {time, state_names, control_names, parameter_names, states, 
            controls, parameters};
}

MucoSolution convert(const tropter::Solution& tropSol) {
    // TODO enhance when solution contains more info than iterate.
    return convert<MucoSolution, tropter::Solution>(tropSol);
}

tropter::Iterate convert(const MucoIterate& mucoIter) {
    tropter::Iterate tropIter;
    if (mucoIter.empty()) return tropIter;

    using Eigen::Map;
    using Eigen::RowVectorXd;
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const auto& time = mucoIter.getTime();
    tropIter.time = Map<const RowVectorXd>(&time[0], time.size());

    tropIter.state_names = mucoIter.getStateNames();
    tropIter.control_names = mucoIter.getControlNames();
    tropIter.parameter_names = mucoIter.getParameterNames();

    int numTimes = (int)time.size();
    int numStates = (int)tropIter.state_names.size();
    int numControls = (int)tropIter.control_names.size();
    int numParameters = (int)tropIter.parameter_names.size();
    const auto& states = mucoIter.getStatesTrajectory();
    const auto& controls = mucoIter.getControlsTrajectory();
    const auto& parameters = mucoIter.getParameters();
    // Muscollo's matrix is numTimes x numStates;
    // tropter's is numStates x numTimes.
    tropIter.states = Map<const MatrixXd>(
            &states(0, 0), numTimes, numStates).transpose();
    if (numControls) {
        tropIter.controls = Map<const MatrixXd>(
                &controls(0, 0), numTimes, numControls).transpose();
    } else {
        tropIter.controls.resize(numControls, numTimes);
    }
    if (numParameters) {
        tropIter.parameters = Map<const VectorXd>(
                &parameters(0), numParameters);
    } else {
        tropIter.parameters.resize(numParameters);
    }
    return tropIter;
}

tropter::Bounds convert(const MucoBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
tropter::InitialBounds convert(const MucoInitialBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
tropter::FinalBounds convert(const MucoFinalBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}

template <typename T>
class MucoTropterSolver::OCProblem : public tropter::Problem<T> {
public:
    OCProblem(const MucoSolver& solver)
            // TODO set name properly.
            : tropter::Problem<T>(solver.getProblem().getName()),
              m_mucoSolver(solver),
              m_mucoProb(solver.getProblem()),
              m_phase0(m_mucoProb.getPhase(0)) {
        m_model = m_phase0.getModel();
        m_state = m_model.initSystem();

        this->set_time(convert(m_phase0.getTimeInitialBounds()),
                convert(m_phase0.getTimeFinalBounds()));
        auto svNamesInSysOrder = createStateVariableNamesInSystemOrder(m_model);
        for (const auto& svName : svNamesInSysOrder) {
            const auto& info = m_phase0.getStateInfo(svName);
            this->add_state(svName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
        for (const auto& actu : m_model.getComponentList<Actuator>()) {
            // TODO handle a variable number of control signals.
            const auto& actuName = actu.getName();
            const auto& info = m_phase0.getControlInfo(actuName);
            this->add_control(actuName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
        for (std::string name : m_phase0.createParameterNames()) {
            const MucoParameter& parameter = m_phase0.getParameter(name);
            this->add_parameter(name, convert(parameter.getBounds()));
        }
    }
    void initialize_on_mesh(const Eigen::VectorXd&) const override {
        m_mucoProb.initialize(m_model);
    }
    void initialize_on_iterate(const Eigen::VectorXd& parameters)
            const override {
        // If they exist, apply parameter values to the model.
        this->applyParametersToModel(parameters);
    }
    // TODO rename argument "states" to "state".
    void calc_differential_algebraic_equations(
            const tropter::DAEInput<T>& in,
            tropter::DAEOutput<T> out) const override {

        // TODO convert to implicit formulation.

        const auto& states = in.states;
        const auto& controls = in.controls;

        m_state.setTime(in.time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        //
        // TODO do not copy? I think this will still make a copy:
        // TODO use m_state.updY() = SimTK::Vector(states.size(), states.data(), true);
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        if (m_model.getNumControls()) {
            auto& osimControls = m_model.updControls(m_state);
            std::copy(controls.data(), controls.data() + controls.size(),
                    &osimControls[0]);

            m_model.realizeVelocity(m_state);
            m_model.setControls(m_state, osimControls);
        }

        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.
        m_model.realizeAcceleration(m_state);
        std::copy(&m_state.getYDot()[0], &m_state.getYDot()[0] + states.size(),
                out.dynamics.data());

    }
    void calc_integral_cost(const T& time,
            const VectorX<T>& states,
            const VectorX<T>& controls, 
            const VectorX<T>& parameters, T& integrand) const override {
        // TODO would it make sense to a vector of States, one for each mesh
        // point, so that each can preserve their cache?
        m_state.setTime(time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        if (m_model.getNumControls()) {
            auto& osimControls = m_model.updControls(m_state);
            std::copy(controls.data(), controls.data() + controls.size(),
                    &osimControls[0]);
            m_model.realizePosition(m_state);
            m_model.setControls(m_state, osimControls);
        } else {
            m_model.realizePosition(m_state);
        }
        integrand = m_phase0.calcIntegralCost(m_state);
    }
    void calc_endpoint_cost(const T& final_time, const VectorX<T>& states,
            const VectorX<T>& parameters, T& cost) const override {
        // TODO avoid all of this if there are no endpoint costs.
        m_state.setTime(final_time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        // TODO cannot use control signals...
        m_model.updControls(m_state).setToNaN();
        cost = m_phase0.calcEndpointCost(m_state);
    }

private:
    const MucoSolver& m_mucoSolver;
    const MucoProblem& m_mucoProb;
    const MucoPhase& m_phase0;
    mutable Model m_model;
    mutable SimTK::State m_state;

    void applyParametersToModel(const VectorX<T>& parameters) const
    {
        if (parameters.size()) {
            // Warning: memory borrowed, not copied (when third argument to
            // SimTK::Vector constructor is true)
            SimTK::Vector mucoParams(
                (int)m_phase0.createParameterNames().size(),
                parameters.data(), true);

            m_phase0.applyParametersToModel(mucoParams);
            m_model.initSystem();
        }
    }
};

MucoTropterSolver::MucoTropterSolver() {
    constructProperties();
}

void MucoTropterSolver::constructProperties() {
    constructProperty_num_mesh_points(100);
    constructProperty_verbosity(2);
    constructProperty_optim_solver("ipopt");
    constructProperty_optim_max_iterations(-1);
    constructProperty_optim_convergence_tolerance(-1);
    constructProperty_optim_constraint_tolerance(-1);
    constructProperty_optim_hessian_approximation("limited-memory");
    constructProperty_optim_sparsity_detection("random");
    constructProperty_optim_ipopt_print_level(-1);

    constructProperty_guess_file("");
}

std::shared_ptr<const tropter::Problem<double>>
MucoTropterSolver::getTropterProblem() const {
    if (!m_tropProblem) {
        m_tropProblem = std::make_shared<OCProblem<double>>(*this);
    }
    return m_tropProblem;
}

void MucoTropterSolver::clearProblemImpl() {
    clearGuess();
}

void MucoTropterSolver::setProblemImpl(const MucoProblem& /*problem*/) {
    clearProblemImpl();
}

MucoIterate MucoTropterSolver::createGuess(const std::string& type) const {
    OPENSIM_THROW_IF_FRMOBJ(
               type != "bounds"
            && type != "random"
            && type != "time-stepping",
            Exception,
            "Unexpected guess type '" + type +
            "'; supported types are 'bounds', 'random', and "
            "'time-stepping'.");
    auto ocp = getTropterProblem();

    if (type == "time-stepping") {
        return createGuessTimeStepping();
    }

    // TODO avoid performing error checks multiple times; use
    // isObjectUpToDateWithProperties();
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    int N = get_num_mesh_points();

    checkPropertyInSet(*this, getProperty_optim_solver(), {"ipopt", "snopt"});
    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal",
            get_optim_solver(), N);

    tropter::Iterate tropIter;
    if (type == "bounds") {
        tropIter = dircol.make_initial_guess_from_bounds();
    } else if (type == "random") {
        tropIter = dircol.make_random_iterate_within_bounds();
    }
    return convert<MucoIterate, tropter::Iterate>(tropIter);
}

MucoIterate MucoTropterSolver::createGuessTimeStepping() const {
    const auto& problem = getProblem();
    const auto& phase = problem.getPhase();
    const auto& initialTime = phase.getTimeInitialBounds().getUpper();
    const auto& finalTime = phase.getTimeFinalBounds().getLower();
    OPENSIM_THROW_IF_FRMOBJ(finalTime <= initialTime, Exception,
        "Expected lower bound on final time to be greater than "
        "upper bound on initial time, but "
        "final_time.lower: " + std::to_string(finalTime) + "; " +
        "initial_time.upper: " + std::to_string(initialTime) + ".");
    Model model(phase.getModel());
    // Disable all controllers?
    SimTK::State state = model.initSystem();

    // Modify initial state values as necessary.
    Array<std::string> svNames = model.getStateVariableNames();
    for (int isv = 0; isv < svNames.getSize(); ++isv) {
        const auto& svName = svNames[isv];
        const auto& initBounds = phase.getStateInfo(svName).getInitialBounds();
        const auto defaultValue = model.getStateVariableValue(state, svName);
        SimTK::Real valueToUse = defaultValue;
        if (initBounds.isEquality()) {
            valueToUse = initBounds.getLower();
        } else if (!initBounds.isWithinBounds(defaultValue)) {
            valueToUse = 0.5 * (initBounds.getLower() + initBounds.getUpper());
        }
        if (valueToUse != defaultValue) {
            model.setStateVariableValue(state, svName, valueToUse);
        }
    }

    state.setTime(initialTime);
    Manager manager(model, state);
    manager.integrate(finalTime);

    const auto& statesTable = manager.getStatesTable();
    const auto controlsTable = model.getControlsTable();

    // TODO handle parameters.
    return MucoIterate::createFromStatesControlsTables(
            problem, statesTable, controlsTable);
}

void MucoTropterSolver::setGuess(MucoIterate guess) {
    // Ensure the guess is compatible with this solver/problem.
    guess.isCompatible(getProblem(), true);
    clearGuess();
    m_guessFromAPI = std::move(guess);
}
void MucoTropterSolver::setGuessFile(const std::string& file) {
    clearGuess();
    set_guess_file(file);
}
void MucoTropterSolver::clearGuess() {
    m_guessFromAPI = MucoIterate();
    m_guessFromFile = MucoIterate();
    set_guess_file("");
    m_guessToUse.reset();
}
const MucoIterate& MucoTropterSolver::getGuess() const {
    if (!m_guessToUse) {
        if (get_guess_file() != "" && m_guessFromFile.empty()) {
            // The API should make it impossible for both guessFromFile and
            // guessFromAPI to be non-empty.
            assert(m_guessFromAPI.empty());
            // No need to load from file again if we've already loaded it.
            MucoIterate guessFromFile(get_guess_file());
            guessFromFile.isCompatible(getProblem(), true);
            m_guessFromFile = guessFromFile;
            m_guessToUse.reset(&m_guessFromFile);
        } else {
            // This will either be a guess specified via the API, or empty to
            // signal that tropter should use the default guess.
            m_guessToUse.reset(&m_guessFromAPI);
        }
    }
    return m_guessToUse.getRef();
}

void MucoTropterSolver::printOptimizationSolverOptions(std::string solver) {
    if (solver == "ipopt") {
        tropter::optimization::IPOPTSolver::print_available_options();
    } else {
        std::cout << "No info available for " << solver << " options." <<
                std::endl;
    }
}

MucoSolution MucoTropterSolver::solveImpl() const {
    const Stopwatch stopwatch;

    auto ocp = getTropterProblem();

    checkPropertyInSet(*this, getProperty_verbosity(), {0, 1, 2});

    if (get_verbosity()) {
        std::cout << std::string(79, '=') << "\n";
        std::cout << "MucoTropterSolver starting.\n";
        std::cout << std::string(79, '-') << std::endl;
        getProblem().printDescription();
        // We can provide more detail about our problem than tropter can.
        // ocp->print_description();
    }

    // Apply settings/options.
    // -----------------------
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    int N = get_num_mesh_points();

    checkPropertyInSet(*this, getProperty_optim_solver(), {"ipopt", "snopt"});

    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal",
            get_optim_solver(), N);

    dircol.set_verbosity(get_verbosity() >= 1);

    auto& optsolver = dircol.get_opt_solver();

    checkPropertyInRangeOrSet(*this, getProperty_optim_max_iterations(),
            0, std::numeric_limits<int>::max(), {-1});
    if (get_optim_max_iterations() != -1)
        optsolver.set_max_iterations(get_optim_max_iterations());

    checkPropertyInRangeOrSet(*this,
            getProperty_optim_convergence_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    if (get_optim_convergence_tolerance() != -1)
        optsolver.set_convergence_tolerance(get_optim_convergence_tolerance());
    checkPropertyInRangeOrSet(*this,
            getProperty_optim_constraint_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    if (get_optim_constraint_tolerance() != -1)
        optsolver.set_constraint_tolerance(get_optim_constraint_tolerance());

    optsolver.set_hessian_approximation(get_optim_hessian_approximation());

    if (get_optim_solver() == "ipopt") {
        checkPropertyInRangeOrSet(*this, getProperty_optim_ipopt_print_level(),
                0, 12, {-1});
        if (get_verbosity() < 2) {
            optsolver.set_advanced_option_int("print_level", 0);
        } else {
            if (get_optim_ipopt_print_level() != -1) {
                optsolver.set_advanced_option_int("print_level",
                        get_optim_ipopt_print_level());
            }
        }
    }

    checkPropertyInSet(*this, getProperty_optim_sparsity_detection(),
            {"random", "initial-guess"});
    optsolver.set_sparsity_detection(get_optim_sparsity_detection());

    // Set advanced settings.
    //for (int i = 0; i < getProperty_optim_solver_options(); ++i) {
    //    optsolver.set_advanced_option(TODO);
    //}

    tropter::Iterate tropIterate = convert(getGuess());
    tropter::Solution tropSolution = dircol.solve(tropIterate);

    if (get_verbosity()) {
        dircol.print_constraint_values(tropSolution);
    }

    MucoSolution mucoSolution = convert(tropSolution);

    // TODO move this to convert():
    MucoSolver::setSolutionStats(mucoSolution, tropSolution.success,
            tropSolution.status, tropSolution.num_iterations);

    if (get_verbosity()) {
        std::cout << std::string(79, '-') << "\n";
        std::cout << "Elapsed real time: "
                << stopwatch.getElapsedTimeFormatted() << ".\n";
        if (mucoSolution) {
            std::cout << "MucoTropterSolver succeeded!\n";
        } else {
            // TODO cout or cerr?
            std::cout << "MucoTropterSolver did NOT succeed:\n";
            std::cout << "  " << mucoSolution.getStatus() << "\n";
        }
        std::cout << std::string(79, '=') << std::endl;
    }

    return mucoSolution;
}
