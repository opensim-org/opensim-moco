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

#include <OpenSim/Simulation/InverseDynamicsSolver.h>

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

    int numTimes = (int)time.size();
    int numStates = (int)state_names.size();
    int numControls = (int)control_names.size();
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
    return {time, state_names, control_names, states, controls};
}

MucoSolution convert(const tropter::OptimalControlSolution& tropSol) {
    // TODO enhance when solution contains more info than iterate.
    return convert<MucoSolution, tropter::OptimalControlSolution>(tropSol);
}

tropter::OptimalControlIterate convert(const MucoIterate& mucoIter) {
    tropter::OptimalControlIterate tropIter;
    if (mucoIter.empty()) return tropIter;

    using Eigen::Map;
    using Eigen::RowVectorXd;
    using Eigen::MatrixXd;

    const auto& time = mucoIter.getTime();
    tropIter.time = Map<const RowVectorXd>(&time[0], time.size());

    tropIter.state_names = mucoIter.getStateNames();
    tropIter.control_names = mucoIter.getControlNames();

    int numTimes = (int)time.size();
    int numStates = (int)tropIter.state_names.size();
    int numControls = (int)tropIter.control_names.size();
    const auto& states = mucoIter.getStatesTrajectory();
    const auto& controls = mucoIter.getControlsTrajectory();
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
    return tropIter;
}

tropter::Bounds convert(const MucoBounds& mb) {
    return {mb.lower, mb.upper};
}

tropter::InitialBounds convert(const MucoInitialBounds& mb) {
    return {mb.lower, mb.upper};
}
tropter::FinalBounds convert(const MucoFinalBounds& mb) {
    return {mb.lower, mb.upper};
}

template <typename T>
class MucoTropterSolver::OptimalControlProblem :
        public tropter::OptimalControlProblem<T> {
public:
    OptimalControlProblem(const MucoSolver& solver)
            : tropter::OptimalControlProblem<T>(solver.getProblem().getName()),
              m_mucoSolver(solver),
              m_mucoProb(solver.getProblem()),
              m_phase0(m_mucoProb.getPhase(0)) {
        m_model = m_phase0.getModel();
        m_state = m_model.initSystem();
    }
    void initialize_on_mesh(const Eigen::VectorXd&) const override final {
        m_mucoProb.initialize(m_model);
    }
    void calc_endpoint_cost(const T& final_time, const VectorX<T>& states,
            T& cost) const override final {
        // TODO avoid all of this if there are no endpoint costs.
        m_state.setTime(final_time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        // TODO cannot use control signals...
        m_model.updControls(m_state).setToNaN();
        cost = m_phase0.calcEndpointCost(m_state);
    }

protected:
    const MucoSolver& m_mucoSolver;
    const MucoProblem& m_mucoProb;
    const MucoPhase& m_phase0;
    Model m_model;
    mutable SimTK::State m_state;
};

template <typename T>
class MucoTropterSolver::OCPExplicit :
        public MucoTropterSolver::OptimalControlProblem<T> {
public:
    OCPExplicit(const MucoSolver& solver) : OptimalControlProblem<T>(solver) {
        this->set_time(convert(this->m_phase0.getTimeInitialBounds()),
                convert(this->m_phase0.getTimeFinalBounds()));
        auto svNamesInSysOrder =
                createStateVariableNamesInSystemOrder(this->m_model);
        for (const auto& svName : svNamesInSysOrder) {
            const auto& info = this->m_phase0.getStateInfo(svName);
            this->add_state(svName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
        for (const auto& actu :
                this->m_model.template getComponentList<Actuator>()) {
            // TODO handle a variable number of control signals.
            const auto& actuName = actu.getName();
            const auto& info = this->m_phase0.getControlInfo(actuName);
            this->add_control(actuName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
    }
    // TODO rename argument "states" to "state".
    void calc_differential_algebraic_equations(
            const tropter::DAEInput<T>& in,
            tropter::DAEOutput<T> out) const override {

        // TODO convert to implicit formulation.

        const auto& states = in.states;
        const auto& controls = in.controls;

        this->m_state.setTime(in.time);

        std::copy(states.data(), states.data() + states.size(),
                &this->m_state.updY()[0]);
        //
        // TODO do not copy? I think this will still make a copy:
        // TODO use m_state.updY() = SimTK::Vector(states.size(), states.data(), true);
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        if (this->m_model.getNumControls()) {
            auto& osimControls = this->m_model.updControls(this->m_state);
            std::copy(controls.data(), controls.data() + controls.size(),
                    &osimControls[0]);

            this->m_model.realizeVelocity(this->m_state);
            this->m_model.setControls(this->m_state, osimControls);
        }

        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.
        this->m_model.realizeAcceleration(this->m_state);
        std::copy(&this->m_state.getYDot()[0],
                &this->m_state.getYDot()[0] + states.size(),
                out.dynamics.data());
    }
    void calc_integral_cost(const T& time,
            const VectorX<T>& states,
            const VectorX<T>& controls, T& integrand) const override {
        // TODO would it make sense to a vector of States, one for each mesh
        // point, so that each can preserve their cache?
        this->m_state.setTime(time);
        std::copy(states.data(), states.data() + states.size(),
                &this->m_state.updY()[0]);
        if (this->m_model.getNumControls()) {
            auto& osimControls = this->m_model.updControls(this->m_state);
            std::copy(controls.data(), controls.data() + controls.size(),
                    &osimControls[0]);
            this->m_model.realizePosition(this->m_state);
            this->m_model.setControls(this->m_state, osimControls);
        } else {
            this->m_model.realizePosition(this->m_state);
        }
        integrand = this->m_phase0.calcIntegralCost(this->m_state);
    }
};




template <typename T>
class MucoTropterSolver::OCPImplicit :
        public MucoTropterSolver::OptimalControlProblem<T> {
public:
    OCPImplicit(const MucoSolver& solver) : OptimalControlProblem<T>(solver) {
        OPENSIM_THROW_IF(this->m_state.getNZ(), Exception, "Cannot use "
                "implicit dynamics mode if the system has auxiliary states.");
        this->set_time(convert(this->m_phase0.getTimeInitialBounds()),
                convert(this->m_phase0.getTimeFinalBounds()));
        auto svNamesInSysOrder =
                createStateVariableNamesInSystemOrder(this->m_model);
        for (const auto& svName : svNamesInSysOrder) {
            const auto& info = this->m_phase0.getStateInfo(svName);
            this->add_state(svName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
        // Add controls for udot; we call this "w".
        int NU = this->m_state.getNU();
        OPENSIM_THROW_IF(NU != this->m_state.getNQ(), Exception,
                "Internal error.");
        for (int iudot = 0; iudot < NU; ++iudot) {
            auto name = svNamesInSysOrder[iudot];
            auto leafpos = name.find("value");
            OPENSIM_THROW_IF(leafpos == std::string::npos, Exception,
                    "Internal error.");
            name.replace(leafpos, name.size(), "accel");
            // TODO how to choose bounds on udot?
            this->add_control(name, {-1000, 1000});
            this->add_path_constraint(name.substr(0, leafpos) + "residual", 0);
        }
        // Now add actuator controls.
        for (const auto& actu :
                this->m_model.template getComponentList<Actuator>()) {
            // TODO handle a variable number of control signals.
            const auto& actuName = actu.getName();
            const auto& info = this->m_phase0.getControlInfo(actuName);
            this->add_control(actuName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
    }
    // TODO rename argument "states" to "state".
    void calc_differential_algebraic_equations(
            const tropter::DAEInput<T>& in,
            tropter::DAEOutput<T> out) const override {

        const auto& states = in.states;
        const auto& controls = in.controls;

        this->m_state.setTime(in.time);

        // Kinematic differential equations
        // --------------------------------
        // qdot = u
        // TODO does not work for quaternions!
        const auto NQ = this->m_state.getNQ(); // TODO we assume NQ = NU
        const auto& u = states.segment(NQ, NQ);
        out.dynamics.head(NQ) = u;

        // Multibody dynamics: differential equations
        // ------------------------------------------
        // udot = w
        out.dynamics.segment(NQ, NQ) = controls.head(NQ);


        // Multibody dynamics: "F - ma = 0"
        // --------------------------------
        std::copy(states.data(), states.data() + states.size(),
                &this->m_state.updY()[0]);

        // TODO do not copy? I think this will still make a copy:
        // TODO use m_state.updY() = SimTK::Vector(states.size(), states.data(), true);
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        if (this->m_model.getNumControls()) {
            auto& osimControls = this->m_model.updControls(this->m_state);
            std::copy(controls.data() + NQ,
                    controls.data() + controls.size(),
                    &osimControls[0]);

            this->m_model.realizeVelocity(this->m_state);
            this->m_model.setControls(this->m_state, osimControls);
        }

        InverseDynamicsSolver id(this->m_model);
        SimTK::Vector udot(NQ, controls.data() /*, controls.data()*/);
        SimTK::Vector residual = id.solve(this->m_state, udot);

        std::copy(&residual[0], &residual[0] + residual.size(),
                out.path.data());

        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.

    }
    void calc_integral_cost(const T& time,
            const VectorX<T>& states,
            const VectorX<T>& controls, T& integrand) const override {
        const auto NQ = this->m_state.getNQ(); // TODO we assume NQ = NU

        // TODO would it make sense to a vector of States, one for each mesh
        // point, so that each can preserve their cache?
        this->m_state.setTime(time);
        std::copy(states.data(), states.data() + states.size(),
                &this->m_state.updY()[0]);

        // TODO some controls are jerks!!! not actual control signals!
        if (this->m_model.getNumControls()) {
            auto& osimControls = this->m_model.updControls(this->m_state);
            std::copy(controls.data() + NQ,
                    controls.data() + controls.size(),
                    &osimControls[0]);
            this->m_model.realizePosition(this->m_state);
            this->m_model.setControls(this->m_state, osimControls);
        } else {
            this->m_model.realizePosition(this->m_state);
        }
        integrand = this->m_phase0.calcIntegralCost(this->m_state);
    }
};

MucoTropterSolver::MucoTropterSolver() {
    constructProperties();
}

void MucoTropterSolver::constructProperties() {
    constructProperty_num_mesh_points(100);
    constructProperty_verbosity(2);
    constructProperty_dynamics_mode("explicit");
    constructProperty_optim_solver("ipopt");
    constructProperty_optim_max_iterations(-1);
    constructProperty_optim_hessian_approximation("limited-memory");
    constructProperty_optim_ipopt_print_level(-1);

    constructProperty_guess_file("");
}

// TODO rename to createTropterProblem().
// TODO avoid inconsistent tropter problem for guess versus for solving.
std::shared_ptr<const tropter::OptimalControlProblem<double>>
MucoTropterSolver::getTropterProblem() const {
    //if (!m_tropProblem) { // TODO detect if dynamics_mode has changed.
    //    m_tropProblem = std::make_shared<OCProblem<double>>(*this);
    //}
    //return m_tropProblem;
    checkPropertyInSet(*this, getProperty_dynamics_mode(), {"explicit",
                                                            "implicit"});
    if (get_dynamics_mode() == "explicit") {
        return std::make_shared<OCPExplicit<double>>(*this);
    } else if (get_dynamics_mode() == "implicit") {
        return std::make_shared<OCPImplicit<double>>(*this);
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, "Internal error.");
    }
}

void MucoTropterSolver::clearProblemImpl() {
    clearGuess();
}

void MucoTropterSolver::setProblemImpl(const MucoProblem& /*problem*/) {
    clearProblemImpl();
}

MucoIterate MucoTropterSolver::createGuess(const std::string& type) const {
    OPENSIM_THROW_IF_FRMOBJ(type != "bounds" && type != "random",
            Exception,
            "Unexpected guess type '" + type +
            "'; supported types are 'bounds' and 'random'.");
    auto ocp = getTropterProblem();

    // TODO avoid performing error checks multiple times; use
    // isObjectUpToDateWithProperties();
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    int N = get_num_mesh_points();

    checkPropertyInSet(*this, getProperty_optim_solver(), {"ipopt", "snopt"});
    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal",
            get_optim_solver(), N);

    tropter::OptimalControlIterate tropIter;
    if (type == "bounds") {
        tropIter = dircol.make_initial_guess_from_bounds();
    } else if (type == "random") {
        tropIter = dircol.make_random_iterate_within_bounds();
    }
    return convert<MucoIterate, tropter::OptimalControlIterate>(tropIter);
}

void MucoTropterSolver::setGuess(MucoIterate guess) {
    // Ensure the guess is compatible with this solver/problem.
    if (get_dynamics_mode() != "implicit") {
        // TODO check even if implicit.
        guess.isCompatible(getProblem(), true);
    }
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
        tropter::IPOPTSolver::print_available_options();
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
        ocp->print_description();
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

    // Set advanced settings.
    //for (int i = 0; i < getProperty_optim_solver_options(); ++i) {
    //    optsolver.set_advanced_option(TODO);
    //}
    //optsolver.set_advanced_option_string("print_timing_statistics", "yes");

    tropter::OptimalControlIterate tropIterate = convert(getGuess());
    tropter::OptimalControlSolution tropSolution = dircol.solve(tropIterate);

    if (get_verbosity()) {
        dircol.print_constraint_values(tropSolution);
    }

    MucoSolution mucoSolution = convert(tropSolution);
    // TODO move this to convert():
    MucoSolver::setSolutionStatusAndSuccess(mucoSolution,
            tropSolution.success, tropSolution.status);

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
