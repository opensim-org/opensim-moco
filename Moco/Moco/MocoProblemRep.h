#ifndef MOCO_MOCOPROBLEMREP_H
#define MOCO_MOCOPROBLEMREP_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoProblemRep.h                                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

#include "osimMocoDLL.h"
#include "MocoVariableInfo.h"
#include "MocoCost/MocoCost.h"
#include "MocoConstraint.h"
#include "MocoParameter.h"
#include "Components/DiscreteForces.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <simbody/internal/Force_DiscreteForces.h>

namespace OpenSim {

class MocoProblem;

/// The primary intent of this class is for use by MocoSolver%s, but users
/// can also use this class to apply parameter values to the model
/// and evaluate cost terms.
/// This class also checks the MocoProblem for various errors.
/// To get an instance of this class, use MocoProblem::createRep().
/// This interface currently supports only single-phase problems.
/// This class stores a reference (not a copy) to the original MocoProblem
/// from which it was created.
class OSIMMOCO_API MocoProblemRep {
public:
    MocoProblemRep() = default;
    MocoProblemRep(const MocoProblemRep&) = delete;
    MocoProblemRep& operator=(const MocoProblemRep&) = delete;
    MocoProblemRep(MocoProblemRep&& source)
            : m_problem(std::move(source.m_problem)) {
        if (m_problem) initialize();
    }
    MocoProblemRep& operator=(MocoProblemRep&& source) {
        m_problem = std::move(source.m_problem);
        if (m_problem) initialize();
        return *this;
    }

    const std::string& getName() const;

    /// Get a reference to the copy of the model being used by this 
    /// MocoProblemRep.
    /// This model is the same instance as that given to MocoCost,
    /// MocoParameter, and MocoPathConstraint.
    /// TODO
    const Model& getModel() const { return m_model; }
    const Model& getModelIgnoringConstraints() const 
    {   return m_model_ignoring_constraints; }
    int getNumStates() const { return (int)m_state_infos.size(); }
    int getNumControls() const { return (int)m_control_infos.size(); }
    int getNumParameters() const { return (int)m_parameters.size(); }
    /// Get the state names of all the state infos.
    std::vector<std::string> createStateInfoNames() const;
    /// Get the control names of all the control infos.
    std::vector<std::string> createControlInfoNames() const;
    /// Get the names of all the parameters.
    std::vector<std::string> createParameterNames() const;
    /// Get the names of all the MocoPathConstraints.
    std::vector<std::string> createPathConstraintNames() const;
    /// Get the names of all the Lagrange multiplier infos.
    std::vector<std::string> createMultiplierInfoNames() const;
    /// Get the constraint names of all the kinematic constraints. Note: this
    /// should only be called after initialize().
    std::vector<std::string> createKinematicConstraintNames() const;
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MocoInitialBounds getTimeInitialBounds() const;
    /// @copydoc getTimeInitialBounds()
    MocoFinalBounds getTimeFinalBounds() const;
    /// Get information for state variables. If info was not specified for
    /// a coordinate value, the coordinate range is used for the bounds.
    /// If info was not specified for a coordinate speed, the
    /// default_speed_bounds property is used.
    const MocoVariableInfo& getStateInfo(const std::string& name) const;
    /// Get information for actuator controls. If info was not specified for
    /// an actuator, the actuator's min and max control are used for the bounds.
    const MocoVariableInfo& getControlInfo(const std::string& name) const;
    const MocoParameter& getParameter(const std::string& name) const;
    /// Get a MocoPathConstraint from this MocoPhase. Note: this does not
    /// include MocoKinematicConstraints, use getKinematicConstraint() instead.
    const MocoPathConstraint& getPathConstraint(const std::string& name) const;

    /// Get the number of scalar path constraints in the MocoProblem. This does
    /// not include kinematic constraints equations.
    int getNumPathConstraintEquations() const {
        OPENSIM_THROW_IF(m_num_path_constraint_equations == -1, Exception,
                "The number of scalar path constraint equations is not "
                "available until after initialization.");
        return m_num_path_constraint_equations;
    }
    /// Given a kinematic constraint name, get a vector of MocoVariableInfos
    /// corresponding to the Lagrange multipliers for that kinematic constraint.
    /// Note: Since these are created directly from model constraint
    /// information, this should only be called after initialization. TODO
    const std::vector<MocoVariableInfo>&
    getMultiplierInfos(const std::string& kinematicConstraintInfoName) const;
    /// Get a MocoKinematicConstraint from this MocoPhase. Note: this does not
    /// include MocoPathConstraints, use getPathConstraint() instead. Since
    /// these are created directly from model information, this should only be
    /// called after initialization. TODO
    const MocoKinematicConstraint&
    getKinematicConstraint(const std::string& name) const;
    /// Get the number of scalar kinematic constraints in the MocoProblem. This
    /// does not include path constraints equations.
    int getNumKinematicConstraintEquations() const {
        OPENSIM_THROW_IF(m_num_kinematic_constraint_equations == -1, Exception,
                "The number of scalar kinematic constraint equations is not "
                "available until after initialization.");
        return m_num_kinematic_constraint_equations;
    }
    /// TODO
    const std::string& getConstraintForcesPath() const 
    {   return m_constraint_forces_path; }

    /// Print a description of this problem, including costs and variable
    /// bounds. By default, the description is printed to the console (cout),
    /// but you can provide your own stream.
    void printDescription(std::ostream& stream = std::cout) const;

    /// @name Interface for solvers
    /// These functions are for use by MocoSolver%s, but can also be called
    /// by users for debugging.
    /// @{
    /// Calculate the sum of integrand over all the integral cost terms in this
    /// phase for the provided state. That is, the returned value is *not* an
    /// integral over time.
    SimTK::Real calcIntegralCost(const SimTK::State& state) const {
        SimTK::Real integrand = 0;
        for (const auto& cost : m_costs) {
            integrand += cost->calcIntegralCost(state);
        }
        return integrand;
    }
    /// Calculate the sum of all the endpoint cost terms in this phase.
    SimTK::Real calcEndpointCost(const SimTK::State& finalState) const {
        SimTK::Real sum = 0;
        // TODO cannot use controls.
        for (const auto& cost : m_costs) {
            sum += cost->calcEndpointCost(finalState);
        }
        return sum;
    }
    /// Calculate the errors in all the scalar path constraint equations in this
    /// phase.
    void calcPathConstraintErrors(const SimTK::State& state,
            SimTK::Vector& errors) const {

        OPENSIM_THROW_IF(
            errors.size() != getNumPathConstraintEquations(), Exception,
            "The size of the errors vector passed is not consistent with the "
            "number of scalar path constraint equations in this MocoProblem.");

        for (const auto& pc : m_path_constraints) {
            pc->calcPathConstraintErrors(state, errors);
        }
    }
    /// Calculate the errors in all the scalar kinematic constraint equations in
    /// this phase. This may not be the most efficient solution for solvers, but
    /// is rather intended as a convenience method for a quick implementation or
    /// for debugging model constraints causing issues in an optimal control
    /// problem.
    SimTK::Vector calcKinematicConstraintErrors(const SimTK::State& state) const
    {
        SimTK::Vector errors(getNumKinematicConstraintEquations(), 0.0);
        int index = 0;
        int thisConstraintNumEquations;
        for (int i = 0; i < (int)m_kinematic_constraints.size(); ++i) {
            thisConstraintNumEquations =
                m_kinematic_constraints[i].getConstraintInfo()
                                          .getNumEquations();

            SimTK::Vector theseErrors(thisConstraintNumEquations,
                    errors.getContiguousScalarData() + index, true);
            m_kinematic_constraints[i].calcKinematicConstraintErrors(
                    getModel(), state, theseErrors);

            index += thisConstraintNumEquations;
        }
        return errors;
    }

    /// Apply paramater values to the model passed to initialize() within the
    /// current MocoProblem. Values must be consistent with the order of
    /// parameters returned from createParameterNames().
    ///
    /// Note: initSystem() must be called on the model after calls to this
    /// method in order for provided parameter values to be applied to the
    /// model.
    void applyParametersToModel(const SimTK::Vector& parameterValues) const;
    /// @}

private:
    explicit MocoProblemRep(const MocoProblem& problem);
    friend MocoProblem;

    void initialize();

    const MocoProblem* m_problem;

    Model m_model;
    Model m_model_ignoring_constraints;
    std::string m_constraint_forces_path = "constraint_forces";

    std::unordered_map<std::string, MocoVariableInfo> m_state_infos;
    std::unordered_map<std::string, MocoVariableInfo> m_control_infos;

    std::vector<std::unique_ptr<MocoParameter>> m_parameters;
    std::vector<std::unique_ptr<MocoCost>> m_costs;
    std::vector<std::unique_ptr<MocoPathConstraint>> m_path_constraints;
    int m_num_path_constraint_equations = -1;
    int m_num_kinematic_constraint_equations = -1;
    std::vector<MocoKinematicConstraint> m_kinematic_constraints;
    std::map<std::string, std::vector<MocoVariableInfo>> m_multiplier_infos_map;
};

} // namespace OpenSim


#endif // MOCO_MOCOPROBLEMREP_H
