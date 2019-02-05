#ifndef MOCO_MOCOSOLVER_H
#define MOCO_MOCOSOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoSolver.h                                                 *
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

#include "MocoIterate.h"

#include "MocoProblemRep.h"

#include <OpenSim/Common/Object.h>

#include <SimTKcommon/internal/ReferencePtr.h>

namespace OpenSim {

class MocoTool;

// TODO create typed versions?
/*
class MocoSolverOption : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoSolverOption, Object);
public:
    OpenSim_DECLARE_PROPERTY(value, std::string, "TODO");
    MocoSolverOption() {
        constructProperties();
    }
};
 */

// TODO what's the desired behavior upon copy?

/// Once the solver is created, you should not make any edits to the
/// MocoProblem. If you do, you must call resetProblem(const MocoProblem&
/// problem).
class OSIMMOCO_API MocoSolver : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(MocoSolver, Object);
public:

    MocoSolver() = default;

    /// This calls resetProblem() with the provided problem.
    explicit MocoSolver(const MocoProblem& problem);

    virtual ~MocoSolver() = default;

    /// Call this to prepare the solver for use on the provided problem.
    // TODO can only call once?
    // TODO @precondition The problem is well-posed (MocoProblem::isWellPosed
    // ()). Move isWellPosed() to Solver, since evaluating this might require
    // creating the solver.
    void resetProblem(const MocoProblem& problem);

protected:

    //OpenSim_DECLARE_LIST_PROPERTY(options, MocoSolverOption, "TODO");

    /// This is a service for derived classes, because
    /// MocoSolution::setStatus(), MocoSolution::setSuccess(), etc. are private
    /// but this class is a friend of MocoSolution.
    static void setSolutionStats(MocoSolution&,
            bool success, const std::string& status, int numIterations);

    const MocoProblemRep& getProblemRep() const {
        return m_problemRep;
    }

private:

    /// This is called by MocoTool.
    // We don't want to make this public, as users would get confused about
    // whether they should call MocoTool::solve() or MocoSolver::solve().
    MocoSolution solve() const;
    friend MocoTool;

    /// Check that solver is capable of solving this problem.
    virtual void resetProblemImpl(const MocoProblemRep& rep) const = 0;

    /// This is the meat of a solver: solve the problem and return the solution.
    virtual MocoSolution solveImpl() const = 0;

    SimTK::ReferencePtr<const MocoProblem> m_problem;
    mutable SimTK::ResetOnCopy<MocoProblemRep> m_problemRep;

};

} // namespace OpenSim

#endif // MOCO_MOCOSOLVER_H