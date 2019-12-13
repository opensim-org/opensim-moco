# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleMocoCustomEffortGoal.py                               #
# -------------------------------------------------------------------------- #
# Copyright (c) 2019 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Christopher Dembia                                              #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License"); you may    #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# TODO document

import os
import opensim as osim

class MocoCustomEffortGoal(osim.MocoGoal):
    def clone(self):
        return MocoCustomEffortGoal(self)
    def initializeOnModelImpl(self, model):
        self.setNumIntegralsAndOutputs(1, 1)
    def calcIntegrandImpl(self, state, integrand):
        print("DEBUG MocoCustomEffortGoal")
        self.getModel().realizeVelocity(state)
        controls = self.getModel().getControls(state)
        # TODO python: return values don't work like that?
        integrand = controls.normSqr()
    def calcGoalImpl(self, input, cost):
        cost.set(0, input.integral)

# Create MocoStudy.
# =================
study = osim.MocoStudy()
study.setName('sliding_mass')

# Define the optimal control problem.
# ===================================
problem = study.updProblem()

# Model (dynamics).
# -----------------
problem.setModel(osim.ModelFactory.createSlidingPointMass())

# Bounds.
# -------
# Initial time must be 0, final time can be within [0, 5].
problem.setTimeBounds(osim.MocoInitialBounds(0.), osim.MocoFinalBounds(0., 5.))

# Initial position must be 0, final position must be 1.
problem.setStateInfo('/slider/position/value', osim.MocoBounds(-5, 5),
                     osim.MocoInitialBounds(0), osim.MocoFinalBounds(1))
# Initial and final speed must be 0. Use compact syntax.
problem.setStateInfo('/slider/position/speed', [-50, 50], [0], [0])

# Applied force must be between -50 and 50.
problem.setControlInfo('/forceset/actuator', osim.MocoBounds(-50, 50))

# Cost.
# -----
problem.addGoal(osim.MocoFinalTimeGoal('final_time'))
# TODO document.
problem.addGoal(MocoCustomEffortGoal())

# Configure the solver.
# =====================
solver = study.initCasADiSolver()

# Solve the problem.
# ==================
solution = study.solve()

solution.write('sliding_mass_solution.sto')

if os.getenv('OPENSIM_USE_VISUALIZER') != '0':
    study.visualize(solution)
