# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleProjectile.py                                         #
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

import pylab as pl

import opensim as osim

study = osim.MocoStudy()
problem = study.updProblem()

model = osim.ModelFactory.createPlanarPointMass()
model.finalizeFromProperties()
drag_x = osim.SpringGeneralizedForce('tx')
drag_x.setName('drag_x')
drag_x.setViscosity(5.0)
model.addForce(drag_x)
drag_y = osim.SpringGeneralizedForce('ty')
drag_y.setName('drag_y')
drag_y.setViscosity(5.0)
model.addForce(drag_y)

model.finalizeConnections()

problem.setModel(model)
problem.setTimeBounds(0, 0.7)
x_final = 1.0
problem.setStateInfo('/jointset/tx/tx/value', [], 0, x_final)
problem.setStateInfo('/jointset/ty/ty/value', [], 0, 0)
problem.setControlInfoPattern('.*', 0)

goal = osim.MocoOutputEndpointGoal('initial_velocity')
goal.setOutputPath('/bodyset/body|linear_velocity')
goal.setEndpoint('initial')
problem.addGoal(goal)

solution = study.solve()

num_iterations = solution.getNumIterations()

fig = pl.figure()
ax = fig.add_subplot(1, 1, 1)
ax.axis('equal')

for n in range(num_iterations):
    solver = study.initCasADiSolver()
    solver.set_optim_max_iterations(n)

    solution = study.solve().unseal()

    x = solution.getStateMat('/jointset/tx/tx/value')
    y = solution.getStateMat('/jointset/ty/ty/value')
    ax.plot(x, y)

fig.savefig('exampleProjectile_solution.png')








