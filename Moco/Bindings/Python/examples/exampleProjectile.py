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

# TODO: suspended-mass, bouncing ball, ...

import numpy as np
import pylab as pl

import opensim as osim

study = osim.MocoStudy()
problem = study.updProblem()

model = osim.ModelFactory.createPlanarPointMass()
model.updForceSet().clearAndDestroy()
model.finalizeFromProperties()
drag_x = osim.ExpressionBasedCoordinateForce('tx', '-qdot*qdot')
drag_x.setName('drag_x')
model.addForce(drag_x)
drag_y = osim.ExpressionBasedCoordinateForce('ty', '-qdot*qdot')
drag_y.setName('drag_y')
model.addForce(drag_y)
# drag_x = osim.SpringGeneralizedForce('tx')
# drag_x.setName('drag_x')
# drag_x.setViscosity(10.0)
# model.addForce(drag_x)
# drag_y = osim.SpringGeneralizedForce('ty')
# drag_y.setName('drag_y')
# drag_y.setViscosity(10.0)
# model.addForce(drag_y)

model.finalizeConnections()

problem.setModel(model)
problem.setTimeBounds(0, 0.5)
x_final = 1.0
problem.setStateInfo('/jointset/tx/tx/value', [], 0, x_final)
problem.setStateInfo('/jointset/ty/ty/value', [], 0, 0)

goal = osim.MocoOutputEndpointGoal('initial_velocity')
goal.setOutputPath('/bodyset/body|linear_velocity')
goal.setEndpoint('initial')
problem.addGoal(goal)

problem.addGoal(osim.MocoFinalTimeGoal('time'))

solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(5)
guess = solver.createGuess()
N = guess.getNumTimes()
guess.setState('/jointset/ty/ty/value',
               osim.createVectorLinspace(N, 0.5, 0.5))
guess.setState('/jointset/tx/tx/value',
               osim.createVectorLinspace(N, 0, 1.0))
guess.setState('/jointset/tx/tx/speed',
               osim.createVectorLinspace(N, 1.0, 1.0))
guess.setState('/jointset/ty/ty/speed',
               osim.createVectorLinspace(N, 1.0, 1.0))
solver.setGuess(guess)

solution = study.solve()

num_iterations = solution.getNumIterations()

fig = pl.figure()
ax1 = fig.add_subplot(2, 1, 1)
ax1.axis('equal')

ax2 = fig.add_subplot(2, 1, 2)

class HermiteSpline(object):
    """
    Van Loan, C.F., Introduction to Scientific Computing, page 109-110
    """
    def __init__(self, t, y, ydot):
        self.t = t
        self.y = y
        self.n = len(t)
        self.a = y[:-1]
        self.b = ydot[:-1]
        difft = np.diff(t)
        diffy = np.diff(y)
        yp = diffy / difft
        self.c = (yp - ydot[:-1]) / difft
        self.d = (ydot[1:] + ydot[:-1] - 2 * yp) / (difft * difft)

    def locate(self, t):
        if t < self.t[0]:
            raise RuntimeError()
        elif t > self.t[-1]:
            raise RuntimeError()
        interval = 0
        while t > self.t[interval]:
            interval += 1
        return interval


    def eval(self, t):
        y_out = np.empty_like(t)
        for i in range(len(t)):
            interval = self.locate(t[i])
            y_out[i] = (self.a[i] + self.b[i] * (t[i] - self.t[interval]) +
                        self.c[i] * (t[i] - self.t[interval])**2 +
                        self.d[i] * (t[i] - self.t[interval])**3)
        return y_out

for n in range(num_iterations + 1):
    solver.set_optim_max_iterations(n)

    solution = study.solve()
    solution.unseal()

    t = solution.getTimeMat()[::2]

    x = solution.getStateMat('/jointset/tx/tx/value')[::2]
    y = solution.getStateMat('/jointset/ty/ty/value')[::2]
    xdot = solution.getStateMat('/jointset/tx/tx/speed')[::2]
    ydot = solution.getStateMat('/jointset/ty/ty/speed')[::2]

    x_spline = HermiteSpline(t, x, xdot)
    y_spline = HermiteSpline(t, y, ydot)

    t_plot = np.linspace(t[0], t[-1], 400)
    x_plot = x_spline.eval(t_plot)
    y_plot = y_spline.eval(t_plot)

    color = (1 - (n + 1) / float(num_iterations + 1)) * np.array([1, 1, 1])
    ax1.plot(x_plot, y_plot, color=color, marker='o', markersize=2)

    # TODO: Plot the actual spline, not just interpolating.

    # states = solution.exportToStatesTrajectory(model)

    # for itime in range(solution.getNumTimes()):
    #     length = 0.10
    #     slope = ydot[itime] / xdot[itime]
    #     tan = slope
    #     cos = np.cos(np.arctan(tan))
    #     sin = np.sin(np.arctan(tan))
    #     ax1.plot([x[itime] - 0.5 * length * cos, x[itime] + 0.5 * length * cos],
    #             [y[itime] - 0.5 * length * sin, y[itime] + 0.5 * length * sin],
    #             color='tab:blue')
    #
    # ax2.plot(t, ydot, color=color)




fig.savefig('exampleProjectile_solution.svg')
print('Number of iterations: {}'.format(num_iterations))








