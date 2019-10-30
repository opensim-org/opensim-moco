
import numpy as np
import pylab as pl

import opensim as osim


model = osim.Model()
model.setName("isometric_muscle")
model.set_gravity(osim.Vec3(9.81, 0, 0))
body = osim.Body('body', 1.0, osim.Vec3(0), osim.Inertia(0))
model.addComponent(body)

# Allows translation along x.
joint = osim.SliderJoint("joint", model.getGround(), body)
coord = joint.updCoordinate(osim.SliderJoint.Coord_TranslationX)
coord.setName("height")
model.addComponent(joint)

actu = osim.DeGrooteFregly2016Muscle()
actu.setName("actuator")
actu.set_max_isometric_force(15.0)
actu.set_optimal_fiber_length(0.10)
actu.set_tendon_slack_length(0.05)
actu.set_tendon_strain_at_one_norm_force(0.10)
actu.addNewPathPoint("origin", model.updGround(), osim.Vec3(0))
actu.addNewPathPoint("insertion", body, osim.Vec3(0))
actu.set_ignore_tendon_compliance(True)
model.addForce(actu)

body.attachGeometry(osim.Sphere(0.05))

study = osim.MocoStudy()
problem = study.updProblem()
problem.setModel(model);
problem.setTimeBounds(0, 1.0)
initHeight = 0.18
finalHeight = 0.13;
problem.setStateInfo(
    "/joint/height/value", [0.13, 0.19], initHeight, finalHeight)
problem.setStateInfo("/joint/height/speed", [-1, 1], 0, 0)
problem.setControlInfo("/forceset/actuator", [0.01, 1])

problem.addGoal(osim.MocoControlGoal())

solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(5)
solver.set_multibody_dynamics_mode("implicit")
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)

# guess = solver.createGuess()
# guess.setControl('/forceset/actuator',
#                  osim.createVectorLinspace(guess.getNumTimes(), 0, 0))
# solver.setGuess(guess)

solution = study.solve()
# study.visualize(solution)


num_iterations = solution.getNumIterations()

fig = pl.figure()

pl.plot(solution.getTimeMat(), solution.getStatesTrajectoryMat())
pl.legend(solution.getStateNames())
fig.savefig('exampleHangingMuscle_solution.png', dpi=600)

class HermiteSpline(object):
    """
    Van Loan, C.F., Introduction to Scientific Computing, page 109-110
    """
    def __init__(self, t, y, ydot):
        self.t = t
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
        while t > self.t[interval + 1]:
            interval += 1
        return interval
    def __call__(self, t):
        y_out = np.empty_like(t)
        for i in range(len(t)):
            interval = self.locate(t[i])
            out = (self.d[interval] * (t[i] - self.t[interval + 1]) + self.c[
                interval])
            out = out * (t[i] - self.t[interval]) + self.b[interval]
            out = out * (t[i] - self.t[interval]) + self.a[interval]
            y_out[i] = out
        return y_out

model.initSystem()

fig = pl.figure()
ax1 = fig.add_subplot(1, 1, 1)

fig2 = pl.figure()
ax2 = fig2.add_subplot(1, 1, 1)
for n in range(num_iterations + 1):
    solver.set_optim_max_iterations(n)

    solution = study.solve()
    solution.unseal()


    N = solution.getNumTimes()
    states = solution.exportToStatesTrajectory(problem)

    modelCopy = osim.Model(model)
    osim.prescribeControlsToModel(solution, modelCopy,
                                  "PiecewiseLinearFunction")
    modelCopy.initSystem()

    activationdot = np.empty(N)
    # ntfdot = np.empty(N)
    for itime in range(N):
        state = states[itime]
        modelCopy.realizeAcceleration(state)
        activationdot[itime] = state.getZDot().get(0)
        # ntfdot[itime] = state.getZDot().get(1)

    time = solution.getTimeMat()[::2]
    activation = solution.getStateMat('/forceset/actuator/activation')[::2]
    # ntf = solution.getStateMat('/forceset/actuator/normalized_tendon_force')[::2]

    a_spline = HermiteSpline(time, activation, activationdot[::2])
    # ntf_spline = HermiteSpline(time, ntf, ntfdot[::2])
    # print('DEBUG ', ntf, ntfdot[::2])

    t_plot = np.linspace(time[0], time[-1], 400)
    a_plot = a_spline(t_plot)
    # ntf_plot = ntf_spline(t_plot)


    color = (1 - (n + 1) / float(num_iterations + 1)) * np.array([1, 1, 1])
    ax1.plot(t_plot, a_plot, color=color)
    ax1.plot(time, activation, color=color, linestyle='', marker='o',
             markersize=2)

    # ax2.plot(t_plot, ntf_plot, color=color)
    # ax2.plot(time, ntf, color=color, linestyle='', marker='o',
    #          markersize=2)

    tmid = solution.getTimeMat()[1::2]
    activationmid = a_spline(tmid)
    # ntfmid = ntf_spline(tmid)
    for itime in range(len(tmid)):
        length = 0.10
        slope = activationdot[1::2][itime]
        tan = slope
        # TODO: This trig is incorrect when not using equal axes.
        cos = np.cos(np.arctan(tan))
        sin = np.sin(np.arctan(tan))
        t = tmid[itime]
        a = activationmid[itime]
        ax1.plot([t - 0.5 * length * cos, t + 0.5 * length * cos],
                 [a - 0.5 * length * sin, a + 0.5 * length * sin],
                 color='tab:blue')

        # slope = ntfdot[1::2][itime]
        # tan = slope
        # cos = np.cos(np.arctan(tan))
        # sin = np.sin(np.arctan(tan))
        # ntf_value = ntfmid[itime]
        # ax2.plot([t - 0.5 * length, t + 0.5 * length],
        #          [ntf_value - 0.5 * length * slope, ntf_value + 0.5 * length * slope],
        #          color='tab:blue')




fig.savefig('exampleHangingMuscle_activation.png', dpi=600)
# fig2.savefig('exampleHangingMuscle_tendon_force.png', dpi=600)
print('Number of iterations: {}'.format(num_iterations))
#
#
#
#
#
