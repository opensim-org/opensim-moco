User Guide
==========

The document contains instructions on getting started with MoCo and best 
practices for solving direct collocation problems.

Getting Started
---------------
TODO

Best Practices
--------------
- Replace locked OpenSim::Model locked coordinates with OpenSim::WeldJoints 
wherever possible. This reduces the problem size since variables or constraints
associated with the coordinate are removed. In addition, enforcing the locked
coordinate constraint seems to cause numerical issues for some problems (e.g 
problems with tracking costs seem to have trouble tracking data for other 
coordinates when also trying to enforce locked coordinate constraints).

- Keep model actuators limits close the maximum strength necessary to actuate
the system. This can be achieved by setting the bounds accordingly, but it is
recommended to set the optimal force for the actuator and keep the bounds 
consistent (i.e. [-1 1] for bidirectional actuators and [0 1] for undirectional
actuators and muscles).
