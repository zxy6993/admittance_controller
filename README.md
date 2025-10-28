# Admittance-controller
An admittance controller for UR robot.

This package provides an implementation of admittance controller for UR robot.
Before used, a six-DOF F/T sensor should be installed at the end-effector of 
your manipulator with Topic '/external_wrench' (geometry_msgs::WrenchStampedConstPtr).

$\mathbf{M}_d \Delta \ddot{\mathbf{x}}$


## v0.1 2025.10.28
Please rename the package name to 'ur_controllers' before catkin make. 
