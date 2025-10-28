# Admittance-controller
An admittance controller for UR robot.

This package provides an ros_control implementation of admittance controller for UR robot.
Before used, a six-DOF F/T sensor should be installed at the end-effector of 
your manipulator with Topic '/external_wrench' (geometry_msgs::WrenchStampedConstPtr).

$\mathbf{M}_d \ddot{\mathbf{x}} + \mathbf{D}_d \dot{\mathbf{x}} + \mathbf{K}_d \mathbf{x} = F$



## v0.1 2025.10.28
Please rename the package name to 'ur_controllers' before catkin make. 

## References
[1] H. Nikus, “Ridgeback ur5 controller,” 2020. [Online]. Available: https://github.com/epfl-lasa/ridgeback_ur5_controller
