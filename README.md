# piper dual arm teleoperation

This is a dual robotic arm plan & control stack.

## hardware

Dual AgileX Piper 6-axis manipulator.

## feature

* dual arm end effector controlled by VR remote controllers.
* inverse kinematics approximation.
* abnormal motion behaviour filter.
* joint PD control with inverse dynamics feedforward torque.
* mujoco simulation and real hardware control.

## todo

- [ ] Refactor arm model as rigid body tree.
- [ ] Create a config file to replace some constants like ArmModel::num_dof_ and ArmPlanner::num_plan_waypoint_.
- [ ] Remove B-Spline and other interpolation methods in `TrajectoryBuffer`.