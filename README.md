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

## description

`can1` is for left arm;`can2` is for right arm.

## debug

Use script/plot_arm_pos_cmd.py to parse the joint positions commands to dual arms.

## todo