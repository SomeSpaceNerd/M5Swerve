# Software

This directory contains all of the software required to operate M5Swerve

## Driver Station

This is the PC-Side software designed to allow for remotely controlling and commanding the robot.

It was using in Qt Creator and heavily relies on the Qt framework, so the best way to compile and edit the program is to download Qt Creator and load it as a project.

## Robot Code

This is the actual code that runs on the robot controller itself, containing the framework (runtime, HAL, utils, etc) as well as M5Swerve specific code for TeleOp and swerve kinematics

Compilation is done on the robot controller itself and is designed to work best with the Debian images from M5Stack. You can use the deploy_linux batch file or python file directly to deploy your code to the robot controller.



<sub>If you’re wondering why the framework is so overbuilt and called “bellman” it’s because it was originally part of a much larger unreleased project, and because the project is not public yet I am using its codename “bellman” which is a reference to mathematician and programmer  Richard Ernest Bellman</sub>
