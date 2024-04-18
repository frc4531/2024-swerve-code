# This is to help vscode
from typing import TYPE_CHECKING

from wpilib._wpilib import SmartDashboard, wait
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry._geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics._kinematics import SwerveModuleState
from wpimath.trajectory import TrapezoidProfile

# from pathplannerlib.swerve_path_controller import SwervePathController
# from commands.drive_follow_path import DriveFollowPath

from subsystems.drive_subsystem import DriveSubsystem

if TYPE_CHECKING:
    from robot import Robot

import math

from commands2 import (
    CommandBase,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
)
from wpilib import Timer
from wpimath import controller, geometry
from wpimath.kinematics import ChassisSpeeds
from pathplannerlib import PathPlanner

from constants.swerve_constants import AutoConstants


class DriveTrajectory(CommandBase):
    def __init__(self, drive_sub: DriveSubsystem, path, max_vel, max_accel, reversed):
        super().__init__()
        self.drive_sub = drive_sub
        self.path = path
        self.target = PathPlanner.loadPath(self.path, max_vel, max_accel, reversed)

        self.addRequirements(self.drive_sub)
        # self.robot.drivetrain.set_gyro(rotation.degrees())
        # self.robot.drivetrain.set_yaw(0)

        self.drive_sub.reset_odometry(self.target.sample(0).pose)
        self.timer = Timer()

        self.h_controller = betterHolonomicDriveController()
        self.timer.start()

    def execute(self):
        target_state = self.target.sample(self.timer.get())
        current_pose = self.robot.drivetrain.get_pose()
        # print(currentPose)
        self.h_controller.drive(self.robot, current_pose, target_state)

    def isFinished(self):
        finished = self.timer.hasElapsed(self.target.getTotalTime())
        print(finished)
        return finished

    def end(self, interrupted):
        self.timer.stop()
        # self.robot.drivetrain.defense()
        pass

    def interrupted(self):
        self.end(True)


class betterHolonomicDriveController:
    def __init__(self):
        """
        Creates drive controller
        """
        self.x_controller = controller.PIDController(AutoConstants.kPXController, 0, 0)
        self.y_controller = controller.PIDController(AutoConstants.kPYController, 0, 0)
        self.theta_controller = controller.PIDController(
            AutoConstants.kPThetaController, 0, 0
        )
        self.x_controller.reset()
        self.y_controller.reset()
        self.theta_controller.reset()

    def calculate(self, currentPose, targetState):
        """
        Calculates chassis speeds
        @parameter (Pose2d) currentPose		Robot's current pose
        @parameter (State) targetState		Robot's target state
        @returns (tuple) (vx, vy, omega)	Robot's left-right, forward-backward, and angular velocities (m/s, m/s, rad/s)
        """
        # calculates the percent outputs in the x, y, and theta directions
        vx = self.x_controller.calculate(currentPose.X(), targetState.pose.X())
        vy = self.y_controller.calculate(currentPose.Y(), targetState.pose.Y())
        # current and target angles must be put into the range [-math.pi, math.pi)
        targetHeading = self.optimize(currentPose, targetState)
        omega = self.theta_controller.calculate(
            currentPose.rotation().radians(), targetHeading
        )

        return (vx, vy, omega)

    def optimize(self, currentPose, targetState):
        """
        .enableContinuousInput() doesn't work so we have this
        @parameter (Pose2d) currentPose		Robot's current pose
        @parameter (State) targetState		Robot's target state
        @returns (double) targetHeading		optimized heading
        """
        currentHeading = currentPose.rotation().radians()

        targetHeading = targetState.holonomicRotation.radians()
        if targetHeading - currentHeading > math.pi:
            targetHeading -= 2 * math.pi
        elif targetHeading - currentHeading < -math.pi:
            targetHeading += 2 * math.pi

        return targetHeading

    def drive(self, robot, currentPose, targetState):
        """
        Calculates chassis speeds and drives the robot
        """
        calcs = self.calculate(currentPose, targetState)
        left_right = calcs[0]
        forward_back = calcs[1]
        omega = calcs[2]
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx=left_right,
            vy=forward_back,
            omega=omega,
            robotAngle=currentPose.rotation(),
        )

        robot.drivetrain.drive(
            Translation2d(left_right, forward_back),  # Switch params?
            omega,
            field_relative=True,
            is_open_loop=False,
        )
        robot.drivetrain.periodic()
