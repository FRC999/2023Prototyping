// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectoryString extends SequentialCommandGroup {
  /** Creates a new RunTrajectorySequentialCommandGroup. */

   PathPlannerTrajectory trajectoryPath;


  public RunTrajectoryString(String trajectory, double maxVelocity, double maxAcceleration) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Read the trajectory from a file
    trajectoryPath = PathPlanner.loadPath(trajectory, new PathConstraints(maxVelocity, maxAcceleration));

    addCommands(
      new PrintCommand("Starting Trajectory String test"),
      new InstantCommand(RobotContainer.driveSubsystem::zeroEncoders, RobotContainer.driveSubsystem),
      new InstantCommand(RobotContainer.imuSubsystem::zeroHeading, RobotContainer.imuSubsystem),
      new InstantCommand( () -> RobotContainer.driveSubsystem.resetOdometry(trajectoryPath.getInitialPose()) ),  // Set the initial pose of the robot to the one in a trajectory
      new AutonomousTrajectoryRioCommand(trajectoryPath),
      new PrintCommand("End First Trajectory"),
      new WaitCommand(5),
      new InstantCommand(RobotContainer.driveSubsystem::zeroEncoders, RobotContainer.driveSubsystem),
      new InstantCommand(RobotContainer.imuSubsystem::zeroHeading, RobotContainer.imuSubsystem),
      new InstantCommand( () -> RobotContainer.driveSubsystem.resetOdometry(new Pose2d(2.5, 1.0, new Rotation2d(0))) ),
      new AutonomousTrajectoryRioCommand(trajectoryPath),
      new PrintCommand("End Trajectory String test") // Run a trajectory
    );


  }
  
  public RunTrajectoryString(String trajectory) {

    this(trajectory, DriveConstants.maxVelocityDefault, DriveConstants.maxAccelerationDefault);
    System.out.println("*** Run trajectory "+ trajectory);
  }

}
