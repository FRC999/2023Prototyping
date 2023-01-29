// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPartsWithCommands extends SequentialCommandGroup {
  /** Creates a new RunTrajectorySequentialCommandGroup. */

   PathPlannerTrajectory trajectoryPath;
   HashMap<String, Command> eventMap;


  public PathPartsWithCommands(String trajectory, double maxVelocity, double maxAcceleration) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    eventMap = new HashMap<>();
    eventMap.put("mymessage", new PrintCommand("Passed marker 1"));

    // Read the trajectory from a file
    trajectoryPath = PathPlanner.loadPath(trajectory, new PathConstraints(maxVelocity, maxAcceleration));

    FollowPathWithEvents commandWithMarkersTest = new FollowPathWithEvents(
      new AutonomousTrajectoryRioCommand(trajectory),
      trajectoryPath.getMarkers(),
      eventMap);

    addCommands(
      new PrintCommand("Starting Trajectory String test"),
      new InstantCommand(RobotContainer.driveSubsystem::zeroEncoders, RobotContainer.driveSubsystem),
      new InstantCommand(RobotContainer.imuSubsystem::zeroHeading, RobotContainer.imuSubsystem),
      new InstantCommand( () -> RobotContainer.driveSubsystem.resetOdometry(trajectoryPath.getInitialPose()) ),  // Set the initial pose of the robot to the one in a trajectory
      commandWithMarkersTest,
      new PrintCommand("End Trajectory")
    );
  }
  
  public PathPartsWithCommands(String trajectory) {

    this(trajectory, DriveConstants.maxVelocityDefault, DriveConstants.maxAccelerationDefault);
    System.out.println("*** Run trajectory "+ trajectory);
  }

}
