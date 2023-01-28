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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectorySequentialCommandGroup extends SequentialCommandGroup {
  /** Creates a new RunTrajectorySequentialCommandGroup. */

  // ==== Command Limits - meters per second
  double maxVelocity = 2;
  double maxAcceleration = 1;

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or
  // theoretically
  // for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining
  // these
  // values for your robot.

  // The next three numbers are from frankenbot 2021 numbers
  // public static final double ksVolts = 0.834;
  // public static final double kvVoltSecondsPerMeter = 0.816;
  // public static final double kaVoltSecondsSquaredPerMeter = 0.0574;

  // 2023 Frankenbot characterization numbers
  public static final double ksVolts = 0.15219;
  public static final double kvVoltSecondsPerMeter = 2.1748;
  public static final double kaVoltSecondsSquaredPerMeter = 0.49391;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 8.5;

  // Max Velocity Trajectory/Acceleration
  public static final double kMaxSpeedMetersPerSecond = 2;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1;

  public static final double kTrackwidthMeters = Units.inchesToMeters(30); // Needs to match Frankenbot - distance between the wheels
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  // Reasonable baseline values for a RAMSETE follower in units of meters and
  // seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  // Those are from the Frankenbot, so may need to rerun characteristics
  double fullMotorOutput = 1024;
  double encoderUnitsPerShaftRotation = 2048;
  //double trajectoryRioPidP_Value = 0.75 * fullMotorOutput / encoderUnitsPerShaftRotation;
  //double trajectoryRioPidI_Value = 0.005 * fullMotorOutput / encoderUnitsPerShaftRotation;
  //double trajectoryRioPidD_Value = .1;

  // From characterization 2023 - modified
  double trajectoryRioPidP_Value = 0.054151 ;
  double trajectoryRioPidD_Value = 0;
  double trajectoryRioPidI_Value = 0;


  DifferentialDriveVoltageConstraint autoVoltageConstraint;
  TrajectoryConfig config;

  PathPlannerTrajectory trajectoryPath;


  public RunTrajectorySequentialCommandGroup(String trajectory, double maxVelocity, double maxAcceleration) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Read the trajectory from a file
    trajectoryPath = PathPlanner.loadPath(trajectory, new PathConstraints(maxVelocity, maxAcceleration));

    // Setup constants for the trajectory driving
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            ksVolts,
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        10);

    addCommands(
      new InstantCommand( () -> RobotContainer.driveSubsystem.resetOdometry(trajectoryPath.getInitialPose()) ),  // Set the initial pose of the robot to the one in a trajectory
      new AutonomousTrajectoryRioCommand(trajectoryPath) // Run a trajectory
    );


  }
  
  public RunTrajectorySequentialCommandGroup(String trajectory) {
    this(trajectory, DriveConstants.maxVelocityDefault, DriveConstants.maxAccelerationDefault);
  }

}
