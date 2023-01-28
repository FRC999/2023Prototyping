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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

/**
 * Runs trajectory. The command will not update initial odometry of the robot.
 * That should be done by a separate command preceding this one.
 */
public class AutonomousTrajectoryRioCommand extends PPRamseteCommand {
  /** Creates a new AutonomousTrajectoryRioCommand. */

  TrajectoryConfig config;

  PathPlannerTrajectory trajectoryPath;
  
  public AutonomousTrajectoryRioCommand(PathPlannerTrajectory trajectoryPath) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    super(
      trajectoryPath,
      RobotContainer.driveSubsystem::getPose,
      new RamseteController(),
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      RobotContainer.driveSubsystem::getWheelSpeeds,
      new PIDController(DriveConstants.trajectoryRioPidP_Value,
        DriveConstants.trajectoryRioPidI_Value,
        DriveConstants.trajectoryRioPidD_Value),
      new PIDController(DriveConstants.trajectoryRioPidP_Value,
        DriveConstants.trajectoryRioPidI_Value,
        DriveConstants.trajectoryRioPidD_Value),
      // RamseteCommand passes volts to the callback
      RobotContainer.driveSubsystem::tankDriveVolts,
      true,
      RobotContainer.driveSubsystem, RobotContainer.imuSubsystem
    );
    this.trajectoryPath = trajectoryPath;
  }

  // Run trajectory with known maximum velocity and acceleration
  /**
   * @param trajectoryName Filename containing trajectory without .path
   * @param maxVelocity    Maximum velocity m/s
   * @param maxAcceleration  Maximum acceleration m/s^2
   */
  public AutonomousTrajectoryRioCommand(String trajectoryName, double maxVelocity, double maxAcceleration){
    this(PathPlanner.loadPath(trajectoryName, new PathConstraints(maxVelocity, maxAcceleration)));
    System.out.println("initalized trajectory: "+ trajectoryName + "V:"+maxVelocity+" A:"+maxAcceleration);
  }

  // Run trajectory with default maximum velocity and acceleration
  /**
   * @param trajectoryName Filename containing trajectory without .path
   */
  public AutonomousTrajectoryRioCommand(String trajectoryName){
    this(PathPlanner.loadPath(trajectoryName, 
      new PathConstraints(DriveConstants.maxVelocityDefault, DriveConstants.maxAccelerationDefault)));
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto trajectory initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update robot odometry

    //System.out.println("O");

    //RobotContainer.driveSubsystem.updateOdometry();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("*** End trajectory command. Interrupted:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
