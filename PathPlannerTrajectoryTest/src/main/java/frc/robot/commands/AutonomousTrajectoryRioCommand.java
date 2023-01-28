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
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

/**
 * Runs trajectory. The command will not update initial odometry of the robot.
 * That should be done by a separate command preceding this one.
 */
public class AutonomousTrajectoryRioCommand extends PPRamseteCommand {
  /** Creates a new AutonomousTrajectoryRioCommand. */

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
    public static final double fullMotorOutput = 1024;
    public static final double encoderUnitsPerShaftRotation = 2048;
    //double trajectoryRioPidP_Value = 0.75 * fullMotorOutput / encoderUnitsPerShaftRotation;
    //double trajectoryRioPidI_Value = 0.005 * fullMotorOutput / encoderUnitsPerShaftRotation;
    //double trajectoryRioPidD_Value = .1;
  
    // From characterization 2023 - modified
    public static final double trajectoryRioPidP_Value = 0.054151 ;
    public static final double trajectoryRioPidD_Value = 0;
    public static final double trajectoryRioPidI_Value = 0;

  TrajectoryConfig config;

  PathPlannerTrajectory trajectoryPath;
  
  public AutonomousTrajectoryRioCommand(PathPlannerTrajectory trajectoryPath) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    super(
      trajectoryPath,
      RobotContainer.driveSubsystem::getPose,
      new RamseteController(),
      new SimpleMotorFeedforward(
          ksVolts,
          kvVoltSecondsPerMeter,
          kaVoltSecondsSquaredPerMeter),
      kDriveKinematics,
      RobotContainer.driveSubsystem::getWheelSpeeds,
      new PIDController(trajectoryRioPidP_Value,
        trajectoryRioPidI_Value,
        trajectoryRioPidD_Value),
      new PIDController(trajectoryRioPidP_Value,
        trajectoryRioPidI_Value,
        trajectoryRioPidD_Value),
      // RamseteCommand passes volts to the callback
      RobotContainer.driveSubsystem::tankDriveVolts,
      false,
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
    super.initialize();
    System.out.println("Auto trajectory initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update robot odometry

    //System.out.println("O");


    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println("*** End trajectory command. Interrupted:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    RobotContainer.driveSubsystem.updateOdometry();

    return super.isFinished();
  }
}
