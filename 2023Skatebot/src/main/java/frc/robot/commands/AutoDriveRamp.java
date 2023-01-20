// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoDriveRamp extends CommandBase {
  /** Creates a new AutoDriveRamp. */

  private boolean climbFlag = false;
  private final double climbAngle = 12.0;
  private final double balanceChange = -1.5;

  public AutoDriveRamp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
    addRequirements(RobotContainer.pigeonIMUSubsystem);
    addRequirements(RobotContainer.smartDashboard);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveSubsystem.brakeMode();
    RobotContainer.driveSubsystem.manualDrive(0, -0.268);
    System.out.println("Autonomous is running");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.driveSubsystem.manualDrive(0, 0.268);
    RobotContainer.driveSubsystem.stopRobot();
    System.out.println("end of the command:" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    climbFlag = climbFlag || RobotContainer.pigeonIMUSubsystem.getPitch() >= climbAngle;
    System.out.println(RobotContainer.pigeonIMUSubsystem.getPitch());
    return climbFlag && RobotContainer.pigeonIMUSubsystem.getPitch() < (climbAngle + balanceChange);

  }
}
