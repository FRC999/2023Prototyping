// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class DriveManuallyCommand extends CommandBase {
  /** Creates a new DriveManuallyCommand. */
  public DriveManuallyCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveSubsystem.driveTrainBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double move = RobotContainer.driveStick.getY() * (-1);
    double turn = RobotContainer.turnStick.getX()*(-1);

    // Note that turns are adjusted; modify adjustment as needed, depending on how sharp/fast you want the turns to be
    RobotContainer.driveSubsystem.manualDrive(move, turn * DriveConstants.turnAdjust);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
