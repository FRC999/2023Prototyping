// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnToAngleII extends CommandBase {
  /** Creates a new TurnToAngleII. */
  public TurnToAngleII() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem,RobotContainer.pigeonIMUSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*RobotContainer.pigeonIMUSubsystem.zeroYaw();
    System.out.println("Yaw Zeroed");
    RobotContainer.driveSubsystem.ConfigureMotorTurning();
    System.out.println("Motors Configure");
     */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // System.out.println("end of the command:" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Yaw:"+RobotContainer.pigeonIMUSubsystem.getYaw());
    return Math.abs(90 - RobotContainer.pigeonIMUSubsystem.getYaw())<RobotContainer.driveSubsystem.turnTolerance;
  }
}
