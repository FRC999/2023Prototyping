// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveForwardUntilPitch extends CommandBase {
  /** Creates a new DriveForwardUntilPitch. */

  double power;
  double targetPitch;
  boolean compareDirection;

  /**
   * Drive forward with "power" until "pitch" is detected . It's assumed that you start with 
   * @param power - -1..+1; positive number - forward
   * @param targetPitch   -90..90; positive pitch means front faces UP
   * @param compareDirection  true - initial pitch assumed to be lower than target; false - initial pitch assumed to be higher than target
   */
  public DriveForwardUntilPitch(double power, double targetPitch, boolean compareDirection) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem, RobotContainer.pigeonIMUSubsystem);
    this.power=power;
    this.targetPitch=targetPitch;
    this.compareDirection=compareDirection;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveSubsystem.driveForward(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveSubsystem.stopRobot();
    System.out.println("Reached target pitch "+targetPitch);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (compareDirection)?
      (targetPitch>=RobotContainer.pigeonIMUSubsystem.getPitch()):
      (targetPitch<=RobotContainer.pigeonIMUSubsystem.getPitch())
      ;
  }
}
