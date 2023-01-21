// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LoadPathCommand extends CommandBase {
  /** Creates a new LoadPathCommand. */
  public LoadPathCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    System.out.println("TEST0");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TEST1");
    String trajectoryPath=Filesystem.getDeployDirectory() + "10footForward.wpilib.json";
    System.out.println(trajectoryPath);
    System.out.println("This path works");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
