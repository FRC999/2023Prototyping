// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class BalanceOnCharger extends CommandBase {
  /** Creates a new BalanceOnCharger. */
  double targetPitch;
  public BalanceOnCharger(double targetPitch) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem, RobotContainer.pigeonIMUSubsystem);
    this.targetPitch=targetPitch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("Starting Auto-Balance command");

    // Prepare motors, encoders and IMU for the balance
    RobotContainer.driveSubsystem.ConfigureMotorsForBalancePitch();

    // Run the balance
    // Since we apply the coefficient, we can supply the angle vs the raw units
    RobotContainer.driveSubsystem.balanceRobotToPitch(targetPitch);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Real-time PID error
    System.out.println("PID CL Error: " + RobotContainer.driveSubsystem.getRightError() + " T: "+RobotContainer.driveSubsystem.getRightTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Restore motor configuraiton back to driving
    RobotContainer.driveSubsystem.ConfigureMotorsForDriving();

    System.out.println("End of the Balance command. Interrupted:"+interrupted);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
