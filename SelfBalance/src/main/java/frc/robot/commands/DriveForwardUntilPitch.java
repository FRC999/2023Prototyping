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

  boolean continueBalance = true;
  boolean rampReached = false;
  double balancePitch = 0;
  double poorMaxClimbingPower = 0.09; //was 0.09
  double poorMaxClimbingPitch = 15; // Pitch from which we start to reduce the motor power
  double angleTolerance = 14.0;

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
    rampReached = false;
    RobotContainer.driveSubsystem.driveForward(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rampReached && continueBalance) {
    double multiplier;
    multiplier = (RobotContainer.pigeonIMUSubsystem.getPitch() - balancePitch)/poorMaxClimbingPitch;
    power = poorMaxClimbingPower * multiplier;
    power = (power<-poorMaxClimbingPower)?-poorMaxClimbingPower:power;
    power = (power>poorMaxClimbingPower)?poorMaxClimbingPower:power;

      power = (Math.abs(RobotContainer.pigeonIMUSubsystem.getPitch()) < angleTolerance)?0:power;

      RobotContainer.driveSubsystem.driveForward(power);

      System.out.println("P:"+power+" A:"+RobotContainer.pigeonIMUSubsystem.getPitch());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveSubsystem.stopRobot();
    System.out.println("Reached target pitch "+targetPitch+" I:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (! rampReached) {
     rampReached = (compareDirection)?
      (targetPitch<=RobotContainer.pigeonIMUSubsystem.getPitch()):
      (targetPitch>=RobotContainer.pigeonIMUSubsystem.getPitch())
      ;
      return rampReached && ! continueBalance; // if we reached the target pitch and do not need to continue balance, end
    } else {
      return false;
    }
  }
}
