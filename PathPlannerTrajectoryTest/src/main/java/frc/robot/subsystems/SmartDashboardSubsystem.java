// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {}

  void updateEncoders() {
    SmartDashboard.putNumber("Left Encoder Value", RobotContainer.driveSubsystem.getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder Value", RobotContainer.driveSubsystem.getRightEncoder());
    SmartDashboard.putNumber("Left Encoder Velocity", RobotContainer.driveSubsystem.getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Encoder Velocity", RobotContainer.driveSubsystem.getRightEncoderVelocity());
  }

  void updatePigeon() {
    //SmartDashboard.putNumber("Pitch value", RobotContainer.imuSubsystem.getPitch());
    //SmartDashboard.putNumber("Roll value", RobotContainer.imuSubsystem.getRoll());
    SmartDashboard.putNumber("Pidgeon Heading value", RobotContainer.imuSubsystem.getHeading());
    
  }
  
  public void updateAllDisplays() {
    updateEncoders();
    updatePigeon();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
