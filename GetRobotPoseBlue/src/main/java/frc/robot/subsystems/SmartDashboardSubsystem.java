// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {
  
  }

  public void updateRobotPose() {
    double[] robotPoseOne = RobotContainer.networkTablesSubsystem.getLimelightOneRobotPose();
    SmartDashboard.putNumber("RightX", robotPoseOne[0]);
    SmartDashboard.putNumber("RightY", robotPoseOne[1]);
    SmartDashboard.putNumber("RightZ", robotPoseOne[2]);

    double[] robotPoseTwo = RobotContainer.networkTablesSubsystem.getLimelightTwoRobotPose();
    SmartDashboard.putNumber("LeftX", robotPoseTwo[0]);
    SmartDashboard.putNumber("LeftY", robotPoseTwo[1]);
    SmartDashboard.putNumber("LeftZ", robotPoseTwo[2]);

    // RobotContainer.photoVision.getCorners();
  }

  public void updateAllSubsystems() {
    updateRobotPose();
  }

  @Override
  public void periodic() {
    updateAllSubsystems();
    // This method will be called once per scheduler run
  }

}
