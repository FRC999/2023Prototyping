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
    double[] robotPose = RobotContainer.networkTablesSubsystem.getRobotPose();
    SmartDashboard.putNumber("BotPoseX", Math.round(robotPose[0]*10.0)/10.0);
    SmartDashboard.putNumber("BotPoseY", Math.round(robotPose[1]*10.0)/10.0);
    SmartDashboard.putNumber("BotPoseZ", Math.round(robotPose[2]*10.0)/10.0);
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
