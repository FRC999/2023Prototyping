// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NetworkTablesSubsystem extends SubsystemBase {
  /** Creates a new NetworkTablesSubsystem. */

  private NetworkTableInstance ntInst;

  public NetworkTablesSubsystem() {
    ntInst = NetworkTableInstance.getDefault();
  }

  public double[] getRobotPose() {

    double[] robotPose = ntInst.getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    return robotPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
