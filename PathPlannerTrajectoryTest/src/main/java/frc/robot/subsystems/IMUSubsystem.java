// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMUSubsystem extends SubsystemBase {
  /** Creates a new IMUSubsystem. */
  private final int pigeon2DeviceID = 11;  // Change to the right ID per Phoenix Tuner

  private static WPI_Pigeon2 pigeon2;
  private static Pigeon2_Faults pigeonFaults = new Pigeon2_Faults();

  public IMUSubsystem() {
    pigeon2 = new WPI_Pigeon2(pigeon2DeviceID);
  }

  public Rotation2d getRotation2d() {
    return pigeon2.getRotation2d() ;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeon2.setYaw(0);
  }

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return pigeon2.getRotation2d().getDegrees();
  }

    /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -pigeon2.getRate();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
