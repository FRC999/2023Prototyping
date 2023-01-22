// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.PigeonIMU_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMUSubsystem extends SubsystemBase {
  /** Creates a new IMUSubsystem. */
  private WPI_TalonSRX talonMotor;
  private int talonMotorID = 4;

  private final int pigeon2DeviceID = 11;  // Change to the right ID per Phoenix Tuner

  private static WPI_PigeonIMU pigeon;
  private static PigeonIMU_Faults pigeonFaults = new PigeonIMU_Faults();

  public IMUSubsystem() {
    talonMotor = new WPI_TalonSRX(talonMotorID);
    pigeon = new WPI_PigeonIMU(talonMotor);
  }

  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d() ;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeon.setYaw(0);
    pigeon.setFusedHeading(0);
    System.out.println("Yaw and Fused Heading set");
  }

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return pigeon.getRotation2d().getDegrees();
  }

    /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -pigeon.getRate();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
