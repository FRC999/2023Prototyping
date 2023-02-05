// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonIMUSubsystem extends SubsystemBase {
  /** Creates a new PigeonIMUSubsystem. */
  private WPI_TalonSRX pigeonIMUController = new WPI_TalonSRX(4);   //watch for class type!
  private PigeonIMU pigeon = new PigeonIMU(pigeonIMUController);
  public PigeonIMUSubsystem() {
  }

  public double getPitch() {
    //double[] ypr = new double[3];
    //pigeon.getYawPitchRoll(ypr);
    //return ypr[1];
    return pigeon.getPitch();
  }

  public double getRoll() {
    //double[] ypr = new double[3];
    //pigeon.getYawPitchRoll(ypr);
    //return ypr[2];
    return pigeon.getRoll();
  }

  public double getYaw() {
    //double[] ypr = new double[3];
    //pigeon.getYawPitchRoll(ypr);
    //return ypr[0];
    return pigeon.getYaw();
  }

  public double zeroYaw() {
    double temporaryDouble = getYaw();
    pigeon.setYaw(0);
    return temporaryDouble;
  }

  public double setYaw(double y) {
    double temporaryDouble = getYaw();
    pigeon.setYaw(y);
    return temporaryDouble;
  }

  public PigeonIMU getPigeon(){
    return pigeon;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
