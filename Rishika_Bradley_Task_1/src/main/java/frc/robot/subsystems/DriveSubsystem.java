// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private final WPI_TalonFX motor = new WPI_TalonFX(1);
  public DriveSubsystem() {
    motor.configFactoryDefault();
    motor.setInverted(true);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void runMotor() {
    motor.set (TalonFXControlMode.PercentOutput, 0.3);
  }
  public void stopMotor() {
    motor.set (TalonFXControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
