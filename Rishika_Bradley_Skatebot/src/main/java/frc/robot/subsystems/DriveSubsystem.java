// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  private final WPI_TalonFX leftMotor = new WPI_TalonFX(9);
  private final WPI_TalonFX rightMotor = new WPI_TalonFX(10);
  DifferentialDrive motorDrive = new DifferentialDrive(leftMotor, rightMotor);

  public DriveSubsystem() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    motorDrive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void manualDrive(double move, double turn) {
    motorDrive.arcadeDrive(move, turn);
  }

}
