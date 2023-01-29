// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  public WPI_TalonFX motor1 = new WPI_TalonFX(1); 
  public WPI_TalonFX motor2 = new WPI_TalonFX(2); 

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    motor1.configFactoryDefault();
    motor1.setInverted(true);
    motor1.setNeutralMode(NeutralMode.Brake); 
    motor2.configFactoryDefault();
    motor2.setInverted(true);
    motor2.setNeutralMode(NeutralMode.Brake);
    motor2.follow(motor1); 
  }

  public void runMotor() {
    motor1.set(TalonFXControlMode.PercentOutput, 0.3);  
  }

  public void stopMotor() {
    motor1.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void runMotorVariableSpeed(double speed) {
    motor1.set(TalonFXControlMode.PercentOutput, speed);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
