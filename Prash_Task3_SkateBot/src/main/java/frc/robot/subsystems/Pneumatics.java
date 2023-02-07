// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

  private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private static DoubleSolenoid solenoid = 
        new DoubleSolenoid(PneumaticsModuleType., 0, 7);

  /** Creates a new Pneumatics. */
  public Pneumatics() {
    activateCompressor();
    retractCylinder();
  }
  
  public void activateCompressor() {
    compressor.enableDigital();
  }

  public void deactivateCompressor() {
    compressor.disable();
  }

  public void extendCylinder() {
    solenoid.set(Value.kReverse);  
  }

  public void retractCylinder() {
    solenoid.set(Value.kForward);  
  }

  private void toggleCyliner() {
    solenoid.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
